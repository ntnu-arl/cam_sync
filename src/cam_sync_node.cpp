#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <deque>
#include <mutex>
#include <cmath>

class CamSyncNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sync_out_sub_;
    ros::Publisher image_pub_;

    // message_filters subscribers for image + exposure (exact stamp match)
    message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub_;
    message_filters::Subscriber<sensor_msgs::TimeReference> exposure_sub_;

    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::CompressedImage, sensor_msgs::TimeReference> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;

    // Buffer for sync out stamps from IMU
    std::deque<ros::Time> sync_out_stamps_;
    std::mutex sync_mutex_;

    // Maximum allowed time difference between image stamp and nearest sync_out (seconds)
    const double MAX_SYNC_DIFF = 0.2;

    // Maximum buffer size
    const size_t MAX_SYNC_BUFFER = 40;

public:
    CamSyncNode() : nh_("~") {
        // Sync out from IMU — separate timeline, buffered manually
        sync_out_sub_ = nh_.subscribe("/vectornav_driver_node/sync_out_stamp", 100,
            &CamSyncNode::syncOutCallback, this);

        // message_filters subscribers for exact time pairing
        image_sub_.subscribe(nh_, "/gst_image_pub/compressed", 10);
        exposure_sub_.subscribe(nh_, "/gst_image_pub/compressed_exposure_time", 10);

        exact_sync_.reset(new ExactSync(ExactPolicy(10), image_sub_, exposure_sub_));
        exact_sync_->registerCallback(
            boost::bind(&CamSyncNode::syncCallback, this, _1, _2));

        image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(
            "/gst_image_pub/synchronized/compressed", 10);

        ROS_INFO("CamSync node started (ExactTime policy for image+exposure)");
    }

    void syncOutCallback(const std_msgs::Header::ConstPtr& header) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        sync_out_stamps_.push_back(header->stamp);

        while (sync_out_stamps_.size() > MAX_SYNC_BUFFER) {
            sync_out_stamps_.pop_front();
        }
    }

    void syncCallback(const sensor_msgs::CompressedImage::ConstPtr& image,
                      const sensor_msgs::TimeReference::ConstPtr& exposure) {
        ros::Time image_stamp = image->header.stamp;

        // Exposure duration encoded as ros::Time in the time_ref field
        ros::Duration exposure_duration(exposure->time_ref.sec, exposure->time_ref.nsec);

        // Find closest sync_out stamp to the image stamp
        ros::Time best_sync;
        double min_diff = std::numeric_limits<double>::max();
        bool sync_found = false;
        {
            std::lock_guard<std::mutex> lock(sync_mutex_);
            if (sync_out_stamps_.empty()) {
                ROS_WARN_THROTTLE(1.0, "No sync_out stamps available, dropping image");
                return;
            }

            auto best_it = sync_out_stamps_.end();
            for (auto it = sync_out_stamps_.begin(); it != sync_out_stamps_.end(); ++it) {
                double diff = std::abs((image_stamp - *it).toSec());
                if (diff < min_diff) {
                    min_diff = diff;
                    best_sync = *it;
                    best_it = it;
                    sync_found = true;
                }
            }

            // Discard the matched stamp and everything older
            if (best_it != sync_out_stamps_.end()) {
                sync_out_stamps_.erase(sync_out_stamps_.begin(), std::next(best_it));
            }
        }

        if (!sync_found || min_diff > MAX_SYNC_DIFF) {
            ROS_WARN("No sync_out stamp within threshold (min diff: %.1f ms), dropping image",
                min_diff * 1000.0);
            return;
        }

        // Corrected timestamp: sync_out + exposure/2
        ros::Time corrected_stamp = best_sync + ros::Duration(exposure_duration.toSec() / 2.0);

        sensor_msgs::CompressedImage sync_image = *image;
        sync_image.header.stamp = corrected_stamp;

        image_pub_.publish(sync_image);

        ROS_DEBUG("Corrected stamp: sync=%.6f + exp/2=%.6fms -> %.6f (match diff: %.1fms)",
            best_sync.toSec(), exposure_duration.toSec() * 500.0,
            corrected_stamp.toSec(), min_diff * 1000.0);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cam_sync_node");
    CamSyncNode node;
    ros::spin();
    return 0;
}