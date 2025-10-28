#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>
#include <deque>
#include <mutex>
#include <cmath>

class ImageTimestampSynchronizer {
private:
    ros::NodeHandle nh_;
    ros::Subscriber vectornav_sub_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    
    // Buffer to store VectorNav timestamps
    std::deque<std_msgs::Header> vectornav_timestamps_;
    std::mutex timestamp_mutex_;
    
    // Maximum time difference threshold (200ms)
    const double MAX_TIME_DIFF = 0.2; // seconds
    
    // Maximum buffer size (keep ~2 seconds of data at 20Hz)
    const size_t MAX_BUFFER_SIZE = 40;

public:
    ImageTimestampSynchronizer() : nh_("~") {
        // Subscribe to VectorNav timestamps
        vectornav_sub_ = nh_.subscribe("/vectornav_driver_node/sync_out_stamp", 100, 
                                      &ImageTimestampSynchronizer::vectornavCallback, this);
        
        // Subscribe to compressed images
        image_sub_ = nh_.subscribe("/gst_image_pub/compressed", 10, 
                                  &ImageTimestampSynchronizer::imageCallback, this);
        
        // Publisher for synchronized images
        image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/synchronized_image/compressed", 10);
        
        ROS_INFO("Image Timestamp Synchronizer node started");
    }
    
    void vectornavCallback(const std_msgs::Header::ConstPtr& header) {
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        
        // Add new timestamp to buffer
        vectornav_timestamps_.push_back(*header);
        
        // Remove old timestamps to keep buffer size manageable
        while (vectornav_timestamps_.size() > MAX_BUFFER_SIZE) {
            vectornav_timestamps_.pop_front();
        }
        
        // Clean up timestamps older than 1 second
        ros::Time current_time = ros::Time::now();
        while (!vectornav_timestamps_.empty() && 
               (current_time - vectornav_timestamps_.front().stamp).toSec() > 1.0) {
            vectornav_timestamps_.pop_front();
        }
    }
    
    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& image) {
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        
        if (vectornav_timestamps_.empty()) {
            ROS_WARN_THROTTLE(1.0, "No VectorNav timestamps available, dropping image");
            return;
        }
        
        // Find closest timestamp
        ros::Time image_time = image->header.stamp;
        double min_diff = std::numeric_limits<double>::max();
        std_msgs::Header closest_header;
        bool found_match = false;
        
        for (const auto& vn_header : vectornav_timestamps_) {
            double time_diff = std::abs((image_time - vn_header.stamp).toSec());
            
            if (time_diff < min_diff) {
                min_diff = time_diff;
                closest_header = vn_header;
                found_match = true;
            }
        }
        
        // Check if the closest match is within acceptable threshold
        if (!found_match || min_diff > MAX_TIME_DIFF) {
            ROS_WARN("No suitable VectorNav timestamp found for image (min diff: %.3f ms), dropping message", 
                     min_diff * 1000.0);
            return;
        }
        
        // Create synchronized image message
        sensor_msgs::CompressedImage sync_image = *image;
        sync_image.header.stamp = closest_header.stamp;
        sync_image.header.seq = closest_header.seq;
        // Keep the original frame_id from image
        
        // Republish with synchronized timestamp
        image_pub_.publish(sync_image);
        
        ROS_DEBUG("Synchronized image with time diff: %.1f ms", min_diff * 1000.0);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_timestamp_synchronizer");
    
    ImageTimestampSynchronizer synchronizer;
    
    ROS_INFO("Spinning node...");
    ros::spin();
    
    return 0;
}