#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    ROS_INFO("Callback triggered");
    // Check if any tags are detected
    if (msg->detections.empty()) {
        ROS_INFO("No AprilTags detected.");
        return;
    }

    // Print the details of each detected tag
    for (const auto& detection : msg->detections) {
        ROS_INFO("Detected AprilTag ID: %d", detection.id[0]);  // Print the ID of the detected tag
        ROS_INFO("Position - x: %.2f, y: %.2f, z: %.2f",
                 detection.pose.pose.pose.position.x,
                 detection.pose.pose.pose.position.y,
                 detection.pose.pose.pose.position.z);
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "tag_detection_subscriber");
    ros::NodeHandle nh;

    // Subscribe to the AprilTag detections topic
    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, tagDetectionsCallback);

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}

