#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg, tf2_ros::Buffer& tfBuffer) {
    // Loop through each detected tag using index-based loop
    for (size_t i = 0; i < msg->detections.size(); ++i) {
        const auto& detection = msg->detections[i];

        // Extract the pose of the tag in the camera frame
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header = detection.pose.header;
        camera_pose.pose = detection.pose.pose.pose;  // Get pose of the tag

        // Print the original pose in the camera_base frame
        ROS_INFO("Tag ID: %d", detection.id[0]);
        ROS_INFO("Original Pose in camera_base frame: [x: %.2f, y: %.2f, z: %.2f]",
                 camera_pose.pose.position.x,
                 camera_pose.pose.position.y,
                 camera_pose.pose.position.z);

        try {
            // Transform the pose from the 'camera_base' frame to the 'base_link' frame
            geometry_msgs::PoseStamped base_link_pose;
            
            // Transform the pose and handle the exception if the transformation is not available
            base_link_pose = tfBuffer.transform(camera_pose, "base_link", ros::Time(0), detection.pose.header.frame_id);

            // Print the transformed pose in the base_link frame
            ROS_INFO("Transformed Pose in base_link frame: [x: %.2f, y: %.2f, z: %.2f]",
                     base_link_pose.pose.position.x,
                     base_link_pose.pose.position.y,
                     base_link_pose.pose.position.z);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform tag pose: %s", ex.what());
        }
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "tag_pose_transformer_node");
    ros::NodeHandle nh;

    // Set up TF2 listener and buffer
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Subscribe to the AprilTag detections topic
    ros::Subscriber sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>(
        "/tag_detections", 10, boost::bind(&tagDetectionsCallback, _1, boost::ref(tfBuffer)));

    // Keep the node alive and process callbacks
    ros::spin();

    return 0;
}
