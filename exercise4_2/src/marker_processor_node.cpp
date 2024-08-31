#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class MarkerProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_subscriber_;
    
    std::vector<geometry_msgs::PoseWithCovarianceStamped> detected_poses_;

    cv::Point3f mean_pose_;
    cv::Point2f projected_mean_pose_;

    std::string save_image_path_;
    cv::Mat image_;

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;

    // Callback for AprilTag detections
    void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
        ROS_INFO("Detected %lu tags", msg->detections.size());

        for (size_t i = 0; i < msg->detections.size(); ++i) {
            geometry_msgs::PoseWithCovarianceStamped tag_pose = msg->detections[i].pose;

            if (detected_poses_.empty() || tag_pose.header.seq != detected_poses_.front().header.seq) {
                detected_poses_.push_back(tag_pose);
            } else {
                ROS_INFO("Tag detections complete, shutting down subscriber.");
                tag_subscriber_.shutdown();

                loadCameraInfo();
                computeMeanPose();
                projectMeanToImage();
                drawProjectedMean();
            }
        }
    }

    // Loads camera info from the /kinect/rgb/camera_info topic
    void loadCameraInfo() {
        // Wait for CameraInfo message and fetch the camera matrix and distortion coefficients
        boost::shared_ptr<const sensor_msgs::CameraInfo> camera_info = 
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>("kinect/rgb/camera_info", ros::Duration(5));

        // Convert camera matrix K from double to float manually
        std::vector<float> tempK;
        for (size_t i = 0; i < camera_info->K.size(); i++) {
            tempK.push_back(static_cast<float>(camera_info->K[i]));
        }
        camera_matrix_ = cv::Mat(3, 3, CV_32F, tempK.data()).clone();

        // Convert distortion coefficients D from double to float manually
        std::vector<float> tempD;
        for (size_t i = 0; i < camera_info->D.size(); i++) {
            tempD.push_back(static_cast<float>(camera_info->D[i]));
        }
        distortion_coeffs_ = cv::Mat(1, 8, CV_32F, tempD.data()).clone();
    }

    // Computes the mean of the detected marker poses
    void computeMeanPose() {
        ROS_INFO("Computing mean pose from %lu detected poses.", detected_poses_.size());

        mean_pose_ = cv::Point3f(0, 0, 0);

        for (const auto& pose : detected_poses_) {
            mean_pose_.x += pose.pose.pose.position.x;
            mean_pose_.y += pose.pose.pose.position.y;
            mean_pose_.z += pose.pose.pose.position.z;
        }

        float num_poses = static_cast<float>(detected_poses_.size());
        if (num_poses > 0) {
            mean_pose_.x /= num_poses;
            mean_pose_.y /= num_poses;
            mean_pose_.z /= num_poses;
        }

        ROS_INFO("Computed mean pose: (%f, %f, %f)", mean_pose_.x, mean_pose_.y, mean_pose_.z);
    }

    // Projects the computed mean pose to 2D image coordinates
    void projectMeanToImage() {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);  // Rotation vector (no rotation)
        cv::Mat tvec = (cv::Mat_<float>(3, 1) << mean_pose_.x, mean_pose_.y, mean_pose_.z);

        std::vector<cv::Point3f> points3D = {mean_pose_};
        std::vector<cv::Point2f> projected_points;

        cv::projectPoints(points3D, rvec, tvec, camera_matrix_, distortion_coeffs_, projected_points);

        projected_mean_pose_ = projected_points[0];
    }

    // Draws the projected mean pose on the image and saves it
    void drawProjectedMean() {
        if (!image_.empty()) {
            cv::circle(image_, projected_mean_pose_, 10, cv::Scalar(255, 0, 0), -1);  // Draw blue circle at mean point
            cv::imwrite(save_image_path_, image_);
            ROS_INFO("Saved image with projected mean pose at %s", save_image_path_.c_str());
        } else {
            ROS_ERROR("Image not loaded, cannot draw mean pose.");
        }
    }

public:
    MarkerProcessor() : mean_pose_(0, 0, 0), nh_() {
        // Hardcoded image path
        std::string image_path = "../config/image_final";
        save_image_path_ = image_path + "_mean.png";

        // Load the image
        image_ = cv::imread(image_path+".png");
        if (image_.empty()) {
            ROS_ERROR("Failed to load image from %s", image_path.c_str());
        }

        // Subscribe to tag detections
        tag_subscriber_ = nh_.subscribe("/tag_detections", 1000, &MarkerProcessor::tagDetectionCallback, this);

        ROS_INFO("MarkerProcessor initialized.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_processor_node");

    MarkerProcessor processor;

    ros::spin();
    return 0;
}

