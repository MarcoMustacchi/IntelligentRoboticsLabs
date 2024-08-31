#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>  // For cos and sin functions

class PointNode {
   private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher point_pub_;
    std::string node_name_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
                float angle = msg->angle_min + i * msg->angle_increment;  // Polar angle (radians)
                float distance = msg->ranges[i];  // Polar distance (meters)

                // Convert polar coordinates to metric coordinates (X, Y)
                float x = distance * cos(angle);  // X coordinate in meters
                float y = distance * sin(angle);  // Y coordinate in meters

                // Display the polar coordinates
                ROS_INFO("[%s] Polar Coordinates: [angle: %.2f radians, distance: %.2f meters]", node_name_.c_str(), angle, distance);

                // Create PointStamped message for publishing
                geometry_msgs::PointStamped point_msg;
                point_msg.header.stamp = ros::Time::now();
                point_msg.point.x = x;
                point_msg.point.y = y;
                point_msg.point.z = 0.0;

                // Publish the point message
                point_pub_.publish(point_msg);

                // Display the 2D metric coordinates
                ROS_INFO("[%s] Metric Coordinates: [X: %.2f meters, Y: %.2f meters]", node_name_.c_str(), x, y);
            }
        }
    }

   public:
    PointNode() {
        node_name_ = ros::this_node::getName(); // Retrieve the node name
        scan_sub_ = nh_.subscribe("/scan", 1000, &PointNode::scanCallback, this);
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/point_data", 1000);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_node");
    PointNode point_node;
    ros::spin();
    return 0;
}

