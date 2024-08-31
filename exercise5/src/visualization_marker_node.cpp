#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

class VisualizationMarkerNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber person_sub_;
    ros::Publisher marker_pub_;

    void personCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // Create a marker
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = "base_scan"; // or use "map" or another frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "persons";
        marker.id = 0;  // If tracking multiple persons, you may want to increment this
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;

        // Set the position of the marker to the person's coordinates
        marker.pose.position.x = msg->point.x;
        marker.pose.position.y = msg->point.y;
        marker.pose.position.z = msg->point.z;

        // Set the scale of the marker
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color of the marker (green in this case)
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0);  // Infinite lifetime

        // Publish the marker
        marker_pub_.publish(marker);

        ROS_INFO("Marker Published at [x: %.2f, y: %.2f, z: %.2f]", msg->point.x, msg->point.y, msg->point.z);
    }

public:
    VisualizationMarkerNode() {
        person_sub_ = nh_.subscribe("/person_data", 10, &VisualizationMarkerNode::personCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_marker_node");
    VisualizationMarkerNode visualization_marker_node;
    ros::spin();
    return 0;
}

