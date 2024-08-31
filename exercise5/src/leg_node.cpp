#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

class LegNode {
   private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher leg_pub_;
    std::vector<geometry_msgs::Point> points_;
    std::vector<geometry_msgs::Point> leg_centers_;
    std::string node_name_;

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        points_.push_back(msg->point);

        if (points_.size() >= 5) {  // Example threshold for leg detection
            geometry_msgs::Point leg_center = calculateLegCenter();
            leg_centers_.push_back(leg_center);
            publishLegCenter(leg_center);
            points_.clear();
        }
    }

    geometry_msgs::Point calculateLegCenter() {
        geometry_msgs::Point center;
        center.x = 0.0;
        center.y = 0.0;

        for (const auto& point : points_) {
            center.x += point.x;
            center.y += point.y;
        }

        center.x /= points_.size();
        center.y /= points_.size();
        return center;
    }

    void publishLegCenter(const geometry_msgs::Point& leg_center) {
        geometry_msgs::PointStamped leg_msg;
        leg_msg.header.stamp = ros::Time::now();
        leg_msg.point = leg_center;

        // Publish the leg center data
        leg_pub_.publish(leg_msg);
        
        // Add ROS_INFO to display the coordinates of the leg center
        ROS_INFO("[%s] Leg Center: [x: %.2f, y: %.2f]", node_name_.c_str(), leg_center.x, leg_center.y);
    }

   public:
    LegNode() {
        node_name_ = ros::this_node::getName(); // Retrieve the node name
        point_sub_ = nh_.subscribe("/point_data", 1000, &LegNode::pointCallback, this);
        leg_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/leg_data", 1000);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "leg_node");
    LegNode leg_node;
    ros::spin();
    return 0;
}

