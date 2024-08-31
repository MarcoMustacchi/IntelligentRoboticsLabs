#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

class PersonNode {
   private:
    ros::NodeHandle nh_;                  
    ros::Subscriber leg_sub_;           
    ros::Publisher person_pub_;         
    std::vector<geometry_msgs::Point> legs_;
    std::string node_name_;   

    void legCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        legs_.push_back(msg->point);

        if (legs_.size() >= 2) {  // Example threshold for detecting a person
            geometry_msgs::Point person_center = calculatePersonCenter();
            publishPersonCenter(person_center);
            legs_.clear();
        }
    }

    geometry_msgs::Point calculatePersonCenter() {
        geometry_msgs::Point center;
        center.x = 0.0;
        center.y = 0.0;

        for (const auto& leg : legs_) {
            center.x += leg.x;
            center.y += leg.y;
        }

        center.x /= legs_.size(); 
        center.y /= legs_.size(); 
        return center;
    }

    void publishPersonCenter(const geometry_msgs::Point& person_center) {
        geometry_msgs::PointStamped person_msg;
        person_msg.header.stamp = ros::Time::now();
        person_msg.point = person_center;

        // Publish the person center point
        person_pub_.publish(person_msg);

        // Log the published data
        ROS_INFO("[%s] Person Center: [x: %.2f, y: %.2f]", node_name_.c_str(), person_center.x, person_center.y);
    }

   public:
    PersonNode() {
        node_name_ = ros::this_node::getName(); // Retrieve the node name
        leg_sub_ = nh_.subscribe("/leg_data", 1000, &PersonNode::legCallback, this);
        person_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/person_data", 1000);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "person_node");
    PersonNode person_node;
    ros::spin();
    return 0;
}

