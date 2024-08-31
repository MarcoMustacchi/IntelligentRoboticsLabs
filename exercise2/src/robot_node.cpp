#include <ros/ros.h>
#include <exercise2/RoomStatus.h>
#include <std_msgs/Header.h>

class Robot {
public:
    Robot(ros::NodeHandle& nh) : nh_(nh), battery_level_(100.0) {
        service_ = nh_.advertiseService("get_robot_status", &Robot::handleRequest, this);
        ROS_INFO("Robot service ready.");
    }

private:
    bool handleRequest(exercise2::RoomStatus::Request &req, exercise2::RoomStatus::Response &res) {
        res.header.stamp = ros::Time::now();
        res.room_status.room_id = current_room_id_;
        res.room_status.room_name = current_room_name_;
        res.room_status.battery_level = battery_level_;
        ROS_INFO("Request from Charging Station %d: Sending robot status -> Room ID: %d, Room Name: %s, Battery: %.2f%%",
                 req.station_id, res.room_status.room_id, res.room_status.room_name.c_str(), res.room_status.battery_level);
        return true;
    }

    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    uint8_t current_room_id_ = 1;  // Example room ID
    std::string current_room_name_ = "Room A";  // Example room name
    float battery_level_;  // Example battery level
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    Robot robot(nh);
    ros::spin();
    return 0;
}

