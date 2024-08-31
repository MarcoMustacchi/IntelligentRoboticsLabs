#include <ros/ros.h>
#include <exercise1/RoomStatus.h>

void roomStatusCallback(const exercise1::RoomStatus::ConstPtr& msg) {
    ROS_INFO("Robot is in room: %s (ID: %d), Battery level: %.2f%%",
             msg->room_name.c_str(), msg->room_id, msg->battery_level);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "charging_station_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("room_status", 1000, roomStatusCallback);

    ros::spin();

    return 0;
}

