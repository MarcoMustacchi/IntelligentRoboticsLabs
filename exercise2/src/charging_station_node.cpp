#include <ros/ros.h>
#include <exercise2/RoomStatus.h>
#include <std_msgs/Header.h>

class ChargingStation {
public:
    ChargingStation(ros::NodeHandle& nh, uint8_t station_id, double frequency) : nh_(nh), station_id_(station_id) {
        client_ = nh_.serviceClient<exercise2::RoomStatus>("get_robot_status");
        timer_ = nh_.createTimer(ros::Duration(frequency), &ChargingStation::sendRequest, this);
        ROS_INFO("Charging Station %d started with frequency %.2f seconds.", station_id_, frequency);
    }

private:
    void sendRequest(const ros::TimerEvent&) {
        exercise2::RoomStatus srv;
        srv.request.header.stamp = ros::Time::now();
        srv.request.station_id = station_id_;

        if (client_.call(srv)) {
            ROS_INFO("Charging Station %d: Received -> Room ID: %d, Room Name: %s, Battery: %.2f%%",
                     station_id_, srv.response.room_status.room_id, srv.response.room_status.room_name.c_str(), srv.response.room_status.battery_level);
        } else {
            ROS_WARN("Charging Station %d: Failed to call service.", station_id_);
        }
    }

    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    ros::Timer timer_;
    uint8_t station_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "charging_station_node");

    if (argc != 3) {
        ROS_ERROR("Usage: charging_station_node <station_id> <frequency>");
        return 1;
    }

    ros::NodeHandle nh;
    uint8_t station_id = atoi(argv[1]);
    double frequency = atof(argv[2]);

    ChargingStation station(nh, station_id, frequency);
    ros::spin();
    return 0;
}

