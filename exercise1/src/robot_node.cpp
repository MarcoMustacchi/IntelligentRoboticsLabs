#include <ros/ros.h>
#include <exercise1/RoomStatus.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;

    ros::Publisher room_pub = nh.advertise<exercise1::RoomStatus>("room_status", 1000);
    ros::Rate loop_rate(5); // 5Hz

    // Room data
    std::vector<std::string> room_names = {"Lab 1", "Lab 2", "Lab 3", "Lab 4", "Lab 5"};
    std::vector<uint32_t> room_ids = {1, 2, 3, 4, 5};

    // Simulated battery level (starting at 100%)
    float battery_level = 100.0;
    int room_index = 0;

    while (ros::ok()) {
        exercise1::RoomStatus msg;
        msg.room_id = room_ids[room_index];
        msg.room_name = room_names[room_index];
        msg.battery_level = battery_level;

        // Publish the room status
        room_pub.publish(msg);

        // Simulate robot moving to the next room and battery draining
        room_index = (room_index + 1) % room_names.size(); // Loop through rooms
        battery_level -= 5.0; // Reduce battery by 5% each iteration

        // Simulate recharging if battery is low
        if (battery_level <= 0.0) {
            ROS_INFO("Battery low, recharging...");
            battery_level = 100.0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

