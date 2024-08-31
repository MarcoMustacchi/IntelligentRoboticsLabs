#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <exercise3/ChargeBatteryAction.h>

void doneCb(const actionlib::SimpleClientGoalState& state, const exercise3::ChargeBatteryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Charge successful: %d %%", result->success);
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const exercise3::ChargeBatteryFeedbackConstPtr& feedback)
{
  ROS_INFO("Current battery level: %d %%", feedback->current_battery_level);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "charging_station_node");
  actionlib::SimpleActionClient<exercise3::ChargeBatteryAction> ac("/charge_battery", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); // Wait for the action server to start

  ROS_INFO("Action server started, sending goal.");
  exercise3::ChargeBatteryGoal goal;
  goal.max_battery_level = 100; // Set desired battery level

  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin(); // Keep the node running
  return 0;
}

