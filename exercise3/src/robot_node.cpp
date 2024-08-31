#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <exercise3/ChargeBatteryAction.h>

class ChargeBatteryAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<exercise3::ChargeBatteryAction> as_;
  std::string action_name_;
  exercise3::ChargeBatteryFeedback feedback_;
  exercise3::ChargeBatteryResult result_;

public:
  ChargeBatteryAction() :
    as_(nh_, "/charge_battery", boost::bind(&ChargeBatteryAction::executeCB, this, _1), false),
    action_name_("charge_battery")
  {
    as_.start();
  }

  ~ChargeBatteryAction(void)
  {
  }

  void executeCB(const exercise3::ChargeBatteryGoalConstPtr &goal)
  {
    ros::Rate rate(1); // 1Hz
    int current_level = 5; // Start with 5% battery

    // Simulate charging
    while (current_level < goal->max_battery_level)
    {
      // Check if preempted
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        return;
      }

      feedback_.current_battery_level = current_level;
      as_.publishFeedback(feedback_);

      current_level++;
      rate.sleep();
    }

    // Success
    result_.success = true;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_node");
  ChargeBatteryAction charge_battery;
  ros::spin();
  return 0;
}

