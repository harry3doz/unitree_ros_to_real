#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/BmsCmd.h>
#include <unitree_legged_msgs/BmsState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

using namespace UNITREE_LEGGED_SDK;

class WirelessController
{
public:
  ros::NodeHandle nh;
  unitree_legged_msgs::HighState high_state_ros;
  ros::Subscriber high_sub;
  ros::Publisher cmd_pub; 

  WirelessController();
  void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg);

};
