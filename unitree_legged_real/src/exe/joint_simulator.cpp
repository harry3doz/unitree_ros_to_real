#include <string>
#include <sensor_msgs/JointState.h>

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
#include <ros/ros.h>

using namespace UNITREE_LEGGED_SDK;

class JointSimulator
{
public:
    ros::NodeHandle nh;
    unitree_legged_msgs::HighState high_state_ros;
    ros::Subscriber high_sub; 
    ros::Publisher joint_pub;

public:
  JointSimulator()
  {
    high_sub = nh.subscribe("high_state", 1, &JointSimulator::highStateCallback, this);
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  }


  void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
  {
      sensor_msgs::JointState js;
      js.header.stamp = ros::Time::now();
      js.name.resize(12);

      js.name[0] = "FR_hip_joint";
      js.name[1] = "FR_thigh_joint";
      js.name[2] = "FR_calf_joint";
      js.name[3] = "FL_hip_joint";
      js.name[4] = "FL_thigh_joint";
      js.name[5] = "FL_calf_joint";
      js.name[6] = "RR_hip_joint";
      js.name[7] = "RR_thigh_joint";
      js.name[8] = "RR_calf_joint";
      js.name[9] = "RL_hip_joint";
      js.name[10] = "RL_thigh_joint";
      js.name[11] = "RL_calf_joint";

      js.position.resize(12);

      js.position[0] = msg->motorState[FR_0].q;
      js.position[1] = msg->motorState[FR_1].q;
      js.position[2] = msg->motorState[FR_2].q;
      js.position[3] = msg->motorState[FL_0].q;
      js.position[4] = msg->motorState[FL_1].q;
      js.position[5] = msg->motorState[FL_2].q;
      js.position[6] = msg->motorState[RR_0].q;
      js.position[7] = msg->motorState[RR_1].q;
      js.position[8] = msg->motorState[RR_2].q;
      js.position[9] = msg->motorState[RL_0].q;
      js.position[10] = msg->motorState[RL_1].q;
      js.position[11] = msg->motorState[RL_2].q;
    
      joint_pub.publish(js);
  }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_joint_simulator");

    JointSimulator joint_simulator;

    ros::spin();

    return 0;
}
