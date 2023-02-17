#include <string.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/BmsCmd.h>
#include <unitree_legged_msgs/BmsState.h>
#include <unitree_legged_msgs/IMU.h>

#include <std_srvs/Empty.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/joystick.h"


using namespace UNITREE_LEGGED_SDK;

class WirelessController
{
public:
  ros::NodeHandle nh;
  ros::Rate loop_rate; 

  unitree_legged_msgs::HighState high_state_ros;
  ros::Subscriber high_sub;
  ros::Publisher cmd_pub; 

  ros::ServiceClient wpadd_cli;
  ros::ServiceClient wpdel_cli;
  ros::ServiceClient wpclear_cli;
  ros::ServiceClient wpup_cli;
  ros::ServiceClient wpdn_cli;
  ros::ServiceClient startnav_cli;
  ros::ServiceClient stopnav_cli;
  ros::ServiceClient wpsave_cli;
  ros::ServiceClient wpload_cli;
  ros::ServiceClient wpinit_cli;
  ros::ServiceClient initpos_cli;
  ros::ServiceClient wpchange_cli;

  geometry_msgs::Twist vel_cmd;
  std_srvs::Empty srv;

  xRockerBtnDataStruct keydata;

  WirelessController()
    : loop_rate(100)
  {
    this->high_sub = this->nh.subscribe("high_state", 1, &WirelessController::highStateCallback, this);
    this->cmd_pub = this->nh.advertise<geometry_msgs::Twist>("wp_manager/cmd_vel", 10);


    this->wpadd_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/add_wp");
    this->wpdel_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/delete_wp");
    this->wpclear_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/clear_wp");
    this->wpup_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/wp_target_up");
    this->wpdn_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/wp_target_down");
    this->startnav_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/start_nav");
    this->stopnav_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/stop_nav");
    this->wpsave_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/save_wp");
    this->wpload_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/load_wp");
    this->wpinit_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/reset_wp_pos");
    this->initpos_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/init_pos_switch");
    this->wpchange_cli = this->nh.serviceClient<std_srvs::Empty>("wp_manager/wp_name_change");
  }
  

  void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
  {
    memcpy(&keydata, &msg->wirelessRemote, 40);
    //ROS_DEBUG_STREAM("[OK] wirelessremote components = " << (int)keydata.btn.components.A);
    //ROS_DEBUG_STREAM("lx ly rx ry = " << keydata.lx << " " << keydata.ly << " " << keydata.rx << " " << keydata.ry);

    // ---
    // common operations
    // ---

    if ((int)keydata.btn.components.R1 == 1)
    {
      this->wpup_cli.call(this->srv);
      ROS_INFO("[WP] target next");
    }
    if ((int)keydata.btn.components.L1 == 1)
    {
      this->wpdn_cli.call(this->srv);
      ROS_INFO("[WP] target prev");
    }

    // deactivate at stand/walk state
    if (msg->mode > 2 && (int)keydata.btn.components.L2 == 0)
    {
      //ROS_INFO_STREAM_ONCE("[ACTIVE] wp control");
      // wp control
      vel_cmd.linear.x = (abs(keydata.lx) > 0.2)?keydata.lx:0.0;
      vel_cmd.linear.y = (abs(keydata.ly) > 0.2)?keydata.ly:0.0;
      vel_cmd.angular.z = (abs(keydata.rx) > 0.2)?-keydata.rx:0.0;

      this->cmd_pub.publish(vel_cmd);

      // service call
      if ((int)keydata.btn.components.A == 1)
      {
        this->wpadd_cli.call(this->srv);
        ROS_INFO("[WP] add");
      }
      if ((int)keydata.btn.components.B == 1)
      {
        bool result = this->wpclear_cli.call(this->srv);
        if(result)
          ROS_INFO_STREAM("[OK] clear wp");
        else
          ROS_ERROR_STREAM("[ERROR] clear wp servie call failed");
      }
      if ((int)keydata.btn.components.X == 1)
      {
        this->wpinit_cli.call(this->srv);
        ROS_INFO("[WP] reset position");
      }
      if ((int)keydata.btn.components.Y == 1)
      {
        this->initpos_cli.call(this->srv);
        ROS_INFO("[WP] init/wp switch");
      }
      if ((int)keydata.btn.components.R2 == 1)
      {
        this->wpchange_cli.call(this->srv);
        ROS_INFO("[WP] wp name change");
      }
      if ((int)keydata.btn.components.start == 1)
      {
        this->wpsave_cli.call(this->srv);
        ROS_INFO("[WP] save");
      }
      if ((int)keydata.btn.components.select == 1)
      {
        this->wpload_cli.call(this->srv);
        ROS_INFO("[WP] load");
      }

    }
    else if (msg->mode <= 2) // nav start/stop at standing mode
    {
      //ROS_INFO_STREAM_ONCE("[ACTIVE] nav control");

      if ((int)keydata.btn.components.start == 1)
      {
        this->startnav_cli.call(this->srv);
        ROS_INFO("[Nav] start");
      }
      if ((int)keydata.btn.components.B == 1)
      {
        this->stopnav_cli.call(this->srv);
        ROS_INFO("[Nav] stop");
      }

    }

  }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wireless_controller");

    WirelessController wc;

    while(ros::ok())
    {
      ros::spinOnce();
      wc.loop_rate.sleep();
    }

    return 0;
}
