#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

ros::Subscriber sub_high;
ros::Subscriber sub_low;
ros::Subscriber sub_cmd_vel;

ros::Publisher pub_high;
ros::Publisher pub_low;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_imu;
ros::Publisher pub_odom;

long high_count = 0;
long low_count = 0;
long cmd_vel_count = 0;
long timer_count = 0;

void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high.publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::LowState low_state_ros;

    low_state_ros = state2rosMsg(custom.low_state);

    pub_low.publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("cmdVelCallback is running !\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_rot_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high.publish(high_state_ros);

    printf("cmdVelCallback ending !\t%ld\n\n", cmd_vel_count++);
}

/*
// ros timer event
*/
void timerCallback(const ros::TimerEvent&)
{
    //printf("timerCallback is running !\t%ld\n", timer_count);

    // get SDK high_state and publish ros msg
    unitree_legged_msgs::HighState high_state_ros;
    high_state_ros = state2rosMsg(custom.high_state);
    pub_high.publish(high_state_ros);

    // compute imu from high_state lcm msg
    ros::Time current_time;

    current_time = ros::Time::now();

    sensor_msgs::Imu msg_imu;

    msg_imu.header.seq = timer_count;
    msg_imu.header.stamp = current_time;
    msg_imu.header.frame_id = "imu_link";

    msg_imu.orientation.w = custom.high_state.imu.quaternion[0];
    msg_imu.orientation.x = custom.high_state.imu.quaternion[1];
    msg_imu.orientation.y = custom.high_state.imu.quaternion[2];
    msg_imu.orientation.z = custom.high_state.imu.quaternion[3];

    msg_imu.angular_velocity.x = custom.high_state.imu.gyroscope[0];
    msg_imu.angular_velocity.y = custom.high_state.imu.gyroscope[1];
    msg_imu.angular_velocity.z = custom.high_state.imu.gyroscope[2];

    msg_imu.linear_acceleration.x = custom.high_state.imu.accelerometer[0];
    msg_imu.linear_acceleration.y = custom.high_state.imu.accelerometer[1];
    msg_imu.linear_acceleration.z = custom.high_state.imu.accelerometer[2];

    pub_imu.publish(msg_imu);

    // compute odometry from high_state lcm msg
    nav_msgs::Odometry msg_odom;

    msg_odom.header.seq = timer_count;
    msg_odom.header.stamp = current_time;
    msg_odom.header.frame_id = "odom";
    msg_odom.header.frame_id = "base_link";

    msg_odom.pose.pose.position.x = custom.high_state.position[0];
    msg_odom.pose.pose.position.y = custom.high_state.position[1];
    msg_odom.pose.pose.position.z = custom.high_state.position[2];

    msg_odom.pose.pose.orientation.w = custom.high_state.imu.quaternion[0];
    msg_odom.pose.pose.orientation.x = custom.high_state.imu.quaternion[1];
    msg_odom.pose.pose.orientation.y = custom.high_state.imu.quaternion[2];
    msg_odom.pose.pose.orientation.z = custom.high_state.imu.quaternion[3];

    msg_odom.twist.twist.linear.x = custom.high_state.velocity[0];
    msg_odom.twist.twist.linear.y = custom.high_state.velocity[1];
    msg_odom.twist.twist.angular.z = custom.high_state.velocity[2];

    pub_odom.publish(msg_odom);

    //printf("timerCallback\t%ld\n\n", timer_count++);
}



/*
// main function
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_udp");

    ros::NodeHandle nh;

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        sub_low = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

        LoopFunc loop_udpSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));
        LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        ros::spin();

        // printf("low level runing!\n");
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);
        pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
        
        sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
        
        ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        ros::spin();

        // printf("high level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0;
}
