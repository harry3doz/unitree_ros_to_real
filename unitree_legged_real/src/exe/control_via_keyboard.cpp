#include <termios.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "keyconfig.h"


int getch()
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// Store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// Make required changes and apply the settings
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 1;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Get the current character
	ch = getchar();

	// Reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_input_node");

	ros::NodeHandle nh;

	ros::Rate loop_rate(20);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("wp_manager/cmd_vel", 1);

  // wp_manager services
  ros::ServiceClient addwp_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/add_wp");
  ros::ServiceClient delwp_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/delete_wp");
  ros::ServiceClient clearwp_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/clear_wp");
  ros::ServiceClient wpup_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/wp_target_up");
  ros::ServiceClient wpdn_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/wp_target_down");
  ros::ServiceClient wptype_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/wp_type_change");
  ros::ServiceClient wpsave_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/save_wp");
  ros::ServiceClient wpload_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/read_wp");
  ros::ServiceClient wpinit_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/reset_wp_pos");
  ros::ServiceClient initpos_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/init_pos_switch");
  ros::ServiceClient startnav_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/start_nav");
  ros::ServiceClient stopnav_cli = nh.serviceClient<std_srvs::Empty>("/wp_manager/stop_nav");

	geometry_msgs::Twist twist;

  std_srvs::Empty srv;

  WpKey wp_key;


	long count = 0;

	while (ros::ok())
	{
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

    int ch = 0;

    ch = getch();

    wp_key.state = wp_key.wpcfg(static_cast<char>(ch));

		//printf("%ld\n", count++);
		//printf("ch = %d\n\n", ch);

		switch (wp_key.state)
		{
		case WpKey::QUIT:
			printf("already quit!\n");
			return 0;

		case WpKey::UP:
			twist.linear.y = 1.0;
			//printf("move forward!\n");
			break;

		case WpKey::DOWN:
			twist.linear.y = -1.0;
			//printf("move backward!\n");
			break;

		case WpKey::LEFT:
			twist.linear.x = -1.0;
			//printf("move left!\n");
			break;

		case WpKey::RIGHT:
			twist.linear.x = 1.0;
			//printf("move right!\n");
			break;

		case WpKey::ROT_L:
			twist.angular.z = 1.0;
			//printf("turn left!\n");
			break;

		case WpKey::ROT_R:
			twist.angular.z = -1.0;
			//printf("turn right!\n");
			break;

    case WpKey::WP_ADD:
      addwp_cli.call(srv);
			printf("add wp\n");
      break;

    case WpKey::WP_DEL:
      delwp_cli.call(srv);
			printf("del wp\n");
      break;
    
    case WpKey::WP_CLEAR:
      clearwp_cli.call(srv);
			printf("clear wp\n");
      break;

    case WpKey::WP_INIT:
      wpinit_cli.call(srv);
			printf("reset wp position\n");
      break;

    case WpKey::WP_NEXT:
      wpup_cli.call(srv);
			printf("wpup\n");
      break;

    case WpKey::WP_PREV:
      wpdn_cli.call(srv);
			printf("wpdown\n");
      break;

    case WpKey::INIT_SWITCH:
      initpos_cli.call(srv);
      printf("switch wp/init mode\n");
      break;

    case WpKey::TYPE_SWITCH:
      wptype_cli.call(srv);
      printf("switch wp type\n");
      break;

    case WpKey::WP_SAVE:
      wpsave_cli.call(srv);
      printf("wpsave\n");
      break;

    case WpKey::WP_LOAD:
      wpload_cli.call(srv);
      printf("wpload\n");
      break;

    case WpKey::NAV_START:
      startnav_cli.call(srv);
			printf("start nav\n");
      break;

    case WpKey::NAV_STOP:
      stopnav_cli.call(srv);
			printf("stop nav\n");
      break;
      

		default:
			//printf("Stop!\n");
			break;
		}

		pub.publish(twist);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
