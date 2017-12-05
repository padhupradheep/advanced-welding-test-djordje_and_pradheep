#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <kdl/kdl.hpp>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_commands_node");
	ros::NodeHandle node_handle("~");
	geometry_msgs::TwistStamped velocity;

	ros::Publisher velocity_publisher = \
	node_handle.advertise<geometry_msgs::TwistStamped>("/mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command", 1);

	ros::Rate loop_rate(50);
	velocity.header.frame_id = "arm_link_5";
	velocity.twist.linear.x = 0;
	velocity.twist.linear.y = 10;
	velocity.twist.linear.z = 0;
	velocity.twist.angular.x = 0;
	velocity.twist.angular.y = 0;
	velocity.twist.angular.z = 0;

	int count = 0;
	while (ros::ok())
	{
		velocity_publisher.publish(velocity);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
