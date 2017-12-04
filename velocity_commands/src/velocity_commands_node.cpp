#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <kdl/kdl.hpp>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_commands_node");
	double linear_x = 0;
	double linear_y = 0;
	double linear_z = 0;
	double angular_x = 0;
	double angular_y = 0;
	double angular_z = 0;

	ros::NodeHandle node_handle("~");
	node_handle.getParam("linear_x", linear_x);
	node_handle.getParam("linear_y", linear_y);
	node_handle.getParam("linear_z", linear_z);
	node_handle.getParam("angular_x", angular_x);
	node_handle.getParam("angular_y", angular_y);
	node_handle.getParam("angular_z", angular_z);

	geometry_msgs::TwistStamped velocity;

	ros::Publisher velocity_publisher = \
	node_handle.advertise<geometry_msgs::TwistStamped>("/mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command", 1);

	ros::Rate loop_rate(50);
	velocity.header.frame_id = "arm_link_5";
	velocity.twist.linear.x = linear_x;
	velocity.twist.linear.y = linear_y;
	velocity.twist.linear.z = linear_z;
	velocity.twist.angular.x = angular_x;
	velocity.twist.angular.y = angular_y;
	velocity.twist.angular.z = angular_z;

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
