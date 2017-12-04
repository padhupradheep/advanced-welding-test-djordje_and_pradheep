#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_commands_node");
	ros::NodeHandle node_handle;
	std::string cartesian_control_topic = "cartesian_velocity_command";
	std_msgs::String msg;
	ros::Publisher velocity_publisher = \
	node_handle.advertise<std_msgs::String>("cartesian_control_topic", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		velocity_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
