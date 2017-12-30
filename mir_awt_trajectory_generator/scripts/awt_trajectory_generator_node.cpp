/*
Created on: December, 2017
Author(s): Djordje Vukcevic, Pradheep Padmanabhan
Copyright (c) [2017]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <kdl/kdl.hpp>
#include <ros/console.h>
#include <sstream>
#include <Eigen/Dense>
#include <awt_trajectory_generator.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "awt_trajectory_generator_node");
    std::string type;
    double phase = 0;
    double amplitude = 0;
    int num_steps = 0;
    double time_step = 0;
    Eigen::MatrixXd velocity_matrix;
    std::string root_frame = "arm_link_0";

    ros::NodeHandle node_handle("~");
    node_handle.getParam("type", type);
    node_handle.getParam("phase", phase);
    node_handle.getParam("amplitude", amplitude);
    node_handle.getParam("num_steps", num_steps);
    node_handle.getParam("time_step", time_step);

    geometry_msgs::TwistStamped velocity;

    ros::Publisher velocity_publisher = \
    node_handle.advertise<geometry_msgs::TwistStamped>("/arm_1/arm_controller/cartesian_velocity_command", 100);

    traj_gen::AWT_Trajectory_Generator generator;
    generator.generate_trajectory(type,
                                phase,
                                amplitude,
                                num_steps,
                                time_step,
                                velocity_matrix);

    ROS_INFO_STREAM("The publisher need some time to connect to subscribers.");
    while(!velocity_publisher.getNumSubscribers() > 0){}
    ROS_INFO_STREAM("The publisher has connected to subscribers.");


    ros::Rate loop_rate(1/time_step);
    velocity.header.frame_id = root_frame;

    for (int i = 0;  i < velocity_matrix.rows(); i++)
    {
        velocity.twist.linear.x = velocity_matrix(i, 0);
        velocity.twist.linear.y = velocity_matrix(i, 1);
        velocity_publisher.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("The trajectory has been published!");

 return 0;
}
