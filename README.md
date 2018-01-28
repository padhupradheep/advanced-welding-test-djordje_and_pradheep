# Advanced Welding Test - [Team 1]

Implementation of cartesian trajectory controller for youbot arm.

[![Build Status](https://travis-ci.org/HBRS-RM/advanced-welding-test-djordje_and_pradheep.svg?branch=master)](https://travis-ci.org/HBRS-RM/advanced-welding-test-djordje_and_pradheep)

## Team members
*   Djordje Vukcevic - [@DjoleMNE](https://github.com/DjoleMNE)
*   Pradheep Krishna Muthukrishnan Padmanabhan - [@padhupradheep](https://github.com/padhupradheep)

## Description
The purpose of the Advanced Welding Test (AWT) is to perform welding in two Dimensions. The focus is on the precision required to perform a correct welding task. The correct motion following the line, while keeping a constant movement are the keys to a successful full completion of the task [3].

This project deals with desired trajectory generation and usage of a Cartesian trajectory controller on a youBot arm, in order to pass the advanced welding test from RoboCup.

Trajectory specified by a task definition is generated in terms of waypoints. Each generated waypoint represent desired Cartesian velocity.  The generated Cartesian velocity is fed into the "arm_cartesian_control" node. This node is already implemented in "b-it-bots" RoboCup repository.

The controller uses an inverse kinematic velocity solver, to compute required joint velocities which will execute desired waypoint. After computing required joint velocities, this node feed these values to motor controllers of youBot arm. The resulting joint motions  will move the arm's end-effector with predefined/desired Cartesian velocity.

The inverse kinematic velocity solver used as part of "arm_cartesian_control" node, is implemented in KDL open source robot control library.

A user is allowed to define several parameters, to define desire trajectory: phase, amplitude, number of steps, time step, execution side, number of trajectory cycles to be executed.
However, due to time limitation, current implementation of trajectory generator is capable of only creating correct sine trajectory. Nevertheless, due to developed software interface, the package is easily extendible for including other types of trajectories.

The aforementioned package for trajectory generation is successfully tested on real Kuka  youBot robot, for sine type of trajectory.

## Packages

*   ROS - The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms [1]
*   KDL - The Kinematics and Dynamics Library (KDL) develops an application independent framework for modelling and computation of kinematic chains of robots. [2]
*  b-it-bots:  mas_common_robotics and mas_industrial_robotics


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
*   Ubuntu 14.04
*   Ros Indigo
*   Orocos (KDL)

### Running the velocity command node
Note that for execution of sine trajectory, the best initial pose is pre_grasp. Additionally, the awt_trajectory_generator node is waiting for rviz to subscribe on marker topic, to start execution.

*  roscore
*  roslaunch mir_bringup_sim robot.launch
*  rosrun rviz rviz
*  roslaunch mir_arm_cartesian_control arm_cartesian_control_youbot.launch
*  roslaunch mir_awt_trajectory_generator awt_trajectory_generator.launch


## References
*   http://www.ros.org/
*   http://www.orocos.org/kdl/
*   "RoboCup@Work 2017 - Rulebook", Nico Hochgeschwender, Robin Kammel, Gerhard Kraetzschmar, Walter Nowak, Asadollah Norouzi,  Benjamin Schnieders, Sebastian Zug
