# Advanced Welding Test - [Team 1]

Implementation of cartesian trajectory controller for youbot arm.

## Team members
*   Djordje Vukcevic - [@DjoleMNE](https://github.com/DjoleMNE)
*   Pradheep Krishna Muthukrishnan Padmanabhan - [@padhupradheep](https://github.com/padhupradheep)


## Todo's
-   [ ] Moving the youbot arm using velocity commands
-   [ ] Setting up the catkin workspace and cloning mas_common_robotics and mas_industrial_robotics into the workspace.
-   [ ] Git setup
-   [ ] Travis setup

## Introduction
This project deals with implementation of a cartesian trajectory controller on a youbot arm in order to pass the advanced welding test from RoboCup. The desired cartesian position is fed into the cartesian trajectory controller. The controller moves the arm in a smooth line motion to the pre-fed goal position. 

## Packages

*   ROS - The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms [1] 
*   KDL - The Kinematics and Dynamics Library (KDL) develops an application independent framework for modelling and computation of kinematic chains of robots. [2]
*   mas_common_robotics and mas_industrial_robotics 


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
*   Ubuntu 14.04
*   Ros Indigo
*   Orocos(KDL)

### Running the velocity command node
*  roslaunch velocity_commands velocity_commands.launch 
For testing the samw node 
*  rosrun velocity_commands velocity_commands_node_test.py 

## References
*   http://www.ros.org/
*   http://www.orocos.org/kdl/
