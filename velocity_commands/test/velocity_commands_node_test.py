#!/usr/bin/env python

import unittest
import rosunit
import rospy
import rostest
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
PKG = 'velocity_commands'

class TestVelocityCommand(unittest.TestCase):

    def setUp(self):
        self.twist = TwistStamped()
        rospy.Subscriber("mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command")


    def tearDown(self):
        pass


if __name__ == '__main__':
    rospy.init_node('velocity_commands_node_test')
    rostest.rosrun(PKG, 'velocity_commands_node_test', TestVelocityCommand)
