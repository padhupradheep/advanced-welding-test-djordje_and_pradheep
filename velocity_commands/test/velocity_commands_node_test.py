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
        self.result = TwistStamped()
        self.wait_for_result = False
        rospy.Subscriber("/mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command",TwistStamped,self.callback)


    def test1(self):
        while  not self.wait_for_result:
            pass
        self.assertAlmostEqual(self.result.twist.linear.x,0)
        self.assertAlmostEqual(self.result.twist.linear.y,0)
        self.assertAlmostEqual(self.result.twist.linear.z,1)

    def test2(self):
        while  not self.wait_for_result:
            pass
        self.assertAlmostEqual(self.result.twist.angular.x,0)
        self.assertAlmostEqual(self.result.twist.angular.y,0)
        self.assertAlmostEqual(self.result.twist.angular.z,0)

    def test3(self):
        while  not self.wait_for_result:
            pass
        self.assertEquals('arm_link_5', self.result.header.frame_id, "Incorrect link published")

    def callback(self,msg):
        self.result = msg
        self.wait_for_result = True

    def tearDown(self):
        pass


if __name__ == '__main__':
    rospy.init_node('velocity_commands_node_test')
    rostest.rosrun(PKG, 'velocity_commands_node_test', TestVelocityCommand)
