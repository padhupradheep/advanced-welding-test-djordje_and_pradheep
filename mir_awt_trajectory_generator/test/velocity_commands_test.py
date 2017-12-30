#!/usr/bin/env python

import unittest
import rosunit
import rospy
import rostest
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
PKG = 'mir_awt_trajectory_generator'

class TestVelocityCommand(unittest.TestCase):

    def setUp(self):
        self.result = TwistStamped()
        self.wait_for_result = False
        self.component_output = rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped,self.callback)

    def test1(self):
        while not self.wait_for_result:
            pass
        self.assertAlmostEqual(self.result.twist.linear.x,0)
        self.assertAlmostEqual(self.result.twist.linear.y,0)
        self.assertAlmostEqual(self.result.twist.linear.z,1)

    def test2(self):
        while not self.wait_for_result:
            pass
        self.assertAlmostEqual(self.result.twist.angular.x,0)
        self.assertAlmostEqual(self.result.twist.angular.y,0)
        self.assertAlmostEqual(self.result.twist.angular.z,0)

    def test3(self):
        while not self.wait_for_result:
            pass
        self.assertEquals('arm_link_0', self.result.header.frame_id, "Incorrect link published")

    def callback(self,msg):
        self.result = msg
        self.wait_for_result = True

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.
        """
        self.component_output.unregister()

if __name__ == '__main__':
    rospy.init_node('velocity_commands_test')
    rostest.rosrun(PKG, 'velocity_commands_test', TestVelocityCommand)
