#!/usr/bin/env python
import sys
import copy
import rospy
import geometry_msgs.msg
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import brics_actuator.msg
import numpy
from std_msgs.msg import String

class VelocitySimulator:

	def __init__(self):
		'''Initialize ros publisher, ros subscribers'''
		self.new_pose = True
		self.states_sub = rospy.Subscriber("/joint_states", JointState, self.current_configuration_cb)
		self.velocities_sub = rospy.Subscriber("/arm_1/arm_controller/velocity_command", brics_actuator.msg.JointVelocities, self.joint_velocity_cb)
		self.points = JointTrajectoryPoint()
		self.simulation_vel_pub = rospy.Publisher("/arm_1/arm_controller/command", JointTrajectory,queue_size=10)
		self.trajectory = JointTrajectory()
		joints = ["arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"]
		self.trajectory.joint_names = joints

	def joint_velocity_cb(self, msg):

		self.new_pose = False
		# self.points.velocities = msg.velocities
		vels = list()
		for i in range(5):
			vels.append(msg.velocities[i].value)
		self.points.velocities = vels

		self.points.time_from_start = rospy.Duration(1.0)

		self.trajectory.points.append(self.points)
		self.simulation_vel_pub.publish(self.trajectory)

	def current_configuration_cb(self, msg):
		if(self.new_pose):
			self.points.positions = msg.position


def main(args):
	'''Initializes and cleanup ros node'''
	vs = VelocitySimulator()
	rospy.init_node('awt_trajectory_simulator_node', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down Velocity Simulator node"

if __name__ == '__main__':
	main(sys.argv)
