#!/usr/bin/env python

import sys

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the DroneController class, which handles interactions with the drone
from drone_controller import BasicDroneController


# TF Libraries
import tf2_ros
from geometry_msgs.msg import Twist


# Our controller definition, note that we extend the DroneVideoDisplay class
class PDIController():
	def __init__(self):
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

	def run(self, t):
		pass

	def navigate_to(self, source_frame, target_frame):
		tfBuffer = tf2_ros.Buffer()
		tfListener = tf2_ros.TransformListener(tfBuffer)
		trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time())
		
		# Create a timer object that will sleep long enough to result in
		# a 10Hz publishing rate
		r = rospy.Rate(10) # 10hz
		
		# Loop until target is arrived
		while trans.magnitude > 1:
			trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time())
			# pass commands to controller
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

# Setup the application
if __name__=='__main__':
	payload_frame = "payload_frame"
	dropoff_frame = "dropoff frame"
	drone_frame = "drone frame"
	landing_zone = "landing frame"
	
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_shipping_system')

	# Create controller to communicate with the drone
	controller = BasicDroneController()
	# Create PDI controller to navigate drone
	navigator = PDIController()

	# Take off
	controller.SendTakeoff()
	
	# Navigate to first payload
	navigator.navigate_to(drone_frame, payload_frame)

	# Descend down
	controller.Descend()

	# Enable gripper

	# Ascend	
	controller.Ascend()
	
	# Navigate to drop-off
	navigator.navigate_to(drone_frame, dropoff_frame)

	# Disable gripper

	# Navigate to landing spot
	navigator.navigate_to(drone_frame, landing_zone)

	# Land
	controller.SendLand()