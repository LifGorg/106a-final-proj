#!/usr/bin/env python

import sys

from drone_status import DroneStatus

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the DroneController class, which handles interactions with the drone
from drone_controller import BasicDroneController, PIDController


# TF Libraries
import tf2_ros
from geometry_msgs.msg import Twist

import time

import signal

COMMAND_PERIOD = 100 #ms

def exit_handler(signum, frame):
	drone.SendEmergency()
	time.sleep(2)
	exit(1)
	
# Setup the application
if __name__=='__main__':

	signal.signal(signal.SIGINT, exit_handler)

	payload_frame = "ar_marker_2"
	dropoff_frame = "dropoff frame"
	drone_frame = "ar_marker_1"
	landing_zone = "landing frame"
	
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_shipping_system')

	# Create controller to communicate with the drone
	drone = BasicDroneController()
	# Create PDI controller to navigate drone
	navigator = PIDController()

	# Take off
	drone.SendTakeoff()

	time.sleep(5)

	# Navigate to first payload
	drone.navigate(navigator, drone_frame, payload_frame)

	drone.SetCommand(0, 0, 0, 0)

	time.sleep(5)

	# # Descend down
	# drone.Descend()

	# Enable gripper
	# Ascend	
	# controller.Ascend()
	
	# # Navigate to drop-off
	# navigator.navigate_to(drone_frame, dropoff_frame)

	# # Disable gripper

	# # Navigate to landing spot
	# navigator.navigate_to(drone_frame, landing_zone)

	# Land
	drone.SendLand()