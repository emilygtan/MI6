#!/usr/bin/python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from enum import Enum
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError


class MainDriver:
	def __init__(self):
		self.states = States(1)

		self.state_pub = rospy.Publisher("/state", String, queue_size=2)
		self.left_pub = rospy.Publisher("/left_scan", LaserScan, queue_size=2)
		self.right_pub = rospy.Publisher("/right_scan", LaserScan, queue_size=2)

		rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)

		self.ar_data = None
		rospy.loginfo("Starting with " + str(self.states))
	def laser_callback(self, msg):
		self.laser_data = msg
		self.process_data()

	def ar_callback(self, msg):
		self.ar_data = msg
		self.process_data()

	def process_data(self):
		if self.ar_data != None and len(self.ar_data.markers) > 0:
			if self.loop_ar(19, self.ar_data.markers):
				self.states = States(3)
				rospy.logwarn("Switching to Overpass")			

			if self.loop_ar(20, self.ar_data.markers) and self.ar_valid_distance(20, self.ar_data.markers, 2):
				self.states = States(4)
				rospy.logwarn("Switching to Yellow Brick")	
	 
			if self.loop_ar(21, self.ar_data.markers) and self.ar_valid_distance(21, self.ar_data.markers, 1.7):
				self.states = States(5)
				rospy.logwarn("Switching to Underpass")


		elif self.states == States.MESH_WALL_FIRST: # check if on second mesh part
			# get indices for 90 degrees right and 90 degrees left
			right_index, left_index = self.get_index(180, self.laser_data)
			middle = int((right_index + left_index) / 2)

			# get the ranges for the left and right side
			scan_left = self.laser_data.ranges[middle : left_index]
			scan_right = self.laser_data.ranges[right_index : middle]
			
			#print right_index, middle, left_index
			# find the average of those ranges
			left_avg = np.mean(self.laser_data.ranges[780:908])
			right_avg = np.mean(self.laser_data.ranges[172:300])	

			#print "left_avg: ", left_avg
			#print "right_avg: ", right_avg

			if right_avg > left_avg: # greater than
				self.states = States(8)
				print right_avg
				print left_avg
				rospy.loginfo("Switching to Second Mesh")

		self.send_state()

	def send_state(self):
		state_string = String()
		if self.states == States(1):
			state_string.data = "WHS"
		elif self.states == States(2):
			state_string.data = "HPS"
		elif self.states == States(3):
			state_string.data = "OPS"
		elif self.states == States(4):
			state_string.data = "YBWS"
		elif self.states == States(5):
			state_string.data = "UPS"
		elif self.states == States(6):
			state_string.data = "BBS"
		elif self.states == States(7):
			state_string.data = "MWS"
		elif self.states == States(8):
			state_string.data = "MWS2"

		self.state_pub.publish(state_string)

	def loop_ar(self, id, markers):
		for tag in markers:
			if tag.id == id:
				return True

		return False

	def ar_valid_distance(self, id, markers, distance):
		for tag in markers:
			if tag.id == id:
				if float(tag.pose.pose.position.z) < distance:
					return True
		return False

	def get_index(self, angle, scan):

		angle_radian = angle * (np.pi/180)
		start = scan.angle_min
		end = scan.angle_max

		radian_per_index = scan.angle_increment

		offsets = angle_radian / 2
		index_offset = int(offsets / radian_per_index)
		return 540 - index_offset, 540 + index_offset

'''
	def get_avg(self, ranges):
		sum = 0
		for x in ranges:
			sum += x

		return int(sum / len(ranges))
'''

class States(Enum):
	ROLLING_WEAVE = 1
	HAIRPIN = 2
	OVERPASS = 3
	YELLOW_BRICK = 4
	UNDERPASS = 5
	BOA_BASIN = 6
	MESH_WALL_FIRST = 7
	MESH_WALL_SECOND = 8

if __name__ == "__main__":
	rospy.init_node("state_machine")
	node = MainDriver()
	rospy.spin()
