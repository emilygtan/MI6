#!/usr/bin/python

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class Safety:
	def __init__(self):
		rospy.Subscriber ("/scan", LaserScan, self.laserScan_callback)
		rospy.Subscriber ("ackermann_cmd_mux/output", AckermannDriveStamped, self.ackermann_cmd_input_callback)
		self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)

		self.safe = True
		self.front = 0.4
		self.left = -0.125
		self.right = 0.125
		
	def laserScan_callback(self, msg):
		count = 0
		for x in range(180,900):
			currentAngle = (x-180) / 4
			currentAngle = math.radians(currentAngle)
			currentRange = msg.ranges[x]
			xposs = currentRange * math.cos(currentAngle)
			yposs = currentRange * math.sin(currentAngle)
		
			if xposs > self.left and xposs < self.right and yposs < self.front:
				count = count + 1
			if count > 4:
				self.safe = False
			else:
				self.safe = True

	def ackermann_cmd_input_callback(self, msg):
		print("ackermann_callback")
	
		if self.safe or msg.drive.speed < 0:
			self.cmd_pub.publish(msg)
			rospy.loginfo("go")
			
		else:
			stop_msg = AckermannDriveStamped()
			stop_msg.drive.speed = 0.0
			stop_msg.drive.steering_angle = msg.drive.steering_angle
			stop_msg.header.stamp = rospy.Time.now()
			self.cmd_pub.publish(stop_msg)
			rospy.loginfo("stop")

if __name__ == "__main__":
 	rospy.init_node("safety_controller_node")
 	node = Safety()
 	rospy.spin()
