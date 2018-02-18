#!/usr/bin/python

import rospy 
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped

class parkNode:
	def __init__(self):
		rospy.Subscriber("/ar_position", Point, self.park_command)
		self.drive_cmd = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
		self.error_width = 0

		self.Kp_steer = 1.34
		self.Kp_speed = 1.7
		print "Running"

	def park_command(self, x):
		# Proportional controller
		des_x = 0.01
		print("x.data: " + str(x.x))
		self.error_width = des_x - x.x
		
		distance = x.z
		print("width: " + str(self.error_width))
		print("distance: " + str(distance))
		
		self.run_park_command(self.error_width, distance)

	def run_park_command(self, error_width, distance):
		p_control_steer = self.Kp_steer * error_width
		p_control_speed = self.Kp_speed * distance

		u_steer = p_control_steer
		saturation = 0.3

	
		max_speed_sat = 0.7

		if u_steer > 0.3:
			u_steer = saturation
		elif saturation < -0.3:
			u_steer = -saturation

		if p_control_speed > max_speed_sat:
			p_control_speed = max_speed_sat

		u_speed = p_control_speed

		print("u_steer: "+ str(u_steer))
		u_msg = AckermannDriveStamped()
		u_msg.drive.steering_angle = u_steer
		u_msg.drive.speed = u_speed
		u_msg.header.stamp = rospy.Time.now()

		if distance > 0.25:
			self.drive_cmd.publish(u_msg)

if __name__ == "__main__":
    rospy.init_node("park_controller")
    node = parkNode()
    rospy.spin()
