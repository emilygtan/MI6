#!/usr/bin/python

import rospy
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String



class MeshWall:
	def __init__(self):
		rospy.Subscriber("/state", String, self.mesh_wall)
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)

		self.drive_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

		# Desired distance from the wall
		self.d_des = 0.75 #left_.4 right_.9
		self.speed_des = 2

		# Controller gains
		self.Kp = 1.35
		self.Kd = .02

		# Error variables
		self.error = 0
		self.error_rate = 0

		# Time variables
		self.old_time = time.time()
		self.old_error = 0

		self.status = None

		self.new_wall_trigger = False

	def mesh_wall(self, msg):
		self.status = msg.data

	def laser_callback(self, msg):
		if self.status != None:
			right = None
			left = None

			if self.status == "MWS": # right MWS
				a = 172
				b = 300
				left = False
				right = True
				#rospy.loginfo("Mesh Wall One")

			elif self.status == "MWS2": # second part # left MWS2
				a = 780
				b = 908
				left = True
				right = False

			#	if not self.new_wall_trigger:
			#		self.d_des = np.mean(msg.ranges[a:b])
			#	rospy.loginfo("New Distance: " + str(self.d_des))
				self.new_wall_trigger = True

				#rospy.loginfo("Mesh Wall Two")

			if right or left:
				#rospy.loginfo("Starting Mesh Wall")
				
				smallestDistance = 300
				for i in range(a, b):
					if msg.ranges[i] < smallestDistance:
						smallestDistance = msg.ranges[i]
			 			smallestIndex = i
				self.error = self.d_des - smallestDistance
				print "error: ", self.error

				new_time = time.time()
				new_error = self.error
				delta_time = new_time - self.old_time
				delta_error = new_error - self.old_error
				self.old_time = time.time()
				self.old_error = self.error
				#print 'time: ', delta_time
				#print 'error: ', delta_error
				self.error_rate = delta_error/delta_time
				print "error rate: ", self.error_rate

				p_control = self.Kp * self.error
				d_control = self.Kd * self.error_rate

				saturation = 0.3

				if right:
					u_steer = p_control# + d_control
					if u_steer > saturation:
						u_steer = 0.3 
					elif u_steer < -saturation:
						u_steer = -0.3

				elif left:
					u_steer = -(p_control)# + d_control)
					print 'u_steer:', u_steer
					if u_steer > saturation:
						u_steer = 0.3 
					elif u_steer < -saturation:
						u_steer = -0.3

				u_steer_msg = AckermannDriveStamped()
				u_steer_msg.drive.steering_angle = u_steer
				u_steer_msg.drive.speed = self.speed_des
				u_steer_msg.header.stamp = rospy.Time.now()
				self.drive_pub.publish(u_steer_msg)

if __name__ == "__main__":
	rospy.init_node("mesh_wall")
	node = MeshWall()
	rospy.spin()
