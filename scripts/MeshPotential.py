#!/usr/bin/python

from __future__ import division
import rospy
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import collections

class MeshPotential:

	def __init__(self):
		rospy.Subscriber("/state", String, self.state_callback)
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		self.state = None

		self.drive_cmd = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
		self.pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 3)
		self.range_max = 20
		self.n_turngain = .0013
		self.kp = .8
		self.kd = 1.7
		self.old_vals = collections.deque([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		
	
	def state_callback(self, msg):
		self.state = msg.data
		self.state = "BBS"
	
	def laser_callback(self, msg):
		if str(self.state) == "BBS": #change back to rolling weave node
			first_ind, last_ind = self.get_index(140, msg)
			frontf_ind, frontl_ind = self.get_index(100, msg)
			front_potential = msg.ranges[frontf_ind:frontl_ind] 
			left_potential = msg.ranges[first_ind:int(first_ind + (last_ind - first_ind)/2)]
			right_potential = msg.ranges[int(first_ind + (last_ind-first_ind)/2): last_ind]
		
			raw_turn = self.calc_potential(left_potential, self.n_turngain) - self.calc_potential(right_potential, self.n_turngain)
		
			#Derivative 
		
		
			der_turn = ((self.old_vals.popleft()-self.kp*raw_turn)/10)
			pro_turn = self.kp*raw_turn + self.kd*der_turn

			speed_potential = 1/(1.2*abs(raw_turn))+.3
			if(speed_potential > 5.0 ):
				speed_potential = 5.0
			elif(speed_potential < .6):
				speed_potential = .6
			print "speed_potential: " + str(speed_potential)

			print "raw turn: " + str(raw_turn)
			print "derivative turn: " + str(der_turn)
			print "pro_turn: " + str(pro_turn)
			print "right potential: " + str(self.calc_potential(right_potential, self.n_turngain))
			print "left potential: " + str(self.calc_potential(left_potential, self.n_turngain)) 
			drive_msg = AckermannDriveStamped()
			drive_msg.drive.steering_angle = pro_turn
			drive_msg.drive.speed = speed_potential
			self.pub.publish(drive_msg)
		
			self.old_vals.append(pro_turn)

		
	def get_index(self, angle, scan):

		angle_radian = angle * (np.pi/180)
		start = scan.angle_min
		end = scan.angle_max

		radian_per_index = scan.angle_increment

		offsets = angle_radian / 2
		index_offset = int(offsets / radian_per_index)
		return 540 - index_offset, 540 + index_offset
	
	def calc_potential(self, angle_array, ngain):
		rho = 0
		rho_0 = 200
		pot_force = 0
		for i in range(0, len(angle_array)):
			rho = angle_array[i]
			if(rho <= rho_0):
				pot_force += (ngain/2)*(((1/rho)-(1/rho_0))**2 )
		return pot_force


if __name__ == "__main__":
	rospy.init_node("Mesh_potential")
	node = MeshPoetential()
	rospy.spin()
