#!/usr/bin/python

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String 

class WaterYellowBrickNode:
	def __init__(self):
		rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.echo_command_callback)
		rospy.Subscriber("/state", String, self.state_callback)

		self.vid_cmd = rospy.Publisher("YellowBrick_echo", Image, queue_size = 10) 
		self.drive_cmd = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

		self.des_speed = 2  #3.5
		self.error = 0
		self.error_rate = 0
		self.center_offset = 0

		self.Kp = 0.0012 #0.0015
		self.Kd = 0.0006 #0.0006

		self.old_error = 0
		self.bridge = CvBridge()

		self.state = None
		self.just_started = True
	def state_callback(self, msg):
		self.state = msg.data
		print "state is: " + self.state

	def echo_command_callback(self, image):
		if self.state == "YBWS":
			if self.just_started:
				forwards = AckermannDriveStamped()
				forwards.drive.speed = 3
				for x in range(0, 100):
					rospy.logwarn("HARD CODE FORWARDS")
					self.drive_cmd.publish(forwards)
				self.just_started = False

			self.cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
			cut_image = self.cutImage(self.cv_image)
			self.original = cut_image

			HSV = self.convertToHSV(cut_image)
			thresh = self.thresholdImage(HSV)
			contours, img = self.getContours(thresh)  
			
			center_x = None # <-- worst programming ever lols
			center_y = None
			if len(contours) > 0:
				sorted_contours = sorted (contours, key=cv2.contourArea, reverse=True)
				largest_cnt = sorted_contours[0]

				#find center of contour
				M = cv2.moments(largest_cnt)
				if M['m00'] != 0:
					center_x = int(M['m10']/M['m00'])			
					center_y = int(M['m01']/M['m00'])
					cv2.circle(self.original, (center_x, center_y), 20, (0, 255, 255), 3)

				self.line_follower(center_x)

			#self.drawBox(img, contours)
			#self.drawText(self.original)

			topub = self.bridge.cv2_to_imgmsg(self.original,"bgr8")
			self.vid_cmd.publish(topub)
			#print "enable"
			
	def cutImage(self, image):
		h, w, channels = image.shape
		new_image = image[int(0.45 * h):int(0.8 * h), 0:w] #0.45 0.8
		return new_image

	def convertToHSV(self, image):
		HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		HSV = cv2.blur(HSV, (7,15))
		return HSV

	def thresholdImage(self, image):
		yellow_lower_range = np.array([20, 215, 170])
		yellow_upper_range = np.array([35,255,255])
		mask = cv2.inRange(image, yellow_lower_range, yellow_upper_range)
		#mask= cv2.blur(mask, (7,11))
		return mask

	def getContours(self, image):
		ret, thresh = cv2.threshold(image, 127, 255, 0)
		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contours, cv2.drawContours(self.original, contours, -1, (0,255,0), 3)

	def drawBox(self, image, contours):
		color = (255,0,0)
		scale = 3
		a,b,c,maxh = cv2.boundingRect(contours[0])
		maxcontour = contours[0]
		for contour in contours:
			#print contour
			x,y,w,h = cv2.boundingRect(contour)
			if h>maxh:
				maxh = h
				maxcontour = contour
		
		xf,yf,wf,hf = cv2.boundingRect(maxcontour)
		cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
		self.boxparam = [xf,yf,wf,hf]


	def drawText(self, image):
		color = (0,255,0)
		x = self.boxparam[0]
		y = self.boxparam[1]
		w = self.boxparam[2]
		h = self.boxparam[3]
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 1
		linetype = 4
		text = str(w)
		point = (x,y-20)
		cv2.putText(self.original,text,point,font, fontScale,color,linetype)
		cv2.putText(self.original,str(x-(self.original.shape[1]/2)),(x+100, y-20), font, fontScale, (255, 0, 0), linetype)

	def line_follower(self, center_x):
		#x = boxparam[0]
		#w = boxparam[2]

		# Proportional controller
		height, width, channels = self.cv_image.shape
	
		center = width/2 + self.center_offset
		self.error = center - center_x
		
		# Derivative controller
		new_error = self.error
		delta_error = new_error - self.old_error
		self.old_error = self.error
		self.error_rate = delta_error

		self.run_park_command(self.error, self.error_rate)

	def run_park_command(self, error, error_rate):
		p_control = self.Kp*error
		d_control = self.Kd*error_rate

		u_steer = p_control + d_control

		steer_saturation = 0.34

		if u_steer > steer_saturation:
			u_steer = steer_saturation
		elif u_steer < -steer_saturation:
			u_steer = -steer_saturation

		speed = 1/(3*abs(u_steer))+.3


		if speed > self.des_speed:
			speed = self.des_speed
		elif speed < .6:
			speed = .6
		
		print "speed: ", speed

		u_msg = AckermannDriveStamped()
		u_msg.drive.steering_angle = u_steer
		u_msg.drive.speed = speed
		u_msg.header.stamp = rospy.Time.now()
		self.drive_cmd.publish(u_msg)	

if __name__ == "__main__":
	rospy.init_node("WaterYellowBrickNode")
	node = WaterYellowBrickNode()
	rospy.spin()
