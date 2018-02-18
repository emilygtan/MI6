#!/usr/bin/python

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError

class StopLightNode:
	def __init__(self):
		rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.echo_callback)

		self.vid_cmd = rospy.Publisher("Stop_Light_echo", Image, queue_size = 10) 
		self.go_msg = rospy.Publisher("/Go_msg", String, queue_size = 10)


		self.bridge = CvBridge()


	def echo_callback(self):
		self.cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
		cut_image = self.cutImage(self.cv_image)
		self.original = cut_image

		HSV = self.convertToHSV(cut_image)
		green_mask, yellow_mask, red_mask = self.thresholdImage(HSV)
		green_contours, yellow_contours, red_contours = self.getContours(green_mask, yellow_mask, red_mask)  
		self.drawBox(green_contours, yellow_contours, red_contours)

		self.drawTextGreen(self.original)
		self.drawTextYellow(self.original)
		self.drawTextRed(self.original)

		topub = self.bridge.cv2_to_imgmsg(self.original,"bgr8")
		self.vid_cmd.publish(topub)

	def cutImage(self, image):
		h, w, channels = image.shape
		new_image = image[int(0.4 * h):int(0.8 * h), 0:w]
		return new_image

	def convertToHSV(self, cut_image):
		return cv2.cvtColor(cut_image, cv2.COLOR_BGR2HSV)

	def thresholdImage(self, HSV):
		# GREEN
		green_lower_range = np.array([50,222,248])
		green_upper_range = np.array([50,222,248])
		green_mask= cv2.inRange(HSV, green_lower_range, green_upper_range)

		# YELLOW
		yellow_lower_range = np.array([30, 212, 253])
		yellow_upper_range = np.array([30, 212, 253])
		yellow_mask= cv2.inRange(HSV, yellow_lower_range, yellow_upper_range)

		# RED
		red_lower_range = np.array([1,224,224])
		red_upper_range = np.array([1,224,224])
		red_mask = cv2.inRange(HSV, red_lower_range, red_upper_range)
		return green_mask, yellow_mask, red_mask

	def getContours(self, green, yellow, red):
		# GREEN
		im2, green_contours, hierarchy = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(self.original, green_contours, -1, (255,0,0), 3)

		# YELLOW
		im2, yellow_contours, hierarchy = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(self.original, yellow_contours, -1, (255,0,0), 3)

		# RED
		im2, red_contours, hierarchy = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(self.original, red_contours, -1, (255,0,0), 3)		
		return green_contours, yellow_contours, red_contours

	def drawBox(self, green_contours, yellow_contours, red_contours):
		# GREEN
		color = (255,0,0)
		scale = 3
		a,b,c,maxh = cv2.boundingRect(green_contours[0])
		maxcontour = green_contours[0]
		for contour in green_contours:
			x,y,w,h = cv2.boundingRect(contour)
			if h>maxh:
				maxh = h
				maxcontour = contour
		
		xf,yf,wf,hf = cv2.boundingRect(maxcontour)
		cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
		self.green_boxparam = [xf,yf,wf,hf]

		# YELLOW
		color = (255,0,0)
		scale = 3
		a,b,c,maxh = cv2.boundingRect(yellow_contours[0])
		maxcontour = yellow_contours[0]
		for contour in yellow_contours:
			x,y,w,h = cv2.boundingRect(contour)
			if h>maxh:
				maxh = h
				maxcontour = contour
		
		xf,yf,wf,hf = cv2.boundingRect(maxcontour)
		cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
		self.yellow_boxparam = [xf,yf,wf,hf]

		# RED
		color = (255,0,0)
		scale = 3
		a,b,c,maxh = cv2.boundingRect(red_contours[0])
		maxcontour = red_contours[0]
		for contour in red_contours:
			x,y,w,h = cv2.boundingRect(contour)
			if h>maxh:
				maxh = h
				maxcontour = contour
		
		xf,yf,wf,hf = cv2.boundingRect(maxcontour)
		cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
		self.red_boxparam = [xf,yf,wf,hf]


	def drawTextGreen(self, image):
		color = (0,255,0)
		x = self.green_boxparam[0]
		y = self.green_boxparam[1]
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 1
		linetype = 4
		text = "Green"
		point = (x+60,y+40)
		cv2.putText(self.original, text, point, font, fontScale, color, linetype)

	def drawTextYellow(self, image):
		color = (0,255,255)
		x = self.yellow_boxparam[0]
		y = self.yellow_boxparam[1]
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 1
		linetype = 4
		text = "Yellow"
		point = (x+60,y+40)
		cv2.putText(self.original, text, point, font, fontScale, color, linetype)

	def drawTextRed(self, image):
		color = (0,0,255)
		x = self.red_boxparam[0]
		y = self.red_boxparam[1]
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 1
		linetype = 4
		text = "Red"
		point = (x+60,y+40)
		cv2.putText(self.original, text, point, font, fontScale, color, linetype)

if __name__ == "__main__":
	rospy.init_node("Stop_light_node")
	node = StopLightNode()
	rospy.spin()
