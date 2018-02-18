#!/usr/bin/python

import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point

class arMonitor:
	def __init__(self):
		rospy.Subscriber ("/ar_pose_marker", AlvarMarkers, self.ar_callback)
		self.x_pub = rospy.Publisher("/ar_position", Point, queue_size=3)

		self.ar_id = 1
		print "Running"
	def getAngle(self, x, y, z, w):
		(r, p, y) = tf.transformations.euler_from_quaternion([x, y, z, w])
		rospy.loginfo("yaw_rot" + str(p))
	def ar_callback(self, msg):
		if len(msg.markers) > 0:
			self.getAngle(msg.markers[0].pose.pose.orientation.x, msg.markers[0].pose.pose.orientation.y, msg.markers[0].pose.pose.orientation.z, msg.markers[0].pose.pose.orientation.w)
			rospy.loginfo("x_loc: " + str(msg.markers[0].pose.pose.position.x))
			rospy.loginfo("y_loc: " + str(msg.markers[0].pose.pose.position.y)) 

			x_msg = Point()
			x_msg.x = msg.markers[0].pose.pose.position.x
			x_msg.z = msg.markers[0].pose.pose.position.z

			#if msg.markers[0].id == self.ar_id
			self.x_pub.publish(x_msg)
	
		else:
			x_msg = Point()
			x_msg.x = 0.01
			x_msg.z = 0.2
			self.x_pub.publish(x_msg)
if __name__ == "__main__":
    rospy.init_node("ar_monitor")
    node = arMonitor()
    rospy.spin()

