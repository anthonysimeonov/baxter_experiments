#!/usr/bin/env python

import rospy
import sensor_msgs.msg as sensor_msg
from copy import deepcopy
#import CameraInfo, Image

class CameraRerouter:
	def __init__(self):
		self.image_sub = rospy.Subscriber('/cameras/left_hand_camera/image', sensor_msg.Image, self.img_handler)
		self.info_sub = rospy.Subscriber('/camera/left_hand_camera/camera_info', sensor_msg.CameraInfo, self.info_handler)
		self.image_pub = rospy.Publisher('/cameras/virtual_camera/image', sensor_msg.Image, queue_size=0)
		self.info_pub = rospy.Publisher('/cameras/virtual_camera/camera_info', sensor_msg.CameraInfo, queue_size=0)

	def img_handler(self, data):
		self.image_pub.publish(data)
	
	def info_handler(self, data):
		data.roi.x_offset = 0
		data.roi.y_offset = 0
		self.info_pub.publish(data)

if __name__ == '__main__':
	rospy.init_node('camera_rerouter', anonymous=True)
	rate = rospy.Rate(10)
	camera_rerouter = CameraRerouter()
	while not rospy.is_shutdown():
		rate.sleep()
