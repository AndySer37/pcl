#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
from time import time, sleep
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray

class pcl2img():
	def __init__(self): 
		rospy.Subscriber('/pcl_array', PoseArray, self.call_back)
		self.boundary = 50
		self.height = self.width = 480.0
		self.point_size = 2

	def toIMG(self, pcl_size, pcl_array, plane):
		min_m = 10e5
		min_n = 10e5
		max_m = -10e5
		max_n = -10e5
		for i in range(pcl_size):
			if min_m > pcl_array[i][0]:
				min_m = pcl_array[i][0]
			if min_n > pcl_array[i][1]:
				min_n = pcl_array[i][1]
			if max_m < pcl_array[i][0]:
				max_m = pcl_array[i][0]
			if max_n < pcl_array[i][1]:
				max_n = pcl_array[i][1]

		m_size = max_m - min_m
		n_size = max_n - min_n
		max_size = None
		min_size = None
		shift_m = False
		shift_n = False
		if m_size > n_size:
			max_size = m_size
			min_size = n_size
			shift_n = True
		else:
			max_size = n_size
			min_size = m_size
			shift_m = True
		scale = float((self.height-self.boundary*2)/max_size)
		shift_size = int(round((self.height - self.boundary*2 - min_size*scale)/2))
		img = np.zeros((int(self.height), int(self.width), 3), np.uint8)
		for i in range(pcl_size):
			if shift_m:
				pcl_array[i][0] = int(round((pcl_array[i][0] - min_m)*scale)) + shift_size + self.boundary
				pcl_array[i][1] = int(round((pcl_array[i][1] - min_n)*scale)) + self.boundary
			elif shift_n:
				pcl_array[i][0] = int(round((pcl_array[i][0] - min_m)*scale)) + self.boundary
				pcl_array[i][1] = int(round((pcl_array[i][1] - min_n)*scale)) + shift_size + self.boundary
			for m in range(-self.point_size, self.point_size + 1):
				for n in range(-self.point_size, self.point_size + 1):
					img[pcl_array[i][0] + m  , pcl_array[i][1] + n] = (0,255,255)

		cv2.imwrite( "Image_" + plane + ".jpg", img )

	def call_back(self, msg):
		#rospy.loginfo("recieve")
		#msg.data = np.asarray(points, np.float32).tostring()
		pcl_size = len(msg.poses)
		plane_xy = []
		plane_yz = []
		plane_xz = []
		#self.pcl_array_XY = self.pcl_array_YZ = self.pcl_array_XZ = msg
		for i in range(pcl_size):
			plane_xy.append([msg.poses[i].position.x, msg.poses[i].position.y])
			plane_yz.append([msg.poses[i].position.y, msg.poses[i].position.z])
			plane_xz.append([msg.poses[i].position.x, msg.poses[i].position.z])
		self.toIMG(pcl_size, plane_xy, 'xy')
		self.toIMG(pcl_size, plane_yz, 'yz')
		self.toIMG(pcl_size, plane_xz, 'xz')
		print "Save image"

if __name__ == '__main__':
	rospy.init_node('pcl2img')
	rospy.loginfo("PCL to Image")
	foo = pcl2img()
	rospy.spin()