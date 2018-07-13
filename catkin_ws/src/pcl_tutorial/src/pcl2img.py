#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from robotx_msgs.msg import PCL_points
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import sys
from os.path import expanduser
caffe_root = expanduser("~")
sys.path.insert(0, caffe_root + '/caffe/python')
import caffe
import os

class pcl2img():
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		rospy.Subscriber('/pcl_points', PCL_points, self.call_back)
		#rospy.Subscriber('/pcl_array', PoseArray, self.call_back)
		self.boundary = 50
		self.height = self.width = 480.0
		self.point_size = 4	# must be integer
		self.image = np.zeros((int(self.height), int(self.width), 3), np.uint8)
		self.index = 0
		# ***************************************************************
		# Get the position of caffemodel folder
		# ***************************************************************
		#self.model_name = rospy.get_param('~model_name')
		self.model_name = "CaffeNet"
		rospy.loginfo('[%s] model name = %s' %(self.node_name, self.model_name))
		rospack = rospkg.RosPack()
		self.model_Base_Dir = rospack.get_path('pcl_tutorial') + '/models/' + self.model_name + '/'
		self.labels = []
		with open(self.model_Base_Dir+'label.txt', 'r') as f:
			lines = f.readlines()
		for line in lines:
			line = line.replace('\n', '')
			self.labels.append(line)
        # ***************************************************************
        # Variables
        # ***************************************************************
		self.bridge = CvBridge()
		self.pred_count = 0
		self.dim = (227, 227)
		self.img = None

		# ***************************************************************
		# Set up caffe
		# ***************************************************************

		self.model_def = self.model_Base_Dir + self.model_name.lower() + '.prototxt'
		self.model_weights = self.model_Base_Dir + self.model_name.lower() + '.caffemodel'
		self.net = caffe.Net(	self.model_def,        # defines the structure of the model
								self.model_weights,    # contains the trained weights
								caffe.TEST)


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
					if plane == 'xy':
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][0] = 255
					elif plane == 'yz':
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][1] = 255
					elif plane == 'xz':
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][2] = 255
		#cv2.imwrite( "Image_" + plane + ".jpg", img )

	def classify(self):
		# ***************************************************************
		# Using Caffe Model to do prediction
		# ***************************************************************
		img = cv2.resize(self.image, self.dim)
		caffe.set_device(0)
		caffe.set_mode_gpu()
		t_start = time.clock()
		transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
		transformer.set_transpose('data', (2, 0, 1))
		self.net.blobs['data'].reshape(1, 3, self.dim[0], self.dim[1])
		self.net.blobs['data'].data[...] = transformer.preprocess('data', img)
		output = self.net.forward()
		output_prob = output['prob'][0]
		output_max_class = output_prob.argmax()
		print "prediction time taken = ", time.clock() - t_start
		print "Predict: ", self.labels[output_max_class]


	def call_back(self, msg):
		cluster_num = len(msg.list)
		#pcl_size = len(msg.poses)
		for i in range(cluster_num):
			self.image = np.zeros((int(self.height), int(self.width), 3), np.uint8)
			plane_xy = []
			plane_yz = []
			plane_xz = []
			pcl_size = len(msg.list[i].poses)
			for j in range(pcl_size): # project to XY, YZ, XZ plane
				plane_xy.append([msg.list[i].poses[j].position.x, msg.list[i].poses[j].position.y])
				plane_yz.append([msg.list[i].poses[j].position.y, msg.list[i].poses[j].position.z])
				plane_xz.append([msg.list[i].poses[j].position.x, msg.list[i].poses[j].position.z])
			self.toIMG(pcl_size, plane_xy, 'xy')
			self.toIMG(pcl_size, plane_yz, 'yz')
			self.toIMG(pcl_size, plane_xz, 'xz')
			self.classify()
			#cv2.imwrite( "Image" + str(self.index) + ".jpg", self.image)
			#self.index = self.index + 1
			#print "Save image"

if __name__ == '__main__':
	rospy.init_node('pcl2img')
	foo = pcl2img()
	rospy.spin()