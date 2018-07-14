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
from visualization_msgs.msg import Marker, MarkerArray
from robotx_msgs.msg import PCL_points, ObstaclePose, ObstaclePoseList
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
		self.pub_marker = rospy.Publisher("/obs_classify", MarkerArray, queue_size = 1)
		#rospy.Subscriber('/pcl_array', PoseArray, self.call_back)
		self.boundary = 50
		self.height = self.width = 480.0
		self.point_size = 4	# must be integer
		self.image = np.zeros((int(self.height), int(self.width), 3), np.uint8)
		#self.index = 0

		# ***************************************************************
		# Get the position of caffemodel folder
		# ***************************************************************
		#self.model_name = rospy.get_param('~model_name')
		self.model_name = "CaffeNet"
		rospy.loginfo('[%s] model name = %s' %(self.node_name, self.model_name))
		rospack = rospkg.RosPack()
		self.model_Base_Dir = rospack.get_path('classify') + '/models/' + self.model_name + '/'
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
		self.model_weights = self.model_Base_Dir + self.model_name.lower() + '_xyz.caffemodel'
		self.net = caffe.Net(	self.model_def,        # defines the structure of the model
								self.model_weights,    # contains the trained weights
								caffe.TEST)


	def toIMG(self, pcl_size, pcl_array, plane):
		min_x = 10e5
		min_y = 10e5
		min_z = 10e5
		max_x = -10e5
		max_y = -10e5
		max_z = -10e5
		for i in range(pcl_size):
			if min_x > pcl_array[i][0]:
				min_x = pcl_array[i][0]
			if min_y > pcl_array[i][1]:
				min_y = pcl_array[i][1]
			if min_z > pcl_array[i][2]:
				min_z = pcl_array[i][2]
			if max_x < pcl_array[i][0]:
				max_x = pcl_array[i][0]
			if max_y < pcl_array[i][1]:
				max_y = pcl_array[i][1]
			if max_z < pcl_array[i][2]:
				max_z = pcl_array[i][2]
		x_size = max_x - min_x
		y_size = max_y - min_y
		z_size = max_z - min_z
		z_norm = None
		if z_size != 0:
			z_norm = 254.0/z_size
		max_size = None
		min_size = None
		shift_m = False
		shift_n = False
		if x_size > y_size:
			max_size = x_size
			min_size = y_size
			shift_n = True
		else:
			max_size = y_size
			min_size = x_size
			shift_m = True
		scale = float((self.height-self.boundary*2)/max_size)
		shift_size = int(round((self.height - self.boundary*2 - min_size*scale)/2))
		img = np.zeros((int(self.height), int(self.width), 3), np.uint8)
		for i in range(pcl_size):
			if shift_m:
				pcl_array[i][0] = int(round((pcl_array[i][0] - min_x)*scale)) + shift_size + self.boundary
				pcl_array[i][1] = int(round((pcl_array[i][1] - min_y)*scale)) + self.boundary
			elif shift_n:
				pcl_array[i][0] = int(round((pcl_array[i][0] - min_x)*scale)) + self.boundary
				pcl_array[i][1] = int(round((pcl_array[i][1] - min_y)*scale)) + shift_size + self.boundary
			for m in range(-self.point_size, self.point_size + 1):
				for n in range(-self.point_size, self.point_size + 1):
					if z_norm != None:
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][0] = int(round(pcl_array[i][2]*z_norm))
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][1] = int(round(pcl_array[i][2]*z_norm))
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][2] = int(round(pcl_array[i][2]*z_norm))
					else:
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][0] = 255
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][2] = 255
						self.image[pcl_array[i][0] + m  , pcl_array[i][1] + n][1] = 255
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
		return self.labels[output_max_class]

	def drawRviz(self, obs_list):
		marker_array = MarkerArray()
		# marker_array.markers.resize(obs_list.size)
		for i in range(obs_list.size):
			marker = Marker()
			marker.header.frame_id = obs_list.header.frame_id
			marker.id = i
			marker.header.stamp = rospy.Time.now()
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.lifetime = rospy.Duration(0.5)
			marker.pose.position.x = obs_list.list[i].x
			marker.pose.position.y = obs_list.list[i].y
			marker.pose.position.z = obs_list.list[i].z
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 1
			marker.scale.y = 1
			marker.scale.z = 1
			if obs_list.list[i].type == "buoy":
				marker.color.r = 0
				marker.color.g = 0
				marker.color.b = 1
				marker.color.a = 0.5
			elif obs_list.list[i].type == "totem":
				marker.color.r = 0
				marker.color.g = 1
				marker.color.b = 0
				marker.color.a = 0.5
			elif obs_list.list[i].type == "dock":
				marker.color.r = 1
				marker.color.g = 1
				marker.color.b = 1
				marker.color.a = 0.5
				marker.scale.x = 6
				marker.scale.y = 6
				marker.scale.z = 1
			else:
				marker.color.r = 1
				marker.color.g = 0
				marker.color.b = 0
				marker.color.a = 0.5
			marker_array.markers.append(marker)
		self.pub_marker.publish(marker_array)

	def call_back(self, msg):
		obs_list = ObstaclePoseList()
		obs_list.header.frame_id = msg.header.frame_id
		obs_list.size = 0
		cluster_num = len(msg.list)
		#pcl_size = len(msg.poses)
		for i in range(cluster_num):
			self.image = np.zeros((int(self.height), int(self.width), 3), np.uint8)
			img = []
			pcl_size = len(msg.list[i].poses)
			avg_x = avg_y = avg_z = 0
			for j in range(pcl_size): # project to XY, YZ, XZ plane
				avg_x = avg_x + msg.list[i].poses[j].position.x
				avg_y = avg_y + msg.list[i].poses[j].position.y
				avg_z = avg_z + msg.list[i].poses[j].position.z
				img.append([msg.list[i].poses[j].position.x, msg.list[i].poses[j].position.y, msg.list[i].poses[j].position.z])
			self.toIMG(pcl_size, img, 'img')
			model_type = self.classify()
			# ***************************************************************
			# Add to obstacle list
			# ***************************************************************
			obs = ObstaclePose()
			obs.x = float(avg_x/pcl_size)
			obs.y = float(avg_y/pcl_size)
			obs.z = float(avg_z/pcl_size)
			obs.type = model_type
			obs_list.list.append(obs)
			cv2.imwrite( "Image.jpg", self.image)
			#self.index = self.index + 1
			#print "Save image"
		obs_list.size = cluster_num
		self.drawRviz(obs_list)

if __name__ == '__main__':
	rospy.init_node('pcl2img')
	foo = pcl2img()
	rospy.spin()