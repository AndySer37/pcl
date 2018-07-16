#!/usr/bin/env python
import rospy
from tf import TransformListener,TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
import roslib
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObjectPose, ObjectPoseList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class pcl2img():
	def __init__(self): 
		# ======== Subscriber ========
		rospy.Subscriber("/obj_list", ObjectPoseList, call_back, queue_size=10)
		#rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)

		# ======== Publisher ========
		pub_obj = rospy.Publisher("/object_list/map", ObjectPoseList, queue_size=1)
		#pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)

		# ======== Declare Variable ========
		self.map = ObjectPoseList()
		self.obj_list = None
		self.matching = []
		self.r_threshold = 1
		self.prior_mean = None
		self.prior_cov = None
		self.covX = None
		self.covY = None
		# ======== Get from odometry =======
		self.pos_covariance = np.diag([3., 3.])
		self.sensor_error = 1.

	def call_back(self, msg):
		try:
			rospy.loginfo("Process Object List")
			self.obj_list = ObjectPoseList()
			self.obj_list = msg
			self.matching = []
			position, quaternion = tf_.lookupTransform( "/map", "/velodyne",rospy.Time(0))
			transpose_matrix = transformer.fromTranslationRotation(position, quaternion)
			for obj_index in range(self.obj_list.size):
				center_x = self.obj_list.list[obj_index].position.x
				center_y = self.obj_list.list[obj_index].position.y
				center_z = self.obj_list.list[obj_index].position.z
				center  = np.array([center_x, center_y, center_z, 1])
				new_center = np.dot(transpose_matrix, center)
				self.obj_list.list[obj_index].position.x = new_center[0]
				self.obj_list.list[obj_index].position.y = new_center[1]
				self.obj_list.list[obj_index].position.z = new_center[2]
				self.obj_list.header.frame_id = "map"
			if self.first:
				self.obj_list = self.map
			else:
				for i in range(self.map.size):
					self.map.list[i].occupy = False
				self.data_associate()
				self.update_map()
			pub_obj.publish(self.obj_list)

		except (LookupException, ConnectivityException, ExtrapolationException):
			print "TF recieve error"

	def data_associate(self):
		for i in range(self.obj_list.size):
			min_dis = 10e5
			index = None
			for j in range(self.map.size):
				if self.map.list[j].occupy:
					dis = self.distance(self.obj_list.list[i], self.map.list[j])
					if dis < min_dis:
						index = j
						min_dis = dis
			if min_dis < self.r_threshold:
				self.map.list[index].occupy = True
			self.matching.append(index)

	def update_map(self):
		for i in range(self.obj_list.size):
			index = self.matching[i]
			if self.matching[i] != None:
				# Kalman filter update position
				pos, cov = self.kalman_filter(self.obj_list.list[i].position.x, self.obj_list.list[i].position.y)	
				self.map.list[index].position.x = pos[0]
				self.map.list[index].position.y = pos[1]
				self.map.list[index].covarianceX = cov[0][0]
				self.map.list[index].covarianceY = cov[1][1]
				self.covX = cov[0][0]
				self.covY = cov[1][1]
			else:
				obj = ObjectPose()
				obj.position = self.obj_list.list[i]
				obj.covarianceX = self.covX
				obj.covarianceY = self.covY
				self.map.list.append(obj)
		self.map.size = len(self.map.list)


	def kalman_filter(self, x, y):
		#======= Predict =======
		if self.prior_mean == None:	# Recieve first measurement
			self.prior_mean = np.array([x, y])		# State vector
			self.prior_cov = self.pos_covariance		# Covariance matrix
		F = np.array([[1., 0], [0, 1.]])			# State transition matrix
		predict_mean = np.dot(F, self.prior_mean)
		predict_cov = np.dot(F, self.prior_cov).np.dot(F.T)
		#predict_pos = np.random.multivariate_normal(predict_mean, predict_cov, 100)

		#======= Update step =======
		z = np.array([x, y])				# Measurement
		H = np.array([[1., 0]])				# Measurement function (Nothing to convert to measurement space)
		R = np.array([[self.sensor_error]]) # Measurement covariance (For sensor)
		S = np.dot(H, predict_cov).np.dot(H.T) + R 				# System uncertainty
		K = np.dot(predict_cov, H.T).np.dot(np.linalg.inv(S))	# Kalman gain
		residual = z - np.dot(H, predict_mean)					# Residual = measurement - prediction
		posterior_mean = predict_mean + np.dot(K, residual)
		posterior_cov = predict_cov - np.dot(K, H).dot(predict_cov)
		self.prior_mean = posterior_mean
		self.prior_cov = posterior_cov
		return posterior_mean, posterior_cov

	def distance(self, a, b): # caculate distance between two 3d points
		return math.sqrt((a.position.x-b.position.x)**2 + (a.position.y-b.position.y)**2 + (a.position.z-b.position.z)**2)

if __name__ == "__main__":
	rospy.init_node('mapping')
	# Tell ROS that we're making a new node.
	rospy.init_node("mapping",anonymous=False)
	rospy.loginfo("Start Mapping")
	tf_ = TransformListener()
	transformer = TransformerROS()
	foo = mapping()
	rospy.spin()