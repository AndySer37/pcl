#!/usr/bin/env python
import rospy
from tf import TransformListener,TransformerROS, transformations
from tf import LookupException, ConnectivityException, ExtrapolationException
import roslib
import math
import scipy.stats
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObjectPose, ObjectPoseList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class mapping():
	def __init__(self): 
		# ======== Subscriber ========
		odom_sub = Subscriber("/odometry/filtered", Odometry)
		obj_sub = Subscriber("/object_list", ObjectPoseList)
		ats = ApproximateTimeSynchronizer((odom_sub, obj_sub),queue_size = 1, slop = 0.1)
		ats.registerCallback(self.call_back)
		#rospy.Subscriber("/object_list", ObjectPoseList, self.call_back, queue_size=10)
		#rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)

		# ======== Publisher ========
		self.pub_obj = rospy.Publisher("/object_list/map", ObjectPoseList, queue_size=1)
		self.pub_marker = rospy.Publisher("/obj_classify", MarkerArray, queue_size = 1)
		#pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)

		# ======== Declare Variable ========
		self.first = True
		self.robot_pose = None
		self.map = ObjectPoseList()
		self.obj_list = None
		self.frame_id = "odom"
		self.matching = []
		self.remove_list = []
		self.remove_threshold = 0.05
		self.r_threshold = 5
		self.prob_threshold = 0.3
		self.velodyne_range = 30.
		self.prior_mean = None
		self.prior_cov = None
		self.measurement_var = 1.
		self.init_varX = 1.
		self.init_varY = 1.
		self.kernel = scipy.stats.norm(loc = 0, scale = 0.5)
		# ======== Get from odometry =======
		self.sensor_error = 1.

	def call_back(self, odom_msg, obj_msg):
		rospy.loginfo("Process Object List")
		self.obj_list = ObjectPoseList()
		self.obj_list = obj_msg
		self.map_confidence = ObjectPoseList()
		self.map_confidence.header.frame_id = self.frame_id
		self.map.header.frame_id = self.frame_id
		self.matching = []
		self.remove_list = []
		vlp2robot = transformations.quaternion_from_euler(0, 0, np.pi)
		vlp2robot_matrix = transformer.fromTranslationRotation([0, 0, 0], vlp2robot)
		position = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]
		quaternion = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
		transpose_matrix = transformer.fromTranslationRotation(position, quaternion)
		self.robot_pose = np.dot(transpose_matrix, [0, 0, 0, 1])
		for obj_index in range(self.obj_list.size):
			center_x = self.obj_list.list[obj_index].position.x
			center_y = self.obj_list.list[obj_index].position.y
			center_z = self.obj_list.list[obj_index].position.z
			center  = np.array([center_x, center_y, center_z, 1])
			robot_scene = np.dot(vlp2robot_matrix, center)
			new_center = np.dot(transpose_matrix, robot_scene)
			self.obj_list.list[obj_index].position.x = new_center[0]
			self.obj_list.list[obj_index].position.y = new_center[1]
			self.obj_list.list[obj_index].position.z = new_center[2]
			self.obj_list.list[obj_index].varianceX = odom_msg.pose.covariance[0]
			self.obj_list.list[obj_index].varianceY = odom_msg.pose.covariance[7]
			self.obj_list.header.frame_id = self.frame_id
		if self.first:
			self.map = self.obj_list
			self.first = False
			for i in range(self.map.size):
				self.map.list[i].occupy = False
				#self.map.list[i].varianceX = self.init_varX
				#self.map.list[i].varianceY = self.init_varY
		else:
			for i in range(self.map.size):
				self.map.list[i].occupy = False
			self.data_associate()
			self.update_map()
			for i in range(self.map.size):
				mean_x, mean_y = self.map.list[i].position.x, self.map.list[i].position.y
				var_x, var_y = self.map.list[i].varianceX, self.map.list[i].varianceY
				prob_x = scipy.stats.norm(mean_x, var_x).pdf(mean_x)
				prob_y = scipy.stats.norm(mean_y, var_y).pdf(mean_y)
				print prob_x, prob_y
				if prob_x > self.prob_threshold and prob_y > self.prob_threshold:
					self.map_confidence.list.append(self.map.list[i])
				elif prob_x < self.remove_threshold and prob_y < self.remove_threshold:
					self.remove_list.append(i)
			for i in self.remove_list:
				del self.map.list[i]
			self.map.size = len(self.map.list)
		self.map.header.stamp = rospy.Time.now()
		print self.map.size
		self.map_confidence.size = len(self.map_confidence.list)
		self.map.header.stamp = rospy.Time.now()
		self.map_confidence.header.stamp = rospy.Time.now()
		self.pub_obj.publish(self.map)
		self.drawRviz(self.map_confidence)

	def data_associate(self):
		for i in range(self.obj_list.size):
			min_dis = 10e5
			index = None
			for j in range(self.map.size):
				if not self.map.list[j].occupy:
					dis = self.distance(self.obj_list.list[i], self.map.list[j])
					if dis < min_dis:
						index = j
						min_dis = dis
			if min_dis < self.r_threshold:
				self.map.list[index].occupy = True
			else:
				index = None
			self.matching.append(index)

	def update_map(self):
		for i in range(self.obj_list.size):
			index = self.matching[i]
			if self.matching[i] != None:
				# Kalman filter update position
				# ======= Kalman filter for x =======
				prior_mean = self.map.list[index].position.x
				prior_var = self.map.list[index].varianceX
				z_mean = self.obj_list.list[i].position.x
				z_var = self.measurement_var
				x_mean, x_var = self.kalman_filter_1D(prior_mean, prior_var, z_mean, z_var)
				self.map.list[index].position.x = x_mean
				self.map.list[index].varianceX = x_var
				# ======= Kalman filter for y =======
				prior_mean = self.map.list[index].position.y
				prior_var = self.map.list[index].varianceY
				z_mean = self.obj_list.list[i].position.y
				z_var = self.measurement_var
				y_mean, y_var = self.kalman_filter_1D(prior_mean, prior_var, z_mean, z_var)
				self.map.list[index].position.y = y_mean
				self.map.list[index].varianceY = y_var
			else:
				obj = ObjectPose()
				obj = self.obj_list.list[i]
				#obj.varianceX = self.init_varX
				#obj.varianceY = self.init_varY
				self.map.list.append(obj)
		for j in range(self.map.size):
			if not j in self.matching:
				if self.distance_to_robot(self.map.list[j]) < self.velodyne_range:
					print "Haha"
					self.map.list[j].varianceX = self.map.list[j].varianceX*1.5
					self.map.list[j].varianceY = self.map.list[j].varianceY*1.5
		self.map.size = len(self.map.list)

	def kalman_filter_1D(self, prior_mean, prior_var, z_mean, z_var):
		#======= Predict =======
		predict = scipy.stats.norm(loc = prior_mean + self.kernel.mean(), scale = np.sqrt(prior_var + self.kernel.var()))
		#======= Update step =======
		likelihood = scipy.stats.norm(loc = z_mean, scale = np.sqrt(z_var))
		posterior = self.gaussian_multiply(likelihood, predict)
		return posterior.mean(), posterior.var()

	def kalman_filter_2D(self, x, y, index):
		#======= Predict =======
		'''if self.prior_mean == None:	# Recieve first measurement
			self.prior_mean = np.array([x, y])		# State vector
			self.prior_cov = self.pos_covariance		# Covariance matrix'''
		prior_mean = np.array([self.map.list[index].position.x, self.map.list[index].position.y])
		#prior_cov = np.diag([0.5, 0.5])
		prior_cov = np.diag([self.map.list[index].varianceX, self.map.list[index].varianceY])
		F = np.array([[1., 0], [0, 1.]])			# State transition matrix
		predict_mean = np.dot(F, prior_mean)
		predict_cov = np.dot(F, prior_cov).dot(F.T)
		#predict_pos = np.random.multivariate_normal(predict_mean, predict_cov, 100)

		#======= Update step =======
		z = np.array([x, y])				# Measurement
		H = np.array([[1., 0]])				# Measurement function (Nothing to convert to measurement space)
		R = np.array([[self.sensor_error]]) # Measurement covariance (For sensor)
		S = np.dot(H, predict_cov).dot(H.T) + R 				# System uncertainty
		K = np.dot(predict_cov, H.T).dot(np.linalg.inv(S))	# Kalman gain
		residual = z - np.dot(H, predict_mean)					# Residual = measurement - prediction
		posterior_mean = predict_mean + np.dot(K.T, residual)
		posterior_cov = predict_cov - np.dot(K, H).dot(predict_cov)
		return posterior_mean, posterior_cov

	def gaussian_multiply(self, g1, g2):
		g1_mean, g1_var = g1.stats(moments='mv')
		g2_mean, g2_var = g1.stats(moments='mv')
		mean = (g1_var * g2_mean + g2_var * g1_mean) / (g1_var + g2_var)
		variance = (g1_var * g2_var) / (g1_var + g2_var)
		return scipy.stats.norm(loc = mean, scale = np.sqrt(variance))

	def distance(self, a, b): # caculate distance between two 3d points
		return math.sqrt((a.position.x-b.position.x)**2 + (a.position.y-b.position.y)**2 + (a.position.z-b.position.z)**2)

	def distance_to_robot(self, p): # caculate distance between two 3d points
		return math.sqrt((p.position.x - self.robot_pose[0])**2 + (p.position.y - self.robot_pose[1])**2 + (p.position.z - self.robot_pose[2])**2)

	def drawRviz(self, obj_list):
		marker_array = MarkerArray()
		# marker_array.markers.resize(obj_list.size)
		for i in range(obj_list.size):
			marker = Marker()
			marker.header.frame_id = obj_list.header.frame_id
			marker.id = i
			marker.header.stamp = rospy.Time.now()
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.lifetime = rospy.Duration(0.5)
			marker.pose.position.x = obj_list.list[i].position.x
			marker.pose.position.y = obj_list.list[i].position.y
			marker.pose.position.z = obj_list.list[i].position.z
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 1
			marker.scale.y = 1
			marker.scale.z = 1
			if obj_list.list[i].type == "buoy":
				marker.color.r = 0
				marker.color.g = 0
				marker.color.b = 1
				marker.color.a = 0.5
			elif obj_list.list[i].type == "totem":
				marker.color.r = 0
				marker.color.g = 1
				marker.color.b = 0
				marker.color.a = 0.5
			elif obj_list.list[i].type == "dock":
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

if __name__ == "__main__":
	rospy.init_node('mapping')
	# Tell ROS that we're making a new node.
	rospy.init_node("mapping",anonymous=False)
	rospy.loginfo("Start Mapping")
	tf_ = TransformListener()
	transformer = TransformerROS()
	foo = mapping()
	rospy.spin()