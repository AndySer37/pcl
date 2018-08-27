#!/usr/bin/env python
import rospy
import roslib
import math
import scipy.stats
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObjectPose, ObjectPoseList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import numpy as np

class mapping_pcl():
	def __init__(self): 
		# ======== Subscriber ========
		rospy.Subscriber("/obj_list", ObjectPoseList, self.call_back, queue_size=1)
		#rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)

		# ======== Publisher ========
		self.pub_obj = rospy.Publisher("/obj_list/map", ObjectPoseList, queue_size=1)
		self.pub_marker = rospy.Publisher("/obj_marker/map", MarkerArray, queue_size = 1)
		self.pub_path = rospy.Publisher("/slam/path", Path, queue_size = 1)
		#pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)

		# ======== Get ROS parameter ========
		self.visual = rospy.get_param('~visual', False)

		# ======== Declare Variable ========
		self.cosine = None
		self.sine = None
		self.path = Path()
		self.old_obj_list = None
		self.first = True
		self.old_pos = [0, 0, 0]
		self.new_pos = [0, 0, 0]
		self.map = ObjectPoseList()
		self.obj_list = None
		self.frame_id = "odom"
		self.matching = []
		self.remove_list = []
		self.remove_threshold = 0.05
		self.r_threshold = 10
		self.prob_threshold = 0.3
		self.update_range = 40.
		self.punish_range = 20.
		self.classify_range = 10.
		self.prior_mean = None
		self.prior_cov = None
		self.punish_no_detect = 2
		self.punish_unclassify = 1.5
		self.measurement_var = 1.
		self.init_varX = 2.
		self.init_varY = 2.
		self.kernel = scipy.stats.norm(loc = 0, scale = 0.5)
		# ======== Get from odometry =======
		#self.pos_covariance = np.diag([3., 3.])
		self.sensor_error = 1.

	def call_back(self, msg):
		#rospy.loginfo("Process Object List")
		self.obj_list = ObjectPoseList()
		self.obj_list = msg
		self.map_confidence = ObjectPoseList()
		self.map_confidence.header.frame_id = self.frame_id
		self.map.header.frame_id = self.frame_id
		self.matching = []
		self.remove_list = []
		self.obj_list.header.frame_id = self.frame_id
		# First obj_list recieved
		if self.first:
			self.first = False
			self.map = self.obj_list
			self.old_pos = [0, 0, 0]
			self.new_pos = [0, 0, 0]
			for i in range(self.map.size):
				self.map.list[i].occupy = False
				self.map.list[i].varianceX = self.init_varX
				self.map.list[i].varianceY = self.init_varY
		else:
			self.old_pos = self.new_pos
			for i in range(self.map.size):
				self.map.list[i].occupy = False
			self.data_associate()
			self.estimate_transform()
			self.map = self.obj_list
			#self.update_map()
			'''for i in range(self.map.size):
				mean_x, mean_y = self.map.list[i].position.x, self.map.list[i].position.y
				var_x, var_y = self.map.list[i].varianceX, self.map.list[i].varianceY
				prob_x = scipy.stats.norm(mean_x, var_x).pdf(mean_x)
				prob_y = scipy.stats.norm(mean_y, var_y).pdf(mean_y)
				#print prob_x, prob_y
				if prob_x > self.prob_threshold and prob_y > self.prob_threshold:
					self.map_confidence.list.append(self.map.list[i])
				elif prob_x < self.remove_threshold and prob_y < self.remove_threshold:
					self.remove_list.append(i)
			remove_num = 0	#ensure the index are correct during removing
			for i in self.remove_list:
				del self.map.list[i - remove_num]
				remove_num = remove_num + 1
			self.map.size = len(self.map.list)
		self.map.header.stamp = rospy.Time.now()
		self.map_confidence.size = len(self.map_confidence.list)
		self.map.header.stamp = rospy.Time.now()
		self.map_confidence.header.stamp = rospy.Time.now()
		self.pub_obj.publish(self.map)
		self.drawRviz(self.map_confidence)'''

	def data_associate(self):
		for i in range(self.obj_list.size):
			min_dis = 10e5
			index = None
			for j in range(self.map.size):
				if self.map.list[j].type == self.obj_list.list[i].type:
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

	def compute_center(self):
		sum_obj_x = 0.
		sum_obj_y = 0.
		sum_map_x = 0.
		sum_map_y = 0.
		for i in range(len(self.matching)):
			if self.matching[i] != None:
				map_index = self.matching[i]
				sum_obj_x = sum_obj_x + self.obj_list.list[i].position.x
				sum_obj_y = sum_obj_y + self.obj_list.list[i].position.y
				sum_map_x = sum_map_x + self.map.list[map_index].position.x
				sum_map_y = sum_map_y + self.map.list[map_index].position.y
		avg_obj_x = sum_obj_x/len(self.matching)
		avg_obj_y = sum_obj_y/len(self.matching)
		avg_map_x = sum_map_x/len(self.matching)
		avg_map_y = sum_map_y/len(self.matching)
		return avg_obj_x, avg_obj_y, avg_map_x, avg_map_y

	def estimate_transform(self):
		obj_cx, obj_cy, map_cx, map_cy = self.compute_center()
		obj_list = []
		map_list = []
		matching_num = 0
		cs = 0.
		ss = 0.
		rr = 0.
		ll = 0.
		# ========== Similarity Transform ==========
		for i in range(len(self.matching)):
			if self.matching[i] != None:
				matching_num = matching_num + 1
				map_index = self.matching[i]
				obj_ = [self.obj_list.list[i].position.x-obj_cx, self.obj_list.list[i].position.y-obj_cy]
				map_ = [self.map.list[map_index].position.x-map_cx, self.map.list[map_index].position.y-map_cy]
				obj_list.append(obj_)
				map_list.append(map_)
				cs = cs + (obj_[0]*map_[0] + obj_[1]*map_[1])
				ss = ss + (-obj_[0]*map_[1] + obj_[1]*map_[0])
				rr = rr + (obj_[0]*obj_[0] + obj_[1]*obj_[1])
				ll = ll + (map_[0]*map_[0] + map_[1]*map_[1])
		if matching_num >= 2: # We need four position(2 pairs) to get the unique solution
			lamda = math.sqrt(rr/ll)
			#print lamda
			c = cs / math.sqrt(cs**2 + ss**2)
			s = ss / math.sqrt(cs**2 + ss**2)
			txty = np.array([obj_cx, obj_cy]) - lamda*(np.array([[c,-s],[s, c]]).dot(np.array([map_cx, map_cy])))
			tx = txty[0]
			ty = txty[1]
			translation = [tx, ty]
			angle = (np.arcsin(s)/np.pi)*180
			print angle
			txty_, c_, s_ = self.lidar2robot(txty, c, s, [map_cx, map_cy])
			#print "==============="
			rot_mat = np.array([[c_, -s_], [s_, c_]])
			pre_pos = np.array([map_cx, map_cy])
			tran_mat = np.array(txty_)
			new_pos_wo_rot = lamda * rot_mat.dot([0, 0]) + tran_mat
			new_pos = new_pos_wo_rot
			new_pos = lamda * rot_mat.dot(new_pos_wo_rot)
			self.new_pos[:2] = [self.old_pos[0] + new_pos[0], self.old_pos[1] + new_pos[1]]
			self.drawPath()
			#return rotation, translation
		else:
			rospy.info("FUCK")

	def lidar2robot(self, tran, c, s, map_c):
		'''position = [0, 0, 0]
		rad = math.atan2(tf_points.centroids[i].y, tf_points.centroids[i].x)
		quaternion = tf.transformations.quaternion_from_euler(0., 0., -rad)
		transformer = tf.TransformerROS()
		transpose_matrix = transformer.fromTranslationRotation(position, quaternion)'''
		#================================
		#for self revolve problem
		self_revolve_pos = np.array([[c, -s], [s, c]]).dot(map_c)
		new_tran = [-(tran[1]), tran[0]]
		#cos(x) =  cos(-x)
		#sin(x) = -sin(-x)
		if self.sine == None or self.cosine == None:
			self.sine = s
			self.cosine = c
		else:
			self.sine = self.sine*c + self.cosine*s
			self.cosine = self.cosine*c - self.sine*s
		c_ = self.cosine
		s_ = -self.sine
		return new_tran, c_, s_

	def update_map(self):
		for i in range(self.obj_list.size):
			index = self.matching[i]
			if self.distance_to_robot(self.obj_list.list[i]) < self.update_range:
				if index != None:
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
				if self.distance_to_robot(self.map.list[j]) < self.punish_range:
					self.map.list[j].varianceX = self.map.list[j].varianceX*self.punish_no_detect
					self.map.list[j].varianceY = self.map.list[j].varianceY*self.punish_no_detect
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
		return math.sqrt((p.position.x - self.new_pos[0])**2 + (p.position.y - self.new_pos[1])**2 + (p.position.z - self.new_pos[2])**2)

	def drawPath(self):
		p = PoseStamped()
		#p.header = self.obj_list.header
		p.pose.position.x = self.new_pos[0]
		p.pose.position.y = self.new_pos[1]
		self.path.poses.append(p)
		self.path.header = self.obj_list.header
		self.pub_path.publish(self.path)

	def drawRviz(self, obj_list):
		marker_array = MarkerArray()
		# marker_array.markers.resize(obj_list.size)
		print obj_list.size
		for i in range(obj_list.size):
			#print obj_list.list[i].varianceX, obj_list.list[i].varianceY
			marker = Marker()
			marker.header.frame_id = obj_list.header.frame_id
			marker.id = i
			marker.header.stamp = rospy.Time.now()
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.lifetime = rospy.Duration(3)
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
			# Buoy  -> BLUE
			# Totem -> GREEN
			# Dock  -> WHITE
			# Not been classified -> RED
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
				marker.scale.x = 2
				marker.scale.y = 2
				marker.scale.z = 2
			marker_array.markers.append(marker)
		self.pub_marker.publish(marker_array)

if __name__ == "__main__":
	rospy.init_node('mapping_pcl')
	# Tell ROS that we're making a new node.
	rospy.init_node("mapping_pcl",anonymous=False)
	rospy.loginfo("Start Mapping")
	foo = mapping_pcl()
rospy.spin()