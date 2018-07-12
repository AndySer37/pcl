#!/usr/bin/env python
import rospy
from tf import TransformListener,TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
import roslib
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObstaclePose, ObstaclePoseList, Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class pcl2img():
	def __init__(self): 
		# ======== Subscriber ========
		rospy.Subscriber("/obstacle_list", ObstaclePoseList, call_back, queue_size=10)
		#rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)

		# ======== Publisher ========
		pub_obs = rospy.Publisher("/obstacle_list/map", ObstaclePoseList, queue_size=1)
		#pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)

		# ======== Declare Variable ========
		self.map = ObstaclePoseList()

	def call_back(msg):
		try:
		print ("Process Obstacle List")
			obs_list = ObstaclePoseList()
			obs_list = msg
			position, quaternion = tf_.lookupTransform( "/map", "/velodyne",rospy.Time(0))
			transpose_matrix = transformer.fromTranslationRotation(position, quaternion)
			for obs_index in range(obs_list.size):
				origin_p  = np.array([obs_list.list[obs_index].x, obs_list.list[obs_index].y, obs_list.list[obs_index].z, 1])
				origin_p1 = np.array([obs_list.list[obs_index].x_min_x, obs_list.list[obs_index].x_min_y, 0, 1])
				origin_p2 = np.array([obs_list.list[obs_index].y_min_x, obs_list.list[obs_index].y_min_y, 0, 1])
				origin_p3 = np.array([obs_list.list[obs_index].x_max_x, obs_list.list[obs_index].x_max_y, 0, 1])
				origin_p4 = np.array([obs_list.list[obs_index].y_max_x, obs_list.list[obs_index].y_max_y, 0, 1])
				new_p = np.dot(transpose_matrix, origin_p)
				new_p1 = np.dot(transpose_matrix, origin_p1)
				new_p2 = np.dot(transpose_matrix, origin_p2)
				new_p3 = np.dot(transpose_matrix, origin_p3)
				new_p4 = np.dot(transpose_matrix, origin_p4)
				obs_list.list[obs_index].x = new_p[0]
				obs_list.list[obs_index].y = new_p[1]
				obs_list.list[obs_index].x_min_x = new_p1[0]
				obs_list.list[obs_index].x_min_y = new_p1[1]
				obs_list.list[obs_index].y_min_x = new_p2[0]
				obs_list.list[obs_index].y_min_y = new_p2[1]
				obs_list.list[obs_index].x_max_x = new_p3[0]
				obs_list.list[obs_index].x_max_y = new_p3[1]
				obs_list.list[obs_index].y_max_x = new_p4[0]
				obs_list.list[obs_index].y_max_y = new_p4[1]
				obs_list.header.frame_id = "map"
			pub_obs.publish(obs_list)

		except (LookupException, ConnectivityException, ExtrapolationException):
			print "Nothing Happen"

if __name__ == "__main__":
	rospy.init_node('mapping')
	# Tell ROS that we're making a new node.
	rospy.init_node("mapping",anonymous=False)
	rospy.loginfo("Start Mapping")
	tf_ = TransformListener()
	transformer = TransformerROS()
	foo = mapping()
	rospy.spin()