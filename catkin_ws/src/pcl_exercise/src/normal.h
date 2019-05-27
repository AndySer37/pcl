#include <iostream>
// Ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// Pcl icp
#include <pcl/registration/icp.h>
// Pcl passthrough
#include <pcl/filters/passthrough.h>
// Pcl outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>
// Pcl plane filter
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// Pcl normal
#include <pcl/features/normal_3d.h>

using namespace ros;
using namespace pcl;
using namespace std;
const double PI  =3.141592653589793238463;

class mapping{
  public:
	mapping();
	void process_node();    // sensor_msgs::PointCloud2
  private:
  	string path;
	Publisher pc_map;
	Publisher pc_process;

	PointCloud<PointXYZ>::Ptr map;
	PointCloud<PointXYZ>::Ptr map_process;
	PointCloud<PointXYZ>::Ptr pc_input;
	PointCloud<PointXYZ>::Ptr pc_filter;
	PointCloud<PointXYZ>::Ptr pc_target;
	PointCloud<PointXYZ>::Ptr result;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

	sensor_msgs::PointCloud2 ros_cloud_msg;
	sensor_msgs::PointCloud2 origin_map;


	Publisher marker_pub;
	uint32_t shape;
	visualization_msgs::Marker marker;
};