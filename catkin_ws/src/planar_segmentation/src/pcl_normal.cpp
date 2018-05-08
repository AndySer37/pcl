#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <ros/console.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/transforms.h>  
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
pcl::visualization::PCLPlotter plotter;
bool first_Set_Viewer = true;
int counter = 0;

void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	if (counter == 10){
		counter = 0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg (*cloud_msg, *cloud);

		// normal of PCL and combine to pointxyz
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(0.3);
		ne.compute(*normals);
		pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );

		if (first_Set_Viewer)
		{
			first_Set_Viewer = false;
			viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
			viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals, 10, 0.5, "cloud_normals");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normals");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals");

		}
		viewer->updatePointCloud(cloud, "cloud");
		viewer->removePointCloud ("cloud_normals");
		viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals, 10, 2, "cloud_normals");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals");
	}
	counter += 1;
}

void cbViewer(const ros::TimerEvent& event)
{
	viewer->spinOnce();
	
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_normal");
	string name = ros::this_node::getName();
	ROS_INFO("[%s] Initializing ", name.c_str());

	ros::NodeHandle nh("~");

	ros::Subscriber sub_velodyne_pcl = nh.subscribe("/velodyne_points", 1000, cbPointCloud);
	//Visualization
	
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	int v1(0);
	viewer->createViewPort(-50.0, -50.0, 50, 50.0, v1);
	viewer->initCameraParameters ();

	ros::Timer timer = nh.createTimer(ros::Duration(1.0), cbViewer);
	
	ros::spin();

	return 0;
}
