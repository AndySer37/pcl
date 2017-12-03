#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate r(1000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //rviz
  visualization_msgs::Marker viz_p;
  viz_p.header.frame_id = "/pcl_frame";
  viz_p.header.stamp = ros::Time::now();
  viz_p.ns = "ppp";
  viz_p.action = visualization_msgs::Marker::ADD;
  viz_p.id = 0;
  viz_p.type = visualization_msgs::Marker::POINTS;
  viz_p.scale.x = 0.02;
  viz_p.scale.y = 0.02;
  viz_p.color.g = 1.0f;
  viz_p.color.a = 1.0;

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  while (ros::ok())
  {
    std::cout<< "1111";
    // Generate the data
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1.0;
      geometry_msgs::Point p;
      p.x = cloud->points[i].x;
      p.y = cloud->points[i].y;
      p.z = cloud->points[i].z;
      viz_p.points.push_back(p);
    }

    // Set a few outliers
    cloud->points[0].z = 2.0;
    cloud->points[3].z = -2.0;
    cloud->points[6].z = 4.0;
    geometry_msgs::Point p;
    p.z = cloud->points[0].z;
    p.z = cloud->points[6].z;
    p.z = cloud->points[3].z;
    viz_p.points.push_back(p);

    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
      std::cerr << "    " << cloud->points[i].x << " "
                          << cloud->points[i].y << " "
                          << cloud->points[i].z << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
      std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                 << cloud->points[inliers->indices[i]].y << " "
                                                 << cloud->points[inliers->indices[i]].z << std::endl;
    marker_pub.publish(viz_p);
    sleep(3);
  }

  return (0);
}