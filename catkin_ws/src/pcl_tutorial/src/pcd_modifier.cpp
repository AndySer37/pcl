#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		return -1;
	}
  /*
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI/2; // The angle of rotation in radians
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
  pcl::transformPointCloud (*cloud, *cloud, transform);
  */
  
	//const float nan_point = std::numeric_limits<float>::quiet_NaN();
  float sum_x=0, sum_y=0, sum_z=0;
	for (size_t i = 0; i < cloud->points.size (); ++i)
  	{
      /*
  		if ((cloud->points[i].y < 0) || ((cloud->points[i].x*cloud->points[i].x+cloud->points[i].y*cloud->points[i].y)<0.015) || (((cloud->points[i].z > 0)&&(cloud->points[i].z < 0.15)&&((cloud->points[i].x*cloud->points[i].x+cloud->points[i].y*cloud->points[i].y)<0.028) )) ){
  			cloud->points[i].x = nan_point;
  			cloud->points[i].y = nan_point;
  			cloud->points[i].z = nan_point;
  		} 
    	cloud->points[i].r = 255;
    	cloud->points[i].g = 0;
    	cloud->points[i].b = 0;
      */
      sum_x += cloud->points[i].x;
      sum_y += cloud->points[i].y;
      sum_z += cloud->points[i].z;
  	}
    sum_x /= cloud->points.size ();
    sum_y /= cloud->points.size ();
    sum_z /= cloud->points.size ();
  //std::cout << "origin size: " << cloud->points.size () << std::endl;
  //std::vector<int> indices;
  //pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  //std::cout << "size: " << cloud->points.size () << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x -= sum_x;
      cloud->points[i].y -= sum_y;
      cloud->points[i].z -= sum_z;
      
      cloud->points[i].r = 255;
      cloud->points[i].g = 0;
      cloud->points[i].b = 0;
    }
  
  pcl::io::savePCDFileASCII (argv[1], *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

}
