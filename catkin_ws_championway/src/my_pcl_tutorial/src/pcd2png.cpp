#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLImage.h>
#include <pcl/point_cloud.h>
int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		return -1;
	}
	std::cout<<"load image"<<std::endl;
	/*
	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (0.05f, 0.05f, 0.05f);
  	sor.filter (*cloud);
	*/
	// Source point cloud (needs to be filled with data of course)
	//cl::PointCloud<pcl::PointXYZLabel> cloud_image;
	pcl::PointCloud<pcl::PointXYZL> cloud_image;
	// Target image
	pcl::PCLImage image;
	// Create PointCloudImageExtractor subclass that can handle "label" field
	pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGB> pcie;
	// Set it up if not happy with the defaults
	//pcie.setColorMode(pcie.COLORS_RGB_RANDOM);
	// Try to extract an image
	bool success = pcie.extract(*cloud, image);
	std::cout<<success<<std::endl;
	// Save to file if succeeded
	if (success)
	{
	  pcl::io::savePNGFile ("filename.png", image);
	  std::cout<<"save"<<std::endl;
	}
}