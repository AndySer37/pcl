#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>
int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the PFH descriptors for each point.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	vfh.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(true);

	vfh.compute(*descriptor);
	pcl::io::savePCDFileASCII ("descriptor.xml", *descriptor);

	// Plotter object.
	pcl::visualization::PCLHistogramVisualizer viewer;
	// We need to set the size of the descriptor beforehand.
	viewer.addFeatureHistogram(*descriptor, 308);

	viewer.spin();
}