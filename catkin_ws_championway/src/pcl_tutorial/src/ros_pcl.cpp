#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// include PCL(point cloud library)
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

//declare point cloud pointer
PointCloudXYZ::Ptr cloud_XYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_output;

//declare publisher
ros::Publisher pub_XYZRGB;
ros::Publisher pub_pointcloud2;

//declare global variable
bool lock = false;
void pointcloud_processing(void);

//call back function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;
    
    pcl::fromROSMsg (*input, *cloud_XYZ); //covert from ros type to pcl type
    copyPointCloud(*cloud_XYZ, *cloud_XYZRGB); //it will include frame information

    for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
    {
      cloud_XYZRGB->points[i].r = 255;
      cloud_XYZRGB->points[i].g = 0;
      cloud_XYZRGB->points[i].b = 0;
    }
    pointcloud_processing();
  }
  else
  {
    std::cout << "lock" << std::endl;
  }
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void pointcloud_processing()
{
  ROS_INFO("Success output");
  pub_XYZRGB.publish(*cloud_XYZRGB);
  pcl::toROSMsg(*cloud_XYZRGB, ros_output); //covert from pcl type to ros type
  pub_pointcloud2.publish(ros_output);
  lock = false;
}

int main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "ros_pcl");
     ros::NodeHandle nh;

     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);

     // Create a ROS publisher for the output point cloud
     pub_XYZRGB = nh.advertise<PointCloudXYZRGB> ("/output_pcl", 1);
     pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2> ("/output_ros", 1);

     // Spin
     ros::spin ();
}