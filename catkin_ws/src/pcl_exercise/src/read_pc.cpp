#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

ros::Publisher pub;   
using namespace ros;
using namespace pcl;
using namespace std;

int main (int argc, char** argv){
      // Initialize ROS
      init (argc, argv, "read_pc");
      NodeHandle nh;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc (new pcl::PointCloud<pcl::PointXYZRGB>);

      if (pcl::io::loadPCDFile<PointXYZRGB>("/home/andyser/code/test_pcd.pcd", *pc) == -1){
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return -1;
      }    
      cout << "Loaded pcd file." << endl;
      Publisher pc2;
      pc2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc", 10);
      sensor_msgs::PointCloud2 object_cloud_msg;
      toROSMsg(*pc, object_cloud_msg);
      object_cloud_msg.header.frame_id = "camera_link";
      while(1)
            pc2.publish(object_cloud_msg);
      
      // Spin
      spin ();
}