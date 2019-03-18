#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
ros::Publisher pub;   
using namespace std;
using namespace ros;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
      // Create a container for the data.
      sensor_msgs::PointCloud2 output;
      
      // Do data processing here...
      output = *input;
    
      // Publish the data.
      pub.publish (output);
      cout << "finish" << endl;
    }

int main (int argc, char** argv){
      // Initialize ROS
      init (argc, argv, "test1");
      NodeHandle nh;
    
      // Create a ROS subscriber for the input point cloud
      Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    
      // Create a ROS publisher for the output point cloud
      pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    
      // Spin
      spin ();
}