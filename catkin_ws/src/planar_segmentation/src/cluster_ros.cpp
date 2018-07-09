#include <ros/ros.h>
#include <cmath>        // std::abs
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Time.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h>
using namespace Eigen;
using namespace message_filters;
//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
//declare point cloud
PointCloudXYZ::Ptr cloud_inXYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr cloud_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr plane_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_h (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_f (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_plane (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr hold_plane (new PointCloudXYZRGB);
//PointCloudXYZRGB::Ptr wall (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_out;
//sensor_msgs::PointCloud2 ros_wall;
//declare ROS publisher
ros::Publisher pub_result;
//ros::Publisher pub_wall;
ros::Publisher pub_marker;
ros::Publisher pub_marker_line;
ros::Publisher pub_obstacle;

//declare global variable
bool lock = false;
float low = -0.58;
float high = 1.5-low;
float thres_low = 0.03;
float thres_high = 1.5;
int j = 0;
visualization_msgs::MarkerArray marker_array;

//declare function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function
void cluster_pointcloud(void); //point cloud clustering

//void callback(const sensor_msgs::PointCloud2ConstPtr& input, const robotx_msgs::BoolStampedConstPtr& tf_bool)
void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  /*try{
    lr->waitForTransform("/map", "/robot", ros::Time::now(), ros::Duration(0.5) );
  }
  catch(tf::TransformException ex){
    ROS_ERROR("transform exception : %s", ex.what());
  }*/

  if (!lock){
    lock = true;
    //covert from ros type to pcl type
    //pcl_t = input->header.stamp;
    pcl::fromROSMsg (*input, *cloud_inXYZ);
    copyPointCloud(*cloud_inXYZ, *cloud_in);
    //set color for point cloud
    for (size_t i = 0; i < cloud_in->points.size(); i++){
      cloud_in->points[i].r = 255;
      cloud_in->points[i].g = 255;
      cloud_in->points[i].b = 0;
    }
    //point cloud clustering
    /*if(tf_bool->data){
      cluster_pointcloud();
    }*/
    cluster_pointcloud();
    
  }
  else{
    std::cout << "lock" << std::endl;
  }
}

int point_cloud_color(int seed){
  srand(seed);
  int min = 80;
  int max = 255;
  int x = rand() % (max - min + 1) + min;
  std::cout<< "random" << x << std::endl;
  return x;
  /*if(input < 0){
    return 0;
  }
  else if(input > 255){
    return 255;
    std::cout << "Skip" << std::endl;
  }
  else{
    return input;
  }*/
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void cluster_pointcloud()
{
  pcl::PCDWriter writer;
  std::cout<< "start processing point clouds" << std::endl;
  copyPointCloud(*cloud_in, *cloud_filtered);
  
  //========== Remove NaN point ==========
  /*std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);*/

  //========== Downsample ==========
  /*pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.08f, 0.08f, 0.08f); //unit:cetimeter
  vg.filter (*cloud_filtered);
  std::cout << "Filtering successfully" << std::endl;*/
  //copyPointCloud(*cloud_filtered, *wall);
  //wall->clear();

  //========== Outlier remove ==========
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> out_filter;
  out_filter.setInputCloud (cloud_filtered);
  out_filter.setMeanK (50);
  out_filter.setStddevMulThresh (1.0);
  out_filter.filter (*cloud_filtered);*/
  
  //========== Planar filter ==========
  /*pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.02);
  copyPointCloud(*cloud_filtered, *wall);
  wall->clear();
  int nr_points = (int) cloud_filtered->points.size ();
  const float nan_point = std::numeric_limits<float>::quiet_NaN();
  bool find_floor = false;
  int plane_z = 0;
  while (cloud_filtered->points.size () > 0.2 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    plane_z = 0;
    for (int i = 0; i < cloud_plane->points.size(); i++)
    {
      plane_z += cloud_plane->points[i].z;
    }
    plane_z =(float)plane_z/cloud_plane->points.size();
    //high = thres_high + low;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
    //if (std::abs(coefficients->values[2])>0.2 && std::abs(coefficients->values[2]<0.85))
    if (std::abs(coefficients->values[2] < 0.2))
    {
      pcl::ModelCoefficients line; 
      pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices); 
      pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
      seg.setOptimizeCoefficients(true); 
      seg.setModelType(pcl::SACMODEL_LINE); 
      seg.setMethodType(pcl::SAC_RANSAC); 
      seg.setDistanceThreshold(1); 
      seg.setInputCloud(cloud_plane); 
      seg.segment(*line_inliers, line);
      std::cout<<line<<std::endl;
      //std::cout << "hold plane"<< std::endl;
      //plane_indices->indices.insert(plane_indices->indices.end(), inliers->indices.begin(), inliers->indices.end());
      //*wall += *cloud_plane;
    }
    //else if (plane_z > -0.3)
    //{
      //*hold_plane += *cloud_plane;
      //for (int i = 0; i < cloud_plane->points.size(); i++)
      //{
        //low += cloud_plane->points[i].z;
      //}
      //low =(float)low/cloud_plane->points.size() - thres_low;
      //high = thres_high + low;
      //find_floor = true;
    //}
  }
  //*cloud_filtered += *hold_plane;*/

  //========== Remove Higer and Lower Place ==========
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_h_l_place;
  pcl::PointIndices::Ptr hl_indices (new pcl::PointIndices);
  //std::cout<< low << "," << high << std::endl;
  for (int i = 0; i < cloud_filtered->points.size(); i++)
  {
    if (cloud_filtered->points[i].y <=3.2 && cloud_filtered->points[i].y >= -3.5 && cloud_filtered->points[i].x >= -1.5 && cloud_filtered->points[i].x <= 1.5)
    {
      hl_indices->indices.push_back(i);
    }
  }
  extract_h_l_place.setInputCloud(cloud_filtered);
  extract_h_l_place.setIndices(hl_indices);
  extract_h_l_place.setNegative(true);
  extract_h_l_place.filter(*cloud_h);
  *cloud_filtered = *cloud_h;

  //========== Project To Ground ==========
  // Create a set of planar coefficients with X=Y=0,Z=1
  /*cl::ModelCoefficients::Ptr c (new pcl::ModelCoefficients ());
  c->values.resize (4);
  c->values[0] = 0;
  c->values[1] = 0;
  c->values[2] = 1.0;
  c->values[3] = 0;
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (c);
  proj.filter (*cloud_filtered);*/
  /*for (size_t i = 0; i < wall->points.size(); i++){
    wall->points[i].r = 255;
    wall->points[i].g = 0;
    wall->points[i].b = 255;
  }*/
  
  //========== Wall Detector Node ==========
  /*pcl::ModelCoefficients line; 
  pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices); 
  pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
  seg.setOptimizeCoefficients(true); 
  seg.setModelType(pcl::SACMODEL_LINE); 
  seg.setMethodType(pcl::SAC_RANSAC); 
  seg.setDistanceThreshold(1); 
  seg.setInputCloud(cloud_filtered); 
  seg.segment(*line_inliers, line);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_line;
  extract_line.setInputCloud(cloud_filtered);
  extract_line.setIndices(line_inliers);
  extract_line.setNegative(false);
  extract_line.filter(*wall);
  std::cout<<line<<std::endl;*/
  //*cloud_filtered = *cloud_h;
  /*pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud_filtered));
  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_line);

  ransac.setDistanceThreshold (0.01);
  ransac.computeModel();

  Eigen::VectorXf line_coefficients;
  ransac.getModelCoefficients(line_coefficients);
  std::cout<<line_coefficients<<std::endl;*/

  //========== Outlier remove ==========
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> out_filter;
  out_filter.setInputCloud (cloud_filtered);
  out_filter.setMeanK (50);
  out_filter.setStddevMulThresh (1.0);
  out_filter.filter (*cloud_filtered);*/

  //========== Point Cloud Clustering ==========
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);
  // Create cluster object
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (1.3); // unit: meter
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (1000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  int num_cluster = 0;
  int start_index = 0;
  int set_r=0, set_g=0, set_b=0;
  int rand = 0;
  //int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    num_cluster++;
    Eigen::Vector4f centroid;
    PointCloudXYZRGB::Ptr cloud_cluster (new PointCloudXYZRGB);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      result->points.push_back(cloud_filtered->points[*pit]);
    }
    
    result->width = result->points.size();
    result->height = 1;
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    // Start_index = result->points.size();

    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_cluster);
    normalEstimation.setRadiusSearch(0.5);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud_cluster);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.5);
    pfh.compute(*descriptors);

    std::stringstream ss, tt;
    ss << "descriptors_" << j << ".pcd";
    tt << "model_" << j << ".pcd";
    std::cout<<j<<std::endl;
    pcl::io::savePCDFileASCII (ss.str(), *descriptors);
    pcl::io::savePCDFileASCII (tt.str(), *cloud_cluster);
    j++;

  }


  result->header.frame_id = cloud_in->header.frame_id;
  //writer.write<pcl::PointXYZRGB> ("result2.pcd", *result, false);
  std::cout << "Finish" << std::endl << std::endl;
  pcl::toROSMsg(*result, ros_out);
  //pcl::toROSMsg(*wall, ros_wall);
  ros_out.header.stamp = ros::Time::now();
  //ros_wall.header.stamp = pcl_t;
  pub_result.publish(ros_out);
  //pub_wall.publish(ros_wall);
  lock = false;
  result->clear();
  hold_plane->clear();
  ros::Duration(2.0).sleep();
  //wall->clear();
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  /*message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<robotx_msgs::BoolStamped> bool_sub(nh, "/tf_transform", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, robotx_msgs::BoolStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, bool_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));*/


  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, callback);
  // Create a ROS publisher for the output point cloud
  pub_marker = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker", 1);
  pub_result = nh.advertise<sensor_msgs::PointCloud2> ("/cluster_result", 1);
  //pub_wall = nh.advertise<sensor_msgs::PointCloud2> ("/wall", 1);
  // Spin
  ros::spin ();
}