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
#include <robotx_msgs/ObstaclePose.h>
#include <robotx_msgs/ObstaclePoseList.h>
#include <robotx_msgs/BoolStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/keypoints/iss_3d.h>
using namespace Eigen;
using namespace message_filters;
//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef boost::shared_ptr <robotx_msgs::BoolStamped const> BoolStampedConstPtr;
typedef pcl::PFHSignature125 PFH125;

//declare point cloud
PointCloudXYZ::Ptr cloud_inXYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr cloud_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr plane_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_h (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_f (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_plane (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_scene(new PointCloudXYZRGB);
//PointCloudXYZRGB::Ptr wall (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_out;

//declare ROS publisher
ros::Publisher pub_result;
ros::Publisher pub_marker;
ros::Publisher pub_marker_line;
ros::Publisher pub_obstacle;

//declare global variable
std_msgs::String pcl_frame_id; 
bool lock = false;
float low = -0.25;
float high = 1.5-low;
float thres_low = 0.03;
float thres_high = 1.5;
visualization_msgs::MarkerArray marker_array;
visualization_msgs::MarkerArray marker_array_line;
ros::Time pcl_t;

//declare function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function
void cluster_pointcloud(void); //point cloud clustering
void drawRviz(robotx_msgs::ObstaclePoseList); //draw marker in Rviz
void drawRviz_line(robotx_msgs::ObstaclePoseList); //draw marker line list in Rviz
int classify(PointCloudXYZRGB::Ptr cloud_cluster, PointCloudXYZRGB::Ptr cloud_model, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_model, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_scene);

//void callback(const sensor_msgs::PointCloud2ConstPtr& input, const robotx_msgs::BoolStampedConstPtr& tf_bool)
void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;
    //covert from ros type to pcl type
    pcl_frame_id.data = input->header.frame_id;
    pcl_t = input->header.stamp;
    pcl::fromROSMsg (*input, *cloud_inXYZ);
    copyPointCloud(*cloud_inXYZ, *cloud_in);

    //set color for point cloud
    for (size_t i = 0; i < cloud_in->points.size(); i++){
      cloud_in->points[i].r = 255;
      cloud_in->points[i].g = 255;
      cloud_in->points[i].b = 0;
    }
    cluster_pointcloud();
  }
  else{
    std::cout << "lock" << std::endl;
  }
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void cluster_pointcloud()
{
  std::cout<< "start processing point clouds" << std::endl;
  copyPointCloud(*cloud_in, *cloud_filtered);
  //========== Remove NaN point ==========
  /*std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);*/

  //========== Outlier remove ==========
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> out_filter;
  out_filter.setInputCloud (cloud_filtered);
  out_filter.setMeanK (50);
  out_filter.setStddevMulThresh (1.0);
  out_filter.filter (*cloud_filtered);*/

  //========== Remove Higer and Lower Place ==========
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_h_l_place;
  pcl::PointIndices::Ptr hl_indices (new pcl::PointIndices);
  for (int i = 0; i < cloud_filtered->points.size(); i++)
  {
    if(cloud_filtered->points[i].y <=3.2 && cloud_filtered->points[i].y >= -3.5 && cloud_filtered->points[i].x >= -1.5 && cloud_filtered->points[i].x <= 1.5)
    {
      hl_indices->indices.push_back(i);
    }
  }
  extract_h_l_place.setInputCloud(cloud_filtered);
  extract_h_l_place.setIndices(hl_indices);
  extract_h_l_place.setNegative(true);
  extract_h_l_place.filter(*cloud_h);
  *cloud_filtered = *cloud_h;

  //========== Downsample ==========
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_filtered);
  vg.setLeafSize (0.08f, 0.08f, 0.08f); //unit:cetimeter
  vg.filter (*cloud_filtered);

  //========== Point Cloud Clustering ==========
  // Declare variable
  int num_cluster = 0;
  int start_index = 0;
  robotx_msgs::ObstaclePoseList ob_list;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  // Create cluster object
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (1.3);// unit: meter
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    // Declare variable
    float x_min_x = 10000;
    float x_min_y = 10000;
    float y_min_x = 10000;
    float y_min_y = 10000;
    float x_max_x = -10000;
    float x_max_y = -10000;
    float y_max_x = -10000;
    float y_max_y = -10000;
    robotx_msgs::ObstaclePose ob_pose;
    Eigen::Vector4f centroid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
      result->points.push_back(cloud_filtered->points[*pit]);
      if (cloud_filtered->points[*pit].x < x_min_x)
      {
        x_min_x = cloud_filtered->points[*pit].x;
        x_min_y = cloud_filtered->points[*pit].y;
      }
      if (cloud_filtered->points[*pit].x > x_max_x)
      {
        x_max_x = cloud_filtered->points[*pit].x;
        x_max_y = cloud_filtered->points[*pit].y;
      }
      if (cloud_filtered->points[*pit].y < y_min_y)
      {
        y_min_x = cloud_filtered->points[*pit].x;
        y_min_y = cloud_filtered->points[*pit].y;
      }
      if (cloud_filtered->points[*pit].y > y_max_y)
      {
        y_max_x = cloud_filtered->points[*pit].x;
        y_max_y = cloud_filtered->points[*pit].y;
      }
    }
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    num_cluster++;
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals_scene(new pcl::PointCloud<pcl::Normal>);
      // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_scene(new pcl::PointCloud<pcl::PFHSignature125>());
    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation_scene;
    normalEstimation_scene.setInputCloud(cloud_cluster);
    normalEstimation_scene.setRadiusSearch(0.5);//5
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_scene(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation_scene.setSearchMethod(kdtree_scene);
    normalEstimation_scene.compute(*normals_scene);
    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_scene;
    pfh_scene.setInputCloud(cloud_cluster);
    pfh_scene.setInputNormals(normals_scene);
    pfh_scene.setSearchMethod(kdtree_scene);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh_scene.setRadiusSearch(0.5);
    pfh_scene.compute(*descriptors_scene);

    // Declare classify variable
    int buoy = 0;
    int totem = 0;
    int dock = 0;

    for(int n = 0; n <= 19; n++)
    {
      // Step 1: get input data & get keypoints
      // Object for storing the point cloud.
      PointCloudXYZRGB::Ptr cloud_model(new PointCloudXYZRGB);
      pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_model(new pcl::PointCloud<pcl::PFHSignature125>);
      // Read a PCD file from disk.
      std::stringstream ss, tt;
      ss << "totem/descriptors_" << n << ".pcd";
      tt << "totem/model_" << n << ".pcd";
      if (pcl::io::loadPCDFile<pcl::PFHSignature125>(ss.str (), *descriptors_model) != 0)
      {
        std::cout<< "No file to read" << std::endl;
        break;
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(tt.str (), *cloud_model) != 0)
      {
        std::cout<< "No file to read" << std::endl;
        break;
      }
      totem += classify(cloud_cluster, cloud_model, descriptors_model, descriptors_scene);
    }
    std::cout<<"totem: "<<totem<<std::endl;

    for(int n = 0; n <= 19; n++)
    {
      // Step 1: get input data & get keypoints
      // Object for storing the point cloud.
      PointCloudXYZRGB::Ptr cloud_model(new PointCloudXYZRGB);
      pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_model(new pcl::PointCloud<pcl::PFHSignature125>);
      // Read a PCD file from disk.
      std::stringstream ss, tt;
      ss << "buoy/descriptors_" << n << ".pcd";
      tt << "buoy/model_" << n << ".pcd";
      if (pcl::io::loadPCDFile<pcl::PFHSignature125>(ss.str (), *descriptors_model) != 0)
      {
        std::cout<< "No file to read" << std::endl;
        break;
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(tt.str (), *cloud_model) != 0)
      {
        std::cout<< "No file to read" << std::endl;
        break;
      }
      buoy += classify(cloud_cluster, cloud_model, descriptors_model, descriptors_scene);
    }
    std::cout<<"buoy: "<<buoy<<std::endl;

    for(int n = 0; n <= 19; n++)
    {
      // Step 1: get input data & get keypoints
      // Object for storing the point cloud.
      PointCloudXYZRGB::Ptr cloud_model(new PointCloudXYZRGB);
      pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_model(new pcl::PointCloud<pcl::PFHSignature125>);
      // Read a PCD file from disk.
      std::stringstream ss, tt;
      ss << "dock/descriptors_" << n << ".pcd";
      tt << "dock/model_" << n << ".pcd";
      if (pcl::io::loadPCDFile<pcl::PFHSignature125>(ss.str (), *descriptors_model) != 0)
      {
        std::cout<< "No file to read" << std::endl;
        break;
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(tt.str (), *cloud_model) != 0)
      {
        std::cout<< "No file to read" << std::endl;
        break;
      }
      dock += classify(cloud_cluster, cloud_model, descriptors_model, descriptors_scene);
    }
    std::cout<<"dock: "<<totem<<std::endl;

    if(totem != 0 || buoy != 0 || dock != 0)
    {
      if(buoy > totem && buoy > dock)
      {
        ob_pose.r = 1;
        std::cout<<"Found Buoy"<<std::endl;
      }
      else if(totem > buoy && totem > dock)
      {
        ob_pose.r = 2;
        std::cout<<"Found Totem"<<std::endl;
      }
      else if(dock > buoy && dock > totem)
      {
        ob_pose.r = 3;
        std::cout<<"Found Dock"<<std::endl;
      }
      else
      {
        std::cout<<"Not sure"<<std::endl;
      }
    }
    else
    {
      ob_pose.r = 0;
      std::cout << "No matching model" << std::endl;
    }

    //ob_pose.header.stamp = ros::Time::now();
    ob_pose.header.stamp = pcl_t;
    ob_pose.header.frame_id = cloud_in->header.frame_id;
    ob_pose.x = centroid[0];
    ob_pose.y = centroid[1];
    ob_pose.z = centroid[2];
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D (*cloud_cluster, min, max);
    ob_pose.min_x = min[0];
    ob_pose.max_x = max[0];
    ob_pose.min_y = min[1];
    ob_pose.max_y = max[1];
    ob_pose.min_z = min[2];
    ob_pose.max_z = max[2];
    ob_pose.x_min_x = x_min_x;
    ob_pose.x_min_y = x_min_y;
    ob_pose.x_max_x = x_max_x;
    ob_pose.x_max_y = x_max_y;
    ob_pose.y_min_x = y_min_x;
    ob_pose.y_min_y = y_min_y;
    ob_pose.y_max_x = y_max_x;
    ob_pose.y_max_y = y_max_y;
    //ob_pose.r = 1;
    ob_list.list.push_back(ob_pose);
    start_index = result->points.size();
  }

  //set obstacle list
  //ob_list.header.stamp = ros::Time::now();
  ob_list.header.stamp = pcl_t;
  ob_list.header.frame_id = cloud_in->header.frame_id;
  ob_list.size = num_cluster;
  pub_obstacle.publish(ob_list);
  drawRviz(ob_list);
  drawRviz_line(ob_list);
  result->header.frame_id = cloud_in->header.frame_id;
  pcl::toROSMsg(*result, ros_out);
  //ros_out.header.stamp = ros::Time::now();
  ros_out.header.stamp = pcl_t;
  pub_result.publish(ros_out);
  lock = false;
  result->clear();
  std::cout << "Finish" << std::endl << std::endl; 
}

void drawRviz_line(robotx_msgs::ObstaclePoseList ob_list){
  marker_array_line.markers.resize(ob_list.size);
  for (int i = 0; i < ob_list.size; i++)
  {
    marker_array_line.markers[i].header.frame_id = pcl_frame_id.data;
    marker_array_line.markers[i].id = i;
    marker_array_line.markers[i].header.stamp = ob_list.header.stamp;
    marker_array_line.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
    marker_array_line.markers[i].action = visualization_msgs::Marker::ADD;
    //marker_array.markers[i].pose.orientation.w = 1.0;
    marker_array_line.markers[i].points.clear();
    marker_array_line.markers[i].lifetime = ros::Duration(0.5);
    marker_array_line.markers[i].scale.x = (0.1);
    geometry_msgs::Point x_min;
    x_min.x = ob_list.list[i].x_min_x;
    x_min.y = ob_list.list[i].x_min_y;
    geometry_msgs::Point x_max;
    x_max.x = ob_list.list[i].x_max_x;
    x_max.y = ob_list.list[i].x_max_y;
    geometry_msgs::Point y_min;
    y_min.x = ob_list.list[i].y_min_x;
    y_min.y = ob_list.list[i].y_min_y;
    geometry_msgs::Point y_max;
    y_max.x = ob_list.list[i].y_max_x;
    y_max.y = ob_list.list[i].y_max_y;
    marker_array_line.markers[i].points.push_back(x_min);
    marker_array_line.markers[i].points.push_back(y_min);
    marker_array_line.markers[i].points.push_back(x_max);
    marker_array_line.markers[i].points.push_back(y_max);
    marker_array_line.markers[i].points.push_back(x_min);
    if (ob_list.list[i].r == 1)
    {
      marker_array_line.markers[i].text = "Buoy";
      marker_array_line.markers[i].color.r = 0;
      marker_array_line.markers[i].color.g = 0;
      marker_array_line.markers[i].color.b = 1;
      marker_array_line.markers[i].color.a = 1;
    }
    else if (ob_list.list[i].r == 2)
    {
      marker_array_line.markers[i].text = "Totem";
      marker_array_line.markers[i].color.r = 0;
      marker_array_line.markers[i].color.g = 1;
      marker_array_line.markers[i].color.b = 0;
      marker_array_line.markers[i].color.a = 1;
    }
    else if (ob_list.list[i].r == 3)
    {
      marker_array_line.markers[i].text = "Dock";
      marker_array_line.markers[i].color.r = 1;
      marker_array_line.markers[i].color.g = 1;
      marker_array_line.markers[i].color.b = 1;
      marker_array_line.markers[i].color.a = 1;
    }
    else
    {
      marker_array_line.markers[i].color.r = 1;
      marker_array_line.markers[i].color.g = 0;
      marker_array_line.markers[i].color.b = 0;
      marker_array_line.markers[i].color.a = 1;
    }
  }
  pub_marker_line.publish(marker_array_line);
}

void drawRviz(robotx_msgs::ObstaclePoseList ob_list){
      marker_array.markers.resize(ob_list.size);
      std_msgs::ColorRGBA c;
      for (int i = 0; i < ob_list.size; i++)
      {
        marker_array.markers[i].header.frame_id = pcl_frame_id.data;
        marker_array.markers[i].id = i;
        marker_array.markers[i].header.stamp = ob_list.header.stamp;
        marker_array.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].lifetime = ros::Duration(0.5);
        marker_array.markers[i].pose.position.x = ob_list.list[i].x;
        marker_array.markers[i].pose.position.y = ob_list.list[i].y;
        marker_array.markers[i].pose.position.z = ob_list.list[i].z;
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;
        marker_array.markers[i].scale.x = (ob_list.list[i].max_x-ob_list.list[i].min_x);
        marker_array.markers[i].scale.y = (ob_list.list[i].max_y-ob_list.list[i].min_y);
        marker_array.markers[i].scale.z = (ob_list.list[i].max_z-ob_list.list[i].min_z);
        if (marker_array.markers[i].scale.x ==0)
          marker_array.markers[i].scale.x=0.1;

        if (marker_array.markers[i].scale.y ==0)
          marker_array.markers[i].scale.y=0.1;

        if (marker_array.markers[i].scale.z ==0)
          marker_array.markers[i].scale.z=0.1;
        if (ob_list.list[i].r == 1)
        {
          marker_array.markers[i].text = "Buoy";
          marker_array.markers[i].color.r = 0;
          marker_array.markers[i].color.g = 0;
          marker_array.markers[i].color.b = 1;
          marker_array.markers[i].color.a = 0.5;
        }
        else if (ob_list.list[i].r == 2)
        {
          marker_array.markers[i].text = "Totem";
          marker_array.markers[i].color.r = 0;
          marker_array.markers[i].color.g = 1;
          marker_array.markers[i].color.b = 0;
          marker_array.markers[i].color.a = 0.5;
        }
        else if (ob_list.list[i].r == 3)
        {
          marker_array.markers[i].text = "Dock";
          marker_array.markers[i].color.r = 1;
          marker_array.markers[i].color.g = 1;
          marker_array.markers[i].color.b = 1;
          marker_array.markers[i].color.a = 0.5;
        }
        else
        {
          marker_array.markers[i].color.r = 1;
          marker_array.markers[i].color.g = 0;
          marker_array.markers[i].color.b = 0;
          marker_array.markers[i].color.a = 0.5;
        }
      }
      pub_marker.publish(marker_array);
}

int classify(PointCloudXYZRGB::Ptr cloud_cluster, PointCloudXYZRGB::Ptr cloud_model, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_model, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_scene)
{
      // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
      pcl::KdTreeFLANN<pcl::PFHSignature125> matching;
      matching.setInputCloud(descriptors_model);
      // A Correspondence object stores the indices of the query and the match,
      // and the distance/weight.
      pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
      // Check every descriptor computed for the scene.
      double min_dist = 1000000;
      for (size_t i = 0; i < descriptors_scene->size(); ++i)
      {
        std::vector<int> neighbors(1);
        std::vector<float> squaredDistances(1);
        // Ignore NaNs.
        if (pcl_isfinite(descriptors_scene->at(i).histogram[0]))
        {
          // Find the nearest neighbor (in descriptor space)...
          int neighborCount = matching.nearestKSearch(descriptors_scene->at(i), 1, neighbors, squaredDistances);
          // ...and add a new correspondence if the distance is less than a threshold
          if(squaredDistances[0] < min_dist)
            min_dist = squaredDistances[0];
          if (neighborCount == 1)
          {
            std::cout << neighbors[0] << " " << squaredDistances[0] << std::endl;
            pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
            correspondences->push_back(correspondence);
          }
        }
      }

      // Step 4: calculate correspondence grouping
      // Geometric consistency check
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
      std::vector<pcl::Correspondences> clusteredCorrespondences;
      pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> grouping;
      grouping.setSceneCloud(cloud_cluster);
      grouping.setInputCloud(cloud_model);
      grouping.setModelSceneCorrespondences(correspondences);
      // Minimum cluster size. Default is 3 (as at least 3 correspondences
      // are needed to compute the 6 DoF pose).
      grouping.setGCThreshold(4);
      // Resolution of the consensus set used to cluster correspondences together,
      // in metric units. Default is 1.0.
      grouping.setGCSize(0.08);
      grouping.recognize(transformations, clusteredCorrespondences);
      if(transformations.size() > 0)
      {
        return 1;
      }
      return 0;
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  tf::TransformListener listener(ros::Duration(1.0));
  // Create a ROS subscriber for the input point cloud
  /*message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<robotx_msgs::BoolStamped> bool_sub(nh, "/tf_transform", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, robotx_msgs::BoolStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, bool_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));*/

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, callback);
  // Create a ROS publisher for the output point cloud
  pub_obstacle = nh.advertise< robotx_msgs::ObstaclePoseList > ("/obstacle_list", 10);
  pub_marker = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker", 1);
  pub_marker_line = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker_line", 1);
  pub_result = nh.advertise<sensor_msgs::PointCloud2> ("/cluster_result", 1);
  ros::spin ();
}