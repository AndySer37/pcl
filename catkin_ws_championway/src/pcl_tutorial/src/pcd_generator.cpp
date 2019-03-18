#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>
int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  // Fill in the cloud data
  cloud.width    = 30;
  cloud.height   = 30;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  
  // Example here
  /*
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].r = 255;
    cloud.points[i].g = 0;
    cloud.points[i].b = 0;
  }
  */
  


  // To create a sphere with only surface (radius = 10) uncomment below
  /*
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float z = ((rand() % 200) - 100) / 10.0;
    float theta = (rand() % 628) - 314 / 100.0;
    cloud.points[i].z = z;
    cloud.points[i].x = sqrt(10*10-z*z)*sin(theta);
    cloud.points[i].y = sqrt(10*10-z*z)*cos(theta);
    cloud.points[i].r = 255;
    cloud.points[i].g = 0;
    cloud.points[i].b = 0;
  }
  */
  

  // To create a cylinder with only surface (radius = 5, height = 30) uncomment below
  /*
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {  
    float z = (rand() % 300) / 10.0;
    float theta = (rand() % 628) - 314 / 100.0;
    cloud.points[i].z = z;
    cloud.points[i].x = 5*sin(theta);
    cloud.points[i].y = 5*cos(theta);
    cloud.points[i].r = 255;
    cloud.points[i].g = 0;
    cloud.points[i].b = 0;
  }

  // To add the top and bottom lid
  pcl::PointCloud<pcl::PointXYZRGB> cloud2;
  cloud2.width    = 50;
  cloud2.height   = 50;
  cloud2.is_dense = false;
  cloud2.points.resize (cloud2.width * cloud2.height);
  for (size_t i = 0; i < cloud2.points.size (); ++i)
  {  
    float z = ((rand() % 2)==0) ? 30 : 0;
    float theta = (rand() % 628) - 314 / 100.0;
    float radius = (rand() % 50) / 10.0;
    cloud2.points[i].z = z;
    cloud2.points[i].x = radius*sin(theta);
    cloud2.points[i].y = radius*cos(theta);
    cloud2.points[i].r = 255;
    cloud2.points[i].g = 0;
    cloud2.points[i].b = 0;
  }
  cloud = cloud + cloud2;
  */ 
  

  // Let's create a buoy (only one perspective)
  // Create a sphere as before but change 2 things
  // 1. the theta due to perspective limit
  // 2. the bottom part that is submerged in water
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float in_water = -10.0 + 3.0;
    float z = ((rand() % 200) - 100) / 10.0;
    float theta = ((rand() % 314) - 157) / 100.0;
    
    while(z < in_water)
      z = ((rand() % 200) - 100) / 10.0;

    cloud.points[i].z = z + 10.0;
    cloud.points[i].x = sqrt(10*10-z*z)*sin(theta);
    cloud.points[i].y = sqrt(10*10-z*z)*cos(theta);
    cloud.points[i].r = 0;
    cloud.points[i].g = 255;
    cloud.points[i].b = 0;
  }
  


  // Let's create a totem (only one perspective)
  // Create a sphere as before but change 2 things
  // 1. the theta due to perspective limit
  // 2. the bottom part that is submerged in water
  // To create a cylinder with only surface (radius = 5, height = 30) uncomment below
  /*
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {  
    float in_water = 3.0;
    float z = (rand() % 300) / 10.0;
    float theta = (rand() % 628) - 314 / 100.0;
    while(z < in_water)
      z = (rand() % 300) / 10.0;
    cloud.points[i].z = z;
    cloud.points[i].x = 5*sin(theta);
    cloud.points[i].y = 5*cos(theta);
    cloud.points[i].r = 255;
    cloud.points[i].g = 0;
    cloud.points[i].b = 0;
  }

  // To add the top and bottom lid
  pcl::PointCloud<pcl::PointXYZRGB> cloud2;
  //pcl::PointCloud<pcl::PointXYZ> cloud2;
  cloud2.width    = 10;
  cloud2.height   = 10;
  cloud2.is_dense = false;
  cloud2.points.resize (cloud2.width * cloud2.height);
  for (size_t i = 0; i < cloud2.points.size (); ++i)
  {  
    float z = 30;
    float theta = (rand() % 628) - 314 / 100.0;
    float radius = (rand() % 50) / 10.0;
    cloud2.points[i].z = z;
    cloud2.points[i].x = radius*sin(theta);
    cloud2.points[i].y = radius*cos(theta);
    cloud2.points[i].r = 255;
    cloud2.points[i].g = 0;
    cloud2.points[i].b = 0;
  }
  cloud = cloud + cloud2;
  */


  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  //for (size_t i = 0; i < cloud.points.size (); ++i)
  //  std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}