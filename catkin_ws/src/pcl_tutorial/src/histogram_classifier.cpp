#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <math.h> 
#include <pcl/visualization/histogram_visualizer.h>
struct radius_histogram_descriptor {
  float histogram[10] = {0};
};

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


  //Read files
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }

  std::cout << "Loaded :\n"
            << argv[1]
            << "\nsize: "
            << cloud->width * cloud->height
            << std::endl;
            

  std::cout << "/////////////// Starting Histogram Classifier ///////////////" << std::endl;
  // Create descriptor
  radius_histogram_descriptor descriptor; 

  // Step 1: find the center x, y and maximum r from cloud
  float min_r = 0, max_r = 0;
  float center_x = 0, center_y = 0;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    center_x += cloud->points[i].x;
    center_y += cloud->points[i].y;
  }
  center_x /= cloud->points.size ();
  center_y /= cloud->points.size ();

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    float radius = sqrt((cloud->points[i].x-center_x)*(cloud->points[i].x-center_x)+(cloud->points[i].y-center_y)*(cloud->points[i].y-center_y));
    if ( radius > max_r)
      max_r = radius;
  }

  // to prevent border conditions, I expand the and max by a small value
  max_r += 0.01;
  
  // Step 2: create histogram (10 intervals)
  // Place points in the histogram
  float interval_size = 0.1;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    float radius = sqrt((cloud->points[i].x-center_x)*(cloud->points[i].x-center_x)+(cloud->points[i].y-center_y)*(cloud->points[i].y-center_y));    
    descriptor.histogram[int((radius/max_r)/interval_size)] += 1;
  }

  // Normalize the histogram
  int i;
  for (i = 0; i < 10; i++){
    descriptor.histogram[i] /= cloud->points.size ();
  }

  // Print Hist
  std::cout << "Histogram: \n";
  for(i=0;i<10;i++){
    std::cout << descriptor.histogram[i] << " ";
  }
  std::cout << std::endl;


  // Step 3: classification
  // calculate difference to classes
  // Totem:
  float dist_totem = 0;
  float dist_buoy = 0;
  float totem[10] = {0.04, 0.18, 0.22, 0.13, 0.16, 0.1, 0.1, 0.03, 0.04, 0.02};
  float buoy[10] = {0.04, 0.07, 0.22, 0.13, 0.17, 0.15, 0.09, 0.09, 0, 0.04};

  for (i = 0; i < 10; i++){
    dist_totem += std::abs((totem[i] - descriptor.histogram[i]));
    dist_buoy += std::abs((buoy[i] - descriptor.histogram[i]));
  }
  std::cout << "dist to totem : " << dist_totem << std::endl;
  std::cout << "dist to buoy : " << dist_buoy << std::endl;
}