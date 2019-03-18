#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointXYZINormal PointTypeFull;
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB);

void max_min(float &i, float &max, float &min)
{
  if(i > max){max = i;}
  if(i < min){min = i;}
}

int main (int argc, char** argv)
{
  // Data containers used
  pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);

  // Load the input point cloud
  std::cerr << "Loading...\n", tt.tic ();
  pcl::io::loadPCDFile ("plane_model.pcd", *cloud_in);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";

  float max_x = cloud_in->points[0].x, max_y = cloud_in->points[0].y, max_z = cloud_in->points[0].z;
  float min_x = cloud_in->points[0].x, min_y = cloud_in->points[0].y, min_z = cloud_in->points[0].z;
  for (int i = 0; i < cloud_in->points.size(); ++i)
  {
    max_min(cloud_in->points[i].x, max_x, min_x);
    max_min(cloud_in->points[i].y, max_y, min_y);
    max_min(cloud_in->points[i].z, max_z, min_z);
  }

  for (float scan = min_z; scan <= max_z+10; scan = scan+10)
  {
    
  }

  // Save the output point cloud
  std::cerr << "Saving...\n", tt.tic ();
  pcl::io::savePCDFile ("output.pcd", *cloud_out);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  return (0);
}