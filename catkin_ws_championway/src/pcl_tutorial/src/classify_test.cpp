#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/voxel_grid.h>


/*double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}*/

int main(int argc, char** argv)
{
	bool is_match = false;
	int max_corr = 0;
	int match_model_index = -1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr matching_model(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::CorrespondencesPtr matching_corr(new pcl::Correspondences());
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > matching_trans;
	std::vector<pcl::Correspondences> matching_clusteredCorrespondences;

	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud_scene) != 0)
	{
		return -1;
	}

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (cloud_scene);
  	sor.setLeafSize (0.05f, 0.05f, 0.05f);
  	sor.filter (*cloud_scene);

  	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals_scene(new pcl::PointCloud<pcl::Normal>);
  	// Object for storing the PFH descriptors for each point.
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_scene(new pcl::PointCloud<pcl::PFHSignature125>());
	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation_scene;
	normalEstimation_scene.setInputCloud(cloud_scene);
	normalEstimation_scene.setRadiusSearch(0.1);//5
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_scene(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation_scene.setSearchMethod(kdtree_scene);
	normalEstimation_scene.compute(*normals_scene);

	// PFH estimation object.
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_scene;
	pfh_scene.setInputCloud(cloud_scene);
	pfh_scene.setInputNormals(normals_scene);
	pfh_scene.setSearchMethod(kdtree_scene);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	pfh_scene.setRadiusSearch(0.15);

	pfh_scene.compute(*descriptors_scene);

	for(int n = 1; n < 8; n++)
	{

		// Step 1: get input data & get keypoints

		// Object for storing the point cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZRGB>);
		// Object for storing the normals.
		pcl::PointCloud<pcl::Normal>::Ptr normals_model(new pcl::PointCloud<pcl::Normal>);
		
		// Read a PCD file from disk.
		std::stringstream ss;
	    ss << "totem_" << n << ".pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(ss.str (), *cloud_model) != 0)
		{
			return -1;
		}
		std::cout << "Read totem_" << n << std::endl;
		
		// Step 2: calculate descriptors from keypoints

		// Object for storing the PFH descriptors for each point.
		pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_model(new pcl::PointCloud<pcl::PFHSignature125>());
		// Estimate the normals.
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation_model;
		normalEstimation_model.setInputCloud(cloud_model);
		normalEstimation_model.setRadiusSearch(0.1);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_model(new pcl::search::KdTree<pcl::PointXYZRGB>);
		normalEstimation_model.setSearchMethod(kdtree_model);
		normalEstimation_model.compute(*normals_model);

		// PFH estimation object.
		pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_model;
		pfh_model.setInputCloud(cloud_model);
		pfh_model.setInputNormals(normals_model);
		pfh_model.setSearchMethod(kdtree_model);
		// Search radius, to look for neighbors. Note: the value given here has to be
		// larger than the radius used to estimate the normals.
		pfh_model.setRadiusSearch(0.15);

		pfh_model.compute(*descriptors_model);

		// Step 3: calculate correspondences of descriptors

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
					//std::cout << neighbors[0] << " " << squaredDistances[0] << std::endl;
					pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
					correspondences->push_back(correspondence);
				}
			}
		}
		std::cout << "min neighbor dist: " << min_dist << std::endl;
		std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;

		// Step 4: calculate correspondence grouping
		// Geometric consistency check
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
		std::vector<pcl::Correspondences> clusteredCorrespondences;
		pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> grouping;
		grouping.setSceneCloud(cloud_scene);
		grouping.setInputCloud(cloud_model);
		grouping.setModelSceneCorrespondences(correspondences);
		// Minimum cluster size. Default is 3 (as at least 3 correspondences
		// are needed to compute the 6 DoF pose).
		grouping.setGCThreshold(3);
		// Resolution of the consensus set used to cluster correspondences together,
		// in metric units. Default is 1.0.
		grouping.setGCSize(0.005);

		grouping.recognize(transformations, clusteredCorrespondences);

		std::cout << "Model instances found: " << transformations.size() << std::endl << std::endl;

		if(transformations.size() > 0 && max_corr < correspondences->size())
		{
			matching_model = cloud_model;
			max_corr = correspondences->size();
			matching_corr = correspondences;
			match_model_index = n;
			matching_clusteredCorrespondences = clusteredCorrespondences;
			matching_trans = transformations;
			is_match = true;
		}
	}

	if (is_match)
	{
		/*for (size_t i = 0; i < matching_trans.size(); i++)
		{
			std::cout << "Instance " << (i + 1) << ":" << std::endl;
			std::cout << "\tHas " << matching_clusteredCorrespondences[i].size() << " correspondences." << std::endl << std::endl;

			Eigen::Matrix3f rotation = matching_trans[i].block<3, 3>(0, 0);
			Eigen::Vector3f translation = matching_trans[i].block<3, 1>(0, 3);
			printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
			printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
			printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
			std::cout << std::endl;
			printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
		}*/
		std::cout << "matching_corr: " << matching_corr->size() << std::endl;
		std::cout << "max_corr: " << max_corr << std::endl;
		pcl::visualization::PCLVisualizer viewer ("correspondences");
		viewer.addPointCloud (cloud_scene, "scene_cloud");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_cloud");

		//try not to let scene and model collision
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model_off(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud (*matching_model, *cloud_model_off, Eigen::Vector3f (-2,0,0), Eigen::Quaternionf (1, 0, 0, 0));//-20
		for (size_t i = 0; i < cloud_model_off->points.size (); ++i)
		{
		  	cloud_model_off->points[i].r = 255;
		    cloud_model_off->points[i].g = 255;
		    cloud_model_off->points[i].b = 0;
		}
		std::cout << "The matching model is: " << match_model_index << std::endl;
		viewer.addPointCloud (cloud_model_off, "model_cloud_off");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_cloud_off");

		
			
		for (size_t j = 0; j < matching_corr->size(); ++j){
			std::stringstream ss_line;
		    ss_line << "correspondence_line" << 0 << "_" << j;
		    pcl::Correspondence corr = matching_corr->at(j);
			pcl::PointXYZRGB model_point = cloud_model_off->at (corr.index_query);
		    pcl::PointXYZRGB scene_point = cloud_scene->at (corr.index_match);
		    viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (model_point, scene_point, 50, 0, 50, ss_line.str ());
		}
	  	
	  	//viewer.addCoordinateSystem();
		while (!viewer.wasStopped ())
		{
		    viewer.spinOnce ();
		}
	}
	else
	{
		std::cout << "No matching model" << std::endl;
	}

	
}

	