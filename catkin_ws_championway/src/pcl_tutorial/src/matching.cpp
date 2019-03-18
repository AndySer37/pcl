#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <iostream>

int
main(int argc, char** argv)
{
	// Object for storing the pfh descriptors for the scene.
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sceneDescriptors(new pcl::PointCloud<pcl::PFHSignature125>());
	// Object for storing the pfh descriptors for the model.
	pcl::PointCloud<pcl::PFHSignature125>::Ptr modelDescriptors(new pcl::PointCloud<pcl::PFHSignature125>());

	// Read the already computed descriptors from disk.
	if (pcl::io::loadPCDFile<pcl::PFHSignature125>(argv[1], *sceneDescriptors) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PFHSignature125>(argv[2], *modelDescriptors) != 0)
	{
		return -1;
	}

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::PFHSignature125> matching;
	matching.setInputCloud(modelDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptors->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(sceneDescriptors->at(i).histogram[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(sceneDescriptors->at(i), 1, neighbors, squaredDistances);
			
			// ...and add a new correspondence if the distance is less than a threshold
			std::cout << neighbors[0] << " " << squaredDistances[0] << std::endl;
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				//std::cout << sceneDescriptors->at(i).histogram[0] << " " << modelDescriptors->at(i).histogram[neighbors[0]] << std::endl;
				//std::cout << neighbors[0] << " " << squaredDistances[0] << std::endl;
				pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
			}
		}
	}
	std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;

	
	

}
