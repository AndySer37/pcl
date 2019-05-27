#include "normal.h"

mapping::mapping(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	map_process.reset(new PointCloud<PointXYZ>());
	pc_input.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	pc_target.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());

	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>());


	path = package::getPath("pcl_exercise");
	if (pcl::io::loadPCDFile<PointXYZ> (path + "/pcd/1.pcd", *pc_input) == -1){
		PCL_ERROR ("Couldn't read file 1.pcd \n");
		return ;
	}
	process_node();
}
void mapping::process_node(){
	ne.setInputCloud (pc_input);

	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.05);

	// Compute the features
	ne.compute (*cloud_normals);

	cout << cloud_normals->points.size() << endl;

}
int main(int argc, char** argv){
	init(argc, argv, "mapping");
	mapping mapping;
	spin();
	return 0;
}