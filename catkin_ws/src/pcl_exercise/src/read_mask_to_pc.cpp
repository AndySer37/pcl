#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include<fstream>
#include<vector>
ros::Publisher pub;   
using namespace ros;
using namespace pcl;
using namespace std;
using namespace cv;

typedef uint32_t uint32;
struct PointCloudLabel
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  uint32 label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointCloudLabel,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (uint32, label, label)
)
typedef pcl::PointCloud<PointCloudLabel> PointLabel;

void getXYZ(float* a, float* b,float zc,float fx,float fy,float cx,float cy){

	float inv_fx = 1.0/fx;
	float inv_fy = 1.0/fy;
	*a = (*a - cx) * zc * inv_fx;
	*b = (*b - cy) * zc * inv_fy;
	return;
}

#define SIZE 200


int main (int argc, char** argv){
	char line[SIZE];
	int pcd_num = 0;
	init (argc, argv, "read_mask_to_pc");
	NodeHandle nh;
	int count = 0;
	int count1 = 0;
	bool test = false;
	
	string path = "/home/andyser/data/subt_real/pc/extinguisher/";
	vector<int>  list;
	list.clear();
    fstream fin;
    struct stat st = {0};
	if (stat(path.c_str(), &st) == -1) {
	    mkdir(path.c_str(), 0700);
	}

  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  	// Create the segmentation object
  	pcl::SACSegmentation<PointXYZRGB> seg;

	pcl::PointCloud<PointXYZRGB>::iterator index;
	pcl::ConditionAnd<PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<PointXYZRGB> ());
    	range_cond->addComparison (pcl::FieldComparison<PointXYZRGB>::ConstPtr (new pcl::FieldComparison<PointXYZRGB> ("x", pcl::ComparisonOps::GT, 0.0)));
    	range_cond->addComparison (pcl::FieldComparison<PointXYZRGB>::ConstPtr (new pcl::FieldComparison<PointXYZRGB> ("x", pcl::ComparisonOps::LT, 3.0)));
	pcl::ConditionalRemoval<PointXYZRGB> condrem;

	PointCloud<PointXYZRGB>::Ptr pc (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr label_pc (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr origin_pc (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr pc_filter (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr pc_filter_2 (new PointCloud<PointXYZRGB>);
	//pcl::PCLPointCloud2::Ptr cloud_filtered1 (new pcl::PCLPointCloud2 ());

	sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration(10));
	float fx = msg->P[0];
	float fy = msg->P[5];
	float cx = msg->P[2];
	float cy = msg->P[6];

    fin.open("/home/andyser/data/subt_real/val.txt",ios::in);
    fin.getline(line,sizeof(line),'\n');
    while(!fin.eof()){
    	count = 0;
    	
    	pc_filter->clear();
    	label_pc->clear();
    	origin_pc->clear();
    	Mat img[3];
    	
    	// color  mask  depth
    	for (int i = 0;i < 3; i++){
    		cout << line << endl;
    		img[i] = imread(line,CV_LOAD_IMAGE_UNCHANGED);
    		fin.getline(line,sizeof(line),'\n');
    	}
		//s = s + "pc/extinguisher/";

		// cout << s << endl;    	
		// if (stat(s.c_str(), &st) == -1) {
		//     mkdir(s.c_str(), 0700);
		// }	


		cvtColor(img[0], img[0], CV_RGB2BGR);
		for( int nrow = 0; nrow < img[1].rows; nrow++){  
			for(int ncol = 0; ncol < img[1].cols; ncol++){  
				if (img[2].at<unsigned short int>(nrow,ncol) > 1){

					pcl::PointXYZRGB point;
					//PointLabel point;

					float* x = new float(nrow);
					float* y = new float(ncol);
				 	float z = float(img[2].at<unsigned short int>(nrow,ncol))/1000.;
					getXYZ(y,x,z,fx,fy,cx,cy);
					point.x = z;
					point.y = -*y;
					point.z = -*x;
					Vec3b intensity =  img[0].at<Vec3b>(nrow, ncol); 
					if (img[1].at<uint8_t>(nrow,ncol) != 0)				
						list.push_back(count);	
					point.r = int(intensity[0]);
					point.g = int(intensity[1]);
					point.b = int(intensity[2]);
					pc->points.push_back(point);
					free(x);
					free(y);
					count++;
					// delete x;
					// delete y;
				} 
			}  
		} 


		label_pc->points.resize(pc->points.size());
		label_pc->width = pc->points.size();
  		label_pc->height = 1;

  		origin_pc->points.resize(pc->points.size());
  		origin_pc->width = pc->points.size();
  		origin_pc->height = 1;

		for (int i = 0;i < pc->points.size();i++){
			label_pc->points[i].x = pc->points[i].x;
			label_pc->points[i].y = pc->points[i].y;
			label_pc->points[i].z = pc->points[i].z;
			if (test){
				label_pc->points[i].r = pc->points[i].r;
				label_pc->points[i].g = pc->points[i].g;
				label_pc->points[i].b = pc->points[i].b;
			}
			origin_pc->points[i].x = pc->points[i].x;
			origin_pc->points[i].y = pc->points[i].y;
			origin_pc->points[i].z = pc->points[i].z;
			origin_pc->points[i].r = pc->points[i].r;
			origin_pc->points[i].g = pc->points[i].g;
			origin_pc->points[i].b = pc->points[i].b;
		}
		//cout << int(label_pc->points[0].r) << " " << int(label_pc->points[0].g)<< " " << int(label_pc->points[0].b) << endl;
		if (!test){
			for (int i = 0; i < list.size(); i++){
				label_pc->points[list[i]].r = 1;
				label_pc->points[list[i]].g = 1;
				label_pc->points[list[i]].b = 1;
				// label_pc->points[list[i]].label = 1;
			}
		}	
		list.clear();
		if (label_pc->points.size()!=0){

			//pcl::VoxelGrid<PointXYZRGB> sor;
			//sor.setInputCloud (label_pc);
			//sor.setLeafSize (0.01f, 0.01f, 0.01f);
			//sor.filter (*pc_filter);


  			// Optional
  			seg.setOptimizeCoefficients (true);
  			// Mandatory
  			seg.setModelType (pcl::SACMODEL_PLANE);
  			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (200);
  			seg.setDistanceThreshold (0.1);

  			seg.setInputCloud (label_pc);
  			seg.segment (*inliers, *coefficients);
			/*
			for (size_t i = 0; i < inliers->indices.size (); ++i){

				cout << inliers->indices[i]<< endl;
				pcl::PointXYZRGB point;			
				point.x = label_pc->points[inliers->indices[i]].x;
				point.y = label_pc->points[inliers->indices[i]].y;
				point.z = label_pc->points[inliers->indices[i]].z;
				point.r = origin_pc->points[inliers->indices[i]].r;
				point.g = origin_pc->points[inliers->indices[i]].g;
				point.b = origin_pc->points[inliers->indices[i]].b;
				pc_filter->points.push_back(point);
			}
			*/

			extract.setInputCloud (label_pc);
    			extract.setIndices (inliers);
    			extract.setNegative (true);
    			extract.filter (*pc_filter);

    			// build the filter
    			condrem.setCondition (range_cond);
    			condrem.setInputCloud (pc_filter);
    			condrem.setKeepOrganized(true);
    			// apply filter
    			condrem.filter (*pc_filter_2);
			
			/*index = pc_filter->begin();
			for (size_t i = 0; i < pc_filter->points.size (); ++i){
				if (pc_filter->points[i].x > 1){
					//cout << i << "  " << pc_filter->points[i].z << endl;
					pc_filter->erase(index);
				}
				index ++ ;				
			}*/
			

			pc_filter_2->points.resize(pc_filter_2->points.size());
			pc_filter_2->width = pc_filter->points.size();
  			pc_filter_2->height = 1;


			stringstream ss;
			ss << pcd_num;
			pcl::io::savePCDFile(path + "label/" + ss.str() + ".pcd", *pc_filter_2);
			pcl::io::savePCDFile(path + "origin/" +ss.str() + ".pcd", *origin_pc);
			cout << "Save PDC file: " << pcd_num << ".pcd" << endl;
			pcd_num++;
			pc->clear();
			pc_filter->clear();
			pc_filter_2->clear();
			label_pc->clear();
		}
    }

	// Publisher pc2;
	// pc2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc", 10);
	// sensor_msgs::PointCloud2 object_cloud_msg;
	// toROSMsg(*pc, object_cloud_msg);
	// pc->clear();
	// label_pc->clear();
	// object_cloud_msg.header.frame_id = "camera_link";
	// while(1){
	//     pc2.publish(object_cloud_msg);
	// 	Duration(1).sleep();
	// }
	// Spin
	spin ();
}