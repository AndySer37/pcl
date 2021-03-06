#include "depth_to_point.h"

// void depth_to_point::generate_pointcloud(PointXYZRGB::Ptr point){
// 	/*Decode text pointcloud and text direction
// 	*/
// 	std_msgs::Float32MultiArray pointarray = text_segment_msg.pc_array;
// 	geometry_msgs::Vector3 text_pose_x = text_segment_msg.text_direc;
// 	point->points.resize(pointarray.layout.dim[0].size/7) ;
// 	point->height = 1;
// 	point->width = pointarray.layout.dim[0].size/7;
// 	point->header.frame_id = "camera1_color_optical_frame";
// 	int count = 0;
// 	for (int i=0;i<pointarray.layout.dim[0].size/7;i++){
// 		point->points[i].x=pointarray.data[count++];
// 		point->points[i].y=pointarray.data[count++];
// 		point->points[i].z=pointarray.data[count++];
// 		point->points[i].r=pointarray.data[count++];
// 		point->points[i].g=pointarray.data[count++];
// 		point->points[i].b=pointarray.data[count++];
// 		count++;
// 	}

// 	std::vector<int> indices;
// 	sensor_msgs::PointCloud2 object_cloud_msg;
// 	pcl::toROSMsg(*point, object_cloud_msg);
// 	obstacle_cloud_publisher.publish(object_cloud_msg);
// 	pcl::removeNaNFromPointCloud(*point, *point, indices);
// 	return ;
// }

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

void depth_to_point::getXYZ(float* a, float* b,float zc){

	float inv_fx = 1.0/fx;
	float inv_fy = 1.0/fy;
	*a = (*a - cx) * zc * inv_fx;
	*b = (*b - cy) * zc * inv_fy;
	return;
}

void depth_to_point::callback_save(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth_image){
	cv_bridge::CvImagePtr img_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
	cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

	if (count % 35 == 0){
		//imwrite("/home/andyser/data/1.png", img_ptr_img->image);
		stringstream ss;
		ss << img_num;
		cvtColor(img_ptr_img->image, img_ptr_img->image, CV_BGR2RGB);
		imwrite(path + "image/" + obj + scene + ss.str() + ".jpg",img_ptr_img->image); 
		imwrite(path + "depth/" + obj + scene + ss.str() + ".png",img_ptr_depth->image); 
		img_num ++;
	}
   	count ++;
	return;
}
void depth_to_point::callback_sync(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth_image){
	pc.reset(new PointCloud<PointXYZRGB>());
	cv_bridge::CvImagePtr img_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
	cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
	for( int nrow = 0; nrow < img_ptr_depth->image.rows; nrow++){  
       for(int ncol = 0; ncol < img_ptr_depth->image.cols; ncol++){  
       	if (img_ptr_depth->image.at<unsigned short int>(nrow,ncol) > 1){
       		
       		pcl::PointXYZRGB point;
       		float* x = new float(nrow);
       		float* y = new float(ncol);
       	 	float z = float(img_ptr_depth->image.at<unsigned short int>(nrow,ncol))/1000.;

       		getXYZ(y,x,z);
       		point.x = z;
       		point.y = -*y;
       		point.z = -*x;
       		Vec3b intensity =  img_ptr_img->image.at<Vec3b>(nrow, ncol); 
       		point.r = int(intensity[0]);
       		point.g = int(intensity[1]);
       		point.b = int(intensity[2]);
       		pc->points.push_back(point);
       		free(x);
       		free(y);
       		// delete x;
       		// delete y;
       	} 
       }  
    } 

    //cout << pc->points.size() << endl;
    
    sensor_msgs::PointCloud2 object_cloud_msg;
    toROSMsg(*pc, object_cloud_msg);
    object_cloud_msg.header.frame_id = "camera_link";
    pc2.publish(object_cloud_msg);

	// pc->width    = pc->points.size();
	// pc->height   = 1;
	// pc->is_dense = false;
 //   	pcl::io::savePCDFileASCII ("/home/andyser/code/test_pcd.pcd", *pc);
	return;
}
depth_to_point::depth_to_point(){
	obj = "backpack/";
	scene = "scene000077/";

	count = 0;
	img_num = 0;
	struct stat st = {0};

	////////////////// create folder
	path = "/home/andyser/data/subt_real/";
	if (stat(path.c_str(), &st) == -1) {
	    mkdir(path.c_str(), 0700);
	}	
	if (stat((path + "depth/" + obj).c_str(), &st) == -1) {
	    mkdir((path + "depth/"+ obj).c_str(), 0700);
	}
	if (stat((path + "image/" + obj).c_str(), &st) == -1) {
	    mkdir((path + "image/" + obj).c_str(), 0700);
	}
	if (stat((path + "depth/" + obj + scene).c_str(), &st) == -1) {
	    mkdir((path + "depth/" + obj + scene).c_str(), 0700);
	}
		if (stat((path + "image/" + obj + scene).c_str(), &st) == -1) {
	    mkdir((path + "image/" + obj + scene).c_str(), 0700);
	}
	//////////////////

	NodeHandle nh;
	pc2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc", 10);
	//depth_image = nh.subscribe<sensor_msgs::Image>("/depth_image", 1, &depth_to_point::callback,this); 
	img_sub.subscribe(nh, "/camera/color/image_rect_color", 1);
	depth_sub.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);
	sync_.reset(new Sync(MySyncPolicy(1), img_sub, depth_sub));
	sync_->registerCallback(boost::bind(&depth_to_point::callback_save, this, _1, _2));

}
void depth_to_point::get_msg(){
	// sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration(10));
	// fx = msg->P[0];
	// fy = msg->P[5];
	// cx = msg->P[2];
	// cy = msg->P[6];
	// int count = 0;
	// for(int i = 0; i < 3; i++)
	// 	for(int j = 0; j < 4; j++)
	// 		Projection[i][j] = msg->P[count++];

	// realsense2_camera::ExtrinsicsConstPtr msg1 = ros::topic::waitForMessage<realsense2_camera::Extrinsics>("/camera/extrinsics/depth_to_color",ros::Duration(10));
	// count = 0;
	// for(int i = 0; i < 3; i++)
	// 	for(int j = 0; j < 3; j++)
	// 		extrinsics[i][j] = msg1->rotation[count++];
	// for(int i = 0; i < 3 ; i++)
	// 	extrinsics[i][3] = msg1->translation[i];
	return;
}
int main(int argc, char** argv){
    init(argc, argv, "depth_to_point");
    depth_to_point depth_to_point;
    depth_to_point.get_msg();
    spin();
    return 0;
}