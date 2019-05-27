#include "save_rgb.h"

void save_rgb::callback_save(const sensor_msgs::ImageConstPtr& image){
	cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

	if (count % 12 == 0){
		//imwrite("/home/andyser/data/1.png", img_ptr_img->image);
		stringstream ss;
		ss << img_num;
		cvtColor(img_ptr_img->image, img_ptr_img->image, CV_BGR2RGB);
		imwrite(path + "image/" + obj + scene + ss.str() + ".jpg",img_ptr_img->image); 
		img_num ++;
	}
   	count ++;
	return;
}

save_rgb::save_rgb(){
	obj = "demo/";
	scene = "scene000019/";

	count = 0;
	img_num = 0;
	struct stat st = {0};

	////////////////// create folder
	path = "/home/andyser/data/VIMO_demo/";
	if (stat(path.c_str(), &st) == -1) {
	    mkdir(path.c_str(), 0700);
	}	
	if (stat((path + "image/" + obj).c_str(), &st) == -1) {
	    mkdir((path + "image/" + obj).c_str(), 0700);
	}
	if (stat((path + "image/" + obj + scene).c_str(), &st) == -1) {
    	mkdir((path + "image/" + obj + scene).c_str(), 0700);
	}

	NodeHandle nh;
	rgb_image = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, &save_rgb::callback_save,this); 
}
int main(int argc, char** argv){
    init(argc, argv, "save_rgb");
    save_rgb save_rgb;
    spin();
    return 0;
}