#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// create folder
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
//

using namespace ros;
using namespace std;
using namespace cv;

class save_rgb{
  public:
    save_rgb();
    void callback_save(const sensor_msgs::ImageConstPtr&);
  private:
  	ros::Subscriber rgb_image;
  	int count;
  	int img_num;
  	string path;
  	string scene;
  	string obj;
};