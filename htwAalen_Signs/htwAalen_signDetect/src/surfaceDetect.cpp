/*
 * signDetect.cpp
 *
 *      Author: Christian Holl
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>



namespace enc = sensor_msgs::image_encodings;




static const char WINDOW[] = "Image window";

class SignDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber depth_camera_sub;
  image_transport::CameraPublisher depth_camera_pub;

  typedef cv::Vec<uchar, 3> Vec3char;
  typedef cv::Vec<short, 1> Vec1shrt;


public:
  SignDetector()
    : it_(nh_)
  {
    depth_camera_pub = it_.advertiseCamera("debug", 1);
    depth_camera_sub = it_.subscribeCamera("/camera/depth/image_raw", 1, &SignDetector::imageCb, this);//, image_transport::TransportHints("compressed"));

    cv::namedWindow(WINDOW);
  }

  ~SignDetector()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::Mat out(480,640, CV_8UC3);
    cv::Mat cpy=cv_ptr->image.clone();


#if(0)
		static int imgcnt;
		if(imgcnt == 10)
		{
			cv::imwrite("/home/cyborg-x1/image/sFDetect.jpg",cv_ptr->image);
			imgcnt++;
		}
		else
		{
			if(imgcnt<10) imgcnt++;
		}
#endif

    	cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);


    //Publish image again
    sensor_msgs::CameraInfo infoOut=*info;
    depth_camera_pub.publish(*cv_ptr->toImageMsg(), infoOut,info->header.stamp);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  SignDetector ic;
  ros::spin();
  return 0;
}


