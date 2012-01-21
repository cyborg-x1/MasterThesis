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

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class SignDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  SignDetector()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("debug", 1);
    image_sub_ = it_.subscribe("in", 1, &SignDetector::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~SignDetector()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    	int MAX_KERNEL_LENGTH=31;


    	cv::Mat cpy=cv_ptr->image.clone();
    	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
		{
			cv::GaussianBlur(cpy, cv_ptr->image,  cv::Size( i, i ), 0, 10 );
		}
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,255,255));

    	cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  SignDetector ic;
  ros::spin();
  return 0;
}


