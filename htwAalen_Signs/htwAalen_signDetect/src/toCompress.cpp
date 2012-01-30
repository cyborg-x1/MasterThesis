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

class toCompress
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber depth_camera_in;
  image_transport::CameraPublisher depth_camera_compressed_out;

  typedef cv::Vec<float, 1> Vec1flt;
  typedef cv::Vec<uchar, 3> Vec3char;
  typedef cv::Vec<short, 1> Vec1shrt;


public:
  toCompress()
    : it_(nh_)
  {
    depth_camera_compressed_out = it_.advertiseCamera("/comp", 1);
    depth_camera_in = it_.subscribeCamera("/in", 1, &toCompress::imageCb, this);

    cv::namedWindow(WINDOW); //TODO Remove
  }

  ~toCompress()
  {
    cv::destroyWindow(WINDOW); //TODO Remove
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info)
  {
    cv_bridge::CvImagePtr img;
    //printf("encoding: %s\n", msg->encoding.c_str());


    if(msg->encoding=="16UC1") //Kinect raw image (millimeters)
    {
		try
		{
		  img = cv_bridge::toCvCopy(msg, "16UC1");
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		//Create output mat
		cv::Mat out(480,640, CV_8UC3);

		//Copy into bgr8 output mat
		for(int y = 0; y < img->image.rows; y++)
		{
			for(int x = 0; x < img->image.cols; x++)
			{
				uchar lowerByte=img->image.at<Vec1shrt>(y,x)[0]&0xFF;
				uchar upperByte=(img->image.at<Vec1shrt>(y,x)[0]&0xFF00)>>8;

				out.at<Vec3char>(y,x)[0]=((!(upperByte%2)) || !upperByte)?lowerByte:~lowerByte;
				out.at<Vec3char>(y,x)[1]=upperByte;
				out.at<Vec3char>(y,x)[2]=0;
			}
		}

		//Set new image and encoding
		img->image=out;
		img->encoding="bgr8";


		//Publish image
		sensor_msgs::CameraInfo infoOut=*info;
		depth_camera_compressed_out.publish(*img->toImageMsg(), infoOut,info->header.stamp);
    }
    else if(msg->encoding=="32FC1") //ROS depth image (meters)
    {
		try
		{
		  img = cv_bridge::toCvCopy(msg, "32FC1");
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		//Create output mat
		cv::Mat out(480,640, CV_8UC3);

		//Copy into bgr8 output mat
		for(int y = 0; y < img->image.rows; y++)
		{
			for(int x = 0; x < img->image.cols; x++)
			{
				uchar lowerByte=((unsigned int)(img->image.at<Vec1flt>(y,x)[0]*1000))&0xFF;
				uchar upperByte=(((unsigned int)(img->image.at<Vec1flt>(y,x)[0]*1000))&0xFF00)>>8;

				out.at<Vec3char>(y,x)[0]=((!(upperByte%2)) || !upperByte)?lowerByte:~lowerByte;
				out.at<Vec3char>(y,x)[1]=upperByte;
				out.at<Vec3char>(y,x)[2]=0;
			}
		}


		//Set new image and encoding
		img->image=out;
		img->encoding="bgr8";


		//Publish image
		sensor_msgs::CameraInfo infoOut=*info;
		depth_camera_compressed_out.publish(*img->toImageMsg(), infoOut,info->header.stamp);
    }


    }



};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  toCompress ic;
  ros::spin();
  return 0;
}


