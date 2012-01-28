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

class toCompressFromCompress
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber depth_camera_in;
  image_transport::CameraPublisher depth_camera_compressed_out;
  image_transport::CameraPublisher depth_camera_uncompressed_out;

  typedef cv::Vec<float, 1> Vec1flt;
  typedef cv::Vec<uchar, 3> Vec3char;
  typedef cv::Vec<short, 1> Vec1shrt;


public:
  toCompressFromCompress()
    : it_(nh_)
  {
    depth_camera_compressed_out = it_.advertiseCamera("/comp", 1);
    depth_camera_uncompressed_out = it_.advertiseCamera("/uncomp",1);
    depth_camera_in = it_.subscribeCamera("/in", 1, &toCompressFromCompress::imageCb, this);

    cv::namedWindow(WINDOW); //TODO Remove
  }

  ~toCompressFromCompress()
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
				out.at<Vec3char>(y,x)[0]=img->image.at<Vec1shrt>(y,x)[0]&0xFF;
				out.at<Vec3char>(y,x)[1]=(img->image.at<Vec1shrt>(y,x)[0]&0xFF00)>>8;
				out.at<Vec3char>(y,x)[2]=out.at<Vec3char>(y,x)[1]-out.at<Vec3char>(y,x)[0];
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
				out.at<Vec3char>(y,x)[0]=((unsigned int)(img->image.at<Vec1flt>(y,x)[0]*1000))&0xFF;
				out.at<Vec3char>(y,x)[1]=(((unsigned int)(img->image.at<Vec1flt>(y,x)[0]*1000))&0xFF00)>>8;
				out.at<Vec3char>(y,x)[2]=out.at<Vec3char>(y,x)[1]-out.at<Vec3char>(y,x)[0];
			}
		}


		//Set new image and encoding
		img->image=out;
		img->encoding="bgr8";


		//Publish image
		sensor_msgs::CameraInfo infoOut=*info;
		depth_camera_compressed_out.publish(*img->toImageMsg(), infoOut,info->header.stamp);
    }
    else if(msg->encoding=="bgr8")
    {

		try
		{
		  img = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		//Create output mat
		cv::Mat out(480,640, CV_32FC1);

		//Copy into bgr8 output mat
		for(int y = 0; y < img->image.rows; y++)
		{
			for(int x = 0; x < img->image.cols; x++)
			{
				out.at<Vec1flt>(y,x)[0]=((float)(img->image.at<Vec3char>(y,x)[0]|(img->image.at<Vec3char>(y,x)[1])<<8))/1000;
			}
		}


		//Set new image and encoding
		img->image=out;
		img->encoding="32FC1";


		//Publish image
		sensor_msgs::CameraInfo infoOut=*info;
		depth_camera_uncompressed_out.publish(*img->toImageMsg(), infoOut,info->header.stamp);
    }
    else
    {
    	ROS_ERROR("Unsupported format: [%s]",msg->encoding.c_str());
    	return;
    }

//	cv::imshow(WINDOW, img->image); //TODO Remove
//	cv::waitKey(3);					//TODO Remove

    }



};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  toCompressFromCompress ic;
  ros::spin();
  return 0;
}


