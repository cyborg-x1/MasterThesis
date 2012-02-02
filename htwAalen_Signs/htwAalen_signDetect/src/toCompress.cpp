/*
 * signDetect.cpp
 *
 *      Author: Christian Holl
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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
  image_transport::CameraSubscriber rgb_camera_in;
  image_transport::CameraPublisher depth_camera_compressed_out;







  //Counter for images
  unsigned int image_count;

  //Types for different mats
  typedef cv::Vec<float, 1> Vec1flt;
  typedef cv::Vec<uchar, 3> Vec3char;
  typedef cv::Vec<short, 1> Vec1shrt;

  //Mat for storage
  cv::Mat store;
  cv::Mat disturb;

public:
  toCompress()
    : nh_("~")
    , it_(nh_)
    , image_count(0)
  	, store(480,640,CV_16UC1)
  	, disturb(480,640,CV_16UC1)
  {
    depth_camera_compressed_out = it_.advertiseCamera("/comp", 1);
    depth_camera_in = it_.subscribeCamera("/in", 1, &toCompress::imageCb, this);

    disturb=cv::Mat::zeros(disturb.rows,disturb.cols,CV_16UC1);




	cv::Mat disturb_in=cv::imread("/home/cyborg-x1/disturbance.png",1);

	for(int y = 0; y < disturb_in.rows; y++)
	{
		for(int x = 0; x < disturb_in.cols; x++)
		{
			if(disturb_in.at<Vec3char>(y,x)[0] || disturb_in.at<Vec3char>(y,x)[1])
			{
				disturb.at<Vec1shrt>(y,x)[0]=(disturb_in.at<Vec3char>(y,x)[0] | disturb_in.at<Vec3char>(y,x)[1]<<8);
				printf("%i,", disturb.at<Vec1shrt>(y,x)[0]);
			}
		}
	}
	cv::medianBlur(disturb,disturb,3);
	printf("\n");


    cv::namedWindow(WINDOW); //TODO Remove

  }

  ~toCompress()
  {
    cv::destroyWindow(WINDOW); //TODO Remove
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info)
  {
    cv_bridge::CvImagePtr img;
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



		//Update non zero pixels
		for(int y = 0; y < store.rows; y++)
		{
			for(int x = 0; x < store.cols; x++)
			{
				if(img->image.at<Vec1shrt>(y,x)[0])
				{
					store.at<Vec1shrt>(y,x)[0]=img->image.at<Vec1shrt>(y,x)[0]-disturb.at<Vec1shrt>(y,x)[0];
				}
			}
		}


//		 cv::imshow(WINDOW, disturb);
//		 cv::waitKey(3);




		image_count++;
		if(image_count>=3)
		{

		//Create output mat
		cv::Mat out(480,640, CV_8UC3);

		//Copy into bgr8 output mat
		for(int y = 0; y < store.rows; y++)
		{
			for(int x = 0; x < store.cols; x++)
			{
				uchar lowerByte=store.at<Vec1shrt>(y,x)[0]&0xFF;
				uchar upperByte=(store.at<Vec1shrt>(y,x)[0]&0xFF00)>>8;

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

		image_count=0;
		store = cv::Mat::zeros(store.rows,store.cols,CV_16U);
		}
	}
	else
	{
		ROS_ERROR("Unsupported format: [%s]",msg->encoding.c_str());
		return;
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


