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
#include <string>
#include <math.h>

namespace enc = sensor_msgs::image_encodings;


static const char WINDOW[] = "Image window";

class fromCompress
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
  fromCompress()
    : it_(nh_)
  {
	std::string tp_hint;

    depth_camera_uncompressed_out = it_.advertiseCamera("/uncomp",1);
    depth_camera_in = it_.subscribeCamera("/in", 1, &fromCompress::imageCb, this, image_transport::TransportHints("compressed"));

    cv::namedWindow(WINDOW); //TODO Remove
  }

  ~fromCompress()
  {
    cv::destroyWindow(WINDOW); //TODO Remove
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info)
  {
    cv_bridge::CvImagePtr img;

    	//Check encoding
		if(msg->encoding=="bgr8")
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

					uchar lowerByte=img->image.at<Vec3char>(y,x)[0];
					uchar upperByte=img->image.at<Vec3char>(y,x)[1];

					lowerByte=(((!(upperByte%2)) || !upperByte)?lowerByte:~lowerByte);

					unsigned short currentShort=(upperByte<<8) | (lowerByte<<0);

		//			if(x==100 && y==100) printf("Uncomp: %#x %#x %#x\n",upperByte, lowerByte, ~lowerByte);

					out.at<Vec1flt>(y,x)[0]=(float)currentShort/1000;
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

    }



};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  fromCompress ic;
  ros::spin();
  return 0;
}


