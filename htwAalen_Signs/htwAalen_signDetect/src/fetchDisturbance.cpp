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

  int count;
  int distance_mm;

  cv::Mat disturb;


  typedef cv::Vec<float, 1> Vec1flt;
  typedef cv::Vec<uchar, 3> Vec3char;
  typedef cv::Vec<short, 1> Vec1shrt;
  typedef cv::Vec<char, 1> Vec3schar;

public:
  fromCompress()
    : it_(nh_)
    ,count(0)
  	,disturb(480,640,CV_8UC3)
  {
	std::string tp_hint;

    depth_camera_in = it_.subscribeCamera("/camera/depth_registered/image_rect_raw", 1, &fromCompress::imageCb, this);
    nh_.param("distance",distance_mm, 1173);
    disturb=cv::Mat::zeros(disturb.rows,disturb.cols,CV_8UC3);

    printf("Using distance: %i \n ",distance_mm);

    cv::namedWindow(WINDOW); //TODO Remove
  }

  ~fromCompress()
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


		count++;
		if(count >= 3)
		{
			for(int y = 0; y < disturb.rows; y++)
			{
				for(int x = 0; x < disturb.cols; x++)
				{
					if(img->image.at<Vec1shrt>(y,x)[0]>100)
					{
						short value=img->image.at<Vec1shrt>(y,x)[0]-distance_mm;
						//printf("%i \n", value);
						if(value<0)printf("d___\n");

						disturb.at<Vec3char>(y,x)[0]=value&0x00FF;
						disturb.at<Vec3char>(y,x)[1]=(value&0xFF00)>>8;
						disturb.at<Vec3char>(y,x)[2]=0;
					}
					else
					{
						disturb.at<Vec3char>(y,x)[0]=0;
						disturb.at<Vec3char>(y,x)[1]=0;
						if(img->image.at<Vec1shrt>(y,x)[0]!=0)
						{
							printf("Found strange pixel at (%i/%i) with Value (%i) \n",x,y,img->image.at<Vec1shrt>(y,x)[0]);
							disturb.at<Vec3char>(y,x)[2]=0xFF;
						}
						else
						{
							disturb.at<Vec3char>(y,x)[2]=0;
						}
					}
				}
			}

			cv::imwrite("disturbance.png",disturb);

			//successfull end
			printf("Finished... End!\n");
			exit(0);

		}


	}
	else
	{
		ROS_ERROR("Unsupported format: [%s]\n",msg->encoding.c_str());
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


