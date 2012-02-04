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
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <htwAalen_signDetect/toCompressConfig.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace enc = sensor_msgs::image_encodings;



static const char WINDOW[] = "Image window";

class toCompress
{





  ros::NodeHandle nh_;

  //Dynamic reconfigure
   dynamic_reconfigure::Server<htwAalen_signDetect::toCompressConfig> reconfServer;
   dynamic_reconfigure::Server<htwAalen_signDetect::toCompressConfig>::CallbackType reconfCbType;


   double dyn0, dyn1, dyn2, dyn3, dyn4, dyn5, dyn6, dyn7;
   bool en;


  //Image transport
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

    reconfCbType = boost::bind(&toCompress::reconfigCb, this ,_1, _2);
    reconfServer.setCallback(reconfCbType);

    cv::namedWindow(WINDOW); //TODO Remove

  }

  ~toCompress()
  {
    cv::destroyWindow(WINDOW); //TODO Remove
  }



  void reconfigCb(htwAalen_signDetect::toCompressConfig &config, uint32_t level)
  {


    dyn0=config.double_param0;
    dyn1=config.double_param1;
    dyn2=config.double_param2;
    dyn3=config.double_param3;
    dyn4=config.double_param4;
    dyn5=config.double_param5;
    dyn6=config.double_param6;
    dyn7=config.double_param7;
    en=config.enable_filter;

  }

  void from16UC1to32FC1(cv::Mat &src, cv::Mat &dst)
  {
		for(int y = 0; y < 480; y++)
		{
			for(int x = 0; x < 640; x++)
			{
				dst.at<Vec1flt>(y,x)[0]=((float)src.at<Vec1shrt>(y,x)[0])/1000.0;
			}
		}
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


		cv::Mat filtered_in(480,640,CV_32FC1);
		cv::Mat filtered(480,640,CV_32FC1);

		from16UC1to32FC1(img->image,filtered_in);

		if(en)
		{
			//	cv::boxFilter(filtered,filtered,3,cv::Size(6,2),cv::Point(-1,-1),1,0);
			//	cv::medianBlur(filtered,filtered,dyn3);
			//4.7 0.1 6.8
			cv::bilateralFilter(filtered_in,filtered, dyn0,dyn1,dyn2,cv::BORDER_DEFAULT);
		}


		//Update non zero pixels
		for(int y = 0; y < store.rows; y++)
		{
			for(int x = 0; x < store.cols; x++)
			{
				short realValue=img->image.at<Vec1shrt>(y,x)[0];
				short filteredValue=filtered.at<Vec1flt>(y,x)[0]*1000;
				if(realValue)
				{//dyn0=12
					if((1 && filteredValue) || abs(realValue - filteredValue)<=(pow(realValue,2)/(dyn0*10000))) //Limit difference from filtered and real points
					{
						store.at<Vec1shrt>(y,x)[0]=filteredValue;
					}
					else
					{
						store.at<Vec1shrt>(y,x)[0]=filtered_in.at<Vec1flt>(y,x)[0]*1000;
					}
				}
			}
		}

		if(en)
		{
			//cv::medianBlur(store,store,3);
		}


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
  ros::init(argc, argv, "toCompress");
  toCompress ic;
  ros::spin();
  return 0;
}


