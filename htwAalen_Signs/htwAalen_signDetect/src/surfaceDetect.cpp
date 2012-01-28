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
//	printf("%i\n", out.at<unsigned short>(480/2,680/2));

	for(int y = 0; y < cv_ptr->image.rows; y++)
    {
        for(int x = 0; x < cv_ptr->image.cols; x++)
        {
        	out.at<Vec3char>(y,x)[0]=cv_ptr->image.at<Vec1shrt>(y,x)[0]&0xFF;
        	out.at<Vec3char>(y,x)[1]=(cv_ptr->image.at<Vec1shrt>(y,x)[0]&0xFF00)>>8;
        	out.at<Vec3char>(y,x)[2]=~((cv_ptr->image.at<Vec1shrt>(y,x)[0]&0xFF00)>>8);
        }
    }

	cv_ptr->image=out;
	cv_ptr->encoding="bgr8";


#if 0
    	cv::Mat cpy=cv_ptr->image.clone();
		cv::Mat cornerimage(480,640,CV_8UC1);


//    	int MAX_KERNEL_LENGTH=31;
//    	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//		{
//			cv::GaussianBlur(cpy, cv_ptr->image,  cv::Size( i, i ), 0, 2 );
//		}
		//cv::medianBlur(cpy, cv_ptr->image,  5);



		for(int y = 0; y < cv_ptr->image.rows; y++)
	    {
	        float* Dy = cv_ptr->image.ptr<float>(y);
	        for(int x = 0; x < cv_ptr->image.cols; x++)
	        {
	        	float value=Dy[x];
	        	if(value > 5 &&0)
	        	{
	        		Dy[x]=((int)(value));
	        	}
	        	else if(value > 1.8)
	        	{
	        		Dy[x]=((int)(value * 10 + .5) / 10.0);
	        	}
	        	else if(value > 1.0)
	        	{
	        		Dy[x]=((int)(value * 1000 + .5) / 1000.0);
	        	}
	        	else
	        	{
	        		Dy[x]=((int)(value * 10000 + .5) / 10000.0);
	        	}

	        	if(Dy[x] < 0)
	        	{
	        		//nan
	        		Dy[x]=0;
	        	}
	        }
	    }




		//cv::medianBlur(cv_ptr->image, cv_ptr->image,  3.5);

		for(int y = 0; y < cv_ptr->image.rows; y++)
	    {
	        float* Dy = cv_ptr->image.ptr<float>(y);
	        for(int x = 0; x < cv_ptr->image.cols; x++)
	        {
	        	cornerimage.at<unsigned char>(y,x)=255*((Dy[x])/(10));
	        }
	    }
	//	cv::Canny(cornerimage,cornerimage, 10, 20, 3 );
	//	cv::Sobel(cv_ptr->image,cornerimage,5,3,3,5,3);

		cv::GaussianBlur(cornerimage,cornerimage,  cv::Size( 3, 3 ), 0, 3 );
		cv::Laplacian(cornerimage,cornerimage,0,5,3,20);

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


