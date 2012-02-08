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

	//Testing variables
	double dyn0, dyn1, dyn2, dyn3, dyn4, dyn5, dyn6, dyn7;
	int maxdiv;
	bool 		MyOwn_NopenCV;

	//Final variables for node
	bool measure_performance;

	//Image transport
	image_transport::ImageTransport it_;
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

/////////////New stuff for doing two image transports into one callback

	ros::NodeHandlePtr rgb_nh_;
	boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;

	// Subscriptions for camera picture and depth
	image_transport::SubscriberFilter sub_depth_, sub_rgb_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
			sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
	typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
	boost::shared_ptr<Synchronizer> sync_;

	// Publications
	boost::mutex connect_mutex_;
	image_geometry::PinholeCameraModel model_;

public:
	toCompress() :
			nh_("~"), it_(nh_), image_count(0), store(480, 640, CV_16UC1), disturb(
					480, 640, CV_16UC1)
	{
		rgb_nh_.reset(new ros::NodeHandle(nh_, "rgb"));
		ros::NodeHandle depth_nh(nh_, "depth_registered");
		rgb_it_.reset(new image_transport::ImageTransport(*rgb_nh_));
		depth_it_.reset(new image_transport::ImageTransport(depth_nh));

		// Read parameters
		int queue_size;
		nh_.param("queue_size", queue_size, 5);

		// Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
		sync_.reset(
				new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_,
						sub_info_));
		sync_->registerCallback(
				boost::bind(&toCompress::imageCb, this, _1, _2, _3));

		// Monitor whether anyone is subscribed to the output
		image_transport::SubscriberStatusCallback connect_cb = boost::bind(
				&toCompress::connectCb, this);
		// Make sure we don't enter connectCb() between advertising and assigning
		boost::lock_guard<boost::mutex> lock(connect_mutex_);

		//Advertise out camera
		depth_camera_compressed_out = it_.advertiseCamera("compressed", 1,
				connect_cb, connect_cb);
		reconfCbType = boost::bind(&toCompress::reconfigCb, this, _1, _2);
		reconfServer.setCallback(reconfCbType);

		cv::namedWindow(WINDOW); //TODO Remove

	}

	~toCompress()
	{
		cv::destroyWindow(WINDOW); //TODO Remove
	}

	void connectCb()
	{
		boost::lock_guard<boost::mutex> lock(connect_mutex_);
		if (depth_camera_compressed_out.getNumSubscribers() == 0)
		{
			ROS_INFO("CompressNode unsubscribing...");
			sub_depth_.unsubscribe();
			sub_rgb_.unsubscribe();
			sub_info_.unsubscribe();
		}
		else if (!sub_depth_.getSubscriber())
		{
			ROS_INFO("CompressNode now subscribing...");
			sub_depth_.subscribe(*depth_it_, "image_rect", 1);
			sub_rgb_.subscribe(*rgb_it_, "image_rect_color", 1);
			sub_info_.subscribe(*rgb_nh_, "camera_info", 1);
		}
	}

	void reconfigCb(htwAalen_signDetect::toCompressConfig &config,
			uint32_t level)
	{
		measure_performance = config.Measure_Performance;
		MyOwn_NopenCV = config.MyOwn_NopenCV;

		dyn0 = config.double_param0;
		dyn1 = config.double_param1;
		dyn2 = config.double_param2;
		dyn3 = config.double_param3;
		dyn4 = config.double_param4;
		dyn5 = config.double_param5;
		dyn6 = config.double_param6;
		dyn7 = config.double_param7;
		maxdiv =config.maxdiv;


	}

	void from16UC1to32FC1(const cv::Mat &src, cv::Mat &dst)
	{
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				dst.at<Vec1flt>(y, x)[0] = ((float) src.at<Vec1shrt>(y, x)[0])
						/ 1000.0;
			}
		}
	}

	void from32FC1to16UC1(const cv::Mat &src, cv::Mat &dst)
	{
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				dst.at<Vec1shrt>(y, x)[0] =
						(unsigned short int) (src.at<Vec1flt>(y, x)[0] * 1000);
			}
		}
	}



	void myFilter(const cv::Mat &src, cv::Mat &dst)
	{

		ros::Time begin = ros::Time::now();

		if (src.type() == CV_16UC1)
		{
			cv::Mat filter = src.clone();
			cv::boxFilter(filter, filter, 3, cv::Size(6, 2), cv::Point(-1, -1), 1, 0);
			//cv::GaussianBlur(filter,filter,cv::Size(dyn4,dyn5),dyn6,dyn7);
			cv::medianBlur(filter, filter, 3);

			//Update non zero pixels
			for (int y = 0; y < src.rows; y++)
			{
				for (int x = 0; x < src.cols; x++)
				{
					short realValue = src.at<Vec1shrt>(y, x)[0];
					short filteredValue = filter.at<Vec1shrt>(y, x)[0];
					short maxDifference = pow(realValue, 2) / (480000); //Maximal difference from the real value
					if (realValue>0)
					{
						if (abs(realValue - filteredValue) > maxDifference)
						{
							dst.at<Vec1shrt>(y, x)[0] = realValue;// realValue; //TODO maybe we should use the maxdiff value here?
						}
						else
						{
							dst.at<Vec1shrt>(y, x)[0] = filteredValue;
						}
					}
				}
			}
			cv::medianBlur(dst, dst, 3);

		}
		else
		{
			ROS_ERROR("MyFilter: Wrong Image Type");
		}

		ROS_INFO("Filtering took %f \n", (begin.nsec-ros::Time::now().nsec)/1000000000.0);
	}

	void OpenCVBilateralFilter(const cv::Mat &src, cv::Mat &dst)
	{



		cv::Mat filtered_in;
		cv::Mat filtered(480, 640, CV_32FC1);
		dst=cv::Mat(480,640,CV_16UC1);

		if (src.type() == CV_32FC1)
		{
			filtered_in = src;
		}
		else if (src.type() == CV_16UC1)
		{
			filtered_in = cv::Mat(480, 640, CV_32FC1);
			from16UC1to32FC1(src, filtered_in);
		}
		else
		{
			ROS_INFO("OpenCVBilateralFilter: Unsupported format!");
		}
		ros::Time begin = ros::Time::now();
		cv::bilateralFilter(filtered_in, filtered, dyn0, dyn1, dyn2, cv::BORDER_DEFAULT);



		//Update non zero pixels
		for (int y = 0; y < src.rows; y++)
		{
			for (int x = 0; x < src.cols; x++)
			{
				float realValue = src.at<Vec1flt>(y, x)[0];
				//float filteredValue = filtered.at<Vec1flt>(y, x)[0];
				if (realValue>0)
				{
						dst.at<Vec1shrt>(y, x)[0]=(short)(filtered.at<Vec1flt>(y, x)[0]*1000.0);
				}
				else
				{
					    dst.at<Vec1shrt>(y, x)[0]=0;
				}
			}
		}


		ROS_INFO("Filtering took %f \n", (begin.nsec-ros::Time::now().nsec)/1000000000.0);
	}











	void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
			     const sensor_msgs::ImageConstPtr& rgb_msg,
			     const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		cv_bridge::CvImagePtr imgPtrDepth, imgPtrRGB;
		if (depth_msg->encoding == "16UC1") //Kinect raw image (millimeters)
		{
			try
			{
				imgPtrDepth = cv_bridge::toCvCopy(depth_msg, "16UC1");
				imgPtrRGB = cv_bridge::toCvCopy(rgb_msg, "mono8");

			} catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			//Filter the current picture
			if(MyOwn_NopenCV)
			{
				myFilter(imgPtrDepth->image,store);
			}
			else
			{
				OpenCVBilateralFilter(imgPtrDepth->image,store);
			}


			//combine 3 Pictures to one
			image_count++;
			if (image_count >= 3)
			{

				//Create output mat
				cv::Mat out(480, 640, CV_8UC3);

				//Copy into bgr8 output mat
				for (int y = 0; y < store.rows; y++)
				{
					for (int x = 0; x < store.cols; x++)
					{
						uchar lowerByte = store.at<Vec1shrt>(y, x)[0] & 0xFF;
						uchar upperByte = (store.at<Vec1shrt>(y, x)[0] & 0xFF00)
								>> 8;

						out.at<Vec3char>(y, x)[0] =
								((!(upperByte % 2)) || !upperByte) ?
										lowerByte : ~lowerByte;
						out.at<Vec3char>(y, x)[1] = upperByte;
						out.at<Vec3char>(y, x)[2] = 0;
					}
				}

				//Set new image and encoding
				imgPtrDepth->image = out;
				imgPtrDepth->encoding = "bgr8";

				//Publish image
				sensor_msgs::CameraInfo infoOut = *info_msg;
				depth_camera_compressed_out.publish(*imgPtrDepth->toImageMsg(), infoOut,
						info_msg->header.stamp);

				image_count = 0;
				store = cv::Mat::zeros(store.rows, store.cols, CV_16U);
			}


			cv::Mat testin=imgPtrRGB->image;
			cv::medianBlur(testin,testin,dyn5);
			cv::Mat testout;

			try
			{
//				cv::boxFilter(testin, testin, 3, cv::Size(6, 2), cv::Point(-1, -1), 1, 0);
				cv::Canny(testin,testout,dyn6,dyn7);
			} catch (cv::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}


			cv::imshow(WINDOW,testout);
			cv::waitKey(3);

		}
		else
		{
			ROS_ERROR("Unsupported format: [%s]", depth_msg->encoding.c_str());
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

