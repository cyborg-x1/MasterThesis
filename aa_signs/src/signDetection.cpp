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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <set>
#include <iterator>
#include <fstream>
#include <iostream>
#include <KinectTools/KinectTools.hpp>

#include "aa_signs/signDetectionConfig.h"
namespace enc = sensor_msgs::image_encodings;


class signDetection
{
	//NodeHandle
	ros::NodeHandle nh_;

	//Dynamic reconfigure
	dynamic_reconfigure::Server<aa_signs::signDetectionConfig> reconfServer;
	dynamic_reconfigure::Server<aa_signs::signDetectionConfig>::CallbackType reconfCbType;

	//Types for different mats
	typedef cv::Vec<float, 1> Vec1flt;
	typedef cv::Vec<uchar, 3> Vec3uchar;
	typedef cv::Vec<short, 1> Vec1shrt;
	typedef cv::Vec<uchar, 1> Vec1uchar;


	//Publishers for output debug info
	image_transport::ImageTransport it_out;
	image_transport::CameraPublisher rgb_out;
	image_transport::CameraPublisher depth_out;

	//Node handle
	ros::NodeHandlePtr rgb_nh_;
	boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;


	//Image transports and synchronizers (stolen from xyzrgb-pointcloud-nodelet :-) *thanks* )
	image_transport::SubscriberFilter sub_depth_, sub_rgb_;

	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
			sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;

	typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

	boost::shared_ptr<Synchronizer> sync_;

	image_geometry::PinholeCameraModel model_;

	//Surface Angles
	int max_angle_x;
	int max_angle_y;

public:
	signDetection() :
			nh_("~"), it_out(nh_)
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
				boost::bind(&signDetection::imageCb, this, _1, _2, _3));




		//Advertise out camera
		reconfCbType = boost::bind(&signDetection::reconfigCb, this, _1, _2);
		reconfServer.setCallback(reconfCbType);

		rgb_out = it_out.advertiseCamera("out_rgb",1);
		depth_out = it_out.advertiseCamera("out_depth",1);

		//Subscribe topics
		sub_depth_.subscribe(*depth_it_, "image_rect", 1);
		sub_rgb_.subscribe(*rgb_it_, "image_rect_color", 1);
		sub_info_.subscribe(*rgb_nh_, "camera_info", 1);
	}

	~signDetection()
	{

	}

	void reconfigCb(aa_signs::signDetectionConfig &config,
			uint32_t level)
	{
		max_angle_x=config.max_angle_x;
		max_angle_y=config.max_angle_y;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
			     const sensor_msgs::ImageConstPtr& rgb_msg,
			     const sensor_msgs::CameraInfoConstPtr& info_msg)
	{

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(info_msg);
		cv_bridge::CvImagePtr imgPtrDepth, imgPtrRGB;

		//Kinect raw image (millimeters)
		if (depth_msg->encoding == "16UC1")
		{
			try
			{
				imgPtrDepth = cv_bridge::toCvCopy(depth_msg, "16UC1");
				imgPtrRGB = cv_bridge::toCvCopy(rgb_msg, "bgr8");

			} catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}


			cv::Mat origDepth, origRGB, steps, neighbor_map, normals, xy;

			origDepth=imgPtrDepth->image.clone();
			origRGB=imgPtrRGB->image.clone();


			KinTo::convertKinectRawToSteps(origDepth,steps);
			KinTo::createRelationNeighbourhoodMap(steps,neighbor_map);
			KinTo::stepMapBlur(steps,neighbor_map,steps);
			KinTo::convertStepsToKinectRaw(steps,imgPtrDepth->image);
			KinTo::blurDepth(imgPtrDepth->image,imgPtrDepth->image);
			KinTo::createXYMap(imgPtrDepth->image,info_msg,xy);
			KinTo::createNormalMap(imgPtrDepth->image,neighbor_map, xy,normals);

			rgb_out.publish(imgPtrRGB->toImageMsg(),info_msg);
			depth_out.publish(imgPtrDepth->toImageMsg(),info_msg);
		}

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "signDetection");
	signDetection ic;
	ros::spin();
	return 0;
}

