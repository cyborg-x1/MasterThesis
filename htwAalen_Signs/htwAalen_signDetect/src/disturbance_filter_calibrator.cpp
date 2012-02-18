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
#include <htwAalen_signDetect/disturbance_filter_calibratorConfig.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>


namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class disturbance_filter_calibrator
{
	//NodeHandle
	ros::NodeHandle nh_;

	//Dynamic reconfigure
	dynamic_reconfigure::Server<htwAalen_signDetect::disturbance_filter_calibratorConfig> reconfServer;
	dynamic_reconfigure::Server<htwAalen_signDetect::disturbance_filter_calibratorConfig>::CallbackType reconfCbType;

	//Testing variables
	double dyn0, dyn1, dyn2, dyn3, dyn4, dyn5, dyn6, dyn7;

	//Publisher for display point cloud
	image_transport::ImageTransport it_out;
	image_transport::CameraPublisher advisor_rgb_out;
	image_transport::CameraPublisher advisor_depth_out;

	//Counter for images
	unsigned int image_count;

	//Types for different mats
	typedef cv::Vec<float, 1> Vec1flt;
	typedef cv::Vec<uchar, 3> Vec3char;
	typedef cv::Vec<ushort, 1> Vec1shrt;
	typedef cv::Vec<uchar, 1> Vec1char;


	//Mat for storage
	cv::Mat store;
	cv::Mat disturb;

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


	//Advisor Variables
	unsigned short advisor_distance;
	int advisor_pcl_overlay;

	int pcl_picture_grid;
	int pcl_picture_grid_space;

    bool pcl_highlight_row_enable;
	int pcl_highlight_row;

	bool pcl_highlight_col_enable;
	int pcl_highlight_col;

	bool pcl_zero_row_enable;
	int pcl_zero_row;

	bool pcl_zero_col_enable;
	int pcl_zero_col;


public:
	disturbance_filter_calibrator() :
			nh_("~"), it_out(nh_), image_count(0), store(480, 640, CV_16UC1), disturb(480, 640, CV_16UC1)
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
				boost::bind(&disturbance_filter_calibrator::imageCb, this, _1, _2, _3));


		//Advertise out camera
		reconfCbType = boost::bind(&disturbance_filter_calibrator::reconfigCb, this, _1, _2);
		reconfServer.setCallback(reconfCbType);

		//Subscribe topics
		sub_depth_.subscribe(*depth_it_, "image_rect", 1);
		sub_rgb_.subscribe(*rgb_it_, "image_rect_color", 1);
		sub_info_.subscribe(*rgb_nh_, "camera_info", 1);

		//Advertise rviz calibration advisor output
		advisor_rgb_out = it_out.advertiseCamera("advise_rgb",1);
		advisor_depth_out = it_out.advertiseCamera("advise_depth",1);


		cv::namedWindow(WINDOW); //TODO Remove

	}

	~disturbance_filter_calibrator()
	{
		cv::destroyWindow(WINDOW); //TODO Remove
	}

	void reconfigCb(htwAalen_signDetect::disturbance_filter_calibratorConfig &config,
			uint32_t level)
	{
		dyn0 = config.double_param0;
		dyn1 = config.double_param1;
		dyn2 = config.double_param2;
		dyn3 = config.double_param3;
		dyn4 = config.double_param4;
		dyn5 = config.double_param5;
		dyn6 = config.double_param6;
		dyn7 = config.double_param7;


		if(config.use_custom_distance)
		{
			advisor_distance=config.custom_distance;
		}
		else
		{
			advisor_distance=config.distance;
		}

		advisor_pcl_overlay=config.pcl_picture;


		pcl_picture_grid=config.pcl_picture_grid;

		pcl_picture_grid_space=config.pcl_picture_grid_space;

	    pcl_highlight_row_enable=config.pcl_highlight_row_enable;
		pcl_highlight_row=config.pcl_highlight_row;

		pcl_highlight_col_enable=config.pcl_highlight_col_enable;
		pcl_highlight_col=config.pcl_highlight_col;

		pcl_zero_row_enable=config.pcl_zero_row_enable;
		pcl_zero_row=config.pcl_zero_row;

		pcl_zero_col_enable=config.pcl_zero_col_enable;
		pcl_zero_col=config.pcl_zero_col;
	}

	void from16UC1to32FC1(const cv::Mat &src, cv::Mat &dst)
	{
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				dst.at<Vec1flt>(y, x)[0] = ((float) src.at<Vec1shrt>(y, x)[0]) / 1000.0;
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

	void RangeFilter(const cv::Mat &src, cv::Mat &dst, short min_range, short max_range, bool ignore_bad_points=true)
	{
		if(src.type() == CV_16UC1)
		{
			dst = src.clone();
			for (int y = 0; y < src.rows; y++)
			{
				for (int x = 0; x < src.cols; x++)
				{
					short value=src.at<Vec1shrt>(y,x)[0];
					if(value<min_range || value>max_range)
					{
						if(!(ignore_bad_points&&(value<10 || value>10000)))
						{
							dst.at<Vec1shrt>(y,x)[0]=0;
						}
					}
				}
			}
		}
		else
		{
			ROS_ERROR("RangeFilter: Wrong depth image type, node supports only CV_16UC1 (Rectified raw!) !\n");
		}
	}

	void DetectSpecialPoints(const cv::Mat &src)
	{
		if(src.type() == CV_16UC1)
		{
			for (int y = 0; y < src.rows; y++)
			{
				for (int x = 0; x < src.cols; x++)
				{
					short value=src.at<Vec1shrt>(y,x)[0];
					if(value<0 || value>10000)
					{
						printf("Start:(%i/%i//%i)",x,y,value);
					}
				}
			}
		}
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
				imgPtrRGB = cv_bridge::toCvCopy(rgb_msg, "bgr8");

			} catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}


			//Make a copy of the original pictures
			cv::Mat orig_depth=imgPtrDepth->image.clone();
			cv::Mat orig_rgb=imgPtrRGB->image.clone();




				//Walk through
				int pixelright=0,pixelamount=0;
				for(int y = 0; y < imgPtrDepth->image.rows; y++)
				{
					for(int x = 0; x < imgPtrDepth->image.cols; x++)
					{
						ushort currentDepth=imgPtrDepth->image.at<Vec1shrt>(y,x)[0];

						//ADVISOR OVERLAY
						if(advisor_pcl_overlay==0)
						{
							if(currentDepth>0)
							{
								imgPtrRGB->image.at<Vec3char>(y,x)[0]=0;
								if(abs(currentDepth-advisor_distance)<5)
								{
									imgPtrRGB->image.at<Vec3char>(y,x)[1]=255;
									imgPtrRGB->image.at<Vec3char>(y,x)[2]=0;
									pixelright++;
								}
								else
								{
									imgPtrRGB->image.at<Vec3char>(y,x)[1]=0;
									imgPtrRGB->image.at<Vec3char>(y,x)[2]=255;
								}
								pixelamount++;
							}
						}

						//Grid and line highlighting/zeroing
						switch(pcl_picture_grid)
						{
							case 0: break;

							case 1:
							{
								if(!(x%pcl_picture_grid_space))
								{
									imgPtrRGB->image.at<Vec3char>(y,x)[0]=255;
								}
								break;
							}
							case 2:
							{
								if(!(y%pcl_picture_grid_space))
								{
									imgPtrRGB->image.at<Vec3char>(y,x)[0]=255;
								}
								break;
							}

							case 3:
							{
								if((!(y%pcl_picture_grid_space)) || (!(x%pcl_picture_grid_space)))
								{
									imgPtrRGB->image.at<Vec3char>(y,x)[0]=255;
								}
								break;
							}
						}

						if(pcl_highlight_row_enable && y == pcl_highlight_row )
						{
							imgPtrRGB->image.at<Vec3char>(y,x)[0]=255;
						}

						if(pcl_highlight_col_enable && x == pcl_highlight_col)
						{
							imgPtrRGB->image.at<Vec3char>(y,x)[0]=255;
						}


						if(pcl_zero_row_enable && y == pcl_zero_row )
						{
							imgPtrDepth->image.at<Vec1shrt>(y,x)[0]=0;
						}

						if(pcl_zero_col_enable && x == pcl_zero_col)
						{
							imgPtrDepth->image.at<Vec1shrt>(y,x)[0]=0;
						}

					}
				}

				//Draw percent of green pixels
				if(advisor_pcl_overlay==0)//Advisor Image
				{
					double percent=((double)pixelright)/((double)pixelamount)*100;
					std::ostringstream percent_ss;
					percent_ss << percent << "%";
					cv::putText(imgPtrRGB->image,percent_ss.str(),cv::Point(100,300),cv::FONT_HERSHEY_COMPLEX,5,CV_RGB(255,255,255));
				}

















			//Publish image
			advisor_rgb_out.publish(imgPtrRGB->toImageMsg(),info_msg);
			advisor_depth_out.publish(imgPtrDepth->toImageMsg(),info_msg);


			//ROS_INFO("Got Image!");
		}
		else
		{
			ROS_ERROR("Wrong encoding of depth image, needs to be 16UC1!");
		}
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "disturbance_filter_calibrator");
	disturbance_filter_calibrator ic;
	ros::spin();
	return 0;
}

