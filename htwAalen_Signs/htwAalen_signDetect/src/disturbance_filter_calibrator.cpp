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
#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <set>
#include <vector>
#include <iterator>
#include <fstream>
#include <iostream>

#include "kinectStepLUT.h"
#include "DiscreteFillAndSmoothFilter.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

static const short KinectValues[]=
{
	#include "../data/Kinectvalues"
};

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
	typedef cv::Vec<short, 1> Vec1shrt;
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

	std::vector<cv::Point> statistics;

	cv::Mat stepDisturb;

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

	bool pcl_filter_test;

	bool pcl_stop_output;

	bool pcl_value_print;

	bool pcl_gather_values;

	bool fetchValues;

	bool outputValues;

	bool fetchStepDistubMap;

	bool subtractStepDisturbMap;

	cv::Rect dataframe;
	//Values
	std::set<short> values;

public:
	disturbance_filter_calibrator() :
			nh_("~"), it_out(nh_), image_count(0), store(480, 640, CV_16UC1), disturb(480, 640, CV_16UC1)
	{
		rgb_nh_.reset(new ros::NodeHandle(nh_, "rgb"));
		ros::NodeHandle depth_nh(nh_, "depth_registered");
		rgb_it_.reset(new image_transport::ImageTransport(*rgb_nh_));
		depth_it_.reset(new image_transport::ImageTransport(depth_nh));


		pcl_value_print=false;
		fetchValues = false;
		outputValues = false;


		dataframe.height=0;
		dataframe.width=0;
		dataframe.x=0xFFFF;
		dataframe.y=0xFFFF;

		// Read parameters
		int queue_size;
		nh_.param("queue_size", queue_size, 5);


		//Read kinect values
		for (unsigned int v = 0; v < sizeof(KinectValues)/2; ++v)
		{
			values.insert(KinectValues[v]);
		}





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


		stepDisturb=cv::Mat::zeros(480,640,CV_16UC1);


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

		pcl_stop_output=config.pcl_stop_output;

		pcl_filter_test=config.pcl_filter_test;

		if(config.pcl_value_print)
		{
			pcl_value_print=config.pcl_value_print;
			config.pcl_value_print=false;
		}

		if(config.fetchValues)
		{
			fetchValues=config.fetchValues;
			config.fetchValues=false;
		}

		if(config.outputValues)
		{
			outputValues=config.outputValues;
			config.outputValues=false;
		}

		pcl_gather_values=config.pcl_gather_values;

		if(config.pcl_value_catch)
		{
			fetchStepDistubMap=config.pcl_value_catch;
			config.pcl_value_catch=false;
		}

		subtractStepDisturbMap=config.pcl_subtr_disturb;

	}

	void from16UC1to32FC1(const cv::Mat &src, cv::Mat &dst)
	{
		for (int y = 0; y < 480; y++) //TODO replace with image width and height
		{
			for (int x = 0; x < 640; x++)
			{
				dst.at<Vec1flt>(y, x)[0] = ((float) src.at<Vec1shrt>(y, x)[0]) / 1000.0;
			}
		}
	}

	void from32FC1to16UC1(const cv::Mat &src, cv::Mat &dst)
	{
		for (int y = 0; y < 480; y++) //TODO replace with image width and height
		{
			for (int x = 0; x < 640; x++)
			{
				dst.at<Vec1shrt>(y, x)[0] =
						(unsigned short int) (src.at<Vec1flt>(y, x)[0] * 1000);
			}
		}
	}

	void myFilter1(const cv::Mat &src, cv::Mat &dst)
	{

		if (src.type() == CV_16UC1)
		{
			cv::Mat filter_in = src.clone();



			if(dyn0==6)
			{
				//Subtract disturbance
				for (int y = 0; y < src.rows; y++)
				{
					for (int x = 0; x < src.cols; x++)
					{
						short a=filter_in.at<Vec1shrt>(y, x)[0];
						short b=disturb.at<Vec1shrt>(y, x)[0];
						short c=a-b;

						filter_in.at<Vec1shrt>(y, x)[0]=c;
					}
				}

			}
			cv::Mat filter=filter_in.clone();


			cv::boxFilter(filter, filter, 3, cv::Size(7, 3), cv::Point(-1, -1), 1, 0);
			//cv::GaussianBlur(filter,filter,cv::Size(dyn4,dyn5),dyn6,dyn7);
			cv::medianBlur(filter, filter, 3);

			//Update non zero pixels
			for (int y = 0; y < src.rows; y++)
			{
				for (int x = 0; x < src.cols; x++)
				{
					short realValue = filter_in.at<Vec1shrt>(y, x)[0];
					short filteredValue = filter.at<Vec1shrt>(y, x)[0];
					short maxDifference = pow((float)realValue, 2) / (480000); //Maximal difference from the real value
					if (realValue>0)
					{
						if(abs(realValue - filteredValue) > maxDifference)
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

	}

	void filter_test(const cv::Mat &src, cv::Mat &dst)
	{

		cv::Rect currentdata_ROI=DiscreteFillAndSmoothFilter::roiFinder(src);
		if((currentdata_ROI.x<dataframe.x) && currentdata_ROI.x != -1)
		{
			dataframe.x=currentdata_ROI.x;
		}
		if(currentdata_ROI.y<dataframe.y && currentdata_ROI.y != -1)
		{
			dataframe.y=currentdata_ROI.y;
		}
		if(currentdata_ROI.height>dataframe.height)
		{
			dataframe.height=currentdata_ROI.height;
		}
		if(currentdata_ROI.width>dataframe.width)
		{
			dataframe.width=currentdata_ROI.width;
		}
		ROS_INFO("(%i/%i) %ix%i",dataframe.x, dataframe.y, dataframe.width, dataframe.height);


		cv::Mat orig=src.clone();
		dst=src.clone();
		DiscreteFillAndSmoothFilter::convertKinectRawToSteps(orig,dst);
//		for (int y = 0; y < src.rows; y++)
//		{
//			for (int x = 0; x < src.cols; x++)
//			{
//				if(dst.at<Vec1shrt>(y,x)[0]%((int)dyn0))
//				{
//					dst.at<Vec1shrt>(y,x)[0]-=dst.at<Vec1shrt>(y,x)[0]%4;
//				}
//			}
//		}
		//DiscreteFillAndSmoothFilter::verticalsFinder(dst);


		//DiscreteFillAndSmoothFilter::convertStepsToKinectRaw(dst,dst);
		//myFilter1(dst,dst);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
			     const sensor_msgs::ImageConstPtr& rgb_msg,
			     const sensor_msgs::CameraInfoConstPtr& info_msg)
	{


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


			//Make a copy of the original pictures
			cv::Mat orig_depth=imgPtrDepth->image.clone();
			cv::Mat orig_rgb=imgPtrRGB->image.clone();



//			if(fetchStepDistubMap)
//			{
//				fetchStepDistubMap=false;
//				DiscreteFillAndSmoothFilter::captureDifferenceStepMap(orig_depth,stepDisturb,1000);
//			}



			if(subtractStepDisturbMap)
			{
				DiscreteFillAndSmoothFilter::convertKinectRawToSteps(imgPtrDepth->image,imgPtrDepth->image);
				imgPtrDepth->image-=stepDisturb;
				DiscreteFillAndSmoothFilter::convertStepsToKinectRaw(imgPtrDepth->image,imgPtrDepth->image);
			}

			//Fetch New Unknown Values
			if(fetchValues)
			{
				cv::Rect roi=DiscreteFillAndSmoothFilter::roiFinder(imgPtrRGB->image);

				int each=50;
				cv::Mat testzone=orig_depth(roi);
				int size_x=testzone.cols, size_y=testzone.rows;
				for (int i = 0; i < (size_x*size_y); i++)
				{

					//Forward direction x -
					int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;
					if(!(x_xfw%each) && !(y_xfw%each))
					{
						if(testzone.at<Vec1shrt>(y_xfw,x_xfw)[0]!=0)
						{
							statistics.push_back(cv::Point(advisor_distance,testzone.at<Vec1shrt>(y_xfw,x_xfw)[0]));
							std::cout<<x_xfw<<" "<<y_xfw<<":"<<advisor_distance<<" - "<<testzone.at<Vec1shrt>(y_xfw,x_xfw)[0]<<std::endl;
						}
					}
				}
				std::cout<<std::endl;
				fetchValues=false;
			}

			//Save Values to disk ...
//			if(outputValues)
//			{
//				std::ofstream outputfile;
//				 outputfile.open ("/home/cyborg-x1/kinect_stats.csv");
//				 outputfile << "Laser;Kinect"<<std::endl;
//				 for(std::vector<cv::Point>::iterator it=statistics.begin(); it != statistics.end(); it++)
//				 {
//					 cv::Point current=*it;
//					 outputfile<<current.x<<";"<<current.y<<std::endl;
//				 }
//				 outputfile.close();
//				outputValues=false;
//
//			}


				//Walk through
				int pixelright=0,pixelamount=0;
				for(int y = 0; y < imgPtrDepth->image.rows; y++)
				{
					for(int x = 0; x < imgPtrDepth->image.cols; x++)
					{
						ushort currentDepth=imgPtrDepth->image.at<Vec1shrt>(y,x)[0];


						//Gather new values
						if(pcl_gather_values)
						{
							if(values.insert(imgPtrDepth->image.at<Vec1shrt>(y,x)[0]).second)//Is it new?
							{
								ROS_WARN("NEW VALUE FOUND!: %i",imgPtrDepth->image.at<Vec1shrt>(y,x)[0]);
							}
						}

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

				if(pcl_filter_test && !pcl_stop_output)
				{

					cv::Mat normals, step, neighbors, range, raw;
					image_geometry::PinholeCameraModel model;
					model.fromCameraInfo(info_msg);


					DiscreteFillAndSmoothFilter::RangeFilter(imgPtrDepth->image, range, 0, 4600);

					DiscreteFillAndSmoothFilter::convertKinectRawToSteps(range,step);

					DiscreteFillAndSmoothFilter::gapStepMapGapFiller(step,step,40);

					DiscreteFillAndSmoothFilter::createRelationNeighbourhoodMap(step,neighbors,4);

					DiscreteFillAndSmoothFilter::stepMapBlur(step,neighbors,step);

					DiscreteFillAndSmoothFilter::convertStepsToKinectRaw(step,imgPtrDepth->image);

					DiscreteFillAndSmoothFilter::createNormalMap(imgPtrDepth->image, neighbors, normals, model);


					//Bluring real image
					myFilter1(imgPtrDepth->image,imgPtrDepth->image);

					//cv::rectangle(imgPtrRGB->image,model.rawRoi(),cv::Scalar(255,0,0,0),3,8,0);
					//cv::circle(imgPtrRGB->image,cv::Point(model.cx(),model.cy()),5,cv::Scalar(255,0,0,0),2);

					//cv::circle(imgPtrDepth->image,cv::Point(model.cx(),model.cy()),5,cv::Scalar(0),5);
				}




				if(outputValues)
				{
					DiscreteFillAndSmoothFilter::convertKinectRawToSteps(imgPtrDepth->image,imgPtrDepth->image);
					DiscreteFillAndSmoothFilter::stepMapBlur(imgPtrDepth->image,imgPtrDepth->image,4);
					DiscreteFillAndSmoothFilter::convertStepsToKinectRaw(imgPtrDepth->image,imgPtrDepth->image);

					for(int y = 0; y < imgPtrDepth->image.rows; y++)
					{
						for(int x = 0; x < imgPtrDepth->image.cols; x++)
						{
							if(pcl_highlight_row==y)cout<<(unsigned short)imgPtrDepth->image.at<Vec1shrt>(y,x)[0]<<";"<<(unsigned short)kinect_depth_to_step_LUT[imgPtrDepth->image.at<Vec1shrt>(y,x)[0]]<<endl;
						}
					}
				}

			//Publish image
			if(!pcl_stop_output)
			{
				cv::Mat out=cv::Mat(480,640,CV_32FC1);
				from16UC1to32FC1(imgPtrDepth->image,out);
				imgPtrDepth->image=out;
				imgPtrDepth->encoding="32FC1";


				advisor_rgb_out.publish(imgPtrRGB->toImageMsg(),info_msg);
				advisor_depth_out.publish(imgPtrDepth->toImageMsg(),info_msg);
			}


			if(outputValues)
			{
				pcl_stop_output=true;
				outputValues=false;
			}

			if(pcl_value_print)
			{
				int i=0;
				for(std::set<short>::iterator it=values.begin();it != values.end();it++)
				{
					i++;
					printf("%i, ",*it);
				}
				pcl_value_print=0;
				printf("(Values: %i)\n",i);
			}

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

