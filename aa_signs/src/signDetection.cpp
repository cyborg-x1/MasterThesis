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
#include <boost/filesystem.hpp>

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
#include <KinectTools/tmpMatch.hpp>
#include <tinyxml.h>
#include "aa_signs/signDetectionConfig.h"
namespace enc = sensor_msgs::image_encodings;


class sign
{
	std::string name;
	cv::Mat img;
public:
	sign(std::string name, cv::Mat &img)
	:name(name)
	,img(img)
	{}
	std::string getName()
	{
		return name;
	}
	const cv::Mat& getImg()
	{
		return img;
	}
};

class signDetection
{
	//NodeHandle
	ros::NodeHandle nh_;

	//Dynamic reconfigure
	dynamic_reconfigure::Server<aa_signs::signDetectionConfig> reconfServer;
	dynamic_reconfigure::Server<aa_signs::signDetectionConfig>::CallbackType reconfCbType;

	//Sign Patterns
	std::vector<sign> signs;

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
	int x_angle_min, x_angle_max,  y_angle_min, y_angle_max, z_angle_min, z_angle_max, blur_depth, blur_angles;
	int x_min, x_max,  y_min, y_max, z_min, z_max;
	int surface_w_min,surface_w_max,surface_h_min,surface_h_max;
	bool show_angles_ok;
	bool new_template_matching;

	std::vector<KinTo::MatchTempProfile> tempProfiles;

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
		std::string settingsfile;

		nh_.param("queue_size", queue_size, 5);
		nh_.param<std::string>("settingsfile", settingsfile, "");
		ROS_INFO("Queue: %i",queue_size);

		if(settingsfile.size())
		{
			ROS_INFO("Settingsfile: %s",settingsfile.c_str());
			TiXmlDocument settings(settingsfile);

			if(settings.LoadFile())
			{
				TiXmlNode *parent, *pChild;

				static int nonames=0;
				//Parsing the XML
				parent=settings.FirstChildElement();


				for ( pChild = parent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
				{
					//Look for Elements
					if(pChild->Type()==TiXmlNode::TINYXML_ELEMENT)
					{
						TiXmlElement* pElement=pChild->ToElement();
						if(pElement->ValueStr()=="sign")//If the element is sign
						{
							TiXmlAttribute* pAttrib=pElement->FirstAttribute();
							bool imgloaded=false;
							printf("\n");
							std::string name;
							std::string file;
							std::string file_imp;
							while (pAttrib) //fetch attributes
							{
								if(pAttrib->NameTStr()=="name") //sign name
								{
									name=pAttrib->Value();
								}
								if(pAttrib->NameTStr()=="file") //file name
								{
									file=pAttrib->Value();
								}
								if(pAttrib->NameTStr()=="file_imp") //file name (important pixels)
								{
									file_imp=pAttrib->Value();
								}

								pAttrib=pAttrib->Next();
							}


							if(!name.size())//if no name tag is specified create a noname
							{
								std::stringstream ss;
								ss<<"NONAME_"<<nonames;
								name=ss.str();
								nonames++;
							}

							if(file.size())
							{
								boost::filesystem::path picturePath(file);
								if(picturePath.is_relative())
								{
									boost::filesystem::path xmlfilepath(settingsfile);
									boost::filesystem::path xmldir=xmlfilepath.parent_path();
									picturePath=xmldir/=picturePath;
								}

								boost::filesystem::path picturePath2(file_imp);
								if(picturePath2.is_relative())
								{
									boost::filesystem::path xmlfilepath(settingsfile);
									boost::filesystem::path xmldir=xmlfilepath.parent_path();
									picturePath2=xmldir/=picturePath2;
								}

								ROS_INFO("File: %s", picturePath.string().c_str());
								cv::Mat img=cv::imread(picturePath.string());
								cv::Mat img_imp=cv::imread(picturePath2.string());
								if(!img.empty())
								{
									if(!img_imp.empty())
									{
										ROS_INFO("Loaded img_imp: %s",file_imp.c_str());
										tempProfiles.push_back(KinTo::MatchTempProfile(img,img_imp,name,0.08,0.2,127,0.2,1,0.86));
									}
									else
									{
										tempProfiles.push_back(KinTo::MatchTempProfile(img,name,0.08,0.2,127,0.2,1,0.86));
										if(file_imp.size())
										{
											ROS_ERROR("Could not load given imp_file: %s, going on without it...",file_imp.c_str());
										}
									}

									signs.push_back(sign(name,img));
									imgloaded=true;
								}
							}
							else
							{
								imgloaded=false;
							}

							if(imgloaded)
							{
								ROS_INFO("New Sign: %s",name.c_str());
							}
							else
							{
								ROS_INFO("Sign: %s, Could not be loaded!",name.c_str());
							}
						}
					}
				}
			}
			else
			{
				ROS_ERROR("Could not load settings file: %s", settingsfile.c_str());
			}
		}
		else
		{
			ROS_ERROR("NO SETTINGS FILE GIVEN!!!");
		}


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
		x_angle_min=config.x_angle_min;
		x_angle_max=config.x_angle_max;
		y_angle_min=config.y_angle_min;
		y_angle_max=config.y_angle_max;
		z_angle_min=config.z_angle_min;
		z_angle_max=config.z_angle_max;


		x_min=config.x_min;
		x_max=config.x_max;
		y_min=config.y_min;
		y_max=config.y_max;
		z_min=config.z_min;
		z_max=config.z_max;

		blur_angles=config.blur_angles;
		blur_depth=config.blur_depth;

		surface_w_min=config.surface_w_min;
		surface_w_max=config.surface_w_max;
		surface_h_min=config.surface_h_min;
		surface_h_max=config.surface_h_max;

		show_angles_ok=config.show_angles_ok;

		new_template_matching=config.new_template_matching;
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


			cv::Mat steps, neighbor_map, normals, xy,angles, angles_ok, filtered_bgr;


			///
			KinTo::convertKinectRawToSteps(imgPtrDepth->image,steps);  ///?
			KinTo::createRelationNeighbourhoodMap(steps,neighbor_map);
			KinTo::crossDepthBlur(imgPtrDepth->image,neighbor_map,imgPtrDepth->image,blur_depth);
			///

			///
			KinTo::createXYMap(imgPtrDepth->image,info_msg,xy);
			KinTo::XYZrangeFilter(imgPtrDepth->image,xy,imgPtrDepth->image,x_min, x_max,  y_min, y_max, z_min, z_max);
			///

			///
			KinTo::createNormalMap(imgPtrDepth->image,neighbor_map, xy,normals);
			KinTo::createAngleMap(normals,angles);
			///

			///
			KinTo::crossAnglesBlur(angles,neighbor_map,angles,blur_angles);
			KinTo::anglesFilter(angles, angles_ok, x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max,true);
			///


			cv::blur(angles_ok,angles_ok,cv::Size(5,5),cv::Point(-1,-1),0);
			cv::threshold(angles_ok,angles_ok,1,255,0);

			//filtered_bgr=imgPtrRGB->image;
			//KinTo::BGRFilter(imgPtrRGB->image,filtered_bgr,0,70,0,70,50,255);
			KinTo::RedFilter(imgPtrRGB->image,filtered_bgr);


			std::vector<cv::Rect> rois;
			KinTo::SurfaceExtractor(angles_ok,neighbor_map,rois,surface_w_min,surface_h_min,surface_w_max,surface_h_max);


			//Look through the surfaces...
			for(std::vector<cv::Rect>::iterator it=rois.begin();it!=rois.end();it++)
			{
				if(new_template_matching)//proportion enhanced template matching
				{
					cv::Mat roi_mat=imgPtrRGB->image(*it);
					KinTo::proportionEnhancedTemplateMatching(tempProfiles,roi_mat,100);
					int i=0;
					KinTo::Match bestMatch; //Best match for ROI
					for(std::vector<KinTo::MatchTempProfile>::iterator it_prof=tempProfiles.begin();it_prof!=tempProfiles.end();it_prof++)
					{
						KinTo::MatchTempProfile prof=(*it_prof);
						const std::vector<KinTo::Match>& match_vec=prof.getMatches();

						cv::Point last_center(-1,-1);
						i++;

						bestMatch.congruence=0;

						for(std::vector<KinTo::Match>::const_reverse_iterator it_matches=match_vec.rbegin()
								;it_matches!=match_vec.rend();it_matches++)
						{
							it_matches->printMatch();




							if(it_matches->congruence>bestMatch.congruence)
							{
								bestMatch=*it_matches;
							}
						}
						it_prof->clearMatches();
					}
					std::string str;
					str+=bestMatch.name[0];
					cv::putText(roi_mat,str,bestMatch.center,CV_FONT_NORMAL,1,cv::Scalar(0,0,255));

				}
				else //with circles
				{
					//Circledetection taken from http://opencv.willowgarage.com/documentation/cpp/imgproc_feature_detection.html:
					//and modified
					cv::Mat current_surface=filtered_bgr(*it),gray;
				    cv::cvtColor(current_surface, gray, CV_BGR2GRAY);
				    // smooth it, otherwise a lot of false circles may be detected
				    cv::GaussianBlur( gray, gray, cv::Size(5, 5), 2, 2 );
				    std::vector<cv::Vec3f> circles;

				    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/4, 100, 50 );
				    float lowest_global;
				    //circles
				    for( size_t i = 0; i < circles.size(); i++ )
				    {
				    	bool found=false;
					    lowest_global=FLT_MAX;
					    std::vector<sign>::iterator found_sign;
				    	 cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				    	 int radius = cvRound(circles[i][2]);

				    	 if(center.x-radius>0 &&
				    	    center.y-radius>0 &&
				    	    center.x+radius<(*it).width &&
				    	    center.y+radius<(*it).height && center.x>0 && center.y>0
				    	    )
				    	 {

							 //Create a new ROI for the current circle
							 int start_x=center.x-radius-10;
							 int start_y=center.y-radius-10;
							 int end_x=center.x+radius+10;
							 int end_y=center.y+radius+10;

							 if(end_x>=(*it).width)end_x=(*it).width-1;
							 if(end_y>=(*it).height)end_y=(*it).height-1;

							 if(start_x<0)start_x=0;
							 if(start_y<0)start_y=0;

							 //ROI
							 cv::Mat circleplace=imgPtrRGB->image(*it)(cv::Rect(start_x,start_y,end_x-start_x,end_y-start_y));

							 for(std::vector<sign>::iterator it2=signs.begin();it2!=signs.end();it2++)
							 {
								 cv::Mat sign_image,sign_space_image;
								 //resize the template to the circle size
								 cv::resize((*it2).getImg(),sign_image,cv::Size(radius+5,radius+5),0,0,cv::INTER_AREA);
								 cv::cvtColor(sign_image, sign_image, CV_BGR2GRAY);
								 cv::cvtColor(circleplace, sign_space_image, CV_BGR2GRAY);



								 //Output template matching
								 cv::Mat result;
								 cv::matchTemplate(sign_space_image,sign_image,result,CV_TM_SQDIFF_NORMED);

								 float lowest_local=result.at<float>(0,0);

								 //Calculate interesting pixels
								 int rows=result.rows;//-sign_image.rows+1;
								 int cols=result.cols;//-sign_image.cols+1;


								 for (int i = 0; i < (rows*cols); i++)
								 {
									int y=i/rows;
									int x=i-y*cols;

									//ROS_INFO("%f",result.at<float>(y,x));
									if(result.at<float>(y,x)<lowest_local)
									{
										lowest_local=result.at<float>(y,x);
									}
								 }
								 if(lowest_local<lowest_global && lowest_local<0.15)
								 {
									 found=true;
									 found_sign=it2;
									 lowest_global=lowest_local;
									 ROS_INFO("LOW: %f",lowest_global);
								 }
							 }

					    	 if(found)
					    	 {

					    		cv::Mat blub=imgPtrRGB->image(*it);
					    		ROS_INFO("Found sign: %s",found_sign->getName().c_str());
					    		if(found_sign->getName()=="left")
					    		{
					    			cv::circle( blub, center, radius, cv::Scalar(0,255,0), 3, 8, 0 );
					    		}
					    		if(found_sign->getName()=="right")
								{
									cv::circle( blub, center, radius, cv::Scalar(255,0,0), 3, 8, 0 );
								}
					    		if(found_sign->getName()=="stop")
								{
									cv::circle( blub, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
								}
					    	 }

				    	 }
				    }
				}

			}


			for(std::vector<cv::Rect>::iterator it=rois.begin();it!=rois.end();it++)
			{
				if(show_angles_ok)
				{
					cv::rectangle(angles_ok,*it,cv::Scalar(100),1,0,0);
				}
				else
				{
					cv::rectangle(imgPtrRGB->image,*it,cv::Scalar(100),1,0,0);
				}
			}



			if(show_angles_ok)
			{
				imgPtrRGB->image=angles_ok;
				imgPtrRGB->encoding="mono8";
			}

			//imgPtrRGB->image=filtered_bgr;

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
