#ifndef DISCRETEFILLANDSMOOTHFILTER_H_
#define DISCRETEFILLANDSMOOTHFILTER_H_

#include <set>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "frequency.hpp"
#include <iostream>
#include <vector>
#include <iterator>
#include <math.h>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


class Blob : public cv::Rect
{
private:
	int _pixels;

public:
	Blob(int x, int y,int width, int height, int pixelsize)
	{
		cv::Rect(x,y,width,height);
	}

	int getPixelSize()
	{
		return _pixels;
	}

	void setPixelSize(int pixelsize)
	{
		_pixels=pixelsize;
	}

	double getBlobComplexity()
	{
		return (double)(cv::Rect::width*cv::Rect::height)/(double)(_pixels);
	}
};



class DiscreteFillAndSmoothFilter
{

	typedef enum
	{

	}states_seek_t;

	typedef enum
	{
		Dir_Horizontal,
		Dir_Vertical,
	}scan_direction_t;


	cv::Mat cache;
	cv::Mat orig;
	cv::Mat dst;
	scan_direction_t dir;

	short stored_value;

	typedef cv::Vec<uchar, 3> Vec3char;

public:

	/**
	 * Lookup table for converting available kinect depths into numbers 0-824
	 */
	static const short kinect_depth_to_step_LUT[];

	/**
	 * Lookup table for converting step numbers back to depths
	 */
	static const short kinect_step_to_depth_LUT[];

	/**
	 * Number of available steps
	 */
	static const short kinect_steps_count;

	/**
	 * Maximum depth value of the kinect
	 */
	static const short kinect_depths_count;

	/**
	 * Type for accessing kinect raw images and step images
	 */
	typedef cv::Vec<short, 1> Vec1shrt;

	DiscreteFillAndSmoothFilter(){};

	DiscreteFillAndSmoothFilter(const cv::Mat &src, cv::Mat &dst, scan_direction_t dir);

	virtual ~DiscreteFillAndSmoothFilter();

	/**
	 * This function converts a kinect depth image into steps
	 *  @param [in] src The source image (CV_16UC1)
	 *  @param [out] dst The destination image(CV_16UC1)
	 */
	static void convertKinectRawToSteps(const cv::Mat &src, cv::Mat &dst)
	{
		if(src.type() == CV_16UC1 && CV_16UC1 == dst.type())
		{
			int size_x=src.cols, size_y=src.rows;

			for (int i = 0; i < (size_x*size_y); i++)
			{
				//Forward direction x -
				int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;
				short current=src.at<Vec1shrt>(y_xfw,x_xfw)[0];
				if(current>=0 && current < kinect_depths_count)
				{
					if(kinect_depth_to_step_LUT[current]>=0)
					{
						dst.at<Vec1shrt>(y_xfw,x_xfw)[0]=kinect_depth_to_step_LUT[current];
					}
					else
					{
						std::cerr<<"convertKinectRawToSteps: Unregistered Depth Value found! You must use unedited KINECT RAW data for this! "
								 <<"Value is: "<<current<<std::endl;
					}
				}
				else
				{
					std::cerr<<"convertKinectRawToSteps: Unregistered Depth Value found (too big)! You must use unedited KINECT RAW data for this! "
							 <<"Value is: "<<current<<std::endl;
				}
			}
		}
		else
		{
			std::cerr<<"convertKinectRawToSteps: Wrong matrix type! You must use unedited KINECT RAW (CV_16UC1) data for this! "<<std::endl;
		}
	}

	/**
	 * This function converts a step image into kinect depth image
	 *  @param [in] src The source image (CV_16UC1)
	 *  @param [out] dst The destination image(CV_16UC1)
	 */
	static void convertStepsToKinectRaw(const cv::Mat &src, cv::Mat &dst)
	{
		if(src.type() == CV_16UC1 && CV_16UC1 == dst.type())
		{
			int size_x=src.cols, size_y=src.rows;

			for (int i = 0; i < (size_x*size_y); i++)
			{
				//Forward direction x -
				int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;
				short current=src.at<Vec1shrt>(y_xfw,x_xfw)[0];
				if(current>=0 && current < kinect_steps_count)
				{
					dst.at<Vec1shrt>(y_xfw,x_xfw)[0]=kinect_step_to_depth_LUT[current];
				}
				else
				{
					std::cerr<<"convertStepsToKinectRaw: Unregistered Step Value found (too big)! You must use a KINECT STEP image for this! "
							 <<"Value is: "<<current<<std::endl;
				}
			}
		}
		else
		{
			std::cerr<<"convertStepsToKinectRaw: Wrong matrix type! You must use unedited KINECT RAW (CV_16UC1) data for this! "<<std::endl;
		}
	}

	/**
	 * This function searches for the interesting part of an image.
	 * It creates a cv::Rect which includes all non-zero pixels.
	 * @param [in] src The Mat to create the ROI for
	 * @return ROI
	 */
	static cv::Rect roiFinder(const cv::Mat &src)
	{
		int size_x=src.cols, size_y=src.rows;
		int roi_xs=-1,roi_ys=-1,roi_xe=-1,roi_ye=-1;

		//The first two rows and cols most unlikely contain useful data,
		//but some strange readings. So we need can skip them.



		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Forward direction x -
			int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;

			//Forward direction y
			int x_yfw=i/size_y, y_yfw=i-x_yfw*size_y;

			//Backward direction x
			int x_xbw=size_x-x_xfw-1, y_xbw=size_y-y_xfw-1;

			//Backward direction y
			int x_ybw=size_x-x_yfw-1, y_ybw=size_y-y_yfw-1;

			//ROI Seek
			if(src.at<Vec1shrt>(y_xfw,x_xfw)[0]>0 && roi_ys<0)
			{
				roi_ys=y_xfw;
			}
			if(src.at<Vec1shrt>(y_yfw,x_yfw)[0]>0 && roi_xs<0)
			{
				roi_xs=x_yfw;
			}
			if(src.at<Vec1shrt>(y_xbw,x_xbw)[0]>0 && roi_ye<0)
			{
				roi_ye=y_xbw;
			}
			if(src.at<Vec1shrt>(y_ybw,x_ybw)[0]>0 && roi_xe<0)
			{
				roi_xe=x_ybw;
			}

			if(roi_xs>=0 && roi_ys>=0 && roi_xe>=0 && roi_ye>=0) break;
		}

		return cv::Rect(roi_xs,roi_ys,roi_xe-roi_xs,roi_ye-roi_ys);
	}

	/**
	 * This function reduces a depth image to a given range
	 * @param[in] src Source image
	 * @param[out]dst Destination image
	 * @param[in] min_range The minimal distance allowed
	 * @param[in] max_range The maximal distance allowed
	 */
	static void RangeFilter(const cv::Mat &src, cv::Mat &dst, short min_range, short max_range)
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
							dst.at<Vec1shrt>(y,x)[0]=0;
					}
				}
			}
		}
		else
		{
			std::cerr<<"RangeFilter: Wrong depth image type, node supports only CV_16UC1 (Rectified raw!) !"<<std::endl;
		}
	}


	/**
	 * This creates a
	 */
	static void captureDifferenceStepMap(const cv::Mat &src, cv::Mat &dst, short distance)
	{
		if(src.type() == CV_16UC1 && CV_16UC1 == dst.type())
		{

			DiscreteFillAndSmoothFilter::convertKinectRawToSteps(src,dst);
			dst=dst-kinect_depth_to_step_LUT[distance];
		}
		else
		{
			std::cerr<<"Only CV_16UC1 images are supported for in- and output"<<std::endl;
		}
	}

	static void hardEtchFinder(cv::Mat &src, cv::Mat &display, unsigned short threshold)
	{
		int size_x=src.cols, size_y=src.rows;

		display=cv::Mat::zeros(480,640,CV_8UC3);

		   using namespace boost::accumulators;
		   accumulator_set<double, stats<
		       tag::median,
		       tag::mean

		   > > stats;

		   short last_hdiff=0,last_vdiff=0;
										/*-1 Coll*/
		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Forward direction x -
			int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;

			//Forward direction y
			int x_yfw=i/size_y, y_yfw=i-x_yfw*size_y;


			int len=20;
			if(y_yfw<(size_y-len))
			{
				short left=src.at<Vec1shrt>(y_yfw,x_yfw)[0];
				short right=src.at<Vec1shrt>(y_yfw+len,x_yfw)[0];

				if(left != 0 && right != 0)
				{
					short diff=abs(left-right);
					if(y_yfw!=0)
					{
						display.at<Vec3char>(y_yfw+len/2,x_yfw)[0]=(abs(diff-last_vdiff)>threshold)?diff*20:0;
						display.at<Vec3char>(y_yfw+len/2,x_yfw)[1]=(abs(diff-last_vdiff)>threshold)?diff*20:0;
						display.at<Vec3char>(y_yfw+len/2,x_yfw)[2]=(abs(diff-last_vdiff)>threshold)?diff*20:0;
					}
					last_vdiff=diff;
				}
			}




			if(x_xfw<(size_x-len))
			{
				short left=src.at<Vec1shrt>(y_xfw,x_xfw)[0];
				short right=src.at<Vec1shrt>(y_xfw,x_xfw+len)[0];
				short diff=abs(left-right);

				if(left != 0 && right != 0)
				{
					if(x_xfw!=0)
					{
						display.at<Vec3char>(y_xfw,x_xfw+len/2)[0]+=(abs(diff-last_hdiff)>threshold)?diff*20:0;
						display.at<Vec3char>(y_xfw,x_xfw+len/2)[1]+=(abs(diff-last_hdiff)>threshold)?diff*20:0;
						display.at<Vec3char>(y_xfw,x_xfw+len/2)[2]+=(abs(diff-last_hdiff)>threshold)?diff*20:0;
					}
					last_hdiff=diff;
				}
			}

			continue;
///////////////////////////////////////
			//HardEtches
			if(y_yfw<(size_y-1))
			{
				short left=src.at<Vec1shrt>(y_yfw,x_yfw)[0];
				short right=src.at<Vec1shrt>(y_yfw+1,x_yfw)[0];

				unsigned short diff=abs(left-right);

				if(diff>threshold)
				{
					display.at<Vec3char>(y_yfw,x_yfw)[2]=255;
					display.at<Vec3char>(y_yfw,x_yfw+1)[2]=255;
					display.at<Vec3char>(y_yfw,x_yfw)[1]=0;
					display.at<Vec3char>(y_yfw,x_yfw+1)[1]=0;
					display.at<Vec3char>(y_yfw,x_yfw)[0]=0;
					display.at<Vec3char>(y_yfw,x_yfw+1)[0]=0;
				}


			}

			if(x_xfw<(size_x-1))
			{
				short up=src.at<Vec1shrt>(y_xfw,x_xfw)[0];
				short down=src.at<Vec1shrt>(y_xfw,x_xfw+1)[0];

				unsigned short diff=abs(up-down);

				if(diff>threshold)
				{
					display.at<Vec3char>(y_xfw,x_xfw)[2]=255;
					display.at<Vec3char>(y_xfw,x_xfw+1)[2]=255;
					display.at<Vec3char>(y_xfw,x_xfw)[1]=0;
					display.at<Vec3char>(y_xfw,x_xfw+1)[1]=0;
					display.at<Vec3char>(y_xfw,x_xfw)[0]=0;
					display.at<Vec3char>(y_xfw,x_xfw+1)[0]=0;
				}
			}

		}
	}

	/**
	 * Blur Filter for a step map
	 * @param[in] src input depth image (must be a step map!)
	 * @param[in] dst output depth image (step map)
	 * @param[in] threshold This is the difference the values are allowed to have from the current pixel,
	 *                       to be used for creating the average of surrounding pixels for the new value of the current pixel.
	 */
	static void stepMapBlur(const cv::Mat &src, cv::Mat &dst, unsigned short threshold=4)
	{

		dst=src.clone();

		int size_x=src.cols, size_y=src.rows;


		int th=threshold;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Forward direction x -
			int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;

			int x=x_xfw;
			int y=y_xfw;

			int curValue=src.at<Vec1shrt>(y,x)[0];


			bool _IsNotTopRow=(y>0);
			bool _IsNotLeftCol=(x>0);
			bool _IsNotBottomRow=(y<size_y);
			bool _IsNotRightCol=(x<size_x);

			short avg=curValue;
			short lS=0;
			short cnt=1;
			if(curValue>0)
			{
				//Top Row
				if(_IsNotTopRow)
				{
					//Left Top Cell
					if(_IsNotLeftCol)
					{
						int C_TL=dst.at<Vec1shrt>(y_xfw-1,x_xfw-1)[0];
						if(C_TL>0) if(abs(C_TL-curValue)<th)
						{
							if(abs(curValue-C_TL)==1)lS++;
							avg+=C_TL;
							cnt++;
						}
					}

					//Middle Top Cell
					int C_TM=dst.at<Vec1shrt>(y_xfw-1,x_xfw)[0];
					if(C_TM>0) if(abs(C_TM-curValue)<th)
					{
						avg+=C_TM;
						cnt++;
					}

					//Right Top Cell
					if(_IsNotRightCol)
					{
						int C_TR=dst.at<Vec1shrt>(y_xfw-1,x_xfw+1)[0];
						if(C_TR>0) if(abs(C_TR-curValue)<th)
						{
							avg+=C_TR;
							cnt++;
						}
					}

				}

				//Middle Row

				//Left Middle Cell
				if(_IsNotLeftCol)
				{
					int C_ML=dst.at<Vec1shrt>(y_xfw,x_xfw-1)[0];
					if(C_ML>0) if(abs(C_ML-curValue)<th)
					{
						if(abs(curValue-C_ML)==1)lS++;
						avg+=C_ML;
						cnt++;
					}
				}

				//Right Middle Cell
				if(_IsNotRightCol)
				{
					int C_MR=dst.at<Vec1shrt>(y_xfw,x_xfw+1)[0];
					if(C_MR>0) if(abs(C_MR-curValue)<th)
					{
						avg+=C_MR;
						cnt++;
					}
				}

				//Bottom Row
				if(_IsNotBottomRow)
				{
					//Left Bottom Cell
					if(_IsNotLeftCol)
					{
						int C_BL=dst.at<Vec1shrt>(y_xfw+1,x_xfw-1)[0];
						if(C_BL>0) if(abs(C_BL-curValue)<th)
						{
							if(abs(curValue-C_BL)==1)lS++;
							avg+=C_BL;
							cnt++;
						}
					}

					//Middle Bottom Cell
					int C_BM=dst.at<Vec1shrt>(y_xfw+1,x_xfw)[0];
					if(C_BM>0) if(abs(C_BM-curValue)<th)
					{
						avg+=C_BM;
						cnt++;
					}

					//Right Bottom Cell
					if(_IsNotRightCol)
					{
						int C_BR=dst.at<Vec1shrt>(y_xfw+1,x_xfw+1)[0];
						if(C_BR>0) if(abs(C_BR-curValue)<th)
						{
							avg+=C_BR;
							cnt++;
						}
					}

				}



			}

			int result=avg/cnt;
			dst.at<Vec1shrt>(y,x)[0]=result;

		}
	}

	/**
	 *
	 */
	static void stepMapFlatten(const cv::Mat &src, cv::Mat &dst,  unsigned short threshold=4, unsigned short threshold2=8)
	{
		dst=src.clone();

		int size_x=src.cols, size_y=src.rows;

		int storeVal=0;
		int overwriteVal=0;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Forward direction x -
			int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;

			int x=x_xfw;
			int y=y_xfw;


			if(!x_xfw)
			{
				storeVal=0;
			}

			int curValue=src.at<Vec1shrt>(y,x)[0];
			int diff=abs(curValue-storeVal);
			int diff2=abs(curValue-overwriteVal);

			if(!storeVal && curValue)
			{
				storeVal=curValue;
				overwriteVal=curValue;
			}
			else if(curValue)
			{
				if(diff<threshold && diff2<threshold2)
				{
					storeVal=curValue;
					dst.at<Vec1shrt>(y,x)[0]=overwriteVal;
				}
				else
				{
					storeVal=curValue;
					overwriteVal=curValue;
				}
			}
		}



		for (int i = 0; i < (size_x*size_y); i++)
		{

			//Forward direction y
			int x_yfw=i/size_y, y_yfw=i-x_yfw*size_y;
			int x=x_yfw;
			int y=y_yfw;


			if(!y_yfw)
			{
				storeVal=0;
			}

			int curValue=src.at<Vec1shrt>(y,x)[0];
			int diff=abs(curValue-storeVal);
			int diff2=abs(curValue-overwriteVal);

			if(!storeVal && curValue)
			{
				storeVal=curValue;
				overwriteVal=curValue;
			}
			else if(curValue)
			{
				if(diff<threshold && diff2<threshold2)
				{
					storeVal=curValue;
					dst.at<Vec1shrt>(y,x)[0]=overwriteVal;
				}
				else
				{
					storeVal=curValue;
					overwriteVal=curValue;
				}
			}
		}

	}




	static std::vector<cv::Rect> blobStepDetector(const cv::Mat &src, cv::Mat &out, unsigned short threshold=4)
	{

	}

private:

	void filter();
};




#endif /* DISCRETEFILLANDSMOOTHFILTER_H_ */
