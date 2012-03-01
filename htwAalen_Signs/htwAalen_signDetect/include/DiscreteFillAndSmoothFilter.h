#ifndef DISCRETEFILLANDSMOOTHFILTER_H_
#define DISCRETEFILLANDSMOOTHFILTER_H_

#include <set>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <iterator>
#include <math.h>


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


private:

	void filter();
};




#endif /* DISCRETEFILLANDSMOOTHFILTER_H_ */
