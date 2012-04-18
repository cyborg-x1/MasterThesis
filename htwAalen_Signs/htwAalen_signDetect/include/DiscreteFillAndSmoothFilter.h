#ifndef DISCRETEFILLANDSMOOTHFILTER_H_
#define DISCRETEFILLANDSMOOTHFILTER_H_

#include <set>
#include <iomanip>
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
#include <image_geometry/pinhole_camera_model.h>

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


	typedef enum
	{
		fromNAN,
		toNAN,
		flat,
		toDepth,
		fromDepth
	}depth_direction;


	cv::Mat cache;
	cv::Mat orig;
	cv::Mat dst;
	scan_direction_t dir;

	short stored_value;

	typedef cv::Vec<uchar, 3> Vec3char;
	typedef cv::Vec<uchar, 2> Vec2char;
	typedef cv::Vec<ushort, 2> Vec2short;

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
		if(src.type() == CV_16UC1)
		{
			dst=src.clone();

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
	 * This function fills holes in the depth map, but only if the difference
	 * between the pixel to the pixel on the end of the gap is
	 * smaller or equal to the amount o pixels between them.
	 */
	static void gapStepMapGapFiller(const cv::Mat &src, cv::Mat &dst, uchar max_size)
	{

		int size_x=src.cols, size_y=src.rows;

		//Copy src into dst ...
		dst=src.clone();

		//These will contain the the length and the value of the pixel on the end of each gap (for the filler)
		//First short value: Value of the pixel at the end of the gap
		//Second: Pixels to be filled
		cv::Mat h_mark=cv::Mat::zeros(size_y,size_x,CV_16UC2); //horizontal
		cv::Mat v_mark=cv::Mat::zeros(size_y,size_x,CV_16UC2); //vertical


		//These variables will be true after the first pixel in each row/col
		bool h_start=0; //hor
		bool v_start=0; //vert

		//These variables will store the coordinates for the begin of the current gap (last non NAN pixel)
		int h_gap_start=-1;
		int v_gap_start=-1;

		//These variables are true if we are inside a gap
		bool h_inside_gap=false;
		bool v_inside_gap=false;

		//These variables are the counters for the gap length
		int h_gap_len=0;
		int v_gap_len=0;

		//Locate the gaps
		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Scanning horizontal...
			int y_H=i/size_x, x_H=i-y_H*size_x;

			//Scanning vertical
			int x_V=i/size_y, y_V=i-x_V*size_y;

			//Reset the start condition at the begin of each col/row
			if(x_H==0)
			{
				h_start=false;
				h_gap_len=0;
				h_inside_gap=false;
			}



			//Grab the current values of the source image
			if(x_H < size_x-1) //Horizontal
			{
				//Found first pixel?
				if(!h_start && src.at<Vec1shrt>(y_H,x_H+1)[0]) h_start=true;

				if(h_start)
				{
					short next=src.at<Vec1shrt>(y_H,x_H+1)[0];

					if(!next && !h_inside_gap)//Next pixel is NAN
					{
						h_inside_gap=true;
						h_gap_start=x_H;
						h_gap_len=0;
					}
					else if(h_inside_gap)
					{
						h_gap_len++; //increase count
						if(next!=0)//next pixel is not NAN
						{
							h_mark.at<Vec2short>(y_H,h_gap_start)[0]=next; 		//End Value
						    h_mark.at<Vec2short>(y_H,h_gap_start)[1]=h_gap_len; //Pixels to be filled
						    h_gap_len=0; //Clear the gap length
						    h_inside_gap=false; //Back to outside gap
						}
					}
				}
			}



			///////////
			///Vertical
			///////////


			//Reset the start condition at the begin of each col/row
			if(y_V==0)
			{
				v_start=false;
				v_gap_len=0;
				v_inside_gap=false;
			}

			//Grab the current values of the source image
			if(y_V < size_y-1) //Horizontal
			{
				//Found first pixel?
				if(!v_start && src.at<Vec1shrt>(y_V+1,x_V)[0]) v_start=true;

				if(v_start)
				{
					short next=src.at<Vec1shrt>(y_V+1,x_V)[0];

					if(!next && !v_inside_gap)//Next pixel is NAN
					{
						v_inside_gap=true;
						v_gap_start=y_V;
						v_gap_len=0;
					}
					else if(v_inside_gap)
					{
						v_gap_len++; //increase count
						if(next!=0)//next pixel is not NAN
						{
							v_mark.at<Vec2short>(v_gap_start,x_V)[0]=next; 		//End Value
						    v_mark.at<Vec2short>(v_gap_start,x_V)[1]=v_gap_len; //Pixels to be filled
						    v_gap_len=0; //Clear the gap length
						    v_inside_gap=false; //Back to outside gap
						}
					}
				}
			}

		}


		///////////////
		//Fill the gaps
		///////////////

		h_inside_gap=false;
		v_inside_gap=false;


		//These variables keep the values for the current gap extracted from the gap location map
		short h_end_val=0;
		short h_start_val=0;
		short v_end_val=0;
	    short v_start_val=0;

	    //These variables store the difference between start and endpixel
	    short h_gap_diff=0;
	    short v_gap_diff=0;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Scanning horizontal...
			int y_H=i/size_x, x_H=i-y_H*size_x;

			//Scanning vertical
			int x_V=i/size_y, y_V=i-x_V*size_y;







			//HORIZONTAL
			if(!h_inside_gap && h_mark.at<Vec2short>(y_H,x_H)[0])
			{
				//Getting all values of the current positon
				h_end_val = h_mark.at<Vec2short>(y_H,x_H)[0];
				h_gap_len = h_mark.at<Vec2short>(y_H,x_H)[1];
				h_start_val = src.at<Vec1shrt>(y_H,x_H)[0];
				h_gap_diff = h_end_val-h_start_val;
				h_gap_start=x_H;

				//Do we fill the gap?
				//Difference between start and and smaller then gap size?
				//Gap smaller than max_size parameter?
				if(h_gap_len>abs(h_gap_diff)&&  abs(h_gap_diff) < 10 && h_gap_len<max_size)
				h_inside_gap = true;
			}
			else if(h_inside_gap)
			{
				//Create a gradient according to the current position
											//DifferenceStartEnd*CurrentPos/GapLength+StartValue
				int h_fill_pos=x_H-h_gap_start;
				int value = h_gap_diff*(h_fill_pos)/h_gap_len+h_start_val;
				dst.at<Vec1shrt>(y_H,x_H)[0]=value;
				if(h_fill_pos>(h_gap_len-1))h_inside_gap=false;
			}

			//VERTICAL
			if(!v_inside_gap && v_mark.at<Vec2short>(y_V,x_V)[0])
			{
				//Getting all values of the current positon
				v_end_val = v_mark.at<Vec2short>(y_V,x_V)[0];
				v_gap_len = v_mark.at<Vec2short>(y_V,x_V)[1];
				v_start_val = src.at<Vec1shrt>(y_V,x_V)[0];
				v_gap_diff = v_end_val-v_start_val;
				v_gap_start=y_V;

				//Do we fill the gap?
				//Difference between start and and smaller then gap size?
				//Gap smaller than max_size parameter?
				if(v_gap_len>abs(v_gap_diff)&&  abs(v_gap_diff) < 10 && v_gap_len<max_size)
				v_inside_gap = true;
			}
			else if(v_inside_gap)
			{
				//Create a gradient according to the current position
											//DifferenceStartEnd*CurrentPos/GapLength+StartValue
				int v_fill_pos=y_V-v_gap_start;
				int value = v_gap_diff*(v_fill_pos)/v_gap_len+v_start_val;
				dst.at<Vec1shrt>(y_V,x_V)[0]=value;
				if(v_fill_pos>(v_gap_len-1))v_inside_gap=false;
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
						if(C_TL>0) if(abs(C_TL-curValue)<threshold)
						{
							avg+=C_TL;
							cnt++;
						}
					}

					//Middle Top Cell
					int C_TM=dst.at<Vec1shrt>(y_xfw-1,x_xfw)[0];
					if(C_TM>0) if(abs(C_TM-curValue)<threshold)
					{
						avg+=C_TM;
						cnt++;
					}

					//Right Top Cell
					if(_IsNotRightCol)
					{
						int C_TR=dst.at<Vec1shrt>(y_xfw-1,x_xfw+1)[0];
						if(C_TR>0) if(abs(C_TR-curValue)<threshold)
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
					if(C_ML>0) if(abs(C_ML-curValue)<threshold)
					{
						avg+=C_ML;
						cnt++;
					}
				}

				//Right Middle Cell
				if(_IsNotRightCol)
				{
					int C_MR=dst.at<Vec1shrt>(y_xfw,x_xfw+1)[0];
					if(C_MR>0) if(abs(C_MR-curValue)<threshold)
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
						if(C_BL>0) if(abs(C_BL-curValue)<threshold)
						{
							avg+=C_BL;
							cnt++;
						}
					}

					//Middle Bottom Cell
					int C_BM=dst.at<Vec1shrt>(y_xfw+1,x_xfw)[0];
					if(C_BM>0) if(abs(C_BM-curValue)<threshold)
					{
						avg+=C_BM;
						cnt++;
					}

					//Right Bottom Cell
					if(_IsNotRightCol)
					{
						int C_BR=dst.at<Vec1shrt>(y_xfw+1,x_xfw+1)[0];
						if(C_BR>0) if(abs(C_BR-curValue)<threshold)
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
	 * Blur Filter for a step map for a already available neighbormap
	 * @param[in] src input depth image (must be a step map!)
	 * @param[in] dst output depth image (step map)
	 * @param[in] neighbors The neigbormap
	 */
	static void stepMapBlur(const cv::Mat &src, cv::Mat &neigbors ,cv::Mat &dst)
	{

		dst = src.clone();
		int size_x=src.cols, size_y=src.rows;
		int x,y,avg;
		uchar nb;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;
			nb=neigbors.at<Vec2char>(y,x)[0];
			avg=dst.at<Vec1shrt>(y,x)[0];

			if(nb>0)
			{
				if(nb&0x01)
				{
					avg+=dst.at<Vec1shrt>(y-1,x-1)[0];
				}
				if(nb&0x02)
				{
					avg+=dst.at<Vec1shrt>(y-1,x)[0];
				}
				if(nb&0x04)
				{
					avg+=dst.at<Vec1shrt>(y-1,x+1)[0];
				}
				if(nb&0x08)
				{
					avg+=dst.at<Vec1shrt>(y,x+1)[0];
				}
				if(nb&0x10)
				{
					avg+=dst.at<Vec1shrt>(y+1,x+1)[0];
				}
				if(nb&0x20)
				{
					avg+=dst.at<Vec1shrt>(y+1,x)[0];
				}
				if(nb&0x40)
				{
					avg+=dst.at<Vec1shrt>(y+1,x-1)[0];
				}
				if(nb&0x80)
				{
					avg+=dst.at<Vec1shrt>(y,x-1)[0];
				}


				//Create average
				avg/=neigbors.at<Vec2char>(y,x)[1]+1;

				//Save back
				dst.at<Vec1shrt>(y,x)[0]=avg;

			}
		}
	}


	/**
	 * This function creates a neighborhood map of the step map
	 * In the first value of each pixel there will be a 8 bit value
	 * indicating which pixel is a close neighbor and belongs to the
	 * same object as the current pixel.
	 *
	 * The are for the following pixels: <br/>
	 * <table>
	 * 	<tr>
	 * 		<td>0</td><td>1</td><td>2</td>
	 * 	</tr>
	 * <tr>
	 * 		<td>7</td><td>X</td><td>3</td>
	 * 	</tr>
	 * 	<tr>
	 * 		<td>6</td><td>5</td><td>4</td>
	 * 	</tr>
	 * </table>
	 *
	 * <br/>
	 *
	 * The second value is the amount of neighbors belonging to the same object as the current pixel.
	 *
	 * @param src The source image
	 * @param map_out The neighborhood map
	 * @param threshold The biggest difference a pixel from the current can have to be a close neighbor.
	 *
	 */
	static void createRelationNeighbourhoodMap(const cv::Mat &src, cv::Mat &map_out, unsigned short threshold=4)
	{
		cv::Mat in=src.clone();
		map_out=cv::Mat::zeros(src.rows,src.cols,CV_8UC3);

		int size_x=in.cols, size_y=in.rows;

		for (int i = 0; i < (size_x*size_y); i++)
		{
//			//Forward direction x
			int y_xfw=i/size_x, x_xfw=i-y_xfw*size_x;

			int x=x_xfw;
			int y=y_xfw;

			int curValue=in.at<Vec1shrt>(y,x)[0];


			bool _IsNotTopRow=(y>0);
			bool _IsNotLeftCol=(x>0);
			bool _IsNotBottomRow=(y<(size_y-1));
			bool _IsNotRightCol=(x<(size_x-1));

			short cnt=0;
			unsigned char neighbors_close=0;
			unsigned char neighbors_nNAN=0;

			if(curValue>0)
			{
				//Top Row
				if(_IsNotTopRow)
				{
					//Left Top Cell
					if(_IsNotLeftCol)
					{
						int C_TL=in.at<Vec1shrt>(y_xfw-1,x_xfw-1)[0]; //Get top left cell
						if(C_TL>0)
						{
							neighbors_nNAN|=0x01;
							if(abs(C_TL-curValue)<threshold) //Bigger then zero and difference to current pixel smaller threshold?
							{
								neighbors_close|=0x01;
								cnt++;
							}
						}
					}

					//Middle Top Cell
					int C_TM=in.at<Vec1shrt>(y_xfw-1,x_xfw)[0];
					if(C_TM>0)
					{
						neighbors_nNAN|=0x00;
						if(abs(C_TM-curValue)<threshold)
						{
							neighbors_close|=0x02;
							cnt++;
						}
					}

					//Right Top Cell
					if(_IsNotRightCol)
					{
						int C_TR=in.at<Vec1shrt>(y_xfw-1,x_xfw+1)[0];
						if(C_TR>0)
						{
							neighbors_nNAN|=0x00;
							if(abs(C_TR-curValue)<threshold)
							{
								neighbors_close|=0x04;
								cnt++;
							}
						}
					}
				}


				//Middle Row

				//Left Middle Cell
				if(_IsNotLeftCol)
				{
					int C_ML=in.at<Vec1shrt>(y_xfw,x_xfw-1)[0];
					if(C_ML>0)
					{
						neighbors_nNAN|=0x00;
						if(abs(C_ML-curValue)<threshold)
						{
							neighbors_close|=0x80;
							cnt++;
						}
					}
				}

				//Right Middle Cell
				if(_IsNotRightCol)
				{
					int C_MR=in.at<Vec1shrt>(y_xfw,x_xfw+1)[0];
					if(C_MR>0)
					{
						neighbors_nNAN|=0x00;
						if(abs(C_MR-curValue)<threshold)
						{
							neighbors_close|=0x08;
							cnt++;
						}
					}
				}

				//Bottom Row
				if(_IsNotBottomRow)
				{
					//Left Bottom Cell
					if(_IsNotLeftCol)
					{
						int C_BL=in.at<Vec1shrt>(y_xfw+1,x_xfw-1)[0];
						if(C_BL>0)
						{
							neighbors_nNAN|=0x00;
							if(abs(C_BL-curValue)<threshold)
							{
								neighbors_close|=0x40;
								cnt++;
							}
						}
					}

					//Middle Bottom Cell
					int C_BM=in.at<Vec1shrt>(y_xfw+1,x_xfw)[0];
					if(C_BM>0)
					{
						neighbors_nNAN|=0x00;
						if(abs(C_BM-curValue)<threshold)
						{
							neighbors_close|=0x20;
							cnt++;
						}
					}

					//Right Bottom Cell
					if(_IsNotRightCol)
					{
						int C_BR=in.at<Vec1shrt>(y_xfw+1,x_xfw+1)[0];
						if(C_BR>0)
						{
							neighbors_nNAN|=0x00;
							if(abs(C_BR-curValue)<threshold)
							{
								neighbors_close|=0x10;
								cnt++;
							}
						}
					}

				}

			}//IF NOT NULL END

			map_out.at<Vec3char>(y,x)[0]=neighbors_close;
			map_out.at<Vec3char>(y,x)[1]=cnt;

		}//FOR END
	}


	/**
	 * This computes the normals out of the depth images, but requires a neighborhood map.
	 *
	 */
	static void createNormalMap(const cv::Mat &src, const cv::Mat &neighbor_map, cv::Mat &dst, image_geometry::PinholeCameraModel &model)
	{
		dst=cv::Mat::zeros(src.rows,src.cols,CV_16UC3);
		int size_x=src.cols, size_y=src.rows;
		//bool variables
		int y,x;
		uchar nb=0;
		short TL=0,TM=0,TR=0,ML=0,MR=0,BL=0,BM=0,BR=0;
		short center=0, left=0,right=0,top=0,bottom=0;
		short left_cnt=0,right_cnt=0,top_cnt=0,bottom_cnt=0;


		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;
			nb=neighbor_map.at<Vec2char>(y,x)[0];
			center=src.at<Vec1shrt>(y,x)[0];

			if(nb>0)
			{
				//Average generation for 9 pixels
				if(nb&0x01) //TOP LEFT
				{
					TL=src.at<Vec1shrt>(y-1,x-1)[0];
					top+=TL;
					top_cnt++;
					left+=TL;
					left_cnt++;
				}
				if(nb&0x02) //TOP MIDDLE
				{
					TM=src.at<Vec1shrt>(y-1,x)[0];
					top+=TM;
					top_cnt++;
				}
				if(nb&0x04) //TOP RIGHT
				{
					TR=src.at<Vec1shrt>(y-1,x+1)[0];
					top+=TR;
					top_cnt++;
					right+=TR;
					right_cnt++;
				}
				if(nb&0x08) //VMIDDLE RIGHT
				{
					MR=src.at<Vec1shrt>(y,x+1)[0];
					right+=MR;
					right_cnt++;
				}
				if(nb&0x10) //BOTTOM RIGHT
				{
					BR=src.at<Vec1shrt>(y+1,x+1)[0];
					bottom+=BR;
					bottom_cnt++;
					right+=BR;
					right_cnt++;
				}
				if(nb&0x20) //BOTTOM MIDDLE
				{
					BM=src.at<Vec1shrt>(y+1,x)[0];
					bottom+=BM;
					bottom_cnt++;
				}
				if(nb&0x40) //BOTTOM LEFT
				{
					BL=src.at<Vec1shrt>(y+1,x-1)[0];
					bottom+=BL;
					bottom_cnt++;

					left+=BL;
					left_cnt++;
				}
				if(nb&0x80) //VMIDDLE LEFT
				{
					ML=src.at<Vec1shrt>(y,x-1)[0];

					left+=ML;
					left_cnt++;
				}




				//Save
				//dst.at<Vec1shrt>(y,x)[0]=avg;
			}


		}
	}





//
//	static std::vector<cv::Rect> blobStepDetector(const cv::Mat &src, cv::Mat &out, unsigned short threshold=4)
//	{
//
//	}

private:

	void filter();
};




#endif /* DISCRETEFILLANDSMOOTHFILTER_H_ */
