/*
 * KinectTools.cpp
 *
 *  Created on: 24.04.2012
 *      Author: cyborg-x1
 */

#include "KinectTools/KinectTools.hpp"


namespace KinTo
{
	#include "kinectStepLUT"
	#include "ArcCosValues"

	void convertKinectRawToSteps(const cv::Mat &src, cv::Mat &dst)
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

	void convertStepsToKinectRaw(const cv::Mat &src, cv::Mat &dst)
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

	cv::Rect roiFinder(const cv::Mat &src)
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

	void RangeFilter(const cv::Mat &src, cv::Mat &dst, short min_range, short max_range)
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

	void gapStepMapGapFiller(const cv::Mat &src, cv::Mat &dst, uchar max_size)
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
							h_mark.at<Vec2shrt>(y_H,h_gap_start)[0]=next; 		//End Value
							h_mark.at<Vec2shrt>(y_H,h_gap_start)[1]=h_gap_len; //Pixels to be filled
							h_gap_len=0; //Clear the gap length
							h_inside_gap=false; //Back to outside gap
						}
					}
				}
			}
		}
	}

	void createRelationNeighbourhoodMap(const cv::Mat &src, cv::Mat &map_out, unsigned short threshold)
	{
		cv::Mat in=src.clone();
		map_out=cv::Mat::zeros(src.rows,src.cols,CV_8UC3);

		int size_x=in.cols, size_y=in.rows;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			//Forward direction x
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
						neighbors_nNAN|=0x02;
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
							neighbors_nNAN|=0x04;
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
						neighbors_nNAN|=0x80;
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
						neighbors_nNAN|=0x08;
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
							neighbors_nNAN|=0x40;
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
						neighbors_nNAN|=0x20;
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
							neighbors_nNAN|=0x10;
							if(abs(C_BR-curValue)<threshold)
							{
								neighbors_close|=0x10;
								cnt++;
							}
						}
					}

				}

			}//IF NOT NULL END

			map_out.at<Vec3uchar>(y,x)[0]=neighbors_close;
			map_out.at<Vec3uchar>(y,x)[1]=neighbors_nNAN;
			map_out.at<Vec3uchar>(y,x)[2]=cnt;

		}//FOR END
	}

	void stepMapBlur(const cv::Mat &src, cv::Mat &neigbors ,cv::Mat &dst)
	{

		dst = src.clone(); //TODO clone only if not the same
		int size_x=src.cols, size_y=src.rows;
		int x,y,avg;
		uchar nb;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;
			nb=neigbors.at<Vec3uchar>(y,x)[0];
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
				avg/=neigbors.at<Vec3uchar>(y,x)[2]+1;

				//Save back
				dst.at<Vec1shrt>(y,x)[0]=avg;

			}
		}
	}

	void createNormalMap(const cv::Mat &src, const cv::Mat &neighbor_map, const cv::Mat &xy, cv::Mat &normals)
	{
		normals=cv::Mat::zeros(src.rows,src.cols,CV_16UC3);

		int size_x=src.cols, size_y=src.rows;
		//bool variables
		int y,x;
		uchar nb=0;

		//Storing the resulting vectors
		cv::Mat V_top=cv::Mat::zeros(1, 1, CV_16UC3);
		cv::Mat V_left=cv::Mat::zeros(1, 1, CV_16UC3);
		cv::Mat V_right=cv::Mat::zeros(1, 1, CV_16UC3);
		cv::Mat V_bottom=cv::Mat::zeros(1, 1, CV_16UC3);

		//Results
		cv::Mat V_tr=cv::Mat::zeros(1, 1, CV_16UC1);
		cv::Mat V_bl=cv::Mat::zeros(1, 1, CV_16UC1);

		int cur=0,cur_x=0,cur_y=0;
		int cnt=0;

		int top_x,top_y,top_z;
		int left_x,left_y,left_z;
		int right_x,right_y,right_z;
		int bottom_x,bottom_y,bottom_z;


		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;
			nb=neighbor_map.at<Vec3uchar>(y,x)[0];

			cur=src.at<Vec1shrt>(y,x)[0];
			cur_x=xy.at<Vec2shrt>(y,x)[0];
			cur_y=xy.at<Vec2shrt>(y,x)[1];

			cnt=0;

			if(!nb || !cur)continue; //If this pixel has no suitable neighbors or if its zero skip it

				if((nb&0x02)&&(nb&0x08)) //TOP MIDDLE
				{
					top_x=xy.at<Vec2shrt>(y-1,x)[0]-cur_x; //X
					top_y=xy.at<Vec2shrt>(y-1,x)[1]-cur_y; //Y
					top_z=src.at<Vec1shrt>(y-1,x)[0]-cur;  //Z

					right_x=xy.at<Vec2shrt>(y,x+1)[0]-cur_x;
					right_y=xy.at<Vec2shrt>(y,x+1)[1]-cur_y;
					right_z=src.at<Vec1shrt>(y,x+1)[0]-cur;

					normals.at<Vec3shrt>(y,x)[0]+=top_y*right_z-top_z*right_y;
					normals.at<Vec3shrt>(y,x)[1]+=top_z*right_x-top_x*right_z;
					normals.at<Vec3shrt>(y,x)[2]+=top_x*right_y-top_y*right_x;
					cnt++;

				}

				if((nb&0x20)&&(nb&0x80)) //BOTTOM MIDDLE
				{
					bottom_x=xy.at<Vec2shrt>(y+1,x)[0]-cur_x;
					bottom_y=xy.at<Vec2shrt>(y+1,x)[1]-cur_y;
					bottom_z=src.at<Vec1shrt>(y+1,x)[0]-cur;

					left_x=xy.at<Vec2shrt>(y,x-1)[0]-cur_x;
					left_y=xy.at<Vec2shrt>(y,x-1)[1]-cur_y;
					left_z=src.at<Vec1shrt>(y,x-1)[0]-cur;

					normals.at<Vec3shrt>(y,x)[0]+=bottom_y*left_z-bottom_z*left_y;
					normals.at<Vec3shrt>(y,x)[1]+=bottom_z*left_x-bottom_x*left_z;
					normals.at<Vec3shrt>(y,x)[2]+=bottom_x*left_y-bottom_y*left_x;



					cnt++;
				}

				if(cnt>1)
				{
					normals.at<Vec3shrt>(y,x)[0]/=cnt;
					normals.at<Vec3shrt>(y,x)[1]/=cnt;
					normals.at<Vec3shrt>(y,x)[2]/=cnt;
				}

		}//FOR END
	}

	void createXYMap(const cv::Mat &src, const sensor_msgs::CameraInfoConstPtr& info_msg, cv::Mat &xy)
	{
		if(src.type() == CV_16UC1)
		{
			xy=cv::Mat::zeros(src.rows,src.cols,CV_16UC2);

			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(info_msg);

			int center_x = model.cx()*100;					//319.5*100=319500
			int center_y = model.cy()*100;					//239.5*100=319500

			int constant_x = model.fx()*100;
			int constant_y = model.fy()*100;

			int size_x=src.cols, size_y=src.rows;

			int x,y;
			for (int i = 0; i < (size_x*size_y); i++)
			{
				y=i/size_x;
				x=i-y*size_x;

				if(x>0)
				{
					short depth=src.at<Vec1shrt>(y,x)[0];
					xy.at<Vec2shrt>(y,x)[0] = ((x*100 - center_x) * depth) / constant_x;
					xy.at<Vec2shrt>(y,x)[1] = ((y*100 - center_y) * depth) / constant_y;
				}
			}
		}
		else
		{
			ROS_ERROR("WRONG TYPE: createXYMap");
		}
	}

	void crossDepthBlur(const cv::Mat &depth, const cv::Mat &neighbors, cv::Mat &depth_out, int max_size)
	{

		cv::Mat dst=cv::Mat::zeros(depth.rows,depth.cols,CV_16UC1);
		int size_x=depth.cols, size_y=depth.rows;
		int y,x;
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;

			//Sum all pixels
			int sum=depth.at<Vec1shrt>(y, x)[0];

			//If current pixel is 0 go to next
			if(sum == 0) continue;

			//Count all values
			int cnt=1;

			//Get the neighbors of the current pixel
			uchar cur_nb=neighbors.at<Vec3uchar>(y,x)[0];

			//Usable neighbors
			bool nb_top=(1<<1)&cur_nb;
			bool nb_right=(1<<3)&cur_nb;
			bool nb_bottom=(1<<5)&cur_nb;
			bool nb_left=(1<<7)&cur_nb;

			for(int j=1; j<=max_size; j++)
			{
				if(nb_top)
				{
					nb_top=neighbors.at<Vec3uchar>(y-j,x)[0]&(1<<1);
					sum+=depth.at<Vec1shrt>(y-j, x)[0];
					cnt++;
				}
				if(nb_right)
				{
					nb_right=neighbors.at<Vec3uchar>(y,x+j)[0]&(1<<3);
					sum+=depth.at<Vec1shrt>(y, x+j)[0];
					cnt++;
				}
				if(nb_bottom)
				{
					nb_bottom=neighbors.at<Vec3uchar>(y+j,x)[0]&(1<<5);
					sum+=depth.at<Vec1shrt>(y+j, x)[0];
					cnt++;
				}
				if(nb_left)
				{
					nb_left=neighbors.at<Vec3uchar>(y,x-j)[0]&(1<<7);
					sum+=depth.at<Vec1shrt>(y, x-j)[0];
					cnt++;
				}

				//If no suitable neighbor is available exit loop
				if(nb_left + nb_right + nb_top + nb_bottom == 0) break;
			}

			dst.at<Vec1shrt>(y,x)=sum/cnt;
		}
		depth_out=dst;
	}

	void crossNormalBlur(const cv::Mat &normals, const cv::Mat &neighbors, cv::Mat &normals_out, int max_size)
	{

		cv::Mat dst=cv::Mat::zeros(normals.rows,normals.cols,CV_16UC3);
		int size_x=normals.cols, size_y=normals.rows;
		int y,x;
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;

			//Get the neighbors of the current pixel
			uchar cur_nb=neighbors.at<Vec3uchar>(y,x)[0];
			if(cur_nb == 0) continue;


			//Sum all pixels
			int sum_x=normals.at<Vec3shrt>(y, x)[0];
			int sum_y=normals.at<Vec3shrt>(y, x)[1];
			int sum_z=normals.at<Vec3shrt>(y, x)[2];

			//Count all values
			int cnt=1;

			//Usable neighbors
			bool nb_top=(1<<1)&cur_nb;
			bool nb_right=(1<<3)&cur_nb;
			bool nb_bottom=(1<<5)&cur_nb;
			bool nb_left=(1<<7)&cur_nb;

			for(int j=1; j<=max_size; j++)
			{
				if(nb_top)
				{
					nb_top=neighbors.at<Vec3uchar>(y-j,x)[0]&(1<<1);
					sum_x+=normals.at<Vec3shrt>(y-j, x)[0];
					sum_y+=normals.at<Vec3shrt>(y-j, x)[1];
					sum_z+=normals.at<Vec3shrt>(y-j, x)[2];
					cnt++;
				}
				if(nb_right)
				{
					nb_right=neighbors.at<Vec3uchar>(y,x+j)[0]&(1<<3);
					sum_x+=normals.at<Vec3shrt>(y, x+j)[0];
					sum_y+=normals.at<Vec3shrt>(y, x+j)[1];
					sum_z+=normals.at<Vec3shrt>(y, x+j)[2];
					cnt++;
				}
				if(nb_bottom)
				{
					nb_bottom=neighbors.at<Vec3uchar>(y+j,x)[0]&(1<<5);
					sum_x+=normals.at<Vec3shrt>(y+j, x)[0];
					sum_y+=normals.at<Vec3shrt>(y+j, x)[1];
					sum_z+=normals.at<Vec3shrt>(y+j, x)[2];
					cnt++;
				}
				if(nb_left)
				{
					nb_left=neighbors.at<Vec3uchar>(y,x-j)[0]&(1<<7);
					sum_x+=normals.at<Vec3shrt>(y, x-j)[0];
					sum_y+=normals.at<Vec3shrt>(y, x-j)[1];
					sum_z+=normals.at<Vec3shrt>(y, x-j)[2];
					cnt++;
				}

				//If no suitable neighbor is available exit loop
				if(nb_left + nb_right + nb_top + nb_bottom == 0) break;
			}

			dst.at<Vec3shrt>(y,x)[0]=sum_x/cnt;
			dst.at<Vec3shrt>(y,x)[1]=sum_y/cnt;
			dst.at<Vec3shrt>(y,x)[2]=sum_z/cnt;
		}
		normals_out=dst;
	}

	void createAngleMap(const cv::Mat &normals, cv::Mat &angles)
	{
		angles=cv::Mat::zeros(normals.rows,normals.cols,CV_8UC3);
		int size_x=normals.cols, size_y=normals.rows;
		//bool variables
		int y,x;
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;


			short g1=(normals.at<Vec3shrt>(y,x)[0]);
			short g2=(normals.at<Vec3shrt>(y,x)[1]);
			short g3=(normals.at<Vec3shrt>(y,x)[2]);



			double vector_length=sqrt(g1*g1+g2*g2+g3*g3);
			if(!vector_length)continue;

			angles.at<Vec3uchar>(y,x)[0]=preCalcCos[(int)(g1*1000/vector_length)+1000]/100;
			angles.at<Vec3uchar>(y,x)[1]=preCalcCos[(int)(g2*1000/vector_length)+1000]/100;
			angles.at<Vec3uchar>(y,x)[2]=preCalcCos[(int)(g3*1000/vector_length)+1000]/100;
		}
	}

	void crossAnglesBlur(const cv::Mat &angles, const cv::Mat &neighbors, cv::Mat &angles_out, int max_size)
	{

		cv::Mat dst=cv::Mat::zeros(angles.rows,angles.cols,CV_8UC3);
		int size_x=angles.cols, size_y=angles.rows;
		int y,x;
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;

			//Get the neighbors of the current pixel
			uchar cur_nb=neighbors.at<Vec3uchar>(y,x)[0];
			if(cur_nb == 0) continue;


			//Sum all pixels
			int sum_x=angles.at<Vec3uchar>(y, x)[0];
			int sum_y=angles.at<Vec3uchar>(y, x)[1];
			int sum_z=angles.at<Vec3uchar>(y, x)[2];

			//Count all values
			int cnt=1;

			//Usable neighbors
			bool nb_top=(1<<1)&cur_nb;
			bool nb_right=(1<<3)&cur_nb;
			bool nb_bottom=(1<<5)&cur_nb;
			bool nb_left=(1<<7)&cur_nb;

			for(int j=1; j<=max_size; j++)
			{
				if(nb_top)
				{
					nb_top=neighbors.at<Vec3uchar>(y-j,x)[0]&(1<<1);
					sum_x+=angles.at<Vec3uchar>(y-j, x)[0];
					sum_y+=angles.at<Vec3uchar>(y-j, x)[1];
					sum_z+=angles.at<Vec3uchar>(y-j, x)[2];
					cnt++;
				}
				if(nb_right)
				{
					nb_right=neighbors.at<Vec3uchar>(y,x+j)[0]&(1<<3);
					sum_x+=angles.at<Vec3uchar>(y, x+j)[0];
					sum_y+=angles.at<Vec3uchar>(y, x+j)[1];
					sum_z+=angles.at<Vec3uchar>(y, x+j)[2];
					cnt++;
				}
				if(nb_bottom)
				{
					nb_bottom=neighbors.at<Vec3uchar>(y+j,x)[0]&(1<<5);
					sum_x+=angles.at<Vec3uchar>(y+j, x)[0];
					sum_y+=angles.at<Vec3uchar>(y+j, x)[1];
					sum_z+=angles.at<Vec3uchar>(y+j, x)[2];
					cnt++;
				}
				if(nb_left)
				{
					nb_left=neighbors.at<Vec3uchar>(y,x-j)[0]&(1<<7);
					sum_x+=angles.at<Vec3uchar>(y, x-j)[0];
					sum_y+=angles.at<Vec3uchar>(y, x-j)[1];
					sum_z+=angles.at<Vec3uchar>(y, x-j)[2];
					cnt++;
				}

				//If no suitable neighbor is available exit loop
				if(nb_left + nb_right + nb_top + nb_bottom == 0) break;
			}

			dst.at<Vec3uchar>(y,x)[0]=sum_x/cnt;
			dst.at<Vec3uchar>(y,x)[1]=sum_y/cnt;
			dst.at<Vec3uchar>(y,x)[2]=sum_z/cnt;
		}
		angles_out=dst;
	}

	void anglesFilter(const cv::Mat &angles, cv::Mat &angles_out, unsigned int x_angle_min,unsigned int x_angle_max,unsigned int y_angle_min,unsigned int y_angle_max,unsigned int z_angle_min,unsigned int z_angle_max, bool binary)
	{
		cv::Mat dst;
		if(!binary)
		{
			dst=cv::Mat::zeros(angles.rows,angles.cols,CV_8UC3);
		}
		else
		{
			dst=cv::Mat::zeros(angles.rows,angles.cols,CV_8UC1);
		}

		int size_x=angles.cols, size_y=angles.rows;
		int y,x;
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;
			unsigned int a_x=angles.at<Vec3uchar>(y,x)[0];
			unsigned int a_y=angles.at<Vec3uchar>(y,x)[1];
			unsigned int a_z=angles.at<Vec3uchar>(y,x)[2];

			if(a_x >= x_angle_min && a_x <= x_angle_max && a_y >= y_angle_min && a_y <= y_angle_max && a_z >= z_angle_min && a_z <= z_angle_max )
			{
				if(!binary)
				{
					dst.at<Vec3uchar>(y,x)[0]=a_x;
					dst.at<Vec3uchar>(y,x)[1]=a_y;
					dst.at<Vec3uchar>(y,x)[2]=a_z;
				}
				else
				{
					dst.at<Vec1uchar>(y,x)[0]=255;
				}
			}
		}
		angles_out=dst;
	}

	void XYZrangeFilter(const cv::Mat &depth, const cv::Mat &xy, cv::Mat &depth_out, int min_x, int max_x, int min_y, int max_y, int min_z, int max_z)
	{

		int size_x=depth.cols, size_y=depth.rows;
		depth_out=depth.clone();
		int y,x;
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;


			short cur_z=depth_out.at<Vec1shrt>(y,x)[0];


			if(cur_z == 0) continue; //If depth == 0 continue with next pixel

			int cur_x=xy.at<Vec2shrt>(y,x)[0];
			int cur_y=xy.at<Vec2shrt>(y,x)[1];

			if(!((cur_x >= min_x && cur_x <= max_x)&&(cur_y >= min_y && cur_y <= max_y)&&(cur_z >= min_z && cur_z <= max_z)))
			{
				depth_out.at<Vec1shrt>(y,x)[0]=0;
			}
		}
	}


	///BLOB Separator

	BlobSurfaces::BlobSurfaces(const cv::Mat &angles_ok, std::vector<cv::Rect> &ROIs, int threshold)
	:regions(cv::Mat::zeros(angles_ok.rows,angles_ok.cols,CV_8UC1)),
	 size_x(angles_ok.cols),
	 size_y(angles_ok.rows),
	 in(angles_ok),
	 rois(ROIs),
	 threshold(threshold)
	{}

	void BlobSurfaces::setMinMax(int x, int y)
	{
		if(x<min_x) min_x=x;
		if(y<min_y) min_y=y;
		if(x>max_x) max_x=x;
		if(y>max_y) max_y=y;
	}

	void BlobSurfaces::resetMinMax(int x, int y)
	{
		max_x=min_x=x;
		max_y=min_y=y;
	}

	void BlobSurfaces::addRectFromMinMax()
	{
		rois.push_back(cv::Rect(min_x,min_y,max_x,max_y));
	}

	BlobSurfaces::Direction BlobSurfaces::checkDirection(int x_cd, int y_cd, BlobSurfaces::Direction current_dir)
	{
		if(x_cd>=size_x || y_cd>=size_y)
		{
			ROS_ERROR("Error Object::extract outside of mat bounds!!");
			throw(0);
		}
		if(!current_dir)//if we are at the first value return begin direction
		{
			if	   (get_pixel_value_in_dir(x_cd,y_cd,4)){current_dir=right;}
			else if(get_pixel_value_in_dir(x_cd,y_cd,5)){current_dir=bottom_right;}
			else if(get_pixel_value_in_dir(x_cd,y_cd,6)){current_dir=bottom;}
			else if(get_pixel_value_in_dir(x_cd,y_cd,7)){current_dir=bottom_left;}
			return current_dir;
		}

		unsigned int left_of_curdir=(current_dir != 1)?current_dir--:8;
		unsigned int right_of_curdir=(current_dir != 8)?current_dir++:1;

		bool pix_in_dir=get_pixel_value_in_dir(x_cd,y_cd,current_dir);	   	 //The pix bool value (b/w) in current move direction
		bool box_left_dir=get_pixel_value_in_dir(x_cd,y_cd,left_of_curdir);  //in box left of the box of move dir
		bool box_right_dir=get_pixel_value_in_dir(x_cd,y_cd,right_of_curdir);//right



		if(!pix_in_dir ||(box_left_dir && box_right_dir))
		{
			bool pix_found=0;
			int new_dir=current_dir-2;
			if(new_dir <=0) new_dir+=8;

			while(!pix_found)
			{
				switch(new_dir)
				{
				case 1: pix_found=get_pixel_value_in_dir(x_cd,y_cd,1); break;
				case 2: pix_found=get_pixel_value_in_dir(x_cd,y_cd,2); break;
				case 3: pix_found=get_pixel_value_in_dir(x_cd,y_cd,3); break;
				case 4: pix_found=get_pixel_value_in_dir(x_cd,y_cd,4); break;
				case 5: pix_found=get_pixel_value_in_dir(x_cd,y_cd,5); break;
				case 6: pix_found=get_pixel_value_in_dir(x_cd,y_cd,6); break;
				case 7: pix_found=get_pixel_value_in_dir(x_cd,y_cd,7); break;
				case 8: pix_found=get_pixel_value_in_dir(x_cd,y_cd,8); break;

				default: throw("Unhandled exception"); break;
				}

				if(!pix_found)
				{
					new_dir++;
					if(new_dir > 8) new_dir=new_dir -8;
				}
				else
				{
					current_dir=new_dir;
				}
			}
		}

		return current_dir;
	}

	bool BlobSurfaces::get_pixel_value_in_dir(int x_gv,  int y_gv, unsigned int dir)
	{
		switch(dir)
		{
			case none:		 	ROS_ERROR("get Pixel in dir -> direction NONE!!!!"); throw(0); break;
			case top_left: 		x_gv--; y_gv--; 	break;
			case top: 	 		y_gv--; 			break;
			case top_right: 	x_gv++; y_gv--; 	break;
			case right: 		x_gv++;      		break;
			case bottom_right: 	x_gv++; y_gv++; 	break;
			case bottom: 	 	y_gv++; 			break;
			case bottom_left: 	x_gv--; y_gv++; 	break;
			case left:			x_gv--; 	  		break;
		}

		if((x_gv>=0 && y_gv>=0) && (x_gv<(int)size_x && y_gv<(int)size_y))
		{
			return in.at<Vec3uchar>(y_gv,x_gv)[0] > threshold;
		}
		else
		{
			return 0;
		}
	}







	void BlobSurfaces::stateMachine()
	{
		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;

			if(in.at<Vec3uchar>(y,x)[0])
			{

			}

			//Find reorientation points or border pixel direction
					unsigned int state=0;
					Direction current_dir, new_dir;
					unsigned int x_dim=x;
					unsigned int y_dim=y;
					resetMinMax(x,y);

					while(state < 2)
					{
						switch(state)
						{
						case 0: //Start state find first direction
								current_dir=checkDirection(x_dim,y_dim,none);
								if(current_dir)
								{
									state=1;
									regions.at<Vec1uchar>(y_dim,x_dim)[0]++;
								}
								else //Single pixel
								{
									regions.at<Vec1uchar>(y_dim,x_dim)[0]+=2;
								}
						break;

						case 1: //Move
								switch(current_dir)
								{
									case none: break;
									case top_left: 		x_dim--;y_dim--; break;
									case top: 	     			y_dim--; break;
									case top_right: 	x_dim++;y_dim--; break;
									case right: 		x_dim++;      	 break;
									case bottom_right: 	x_dim++;y_dim++; break;
									case bottom: 	 			y_dim++; break;
									case bottom_left: 	x_dim--;y_dim++; break;
									case left: 			x_dim--; 	  	 break;
								}
								//If we reached start again stop and go to state 2
								if(x==x_dim && y==y_dim)
								{
									state=2;
								}
								else
								{
									//Check if we still go into this direction
									new_dir=checkDirection(x_dim,y_dim,current_dir);
									current_dir=new_dir;
								}
								setMinMax(x_dim,y_dim);
								regions.at<Vec1uchar>(y_dim,x_dim)[0]++;

						break;
						default:
							ROS_ERROR("extractBorderPoints:: Unknown STATE!");
							throw(0);
							break;
						}
					}
		}
	}




} /* namespace KinTo */
