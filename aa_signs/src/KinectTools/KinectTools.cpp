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

			if(!nb)continue;

//			if(((nb&0x22)&&(nb&0x88)))//Check if at least one normal can be built for this pixel
//			{
				if((nb&0x02)&&(nb&0x08)) //TOP MIDDLE
				{
					top_x=xy.at<Vec2shrt>(y-1,x)[0]-cur_x; //X
					top_y=xy.at<Vec2shrt>(y-1,x)[1]-cur_y; //Y
					top_z=src.at<Vec1shrt>(y-1,x)[0]-cur;  //Z


		//		}
		//		if(nb&0x08) //VMIDDLE RIGHT
		//		{
					right_x=xy.at<Vec2shrt>(y,x+1)[0]-cur_x;
					right_y=xy.at<Vec2shrt>(y,x+1)[1]-cur_y;
					right_z=src.at<Vec1shrt>(y,x+1)[0]-cur;

					//cout<<"TOP: "<<top_x<<" "<<top_y<<" "<<top_z<<endl;
					//cout<<"RIGHT: "<<right_x<<" "<<right_y<<" "<<right_z<<endl;


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
//				}
//				if(nb&0x80) //VMIDDLE LEFT
//				{
					left_x=xy.at<Vec2shrt>(y,x-1)[0]-cur_x;
					left_y=xy.at<Vec2shrt>(y,x-1)[1]-cur_y;
					left_z=src.at<Vec1shrt>(y,x-1)[0]-cur;

					//cout<<"BOTTOM: "<<bottom_x<<" "<<bottom_y<<" "<<bottom_z<<endl;
					//cout<<"LEFT: "<<left_x<<" "<<left_y<<" "<<left_z<<endl;



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

				//cout<<"300NORMAL: ("<<normals.at<Vec3shrt>(y,x)[0]<<"|"<<normals.at<Vec3shrt>(y,x)[1]<<"|"<<normals.at<Vec3shrt>(y,x)[2]<<")"<<endl;


				//Save
				//dst.at<Vec1shrt>(y,x)[0]=avg;
//			}


		}//FOR END
	}

	void createXYMap(const cv::Mat &src, const sensor_msgs::CameraInfoConstPtr& info_msg, cv::Mat &xy)
	{
		if(src.type() == CV_16UC1)
		{
			xy=cv::Mat::zeros(src.rows,src.cols,CV_32SC2);

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
					xy.at<Vec2shrt>(y,x)[0] = ((y*100 - center_y) * depth) / constant_y;
				}
			}
		}
		else
		{
			ROS_ERROR("WRONG TYPE: createXYMap");
		}
	}

	void blurDepth(const cv::Mat &src, cv::Mat &dst)
	{

		if (src.type() == CV_16UC1)
		{
			cv::Mat filter_in = src.clone();
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

	int heron_sqrt(int f, int max_it)
	{
		if(f<0) std::cerr<<"heron_sqrt: ERROR NEGATIVE VALUES ARE UNSUPPORTED!"<<std::endl;

		if(f==1 || f==0)return f;

		//Calculate start value
		int f_g=(f%2)?f-1:f; //create even number
		int x=(f*10000/f_g)/2;
		int x_m=0;
		for(int i=0; i<max_it;  i++)
		{
				x_m=(x+(f*10000)/x)/2; //next Iteration
				if(x==x_m)break; //End if it's the same value like before
				x=x_m; //Assign new start value
		}
		return x;
	}

	/**
	 * This creates a viewable image form the normal map
	 */
	void rgbNormals(const cv::Mat &src, cv::Mat &dst)
	{

		dst=cv::Mat::zeros(src.rows,src.cols,CV_8UC3);
		int size_x=src.cols, size_y=src.rows;
		//bool variables
		int y,x;

		for (int i = 0; i < (size_x*size_y); i++)
		{
			y=i/size_x;
			x=i-y*size_x;


			short g1=(src.at<Vec3shrt>(y,x)[0]);
			short g2=(src.at<Vec3shrt>(y,x)[1]);
			short g3=(src.at<Vec3shrt>(y,x)[2]);


//			g1*=10;
//			g2*=10;
//			g3*=10;
//
//			if(g1>255)g1=255;
//			if(g2>255)g2=255;
//			if(g3>255)g3=255;


			int vector_length=heron_sqrt((g1*g1)+(g2*g2)+(g3*g3),7);


			int angle_x=acos((double)g2/vector_length*180/3.14);


				dst.at<Vec3uchar>(y,x)[0]=angle_x;
				dst.at<Vec3uchar>(y,x)[1]=0;
				dst.at<Vec3uchar>(y,x)[2]=0;
		}
	}


} /* namespace KinTo */
