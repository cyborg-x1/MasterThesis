/*
 * tmpMatch.cpp
 *
 *  Created on: 21.06.2012
 *      Author: cyborg-x1
 */

#include "KinectTools/tmpMatch.hpp"

namespace KinTo
{


void MatchTempProfile::removeAlpha(const cv::Mat &in, cv::Mat &out)
{

	std::cout<<"Channels:"<<in.channels()<<std::endl;
	if(in.channels()!=4)
	{
		out=in.clone();
	}
	else
	{
		int size_x=in.cols;
		int size_y=in.rows;

		std::cout<<"IN "<<in.channels()<<std::endl;
		out=cv::Mat::zeros(in.size(),CV_8UC3);

		for(int i=0;i<(size_x*size_y);i++)
		{
			int x=i/size_y;
			int y=i-x*size_y;
			if(in.at<Vec4uchar>(y,x)[3]==255)
			{
				out.at<Vec3uchar>(y,x)[0]=in.at<Vec4uchar>(y,x)[0];
				out.at<Vec3uchar>(y,x)[1]=in.at<Vec4uchar>(y,x)[1];
				out.at<Vec3uchar>(y,x)[2]=in.at<Vec4uchar>(y,x)[2];
			}
			else
			{
				out.at<Vec3uchar>(y,x)[0]=255;
				out.at<Vec3uchar>(y,x)[1]=255;
				out.at<Vec3uchar>(y,x)[2]=255;
			}
		}
	}
}



void MatchTempProfile::constructor_extension(int threshold)
{

		tmp2=tmp.clone();
		removeAlpha(tmp2,tmp);

		cv::cvtColor(tmp, gray, CV_BGR2GRAY);
		bool hasAlpha=false;

		//Check if there is a alpha channel
		switch(tmp2.channels())
		{
		case 1:

			break;
		case 3:

			break;
		case 4:
			//hasAlpha=true;
			break;
		default:
			std::cerr<<"Error: "<<__PRETTY_FUNCTION__<<" Unsupported amount of channels!"<<std::endl;
			break;
		}

		//Get size
		int size_x=tmp.cols;
		int size_y=tmp.rows;

		//If it has an alpha channel set start to false
		bool found_dia_start=!hasAlpha;
		bool found_dia_end=false;

		int last_change=0;
		int last_size=0;

		//get the first value: dark/notDark;
		bool dark=gray.at<Vec1uchar>(0,0)[0]<threshold;

		for(int i=0;i<(size_x*size_y);i++)
		{

			int x_dia=i;
			int y_dia=i;


			if(x_dia<size_x && y_dia<size_y && !found_dia_end)
			{
				if(found_dia_start)
				{
					if(tmp2.at<Vec4uchar>(y_dia,x_dia)[3]==255 || !hasAlpha)
					{
						if(dark && gray.at<Vec1uchar>(y_dia,x_dia)[1]>=threshold)
						{
							int current_size=i-last_change;
							if(last_size!=0)
							{
								Proportion prop;
								prop.x=x_dia;
								prop.y=y_dia;
								prop.length_l=last_size;
								prop.length_c=current_size;
								prop.proportion=(double)last_size/(double)current_size;
								proportions.push_back(prop);
								std::cout<<prop.proportion<<std::endl;
							}
							last_size=current_size;
							last_change=i;
							dark=false;
						}
						else if(!dark && gray.at<Vec1uchar>(y_dia,x_dia)[1]<=threshold)
						{
							int current_size=i-last_change;
							if(last_size!=0)
							{
								Proportion prop;
								prop.x=x_dia;
								prop.y=y_dia;
								prop.length_l=last_size;
								prop.length_c=current_size;
								prop.proportion=-(double)last_size/(double)current_size;
								proportions.push_back(prop);
							}
							last_size=i-last_change;
							last_change=i;
							dark=true;
						}

					}
					else
					{

						found_dia_end=true;
					}

				}
				else
				{
					if(tmp2.at<Vec4uchar>(y_dia,x_dia)[3]==255)
					{
						//get the first value of a non translucent pixel: dark/notDark;
						dark=gray.at<Vec1uchar>(y_dia,x_dia)[0]<threshold;
						last_change=i;
						found_dia_start=true;
						i--;
					}
				}
			}
			else
			{
				break;
			}
		}
}

MatchTempProfile::MatchTempProfile(const cv::Mat &tmp, std::string name, double minimal_target_size, double max_target_size, uchar threshold, double max_proportion_diff, double min_coverage, double min_congruence)
:tmp(tmp)
,begin(0,0)
,end(tmp.cols-1,tmp.rows-1)
,last_pair()
,max_proportion_diff(max_proportion_diff)
,min_congruence(min_congruence)
,min_coverage(min_coverage)
,name(name)
,min_target_size(minimal_target_size)
,max_target_size(max_target_size)
,hasImportantPixels(false)
{
	constructor_extension(threshold);
}

MatchTempProfile::MatchTempProfile(const cv::Mat &tmp, const cv::Mat &tmp_imp, std::string name, double minimal_target_size, double max_target_size, uchar threshold, double max_proportion_diff, double min_coverage, double min_congruence)
:tmp(tmp)
,begin(0,0)
,end(tmp.cols-1,tmp.rows-1)
,last_pair()
,max_proportion_diff(max_proportion_diff)
,min_congruence(min_congruence)
,min_coverage(min_coverage)
,name(name)
,min_target_size(minimal_target_size)
,max_target_size(max_target_size)
,important_pixels(tmp_imp)
,hasImportantPixels(true)
{
	constructor_extension(threshold);
}

void MatchTempProfile::templateMatching(Proportion fromTemplate, Proportion fromTarget,const cv::Mat& target,int threshold)
{

	//scaling factor for the template
	double scaling_factor=(double)fromTarget.length_l+(double)fromTarget.length_c;
	scaling_factor/=(double)fromTemplate.length_c+(double)fromTemplate.length_l;

	if(scaling_factor<=0)
	{
		std::cerr<<"MatchTemplateProfile::templateMatching scaling_factor smaller or equal zero"<<std::endl;
		return;
	}


	if(scaling_factor<min_target_size || scaling_factor>max_target_size)return;

	//scale the template picture
	cv::Mat scaled_template, scaled_imp_template, scaled_alpha_tmp;
	cv::resize(gray,scaled_template,cv::Size(0,0),scaling_factor,scaling_factor);
	cv::resize(tmp2,scaled_alpha_tmp,cv::Size(0,0),scaling_factor,scaling_factor);

	if(!important_pixels.empty())
	{
		cv::resize(important_pixels,scaled_imp_template,cv::Size(0,0),scaling_factor,scaling_factor);
	}



	//recalculate the point of the proportion
	int x_st=fromTemplate.x*scaling_factor;
	int y_st=fromTemplate.y*scaling_factor;

	//calculate the point for template placement
	int x_tplace=fromTarget.x-x_st;
	int y_tplace=fromTarget.y-y_st;

	int size_x=scaled_template.cols;
	int size_y=scaled_template.rows;

	cv::Rect r;
	r.height=scaled_template.rows;
	r.width=scaled_template.cols;
	r.x=x_tplace;
	r.y=y_tplace;


	int cnt_pixelsCmp=0;
	int cnt_pixelsOk=0;
	int cnt_pixelsNegative=0;
	int pixelAlpha=0;

	for(int i=0; i<(size_x*size_y);i++)
	{
		int y_temp=i/size_x;
		int x_temp=i-y_temp*size_x;

		int x_targ=x_temp+x_tplace;
		int y_targ=y_temp+y_tplace;



		if(x_targ>=0 && x_targ<target.cols &&
		   y_targ>=0 && y_targ<target.rows )
		{


			if(scaled_alpha_tmp.channels()==4 && scaled_alpha_tmp.at<Vec4uchar>(y_temp,x_temp)[3]==0)
			{
				pixelAlpha++;
				continue;
			}
			cnt_pixelsCmp++;


			bool val_tmp=scaled_template.at<Vec1uchar>(y_temp,x_temp)[0]>127;
			bool val_trg=target.at<Vec1uchar>(y_targ,x_targ)[0]>threshold;


			if(val_tmp==val_trg)
			{
				cnt_pixelsOk++;
			}
			else
			{
				if(hasImportantPixels)
				{
					if(scaled_imp_template.at<Vec1uchar>(y_temp,x_temp)[0]==0)
					{
						cnt_pixelsNegative++;
					}
				}
			}
		}
	}



	double congruence=((double)cnt_pixelsOk-cnt_pixelsNegative/2)/(double)cnt_pixelsCmp;
	double coverage=((double)cnt_pixelsCmp+pixelAlpha)/(double)(size_x*size_y);

//	std::cout<<"congruence "<<congruence<<std::endl;
//	std::cout<<"coverage   "<<coverage<<std::endl;
//	std::cout<<"OK		   "<<cnt_pixelsOk<<std::endl;
//	std::cout<<"Alpha      "<<pixelAlpha<<std::endl;
//	std::cout<<"CMP        "<<cnt_pixelsCmp<<std::endl;
//	std::cout<<"Negative   "<<cnt_pixelsNegative<<std::endl;
//
//
//

	if(congruence>=min_congruence && coverage>=min_coverage)
	{
		Match m;
		m.congruence=congruence;
		m.coverage=coverage;
		m.rect.x=x_tplace;
		m.rect.y=y_tplace;
		m.rect.width=size_x;
		m.rect.height=size_y;
		m.center=cv::Point(x_tplace+size_x/2, y_tplace+size_y/2);
		m.name=name;

		matches.push_back(m);
	}
}





void MatchTempProfile::checkProportion(Proportion proportion, const cv::Mat &target, int threshold)
{
	//Move second to first
	last_pair.first=last_pair.second;
	//Save current to second
	last_pair.second=proportion;

	//If it already contains a pair
	if(last_pair.first.proportion!=-1)
	{
		//Seek for pair in proportions
		for(std::vector<Proportion>::iterator it=proportions.begin()
				;it!=proportions.end();it++)
		{
			Proportion *p1=&(*it);
			if(std::fabs(p1->proportion-last_pair.first.proportion)<=max_proportion_diff)
			{
				if(it+1 != proportions.end())
				{
					Proportion *p2=&(*(it+1));
					if(std::fabs(p2->proportion-last_pair.second.proportion)<=max_proportion_diff)
					{
						templateMatching(*(it+1), proportion, target, threshold);
					}
				}
			}
		}
	}





}

void MatchTempProfile::reset_ProportionCheck()
{
	last_pair=proportion_pair();
}

void proportionEnhancedTemplateMatching(std::vector<MatchTempProfile> &templates, const cv::Mat &target, uchar threshold)
{

	cv::Mat blured;
	cv::cvtColor(target, blured, CV_BGR2GRAY);
//	cv::equalizeHist(blured,blured);
	cv::threshold(blured,blured,threshold,255,0);

	cv::GaussianBlur(blured,blured,cv::Size(3,3),2,2,0);

//	cv::imshow("seek",blured);



	int size_x=target.cols;
	int size_y=target.rows;

	int x_dia=size_x-1;
	int y_dia=0;

	int start_col=size_x-1;
	int start_row=0;

	int last_length=-1;
	int current_length=0;
	int dark=0;

	for(int i=0; i<(size_x*size_y);i++)
	{


		if(y_dia>=size_y || x_dia>=size_x)
		{
			if(start_col>0)
			{
				start_col--;
				x_dia=start_col;
				y_dia=0;
			}
			else
			{
				start_row++;
				x_dia=0;
				y_dia=start_row;
			}
		}
		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////DIAGONAL X/Y////////////////////////////////

		if(y_dia == 0 || x_dia==0)
		{
			dark=blured.at<Vec1uchar>(y_dia,x_dia)[0]<threshold;
			last_length=-1;
			current_length=0;
			for(std::vector<MatchTempProfile>::iterator it=templates.begin();
										it!=templates.end();
										it++)
			{
				it->reset_ProportionCheck();
			}
		}

		bool current=blured.at<Vec1uchar>(y_dia,x_dia)[0]<threshold;

		if(dark != current)
		{
			dark=current;
			if(last_length!=-1)
			{
				Proportion prop;
				prop.length_c=current_length;
				prop.length_l=last_length;
				prop.proportion=(1-2*dark)*(double)last_length/(double)current_length;
				prop.x=x_dia;
				prop.y=y_dia;

				for(std::vector<MatchTempProfile>::iterator it=templates.begin();
								it!=templates.end();
								it++)
				{
					it->checkProportion(prop,blured,127);
				}
			}
			last_length=current_length;
			current_length=0;

		}
		current_length++;


		/////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////
		x_dia++;
		y_dia++;
	}



}

} /* namespace KinTo */
