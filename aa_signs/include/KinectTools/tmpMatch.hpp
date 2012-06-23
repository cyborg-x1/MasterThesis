#ifndef TMPMATCH_H_
#define TMPMATCH_H_

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <math.h>
#include <vector>
#include <set>
#include <KinectTools/KinectTools.hpp>

namespace KinTo
{


class Proportion
{
public:
	double proportion;
	int x;
	int y;
	int length_l;
	int length_c;

	Proportion()
	:proportion(-1)
	,x(-1)
	,y(-1)
	,length_l(-1)
	,length_c(-1)
	{}
};

class Match
{
public:
	// matching pixels/compared pixels
	double congruence;

	// compared pixels / resized template size
	double coverage;

	//Name of the template
	std::string name;

	//The location of the match in the target picture
	cv::Rect rect;

	//The center location of the match
	cv::Point center;

	void printMatch() const
	{
		std::cout<<"__________________________________________________"<<name<<std::endl;
		std::cout<<"Name: "<<name<<std::endl;
		std::cout<<"Position/Width/Height: ("<<rect.x<<"|"<<rect.y<<") / "<<rect.width<<" / "<<rect.height<<std::endl;
		std::cout<<"C-Position: "<<"("<<center.x<<"|"<<center.y<<")"<<std::endl;
		std::cout<<"Coverage: "<<coverage<<std::endl;
		std::cout<<"Congruence: "<<congruence<<std::endl;
		std::cout<<"__________________________________________________"<<name<<std::endl;
	}
};

class MatchTempProfile
{
public:

	typedef std::pair<Proportion, Proportion> proportion_pair;

private:
	std::vector<Match> matches;
	std::vector<Proportion> proportions;
	cv::Mat tmp, gray;
	cv::Point begin, end;
	proportion_pair last_pair;
	double max_proportion_diff;
	double min_congruence;
	double min_coverage;
	std::string name;
	double min_target_size;
	double max_target_size;
	cv::Mat important_pixels;
	bool hasImportantPixels;

	void constructor_extension(int threshold);
	void templateMatching(Proportion fromTemplate, Proportion fromTarget,const cv::Mat& target,int threshold);
public:
	MatchTempProfile(const cv::Mat &tmp, std::string name, double minimal_target_size=.1, double max_target_size=0.25, uchar threshold=127, double max_proportion_diff=0.4, double min_coverage=0.6, double min_congruence=0.7);

	MatchTempProfile(const cv::Mat &tmp, const cv::Mat &tmp_imp, std::string name, double minimal_target_size=.1, double max_target_size=0.25, uchar threshold=127, double max_proportion_diff=0.4, double min_coverage=0.6, double min_congruence=0.8);

	void checkProportion(Proportion proportion, const cv::Mat &target, int threshold=127);

	void reset_ProportionCheck();

	const std::vector<Match>& getMatches()
	{
		return matches;
	}



	void clearMatches()
	{
		matches.clear();
	}
};



void proportionEnhancedTemplateMatching(std::vector<MatchTempProfile> &templates, const cv::Mat &target, uchar threshold=127);






} /* namespace KinTo */
#endif /* TMPMATCH_H_ */
