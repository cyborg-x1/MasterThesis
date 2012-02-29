#include "DiscreteFillAndSmoothFilter.h"
#include "kinectStepLUT.h"
DiscreteFillAndSmoothFilter::DiscreteFillAndSmoothFilter(std::set<short> &discreteValues)
:discreteValues(discreteValues)
{
	// TODO Auto-generated constructor stub
	short i=0;
	for(std::set<short>::iterator it=discreteValues.begin();
			it!=discreteValues.end();it++)
	{
		short cur=*it;
		if(kinect_value_LUT[cur]!=i)
		{
			std::cout<<"ERROR: Wrong value in Lookup Table!"<<" Value:"<<kinect_value_LUT[cur]<<" i:"<<i<<std::endl;
		}
		i++;
	}
}

DiscreteFillAndSmoothFilter::~DiscreteFillAndSmoothFilter()
{
	// TODO Auto-generated destructor stub
}


short DiscreteFillAndSmoothFilter::Fill(short input)
{

}

short DiscreteFillAndSmoothFilter::Seek(short input)
{

}

void DiscreteFillAndSmoothFilter::resetSeek()
{

}

short DiscreteFillAndSmoothFilter::absoluteStepDifference(short value1, short value2)
{
	short v1 = kinect_value_LUT[value1];
	short v2 = kinect_value_LUT[value2];
#if __kinect_step_check_value
	if(v1 > -1 && v1 <= kinect_value_size && v2 > -1 && v2 <= kinect_value_size)
#endif
	{
		return abs(kinect_value_LUT[value1]-kinect_value_LUT[value2]);
	}
#if __kinect_step_check_value
	else
	{
		std::cout<<"ERROR: Unknown Step Value!!!";
		return -30000;
	}
#endif
}
