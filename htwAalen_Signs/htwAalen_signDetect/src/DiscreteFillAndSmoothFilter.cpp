#include "DiscreteFillAndSmoothFilter.h"

DiscreteFillAndSmoothFilter::DiscreteFillAndSmoothFilter(std::set<short> &discreteValues)
{
	// TODO Auto-generated constructor stub

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

int DiscreteFillAndSmoothFilter::absoluteStepDifference(short value1, short value2)
{
	return abs(std::distance(discreteValues.find(value1),discreteValues.find(value2)));
}
