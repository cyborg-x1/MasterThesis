#ifndef DISCRETEFILLANDSMOOTHFILTER_H_
#define DISCRETEFILLANDSMOOTHFILTER_H_

#include <set>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <math.h>

class DiscreteFillAndSmoothFilter
{
public:

	typedef enum
	{
		SeekPlausibleValue,
		SeekStepOrZero,
		SeekNonZero,
		SeekBackwardOrForwardStep,
	}states_seek_t;

	states_seek_t state_seek;

	short stored_value;
	std::set<short>	&discreteValues;

	DiscreteFillAndSmoothFilter(std::set<short> &discreteValues);

	virtual ~DiscreteFillAndSmoothFilter();

	/**
	 * This function is used to fill a mat
	 */
	short Fill(short input);

	/**
	 * This function resets the seek function
	 */
	short Seek(short input);

	/**
	 * Reset the seek state machine
	 */
	void resetSeek();


	/**
	 * This returns the absolute discrete step difference
	 */
	int absoluteStepDifference(short value1, short value2);
};

#endif /* DISCRETEFILLANDSMOOTHFILTER_H_ */
