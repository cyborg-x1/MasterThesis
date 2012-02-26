#ifndef DISCRETEFILLANDSMOOTHFILTER_H_
#define DISCRETEFILLANDSMOOTHFILTER_H_

#include <set>


class DiscreteFillAndSmoothFilter
{
public:


	typedef enum
	{
		SeekPlausibleValue,
		SeekStepOrZero,
		SeekNonZero,
		SeekBackwardOrForwardStep,
	}states_t;

	states_t state;
	short stored_value;
	std::set<short>	discreteValues;

	DiscreteFillAndSmoothFilter();
	virtual ~DiscreteFillAndSmoothFilter();


	void nextValue(short input,short &output);
	void reset();
};

#endif /* DISCRETEFILLANDSMOOTHFILTER_H_ */
