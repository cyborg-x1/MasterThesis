#ifndef FREQUENCY_HPP_
#define FREQUENCY_HPP_


#include <iostream>
#include <string>
#include <set>

using namespace std;


template <class T> class freq
{
private:
	T _val;
	mutable long _count;

public:
	freq(T value)
	{
		_val=value;
		_count=1;
	}

	long getCount() const
	{
		return _count;
	}

	T getValue() const
	{
		return _val;
	}

	void incCount() const
	{
		_count++;
	}

	bool operator< (const freq<T>& cmp) const
	{
		return _val< cmp._val;
	};
};

template <class VALUE>class frequency_set : public std::set< freq<VALUE> >
{
	typedef freq<VALUE> frequency;
	typedef _Rb_tree_const_iterator<frequency> set_frequency_iteratortype;
	typedef std::pair<set_frequency_iteratortype, bool> pair_frequency;

public:
	pair_frequency insert(const VALUE& __x)
	{
		pair_frequency p=set< freq<VALUE> >::insert(__x);
		if(!p.second)p.first->incCount();
		return p;
	}
	std::pair<VALUE, long> getMax()
	{
		long max=0;
		VALUE max_val;
		for(set_frequency_iteratortype it=this->begin(); it!=this->end() ; it++)
		{
			if((*it).getCount()>max)
			{
				max=(*it).getCount();
				max_val=(*it).getValue();
			}
		}
		return std::pair<VALUE, long>(max_val, max);
	}
};

#endif /* FREQUENCY_HPP_ */
