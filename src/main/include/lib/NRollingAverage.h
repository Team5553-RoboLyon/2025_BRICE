//from t-nor
#ifndef __NROLLINGAVERAGE_H
#define __NROLLINGAVERAGE_H

#include <cstdlib>
// ***************************************************************************************
// ***************************************************************************************
// **																					**
// **								NRollingAverage.h									**
// **																					**
// ***************************************************************************************
// ***************************************************************************************
class NdoubleRollingAverage // unnecessary now
{
public:
	NdoubleRollingAverage();
	NdoubleRollingAverage(const int table_size, const double initial_average = 0.0 );
	~NdoubleRollingAverage();

	void reset(const double initial_average = 0.0);
	double add(const double value);
	inline double get(){return m_average;}
	
	int	m_last;
	int	m_index;
	double			m_average;
	double			m_sum;
	double*			m_pdouble;
};

class NfloatRollingAverage
{
public:
	NfloatRollingAverage(const unsigned short table_size, const float initial_average = 0.0f );
	~NfloatRollingAverage();

	void reset(const float initial_average = 0.0f);
	float add(const float value);
	inline float get(){return m_average;}
	
	unsigned short	m_last;
	unsigned short	m_index;
	float			m_average;
	float			m_sum;
	float		   *m_pfloat;
};

class NlongRollingAverage
{
public:
	NlongRollingAverage(const unsigned short table_size, const long initial_average = 0.0f );
	~NlongRollingAverage();

	void reset(const long initial_average = 0.0f);
	long add(const long value);
	inline long get(){return m_average;}
	
	unsigned short	m_last;
	unsigned short	m_index;
	long			m_average;
	long			m_sum;
	long		   *m_plong;
};





#endif // __NROLLINGAVERAGE_H 