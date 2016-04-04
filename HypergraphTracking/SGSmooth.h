/******************************************************************************
 * Name : CSGSmooth
 * Date : 2015.05.04
 * Author : HAANJU.YOO
 * Version : 0.9
 * Description :
 *	code is based on 
 *	- 'http://www.ks.uiuc.edu/Research/vmd/mailing_list/vmd-l/att-13397/sgsmooth.C'
 *	- MATLAB's 'smooth.m'
 *
 ******************************************************************************/
#pragma once
#include <queue>

#define SGS_DEFAULT_SPAN (9)
#define SGS_DEFAULT_DEGREE (1)

struct Qset
{
	int rows;
	int cols;
	std::vector<double> Q;
	std::vector<double> Qbegin;
	std::vector<double> Qmid;
	std::vector<double> Qend;
};

class CSGSmooth
{
public:
	CSGSmooth(void);
	CSGSmooth(int span, int degree, std::vector<double> *initialData = NULL);
	~CSGSmooth(void);

	// setter
	int Insert(double newData);
	int Insert(std::vector<double> &queueNewData);
	int ReplaceBack(double replaceData);
	void PopBack(void);
	void SetQ(Qset &Q) { Qset_ = Q; };
	void SetPrecomputedQsets(std::vector<Qset> *Qsets) { precomputedQsets_ = Qsets; }
	void SetSmoother(std::deque<double> &data, std::deque<double> &smoothedData, int span, int degree);

	// getter
	double GetResult(int pos);
	std::vector<double> GetResults(int startPos, int endPos);
	void GetSmoother(std::deque<double> &data, std::deque<double> &smoothedData, int &span, int &degree);
	size_t size(void) const { return size_; }

	// others
	bool UpdateQ(int windowSize);
	static Qset CalculateQ(int windowSize, int degree = SGS_DEFAULT_DEGREE);
	
private:
	int Smoothing(void);	
	std::vector<double> Filter(std::vector<double> &coefficients, std::deque<double> &data, int startPos = 0);

	int span_;
	int degree_;
	size_t size_;
	Qset Qset_;
	std::deque<double> data_;
	std::deque<double> smoothedData_;	
	std::vector<Qset> *precomputedQsets_;
};

