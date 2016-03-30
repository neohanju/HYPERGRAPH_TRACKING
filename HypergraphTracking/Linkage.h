#pragma once

class CLinkage
{
public:
	CLinkage(void);
	~CLinkage(void);

	int id_;
	int from_;
	int to_;
	double cost_;
};

