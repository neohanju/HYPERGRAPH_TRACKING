#include "Track.h"


CTrack::CTrack(int nIdx, int nTimeStart) 
	: id_(nIdx), timeStart_(nTimeStart), timeEnd_(0), cost_(0.0)
{
}


CTrack::~CTrack(void)
{
}
