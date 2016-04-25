/*************************************************************/
/* Timer.h														*/
/* Based on Mtimer.h by Maxwell Thomas							*/

/*************************************************************/
#pragma once
#ifndef __MTIMER_H
#define __MTIMER_H

/**
* Event timer class
* Times an event and returns the duration in miliseconds
*/


class Timer
{
protected:

	 LONGLONG m_llFrequency;
	 LONGLONG m_llCorrection;

	 LARGE_INTEGER m_liStart;
	 LARGE_INTEGER m_liStop;


    timespec start_t;
    timespec stop_t;



public:
mTimer(void)
{

	LARGE_INTEGER liFrequency;

}
/**
* Starts the event clock
*/
void Start(void)
{

    clock_gettime(CLOCK_MONOTONIC,&start_t);

}
/**
* Stops the event clock
*/
void Stop(void)
{

    clock_gettime(CLOCK_MONOTONIC,&stop_t);

}


/**Get a timestamp. Is equivalent to calling Stop() then calling GetDuration();
*/
double GetTimestamp(void)
{
	Stop();
	return GetDuration();
}

/**
* Returns the number of miliseconds between calls to Start and the latest Stop call. User must call stop() before each call to capture timestamp.
* @return	miliseconds
*/
double GetDuration(void) const
{


    return (double(stop_t.tv_sec)*1.0e3 + double(stop_t.tv_nsec/1000000.0))- (double(start_t.tv_sec)*1.0e3 + double(start_t.tv_nsec)/1000000.0);


return -9999.00;
}
};

