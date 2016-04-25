/*************************************************************/
/* MTimer.h														*/
/* Based on duration.h from code project by Laurent Guinnard	*/
/* Maxwell Thomas												*/
/* Fri June 29 2007
/* Updates:
/* Sunday Jun 19, 2011
/* 															*/
/* $Id$													        */
/*************************************************************/
#pragma once
#ifndef __MTIMER_H
#define __MTIMER_H

/**
* Event timer class
* Times an event and returns the duration in miliseconds
*/
#define _LINUX_

#if defined(_MSC_VER)
#include "WINDOWS.h"
#elif defined(_LINUX_)
#include <time.h>
#define LONGLONG __int64_t long long
#define LARGE_INTEGER __int64_t
#endif

class mTimer
{
protected:

	 LONGLONG m_llFrequency;
	 LONGLONG m_llCorrection;

	 LARGE_INTEGER m_liStart;
	 LARGE_INTEGER m_liStop;

#ifdef _LINUX_
    timespec start_t;
    timespec stop_t;
#endif


public:
mTimer(void)
{

	LARGE_INTEGER liFrequency;


#if defined(_MSC_VER)
	QueryPerformanceFrequency(&liFrequency);
	m_llFrequency = liFrequency.QuadPart;

	// Calibration
	Start();
	Stop();

	m_llCorrection = m_liStop.QuadPart-m_liStart.QuadPart;
#elif defined(_LINUX_)

#endif
}
/**
* Starts the event clock
*/
void Start(void)
{
#if defined(_MSC_VER)
	// Ensure we will not be interrupted by any other thread for a while
	//Sleep(0);
	QueryPerformanceCounter(&m_liStart);
#elif defined(_LINUX_)
    clock_gettime(CLOCK_MONOTONIC,&start_t);
#endif
}
/**
* Stops the event clock
*/
void Stop(void)
{
#if defined(_MSC_VER)
	QueryPerformanceCounter(&m_liStop);
#elif defined (_LINUX_)
    clock_gettime(CLOCK_MONOTONIC,&stop_t);
#endif
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

#if defined(_MSC_VER)
	return (double)(m_liStop.QuadPart-m_liStart.QuadPart-m_llCorrection)*1000000.0 / (m_llFrequency * 1000);
#elif defined(_LINUX_)
    return (double(stop_t.tv_sec)*1.0e3 + double(stop_t.tv_nsec/1000000.0))- (double(start_t.tv_sec)*1.0e3 + double(start_t.tv_nsec)/1000000.0);
#endif

return -9999.00;
}
};
#endif
