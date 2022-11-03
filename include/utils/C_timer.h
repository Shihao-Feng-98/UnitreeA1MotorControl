/*
Description: The C style timer
Email: 13247344844@163.com
Author: Shihao Feng
Update time: 2022-11-03
*/

#ifndef C_TIMER_H 
#define C_TIMER_H

#include <iostream>
#include <sys/time.h>

class CTimer
{
public:
    inline CTimer() 
    {
        gettimeofday(&tv_start, NULL);
    }

    inline void reset() 
    {
        gettimeofday(&tv_start, NULL);
    }

    // Reset time and return the interval between two reset moments
    // The interval between last reset moment and this moment
    inline double end()
    {
        gettimeofday(&tv_end, NULL);
        double elapsed_us = (tv_end.tv_sec - tv_start.tv_sec) * 1000 * 1000 +
                            (tv_end.tv_usec - tv_start.tv_usec);
        return elapsed_us;
    }

private:
    struct timeval tv_start, tv_end;
};

#endif 