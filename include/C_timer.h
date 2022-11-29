/*
Description: C-style timer
     Author: Shihao_Feng
      Email: 13247344844@163.com
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
        gettimeofday(&_tv_start, NULL);
    }

    inline void reset() 
    {
        gettimeofday(&_tv_start, NULL);
    }

    // Reset time and return the interval between two reset moments
    // The interval between last reset moment and this moment
    inline double end()
    {
        gettimeofday(&_tv_end, NULL);
        double elapsed_us = (_tv_end.tv_sec - _tv_start.tv_sec) * 1000 * 1000 +
                            (_tv_end.tv_usec - _tv_start.tv_usec);
        return elapsed_us;
    }

private:
    struct timeval _tv_start, _tv_end;
};

#endif 