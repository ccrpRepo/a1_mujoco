#ifndef TIMECOUNTER_H
#define TIMECOUNTER_H

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

// 时间戳  微秒级， 需要#include <sys/time.h>
inline long long getSystemTime()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;
}
// 时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond()
{
    double time = getSystemTime() * 0.000001;
    return time;
}
// 等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(long long startTime, long long waitTime)
{
    // std::cout << "time consume: " << getSystemTime() - startTime << std::endl;
    if (getSystemTime() - startTime > waitTime)
    {
        // std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        //           << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while (getSystemTime() - startTime < waitTime)
    {
        usleep(50);
    }
}

#endif // TIMECOUNTER_H