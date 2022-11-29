/*
Description: Create real-time threads for periodic task
     Author: Shihao_Feng
      Email: 13247344844@163.com
*/

#ifndef PERIODIC_RT_TASK_H
#define PERIODIC_RT_TASK_H

#include <iostream>
using namespace std;
#include <string>
#include <pthread.h> // -lpthread

class PeriodicRtTask
{
public:
    // @paras: 线程名称 优先级 回调函数指针参数 绑定的CPU
    PeriodicRtTask(string thread_name, int priority, void* (*thread_func)(void *), int CPU_id = -1); 
    ~PeriodicRtTask();

private:
    bool _init_task(int priority);

    pthread_t _thread; // 线程句柄
    pthread_attr_t _attr; // 线程参数
    struct sched_param _param; // 调度参数
    string _thread_name;
    cpu_set_t _mask; // CPU核心的集合
    bool _thread_failed;
};

#endif