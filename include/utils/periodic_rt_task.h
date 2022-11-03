/*
Description: real-time thread for control
Email: 13247344844@163.com
Author: Shihao Feng
Update time: 2022-11-03
*/

#ifndef PERIODIC_RT_TASK_H
#define PERIODIC_RT_TASK_H

#include <iostream>
#include <string>
using namespace std;
#include <pthread.h> // -lpthread


class PeriodicRtTask
{
public:
    PeriodicRtTask(string thread_name, int priority, void* (*thread_func)(void *)); // 函数指针参数
    ~PeriodicRtTask();

private:
    bool init_task(int priority);

    pthread_t thread; // 线程句柄
    pthread_attr_t attr; // 线程参数
    struct sched_param param; // 调度参数
    string thread_name;
    bool thread_failed;
};

PeriodicRtTask::PeriodicRtTask(string thread_name, int priority, void* (*thread_func)(void *)) 
{
    this->thread_name = thread_name;
    if (init_task(priority))
    {
        // 创建线程
        int ret = pthread_create(&thread, &attr, thread_func, NULL);
        if (ret) {
            thread_failed = true;
            cout << thread_name << " create failed\n";
        }
        thread_failed = false;
        cout << thread_name << " create successed\n";
    }
    else 
    {
        thread_failed = true;
        cout << thread_name << " init failed\n";
    }
}

bool PeriodicRtTask::init_task(int priority)
{
    // 初始化参数
    int ret = pthread_attr_init(&attr);
    if (ret) {
        cout << "init " << thread_name << " attributes failed\n";
        return false;
    }
    // 设置调度策略和线程优先级
    ret = pthread_attr_setschedpolicy(&attr, SCHED_RR); // 调度算法 RR
    if (ret) {
        cout << thread_name << " setschedpolicy failed\n";
        return false;
    }
    // 设置优先级
    param.sched_priority = priority;
    ret= pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        cout << thread_name << " setschedparam failed\n";
        return false;
    }
    // 设置继承 放弃继承
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); 
    if (ret) {
        cout << thread_name << " setinheritsched failed\n";
        return false;
    }
    return true;
}

PeriodicRtTask::~PeriodicRtTask()
{
    if (!thread_failed)
    {
        pthread_join(thread, NULL);
        cout << thread_name << " joined\n";
    }
}

#endif