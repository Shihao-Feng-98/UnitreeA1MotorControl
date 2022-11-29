#include <periodic_rt_task.h>

PeriodicRtTask::PeriodicRtTask(string thread_name, int priority, void* (*thread_func)(void *), int CPU_id) 
{
    this->_thread_name = thread_name;
    if (_init_task(priority)){
        // 创建线程
        int ret = pthread_create(&_thread, &_attr, thread_func, NULL);
        if (ret) {
            _thread_failed = true;
            cout << thread_name << " create failed\n";
        }
        _thread_failed = false;
        cout << thread_name << " create successed\n";
    }
    else {
        _thread_failed = true;
        cout << thread_name << " init failed\n";
    }
    
    // 线程绑定CPU(i7-1260p (8+4)核心16线程)
    if (CPU_id >= 0 && CPU_id < 16) {
        CPU_ZERO(&_mask); // 清空集合
        CPU_SET(CPU_id, &_mask); // 将CPU核心加入到集合中
        if(pthread_setaffinity_np(pthread_self(), sizeof(_mask), &_mask) != 0){
            cout << thread_name << "cound not set CPU affinity\n";
        }
    }
}

bool PeriodicRtTask::_init_task(int priority)
{
    // 初始化参数
    int ret = pthread_attr_init(&_attr);
    if (ret) {
        cout << "init " << _thread_name << " attributes failed\n";
        return false;
    }
    // 设置调度策略和线程优先级
    ret = pthread_attr_setschedpolicy(&_attr, SCHED_FIFO); // 调度算法 RR 或者 FIFO
    if (ret) {
        cout << _thread_name << " setschedpolicy failed\n";
        return false;
    }
    // 设置优先级
    _param.sched_priority = priority;
    ret= pthread_attr_setschedparam(&_attr, &_param);
    if (ret) {
        cout << _thread_name << " setschedparam failed\n";
        return false;
    }
    // 设置继承 放弃继承
    ret = pthread_attr_setinheritsched(&_attr, PTHREAD_EXPLICIT_SCHED); 
    if (ret) {
        cout << _thread_name << " setinheritsched failed\n";
        return false;
    }
    return true;
}

PeriodicRtTask::~PeriodicRtTask()
{
    if (!_thread_failed)
    {
        pthread_join(_thread, NULL);
        cout << _thread_name << " joined\n";
    }
}