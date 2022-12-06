#include <pthread.h> // -lpthread
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)
#include <unistd.h> // sleep
#include <memory>
#include <iostream>
using namespace std;

#include <C_timer.h>
#include <periodic_rt_task.h>
#include "motor_control.h"

pthread_barrier_t g_barr_start; // 线程开始标志位
bool g_stop_all = false; // 线程停止标志位

unique_ptr<MotorControl> motor0, motor1, motor2, motor3;
double q0_init, q1_init, q2_init, q3_init;
Vector2d res0, res1, res2, res3;

Vector2d sin_func(const double &init, const double &T, double s)
{
    Vector2d res;
    res(0) = init + M_PI/3 * sin(2*M_PI*s);
    res(1) = M_PI/3 * 2*M_PI / T * cos(2*M_PI*s);
    return res;
}

// ======== Main Control Thread Function ========  
void* main_control_loop(void* argc)
{   
    CTimer timer_step, timer_total;
    const double dt = 0.002; // 2ms
    double time_since_run = 0.;
    int iteration_main = 0;

    double s = 0.;
    double T = 3.;

    pthread_barrier_wait(&g_barr_start);

    timer_total.reset();
    // run periodic task
    while(time_since_run < T){
        timer_step.reset();

        // run task
        s = time_since_run/T;
        res0 = sin_func(q0_init, T, s);
        res1 = sin_func(q1_init, T, s);
        res2 = sin_func(q2_init, T, s);
        res3 = sin_func(q3_init, T, s);

        ++iteration_main;
        time_since_run += dt;
        // wait the rest of period (us)
        while (timer_step.end() < dt*1000*1000);
    }
    g_stop_all = true;
    cout << "Main actual time: " << timer_total.end()/1000/1000 << " s\n";
    cout << "Main desired time: " << time_since_run << " s\n";
    cout << "Main iteration: " << iteration_main << endl;

    return nullptr;
}

// ======== FR Control Thread Function ========  
void* FR_control_loop(void* argc)
{
    CTimer timer_FR;
    const double dt = 0.002; 
    int iteration_FR = 0;

    pthread_barrier_wait(&g_barr_start);

    // run periodic task
    while(!g_stop_all){
        timer_FR.reset();
        
        // run task
        motor0->run(0., res0(1), res0(0));

        ++iteration_FR;
        // wait the rest of period (us)
        while (timer_FR.end() < dt*1000*1000);
    }
    motor0->stop();
    cout << "FR iteration: " << iteration_FR << endl; 

    return nullptr;
}

// ======== FL Control Thread Function ========  
void* FL_control_loop(void* argc)
{
    CTimer timer_FL;
    const double dt = 0.002; 
    int iteration_FL = 0;

    pthread_barrier_wait(&g_barr_start);

    // run periodic task
    while(!g_stop_all){
        timer_FL.reset();
        
        // run task
        motor1->run(0., res1(1), res1(0));

        ++iteration_FL;
        // wait the rest of period (us)
        while (timer_FL.end() < dt*1000*1000);
    }
    motor1->stop();
    cout << "FL iteration: " << iteration_FL << endl; 

    return nullptr;
}

// ======== RR Control Thread Function ========  
void* RR_control_loop(void* argc)
{
    CTimer timer_RR;
    const double dt = 0.002; 
    int iteration_RR = 0;

    pthread_barrier_wait(&g_barr_start);

    // run periodic task
    while(!g_stop_all){
        timer_RR.reset();
        
        // run task
        motor2->run(0., res2(1), res2(0));

        ++iteration_RR;
        // wait the rest of period (us)
        while (timer_RR.end() < dt*1000*1000);
    }
    motor2->stop();
    cout << "RR iteration: " << iteration_RR << endl; 

    return nullptr;
}

// ======== RL Control Thread Function ========  
void* RL_control_loop(void* argc)
{
    CTimer timer_RL;
    const double dt = 0.002;
    int iteration_RL = 0;

    pthread_barrier_wait(&g_barr_start);

    // run periodic task
    while(!g_stop_all){
        timer_RL.reset();
        
        // run task
        motor3->run(0., res3(1), res3(0));

        ++iteration_RL;
        // wait the rest of period (us)
        while (timer_RL.end() < dt*1000*1000);
    }
    motor3->stop();
    cout << "RL iteration: " << iteration_RL << endl; 

    return nullptr;
}

int main(int argc, char** argv)
{
    /*
    mlockall 锁定进程中所有映射到地址空间的页
    MCL_CURRENT 已经映射的进程地址，MCL_FUTURE 将来映射的进程地址
    */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        cout << "mlockall failed: %m\n"; 
        return -2;
    }

    SerialPort serial_port0("/dev/unitree_usb0");
    SerialPort serial_port1("/dev/unitree_usb1");
    SerialPort serial_port2("/dev/unitree_usb2");
    SerialPort serial_port3("/dev/unitree_usb3");

    motor0 = make_unique<MotorControl>(&serial_port0, 1, 0.);
    motor1 = make_unique<MotorControl>(&serial_port1, 2, 0.);
    motor2 = make_unique<MotorControl>(&serial_port2, 1, 0.);
    motor3 = make_unique<MotorControl>(&serial_port3, 2, 0.);

    q0_init = motor0->q;
    q1_init = motor1->q;
    q2_init = motor2->q;
    q3_init = motor3->q;

    res0(0) = q0_init;
    res1(0) = q1_init;
    res2(0) = q2_init;
    res3(0) = q3_init;

    // 初始化屏障
    pthread_barrier_init(&g_barr_start, NULL, 5);

    // 创建线程
    PeriodicRtTask *main_control_task = new PeriodicRtTask("[Main Control Thread]", 95, main_control_loop, 3);
    PeriodicRtTask *FR_control_task = new PeriodicRtTask("[FR Control Thread]", 95, FR_control_loop, 4);
    PeriodicRtTask *FL_control_task = new PeriodicRtTask("[FL Control Thread]", 95, FL_control_loop, 5);
    PeriodicRtTask *RR_control_task = new PeriodicRtTask("[RR Control Thread]", 95, RR_control_loop, 6);
    PeriodicRtTask *RL_control_task = new PeriodicRtTask("[RL Control Thread]", 95, RL_control_loop, 7);

    // 等待线程创建
    sleep(1); 

    // 合并线程
    delete RL_control_task;
    delete RR_control_task;
    delete FL_control_task;
    delete FR_control_task;
    delete main_control_task;

    return 0;
}
