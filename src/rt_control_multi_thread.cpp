#include <pthread.h> // -lpthread
#include <semaphore.h> // 信号量
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)
#include <unistd.h> // sleep
#include <memory>
#include <iostream>
using namespace std;

#include <C_timer.h>
#include <periodic_rt_task.h>
#include "motor_control.h"

// 信号量
sem_t g_sem;
// 互斥锁 
pthread_mutex_t g_mutex_FR = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_mutex_FL = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_mutex_RR = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_mutex_RL = PTHREAD_MUTEX_INITIALIZER;
// 控制计算完成，可以进行电机指令的收发
pthread_cond_t g_cond_ctrl_finished = PTHREAD_COND_INITIALIZER; 

// 标志位
bool g_stop_all = false; 
bool g_FR_finished = true;
bool g_FL_finished = true;
bool g_RR_finished = true;
bool g_RL_finished = true;

CTimer g_timer_total;
unique_ptr<MotorControl> motor0, motor1, motor2, motor3;
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
    CTimer timer_step;
    const double dt = 0.002;
    double time_since_run = 0.;
    int iteration = 0;
    int sval; 

    double T = 5.;
    double s = 0.;
    double q0_init, q1_init, q2_init, q3_init;
    q0_init = motor0->q;
    // q1_init = motor1->q;
    // q2_init = motor2->q;
    q3_init = motor3->q;
    
    g_timer_total.reset();
    // init
    timer_step.reset();

    pthread_mutex_lock(&g_mutex_FR); 
    pthread_mutex_lock(&g_mutex_FL); 
    pthread_mutex_lock(&g_mutex_RR); 
    pthread_mutex_lock(&g_mutex_RL); 
    
    // run task
    s = time_since_run/T;
    res0 = sin_func(q0_init, T, s);
    // res1 = sin_func(q1_init, T, s);
    // res2 = sin_func(q2_init, T, s);
    res3 = sin_func(q3_init, T, s);
    ++iteration;
    time_since_run += dt;
    // cout << "control_task: " << iteration << endl;
    
    pthread_cond_broadcast(&g_cond_ctrl_finished);
    g_FL_finished = false;
    g_FR_finished = false;
    g_RR_finished = false;
    g_RL_finished = false;
    pthread_mutex_unlock(&g_mutex_RL); 
    pthread_mutex_unlock(&g_mutex_RR); 
    pthread_mutex_unlock(&g_mutex_FL); 
    pthread_mutex_unlock(&g_mutex_FR); 

    // run periodic task
    while(time_since_run < T){
        // wait for child threads
        while(1){
            sem_getvalue(&g_sem, &sval);
            if (sval == 0) {break;}
        }

        ++iteration;
        time_since_run += dt;
        // wait the rest of period (us)
        while (timer_step.end() < dt*1000*1000);

        timer_step.reset();

        pthread_mutex_lock(&g_mutex_FR); 
        pthread_mutex_lock(&g_mutex_FL); 
        pthread_mutex_lock(&g_mutex_RR); 
        pthread_mutex_lock(&g_mutex_RL); 

        // run task
        s = time_since_run/T;
        res0 = sin_func(q0_init, T, s);
        // res1 = sin_func(q1_init, T, s);
        // res2 = sin_func(q2_init, T, s);
        res3 = sin_func(q3_init, T, s);
        // cout << "control_task: " << iteration << endl;

        // 通知所有子线程执行
        pthread_cond_broadcast(&g_cond_ctrl_finished);
        // flag reset
        g_FL_finished = false;
        g_FR_finished = false;
        g_RR_finished = false;
        g_RL_finished = false;
        // sem reset
        sem_post(&g_sem); // +1
        sem_post(&g_sem); // +1
        sem_post(&g_sem); // +1
        sem_post(&g_sem); // +1

        pthread_mutex_unlock(&g_mutex_RL); 
        pthread_mutex_unlock(&g_mutex_RR); 
        pthread_mutex_unlock(&g_mutex_FL); 
        pthread_mutex_unlock(&g_mutex_FR); 
    }
    g_stop_all = true;
    cout << "Iteration: " << iteration << endl;
    cout << "Desired time: " << time_since_run << " s" << endl;

    return nullptr;
}

// ======== FR Control Thread Function ========  
void* FR_control_loop(void* argc)
{
    // int iteration_FR = 0;

    // run periodic task
    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_FR); 

        // wait for condition
        while(g_FR_finished){ 
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_FR);
        }

        // run task
        motor0->run(0., res0(1), res0(0));
        // cout << "FR_task: " << ++iteration_FR << endl;

        // flag reset
        g_FR_finished = true;

        sem_wait(&g_sem); // -1

        pthread_mutex_unlock(&g_mutex_FR); 
    }
    // motor stop
    cout << "FR actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    motor0->stop();
    return nullptr;
}

// ======== FL Control Thread Function ========  
void* FL_control_loop(void* argc)
{
    // int iteration_FL = 0;

    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_FL); 

        while(g_FL_finished){
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_FL);
        }

        // run task
        // motor1->run(0., res1(1), res1(0));
        // cout << "FL_task: " << ++iteration_FL << endl;
 
        g_FL_finished = true;

        sem_wait(&g_sem); // -1
        
        pthread_mutex_unlock(&g_mutex_FL);
    }
    // motor stop
    cout << "FL actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    // motor1->stop();
    return nullptr;
}


// ======== RR Control Thread Function ========  
void* RR_control_loop(void* argc)
{
    // int iteration_RR = 0;

    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_RR); 

        while(g_RR_finished){
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_RR);
        }

        // run task
        // motor2->run(0., res2(1), res2(0));
        // cout << "RR_task: " << ++iteration_RR << endl;

        g_RR_finished = true;
        
        sem_wait(&g_sem); // -1

        pthread_mutex_unlock(&g_mutex_RR);
    }
    // motor stop
    cout << "RR actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    // motor2->stop();
    return nullptr;
}

// ======== RL Control Thread Function ========  
void* RL_control_loop(void* argc)
{
    // int iteration_RL = 0;

    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_RL); 

        while(g_RL_finished){
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_RL);
        }

        // run task
        motor3->run(0., res3(1), res3(0));
        // cout << "RL_task: " << ++iteration_RL << endl;

        g_RL_finished = true;
        
        sem_wait(&g_sem); // -1

        pthread_mutex_unlock(&g_mutex_RL);
    }
    // motor stop
    cout << "RL actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    motor3->stop();
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
    // SerialPort serial_port1("/dev/unitree_usb1");
    // SerialPort serial_port2("/dev/unitree_usb2");
    SerialPort serial_port3("/dev/unitree_usb3");

    motor0 = make_unique<MotorControl>(&serial_port0, 1, 0.);
    // motor1 = make_unique<MotorControl>(&serial_port1, 2, 0.);
    // motor2 = make_unique<MotorControl>(&serial_port2, 1, 0.);
    motor3 = make_unique<MotorControl>(&serial_port3, 2, 0.);

    sem_init(&g_sem, 0, 4);

    // creat threads
    PeriodicRtTask *FR_control_task = new PeriodicRtTask("[FR Control Thread]", 95, FR_control_loop, 5);
    PeriodicRtTask *FL_control_task = new PeriodicRtTask("[FL Control Thread]", 95, FL_control_loop, 6);
    PeriodicRtTask *RR_control_task = new PeriodicRtTask("[RR Control Thread]", 95, RR_control_loop, 7);
    PeriodicRtTask *RL_control_task = new PeriodicRtTask("[RL Control Thread]", 95, RL_control_loop, 8);
    PeriodicRtTask *main_control_task = new PeriodicRtTask("[Main Control Thread]", 95, main_control_loop, 9);

    sleep(1); 

    // join threads
    delete FR_control_task;
    delete FL_control_task;
    delete RR_control_task;
    delete RL_control_task;
    delete main_control_task;

    sem_destroy(&g_sem);

    return 0;
}