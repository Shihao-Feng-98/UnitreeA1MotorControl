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
#include "traj_generator.h"

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
unique_ptr<LegControl> leg_control_FR, leg_control_FL, leg_control_RR, leg_control_RL;
Vector3d q_d_FR, q_d_FL, q_d_RR, q_d_RL;
Vector3d dq_d_FR, dq_d_FL, dq_d_RR, dq_d_RL;

Vector2d sin_func(const double &init, const double &T, double s)
{
    Vector2d res;
    res(0) = init + M_PI/2 * sin(2*M_PI*s);
    res(1) = M_PI/2 * 2*M_PI / T * cos(2*M_PI*s);
    return res;
}

// ======== Main Control Thread Function ========  
void* main_control_loop(void* argc)
{   
    CTimer timer_step;
    const double dt = 0.004;
    double time_since_run = 0.;
    int iteration = 0;
    int sval; 

    double T_init = 3.;
    double s = 0.;
    TrajGenerator traj_generator;
    vector<Vector3d> res_FR(3), res_FL(3), res_RR(3), res_RL(3);
    Vector3d init_FR, init_FL, init_RR, init_RL;
    Vector3d target_FR, target_FL, target_RR, target_RL;
    init_FR = leg_control_FR->q;
    init_FL = leg_control_FL->q;
    init_RR = leg_control_RR->q;
    init_RL = leg_control_RL->q;
    target_FR << -60./180.*M_PI, -30./180.*M_PI, 30./180.*M_PI;
    target_FL << 60./180.*M_PI, -30./180.*M_PI, 30./180.*M_PI;
    target_RR << -60./180.*M_PI, -30./180.*M_PI, 30./180.*M_PI;
    target_RL << 60./180.*M_PI, -30./180.*M_PI, 30./180.*M_PI;

    g_timer_total.reset();
    // init
    timer_step.reset();
    pthread_mutex_lock(&g_mutex_FR); 
    pthread_mutex_lock(&g_mutex_FL); 
    pthread_mutex_lock(&g_mutex_RR); 
    pthread_mutex_lock(&g_mutex_RL); 
    
    // run task
    s = time_since_run/T_init;
    if (s > 1.) {s = 1.;}
    res_FR = traj_generator.p2p_traj(init_FR, target_FR, T_init, s);
    dq_d_FR = res_FR[1];
    q_d_FR = res_FR[0];
    res_FL = traj_generator.p2p_traj(init_FL, target_FL, T_init, s);
    dq_d_FL = res_FL[1];
    q_d_FL = res_FL[0];
    res_RR = traj_generator.p2p_traj(init_RR, target_RR, T_init, s);
    dq_d_RR = res_RR[1];
    q_d_RR = res_RR[0];
    res_RL = traj_generator.p2p_traj(init_RL, target_RL, T_init, s);
    dq_d_RL = res_RL[1];
    q_d_RL = res_RL[0];

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
    while(time_since_run < T_init){
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
        // lock
        pthread_mutex_lock(&g_mutex_FR); 
        pthread_mutex_lock(&g_mutex_FL); 
        pthread_mutex_lock(&g_mutex_RR); 
        pthread_mutex_lock(&g_mutex_RL); 
        // run task
        s = time_since_run/T_init;
        if (s > 1.) {s = 1.;}
        res_FR = traj_generator.p2p_traj(init_FR, target_FR, T_init, s);
        dq_d_FR = res_FR[1];
        q_d_FR = res_FR[0];
        res_FL = traj_generator.p2p_traj(init_FL, target_FL, T_init, s);
        dq_d_FL = res_FL[1];
        q_d_FL = res_FL[0];
        res_RR = traj_generator.p2p_traj(init_RR, target_RR, T_init, s);
        dq_d_RR = res_RR[1];
        q_d_RR = res_RR[0];
        res_RL = traj_generator.p2p_traj(init_RL, target_RL, T_init, s);
        dq_d_RL = res_RL[1];
        q_d_RL = res_RL[0];
        // cout << "control_task: " << iteration << endl;
        // broadcast all child thread
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
        // unlock
        pthread_mutex_unlock(&g_mutex_RL); 
        pthread_mutex_unlock(&g_mutex_RR); 
        pthread_mutex_unlock(&g_mutex_FL); 
        pthread_mutex_unlock(&g_mutex_FR); 
    }
    g_stop_all = true;
    // cout << "Iteration: " << iteration << endl;
    // cout << "Desired time: " << time_since_run << " s" << endl;

    return nullptr;
}

// ======== FR Control Thread Function ========  
void* FR_control_loop(void* argc)
{
    // run periodic task
    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_FR); 
        // wait for condition
        while(g_FR_finished){ 
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_FR);
        }
        // run task
        leg_control_FR->run(Vector3d::Zero(), dq_d_FR, q_d_FR);

        // flag reset
        g_FR_finished = true;
        sem_wait(&g_sem); // -1
        pthread_mutex_unlock(&g_mutex_FR); 
    }
    // leg stop
    // cout << "FR actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    leg_control_FR->stop();
    return nullptr;
}

// ======== FL Control Thread Function ========  
void* FL_control_loop(void* argc)
{
    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_FL); 
        while(g_FL_finished){
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_FL);
        }
        // run task
        leg_control_FL->run(Vector3d::Zero(), dq_d_FL, q_d_FL);
 
        g_FL_finished = true;
        sem_wait(&g_sem); // -1
        pthread_mutex_unlock(&g_mutex_FL);
    }
    // leg stop
    // cout << "FL actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    leg_control_FL->stop();
    return nullptr;
}


// ======== RR Control Thread Function ========  
void* RR_control_loop(void* argc)
{
    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_RR); 
        while(g_RR_finished){
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_RR);
        }
        // run task
        leg_control_RR->run(Vector3d::Zero(), dq_d_RR, q_d_RR);

        g_RR_finished = true;        
        sem_wait(&g_sem); // -1
        pthread_mutex_unlock(&g_mutex_RR);
    }
    // leg stop
    // cout << "RR actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    leg_control_RR->stop();
    return nullptr;
}

// ======== RL Control Thread Function ========  
void* RL_control_loop(void* argc)
{
    while(!g_stop_all){
        pthread_mutex_lock(&g_mutex_RL); 
        while(g_RL_finished){
            pthread_cond_wait(&g_cond_ctrl_finished, &g_mutex_RL);
        }
        // run task
        leg_control_RL->run(Vector3d::Zero(), dq_d_RL, q_d_RL);

        g_RL_finished = true;
        sem_wait(&g_sem); // -1
        pthread_mutex_unlock(&g_mutex_RL);
    }
    // leg stop
    // cout << "RL actual time: " << g_timer_total.end()/1000/1000 << " s\n";
    leg_control_RL->stop();
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

    SerialPort serial_port_FR("/dev/unitree_usb1");
    SerialPort serial_port_FL("/dev/unitree_usb3");
    SerialPort serial_port_RR("/dev/unitree_usb2");
    SerialPort serial_port_RL("/dev/unitree_usb0");
    vector<double> offset_FR{OFFSET_FR0, OFFSET_FR1, OFFSET_FR2};
    vector<double> offset_FL{OFFSET_FL0, OFFSET_FL1, OFFSET_FL2};
    vector<double> offset_RR{OFFSET_RR0, OFFSET_RR1, OFFSET_RR2};
    vector<double> offset_RL{OFFSET_RL0, OFFSET_RL1, OFFSET_RL2};
    leg_control_FR = make_unique<LegControl>(&serial_port_FR, offset_FR, LegType::FR);
    leg_control_FL = make_unique<LegControl>(&serial_port_FL, offset_FL, LegType::FL);
    leg_control_RR = make_unique<LegControl>(&serial_port_RR, offset_RR, LegType::RR);
    leg_control_RL = make_unique<LegControl>(&serial_port_RL, offset_RL, LegType::RL);

    sem_init(&g_sem, 0, 4);

    // creat threads
    PeriodicRtTask *FR_control_task = new PeriodicRtTask("[FR Control Thread]", 95, FR_control_loop, 4);
    PeriodicRtTask *FL_control_task = new PeriodicRtTask("[FL Control Thread]", 95, FL_control_loop, 5);
    PeriodicRtTask *RR_control_task = new PeriodicRtTask("[RR Control Thread]", 95, RR_control_loop, 6);
    PeriodicRtTask *RL_control_task = new PeriodicRtTask("[RL Control Thread]", 95, RL_control_loop, 7);
    PeriodicRtTask *main_control_task = new PeriodicRtTask("[Main Control Thread]", 95, main_control_loop, 3);

    sleep(2); 

    // join threads
    delete FR_control_task;
    delete FL_control_task;
    delete RR_control_task;
    delete RL_control_task;
    delete main_control_task;

    sem_destroy(&g_sem);

    return 0;
}
