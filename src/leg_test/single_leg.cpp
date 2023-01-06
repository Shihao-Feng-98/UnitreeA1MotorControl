/*
Example for real-time leg control with one thread
*/

#include <iostream>
#include <vector>
#include <memory> // unique_ptr c++11 make_unique c+14
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)
#include <unistd.h> // sleep
using namespace std;

#include "C_timer.h"
#include "periodic_rt_task.h"
#include "motor_control.h"
#include "traj_generator.h"

// gobal variable
unique_ptr<LegControl> leg_control;
const double dt = 0.004; // 5ms

void* main_loop(void* argc)
{
    CTimer timer_step, timer_total;
    double t_since_run = 0;
    size_t iteration = 0;
    double T_traj = 5.;
    double T_init = 3.;
    double s = 0.;

    TrajGenerator traj_generator;
    Vector3d q_init, q_target;
    q_init = leg_control->q;
    vector<Vector3d> res(3);

    cout << "[Main Thread]: thread start\n";
    // init
    while (t_since_run < T_init)
    {
        timer_step.reset();

        s = t_since_run/T_init;
        res = traj_generator.p2p_traj(q_init, Vector3d::Zero(), T_init, s);

        leg_control->run(Vector3d::Zero(), res[1], res[0]);

        t_since_run += dt;
        while (timer_step.end() < dt*1000*1000);
    }
    t_since_run = 0.;

    // timer_total.reset();
    // while (t_since_run < T_traj)
    // {
    //     timer_step.reset();

    //     s = t_since_run/T_traj;
    //     q_target << 0., 0., 0.;
    //     res = traj_generator.p2p_traj(Vector3d::Zero(), q_target, T_traj, s);

    //     leg_control->run(Vector3d::Zero(), res[1], res[0]);
        
    //     ++iteration;
    //     t_since_run += dt;
    //     while (timer_step.end() < dt*1000*1000);
    // }
    // cout << "time: " << timer_total.end()/1000 << " ms\n";
    // cout << "iteration: " << iteration << endl;
    
    // motor stop
    leg_control->stop();

    cout << "[Main Thread]: thread end\n";
    return nullptr;
}

int main(int argc, char **argv)
{
    /*
    mlockall 锁定进程中所有映射到地址空间的页
    MCL_CURRENT 已经映射的进程地址，MCL_FUTURE 将来映射的进程地址
    */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        cout << "mlockall failed: %m\n"; // 输入上一个函数的错误信息
        return -2;
    }

    SerialPort serial_port("/dev/unitree_usb0");
    vector<double> offset_vec{0.,0.,0.};
    leg_control = make_unique<LegControl>(&serial_port, offset_vec, LegType::FR);

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 95, main_loop);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}
