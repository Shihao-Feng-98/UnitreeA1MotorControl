/*
For motor calibration and test 
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

// gobal variable
unique_ptr<MotorControl> motor;
const double dt = 0.002;

Vector2d p2p_traj(const double &init, const double &target, const double &T, double s)
{
    Vector2d res;
    double s_new, ds_new;
    s_new = 3*pow(s,2) - 2*pow(s,3);
    ds_new = 6/T * s * (1-s);
    res(0) = init + s_new* (target - init);
    res(1) = ds_new * (target - init);
    return res;
}

void* main_loop(void* argc)
{
    CTimer timer_step;
    double t_since_run = 0;
    double T = 1.;
    double s = 0.;
    double init, target;
    Vector2d res;
    
    cout << "[Main Thread]: thread start\n";
    
    // motor zero init
    init = motor->q;
    target = 0.;
    while (t_since_run < T)
    {
        timer_step.reset();

        s = t_since_run/T;
        res = p2p_traj(init, target, T, s);
        motor->run(0., res(1), res(0));

        t_since_run += dt;
        while (timer_step.end() < dt*1000*1000);
    }
    t_since_run = 0.; // reset

    // set pos
    while (1)
    {
        cout << *motor << endl;
        cout << "enter target pos [-pi,pi] or 1000 to exit: ";
        cin >> target;

        if (target <= M_PI && target >= -M_PI)
        {
            init = motor->q;
            while (t_since_run < T)
            {
                timer_step.reset();

                s = t_since_run/T;
                res = p2p_traj(init, target, T, s);
                motor->run(0., res(1), res(0));

                t_since_run += dt;
                while (timer_step.end() < dt*1000*1000);
            }
            t_since_run = 0.; // reset
        }
        else if(target == 1000){break;}
    }
    
    // motor stop
    motor->stop();

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

    int motor_id = atoi(argv[1]);
    // SerialPort serial_port("/dev/unitree_usb0");
    SerialPort serial_port("/dev/ttyUSB0");
    motor = make_unique<MotorControl>(&serial_port, motor_id, 0.);

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 95, main_loop, 5);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}
