/*
Example for real-time motor control with one thread
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
unique_ptr<MotorControl> motor0,motor1,motor2;
const double dt = 0.004; // 4ms
double q0_init, q1_init, q2_init;

Vector2d sin_func(const double &init, const double &T, double s)
{
    Vector2d res;
    res(0) = init + 5./180.*M_PI * sin(2*M_PI*s);
    res(1) = 5./180.*M_PI * 2*M_PI / T * cos(2*M_PI*s);
    return res;
}

void* main_loop(void* argc)
{
    CTimer timer_step, timer_total;
    double t_since_run = 0;
    size_t iteration = 0;
    double T = 1.;
    double s = 0.;

    Vector2d res0, res1, res2;

    cout << "[Main Thread]: thread start\n";

    timer_total.reset();
    while (t_since_run < 300)
    {
        timer_step.reset();

        // do something
        s += dt/T;
        s = fmod(s, 1.);
        res0 = sin_func(q0_init, T, s);
        res1 = sin_func(q1_init, T, s);
        res2 = sin_func(q2_init, T, s);

        motor0->run(0., res0(1), res0(0), 0.1, 3.0);
        motor1->run(0., res1(1), res1(0), 0.1, 3.0);
        motor2->run(0., res2(1), res2(0), 0.1, 3.0);

        ++iteration;
        t_since_run += dt;
        while (timer_step.end() < dt*1000*1000);
    }
    cout << "time: " << timer_total.end()/1000 << " ms\n";
    // motor stop
    motor0->stop();
    motor1->stop();
    motor2->stop();
    cout << "iteration: " << iteration << endl;
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

    SerialPort serial_port_FR("/dev/unitree_usb1");

    motor0 = make_unique<MotorControl>(&serial_port_FR, 0, 1.9784);
    motor1 = make_unique<MotorControl>(&serial_port_FR, 1, 0.8156);
    motor2 = make_unique<MotorControl>(&serial_port_FR, 2, -0.6057);

    q0_init = motor0->q;
    q1_init = motor1->q;
    q2_init = motor2->q;

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 99, main_loop);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}
