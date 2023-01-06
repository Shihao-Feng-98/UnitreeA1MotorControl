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
unique_ptr<MotorControl> motor0, motor1, motor2, motor3;
const double dt = 0.005; // 5ms
double q0_init, q1_init, q2_init, q3_init;


Vector2d sin_func(const double &init, const double &T, double s)
{
    Vector2d res;
    res(0) = init + M_PI/2 * sin(2*M_PI*s);
    res(1) = M_PI/2 * 2*M_PI / T * cos(2*M_PI*s);
    return res;
}

void* main_loop(void* argc)
{
    CTimer timer_step, timer_total;
    double t_since_run = 0;
    size_t iteration = 0;
    double T = 5.;
    double s = 0.;

    Vector2d res0, res1, res2, res3;

    cout << "[Main Thread]: thread start\n";

    timer_total.reset();
    while (t_since_run < T)
    {
        timer_step.reset();

        // do something
        s = t_since_run/T;
        res0 = sin_func(q0_init, T, s);
        res1 = sin_func(q1_init, T, s);
        res2 = sin_func(q2_init, T, s);
        res3 = sin_func(q3_init, T, s);
        motor0->run(0., res0(1), res0(0));
        motor1->run(0., res1(1), res1(0));
        motor2->run(0., res2(1), res2(0));
        motor3->run(0., res3(1), res3(0));
        
        ++iteration;
        t_since_run += dt;
        while (timer_step.end() < dt*1000*1000);
    }
    cout << "time: " << timer_total.end()/1000 << " ms\n";
    // motor stop
    motor0->stop();
    motor1->stop();
    motor2->stop();
    motor3->stop();
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

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 95, main_loop);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}
