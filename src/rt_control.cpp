/*
Example for real-time motor control with one thread
*/

#include <iostream>
#include <vector>
#include <memory> // unique_ptr c++11 make_unique c+14
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)
#include <unistd.h> // sleep
using namespace std;

#include "utils/C_timer.h"
#include "utils/periodic_rt_task.h"
#include "motor_control.h"
#include "traj_generator.h"

// gobal variable
unique_ptr<MotorControl> motor1, motor2;
const double dt = 0.0033;
size_t iteration = 0;

void* main_loop(void* argc)
{
    CTimer timer_step;
    double t_since_run = 0;
    double T = 2.;
    double s = 0.;
    Vector3d q_start, q_end;
    vector<Vector3d> res;
    q_start << motor1->q, motor2->q, 0.;
    q_end = q_start + M_PI * Vector3d::Ones();

    Point2PointTrajGenerator traj_gen;

    cout << "[Main Thread]: thread start\n";

    while (t_since_run < T)
    {
        timer_step.reset();

        // do something
        s = t_since_run/T;
        res = traj_gen.linear_traj(q_start, q_end, T, s);
        motor1->run(0., res[1](0), res[0](0));
        motor2->run(0., res[1](1), res[0](1));

        iteration++;
        t_since_run += dt;
        while (timer_step.end() < dt*1000*1000);
    }
    // motor stop
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

    SerialPort serial_port("/dev/ttyUSB0");
    motor1 = make_unique<MotorControl>(&serial_port, 1, 0.);
    motor2 = make_unique<MotorControl>(&serial_port, 2, 0.);

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 95, main_loop);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}
