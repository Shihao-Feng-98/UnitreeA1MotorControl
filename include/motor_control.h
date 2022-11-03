/*
Description: unitree motor control
Email: 13247344844@163.com
Author: Shihao Feng
Update time: 2022-11-03
*/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <eigen3/Eigen/Dense> 
using namespace Eigen;
#include "serialPort/SerialPort.h"
#include <robot_paras.h>
// #include <leg_kinematics.h>

class MotorControl
{
public:
    MotorControl(SerialPort* serial_ptr, unsigned short id, double offset);
    // ~MotorControl();

    bool stop(); // stop the motor
    bool run(double tau_d, double dq_d, double q_d, double kp = 0.1, double kd = 3.0); // joint PD control
    bool run(double tau_d); // pure force control
    friend std::ostream & operator<<(std::ostream &os, const MotorControl &motor); // overload opperator<<  as friend

    double temp; // the motor temperature
    double q; // the motor position w.r.t output shaft
    double dq; // the motor velocity w.r.t output shaft
    double tau; // the motor tau w.r.t output shaft

private:
    // check cmd safety
    bool _cmd_is_safe(double tau_d, double dq_d, double q_d, double kp, double kd); 
    void _extract(); // decode w.r.t output shaft
    bool _send_recv();

    SerialPort* _serial_ptr; // the serial port handle ptr
    MOTOR_send _motor_s; 
    MOTOR_recv _motor_r;
    double _offset; // the motor offset between actual zero and theoretical zero w.r.t output shaft
    bool _sta; // the state of send and receive
};


// class LegControl
// {
// public:
//     LegControl(SerialPort* serial_ptr, vector<double> offset_list, LegType leg);
//     ~LegControl();

//     bool stop(); // stop the leg
//     bool run(Vector3d tau_d, Vector3d dq_d, Vector3d q_d, 
//                 Vector3d kp = Vector3d(0.1,0.1,0.1), 
//                 Vector3d kd = Vector3d(3.0,3.0,3.0));
//     bool run(Vector3d tau_d);
//     // overload opperator<<  as friend
//     friend std::ostream & operator<<(std::ostream &os, const LegControl &leg); 
    
//     vector<double> temp_list;
//     Vector3d tau, dq, q;
//     Vector3d foot_force, foot_vel, foot_pos;
//     Matrix3d jacobian;

// private:
//     bool cmd_is_safe(Vector3d q_d); // tau_d, dq_d在MotorControl中检查
//     void extract();

//     SerialPort* serial_ptr;
//     vector<double> offset_list;
//     vector<MotorControl*> motors;
//     LegType leg_type;  
//     LegKinematicsBennett* kine_ptr; 
//     Vector3d axis_transfer; // transform from theoretical axis to motor axis
// };


#endif
