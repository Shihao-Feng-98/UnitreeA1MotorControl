/*
Description: unitree motor control
Email: 13247344844@163.com
Author: Shihao Feng
Update time: 2022-11-03
*/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <iostream>
using namespace std;
#include <Eigen/Dense> 
using namespace Eigen;

#include "serialPort/SerialPort.h"
#include <robot_paras.h>

/*
Note: Both input and output of this class are referenced to 
        the theoretical motor output shaft zero
*/
class MotorControl
{
public:
    MotorControl(SerialPort* serial_ptr, unsigned short id, double offset);
    // ~MotorControl();

    bool stop(); // stop the motor
    bool run(double tau_d, double dq_d, double q_d, double kp = 0.1, double kd = 3.0); // joint PD control
    bool run(double tau_d); // pure force control
    friend ostream & operator<<(ostream &os, const MotorControl &motor); // overload opperator<<  as friend

    double temp; // the motor temperature
    double q; // the motor position w.r.t output shaft with offset
    double dq; // the motor velocity w.r.t output shaft with offset
    double tau; // the motor tau w.r.t output shaft with offset

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

/*
Note: Both input and output of this class are referenced to 
        the theoretical joint axis (see kinematics definition)
*/
class LegControl
{
public:
    LegControl(SerialPort* serial_ptr, vector<double> offset_vec, LegType leg);
    ~LegControl();

    bool stop(); // stop the leg
    bool run(const Vector3d &tau_d, const Vector3d &dq_d, const Vector3d &q_d, 
                Vector3d kp = Vector3d(0.1,0.1,0.1), 
                Vector3d kd = Vector3d(3.0,3.0,3.0));
    bool run(const Vector3d &tau_d);
    // overload opperator<<  as friend
    friend ostream & operator<<(ostream &os, const LegControl &leg); 
    
    vector<double> temp_vec;
    Vector3d tau, dq, q; // the theoretical axis

private:
    bool _cmd_is_safe(Vector3d q_d); // tau_d, dq_d在MotorControl中检查
    void _extract();

    SerialPort* _serial_ptr; // TODO: const string -> creat SerialPort
    vector<double> _offset_vec;
    LegType leg_type;  
    vector<MotorControl*> _motors;
    Matrix3d _pos_mapping, _vel_mapping; // transform from theoretical joint axis to motor 
    Matrix3d _pos_inv_mapping, _vel_inv_mapping; // transform from motor to theoretical joint axis 
};

#endif
