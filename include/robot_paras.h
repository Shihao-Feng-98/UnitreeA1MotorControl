#ifndef ROBOT_PARAS_H
#define ROBOT_PARAS_H

#include <math.h>
#include <vector>
using namespace std;
#include<eigen3/Eigen/Dense> 
using namespace Eigen;

#define GRAVITY -9.81
#define NUM_DOF 12
#define DT 0.0033 // s 控制周期300hz
// unitree motor 
#define GEAR_RATIO 9.1 // 减速比
#define SAFETY_FACTOR 0.8 // 安全系数
#define MOTOR_TAU_MAX 33.5 // Nm
#define MOTOR_VEL_MAX 21. // rad/s
// motor offset
#define OFFSET_FR0 0. // todo 
#define OFFSET_FR1 0.
#define OFFSET_FR2 0.
#define OFFSET_FL0 0. // 记得修改
#define OFFSET_FL1 0.
#define OFFSET_FL2 0.
#define OFFSET_RR0 0. // todo 
#define OFFSET_RR1 0.
#define OFFSET_RR2 0.
#define OFFSET_RL0 0. // todo 
#define OFFSET_RL1 0.
#define OFFSET_RL2 0.
// linkage limit
#define ETA_MIN 8./180.*M_PI 
#define ETA_MAX 90./180.*M_PI


// --------------------
// | FR            RR |
// |    <---o         |
// |        |         |
// |        v         |
// | FL            RL |
// --------------------
enum class LegType{FR, FL, RR, RL};
// enum class GaitPhaseType{SWING, STANCE};
// enum class GaitType{WALK, TROT};

// class GaitParas
// {
// public:
//     double duty_factor[4];
//     double raletive_phase[4];
// };


// enum class RobotMode{STOP, STAND, LOCOMOTION}; // todo 状态机设计模式
// enum class GaitStateType{TOUCH_DOWN, LIFT_OFF, EARLY_TOUCH_DOWN};

// struct UsrHighCmd
// {
//     RobotMode mode_d;
//     GaitType gait_d;
//     double speed[3]; // vx vy wz
//     double pose[3]; // roll pitch yaw
//     double height; // body height
// };

struct LegStates
{
    // feed back states
    Vector3d tau, dq, q;
    Vector3d foot_vel, foot_pos;
    Matrix3d jacobian;
    // desired states (cmd)
    Vector3d tau_d, dq_d, q_d;
    Vector3d foot_force_d, foot_vel_d, foot_pos_d;
    // 默认构造
    LegStates()
    {
        tau.setZero(); dq.setZero(); q.setZero();
        foot_vel.setZero(); foot_pos.setZero();
        jacobian.setIdentity();
        tau_d.setZero(); dq_d.setZero(); q_d.setZero();
        foot_force_d.setZero(); foot_vel_d.setZero(); foot_pos_d.setZero();
    }
};


#endif
