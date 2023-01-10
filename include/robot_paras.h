#ifndef ROBOT_PARAS_H
#define ROBOT_PARAS_H

#include <math.h>

#define NUM_LEG 4
#define NUM_DOF 12
#define GRAVITY -9.81
#define DT 0.00333 // 3.33ms 300hz

// Unitree motor 
#define GEAR_RATIO 9.1 
#define SAFETY_FACTOR 0.8 
#define MOTOR_TAU_MAX 33.5 // Nm
#define MOTOR_VEL_MAX 21. // rad/s
// motors offset
#define OFFSET_FR0 1.3062
#define OFFSET_FR1 0.4464  // 差了一点
#define OFFSET_FR2 -0.6019

#define OFFSET_FL0 -1.3983
#define OFFSET_FL1 1.0543  // 差了一点
#define OFFSET_FL2 -0.4027

#define OFFSET_RR0 -0.8920
#define OFFSET_RR1 0.8409 // 差了一点
#define OFFSET_RR2 -0.6245

#define OFFSET_RL0 1.3551 
#define OFFSET_RL1 0.6754  // 差了一点
#define OFFSET_RL2 -0.4856
// Linkage limit
#define ETA_MIN 8./180.*M_PI  
#define ETA_MAX 90./180.*M_PI
#define X_ALONG_ANGLE_MIN -15./180.*M_PI
#define X_ALONG_ANGLE_MAX 105./180.*M_PI
#define Y_ALONG_ANGLE_LIMIT 80./180.*M_PI
/*  */
// --------------------
// | FR            RR |
// |    <---o         |
// |        |         |
// |        v         |
// | FL            RL |
// --------------------
enum class LegType{FR, FL, RR, RL};


#endif