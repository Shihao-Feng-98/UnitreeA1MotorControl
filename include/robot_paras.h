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
#define OFFSET_FR0 0. // todo 
#define OFFSET_FR1 0.
#define OFFSET_FR2 0.
#define OFFSET_FL0 0. 
#define OFFSET_FL1 0.
#define OFFSET_FL2 0.
#define OFFSET_RR0 0. // todo 
#define OFFSET_RR1 0.
#define OFFSET_RR2 0.
#define OFFSET_RL0 0. // todo 
#define OFFSET_RL1 0.
#define OFFSET_RL2 0.
// Linkage limit
#define ETA_MIN 8./180.*M_PI  
#define ETA_MAX 90./180.*M_PI
#define X_ALONG_ANGLE_MIN -15./180.*M_PI
#define X_ALONG_ANGLE_MAX 105./180.*M_PI
#define Y_ALONG_ANGLE_LIMIT 80./180.*M_PI

// --------------------
// | FR            RR |
// |    <---o         |
// |        |         |
// |        v         |
// | FL            RL |
// --------------------
enum class LegType{FR, FL, RR, RL};


#endif