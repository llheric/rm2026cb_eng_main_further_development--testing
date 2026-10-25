#ifndef _ROBOT_CONF_H_
#define _ROBOT_CONF_H_


// some special mode
#define CV_DEBUG 0

// for adjustment
#define CV_PID_ADJUST 0
// for new function test


// ******** begin macro of motor ID ******** //

#define CHA_RF_ID 1
#define CHA_LF_ID 2
#define CHA_LB_ID 3
#define CHA_RB_ID 4

//#define FRIC_L_ID 2 // left friction wheel
//#define FRIC_R_ID 1 // right friction wheel
//#define FEEDER_ID 1 // puller wheel
//#define PITCH_ID 5
//#define YAW_ID 6

// ******** end macro of motor ID ******** //

// ******** begin macro of joint ******** //

// Numbers of joint motors
#define JOINT_MOTOR_COUNT 6

// Bit 0 = motor ID 1, bit 5 = motor ID 6.
// Use 0x3Fu only after all six motors are connected; 0x3Eu tests IDs 2-6.

// Enable motor 5 only
//#define ARM_ACTIVE_MOTOR_MASK 0x10u

// Enable motor 4 only
// #define ARM_ACTIVE_MOTOR_MASK 0x08u

// Enable motor 6 only
// #define ARM_ACTIVE_MOTOR_MASK 0x20u

// Enable motor 1 only
// #define ARM_ACTIVE_MOTOR_MASK 0x01u

// Enable motor 2 only
// #define ARM_ACTIVE_MOTOR_MASK 0x02u

// Enable motor 3 only
// #define ARM_ACTIVE_MOTOR_MASK 0x04u


//Enable motor 5
//#define ARM_ACTIVE_MOTOR_MASK 0x10u

//Enable motors 5, 4
//#define ARM_ACTIVE_MOTOR_MASK 0x18u

//Enable motors 5, 4, 6
// #define ARM_ACTIVE_MOTOR_MASK 0x38u

//Enable motors 5, 4, 6, 1
//#define ARM_ACTIVE_MOTOR_MASK 0x39u

//Enable motors 5, 4, 6, 1, 2
#define ARM_ACTIVE_MOTOR_MASK 0x3Bu

//Enable motors 5, 4, 6, 1, 2, 3 
//#define ARM_ACTIVE_MOTOR_MASK 0x3Fu


#if ARM_ACTIVE_MOTOR_MASK == 0 || (ARM_ACTIVE_MOTOR_MASK & ~0x3Fu)
#error Invalid ARM_ACTIVE_MOTOR_MASK
#endif





// ******** end macro of joint ******** //

// ******** begin macro of chassis ******** //

#define CHASSIS_ON 1

#define MECANUM 1
#define OMNI 0

#define CHASSIS_TYPE OMNI	// choose the type of chassis

// ID of motors

#define ACC_ALLOWED 10.0f // acceleration limit of chassis
#define DEC_ALLOWED 10.0f // deceleration limit of chassis

#define YAW_OFFSET_FORWARD 2875 // 0-8192, yaw value when gimbal is directed at armor 0, which is the positive forward direction of chassis

/* chassis maximum translation speed, unit is mm/s */
#define _CHASSIS_MAX_VX_SPEED 1500.0f
#define _CHASSIS_MAX_VY_SPEED 1500.0f
/* chassis maximum rotation speed, unit is degree/s */
#define _CHASSIS_MAX_VR_SPEED 920.0f

// PID of wheels
#define WHEEL_RF_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1
#define WHEEL_LF_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1
#define WHEEL_LB_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1
#define WHEEL_RB_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1

#define ROTATE_PID 		3, 0, 10, 1000, 1200, 200, 0, 1 // PID of following

// ******** end macro of chassis ******** //


// ******** begin macro of constants ******** //

#define PI 3.1416f
#define DEGREE_TO_MOTOR  22.7556f // 8192/360
#define RADIAN_TO_DEGREE 57.3f 		// 180/3.14
#define DEGREE_TO_RADIAN 0.0174f 	// 3.14f/180.0f
#define SQUARE_ROOT_OF_2 1.41421356f
#define SQUARE_ROOT_OF_2_DIVIDED_BY_2 0.70710678f

#define M3508_MAX_WHEEL_RPM 8740 // rotor speed
#define M3508_DECELE_RATIO 19.0f // M3508_decelerate_ratio
#define M2006_REDUCTION_RATIO 36.0f // M2006_decelerate_ratio
#define GM6020_MAX_CODE_DISC_VALUE 8192
#define GM6020_CODE_DISC_VALUE_CHANGE_PER_90_DEGREE 2048

// ******** end macro of constants ******** //


// ******** begin macro of robot Mechanical constants ******** //

#define _WHEEL_RADIUS      72.0f  // the radius of wheel(mm)
#define _WHEEL_PERIMETER   358.0f // the perimeter of wheel(mm)

#define _CHASSIS_WHEELTRACK  380.0f // wheel track distance(mm)
#define _CHASSIS_WHEELBASE   230.0f // wheel base distance(mm)
#define _CHASSIS_RADIUS      480.0f // radius of chassis(mm)

// ******** end macro of robot Mechanical constants ******** //

#include "comm.h"

static int top_speeds[10] = {80, 80, 80, 80, 80, 80, 80, 80, 80, 80};//top speed target for power prioritized chassis 1-10 level

#endif
