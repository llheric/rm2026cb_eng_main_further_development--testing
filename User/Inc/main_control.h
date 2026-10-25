#ifndef _MAIN_CONTROL_H_
#define _MAIN_CONTROL_H_

#include "func_lib.h"
#include "buttons.h"
#include "robot_conf.h"
//#include "board_comm.h"
//#include "gimbal.h"
//#include "shoot_17mm.h"

extern vt_data_t vt_data;
extern auto_aimbot_t auto_aimbot;

#define PERFORMANCE_TEST_ON 1

/// for 3 states of control.triSwitch
#define SW_LEFT  0
#define SW_MID   1
#define SW_RIGHT 2 

#define YAW_INC -4.5f // increase speed of yaw when controlled by contoller
#define PIT_INC -3.0f // increase speed of pitch when controlled by contoller

// states of control mode of the whole robot
typedef enum{
	OFF_MODE,
	CONTROLLER_MODE1,
	CONTROLLER_MODE2,
	MOUSE_KEY_MODE,
	PITCH_GRA_COM_ADJUST_MODE,
} control_mode_e;

// states of gimbal mode
typedef enum {
	GIMBAL_IDLE,
	GIMBAL_CONTROLLER,
	GIMBAL_AUTO_AIM,
	GIMBAL_MOUSE_KEY_CONTROL,
	GIMBAL_PERFORMANCE_TEST,
} gimbal_mode_e;

typedef enum {
	CHASSIS_IDLE,
	CHASSIS_CONTROLLER,
	CHASSIS_MOUSE_KEY_CONTROL,
} chassis_mode_e;

typedef enum {
	SHOOT_17MM_IDLE,
	SHOOT_17MM_CONTROLLER,
	SHOOT_17MM_AUTO_AIM,
	SHOOT_17MM_MOUSE_KEY_CONTROL,
} shoot_17mm_mode_e;

typedef enum {
	CHASSIS_GIMBAL_SEPERATE = 0,
	CHASSIS_GIMBAL_FOLLOW_0_DEG = 1,
	CHASSIS_GIMBAL_FOLLOW_180_DEG = 2,
} cha_gim_mode_e;

typedef enum {
	ROTATE_OFF = 0,
	ROTATE_SLOW = 1,
	ROTATE_FAST = 2,
} rotate_mode_e;

typedef enum {
	SHOOT_OFF,
	SHOOT_SINGLE,
	SHOOT_BURST3,
	SHOOT_AUTO,
	SHOOT_AUTO_AIMBOT,
} shooter_mode_e;

typedef enum {
	FRIC_ON,
	FRIC_OFF,
} fric_mode_e;

// internal, information of mode including all modes
typedef struct {
	// informtion about mode
	control_mode_e control_mode;
	
	gimbal_mode_e gimbal_mode;
	chassis_mode_e chassis_mode;
	shoot_17mm_mode_e shoot_17mm_mode;
	
	cha_gim_mode_e cha_gim_mode;
	rotate_mode_e rotate_mode;
	fric_mode_e fric_mode;
	shooter_mode_e shooter_mode;
	
	uint8_t heat_spare;
} main_control_mode_t;

typedef struct {
	uint8_t key_w;
	uint8_t key_s;
	uint8_t key_a;
	uint8_t key_d;
	uint8_t key_ctrl;
	uint8_t key_shift;
	uint8_t key_q;
	uint8_t key_e;
	uint8_t key_c;
	uint8_t key_v;
	
	uint8_t mouse_left;
	uint8_t mouse_right;
} keyboard_t;

void main_control_task(void*);
extern main_control_mode_t main_control_mode;
//extern gimbal_command_t gimbal_cmd;

#endif

