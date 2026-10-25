#ifndef _BOARD_COMM_H
#define _BOARD_COMM_H

#include "func_lib.h"
#include "robot_conf.h"
#include "os.h"
#include "imu.h"
#include "board_communicate.h"
#include "video_trans.h"

/*=========================================Begin of Users' Data TypeDef ========================================= */
#define CMD_DEMO_TRANS 0x0412
typedef struct __attribute__((packed)) {
	uint8_t gripper;
} demo_trans_t;

#define CMD_CONTROL_CMD_TRANS 0x0405
typedef struct __attribute__((packed)) {
	uint8_t control_protection;
	int triswitch[2];
	uint8_t gripper;
} control_cmd_trans_t;

#define CMD_ROBOT_STATUS_UPPER_TRANS 0x0403
typedef struct __attribute__((packed)) {
	uint8_t controller_online;
} robot_status_upper_trans_t;

#define CMD_ROBOT_STATUS_JUDGE_TRANS 0x0404
typedef struct __attribute__((packed)) {
	uint8_t robot_id;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
} robot_status_judge_trans_t;

#define CMD_KEYBOARD 0x0406
typedef struct __attribute__((packed)) {
	uint8_t Key_W;
	uint8_t Key_A;
	uint8_t Key_S;
	uint8_t Key_D;
	uint8_t Key_Q;
	uint8_t Key_E;
	uint8_t Key_Ctrl;
	uint8_t Key_Shift;
} keyboard_trans_t;


/*=========================================End of Users' Data TypeDef   ========================================= */

void board_comm_init(void);
void board_comm_task(void *args);

extern control_cmd_trans_t control_cmd_trans;
extern volatile uint32_t control_cmd_rx_count;
extern volatile uint32_t control_cmd_rx_time;
extern volatile uint8_t control_cmd_rx_enable;

#endif

