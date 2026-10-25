/*	How to use?
		If you want to send message:
		1. add a struct typedef and header in board_comm.h
		2. define a function in Packer section
		3. register the packer with header and priority in board_comm_init()
		
		If you want to receive message:
		1. add a struct typedef and header in board_transmit.h
		2. define a variable in Data section
		3. define a function in Handler section
		4. register the handler with header in board_transmit_init()
		5. if you want a global variable for all files, declare extern in board_comm.h
		
		If the sent and received data is the same, then the header (command id) and struct define should be the same
		
		If you are confused with how to write a packer and handler, hope the demo_trans will help you understand
	*/
#include "board_comm.h"

uint32_t board_comm_online;

/*================================Begin of Data ===========================================================*/

demo_trans_t demo_trans;

control_cmd_trans_t control_cmd_trans;
volatile uint32_t control_cmd_rx_count;
volatile uint32_t control_cmd_rx_time;
volatile uint8_t control_cmd_rx_enable;

robot_status_upper_trans_t robot_status_upper_trans;
robot_status_judge_trans_t robot_status_judge_trans;
keyboard_trans_t keyboard_trans;
extern bool grip;
/*================================End of Data=============================================================*/

/*================================Begin of Packer Define =================================================*/
void demo_packer(uint8_t buf[8]) {
	demo_trans.gripper = grip;
	memcpy(buf, &demo_trans, sizeof(demo_trans_t));
}

void keyboard_packer(uint8_t buf[8]) {
	keyboard_trans.Key_W = KEY_W;
	keyboard_trans.Key_A = KEY_A;
	keyboard_trans.Key_S = KEY_S;
	keyboard_trans.Key_D = KEY_D;
	keyboard_trans.Key_Q = KEY_Q;
	keyboard_trans.Key_E = KEY_E;
	keyboard_trans.Key_Ctrl = KEY_CTRL;
	keyboard_trans.Key_Shift = KEY_SHIFT;
	memcpy(buf, &keyboard_trans, sizeof(keyboard_trans_t));
}
/*
void robot_status_judge_packer(uint8_t buf[8]) {
	robot_status_judge_trans.robot_id = ref_robot_status.robot_id;
	robot_status_judge_trans.shooter_barrel_cooling_value = ref_robot_status.shooter_barrel_cooling_value;
	robot_status_judge_trans.shooter_barrel_heat_limit = ref_robot_status.shooter_barrel_heat_limit;
	memcpy(buf, &robot_status_judge_trans, sizeof(robot_status_judge_trans_t));
}
*/
/*================================End of Packer Define =================================================*/

/*================================Begin of Handler Define =================================================*/
void demo_handler(uint8_t *data) {
	memcpy(&demo_trans, data, sizeof(demo_trans_t));
	board_comm_online = 200;
}

void control_cmd_handler(uint8_t *data) {
	// A classic CAN frame has at most 8 data bytes. This struct is 10 bytes.
	// Preserve the existing first-eight-byte layout without reading beyond it.
	memset(&control_cmd_trans, 0, sizeof(control_cmd_trans));
	memcpy(&control_cmd_trans, data, 8);
	control_cmd_rx_enable = data[0];
	control_cmd_rx_time = HAL_GetTick();
	control_cmd_rx_count++;
	board_comm_online = 200;
}

void robot_status_upper_handler(uint8_t *data) {
	memcpy(&robot_status_upper_trans, data, sizeof(robot_status_upper_trans_t));
	board_comm_online = 200;
}

/*================================End of Handler Define====================================================*/

void board_comm_init(void) {
	board_comm_online = 0;
	board_communicate_init();
	
	// ==== register packer ====
	comm_packer_register(CMD_DEMO_TRANS, demo_packer, PRIO_NORM);
	comm_packer_register(CMD_KEYBOARD, keyboard_packer, PRIO_NORM);
	//comm_packer_register(CMD_ROBOT_STATUS_JUDGE_TRANS, robot_status_judge_packer, PRIO_LOW);
	
	//comm_packer_register(CMD_CONTROL_CMD_TRANS, control_cmd_packer, PRIO_LOW);
	
	// ==== register handler ====
	//comm_handler_register(CMD_DEMO_TRANS, demo_handler);
	//comm_handler_register(CMD_CHASSIS_CMD_TRANS, chassis_cmd_handler);
	comm_handler_register(CMD_CONTROL_CMD_TRANS, control_cmd_handler);
	//comm_handler_register(CMD_ROBOT_STATUS_UPPER_TRANS, robot_status_upper_handler);
	
	osThreadCreate("board_transmit_task", board_comm_task, NULL, osPriorityNormal, 128);
}

void board_comm_task(void *args) {
	while (1) {
		if(board_comm_online) {
			board_comm_online --;
		}
		
		comm_sender();
		delay(1);
	}
}
