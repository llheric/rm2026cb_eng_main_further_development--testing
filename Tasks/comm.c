#include "comm.h"
#include "main_control.h"
#define AVERAGE_SHOOT_SPEED 0
extern UART_HandleTypeDef huart6;
extern main_control_mode_t main_control_mode;

// extern uint32_t board_comm_online;
// extern shooter_status_trans_t shooter_status_trans;

/*================================Begin of Judgement Data ===========================================================*/
auto_aimbot_t auto_aimbot;	 // rx data of aimbot
gimbal_state_t gimbal_state; // tx data of gimbal state
robot_state_t robot_state;	 // tx data of robot state

user_defined_data_t custom_controller; // data of custom controller

gimbal_log_data_t gimbal_log_data;

uint8_t aimbot_online;

uint8_t force_nx_transmit = 0;
uint8_t manual_online = 0;
float last_valid_speed = AVERAGE_SHOOT_SPEED;
uint32_t custom_comm_online = 0;

/*================================End of Judgement Data=============================================================*/

/*================================Begin of Judgement Handler Define =================================================*/
void custom_controller_handler(uint8_t *data)
{
	custom_comm_online += 1;
	memcpy(&custom_controller, data, sizeof(user_defined_data_t));
}

/*================================End of Judgement Handler Define====================================================*/

/*================================Begin of NX_data Handler Define =================================================*/
/*
uint32_t auto_receive_time;
void auto_aimbot_handler(uint8_t *data) {
	memcpy(&auto_aimbot, data, sizeof(auto_aimbot_t));
	aimbot_online = 50;
	auto_receive_time = time();
}
*/
/*================================End of NX_data Handler Define =================================================*/
/*
void send_gimbal_state() {
	gimbal_state.cur_yaw   = -1.0f * attitude.totalYaw;
	gimbal_state.cur_pitch = attitude.pitch;
	gimbal_state.cur_roll  = -1.0f * attitude.roll;
	if (board_comm_online) {
		gimbal_state.shoot_speed = shooter_status_trans.initial_speed;
	}
	gimbal_state.aim_enable  = (main_control_mode.gimbal_mode == GIMBAL_AUTO_AIM);

	comm_transmit_data(uart_PC.huart, 0xa5, CMD_GIMBAL_STATE, (uint8_t *)&gimbal_state, sizeof(gimbal_state_t));
}

void send_gimbal_log_data(gimbal_log_data_t *data) {
	comm_transmit_data(
		uart_PC.huart, 0xa5, CMD_GIMBAL_LOG_DATA,
		(uint8_t*)data, sizeof(gimbal_log_data_t)
	);
}
*/
void Comm_init(void)
{
	comm_init();
	comm_rx_init(cuart1.huart); // for NX
	comm_rx_init(cuart3.huart); // for old controller
	comm_rx_init(cuart6.huart); // for reference
	// ==== begin of reference handler ====

	// ==== end of reference handler ====

	// ==== begin of user handler ====
	// protocol_register(CMD_AUTO_AIMBOT_ID , auto_aimbot_handler);
	protocol_register(CUSTOM_ROBO_DATA, custom_controller_handler);
	// ==== end of user handler ====

	UART_Start_Receive_UnfixedLength_IT(&cuart1, 256);
	UART_Start_Receive_UnfixedLength_IT(&cuart6, 256);
	osThreadCreate("comm_task", Comm_task, NULL, osPriorityNormal, 128);
}

void Comm_task(void *args)
{
	while (1)
	{
		// if (aimbot_online) aimbot_online --;

		// manual_online = control.online || ((HAL_GetTick() - vt_data.vt_lastpack_time < 100) ? 1 : 0);

		// send_gimbal_state();

		delay(10);
	}
}
