#ifndef __COMM_H__
#define __COMM_H__

#include "func_lib.h"
#include "robot_conf.h"
#include "os.h"

/*======================================Begin of Judgement Data TypeDef ===============================================*/
#define CMD_EXT_GAME_STATUS 0x0001
typedef struct __attribute__((packed))
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* 0x0002 */
#define CMD_EXT_GAME_RESULT 0x0002
typedef struct __attribute__((packed))
{
	uint8_t winner;
} ext_game_result_t;


/* 0x0003 */
#define CMD_EXT_GAME_ROBOT_HP 0x0003
typedef struct __attribute__((packed))
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t reserved1;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t reserved2;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;


/* 0x0101 */
#define CMD_EXT_EVENT_DATA 0x0101
typedef struct __attribute__((packed))
{
	uint32_t event_type;
} ext_event_data_t;

/* 0x0104 */
#define CMD_EXT_REFEREE_WARNING 0x0104
typedef struct __attribute__((packed))
{
 uint8_t level;
 uint8_t offending_robot_id;
	uint8_t count;
} ext_referee_warning_t;

/* 0x0105 */
#define CMD_EXT_DART_INFO 0x0105
typedef struct __attribute__((packed))
{
 uint8_t  dart_remaining_time; 
 uint16_t dart_info; 
} ext_dart_info_t;

/* 0x0201 */
#define CMD_EXT_ROBOT_STATUS 0x0201
typedef struct __attribute__((packed))
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_robot_status_t;
/* Robot Id:
		001 Hero 002 Engineer 003 004 005 Infantry 006 Aerial 007 Sentry (Red)
		101 Hero 102 Engineer 103 104 105 Infantry 106 Aerial 107 Sentry (Blue)
*/

/* 0x0202 */
#define CMD_EXT_POWER_HEAT_DATA 0x0202
typedef struct __attribute__((packed))
{
	uint16_t reserved1;
	uint16_t reserved2;
	float reserved3;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat; // heat of shooter
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

/* 0x0203 */
#define CMD_EXT_GAME_ROBOT_POS 0x0203
typedef struct __attribute__((packed))
{
 float x;
 float y;
 float angle;
} ext_game_robot_pos_t;

/* 0x0204 */
#define CMD_EXT_BUFF 0x0204
typedef struct __attribute__((packed))
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy; 
}ext_buff_t;

/* 0x0206 */
#define CMD_EXT_ROBOT_HURT 0x0206
typedef struct __attribute__((packed))
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
} ext_robot_hurt_t;

/* 0x0207 */
#define CMD_EXT_SHOOT_DATA 0x0207
typedef struct __attribute__((packed))
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t launching_frequency;
	float initial_speed;
} ext_shoot_data_t;

/* 0x0208 */
#define CMD_EXT_PROJECTILE_ALLOWANCE 0x0208
typedef struct __attribute__((packed))
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
	uint16_t projectile_allowance_fortress;
} ext_projectile_allowance_t;

/* 0x0209 */
#define CMD_EXT_RFID_STATUS 0x0209
typedef struct __attribute__((packed))
{
	uint32_t rfid_status;
} ext_rfid_status_t;

/* 0x020A */
#define CMD_EXT_DART_CLIENT_STATUS 0x020A
typedef struct __attribute__((packed))
{ 
	uint8_t dart_launch_opening_status;  
	uint8_t reserved;  
	uint16_t target_change_time;  
	uint16_t latest_launch_cmd_time; 
} ext_dart_client_cmd_t;

/* 0x020B */
#define CMD_EXT_GROUND_ROBOT_POSITION 0x020B
typedef struct __attribute__((packed))
{
 float hero_x;  
 float hero_y;  
 float engineer_x;  
 float engineer_y;  
 float standard_3_x;  
 float standard_3_y;  
 float standard_4_x;  
 float standard_4_y;  
 float reserved1;  
 float reserved2;
} ext_ground_robot_position_t;

/* 0x020C */
#define CMD_EXT_RADAR_MARK_PROGRESS 0x020C
typedef struct __attribute__((packed))
{
	uint8_t mark_progress;
} ext_radar_mark_data_t;

/* 0x020D */
#define CMD_EXT_SENTRY_INFO 0x020D
typedef struct __attribute__((packed))
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
} ext_sentry_info_t;

/* 0x020E */
#define CMD_EXT_RADAR_INFO 0x020E
typedef struct __attribute__((packed))
{
 uint32_t radar_info;
} ext_radar_info_t;

/* 0x0301 */
#define CMD_EXT_STUDENT_DATA 0x0301
typedef struct __attribute__((packed))
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
	uint8_t user_data[32]; // maximum to 133
} ext_student_interactive_header_data_t;

#define CMD_VT_CONTROL 0x0304
typedef struct __attribute__((packed))
{ 
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	uint8_t left_button_down; 
	uint8_t right_button_down; 
	uint16_t keyboard_value; 
	uint16_t reserved; 
} ext_robot_command_t;

/* 0x0303 */
#define CMD_MAP_MARK 0x0303
typedef struct __attribute__((packed)){
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	uint8_t cmd_source;
} ext_map_mark_t;

/* 0x0305 */
#define CMD_MAP_TARGET_MARK 0x0305
typedef struct __attribute__((packed))
{
 uint16_t target_robot_id; 
 float target_position_x; 
 float target_position_y;
} ext_map_target_robot_data_t;

/* 0x0307 */
#define CMD_MAP_PATH_DATA 0x0307
typedef struct __attribute__((packed))
{
 uint8_t intention; 
 uint16_t start_position_x; 
 uint16_t start_position_y; 
 int8_t delta_x[49]; 
 int8_t delta_y[49]; 
 uint16_t sender_id; 
} ext_map_path_t;

#define CMD_CUSTOM_MESSAGE 0x0308
typedef struct __attribute__((packed))
{
 uint16_t sender_id; 
 uint16_t receiver_id; 
 uint8_t user_data[30];
} ext_custom_message_t;

/*=========================================End of Judgement Data TypeDef============================================*/

/*=========================================Begin of Users' Data TypeDef ========================================= */
#define CMD_GIMBAL_STATE 0x1021
typedef struct __attribute__((packed))
{
	float cur_yaw;
	float cur_pitch;
	float cur_roll;
	float shoot_speed;
	uint8_t aim_enable;
} gimbal_state_t;

#define CMD_ROBOT_STATE 0x1022
typedef struct __attribute__((packed)){
  ext_game_robot_HP_t robot_HP;
	uint8_t robot_id;//+20 if entering organ mode
} robot_state_t;

#define CMD_AUTO_AIMBOT_ID 0x0503
typedef struct __attribute__((packed))
{
	float yaw;
	float pitch;
	float yaw_spd;
	float pitch_spd;
	float dist;
	uint8_t shoot;
	uint8_t enemy_id;
} auto_aimbot_t;

#define CMD_GIMBAL_LOG_DATA 0x0765
typedef struct __attribute__((packed)){
	uint8_t flag;
	uint32_t time_stamp;	// [ms]
	float ctrl_pitch_out; // [unit]
	float ctrl_yaw_out; 	// [unit]
	float p_pos_ref;			// [deg]
	float p_spd_ref;			// [rad/s]
	float y_pos_ref;			// [deg]
	float y_spd_ref;			// [rad/s]
	float p_pos_meas;			// [deg]
	float p_spd_meas;			// [rad/s]
	float y_pos_meas;			// [deg]
	float y_spd_meas;			// [rad/s]
} gimbal_log_data_t;

//Custom controller data
/* 0x0302 */  
#define CUSTOM_ROBO_DATA 0x0302 
typedef struct __attribute__((packed))
{
	int32_t J1;
	int32_t J2;
	int32_t J3;
	int32_t J4;
	int32_t J5;
	int32_t J6;
	
	uint8_t blank[6];
} user_defined_data_t;
/*=========================================End of Users' Data TypeDef   ========================================= */

#define uart_PC cuart1
#define uart_reference cuart6

extern uint8_t manual_online;
extern uint8_t aimbot_online;

void Comm_task(void *);
void Comm_init(void);

void send_gimbal_log_data(gimbal_log_data_t *data);

#endif
