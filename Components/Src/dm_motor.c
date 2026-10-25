#include "dm_motor.h"
#include "os.h"
#include "string.h"
#include "can_driver.h"
#include <float.h>

extern CAN_HandleTypeDef hcan1, hcan2;

DMMotorRegistry DMMotor_Registry = {0};

DMMotor DMMotors_CAN1[16];
DMMotor DMMotors_CAN2[16];

#define DM_CLEARER_FLAG 0xFB
#define DM_ENABLE_FLAG  0xFC
#define DM_DISABLE_FLAG 0xFD
#define DM_ZEROPOS_FLAG 0xFE

static inline void DMMotor_AssertFail(const char* msg) {
	// TODO: log msg
	while(1);
	// What Fucking Thing are you doing
}

#define DM_ASSERT_MSG(expr, msg) do { if(!(expr)) DMMotor_AssertFail(msg); } while(0)

// ===================================== Begin Utility Functions =====================================

float Hex_To_Float(uint32_t *Byte) { return *((float*)Byte); }
uint32_t FloatTohex(float HEX) { return *( uint32_t *)&HEX; }

int float_to_uint(float x_float, float x_min, float x_max, int bits) {
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

CAN_HandleTypeDef* get_can_handle(DMMotor_CAN_t bus) {
	DM_ASSERT_MSG( bus == USE_CAN1 || bus == USE_CAN2, "DMMotor: CAN bus invalid");
	if (bus == USE_CAN1) return &hcan1;
	if (bus == USE_CAN2) return &hcan2;
	return NULL; // Should never reach here
}

DMMotor* get_motor_handle(DMMotor_CAN_t bus, uint16_t id) {
	DM_ASSERT_MSG( bus == USE_CAN1 || bus == USE_CAN2, "DMMotor: CAN bus invalid");
	DM_ASSERT_MSG( /*0 <= id &&*/ id <= 15, "DMMotor: Motor ID out of range (0-15)");
	if (bus == USE_CAN1) return &DMMotors_CAN1[id];
	if (bus == USE_CAN2) return &DMMotors_CAN2[id];
	return NULL; // Should never reach here
}

HAL_StatusTypeDef dm_motor_magic(uint16_t id, uint8_t flag, CAN_HandleTypeDef* hcan) {
	uint8_t DM_Magic_Cmd[8]	= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, flag};
	can_tx_status_t st = CAN_Driver_Send(hcan, id, 8, DM_Magic_Cmd);
	return (st == CAN_TX_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef read_motor_data(uint16_t id, uint8_t rid, CAN_HandleTypeDef* hcan) {
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	can_tx_status_t st = CAN_Driver_Send(hcan, 0x7FF, 4, data);
	return (st == CAN_TX_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef write_motor_data(uint16_t id, uint8_t rid, uint32_t value, CAN_HandleTypeDef* hcan) {
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, 0xff, 0xff, 0xff, 0xff};
	*((uint32_t*)(&data[4])) = value;
	can_tx_status_t st = CAN_Driver_Send(hcan, 0x7FF, 8, data);
	return (st == CAN_TX_OK) ? HAL_OK : HAL_ERROR;
}
/* Bounded retries: a missing motor must never hold STATE_INIT forever. */
static HAL_StatusTypeDef wait_register_valuef(
    uint16_t id, uint8_t reg, volatile float *target,
    CAN_HandleTypeDef *hcan, uint32_t delay_ms
) {
    for (uint32_t attempt = 0; attempt < 60; ++attempt) {
        if ((attempt % 5) == 0) {
            HAL_StatusTypeDef result = read_motor_data(id, reg, hcan);
            if (result != HAL_OK) return result;
        }
        osDelay(delay_ms);
        if (*target > 0.0f && *target <= FLT_MAX) return HAL_OK;
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef wait_register_valueu8(
    uint16_t id, uint8_t reg, volatile uint8_t *target,
    CAN_HandleTypeDef *hcan, uint32_t delay_ms
) {
    for (uint32_t attempt = 0; attempt < 60; ++attempt) {
        if ((attempt % 5) == 0) {
            HAL_StatusTypeDef result = read_motor_data(id, reg, hcan);
            if (result != HAL_OK) return result;
        }
        osDelay(delay_ms);
        if (*target != 0) return HAL_OK;
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef wait_motor_enabled(
    uint16_t id, DMMotorStatus *status, CAN_HandleTypeDef *hcan
) {
    for (uint32_t attempt = 0; attempt < 100; ++attempt) {
        HAL_StatusTypeDef result = dm_motor_magic(id, DM_ENABLE_FLAG, hcan);
        if (result != HAL_OK) return result;
        osDelay(5);
        if (status->state == Enabled) return HAL_OK;
        if (status->state >= OverVoltage) return HAL_ERROR;
    }
    return HAL_TIMEOUT;
}

// ==================================== End Utility Functions =====================================

// ===================================== Begin API Functions =====================================

void DMMotor_General_Init(void) {
	memset(DMMotors_CAN1, 0, sizeof(DMMotors_CAN1));
	memset(DMMotors_CAN2, 0, sizeof(DMMotors_CAN2));
	memset(&DMMotor_Registry, 0, sizeof(DMMotor_Registry));
	
	for (uint8_t i = 0; i < 16; i++) {
		DMMotors_CAN1[i].status.state = Disabled;
		DMMotors_CAN2[i].status.state = Disabled;

		DMMotors_CAN1[i].info.id = i;
		DMMotors_CAN2[i].info.id = i;

		DMMotors_CAN1[i].info.bus = USE_CAN1;
		DMMotors_CAN2[i].info.bus = USE_CAN2;
	}
}

HAL_StatusTypeDef DMMotor_Enable(DMMotor_CAN_t bus, uint16_t id) {
    CAN_HandleTypeDef *hcan = get_can_handle(bus);
    DMMotor *motor = get_motor_handle(bus, id);
    DMMotorInfo *info = &motor->info;
    DMMotorStatus *status = &motor->status;
    HAL_StatusTypeDef result;

    info->id = id;
    info->bus = bus;
    memset(&motor->command, 0, sizeof(motor->command));
    status->init_step = 1;
    result = dm_motor_magic(id, DM_CLEARER_FLAG, hcan);
    if (result != HAL_OK) return result;
    status->state = Disabled;
    osDelay(5);
    result = wait_motor_enabled(id, status, hcan);
    if (result != HAL_OK) return result;

    status->state = Initing;
    info->P_max = 0.0f;
    info->V_max = 0.0f;
    info->T_max = 0.0f;
    info->mode = 0;
    status->init_step = 2;
    result = wait_register_valuef(id, 0x15, &info->P_max, hcan, 5);
    if (result != HAL_OK) return result;
    status->init_step = 3;
    result = wait_register_valuef(id, 0x16, &info->V_max, hcan, 5);
    if (result != HAL_OK) return result;
    status->init_step = 4;
    result = wait_register_valuef(id, 0x17, &info->T_max, hcan, 5);
    if (result != HAL_OK) return result;
    status->init_step = 5;
    result = wait_register_valueu8(id, 0x0A, &info->mode, hcan, 5);
    if (result != HAL_OK) return result;
    if (info->mode != 1) return HAL_ERROR; // Commands below use MIT mode.

    status->init_step = 6;
    status->state = Initing;
    result = wait_motor_enabled(id, status, hcan);
    if (result != HAL_OK) return result;
    status->init_step = 7;
    return HAL_OK;
}

HAL_StatusTypeDef DMMotor_Disable(DMMotor_CAN_t bus, uint16_t id) {
	CAN_HandleTypeDef *hcan;

	hcan = get_can_handle(bus);
	DM_ASSERT_MSG( /*0 <= id &&*/ id <= 15, "DMMotor: Motor ID out of range (0-15)");
	
	return dm_motor_magic(id, DM_DISABLE_FLAG, hcan);
}

HAL_StatusTypeDef DMMotor_Restart(DMMotor_CAN_t bus, uint16_t id) {
	HAL_StatusTypeDef return_status;

	return_status = DMMotor_Disable(bus, id);
	if (return_status != HAL_OK) return return_status;

	return_status = DMMotor_Enable(bus, id);
	return return_status;
}

/* =====================================
 * Data Format:
 * *=============================*
 * | Bit 0  - 3  | ID            |
 * *=============================*
 * | Bit 4  - 7  | State         |
 * *=============================*
 * | Bit 8  - 23 | Position      |
 * *=============================*
 * | Bit 24 - 35 | Velocity      |
 * *=============================*
 * | Bit 36 - 47 | Torque        |
 * *=============================*
 * | Bit 48 - 55 | MOS Temp      |
 * *=============================*
 * 
 * - ID: Motor CAN ID (0-15)
 * - State: see DM_Motor_status_t
 * - Position: [-P_max, P_max] in degrees
 * - Velocity: [-V_max, V_max] in rad/s
 * - Torque: [-T_max, T_max] in N*m
 * 
 * ===================================== */

void DMMotor_ESCDataUnpack(DMMotor_CAN_t bus, const uint8_t* data){
	uint16_t id = data[0] & 0x0F;
	DMMotor *motor = get_motor_handle(bus, id);
	DMMotorInfo *info = &motor->info;
	DMMotorStatus *status = &motor->status;
	
	status->receive_time = HAL_GetTick();

	if ((data[0] & 0x0F) == id && data[1] == 0 && (data[2] == 0x33 || data[2] == 0x55)) {
		// Register Read Recieve
		switch (data[3]) {
			case 0x0A:
				// 1 for MIT
				info->mode = *((uint32_t*)(data+4));
				break;
			case 0x15:
				// Max Position [deg]
				info->P_max = Hex_To_Float((uint32_t*)(data+4)) * RAD_TO_DEGREE;
				break;
			case 0x16:
				// Max Velocity [rad/s]
				info->V_max = Hex_To_Float((uint32_t*)(data+4));
				break;
			case 0x17:
				// Max Torque [Nm]
				info->T_max = Hex_To_Float((uint32_t*)(data+4));
				break;
			default:
				// Unknown Register, What Fucking Thing are you doing
				break;
		}

		return;
	}
	
	// Motor Feedback Recieve
	status->state = (DMMotorState)(data[0] >> 4);

	uint16_t encode_position = (data[1] << 8) | data[2];
	uint16_t encode_velocity = (data[3] << 4) | (data[4] >> 4);
	uint16_t encode_torque   = ((data[4] & 0x0F) << 8) | data[5];
	status->Tmos  = (float)data[6];
	status->Tcoil = (float)data[7];
	// Decode Fix-point Numbers
	status->pos = uint_to_float(encode_position, -info->P_max, +info->P_max, 16);
	status->vel = uint_to_float(encode_velocity, -info->V_max, +info->V_max, 12);
	status->tor = uint_to_float(encode_torque, -info->T_max, +info->T_max, 12);
	
	// Update Total Pos System
	if(status->pos > +info->P_max / 2 && status->lastpos < -info->P_max / 2) status->refPosCycle --;
	if(status->pos < -info->P_max / 2 && status->lastpos > +info->P_max / 2) status->refPosCycle ++;
	status->lastpos  = status->pos;
	status->totalpos = status->pos + status->refPosCycle * info->P_max * 2;
}

/* =====================================
 * Data Format:
 * *=============================*
 * | Bit 0  - 15 | Position      |
 * *=============================*
 * | Bit 16 - 27 | Velocity      |
 * *=============================*
 * | Bit 28 - 39 | Kp            |
 * *=============================*
 * | Bit 40 - 51 | Kd            |
 * *=============================*
 * | Bit 52 - 63 | Torque        |
 * *=============================*
 * 
 * - Position: [-P_max, P_max] in degrees
 * - Velocity: [-V_max, V_max] in rad/s
 * - Kp: [0, 500]
 * - Kd: [0, 5]
 * - Torque: [-T_max, T_max] in N*m
 * 
 * ===================================== */
void pack_command_data(
	DMMotorInfo* info,
	DMMotorCommand* cmd,
	uint8_t data[8]
) {
	uint16_t pos_tmp = float_to_uint(cmd->target_pos, -info->P_max, +info->P_max, 16);
	uint16_t vel_tmp = float_to_uint(cmd->target_vel, -info->V_max, +info->V_max, 12);
	uint16_t tor_tmp = float_to_uint(cmd->target_tor, -info->T_max, +info->T_max, 12);
	uint16_t kp_tmp  = float_to_uint(cmd->Kp, KP_MIN, KP_MAX, 12);
	uint16_t kd_tmp  = float_to_uint(cmd->Kd, KD_MIN, KD_MAX, 12);

	// Data Layout
	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
}

HAL_StatusTypeDef DMMotor_SendCommand(DMMotor_CAN_t bus, uint16_t id) {
	uint8_t data[8];
	CAN_HandleTypeDef *hcan;
	DMMotor *motor;

	hcan  = get_can_handle(bus);
	motor = get_motor_handle(bus, id);
	
	DMMotorInfo *info = &motor->info;
	DMMotorStatus *status = &motor->status;
	DMMotorCommand *cmd = &motor->command;

	// Check Motor State
	if (status->state != Enabled) return HAL_ERROR;
	
	// If position is used, refPosCycle must be zero
	if (status->refPosCycle != 0 && cmd->Kp > 0) {
		status->state = ParamErr;
		return HAL_ERROR;
	}
	
	// Check Command Parameters
	if (cmd->target_pos > info->P_max || cmd->target_pos < -info->P_max ||
			cmd->target_vel > info->V_max || cmd->target_vel < -info->V_max ||
			cmd->target_tor > info->T_max || cmd->target_tor < -info->T_max
	) {
		status->state = ParamErr;
		return HAL_ERROR;
	}
	
	// Check Control Gains
	if (cmd->Kp > KP_MAX || cmd->Kp < KP_MIN || 
	    cmd->Kd > KD_MAX || cmd->Kd < KD_MIN
	) {
		status->state = ParamErr;
		return HAL_ERROR;
	}

	// Pack Command Data	
	pack_command_data(info, cmd, data);

	// Send CAN Message
	can_tx_status_t st = CAN_Driver_Send(hcan, info->id, 8, data);
	if (st != CAN_TX_OK && st != CAN_TX_QUEUE_FULL) {
		status->state = CommFail;
		return HAL_ERROR;
	}
	
	cmd->transmit_time = HAL_GetTick();
	return HAL_OK;
}

void DMMotor_SetTorque(DMMotor *motor, float tor) {
	motor->command.target_tor = tor;
}

void DMMotor_SetCommand(DMMotor *motor, float pos, float vel, float tor, float Kp, float Kd) {
	motor->command.target_pos = pos;
	motor->command.target_vel = vel;
	motor->command.target_tor = tor;
	motor->command.Kp         = Kp;
	motor->command.Kd         = Kd;
}

// ===================================== End API Functions =====================================

// ==================================== Begin Registry Functions =====================================

uint8_t DMMotor_Register(DMMotor* motor) {
	DMMotorRegistry* r = &DMMotor_Registry;
	for (uint8_t i = 0; i < r->count; i++) {
		if (r->motors[i] == motor) return 1; // Already registered
	}

	if (r->count >= DMMOTOR_MAX_REGISTRY) return 0; // Registry Full

	r->motors[r->count++] = motor;
	return 1; // Successfully registered
}

void DMMotor_SendAllCommands(void) {
	DMMotorRegistry* r = &DMMotor_Registry;
	for (uint8_t i = 0; i < r->count; i++) {
		DMMotor* motor = r->motors[i];

		// Initialization/recovery belongs to the motor task, never this task.
		if (motor->status.state != Enabled || motor->status.init_step != 7)
			continue;

		DMMotor_SendCommand(motor->info.bus, motor->info.id);
	}
}

// ==================================== End Registry Functions =====================================
