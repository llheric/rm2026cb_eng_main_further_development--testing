#include "dji_motor.h"

extern CAN_HandleTypeDef hcan1, hcan2;

DJIMotor CAN1_Motors[13];
DJIMotor CAN2_Motors[13];
int16_t CAN1_MotorCurrents[13];
int16_t CAN2_MotorCurrents[13];

static DJIMotorConfig motorConfigs[DJI_MOTOR_MAX_COUNT];
static uint8_t motorCount;
static uint8_t txGroupDirty;

DJIMotorHandle hmotors[DJI_MOTOR_MAX_COUNT];
DJIMotorTxGroup txGroups[DJI_MOTOR_MAX_GROUPS];
uint8_t txGroupCount;

static void DJIMotor_AssertFail(const char* msg) { while (1); } // Wtf, you'd better think why you get here

static void DJIMotor_BuildHandle(DJIMotorHandle *handle, const DJIMotorConfig *config) {
	// WARNINGS: phisical ID 5-8 is forbidden for use !!!

	if (config->escId < 1 || config->escId > 4) 
		DJIMotor_AssertFail("DJIMotor: escId out of range (1-4)");
	if (config->type != DJI_MOTOR_M2006 && config->type != DJI_MOTOR_M3508 && config->type != DJI_MOTOR_GM6020) 
		DJIMotor_AssertFail("DJIMotor: type invalid");
	if (config->bus != DJI_MOTOR_BUS_CAN1 && config->bus != DJI_MOTOR_BUS_CAN2)
		DJIMotor_AssertFail("DJIMotor: bus invalid");
	if (config->ctrlMode != DJI_MOTOR_CTRL_CURRENT) 
		DJIMotor_AssertFail("DJIMotor: Only CURRENT control mode supported");

	handle->type = config->type;
	handle->bus  = config->bus;
	handle->ctrlMode = config->ctrlMode;

	handle->escId = config->escId;
	
	// Feedback Standard ID
	if (handle->type == DJI_MOTOR_GM6020){
		// GM6020
		handle->fbkStdId = GM6020_BASE_ID + handle->escId;
	} else {
		// M20006 and M3508
		handle->fbkStdId = BASE_ID_OFFSET + handle->escId;
	}

	// Command Standard ID
	if (handle->type == DJI_MOTOR_GM6020){
		// GM6020 (phisical ID 1-4)
		handle->cmdStdId = 0x1FE;
	} else {
		// M2006 and M3508 (phisical ID 1-4)
		handle->cmdStdId = 0x200;
	}

	// Array index in Motor array and pointers
	if (handle->type == DJI_MOTOR_GM6020) {
		// GM6020 (phisical ID 1-4 map Motor ID 5-8)
		handle->arrayIndex = handle->escId + 4;
	} else {
		// M20006 and M3508 (phisical ID 1-4 map Motor ID 1-4)
		handle->arrayIndex = handle->escId;
	}

	switch (handle->bus) {
		case DJI_MOTOR_BUS_CAN1:
			handle->motor   = &CAN1_Motors[handle->arrayIndex];
			handle->current = &CAN1_MotorCurrents[handle->arrayIndex];
			break;
		case DJI_MOTOR_BUS_CAN2:
			handle->motor   = &CAN2_Motors[handle->arrayIndex];
			handle->current = &CAN2_MotorCurrents[handle->arrayIndex];
			break;
		default:
			DJIMotor_AssertFail("DJIMotor: bus invalid");
			break;
	}
}

static void DJIMotor_InitHandles(void) {
	for (size_t i = 0; i < motorCount; i++) {
		DJIMotor_BuildHandle(&hmotors[i], &motorConfigs[i]);
	}
}

static void DJIMotor_BuildTxGroups(void) {
	txGroupCount = 0;
	for (uint8_t i = 0; i < motorCount; i++) {
		DJIMotorHandle *handle = &hmotors[i];
		
		uint8_t g = 0;
		while (g < txGroupCount) {
			DJIMotorTxGroup *group = &txGroups[g];
			CAN_HandleTypeDef *hcan = (handle->bus == DJI_MOTOR_BUS_CAN1 ? &hcan1 : &hcan2);
			if (group->hcan == hcan && group->cmdStdId == handle->cmdStdId) break;
			
			g++;
		}

		if (g == txGroupCount) {
			if (txGroupCount >= DJI_MOTOR_MAX_GROUPS)
				DJIMotor_AssertFail("DJIMotor: too many Tx groups");
			// New group
			DJIMotorTxGroup *newGroup = &txGroups[txGroupCount++];
			newGroup->hcan = (handle->bus == DJI_MOTOR_BUS_CAN1 ? &hcan1 : &hcan2);
			newGroup->cmdStdId = handle->cmdStdId;
			newGroup->motorCount = 0;

			for (uint8_t s = 0; s < 4; s++) {
				newGroup->hmotors[s] = NULL;
			}
		}

		DJIMotorTxGroup *group = &txGroups[g];
		uint8_t slot = handle->escId - 1;
		if (slot >= 4)
			DJIMotor_AssertFail("DJIMotor: slot index overflow");
		if (group->hmotors[slot] != NULL)
			DJIMotor_AssertFail("DJIMotor: slot already occupied");

		group->hmotors[slot] = handle;
		if (slot + 1 > group->motorCount) {
			group->motorCount = slot + 1;
		}
	}
}

void DJIMotor_General_Init(void) {
	memset((void*)CAN1_Motors, 0, sizeof(DJIMotor)*13);
	memset((void*)CAN2_Motors, 0, sizeof(DJIMotor)*13);
	memset((void*)CAN1_MotorCurrents, 0, sizeof(int16_t)*13);
	memset((void*)CAN2_MotorCurrents, 0, sizeof(int16_t)*13);

	motorCount   = 0;
	txGroupDirty = 1;
	txGroupCount = 0;
}

void DJIMotor_Add(DJIMotorConfig *cfg) {
	if (motorCount >= DJI_MOTOR_MAX_COUNT)
		DJIMotor_AssertFail("DJIMotor: too many motors added");

	motorConfigs[motorCount++] = *cfg;
	txGroupDirty = 1;
}

void DJIMotor_BuildAll(void) {
	DJIMotor_InitHandles();
	DJIMotor_BuildTxGroups();
	txGroupDirty = 0;
}

/* =====================================
 * Data Format:
 * *=============================*
 * | Byte 0 - 1 | Angle (encode) |
 * *=============================*
 * | Byte 2 - 3 | Speed (rpm)    |
 * *=============================*
 * | Byte 4 - 5 | Current (unit) |
 * *=============================*
 * | Byte 6     | Temperature    |
 * *=============================*
 * | Byte 7     | Reserved       |
 * *=============================*
 * 
 * - Angle: [0,8191] for 0 to 360 degrees
 * - Speed: rotor speed in RPM
 * - Current: 
 *   - [-16384,16384] for -20A to 20A current (M3508)
 *   - [-10000,10000] for -10A to 10A current (M2006)
 *   - [-16384,16384] for  -3A to  3A current (GM6020)
 * - Temperature: celsius degree
 * =====================================
 */


void DJIMotor_ESCDataUnpack(uint8_t const *data, DJIMotor *motor) {
	motor->lastAngle = motor->angle;
	motor->angle = (uint16_t)data[0] << 8 | data[1];
	/* deltaAngle = angle - lastAngle (mod 8192) */
	uint16_t deltaAngle = (uint16_t)(motor->angle - motor->lastAngle) & 0x1fff;
	motor->speed = (int16_t)data[2] << 8 | data[3];
	motor->current = (int16_t)data[4] << 8 | data[5];
	motor->temperature = data[6];
	
	motor->totalAngle += deltaAngle;
	/* if deltaAngle >= 4096 */
	if (deltaAngle & 0x1000) {
		/* in which case the motor went backward */
		motor->totalAngle -= MOTOR_ENCODER_RESOLUTION;
	}
}

static void DJIMotor_SendGroup(const DJIMotorTxGroup *group) {
	uint8_t buf[8] = {0};

	for (uint8_t i = 0; i < group->motorCount; i++) {
		DJIMotorHandle *handle = group->hmotors[i];
		if (handle == NULL) continue;

		int16_t current = *(handle->current);
		buf[i << 1]     = current >> 8;
		buf[i << 1 | 1] = current & 0xFF;
	}

	CAN_Driver_Send(group->hcan, group->cmdStdId, 8, buf);
}

void DJIMotor_SendAll(void) {
	if (txGroupDirty) {
		return ;
		DJIMotor_AssertFail("DJIMotor: Tx group dirty, call BuildAll()");
	}

	for (uint8_t i = 0; i < txGroupCount; i++) {
		DJIMotor_SendGroup(&txGroups[i]);
	}
}

