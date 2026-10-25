#ifndef _COMPONENTS_DJI_MOTOR_H
#define _COMPONENTS_DJI_MOTOR_H

#include <stdint.h>
#include "can_driver.h"

#define MOTOR_ENCODER_RESOLUTION 8192

#define BASE_ID_OFFSET 0x200

#define GM6020_BASE_ID BASE_ID_OFFSET + 0x04
#define M2006_BASE_ID  BASE_ID_OFFSET
#define M3508_BASE_ID  BASE_ID_OFFSET

#define DJI_MOTOR_MAX_COUNT 12
#define DJI_MOTOR_MAX_GROUPS 6

typedef enum {
	DJI_MOTOR_M2006  = 0,
	DJI_MOTOR_M3508  = 1,
	DJI_MOTOR_GM6020 = 2
} DJIMotorType;

typedef enum {
	DJI_MOTOR_BUS_CAN1 = 0,
	DJI_MOTOR_BUS_CAN2 = 1
} DJIMotorBus;

typedef enum {
	DJI_MOTOR_CTRL_CURRENT = 0,
	DJI_MOTOR_CTRL_VOLTAGE = 1, // Maybe not used (Only GM6020)
} DJIMotorCtrlMode;

typedef struct {
  int32_t   totalAngle;
  uint16_t  lastAngle;    // [0,8191] last rotor angle
  uint16_t  angle;        // [0,8191]  now rotor angle
  int16_t   speed;        // rotor speed in RPM
  int16_t   current;      // [-16384,16384]
  int8_t    temperature;  // celsius degree
} DJIMotor;

typedef struct {
	DJIMotorType     type;
	DJIMotorBus      bus;
	DJIMotorCtrlMode ctrlMode;

	uint8_t  escId; 	 		// phisical ID on the CAN bus (1-8)
	uint16_t cmdStdId; 		// standard ID for command sending
	uint16_t fbkStdId; 		// standard ID for feedback receiving

	uint8_t arrayIndex; 	// index in the CANx_Motors array

	DJIMotor *motor;			// pointer to the motor data structure
	int16_t  *current;		// pointer to the command current variable

	uint8_t txGroupIndex; // index of the tx group (0-5)
	uint8_t txGroupSlot;  // slot in the tx group (0-3)
} DJIMotorHandle;

typedef struct {
	uint8_t			 escId;		// phisical ID on the CAN bus (1-8)
	DJIMotorType type;
	DJIMotorBus  bus;
	DJIMotorCtrlMode ctrlMode;
} DJIMotorConfig;

typedef struct {
	CAN_HandleTypeDef *hcan;
	uint16_t cmdStdId;

	DJIMotorHandle *hmotors[4];
	uint8_t motorCount;
} DJIMotorTxGroup;

extern DJIMotor CAN1_Motors[];
extern DJIMotor CAN2_Motors[];
extern int16_t CAN1_MotorCurrents[];
extern int16_t CAN2_MotorCurrents[];

void DJIMotor_General_Init(void);
void DJIMotor_Add(DJIMotorConfig *cfg);
void DJIMotor_BuildAll(void);
void DJIMotor_ESCDataUnpack(uint8_t const *data, DJIMotor *motor);
void DJIMotor_SendAll(void);

#endif
