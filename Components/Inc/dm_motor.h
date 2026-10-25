#ifndef _COMPONENTS_DM_MOTOR_H
#define _COMPONENTS_DM_MOTOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define RAD_TO_DEGREE (180.f / 3.14159265f)
#define DEGREE_TO_RAD (3.14159265f / 180.f)

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define DMMOTOR_MAX_REGISTRY 12

typedef enum {
	USE_CAN1=1,
	USE_CAN2=2,
} DMMotor_CAN_t;

typedef enum {
	Disabled     = 0x0,
	Enabled      = 0x1,
	Initing      = 0x2,
	OverVoltage  = 0x8,
	UnderVoltage = 0x9,
	OverCurrent  = 0xA,
	OverMosTemp  = 0xB,
	OverCoilTemp = 0xC,
	CommFail     = 0xD,
	OverLoad     = 0xE,
	ParamErr     = 0xF,
} DMMotorState;

typedef struct {
	volatile DMMotorState state; // Written by CAN receive interrupt
	volatile uint8_t init_step; // 1=enable, 2=P, 3=V, 4=T, 5=mode, 6=final enable, 7=done
	float pos;                // Position [deg]
	float vel;                // Velocity [rad/s]
	float tor;                // Torque [N*m]
	float Tmos;               // MOSFET Temperature [°C]
	float Tcoil;              // Coil Temperature [°C]
	
	float lastpos;            // Last Position [deg]
	float totalpos;           // Total Position (multi-turn) [deg]
	int32_t refPosCycle;      // Reference position cycle count
	
	uint32_t receive_time;    // Timestamp of last data receive
} DMMotorStatus;

typedef struct {
	uint16_t id;              // CAN ID (0-15)
	DMMotor_CAN_t bus;        // CAN bus (CAN1 or CAN2)
	uint8_t mode;             // Control mode (1 for MIT)
	
	float P_max;              // Max position [deg]
	float V_max;              // Max velocity [rad/s]
	float T_max;              // Max torque   [N*m]
} DMMotorInfo;

typedef struct {
	float target_pos; // [deg]
	float target_vel; // [rad/s]
	float target_tor; // [N*m]
	float Kp;
	float Kd;
	uint32_t transmit_time; // Timestamp of last command transmit
} DMMotorCommand;

typedef struct {
	DMMotorStatus  status;
	DMMotorInfo    info;
	DMMotorCommand command;
} DMMotor;

typedef struct {
	DMMotor* motors[DMMOTOR_MAX_REGISTRY];
	uint8_t count;
} DMMotorRegistry;

extern DMMotor DMMotors_CAN1[16];
extern DMMotor DMMotors_CAN2[16];

void DMMotor_General_Init(void);

HAL_StatusTypeDef DMMotor_Enable(DMMotor_CAN_t bus, uint16_t id);
HAL_StatusTypeDef DMMotor_Disable(DMMotor_CAN_t bus, uint16_t id);
HAL_StatusTypeDef DMMotor_Restart(DMMotor_CAN_t bus, uint16_t id);

void DMMotor_SetTorque(DMMotor *motor, float tor);
void DMMotor_SetCommand(DMMotor *motor, float pos, float vel, float tor, float Kp, float Kd);

void DMMotor_ESCDataUnpack(DMMotor_CAN_t bus, const uint8_t* data);
HAL_StatusTypeDef DMMotor_SendCommand(DMMotor_CAN_t bus, uint16_t id);

uint8_t DMMotor_Register(DMMotor* motor);
void DMMotor_SendAllCommands(void);

#endif

