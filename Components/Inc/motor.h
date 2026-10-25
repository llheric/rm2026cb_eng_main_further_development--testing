#ifndef _COMPONENTS_MOTOR_H
#define _COMPONENTS_MOTOR_H

// IMPORTANT: DISCARDED !!!!!!

#include <stdint.h>
#define ESC_BASE_ID 0x200
#define CURRENTS_1_TO_4  0x200
#define CURRENTS_5_TO_8  0x1fe // tmp
#define CURRENTS_9_TO_12 0x2fe // tmp

#define WULIE_CAPACITY  1

typedef struct {
  int32_t   totalAngle;
  uint16_t  lastAngle;    // [0,8191] last rotor angle
  uint16_t  angle;        // [0,8191] now rotor angle
  int16_t   speed;        // rotor speed in RPM
  int16_t   current;      // [-16384,16384] for -20A to 20A current
  int8_t    temperature;  // celsius degree
} CanMotor;

typedef struct {
  int32_t   totalAngle;
  uint16_t  lastAngle;          // [0,8191] last rotor angle
  uint16_t  angle;              // [0,8191] now rotor angle
  int16_t   measuredCurrent;
  int16_t   targetCurrent;
} CanMotor_6623;

typedef enum  {
  CAN1_MOTOR_1_TO_4  = 0x0000U,
  CAN1_MOTOR_5_TO_8  = 0x0001U,
  CAN1_MOTOR_9_TO_12 = 0x0002U,
  CAN2_MOTOR_1_TO_4  = 0x0003U,
  CAN2_MOTOR_5_TO_8  = 0x0004U,
  CAN2_MOTOR_9_TO_12 = 0x0005U
} CAN_Motor_Selection;

// void CanMotor_General_Init(void);
// void CanMotor_SendCurrent(CAN_Motor_Selection selection);
// void CanMotor_ESCDataUnpack(uint8_t const *data, CanMotor *motor);

#endif
