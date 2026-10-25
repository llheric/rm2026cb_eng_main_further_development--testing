#ifndef _COMPONENTS_CONTROLLER_H
#define _COMPONENTS_CONTROLLER_H

#include <stdint.h>

#define RC_BUF_SIZE 40

#define ROLLER1_MAX 0x94
#define SIGNAL_CONTROLLER 				0
#define SIGNAL_VIDEOTRANSMITTER 	1

/// L: control.triSwitch[0]  R: control.triSwitch[1]
#define L    0
#define R    1

/// for control.triSwitch
#define UP   1
#define MID  3
#define DOWN 2 

typedef struct _Mice {
  int16_t  x;             // [-32768, 32767] mouse VELOCITY of X axis, positive is rightward
  int16_t  y;             // [-32768, 32767] mouse VELOCITY of Y axis
  int16_t  z; // USELESS!!!  [-32768, 32767] mouse VELOCITY of Z axis
  uint8_t  leftButton;
  uint8_t  rightButton;
} Mice;

typedef struct _RemoteControlData {
  float     channel[4];    // (-1, 1)    centered channel pos in percentage
  Mice      mice;
  uint16_t  key;
  uint8_t   triSwitch[2];  // 
	uint8_t		signalSource;	 // SIGNAL_CONTROLLER or SIGNAL_VIDEOTRANSMITTER
  uint8_t   online;
	uint8_t   roller1,roller2;
} ControlData;

extern ControlData control;
extern volatile uint8_t rc_buf[RC_BUF_SIZE];
extern volatile uint32_t _lastRCPacketTime;

void Controller_Init(void);
void notifyRCDataUnpacker(void);

#endif
