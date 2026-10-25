#ifndef __VIDEO_TRANS_H
#define __VIDEO_TRANS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc.h"
#include "fifo.h"
#include "protocol.h"
#include "stm32f4xx_hal.h"


#define KEY_W      (vt_data.mouse_key.key & 0x0001)
#define KEY_S      (vt_data.mouse_key.key & 0x0002)
#define KEY_A      (vt_data.mouse_key.key & 0x0004)
#define KEY_D      (vt_data.mouse_key.key & 0x0008)
#define KEY_SHIFT  (vt_data.mouse_key.key & 0x0010)
#define KEY_CTRL   (vt_data.mouse_key.key & 0x0020)
#define KEY_Q      (vt_data.mouse_key.key & 0x0040)
#define KEY_E      (vt_data.mouse_key.key & 0x0080)
#define KEY_R      (vt_data.mouse_key.key & 0x0100)
#define KEY_F      (vt_data.mouse_key.key & 0x0200)
#define KEY_G      (vt_data.mouse_key.key & 0x0400)
#define KEY_Z      (vt_data.mouse_key.key & 0x0800)
#define KEY_X      (vt_data.mouse_key.key & 0x1000)
#define KEY_C      (vt_data.mouse_key.key & 0x2000)
#define KEY_V      (vt_data.mouse_key.key & 0x4000)
#define KEY_B      (vt_data.mouse_key.key & 0x8000)


#define VT_CONTROL_ENABLE 1

#define VT_SIZE       21
#define VT_CRC_LENGTH 2

#define VT_SOF_1 0xA9
#define VT_SOF_2 0x53

typedef enum {
  STEP_UNI_SOF_1 = 0,
  STEP_VT_SOF_2 = 1,
  STEP_VT_CRC16 = 2,
  STEP_COMM_LENGTH_LOW  = 3,
  STEP_COMM_LENGTH_HIGH = 4,
  STEP_COMM_FRAME_SEQ   = 5,
  STEP_COMM_HEADER_CRC8 = 6,
  STEP_COMM_DATA_CRC16  = 7,
} vt_unpack_step_e;

typedef struct __attribute__((packed)) {
  vt_unpack_step_e vt_unpack_step;
  uint8_t vt_packet[PROTOCAL_FRAME_MAX_SIZE];
  uint16_t index;
  uint16_t data_len;
} vt_unpack_data_t;

typedef struct {
  float ch_0;
  float ch_1;
  float ch_2;
  float ch_3;
  uint16_t mode_sw ;
  uint16_t pause ;
  uint16_t fn_1 ;
  uint16_t fn_2 ;
  uint16_t wheel ;
  uint8_t trigger;
} vt_controller_t;

typedef struct {
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left : 2;
  uint8_t mouse_right : 2;
  uint8_t mouse_middle : 2;
  uint16_t key;

} vt_mouse_key_t;

typedef struct {
  vt_controller_t controller;
  vt_mouse_key_t mouse_key;
  int vt_lastpack_time;
} vt_data_t;

extern vt_data_t vt_data;

void video_trans_unpack_data(uint8_t *buffer, uint16_t length, vt_unpack_data_t *p_obj);

#endif
