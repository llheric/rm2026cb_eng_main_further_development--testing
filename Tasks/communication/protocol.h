#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "fifo.h"
#include "crc.h"

/* Protocol Macros Definition ------------------------------------------------*/
//#ifdef PROTOCOL_C

#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id

//#endif

#define PROTOCOL_HEADER_LEN   sizeof(protocol_frame_header_t)
#define PROTOCOL_CMD_LEN      2    //cmdid bytes
#define PROTOCOL_CRC_LEN      2    //crc16 bytes
#define PROTOCAL_FRAME_MAX_SIZE  200

/* Protocol Type */

typedef struct __attribute__((packed)) {
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} protocol_frame_header_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct __attribute__((packed))
{
  protocol_frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

/* Protocol Function Type Definition -----------------------------------------*/

/* Protocol Function Prototypes -------------------------------------------*/

void protocol_unpack_data(uint8_t * buffer, uint16_t length,	unpack_data_t  * p_obj);
//uint8_t protocol_check_frame(uint8_t * frame, uint16_t length);
void    protocol_handle_frame(uint8_t * p_frame);
void    protocol_pack_data(uint8_t * buffer, uint8_t sof, uint16_t cmdid, const uint8_t * data, uint16_t data_length);

typedef void (*handler_t)(uint8_t* data);
typedef void (*default_handler_t)(uint16_t cmd_id, uint8_t* data);


typedef struct {
	uint16_t cmd_id;
	handler_t handler;
} protocol_record_t;

void		protocol_register(uint16_t cmd_id, handler_t handler);
void		protocol_set_default_handler(default_handler_t handler);

#endif
