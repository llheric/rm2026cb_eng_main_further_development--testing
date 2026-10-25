#define PROTOCOL_C

#include "protocol.h"
#include "comm.h"

#define PROTOCOL_RECORD_MAX_NUM 50

protocol_record_t records[PROTOCOL_RECORD_MAX_NUM];
default_handler_t default_handler = NULL;
uint16_t record_length = 0;

/**
 * @brief   decode data from a FIFO component
 * @param   fifo   a FIFO component that contains protocol frame data
 * @param   frame  decoded frame data pointer
 * @param   block  whether this function is behaviored block or non-block
 *          true   function will block when data in the FIFO component is not enough
 *          false  function will fail when data in the FIFO component is not enough
 * @return  true   frame successfully decoded
 *          false  function failed
 * @todo    return a specific error
 */
 
 
void protocol_unpack_data(uint8_t * buffer, uint16_t length,	unpack_data_t  * p_obj) {	
  uint8_t byte = 0;
	uint16_t count = 0;
  while (length - count)
  {
		byte = buffer[count];
		count++;
    switch(p_obj->unpack_step)
    {			
      case STEP_HEADER_SOF:
      {
        if(byte == UP_REG_ID || byte== DN_REG_ID)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[(p_obj->index)++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[(p_obj->index)++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[(p_obj->index)++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - PROTOCOL_HEADER_LEN - PROTOCOL_CRC_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[(p_obj->index)++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[(p_obj->index)++] = byte;
				
        if (p_obj->index == PROTOCOL_HEADER_LEN)
        {
          if ( verify_crc8_checksum(p_obj->protocol_packet, PROTOCOL_HEADER_LEN) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + p_obj->data_len + PROTOCOL_CRC_LEN))
        {
           p_obj->protocol_packet[(p_obj->index)++] = byte;  
        }
        if (p_obj->index >= (PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + p_obj->data_len + PROTOCOL_CRC_LEN))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

            if (verify_crc16_checksum(p_obj->protocol_packet, PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + p_obj->data_len + PROTOCOL_CRC_LEN) )
            {
                protocol_handle_frame(p_obj->protocol_packet);
            }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}



/**
 * @brief  dispatch recieved data to user-defined handler based on different CmdID
 * @param  frame  decoded frame data pointer
 */
void protocol_handle_frame(uint8_t *p_frame) {
	//protocol_frame_header_t *p_header = (protocol_frame_header_t*)p_frame;

	//uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + PROTOCOL_HEADER_LEN);
  uint8_t *data_addr   = p_frame + PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN;
	
	for (int i = 0; i < record_length; ++i) {
		if (records[i].cmd_id == cmd_id) {
			if (records[i].handler) records[i].handler(data_addr);
			return;
		}
	}
	if (default_handler) default_handler(cmd_id, data_addr);
}

/**
 * @brief  pack data into uint8_t array
 * @param  buffer       pointer to the array that storage the packed data
 * @param  sof          header sof
 * @param  cmdid        cmdid
 * @param  data         pointer to data
 * @param  data_length  the length of data (data segment only)
 */
void protocol_pack_data(uint8_t * buffer, uint8_t sof, uint16_t cmdid, const uint8_t * data, uint16_t data_length) {
	//static uint8_t seq = 0;
	uint16_t frame_length = PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + data_length + PROTOCOL_CRC_LEN;
	
	// construct frame_header
	protocol_frame_header_t * p_header = (protocol_frame_header_t *)buffer;
	p_header->sof = sof;
	p_header->data_length = data_length;
	p_header->seq = 0;//seq++;
	append_crc8_checksum(buffer, PROTOCOL_HEADER_LEN);

	// append CmdID
	memcpy(buffer + PROTOCOL_HEADER_LEN, (uint8_t *)&cmdid, PROTOCOL_CMD_LEN);

	// append data
	memcpy(buffer + PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN, data, data_length);

	// append frame_tail
	append_crc16_checksum(buffer, frame_length);
}


void		protocol_register(uint16_t cmd_id, handler_t handler) {
	if (record_length >= PROTOCOL_RECORD_MAX_NUM) return;
	records[record_length].cmd_id = cmd_id;
	records[record_length].handler = handler; 
	record_length++;
}

void		protocol_set_default_handler(default_handler_t handler) {
	default_handler = handler;
}
