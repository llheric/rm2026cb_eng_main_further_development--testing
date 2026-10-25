#define VIDEO_TRANS_C
#include "video_trans.h"  

vt_data_t vt_data;

void vt_handle_frame(uint8_t * p_frame){
	vt_data.controller.ch_0= (((uint16_t)(p_frame[2] | (p_frame[3] << 8)) & 0x7ff) - 1024) / 660.0f;
	vt_data.controller.ch_1 = (((uint16_t)((p_frame[3] >> 3) | (p_frame[4] << 5)) & 0x7ff) - 1024) / 660.0f;
	vt_data.controller.ch_2 = (((uint16_t)((p_frame[4] >> 6) | (p_frame[5] << 2) | p_frame[6] << 10) & 0x7ff) - 1024) / 660.0f;
	vt_data.controller.ch_3 = (((uint16_t)((p_frame[6] >> 1) | (p_frame[7] << 7)) & 0x7ff) - 1024) / 660.0f;
	vt_data.controller.mode_sw = (p_frame[7] >> 4) & 0x3;
	vt_data.controller.pause = (p_frame[7] >> 6) & 0x1;
	vt_data.controller.fn_1 = (p_frame[7] >> 7) & 0x1;
	vt_data.controller.fn_2 = (p_frame[8]) & 0x1;
	vt_data.controller.wheel = (uint16_t)(p_frame[8] >> 1 | (p_frame[9] << 7)) & 0x7ff;
	vt_data.controller.trigger = (p_frame[9] >> 4) & 0x1;

	vt_data.mouse_key.mouse_x = (int16_t)(p_frame[10] | p_frame[11] << 8);
	vt_data.mouse_key.mouse_y = (int16_t)(p_frame[12] | p_frame[13] << 8);
	vt_data.mouse_key.mouse_z = (int16_t)(p_frame[14] | p_frame[15] << 8);
	vt_data.mouse_key.mouse_left = p_frame[16] >> 0 & 0x3;
	vt_data.mouse_key.mouse_right = p_frame[16] >> 2 & 0x3;
	vt_data.mouse_key.mouse_middle = p_frame[16] >> 4 & 0x3;
	vt_data.mouse_key.key = p_frame[17] | (p_frame[18] << 8);
}

uint16_t fail_cnt = 0;

void video_trans_unpack_data(uint8_t * buffer, uint16_t length, vt_unpack_data_t * p_obj){
  uint8_t byte   = 0;
  int count = 0;
  while ((int)length - (int)count > 0) {
    byte = buffer[count];
    switch (p_obj->vt_unpack_step){
      case STEP_UNI_SOF_1:
        if (byte == VT_SOF_1) {
					// Video trasmitter control pack
					if((length - count >= VT_SIZE) && buffer[count + 1] == VT_SOF_2){
						// Fast by-pass if the size requirement is satisfied
						// State remain unchanged to decode following packets
						if(verify_crc16_checksum(buffer + count, VT_SIZE)){
							vt_handle_frame(buffer + count);
							count += VT_SIZE;
							continue;
						}else{
							count += VT_SIZE;
							fail_cnt++;
							continue;
						}
					}else{
						// If it is not in a single transmission, continue byte-by-byte decode
						p_obj->vt_unpack_step = STEP_VT_SOF_2;
						p_obj->vt_packet[(p_obj->index)++] = byte;
					}
        } else if(byte == DN_REG_ID){
					// DJI comm data, usually custom controller pack
					// Jump to protocol.c version decode part
					p_obj->vt_unpack_step = STEP_COMM_LENGTH_LOW;
					p_obj->vt_packet[(p_obj->index)++] = byte;
				} else{
          p_obj->index = 0;
        }
        break;

      case STEP_VT_SOF_2:
        if (byte == VT_SOF_2) {
          p_obj->vt_unpack_step = STEP_VT_CRC16;
          p_obj->vt_packet[(p_obj->index)++] = byte;
        } else {
          p_obj->vt_unpack_step = STEP_UNI_SOF_1;
          p_obj->index          = 0;
        }
        break;

      case STEP_VT_CRC16:
        if (p_obj->index < VT_SIZE) {
          p_obj->vt_packet[(p_obj->index)++] = byte;
        }
        if (p_obj->index >= VT_SIZE) {
          p_obj->vt_unpack_step = STEP_UNI_SOF_1;
          p_obj->index          = 0;

          if (verify_crc16_checksum(p_obj->vt_packet, VT_SIZE)) {
            vt_handle_frame(p_obj->vt_packet);
          }else{
						fail_cnt++;
					}
        }
        break;
			/* 
			/  Code below are merged from original DJI protocol
			*/
				
      case STEP_COMM_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->vt_packet[(p_obj->index)++] = byte;
        p_obj->vt_unpack_step = STEP_COMM_LENGTH_HIGH;
      }break;
      
      case STEP_COMM_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->vt_packet[(p_obj->index)++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - PROTOCOL_HEADER_LEN - PROTOCOL_CRC_LEN))
        {
          p_obj->vt_unpack_step = STEP_COMM_FRAME_SEQ;
        }
        else
        {
          p_obj->vt_unpack_step = STEP_UNI_SOF_1;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_COMM_FRAME_SEQ:
      {
        p_obj->vt_packet[(p_obj->index)++] = byte;
        p_obj->vt_unpack_step = STEP_COMM_HEADER_CRC8;
      }break;

      case STEP_COMM_HEADER_CRC8:
      {
        p_obj->vt_packet[(p_obj->index)++] = byte;
				
        if (p_obj->index == PROTOCOL_HEADER_LEN)
        {
          if ( verify_crc8_checksum(p_obj->vt_packet, PROTOCOL_HEADER_LEN) )
          {
            p_obj->vt_unpack_step = STEP_COMM_DATA_CRC16;
          }
          else
          {
            p_obj->vt_unpack_step = STEP_UNI_SOF_1;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_COMM_DATA_CRC16:
      {
        if (p_obj->index < (PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + p_obj->data_len + PROTOCOL_CRC_LEN))
        {
           p_obj->vt_packet[(p_obj->index)++] = byte;  
        }
        if (p_obj->index >= (PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + p_obj->data_len + PROTOCOL_CRC_LEN))
        {
          p_obj->vt_unpack_step = STEP_UNI_SOF_1;
          p_obj->index = 0;

          if (verify_crc16_checksum(p_obj->vt_packet, PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + p_obj->data_len + PROTOCOL_CRC_LEN) )
          {
            protocol_handle_frame(p_obj->vt_packet);
          }
        }
      }break;
    }
    count++;
  }
  vt_data.vt_lastpack_time = HAL_GetTick();
}



