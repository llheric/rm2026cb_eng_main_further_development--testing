#ifndef __CRC_H
#define __CRC_H

#include <stdint.h>
#include <stddef.h>

uint8_t generate_crc8_checksum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
uint16_t generate_crc16_checksum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

uint8_t verify_crc8_checksum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_checksum(uint8_t* pchMessage, uint32_t dwLength);

void append_crc8_checksum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc16_checksum(uint8_t* pchMessage, uint32_t dwLength);

#endif
