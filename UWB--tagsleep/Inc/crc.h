#ifndef __CRC_H
#define __CRC_H
#include "string.h"
#include "stdint.h"

uint16_t GetCRC(uint8_t *pdata, uint32_t start, uint32_t length);
uint8_t CheckCRC(uint16_t crc, uint8_t *pdata, uint32_t start, uint32_t length);

#endif
