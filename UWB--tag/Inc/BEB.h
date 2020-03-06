#ifndef __BEB_H
#define __BEB_H
#include "stm32l4xx_hal.h"

void RNG_Init(void);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
uint32_t RNG_Get_RandomNum(void);
uint16_t RNG_Get_RandomRange(uint16_t min, uint16_t max);



#endif
