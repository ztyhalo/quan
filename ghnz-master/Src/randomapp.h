#ifndef __RANDOM_APP_H__
#define __RANDOM_APP_H__

#include "stm32l4xx_hal.h"

void RNG_Init(void);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
uint32_t RNG_Get_RandomNum(void);
uint16_t RNG_Get_RandomRange(uint16_t min, uint16_t max);



#endif /*__RANDOM_APP_H__*/
