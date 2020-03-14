#ifndef __RTC_H
#define __RTC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "stm32l4xx_hal_rtc.h"
	 
extern RTC_HandleTypeDef RTC_InitStruct;
	 
void RTC_Config(void);




#ifdef __cplusplus
}
#endif

#endif
