#ifndef __KEY_H
#define __KEY_H

//#include "gpio.h"
#include "main.h"
//#include "stm32l4xx_hal_gpio_ex.h"

#if defined(PCB_V2)

#define SW_PIN											GPIO_PIN_3
#define SW_PIN_Port									GPIOB
#define	SW_EXTI_IRQ									EXTI3_IRQn
#endif






void KeyInit(void);
void KeyCheck(void *pdata);


#endif
