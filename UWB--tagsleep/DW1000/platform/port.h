/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond
 * tick timer. */
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1000

extern SPI_HandleTypeDef hspi1;	
	
void printf2(const char *format, ...);

typedef enum
{
    LED_PC6,
    LED_PC7,
    LED_PC8,
    LED_PC9,
    LED_ALL,
    LEDn
} led_t;

#define SPIx_PRESCALER			SPI_BAUDRATEPRESCALER_8

#define SPIx						    ((SPI_HandleTypeDef *)&hspi1)
#define SPIx_GPIO					  GPIOA
#define SPIx_CS					  	D_CS_Pin
#define SPIx_CS_GPIO				D_CS_GPIO_Port
#define SPIx_SCK					  GPIO_Pin_5
#define SPIx_MISO					  GPIO_Pin_6
#define SPIx_MOSI					  GPIO_Pin_7
#define SPI_I2S_FLAG_RXNE   SPI_FLAG_RXNE

#define DW1000_RSTn					D_RST_Pin
#define DW1000_RSTn_GPIO		D_RST_GPIO_Port

#define DECARSTIRQ                  D_RST_Pin
#define DECARSTIRQ_GPIO             D_RST_GPIO_Port
#define DECARSTIRQ_EXTI             EXTI_LINE_3
#define DECARSTIRQ_EXTI_IRQn        EXTI3_IRQn

#define DECAIRQ                     D_IRQ_Pin
#define DECAIRQ_GPIO                D_IRQ_GPIO_Port
#define DECAIRQ_EXTI                EXTI_LINE_0
#define DECAIRQ_EXTI_IRQn           EXTI0_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE

#define port_SPIx_busy_sending()    	 (__HAL_SPI_GET_FLAG((SPIx),(SPI_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				     (__HAL_SPI_GET_FLAG((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)		    	SPI_I2S_SendData((SPIx),(x))
#define port_SPIx_receive_data()		    SPI_I2S_ReceiveData(SPIx)
#define port_SPIx_disable()			      	__HAL_SPI_DISABLE(SPIx)
#define port_SPIx_enable()              __HAL_SPI_ENABLE(SPIx)
#define port_SPIx_set_chip_select()		  HAL_GPIO_WritePin(SPIx_CS_GPIO,SPIx_CS,GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()	  HAL_GPIO_WritePin(SPIx_CS_GPIO,SPIx_CS,GPIO_PIN_RESET)

#define port_GET_stack_pointer()		  __get_MSP()
#define port_GET_rtc_time()				    RTC_GetCounter()
#define port_SET_rtc_time(x)			    RTC_SetCounter(x)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               HAL_NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                HAL_NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

int is_button_low(uint16_t GPIOpin);

#define is_button_high(x)			  0
#define gpio_set(x)				      0
#define gpio_reset(x)				    0
#define is_gpio_out_low(x)			0
#define is_gpio_out_high(x)			0

/* DW1000 IRQ (EXTI0_IRQ) handler type. */
typedef void (*port_DecaISR_f)(void);

/* DW1000 IRQ handler declaration. */
extern port_DecaISR_f fpport_DecaISR;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_DecaISR_f deca_isr);

void SPI_ChangeRate(uint16_t scalingfactor);
void SPI_ConfigFastRate(uint16_t scalingfactor);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn SPI_SetRateLow()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void SPI_SetRateLow (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn SPI_SetRateHigh()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void SPI_SetRateHigh (void);

unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()

void Reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
