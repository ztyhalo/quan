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
#include "l4spi.h"

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond
 * tick timer. */
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 				1000


	
#define SPIx						    ((SPI_HandleTypeDef *)&hspi1)
	
void printf2(const char *format, ...);

/*spi 配置在spi驱动中*/
	

#define UWB_RST_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define UWB_RST_PIN					        GPIO_PIN_12
#define UWB_RST_GPORT								GPIOB


#define UWB_WEAK_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define UWB_WEAK_PIN					      GPIO_PIN_13
#define UWB_WEAK_GPORT							GPIOB


#define UWB_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define UWB_CS_PIN					       GPIO_PIN_4
#define UWB_CS_GPORT							 GPIOA

#define UWB_IRQ_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define UWB_IRQ_PIN					        GPIO_PIN_15
#define UWB_IRQ_GPORT								GPIOB
#define UWB_EXTI_IRQn           		EXTI15_10_IRQn
#define UWB_EXTI_IRQ_PRIO           2

	


/*dw1000中断*/
#define DWM1000_IRQHandler					EXTI15_10_IRQHandler


#define DECAIRQ_EXTI_IRQn           UWB_IRQ_PIN



#define port_SPIx_busy_sending()    	 (__HAL_SPI_GET_FLAG((SPIx),(SPI_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				     (__HAL_SPI_GET_FLAG((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))

#define port_SPIx_disable()			      	__HAL_SPI_DISABLE(SPIx)
#define port_SPIx_enable()              __HAL_SPI_ENABLE(SPIx)

#define port_SPIx_set_chip_select()		  UWB_CS_GPORT->BRR = UWB_CS_PIN 
#define port_SPIx_clear_chip_select()	  UWB_CS_GPORT->BSRR = UWB_CS_PIN 


ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               HAL_NVIC_DisableIRQ(UWB_EXTI_IRQn)
#define port_EnableEXT_IRQ()                HAL_NVIC_EnableIRQ(UWB_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(UWB_IRQ_GPORT, UWB_IRQ_PIN)

int 		NVIC_DisableDECAIRQ(void);


int is_IRQ_enabled(void);

/* DW1000 IRQ (EXTI0_IRQ) handler type. */
typedef void (*port_DecaISR_f)(void);


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
void Dw100GpioInit(void);


#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
