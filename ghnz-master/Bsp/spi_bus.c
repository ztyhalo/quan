 /*
*********************************************************************************************************
*	                                           UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : can_bus.c
*    Module  : can driver
*    Version : V1.0
*    History :
*   -----------------
*             can底层驱动
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/
#include "dev_bus.h"
#include "spi_bus.h"
#include <string.h>
#include "l4spi.h"
//#include "deca_mutex.h"
#include "deca_device_api.h"
#include "port.h"

#define SPI_TX_QSIZE       256
#define SPI_RX_QSIZE       256

//DEV_BUS_PRO(SPI, uint8_t)

uint8_t gSpiTxBuf[SPI_TX_QSIZE];
uint8_t gSpiRxBuf[SPI_RX_QSIZE];

#if SPI_MODE == SPI_INT_MODE
OS_EVENT			*   gSpiEndSem;
#else
uint32_t     gSpiErrId;
#endif /*SPI_MODE == SPI_INT_MODE*/
///******************************************************************************
//** 函数名称: void CanBusDataInit(uint16_t canid)
//** 功能描述: 初始化设备所需的数据结构
//** 参数描述：can底层驱动初始化
//*******************************************************************************/
#if SPI_MODE == SPI_INT_MODE
void SpiBusDataInit(void)
{
	gSpiEndSem = OSSemCreate(0);
}
#endif /*#if SPI_MODE == SPI_INT_MODE*/
///******************************************************************************
//** 函数名称: void SpiBusInit(void)
//** 功能描述: 根据设备parameter初始化设备
//** 参数描述：can底层驱动初始化
//*******************************************************************************/
void SpiBusInit(void)
{
#if SPI_MODE == SPI_INT_MODE
	SpiBusDataInit();
#endif /* SPI_MODE == SPI_INT_MODE*/
	MX_SPI1_Init();

}

void SpiBusError_Handler(void)
{
	while(1)
	{
		;
	}
}


#if SPI_MODE == SPI_INT_MODE
int ReadFromSPI(uint16_t headerLength,  uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer)
{
	INT8U err;
	int   ret = 0;
	memcpy(&gSpiTxBuf, headerBuffer, headerLength);
	
	port_SPIx_set_chip_select();
	if(Z_SPI_TransmitReceive_IT(gSpiTxBuf, gSpiRxBuf, headerLength+readlength)  != HAL_OK)
	{
		SpiBusError_Handler();
	}	
	OSSemPend(gSpiEndSem, 0, &err);
	if(err == OS_ERR_TIMEOUT)
	{
	 	ret = BUS_RX_TIMEOVER;
	}
	else if(err == OS_ERR_NONE)
	{
		 memcpy(readBuffer, gSpiRxBuf + headerLength, readlength);
		 ret = BUS_RX_NORMAL;
	}
	else
	{
		ret =   BUS_RX_UNKNOWN_ERR;	                                            //error
	}
	port_SPIx_clear_chip_select();
	return ret;
	
}



int WriteToSPI(uint16_t headerLength,  uint8_t *headerBuffer, uint32_t bodylength,  uint8_t *bodyBuffer)
{

	INT8U err;
	int   ret = 0;
  decaIrqStatus_t  stat ;

  stat = DecaMutexON() ;
  port_SPIx_set_chip_select();
	
	memcpy(gSpiTxBuf, headerBuffer, headerLength);
	memcpy(gSpiTxBuf+headerLength, bodyBuffer, bodylength);
	
//	port_SPIx_set_chip_select();
	if(Z_SPI_TransmitReceive_IT(gSpiTxBuf, gSpiRxBuf, headerLength+bodylength)  != HAL_OK)
	{
		SpiBusError_Handler();
	}	
	OSSemPend(gSpiEndSem, 0, &err);
	if(err == OS_ERR_TIMEOUT)
	{
	 	ret = BUS_RX_TIMEOVER;
	}
	else if(err == OS_ERR_NONE)
	{
		 ret = BUS_RX_NORMAL;
	}
	else
	{
		ret =   BUS_RX_UNKNOWN_ERR;	                                            //error
	}
	port_SPIx_clear_chip_select();
	DecaMutexOFF(stat) ;
	return ret;
} // end WriteToSPI()

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	OSIntEnter();  
	OSSemPost(gSpiEndSem);
//	OSIntExit();     
}
#else
int ReadFromSPI(uint16_t headerLength,  uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer)
{
	INT8U err;
	int   ret = 0;
	memcpy(&gSpiTxBuf, headerBuffer, headerLength);
	
	port_SPIx_set_chip_select();
	gSpiErrId = HAL_SPI_TransmitReceive(&hspi1, gSpiTxBuf, gSpiRxBuf, headerLength+readlength, 20);
	if(gSpiErrId  != HAL_OK)
	{
		SpiBusError_Handler();
	}	
	else
	{
		 memcpy(readBuffer, gSpiRxBuf + headerLength, readlength);
		 ret = BUS_RX_NORMAL;
	}
	port_SPIx_clear_chip_select();
	return ret;
	
}

int WriteToSPI(uint16_t headerLength,  uint8_t *headerBuffer, uint32_t bodylength,  uint8_t *bodyBuffer)
{

	INT8U err;
	int   ret = 0;
  decaIrqStatus_t  stat ;

  stat = DecaMutexON() ;
  port_SPIx_set_chip_select();
	
	memcpy(gSpiTxBuf, headerBuffer, headerLength);
	memcpy(gSpiTxBuf+headerLength, bodyBuffer, bodylength);
	
//	port_SPIx_set_chip_select();
//	gSpiErrId = HAL_SPI_TransmitReceive(&hspi1, gSpiTxBuf, gSpiRxBuf, headerLength+readlength, 20);
	gSpiErrId = HAL_SPI_Transmit(&hspi1, gSpiTxBuf, headerLength+bodylength, 20);
	if(gSpiErrId  != HAL_OK)
	{
		SpiBusError_Handler();
	}	
	else 
	{
		 ret = BUS_RX_NORMAL;
	}
	port_SPIx_clear_chip_select();
	DecaMutexOFF(stat) ;
	return ret;
} // end WriteToSPI()

#endif /* SPI_MODE == SPI_INT_MODE*/




