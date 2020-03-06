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
OS_EVENT			*   gSpiEndSem;

///******************************************************************************
//** 函数名称: void CanBusDataInit(uint16_t canid)
//** 功能描述: 初始化设备所需的数据结构
//** 参数描述：can底层驱动初始化
//*******************************************************************************/
void SpiBusDataInit(void)
{
	gSpiEndSem = OSSemCreate(0);
}
///******************************************************************************
//** 函数名称: void SpiBusInit(void)
//** 功能描述: 根据设备parameter初始化设备
//** 参数描述：can底层驱动初始化
//*******************************************************************************/
void SpiBusInit(void)
{
	SpiBusDataInit();
	MX_SPI1_Init();

}

void SpiBusError_Handler(void)
{
	while(1)
	{
		;
	}
}

///******************************************************************************
//** 函数名称: int16_t DevBusRead(uint16_t id, void *buffer, uint32_t size)
//** 功能描述: 读取can数据 阻塞读取
//** 参数描述：can底层驱动初始化
//*******************************************************************************/
//int16_t DevBusRead(uint16_t id, void *buffer, uint32_t size)
//{
//	
//#ifndef NO_SYS
//	INT8U err;
//	OSSemPend(gCAN_BUS_DEV[id].recCanSem, gCAN_BUS_DEV[id].rxTimeout, &err);
//	if(err == OS_ERR_TIMEOUT)
//	{
//	 	return CANBUS_RX_TIMEOVER;
//	}
//	else if(err == OS_ERR_NONE)
//	{
//	 	*(sCAN_FRAME *)buffer = gCAN_BUS_DEV[id].bufRx[gCAN_BUS_DEV[id].bufRxRd++];
////		CAN_BUS_DEV[Id].BufRxRd++;
//		if(gCAN_BUS_DEV[id].bufRxRd > CANBUS_RX_QSIZE - 1)
//		{
//			gCAN_BUS_DEV[id].bufRxRd = 0;	
//		} 
//		 return  CANBUS_RX_NORMAL;
//	}
//	else
//	{
//		return  CANBUS_RX_UNKNOWN_ERR;	                                            //error
//	}
//#endif  /*NO_SYS*/

//}
///******************************************************************************
//** 函数名称: INT16S CanBusIoCtl(INT16U busId, INT16U func, void *argp)
//** 功能描述: 0 device interrupt					if receive buffer full ,new data overlap old data
//** 参数描述：none
//*******************************************************************************/
//int16_t CanBusIoCtl(int16_t busId, int16_t func, void *argp)
//{
//	switch (func)
//	{
//		case CANBUS_SET_RX_TIMEOUT:
//				gCAN_BUS_DEV[busId].rxTimeout = *((int16_t *)argp);
//		break;
//  }
//	return(1);
//}

///******************************************************************************
//** 函数名称: void CanBus0RxHandler(void)
//** 功能描述: 0 device interrupt					if receive buffer full ,new data overlap old data
//** 参数描述：none
//*******************************************************************************/
////void CanBus0RxHandler(void)
////{
//////	int err;
////	OSIntEnter();
////	memset((char *)(&CAN_BUS_DEV[0].BufRx[CAN_BUS_DEV[0].BufRxWr]), 0x00, sizeof(CAN_FRAME));
////	STM32_CAN_Read(0, &(CAN_BUS_DEV[0].BufRx[CAN_BUS_DEV[0].BufRxWr]));
////	CAN_BUS_DEV[0].BufRxWr++;
////	if(CAN_BUS_DEV[0].BufRxWr > CANBUS_RX_QSIZE - 1)
////	{
////	 	CAN_BUS_DEV[0].BufRxWr = 0;
////	}

////	OSSemPost(CAN_RX_SEM);					   //发送信号量 停止can receive time
////	OSIntExit();

////				
////}
//sCAN_FRAME readCanDriData(CAN_RxHeaderTypeDef head, uint8_t * data)
//{
//	sCAN_FRAME tmp;
//	memset((void *)(&tmp), 0x00, sizeof(sCAN_FRAME));
//	
//	tmp.Stdid = (head.IDE == CAN_ID_STD) ? head.StdId : head.ExtId;
//	tmp.DLC = head.DLC;
//	memcpy(tmp.Data, data, tmp.DLC);
//	
//	return tmp;
//	
//}


//void CAN_BusRxDataCallback(uint16_t canid, CAN_RxHeaderTypeDef head, uint8_t * data)
//{
////	memset((void *)(&gCAN_BUS_DEV[canid].bufRx[gCAN_BUS_DEV[canid].bufRxWr++]), 0x00, sizeof(sCAN_FRAME));
//	gCAN_BUS_DEV[canid].bufRx[gCAN_BUS_DEV[canid].bufRxWr++] = readCanDriData(head, data);
//	gCAN_BUS_DEV[canid].bufRxWr %= CANBUS_RX_QSIZE;
//#ifndef NO_SYS
//	OSSemPost(gCAN_BUS_DEV[canid].recCanSem);					   //发送信号量 停止can receive time
//#endif
//}


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
	
	port_SPIx_set_chip_select();
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
