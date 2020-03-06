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
*             can�ײ�����
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/
#include "can_bus.h"
#include <string.h>


sCANBUS_DATA  gCAN_BUS_DEV[STM32_CAN_N_NODE];						//dev name define


/******************************************************************************
** ��������: void CanBusDataInit(uint16_t canid)
** ��������: ��ʼ���豸��������ݽṹ
** ����������can�ײ�������ʼ��
*******************************************************************************/
void CanBusDataInit(uint16_t canid)
{
	memset(gCAN_BUS_DEV + canid,  0x00, sizeof(sCANBUS_DATA));	  //init CAN device struct
#ifndef NO_SYS
	gCAN_BUS_DEV[canid].recCanSem = OSSemCreate(0);
#endif  /*NO_SYS*/

}
/******************************************************************************
** ��������: CPU_INT16S CanBusEnable    (CANBUS_PARA *cfg);
** ��������: �����豸parameter��ʼ���豸
** ����������can�ײ�������ʼ��
*******************************************************************************/
INT16S CanBusEnable(void *cfg)
{
 return 0;

}

/******************************************************************************
** ��������: int16_t DevBusRead(uint16_t id, void *buffer, uint32_t size)
** ��������: ��ȡcan���� ������ȡ
** ����������can�ײ�������ʼ��
*******************************************************************************/
int16_t DevBusRead(uint16_t id, void *buffer, uint32_t size)
{
	
#ifndef NO_SYS
	INT8U err;
	OSSemPend(gCAN_BUS_DEV[id].recCanSem, gCAN_BUS_DEV[id].rxTimeout, &err);
	if(err == OS_ERR_TIMEOUT)
	{
	 	return CANBUS_RX_TIMEOVER;
	}
	else if(err == OS_ERR_NONE)
	{
	 	*(sCAN_FRAME *)buffer = gCAN_BUS_DEV[id].bufRx[gCAN_BUS_DEV[id].bufRxRd++];
//		CAN_BUS_DEV[Id].BufRxRd++;
		if(gCAN_BUS_DEV[id].bufRxRd > CANBUS_RX_QSIZE - 1)
		{
			gCAN_BUS_DEV[id].bufRxRd = 0;	
		} 
		 return  CANBUS_RX_NORMAL;
	}
	else
	{
		return  CANBUS_RX_UNKNOWN_ERR;	                                            //error
	}
#endif  /*NO_SYS*/

}
/******************************************************************************
** ��������: INT16S CanBusIoCtl(INT16U busId, INT16U func, void *argp)
** ��������: 0 device interrupt					if receive buffer full ,new data overlap old data
** ����������none
*******************************************************************************/
int16_t CanBusIoCtl(int16_t busId, int16_t func, void *argp)
{
	switch (func)
	{
		case CANBUS_SET_RX_TIMEOUT:
				gCAN_BUS_DEV[busId].rxTimeout = *((int16_t *)argp);
		break;
  }
	return(1);
}

/******************************************************************************
** ��������: void CanBus0RxHandler(void)
** ��������: 0 device interrupt					if receive buffer full ,new data overlap old data
** ����������none
*******************************************************************************/
//void CanBus0RxHandler(void)
//{
////	int err;
//	OSIntEnter();
//	memset((char *)(&CAN_BUS_DEV[0].BufRx[CAN_BUS_DEV[0].BufRxWr]), 0x00, sizeof(CAN_FRAME));
//	STM32_CAN_Read(0, &(CAN_BUS_DEV[0].BufRx[CAN_BUS_DEV[0].BufRxWr]));
//	CAN_BUS_DEV[0].BufRxWr++;
//	if(CAN_BUS_DEV[0].BufRxWr > CANBUS_RX_QSIZE - 1)
//	{
//	 	CAN_BUS_DEV[0].BufRxWr = 0;
//	}

//	OSSemPost(CAN_RX_SEM);					   //�����ź��� ֹͣcan receive time
//	OSIntExit();

//				
//}
sCAN_FRAME readCanDriData(CAN_RxHeaderTypeDef head, uint8_t * data)
{
	sCAN_FRAME tmp;
	memset((void *)(&tmp), 0x00, sizeof(sCAN_FRAME));
	
	tmp.Stdid = (head.IDE == CAN_ID_STD) ? head.StdId : head.ExtId;
	tmp.DLC = head.DLC;
	memcpy(tmp.Data, data, tmp.DLC);
	
	return tmp;
	
}


void CAN_BusRxDataCallback(uint16_t canid, CAN_RxHeaderTypeDef head, uint8_t * data)
{
//	memset((void *)(&gCAN_BUS_DEV[canid].bufRx[gCAN_BUS_DEV[canid].bufRxWr++]), 0x00, sizeof(sCAN_FRAME));
	gCAN_BUS_DEV[canid].bufRx[gCAN_BUS_DEV[canid].bufRxWr++] = readCanDriData(head, data);
	gCAN_BUS_DEV[canid].bufRxWr %= CANBUS_RX_QSIZE;
#ifndef NO_SYS
	OSSemPost(gCAN_BUS_DEV[canid].recCanSem);					   //�����ź��� ֹͣcan receive time
#endif
}
