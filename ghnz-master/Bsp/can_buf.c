 /*
*********************************************************************************************************
*	                                           UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : can_buf.c
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
#include "can_buf.h"
#include <string.h>





/******************************************************************************
** ��������: void CanBusDataInit(uint16_t canid)
** ��������: ��ʼ���豸��������ݽṹ
** ����������can�ײ�������ʼ��
*******************************************************************************/
void CanBufferDataInit(sCAN_BUFFER * databuf)
{
	memset(databuf, 0x00, sizeof(sCAN_BUFFER));	  //init
#ifndef NO_SYS
	databuf->dataSem = OSSemCreate(0);
#endif  /*NO_SYS*/

}
/******************************************************************************
** ��������: CPU_INT16S CanBusEnable    (CANBUS_PARA *cfg);
** ��������: �����豸parameter��ʼ���豸
** ����������can�ײ�������ʼ��
*******************************************************************************/

int32_t CanBufferWrite(sCAN_BUFFER * databuf, sCAN_FRAME data)
{
		int32_t err = 0;
		databuf->buf[databuf->bufWr] = data;
		databuf->bufWr++;
		databuf->bufWr %= CAN_BUFFER_SIZE;
		
		if(databuf->bufWr == databuf->bufRd)
		{
			err = -1;
		}
#ifndef NO_SYS		
		OSSemPost(databuf->dataSem);	
#endif		
		return err;
		
}

/******************************************************************************
** ��������: int16_t DevBusRead(uint16_t id, void *buffer, uint32_t size)
** ��������: ��ȡcan���� ������ȡ
** ����������can�ײ�������ʼ��
*******************************************************************************/
int32_t CanBufferRead(sCAN_BUFFER * databuf, sCAN_FRAME * data)
{
	if(databuf->bufWr == databuf->bufRd)  return -1;
	
	
	memcpy(data, &databuf->buf[databuf->bufRd], sizeof(sCAN_FRAME));
	databuf->bufRd++;
	databuf->bufRd %= CAN_BUFFER_SIZE;
	return 0;
	
}


	
#ifndef NO_SYS
int32_t SysCanBufferRead(sCAN_BUFFER * databuf, sCAN_FRAME  * data, uint32_t time)
{

	INT8U err;
	OSSemPend(databuf->dataSem, time, &err);
	if(err == OS_ERR_TIMEOUT)
	{
	 	return OS_ERR_TIMEOUT;
	}
	else if(err == OS_ERR_NONE)
	{
			memcpy(data, &databuf->buf[databuf->bufRd], sizeof(sCAN_FRAME));
			databuf->bufRd++;
			databuf->bufRd %= CAN_BUFFER_SIZE;
		 return  OS_ERR_NONE;
	}
	else
	{
		return  CAN_BUFFER_READ_ERR;	                                            //error
	}

}

#endif  /*NO_SYS*/


