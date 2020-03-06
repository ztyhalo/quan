/**
  ******************************************************************************
  * File Name          : uwb_beb.c
  * Description        : This file provides code for the configuration
  *                      of the anchor instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uwb_beb.h"
#include "dw1000_bus.h"
#include "randomapp.h"

uint32_t   gListUwbState;
uint16_t 	 gCollsWindow = 1;				//������˳�ͻ����Cw


uint32_t listen_channel_state(void)
{
	INT8U 		err;
	DWT_SetRxTimeOut(LISTEN_CHANNEL_TIME);				//�趨���ճ�ʱʱ��	
	DWT_SetInterrupt(DWT_INT_ALL, ENABLE);			  //�����ж�
	
	DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���
	
	Dw1000ProcPend(0, &err);
	return DWT_Read32BitReg(SYS_STATUS_ID);
}


void listen_channel_err(void)
{
	;
}


/*******************************************************************************************
* �������ƣ�ChannelStatu ListenChannel(void)
* �����������ŵ�����
* ��ڲ�������
* ���ڲ�������ǰ�ŵ�״̬��ChannelFree -- 1��ChannelBusy -- 0
* ʹ��˵������
********************************************************************************************/
ChannelStatus ListenChannel(void)
{
	uint16_t rngnum;
	uint8_t listencnt = 0;
	
	while(1)
	{

		gListUwbState = listen_channel_state();		
		if ((gListUwbState & SYS_STATUS_RXRFTO) && ((gListUwbState & SYS_STATUS_ALL_RX_GOOD) == 0))												//���ճ�ʱ���ҽ�������ǰ��δ���ڽ�������֡����ʾ�ŵ�����
		{ //�ŵ�����
			
				DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);
				rngnum = RNG_Get_RandomRange(0, gCollsWindow);				//��ȡ�������ֵ
				OSTimeDly(rngnum * BACK_OFF_TIME_MS);
				gListUwbState = listen_channel_state();
			
				if ((gListUwbState & SYS_STATUS_RXRFTO) && ((gListUwbState & SYS_STATUS_ALL_RX_GOOD) == 0))												//���ճ�ʱ���ҽ�������ǰ��δ���ڽ�������֡����ʾ�ŵ�����
				{//���˺��������
					DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);	//�����־λ
					
					listencnt = 0;							//�����ͻ����
					gCollsWindow /= 2;					//���;������ڴ�С
					gCollsWindow = gCollsWindow < COLLISION_WINDOW_MIN ? COLLISION_WINDOW_MIN: gCollsWindow;
					
					return CHANNEL_FREE;
				}
			
		}	
		
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_RX_GOOD);
		listencnt++;
		gCollsWindow = (gCollsWindow * 2 >= COLLISION_WINDOW_MAX ? COLLISION_WINDOW_MAX : gCollsWindow * 2);
		rngnum = RNG_Get_RandomRange(0, gCollsWindow);				//��ȡ�������ֵ
		OSTimeDly(rngnum * BACK_OFF_TIME_MS);
		
		if(listencnt >= BACK_OFF_COUNT_MAX)					//������ͻ16��
		{
			return CHANNEL_BUSY;
		}
	}
		
}


