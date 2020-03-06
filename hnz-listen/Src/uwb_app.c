/**
  ******************************************************************************
  * File Name          : anchor.c
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
#include "uwb_app.h"
#include "dw1000_bus.h"

extern DWT_LocalData_s *pdw1000local;
sAncInfo   gAncInfo;
uint32_t   gUwbStateReg;
uint8_t 	 gUwbRxBuff[DW1000_RX_LEN];
uint32_t   gTraceFrame;
uint8_t    gFrameFunc;
uint32_t   gRecNum;
uint32_t   gUwbRxErr;

static sDW100ConfigPara gDwUserConfig=
{
	RX_ANT_DLY,
	TX_ANT_DLY,
	PAN_ID,
	0
	
};

__inline void SetAncLocationStep(ANC_LOCATION_STEP step)
{
	gAncInfo.step = step;
}

__inline void SetAncLocationState(ANC_LOCATION_STATE state)
{
	gAncInfo.state = state;
}

__inline void SetAncLocationErr(ANC_ERROR err)
{
	gAncInfo.err = err;
}










void UwbRxDataProcess(uint16_t len)
{
	sCAN_FRAME 		tmp;
	int16_t 			i;
	gFrameFunc = gUwbRxBuff[9];
	if(gFrameFunc == 13)
	{
		gFrameFunc = 19;
	}
	else if(gFrameFunc == 12)
	{
		gFrameFunc = 18;
	}
//	gTraceFrame++;
	memset(&tmp, 0x00, sizeof(tmp));
	tmp.Stdid = gTraceFrame++;
	for(i = 0; i < (len/8); i ++)
	{
		memcpy(tmp.Data, gUwbRxBuff+(i *8), 8);
		tmp.DLC = 8;
		CanBufferWrite(&gUwbCanTxBuf, tmp);
	}
	if(len%8)
	{
		memcpy(tmp.Data, gUwbRxBuff+(i *8), len%8);
		tmp.DLC = len%8;
		CanBufferWrite(&gUwbCanTxBuf, tmp);
	}
}



/*******************************************************************************************
* �������ƣ�void UwbReceiveTask(void *pdata)
* ����������UWB���ݽ��մ�������
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UwbReceiveTask(void *pdata)
{
	INT8U 		err;
	uint16_t  frameLen;
	uint16_t  frameCotrl;	
	
//	WaitUwbStartSem();   //�ȴ�������Ϣ
	gDwUserConfig.eui = pdw1000local->partID;
//	DwtInitConfig();
  Dw1000InitConfig(&gDwUserConfig);
	DWT_SetRxTimeOut(0);												//�趨���ճ�ʱʱ�䣬0--û�г�ʱʱ�䣬���޵ȴ�

	dwt_setdblrxbuffmode(1);
	DWT_SetAutoRxReEnable(ENABLE);	
	
	gUwbStateReg = DWT_Read32BitReg(SYS_STATUS_ID);
	
	if((gUwbStateReg & SYS_STATUS_HSRBP) != ((gUwbStateReg&SYS_STATUS_ICRBP) >> 1))
	{
		DWT_Write8BitOffsetReg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
	}
	
	DWT_SetInterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
	
//	DWT_SetInterrupt(DWT_INT_ALL, ENABLE);			//�����ж�
	
	DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���
	
	for(;;)
	{
//		SetAncLocationStep(ANC_INIT_SATA);


//		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���

		
		Dw1000ProcPend(0, &err);
	
		gUwbStateReg = DWT_Read32BitReg(SYS_STATUS_ID);
		if (gUwbStateReg & SYS_STATUS_RXFCG)	//�ɹ�����
		{
			gRecNum++;
//			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//�����־λ
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD); 
			
			frameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//��ý������ݳ���			
			if(frameLen <= DW1000_RX_LEN)
			{
				DWT_ReadRxData(gUwbRxBuff,  frameLen, 0);														//��ȡ��������
				frameCotrl = *((uint16_t *)gUwbRxBuff);
				
				if(frameCotrl & FRAME_CTRL_BYTE_ADDR_LEN_BIT)  										//�ж�ʹ�õ�ͨѶ��ַ���ȣ�16Bit����32Bit��
				{
						gUwbRxErr = 1;;//32Bit��ַ����������������
				}
				else
				{
					UwbRxDataProcess(frameLen);//����֡����
				}
			}
			else
			{
				gUwbRxErr = 2;
			}
			if (pdw1000local->dblbuffon)
			{
//				DWT_RxEnable(DWT_START_RX_IMMEDIATE);
				DWT_Write8BitOffsetReg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
//				DWT_RxEnable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);
				
//				gUwbStateReg = DWT_Read32BitReg(SYS_STATUS_ID);
			}
			else
				DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���
			
    }
		else if (gUwbStateReg & SYS_STATUS_ALL_RX_ERR)
		{
			gUwbRxErr = 3;
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			DWT_RxReset();
			DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���
		}
		else
		{
			gUwbRxErr = 4;
		}
			
		
  }
}

