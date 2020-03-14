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
#include "anchor.h"
#include "filtering.h"
#include "address.h"
#if defined(USE_OLED)
	#include "oled.h"
#endif

static int32_t dist;

static uint64_t PollRxTS;
static uint64_t RespTxTS;
static uint64_t FinalRxTS;

static uint16_t FrameCotrl;				//MAC֡������
static uint32_t gDispCount;

static uint8_t RxBuffer[RX_BUF_LEN];
static uint32_t AncStatusReg = 0;

static Tag2AncDist_s	TagDistBuff[ANC_COMM_NUM_MAX_WITH_TAG];				//Tag��Ϣ�洢

int8_t TagNumNow;								//��վ��Χ�ڱ�ǩ����
uint16_t DistThresh = 1000;			//�����ϱ���ֵ��300cm

int8_t	TagStorePtr;																				//Tag��Ϣ�洢λ��
int8_t  LastTagStorePtr = -1;																//��һ��Tag��Ϣ�洢λ��

/*��վ*/
uint8_t TxRespMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 0xFF, 0xFF, FRAME_RESP, 0x02, 0, 0, 0, 0};		//RESP֡
uint8_t Anc_CommDecResp[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 'V', 'E', FRAME_COMM_DEC_RESP, 0, 0};	//ͨѶ����֡�ظ�

extern DWT_AddrStatus_s 	sDWT;
/*******************************************************************************************
* �������ƣ�UWB_Anchor(void *pdata)
* ����������UWB��Ϊ��վʱ��ִ�к���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UWB_Anchor(void *pdata)
{
	uint16_t FrameLen;

#if defined(USE_OLED)		
	char ModelID[5];
#endif
	INT16U RxFrameDestAddr = 0;					//��������֡��Ŀ���ַ
	
	DWT_SetAddress16(sDWT.ShortAddr);		//����֡���˵�ַ

	WriteToMsg(Anc_CommDecResp, (uint8_t *)&sDWT.ShortAddr, FRMAE_DEC_RESP_ANC_ADDR_IDX, 2, 0);	//����Resp����֡��վ��ַ
	WriteToMsg(TxRespMsg, (uint8_t *)&sDWT.ShortAddr, FRAME_SOURCE_ADDR_IDX, 2, 0);							//����Resp����֡��վ��ַ

#if defined(USE_OLED)	
	sprintf(ModelID, "%04d", sDWT.ShortAddr);
	OLED_ShowString(0, 0, "AncID:    ", 16);
	OLED_ShowString(48, 0, ModelID, 16);
#endif
	
	for(;;)
	{
//		DWT_EnableFrameFilter(SYS_CFG_FFAB);  			//ֻ�����ű�֡
//		DWT_SetAutoRxReEnable(ENABLE);							//�����������Զ�����������ʧ�ܺ��������ճɹ��󲻿������û��ֲ�72ҳ��
		
		DWT_SetRxTimeOut(0);												//�趨���ճ�ʱʱ�䣬0--û�г�ʱʱ�䣬���޵ȴ�

		DWT_SetInterrupt(DWT_INT_ALL, ENABLE);			//�����ж�

		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���

		OSTaskSuspend(OS_PRIO_SELF);								//�������񣬵ȴ������ж�
		
//		while (!((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
//		{ }

		if ((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG)	//�ɹ�����
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//�����־λ
			
//			DWT_EnableFrameFilter(DISABLE);		//�ر�֡����
			
			FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//��ý������ݳ���			
			if(FrameLen <= RX_BUF_LEN)
			{
				DWT_ReadRxData(RxBuffer, FrameLen, 0);														//��ȡ��������
				FrameCotrl = *((uint16_t *)RxBuffer);
				
				if(FrameCotrl & FRAME_CTRL_BYTE_ADDR_LEN_BIT)  										//�ж�ʹ�õ�ͨѶ��ַ���ȣ�16Bit����32Bit��
				{
						//32Bit��ַ��������������
				}
				else
				{
					if((FrameCotrl & FRAME_CTRL_BYTE_FRAME_TYPE_BIT) == 0)	//�ű�֡
					{
						switch(RxBuffer[BEACON_FRAME_TYPE_IDX])
						{
							case FRAME_COMM_DEC:
								ANC_CommDecResp(RxBuffer);
								break;
						}
					}
					else
					{
						RxFrameDestAddr = ((uint16_t)RxBuffer[FRAME_DEST_ADDR_IDX + 1] << 8) | RxBuffer[FRAME_DEST_ADDR_IDX];  //��ȡ����֡Ŀ���ַ

						if(RxFrameDestAddr == sDWT.ShortAddr)  //����֡�Ƿ����Լ���
						{
							switch(RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX])
							{
								case FRAME_POLL:
									Anc_TOF(RxBuffer);
									break;
							}
						}
					}
				}
			}
    }//���ճɹ�
		else
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			DWT_RxReset();
		}
  }
}


/*******************************************************************************************
* �������ƣ�Anc_TOF(uint8_t *pdata)
* ����������tof���
* ��ڲ�����*pdata����������֡��
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
uint16_t TOF_CommCnt;
uint16_t TOF_CommErrCnt;
uint8_t Wait4Fianl;

void Anc_TOF(uint8_t *pdata)
{
	int8_t StratTxStatu = -1;		//�����������Ƿ�ɹ���-1��δ������0���ѿ���
	uint8_t TagID[2];
	uint32_t RespTxTime;
	uint16_t FrameLen;
	
	DWT_EnableFrameFilter(SYS_CFG_FFAD);						//��������֡
	DWT_SetAutoRxReEnable(ENABLE);									//�رս������Զ��ؿ�
	
	ReadFromMsg(TagID, pdata, FRAME_SOURCE_ADDR_IDX, 2, 0);																//��ȡ��ǩ��ַ
	WriteToMsg(TxRespMsg, TagID, FRAME_DEST_ADDR_IDX, 2, 0);																//����Resp����֡��ǩ��ַ
//	WriteToMsg(RxFinalMsg, TagID, FRAME_DEST_ADDR_IDX, 2);															//����Final����֡��ǩ��ַ
	
	PollRxTS = GetRxTimeStamp_u64();																								//���Poll������ʱ��T2
	
	RespTxTime = (PollRxTS + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//����Response����ʱ��T3
	DWT_SetDelayedTRxTime(RespTxTime);																							//����Response����ʱ��T3
	
	DWT_SetRxAfterTxDelay(RESP_TX_TO_FINAL_RX_DLY_UUS);								//���÷�����ɺ��������ӳ�ʱ��
	DWT_SetRxTimeOut(FINAL_RX_TIMEOUT_UUS);														//���ճ�ʱʱ��
	
	DWT_WriteTxData(sizeof(TxRespMsg), TxRespMsg, 0);									//д�뷢������
	DWT_WriteTxfCtrl(sizeof(TxRespMsg), 0, 0);												//�趨���ͳ���
	
	StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);				//�ӳٷ���
	
	Wait4Fianl = 1;
	
	OSTaskSuspend(OS_PRIO_SELF);
	
//	while (!((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
//	{ };
	
	Wait4Fianl = 0;
	
	if((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG)
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//�����־λ
		
		FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//���ݳ���
		if (FrameLen <= RX_BUF_LEN)
		{
			DWT_ReadRxData(RxBuffer, FrameLen, 0);																//��ȡ��������
		
			if(RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_FINAL)								//Final֡
			{
				uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
				uint64_t tof_dtu;
	
				
				HAL_GPIO_TogglePin(LED1_PIN_Port, LED1_PIN);
				TOF_CommCnt++;
				
				RespTxTS = GetTxTimeStamp_u64();			//���response����ʱ��T3
				FinalRxTS = GetRxTimeStamp_u64();			//���final����ʱ��T6
				
				
				FinalMsgGetTS_64(&RxBuffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);				//�ӽ��������ж�ȡT1��T4��T5
				FinalMsgGetTS_64(&RxBuffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
				FinalMsgGetTS_64(&RxBuffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

				caculate_distance(poll_tx_ts, PollRxTS, RespTxTS, resp_rx_ts, final_tx_ts, FinalRxTS, &tof_dtu);
				
				dist = (uint16_t)tof_dtu;
				
				TagStorePtr = AskTagStorePtr(*(uint16_t *)TagID);						//����洢��ַ������-1��ʾ�洢������
				
				if(TagStorePtr != -1)
				{
					if((TagStorePtr != LastTagStorePtr) && (TagDistBuff[LastTagStorePtr].TagDistUpdataFlag == 0) && LastTagStorePtr != -1)
					{
						CalculteTagRealDist(LastTagStorePtr, 0);		//ǰһ����ǩͨѶ����δ��,����ƽ�����룬����ʱ�����ȥ���ֵ����Сֵ
						gDispCount++;
					}
					TagDistBuff[TagStorePtr].TagDistUpdataFlag = 0;
					LastTagStorePtr = TagStorePtr;
					
					if(TagDistBuff[TagStorePtr].CommCnt == 0)		//�ñ�ǩ��һ���뱾��վ����ͨѶ
					{
						TagDistBuff[TagStorePtr].MaxPtr = 0;			//��ʼ�����ֵָ�����Сֵָ��
						TagDistBuff[TagStorePtr].MinPtr = 0;
					}
					
					TagDistBuff[TagStorePtr].TagID = *(uint16_t *)TagID;												//�洢��ǩID
					TagDistBuff[TagStorePtr].Dist[TagDistBuff[TagStorePtr].CommCnt] = dist;			//�洢����ֵ
					
					TagDistBuff[TagStorePtr].MaxPtr = (dist > TagDistBuff[TagStorePtr].Dist[TagDistBuff[TagStorePtr].MaxPtr] ? TagDistBuff[TagStorePtr].CommCnt : TagDistBuff[TagStorePtr].MaxPtr);	//�������ֵָ��
					TagDistBuff[TagStorePtr].MinPtr = (dist < TagDistBuff[TagStorePtr].Dist[TagDistBuff[TagStorePtr].MinPtr] ? TagDistBuff[TagStorePtr].CommCnt : TagDistBuff[TagStorePtr].MinPtr);	//������Сֵָ��
					
					if(++TagDistBuff[TagStorePtr].CommCnt == TAG_COMM_TO_ANC_CNT)								//����ɸ���ͨѶ����ʼ����ƽ������,����ʱ��ȥ���ֵ����Сֵ
					{
						CalculteTagRealDist(TagStorePtr, 0);
						gDispCount++;
					}
#if defined(USE_OLED)
					if(gDispCount >= 10)
					{
						OLED_Clear();
						OLED_printf(0,0,"1:%3d", getTagStoredist(1));
						OLED_printf(0,1,"2:%3d", getTagStoredist(2));
						OLED_printf(0,2,"3:%3d", getTagStoredist(3));
						gDispCount = 0;
					}
#endif
				}
			}
			else
			{
				TOF_CommCnt++;
				TOF_CommErrCnt++;
			}
		}
	}
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE);
		DWT_RxReset();
		TOF_CommCnt++;
		TOF_CommErrCnt++;
	}

	DWT_SetAutoRxReEnable(DISABLE);
	DWT_ForceTRxOff();
	DWT_EnableFrameFilter(DISABLE);		//�ر�֡����
}


/*******************************************************************************************
* �������ƣ�void ANC_CommDecResp(uint8_t *pdata)
* ������������ǩ��λͨѶ�����ظ�
* ��ڲ�����uint8_t *pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void ANC_CommDecResp(uint8_t *pdata)
{
	int8_t StratTxStatu = -1;
	uint64_t FrameRxTime, FrameTxTime;
	
	WriteToMsg(Anc_CommDecResp, pdata + FRAME_DEC_TAG_ADDR_IDX, FRMAE_DEC_RESP_TAG_ADDR_IDX, 2, 0);
	
	FrameRxTime = GetRxTimeStamp_u64();																														//��ȡ����֡����ʱ��
	FrameTxTime = (FrameRxTime + (FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//����ظ�֡����ʱ��
	DWT_SetDelayedTRxTime(FrameTxTime);																														//���ûظ�֡����ʱ��
	
	DWT_WriteTxData(sizeof(Anc_CommDecResp), Anc_CommDecResp, 0);		//д�뷢������
	DWT_WriteTxfCtrl(sizeof(Anc_CommDecResp), 0, 1);								//�趨���ͳ���
	
	DWT_SetInterrupt(DWT_INT_TFRS, ENABLE);
	
	StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED);							//�ӳٷ���
	
	OSTaskSuspend(OS_PRIO_SELF);
	
//	while (!((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//���ϲ�ѯоƬ״ֱ̬���������
//	{ };
	
	DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);	//�����ʶλ
	DWT_SetInterrupt(DWT_INT_TFRS, DISABLE);
}


/*******************************************************************************************
* �������ƣ�int8_t AskTagStorePtr(uint16_t tagid)
* ���������������ǩ��Ϣ�洢λ��
* ��ڲ�����uint16_t tagid
* ���ڲ������洢λ��
* ʹ��˵������
********************************************************************************************/
int8_t AskTagStorePtr(uint16_t tagid)
{
	int8_t i;
	int8_t ptr = -1;
	
	for(i = 0; i < ANC_COMM_NUM_MAX_WITH_TAG; i++)
	{
		if(tagid == TagDistBuff[i].TagID)
		{
			return i;
		}
		else
		{
			ptr = (TagDistBuff[i].TagSta == 0 ? i : ptr);
		}
	}
	return ptr;
}


/*******************************************************************************************
* �������ƣ�uint16_t getTagStoredist(uint16_t tagid)
* ������������ѯ��ǩ��Ϣ�洢λ��
* ��ڲ�����uint16_t tagid
* ���ڲ������洢λ��
* ʹ��˵������
********************************************************************************************/
uint16_t getTagStoredist(uint16_t tagid)
{
	int8_t i;
	
	for(i = 0; i < ANC_COMM_NUM_MAX_WITH_TAG; i++)
	{
		if(tagid == TagDistBuff[i].TagID)
		{
			return TagDistBuff[i].RealDist;
		}
	}
	return 0;
	
}


/*******************************************************************************************
* �������ƣ�void CalculteTagRealDist(int8_t Ptr, uint8_t filtering)
* ���������������ǩƽ�����벢�����ж��Ƿ��������뿪���淶Χ
* ��ڲ�����int8_t Ptr����ǩ��Ϣ�洢λ�ã�
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void CalculteTagRealDist(int8_t Ptr, uint8_t filtering)
{
	uint8_t i;
//	char TagNum_OLED[3];
						
//	TagDistBuff[Ptr].RealDist = 0;			//����ֵ����
	
	if(filtering)			//��Ҫ��ȥ���ֵ����Сֵ
	{
		for(i = 0; i < TagDistBuff[Ptr].CommCnt; i++)
		{
			if((i != TagDistBuff[Ptr].MaxPtr) && (i != TagDistBuff[Ptr].MinPtr))
			{
				TagDistBuff[Ptr].RealDist += TagDistBuff[Ptr].Dist[i];				//���
			}
		}
		TagDistBuff[Ptr].CommCnt -= 2;				//��������2
	}
	else							//����Ҫ��ȥ���ֵ����Сֵ
	{
		for(i = 0; i < TagDistBuff[Ptr].CommCnt; i++)
		{
			TagDistBuff[Ptr].RealDist += TagDistBuff[Ptr].Dist[i];				//���
		}
	}
	
	TagDistBuff[Ptr].RealDist /= (TagDistBuff[Ptr].CommCnt + 1);		//��ƽ��ֵ
	
	TagDistBuff[Ptr].CommCnt = 0;						//ͨѶ��������
	TagDistBuff[Ptr].TagDistUpdataFlag = 1;	//��ǩ������Ϣ�Ѹ���
	
	if((TagDistBuff[Ptr].RealDist < DistThresh) && (TagDistBuff[Ptr].TagSta == 0))			//�жϱ�ǩ�Ƿ��ǵ�һ�ν����ϱ���Χ
	{
		TagDistBuff[Ptr].TagSta = 1;													//�궨��ǩ״̬
		TagNumNow++;																					//��վ�ڱ�ǩ������һ
//		sprintf(TagNum_OLED, "%02d", TagNumNow);
//		OLED_ShowString(56, 2, TagNum_OLED, 16);
//		CAN_CommToSC(TagDistBuff[Ptr].TagID, ��ǩ����);
	}
	else if((TagDistBuff[Ptr].RealDist > DistThresh) && (TagDistBuff[Ptr].TagSta == 1))	//�жϱ�ǩ�Ƿ��뿪�ϱ���Χ
	{
		TagDistBuff[Ptr].TagSta = 0;													//�궨��ǩ״̬
		TagDistBuff[Ptr].RealDist = 0xFFFF;
		if(--TagNumNow < 0)																		//��վ�ڱ�ǩ������һ
		{
			TagNumNow = 0;
		}	
//		sprintf(TagNum_OLED, "%02d", TagNumNow);
//		OLED_ShowString(56, 2, TagNum_OLED, 16);
//		CAN_CommToSC(TagDistBuff[Ptr].TagID, ��ǩ�뿪);
	}
}



bool caculate_distance(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t * val)
{
			uint64_t ra, rb, da, db =0;
			uint64_t tmp = 0;
	
				//t1 t4 t5 ʱ����Ч�Լ��
				if(t4 < t1)
				{
					t4 += DW_TIME_OVER;
					t5 += DW_TIME_OVER;
				}
				if(t5 < t4)
				{
					t5 += DW_TIME_OVER;
				}
				//t2 t3 t6 ʱ����Ч�Լ��
				
				if(t3 < t2)
				{
					t3 += DW_TIME_OVER;
					t6 += DW_TIME_OVER;
				}
				if(t6 < t3)
				{
					t6 += DW_TIME_OVER;
				}

				ra = t4 - t1;														//Tround1 = T4 - T1  
				rb = t6 - t3;										//Tround2 = T6 - T3 
				da = t5 - t4;													//Treply2 = T5 - T4  
				db = t3 - t2;											//Treply1 = T3 - T2  
				if(ra * rb <  da * db)
				{
					*val = 0;
					return false;
				}
				else
				{
					tmp = ((ra * rb - da * db) )/ (ra + rb + da + db);		//���㹫ʽ
				}

				*val = tmp *46903 /100000;
				
				return true;
}



/*******************************************************************************************
* �������ƣ�FinalMsgGetTS()
* ����������
* ��ڲ�����
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void FinalMsgGetTS_64(const uint8_t *ts_field, uint64_t *ts)
{
	uint32_t i;
	uint64_t tmp = 0;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		tmp = ts_field[i];
		*ts += tmp << (i * 8);
	}
}
