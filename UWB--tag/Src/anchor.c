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

static uint16_t FrameCotrl;				//MAC帧控制域
static uint32_t gDispCount;

static uint8_t RxBuffer[RX_BUF_LEN];
static uint32_t AncStatusReg = 0;

static Tag2AncDist_s	TagDistBuff[ANC_COMM_NUM_MAX_WITH_TAG];				//Tag信息存储

int8_t TagNumNow;								//基站范围内标签数量
uint16_t DistThresh = 1000;			//距离上报阈值：300cm

int8_t	TagStorePtr;																				//Tag信息存储位置
int8_t  LastTagStorePtr = -1;																//上一个Tag信息存储位置

/*基站*/
uint8_t TxRespMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 0xFF, 0xFF, FRAME_RESP, 0x02, 0, 0, 0, 0};		//RESP帧
uint8_t Anc_CommDecResp[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 'V', 'E', FRAME_COMM_DEC_RESP, 0, 0};	//通讯声明帧回复

extern DWT_AddrStatus_s 	sDWT;
/*******************************************************************************************
* 函数名称：UWB_Anchor(void *pdata)
* 功能描述：UWB作为基站时的执行函数
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void UWB_Anchor(void *pdata)
{
	uint16_t FrameLen;

#if defined(USE_OLED)		
	char ModelID[5];
#endif
	INT16U RxFrameDestAddr = 0;					//接收数据帧，目标地址
	
	DWT_SetAddress16(sDWT.ShortAddr);		//设置帧过滤地址

	WriteToMsg(Anc_CommDecResp, (uint8_t *)&sDWT.ShortAddr, FRMAE_DEC_RESP_ANC_ADDR_IDX, 2, 0);	//设置Resp发送帧基站地址
	WriteToMsg(TxRespMsg, (uint8_t *)&sDWT.ShortAddr, FRAME_SOURCE_ADDR_IDX, 2, 0);							//设置Resp发送帧基站地址

#if defined(USE_OLED)	
	sprintf(ModelID, "%04d", sDWT.ShortAddr);
	OLED_ShowString(0, 0, "AncID:    ", 16);
	OLED_ShowString(48, 0, ModelID, 16);
#endif
	
	for(;;)
	{
//		DWT_EnableFrameFilter(SYS_CFG_FFAB);  			//只接受信标帧
//		DWT_SetAutoRxReEnable(ENABLE);							//开启接收器自动开启（接收失败后开启，接收成功后不开启，用户手册72页）
		
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待

		DWT_SetInterrupt(DWT_INT_ALL, ENABLE);			//开启中断

		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收

		OSTaskSuspend(OS_PRIO_SELF);								//挂起任务，等待接收中断
		
//		while (!((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//不断查询芯片状态直到接收成功或者出现错误
//		{ }

		if ((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG)	//成功接收
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//清除标志位
			
//			DWT_EnableFrameFilter(DISABLE);		//关闭帧过滤
			
			FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//获得接收数据长度			
			if(FrameLen <= RX_BUF_LEN)
			{
				DWT_ReadRxData(RxBuffer, FrameLen, 0);														//读取接收数据
				FrameCotrl = *((uint16_t *)RxBuffer);
				
				if(FrameCotrl & FRAME_CTRL_BYTE_ADDR_LEN_BIT)  										//判断使用的通讯地址长度（16Bit或者32Bit）
				{
						//32Bit地址，不做处理，丢弃
				}
				else
				{
					if((FrameCotrl & FRAME_CTRL_BYTE_FRAME_TYPE_BIT) == 0)	//信标帧
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
						RxFrameDestAddr = ((uint16_t)RxBuffer[FRAME_DEST_ADDR_IDX + 1] << 8) | RxBuffer[FRAME_DEST_ADDR_IDX];  //读取接收帧目标地址

						if(RxFrameDestAddr == sDWT.ShortAddr)  //接收帧是发给自己的
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
    }//接收成功
		else
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			DWT_RxReset();
		}
  }
}


/*******************************************************************************************
* 函数名称：Anc_TOF(uint8_t *pdata)
* 功能描述：tof测距
* 入口参数：*pdata（接收数据帧）
* 出口参数：无
* 使用说明：无
********************************************************************************************/
uint16_t TOF_CommCnt;
uint16_t TOF_CommErrCnt;
uint8_t Wait4Fianl;

void Anc_TOF(uint8_t *pdata)
{
	int8_t StratTxStatu = -1;		//开启发送器是否成功：-1：未开启，0：已开启
	uint8_t TagID[2];
	uint32_t RespTxTime;
	uint16_t FrameLen;
	
	DWT_EnableFrameFilter(SYS_CFG_FFAD);						//接收数据帧
	DWT_SetAutoRxReEnable(ENABLE);									//关闭接收器自动重开
	
	ReadFromMsg(TagID, pdata, FRAME_SOURCE_ADDR_IDX, 2, 0);																//读取标签地址
	WriteToMsg(TxRespMsg, TagID, FRAME_DEST_ADDR_IDX, 2, 0);																//设置Resp发送帧标签地址
//	WriteToMsg(RxFinalMsg, TagID, FRAME_DEST_ADDR_IDX, 2);															//设置Final接收帧标签地址
	
	PollRxTS = GetRxTimeStamp_u64();																								//获得Poll包接收时间T2
	
	RespTxTime = (PollRxTS + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//计算Response发送时间T3
	DWT_SetDelayedTRxTime(RespTxTime);																							//设置Response发送时间T3
	
	DWT_SetRxAfterTxDelay(RESP_TX_TO_FINAL_RX_DLY_UUS);								//设置发送完成后开启接收延迟时间
	DWT_SetRxTimeOut(FINAL_RX_TIMEOUT_UUS);														//接收超时时间
	
	DWT_WriteTxData(sizeof(TxRespMsg), TxRespMsg, 0);									//写入发送数据
	DWT_WriteTxfCtrl(sizeof(TxRespMsg), 0, 0);												//设定发送长度
	
	StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);				//延迟发送
	
	Wait4Fianl = 1;
	
	OSTaskSuspend(OS_PRIO_SELF);
	
//	while (!((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//不断查询芯片状态直到接收成功或者出现错误
//	{ };
	
	Wait4Fianl = 0;
	
	if((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG)
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//清楚标志位
		
		FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//数据长度
		if (FrameLen <= RX_BUF_LEN)
		{
			DWT_ReadRxData(RxBuffer, FrameLen, 0);																//读取接收数据
		
			if(RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_FINAL)								//Final帧
			{
				uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
				uint64_t tof_dtu;
	
				
				HAL_GPIO_TogglePin(LED1_PIN_Port, LED1_PIN);
				TOF_CommCnt++;
				
				RespTxTS = GetTxTimeStamp_u64();			//获得response发送时间T3
				FinalRxTS = GetRxTimeStamp_u64();			//获得final接收时间T6
				
				
				FinalMsgGetTS_64(&RxBuffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);				//从接收数据中读取T1，T4，T5
				FinalMsgGetTS_64(&RxBuffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
				FinalMsgGetTS_64(&RxBuffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

				caculate_distance(poll_tx_ts, PollRxTS, RespTxTS, resp_rx_ts, final_tx_ts, FinalRxTS, &tof_dtu);
				
				dist = (uint16_t)tof_dtu;
				
				TagStorePtr = AskTagStorePtr(*(uint16_t *)TagID);						//申请存储地址，返回-1表示存储区已满
				
				if(TagStorePtr != -1)
				{
					if((TagStorePtr != LastTagStorePtr) && (TagDistBuff[LastTagStorePtr].TagDistUpdataFlag == 0) && LastTagStorePtr != -1)
					{
						CalculteTagRealDist(LastTagStorePtr, 0);		//前一个标签通讯次数未满,计算平均距离，计算时不需减去最大值和最小值
						gDispCount++;
					}
					TagDistBuff[TagStorePtr].TagDistUpdataFlag = 0;
					LastTagStorePtr = TagStorePtr;
					
					if(TagDistBuff[TagStorePtr].CommCnt == 0)		//该标签第一次与本基站进行通讯
					{
						TagDistBuff[TagStorePtr].MaxPtr = 0;			//初始化最大值指针和最小值指针
						TagDistBuff[TagStorePtr].MinPtr = 0;
					}
					
					TagDistBuff[TagStorePtr].TagID = *(uint16_t *)TagID;												//存储标签ID
					TagDistBuff[TagStorePtr].Dist[TagDistBuff[TagStorePtr].CommCnt] = dist;			//存储距离值
					
					TagDistBuff[TagStorePtr].MaxPtr = (dist > TagDistBuff[TagStorePtr].Dist[TagDistBuff[TagStorePtr].MaxPtr] ? TagDistBuff[TagStorePtr].CommCnt : TagDistBuff[TagStorePtr].MaxPtr);	//更新最大值指针
					TagDistBuff[TagStorePtr].MinPtr = (dist < TagDistBuff[TagStorePtr].Dist[TagDistBuff[TagStorePtr].MinPtr] ? TagDistBuff[TagStorePtr].CommCnt : TagDistBuff[TagStorePtr].MinPtr);	//更新最小值指针
					
					if(++TagDistBuff[TagStorePtr].CommCnt == TAG_COMM_TO_ANC_CNT)								//已完成该轮通讯，开始计算平均距离,计算时减去最大值和最小值
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
	DWT_EnableFrameFilter(DISABLE);		//关闭帧过滤
}


/*******************************************************************************************
* 函数名称：void ANC_CommDecResp(uint8_t *pdata)
* 功能描述：标签定位通讯声明回复
* 入口参数：uint8_t *pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void ANC_CommDecResp(uint8_t *pdata)
{
	int8_t StratTxStatu = -1;
	uint64_t FrameRxTime, FrameTxTime;
	
	WriteToMsg(Anc_CommDecResp, pdata + FRAME_DEC_TAG_ADDR_IDX, FRMAE_DEC_RESP_TAG_ADDR_IDX, 2, 0);
	
	FrameRxTime = GetRxTimeStamp_u64();																														//获取请求帧接收时间
	FrameTxTime = (FrameRxTime + (FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//计算回复帧发送时间
	DWT_SetDelayedTRxTime(FrameTxTime);																														//设置回复帧发送时间
	
	DWT_WriteTxData(sizeof(Anc_CommDecResp), Anc_CommDecResp, 0);		//写入发送数据
	DWT_WriteTxfCtrl(sizeof(Anc_CommDecResp), 0, 1);								//设定发送长度
	
	DWT_SetInterrupt(DWT_INT_TFRS, ENABLE);
	
	StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED);							//延迟发送
	
	OSTaskSuspend(OS_PRIO_SELF);
	
//	while (!((AncStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//不断查询芯片状态直到发送完成
//	{ };
	
	DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);	//清除标识位
	DWT_SetInterrupt(DWT_INT_TFRS, DISABLE);
}


/*******************************************************************************************
* 函数名称：int8_t AskTagStorePtr(uint16_t tagid)
* 功能描述：申请标签信息存储位置
* 入口参数：uint16_t tagid
* 出口参数：存储位置
* 使用说明：无
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
* 函数名称：uint16_t getTagStoredist(uint16_t tagid)
* 功能描述：查询标签信息存储位置
* 入口参数：uint16_t tagid
* 出口参数：存储位置
* 使用说明：无
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
* 函数名称：void CalculteTagRealDist(int8_t Ptr, uint8_t filtering)
* 功能描述：计算标签平均距离并进行判断是否进入或者离开报告范围
* 入口参数：int8_t Ptr（标签信息存储位置）
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void CalculteTagRealDist(int8_t Ptr, uint8_t filtering)
{
	uint8_t i;
//	char TagNum_OLED[3];
						
//	TagDistBuff[Ptr].RealDist = 0;			//计算值归零
	
	if(filtering)			//需要减去最大值和最小值
	{
		for(i = 0; i < TagDistBuff[Ptr].CommCnt; i++)
		{
			if((i != TagDistBuff[Ptr].MaxPtr) && (i != TagDistBuff[Ptr].MinPtr))
			{
				TagDistBuff[Ptr].RealDist += TagDistBuff[Ptr].Dist[i];				//求和
			}
		}
		TagDistBuff[Ptr].CommCnt -= 2;				//计数器减2
	}
	else							//不需要减去最大值和最小值
	{
		for(i = 0; i < TagDistBuff[Ptr].CommCnt; i++)
		{
			TagDistBuff[Ptr].RealDist += TagDistBuff[Ptr].Dist[i];				//求和
		}
	}
	
	TagDistBuff[Ptr].RealDist /= (TagDistBuff[Ptr].CommCnt + 1);		//求平均值
	
	TagDistBuff[Ptr].CommCnt = 0;						//通讯次数归零
	TagDistBuff[Ptr].TagDistUpdataFlag = 1;	//标签距离信息已更新
	
	if((TagDistBuff[Ptr].RealDist < DistThresh) && (TagDistBuff[Ptr].TagSta == 0))			//判断标签是否是第一次进入上报范围
	{
		TagDistBuff[Ptr].TagSta = 1;													//标定标签状态
		TagNumNow++;																					//基站内标签数量加一
//		sprintf(TagNum_OLED, "%02d", TagNumNow);
//		OLED_ShowString(56, 2, TagNum_OLED, 16);
//		CAN_CommToSC(TagDistBuff[Ptr].TagID, 标签进入);
	}
	else if((TagDistBuff[Ptr].RealDist > DistThresh) && (TagDistBuff[Ptr].TagSta == 1))	//判断标签是否离开上报范围
	{
		TagDistBuff[Ptr].TagSta = 0;													//标定标签状态
		TagDistBuff[Ptr].RealDist = 0xFFFF;
		if(--TagNumNow < 0)																		//基站内标签数量减一
		{
			TagNumNow = 0;
		}	
//		sprintf(TagNum_OLED, "%02d", TagNumNow);
//		OLED_ShowString(56, 2, TagNum_OLED, 16);
//		CAN_CommToSC(TagDistBuff[Ptr].TagID, 标签离开);
	}
}



bool caculate_distance(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t * val)
{
			uint64_t ra, rb, da, db =0;
			uint64_t tmp = 0;
	
				//t1 t4 t5 时间有效性检查
				if(t4 < t1)
				{
					t4 += DW_TIME_OVER;
					t5 += DW_TIME_OVER;
				}
				if(t5 < t4)
				{
					t5 += DW_TIME_OVER;
				}
				//t2 t3 t6 时间有效性检查
				
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
					tmp = ((ra * rb - da * db) )/ (ra + rb + da + db);		//计算公式
				}

				*val = tmp *46903 /100000;
				
				return true;
}



/*******************************************************************************************
* 函数名称：FinalMsgGetTS()
* 功能描述：
* 入口参数：
* 出口参数：无
* 使用说明：无
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
