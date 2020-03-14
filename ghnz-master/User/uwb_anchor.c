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
//#include "uwb_app.h"
#define __UWB_ANCHOR_C__
#include "dw1000_bus.h"
#include "uwb_anchor.h"
#include "uwb_common.h"
#include "location_data_app.h"
#include "deca_device_api.h"

//#include "zlist.h"

// 声明一级缓存链表
//LIST_DEFINE(Dist, sDistNote, ANC_LOC_TAG_MAX)

uint32_t   gUwbAncState;
uint32_t   gUwbAncErrState;
uint8_t 	 gAncRxBuff[ANC_RX_LEN_MAX];


sAncDevInfo gAncDev;
sLocTime    gLocTime;
uint16_t    gDist;

sDistList  gDistDataBuf;

uint32_t   gAncErrorId;

static sDW100ConfigPara gDwAnchorConfig=
{
	ANCHOR_RX_ANT_DLY,
	ANCHOR_TX_ANT_DLY,
	ANCHOR_PAN_ID,
	0	
};

/*基站*/
/*                            帧控制     帧序列号  目标PANID      目标地址    源地址     帧功能码  
                               2字节       1字节       2字节        2字节       2字节       1字节  */
uint8_t gAnc_CommDecResp[] = {0x41, 0x88,    0,    0xCA, 0xDE,   0xFF, 0xFF,   'V', 'E', FRAME_COMM_DEC_RESP, 0, 0};														//定位通讯声明帧回复
uint8_t gRxPollMsg[]       = {0x41, 0x88,    0,    0xCA, 0xDE,   0xFF, 0xFF,   'V', 'E', FRAME_POLL, 0, 0};																						//POLL帧
uint8_t gTxRespMsg[]       = {0x41, 0x88,    0,    0xCA, 0xDE,   'V', 'E',    0xFF, 0xFF, FRAME_RESP, 0x02, 0, 0, 0, 0};																//RESP帧
uint8_t gRxFinalMsg[]      = {0x41, 0x88,    0,    0xCA, 0xDE,   0xFF, 0xFF,    'V', 'E', FRAME_FINAL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};			//FINAL帧




void SetAnchorAddr(uint8_t * dest, uint16_t addr)
{
	memcpy(dest+ANCHOR_ADDR_OFF, (void *)&addr, 2);
}

void DebugErrorProcess(void)
{
	while (1)
	{
		;
	}
}

void AncErrorProcess(uint32_t err)
{
	gAncErrorId = err;
	while (1)
	{
		;
	}
}

/*******************************************************************************************
* 函数名称：bool caculate_distance(sLocTime time, uint64_t * val)
* 功能描述：计算距离
* 入口参数：
* 出口参数：无
* 使用说明：无
********************************************************************************************/
bool caculate_distance(sLocTime time, uint64_t * val)
{
			uint64_t ra, rb, da, db =0;
			uint64_t tmp = 0;
	
				//t1 t4 t5 时间有效性检查
				if(time.t4 < time.t1)
				{
					time.t4 += DW_TIME_OVER;
					time.t5 += DW_TIME_OVER;
				}
				if(time.t5 < time.t4)
				{
					time.t5 += DW_TIME_OVER;
				}
				//t2 t3 t6 时间有效性检查
				
				if(time.t3 < time.t2)
				{
					time.t3 += DW_TIME_OVER;
					time.t6 += DW_TIME_OVER;
				}
				if(time.t6 < time.t3)
				{
					time.t6 += DW_TIME_OVER;
				}

				ra = time.t4 - time.t1;														//Tround1 = T4 - T1  
				rb = time.t6 - time.t3;										//Tround2 = T6 - T3 
				da = time.t5 - time.t4;													//Treply2 = T5 - T4  
				db = time.t3 - time.t2;											//Treply1 = T3 - T2  
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
* 函数名称：
* 功能描述：
* 入口参数：
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void AncWriteSendData(uint8_t * data, uint16_t len)
{
	DwtWriteTxData(data, len);
	gAncDev.txPollNum++;
}

/*******************************************************************************************
* 函数名称：bool caculate_distance(sLocTime time, uint64_t * val)
* 功能描述：计算距离
* 入口参数：
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void DwRxDelayErrProcess(void)
{
		gAncDev.devState = UWB_WAIT_STATE;
//		gAncDev.locCount++;
		gAncDev.locErrNum++;
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE);
//		DWT_RxReset();
}

void DwLocSucProcess(void)
{
		gAncDev.devState = UWB_WAIT_STATE;   //定位成功
		gAncDev.locCount++;
}

void WaiteOverErrProcess(void)
{
	gAncDev.waitErr++;
//	DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE);
}

/*******************************************************************************************
* 函数名称：
* 功能描述：一级标签距离缓存处理函数
* 入口参数：
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void DistDataStrInit(sDistNote * data, uint16_t tagId)
{
	memset(data, 0x00, sizeof(sDistNote));
	data->tagId = tagId;
}

void WriteDistDataPro(sDistNote * note, uint16_t val)
{
//	if(note->commCnt >= ANC_TOF_REC_MAX) //错误 越界
//	{
//		DebugErrorProcess();
//	}
//	note->dist[note->commCnt] = val;
//	if(val > note->dist[note->maxPtr])
//		note->maxPtr = note->commCnt;
//	else if(val <  note->dist[note->minPtr])
//		note->minPtr = note->commCnt;
//	note->commCnt++;
//	note->heartCout++;
	
	note->dist[note->wr] = val;
	if(val > note->dist[note->maxPtr])
		note->maxPtr = note->wr;
	else if(val <  note->dist[note->minPtr])
		note->minPtr = note->wr;
	note->wr++;
	note->wr %= ANC_TOF_REC_MAX;
	note->commCnt++;
	note->commCnt  = note->commCnt < ANC_TOF_REC_MAX ? note->commCnt : ANC_TOF_REC_MAX;
	note->heartCout++;
}

sDistList_N * GetDistDataBuf(uint16_t tagId)
{

	sDistList_N * midpoint;
	for(midpoint = gDistDataBuf.rw_p; midpoint != NULL; midpoint = midpoint->next)
	{
		if(midpoint->p->tagId == tagId)                                //标签相等 成功
			return midpoint;			
	}
	
	if(gDistDataBuf.free_p != NULL)  // 没有找到tagid 说明新增
	{
		midpoint = gDistDataBuf.free_p;
		gDistDataBuf.free_p = gDistDataBuf.free_p->next;
		midpoint->next = gDistDataBuf.rw_p;
		gDistDataBuf.rw_p = midpoint;
		DistDataStrInit(midpoint->p, tagId);
		return midpoint;
	}
	else
		return NULL;
	
}

void AddDistData(uint16_t tagId, uint16_t val)
{
	sDistList_N * midpoint;
	
	midpoint = GetDistDataBuf(tagId);
	if(midpoint != NULL)
	{		
		WriteDistDataPro(midpoint->p, val);
	}
}


sDistList_N * GetDistBufPoint(void)
{
	return gDistDataBuf.rw_p;
}


void DistBufDataCount(sDistNote * data)
{
	int i; 
	uint32_t midVal = 0;
	if(data->commCnt < ANC_TOF_MIN_DATA)
	{
//		gAncDev.dataNumErr++;
		for(i = 0; i < data->commCnt; i++)
		{
			midVal += data->dist[i];
		}
		data->realDist = midVal /data->commCnt;
	}
	else
	{
		for(i = 0; i < data->commCnt; i++)
		{
			if(i != data->maxPtr && i != data->minPtr)
				midVal += data->dist[i];
		}
		data->realDist = midVal/(data->commCnt -2);
	}
//	data->commCnt = 0;
}

void DelDistBufData(sDistList_N * before, sDistList_N * del)
{
	Dist_Delete_List(&gDistDataBuf, before, del);
}



void UwbRxErrorProcess(void)
{
	
//	if((gUwbAncState & SYS_STATUS_HSRBP) != ((gUwbAncState&SYS_STATUS_ICRBP) >> 1))
//	{
//		DWT_Write8BitOffsetReg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
//	}
//		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收

	if (gUwbAncState & SYS_STATUS_ALL_RX_ERR)
	{
//			pdw1000local->wait4resp = 0;
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		DWT_ForceTRxOff();
		DWT_RxReset();
//			DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
	}
	else if(gUwbAncState & SYS_STATUS_TXFRS)
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); 
	}
	else if(gUwbAncState & SYS_STATUS_ALL_RX_TO)
	{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bits

//        pdw1000local->wait4resp = 0;

			DWT_ForceTRxOff();
			DWT_RxReset();

	}
	else if(gUwbAncState & SYS_STATUS_ALL_TX)
	{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear RX timeout event bits
			DWT_ForceTRxOff();
			DWT_RxReset();
	}		
	else
	{
		gUwbAncErrState = gUwbAncState;
		DWT_ForceTRxOff();
		DWT_RxReset();
//			DWT_ForceTRxOff();
//			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//			DWT_RxReset();
	}
}

/*******************************************************************************************
* 函数名称：void ANC_CommDecResp(uint8_t *pdata)
* 功能描述：标签定位通讯声明回复
* 入口参数：uint8_t *pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void AncDeclareFrameProcess(uint8_t * pdata, uint16_t len)
{
//	INT8U 		err;
//	uint64_t frameRxTime = 0; 
//	uint64_t frameTxTime = 0;
//	uint64_t curTime = 0;
	gAncDev.devState = UWB_RX_BEA_STATE;    														//设置设备状态为定位状态
	gAncDev.beaconNum++;
	gAncDev.currDevId = *((uint16_t *)(pdata + FIRST_ADDR_OFF));          //当前定位的标签id号
	
	memcpy(gAnc_CommDecResp + FIRST_ADDR_OFF, pdata + FIRST_ADDR_OFF, 2); 	 //设置应答的目的地址
	
	
	DWT_WriteTxData(sizeof(gAnc_CommDecResp), gAnc_CommDecResp, 0);		//写入发送数据
	DWT_WriteTxfCtrl(sizeof(gAnc_CommDecResp), 0, 1);								  //设定发送长度
	
//	frameRxTime = GetRxTimeStamp_u64();			                          //获取请求帧接收时间
//	
//	DWT_SetInterrupt(DWT_INT_TFRS, ENABLE); 													//使能发送中断
//	curTime = GetSysTimeStamp_u64();
//	
//	frameTxTime =  (curTime + (FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	  //计算回复帧发送时间
//	
//	DWT_SetDelayedTRxTime(frameTxTime);																													//设置回复帧发送时间
//	
//	
//	
//	if(DWT_StartTx(DWT_START_TX_DELAYED) != DWT_SUCCESS)
//	{
//		AncErrorProcess(1);
//	}
	gAncDev.txBeaNum++;
	if(DWT_StartTx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
	{
		AncErrorProcess(1);
	}
	gAncDev.devState = UWB_RES_BEA_STATE;    		
	
//	gAncDev.devState = 8;    	

//	Dw1000ProcPend(2,   &err);
//	if(err == OS_ERR_TIMEOUT)
//	{
//		AncErrorProcess(2);
//	}
//	else
//	{
//		gUwbAncState = DWT_Read32BitReg(SYS_STATUS_ID);
//		if(gUwbAncState & SYS_STATUS_TXFRS) //发送完成
//		{
//				DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);	//清除标识位
//		}
//		else
//		{
//			AncErrorProcess(3);
//		}
//	}	
//	DWT_SetInterrupt(DWT_INT_TFRS, DISABLE);

}


/*******************************************************************************************
* 函数名称：Anc_TOF(uint8_t *pdata)
* 功能描述：tof测距
* 入口参数：*pdata（接收数据帧）
* 出口参数：无
* 使用说明：无
********************************************************************************************/


void AncPollProcess(uint8_t *pdata, uint16_t len)
{
	uint32_t respTxTime;
	uint16_t tagId;
	
	uint64_t curTime = 0;
	uint32_t frameLen;
	uint64_t tof_dtu;
	
	INT8U 		err;
	
	tagId = *((uint16_t *)(pdata + SECOND_ADDR_OFF));   //读取标签地址
//	if(tagId != gAncDev.currDevId) 											//收到的数据帧乱序
//	{
//		AncErrorProcess(4);
//		return ;
//	}
	
//	DWT_EnableFrameFilter(SYS_CFG_FFAD);						//接收数据帧
//	DWT_SetAutoRxReEnable(ENABLE);									//关闭接收器自动重开
	
	//ReadFromMsg(TagID, pdata, FRAME_POLL_TAG_ADDR_IDX, 2);																	
//	WriteToMsg(TxRespMsg, TagID, FRAME_RESP_TAG_ADDR_IDX, 2);																//设置Resp发送帧标签地址
//	WriteToMsg(RxFinalMsg, TagID, FRAME_FINAL_TAG_ADDR_IDX, 2);															//设置Final接收帧标签地址

  gAncDev.devState = UWB_RX_POLL_STATE;

	SetFirstAddr(gTxRespMsg, tagId);     //设置Resp发送帧标签地址

	
	gLocTime.t2 = GetRxTimeStamp_u64();																							//获得Poll包接收时间T2
	
//	DwtWriteTxData(gTxRespMsg, sizeof(gTxRespMsg));                                 //写入发送数据
	AncWriteSendData(gTxRespMsg, sizeof(gTxRespMsg));
	
	DWT_SetRxAfterTxDelay(RESP_TX_TO_FINAL_RX_DLY_UUS);															//设置发送完成后开启接收延迟时间
	DWT_SetRxTimeOut(FINAL_RX_TIMEOUT_UUS);																					//接收超时时间
	curTime = GetSysTimeStamp_u64();
	

	respTxTime = (curTime + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	  //计算Response发送时间T3
	DWT_SetDelayedTRxTime(respTxTime);																							//设置Response发送时间T3
	



	if(DWT_StartTx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED)  != DWT_SUCCESS)    //延时发送 如果失败
	{
		AncErrorProcess(5);
	}
	gAncDev.devState = UWB_RES_POLL_STATE;
	
		Dw1000ProcPend(0, &err);
		gUwbAncState = DWT_Read32BitReg(SYS_STATUS_ID);
		if (gUwbAncState & SYS_STATUS_RXFCG)	//成功接收
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//清楚标志位
			frameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//数据长度
			
			if (frameLen <= ANC_RX_LEN_MAX)
			{
					DWT_ReadRxData(gAncRxBuff, frameLen, 0);																//读取接收数据
		
					if(gAncRxBuff[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_FINAL)								//Final帧
					{
							gAncDev.devState = UWB_RX_FINAL_STATE;
				
//						HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED4_PIN);
//						TOF_CommCnt++;
				
						 gLocTime.t3 = GetTxTimeStamp_u64();			//获得response发送时间T3
						 gLocTime.t6 = GetRxTimeStamp_u64();			//获得final接收时间T6
								
						FinalMsgGetTS_64(&gAncRxBuff[FINAL_MSG_POLL_TX_TS_IDX], &gLocTime.t1);				//从接收数据中读取T1，T4，T5
						FinalMsgGetTS_64(&gAncRxBuff[FINAL_MSG_RESP_RX_TS_IDX], &gLocTime.t4);
						FinalMsgGetTS_64(&gAncRxBuff[FINAL_MSG_FINAL_TX_TS_IDX], &gLocTime.t5);
						caculate_distance(gLocTime,  &tof_dtu);
												
						gDist = (uint16_t)tof_dtu;
						DwLocSucProcess();
						AddDistData(tagId, gDist);
						
						gAncDev.devState = UWB_WAIT_STATE;
					}
			}
			else  //不应该到达这里
			{
				AncErrorProcess(6);
			}
//			DWT_Write8BitOffsetReg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
		}
		else
			UwbRxErrorProcess();
		if(gUwbAncState & SYS_STATUS_RXRFTO) //等待超时
		{
			WaiteOverErrProcess();
		}
//		else
//		{
//			DwRxDelayErrProcess();
//		}
//		DWT_SetAutoRxReEnable(DISABLE);									//关闭接收器自动重开
	
}
	/*******************************************************************************************
* 函数名称：UWB_Anchor(void *pdata)
* 功能描述：UWB作为基站时的执行函数
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void UwbAnchorTask(void *pdata)
{
	INT8U 		err;

	uint16_t  frameLen = 0;
	FrameConField frameCon;
	
	Dist_ListDataInit(&gDistDataBuf);
	
	CreatUwbDebugTask();
	
	WaitUwbStartSem();   //等待配置信息
	
  gDwAnchorConfig.eui =  DWT_GetPartID();
  Dw1000InitConfig(&gDwAnchorConfig);	
	
	gAncDev.devNum = GetUwbId();
	DWT_SetAddress16(gAncDev.devNum);		//设置帧过滤地址

	SetAnchorAddr(gAnc_CommDecResp, gAncDev.devNum);  //设置定位声明帧的应答帧基站地址
	SetAnchorAddr(gTxRespMsg, gAncDev.devNum);				//设置Resp发送帧基站地址
	
	LocationDataAppInit();
	
	
	DWT_SetAutoRxReEnable(ENABLE);	
	DWT_EnableFrameFilter(SYS_CFG_FF_ALL_EN);						//是能帧过滤
	
//	dwt_setdblrxbuffmode(1);
	
//	gUwbAncState = DWT_Read32BitReg(SYS_STATUS_ID);
//	
//	if((gUwbAncState & SYS_STATUS_HSRBP) != ((gUwbAncState&SYS_STATUS_ICRBP) >> 1))
//	{
//		DWT_Write8BitOffsetReg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
//	}
	
  DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
	DWT_SetInterrupt(DWT_INT_ALL, ENABLE);			  //开启中断
//	DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
	
	for(;;)
	{
//		DWT_EnableFrameFilter(SYS_CFG_FFAB);  			//只接受信标帧
//		DWT_SetAutoRxReEnable(ENABLE);							//开启接收器自动开启（接收失败后开启，接收成功后不开启，用户手册72页）
		
		
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
		
		SetDebugCount();
		Dw1000ProcPend(0, &err);
		gUwbAncState = DWT_Read32BitReg(SYS_STATUS_ID);
		if (gUwbAncState & SYS_STATUS_RXFCG)	//成功接收
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//清除标志位
			
			frameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//获得接收数据长度			
			if(frameLen <= ANC_RX_LEN_MAX)
			{
				DWT_ReadRxData(gAncRxBuff, frameLen, 0);														//读取接收数据
				frameCon.frameControl = *((uint16_t *)gAncRxBuff);
				
				if(frameCon.FrameConBit.sorcAddrMode == ID_LENGTH_16)    //16位地址
				{
					switch (frameCon.FrameConBit.frameType)
					{
						case BEACON_FRAME:     //信标帧						
								if(gAncRxBuff[BEACON_FRAME_TYPE_IDX] == FRAME_COMM_DEC) //定位信标帧处理
								{
									 AncDeclareFrameProcess(gAncRxBuff, frameLen);
								}
								else
								{
									;//其他信标帧处理
								}
							break;
						
						case    DATA_FRAME:                                             //数据帧处理
						{
//							if(gAncDev.devState == UWB_LOCATION_STATE)     //正在进行标签定位处理
							{
								if(*((uint16_t *)(gAncRxBuff + FIRST_ADDR_OFF)) == gAncDev.devNum)  //接收帧是发给自己的
								{
//									gAncDev.rxDataNum++;
									AncPollProcess(gAncRxBuff, frameLen);
								}
								else //错误，不应该到达这里 其他数据帧
								{
									AncErrorProcess(7);
								}
							}
						}
						break;
						default:
							break;
					}
				}
				else
				{
					AncErrorProcess(8);//错误的地址 格式
				}
			}

//			{
//				DWT_Write8BitOffsetReg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
//			}
    }//接收成功
		else
				UwbRxErrorProcess();
//			if (gUwbAncState & SYS_STATUS_ALL_RX_ERR)
//		{
//			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//			DWT_ForceTRxOff();
//			DWT_RxReset();
//		}
//		else if(gUwbAncState & SYS_STATUS_TXFRS)
//    {
//      DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); 
//		}
//		else if(gUwbAncState & SYS_STATUS_ALL_RX_TO)
//    {
//        DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bits

//        DWT_ForceTRxOff();
//        DWT_RxReset();

//    }
//		else
//		{
//			gUwbAncErrState = gUwbAncState;
//		}
	}
}


/*******************************************************************************************
* 函数名称：UWB_Anchor(void *pdata)
* 功能描述：UWB作为基站时的执行函数
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void AncIrqDeclareFrameProcess(uint8_t * pdata, uint16_t len)
{
//	INT8U 		err;
//	uint64_t frameRxTime = 0; 
//	uint64_t frameTxTime = 0;
//	uint64_t curTime = 0;
	gAncDev.devState = UWB_RX_BEA_STATE;    														//设置设备状态为定位状态
	gAncDev.beaconNum++;
//	gAncDev.currDevId = *((uint16_t *)(pdata + FIRST_ADDR_OFF));          //当前定位的标签id号
	
	memcpy(gAnc_CommDecResp + FIRST_ADDR_OFF, pdata + FIRST_ADDR_OFF, 2); 	 //设置应答的目的地址
	
	
	DWT_WriteTxData(sizeof(gAnc_CommDecResp), gAnc_CommDecResp, 0);		//写入发送数据
	DWT_WriteTxfCtrl(sizeof(gAnc_CommDecResp), 0, 1);								  //设定发送长度
	
	gAncDev.txBeaNum++;
	if(DWT_StartTx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
	{
		AncErrorProcess(1);
	}
	
	gAncDev.devState = UWB_RES_BEA_STATE;  

//	DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
//	DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收	

}


void AncRxPollFrameProcess(uint16_t tagId, uint8_t *pdata, uint16_t len)
{
	uint64_t curTime = 0;
	uint32_t respTxTime;
	
		gAncDev.devState = UWB_RX_POLL_STATE;
		gAncDev.rxPollNum++;
		
		gAncDev.currDevId = tagId;
		SetFirstAddr(gTxRespMsg, tagId);     //设置Resp发送帧标签地址	
		gLocTime.t2 = GetRxTimeStamp_u64();																							//获得Poll包接收时间T2
		AncWriteSendData(gTxRespMsg, sizeof(gTxRespMsg));
		
		DWT_SetRxAfterTxDelay(RESP_TX_TO_FINAL_RX_DLY_UUS);															//设置发送完成后开启接收延迟时间
		DWT_SetRxTimeOut(FINAL_RX_TIMEOUT_UUS);																					//接收超时时间
		curTime = GetSysTimeStamp_u64();
	
		respTxTime = (gLocTime.t2 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	  //计算Response发送时间T3
		DWT_SetDelayedTRxTime(respTxTime);																							//设置Response发送时间T3
	
		
	//	gAncDev.txPollNum++;
		if(DWT_StartTx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED)  != DWT_SUCCESS)    //延时发送 如果失败
		{
			DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
//			AncErrorProcess(5);
			
		}
		gAncDev.devState = UWB_RES_POLL_STATE;
}

void AncRxFinalFrameProcess(uint16_t tagId, uint8_t *pdata, uint16_t len)
{
		uint64_t tof_dtu;
		gAncDev.devState = UWB_RX_FINAL_STATE;
		gAncDev.rxFinalNum++;
 
		 gLocTime.t3 = GetTxTimeStamp_u64();			//获得response发送时间T3
		 gLocTime.t6 = GetRxTimeStamp_u64();			//获得final接收时间T6
				
		FinalMsgGetTS_64(&gAncRxBuff[FINAL_MSG_POLL_TX_TS_IDX], &gLocTime.t1);				//从接收数据中读取T1，T4，T5
		FinalMsgGetTS_64(&gAncRxBuff[FINAL_MSG_RESP_RX_TS_IDX], &gLocTime.t4);
		FinalMsgGetTS_64(&gAncRxBuff[FINAL_MSG_FINAL_TX_TS_IDX], &gLocTime.t5);
		caculate_distance(gLocTime,  &tof_dtu);
								
		gDist = (uint16_t)tof_dtu;
		DwLocSucProcess();
		AddDistData(tagId, gDist);
	 
	 gAncDev.devState = UWB_WAIT_STATE;
}


void AncDataFrameProcess(uint8_t *pdata, uint16_t len)
{
//	uint32_t respTxTime;
	uint16_t tagId;
	
//	uint64_t curTime = 0;
//	uint32_t frameLen;

	
//	INT8U 		err;
	
	tagId = *((uint16_t *)(pdata + SECOND_ADDR_OFF));  
	
	if(pdata[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_POLL)								//poll帧
	{	
			AncRxPollFrameProcess(tagId, pdata, len);
	}
	else if (pdata[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_FINAL)
	{
		 if(tagId == gAncDev.currDevId)
		 {
				AncRxFinalFrameProcess(tagId, pdata, len);
		 }
		 else
		 {
			 gAncDev.idErr++;
		 }
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
	}
	else
	{
		;
	}	
}

static void tx_conf_cb(const dwt_cb_data_s *cb_data)
{
	if(gAncDev.devState == UWB_RES_BEA_STATE)
	{
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收;
	}
}


static void rx_to_cb(const dwt_cb_data_s *cb_data)
{
	if(gAncDev.devState == UWB_RES_POLL_STATE)
	{
		gAncDev.waitErr++;
		gAncDev.devState = UWB_WAIT_STATE;
	}
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
}

static void rx_err_cb(const dwt_cb_data_s *cb_data)
{
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
}

static void else_err_cb(const dwt_cb_data_s *cb_data)
{
		DWT_ForceTRxOff(); // Turn the RX off
		DWT_RxReset(); // Reset in case we were late and a frame was already being received
		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
}


static void UwbRxOkCb(const dwt_cb_data_s *cb_data)
{
	FrameConField frameCon;
	 if (cb_data->datalength <= ANC_RX_LEN_MAX)
	 {
			
			DWT_ReadRxData(gAncRxBuff, cb_data->datalength, 0);														//读取接收数据
			frameCon.frameControl = *((uint16_t *)gAncRxBuff);
			
			if(frameCon.FrameConBit.sorcAddrMode == ID_LENGTH_16)    //16位地址
			{
				switch (frameCon.FrameConBit.frameType)
				{
					case BEACON_FRAME:     //信标帧						
							if(gAncRxBuff[BEACON_FRAME_TYPE_IDX] == FRAME_COMM_DEC) //定位信标帧处理
							{
								 AncIrqDeclareFrameProcess(gAncRxBuff, cb_data->datalength);
							}
							else
							{
								;//其他信标帧处理
							}
						break;
					
					case    DATA_FRAME:                                             //数据帧处理
					{
//							if(gAncDev.devState == UWB_LOCATION_STATE)     //正在进行标签定位处理
						{
							if(*((uint16_t *)(gAncRxBuff + FIRST_ADDR_OFF)) == gAncDev.devNum)  //接收帧是发给自己的
							{
//								gAncDev.rxDataNum++;
								AncDataFrameProcess(gAncRxBuff, cb_data->datalength);
							}
							else //错误，不应该到达这里 其他数据帧
							{
								AncErrorProcess(7);
							}
						}
					}
					break;
					default:
						break;
				}
			}
			else
			{
				AncErrorProcess(8);//错误的地址 格式
			}	 
	}
}

void UwbAnchorIrqTask(void *pdata)
{
	INT8U 		err;

	uint16_t  frameLen = 0;
	FrameConField frameCon;
	
	Dist_ListDataInit(&gDistDataBuf);
	
	CreatUwbDebugTask();
#if CAN_TASK_MODE == CAN_POSITION_MODE	
	WaitUwbStartSem();   //等待配置信息
#endif 	
  gDwAnchorConfig.eui =  DWT_GetPartID();
  Dw1000InitConfig(&gDwAnchorConfig);	
	
	gAncDev.devNum = GetUwbId();
	DWT_SetAddress16(gAncDev.devNum);		//设置帧过滤地址

	SetAnchorAddr(gAnc_CommDecResp, gAncDev.devNum);  //设置定位声明帧的应答帧基站地址
	SetAnchorAddr(gTxRespMsg, gAncDev.devNum);				//设置Resp发送帧基站地址
	
	LocationDataAppInit();
	
	dwt_setcallbacks(&tx_conf_cb, &UwbRxOkCb, &rx_to_cb, &rx_err_cb, &else_err_cb);
	DWT_SetAutoRxReEnable(ENABLE);	
	DWT_EnableFrameFilter(SYS_CFG_FF_ALL_EN);						//是能帧过滤
	
  DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
	DWT_SetInterrupt(DWT_INT_ALL|DWT_INT_TFRS, ENABLE);			  //开启中断
	DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
	
	for(;;)
	{
//		DWT_EnableFrameFilter(SYS_CFG_FFAB);  			//只接受信标帧
//		DWT_SetAutoRxReEnable(ENABLE);							//开启接收器自动开启（接收失败后开启，接收成功后不开启，用户手册72页）
		
		Dw1000ProcPend(0, &err);
//		DWT_SetRxTimeOut(0);												//设定接收超时时间，0--没有超时时间，无限等待
//		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
//		
//		SetDebugCount();
	}

}




