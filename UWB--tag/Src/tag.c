#include "tag.h"
#include "anchor.h"
#include "address.h"
#include "BEB.h"
#include "crc.h"
#if defined(USE_OLED)
	#include "oled.h"
#endif

#define  FLASH_INIT_VALUE					0xFFFFFFFFFFFFFFFF   //FLASH初始化值


Tag_Config_s sTagConfig;
extern DWT_AddrStatus_s 	sDWT;

OS_EVENT  *TagCommSem;			//标签通讯信号量
OS_EVENT  *ID_ConfigSem;		//ID配置信号量

uint16_t WaitForPower = 9000;		//上电等待时间：10s，防止电池馈电时，充电负载电流太大，充不进去

/*标签通讯帧定义*/
uint8_t TxPollMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 'V', 'E', FRAME_POLL, 0, 0,};							//POLL帧
uint8_t RxRespMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 0xFF, 0xFF, FRAME_RESP, 0x02, 0, 0, 0, 0};	//RESP帧
uint8_t TxFinalMsg[] = {
												0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 'V', 'E', FRAME_FINAL, 
												0x00, 0x00, 0x00, 0x00, 0x00,
												0x00, 0x00, 0x00, 0x00, 0x00,
												0x00, 0x00, 0x00, 0x00, 0x00,
												0, 0
											 };																																								//FINAL帧
uint8_t Tag_CommDec[] = {0x00, 0x80, 0x00, 0xCA, 0xDE, 0x00, 0x00, FRAME_COMM_DEC, 0x00, 0x00};					//通讯声明帧

static uint8_t UsartFrameHeader[] = {0x41, 0x8C, 0, 0xFF, 0xFF, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, FRAME_CONFIG_ID};	//无线--串口帧帧头
static uint8_t UsartFramePayLoad[1024];

static uint8_t RxBuffer[RX_BUF_LEN_MAX];
static uint32_t FrameSeqNum = 0;
static uint32_t TagStatusReg = 0;
											 
static uint64_t PollTxTS;
static uint64_t RespRxTS;
static uint64_t FinalTxTS;

/****************************64Bit FLASH数据*************************************
* DWT_WriteToFlash
*  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  
* |     |          |                  |                   |
* |Bit63| Reserved |   Bit47--Bit32   |     Bit31--Bit0   |
* |_ _ _|_ _ _ _ _ |_ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _|
* 
* Bit31--Bit0 :32位CHIP ID
* Bit47--Bit32:16位短地址
* Bit59--Bit56:模块工作模式：0--标签；1--从基站；2--主基站；3--CLE
* Bit63:       模块短地址标识位：0--未分配短地址；1--已分配短地址
*
* TAG_WriteToFlash
*  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
* |							 |              |              |              |             |
* | Bit63--Bit48 | Bit47--Bit40 | Bit39--Bit32 | Bit31--Bit16 | Bit15--Bit0 |
* |_ _ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _ _ _|
* 
* Bit15--Bit0:  Perm
* Bit31--Bit16: JobNum
* Bit39--Bit32: Age
* Bit47--Bit40: Sex
* Bit63--Bit48: MAC ID
*********************************************************************************/
uint64_t DWT_ReadFromFlash;
uint64_t DWT_WriteToFlash;

uint64_t TAG_ReadFromFlash;
uint64_t TAG_WriteToFlash;

uint16_t CollisionWindow = 1;				//随机回退冲突窗口Cw
uint8_t TagExitWorkSpaceTimBegin;		//声明失败标志位，用来开启Tag离面判定计时器
uint8_t TagExitWorkSpace;						//Tag离开工作面标志位：0--Tag在工作面内，1--Tag在工作面外

uint32_t TagExitWorkSpaceTIM;		//Tag离面判定计时器
uint32_t TagCommCycleTIM;				//Tag通讯周期计时器

uint16_t AncAddr_Center;
uint16_t AncAddr_CommBegin;
uint16_t AncAddr_Comm;

uint8_t TagCommToAncNum;			//本轮通讯中，标签需要与之通讯的基站的数量

uint8_t KeepInConfig = 0;			//保持标签配置模式标志，1--保持配置模式，0--退出配置模式

uint8_t TagUsartCrcRepeatNum = 0;	//CRC错误次数

/*******************************************************************************************
* 函数名称：void Tag_Reset_Blink(void)
* 功能描述：系统复位闪烁
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void Tag_Reset_Blink(void)
{
	uint8_t i;
	for(i = 0; i < 10; i++)
	{
		HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED3_PIN | LED4_PIN);
		OSTimeDly(100);
	}
}


/*******************************************************************************************
* 函数名称：UWB_Tag(void *pdata)
* 功能描述：UWB作为标签时的执行函数
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
uint16_t CollisionWindowCnt = 0;

void UWB_Tag(void *pdata)
{
#if defined(USE_OLED)
	char ModelID[5];
#endif
	
	INT8U err;
	uint16_t RNG_Num;
	DWT_SetAddress16(sDWT.ShortAddr);

//	DWT_ConfigEventCounters(ENABLE);			//开启事件计数器
	
	WriteToMsg(TxPollMsg, (uint8_t *)&sDWT.ShortAddr, FRAME_SOURCE_ADDR_IDX, 2, 0);			//设置Poll发送帧，标签地址
	WriteToMsg(TxFinalMsg, (uint8_t *)&sDWT.ShortAddr, FRAME_SOURCE_ADDR_IDX, 2, 0);		//设置Final发送帧，标签地址
	WriteToMsg(Tag_CommDec, (uint8_t *)&sDWT.ShortAddr, FRAME_DEC_TAG_ADDR_IDX, 2, 0);	//设置通讯声明帧，标签地址
	
#if defined(USE_OLED)
	sprintf(ModelID, "%04d", sDWT.ShortAddr);
	OLED_ShowString(0, 0, "TagID:", 16);
	OLED_ShowString(48, 0, ModelID, 16);
#endif
	
	ID_ConfigSem = OSSemCreate(0);	//建立ID配置信号量
	
	Tag_Reset_Blink();				//系统闪烁指示
	OSTimeDly(WaitForPower);	//延时启动任务，10s
	
	FLASH_ReadBytes(TAG_INFO_ADDR, SHORT_ID_ADDR_OFFECT, (uint8_t *)&DWT_ReadFromFlash, 8);
	if(((DWT_ReadFromFlash >> UWB_16BIT_ADDR_FLAG_IDX) & 0x0001) && (DWT_ReadFromFlash != FLASH_INIT_VALUE))		//MAC ID已配置，进入标签任务
	{
		TagCommSem = OSSemCreate(1);		//建立标签通讯信号量
	}
	else
	{
		TagCommSem = OSSemCreate(1);		//建立标签通讯信号量
		//进入睡眠模式
	}

	for(;;)
	{
		if((TagExitWorkSpaceTimBegin == 0) || TagExitWorkSpace == 1)		//声明成功或者标签已离面
		{
//			DWT_EnterSleep();																		//DWM1000开始睡眠
			OSSemPend(TagCommSem,0,&err);
//			DWT_SpiCsWakeup(DummyBuffer, DUMMY_BUFFER_LEN);			//唤醒DWT
			TagCommCycleTIM = (TagExitWorkSpace == 0 ? HAL_GetTick() + SYSTEM_COMM_CYCLE_MS : HAL_GetTick() + SYSTEM_COMM_CYCLE_LP_MS);		//通讯周期计时器开启
		}
		
		HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED4_PIN);
		
		if(ListenChannel() == ChannelFree)	//监听信道是否空闲
		{
			if(CommDec())											//发送通讯声明，获取通讯权
			{
				uint8_t i, j;
				
				if(TagExitWorkSpace == 1)
				{
					TagCommCycleTIM = TagCommCycleTIM + SYSTEM_COMM_CYCLE_MS - SYSTEM_COMM_CYCLE_LP_MS;			//离面状态转换为面内状态，更改通讯周期
				}
				
				TagExitWorkSpaceTimBegin = 0;		//初始化声明失败标志位
				TagExitWorkSpace = 0;						//标签进入工作面
				
				if(AncAddr_Center < 0x02)				//判断标签是否处于边界位置
				{
					AncAddr_CommBegin = 0;																													//位于头边界，开始通讯基站ID为0
					TagCommToAncNum = (TAG_COMM_TO_ANC_NUM_MAX + AncAddr_Center) - 0x02;						//周期内的通讯基站数量减少
				}
				else if(AncAddr_Center > (ANC_NUM - 2))
				{
					AncAddr_CommBegin = AncAddr_Center - 0x02;																			//位于尾边界，开始通讯基站ID
					TagCommToAncNum = (TAG_COMM_TO_ANC_NUM_MAX + ANC_NUM) - AncAddr_Center - 0x02;	//周期内的通讯基站数量减少
				}
				else
				{
					AncAddr_CommBegin = AncAddr_Center - 0x02;																			//未到达边界位置，开始通讯基站ID
					TagCommToAncNum = TAG_COMM_TO_ANC_NUM_MAX;																			//周期内的通讯基站数量正常
				}
/**************************测试代码****************************/
//				TagCommToAncNum = 1;
//				AncAddr_Comm = AncAddr_Center;
/**************************************************************/
				
				for(i = 0; i < TagCommToAncNum; i++)																							//准备和基站依次进行通讯
				{
					AncAddr_Comm = AncAddr_CommBegin + i;																						//计算基站地址
					
//					TimeDlyForUS_380 = 1;																													//延时380us
					
					WriteToMsg(TxPollMsg, (uint8_t *)&AncAddr_Comm, FRAME_DEST_ADDR_IDX, 2, 0);			//设置Poll帧基站地址
					WriteToMsg(TxFinalMsg, (uint8_t *)&AncAddr_Comm, FRAME_DEST_ADDR_IDX, 2, 0);		//设置Final帧基站地址
					
					for(j = 0; j < TAG_COMM_TO_ANC_CNT; j++)																				//和当前基站进行多次通讯
					{
						TAG_TOF();																																		//进行测距通讯
						if(j < (TAG_COMM_TO_ANC_CNT - 1))
						{
							OSTimeDly(2);																																//如果和当前基站不是最后一次通讯，则延迟1ms，等待基站计算数据完毕
						}
					}
				}
			}//CommDec
			else
			{
				if(TagExitWorkSpaceTimBegin == 0)
				{
					TagExitWorkSpaceTimBegin = 1;		//第一次声明失败,开始离面判定计时
					TagExitWorkSpaceTIM = HAL_GetTick() + TAG_EXIT_WORKSPACE_JUDGE_TIME_MS;
				}
				if(TagExitWorkSpace == 0)					//如果在工作面内，则随机回退，继续监听信道，否则睡眠
				{
					CollisionWindow = (CollisionWindow * 2 >= (100 / CSMA_CD_MS) ? (100 / CSMA_CD_MS) : CollisionWindow * 2);
					CollisionWindowCnt = ((CollisionWindow == 100 / CSMA_CD_MS) ? CollisionWindowCnt + 1 : CollisionWindowCnt);
					RNG_Num = RNG_Get_RandomRange(0, CollisionWindow);							//获取随机回退数值
					OSTimeDly(RNG_Num * CSMA_CD_MS);
				}
			}
		}//ListenChannel
		else
		{
			OSTimeDly(SYSTEM_COMM_CYCLE_MS);
		}
	}
}


/*******************************************************************************************
* 函数名称：void TAG_TOF(void)
* 功能描述：tof测距
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TAG_TOF(void)
{
	int8_t StratTxStatu = -1;
	
	DWT_EnableFrameFilter(SYS_CFG_FFAD);					//接收数据帧
	DWT_SetAutoRxReEnable(ENABLE);								//接收器自动重开
	
	DWT_SetInterrupt(DWT_INT_ALL, DISABLE);				//关闭中断，否则从基站切换到标签时，会产生接收中断恢复基站任务

	DWT_SetRxAfterTxDelay(POLL_TX_TO_RESP_RX_DLY_UUS);		//设置发送后开启接收，并设定延迟时间，单位：uus
	DWT_SetRxTimeOut(RESP_RX_TIMEOUT_UUS);								//设置接收超时时间，单位：uus

//	TxPollMsg[ALL_MSG_SN_IDX] = FrameSeqNum;
	DWT_WriteTxData(sizeof(TxPollMsg), TxPollMsg, 0);			//将Poll包数据传给DW1000，将在开启发送时传出去
	DWT_WriteTxfCtrl(sizeof(TxPollMsg), 0, 0);						//设置超宽带发送数据长度

	StratTxStatu = DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);		//开启发送，发送完成后等待一段时间开启接收，等待时间在dwt_setrxaftertxdelay中设置

//	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//不断查询芯片状态直到发送完成
//	{ };
	
//	FrameSeqNum++;
	
	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//不断查询芯片状态直到成功接收或者发生错误或者接收超时
	{ };
	
	if (TagStatusReg & SYS_STATUS_RXFCG)//如果成功接收
	{
		uint16 frame_len;
		
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//清除寄存器标志位

		frame_len = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//获得接收到的数据长度
		if (frame_len <= RX_BUF_LEN)
		{
			DWT_ReadRxData(RxBuffer, frame_len, 0);   														//读取接收数据
		}
		if (RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_RESP)									//判断接收到的数据是否是response数据
		{
			uint32 final_tx_time;
			
			PollTxTS = GetTxTimeStamp_u64();																			//获得POLL发送时间T1
			RespRxTS = GetRxTimeStamp_u64();																			//获得RESPONSE接收时间
			
			final_tx_time = (RespRxTS + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//计算final包发送时间，T5=T4+Treply2
			DWT_SetDelayedTRxTime(final_tx_time);																								//设置final包发送时间T5
			
			FinalTxTS = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;						//final包实际发送时间是计算时间加上发送天线delay
			
			FinalMsgSetTS(&TxFinalMsg[FINAL_MSG_POLL_TX_TS_IDX], PollTxTS);					//将T1，T4，T5写入发送数据
			FinalMsgSetTS(&TxFinalMsg[FINAL_MSG_RESP_RX_TS_IDX], RespRxTS);
			FinalMsgSetTS(&TxFinalMsg[FINAL_MSG_FINAL_TX_TS_IDX], FinalTxTS);
			
			TxFinalMsg[ALL_MSG_SN_IDX] = FrameSeqNum;
			
			DWT_WriteTxData(sizeof(TxFinalMsg), TxFinalMsg, 0);											//将发送数据写入DW1000
			DWT_WriteTxfCtrl(sizeof(TxFinalMsg), 0, 1);															//设定发送数据长度	
			
			StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED);												//设定为延迟发送
			
			while (!(DWT_Read32BitReg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))						//不断查询芯片状态直到发送完成
			{}
				
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_TXFRS);				//清楚标志位
//			HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED3_PIN);					//LED绿灯闪烁
		}
	}
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS);
		DWT_RxReset();
	}
	
	DWT_SetAutoRxReEnable(DISABLE);
	DWT_ForceTRxOff();
	DWT_EnableFrameFilter(DISABLE);		//关闭帧过滤
}


/*******************************************************************************************
* 函数名称：uint8_t CommDec(void)
* 功能描述：标签定位通讯声明
* 入口参数：无
* 出口参数：
* 使用说明：无
********************************************************************************************/
uint32_t DWT_Statu_Dec;

uint8_t CommDec(void)
{
	int8_t StratTxStatu;
	uint16_t FrameLen;
//	DWT_EnableFrameFilter(SYS_CFG_FFAD);  			//只接受数据帧
//	DWT_SetAutoRxReEnable(ENABLE);							//开启接收器自动重开
	
	DWT_ForceTRxOff();
	
	DWT_SetRxAfterTxDelay(FRAME_COMM_DEC_TX_TO_RESP_RX_DLY_UUS);				//设置发送完成后开启接收延迟时间
	DWT_SetRxTimeOut(FRAME_COMM_DEC_RESP_RX_TIMEOUT_UUS);								//接收超时时间
	
	DWT_WriteTxData(sizeof(Tag_CommDec), Tag_CommDec, 0);								//写入发送数据
	DWT_WriteTxfCtrl(sizeof(Tag_CommDec), 0, 1);												//设定发送长度
	
//	DWT_Statu_Dec = DWT_Read32BitReg(SYS_STATUS_ID);
	
	StratTxStatu = DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);				//立即发送，延迟接收
	
//	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_TXFRS)))	//不断查询芯片状态直到接收成功或者发生错误
//	{}
	
	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//不断查询芯片状态直到接收成功或者发生错误
	{}
	
//	DWT_EnableFrameFilter(DISABLE);  			//关闭真过滤
//	DWT_SetAutoRxReEnable(DISABLE);				//关闭接收器自动重开
	
	if(TagStatusReg & SYS_STATUS_RXFCG)
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清除标志位
		
		FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;		//获得接收数据长度
		if (FrameLen <= RX_BUF_LEN)
		{
			DWT_ReadRxData(RxBuffer, FrameLen, 0);															//读取接收数据
			
			if(RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_COMM_DEC_RESP)			//判断帧类型
			{
				ReadFromMsg((uint8_t *)&AncAddr_Center, RxBuffer, FRMAE_DEC_RESP_ANC_ADDR_IDX, 2, 0);		//读取基站地址
				return 1;
			}
		}
	}
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_TX);
//		DWT_RxReset();
		DWT_ForceTRxOff();
	}
	
	return 0;
}


/*******************************************************************************************
* 函数名称：ChannelStatu ListenChannel(void)
* 功能描述：信道监听
* 入口参数：无
* 出口参数：当前信道状态：ChannelFree -- 1，ChannelBusy -- 0
* 使用说明：无
********************************************************************************************/
ChannelStatu ListenChannel(void)
{
	uint16_t RngNum;
	uint16_t FrameLen;
	uint8_t ListenCnt = 0;
	
	DWT_SetRxTimeOut(LISTEN_CHANNEL_TIME);				//设定接收超时时间
	
	while(1)
	{
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//打开接收
		
		while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR_CLE)))	//不断查询芯片状态直到接收成功或者出现错误
		{}
		
		if ((TagStatusReg & SYS_STATUS_RXRFTO) && ((TagStatusReg & SYS_STATUS_ALL_RX_GOOD) == 0))	//接收超时，且接收器当前并未正在接收数据帧。表示信道空闲
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);		//清除标志位
			
			RngNum = RNG_Get_RandomRange(0, CollisionWindow);				//获取随机回退值
			OSTimeDly(RngNum * CSMA_CD_MS);
			
			DWT_RxEnable(DWT_START_RX_IMMEDIATE);										//打开接收
			
			while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR_CLE)))	//不断查询芯片状态直到接收成功或者出现错误
			{}
				
			if ((TagStatusReg & SYS_STATUS_RXRFTO) && ((TagStatusReg & SYS_STATUS_ALL_RX_GOOD) == 0))												//接收超时，且接收器当前并未正在接收数据帧。表示信道空闲
			{
				DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);	//清除标志位
				
				ListenCnt = 0;							//清除冲突次数
				CollisionWindow = (CollisionWindow / 2 == 0 ? 1 : CollisionWindow / 2);				//降低竞争窗口大小
//				CollisionWindow /= 2;
				
				return ChannelFree;
			}
			else if(TagStatusReg & SYS_STATUS_RXFCG)																		
			{
				DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);	//清除标志位
				
				FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//数据长度
			
				if (FrameLen <= RX_BUF_LEN)
				{
					DWT_ReadRxData(RxBuffer, FrameLen, 0);																//读取接收数据
					
					if((RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_COMM_DEC) || (RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_COMM_DEC_RESP))			//是否收到声明帧或者声明回复帧
					{
						OSTimeDly(TAG_COMM_TO_ALL_ANC_TIME_MS);
					}
				}
			}
		}//信道空闲
		else
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_RX_GOOD);
			DWT_RxReset();
			TagExitWorkSpaceTimBegin = 0;			//
		}
		ListenCnt++;
		CollisionWindow = (CollisionWindow * 2 >= (100 / CSMA_CD_MS) ? (100 / CSMA_CD_MS) : CollisionWindow * 2);
		CollisionWindowCnt = ((CollisionWindow == 100 / CSMA_CD_MS) ? CollisionWindowCnt + 1 : CollisionWindowCnt);
		
		if(ListenCnt == 16)					//连续冲突16次
		{
			return ChannelBusy;
		}
	}
}



/*******************************************************************************************
* 函数名称：void UWB_ID_Config(void *pdata)
* 功能描述：标签ID配置函数--申请，更改，删除
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void UWB_ID_Config(void *pdata)
{
	INT8U err;
	
	for(;;)
	{
		if(KeepInConfig == 0)		//判断是否保持在配置模式
		{
			OSSemPend(ID_ConfigSem, 0, &err);
		}
		
		DWT_ForceTRxOff();
		
		DWT_SetRxTimeOut(0);		//设定接收超时时间，0--没有超时时间，无限等待
		
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);	//打开接收器
		
		while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//不断查询芯片状态直到接收成功或者出现错误
		{ }
		
		if ((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG)	//成功接收
		{
			uint16_t FrameLen;
			uint16_t FrameCotrl;
			
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//清除标志位
			
			KeepInConfig = 1;	//置位标志位
			
			FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if(FrameLen <= RX_BUF_LEN_MAX)
			{
				uint16_t Crc, UsartLen;
				
				DWT_ReadRxData(RxBuffer, FrameLen, 0);	//读取无线接收数据
				FrameCotrl = *((uint16_t *)RxBuffer);
				
				if((FrameCotrl & FRAME_CTRL_BYTE_ADDR_LEN_BIT) == FRAME_CTRL_BYTE_DEST_ID_64BIT)	//64Bit数据帧
				{
					uint64_t DestAddr;
					
					ReadFromMsg((uint8_t *)&DestAddr, RxBuffer, FRAME_64BIT_DEST_ADDR_IDX, 8, 0);		//读取接收地址
					
					if(sTagConfig.ChipID == DestAddr)	//接收地址是自己的
					{
						UsartLen = (((uint16_t)RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_LEN_IDX]) << 8) + RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_LEN_IDX + 1];	//读取USART数据长度
						Crc = (((uint16_t)RxBuffer[FRAME_64BIT_DATA_IDX + UsartLen + 2]) << 8) + RxBuffer[FRAME_64BIT_DATA_IDX + UsartLen + 3];		//读取USART CRC值
						
						if(CheckCRC(Crc, RxBuffer, FRAME_64BIT_DATA_IDX, UsartLen + 2))	//校验CRC
						{
							TAG_ConfigID(RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_TYPE_IDX]);
							TAG_ConfigID_ACK(RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_TYPE_IDX]);
						}
						else
						{
							if(TagUsartCrcRepeatNum < USART_FRAME_REPEAT_MAX)
							{
								TAG_ConfigID_ACK(USART_FRAME_REPEAT);
							}
							else
							{
								TAG_ConfigID_ACK(USART_FRAME_ERR);
							}
						}
					}						
				}
			}
		}
		else
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_TX);
			DWT_ForceTRxOff();
		}
	}
}



/*******************************************************************************************
* 函数名称：TAG_ConfigID(uint8_t frametype)
* 功能描述：标签配置ID信息
* 入口参数：frametype--接收到的帧类型
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TAG_ConfigID(uint8_t frametype)
{
	assert_param(IS_USART_FRAME_TYPE(frametype));
	
	switch(frametype)
	{
		case USART_FRAME_SET_ALL:
			TAG_ReadAllConfigInfo();
		case USART_FRAME_SET_CHIPID:
			break;
		case USART_FRAME_SET_MACID:
		{
			ReadFromMsg((uint8_t *)&sTagConfig.MAC_ID, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_MACID_LEN, 1);	//读取并存储MAC ID
			sDWT.ShortAddr = sTagConfig.MAC_ID;
			sDWT.ShortAddrFlag = 1;
			break;
		}
		case USART_FRAME_SET_NAME:
			ReadStringFromMsg(sTagConfig.Name, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);															//读取并存储姓名
			break;
		case USART_FRAME_SET_SEX:
			ReadFromMsg((uint8_t *)&sTagConfig.Sex, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_SEX_LEN, 1);			//读取并存储性别
			break;
		case USART_FRAME_SET_AGE:
			ReadFromMsg((uint8_t *)&sTagConfig.Age, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_AGE_LEN, 1);			//读取并存储年龄
			break;
		case USART_FRAME_SET_JOBNUM:
			ReadFromMsg((uint8_t *)&sTagConfig.JobNum, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_JOBNUM_LEN, 1);//读取并存储工号
			break;
		case USART_FRAME_SET_DEPT:
			ReadStringFromMsg(sTagConfig.Department, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);												//读取并存储部门
			break;
		case USART_FRAME_SET_TITLE:
			ReadStringFromMsg(sTagConfig.Title, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);															//读取并存储职务
			break;
		case USART_FRAME_SET_WORKSPACE:
			ReadStringFromMsg(sTagConfig.WorkSpace, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);													//读取并存储工作区
			break;
		case USART_FRAME_SET_PERM:
			ReadStringFromMsg(sTagConfig.Perm, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);															//读取并存储权限
			break;
		case USART_FRAME_DEINIT:
			KeepInConfig = 0;
			break;
	}
}



/*******************************************************************************************
* 函数名称：TAG_ConfigID_ACK(uint8_t frametype)
* 功能描述：标签配置ID信息时回复时调用
* 入口参数：frametype--接收到的帧类型
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TAG_ConfigID_ACK(uint8_t frametype)
{
	uint16_t DataPayLoadLength;
	uint16_t Crc;
	
	assert_param(IS_USART_FRAME_TYPE(frametype));
	
	WriteToMsg(UsartFrameHeader, (uint8_t *)&sDWT.ChipID, FRAME_64BIT_SOURCE_ADDR_TX_IDX, 8, 0);
	DWT_WriteTxData(sizeof(UsartFrameHeader) + 2, UsartFrameHeader, 0);
	UsartFramePayLoad[0] = 0x5A;		//串口协议帧头
	
	switch(frametype)
	{
		case USART_FRAME_QUERY:
		{
			if(sDWT.ShortAddrFlag == 1)		//标签已配置，回复全部信息
			{
				uint8_t DataWriteOffect = USART_FRAME_CHIPID_IDX;	//初始化数据写入偏移量
				
				FLASH_ReadBytes(TAG_INFO_ADDR, MAC_ID_ADDR_OFFECT, (uint8_t *)&TAG_ReadFromFlash, 8);	//读取组合数据
				
				sTagConfig.Perm[0] = TAG_ReadFromFlash >> UWB_PERM_IDX;
				sTagConfig.JobNum = TAG_ReadFromFlash >> UWB_JOBNUM_IDX;
				sTagConfig.Age = TAG_ReadFromFlash >> UWB_AGE_IDX;
				sTagConfig.Sex = TAG_ReadFromFlash >> UWB_SEX_IDX;
				sTagConfig.MAC_ID = TAG_ReadFromFlash >> UWB_MAC_ID_IDX;
				
				FLASH_ReadBytes(TAG_INFO_ADDR, CHIPID_ADDR_OFFECT, (uint8_t *)&sTagConfig.ChipID, CONFIG_CHIPID_LEN);									//读取sTagConfig.ChipID
				FLASH_ReadBytes(TAG_INFO_ADDR, NAME_ADDR_OFFECT, (uint8_t *)&sTagConfig.Name, CONFIG_NAME_LEN_MAX);										//读取sTagConfig.Name
				FLASH_ReadBytes(TAG_INFO_ADDR, DEPT_ADDR_OFFECT, (uint8_t *)&sTagConfig.Department, CONFIG_DEPT_LEN_MAX);							//读取sTagConfig.Department
				FLASH_ReadBytes(TAG_INFO_ADDR, TITLE_ADDR_OFFECT, (uint8_t *)&sTagConfig.Title, CONFIG_TITLE_LEN_MAX);								//读取sTagConfig.Title
				FLASH_ReadBytes(TAG_INFO_ADDR, WORK_SPACE_ADDR_OFFECT, (uint8_t *)&sTagConfig.WorkSpace, CONFIG_WORKSPACE_LEN_MAX);		//读取sTagConfig.WorkSpace
				
				UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_INFO_ALL;		//写入串口帧类别--回复全部信息
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.ChipID, DataWriteOffect, CONFIG_CHIPID_LEN, 1);		//写入ChipID
				DataWriteOffect += CONFIG_CHIPID_LEN;																																		//更新数据写入偏移量
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.MAC_ID, DataWriteOffect, CONFIG_MACID_LEN, 1);			//写MAC_ID
				DataWriteOffect += CONFIG_MACID_LEN;
				
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Name, DataWriteOffect);								//写入姓名，更新数据读取偏移量
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.Sex, DataWriteOffect, CONFIG_SEX_LEN, 1);					//写入性别
				DataWriteOffect += CONFIG_SEX_LEN;
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.Age, DataWriteOffect, CONFIG_AGE_LEN, 1);					//写入年龄
				DataWriteOffect += CONFIG_AGE_LEN;
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.JobNum, DataWriteOffect, CONFIG_JOBNUM_LEN, 1);		//写入工号
				DataWriteOffect += CONFIG_JOBNUM_LEN;
				
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Department, DataWriteOffect);					//写入部门，更新数据读取偏移量
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Title, DataWriteOffect);							//写入职务，更新数据读取偏移量
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.WorkSpace, DataWriteOffect);					//写入工作区，更新数据读取偏移量
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Perm, DataWriteOffect);								//写入权限，更新数据读取偏移量
				
				DataPayLoadLength = DataWriteOffect - USART_FRAME_DATA_IDX + 2;	//串口负载长度：数据写入偏移量 - 负载起始偏移量 + CRC长度
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&DataPayLoadLength, USART_FRAME_LEN_IDX, 2, 1);		//写入帧负载长度
				
				Crc = GetCRC(UsartFramePayLoad, 0, DataPayLoadLength + 2);
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, DataWriteOffect, 2, 1);		//写入CRC
			}
			else	//标签未配置，仅回复ChipID
			{
				UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_INFO_CHIPID;		//写入串口帧类别
				
				sTagConfig.ChipID = sDWT.ChipID;
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.ChipID, USART_FRAME_CHIPID_IDX, CONFIG_CHIPID_LEN, 1);		//写入ChipID
				
				DataPayLoadLength = CONFIG_CHIPID_LEN + 2;	//串口负载长度：ChipID长度 + CRC长度
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&DataPayLoadLength, USART_FRAME_LEN_IDX, 2, 1);		//写入帧负载长度
				
				Crc = GetCRC(UsartFramePayLoad, 0, DataPayLoadLength + 2);
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX + CONFIG_CHIPID_LEN, 2, 1);		//写入CRC
			}
			DWT_WriteTxData(DataPayLoadLength + 6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//写入发送数据
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 6, 0, 0);														//设定发送长度
			break;
		}
		case USART_FRAME_REPEAT:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_REPEAT;		//写入串口帧类别--重发帧
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//写入CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//写入发送数据
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//设定发送长度
		}
		case USART_FRAME_ERR:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_ERR;		//写入串口帧类别--CRC错误帧
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//写入CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//写入发送数据
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//设定发送长度
		}
		case USART_FRAME_DEINIT:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_DEINIT_ACK;		//写入串口帧类别--CRC错误帧
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//写入CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//写入发送数据
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//设定发送长度
		}
		default:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = frametype;		//写入串口帧类别--ACK回复帧
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//写入CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//写入发送数据
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//设定发送长度
		}
	}
	
	DWT_StartTx(DWT_START_TX_IMMEDIATE);
	
	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//不断查询芯片状态直到发送完成
	{ };
	
	DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//清除寄存器标志位
}


/*******************************************************************************************
* 函数名称：TAG_ReadAllConfigInfo(void)
* 功能描述：标签更新所有配置信息
* 入口参数：frametype--接收到的帧类型
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TAG_ReadAllConfigInfo(void)
{
	uint8_t DataReadOffect = USART_FRAME_MACID_IDX;	//初始化数据读取偏移量（Chip ID不可修改，故不进行读取）
	
	ReadFromMsg((uint8_t *)&sTagConfig.MAC_ID, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_MACID_LEN, 1);		//读取并保存MAC ID
	DataReadOffect += CONFIG_MACID_LEN;																																									//更新数据读取偏移量
	sDWT.ShortAddr = sTagConfig.MAC_ID;
	sDWT.ShortAddrFlag = 1;
	
	DataReadOffect += ReadStringFromMsg(sTagConfig.Name, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect);							//读取并保存姓名，更新数据读取偏移量
	
	ReadFromMsg((uint8_t *)&sTagConfig.Sex, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_SEX_LEN, 1);				//读取并保存性别
	DataReadOffect += CONFIG_SEX_LEN;
	
	ReadFromMsg((uint8_t *)&sTagConfig.Age, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_AGE_LEN, 1);				//读取并保存年龄
	DataReadOffect += CONFIG_AGE_LEN;
	
	ReadFromMsg((uint8_t *)&sTagConfig.JobNum, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_JOBNUM_LEN, 1);	//读取并保存工号
	DataReadOffect += CONFIG_JOBNUM_LEN;
	
	DataReadOffect += ReadStringFromMsg(sTagConfig.Department, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect);				//读取并保存部门，更新数据读取偏移量
	DataReadOffect += ReadStringFromMsg(sTagConfig.Title, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);				//读取并保存职务，更新数据读取偏移量
	DataReadOffect += ReadStringFromMsg(sTagConfig.WorkSpace, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);		//读取并保存工作区，更新数据读取偏移量
	
	ReadStringFromMsg(sTagConfig.Perm, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);													//读取并保存权限
}


/************************************标签数据存储顺序*************************************************************************
* 1. DWT_WriteToFlash -- sDWT.ShortAddr + sDWT.DWT_Mode + sDWT.ShortAddrFlag，64bit
* 2. sTagConfig.ChipID -- 64bit
* 3. TAG_WriteToFlash -- sTagConfig.Perm + sTagConfig.JobNum + sTagConfig.Age + sTagConfig.Sex + sTagConfig.MAC_ID，64bit
* 4. sTagConfig.Name -- 256bit
* 5. sTagConfig.Department -- 256bit
* 6. sTagConfig.Title -- 256bit
* 7. sTagConfig.WorkSpace -- 256bit
******************************************************************************************************************************/

/*******************************************************************************************
* 函数名称：TAG_ReadAllConfigInfo(void)
* 功能描述：标签更新所有配置信息
* 入口参数：frametype--接收到的帧类型
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TAG_SaveConfigInfoToFlash(void)
{
	DWT_WriteToFlash = (((uint64_t)sDWT.ShortAddr << UWB_16BIT_ADDR_IDX) | \
											((uint64_t)sDWT.ShortAddrFlag << UWB_16BIT_ADDR_FLAG_IDX) | \
											((uint64_t)sDWT.DWT_Mode << UWB_MODE_IDX));	
	
	TAG_WriteToFlash = (((uint64_t)sTagConfig.Perm[0] << UWB_PERM_IDX) | \
											((uint64_t)sTagConfig.JobNum << UWB_JOBNUM_IDX) | \
											((uint64_t)sTagConfig.Age << UWB_AGE_IDX) | \
											((uint64_t)sTagConfig.Sex << UWB_SEX_IDX) | \
											((uint64_t)sTagConfig.MAC_ID << UWB_MAC_ID_IDX));
	
	FLASH_WriteBytes(TAG_INFO_ADDR, SHORT_ID_ADDR_OFFECT, &DWT_WriteToFlash, 1);																								//保存sDWT数据
	FLASH_WriteBytes(TAG_INFO_ADDR, CHIPID_ADDR_OFFECT, &sTagConfig.ChipID, CONFIG_CHIPID_LEN >> 3);														//保存ChipID
	FLASH_WriteBytes(TAG_INFO_ADDR, MAC_ID_ADDR_OFFECT, &TAG_WriteToFlash, 1);																									//保存sTagConfig部分组合数据
	FLASH_WriteBytes(TAG_INFO_ADDR, NAME_ADDR_OFFECT, (uint64_t *)sTagConfig.Name, CONFIG_NAME_LEN_MAX >> 3);										//保存sTagConfig.Name
	FLASH_WriteBytes(TAG_INFO_ADDR, DEPT_ADDR_OFFECT, (uint64_t *)sTagConfig.Department, CONFIG_DEPT_LEN_MAX >> 3);							//保存sTagConfig.Department
	FLASH_WriteBytes(TAG_INFO_ADDR, TITLE_ADDR_OFFECT, (uint64_t *)sTagConfig.Title, CONFIG_TITLE_LEN_MAX >> 3);								//保存sTagConfig.Title
	FLASH_WriteBytes(TAG_INFO_ADDR, WORK_SPACE_ADDR_OFFECT, (uint64_t *)sTagConfig.WorkSpace, CONFIG_WORKSPACE_LEN_MAX >> 3);		//保存sTagConfig.WorkSpace
	
}
