#include "address.h"

#define NO_FREE_ADDR				0xFFFE

uint8_t  const  AddrUnMapTbl[256] = {
    0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x00 to 0x0F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x10 to 0x1F                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x20 to 0x2F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x30 to 0x3F                             */
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x40 to 0x4F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x50 to 0x5F                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x60 to 0x6F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x70 to 0x7F                             */
    7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x80 to 0x8F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x90 to 0x9F                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xA0 to 0xAF                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xB0 to 0xBF                             */
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xC0 to 0xCF                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xD0 to 0xDF                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xE0 to 0xEF                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0        /* 0xF0 to 0xFF                             */
};

uint32_t AddrMapTbl[ADDR_16BIT_MAX_NUM / 8 + 1] = {
		0x00000001, 0x00000002, 0x00000004, 0x00000008, 			/* AddrFreeTbl: 0 ～ 3	*/
		0x00000010, 0x00000020, 0x00000040, 0x00000080, 			/* AddrFreeTbl: 4 ～ 7	*/
		0x00000100, 0x00000200, 0x00000400, 0x00000800, 			/* AddrFreeTbl: 8 ～11	*/
		0x00001000, 0x00002000, 0x00004000, 0x00008000, 			/* AddrFreeTbl: 12～15	*/
		0x00010000, 0x00020000, 0x00040000, 0x00080000, 			/* AddrFreeTbl: 16～19	*/
		0x00100000, 0x00200000, 0x00400000, 0x00800000, 			/* AddrFreeTbl: 20～23	*/
		0x01000000, 0x02000000, 0x04000000, 0x08000000, 			/* AddrFreeTbl: 24～27	*/
		0x10000000, 0x20000000, 0x40000000, 0x80000000, 			/* AddrFreeTbl: 28～31	*/
};

uint8_t AddrFreeTbl[ADDR_16BIT_MAX_NUM / 8 + 1];					//空闲地址列表
//uint8_t AddrRdyTbl[ADDR_16BIT_MAX_NUM / 8 + 1];

uint8_t UWB_ModelNum = 4;
uint8_t UWB_TagNum;
uint8_t UWB_AncNum;
uint8_t UWB_ModelNow;
uint8_t UWB_FirstSyncFlag = 0;

uint16_t AddrForAnc_M;

uint32_t AddrFreeGrp = 0xFFFFFFFF;

extern uint64_t AddrWriteToFlash;

static uint32 status_reg;
static uint16 FrameLen;
static uint8 rx_buffer[RX_BUF_LEN+4];

static uint64_t FrameRxTime, FrameTxTime;
	
extern DWT_AddrStatus_s sDWT;
DWT_AddrSta_s sDWT_AddrStatus[ADDR_16BIT_MAX_NUM];

extern uint8_t AllModelHaveAddr;


/*基站*/
extern uint8 RxBeaconMsg[];
extern uint8 TxSyncMsg[];
extern uint8 RxSyncMsg[];

/*标签*/
extern uint8 TxBeaconMsg[];

extern uint8 AddrEnsureMsg[17];
extern uint8 AddrEnsureMsgResp[21];

extern uint8_t Tag_AskForShortID[14];
extern uint8_t Anc_ConfigShortID[16];

extern uint8_t FirstSync[12];
extern uint8_t FirstSyncTest[13];

/**********************************************/
uint64_t SysTime = 0;
//uint32_t FrameSyncTxTime = 0;
//uint64_t SysTimeNow = 0;

/**********************************************/

/*******************************************************************************************
* 函数名称：void Tag_AskFor16BitAddr(void)
* 功能描述：16Bit地址请求函数（标签和基站侧）
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void Tag_AskFor16BitAddr(void)
{
	DWT_SetRxAfterTxDelay(SHORT_ID_CONFIG_RX_DLY_UUS);					//设置发送后开启接收，并设定延迟时间，单位：uus
	DWT_SetRxTimeOut(SHORT_ID_CONFIG_RX_TIMEOUT_UUS);						//设置接收超时时间，单位：uus
	
	WriteToMsg(Tag_AskForShortID, (uint8_t *)&sDWT.ChipID, FRAME_SHORT_ID_ASK_SOURE_ADDR_IDX, 4); 		//写入32位CHIP ID 
	WriteToMsg(Anc_ConfigShortID, (uint8_t *)&sDWT.ChipID, FRAME_SHORT_ID_CONFIG_DEST_ADDR_IDX, 4);  	//写入短地址回复帧目标地址
	
	DWT_WriteTxData(sizeof(Tag_AskForShortID), Tag_AskForShortID, 0);	//将Poll包数据传给DW1000，将在开启发送时传出去
	DWT_WriteTxfCtrl(sizeof(Tag_AskForShortID), 0, 0);								//设置超宽带发送数据长度

	DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);			//开启发送，发送完成后等待一段时间开启接收，等待时间在DWT_SetRxAfterTxDelay中设置
	
	while (!((status_reg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//不断查询芯片状态直到接收成功或者发生错误
	{ };
	
	HAL_GPIO_TogglePin(LED1_PIN_Port, LED1_PIN);	//LED红灯闪烁
	
	if (status_reg & SYS_STATUS_RXFCG)//接收成功
	{
//		DWT_SetAutoRxReEnable(DISABLE);			//关闭接收器自动重开
//		DWT_ForceTRxOff();
//		DWT_EnableFrameFilter(DISABLE);
		
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//清楚标志位
		
		FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//数据长度
		
		if (FrameLen <= RX_BUF_LEN)
		{
			DWT_ReadRxData(rx_buffer, FrameLen, 0);																//读取接收数据
		}
		
		if (memcmp(rx_buffer, Anc_ConfigShortID, SHORT_ADDR_ASK_COMMON_LEN) == 0)  //判断是否为短地址回复帧
		{
			sDWT.ShortAddr = (((uint16_t)rx_buffer[FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX + 1]) << 8) | rx_buffer[FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX];  //读取短地址分配值
			sDWT.ShortAddrFlag = 1;
			
			AddrWriteToFlash = ((uint64_t)sDWT.ChipID << FLASH_ChipID_IDX) | \
												 ((uint64_t)sDWT.ShortAddr << FLASH_16BIT_ID_IDX) | \
												 ((uint64_t)sDWT.ShortAddrFlag << FLASH_16BIT_ID_FLAG_IDX) | \
												 (uint64_t)sDWT.DWT_Mode << FLASH_MODE_IDX;															//组合需保存数据
			
			FLASH_WriteBytes((uint32_t)0, (uint8_t *)&AddrWriteToFlash, 1);													//保存至FLASH
			
			HAL_GPIO_WritePin(LED1_PIN_Port, LED1_PIN, GPIO_PIN_SET);																//LED1灭

			OSTaskResume(UWB_MODECHOOSE_PRIO);
			OSTaskSuspend(OS_PRIO_SELF);
		}
	}//接收成功
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
		DWT_RxReset();
		OSTimeDly(ASK_ADDR_DELAY_MS);//休眠固定时间
	}
}


/*******************************************************************************************
* 函数名称：void CLE_AskFor16BitAddr(uint8_t *pdata)
* 功能描述：16Bit地址请求处理函数（CLE）
* 入口参数：*pdata（接收到的数据帧）
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void CLE_AskFor16BitAddr(uint8_t *pdata)
{
	int8_t StratTxStatu = -1;		//开启发送器是否成功：-1：未开启，0：已开启
	uint32_t ChipID;
	uint16_t FreeAddr16Bit = 0;
	int8_t i;
	
//	DWT_ForceTRxOff();
	
	for(i = 3; i >= 0; i--)
	{
		ChipID = (ChipID << 8) | pdata[FRAME_SHORT_ID_ASK_SOURE_ADDR_IDX + i];   //读取32Bit地址
	}
	
	FreeAddr16Bit = GetFreeAddr();  //从空闲地址中取出一个地址（取出顺序从小到大）
	
//	sDWT_AddrStatus[FreeAddr16Bit].DWT_Mode = pdata[ADDRESS_ASK_MODE_ID_IDX]; //保存相关信息
	sDWT_AddrStatus[FreeAddr16Bit].ChipID = ChipID;
	
	WriteToMsg(Anc_ConfigShortID, (uint8_t *)&ChipID, FRAME_SHORT_ID_CONFIG_DEST_ADDR_IDX, 4); 				//目标地址--即请求者地址（32Bit）
	WriteToMsg(Anc_ConfigShortID, (uint8_t *)&FreeAddr16Bit, FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX, 2);	//分配给其的16Bit地址值
	
	FrameRxTime = GetRxTimeStamp_u64();		//获取请求帧接收时间
	FrameTxTime = (FrameRxTime + (SHORT_ID_CONFIG_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//计算回复帧发送时间。
	DWT_SetDelayedTRxTime(FrameTxTime);//设置回复帧发送时间
	
	DWT_WriteTxData(sizeof(Anc_ConfigShortID), Anc_ConfigShortID, 0);//写入发送数据
	DWT_WriteTxfCtrl(sizeof(Anc_ConfigShortID), 0, 1);//设定发送长度
	
	while(StratTxStatu)
	{
		StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED);		//延迟发送
	}
	
	while (!((status_reg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))//不断查询芯片状态直到发送完成
	{ }
	
	UWB_ModelNow++;
}


/*******************************************************************************************
* 函数名称：uint16_t GetFreeAddr(void)
* 功能描述：获取空闲地址
* 入口参数：无
* 出口参数：AddrFree--16Bit地址
* 使用说明：无
********************************************************************************************/
uint16_t GetFreeAddr(void)
{
	uint32_t AddrFreeGrpNow;
	uint16_t AddrFree;
	uint8_t AddrUnMapTblFoldNum = 0;
	uint8_t y;
	
	AddrFreeGrpNow = AddrFreeGrp;
	
	if(AddrFreeGrpNow == 0x0000)
	{
		return NO_FREE_ADDR;       //当前无空闲地址
	}
	
	while((uint8_t)AddrFreeGrpNow == 0)    //计算AdrUnMapTbl折叠次数
	{
		AddrUnMapTblFoldNum++;
		AddrFreeGrpNow = AddrFreeGrpNow >> 8;
	}
	
	y = AddrUnMapTbl[(uint8_t)AddrFreeGrpNow] + (8 * AddrUnMapTblFoldNum);  //获取当前空闲地址中最小值
	AddrFree = (INT8U)((y << 3) + AddrUnMapTbl[AddrFreeTbl[y]]);
	
	if((AddrFreeTbl[AddrFree >> 3] &= ~AddrMapTbl[(uint8_t)AddrFree & 0x07]) == 0) //将被获取地址从空闲地址列表中清除
	{
		AddrFreeGrp &= ~AddrMapTbl[AddrFree >> 3];
	}
	
	return AddrFree;
}

/*******************************************************************************************
* 函数名称：void Anc_CLE_Release16BitAddr(uint16_t ShortAddr)
* 功能描述：16Bit地址释放处理（CLE）
* 入口参数：ShortAddr（被释放的16Bit地址）
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void Anc_CLE_Release16BitAddr(uint16_t ShortAddr)
{
	AddrFreeGrp |= AddrMapTbl[ShortAddr >> 3];
	AddrFreeTbl[ShortAddr >> 3] |= AddrMapTbl[ShortAddr & 0x07];		//将被释放的16Bit地址添加到空闲地址列表中
	
	sDWT_AddrStatus[ShortAddr].DWT_Mode = 0;  //清除16Bit地址与32位地址关联信息
	sDWT_AddrStatus[ShortAddr].ChipID = 0;
}


