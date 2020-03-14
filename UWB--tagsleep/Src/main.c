/**
  ******************************************************************************
  * 文件名称: main.c
  * 作    者: 张雨生
	* 当前版本：V1.0
	* 完成日期：
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

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"
#include "port.h"
#include "sleep.h"
#include "flash.h"
#include "address.h"
#include "BEB.h"
#include "anchor.h"
#include "adc.h"
#include "rtc.h"
#include "key.h"
#include "tag.h"
#if defined(USE_OLED)
	#include "oled.h"
#endif

DWT_AddrStatus_s 	sDWT = {.ShortAddr = 2};

uint8_t DECA_WorkMode = 0;		//UWB模式定义   0：标签   1：声光报警器

#define DUMMY_BUFFER_LEN		600
uint8_t DummyBuffer[DUMMY_BUFFER_LEN];							//DWT唤醒序列

/*数据速率110K*/
//static DWT_Config_s sConfig_110K = {
//    5,               /* Channel number. */
//    DWT_PRF_64M,     /* Pulse repetition frequency. */
//    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
//    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
//    9,               /* TX preamble code. Used in TX only. */
//    9,               /* RX preamble code. Used in RX only. */
//    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
//    DWT_BR_110K,     /* Data rate. */
//    DWT_PHRMODE_EXT, /* PHY header mode. */
//    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};

/*数据速率850K*/
//static DWT_Config_s sConfig_850K = {
//    5,               /* Channel number. */
//    DWT_PRF_64M,     /* Pulse repetition frequency. */
//    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
//    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
//    9,               /* TX preamble code. Used in TX only. */
//    9,               /* RX preamble code. Used in RX only. */
//    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
//    DWT_BR_850K,     /* Data rate. */
//    DWT_PHRMODE_EXT, /* PHY header mode. */
//    (1025 + 32 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};

/*数据速率6.8M*/
static DWT_Config_s sConfig_6M8 = {
    5,								/* Channel number. */
    DWT_PRF_64M,			/* Pulse repetition frequency. */
    DWT_PLEN_128,			/* Preamble length. Used in TX only. */
    DWT_PAC8,					/* Preamble acquisition chunk size. Used in RX only. */
    9,								/* TX preamble code. Used in TX only. */
    9,								/* RX preamble code. Used in RX only. */
    1,								/* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,				/* Data rate. */
    DWT_PHRMODE_EXT,	/* PHY header mode. */
    (1025 + 64 - 8)		/* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


static DWT_Config_s sConfig;
//static DWT_DeviceEntCnts_s sEntCnt;

DWT_TxConfig_s  sTxConfig =
{
	TC_PGDELAY_CH5,    /*发送脉宽*/
	0x1F1F1F1F         /*发送功率调整*/
//	0x25456585
//	0xC0C0C0C0
};

/*帧格式：
 * byte 0--9(所有帧通用):
 *     - byte 0/1: 帧控制域 (0x8841 表示使用16位地址).
 *     - byte 2  : 消息序列, 每发送一帧加一(用于验证消息的连续性，是否丢帧).
 *     - byte 3/4: PAN TAG_ID (0xDECA).
 *     - byte 5/6: 目标地址.
 *     - byte 7/8: 源地址.
 *     - byte 9  : 帧类别.
 * byte 10--21(各帧均不相同):
 *    Poll 帧:
 *     - no more data
 *    Response 帧:
 *     - byte 10   : RESP状态位 (0x02 表示继续数据传输).
 *     - byte 11/12: RESP数据(byte 10为0x02时，该字节未使用).
 *    Final 帧:
 *     - byte 10 -> 13: poll 发送时间戳.
 *     - byte 14 -> 17: response 接收时间戳.
 *     - byte 18 -> 21: final 发送时间戳.
 * 所有数据帧发送完毕后都会在帧末添加2字节的CRC数据，用于接收校验(由DW1000自动完成).
*/

uint8_t Tag_AskForShortID[] = {0x41, 0xC8, 0, 0xCA, 0xDE, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00, FRAME_SHORT_ID_ASK, 0, 0,};									//设备通讯ID申请
uint8_t Anc_ConfigShortID[] = {0x41, 0x8C, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0xFD, 0xFF, FRAME_SHORT_ID_CONFIG, 0x00, 0x00, 0, 0,};	//设置设备通讯ID

uint8_t ACK_Msg[] = {0x02, 0x00, 0x00, 0, 0};

static uint32_t status_reg = 0;

uint16_t PanID = 0xDECA;			//PAN ID值
uint64_t Eui = 0;					//EUI

uint8_t TimeDlyForUS_380;			//延时时间标志位		6.8Mbps时使用，确保1ms内只发送1帧数据（APS013 Page9）
uint8_t TimeDlyForUS_920;

uint8_t TAG_ID;						//标签ID
uint8_t ANCHOR_ID;					//基站ID

OS_EVENT  *WorkModeSem;				//工作模式信号量

void SystemClock_Config(void);
void TaskStart(void *pdata);

void UWB_ID_Config(void *pdata);

void Anc_TOF(uint8_t *pdata);
void ANC_CommDecResp(uint8_t *pdata);
void TAG_TOF(void);
	
void SendACK(uint8_t *pdata);

extern DWT_LocalData_s *pdw1000local;

/*创建任务堆栈，容量256*/
OS_STK    TaskStartStk[OS_TASK_TMR_STK_SIZE];
OS_STK    UWB_ModeChooseStk[OS_TASK_STAT_STK_SIZE];
OS_STK    UWB_TagStk[OS_TASK_STAT_STK_SIZE];
OS_STK    UWB_AnchorStk[OS_TASK_STAT_STK_SIZE];
OS_STK    LED_BlinkStk[OS_TASK_STAT_STK_SIZE];
OS_STK		UWB_ID_ConfigStk[OS_TASK_STAT_STK_SIZE];

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn GetTxTimeStamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t GetTxTimeStamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int i;
	DWT_ReadTxTimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*******************************************************************************************
* 函数名称：FinalMsgGetTS()
* 功能描述：
* 入口参数：
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void FinalMsgGetTS(const uint8 *ts_field, uint32 *ts)
{
	uint32_t i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn FinalMsgSetTS()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void FinalMsgSetTS(uint8 *ts_field, uint64_t ts)
{
	int i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		ts_field[i] = (uint8) ts;
		ts >>= 8;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn GetRxTimeStamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t GetRxTimeStamp_u64(void)
{
	uint8 ts_tab[5];
	uint64_t ts = 0;
	int i;
	DWT_ReadRxTimeStamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
    return ts;
}

/***********************************************************************************************************
* 函数名称：WriteToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB)
* 功能描述：向数据帧中写入数据
* 入口参数：pMsg--目标帧，pdata--数据源，MsgOffset--写入时数据偏移量，DataLength--数据长度，IsMSB--pdata格式（1：高字节在前，0：低字节在前）
* 出口参数：无
* 使用说明：无
***********************************************************************************************************/
void WriteToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB)
{
	uint8_t i;
	if(IsMSB)	//pdata高字节在前
	{
		uint8_t j = 0;
		for(i = DataLength; i > 0; i--)
		{
			pMsg[MsgOffset + i - 1] = pdata[j];
			j++;
		}
	}
	else			//pdata低字节在前
	{
		for(i = 0; i < DataLength; i++)
		{
			pMsg[MsgOffset + i] = pdata[i];
		}
	}
}


/*******************************************************************************************************
* 函数名称：ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength)
* 功能描述：从数据帧中读取数据
* 入口参数：pdata--读取后数据存储变量，pMsg--数据源，MsgOffset--读取时数据偏移量，DataLength--数据长度，IsMSB--pdata格式（1：高字节在前，0：低字节在前）
* 出口参数：无
* 使用说明：无
*******************************************************************************************************/
void ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB)
{
	uint8_t i;
	if(IsMSB)	//pdata高字节在前
	{
		uint8_t j = 0;
		for(i = DataLength; i > 0; i--)
		{
			pdata[j] = pMsg[MsgOffset + i - 1];
			j++;
		}
	}
	else			//pdata低字节在前
	{
		for(i = 0; i < DataLength; i++)
		{
			pdata[i] = pMsg[MsgOffset + i];
		}
	}
}


/*******************************************************************************************************
* 函数名称：WriteStringToMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset)
* 功能描述：写字符串数据
* 入口参数：pMsg--目标帧，pdata--数据源，MsgOffset--写入时数据偏移量
* 出口参数：无
* 使用说明：无
*******************************************************************************************************/
uint8_t WriteStringToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset)
{
	uint8_t StringLen = 0;
	
	while(pdata[StringLen] != '\0')
	{
		pMsg[MsgOffset + StringLen] = pdata[StringLen];
		StringLen++;
	}
	pMsg[MsgOffset + StringLen] = pdata[StringLen];
	
	return StringLen + 1;
}


/*******************************************************************************************************
* 函数名称：ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength)
* 功能描述：从数据帧中读取数据
* 入口参数：pdata--读取后数据存储变量，pMsg--数据源，MsgOffset--读取时数据偏移量
* 出口参数：字符串长度
* 使用说明：无
*******************************************************************************************************/
uint8_t ReadStringFromMsg(uint8_t *pData, uint8_t *pMsg, uint8_t MsgOffset)
{
	uint8_t StringLen = 0;
	
	while(pMsg[MsgOffset + StringLen] != '\0')
	{
		pData[StringLen] = pMsg[MsgOffset + StringLen];
		StringLen++;
	}
	pData[StringLen] = pMsg[MsgOffset + StringLen];
	
	return StringLen + 1;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	OSInit();           //初始化OS系统
//	WorkModeSem = OSSemCreate(1);		//建立一个信号量，保护DECA_WorkMode
	OSTaskCreate(TaskStart, (void *)0, &TaskStartStk[OS_TASK_TMR_STK_SIZE-1], 4);          //创建TaskStart任务,优先级4
	OSStart();         //开始运行OS系统
}

/*******************************************************************************************
* 函数名称：UWB_ModeChoose(void *pdata)
* 功能描述：UWB模块身份选择：标签、基站、遥控
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
//void UWB_ModeChoose(void *pdata)
//{
//	char ModelID[5];
//	sprintf(ModelID, "%04d", sDWT.ShortAddr);
//	for(;;)
//	{
//		switch(DECA_WorkMode)      //判断UWB工作模式
//		{
//			case DECA_TAG:
//				Debug("Now Model is work as TAG\r\n");
//#if defined(USE_OLED)
//				OLED_ShowString(0, 0, "TagID:", 16);
//				OLED_ShowString(48, 0, ModelID, 16);
//				OLED_ShowString(0, 2, "AncID:", 16);
////				OLED_ShowString(0, 2, "Enter Workspace", 16);								//7.135ms
//#endif
//				OSTaskSuspend(UWB_ANCHOR_PRIO);       	//挂起基站子任务
//				OSTaskResume(UWB_TAG_PRIO);           	//就绪标签子任务
//				break;
//			case DECA_ANCHOR:
//				Debug("Now Model is work as Anchor\r\n");
//#if defined(USE_OLED)
//				OLED_ShowString(0, 0, "AncID:    ", 16);
//				OLED_ShowString(48, 0, ModelID, 16);
////				OLED_ShowString(0, 2, "TagNum:", 16);
////				OLED_ShowString(56, 2, "00", 16);
//#endif
//				OSTaskSuspend(UWB_TAG_PRIO);          	//挂起标签子任务
//				OSTaskResume(UWB_ANCHOR_PRIO);        	//就绪基站子任务
//				break;
//		}
//		OSTaskSuspend(OS_PRIO_SELF);                //模式判断完毕，挂起自己，转入相应的模式继续执行
//	}
//}

/*******************************************************************************************
* 函数名称：TaskStart(void *pdata)
* 功能描述：OS系统运行的第一个任务，负责外设初始化及其它任务的创建
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TaskStart(void *pdata)
{
	sConfig = sConfig_6M8;

	/* 复位所有外设，配置Systick周期1ms */
	HAL_Init();
	/* 配置系统时钟*/
	SystemClock_Config();
	/* 初始化外设 */
	MX_GPIO_Init();
//  MX_USART1_UART_Init();
	MX_SPI1_Init();
#if defined(STM32L412xx) && defined(PCB_V2)
	MX_ADC1_Init();
	KeyInit();
#endif
	RNG_Init();
	RTC_Config();
	SleepInit();
	
#if defined(USE_OLED)
	OLED_Init();			//初始化OLED  
	OLED_Clear();
#endif
	
	if(DECA_WorkMode == 0)
	{
		DWT_SpiCsWakeup(DummyBuffer, DUMMY_BUFFER_LEN);	//唤醒DWT
	}
	
	Reset_DW1000();																		//软件复位DW1000
	SPI_SetRateLow();																	//降低SPI时钟频率72/32MHz（Init状态下，SPICLK不超过3MHz）
	if (DWT_Initialise(DWT_LOADUCODE) == DWT_ERROR)		//初始化DW1000
	{
#if defined(USE_OLED)
		OLED_ShowString(0, 2, "Failed...", 16);
#endif
		while (1)
		{
			Error_Handler();
		}
	}

  SPI_SetRateHigh();																//恢复SPI时钟频率为72/4MHz（IDLE状态下，SPICLK不超过20MHz）
	DWT_Configure(&sConfig);													//配置通信参数
	
	if(DECA_WorkMode == 0)														//配置DWM1000 Sleep模式
	{
		DWT_ConfigureSleep(DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_LOAD_LDO | DWT_LOAD_LDE | DWT_LOADOPSET | DWT_LOADEUI, DWT_WAKE_CS | DWT_WAKE_WK | DWT_SLP_EN);
	}

	sDWT.ChipID = (((uint64_t)pdw1000local -> lotID) << 32) | pdw1000local -> partID;
	sDWT.DWT_Mode = DECA_WorkMode;

	DWT_SetRxAntennaDelay(RX_ANT_DLY);								//设置接收天线延迟
  DWT_SetTxAntennaDelay(TX_ANT_DLY);								//设置发射天线延迟
	
	Eui = sDWT.ChipID;
	DWT_SetPanID(PanID);															//设置帧过滤PANID
	DWT_SetEUI((uint8_t *)&Eui);											//设置帧过滤64位设备地址

	/* 创建任务 */
//	OSTaskCreate(UWB_ModeChoose,(void *)0,&UWB_ModeChooseStk[OS_TASK_STAT_STK_SIZE-1],UWB_MODECHOOSE_PRIO);
	OSTaskCreate(LED_Blink,(void *)0,&LED_BlinkStk[OS_TASK_STAT_STK_SIZE-1],LED_BLINK_PRIO);
	if(DECA_WorkMode == DECA_TAG)
	{
		OSTaskCreateExt(UWB_Tag,
									(void *)0,
									(OS_STK *)&UWB_TagStk[OS_TASK_STAT_STK_SIZE - 1],
									UWB_TAG_PRIO,
									UWB_TAG_PRIO,
									(OS_STK *)&UWB_TagStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

#ifdef PCB_V2									
		OSTaskCreateExt(UWB_ID_Config,
									(void *)0,
									(OS_STK *)&UWB_ID_ConfigStk[OS_TASK_STAT_STK_SIZE - 1],
									UWB_ID_CONFIG_PRIO,
									UWB_ID_CONFIG_PRIO,
									(OS_STK *)&UWB_ID_ConfigStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
#endif
	}
	else if(DECA_WorkMode == DECA_ANCHOR)
	{
		OSTaskCreateExt(UWB_Anchor,
									(void *)0,
									(OS_STK *)&UWB_AnchorStk[OS_TASK_STAT_STK_SIZE - 1],
									UWB_ANCHOR_PRIO,
									UWB_ANCHOR_PRIO,
									(OS_STK *)&UWB_AnchorStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	}								

									
//	OSTaskSuspend(LED_BLINK_PRIO);
//	OSTaskSuspend(UWB_ID_CONFIG_PRIO);
//	OSTaskSuspend(UWB_ID_Config);

	OSTaskDel(OS_PRIO_SELF);
}


/*******************************************************************************************
* 函数名称：void SendACK(uint8_t *pdata)
* 功能描述：ACK回复
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void SendACK(uint8_t *pdata)
{
	DWT_WriteTxData(sizeof(ACK_Msg), ACK_Msg, 0);		//将Poll包数据传给DW1000，将在开启发送时传出去
	DWT_WriteTxfCtrl(sizeof(ACK_Msg), 0, 0);				//设置超宽带发送数据长度

	DWT_StartTx(DWT_START_TX_IMMEDIATE);

	while (!((status_reg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//不断查询芯片状态直到接收成功或者发生错误
	{};
}


/*******************************************************************************************
* 函数名称：void CaculcateTagDist(uint16_t tagid)
* 功能描述：计算标签实际距离
* 入口参数：uint16_t tagid
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void CaculcateTagDist(uint16_t tagid)
{
	
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	
	HAL_RCC_DeInit();

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
#if defined(STM32L412xx)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	
#elif defined(STM32L452xx)
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_RNG | RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED4_PIN | LED3_PIN);
	HAL_GPIO_TogglePin(LED1_PIN_Port, LED1_PIN);
	OSTimeDly(100);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
