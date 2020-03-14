/**
  ******************************************************************************
  * �ļ�����: main.c
  * ��    ��: ������
	* ��ǰ�汾��V1.0
	* ������ڣ�
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

uint8_t DECA_WorkMode = 0;		//UWBģʽ����   0����ǩ   1�����ⱨ����

#define DUMMY_BUFFER_LEN		600
uint8_t DummyBuffer[DUMMY_BUFFER_LEN];							//DWT��������

/*��������110K*/
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

/*��������850K*/
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

/*��������6.8M*/
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
	TC_PGDELAY_CH5,    /*��������*/
	0x1F1F1F1F         /*���͹��ʵ���*/
//	0x25456585
//	0xC0C0C0C0
};

/*֡��ʽ��
 * byte 0--9(����֡ͨ��):
 *     - byte 0/1: ֡������ (0x8841 ��ʾʹ��16λ��ַ).
 *     - byte 2  : ��Ϣ����, ÿ����һ֡��һ(������֤��Ϣ�������ԣ��Ƿ�֡).
 *     - byte 3/4: PAN TAG_ID (0xDECA).
 *     - byte 5/6: Ŀ���ַ.
 *     - byte 7/8: Դ��ַ.
 *     - byte 9  : ֡���.
 * byte 10--21(��֡������ͬ):
 *    Poll ֡:
 *     - no more data
 *    Response ֡:
 *     - byte 10   : RESP״̬λ (0x02 ��ʾ�������ݴ���).
 *     - byte 11/12: RESP����(byte 10Ϊ0x02ʱ�����ֽ�δʹ��).
 *    Final ֡:
 *     - byte 10 -> 13: poll ����ʱ���.
 *     - byte 14 -> 17: response ����ʱ���.
 *     - byte 18 -> 21: final ����ʱ���.
 * ��������֡������Ϻ󶼻���֡ĩ���2�ֽڵ�CRC���ݣ����ڽ���У��(��DW1000�Զ����).
*/

uint8_t Tag_AskForShortID[] = {0x41, 0xC8, 0, 0xCA, 0xDE, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00, FRAME_SHORT_ID_ASK, 0, 0,};									//�豸ͨѶID����
uint8_t Anc_ConfigShortID[] = {0x41, 0x8C, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0xFD, 0xFF, FRAME_SHORT_ID_CONFIG, 0x00, 0x00, 0, 0,};	//�����豸ͨѶID

uint8_t ACK_Msg[] = {0x02, 0x00, 0x00, 0, 0};

static uint32_t status_reg = 0;

uint16_t PanID = 0xDECA;			//PAN IDֵ
uint64_t Eui = 0;					//EUI

uint8_t TimeDlyForUS_380;			//��ʱʱ���־λ		6.8Mbpsʱʹ�ã�ȷ��1ms��ֻ����1֡���ݣ�APS013 Page9��
uint8_t TimeDlyForUS_920;

uint8_t TAG_ID;						//��ǩID
uint8_t ANCHOR_ID;					//��վID

OS_EVENT  *WorkModeSem;				//����ģʽ�ź���

void SystemClock_Config(void);
void TaskStart(void *pdata);

void UWB_ID_Config(void *pdata);

void Anc_TOF(uint8_t *pdata);
void ANC_CommDecResp(uint8_t *pdata);
void TAG_TOF(void);
	
void SendACK(uint8_t *pdata);

extern DWT_LocalData_s *pdw1000local;

/*���������ջ������256*/
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
* �������ƣ�FinalMsgGetTS()
* ����������
* ��ڲ�����
* ���ڲ�������
* ʹ��˵������
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
* �������ƣ�WriteToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB)
* ����������������֡��д������
* ��ڲ�����pMsg--Ŀ��֡��pdata--����Դ��MsgOffset--д��ʱ����ƫ������DataLength--���ݳ��ȣ�IsMSB--pdata��ʽ��1�����ֽ���ǰ��0�����ֽ���ǰ��
* ���ڲ�������
* ʹ��˵������
***********************************************************************************************************/
void WriteToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB)
{
	uint8_t i;
	if(IsMSB)	//pdata���ֽ���ǰ
	{
		uint8_t j = 0;
		for(i = DataLength; i > 0; i--)
		{
			pMsg[MsgOffset + i - 1] = pdata[j];
			j++;
		}
	}
	else			//pdata���ֽ���ǰ
	{
		for(i = 0; i < DataLength; i++)
		{
			pMsg[MsgOffset + i] = pdata[i];
		}
	}
}


/*******************************************************************************************************
* �������ƣ�ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength)
* ����������������֡�ж�ȡ����
* ��ڲ�����pdata--��ȡ�����ݴ洢������pMsg--����Դ��MsgOffset--��ȡʱ����ƫ������DataLength--���ݳ��ȣ�IsMSB--pdata��ʽ��1�����ֽ���ǰ��0�����ֽ���ǰ��
* ���ڲ�������
* ʹ��˵������
*******************************************************************************************************/
void ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB)
{
	uint8_t i;
	if(IsMSB)	//pdata���ֽ���ǰ
	{
		uint8_t j = 0;
		for(i = DataLength; i > 0; i--)
		{
			pdata[j] = pMsg[MsgOffset + i - 1];
			j++;
		}
	}
	else			//pdata���ֽ���ǰ
	{
		for(i = 0; i < DataLength; i++)
		{
			pdata[i] = pMsg[MsgOffset + i];
		}
	}
}


/*******************************************************************************************************
* �������ƣ�WriteStringToMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset)
* ����������д�ַ�������
* ��ڲ�����pMsg--Ŀ��֡��pdata--����Դ��MsgOffset--д��ʱ����ƫ����
* ���ڲ�������
* ʹ��˵������
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
* �������ƣ�ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength)
* ����������������֡�ж�ȡ����
* ��ڲ�����pdata--��ȡ�����ݴ洢������pMsg--����Դ��MsgOffset--��ȡʱ����ƫ����
* ���ڲ������ַ�������
* ʹ��˵������
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
	OSInit();           //��ʼ��OSϵͳ
//	WorkModeSem = OSSemCreate(1);		//����һ���ź���������DECA_WorkMode
	OSTaskCreate(TaskStart, (void *)0, &TaskStartStk[OS_TASK_TMR_STK_SIZE-1], 4);          //����TaskStart����,���ȼ�4
	OSStart();         //��ʼ����OSϵͳ
}

/*******************************************************************************************
* �������ƣ�UWB_ModeChoose(void *pdata)
* ����������UWBģ�����ѡ�񣺱�ǩ����վ��ң��
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
//void UWB_ModeChoose(void *pdata)
//{
//	char ModelID[5];
//	sprintf(ModelID, "%04d", sDWT.ShortAddr);
//	for(;;)
//	{
//		switch(DECA_WorkMode)      //�ж�UWB����ģʽ
//		{
//			case DECA_TAG:
//				Debug("Now Model is work as TAG\r\n");
//#if defined(USE_OLED)
//				OLED_ShowString(0, 0, "TagID:", 16);
//				OLED_ShowString(48, 0, ModelID, 16);
//				OLED_ShowString(0, 2, "AncID:", 16);
////				OLED_ShowString(0, 2, "Enter Workspace", 16);								//7.135ms
//#endif
//				OSTaskSuspend(UWB_ANCHOR_PRIO);       	//�����վ������
//				OSTaskResume(UWB_TAG_PRIO);           	//������ǩ������
//				break;
//			case DECA_ANCHOR:
//				Debug("Now Model is work as Anchor\r\n");
//#if defined(USE_OLED)
//				OLED_ShowString(0, 0, "AncID:    ", 16);
//				OLED_ShowString(48, 0, ModelID, 16);
////				OLED_ShowString(0, 2, "TagNum:", 16);
////				OLED_ShowString(56, 2, "00", 16);
//#endif
//				OSTaskSuspend(UWB_TAG_PRIO);          	//�����ǩ������
//				OSTaskResume(UWB_ANCHOR_PRIO);        	//������վ������
//				break;
//		}
//		OSTaskSuspend(OS_PRIO_SELF);                //ģʽ�ж���ϣ������Լ���ת����Ӧ��ģʽ����ִ��
//	}
//}

/*******************************************************************************************
* �������ƣ�TaskStart(void *pdata)
* ����������OSϵͳ���еĵ�һ�����񣬸��������ʼ������������Ĵ���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TaskStart(void *pdata)
{
	sConfig = sConfig_6M8;

	/* ��λ�������裬����Systick����1ms */
	HAL_Init();
	/* ����ϵͳʱ��*/
	SystemClock_Config();
	/* ��ʼ������ */
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
	OLED_Init();			//��ʼ��OLED  
	OLED_Clear();
#endif
	
	if(DECA_WorkMode == 0)
	{
		DWT_SpiCsWakeup(DummyBuffer, DUMMY_BUFFER_LEN);	//����DWT
	}
	
	Reset_DW1000();																		//�����λDW1000
	SPI_SetRateLow();																	//����SPIʱ��Ƶ��72/32MHz��Init״̬�£�SPICLK������3MHz��
	if (DWT_Initialise(DWT_LOADUCODE) == DWT_ERROR)		//��ʼ��DW1000
	{
#if defined(USE_OLED)
		OLED_ShowString(0, 2, "Failed...", 16);
#endif
		while (1)
		{
			Error_Handler();
		}
	}

  SPI_SetRateHigh();																//�ָ�SPIʱ��Ƶ��Ϊ72/4MHz��IDLE״̬�£�SPICLK������20MHz��
	DWT_Configure(&sConfig);													//����ͨ�Ų���
	
	if(DECA_WorkMode == 0)														//����DWM1000 Sleepģʽ
	{
		DWT_ConfigureSleep(DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_LOAD_LDO | DWT_LOAD_LDE | DWT_LOADOPSET | DWT_LOADEUI, DWT_WAKE_CS | DWT_WAKE_WK | DWT_SLP_EN);
	}

	sDWT.ChipID = (((uint64_t)pdw1000local -> lotID) << 32) | pdw1000local -> partID;
	sDWT.DWT_Mode = DECA_WorkMode;

	DWT_SetRxAntennaDelay(RX_ANT_DLY);								//���ý��������ӳ�
  DWT_SetTxAntennaDelay(TX_ANT_DLY);								//���÷��������ӳ�
	
	Eui = sDWT.ChipID;
	DWT_SetPanID(PanID);															//����֡����PANID
	DWT_SetEUI((uint8_t *)&Eui);											//����֡����64λ�豸��ַ

	/* �������� */
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
* �������ƣ�void SendACK(uint8_t *pdata)
* ����������ACK�ظ�
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void SendACK(uint8_t *pdata)
{
	DWT_WriteTxData(sizeof(ACK_Msg), ACK_Msg, 0);		//��Poll�����ݴ���DW1000�����ڿ�������ʱ����ȥ
	DWT_WriteTxfCtrl(sizeof(ACK_Msg), 0, 0);				//���ó�����������ݳ���

	DWT_StartTx(DWT_START_TX_IMMEDIATE);

	while (!((status_reg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߷�������
	{};
}


/*******************************************************************************************
* �������ƣ�void CaculcateTagDist(uint16_t tagid)
* ���������������ǩʵ�ʾ���
* ��ڲ�����uint16_t tagid
* ���ڲ�������
* ʹ��˵������
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
