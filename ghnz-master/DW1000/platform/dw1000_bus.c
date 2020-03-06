 /*
*********************************************************************************************************
*	                                           UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : dw1000_bus.c
*    Module  : dw1000 bus
*    Version : V1.0
*    History :
*   -----------------
*             dw1000����
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/
#include "dw1000_bus.h"
#include <string.h>


OS_EVENT			*   gDw1000Sem;


/*��������6.8M*/
static DWT_Config_s gDw1000Config = {
    5,               						/* Channel number. */
    DWT_PRF_64M,     						/* Pulse repetition frequency. */
    DWT_PLEN_64,   	 						/* Preamble length. Used in TX only. */
    DWT_PAC8,        						/* Preamble acquisition chunk size. Used in RX only. */
    9,               						/* TX preamble code. Used in TX only. */
    9,               						/* RX preamble code. Used in RX only. */
    1,               						/* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      						/* Data rate. */
    DWT_PHRMODE_EXT, 						/* PHY header mode. */
    (1025 + 64 - 8)  						/* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


/*DW1000��������****************/

DWT_TxConfig_s  gDwTxConfig =
{
	TC_PGDELAY_CH5,    /*��������*/
	0x1F1F1F1F         /*���͹��ʵ���*/
//	0x25456585
//	0xC0C0C0C0
};

/*******************************************************************************************
* �������ƣ�void Dw1000SemInit(void)
* ����������DW1000 ���ݳ�ʼ��
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/

void Dw1000SemInit(void)
{
	gDw1000Sem = OSSemCreate(0);
}
/*******************************************************************************************
* �������ƣ�void Dw1000InitConfig(sDW100ConfigPara * config)
* ����������DW1000 ��ʼ������
* ��ڲ��������ò���
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/

void Dw1000InitConfig(sDW100ConfigPara * config)
{
  SPI_SetRateHigh();																//�ָ�SPIʱ��Ƶ��Ϊ72/4MHz��IDLE״̬�£�SPICLK������20MHz��
	DWT_Configure(&gDw1000Config);													//����ͨ�Ų���


	DWT_SetRxAntennaDelay(config->rxAntDelay);				//���ý��������ӳ�
  DWT_SetTxAntennaDelay(config->txAntDelay);								//���÷��������ӳ�
	
	DWT_SetPanID(config->panId);											//����֡����PANID
	DWT_SetEUI((uint8_t *)&(config->eui));						//����֡����64λ�豸��ַ
}


/*******************************************************************************************
* �������ƣ�GetTxTimeStamp_u64
* ����������Ӧ�ò�Ҫ�õ���ͨ�ú�������
* ��ڲ��������ò���
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/


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


uint64_t GetSysTimeStamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int i;
	DWT_ReadSysTime(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
    return ts;
}


void DwtWriteTxData(uint8_t * data, uint16_t len)
{
	DWT_WriteTxData(len, data, 0);									//д�뷢������
	DWT_WriteTxfCtrl(len, 0, 1);									  //�趨���ͳ���
}

/*******************************************************************************************
* �������ƣ�HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* ����������GPIO�ⲿ�жϻص�����
* ��ڲ�����GPIO_Pin
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	port_DisableEXT_IRQ();
	OSSemPost(gDw1000Sem);
}

void Dw1000ProcPend(uint16_t timeout, INT8U *perr)
{
	OSSemPend(gDw1000Sem,  timeout, perr);
}

void DWT_SetSendInterrupt(uint8 enable)
{
	;
}

void Dw1000Init(void)
{
	 Dw1000SemInit();
	 Dw100GpioInit();
	 Reset_DW1000();
}
