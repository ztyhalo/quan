/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "stm32l4xx.h"
#include <ucos_ii.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "filtering.h"
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

#define USE_OLED							//使用OLED显示屏
#define PCB_V1								//印制板第一版
//#define PCB_V2								//印制板第二版

/*引脚名称定义*/
#if defined(STM32L412xx)
	#define D_RST_Pin                   GPIO_PIN_3
	#define D_RST_GPIO_Port             GPIOA
	#define D_CS_Pin                    GPIO_PIN_4
	#define D_CS_GPIO_Port              GPIOA
	#define D_IRQ_Pin                   GPIO_PIN_0
	#define D_IRQ_GPIO_Port             GPIOB
	#define D_IRQ_EXTI_IRQn             EXTI0_IRQn
	#define D_WAKEUP_Pin                GPIO_PIN_1
	#define D_WAKEUP_GPIO_Port					GPIOB
	
#if defined(PCB_V1)
	#define LED1_PIN										GPIO_PIN_14
	#define LED1_PIN_Port								GPIOC
	#define LED3_PIN                		GPIO_PIN_2
	#define LED4_PIN                 		GPIO_PIN_1
	#define LED3_LED4_PIN_Port					GPIOA
	
#elif defined(PCB_V2)
	#define LTC3401_MODE_Pin						GPIO_PIN_11
	#define LTC3401_MODE_Port						GPIOA
	#define LED1_PIN										GPIO_PIN_2
	#define LED1_PIN_Port								GPIOA
	#define LED3_PIN										GPIO_PIN_8
	#define LED4_PIN                 		GPIO_PIN_2
	#define LED3_LED4_PIN_Port					GPIOA
	#define LED5_PIN										LED4_PIN
	#define LED6_PIN										LED3_PIN
	#define LED_PIN_Port								GPIOA
	
	#define CHARGE_WAKEUP_Pin						GPIO_PIN_7
	#define CHARGE_WAKEUP_GPIO_Port			GPIOB
	#define CHARGE_WAKEUP_EXTI_IRQ			EXTI9_5_IRQn
	
	#define SW_PIN											GPIO_PIN_3
	#define SW_PIN_Port									GPIOB
	#define	SW_EXTI_IRQ									EXTI3_IRQn
	
	#define SW_IRQHandler								EXTI3_IRQHandler
	#define CHARGE_WAKEUP_IRQHandler		EXTI9_5_IRQHandler
	
	#define LTC3401_BURST_MODE_ENABLE		HAL_GPIO_WritePin(LTC3401_MODE_Port, LTC3401_MODE_Pin, GPIO_PIN_SET)
	#define LTC3401_BURST_MODE_DISABLE	HAL_GPIO_WritePin(LTC3401_MODE_Port, LTC3401_MODE_Pin, GPIO_PIN_RESET)
#endif

//	#define RS485_TX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_SET)
//	#define RS485_RX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_RESET)
	
	#define DWM1000_IRQHandler					EXTI0_IRQHandler
	
	
	
#elif defined(STM32L452xx)

	#define D_RST_Pin                   GPIO_PIN_12
	#define D_RST_GPIO_Port             GPIOB
	#define D_CS_Pin                    GPIO_PIN_4
	#define D_CS_GPIO_Port              GPIOA
	#define D_IRQ_Pin                   GPIO_PIN_15
	#define D_IRQ_GPIO_Port             GPIOB
	#define D_IRQ_EXTI_IRQn             EXTI15_10_IRQn
	#define D_WAKEUP_Pin                GPIO_PIN_13
	#define D_WAKEUP_GPIO_Port					GPIOB
	#define D_RS485_TX_Ctrl_Pin					GPIO_PIN_8
	#define D_RS485_TX_Ctrl_GPIO_Port		GPIOA
	#define LED1_PIN										GPIO_PIN_14
	#define LED1_PIN_Port								GPIOC
	#define LED3_PIN                		GPIO_PIN_2
	#define LED4_PIN                 		GPIO_PIN_1
	#define LED3_LED4_PIN_Port					GPIOA

	#define RS485_TX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_SET)
	#define RS485_RX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_RESET)
	
	#define DWM1000_IRQHandler					EXTI15_10_IRQHandler

#endif


/*模块工作模式定义*/
#define DECA_TAG							0 //标签
#define DECA_ANCHOR						1 //主基站
#define DECA_CLE							3 //中央位置处理单元
#define DECA_REMOTECONTROL		4 //遥控

/*任务优先级定义*/
#define KEY_CHECK_PRIO		      	5		
#define UWB_TAG_PRIO             	6
#define UWB_ANCHOR_PRIO          	7
#define UWB_ID_CONFIG_PRIO				11
#define LED_BLINK_PRIO           	27

/*帧类型定义*/
#define FRAME_SHORT_ID_ASK	 								1 	//16位ID申请
#define FRAME_SHORT_ID_CONFIG	 							2		//16位地址设置
#define FRAME_FOR_16BIT_ADDR_ENSURE					3		//16位地址确认
#define FRAME_FOR_16BIT_ADDR_ENSURE_RESP		4		//16位地址确认回复
#define FRAME_FOR_FIRST_CLOCK_SYNC					5		//首次时钟同步开始
#define FRAME_FOR_CLOCK_SYNC								6		//时钟同步
#define FRAME_FOR_TAG_BEACON								7		//标签定位广播帧
#define FRAME_FOR_TS												8		//定位信息帧
#define FRAME_SUPER													9		//超帧
#define FRAME_ASK_FOR_GTS										10	//GTS请求帧
#define	FRAME_POLL													11	//Poll帧
#define FRAME_RESP													12	//Resp帧
#define FRAME_FINAL													13	//Final帧
#define FRAME_DIST_INFO											14	//距离信息帧
#define FRAME_COMM_DEC											15	//Tag通讯声明帧
#define FRAME_COMM_DEC_RESP									16	//Anc通讯声明回复帧
#define FRAME_CONFIG_ID											32	//标签ID信息配置帧

/*天线延迟时间*/
#define TX_ANT_DLY                 16440      //接收天线延迟
#define RX_ANT_DLY                 16440      //发送天线延迟

/*MAC 帧字段定义*/
#define FRAME_CTRL_BYTE_FRAME_TYPE_BIT					0x0007    //MAC 帧控制段中帧类型位
#define FRAME_CTRL_BYTE_ACK_BIT									0x0020    //MAC 帧控制段中确认请求位
#define FRAME_CTRL_BYTE_PANID_BIT								0x0040    //MAC 帧控制段中PAN ID位
#define FRAME_CTRL_BYTE_ADDR_LEN_BIT						0xCC00    //MAC 帧控制段中地址长度位

#define FRAME_CTRL_BYTE_SOURCE_ID_64BIT					0xC800
#define FRAME_CTRL_BYTE_DEST_ID_64BIT						0x8C00

/*IEEE 802.15.4	帧类型*/
#define FRAME_BEACON										0x0000		//信标帧
#define FRAME_DATA											0x0001		//数据帧
#define FRAME_ACK												0x0002		//确认帧
#define FRAME_CMD												0x0003		//命令帧
#define FRAME_RESERVED_4								0x0004		//保留帧4
#define FRAME_RESERVED_5								0x0005		//保留帧5
#define FRAME_RESERVED_6								0x0006		//保留帧6
#define FRAME_RESERVED_7								0x0007		//保留帧7


#define ADDR_FOR_BEACON									0xFFFF		//信标帧广播地址
	

/*长度*/
#define ALL_MSG_COMMON_LEN         							10				//通讯帧通用部分长度

#define RX_BUF_LEN            									55
#define RX_BUF_LEN_MAX													1024			//接收缓存区最大长度

#define SHORT_ADDR_ASK_COMMON_LEN								12				//16位ID申请帧通用部分长度


/*数据位索引*/
/*通用*/
#define FRAME_DEST_ADDR_IDX												5		//目标地址设置索引
#define FRAME_SOURCE_ADDR_IDX			   							7		//源地址设置索引

#define ALL_MSG_SN_IDX                        		2   //帧序列值索引

/*特殊*/
#define FRAME_DEC_TAG_ADDR_IDX										5		//通讯声明帧，标签地址索引
#define FRMAE_DEC_RESP_TAG_ADDR_IDX								5		//通讯声明回复帧，标签地址索引
#define FRMAE_DEC_RESP_ANC_ADDR_IDX								7		//通讯声明回复帧，基站地址索引

#define FRAME_SHORT_ID_ASK_DEST_ADDR_IDX					5		//设备ID申请帧，目标地址索引
#define FRAME_SHORT_ID_ASK_SOURE_ADDR_IDX					7		//设备ID申请帧，源地址索引

#define FRAME_SHORT_ID_CONFIG_DEST_ADDR_IDX				5		//设备ID设置帧，目标地址索引
#define FRAME_SHORT_ID_CONFIG_SOURE_ADDR_IDX			9		//设备ID设置帧，源地址索引
#define FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX				12	//设备ID设置帧，16位ID分配值索引

#define FRAME_16BIT_ADDR_TYPE_IDX									9		//16Bit地址数据帧，帧类型索引
#define FRAME_32BIT_ADDR_TYPE_IDX									11	//32Bit地址数据帧，帧类型索引
#define FRAME_64BIT_ADDR_TYPE_IDX									15	//64Bit地址数据帧，帧类型索引

#define BEACON_FRAME_TYPE_IDX											7		//信标帧，帧子类型索引


/*延时时间及超时时间*/
#define FRAME_COMM_DEC_TX_TO_RESP_RX_DLY_UUS			400			//通讯声明帧发送完成到接收回复帧延迟时间
#define FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS			500			//通讯声明帧接收完成到发送回复帧延迟时间
#define FRAME_COMM_DEC_RESP_RX_TIMEOUT_UUS				1000		//通讯声明帧回复接收超时时间


/*FLASH数据位索引*/
//#define FLASH_ChipID_IDX														0		//FLASH数据，32位CHIP ID索引
//#define FLASH_16BIT_ID_IDX													32	//FLASH数据，16位地址索引
//#define FLASH_MODE_IDX															56  //FLASH数据，模块身份索引
//#define FLASH_16BIT_ID_FLAG_IDX											63	//FLASH数据，16位地址标识位索引

/* UWB microsecond (uus) 和 device time unit (dtu, 1/(499.2MHz*128)≈15.65ps) 换算系数.
 * 1 uus = 512 / 499.2 us 
 * 1 us  = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME											65536		//UUS与DWT芯片时间转换系数

/*标签部分*/
//#define POLL_TX_TO_RESP_RX_DLY_UUS          2000	//POLL发送完成到开始接收RESP延迟时间
//#define RESP_RX_TIMEOUT_UUS                 2700	//RESP接收超时时间
//#define RESP_RX_TO_FINAL_TX_DLY_UUS         3100	//RESP接收完成到考试发送FINAL延迟时间

#define POLL_TX_TO_RESP_RX_DLY_UUS          150		//POLL发送完成到开始接收RESP延迟时间
#define RESP_RX_TIMEOUT_UUS                 2700	//RESP接收超时时间
#define RESP_RX_TO_FINAL_TX_DLY_UUS         400		//RESP接收完成到考试发送FINAL延迟时间   最小可设为260

/*基站部分*/
//#define POLL_RX_TIMEOUT_UUS									1000	//POLL接收超时时间
//#define POLL_RX_TO_RESP_TX_DLY_UUS          2600  //POLL接收完成到开始发送RESP延迟时间
//#define RESP_TX_TO_FINAL_RX_DLY_UUS         2500  //RESP发送完成到开始接收FINAL延迟时间
//#define FINAL_RX_TIMEOUT_UUS                4300  //FINAL接收超时时间

#define POLL_RX_TIMEOUT_UUS									1000	//POLL接收超时时间
#define POLL_RX_TO_RESP_TX_DLY_UUS          500  	//POLL接收完成到开始发送RESP延迟时间
#define RESP_TX_TO_FINAL_RX_DLY_UUS         250   //RESP发送完成到开始接收FINAL延迟时间
#define FINAL_RX_TIMEOUT_UUS                700  	//FINAL接收超时时间

#define SHORT_ID_CONFIG_RX_DLY_UUS					800		//16位ID设置帧接收延时时间
#define SHORT_ID_CONFIG_RX_TIMEOUT_UUS			1500	//16位ID设置帧接收超时时间
#define SHORT_ID_CONFIG_TX_DLY_UUS					1000	//16位ID设置帧发送延时时间

#define LISTEN_CHANNEL_TIME									700		//信道监听时间700us

#define SPEED_OF_LIGHT                      299702547  //光速

#define ASK_ADDR_DELAY_MS										100

#define ADDR_16BIT_MAX_NUM									255			//16Bit地址分配最大值--256个地址，该值最大为65535-3（0xFFFC）:0xFFFF--广播帧，0xFFFE--无可分配地址，0xFFFD--CLE地址（该值固定）

#define TAG_NUM_MAX													5

#define CSMA_CD_MS													4		//随机回退时间单位使用
#define TAG_COMM_TO_ALL_ANC_TIME_MS					100	//标签和所有基站的测距通讯时间

#define SYSTEM_COMM_CYCLE_MS								100		//人员定位系统通讯周期(面内)
#define SYSTEM_COMM_CYCLE_LP_MS							5000	//人员定位系统通讯周期(面外)
#define TAG_EXIT_WORKSPACE_JUDGE_TIME_MS		5000	//人员离面判定时间

#define SYSTEM_RESET_JUDGE_TIME_MS					5000	//系统强制复位判定时间
#define ID_CONFIG_JUDGE_TIME_MS							1000	//进入ID配置模式判定时间
#define SPI_TIMEOUT													23		//SPI通讯超时时间--5次while循环

/*工作面内的基站数量*/
#define	ANC_NUM															255	//基站的数量

/*一个通讯周期内，标签与基站的数量对应关系*/
#define TAG_COMM_TO_ANC_NUM_MAX							5		//标签在一个通讯周期内通讯的基站数
#define TAG_COMM_TO_ANC_CNT									1		//标签在一个通讯周期内与同一基站的通讯次数

#define ANC_COMM_NUM_MAX_WITH_TAG						10	//基站能同时容纳的标签量


//#define __DEBUG

#ifdef __DEBUG
	#define Debug(format,...)			printf("File: "__FILE__", Line: %d, "format,__LINE__,##__VA_ARGS__)			//打印报文信息，测试版本功能
#else
	#define Debug(format,...)

#endif


/* USER CODE END Private defines */

void WriteToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB);
void ReadFromMsg(uint8_t *pdata, uint8_t *pMsg, uint8_t MsgOffset, uint8_t DataLength, uint8_t IsMSB);
uint8_t WriteStringToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset);
uint8_t ReadStringFromMsg(uint8_t *pData, uint8_t *pMsg, uint8_t MsgOffset);

void FinalMsgGetTS(const uint8 *ts_field, uint32 *ts);
void FinalMsgSetTS(uint8 *ts_field, uint64_t ts);

uint64_t GetRxTimeStamp_u64(void);
uint64_t GetTxTimeStamp_u64(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
