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

#define UWB_TAG_MODE   	0
#define UWB_ALARM_MODE  1
#define UWB_LISTEN_MODE 2            //仅仅监听uwb数据

#define DECA_WORK_MODE  UWB_ALARM_MODE

/* Private defines -----------------------------------------------------------*/

//#define USE_OLED
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
	#define D_RS485_TX_Ctrl_Pin					GPIO_PIN_8
	#define D_RS485_TX_Ctrl_GPIO_Port		GPIOA
	#define LED1_PIN										GPIO_PIN_14
	#define LED1_PIN_Port								GPIOC
	#define LED3_PIN                		GPIO_PIN_2
	#define LED4_PIN                 		GPIO_PIN_1
	#define LED3_LED4_PIN_Port					GPIOA

	#define LED_RED_ON     HAL_GPIO_WritePin(LED3_LED4_PIN_Port, LED3_PIN, GPIO_PIN_RESET)
	#define LED_RED_OFF    HAL_GPIO_WritePin(LED3_LED4_PIN_Port, LED3_PIN, GPIO_PIN_SET)

	#define RS485_TX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_SET)
	#define RS485_RX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_RESET)
	
	//#define DWM1000_IRQHandler					EXTI0_IRQHandler
	
	#if defined(USE_OLED)
		#define I2C_SDA_Pin									GPIO_PIN_7
		#define I2C_SCL_Pin									GPIO_PIN_6
		#define I2C_GPIO_Port								GPIOB
	#endif
	
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

	#define LED_RED_ON     HAL_GPIO_WritePin(LED3_LED4_PIN_Port, LED3_PIN, GPIO_PIN_RESET)
	#define LED_RED_OFF    HAL_GPIO_WritePin(LED3_LED4_PIN_Port, LED3_PIN, GPIO_PIN_SET)

	#define RS485_TX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_SET)
	#define RS485_RX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_RESET)
	


#endif


/*模块工作模式定义*/
#define DECA_TAG							0 //标签
#define DECA_ANCHOR_MASTER		1 //主基站
#define DECA_ANCHOR_SLAVER		2 //从基站
#define DECA_CLE							3 //中央位置处理单元
#define DECA_REMOTECONTROL		4 //遥控

/*任务优先级定义*/
#define UWB_MAIN_PRIO             5
//#define	 CANTX_TASK_PRIO												 6u      /* can transmit task                        */
//#define	 CANRX_TASK_PRIO												 7u      /* can receive  task                        */
#define UWB_MODECHOOSE_PRIO      	5		
#define UWB_TAG_PRIO             	11
#define UWB_ANCHOR_PRIO          	10
#define UWB_REMOTECONTROL_PRIO   	8
#define USART1_RXDATA_PRIO       	9
#define USART1_TXDATA_PRIO       	10
#define UWB_GET16BITADDRESS_PRIO	11
#define UWB_CLE_PRIO							12
#define LED_BLINK_PRIO           	27

/*帧类型定义*/
#define FRAME_FOR_16BIT_ADDR_ASK	 					1 	//16位地址申请
#define FRAME_FOR_16BIT_ADDR_RESP	 					2		//16位地址申请回复
#define FRAME_FOR_16BIT_ADDR_ENSURE					3		//16位地址确认
#define FRAME_FOR_16BIT_ADDR_ENSURE_RESP		4		//16位地址确认回复
#define FRAME_FOR_FIRST_CLOCK_SYNC					5		//首次时钟同步开始
#define FRAME_FOR_CLOCK_SYNC								6		//时钟同步
#define FRAME_FOR_TAG_BEACON								7		//标签定位广播帧
#define FRAME_FOR_TS												8		//定位信息帧
#define FRAME_SUPER													9		//超帧
#define FRAME_ASK_FOR_GTS										10	//GTS请求帧


/*天线延迟时间*/
#define TX_ANT_DLY                 16440      //接收天线延迟
#define RX_ANT_DLY                 16440      //发送天线延迟

/*MAC 帧字段定义*/
#define FRAME_CTRL_BYTE_FRAME_TYPE_BIT					0x0007    //MAC 帧控制段中帧类型位
#define FRAME_CTRL_BYTE_ACK_BIT									0x0020    //MAC 帧控制段中确认请求位
#define FRAME_CTRL_BYTE_PANID_BIT								0x0040    //MAC 帧控制段中PAN ID位
#define FRAME_CTRL_BYTE_ADDR_LEN_BIT						0x4400    //MAC 帧控制段中地址长度位

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

#define SHORT_ADDR_ASK_COMMON_LEN								12				//16位地址申请帧通用部分长度


/*数据位索引*/	
#define SOURCE_ADDR_SET_IDX			   								7		//源地址设置索引
#define ALL_MSG_COMMON_DEST_ADDR_IDX							5		//消息帧通用目标地址索引

#define ALL_MSG_SN_IDX                        		2   //帧序列值索引
//#define FINAL_MSG_POLL_TX_TS_IDX              		10  //finally消息中，POLL发送时间戳索引
//#define FINAL_MSG_RESP_RX_TS_IDX              		14  //finally消息中，RESP发送时间戳索引
//#define FINAL_MSG_FINAL_TX_TS_IDX             		18  //finally消息中，FINAL发送时间戳索引
//#define FINAL_MSG_TS_LEN                      		4   //finally消息中，时间戳长度：4个字节

#define FRAME_DEC_TAG_ADDR_IDX										5		//通讯声明帧，标签地址索引
#define FRMAE_DEC_RESP_TAG_ADDR_IDX								5		//通讯声明回复帧，标签地址索引
#define FRMAE_DEC_RESP_ANC_ADDR_IDX								7		//通讯声明回复帧，基站地址索引

#define FRAME_POLL_ANC_ADDR_IDX										5		//Poll帧，基站地址索引
#define FRAME_POLL_TAG_ADDR_IDX										7		//Poll帧，标签地址索引
#define	FRAME_RESP_TAG_ADDR_IDX										5		//Resp帧，标签地址索引
#define FRAME_RESP_ANC_ADDR_IDX										7		//Resp帧，基站地址索引
#define FRAME_FINAL_TAG_ADDR_IDX									7		//Final帧，标签地址索引
#define FRAME_FINAL_ANC_ADDR_IDX									5		//Fianl帧，基站地址索引


#define FRAME_32BIT_ADDR_TYPE_IDX									11	//32Bit地址数据帧，帧类型索引

#define BEACON_FRAME_TYPE_IDX											7		//信标帧，帧子类型索引


/*延时时间及超时时间*/
#define FRAME_COMM_DEC_TX_TO_RESP_RX_DLY_UUS			150			//通讯声明帧发送完成到接收回复帧延迟时间
//#define FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS			300			//通讯声明帧接收完成到发送回复帧延迟时间
#define FRAME_COMM_DEC_RESP_RX_TIMEOUT_UUS				500			//通讯声明帧回复接收超时时间


/*FLASH数据位索引*/
#define UWB_ChipID_IDX														0		//FLASH数据，32位CHIP ID索引
#define UWB_16BIT_ADDR_IDX												32	//FLASH数据，16位地址索引
#define UWB_MODE_IDX															56  //FLASH数据，模块身份索引
#define UWB_16BIT_ADDR_FLAG_IDX										63	//FLASH数据，16位地址标识位索引



/*标签部分*/
//#define POLL_TX_TO_RESP_RX_DLY_UUS          150		//POLL发送完成到开始接收RESP延迟时间
//#define RESP_RX_TIMEOUT_UUS                 2700	//RESP接收超时时间
//#define RESP_RX_TO_FINAL_TX_DLY_UUS         3100	//RESP接收完成到考试发送FINAL延迟时间

#define POLL_TX_TO_RESP_RX_DLY_UUS          150		//POLL发送完成到开始接收RESP延迟时间
#define RESP_RX_TIMEOUT_UUS                 2700	//RESP接收超时时间
#define RESP_RX_TO_FINAL_TX_DLY_UUS         300		//RESP接收完成到考试发送FINAL延迟时间   最小可设为260

/*基站部分*/
//#define POLL_RX_TIMEOUT_UUS									1000	//POLL接收超时时间
//#define POLL_RX_TO_RESP_TX_DLY_UUS          2600  //POLL接收完成到开始发送RESP延迟时间
//#define RESP_TX_TO_FINAL_RX_DLY_UUS         500   //RESP发送完成到开始接收FINAL延迟时间
//#define FINAL_RX_TIMEOUT_UUS                4300  //FINAL接收超时时间

#define POLL_RX_TIMEOUT_UUS									1000	//POLL接收超时时间

//#define RESP_TX_TO_FINAL_RX_DLY_UUS         150   //RESP发送完成到开始接收FINAL延迟时间
//#define FINAL_RX_TIMEOUT_UUS                700  	//FINAL接收超时时间

#define LISTEN_CHANNEL_TIME									200		//信道监听时间200us

#define SPEED_OF_LIGHT                      299702547  //光速

#define ASK_ADDR_DELAY_MS										100

#define ADDR_16BIT_MAX_NUM									255			//16Bit地址分配最大值--256个地址，该值最大为65535-3（0xFFFC）:0xFFFF--广播帧，0xFFFE--无可分配地址，0xFFFD--CLE地址（该值固定）

#define TAG_NUM_MAX													5

#define TAG_COMM_TO_ANC_TIME_MS							5		//标签和基站单次测距时间，作为随机回退时间单位使用
#define TAG_COMM_TO_ALL_ANC_TIME_MS					100	//标签和所有基站的测距通讯时间

#define SYSTEM_COMM_CYCLE_MS								1000	//人员定位系统通讯周期(面内)
#define SYSTEM_COMM_CYCLE_LP_MS							30000	//人员定位系统通讯周期(面外)
#define TAG_EXIT_WORKSPACE_JUDGE_TIME_MS		5000	//人员离面判定时间

/*工作面内的基站数量*/
#define	ANC_NUM															255	//基站的数量

/*一个通讯周期内，标签与基站的数量对应关系*/
#define TAG_COMM_TO_ANC_NUM_MAX							7		//标签在一个通讯周期内通讯的基站数





#define __DEBUG

#ifdef __DEBUG
	#define Debug(format,...)			printf("File: "__FILE__", Line: %d, "format,__LINE__,##__VA_ARGS__)			//打印报文信息，测试版本功能
#else
	#define Debug(format,...)

#endif


//typedef struct
//{
//	uint16_t TagID;
//	uint16_t Dist[5];
//	uint16_t RealDist;
//	uint16_t heartCout;           //添加心跳计数
//	uint16_t oldHeartCout;           //添加心跳计数
//	uint8_t  CommCnt;							//单周期内标签与同一基站的通讯次数
//	uint8_t  TagSta;							//标签的状态（相对于该基站的距离）：0--大于上报范围，1--小于上报范围
//	uint8_t  TagDistUpdataFlag;
//	uint8_t  mark;
//	uint8_t  MaxPtr;
//	uint8_t  MinPtr;	
//	uint8_t  errCount;          //离线错误计数
//}Tag2AncDist_s;

typedef enum
{
	ChannelBusy = 0x00,
	ChannelFree = 0x01
}ChannelStatu;

//extern Tag2AncDist_s	TagDistBuff[10];	


/* USER CODE END Private defines */

void WriteToMsg(uint8_t *pMsg, uint8_t *pdata, uint8_t MsgOffset, uint8_t DataLength);
uint64_t GetRxTimeStamp_u64(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
