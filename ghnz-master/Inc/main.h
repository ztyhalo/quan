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
#define UWB_LISTEN_MODE 2            //��������uwb����

#define DECA_WORK_MODE  UWB_ALARM_MODE

/* Private defines -----------------------------------------------------------*/

//#define USE_OLED
/*�������ƶ���*/
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


/*ģ�鹤��ģʽ����*/
#define DECA_TAG							0 //��ǩ
#define DECA_ANCHOR_MASTER		1 //����վ
#define DECA_ANCHOR_SLAVER		2 //�ӻ�վ
#define DECA_CLE							3 //����λ�ô���Ԫ
#define DECA_REMOTECONTROL		4 //ң��

/*�������ȼ�����*/
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

/*֡���Ͷ���*/
#define FRAME_FOR_16BIT_ADDR_ASK	 					1 	//16λ��ַ����
#define FRAME_FOR_16BIT_ADDR_RESP	 					2		//16λ��ַ����ظ�
#define FRAME_FOR_16BIT_ADDR_ENSURE					3		//16λ��ַȷ��
#define FRAME_FOR_16BIT_ADDR_ENSURE_RESP		4		//16λ��ַȷ�ϻظ�
#define FRAME_FOR_FIRST_CLOCK_SYNC					5		//�״�ʱ��ͬ����ʼ
#define FRAME_FOR_CLOCK_SYNC								6		//ʱ��ͬ��
#define FRAME_FOR_TAG_BEACON								7		//��ǩ��λ�㲥֡
#define FRAME_FOR_TS												8		//��λ��Ϣ֡
#define FRAME_SUPER													9		//��֡
#define FRAME_ASK_FOR_GTS										10	//GTS����֡


/*�����ӳ�ʱ��*/
#define TX_ANT_DLY                 16440      //���������ӳ�
#define RX_ANT_DLY                 16440      //���������ӳ�

/*MAC ֡�ֶζ���*/
#define FRAME_CTRL_BYTE_FRAME_TYPE_BIT					0x0007    //MAC ֡���ƶ���֡����λ
#define FRAME_CTRL_BYTE_ACK_BIT									0x0020    //MAC ֡���ƶ���ȷ������λ
#define FRAME_CTRL_BYTE_PANID_BIT								0x0040    //MAC ֡���ƶ���PAN IDλ
#define FRAME_CTRL_BYTE_ADDR_LEN_BIT						0x4400    //MAC ֡���ƶ��е�ַ����λ

/*IEEE 802.15.4	֡����*/
#define FRAME_BEACON										0x0000		//�ű�֡
#define FRAME_DATA											0x0001		//����֡
#define FRAME_ACK												0x0002		//ȷ��֡
#define FRAME_CMD												0x0003		//����֡
#define FRAME_RESERVED_4								0x0004		//����֡4
#define FRAME_RESERVED_5								0x0005		//����֡5
#define FRAME_RESERVED_6								0x0006		//����֡6
#define FRAME_RESERVED_7								0x0007		//����֡7


#define ADDR_FOR_BEACON									0xFFFF		//�ű�֡�㲥��ַ
	

/*����*/
#define ALL_MSG_COMMON_LEN         							10				//ͨѶ֡ͨ�ò��ֳ���

#define RX_BUF_LEN            									55
#define RX_BUF_LEN_MAX													1024			//���ջ�������󳤶�

#define SHORT_ADDR_ASK_COMMON_LEN								12				//16λ��ַ����֡ͨ�ò��ֳ���


/*����λ����*/	
#define SOURCE_ADDR_SET_IDX			   								7		//Դ��ַ��������
#define ALL_MSG_COMMON_DEST_ADDR_IDX							5		//��Ϣ֡ͨ��Ŀ���ַ����

#define ALL_MSG_SN_IDX                        		2   //֡����ֵ����
//#define FINAL_MSG_POLL_TX_TS_IDX              		10  //finally��Ϣ�У�POLL����ʱ�������
//#define FINAL_MSG_RESP_RX_TS_IDX              		14  //finally��Ϣ�У�RESP����ʱ�������
//#define FINAL_MSG_FINAL_TX_TS_IDX             		18  //finally��Ϣ�У�FINAL����ʱ�������
//#define FINAL_MSG_TS_LEN                      		4   //finally��Ϣ�У�ʱ������ȣ�4���ֽ�

#define FRAME_DEC_TAG_ADDR_IDX										5		//ͨѶ����֡����ǩ��ַ����
#define FRMAE_DEC_RESP_TAG_ADDR_IDX								5		//ͨѶ�����ظ�֡����ǩ��ַ����
#define FRMAE_DEC_RESP_ANC_ADDR_IDX								7		//ͨѶ�����ظ�֡����վ��ַ����

#define FRAME_POLL_ANC_ADDR_IDX										5		//Poll֡����վ��ַ����
#define FRAME_POLL_TAG_ADDR_IDX										7		//Poll֡����ǩ��ַ����
#define	FRAME_RESP_TAG_ADDR_IDX										5		//Resp֡����ǩ��ַ����
#define FRAME_RESP_ANC_ADDR_IDX										7		//Resp֡����վ��ַ����
#define FRAME_FINAL_TAG_ADDR_IDX									7		//Final֡����ǩ��ַ����
#define FRAME_FINAL_ANC_ADDR_IDX									5		//Fianl֡����վ��ַ����


#define FRAME_32BIT_ADDR_TYPE_IDX									11	//32Bit��ַ����֡��֡��������

#define BEACON_FRAME_TYPE_IDX											7		//�ű�֡��֡����������


/*��ʱʱ�估��ʱʱ��*/
#define FRAME_COMM_DEC_TX_TO_RESP_RX_DLY_UUS			150			//ͨѶ����֡������ɵ����ջظ�֡�ӳ�ʱ��
//#define FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS			300			//ͨѶ����֡������ɵ����ͻظ�֡�ӳ�ʱ��
#define FRAME_COMM_DEC_RESP_RX_TIMEOUT_UUS				500			//ͨѶ����֡�ظ����ճ�ʱʱ��


/*FLASH����λ����*/
#define UWB_ChipID_IDX														0		//FLASH���ݣ�32λCHIP ID����
#define UWB_16BIT_ADDR_IDX												32	//FLASH���ݣ�16λ��ַ����
#define UWB_MODE_IDX															56  //FLASH���ݣ�ģ���������
#define UWB_16BIT_ADDR_FLAG_IDX										63	//FLASH���ݣ�16λ��ַ��ʶλ����



/*��ǩ����*/
//#define POLL_TX_TO_RESP_RX_DLY_UUS          150		//POLL������ɵ���ʼ����RESP�ӳ�ʱ��
//#define RESP_RX_TIMEOUT_UUS                 2700	//RESP���ճ�ʱʱ��
//#define RESP_RX_TO_FINAL_TX_DLY_UUS         3100	//RESP������ɵ����Է���FINAL�ӳ�ʱ��

#define POLL_TX_TO_RESP_RX_DLY_UUS          150		//POLL������ɵ���ʼ����RESP�ӳ�ʱ��
#define RESP_RX_TIMEOUT_UUS                 2700	//RESP���ճ�ʱʱ��
#define RESP_RX_TO_FINAL_TX_DLY_UUS         300		//RESP������ɵ����Է���FINAL�ӳ�ʱ��   ��С����Ϊ260

/*��վ����*/
//#define POLL_RX_TIMEOUT_UUS									1000	//POLL���ճ�ʱʱ��
//#define POLL_RX_TO_RESP_TX_DLY_UUS          2600  //POLL������ɵ���ʼ����RESP�ӳ�ʱ��
//#define RESP_TX_TO_FINAL_RX_DLY_UUS         500   //RESP������ɵ���ʼ����FINAL�ӳ�ʱ��
//#define FINAL_RX_TIMEOUT_UUS                4300  //FINAL���ճ�ʱʱ��

#define POLL_RX_TIMEOUT_UUS									1000	//POLL���ճ�ʱʱ��

//#define RESP_TX_TO_FINAL_RX_DLY_UUS         150   //RESP������ɵ���ʼ����FINAL�ӳ�ʱ��
//#define FINAL_RX_TIMEOUT_UUS                700  	//FINAL���ճ�ʱʱ��

#define LISTEN_CHANNEL_TIME									200		//�ŵ�����ʱ��200us

#define SPEED_OF_LIGHT                      299702547  //����

#define ASK_ADDR_DELAY_MS										100

#define ADDR_16BIT_MAX_NUM									255			//16Bit��ַ�������ֵ--256����ַ����ֵ���Ϊ65535-3��0xFFFC��:0xFFFF--�㲥֡��0xFFFE--�޿ɷ����ַ��0xFFFD--CLE��ַ����ֵ�̶���

#define TAG_NUM_MAX													5

#define TAG_COMM_TO_ANC_TIME_MS							5		//��ǩ�ͻ�վ���β��ʱ�䣬��Ϊ�������ʱ�䵥λʹ��
#define TAG_COMM_TO_ALL_ANC_TIME_MS					100	//��ǩ�����л�վ�Ĳ��ͨѶʱ��

#define SYSTEM_COMM_CYCLE_MS								1000	//��Ա��λϵͳͨѶ����(����)
#define SYSTEM_COMM_CYCLE_LP_MS							30000	//��Ա��λϵͳͨѶ����(����)
#define TAG_EXIT_WORKSPACE_JUDGE_TIME_MS		5000	//��Ա�����ж�ʱ��

/*�������ڵĻ�վ����*/
#define	ANC_NUM															255	//��վ������

/*һ��ͨѶ�����ڣ���ǩ���վ��������Ӧ��ϵ*/
#define TAG_COMM_TO_ANC_NUM_MAX							7		//��ǩ��һ��ͨѶ������ͨѶ�Ļ�վ��





#define __DEBUG

#ifdef __DEBUG
	#define Debug(format,...)			printf("File: "__FILE__", Line: %d, "format,__LINE__,##__VA_ARGS__)			//��ӡ������Ϣ�����԰汾����
#else
	#define Debug(format,...)

#endif


//typedef struct
//{
//	uint16_t TagID;
//	uint16_t Dist[5];
//	uint16_t RealDist;
//	uint16_t heartCout;           //�����������
//	uint16_t oldHeartCout;           //�����������
//	uint8_t  CommCnt;							//�������ڱ�ǩ��ͬһ��վ��ͨѶ����
//	uint8_t  TagSta;							//��ǩ��״̬������ڸû�վ�ľ��룩��0--�����ϱ���Χ��1--С���ϱ���Χ
//	uint8_t  TagDistUpdataFlag;
//	uint8_t  mark;
//	uint8_t  MaxPtr;
//	uint8_t  MinPtr;	
//	uint8_t  errCount;          //���ߴ������
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
