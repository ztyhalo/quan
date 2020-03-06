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

#define USE_OLED							//ʹ��OLED��ʾ��
#define PCB_V1								//ӡ�ư��һ��
//#define PCB_V2								//ӡ�ư�ڶ���

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


/*ģ�鹤��ģʽ����*/
#define DECA_TAG							0 //��ǩ
#define DECA_ANCHOR						1 //����վ
#define DECA_CLE							3 //����λ�ô���Ԫ
#define DECA_REMOTECONTROL		4 //ң��

/*�������ȼ�����*/
#define KEY_CHECK_PRIO		      	5		
#define UWB_TAG_PRIO             	6
#define UWB_ANCHOR_PRIO          	7
#define UWB_ID_CONFIG_PRIO				11
#define LED_BLINK_PRIO           	27

/*֡���Ͷ���*/
#define FRAME_SHORT_ID_ASK	 								1 	//16λID����
#define FRAME_SHORT_ID_CONFIG	 							2		//16λ��ַ����
#define FRAME_FOR_16BIT_ADDR_ENSURE					3		//16λ��ַȷ��
#define FRAME_FOR_16BIT_ADDR_ENSURE_RESP		4		//16λ��ַȷ�ϻظ�
#define FRAME_FOR_FIRST_CLOCK_SYNC					5		//�״�ʱ��ͬ����ʼ
#define FRAME_FOR_CLOCK_SYNC								6		//ʱ��ͬ��
#define FRAME_FOR_TAG_BEACON								7		//��ǩ��λ�㲥֡
#define FRAME_FOR_TS												8		//��λ��Ϣ֡
#define FRAME_SUPER													9		//��֡
#define FRAME_ASK_FOR_GTS										10	//GTS����֡
#define	FRAME_POLL													11	//Poll֡
#define FRAME_RESP													12	//Resp֡
#define FRAME_FINAL													13	//Final֡
#define FRAME_DIST_INFO											14	//������Ϣ֡
#define FRAME_COMM_DEC											15	//TagͨѶ����֡
#define FRAME_COMM_DEC_RESP									16	//AncͨѶ�����ظ�֡
#define FRAME_CONFIG_ID											32	//��ǩID��Ϣ����֡

/*�����ӳ�ʱ��*/
#define TX_ANT_DLY                 16440      //���������ӳ�
#define RX_ANT_DLY                 16440      //���������ӳ�

/*MAC ֡�ֶζ���*/
#define FRAME_CTRL_BYTE_FRAME_TYPE_BIT					0x0007    //MAC ֡���ƶ���֡����λ
#define FRAME_CTRL_BYTE_ACK_BIT									0x0020    //MAC ֡���ƶ���ȷ������λ
#define FRAME_CTRL_BYTE_PANID_BIT								0x0040    //MAC ֡���ƶ���PAN IDλ
#define FRAME_CTRL_BYTE_ADDR_LEN_BIT						0xCC00    //MAC ֡���ƶ��е�ַ����λ

#define FRAME_CTRL_BYTE_SOURCE_ID_64BIT					0xC800
#define FRAME_CTRL_BYTE_DEST_ID_64BIT						0x8C00

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

#define SHORT_ADDR_ASK_COMMON_LEN								12				//16λID����֡ͨ�ò��ֳ���


/*����λ����*/
/*ͨ��*/
#define FRAME_DEST_ADDR_IDX												5		//Ŀ���ַ��������
#define FRAME_SOURCE_ADDR_IDX			   							7		//Դ��ַ��������

#define ALL_MSG_SN_IDX                        		2   //֡����ֵ����

/*����*/
#define FRAME_DEC_TAG_ADDR_IDX										5		//ͨѶ����֡����ǩ��ַ����
#define FRMAE_DEC_RESP_TAG_ADDR_IDX								5		//ͨѶ�����ظ�֡����ǩ��ַ����
#define FRMAE_DEC_RESP_ANC_ADDR_IDX								7		//ͨѶ�����ظ�֡����վ��ַ����

#define FRAME_SHORT_ID_ASK_DEST_ADDR_IDX					5		//�豸ID����֡��Ŀ���ַ����
#define FRAME_SHORT_ID_ASK_SOURE_ADDR_IDX					7		//�豸ID����֡��Դ��ַ����

#define FRAME_SHORT_ID_CONFIG_DEST_ADDR_IDX				5		//�豸ID����֡��Ŀ���ַ����
#define FRAME_SHORT_ID_CONFIG_SOURE_ADDR_IDX			9		//�豸ID����֡��Դ��ַ����
#define FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX				12	//�豸ID����֡��16λID����ֵ����

#define FRAME_16BIT_ADDR_TYPE_IDX									9		//16Bit��ַ����֡��֡��������
#define FRAME_32BIT_ADDR_TYPE_IDX									11	//32Bit��ַ����֡��֡��������
#define FRAME_64BIT_ADDR_TYPE_IDX									15	//64Bit��ַ����֡��֡��������

#define BEACON_FRAME_TYPE_IDX											7		//�ű�֡��֡����������


/*��ʱʱ�估��ʱʱ��*/
#define FRAME_COMM_DEC_TX_TO_RESP_RX_DLY_UUS			400			//ͨѶ����֡������ɵ����ջظ�֡�ӳ�ʱ��
#define FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS			500			//ͨѶ����֡������ɵ����ͻظ�֡�ӳ�ʱ��
#define FRAME_COMM_DEC_RESP_RX_TIMEOUT_UUS				1000		//ͨѶ����֡�ظ����ճ�ʱʱ��


/*FLASH����λ����*/
//#define FLASH_ChipID_IDX														0		//FLASH���ݣ�32λCHIP ID����
//#define FLASH_16BIT_ID_IDX													32	//FLASH���ݣ�16λ��ַ����
//#define FLASH_MODE_IDX															56  //FLASH���ݣ�ģ���������
//#define FLASH_16BIT_ID_FLAG_IDX											63	//FLASH���ݣ�16λ��ַ��ʶλ����

/* UWB microsecond (uus) �� device time unit (dtu, 1/(499.2MHz*128)��15.65ps) ����ϵ��.
 * 1 uus = 512 / 499.2 us 
 * 1 us  = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME											65536		//UUS��DWTоƬʱ��ת��ϵ��

/*��ǩ����*/
//#define POLL_TX_TO_RESP_RX_DLY_UUS          2000	//POLL������ɵ���ʼ����RESP�ӳ�ʱ��
//#define RESP_RX_TIMEOUT_UUS                 2700	//RESP���ճ�ʱʱ��
//#define RESP_RX_TO_FINAL_TX_DLY_UUS         3100	//RESP������ɵ����Է���FINAL�ӳ�ʱ��

#define POLL_TX_TO_RESP_RX_DLY_UUS          150		//POLL������ɵ���ʼ����RESP�ӳ�ʱ��
#define RESP_RX_TIMEOUT_UUS                 2700	//RESP���ճ�ʱʱ��
#define RESP_RX_TO_FINAL_TX_DLY_UUS         400		//RESP������ɵ����Է���FINAL�ӳ�ʱ��   ��С����Ϊ260

/*��վ����*/
//#define POLL_RX_TIMEOUT_UUS									1000	//POLL���ճ�ʱʱ��
//#define POLL_RX_TO_RESP_TX_DLY_UUS          2600  //POLL������ɵ���ʼ����RESP�ӳ�ʱ��
//#define RESP_TX_TO_FINAL_RX_DLY_UUS         2500  //RESP������ɵ���ʼ����FINAL�ӳ�ʱ��
//#define FINAL_RX_TIMEOUT_UUS                4300  //FINAL���ճ�ʱʱ��

#define POLL_RX_TIMEOUT_UUS									1000	//POLL���ճ�ʱʱ��
#define POLL_RX_TO_RESP_TX_DLY_UUS          500  	//POLL������ɵ���ʼ����RESP�ӳ�ʱ��
#define RESP_TX_TO_FINAL_RX_DLY_UUS         250   //RESP������ɵ���ʼ����FINAL�ӳ�ʱ��
#define FINAL_RX_TIMEOUT_UUS                700  	//FINAL���ճ�ʱʱ��

#define SHORT_ID_CONFIG_RX_DLY_UUS					800		//16λID����֡������ʱʱ��
#define SHORT_ID_CONFIG_RX_TIMEOUT_UUS			1500	//16λID����֡���ճ�ʱʱ��
#define SHORT_ID_CONFIG_TX_DLY_UUS					1000	//16λID����֡������ʱʱ��

#define LISTEN_CHANNEL_TIME									700		//�ŵ�����ʱ��700us

#define SPEED_OF_LIGHT                      299702547  //����

#define ASK_ADDR_DELAY_MS										100

#define ADDR_16BIT_MAX_NUM									255			//16Bit��ַ�������ֵ--256����ַ����ֵ���Ϊ65535-3��0xFFFC��:0xFFFF--�㲥֡��0xFFFE--�޿ɷ����ַ��0xFFFD--CLE��ַ����ֵ�̶���

#define TAG_NUM_MAX													5

#define CSMA_CD_MS													4		//�������ʱ�䵥λʹ��
#define TAG_COMM_TO_ALL_ANC_TIME_MS					100	//��ǩ�����л�վ�Ĳ��ͨѶʱ��

#define SYSTEM_COMM_CYCLE_MS								100		//��Ա��λϵͳͨѶ����(����)
#define SYSTEM_COMM_CYCLE_LP_MS							5000	//��Ա��λϵͳͨѶ����(����)
#define TAG_EXIT_WORKSPACE_JUDGE_TIME_MS		5000	//��Ա�����ж�ʱ��

#define SYSTEM_RESET_JUDGE_TIME_MS					5000	//ϵͳǿ�Ƹ�λ�ж�ʱ��
#define ID_CONFIG_JUDGE_TIME_MS							1000	//����ID����ģʽ�ж�ʱ��
#define SPI_TIMEOUT													23		//SPIͨѶ��ʱʱ��--5��whileѭ��

/*�������ڵĻ�վ����*/
#define	ANC_NUM															255	//��վ������

/*һ��ͨѶ�����ڣ���ǩ���վ��������Ӧ��ϵ*/
#define TAG_COMM_TO_ANC_NUM_MAX							5		//��ǩ��һ��ͨѶ������ͨѶ�Ļ�վ��
#define TAG_COMM_TO_ANC_CNT									1		//��ǩ��һ��ͨѶ��������ͬһ��վ��ͨѶ����

#define ANC_COMM_NUM_MAX_WITH_TAG						10	//��վ��ͬʱ���ɵı�ǩ��


//#define __DEBUG

#ifdef __DEBUG
	#define Debug(format,...)			printf("File: "__FILE__", Line: %d, "format,__LINE__,##__VA_ARGS__)			//��ӡ������Ϣ�����԰汾����
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
