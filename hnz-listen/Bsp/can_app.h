/*
*********************************************************************************************************
*	                                            UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : can_app.h
*    Module  : can driver
*    Version : V1.0
*    History :
*   -----------------
*             can底层驱动
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/

#ifndef __CAN_APP_H__
#define __CAN_APP_H__

#include "l4can.h"
#include "can_bus.h"
#include "string.h"
#include "protocol.h"
#include "can_buf.h"

/****************************************************************************************************
  CAN  version define
 ***************************************************************************************************/
#define CAN_POSITION_MODE 		0
#define CAN_MONITOR_MODE 			1

#define CAN_TASK_MODE   CAN_MONITOR_MODE

#define UWB_ID  7
#define CAN_DEV_ID  0


/****************************************************************************************************
 CAN  MESSAGE PARA DEFINE
 ***************************************************************************************************/



/****************************************************************************************************
 CAN  ERROR DEFINE
 ***************************************************************************************************/
#define CAN_TX_FULL   -1
#define INSERT_FAIL   -2
#define INIT_ERR_ID   -3



#define REQ_CONFIG_DELAY 500
#define UWB_HEART_DELAY 4000
#define UWB_MODEL        0x31

/****************************************************************************************************
 UWB MODULE STATE
 ***************************************************************************************************/
 enum
 {
	UWB_INIT = 0,
 	UWB_NORMAL
 
 };

 
 enum
 {
	 UWB_CONFIG_REQ = 0,
	 UWB_INFO_REPORT,
	 REQ_UWB_INFO,
	 SET_UWB_PARA,	 
	 UWB_HEART = 6
	 
 };
 
 
 typedef  union {
		struct {
			u32 RID:3;				//目标ID(接收方)
			u32 TID:3;				//源ID(发送方)
			u32 FT:10;				//帧类型
			u32 SN:4;				//子帧序列号
			u32 SUM:5;				//总帧数
			u32 SUB:1;				//标明是总帧还是子帧
			u32 ACK:1;				//应答位
			u32 RD:2;				//保留位
		} ID;
		u32 u32Id;
} CanHeadID;
 /****************************************************************************************************
uwb dev of info
 ***************************************************************************************************/
 typedef struct
 {
		uint16_t standNum;   //支架号
		union {
			struct {
				uint8_t workType:1; //工作模式
				uint8_t distType:1;  //距离类型
				uint8_t reserve:2;   //保留
				uint8_t rate:4;      //发射功率
			}sUwbSet;
			uint8_t uPara;
		}uwbPara;
//		uint8_t devType;
		uint8_t interval;    //时间间隔
		uint16_t extent;      //检测范围
		uint16_t  height;     //支架高度
					
}sUWB_RUN_INFO;
 

typedef struct
{
	uint8_t peoplenum; //人数
	union {
		struct {
			uint8_t tagState:4; //标签状态
			uint8_t author:4;    //权限
		}sTag;
		uint8_t tag;
	}tagInfo;
	uint16_t tagId;       //标签id
	uint16_t tagDist;     //标签距离
	uint8_t  reserve;
	uint8_t  crc;
}sTag_Info;
/****************************************************************************************************
CAN TX MESSAGE 	STATE
 ***************************************************************************************************/
enum									//发送结果定义 
{
	CAN_TRS_FAIL = 0,
	CAN_TRS_SUCCESS
};


/****************************************************************************************************
CAN TX MESSAGE 	TYPE ENUM
 ***************************************************************************************************/
enum									//发送类型
{
	CAN_ASK = 0,
	CAN_ACK
};

/****************************************************************************************************
CAN rx and tx task  priority
 ***************************************************************************************************/

#define	 CANTX_TASK_PRIO												 10u      /* can transmit task                        */
#define	 CANRX_TASK_PRIO												 4u      /* can receive  task                        */
/****************************************************************************************************
 CAN rx and tx task  stacks
 ***************************************************************************************************/
#define		CANTX_TASK_SIZE                        256u
#define 	CANRX_TASK_SIZE                        256u

extern sUWB_RUN_INFO 	gUwbRunPara;
extern sCAN_BUFFER 		gUwbCanTxBuf;

void WaitUwbStartSem(void);
void CanAppInit   (void);

uint16_t GetUwbId(void);

int32_t  WriteReportInfo(uint16_t tagNum, uint16_t tagId, uint16_t dist, uint16_t state);
int32_t CheckTagState(uint16_t dist);
uint16_t GetUwbReportTime(void);

#endif /*__CAN_APP_H__*/


