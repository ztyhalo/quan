#ifndef __UWP_APP_H__
#define __UWP_APP_H__


#include "stdint.h"
#include "stdbool.h"
#include "can_app.h"
#include "deca_device_api.h"
#include "deca_regs.h"
//#include "filtering.h"


#define DW1000_RX_LEN       64

#define TX_ANT_DLY                 16440      //���������ӳ�
#define RX_ANT_DLY                 16440      //���������ӳ�

#define PAN_ID 											0xDECA					//PAN IDֵ


#define FRAME_CTRL_BYTE_ADDR_LEN_BIT						0x4400    //MAC ֡���ƶ��е�ַ����λ
/*��λ֡��ʽ*/
#define FINAL_MSG_TS_LEN                      		5  //finally��Ϣ�У�ʱ������ȣ�5���ֽ�
#define FINAL_MSG_POLL_TX_TS_IDX              		10  //finally��Ϣ�У�POLL����ʱ�������
#define FINAL_MSG_RESP_RX_TS_IDX              		(FINAL_MSG_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN)  //finally��Ϣ�У�RESP����ʱ�������
#define FINAL_MSG_FINAL_TX_TS_IDX             		(FINAL_MSG_RESP_RX_TS_IDX +  FINAL_MSG_TS_LEN) //finally��Ϣ�У�FINAL����ʱ�������


typedef enum {
	ANC_INIT_SATA =0,
	ANC_RECEIVE_COMM,   //anchor�յ�����֡
	ANC_COMM_RES_SEND,
	ANC_RESCOMM_OK,
	ANC_RECE_POLL,
	ANC_RESPOLL_SEND,
	ANC_RESPOLL_OK,
	ANC_RECE_FINAL
	
}ANC_LOCATION_STEP;

typedef enum{
	ANC_LOCATION_OK = 0,
	ANC_LOCATION_ERR
	
}ANC_LOCATION_STATE;

typedef enum{
	ANC_FINAL_OUT_TIME = 1,
	
}ANC_ERROR;


typedef struct {
	ANC_LOCATION_STATE state;
	ANC_ERROR 				 err;
	ANC_LOCATION_STEP  step;
}sAncInfo;

void FinalMsgGetTS_64(const uint8_t *ts_field, uint64_t *ts);
uint64_t GetSysTimeStamp_u64(void);

void SetAncLocationStep(ANC_LOCATION_STEP step);
void SetAncLocationErr(ANC_ERROR err);
void UwbReceiveTask(void *pdata);

#endif /*__UWP_APP_H__*/
