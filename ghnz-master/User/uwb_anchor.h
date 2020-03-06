#ifndef __UWB_ANCHOR_H__
#define __UWB_ANCHOR_H__


#include "stdint.h"
#include "stdbool.h"
#include "can_app.h"
#include "deca_device_api.h"
#include "deca_regs.h"
//#include "filtering.h"
#include "zlist.h"


#define DW_TIME_OVER        							0x10000000000


#define ANCHOR_TX_ANT_DLY                 16440      //���������ӳ�
#define ANCHOR_RX_ANT_DLY                 16440      //���������ӳ�

#define ANCHOR_PAN_ID 										0xDECA					//��վPAN IDֵ

#define ANCHOR_ADDR_OFF										7		//��վ��ַ��֡�е�ƫ����
#define ENCHOR_SOURECE_ADDR_OFF            5

#define ANC_RX_LEN_MAX										64


typedef struct
{
	uint16_t devNum;            //�豸���
	uint16_t devState;
	uint16_t currDevId;         //��ǰ���ж�λ�ı�ǩid��
	uint32_t locCount;          //��λ�ɹ�����
	uint32_t locErrNum;         //��λ�������
	uint32_t beaconNum;         //�����ű�֡����
	uint32_t rxDataNum;           //��������֡����
	uint32_t sendNum;           //���͵�֡����
	uint32_t dataNumErr;        //�������ݴ������
	uint32_t waitErr;        //�ȴ��������
}sAncDevInfo;

typedef struct
{
	uint64_t t1;
	uint64_t t2;
	uint64_t t3;
	uint64_t t4;
	uint64_t t5;
	uint64_t t6;
}sLocTime;


typedef enum{
	UWB_DATA_STATE =0,
	UWB_LOCATION_STATE
}UWB_DEV_STATE;

#define FRAME_CTRL_BYTE_ADDR_LEN_BIT						0x4400    //MAC ֡���ƶ��е�ַ����λ
/*��λ֡��ʽ*/
#define FINAL_MSG_TS_LEN                      		5  //finally��Ϣ�У�ʱ������ȣ�5���ֽ�
#define FINAL_MSG_POLL_TX_TS_IDX              		10  //finally��Ϣ�У�POLL����ʱ�������
#define FINAL_MSG_RESP_RX_TS_IDX              		(FINAL_MSG_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN)  //finally��Ϣ�У�RESP����ʱ�������
#define FINAL_MSG_FINAL_TX_TS_IDX             		(FINAL_MSG_RESP_RX_TS_IDX +  FINAL_MSG_TS_LEN) //finally��Ϣ�У�FINAL����ʱ�������

#define ANC_LOC_TAG_MAX									10	//��վ��ͬʱ���ɵı�ǩ��
#define ANC_TOF_REC_MAX									20		//��ǩ��һ��ͨѶ��������ͬһ��վ��ͨѶ����
#define ANC_TOF_MIN_DATA                 5

typedef struct
{
	uint16_t tagId;                     //��¼�ı�ǩid
	uint16_t dist[ANC_TOF_REC_MAX];     //�ñ�ǩ���վ�ľ���buf
	uint16_t realDist;                  //����ȥ����ľ���
	uint16_t heartCout;           			//��������
	uint16_t oldHeartCout;           		//�ɵ���������
	uint8_t  commCnt;							      //�������ڱ�ǩ��ͬһ��վ��ͨѶ����
	uint8_t  mark;
	uint8_t  maxPtr;                      //���ֵָ��
	uint8_t  minPtr;	                   //��Сֵָ��
	uint8_t  errCount;                   //���ߴ������
}sDistNote;


// ����һ����������
LIST_DEFINE(Dist, sDistNote, ANC_LOC_TAG_MAX)
#ifdef __UWB_ANCHOR_C__
	LIST_DEFINE_FUNC(Dist, sDistNote, ANC_LOC_TAG_MAX)
#endif /*__UWB_ANCHOR_C__*/






void FinalMsgGetTS_64(const uint8_t *ts_field, uint64_t *ts);
uint64_t GetSysTimeStamp_u64(void);

void UwbAnchorTask(void *pdata);
sDistList_N * GetDistBufPoint(void);
void DistBufDataCount(sDistNote * data);
void DelDistBufData(sDistList_N * before, sDistList_N * del);
#endif /*__UWB_ANCHOR_H__*/
