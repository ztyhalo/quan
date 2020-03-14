#ifndef __ARCHOR_H__
#define __ARCHOR_H__

#include "main.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	uint16_t TagID;
	uint16_t Dist[TAG_COMM_TO_ANC_CNT];
	uint16_t RealDist;						//�˲�������ʵ�ʾ��룬0xFFFF��ʾ�����Զ
	uint8_t  CommCnt;							//�������ڱ�ǩ��ͬһ��վ����ͨѶ����
	uint8_t  TagSta;							//��ǩ��״̬������ڸû�վ�ľ��룩��0--�����ϱ���Χ��1--С���ϱ���Χ
	uint8_t  TagDistUpdataFlag;
	uint8_t  MaxPtr;
	uint8_t  MinPtr;
}Tag2AncDist_s;


#define DW_TIME_OVER        0x10000000000

/*��λ֡��ʽ*/
#define FINAL_MSG_TS_LEN                      		5  //finally��Ϣ�У�ʱ������ȣ�5���ֽ�
#define FINAL_MSG_POLL_TX_TS_IDX              		10  //finally��Ϣ�У�POLL����ʱ�������
#define FINAL_MSG_RESP_RX_TS_IDX              		(FINAL_MSG_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN)  //finally��Ϣ�У�RESP����ʱ�������
#define FINAL_MSG_FINAL_TX_TS_IDX             		(FINAL_MSG_RESP_RX_TS_IDX +  FINAL_MSG_TS_LEN) //finally��Ϣ�У�FINAL����ʱ�������


void UWB_Anchor(void *pdata);
void Anc_TOF(uint8_t *pdata);
void ANC_CommDecResp(uint8_t *pdata);
int8_t AskTagStorePtr(uint16_t tagid);
uint16_t getTagStoredist(uint16_t tagid);
void CalculteTagRealDist(int8_t Ptr, uint8_t filtering);

bool caculate_distance(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t * val);
void FinalMsgGetTS_64(const uint8_t *ts_field, uint64_t *ts);

#endif /*__ARCHOR_H__*/
