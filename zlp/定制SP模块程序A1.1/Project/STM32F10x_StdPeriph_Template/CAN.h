# ifndef __CAN_H
#define  __CAN_H
#include "stm32f10x.h"
#include "voice.h"


//---------------------------- config ----------------------------
#define CAN_BUFFER_DEPTH                60
#define FRAMEID_UPTOLOW_PLAY          0x481
#define FRAMEID_UPTOLOW_STOP          0x482
#define FRAMEID_LOWTOUP_PLAYACK       0x681
#define FRAMEID_LOWTOUP_STOPACK       0x682
#define FRAMEID_ONLINECHECK           0x48F
#define FRAMEID_ONLINECHECKACK        0x68F
#define FRAMEID_UPTOLOW_RESET         0x5FF  //20190426
#define PLAYLISTDEPTH                 10



/****************************************************************************
* ��������: CAN���Ͷ���
* ʹ��˵��:
****************************************************************************/
typedef struct{
	u32 id;                             // ID
	u8 dlc;                             // ����֡�����ֽ���
	u8 msgflg;							// ��Ϣ���ԡ���չ֡���Ǳ�׼֡��
	u8 dat[8];                          // ����֡����
	u8 rsdintval;                       // �ط����
	u8 rsdtimer;
    u8 rsdcnt;                          // �ط�����
    u8 sndflg;                          // ���ͱ�ʶ
//    u8 successflg;                      // �ɹ���ʶ
}CAN_FrameStruct;

/****************************************************************************
* ��������: FIFO���Ͷ���
* ʹ��˵��:
****************************************************************************/

typedef struct{
    u8 wr;                              // ��ָ��
    u8 rd;                              // дָ��
    u8 num;                             // ��ǰFIFO������(��)����
    CAN_FrameStruct buf[CAN_BUFFER_DEPTH];
}CAN_FIFOStruct;
/****************************************************************************
* ��������: ����������Ϣ�
* ʹ��˵��:
****************************************************************************/
typedef struct {
// 	union {
// 		u16 segaddr16;
// 		struct{
// 			u8 addrlow;
// 			u8 addrHigh;
// 		}segaddr8;	
// 	}segaddr;
    u8  segaddr;
	u8  playtimes;
	u8  playtype;
// 	u8  playprior;
    u8  playstatus;//״̬ 0��δ���ţ�1�ǲ������
    u16  playinterval;

}Segment;
typedef struct {
    u8 savepoint;
    u8 readpoint;
    u8 totalnum;
    Segment Voicelist[PLAYLISTDEPTH];
   // Segment Voicelist[10];
}_Playlistinfo;
/****************************************************************************
* ��������: CAN�ܽŶ����
* ʹ��˵��:
****************************************************************************/
// CAN1
#define CAN1_PORT			  GPIOA
#define CAN1_RX_PIN			GPIO_Pin_11
#define CAN1_TX_PIN			GPIO_Pin_12

#define CAN1_GPIO_RCC		RCC_AHB1Periph_GPIOA
#define CAN1_APB			APB1
#define CAN1_RCC			RCC_APB1Periph_CAN1


extern CAN_FIFOStruct CANRxBuf;
extern CAN_FIFOStruct CANTxBuf;
extern CAN_FIFOStruct CANRTxBuf;
extern _Playlistinfo Playlistinfo ;
extern u8 FiniteNodeSame;  //�����޴νṹ���е�Ԫ����ͬ��־λ��Ϊ1ʱ��ʾ��ͬ
extern u8 Bisuocomplete;//���������߱�����Ƥ�����߱���������ɱ�־λ��Ϊ1ʱ��ʾ������ɡ�
extern u32 RESETID;
/****************************************************************************
* ��������: public��������
* ʹ��˵��:
****************************************************************************/
/*CAN��ʼ��*/
void CAN1Init(void);
/*��ʼ��FIFO���ݽṹ*/
void CAN_FIFOFlush(CAN_FIFOStruct *pBuf);

/*��FIFO�з���һ��(��)����,�ɹ�����0*/
void CAN_FIFOPutOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*��FIFO��ȡ��һ��(��)����,�ɹ�����0*/
void CAN_FIFOGetOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*��FIFO�п���һ��(��)����,�ɹ�����0*/
void CAN_FIFOCopyOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*ɾ��FIFO�е�һ��(��)����,�ɹ�����0*/
void CAN_FIFODeleteOne(CAN_FIFOStruct *pBuf);

/*�鿴һ��FIFO�������м�������(��)*/
u8 CAN_FIFOPeek(CAN_FIFOStruct *pBuf);

/*CAN��������,�ڸ����ȼ��ж���ֱ�ӵ���*/
void CAN_RcvDataInt(void);

/*CAN��������,����ѭ����ֱ�ӵ���*/
void CAN_SendData(CAN_FrameStruct *q);

//
void CAN_PlaySoundAnalyse(CAN_FrameStruct *CanMsg);
//
void ClearPlaylist(u8 cmd,u8 segid);
//
void CAN_TxFrame(u32 AnswerCode,u8 *p,u8 Dlc);
//
u8 CAN_RxDateAnalyse(_PlayICB *pt);
//
u8 CAN_RxStopDateAnalyse(u8 *pt);

#endif



/****************************end of file*****************************/
