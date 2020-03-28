#ifndef     __VOICE_H
#define    __VOICE_H

#define ADCBUFFER    80//ADC ���ݻ���
#define OFFLINETICK  5000
#define PLAYALLDELAY 8000

#define  FINITEPLAYLISTDEPTH   60  
#define  INFINITEPLAYLISTDEPTH   60  

#define BUFFER_EMPTY 1
#define BUFFER_GETOK 2
#define BUFFER_RELEASEOK 3

#define STOP_ALLFINITE            0x00
#define STOP_ALLINFINITE          0x01
#define STOP_SPCINFINITE          0x02

#define STOP_ACK_OK	        0x00
#define STOP_ACK_TYPEERR    0x01
#define CONFIG_ACK_OK       0x00
#define CONFIG_ACK_TYPEERR  0x01
#define CONFIG_ACK_OVERFLOW	0x02

/*��ʼ��ַλ��FLASH ��128��ҳ�棬ÿ��ҳ��Ϊ1K��FLASH��ʼ��ַΪ0x08000000*/
#define EEPROM_START_ADDRESS                ((u32)0x0801FC00)
#define EEPROM1                             ((u32)(EEPROM_START_ADDRESS + 0x00))     
#define EEPROM2                             ((u32)(EEPROM_START_ADDRESS + 0x04))

#define MSG_QUEUE_SIZE               100 

typedef struct {

  u8  segaddr;//����Ƭ�α��
	u8  playtype;//�������� 0Ϊ���޴Σ�1Ϊ���޴β���
	u8  playtimes;//���Ŵ��������ڲ�������Ϊ0ʱ��Ч
	u8  playprior;//�������ȼ�
//  u8  blockvalid;
//  u8  blockactive;
  u16 playinterval;//���ż��
}_PlayICB;
 

typedef struct _PlayICBBuf{

	u8  segaddr;//����Ƭ�α��
	u8  playtype;//�������� 0Ϊ���޴Σ�1Ϊ���޴β���
	u8  playtimes;//���Ŵ��������ڲ�������Ϊ0ʱ��Ч
	u8  playprior;//�������ȼ�
	u8  blockvalid;//��������Ч
//    u8  blockactive;
    u16  playinterval;//���ż��
    u16  playtick;

	struct _PlayICBBuf *prev;
	struct _PlayICBBuf *next;
}_PlayICBBuf;

typedef struct {

	u8 complete;  //���״̬	
	u8 timeinterval;//���ż��
	u8 PlayStatus;//����״̬
    u8 playtimes;//���Ŵ���
	u8 mode;      //����ģʽ
    u8 segaddr;//���Ŷε�ַ
	u8 allcomplete;//��������Ч
	u8 blockvalid;//��������Ч
    u16 fillnum; //


}_VM8978Status;

typedef struct
{
    u16 VolumePre;
    u16 VolumeNow;
}_VolumeCtr;

typedef struct
{
    u16 OffLineTimer;
    u16 PlayIntervalTimer;
}_SoftTimer;


//------------------------

extern _PlayICBBuf PlayIBC[FINITEPLAYLISTDEPTH];//���в����б�

extern _PlayICBBuf *FiniteListHead;//���޴δ������б�ͷָ�룬
extern _PlayICBBuf *FiniteListTail;//���޴δ������б�βָ��
extern _PlayICBBuf *FiniteListCurPlay;//���޴ε�ǰ����������ָ��

extern _PlayICBBuf FiniteList;//���޴β��ű���
extern _PlayICBBuf FiniteListBuf;//���޴β��ű��������

extern _PlayICBBuf *InFiniteListHead;//���޴δ������б�ͷָ�룬
extern _PlayICBBuf *InFiniteListTail;//���޴δ������б�βָ��
extern _PlayICBBuf *InFiniteListCurPlay;//���޴ε�ǰ����������ָ��


extern u8 FiniteListNodeNum;//���޴λ������ڵ���Ŀ
extern u8 InFiniteListNodeNum;//���ߴλ������ڵ���Ŀ
extern _VM8978Status VM8978Status;
extern _SoftTimer SoftTimer;
extern _VolumeCtr VolumeCtr;
extern u16 ADCConvertedValue[ADCBUFFER];

extern OS_EVENT *MsgQueue;
extern void     *MsgQueueTbl[MSG_QUEUE_SIZE];
//------------------------��������-------------------------------------------------
void Init_PlayStatus(void);
void Init_FreeList(void);

u8 PushNode(_PlayICB *pt);
s8 GetNode(_PlayICBBuf * pt,u8 prior);
s8 ReleaseNode(_PlayICBBuf * pt);
u8 DelNode(u16 segmentaddr);
u8 GetNodeMaxPrior(_PlayICBBuf * pt);
u8 Del_AllNode(void);


void DMAADCInit_Base(void);
void ADC1Init_Base(void);
void ADCfliter(u16 *p,u16 num);


#endif
/******************************************end of file*****************************/
