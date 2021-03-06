#ifndef     __VOICE_H
#define    __VOICE_H

#define ADCBUFFER    80//ADC 数据缓冲
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

/*起始地址位于FLASH 第128个页面，每个页面为1K，FLASH起始地址为0x08000000*/
#define EEPROM_START_ADDRESS                ((u32)0x0801FC00)
#define EEPROM1                             ((u32)(EEPROM_START_ADDRESS + 0x00))     
#define EEPROM2                             ((u32)(EEPROM_START_ADDRESS + 0x04))

#define MSG_QUEUE_SIZE               100 

typedef struct {

  u8  segaddr;//语音片段编号
	u8  playtype;//播放类型 0为有限次，1为无限次播放
	u8  playtimes;//播放次数，仅在播放类型为0时有效
	u8  playprior;//播放优先级
//  u8  blockvalid;
//  u8  blockactive;
  u16 playinterval;//播放间隔
}_PlayICB;
 

typedef struct _PlayICBBuf{

	u8  segaddr;//语音片段编号
	u8  playtype;//播放类型 0为有限次，1为无限次播放
	u8  playtimes;//播放次数，仅在播放类型为0时有效
	u8  playprior;//播放优先级
	u8  blockvalid;//语音段有效
//    u8  blockactive;
    u16  playinterval;//播放间隔
    u16  playtick;

	struct _PlayICBBuf *prev;
	struct _PlayICBBuf *next;
}_PlayICBBuf;

typedef struct {

	u8 complete;  //完成状态	
	u8 timeinterval;//播放间隔
	u8 PlayStatus;//播放状态
    u8 playtimes;//播放次数
	u8 mode;      //播放模式
    u8 segaddr;//播放段地址
	u8 allcomplete;//语音段有效
	u8 blockvalid;//语音段有效
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

extern _PlayICBBuf PlayIBC[FINITEPLAYLISTDEPTH];//空闲播放列表

extern _PlayICBBuf *FiniteListHead;//有限次待播放列表头指针，
extern _PlayICBBuf *FiniteListTail;//有限次待播放列表尾指针
extern _PlayICBBuf *FiniteListCurPlay;//有限次当前播放语音块指针

extern _PlayICBBuf FiniteList;//有限次播放变量
extern _PlayICBBuf FiniteListBuf;//有限次播放变量缓存�

extern _PlayICBBuf *InFiniteListHead;//无限次待播放列表头指针，
extern _PlayICBBuf *InFiniteListTail;//无限次待播放列表尾指针
extern _PlayICBBuf *InFiniteListCurPlay;//无限次当前播放语音块指针


extern u8 FiniteListNodeNum;//有限次缓冲区节点数目
extern u8 InFiniteListNodeNum;//无线次缓冲区节点数目
extern _VM8978Status VM8978Status;
extern _SoftTimer SoftTimer;
extern _VolumeCtr VolumeCtr;
extern u16 ADCConvertedValue[ADCBUFFER];

extern OS_EVENT *MsgQueue;
extern void     *MsgQueueTbl[MSG_QUEUE_SIZE];
//------------------------函数声明-------------------------------------------------
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
