#ifndef     __VOICE_H
#define    __VOICE_H

#define ADCBUFFER    80//ADC Êı¾İ»º³å
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

/*ÆğÊ¼µØÖ·Î»ÓÚFLASH µÚ128¸öÒ³Ãæ£¬Ã¿¸öÒ³ÃæÎª1K£¬FLASHÆğÊ¼µØÖ·Îª0x08000000*/
#define EEPROM_START_ADDRESS                ((u32)0x0801FC00)
#define EEPROM1                             ((u32)(EEPROM_START_ADDRESS + 0x00))     
#define EEPROM2                             ((u32)(EEPROM_START_ADDRESS + 0x04))

#define MSG_QUEUE_SIZE               100 

typedef struct {

  u8  segaddr;//ÓïÒôÆ¬¶Î±àºÅ
	u8  playtype;//²¥·ÅÀàĞÍ 0ÎªÓĞÏŞ´Î£¬1ÎªÎŞÏŞ´Î²¥·Å
	u8  playtimes;//²¥·Å´ÎÊı£¬½öÔÚ²¥·ÅÀàĞÍÎª0Ê±ÓĞĞ§
	u8  playprior;//²¥·ÅÓÅÏÈ¼¶
//  u8  blockvalid;
//  u8  blockactive;
  u16 playinterval;//²¥·Å¼ä¸ô
}_PlayICB;
 

typedef struct _PlayICBBuf{

	u8  segaddr;//ÓïÒôÆ¬¶Î±àºÅ
	u8  playtype;//²¥·ÅÀàĞÍ 0ÎªÓĞÏŞ´Î£¬1ÎªÎŞÏŞ´Î²¥·Å
	u8  playtimes;//²¥·Å´ÎÊı£¬½öÔÚ²¥·ÅÀàĞÍÎª0Ê±ÓĞĞ§
	u8  playprior;//²¥·ÅÓÅÏÈ¼¶
	u8  blockvalid;//ÓïÒô¶ÎÓĞĞ§
//    u8  blockactive;
    u16  playinterval;//²¥·Å¼ä¸ô
    u16  playtick;

	struct _PlayICBBuf *prev;
	struct _PlayICBBuf *next;
}_PlayICBBuf;

typedef struct {

	u8 complete;  //Íê³É×´Ì¬	
	u8 timeinterval;//²¥·Å¼ä¸ô
	u8 PlayStatus;//²¥·Å×´Ì¬
    u8 playtimes;//²¥·Å´ÎÊı
	u8 mode;      //²¥·ÅÄ£Ê½
    u8 segaddr;//²¥·Å¶ÎµØÖ·
	u8 allcomplete;//ÓïÒô¶ÎÓĞĞ§
	u8 blockvalid;//ÓïÒô¶ÎÓĞĞ§
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

extern _PlayICBBuf PlayIBC[FINITEPLAYLISTDEPTH];//¿ÕÏĞ²¥·ÅÁĞ±í

extern _PlayICBBuf *FiniteListHead;//ÓĞÏŞ´Î´ı²¥·ÅÁĞ±íÍ·Ö¸Õë£¬
extern _PlayICBBuf *FiniteListTail;//ÓĞÏŞ´Î´ı²¥·ÅÁĞ±íÎ²Ö¸Õë
extern _PlayICBBuf *FiniteListCurPlay;//ÓĞÏŞ´Îµ±Ç°²¥·ÅÓïÒô¿éÖ¸Õë

extern _PlayICBBuf FiniteList;//ÓĞÏŞ´Î²¥·Å±äÁ¿
extern _PlayICBBuf FiniteListBuf;//ÓĞÏŞ´Î²¥·Å±äÁ¿»º´æÎ

extern _PlayICBBuf *InFiniteListHead;//ÎŞÏŞ´Î´ı²¥·ÅÁĞ±íÍ·Ö¸Õë£¬
extern _PlayICBBuf *InFiniteListTail;//ÎŞÏŞ´Î´ı²¥·ÅÁĞ±íÎ²Ö¸Õë
extern _PlayICBBuf *InFiniteListCurPlay;//ÎŞÏŞ´Îµ±Ç°²¥·ÅÓïÒô¿éÖ¸Õë


extern u8 FiniteListNodeNum;//ÓĞÏŞ´Î»º³åÇø½ÚµãÊıÄ¿
extern u8 InFiniteListNodeNum;//ÎŞÏß´Î»º³åÇø½ÚµãÊıÄ¿
extern _VM8978Status VM8978Status;
extern _SoftTimer SoftTimer;
extern _VolumeCtr VolumeCtr;
extern u16 ADCConvertedValue[ADCBUFFER];

extern OS_EVENT *MsgQueue;
extern void     *MsgQueueTbl[MSG_QUEUE_SIZE];
//------------------------º¯ÊıÉùÃ÷-------------------------------------------------
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
