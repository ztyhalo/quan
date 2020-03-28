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
* ¹¦ÄÜÃèÊö: CANÀàĞÍ¶¨Òå
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
typedef struct{
	u32 id;                             // ID
	u8 dlc;                             // Êı¾İÖ¡Êı¾İ×Ö½ÚÊı
	u8 msgflg;							// ÏûÏ¢ÊôĞÔ¡¢À©Õ¹Ö¡»¹ÊÇ±ê×¼Ö¡¡£
	u8 dat[8];                          // Êı¾İÖ¡Êı¾İ
	u8 rsdintval;                       // ÖØ·¢¼ä¸ô
	u8 rsdtimer;
    u8 rsdcnt;                          // ÖØ·¢´ÎÊı
    u8 sndflg;                          // ·¢ËÍ±êÊ¶
//    u8 successflg;                      // ³É¹¦±êÊ¶
}CAN_FrameStruct;

/****************************************************************************
* ¹¦ÄÜÃèÊö: FIFOÀàĞÍ¶¨Òå
* Ê¹ÓÃËµÃ÷:
****************************************************************************/

typedef struct{
    u8 wr;                              // ¶ÁÖ¸Õë
    u8 rd;                              // Ğ´Ö¸Õë
    u8 num;                             // µ±Ç°FIFOÄÚÊı¾İ(°ü)ÊıÁ¿
    CAN_FrameStruct buf[CAN_BUFFER_DEPTH];
}CAN_FIFOStruct;
/****************************************************************************
* ¹¦ÄÜÃèÊö: ÓïÒô²¥·ÅĞÅÏ¢å
* Ê¹ÓÃËµÃ÷:
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
    u8  playstatus;//×´Ì¬ 0ÊÇÎ´²¥·Å£¬1ÊÇ²¥·ÅÍê±Ï
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
* ¹¦ÄÜÃèÊö: CAN¹Ü½Å¶¨Òåå
* Ê¹ÓÃËµÃ÷:
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
extern u8 FiniteNodeSame;  //ÓëÓĞÏŞ´Î½á¹¹ÌåÖĞµÄÔªËØÏàÍ¬±êÖ¾Î»£¬Îª1Ê±±íÊ¾ÏàÍ¬
extern u8 Bisuocomplete;//¹¤×÷ÃæÑØÏß±ÕËø»òÆ¤´øÑØÏß±ÕËø²¥·ÅÍê³É±êÖ¾Î»£¬Îª1Ê±±íÊ¾²¥·ÅÍê³É¡£
extern u32 RESETID;
/****************************************************************************
* ¹¦ÄÜÃèÊö: publicº¯ÊıÉùÃ÷
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
/*CAN³õÊ¼»¯*/
void CAN1Init(void);
/*³õÊ¼»¯FIFOÊı¾İ½á¹¹*/
void CAN_FIFOFlush(CAN_FIFOStruct *pBuf);

/*¸øFIFOÖĞ·ÅÈëÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0*/
void CAN_FIFOPutOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*´ÓFIFOÖĞÈ¡³öÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0*/
void CAN_FIFOGetOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*´ÓFIFOÖĞ¿½±´Ò»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0*/
void CAN_FIFOCopyOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*É¾³ıFIFOÖĞµÚÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0*/
void CAN_FIFODeleteOne(CAN_FIFOStruct *pBuf);

/*²é¿´Ò»ÑÛFIFOÖĞÏÖÔÚÓĞ¼¸¸öÊı¾İ(°ü)*/
u8 CAN_FIFOPeek(CAN_FIFOStruct *pBuf);

/*CAN½ÓÊÕÊı¾İ,ÔÚ¸ßÓÅÏÈ¼¶ÖĞ¶ÏÖĞÖ±½Óµ÷ÓÃ*/
void CAN_RcvDataInt(void);

/*CAN·¢ËÍÊı¾İ,ÔÚÖ÷Ñ­»·ÖĞÖ±½Óµ÷ÓÃ*/
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
