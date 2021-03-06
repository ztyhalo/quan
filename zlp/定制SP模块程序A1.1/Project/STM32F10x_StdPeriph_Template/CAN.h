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
* 功能描述: CAN类型定义
* 使用说明:
****************************************************************************/
typedef struct{
	u32 id;                             // ID
	u8 dlc;                             // 数据帧数据字节数
	u8 msgflg;							// 消息属性、扩展帧还是标准帧。
	u8 dat[8];                          // 数据帧数据
	u8 rsdintval;                       // 重发间隔
	u8 rsdtimer;
    u8 rsdcnt;                          // 重发次数
    u8 sndflg;                          // 发送标识
//    u8 successflg;                      // 成功标识
}CAN_FrameStruct;

/****************************************************************************
* 功能描述: FIFO类型定义
* 使用说明:
****************************************************************************/

typedef struct{
    u8 wr;                              // 读指针
    u8 rd;                              // 写指针
    u8 num;                             // 当前FIFO内数据(包)数量
    CAN_FrameStruct buf[CAN_BUFFER_DEPTH];
}CAN_FIFOStruct;
/****************************************************************************
* 功能描述: 语音播放信息�
* 使用说明:
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
    u8  playstatus;//状态 0是未播放，1是播放完毕
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
* 功能描述: CAN管脚定义�
* 使用说明:
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
extern u8 FiniteNodeSame;  //与有限次结构体中的元素相同标志位，为1时表示相同
extern u8 Bisuocomplete;//工作面沿线闭锁或皮带沿线闭锁播放完成标志位，为1时表示播放完成。
extern u32 RESETID;
/****************************************************************************
* 功能描述: public函数声明
* 使用说明:
****************************************************************************/
/*CAN初始化*/
void CAN1Init(void);
/*初始化FIFO数据结构*/
void CAN_FIFOFlush(CAN_FIFOStruct *pBuf);

/*给FIFO中放入一个(包)数据,成功返回0*/
void CAN_FIFOPutOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*从FIFO中取出一个(包)数据,成功返回0*/
void CAN_FIFOGetOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*从FIFO中拷贝一个(包)数据,成功返回0*/
void CAN_FIFOCopyOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q);

/*删除FIFO中第一个(包)数据,成功返回0*/
void CAN_FIFODeleteOne(CAN_FIFOStruct *pBuf);

/*查看一眼FIFO中现在有几个数据(包)*/
u8 CAN_FIFOPeek(CAN_FIFOStruct *pBuf);

/*CAN接收数据,在高优先级中断中直接调用*/
void CAN_RcvDataInt(void);

/*CAN发送数据,在主循环中直接调用*/
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
