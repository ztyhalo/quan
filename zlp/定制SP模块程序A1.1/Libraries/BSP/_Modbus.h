#ifndef __MODBUS_H__
#define __MODBUS_H__

//------------------------- config -------------------------------
//#define _MODBUS_BUFFER
#define MODBUS_BUFFER_SIZE  20
#define MODBUS_BUFFER_DEPTH 5

/****************************************************************************
 * 功能描述: Modbus类型定义
 * 使用说明:
 ****************************************************************************/

struct tty_queue {
    u8 valid;                              // 缓冲里有一帧数据
    u8 head;                               // 头指针
    u8 tail;                               // 尾指针
    u8 buf[MODBUS_BUFFER_SIZE];                  // 缓冲区
};

struct tty_struct {
    u8 stopped;                            // 停止标志
    struct tty_queue read_q;               // 收到的数据缓冲区
    struct tty_queue write_q;              // 待发送的数据缓冲区
    struct tty_queue secondary;            // 辅助缓冲区
};

struct tty_FIFOStruct{
    u8 wr;                                  // 读指针
    u8 rd;                                  // 写指针
    u8 num;                                 // 当前FIFO内数据(包)数量
    struct tty_queue Buffer[MODBUS_BUFFER_SIZE];    // FIFO存储体,存放数据的缓冲区
};


/****************************************************************************
* 功能描述: public函数声明
* 使用说明:
****************************************************************************/
#ifdef _MODBUS_BUFFER
/*初始化FIFO*/
void Modbus_FIFOInit(struct tty_FIFOStruct *pBuf);

/*给FIFO中放入一个(包)数据,成功返回0*/
u8 Modbus_FIFOPutOne(struct tty_FIFOStruct *pBuf, u8 *q);

/*从FIFO中取出一个(包)数据,成功返回0*/
u8 Modbus_FIFOGetOne(struct tty_FIFOStruct *pBuf, u8 *q);

/*偷看一眼FIFO中现在有几个数据(包)*/
u8 Modbus_FIFOPeek(struct tty_FIFOStruct *pBuf);
#endif

/* 从中断中接收数据 */
void Modbus_RcvDataInt(void);

/* 在主循环中发送数据 */
void Modbus_SendDataLoop(void);

/* 串口故障检测 */
void Modbus_ErrCheck(void);

/* 计算CRC */
void Modbus_CRCCal(u8* dat,  u8 len, u8* crc);

#endif

