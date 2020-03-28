#include "stm32f10x.h"
#include "_Modbus.h"


//RS-485时用于DE控制
#define DE   GPIO_Pin_0           



/* 因为MODBUS为一问一答式,一般不用设置buffer */
#ifdef _MODBUS_BUFFER
struct tty_FIFOStruct ttyRxBuf =
{
    0, 0, 0
};

struct tty_FIFOStruct ttyTxBuf =
{
    0, 0, 0
};
#endif

// MODBUS数据结构,包括接收,发送,辅助buffer
struct tty_struct tty =
{
    0
};


//用于存放计算后的CRC值
u8 CRCbuf[2] = {0}; 
/****************************************************************************
* 函数名称: Timer3Init_Base()
* 功能描述: 提供帧结束定时
* 入口参数: 无
* 出口参数: 无
* 使用说明: 需要实现本函数定时功能，以提供MODBUS帧结束判断
****************************************************************************/
void Timer3Init_Base(void)
{


}
/****************************************************************************
* 函数名称: Modubs_FIFOInit()
* 功能描述: 初始化FIFO
* 入口参数: 无
* 出口参数: 无
* 使用说明: 该函数没有实现,需要在定义数据结构时 顺便 初始化了
****************************************************************************/
#ifdef _MODBUS_BUFFER
void Modbus_FIFOInit(struct tty_FIFOStruct *pBuf)
{
}
#endif

/****************************************************************************
* 函数名称: FIFOGetOne()
* 功能描述: 给FIFO中放入一个(包)数据,成功返回0
* 入口参数: FIFO结构地址,待放入数据的地址
* 出口参数: 成功返回0,不成功返回1:比如FIFO满了
* 使用说明:
****************************************************************************/
#ifdef _MODBUS_BUFFER
u8 Modbus_FIFOPutOne(struct tty_FIFOStruct *pBuf, u8 *q)
{
    if(pBuf->num < MODBUS_BUFFER_SIZE){
        memcpy((s8*)&pBuf->Buffer[pBuf->wr], (cs8*)q, sizeof(struct tty_queue));
        pBuf->num++;
        pBuf->wr++;
        if(pBuf->wr > MODBUS_BUFFER_SIZE){
            pBuf->wr = 0;
        }
        return 0;
    }
    return -1;
}
#endif

/****************************************************************************
* 函数名称: FIFOPutOne()
* 功能描述: 从FIFO中取出一个(包)数据,成功返回0
* 入口参数: FIFO结构地址,取出数据要放到的地址
* 出口参数: 成功返回0,不成功返回1:比如FIFO空了
* 使用说明:
****************************************************************************/
#ifdef _MODBUS_BUFFER
u8 Modbus_FIFOGetOne(struct tty_FIFOStruct *pBuf, u8 *q)
{
    if(pBuf->num > 0){
        memcpy((s8*)q, (cs8*)&pBuf->Buffer[pBuf->rd], sizeof(struct tty_queue));
        pBuf->num--;
        pBuf->rd++;
        if(pBuf->rd > MODBUS_BUFFER_SIZE){
            pBuf->rd = 0;
        }
        return 0;
    }
    return -1;
}
#endif


/****************************************************************************
* 函数名称: FIFOPeek
* 功能描述: 偷看一眼FIFO中现在有几个数据(包)
* 入口参数: 无
* 出口参数: FIFO中实体的个数
* 使用说明:
****************************************************************************/
#ifdef _MODBUS_BUFFER
u8 Modbus_FIFOPeek(struct tty_FIFOStruct *pBuf)
{
    return pBuf->num;
}
#endif


/****************************************************************************
* 函数名称: ttyRcvDataInt()
* 功能描述: 接收数据并放入MODBUS数据结构中
* 入口参数: 无
* 出口参数: 无
* 使用说明: 在中断中直接调用,注意这里需要用到一路定时器
****************************************************************************/
void Modbus_RcvDataInt(void)
{
    u8 tmp;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){           // 串口1
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
        tmp = USART_ReceiveData(USART1);;
        Timer3Init_Base();                                             // 需要重定义
        if((tty.read_q.valid == 0) && (tty.read_q.head < MODBUS_BUFFER_SIZE)){
            tty.read_q.buf[tty.read_q.head ++] = tmp;
        }
    }
}

/****************************************************************************
* 函数名称: Modbus_SendDataLoop()
* 功能描述: 将MODBUS待发送数据进行发送
* 入口参数: 无
* 出口参数: 无
* 使用说明: 在主循环中直接调用,也可在中断中直接调用
****************************************************************************/
void Modbus_SendDataLoop(void)
{
    if(tty.write_q.valid){
        if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {// 如果串口发送寄存器非空,返回
            if (tty.write_q.tail < tty.write_q.head) {
				GPIO_SetBits(GPIOA,DE);						   // 正常发送
               
				USART_SendData(USART1, tty.write_q.buf[tty.write_q.tail++]);
			    USART_ClearFlag(USART1,USART_FLAG_TC);
	            USART_ClearFlag(USART1,USART_FLAG_TXE);
            }
            else if (USART_GetITStatus(USART1,USART_IT_TC)!= RESET) {// 确保最后一个数全部发出去了,再禁止485发送
				GPIO_ResetBits(GPIOA,DE);
                tty.write_q.valid  = 0;
                tty.write_q.head   = 0;
                tty.write_q.tail   = 0;
            }
        }
    }
}

/****************************************************************************
* 函数名称: UartErrCheck()
* 功能描述: 检查串口是否死掉
* 入口参数: 无
* 出口参数: 无
* 使用说明: 在需要时调用
****************************************************************************/
//void Modbus_ErrCheck(void)
//{
//    u8 tmp;
//    //接收中如果出现错误  废掉数据 复位串口
//    if((RCSTAbits.OERR==1)||(RCSTAbits.FERR==1)){
//        tmp = RCREG;
//        RCSTAbits.SPEN=0;
//        RCSTAbits.CREN=0;
//
//        RCSTAbits.SPEN=1;
//        RCSTAbits.CREN=1;
//        if(tty.read_q.valid == 0){
//            tty.read_q.head = 0;
//        }
//    }
//}

/****************************************************************************
* 函数名称：CRCCal ()
* 功能描述: 生成CRC校验码
* 入口参数：数据起始地址,数据长度,生成的CRC存放地址
* 出口参数：无
* 使用说明: 无
****************************************************************************/
void Modbus_CRCCal(u8 * dat,  u8 len, u8 *crc)
{
    u8 i, j, tmp;
    u16 crcVal = 0xffff;
    for(i = 0; i < len; i ++){
        crcVal ^= dat[i];
        for(j = 0; j < 8; j ++){
            tmp = crcVal & 0x01;
            crcVal >>= 1;
            if(tmp){
                crcVal ^= 0xa001;
            }
        }
    }
    crc[0] = (u8)(crcVal &  0xff);          //little-endian
    crc[1] = (u8)(crcVal >> 0x08);
}


