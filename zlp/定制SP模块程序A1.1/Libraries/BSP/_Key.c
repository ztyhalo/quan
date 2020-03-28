#include "stm32f10x.h"
#include "_Key.h"

struct Key_FIFOStruct keyBuf = {            // keyboard buffer
//  wr rd  num
    0, 0, 0
};

struct Key_ScanStruct key = {               // keyboard variables
// cur old tmr state
    0, 0, 0, KEY_STATE_SHORT
};

/*----------------------------------接口函数定义 ----------------------------------*/

/****************************************************************************
* 函数名称: 按键的FIFO操作,包括放入一个键值,取出一个键值,偷看一眼里面键值的个数
* 功能描述: 返回0表示成功
* 入口参数:
* 出口参数:
* 使用说明:
****************************************************************************/
void Key_FIFOPutOne(u8 keycode)
{
    keyBuf.buffer[keyBuf.wr] = keycode;            // 放入数据,并且使wr指针前移
    keyBuf.wr++;
    if(keyBuf.wr > KEY_BUFFER_SIZE - 1){
        keyBuf.wr = 0;
    }

    if(keyBuf.num < KEY_BUFFER_SIZE){
        keyBuf.num++;                       // 如果buffer之前为未满,则buffer数据个数+1
    }
    else{                                   // 如果buffer之前为已满,则冲掉最老的一个键值,buffer数据个数不变
        keyBuf.rd = keyBuf.wr;              // 同时读指针前移,因为有一个数据不用读了,被冲掉了
    }
}

u8 Key_FIFOGetOne(void)
{
    u8 keycode;
    if(keyBuf.num == 0){
        return (-1);                          // 如果没有数据,返回-1
    }
    keycode = keyBuf.buffer[keyBuf.rd];     // 如果有数据,返回一个键值
    keyBuf.num--;
    keyBuf.rd++;                            // 调整指针
    if(keyBuf.rd > KEY_BUFFER_SIZE - 1){
        keyBuf.rd = 0;
    }
    return keycode;
}

u8 Key_FIFOPeek(void)
{
    return keyBuf.num;                      // 偷看一眼buffer里面现在有几个数据
}

void Key_FIFOFlush(void)
{
    keyBuf.wr  = 0;
    keyBuf.rd  = 0;
    keyBuf.num = 0;
}


/*******************************************************************************
 * 函数名称：KeyInit()
 * 功能描述: 键盘初始化,包括初始化IO和数据结构
 * 入口参数：无
 * 出口参数：无
 * 使用说明：数据结构初始化时就初始化了,这里就不用初始化了
 ******************************************************************************/
void KeyInit(void)
{
	//该处需根据硬件初始化对应端口
    //ADCON1 = 0x0f;                          // 禁止AD转换功能
    //TRISA |= 0x1f;                          // 配置相应引脚为输入
}

/*******************************************************************************
 * 函数名称：KeyScan()
 * 功能描述: 键盘按键处理
 * 入口参数：无
 * 出口参数：无
 * 使用说明：有实时内核则加上10ms系统延时,前后台程序则在10ms中断中调用
 ******************************************************************************/
void KeyScan(void)
{
//    key.cur  = (PORTA) & 0x1f;

    // 状态机开始
    switch(key.state){
        case KEY_STATE_SHORT:               // 等待按键按下
            if((key.cur == 0x00) || (key.cur != key.old)){
                key.old = key.cur;
                key.tmr = 0;
            }
            else if(key.tmr < KEY_TMR_DEBOUNCE){
                key.tmr++;
            }
            else{
                key.tmr = 0;
                key.state = KEY_STATE_LONG;
                // 发键值: 短按 按下
                Key_FIFOPutOne(key.old + KEY_OFFSET_SHORT);
            }
            break;

        case KEY_STATE_LONG:                // 等待按键长按
            if(key.old == key.cur){
                if(key.tmr < KEY_TMR_RPT_START){
                    key.tmr++;
                }
                else{
                    key.tmr   = 0;
                    key.state = KEY_STATE_RPT;
                    // 发键值: 长按 按下
                    //Key_FIFOPutOne(key.old + KEY_OFFSET_RPT);
                }
            }
            else{
                key.tmr   = 0;
                key.state = KEY_STATE_SHORT;
                // 发键值: 短按 抬起
                if(key.cur == 0){
                    Key_FIFOPutOne(key.old + KEY_OFFSET_SHORTUP);
                }
            }
            break;

        case KEY_STATE_RPT:                 // 按键长按重复阶段
            if(key.old == key.cur){
                if(key.tmr < KEY_TMR_RPT_DLY){
                    key.tmr++;
                }
                else{
                    key.tmr = 0;
                    // 发键值: 长按 重复
                    //Key_FIFOPutOne(key.old + KEY_OFFSET_RPT);
                }
            }
            else{
                key.tmr   = 0;
                key.state = KEY_STATE_SHORT;
                // 发键值: 长按 抬起
                if(key.cur == 0){
                    Key_FIFOPutOne(key.old + KEY_OFFSET_LONGUP);
                }
            }
            break;

        default:
            key.state = KEY_STATE_SHORT;
            break;
    }
}


