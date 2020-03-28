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

/*----------------------------------�ӿں������� ----------------------------------*/

/****************************************************************************
* ��������: ������FIFO����,��������һ����ֵ,ȡ��һ����ֵ,͵��һ�������ֵ�ĸ���
* ��������: ����0��ʾ�ɹ�
* ��ڲ���:
* ���ڲ���:
* ʹ��˵��:
****************************************************************************/
void Key_FIFOPutOne(u8 keycode)
{
    keyBuf.buffer[keyBuf.wr] = keycode;            // ��������,����ʹwrָ��ǰ��
    keyBuf.wr++;
    if(keyBuf.wr > KEY_BUFFER_SIZE - 1){
        keyBuf.wr = 0;
    }

    if(keyBuf.num < KEY_BUFFER_SIZE){
        keyBuf.num++;                       // ���buffer֮ǰΪδ��,��buffer���ݸ���+1
    }
    else{                                   // ���buffer֮ǰΪ����,�������ϵ�һ����ֵ,buffer���ݸ�������
        keyBuf.rd = keyBuf.wr;              // ͬʱ��ָ��ǰ��,��Ϊ��һ�����ݲ��ö���,�������
    }
}

u8 Key_FIFOGetOne(void)
{
    u8 keycode;
    if(keyBuf.num == 0){
        return (-1);                          // ���û������,����-1
    }
    keycode = keyBuf.buffer[keyBuf.rd];     // ���������,����һ����ֵ
    keyBuf.num--;
    keyBuf.rd++;                            // ����ָ��
    if(keyBuf.rd > KEY_BUFFER_SIZE - 1){
        keyBuf.rd = 0;
    }
    return keycode;
}

u8 Key_FIFOPeek(void)
{
    return keyBuf.num;                      // ͵��һ��buffer���������м�������
}

void Key_FIFOFlush(void)
{
    keyBuf.wr  = 0;
    keyBuf.rd  = 0;
    keyBuf.num = 0;
}


/*******************************************************************************
 * �������ƣ�KeyInit()
 * ��������: ���̳�ʼ��,������ʼ��IO�����ݽṹ
 * ��ڲ�������
 * ���ڲ�������
 * ʹ��˵�������ݽṹ��ʼ��ʱ�ͳ�ʼ����,����Ͳ��ó�ʼ����
 ******************************************************************************/
void KeyInit(void)
{
	//�ô������Ӳ����ʼ����Ӧ�˿�
    //ADCON1 = 0x0f;                          // ��ֹADת������
    //TRISA |= 0x1f;                          // ������Ӧ����Ϊ����
}

/*******************************************************************************
 * �������ƣ�KeyScan()
 * ��������: ���̰�������
 * ��ڲ�������
 * ���ڲ�������
 * ʹ��˵������ʵʱ�ں������10msϵͳ��ʱ,ǰ��̨��������10ms�ж��е���
 ******************************************************************************/
void KeyScan(void)
{
//    key.cur  = (PORTA) & 0x1f;

    // ״̬����ʼ
    switch(key.state){
        case KEY_STATE_SHORT:               // �ȴ���������
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
                // ����ֵ: �̰� ����
                Key_FIFOPutOne(key.old + KEY_OFFSET_SHORT);
            }
            break;

        case KEY_STATE_LONG:                // �ȴ���������
            if(key.old == key.cur){
                if(key.tmr < KEY_TMR_RPT_START){
                    key.tmr++;
                }
                else{
                    key.tmr   = 0;
                    key.state = KEY_STATE_RPT;
                    // ����ֵ: ���� ����
                    //Key_FIFOPutOne(key.old + KEY_OFFSET_RPT);
                }
            }
            else{
                key.tmr   = 0;
                key.state = KEY_STATE_SHORT;
                // ����ֵ: �̰� ̧��
                if(key.cur == 0){
                    Key_FIFOPutOne(key.old + KEY_OFFSET_SHORTUP);
                }
            }
            break;

        case KEY_STATE_RPT:                 // ���������ظ��׶�
            if(key.old == key.cur){
                if(key.tmr < KEY_TMR_RPT_DLY){
                    key.tmr++;
                }
                else{
                    key.tmr = 0;
                    // ����ֵ: ���� �ظ�
                    //Key_FIFOPutOne(key.old + KEY_OFFSET_RPT);
                }
            }
            else{
                key.tmr   = 0;
                key.state = KEY_STATE_SHORT;
                // ����ֵ: ���� ̧��
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


