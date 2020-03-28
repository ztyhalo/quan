#ifndef __KEYB_H__
#define __KEYB_H__

//------------------------------------- �������� --------------------------------------------
#define KEY_NUM                     1       // max number of keyboard
#define KEY_BUFFER_SIZE             10      // key fifo buffer

//---------------------------------- ����ɨ��״̬�� -----------------------------------------
#define KEY_STATE_SHORT             1       // �ȴ��̰���,��Ҫȥ��
#define KEY_STATE_LONG              2       // �ȴ�����
#define KEY_STATE_RPT               3       // �������ظ�����ֵ

#define KEY_EN_SHORT                1       // ����ʹ��:�̰�
#define KEY_EN_LONG                 0       // ����ʹ��:����
#define KEY_EN_RPT                  0       // ����ʹ��:�ظ�
#define KEY_EN_SHORTUP              1       // ����ʹ��:�̰����̧��
#define KEY_EN_LONGUP               1       // ����ʹ��:�������̧��

#define KEY_TMR_DEBOUNCE            4       // ȥ��ʱ��40ms
#define KEY_TMR_RPT_START           80      // ����ʱ��800ms
#define KEY_TMR_RPT_DLY             20      // �����ظ�200ms

#define KEY_OFFSET_UP               0x80    // if key1=1 key1-up=0x81
#define KEY_OFFSET_LONGUP           0x80    // if key1=1 key1-up=0x81
#define KEY_OFFSET_SHORTUP          0x80    // if key1=1 key1-up=0x81
#define KEY_OFFSET_RPT              0x20    // if key1=1 key1-repeat=0x41
#define KEY_OFFSET_SHORT            0x00    // if key1=1 key1-repeat=0x41
#define KEY_OFFSET_SHIFT            0x00    // if key1=1 shift1+key1=0x21

//------------------------------------- �������� --------------------------------------------
struct Key_FIFOStruct{
    u8 wr;                                  // read index
    u8 rd;                                  // write index
    u8 num;                                 // number of data
    u8 buffer[KEY_BUFFER_SIZE];             // data buffer
};

//---------------------------------- ����ɨ�����ݽṹ ---------------------------------------
struct Key_ScanStruct
{
    u8 cur;                                 // current value
    u8 old;                                 // old value
    u8 tmr;                                 // key down timer
    u8 state;                               // fsm state
    //u8 code;                                // keycode
};

#endif

//------------------------------------- �������� --------------------------------------------
void Key_FIFOPutOne(u8 keycode);
u8 Key_FIFOGetOne(void);
u8 Key_FIFOPeek(void);
void Key_FIFOFlush(void);
void KeyInit(void);
void KeyScan(void);

//--------------------------------------- �� �� ---------------------------------------------

