#ifndef __MODBUS_H__
#define __MODBUS_H__

//------------------------- config -------------------------------
//#define _MODBUS_BUFFER
#define MODBUS_BUFFER_SIZE  20
#define MODBUS_BUFFER_DEPTH 5

/****************************************************************************
 * ��������: Modbus���Ͷ���
 * ʹ��˵��:
 ****************************************************************************/

struct tty_queue {
    u8 valid;                              // ��������һ֡����
    u8 head;                               // ͷָ��
    u8 tail;                               // βָ��
    u8 buf[MODBUS_BUFFER_SIZE];                  // ������
};

struct tty_struct {
    u8 stopped;                            // ֹͣ��־
    struct tty_queue read_q;               // �յ������ݻ�����
    struct tty_queue write_q;              // �����͵����ݻ�����
    struct tty_queue secondary;            // ����������
};

struct tty_FIFOStruct{
    u8 wr;                                  // ��ָ��
    u8 rd;                                  // дָ��
    u8 num;                                 // ��ǰFIFO������(��)����
    struct tty_queue Buffer[MODBUS_BUFFER_SIZE];    // FIFO�洢��,������ݵĻ�����
};


/****************************************************************************
* ��������: public��������
* ʹ��˵��:
****************************************************************************/
#ifdef _MODBUS_BUFFER
/*��ʼ��FIFO*/
void Modbus_FIFOInit(struct tty_FIFOStruct *pBuf);

/*��FIFO�з���һ��(��)����,�ɹ�����0*/
u8 Modbus_FIFOPutOne(struct tty_FIFOStruct *pBuf, u8 *q);

/*��FIFO��ȡ��һ��(��)����,�ɹ�����0*/
u8 Modbus_FIFOGetOne(struct tty_FIFOStruct *pBuf, u8 *q);

/*͵��һ��FIFO�������м�������(��)*/
u8 Modbus_FIFOPeek(struct tty_FIFOStruct *pBuf);
#endif

/* ���ж��н������� */
void Modbus_RcvDataInt(void);

/* ����ѭ���з������� */
void Modbus_SendDataLoop(void);

/* ���ڹ��ϼ�� */
void Modbus_ErrCheck(void);

/* ����CRC */
void Modbus_CRCCal(u8* dat,  u8 len, u8* crc);

#endif

