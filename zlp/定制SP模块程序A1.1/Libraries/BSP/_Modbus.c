#include "stm32f10x.h"
#include "_Modbus.h"


//RS-485ʱ����DE����
#define DE   GPIO_Pin_0           



/* ��ΪMODBUSΪһ��һ��ʽ,һ�㲻������buffer */
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

// MODBUS���ݽṹ,��������,����,����buffer
struct tty_struct tty =
{
    0
};


//���ڴ�ż�����CRCֵ
u8 CRCbuf[2] = {0}; 
/****************************************************************************
* ��������: Timer3Init_Base()
* ��������: �ṩ֡������ʱ
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: ��Ҫʵ�ֱ�������ʱ���ܣ����ṩMODBUS֡�����ж�
****************************************************************************/
void Timer3Init_Base(void)
{


}
/****************************************************************************
* ��������: Modubs_FIFOInit()
* ��������: ��ʼ��FIFO
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: �ú���û��ʵ��,��Ҫ�ڶ������ݽṹʱ ˳�� ��ʼ����
****************************************************************************/
#ifdef _MODBUS_BUFFER
void Modbus_FIFOInit(struct tty_FIFOStruct *pBuf)
{
}
#endif

/****************************************************************************
* ��������: FIFOGetOne()
* ��������: ��FIFO�з���һ��(��)����,�ɹ�����0
* ��ڲ���: FIFO�ṹ��ַ,���������ݵĵ�ַ
* ���ڲ���: �ɹ�����0,���ɹ�����1:����FIFO����
* ʹ��˵��:
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
* ��������: FIFOPutOne()
* ��������: ��FIFO��ȡ��һ��(��)����,�ɹ�����0
* ��ڲ���: FIFO�ṹ��ַ,ȡ������Ҫ�ŵ��ĵ�ַ
* ���ڲ���: �ɹ�����0,���ɹ�����1:����FIFO����
* ʹ��˵��:
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
* ��������: FIFOPeek
* ��������: ͵��һ��FIFO�������м�������(��)
* ��ڲ���: ��
* ���ڲ���: FIFO��ʵ��ĸ���
* ʹ��˵��:
****************************************************************************/
#ifdef _MODBUS_BUFFER
u8 Modbus_FIFOPeek(struct tty_FIFOStruct *pBuf)
{
    return pBuf->num;
}
#endif


/****************************************************************************
* ��������: ttyRcvDataInt()
* ��������: �������ݲ�����MODBUS���ݽṹ��
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: ���ж���ֱ�ӵ���,ע��������Ҫ�õ�һ·��ʱ��
****************************************************************************/
void Modbus_RcvDataInt(void)
{
    u8 tmp;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){           // ����1
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
        tmp = USART_ReceiveData(USART1);;
        Timer3Init_Base();                                             // ��Ҫ�ض���
        if((tty.read_q.valid == 0) && (tty.read_q.head < MODBUS_BUFFER_SIZE)){
            tty.read_q.buf[tty.read_q.head ++] = tmp;
        }
    }
}

/****************************************************************************
* ��������: Modbus_SendDataLoop()
* ��������: ��MODBUS���������ݽ��з���
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: ����ѭ����ֱ�ӵ���,Ҳ�����ж���ֱ�ӵ���
****************************************************************************/
void Modbus_SendDataLoop(void)
{
    if(tty.write_q.valid){
        if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {// ������ڷ��ͼĴ����ǿ�,����
            if (tty.write_q.tail < tty.write_q.head) {
				GPIO_SetBits(GPIOA,DE);						   // ��������
               
				USART_SendData(USART1, tty.write_q.buf[tty.write_q.tail++]);
			    USART_ClearFlag(USART1,USART_FLAG_TC);
	            USART_ClearFlag(USART1,USART_FLAG_TXE);
            }
            else if (USART_GetITStatus(USART1,USART_IT_TC)!= RESET) {// ȷ�����һ����ȫ������ȥ��,�ٽ�ֹ485����
				GPIO_ResetBits(GPIOA,DE);
                tty.write_q.valid  = 0;
                tty.write_q.head   = 0;
                tty.write_q.tail   = 0;
            }
        }
    }
}

/****************************************************************************
* ��������: UartErrCheck()
* ��������: ��鴮���Ƿ�����
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: ����Ҫʱ����
****************************************************************************/
//void Modbus_ErrCheck(void)
//{
//    u8 tmp;
//    //������������ִ���  �ϵ����� ��λ����
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
* �������ƣ�CRCCal ()
* ��������: ����CRCУ����
* ��ڲ�����������ʼ��ַ,���ݳ���,���ɵ�CRC��ŵ�ַ
* ���ڲ�������
* ʹ��˵��: ��
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


