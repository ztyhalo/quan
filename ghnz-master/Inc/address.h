#ifndef __ADDRESS_H
#define __ADDRESS_H

#include "main.h"
#include "flash.h"
#include "usart.h"

typedef struct
{
	uint8_t  DWT_Mode;      	//ģ�鹤��ģʽ��0--��ǩ������--��վ
	uint32_t ChipID;					//32λоƬID
	uint16_t ShortAddr;				//16λ�����ַ
	uint8_t  ShortAddrFlag;		//16λID��ʶλ��0--δ����16λID��1--�ѷ���16λID
}DWT_AddrStatus_s;

typedef struct
{
	uint8_t DWT_Mode;
	uint32_t ChipID;
}DWT_AddrSta_s;

void Tag_AskFor16BitAddr(void);
void Tag_EnsureFor16BitAddr(void);
void CLE_AskFor16BitAddr(uint8_t *pdata);
void CLE_EnsureFor16BitAddr(uint8_t *pdata);
uint16_t GetFreeAddr(void);




#endif
