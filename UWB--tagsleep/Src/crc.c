#include "crc.h"



/*******************************************************************************************
* �������ƣ�GetCRC(uint8_t *pdata, uint32_t start, uint32_t length)
* ������������������֡��CRCֵ
* ��ڲ�����*pdata--����֡��start--CRC��ʼ����λ�ã�length--CRC���㳤��
* ���ڲ�����CRCֵ
* ʹ��˵������
********************************************************************************************/
uint16_t GetCRC(uint8_t *pdata, uint32_t start, uint32_t length)
{
	uint8_t Crc16_low, Crc16_High, c16_Low, c16_High;
	uint16_t Crc;
	
	Crc16_low = 0xFF;
	Crc16_High = 0xFF;
	
	for(uint32_t i = start; i < start + length; i++)
	{
		Crc16_low ^= pdata[i];		//��׼�����͵�������Crc16_low�������
		
		for(uint8_t j = 0; j < 8; j++)
		{
			c16_Low = Crc16_low;
			c16_High = Crc16_High;
			
			Crc16_low = Crc16_low >> 1;			//Crc16_low��Crc16_High������һλ
			Crc16_High = Crc16_High >> 1;
			
			if(c16_High & 0x01)
			{
				Crc16_low |= 0x80;	//���CRC���ֽ�LSBΪ1����CRC���ֽ�MSB��λ
			}
			
			if(c16_Low & 0x01)
			{
				Crc16_low ^= 0x01;	//���CRC���ֽ�LSBΪ1��CRC���ֽ�LSBȡ����CRC���ֽ�Bit7��Bit5ȡ��
				Crc16_High ^= 0xA0;
			}
		}
	}
	Crc = Crc16_High;
	Crc = (Crc << 8) + Crc16_low;
	
	return Crc;
}


/*******************************************************************************************
* �������ƣ�CheckCRC(uint16_t crc, uint8_t *pdata, uint32_t start, uint32_t length)
* ����������CRCУ��
* ��ڲ�����crc--���յ���CRC��*pdata--����֡��start--CRC��ʼ����λ�ã�length--CRC���㳤��
* ���ڲ�����У������1--У��ͨ����0--У��δͨ��
* ʹ��˵������
********************************************************************************************/
uint8_t CheckCRC(uint16_t crc, uint8_t *pdata, uint32_t start, uint32_t length)
{
	uint16_t CRC;
	
	CRC = GetCRC(pdata, start, length);
	
	return CRC == crc ? 1: 0;
}


