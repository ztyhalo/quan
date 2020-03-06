#include "crc.h"



/*******************************************************************************************
* 函数名称：GetCRC(uint8_t *pdata, uint32_t start, uint32_t length)
* 功能描述：计算数据帧的CRC值
* 入口参数：*pdata--数据帧，start--CRC开始计算位置，length--CRC计算长度
* 出口参数：CRC值
* 使用说明：无
********************************************************************************************/
uint16_t GetCRC(uint8_t *pdata, uint32_t start, uint32_t length)
{
	uint8_t Crc16_low, Crc16_High, c16_Low, c16_High;
	uint16_t Crc;
	
	Crc16_low = 0xFF;
	Crc16_High = 0xFF;
	
	for(uint32_t i = start; i < start + length; i++)
	{
		Crc16_low ^= pdata[i];		//将准备发送的数据与Crc16_low进行异或
		
		for(uint8_t j = 0; j < 8; j++)
		{
			c16_Low = Crc16_low;
			c16_High = Crc16_High;
			
			Crc16_low = Crc16_low >> 1;			//Crc16_low和Crc16_High均右移一位
			Crc16_High = Crc16_High >> 1;
			
			if(c16_High & 0x01)
			{
				Crc16_low |= 0x80;	//如果CRC高字节LSB为1，则CRC低字节MSB置位
			}
			
			if(c16_Low & 0x01)
			{
				Crc16_low ^= 0x01;	//如果CRC低字节LSB为1，CRC低字节LSB取反，CRC高字节Bit7和Bit5取反
				Crc16_High ^= 0xA0;
			}
		}
	}
	Crc = Crc16_High;
	Crc = (Crc << 8) + Crc16_low;
	
	return Crc;
}


/*******************************************************************************************
* 函数名称：CheckCRC(uint16_t crc, uint8_t *pdata, uint32_t start, uint32_t length)
* 功能描述：CRC校验
* 入口参数：crc--接收到的CRC，*pdata--数据帧，start--CRC开始计算位置，length--CRC计算长度
* 出口参数：校验结果：1--校验通过，0--校验未通过
* 使用说明：无
********************************************************************************************/
uint8_t CheckCRC(uint16_t crc, uint8_t *pdata, uint32_t start, uint32_t length)
{
	uint16_t CRC;
	
	CRC = GetCRC(pdata, start, length);
	
	return CRC == crc ? 1: 0;
}


