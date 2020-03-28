#include <string.h>
#include <stdio.h>
#include "diskio.h"
//#include "stm32_eval_spi_sd.h"
#include "sdio_sdcard.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_crc.h"
//#include "RealTime.h"
/*簇大小定义*/
#define SECTOR_SIZE 512U

uint8_t TestBuffer[BLOCK_SIZE];

void disk_Error(void)
{
   SD_Init();
}
/********************************************************************************************
* 函数名称：
* 功能描述：磁盘初始化
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
DSTATUS disk_initialize(BYTE drv)
{
	SD_Error Status;
	SD_NVIC_Configuration();
	Status = SD_Init();         // SD卡初始化
	if(Status==SD_OK)
	{return RES_OK;}
	else
  {return RES_ERROR;}
//	return 0;
}
/********************************************************************************************
* 函数名称：
* 功能描述：磁盘获取状态
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
DSTATUS disk_status(BYTE drv)
{
	return 0;
}
/********************************************************************************************
* 函数名称：
* 功能描述：磁盘读取簇
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
DRESULT disk_read(BYTE drv,BYTE *buff,DWORD sector,BYTE count)
{
	SD_Error Status;
	uint8_t  reTryTime = 3;				 // 重读次数

	while (reTryTime)
	{
		if(count == 1)
		{
			if(SD_ReadBlock((uint8_t*)buff, sector, SECTOR_SIZE) != SD_OK)
			{
				SD_Init();
				reTryTime--;
		    	continue;
			}
		}
		else
		{
			if(SD_ReadMultiBlocks((uint8_t*)buff, sector, SECTOR_SIZE, count) != SD_OK)
			{
				SD_Init();
				reTryTime--;
			    continue;
			}
		}

		Status = SD_WaitReadOperation();	     //等待dma传输结束
		if (Status != SD_OK)
		{
	    	printf("Need to inital SD again!\r\n");
			SD_Init();

			reTryTime--;
			continue;
		}

    	Status = SD_WaitTransferOK();

		if (Status == SD_OK)	    // 如果未超时
		{
		    break;
		}
		else
		{
		    reTryTime--;
		}
	}

	if (Status == SD_OK)
		return RES_OK;
	else
	    return RES_ERROR;
}
/********************************************************************************************
* 函数名称：
* 功能描述：磁盘写入簇
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
#if _READONLY == 0
DRESULT disk_write (BYTE drv,const BYTE *buff,DWORD sector,BYTE count)
{
	SD_Error Status;
	uint8_t  reTryTime = 3;				             // 重写次数
	//__IO uint32_t CRCValue1 = 0, CRCValue2 = 0;		 // 用于存放产生的CRC校验值

	while (reTryTime)
	{
		if(count == 1)
		{
			if(SD_WriteBlock((uint8_t*)buff, sector, SECTOR_SIZE) != SD_OK)
			{
				SD_Init();
				reTryTime--;
			    continue;
			}
		}
		else
		{
			if(SD_WriteMultiBlocks((uint8_t*)buff, sector, SECTOR_SIZE, count) != SD_OK)
			{
				SD_Init();
				reTryTime--;
			    continue;
			}
		}

		Status = SD_WaitWriteOperation();	     //等待dma传输结束
		if (Status != SD_OK)
		{
	    	printf("Need to inital SD again!\r\n");
			SD_Init();

			reTryTime--;
			continue;
		}

        Status = SD_WaitTransferOK();

		if (Status == SD_OK)	    // 如果未超时
		{
		    //if (sector < 10)
//			CRCValue1 = CRC_CalcBlockCRC((uint32_t *)buff, SECTOR_SIZE>>2);
//			CRC_ResetDR();
//			disk_read(0, TestBuffer, sector, 1);
//			CRCValue2 = CRC_CalcBlockCRC((uint32_t *)TestBuffer, SECTOR_SIZE>>2);
//			CRC_ResetDR();
//			
//			if (CRCValue1 != CRCValue2)
//			{
//				printf("CRC check error!\r\n");
//				reTryTime--;
//				if ((sector<3)&&(reTryTime==0))
//				{
//					disk_fatRecover(0);
//					return RES_ERROR;
//				}
//				continue;
//			}
			break;
		}
		else
		{
		    reTryTime--;
		}
	}

	if (Status == SD_OK)
		return RES_OK;
	else
	    return RES_ERROR;
}

/********************************************************************************************
* 函数名称：
* 功能描述：磁盘分区表恢复
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
DSTATUS disk_fatRecover(BYTE drv)
{
    SD_Error errorstatus = SD_OK;

	SD_ReadBlock(TestBuffer, 6, BLOCK_SIZE);
    SD_WaitReadOperation();	                 // 等待dma传输结束
    errorstatus = SD_WaitTransferOK();       // 等待sdio到sd卡传输结束

    if (errorstatus != SD_OK)
    {
        return RES_ERROR;
    }

    SD_WriteBlock(TestBuffer, 0, BLOCK_SIZE);
	SD_WaitWriteOperation();                 // 等待dma传输结束
    errorstatus = SD_WaitTransferOK();       // 等待sdio到sd卡传输结束

    if (errorstatus != SD_OK)
    {
        return RES_ERROR;
    }

    SD_ReadBlock(TestBuffer, 7, BLOCK_SIZE);
	SD_WaitReadOperation();	                 // 等待dma传输结束
    errorstatus = SD_WaitTransferOK();       // 等待sdio到sd卡传输结束

    if (errorstatus != SD_OK)
    {
        return RES_ERROR;
    }

    SD_WriteBlock(TestBuffer, 1, BLOCK_SIZE);
    SD_WaitWriteOperation();                 // 等待dma传输结束
    errorstatus = SD_WaitTransferOK();       // 等待sdio到sd卡传输结束

    if (errorstatus != SD_OK)
    {
        return RES_ERROR;
    }

    SD_ReadBlock(TestBuffer, 8, BLOCK_SIZE);
	SD_WaitReadOperation();	                 // 等待dma传输结束
    errorstatus = SD_WaitTransferOK();       // 等待sdio到sd卡传输结束

    if (errorstatus != SD_OK)
    {
        return RES_ERROR;
    }

    SD_WriteBlock(TestBuffer, 2, BLOCK_SIZE);
    SD_WaitWriteOperation();                 // 等待dma传输结束
    errorstatus = SD_WaitTransferOK();       // 等待sdio到sd卡传输结束

    if (errorstatus != SD_OK)
    {
        return RES_ERROR;
    }

    return RES_OK;
}

#endif
/********************************************************************************************
* 函数名称：
* 功能描述：磁盘控制
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
DRESULT disk_ioctl (BYTE drv,BYTE ctrl,void *buff)
{
	return RES_OK;
}
/********************************************************************************************
* 函数名称：
* 功能描述：获取系统时间
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
//DWORD get_fattime(void)
//{
//	DWORD RetVal = 0;
//    DWORD Temp = 0;
//	RTCTimeDateData CurrentTimeDate;

//	RTCReadTimeData(&CurrentTimeDate);   // 读取当前时间日期

//    Temp = CurrentTimeDate.Date.Years - 1980;
//    RetVal = Temp << 25;                 // bit31:25 年 （0~127）（从1980 开始）
//    Temp = CurrentTimeDate.Date.Months;
//    RetVal |= (Temp << 21);              // bit24:21 月 （1~12）
//    Temp = CurrentTimeDate.Date.Days;
//    RetVal |= (Temp << 16);              // bit20:16 日 （1~31）

//    Temp = CurrentTimeDate.Time.Hours;
//    RetVal |= (Temp << 11);              // bit15:11 时（0~23）
//    Temp = CurrentTimeDate.Time.Minutes;
//    RetVal |= (Temp <<  5);              // bit10:5 分钟（0~59）
//    Temp = CurrentTimeDate.Time.Seconds;
//    RetVal |= Temp;                      // bit4:0  秒 （0~59）

//	return RetVal;
//}

/******************* (C) COPYRIGHT 2009 www.armjishu.com *****END OF FILE****/
