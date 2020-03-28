/**
  ******************************************************************************
  * @file    sd_bsp.h
  * @author  辛鑫
  * @version V1.0
  * @date    2013-7-25
  * @brief   在该头文件中将定义有关SD卡的io操作。
  ******************************************************************************
  * @copy
  * <h2><center>&copy; COPYRIGHT 天津华宁电子有限公司 开发一部</center></h2>
  */
/*
 * 防止多次包含机制
 */
#ifndef __SD_BSP_H__
#define __SD_BSP_H__

#define SD_CD_GPIO      GPIOB, GPIO_Pin_15
#define SD_WP_GPIO      GPIOC,  GPIO_Pin_7
#define SYS_WKUP        GPIOA,  GPIO_Pin_0
#define ARM_BOOT_SW     GPIOA,  GPIO_Pin_1
#define MUM_BOARD		GPIOB,  GPIO_Pin_7
#define BATTERY_POWER   GPIOB,  GPIO_Pin_6

/**
  * @brief  SD Card States
  */
typedef enum
{
  SD_NOMAL                  = ((uint8_t)0x00),
  SD_PROTECT                = ((uint8_t)0x01),
  SD_NOEXIST                = ((uint8_t)0x02)
} SDState;

//输入--------------------------------------------------------------------------
__inline uint8_t read_SDWP_Check()              // SD卡保护检测
{
	return  GPIO_ReadInputDataBit(SD_WP_GPIO);
}

__inline uint8_t read_SDCD_Check()              // SD卡插入检测
{
	return  GPIO_ReadInputDataBit(SD_CD_GPIO);
}

__inline uint8_t read_SYS_WKUP_Check()        // 录音输入检测
{
	return  GPIO_ReadInputDataBit(SYS_WKUP);
}

__inline uint8_t read_ARM_BOOT_Check()        // USB插入检测
{
	return  GPIO_ReadInputDataBit(ARM_BOOT_SW);
}

__inline uint8_t read_MumBoard_Check()        // 母板检测
{
	return  GPIO_ReadInputDataBit(MUM_BOARD);
}

__inline void Battery_ON()
{
	GPIO_ResetBits(BATTERY_POWER);
}

__inline void Battery_OFF()
{
	GPIO_SetBits(BATTERY_POWER);
}

#endif
/****************************************** END OF FILE **************************************/
