/**
  ******************************************************************************
  * @file    sd_bsp.h
  * @author  ����
  * @version V1.0
  * @date    2013-7-25
  * @brief   �ڸ�ͷ�ļ��н������й�SD����io������
  ******************************************************************************
  * @copy
  * <h2><center>&copy; COPYRIGHT ������������޹�˾ ����һ��</center></h2>
  */
/*
 * ��ֹ��ΰ�������
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

//����--------------------------------------------------------------------------
__inline uint8_t read_SDWP_Check()              // SD���������
{
	return  GPIO_ReadInputDataBit(SD_WP_GPIO);
}

__inline uint8_t read_SDCD_Check()              // SD��������
{
	return  GPIO_ReadInputDataBit(SD_CD_GPIO);
}

__inline uint8_t read_SYS_WKUP_Check()        // ¼��������
{
	return  GPIO_ReadInputDataBit(SYS_WKUP);
}

__inline uint8_t read_ARM_BOOT_Check()        // USB������
{
	return  GPIO_ReadInputDataBit(ARM_BOOT_SW);
}

__inline uint8_t read_MumBoard_Check()        // ĸ����
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
