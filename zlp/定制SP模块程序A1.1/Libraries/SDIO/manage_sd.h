/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : manage_flash.h
* Author             : 辛鑫
* Version            : V1.0.1
* Date               : 07/06/2013
* Description        : All functions related to the NAND process
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MANAGE_FLASH_H
#define __MANAGE_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "spi_flash.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define NAND_OK      0
#define NAND_FAIL    1

#define FREE_SECTOR  (1 << 12 )
#define BAD_SECTOR   (1 << 13 )
#define VALID_SECTOR (1 << 14 )
#define USED_SECTOR  (1 << 15 )

#define MAX_PHY_SECTORS_PER_ZONE  511
#define MAX_LOG_SECTORS_PER_ZONE  500     // 留有11个扇区用做为交换扇区

#define SCSI_BLKLEN_VALID      0xffff
/* Private Structures---------------------------------------------------------*/
typedef enum {
  WRITE_IDLE = 0,
  POST_WRITE,
  PRE_WRITE,
  WRITE_CLEANUP,
  WRITE_ONGOING
}WRITE_STATE;

typedef enum {
  OLD_SECTOR = 0,
  UNUSED_SECTOR
}SECTOR_STATE;

/* Private macro --------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------*/
/* exported functions ---------------------------------------------------------*/
uint16_t Flash_Init (void);
uint16_t Flash_Write (uint32_t Memory_Offset, uint8_t *Writebuff, uint16_t Transfer_Length);
uint16_t Flash_Read  (uint32_t Memory_Offset, uint8_t *Readbuff, uint16_t Transfer_Length);
uint16_t Flash_Format (void);

NAND_ADDRESS Flash_PhyAddrToLogAddr (uint32_t Address);
uint32_t Flash_ConvertPhyAddress(NAND_ADDRESS Address);
NAND_ADDRESS Flash_GetAddress(uint32_t total_sector);

#endif
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

