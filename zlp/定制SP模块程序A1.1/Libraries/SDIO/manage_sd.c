/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : manage_flash.c
* Author             : 辛鑫
* Version            : V1.0.1
* Date               : 07/06/2013
* Description        : manage NAND operationx state machine
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
#include "platform_config.h"

/* Includes ------------------------------------------------------------------*/
#include "manage_flash.h"
#include "mass_mal.h"
#include "memory.h"

#ifdef   DEBUG_LCD
	#include "hw_config.h"
    #include "stm3210c_eval_lcd.h"
#endif  /*DEBUG_LCD*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* extern variables-----------------------------------------------------------*/
extern uint32_t SCSI_LBA;
extern uint32_t SCSI_BlkLen;

extern uint16_t PAGES_pre_Block;
/* Private variables ---------------------------------------------------------*/
//uint16_t LUT[512]; //Look Up Table Buffer
WRITE_STATE  Write_State;
NAND_ADDRESS writeAddr, freeAddr;

uint16_t  Initial_Page, CurrentZone = 0;
uint16_t  Written_Block = 0, Written_Pages = 0;
uint32_t  CurrentFreeSector = MAX_LOG_SECTORS_PER_ZONE + 1;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static uint16_t Flash_Sector_Updata(void);
static uint16_t Flash_Copy(NAND_ADDRESS Address_Src, NAND_ADDRESS Address_Dest, uint16_t PageToCopy);
//static uint16_t Flash_BuildLUT(uint8_t ZoneNbr);
static uint16_t Flash_EraseSector (NAND_ADDRESS Address);

/*******************************************************************************
* Function Name  : Flash_Init
* Description    : Init NAND Interface
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
uint16_t Flash_Init(void)
{
  uint16_t Status = NAND_OK;

  SPI_FLASH_Init();               // SPI接口Flash初始化
  Write_State = WRITE_IDLE;

  return Status;
}

/*******************************************************************************
* Function Name  : Flash_GetAddress
* Description    : 将总扇区数转换为块、扇区、页数
* Input          : 扇区数
* Return         : Address_t
*******************************************************************************/
NAND_ADDRESS Flash_GetAddress(uint32_t total_sector)
{
  NAND_ADDRESS Address_t;

  Address_t.Page   = 0;
  Address_t.Sector = total_sector & (PAGE_NUM_PER_SECTOR - 1);
  Address_t.Block  = total_sector / SECTOR_NUM_PER_BLOCK;
  Address_t.Zone   = 0;

  while (Address_t.Block >= BLOCK_NUM_PER_ZONE)    // 超过分区
  {
    Address_t.Block -= BLOCK_NUM_PER_ZONE;
    Address_t.Zone++;                              // 分区数加1
  }
  return Address_t;
}

/*******************************************************************************
* Function Name  : Flash__PhyAddrToLogAddr
* Description    : 将物理地址转换为块、扇区、页数
* Input          : 物理地址
* Return         : Address_t
*******************************************************************************/
NAND_ADDRESS Flash_PhyAddrToLogAddr (uint32_t Address)
{
  NAND_ADDRESS Address_t = {0, 0, 0, 0};

  Address_t.Page = Address / FLASH_PAGE_SIZE;
  while (Address_t.Page >= PAGE_NUM_PER_SECTOR)
  {
      Address_t.Page -= PAGE_NUM_PER_SECTOR;
      Address_t.Sector++;
  }

  while (Address_t.Sector >= SECTOR_NUM_PER_BLOCK)
  {
      Address_t.Sector -= SECTOR_NUM_PER_BLOCK;
      Address_t.Block++;
  }

  while (Address_t.Block >= BLOCK_NUM_PER_ZONE)    // 超过分区块数
  {
      Address_t.Block -= BLOCK_NUM_PER_ZONE;
      Address_t.Zone++;                            // 分区数加1
  }
  return Address_t;
}

/*******************************************************************************
* Function Name  : Flash_ConvertPhyAddress
* Description    : 将逻辑地址转换为物理地址
* Input          : 逻辑地址
* Output         : None
* Return         : Status
*******************************************************************************/
uint32_t Flash_ConvertPhyAddress (NAND_ADDRESS Address)
{
  uint32_t addr, ret;

  addr = CHANGE_PHY_ADDR(Address);            // 进行地址转换，地址从0开始
  ret  = SPI_FLASH_AddrCheck(addr);           // 进行地址越界检测

  if(ret)
    return NAND_FAIL;

  return addr;
}

/*******************************************************************************
* Function Name  : Flash_Format
* Description    : Format the entire NAND flash
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
uint16_t Flash_Format (void)
{
  SPI_FLASH_BulkErase();           // 擦除整个磁盘
  //Flash_BuildLUT(0);               // 建立第一分区

  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_GetFreeSector
* Description    : 获取一块空余的扇区
* Input          : None
* Return         : 扇区记录信息
*******************************************************************************/
static uint16_t Flash_GetFreeSector (void)
{
    uint32_t pCurrentSector;

    pCurrentSector = CurrentFreeSector;

    /* 随机选取，防止一个扇区反复写 */
    if (CurrentFreeSector < MAX_PHY_SECTORS_PER_ZONE)
        CurrentFreeSector++;
    else
        CurrentFreeSector = MAX_LOG_SECTORS_PER_ZONE + 1;

    return pCurrentSector;
}

/*******************************************************************************
* Function Name  : Flash_Sector_Updata
* Description    : 更新写扇区信息表
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static uint16_t Flash_Sector_Updata (void)
{
  uint32_t  Page_Back;

  Page_Back = writeAddr.Page;       // 记录当前页位置
  /* precopy old first pages */
  if (Initial_Page != 0)            // 块中已使用的页数
  {
      freeAddr.Page = writeAddr.Page = 0;
      Flash_Copy (freeAddr, writeAddr, Initial_Page);  // 将原有的页中内容写入writeAddr中

      #ifdef   DEBUG_LCD
        Current_LCD_Line++;
        if (Current_LCD_Line > MAX_LCD_LINE)
        {
            Current_LCD_Line = 0;
            LCD_Clear(LCD_COLOR_GREEN);
        }

        LCD_DisplayStringLine(LINE(Current_LCD_Line), "Copy initial page");
      #endif  /*DEBUG_LCD*/
  }

  /* postcopy remaining pages */
  if ((PAGE_NUM_PER_SECTOR - Page_Back) != 0)
  {
      writeAddr.Page = freeAddr.Page = Page_Back;
      Flash_Copy(freeAddr, writeAddr, PAGE_NUM_PER_SECTOR - Page_Back);  // 将写后剩余页拷入writeAddr中

      #ifdef   DEBUG_LCD
        Current_LCD_Line++;
        if (Current_LCD_Line > MAX_LCD_LINE)
        {
            Current_LCD_Line = 0;
            LCD_Clear(LCD_COLOR_GREEN);
        }

        LCD_DisplayStringLine(LINE(Current_LCD_Line), "Copy remain page");
      #endif  /*DEBUG_LCD*/
  }

  /* assign logical address to new block */
  freeAddr.Page  = 0;
  writeAddr.Page = 0;

  /* erase old sector */
  Flash_EraseSector(freeAddr);                           // 擦除空闲扇区

  writeAddr.Sector++;                                    // 进入下一扇区
  if (writeAddr.Sector >= SECTOR_NUM_PER_BLOCK)
  {

    #ifdef   DEBUG_LCD
      Current_LCD_Line++;
      if (Current_LCD_Line > MAX_LCD_LINE)
      {
          Current_LCD_Line = 0;
          LCD_Clear(LCD_COLOR_GREEN);
      }

      LCD_DisplayStringLine(LINE(Current_LCD_Line), "Entry next block!");
    #endif  /*DEBUG_LCD*/

      writeAddr.Sector = 0;
      writeAddr.Block++;
  }
  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_EraseSector
* Description    : 擦除某个扇区
* Input          : Address为需要擦除的扇区逻辑地址
* Return         : Status
*******************************************************************************/
static uint16_t Flash_EraseSector (NAND_ADDRESS Address)
{
    uint32_t addr;

    Address.Page = 0;                          // 扇区起始位置
    addr = Flash_ConvertPhyAddress(Address);   // 转换为物理地址
    if (addr == 1)
        return NAND_FAIL;

    SPI_FLASH_SectorErase(addr);
    return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_Copy
* Description    : 拷贝多页内容
* Input          : Address_Src为需要复制的源地址，Address_Dest为目标地址，PageToCopy
                   为要复制的页数
* Return         : Status
*******************************************************************************/
static uint16_t Flash_Copy(NAND_ADDRESS Address_Src, NAND_ADDRESS Address_Dest, uint16_t PageToCopy)
{
  uint8_t  Copybuff[FLASH_PAGE_SIZE];                    // 一页为256byte
  uint32_t addr_src, addr_dest;

  for ( ; PageToCopy > 0 ; PageToCopy--)
  {
    addr_src  = Flash_ConvertPhyAddress(Address_Src);
    addr_dest = Flash_ConvertPhyAddress(Address_Dest);
    if ((addr_src == 1)|(addr_dest == 1))
    {

#ifdef   DEBUG_LCD
    Current_LCD_Line++;
    if (Current_LCD_Line > MAX_LCD_LINE)
    {
        Current_LCD_Line = 0;
        LCD_Clear(LCD_COLOR_GREEN);
    }

    LCD_DisplayStringLine(LINE(Current_LCD_Line), "Check addr beyond!");
#endif  /*DEBUG_LCD*/

        return NAND_FAIL;
    }

    SPI_FLASH_BufferRead ((uint8_t *)Copybuff, addr_src, FLASH_PAGE_SIZE);   // 读取原页内容
    SPI_FLASH_PageWrite ((uint8_t *)Copybuff, addr_dest, FLASH_PAGE_SIZE);   // 写入目的页

    SPI_FLASH_AddressIncrement(&Address_Src);                                // 原地址增加一页
    SPI_FLASH_AddressIncrement(&Address_Dest);                               // 目的地址增加一页
  }

  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_Read
* Description    : 读函数
* Input          : Memory_Offset为读取地址，Readbuff为读入缓存，Transfer_Length为读取长度
* Return         : Status
*******************************************************************************/
uint16_t Flash_Read(uint32_t Memory_Offset, uint8_t *Readbuff, uint16_t Transfer_Length)
{
  NAND_ADDRESS logAddress;

  logAddress = Flash_PhyAddrToLogAddr(Memory_Offset);

  if (logAddress.Zone != CurrentZone)                // 如果需要换区
  {
      CurrentZone = logAddress.Zone;
  }

  //sector_num = phyAddress.Block * NAND_BLOCK_SIZE + phyAddress.Sector;
  SPI_FLASH_BufferRead(Readbuff, Memory_Offset, Transfer_Length);   // 读取内容

  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Check_If_SectorEnd
* Description    : 检查是否达到扇区结束函数
* Input          : currentPage为判定的页数
* Return         : -
*******************************************************************************/
void Check_If_SectorEnd(uint32_t currentPage)
{
    if (currentPage == PAGE_NUM_PER_SECTOR) // 如果要写页数达到扇区中最大页数
    {
      #ifdef   DEBUG_LCD
          Current_LCD_Line++;
          if (Current_LCD_Line > MAX_LCD_LINE)
          {
              Current_LCD_Line = 0;
              LCD_Clear(LCD_COLOR_GREEN);
          }

          LCD_DisplayStringLine(LINE(Current_LCD_Line), "Reaches sector end");
      #endif  /*DEBUG_LCD*/

        Flash_Sector_Updata();              // 更新扇区信息
        Write_State = WRITE_IDLE;           // 标记为开始写状态
    }
    else
        Write_State = WRITE_ONGOING;        // 标记为继续写状态
}

/*******************************************************************************
* Function Name  : Check_If_SCSI_BlkLen
* Description    : 检查是否达到块长度函数
* Input          : currentPage为判定的页数
* Return         : -
*******************************************************************************/
void Check_If_SCSI_BlkLen(uint32_t currentPage)
{
    if (Written_Block == SCSI_BlkLen)       // 如果页数达到USB本次传输长度
    {
      #ifdef   DEBUG_LCD
        Current_LCD_Line++;
        if (Current_LCD_Line > MAX_LCD_LINE)
        {
            Current_LCD_Line = 0;
            LCD_Clear(LCD_COLOR_GREEN);
        }

        LCD_DisplayStringLine(LINE(Current_LCD_Line), "Reaches SCSI_BlkLen");
      #endif  /*DEBUG_LCD*/

        Flash_Sector_Updata();              // 更新扇区信息
        Written_Pages = 0;
        Written_Block = 0;
        Write_State = WRITE_IDLE;           // 标记为开始写状态
    }
    else
    {
        Check_If_SectorEnd(currentPage);
    }
}

/*******************************************************************************
* Function Name  : Flash_Write_OnePage
* Description    : 写FAT一页实际是写FLASH两页
* Input          : Writebuff为写入数据，Address为要写逻辑地址
* Return         : Status
*******************************************************************************/
void Flash_Write_OnePage(uint8_t *Writebuff, NAND_ADDRESS *Address)
{
    uint32_t addr_dest;
    /* write the new page */
    addr_dest = Flash_ConvertPhyAddress(*Address); // 转换为物理地址，USB传输已经检查地址越界

    SPI_FLASH_PageWrite(Writebuff, addr_dest, FLASH_PAGE_SIZE);   // 写一页到空余扇区

    Address->Page++;
}

/*******************************************************************************
* Function Name  : Flash_Write
* Description    : 写函数
* Input          : Memory_Offset为写入地址，Writebuff为要写数据，Transfer_Length为要写的数据长度
* Return         : Status
*******************************************************************************/
uint16_t Flash_Write(uint32_t Memory_Offset, uint8_t *Writebuff, uint16_t Transfer_Length)
{
  uint32_t sector_num, pagesForWrite;

  pagesForWrite = Transfer_Length / FLASH_PAGE_SIZE;      // 需要写的页数
  /* check block status and calculate start and end addreses */
  writeAddr = Flash_PhyAddrToLogAddr(Memory_Offset);

  if (writeAddr.Zone != CurrentZone)
  {
      CurrentZone = writeAddr.Zone;
  }

  while (pagesForWrite--)
  {
      /*  IDLE state  */
      /****************/
      if (Write_State == WRITE_IDLE)       // 如果是开始新扇区写状态
      {
          sector_num = Flash_GetFreeSector();                    // 获取空余扇区用作交换块

          freeAddr = Flash_GetAddress(sector_num);               // 获得逻辑地址
          Initial_Page  = writeAddr.Page;                        // 记录起始写的页数

          writeAddr.Page = 0;
          Flash_Copy (writeAddr, freeAddr, PAGE_NUM_PER_SECTOR); // 将原有的扇区所有内容写入空闲页freeAddr中

          Flash_EraseSector(writeAddr);                          // 擦除原来使用的扇区
		  writeAddr.Page = Initial_Page;						 // 恢复原页

          Flash_Write_OnePage(Writebuff, &writeAddr);
          Written_Pages++;
	      Writebuff += FLASH_PAGE_SIZE;

          if (SCSI_BlkLen == SCSI_BLKLEN_VALID)                  // 通过文件系统写
          {
              if (pagesForWrite)
			      Check_If_SectorEnd(writeAddr.Page);
          }
          else                                                   // 通过上位机USB写
          {
              if(!(Written_Pages % PAGES_pre_Block))             // 写到块的整数倍
              {
                #ifdef   DEBUG_LCD
                    Current_LCD_Line++;
                    if (Current_LCD_Line > MAX_LCD_LINE)
                    {
                        Current_LCD_Line = 0;
                        LCD_Clear(LCD_COLOR_GREEN);
                    }

                    LCD_DisplayStringLine(LINE(Current_LCD_Line), "Write full one block");
                #endif  /*DEBUG_LCD*/

                  Written_Block++;
              }
              Check_If_SCSI_BlkLen(writeAddr.Page);
          }
      }
      /* WRITE state */
      else if (Write_State == WRITE_ONGOING)                     // 如果为继续写状态
      {
          Flash_Write_OnePage((uint8_t *)Writebuff, &writeAddr);
          Written_Pages++;
	      Writebuff += FLASH_PAGE_SIZE;

          if (SCSI_BlkLen == SCSI_BLKLEN_VALID)                  // 通过文件系统写
          {
              if (pagesForWrite)
			      Check_If_SectorEnd(writeAddr.Page);
          }
          else                                                   // 通过上位机USB写
          {
              if(!(Written_Pages % PAGES_pre_Block))             // 写到块的整数倍
              {
                #ifdef   DEBUG_LCD
                    Current_LCD_Line++;
                    if (Current_LCD_Line > MAX_LCD_LINE)
                    {
                        Current_LCD_Line = 0;
                        LCD_Clear(LCD_COLOR_GREEN);
                    }

                    LCD_DisplayStringLine(LINE(Current_LCD_Line), "Write full one block");
                #endif  /*DEBUG_LCD*/

                  Written_Block++;
              }
              Check_If_SCSI_BlkLen(writeAddr.Page);
          }
      }
  }

  if (SCSI_BlkLen == SCSI_BLKLEN_VALID)                  // 通过文件系统写，收尾工作
  {
      Flash_Sector_Updata();                             // 更新扇区信息
      Written_Pages = 0;
      Write_State = WRITE_IDLE;                          // 标记为开始写状态
  }

  return NAND_OK;
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

