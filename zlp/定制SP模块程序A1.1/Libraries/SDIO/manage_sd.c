/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : manage_flash.c
* Author             : ����
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

  SPI_FLASH_Init();               // SPI�ӿ�Flash��ʼ��
  Write_State = WRITE_IDLE;

  return Status;
}

/*******************************************************************************
* Function Name  : Flash_GetAddress
* Description    : ����������ת��Ϊ�顢������ҳ��
* Input          : ������
* Return         : Address_t
*******************************************************************************/
NAND_ADDRESS Flash_GetAddress(uint32_t total_sector)
{
  NAND_ADDRESS Address_t;

  Address_t.Page   = 0;
  Address_t.Sector = total_sector & (PAGE_NUM_PER_SECTOR - 1);
  Address_t.Block  = total_sector / SECTOR_NUM_PER_BLOCK;
  Address_t.Zone   = 0;

  while (Address_t.Block >= BLOCK_NUM_PER_ZONE)    // ��������
  {
    Address_t.Block -= BLOCK_NUM_PER_ZONE;
    Address_t.Zone++;                              // ��������1
  }
  return Address_t;
}

/*******************************************************************************
* Function Name  : Flash__PhyAddrToLogAddr
* Description    : �������ַת��Ϊ�顢������ҳ��
* Input          : �����ַ
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

  while (Address_t.Block >= BLOCK_NUM_PER_ZONE)    // ������������
  {
      Address_t.Block -= BLOCK_NUM_PER_ZONE;
      Address_t.Zone++;                            // ��������1
  }
  return Address_t;
}

/*******************************************************************************
* Function Name  : Flash_ConvertPhyAddress
* Description    : ���߼���ַת��Ϊ�����ַ
* Input          : �߼���ַ
* Output         : None
* Return         : Status
*******************************************************************************/
uint32_t Flash_ConvertPhyAddress (NAND_ADDRESS Address)
{
  uint32_t addr, ret;

  addr = CHANGE_PHY_ADDR(Address);            // ���е�ַת������ַ��0��ʼ
  ret  = SPI_FLASH_AddrCheck(addr);           // ���е�ַԽ����

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
  SPI_FLASH_BulkErase();           // ������������
  //Flash_BuildLUT(0);               // ������һ����

  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_GetFreeSector
* Description    : ��ȡһ����������
* Input          : None
* Return         : ������¼��Ϣ
*******************************************************************************/
static uint16_t Flash_GetFreeSector (void)
{
    uint32_t pCurrentSector;

    pCurrentSector = CurrentFreeSector;

    /* ���ѡȡ����ֹһ����������д */
    if (CurrentFreeSector < MAX_PHY_SECTORS_PER_ZONE)
        CurrentFreeSector++;
    else
        CurrentFreeSector = MAX_LOG_SECTORS_PER_ZONE + 1;

    return pCurrentSector;
}

/*******************************************************************************
* Function Name  : Flash_Sector_Updata
* Description    : ����д������Ϣ��
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static uint16_t Flash_Sector_Updata (void)
{
  uint32_t  Page_Back;

  Page_Back = writeAddr.Page;       // ��¼��ǰҳλ��
  /* precopy old first pages */
  if (Initial_Page != 0)            // ������ʹ�õ�ҳ��
  {
      freeAddr.Page = writeAddr.Page = 0;
      Flash_Copy (freeAddr, writeAddr, Initial_Page);  // ��ԭ�е�ҳ������д��writeAddr��

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
      Flash_Copy(freeAddr, writeAddr, PAGE_NUM_PER_SECTOR - Page_Back);  // ��д��ʣ��ҳ����writeAddr��

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
  Flash_EraseSector(freeAddr);                           // ������������

  writeAddr.Sector++;                                    // ������һ����
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
* Description    : ����ĳ������
* Input          : AddressΪ��Ҫ�����������߼���ַ
* Return         : Status
*******************************************************************************/
static uint16_t Flash_EraseSector (NAND_ADDRESS Address)
{
    uint32_t addr;

    Address.Page = 0;                          // ������ʼλ��
    addr = Flash_ConvertPhyAddress(Address);   // ת��Ϊ�����ַ
    if (addr == 1)
        return NAND_FAIL;

    SPI_FLASH_SectorErase(addr);
    return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_Copy
* Description    : ������ҳ����
* Input          : Address_SrcΪ��Ҫ���Ƶ�Դ��ַ��Address_DestΪĿ���ַ��PageToCopy
                   ΪҪ���Ƶ�ҳ��
* Return         : Status
*******************************************************************************/
static uint16_t Flash_Copy(NAND_ADDRESS Address_Src, NAND_ADDRESS Address_Dest, uint16_t PageToCopy)
{
  uint8_t  Copybuff[FLASH_PAGE_SIZE];                    // һҳΪ256byte
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

    SPI_FLASH_BufferRead ((uint8_t *)Copybuff, addr_src, FLASH_PAGE_SIZE);   // ��ȡԭҳ����
    SPI_FLASH_PageWrite ((uint8_t *)Copybuff, addr_dest, FLASH_PAGE_SIZE);   // д��Ŀ��ҳ

    SPI_FLASH_AddressIncrement(&Address_Src);                                // ԭ��ַ����һҳ
    SPI_FLASH_AddressIncrement(&Address_Dest);                               // Ŀ�ĵ�ַ����һҳ
  }

  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Flash_Read
* Description    : ������
* Input          : Memory_OffsetΪ��ȡ��ַ��ReadbuffΪ���뻺�棬Transfer_LengthΪ��ȡ����
* Return         : Status
*******************************************************************************/
uint16_t Flash_Read(uint32_t Memory_Offset, uint8_t *Readbuff, uint16_t Transfer_Length)
{
  NAND_ADDRESS logAddress;

  logAddress = Flash_PhyAddrToLogAddr(Memory_Offset);

  if (logAddress.Zone != CurrentZone)                // �����Ҫ����
  {
      CurrentZone = logAddress.Zone;
  }

  //sector_num = phyAddress.Block * NAND_BLOCK_SIZE + phyAddress.Sector;
  SPI_FLASH_BufferRead(Readbuff, Memory_Offset, Transfer_Length);   // ��ȡ����

  return NAND_OK;
}

/*******************************************************************************
* Function Name  : Check_If_SectorEnd
* Description    : ����Ƿ�ﵽ������������
* Input          : currentPageΪ�ж���ҳ��
* Return         : -
*******************************************************************************/
void Check_If_SectorEnd(uint32_t currentPage)
{
    if (currentPage == PAGE_NUM_PER_SECTOR) // ���Ҫдҳ���ﵽ���������ҳ��
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

        Flash_Sector_Updata();              // ����������Ϣ
        Write_State = WRITE_IDLE;           // ���Ϊ��ʼд״̬
    }
    else
        Write_State = WRITE_ONGOING;        // ���Ϊ����д״̬
}

/*******************************************************************************
* Function Name  : Check_If_SCSI_BlkLen
* Description    : ����Ƿ�ﵽ�鳤�Ⱥ���
* Input          : currentPageΪ�ж���ҳ��
* Return         : -
*******************************************************************************/
void Check_If_SCSI_BlkLen(uint32_t currentPage)
{
    if (Written_Block == SCSI_BlkLen)       // ���ҳ���ﵽUSB���δ��䳤��
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

        Flash_Sector_Updata();              // ����������Ϣ
        Written_Pages = 0;
        Written_Block = 0;
        Write_State = WRITE_IDLE;           // ���Ϊ��ʼд״̬
    }
    else
    {
        Check_If_SectorEnd(currentPage);
    }
}

/*******************************************************************************
* Function Name  : Flash_Write_OnePage
* Description    : дFATһҳʵ����дFLASH��ҳ
* Input          : WritebuffΪд�����ݣ�AddressΪҪд�߼���ַ
* Return         : Status
*******************************************************************************/
void Flash_Write_OnePage(uint8_t *Writebuff, NAND_ADDRESS *Address)
{
    uint32_t addr_dest;
    /* write the new page */
    addr_dest = Flash_ConvertPhyAddress(*Address); // ת��Ϊ�����ַ��USB�����Ѿ�����ַԽ��

    SPI_FLASH_PageWrite(Writebuff, addr_dest, FLASH_PAGE_SIZE);   // дһҳ����������

    Address->Page++;
}

/*******************************************************************************
* Function Name  : Flash_Write
* Description    : д����
* Input          : Memory_OffsetΪд���ַ��WritebuffΪҪд���ݣ�Transfer_LengthΪҪд�����ݳ���
* Return         : Status
*******************************************************************************/
uint16_t Flash_Write(uint32_t Memory_Offset, uint8_t *Writebuff, uint16_t Transfer_Length)
{
  uint32_t sector_num, pagesForWrite;

  pagesForWrite = Transfer_Length / FLASH_PAGE_SIZE;      // ��Ҫд��ҳ��
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
      if (Write_State == WRITE_IDLE)       // ����ǿ�ʼ������д״̬
      {
          sector_num = Flash_GetFreeSector();                    // ��ȡ������������������

          freeAddr = Flash_GetAddress(sector_num);               // ����߼���ַ
          Initial_Page  = writeAddr.Page;                        // ��¼��ʼд��ҳ��

          writeAddr.Page = 0;
          Flash_Copy (writeAddr, freeAddr, PAGE_NUM_PER_SECTOR); // ��ԭ�е�������������д�����ҳfreeAddr��

          Flash_EraseSector(writeAddr);                          // ����ԭ��ʹ�õ�����
		  writeAddr.Page = Initial_Page;						 // �ָ�ԭҳ

          Flash_Write_OnePage(Writebuff, &writeAddr);
          Written_Pages++;
	      Writebuff += FLASH_PAGE_SIZE;

          if (SCSI_BlkLen == SCSI_BLKLEN_VALID)                  // ͨ���ļ�ϵͳд
          {
              if (pagesForWrite)
			      Check_If_SectorEnd(writeAddr.Page);
          }
          else                                                   // ͨ����λ��USBд
          {
              if(!(Written_Pages % PAGES_pre_Block))             // д�����������
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
      else if (Write_State == WRITE_ONGOING)                     // ���Ϊ����д״̬
      {
          Flash_Write_OnePage((uint8_t *)Writebuff, &writeAddr);
          Written_Pages++;
	      Writebuff += FLASH_PAGE_SIZE;

          if (SCSI_BlkLen == SCSI_BLKLEN_VALID)                  // ͨ���ļ�ϵͳд
          {
              if (pagesForWrite)
			      Check_If_SectorEnd(writeAddr.Page);
          }
          else                                                   // ͨ����λ��USBд
          {
              if(!(Written_Pages % PAGES_pre_Block))             // д�����������
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

  if (SCSI_BlkLen == SCSI_BLKLEN_VALID)                  // ͨ���ļ�ϵͳд����β����
  {
      Flash_Sector_Updata();                             // ����������Ϣ
      Written_Pages = 0;
      Write_State = WRITE_IDLE;                          // ���Ϊ��ʼд״̬
  }

  return NAND_OK;
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

