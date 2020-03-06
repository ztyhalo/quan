#include "flash.h"

#if defined(STM32L412xx)
	#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_63   											 /* Start @ of user Flash area */
	#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */
#elif defined(STM32L452xx)
	#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_127   											/* Start @ of user Flash area */
	#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */
#endif


uint32_t Address = 0, PAGEError = 0;

static uint32_t GetPage(uint32_t Address);

static FLASH_EraseInitTypeDef  EraseInitStruct;


/*************************************************************************************************
* �������ƣ�FLASH_WriteBytes(uint32_t Addr, uint16_t AddrOffect, uint8_t *Buffer, uint32_t Length)
* ������������FLASHд������
* ��ڲ�����Addr--��ʼ��ַ��AddrOffect--��ַƫ������Buffer--��д�����ݣ�Length--���ݳ���
* ���ڲ�������
* ʹ��˵������
**************************************************************************************************/
void FLASH_WriteBytes(uint32_t Addr, uint16_t AddrOffect, uint64_t *Buffer, uint32_t Length)
{
	uint32_t pAddr;
	pAddr = Addr + AddrOffect;

  /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
   
   /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = FLASH_BANK_1;
  EraseInitStruct.Page        = GetPage(pAddr);
  EraseInitStruct.NbPages     = 1;
  HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
    
//  Address = Addr + AddrOffect;
  while (Length--)
  {     
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, pAddr, *Buffer) == HAL_OK)
    {    
        pAddr += 8;
        Buffer += 1;
    }
  }
  HAL_FLASH_Lock();
}


/************************************************************************************************
* �������ƣ�FLASH_ReadBytes(uint32_t Addr, uint16_t AddrOffset, uint8_t *Buffer, uint16_t Length)
* ������������FLASH��ȡ����
* ��ڲ�����Addr--��ʼ��ַ��AddrOffset--ƫ������Buffer--���ݶ�ȡ�󱣴�λ�ã�Length--���ݳ���
* ���ڲ�������
* ʹ��˵������
*************************************************************************************************/
void FLASH_ReadBytes(uint32_t Addr, uint16_t AddrOffset, uint8_t *Buffer, uint16_t Length)
{
	uint8_t *pAddr;
	pAddr=(uint8_t *)(Addr + AddrOffset); 

//	Address = FLASH_USER_START_ADDR;
//  MemoryProgramStatus = 0x0;

	while (Length--)
	{
		*Buffer++ = *pAddr++;
	}  
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}













