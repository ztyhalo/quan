/**
  ******************************************************************************
  * @file    usart.c
  * @author  ����
  * @version V1.0
  * @date    2013-1-5
  * @brief   �ڸ�Դ�ļ���ʵ��:(1)Modbus֡����,֡���ͻ�������(2)Modbusͨ��Э��
  * ������ϵ: ��Ӧ��ͷ�ļ���usart.h,��Ҫ�������ﶨ����Ӻ�����Դ�ļ���Ҫ����usart.h.
  *
  *               >>>>  �ڹ����е�λ��  <<<<
  *               3-Ӧ�ò㣨���͹��ϴ���
  *               2-Э��㣨����CANЭ�飩
  *               2-Э��㣨����CANЭ�飩
  *            ��  2-Э��㣨�첽����Э�飩
  *               2-����㣨��װ�ײ��������ϲ���ã�
  *               1-Ӳ��������
  ******************************************************************************
  * @copy
  * <h2><center>&copy; COPYRIGHT ������������޹�˾ ����һ��</center></h2>
  */

/** @addtogroup ���Ŀ����������
  * @{
  */

/** @defgroup USART
  * @brief �첽����ͨ�Ŵ������
  * @{
  */

/** @defgroup ͷ�ļ�
  * @{
  */
#include"stm32f10x.h"
#include"printf.h"

#include <stdio.h>
/**
  * @}
  */

/** @defgroup �궨��
  * @{
  */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
  * @}
  */

/** @defgroup ��������
  * @{
  */

/**
  * @}
  */

/** @defgroup ��������
  * @{
  */

/**
  * @}
  */

/** @defgroup ��������
  * @{
  */

/**
  * @}
  */

/**
  * @brief ���ڳ�ʼ������
  * @param ��
  * @retval ��
  */
void PFUSARTInit()
{
    USART_InitTypeDef USART_InitStructure;                  /// USART��ʼ�����ݽṹ
	USART_DeInit(PRINT_COM);                                   /// ����

    /*
     * ��USART��ʼ�����ݽṹ�ĳ�Ա��ֵ, 9600,8,1,N, ��ʹ������, ���շ��Ͷ�ʹ��
     */
    USART_InitStructure.USART_BaudRate   = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
                                                            /// ʹ�ܷ��ͺͽ���
    /*
     * USART2
     */
    USART_Init(PRINT_COM, &USART_InitStructure);            /// ����

    USART_ITConfig(PRINT_COM, USART_IT_RXNE, ENABLE);       /// ʹ�ܴ����жϽ���
    USART_Cmd(PRINT_COM, ENABLE);                           /// ʹ�ܴ���
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
	uint32_t timeout = 60000;
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(PRINT_COM, USART_FLAG_TC) == RESET)
    {
	    timeout--;
	    if (timeout == 0)
	    {
		    break;
	    }
	}

    /* e.g. write a character to the USART */
    USART_SendData(PRINT_COM, (u8) ch);

    return ch;
}

/**
  * @brief  Test to see if a key has been pressed on the HyperTerminal
  * @param  key: The key pressed
  * @retval 1: Correct
  *         0: Error
  */
u32 SerialKeyPressed(u8 *key)
{

  if (USART_GetFlagStatus(PRINT_COM, USART_FLAG_RXNE) != RESET)	 // ���ڽ���
  {
    *key = (u8)PRINT_COM->DR & 0xFF;
    return 1;
  }
  else                 // û�н�������
  {
    return 0;
  }
}

/************************************ END OF FILE *******************************************/

