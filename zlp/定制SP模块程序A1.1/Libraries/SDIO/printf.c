/**
  ******************************************************************************
  * @file    usart.c
  * @author  辛鑫
  * @version V1.0
  * @date    2013-1-5
  * @brief   在该源文件中实现:(1)Modbus帧接收,帧发送基本函数(2)Modbus通信协议
  * 依赖关系: 对应的头文件是usart.h,需要调用这里定义的子函数的源文件需要包含usart.h.
  *
  *               >>>>  在工程中的位置  <<<<
  *               3-应用层（检测和故障处理）
  *               2-协议层（控制CAN协议）
  *               2-协议层（组网CAN协议）
  *            √  2-协议层（异步串口协议）
  *               2-抽象层（封装底层驱动供上层调用）
  *               1-硬件驱动层
  ******************************************************************************
  * @copy
  * <h2><center>&copy; COPYRIGHT 天津华宁电子有限公司 开发一部</center></h2>
  */

/** @addtogroup 核心控制器板程序
  * @{
  */

/** @defgroup USART
  * @brief 异步串口通信处理程序
  * @{
  */

/** @defgroup 头文件
  * @{
  */
#include"stm32f10x.h"
#include"printf.h"

#include <stdio.h>
/**
  * @}
  */

/** @defgroup 宏定义
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

/** @defgroup 常量声明
  * @{
  */

/**
  * @}
  */

/** @defgroup 常量定义
  * @{
  */

/**
  * @}
  */

/** @defgroup 变量声明
  * @{
  */

/**
  * @}
  */

/**
  * @brief 串口初始化函数
  * @param 无
  * @retval 无
  */
void PFUSARTInit()
{
    USART_InitTypeDef USART_InitStructure;                  /// USART初始化数据结构
	USART_DeInit(PRINT_COM);                                   /// 清零

    /*
     * 给USART初始化数据结构的成员赋值, 9600,8,1,N, 不使用流控, 接收发送都使能
     */
    USART_InitStructure.USART_BaudRate   = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
                                                            /// 使能发送和接收
    /*
     * USART2
     */
    USART_Init(PRINT_COM, &USART_InitStructure);            /// 配置

    USART_ITConfig(PRINT_COM, USART_IT_RXNE, ENABLE);       /// 使能串口中断接收
    USART_Cmd(PRINT_COM, ENABLE);                           /// 使能串口
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

  if (USART_GetFlagStatus(PRINT_COM, USART_FLAG_RXNE) != RESET)	 // 正在接收
  {
    *key = (u8)PRINT_COM->DR & 0xFF;
    return 1;
  }
  else                 // 没有接收内容
  {
    return 0;
  }
}

/************************************ END OF FILE *******************************************/

