/**
  ******************************************************************************
  * @file    usart.h
  * @author  辛鑫
  * @version V1.0
  * @date    2013-1-5
  * @brief   在该头文件中将定义有关串口通信的数据结构，以及常量。
  * 依赖关系: usart.c对应的头文件,需要调用usart.c定义的子函数的源文件需要包含usart.h.
  *
  *               >>>>  在工程中的位置  <<<<
  *               main.c / main.h / includes.h
  *               3-应用层 (同步参数初始化)
  *               3-应用层 (起停切换和同步控制)
  *               3-应用层（保护监控）
  *               3-应用层（辅助设备状态监测及控制）
  *               3-应用层（对控制层的接口）
  *               2-协议层（控制CAN协议）
  *               2-协议层（组网CAN协议）
  *            √  2-协议层（异步串口通讯协议）
  *               1-硬件驱动层
  ******************************************************************************
  * @copy
  * <h2><center>&copy; COPYRIGHT 天津华宁电子有限公司 开发一部</center></h2>
  */
/*
 * 防止多次包含机制
 */
#ifndef __USART_H__
#define __USART_H__

/** @addtogroup 核心控制器板程序
  * @{
  */

/** @addtogroup USART
  * @{
  */

/** @defgroup 头文件
  * @{
  */

/**
  * @}
  */

/** @defgroup 宏定义
  * @{
  */

#define PRINT_COM          USART2

/* Constants used by Serial Command Line Mode */
#define CMD_STRING_SIZE       20

/** @defgroup 类型定义
  * @{
  */

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

/** @defgroup 变量定义
  * @{
  */

/**
  * @}
  */

/** @defgroup 函数声明
  * @{
  */


/**
  * @}
  */

/** @defgroup 函数定义
  * @{
  */
void PFUSARTInit(void);
void GetInputString(u8 *buffP);
/**
  * @}
  */
/**
  * @}
  */

#endif
/****************************************** END OF FILE **************************************/

