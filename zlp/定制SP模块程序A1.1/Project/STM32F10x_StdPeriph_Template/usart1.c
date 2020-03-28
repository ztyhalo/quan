#include "stm32f10x.h"
#include "usart1.h"

/*******************************************************************************
* Function Name  : Usart_Gpio
* Description    : 485串口配置
* Input          : 无
* Output         : 无
* Return         : 无
*******************************************************************************/
void Usart_Gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//时钟定义
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3, ENABLE);

	// MOSI3的gpio配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// MOSI4的gpio配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// MOSI5的gpio配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

}
/*******************************************************************************
* Function Name  : Usart_config
* Description    : 串口配置
* Input          : 无
* Output         : 无
* Return         : 无
*******************************************************************************/
void Usart_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3,&USART_InitStructure);
}
/*******************************************************************************
* Function Name  : Usart_NVIC
* Description    : 串口配置
* Input          : 无
* Output         : 无
* Return         : 无
*******************************************************************************/
void Usart_NVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//串口中断配置,优先级默认
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;				 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;				 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;				 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	NVIC_Init(&NVIC_InitStructure);
 }
/*******************************************************************************
* Function Name  : USART_puts
* Description    : 串口输出字符串
* Input          : 字符串或者地址
* Output         : 无
* Return         : 无
*******************************************************************************/

void USART1_puts(char *str)
{
   //第一次发送数据的时候先读一次SR,再写一次SD
   USART_GetFlagStatus(USART1, USART_FLAG_TC);
   while(*str)
   {
	   USART_SendData(USART1,*str++);
	   //用tc是因为保证最后一个数据完整，因为关闭使能端是在以为寄存器也发送完成之后
	   while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	   
   }
}
/*******************************************************************************
* Function Name  : USART_putc
* Description    : 串口输出一个字符
* Input          : 字符
* Output         : 无
* Return         : 无
*******************************************************************************/
void USART1_putc(char c)
{
	//读一次sr
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_SendData(USART1,c); 
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}
/*******************************************************************************
* Function Name  : USART_puts
* Description    : 串口输出字符串
* Input          : 字符串或者地址
* Output         : 无
* Return         : 无
*******************************************************************************/

void USART2_puts(char *str)
{
   //第一次发送数据的时候先读一次SR,再写一次SD
   USART_GetFlagStatus(USART2, USART_FLAG_TC);
   while(*str)
   {
	   USART_SendData(USART2,*str++);
	   //用tc是因为保证最后一个数据完整，因为关闭使能端是在以为寄存器也发送完成之后
	   while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	   
   }
}
/*******************************************************************************
* Function Name  : USART_putc
* Description    : 串口输出一个字符
* Input          : 字符
* Output         : 无
* Return         : 无
*******************************************************************************/
void USART2_putc(char c)
{
	//读一次sr
	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	USART_SendData(USART2,c); 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}
/*******************************************************************************
* Function Name  : USART_puts
* Description    : 串口输出字符串
* Input          : 字符串或者地址
* Output         : 无
* Return         : 无
*******************************************************************************/

void USART3_puts(char *str)
{
   //第一次发送数据的时候先读一次SR,再写一次SD
   USART_GetFlagStatus(USART3, USART_FLAG_TC);
   while(*str)
   {
	   USART_SendData(USART3,*str++);
	   //用tc是因为保证最后一个数据完整，因为关闭使能端是在以为寄存器也发送完成之后
	   while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	   
   }
}
/*******************************************************************************
* Function Name  : USART_putc
* Description    : 串口输出一个字符
* Input          : 字符
* Output         : 无
* Return         : 无
*******************************************************************************/
void USART3_putc(char c)
{
	//读一次sr
	USART_GetFlagStatus(USART3, USART_FLAG_TC);
	USART_SendData(USART3,c); 
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}
void USART_INIT(void)
{
    Usart_Gpio();
	Usart_Config ();
	Usart_NVIC();
	//串口使能
	USART_Cmd(USART1, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}



/***************************END OF FILE*******************************************/
