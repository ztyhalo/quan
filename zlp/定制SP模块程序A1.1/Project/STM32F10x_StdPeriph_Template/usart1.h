#ifndef __USART1_H__
#define __USART1_H__

void Usart_Gpio(void);
void Usart_Config(void);
void Usart_NVIC(void);
void USART_CONTROL_TX(void);
void USART_CONTROL_RX(void);
void USART1_puts(char *str);
void USART1_putc(char c);
void USART2_puts(char *str);
void USART2_putc(char c);
void USART3_puts(char *str);
void USART3_putc(char c);
void USART_INIT(void);

#endif
