#include  <cpu.h>
#include  <stm32f10x_conf.h>
#include  <ucos_ii.h>
#include  "sdio_sdcard.h"


//#ifndef GPIO
//#define GPIO


//#define GPIO_Pin_LED GPIO_Pin_8
//#define GPIO_Port_LED GPIOB

#define volume 0x28      //音量大小
//#endif

/* 全局变量声明区------------------------------------------------------*/
#ifndef GlobalVariable
#define GlobalVariable

//extern u8 Input;
//extern u8 InputErr ;

#endif
/* 函数声明区 ---------------------------------------------------------*/
void TargetInit(void);
void SysTickConfiguartion(void);
//void LCDOUTPUT(void *);
void I2C1_Init(void);
u8 WM8978_Init(void);
u8 ReadInput(void);
uint8_t i2c_CheckDevice(uint8_t );
//void LCDDRIVE(u8,u8);
void StartPlay(void);
u8 PlayMusic(u8);
void UserTick(void);
void ConfStartPlay(uint32_t audiofreq, uint16_t num);
/******************************************end of file*************************/
