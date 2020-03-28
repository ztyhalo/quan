# ifndef __CONFIG_H
#define  __CONFIG_H


typedef unsigned char			uint8;			// 无符号8位整型变量
typedef signed char				int8;			// 有符号8位整型变量
typedef unsigned short			uint16;			// 无符号16位整型变量
typedef signed short			int16;			// 有符号16位整型变量
typedef unsigned int			uint32;			// 无符号32位整型变量
typedef signed int				int32;			// 有符号32位整型变量
typedef float					fp32;			// 单精度浮点数（32位长度）
typedef double					fp64;			// 双精度浮点数（64位长度）

#define cs8  const char
#define cs16 const int
#define cs32 const long

//#include    "_CAN.h"
// #include "..STM32Lib.h"
//#include	"main.h"
//#include    "..\user\App.h"
//#include    "..\user\target.h"
//#include    "_CAN.h"

#include "stdint.h"
#include    <string.h>
#include "ucos_ii.h"
#include "CAN.h"
#include "stm32f10x.h"

//#include 	"voice.h"
extern void sysreset(void);
#endif



/****************************end of file*****************************/
