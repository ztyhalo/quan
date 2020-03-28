# ifndef __CONFIG_H
#define  __CONFIG_H


typedef unsigned char			uint8;			// �޷���8λ���ͱ���
typedef signed char				int8;			// �з���8λ���ͱ���
typedef unsigned short			uint16;			// �޷���16λ���ͱ���
typedef signed short			int16;			// �з���16λ���ͱ���
typedef unsigned int			uint32;			// �޷���32λ���ͱ���
typedef signed int				int32;			// �з���32λ���ͱ���
typedef float					fp32;			// �����ȸ�������32λ���ȣ�
typedef double					fp64;			// ˫���ȸ�������64λ���ȣ�

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
