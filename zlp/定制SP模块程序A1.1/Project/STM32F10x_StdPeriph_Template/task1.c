/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 文件名 : task1.c
* 描  述 :读取输入，并判断输入范围
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
*********************************************************************************************
*/

#include "ucos_ii.h"
#include "stm32f10x.h"
#include "target.h"
#include "ff.h"
#include <diskio.h>
#include <stdio.h>
#include <string.h>
#include "wavplay.h"
#include "bsp_wm8978.h"

/********************************************************************************************
* 函数名称：TaskIOINPUT ()
* 功能描述：主程序，检测输入，给关键信号赋值
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
// void TaskIOINPUT(void *pdata)
// {
// 	pdata = pdata;
// 	for(;;)
// 	{
// 		Input = ReadInput();	
// 		if(Input==5) //同时有多个输入
// 		{
// 			InputErr =1;
// 			CORUSCATE=0;
// 		}
// 		else if(Input==0)//
// 		{
// 			InputErr =0;
// 			CORUSCATE=0; 	
//     }else if(Input&&(InputErr==0))
// 		{
// //		  CORUSCATE=1;//有输入开始闪灯	
// 		}
// 		else 	CORUSCATE=0; 
// 		OSTimeDly(10);	
// 	}

// }	
// u8 ReadInput(void)//读输入，只允许同一时刻只有一路输入
// {
// 	u8 In1=0,In2=0,In3=0,In4=0;
// 	In1 = GPIO_ReadInputDataBit(GPIO_Port_PINN1, GPIO_Pin_PINN1);
// 	In2 = GPIO_ReadInputDataBit(GPIO_Port_PINN2, GPIO_Pin_PINN2);
// 	In3 = GPIO_ReadInputDataBit(GPIO_Port_PINN3, GPIO_Pin_PINN3);
// 	In4 = GPIO_ReadInputDataBit(GPIO_Port_PINN4, GPIO_Pin_PINN4);
//   if     (!In1&&In2&&In3&&In4) return 1;                                                                   
// 	else if(In1&&!In2&&In3&&In4) return 2;
// 	else if(In1&&In2&&!In3&&In4) return 3;
// 	else if(In1&&In2&&In3&&!In4) return 4;
// 	else if (In1&&In2&&In3&&In4) return 0;
// 	else	return 5;// 同时有2个按键按下，返回5
// }

