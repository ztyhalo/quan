/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : task1.c
* ��  �� :��ȡ���룬���ж����뷶Χ
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
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
* �������ƣ�TaskIOINPUT ()
* ���������������򣬼�����룬���ؼ��źŸ�ֵ
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
// void TaskIOINPUT(void *pdata)
// {
// 	pdata = pdata;
// 	for(;;)
// 	{
// 		Input = ReadInput();	
// 		if(Input==5) //ͬʱ�ж������
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
// //		  CORUSCATE=1;//�����뿪ʼ����	
// 		}
// 		else 	CORUSCATE=0; 
// 		OSTimeDly(10);	
// 	}

// }	
// u8 ReadInput(void)//�����룬ֻ����ͬһʱ��ֻ��һ·����
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
// 	else	return 5;// ͬʱ��2���������£�����5
// }

