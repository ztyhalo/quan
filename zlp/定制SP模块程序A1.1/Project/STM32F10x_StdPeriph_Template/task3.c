/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : task3.c
* ��  �� :��ȡת�뿪�أ������������ʾ����
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
*********************************************************************************************
*/
#include "ucos_ii.h"
#include "stm32f10x.h"
#include "target.h"



/********************************************************************************************
* �������ƣ�LCDOUTPUT ()
* ����������LCD��ʾ����
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
// void LCDOUTPUT(void *pdata)
// {
// 	uint8_t sw1 =0, sw2=0, sw3 =0;
// 	uint8_t key1=0,key2=0,key3=0,key4=0;
// 	pdata = pdata;
// 	while(1)
// 	{
// 				//��ת�뿪��ֵ
// 		key1 = GPIO_ReadInputDataBit(GPIO_Port_SW1_1, GPIO_Pin_SW1_1);
// 		key2 = GPIO_ReadInputDataBit(GPIO_Port_SW1_2, GPIO_Pin_SW1_2);
// 		key3 = GPIO_ReadInputDataBit(GPIO_Port_SW1_3, GPIO_Pin_SW1_3);
// 		key4 = GPIO_ReadInputDataBit(GPIO_Port_SW1_4, GPIO_Pin_SW1_4);
// 		sw1  = !key1+!key2*4+!key3*2+!key4*8;
// 		if(sw1>9) sw1=0; 
// 		SW1  = sw1;
// 		key1 = GPIO_ReadInputDataBit(GPIO_Port_SW2_1, GPIO_Pin_SW2_1);
// 		key2 = GPIO_ReadInputDataBit(GPIO_Port_SW2_2, GPIO_Pin_SW2_2);
// 		key3 = GPIO_ReadInputDataBit(GPIO_Port_SW2_3, GPIO_Pin_SW2_3);
// 		key4 = GPIO_ReadInputDataBit(GPIO_Port_SW2_4, GPIO_Pin_SW2_4);
// 		sw2  = !key1+!key2*4+!key3*2+!key4*8;
// 		if(sw2>9) sw2=0; 
// 		SW2  = sw2;
// 		key1 = GPIO_ReadInputDataBit(GPIO_Port_SW3_1, GPIO_Pin_SW3_1);
// 		key2 = GPIO_ReadInputDataBit(GPIO_Port_SW3_2, GPIO_Pin_SW3_2);
// 		key3 = GPIO_ReadInputDataBit(GPIO_Port_SW3_3, GPIO_Pin_SW3_3);
// 		key4 = GPIO_ReadInputDataBit(GPIO_Port_SW3_4, GPIO_Pin_SW3_4);
// 		sw3  = !key1+!key2*4+!key3*2+!key4*8;
// 		if(sw3>9) sw3=0; 
// 		SW3  = sw3;	
// 		
// 	  gLCD++;
// 	  if(gLCD>=4) gLCD=1;
// 		if((SW1*100+SW2*10+SW3)>gMaxBoardNum)//��鷶Χ�Ƿ���ȷ0~40
// 		{
// 			switch(gLCD)
// 			{
// 				case 1 : LCDDRIVE(1,10);break;//��Χ����ȷ���EEE
// 				case 2 : LCDDRIVE(2,10);break;//��Χ����ȷ���EEE	
// 				case 3 : LCDDRIVE(3,10);break;//��Χ����ȷ���EEE	
// 			}	
// 		}
// 		else//��Χ��ȷ�����ǰֵ
// 		{
// 			switch(gLCD)
// 	 		{
// 				case 1 : LCDDRIVE(1,SW1);break;//��Χ����ȷ���EEE
// 				case 2 : LCDDRIVE(2,SW2);break;//��Χ����ȷ���EEE	
// 				case 3 : LCDDRIVE(3,SW3);break;//��Χ����ȷ���EEE	
// 			}	
// 		}
// 		OSTimeDly(5);			
//   }

// }
// void LCDDRIVE(u8 LCDNumber,u8 Figure)
// {
// 	u8 i=0;

// 	GPIO_ResetBits(GPIO_Port_LATCH, GPIO_Pin_LATCH);
// 	GPIO_ResetBits(GPIO_Port_SHIFT, GPIO_Pin_SHIFT);	
// 	for(i=0;i<8;i++)
// 	{
// 		GPIO_ResetBits(GPIO_Port_SHIFT, GPIO_Pin_SHIFT);
// 		if((LCD[Figure]>>i)& 0x01) GPIO_ResetBits(GPIO_Port_DS, GPIO_Pin_DS);
// 		else GPIO_SetBits(GPIO_Port_DS, GPIO_Pin_DS);
// 		GPIO_SetBits(GPIO_Port_SHIFT, GPIO_Pin_SHIFT);
// 		GPIO_ResetBits(GPIO_Port_SHIFT, GPIO_Pin_SHIFT);	
//   }
// 	GPIO_SetBits(GPIO_Port_LATCH, GPIO_Pin_LATCH);	
// 	GPIO_ResetBits(GPIO_Port_LATCH, GPIO_Pin_LATCH);	
// 	switch(LCDNumber)
// 	{
// 	  case  1 : GPIO_SetBits(GPIO_Port_LED1, GPIO_Pin_LED1);                    
// 							GPIO_ResetBits(GPIO_Port_LED2, GPIO_Pin_LED2);
// 							GPIO_ResetBits(GPIO_Port_LED3, GPIO_Pin_LED3);		          
// 		          break;
// 	  case  2 : GPIO_SetBits(GPIO_Port_LED2, GPIO_Pin_LED2);
//               GPIO_ResetBits(GPIO_Port_LED1, GPIO_Pin_LED1);                      
//               GPIO_ResetBits(GPIO_Port_LED3, GPIO_Pin_LED3);		          
// 		          break;
// 		case  3 : GPIO_SetBits(GPIO_Port_LED3, GPIO_Pin_LED3);
//               GPIO_ResetBits(GPIO_Port_LED1, GPIO_Pin_LED1);                      
//               GPIO_ResetBits(GPIO_Port_LED2, GPIO_Pin_LED2);	          
// 		          break;	
// 		default:  break;
//   }	
	
// }

