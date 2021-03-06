/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 文件名 : task5.c
* 描  述 :�
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
*********************************************************************************************
*/
#include "ucos_ii.h"
#include "stm32f10x.h"
#include "CAN.h"




/***********************************************************************************************************
* 名  称:	CanSendMessage_Task
* 参  数:	无
* 返回值:	无
* 功  能:   通信帧收发管理及解析
* 备  注:	无
*************************************************************************************************************/
void TaskCanSendMessage(void *pdata)
{
#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
#endif
//    u8 flashcnt = 0;

	CAN_FrameStruct CAN_MessageTx = {0};
	CAN_FrameStruct CAN_MessageRx = {0};									    
	CAN_FrameStruct CAN_MessageRTx = {0};
																			  
    pdata = pdata;
	CAN1Init();								                               
	CAN_FIFOFlush(&CANTxBuf);		                                           //发送缓冲清空

	for(;;)
	{    

 		/***************************发送处理部分******************************/
 		if(CAN_FIFOPeek(&CANTxBuf) > 0){				                       //若单次发送缓冲中有数据帧，则发送

 			OS_ENTER_CRITICAL();
 			CAN_FIFOGetOne(&CANTxBuf,&CAN_MessageTx);
 			OS_EXIT_CRITICAL();

 			CAN_SendData(&CAN_MessageTx);									   
				
 		}
 		/***************************接收处理部分******************************/
 		if(CAN_FIFOPeek(&CANRxBuf) > 0){				                       //若接收缓冲中有数据帧，则将数据考出，做协议分析，在接收中断中收数
 			OS_ENTER_CRITICAL();
 			CAN_FIFOGetOne(&CANRxBuf,&CAN_MessageRx);
 			OS_EXIT_CRITICAL();
            
            CAN_PlaySoundAnalyse(&CAN_MessageRx);	          			       //接收数据解析             											   
 		}

 		/*CAN收发管理*/
 		if(CAN_FIFOPeek(&CANRTxBuf) > 0){
 			if(CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer != 0){
 				CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer --;

 			}else{
 				if(CANRTxBuf.buf[CANRTxBuf.rd].rsdcnt > 0){
 					CAN_FIFOCopyOne(&CANRTxBuf,&CAN_MessageRTx);			   //从重发缓冲区内拷出一帧数据
 					CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer = CANRTxBuf.buf[CANRTxBuf.rd].rsdintval;			
 					CANRTxBuf.buf[CANRTxBuf.rd].rsdcnt --;
 					CAN_SendData(&CAN_MessageRTx);
 				}
 				else{
 					CAN_FIFODeleteOne(&CANRTxBuf);							   //若重发送次数已经到，则删除该帧
 				}		
 			}
 		}
//        flashcnt++;
//        if(flashcnt >= 50){                                                    //1s闪烁一次
//            flashcnt = 0;
//            GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)1);
//        }
//        else{
//            GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)0);
//        }
		OSTimeDly(2);
	}
}


/***********************************************************************************************
*										End Of File
***********************************************************************************************/
