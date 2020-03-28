/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : CanSendMessage.c
* ��  �� :�
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
*********************************************************************************************
*/
#include "ucos_ii.h"
#include "stm32f10x.h"
#include "CAN.h"




/***********************************************************************************************************
* ��  ��:	CanSendMessage_Task
* ��  ��:	��
* ����ֵ:	��
* ��  ��:   ͨ��֡�շ���������
* ��  ע:	��
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
	CAN_FIFOFlush(&CANTxBuf);		                                           //���ͻ������

	for(;;)
	{    

 		/***************************���ʹ�����******************************/
 		if(CAN_FIFOPeek(&CANTxBuf) > 0){				                       //�����η��ͻ�����������֡������

 			OS_ENTER_CRITICAL();
 			CAN_FIFOGetOne(&CANTxBuf,&CAN_MessageTx);
 			OS_EXIT_CRITICAL();

 			CAN_SendData(&CAN_MessageTx);									   
				
 		}
 		/***************************���մ�����******************************/
 		if(CAN_FIFOPeek(&CANRxBuf) > 0){				                       //�����ջ�����������֡�������ݿ�������Э��������ڽ����ж�������
 			OS_ENTER_CRITICAL();
 			CAN_FIFOGetOne(&CANRxBuf,&CAN_MessageRx);
 			OS_EXIT_CRITICAL();
            
            CAN_PlaySoundAnalyse(&CAN_MessageRx);	          			       //�������ݽ���             											   
 		}

 		/*CAN�շ�����*/
 		if(CAN_FIFOPeek(&CANRTxBuf) > 0){
 			if(CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer != 0){
 				CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer --;

 			}else{
 				if(CANRTxBuf.buf[CANRTxBuf.rd].rsdcnt > 0){
 					CAN_FIFOCopyOne(&CANRTxBuf,&CAN_MessageRTx);			   //���ط��������ڿ���һ֡����
 					CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer = CANRTxBuf.buf[CANRTxBuf.rd].rsdintval;			
 					CANRTxBuf.buf[CANRTxBuf.rd].rsdcnt --;
 					CAN_SendData(&CAN_MessageRTx);
 				}
 				else{
 					CAN_FIFODeleteOne(&CANRTxBuf);							   //���ط��ʹ����Ѿ�������ɾ����֡
 				}		
 			}
 		}
//        flashcnt++;
//        if(flashcnt >= 50){                                                    //1s��˸һ��
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
