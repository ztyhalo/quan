/*
*********************************************************************************************
*                                   ÃÏΩÚª™ƒ˛µÁ◊””–œﬁπ´Àæ
*                                        «∂»Î Ωø™∑¢◊È
*
*                                
*
* Œƒº˛√˚ : CanSendMessage.c
* √Ë   ˆ :∂
* ◊˜  ’ﬂ : 
* ∞Ê  ±æ : V1.0
* »’  ∆⁄ :
*********************************************************************************************
*/
#include "ucos_ii.h"
#include "stm32f10x.h"
#include "CAN.h"




/***********************************************************************************************************
* √˚  ≥∆:	CanSendMessage_Task
* ≤Œ   ˝:	Œﬁ
* ∑µªÿ÷µ:	Œﬁ
* π¶  ƒ‹:   Õ®–≈÷° ’∑¢π‹¿Ìº∞Ω‚Œˆ
* ±∏  ◊¢:	Œﬁ
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
	CAN_FIFOFlush(&CANTxBuf);		                                           //∑¢ÀÕª∫≥Â«Âø’

	for(;;)
	{    

 		/***************************∑¢ÀÕ¥¶¿Ì≤ø∑÷******************************/
 		if(CAN_FIFOPeek(&CANTxBuf) > 0){				                       //»Ùµ•¥Œ∑¢ÀÕª∫≥Â÷–”– ˝æ›÷°£¨‘Ú∑¢ÀÕ

 			OS_ENTER_CRITICAL();
 			CAN_FIFOGetOne(&CANTxBuf,&CAN_MessageTx);
 			OS_EXIT_CRITICAL();

 			CAN_SendData(&CAN_MessageTx);									   
				
 		}
 		/***************************Ω” ’¥¶¿Ì≤ø∑÷******************************/
 		if(CAN_FIFOPeek(&CANRxBuf) > 0){				                       //»ÙΩ” ’ª∫≥Â÷–”– ˝æ›÷°£¨‘ÚΩ´ ˝æ›øº≥ˆ£¨◊ˆ–≠“È∑÷Œˆ£¨‘⁄Ω” ’÷–∂œ÷– ’ ˝
 			OS_ENTER_CRITICAL();
 			CAN_FIFOGetOne(&CANRxBuf,&CAN_MessageRx);
 			OS_EXIT_CRITICAL();
            
            CAN_PlaySoundAnalyse(&CAN_MessageRx);	          			       //Ω” ’ ˝æ›Ω‚Œˆ             											   
 		}

 		/*CAN ’∑¢π‹¿Ì*/
 		if(CAN_FIFOPeek(&CANRTxBuf) > 0){
 			if(CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer != 0){
 				CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer --;

 			}else{
 				if(CANRTxBuf.buf[CANRTxBuf.rd].rsdcnt > 0){
 					CAN_FIFOCopyOne(&CANRTxBuf,&CAN_MessageRTx);			   //¥”÷ÿ∑¢ª∫≥Â«¯ƒ⁄øΩ≥ˆ“ª÷° ˝æ›
 					CANRTxBuf.buf[CANRTxBuf.rd].rsdtimer = CANRTxBuf.buf[CANRTxBuf.rd].rsdintval;			
 					CANRTxBuf.buf[CANRTxBuf.rd].rsdcnt --;
 					CAN_SendData(&CAN_MessageRTx);
 				}
 				else{
 					CAN_FIFODeleteOne(&CANRTxBuf);							   //»Ù÷ÿ∑¢ÀÕ¥Œ ˝“—æ≠µΩ£¨‘Ú…æ≥˝∏√÷°
 				}		
 			}
 		}
//        flashcnt++;
//        if(flashcnt >= 50){                                                    //1s…¡À∏“ª¥Œ
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
