
#include "include.h"

u8 NoSameDate=0;
u8 CANRxBufFilled1=0;
u8 CANRxBufFilled2=0;
u8 FiniteNodeSame=0;
u8 Bisuocomplete=0;
u8 NotPushBs=0;
extern 	u16 ERRCNT;
u16 SENDSUC;
u32 RESETID=0;
//----------------------------------- CAN½ÓÊÕbuffer -----------------------------------------
CAN_FIFOStruct CANRxBuf =
{
    0,                                      // wr
    0,                                      // rd
    0,                                      // num
};

//----------------------------------- CAN·¢ËÍbuffer -----------------------------------------
CAN_FIFOStruct CANTxBuf =
{
    0,                                      // wr
    0,                                      // rd
    0,                                      // num
};
//----------------------------------- CANÖØ·¢ËÍbuffer -----------------------------------------
CAN_FIFOStruct CANRTxBuf =
{
	0,
	0,
	0,
};
_Playlistinfo Playlistinfo ;
//--------------º¯ÊıÉùÃ÷Çø---------------------------------------------------
/*³õÊ¼»¯CANÒı½Å*/
void  CAN1_GPIO_Config(void);
/*³õÊ¼»¯CANÖĞ¶Ï*/
void  CAN1_NVIC_Config(void);
/*CANÄ£Ê½ÅäÖÃ*/
void  CAN1_Mode_Config(void);
/*CAN¹ıÂËÆ÷ÅäÖÃ*/
void  CAN1_Filter_Config(void);

//---------------º¯ÊıÊµÌå¶¨ÒåÇø----------------------------------------------
/****************************************************************************
* º¯ÊıÃû³Æ: CAN1Init
* ¹¦ÄÜÃèÊö: CAN ³õÊ¼»¯º¯Êı
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void CAN1Init(void)
{
  CAN1_GPIO_Config();
	CAN1_NVIC_Config();
	CAN1_Mode_Config();
	CAN1_Filter_Config(); 
}

/*******************************************************************************************
* º¯ÊıÃû³Æ£º CAN1_GPIO_Config()
* ¹¦ÄÜÃèÊö£º  ÓÃÓÚÅäÖÃCANÊÕ·¢Òı½Å£¨CAN1_TX(PA.12),CAN1_RX(PA.11)£©£»        
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£º³õÊ¼»¯µ÷ÓÃ
*******************************************************************************************/
void  CAN1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(CAN1_RCC, ENABLE);  		// ¿ªÆôCAN1µÄÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);		//¿ªÆô¸´ÓÃÊ±ÖÓ¹¦ÄÜ

	/* Configure CAN1 pin: RX PA11*/									               // PA11
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // ÉÏÀ­ÊäÈë
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
    
    /* Configure CAN1 pin: TX PA12*/									               // PA12
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // ¸´ÓÃÍÆÍìÊä³ö
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   	GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
}
/*******************************************************************************************
* º¯ÊıÃû³Æ£º CAN1_NVIC_Config()
* ¹¦ÄÜÃèÊö£º ¶ÔCAN1µÄÓÅÏÈ¼¶½øĞĞÉèÖÃ         
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£º³õÊ¼»¯µ÷ÓÃ
*******************************************************************************************/
void  CAN1_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
	/*ÖĞ¶ÏÉèÖÃ*/
	
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;	               //CAN1 RX0ÖĞ¶Ï
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //ÇÀÕ¼ÓÅÏÈ¼¶1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //×ÓÓÅÏÈ¼¶Îª0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//²»Ê¹ÓÃÖĞ¶ÏÓÅÏÈ¼¶Ç¶Ì×¡£ÒòÎªSysTickµÄÖĞ¶ÏÓÅÏÈ¼¶Îª0x0f
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//0¼¶ÓÃÓÚSysTick
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

}
/*******************************************************************************************
* º¯ÊıÃû³Æ£º CAN1_Mode_Config()
* ¹¦ÄÜÃèÊö£º CAN1Í¨ĞÅ²ÎÊıÉèÖÃ      
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£º³õÊ¼»¯µ÷ÓÃ
*******************************************************************************************/
void  CAN1_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN1Í¨ĞÅ²ÎÊıÉèÖÃ**********************************/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CANµ¥Ôª³õÊ¼»¯*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  ¹Ø±ÕÊ±¼ä´¥·¢Í¨ĞÅÄ£Ê½Ê¹ÄÜ
    //CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  ×Ô¶¯ÀëÏß¹ÜÀí
	///////////////////////////////////////////
	CAN_InitStructure.CAN_ABOM=ENABLE;				  //×Ô¶¯ÀëÏß¹ÜÀí,Èç¹ûABOMÎ»Îª¡¯1¡¯£¬bxCAN½øÈëÀëÏß×´Ì¬ºó£¬¾Í×Ô¶¯¿ªÆô»Ö¸´¹ı³Ì¡£
													  //Èç¹ûABOMÎ»Îª¡¯0¡¯£¬Èí¼ş±ØĞëÏÈÇëÇóbxCAN½øÈëÈ»ºóÔÙÍË³ö³õÊ¼»¯Ä£Ê½£¬Ëæºó»Ö¸´¹ı³Ì²Å±»¿ªÆô¡£
	/////////////////////////////////////// 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  Ê¹ÓÃ×Ô¶¯»½ĞÑÄ£Ê½
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ½ûÖ¹±¨ÎÄ×Ô¶¯ÖØ´«	  DISABLE-×Ô¶¯ÖØ´«
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ½ÓÊÕFIFO Ëø¶¨Ä£Ê½  DISABLE-Òç³öÊ±ĞÂ±¨ÎÄ»á¸²¸ÇÔ­ÓĞ±¨ÎÄ  
    CAN_InitStructure.CAN_TXFP=ENABLE;			   //MCR-TXFP  ·¢ËÍFIFOÓÅÏÈ¼¶ ENABLE-ÓÅÏÈ¼¶ÓÉ·¢ËÍÇëÇóµÄË³Ğò¾ö¶¨
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //Õı³£¹¤×÷Ä£Ê½
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ÖØĞÂÍ¬²½ÌøÔ¾¿í¶È 1¸öÊ±¼äµ¥Ôª
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;		   //BTR-TS1 Ê±¼ä¶Î1 Õ¼ÓÃÁË8¸öÊ±¼äµ¥Ôª
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		   //BTR-TS1 Ê±¼ä¶Î2 Õ¼ÓÃÁË7¸öÊ±¼äµ¥Ôª
    CAN_InitStructure.CAN_Prescaler =24;   //²¨ÌØÂÊÎª100k/bps
	  CAN_Init(CAN1, &CAN_InitStructure);
/******************************************************************************************
*²¨ÌØÂÊ¼ÆËã¹«Ê½
*				          Pclk1
* BRR = -----------------------------------------
*		(CAN_Prescaler + 1)*(1 + CAN_BS1 + CAN_BS2)
* 
*CAN_BS1Ò»°ãÈ¡¡¡8
*CAN_BS1Ò»°ãÈ¡¡¡7
******************************************************************************************/
}
/*******************************************************************************************
* º¯ÊıÃû³Æ£º CAN1_Filter_Config()
* ¹¦ÄÜÃèÊö£º CAN1¹ıÂËÆ÷³õÊ¼»¯²ÎÊıÉèÖÃ      
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£º³õÊ¼»¯µ÷ÓÃ
*******************************************************************************************/
void  CAN1_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
/* ÅäÖÃCAN ¹ıÂËÆ÷ ¹²ÓĞ14¸ö,ÏÂÃæÎªµÚÒ»¸öµÄÅäÖÃ */
	CAN_FilterInitStructure.CAN_FilterNumber= 0;                           
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    
    CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)(FRAMEID_UPTOLOW_PLAY)<<21)&0xFFFF0000) >>16;
    CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)(FRAMEID_UPTOLOW_PLAY)<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

	/* ÅäÖÃCAN ¹ıÂËÆ÷ ¹²ÓĞ14¸ö,ÏÂÃæÎªµÚ¶ş¸öµÄÅäÖÃ */
    CAN_FilterInitStructure.CAN_FilterNumber = 1;						   
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)(FRAMEID_UPTOLOW_STOP )<<21)&0xFFFF0000) >>16;
    CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)(FRAMEID_UPTOLOW_STOP )<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

	/* ÅäÖÃCAN ¹ıÂËÆ÷ ¹²ÓĞ14¸ö,ÏÂÃæÎªµÚÈı¸öµÄÅäÖÃ */
    CAN_FilterInitStructure.CAN_FilterNumber = 2;						   
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)(FRAMEID_ONLINECHECK )<<21)&0xFFFF0000) >>16;
    CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)(FRAMEID_ONLINECHECK )<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
	
		/* ÅäÖÃCAN ¹ıÂËÆ÷ ¹²ÓĞ14¸ö,ÏÂÃæÎªµÚÈı¸öµÄÅäÖÃ */
    CAN_FilterInitStructure.CAN_FilterNumber = 3;						   
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)(FRAMEID_UPTOLOW_RESET)<<21)&0xFFFF0000) >>16;
    CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)(FRAMEID_UPTOLOW_RESET )<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

	
    /*CAN1£¨¶ÔÓ¦¸Ã°åµÄCAN1¿Ú£©Í¨ĞÅÖĞ¶ÏÊ¹ÄÜ*/
	CAN_ITConfig(CAN1, CAN_IT_FMP1|CAN_IT_FMP0, ENABLE);
	/*CAN1·¢ËÍÖĞ¶ÏÊ¹ÄÜ*/
	CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);	
}
/****************************************************************************
* º¯ÊıÃû³Æ: FIFOInit()
* ¹¦ÄÜÃèÊö: ³õÊ¼»¯FIFO
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷: ¸Ãº¯ÊıÃ»ÓĞÊµÏÖ,ĞèÒªÔÚ¶¨ÒåFIFO±äÁ¿Ê±³õÊ¼»¯
****************************************************************************/
void CAN_FIFOFlush(CAN_FIFOStruct *pBuf)
{
    pBuf->wr  = 0;
    pBuf->rd  = 0;
    pBuf->num = 0;
}


/****************************************************************************
* º¯ÊıÃû³Æ: FIFOGetOne()
* ¹¦ÄÜÃèÊö: ¸øFIFOÖĞ·ÅÈëÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0
* Èë¿Ú²ÎÊı: FIFO½á¹¹µØÖ·,´ı·ÅÈëÊı¾İµÄµØÖ·
* ³ö¿Ú²ÎÊı:
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void CAN_FIFOPutOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q)
{
    if(pBuf->num < CAN_BUFFER_DEPTH){
        memcpy((s8*)&pBuf->buf[pBuf->wr], (cs8*)q, sizeof(CAN_FrameStruct));
        pBuf->num++;
        pBuf->wr++;
        if(pBuf->wr > CAN_BUFFER_DEPTH - 1){
            pBuf->wr = 0;
        }
    }
}
/****************************************************************************
* º¯ÊıÃû³Æ: CAN_FIFOGetOne()
* ¹¦ÄÜÃèÊö: ´ÓFIFOÖĞÈ¡³öÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0
* Èë¿Ú²ÎÊı: FIFO½á¹¹µØÖ·,È¡³öÊı¾İÒª·Åµ½µÄµØÖ·
* ³ö¿Ú²ÎÊı:
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void CAN_FIFOGetOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q)
{
    if(pBuf->num > 0){
        memcpy((s8*)q, (cs8*)&pBuf->buf[pBuf->rd], sizeof(CAN_FrameStruct));
        pBuf->num--;
        pBuf->rd++;
        if(pBuf->rd > CAN_BUFFER_DEPTH - 1){
            pBuf->rd = 0;
        }
    }
}
/****************************************************************************
* º¯ÊıÃû³Æ: CAN_FIFOCopyOne()
* ¹¦ÄÜÃèÊö: ´ÓFIFOÖĞÈ¡³öÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0
* Èë¿Ú²ÎÊı: FIFO½á¹¹µØÖ·,È¡³öÊı¾İÒª·Åµ½µÄµØÖ·
* ³ö¿Ú²ÎÊı:
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void CAN_FIFOCopyOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q)
{
    if(pBuf->num > 0){
        memcpy((s8*)q, (cs8*)&pBuf->buf[pBuf->rd], sizeof(CAN_FrameStruct));
    }
}
/****************************************************************************
* º¯ÊıÃû³Æ: CAN_FIFODeleteOne()
* ¹¦ÄÜÃèÊö: ´ÓFIFOÖĞÉ¾³ıÒ»¸ö(°ü)Êı¾İ,³É¹¦·µ»Ø0
* Èë¿Ú²ÎÊı: FIFO½á¹¹µØÖ·¡£
* ³ö¿Ú²ÎÊı:
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void CAN_FIFODeleteOne(CAN_FIFOStruct *pBuf)
{
	u8 i;

    if(pBuf->num > 0){
//        memcpy((s8*)q, (cs8*)&pBuf->buf[pBuf->rd], sizeof(CAN_FrameStruct));
//		Length = sizeof(CAN_FrameStruct);
		for(i = 0;i < pBuf->buf[pBuf->rd].dlc;i++){
			pBuf->buf[pBuf->rd].dat[i] = 0;
		}
		pBuf->buf[pBuf->rd].id = 0;
		pBuf->buf[pBuf->rd].dlc = 0;
		pBuf->buf[pBuf->rd].rsdintval = 0;
		pBuf->buf[pBuf->rd].rsdcnt = 0;
//		pBuf->buf[pBuf->rd].successflg = 0;

        pBuf->num--;
        pBuf->rd++;
        if(pBuf->rd > CAN_BUFFER_DEPTH - 1){
            pBuf->rd = 0;
        }
    }
}
/****************************************************************************
* º¯ÊıÃû³Æ: FIFOPeek
* ¹¦ÄÜÃèÊö: Íµ¿´Ò»ÑÛFIFOÖĞÏÖÔÚÓĞ¼¸¸öÊı¾İ(°ü)
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: FIFOÖĞÊµÌåµÄ¸öÊı
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
u8 CAN_FIFOPeek(CAN_FIFOStruct *pBuf)
{
    return pBuf->num;
}


/****************************************************************************
* º¯ÊıÃû³Æ: CAN_RcvDataInt
* ¹¦ÄÜÃèÊö: ´ÓÖĞ¶ÏÖĞ½ÓÊÕCANÊı¾İ
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷: ÔÚ¸ßÓÅÏÈ¼¶ÖĞ¶ÏÖĞµ÷ÓÃ
****************************************************************************/
void CAN_RcvDataInt(void)
{
	u8 i,j,framenum;
	CanRxMsg RxMessage;

	if((framenum = CAN_MessagePending(CAN1,CAN_FIFO0)) != 0){
		for(i = 0;i < framenum;i++){
			CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);			 //¶ÁÈ¡FIFO0ÄÚµÄ±¨ÎÄ		
			if(CANRxBuf.num < CAN_BUFFER_DEPTH){		 //½«Êı¾İ¿½±´µ½½ÓÊÕ»º³åÖĞ
				if(RxMessage.IDE == CAN_ID_EXT){
					CANRxBuf.buf[CANRxBuf.wr].id = RxMessage.ExtId;
				}else{
					CANRxBuf.buf[CANRxBuf.wr].id = RxMessage.StdId;
				}
				CANRxBuf.buf[CANRxBuf.wr].dlc = RxMessage.DLC;
				for(j = 0;j < CANRxBuf.buf[CANRxBuf.wr].dlc;j++){
					CANRxBuf.buf[CANRxBuf.wr].dat[j] = RxMessage.Data[j];
				}
//				memcpy(CANRxBuf.buf[CANRxBuf.wr].dat, RxMessage.Data, CANRxBuf.buf[CANRxBuf.wr].dlc);
				RxMessage.IDE = 0;
				RxMessage.DLC = 0;
				RxMessage.FMI = 0;

				CANRxBuf.num++;
				CANRxBuf.wr++;
				if(CANRxBuf.wr > CAN_BUFFER_DEPTH - 1){
            		CANRxBuf.wr = 0;
            	}
    		}
        else
        {
			CANRxBufFilled1=1;    //CAN½ÓÊÕ»º´æ1ÒÑÌîÂú
        }					
		}
		CAN_FIFORelease(CAN1,CAN_FIFO0);		                 //´ò¿ª½ÓÊÕ»º³åÇø0£¬¿ÉÒÔ½ÓÊÕĞÂÊı¾İ
	}

	if((framenum = CAN_MessagePending(CAN1,CAN_FIFO1)) != 0){
		for(i = 0;i < framenum;i++){
			
			CAN_Receive(CAN1,CAN_FIFO1, &RxMessage);			 //¶ÁÈ¡FIFO1ÄÚµÄ±¨ÎÄ
			if(CANRxBuf.num < CAN_BUFFER_DEPTH){		 //½«Êı¾İ¿½±´µ½½ÓÊÕ»º³åÖĞ
				if(RxMessage.IDE == CAN_ID_EXT){
					CANRxBuf.buf[CANRxBuf.wr].id = RxMessage.ExtId;
				}else{
					CANRxBuf.buf[CANRxBuf.wr].id = RxMessage.StdId;
				}
				CANRxBuf.buf[CANRxBuf.wr].dlc = RxMessage.DLC;
				for(j = 0;j < CANRxBuf.buf[CANRxBuf.wr].dlc;j++){
					CANRxBuf.buf->dat[j] = RxMessage.Data[j];
				}
				RxMessage.IDE = 0;
				RxMessage.DLC = 0;
				RxMessage.FMI = 0;

				CANRxBuf.num++;
            	CANRxBuf.wr++;
				if(CANRxBuf.wr > CAN_BUFFER_DEPTH - 1){
            		CANRxBuf.wr = 0;
            	}
    		}	
			else
			{
				CANRxBufFilled2=1;    //CAN½ÓÊÕ»º´æ2ÒÑÂú
			}	
		}
		CAN_FIFORelease(CAN1,CAN_FIFO1);		                 //´ò¿ª½ÓÊÕ»º³åÇø1£¬¿ÉÒÔ½ÓÊÕĞÂÊı¾İ
	}
}
/****************************************************************************
* º¯ÊıÃû³Æ:	CAN_SendData
* ¹¦ÄÜÃèÊö: ·ÇÖĞ¶ÏÄ£Ê½ÏÂ·¢ËÍCANÊı¾İÖ¡
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: 0--·¢ËÍÊ§°Ü,1--·¢ËÍ³É¹¦
* Ê¹ÓÃËµÃ÷: ²»ÔÚÖĞ¶Ïº¯ÊıÏÂÊ¹ÓÃ
****************************************************************************/
void CAN_SendData(CAN_FrameStruct *q)
{
	u16 i = 0;
	u8 TransmitMailbox = 0;
	CanTxMsg TxMessage;
        
	if((CAN_GetFlagStatus(CAN1,CAN_FLAG_EPV) == SET)||(CAN_GetFlagStatus(CAN1,CAN_FLAG_BOF) == SET)){
		CAN1Init();                                                        // Èç¹û½ÓÊÕ»òÕß·¢ËÍ´íÎó¼Ä´æÆ÷Öµ³¬¹ı×î´óÖµ  ³õÊ¼»¯CANÄ£¿é
	}

	if((*q).msgflg == (CAN_ID_STD|CAN_RTR_DATA)){			                       //Îª±ê×¼Ö¡
		TxMessage.StdId = (*q).id;
		TxMessage.ExtId = 0;
		TxMessage.IDE = CAN_ID_STD;
	}
	else{																	   //ÎªÀ©Õ¹Ö¡
		TxMessage.StdId = 0;										                               
		TxMessage.ExtId = (*q).id;
		TxMessage.IDE = CAN_ID_EXT;											  
	}
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = (*q).dlc;
	for(i = 0;i < (*q).dlc;i++){
		TxMessage.Data[i] = (*q).dat[i];
	}

	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);                                //·¢ËÍÓÊÏäµÄÊıÁ¿

	if(TransmitMailbox == CAN_NO_MB){							               //Ã»ÓĞ¿ÕÏĞÓÊÏäÊ±µÄ´¦Àí
		for(i = 0;i < 10;i++){												   //µÈ´ı100ms
			OSTimeDly(1);
			if((CAN_TransmitStatus(CAN1,TransmitMailbox) != CAN_NO_MB)){			   //ÔÚµÈ´ıÆÚ¼äÓĞ¿ÕÏĞÓÊÏä
				TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);					   //ÖØĞÂ·¢ËÍ¸ÃÖ¡
				break;
			}
		}
		if(i>=10){
			CAN1Init();													   //³¬¹ı100msÒÀ¾ÉÃ»ÓĞ¿ÕÏĞÓÊÏä£¬Ôò³õÊ¼»¯
			TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);						   //³õÊ¼»¯ºóÖØĞÂ·¢ËÍ¸ÃÖ¡
		}
	}
  	i = 0;
  	while(CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK){                     //³¬Ê±µÈ´ı20ms
    	i++;
		OSTimeDly(1);
		if(i > 2){
			break;
		}
  	}
//	CANTxBuf.buf->sndflg = 1;                                                  //ÒÑ·¢ËÍ±êÊ¶ÓĞĞ§¡£
}
/****************************************************************************
* º¯ÊıÃû³Æ: CAN_PlaySoundAnalyse
* ¹¦ÄÜÃèÊö: ´ÓCAN½ÓÊÕÊı¾İÖĞ½âÎöÒª²¥·ÅµÄÎÄ¼ş
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷: Ô
****************************************************************************/
void CAN_PlaySoundAnalyse(CAN_FrameStruct *CanMsg)
{
	#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
  #endif
    u8 TempArray[8] = {0}; 
    u8 i,err,Dlc,err1;
//     static u16 StopInstruct = 0;
    static u16 StopInstruct[2]={0};
    static u16 StopInstruct1[2]={0};
// 	static u16 StopInstruct2[2]={0}; //½ø²ÎÊıÉèÖÃÊ±Ê¹ÓÃ
// 	static u16 StopInstruct3[2]={0}; //½ø²ÎÊıÉèÖÃÊ±Ê¹ÓÃ
    _PlayICB	tmpSegment; 

    err=err;
	err1=err1;
    if((*CanMsg).id == FRAMEID_UPTOLOW_PLAY)
    {
		if((*CanMsg).dlc == 2)
		{
			for(i=0;i<2;i++)
			{
				TempArray[i] = (*CanMsg).dat[i];
			}

			tmpSegment.segaddr = TempArray[0];
			tmpSegment.playinterval = 800;
			tmpSegment.playtimes = TempArray[1]&0X7F;
			tmpSegment.playtype = TempArray[1]&0X80;
			tmpSegment.playprior = 0;//0Ê±ÎªFIFOÄ£Ê½£¬·Ç0ÎªÓÅÏÈ¼¶·ÅÒôÄ£Ê½(ÓÅÏÈ¼¶È¡Öµ·¶Î§£º1~255)£¬Êı×ÖÔ½´óÓÅÏÈ¼¶Ô½¸ß

			if(tmpSegment.playtype != 0)//ÖØ¸´²¥±¨Ñ¹ÈëÁ´±í
			{
				err=CAN_RxDateAnalyse(&tmpSegment);
				if(err==0)
				{
					OS_ENTER_CRITICAL();
					PushNode(&tmpSegment);									   //Ñ¹ÈëÁ´±í
					OS_EXIT_CRITICAL();
				}
				else
				{
					NoSameDate=1;       //½ÓÊÕÊı¾İÓëÖ®Ç°ÓĞÏàÍ¬µÄ£¬²»Ñ¹ÈëÁ´±í
				}
			}
			else//ÓĞÏŞ´Î²¥±¨¿½±´
			{
				if((FiniteList.segaddr==tmpSegment.segaddr)&&(FiniteListNodeNum ==1))
				{
					FiniteNodeSame=1;
				}
				else
				{
					FiniteNodeSame=0;
				}
				if(((FiniteList.segaddr==0X1E)&&(FiniteNodeSame==1)&&(VM8978Status.complete == 0)&&(VM8978Status.segaddr==0X1E))||((FiniteList.segaddr==0X1F)&&(FiniteNodeSame==1)&&(VM8978Status.complete == 0)&&(VM8978Status.segaddr==0X1F)))
				{
					NotPushBs=1;//ÅÄÏÂ±ÕËøÊ½£¬ÉÏÎ»»úÁ¬·¢Á½´Î481 1E/1F 01ÃüÁî£¬¸ù¾İÀÏµÄSPÖ»±¨1´Î£¬Òò´ËÔÚµÚÒ»±é»¹Î´±¨ÍêÊ±£¬½«µÚ¶ş±éµÄÓïÒôÆÁ±Îµô
				}
				else
				{
					FiniteList.segaddr = tmpSegment.segaddr;
					FiniteList.playtype = tmpSegment.playtype;
					FiniteList.playtimes = tmpSegment.playtimes;
					FiniteList.playprior = tmpSegment.playprior;
					FiniteList.playinterval = tmpSegment.playinterval;
					FiniteList.blockvalid=1;
					FiniteListNodeNum = 1;
				}
			}
			CAN_TxFrame(FRAMEID_LOWTOUP_PLAYACK ,TempArray,2);					   //&TempArray[0]   Ó¦´ğÖ¡
		}
    }
    if((*CanMsg).id == FRAMEID_UPTOLOW_STOP)
    {   
        if((*CanMsg).dlc == 1)
        {
            TempArray[0] = (*CanMsg).dat[0];
            StopInstruct[0] = (u16)((*CanMsg).dat[0]&0x03)<<8;

			OSQPost(MsgQueue,(void *)&StopInstruct[0]);

        }
        if((*CanMsg).dlc == 2)
        {
            for(i=0;i<2;i++)
            {
                TempArray[i] = (*CanMsg).dat[i];
            }
			if((TempArray[0]==0x00)&&(TempArray[1]==0x00))//ÉÏÎ»»ú»á·¢481 00 00À´Í£Ö¹ÓïÒô²¥±¨£¨ÔÚ±ÕËø±»Ì§ÆğÊ±£©
			{
				StopInstruct1[1] = (u16)((*CanMsg).dat[0]&0x03)<<8 | ((*CanMsg).dat[1]);
				OSQPost(MsgQueue,(void *)&StopInstruct1[1]);
			}
			if((TempArray[0]==0X00)||(TempArray[0]==0X01)||(TempArray[0]==0X02))
			{
				err1=CAN_RxStopDateAnalyse(&TempArray[1]);
				if(err1==1)
				{
					StopInstruct1[1] = (u16)((*CanMsg).dat[0]&0x03)<<8 | ((*CanMsg).dat[1]);
					OSQPost(MsgQueue,(void *)&StopInstruct1[1]);
				}
			}
// 			else
// 			{
// 				StopInstruct1[1] = (u16)((*CanMsg).dat[0]&0x03)<<8 | ((*CanMsg).dat[1]);
// 				OSQPost(MsgQueue,(void *)&StopInstruct1[1]);
// 			}

        }
		Dlc=(*CanMsg).dlc;
        CAN_TxFrame(FRAMEID_LOWTOUP_STOPACK ,TempArray,Dlc);
    }
    if((*CanMsg).id ==FRAMEID_ONLINECHECK)
    {
        if((*CanMsg).dlc == 0)
        {
            SoftTimer.OffLineTimer = OFFLINETICK;
            CAN_TxFrame(FRAMEID_ONLINECHECKACK ,TempArray,0);					   //&TempArray[0]   Ó¦´ğÖ¡
        }
    }
	if((*CanMsg).id ==FRAMEID_UPTOLOW_RESET) //½Óµ½0X5FFÊ±£¬¸´Î»
	{
		wav_play_stop(); 
		sysreset();
	}
}
/*******************************************************************************
* º¯ÊıÃû³Æ£ºCAN_TxFrame
* ¹¦ÄÜÃèÊö£º
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£º
********************************************************************************/
void CAN_TxFrame(u32 AnswerCode,u8 *p,u8 Dlc)
{
#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
#endif

    CAN_FrameStruct CAN_Msg_Tx ={0};
	
	CAN_Msg_Tx.msgflg = CAN_ID_STD|CAN_RTR_DATA;

	if(AnswerCode == FRAMEID_LOWTOUP_PLAYACK){								//²¥·ÅÓ¦´ğÖ¡
		CAN_Msg_Tx.id = FRAMEID_LOWTOUP_PLAYACK;
		CAN_Msg_Tx.dlc = 2;
		CAN_Msg_Tx.dat[0] = p[0];//*p
        CAN_Msg_Tx.dat[1] = p[1];//*(p+1)
	}
    else if(AnswerCode == FRAMEID_LOWTOUP_STOPACK ){							//ÉèÖÃÓ¦´ğÖ¡

		CAN_Msg_Tx.id = FRAMEID_LOWTOUP_STOPACK;

// 		CAN_Msg_Tx.dlc = 2;
		CAN_Msg_Tx.dlc=Dlc;
		//CAN_Msg_Tx.dat[0] = *p; 
        if(Dlc==1)
        {
			CAN_Msg_Tx.dat[0] = p[0];//*p; 
		}
        if(Dlc==2)
        {			
			CAN_Msg_Tx.dat[0] = p[0];//*p; 
			CAN_Msg_Tx.dat[1] = p[1];//*(p+1); 
		}
    }
    else if(AnswerCode == FRAMEID_ONLINECHECKACK)
    {
		CAN_Msg_Tx.id = FRAMEID_ONLINECHECKACK;

		CAN_Msg_Tx.dlc = 0;
    }
    
	CAN_Msg_Tx.rsdintval = 0;
	CAN_Msg_Tx.rsdtimer = 0;
// 	CAN_Msg_Tx.rsdcnt = 1;

 	OS_ENTER_CRITICAL();
 	CAN_FIFOPutOne(&CANTxBuf,&CAN_Msg_Tx);
 	OS_EXIT_CRITICAL();
}

/****************************************************************************
* º¯ÊıÃû³Æ: CAN_RxDateAnalyse
* ¹¦ÄÜÃèÊö: ÅĞ¶ÏCAN½ÓÊÕÊı¾İÊÇ·ñÓĞÖØ¸´µÄ
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷: Ô
****************************************************************************/
u8 CAN_RxDateAnalyse(_PlayICB *pt)
{
	u8 i,err;
	_PlayICBBuf *ptmp = InFiniteListHead;
	if(InFiniteListNodeNum==0)
  {
     return err=0;  
  }
//	if(InFiniteListNodeNum>0)
	for(i = 0;i <=InFiniteListNodeNum ;i++)
	{
			if(pt->segaddr==ptmp->segaddr)
			{
				return err=1;
			}
     	
			if((pt->segaddr!=ptmp->segaddr)&&(ptmp==InFiniteListTail))	
			{
				return err=0;  
			}
			ptmp=ptmp->next;
  }
	return err;
	
}


/****************************************************************************
* º¯ÊıÃû³Æ: CAN_RxStopDateAnalyse
* ¹¦ÄÜÃèÊö: ÅĞ¶ÏCAN½ÓÊÕµ½µÄÍ£Ö¹Êı¾İÊÇ·ñÔÚÓĞÏŞ´Î²¥·ÅÁ´±íÖĞ
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷: ?
****************************************************************************/
u8 CAN_RxStopDateAnalyse(u8 *pt)
{
	u8 i,err;
	_PlayICBBuf *ptmp = InFiniteListHead;
	_PlayICBBuf  *ptmp1=&FiniteListBuf;
	if((ptmp1->segaddr==(*pt))&&(FiniteListNodeNum == 1))
	{
		err=1;
    }
	else
	{
		if(InFiniteListNodeNum==0)
		{
			return err=0;  
		}
		for(i = 0;i <=InFiniteListNodeNum ;i++)
		{
			if(ptmp->segaddr==(*pt))
			{
				return err=1;
			}
     	
			if((ptmp->segaddr!=(*pt))&&(ptmp==InFiniteListTail))	
			{
				return err=0;  
			}
			ptmp=ptmp->next;
		}
	}	
	return err;
	
}







/*************************end of file*****************************************/


