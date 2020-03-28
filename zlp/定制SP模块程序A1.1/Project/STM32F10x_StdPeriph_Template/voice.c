
#include "include.h"

//******************************************* ºê¶¨Òå***************************
#define Null (_PlayICBBuf *)0

                                            
//******************************************³£Á¿¶¨Òå***************************


//*****************************************±äÁ¿¶¨Òå****************************
_PlayICBBuf PlayIBC[INFINITEPLAYLISTDEPTH];//¿ÕÏĞ²¥·ÅÁĞ±í

_PlayICBBuf *FiniteListHead = (_PlayICBBuf *)0;//ÓĞÏŞ´Î´ı²¥·ÅÁĞ±íÍ·Ö¸Õë£¬
_PlayICBBuf *FiniteListTail =  (_PlayICBBuf *)0;//ÓĞÏŞ´Î´ı²¥·ÅÁĞ±íÎ²Ö¸Õë
_PlayICBBuf *FiniteListCurPlay =  (_PlayICBBuf *)0;//ÓĞÏŞ´Îµ±Ç°²¥·ÅÓïÒô¿éÖ¸Õë

_PlayICBBuf FiniteList;//ÓĞÏŞ´Î²¥·Å±äÁ¿
_PlayICBBuf FiniteListBuf;//ÓĞÏŞ´Î²¥·Å±äÁ¿»º´æ

_PlayICBBuf *InFiniteListHead = (_PlayICBBuf *)0;//ÎŞÏŞ´Î´ı²¥·ÅÁĞ±íÍ·Ö¸Õë£¬
_PlayICBBuf *InFiniteListTail =  (_PlayICBBuf *)0;//ÎŞÏŞ´Î´ı²¥·ÅÁĞ±íÎ²Ö¸Õë
_PlayICBBuf *InFiniteListCurPlay =  (_PlayICBBuf *)0;//ÎŞÏŞ´Îµ±Ç°²¥·ÅÓïÒô¿éÖ¸Õë
_PlayICBBuf *InFiniteListNullTail = (_PlayICBBuf *)0;//ÎŞÏŞ´Î¿Õ½ÚµãÎ²Ö¸Õë


u8 FiniteListNodeNum = 0;//ÓĞÏŞ´Î»º³åÇø½ÚµãÊıÄ¿
u8 InFiniteListNodeNum = 0;//ÎŞÏß´Î»º³åÇø½ÚµãÊıÄ¿

u16 ADCConvertedValue[ADCBUFFER] = {0};

_VM8978Status VM8978Status;

_SoftTimer SoftTimer;
_VolumeCtr VolumeCtr;

OS_EVENT *MsgQueue;
void     *MsgQueueTbl[MSG_QUEUE_SIZE];


//******************************************º¯ÊıÉùÃ÷***************************
void Init_FreeList(void);
//******************************************º¯Êı¶¨Òå***************************
/****************************************************************************
* º¯ÊıÃû³Æ: ADC1Init_Base
* ¹¦ÄÜÃèÊö: ADC1³õÊ¼»¯º¯Êı
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void ADC1Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  ADC_InitTypeDef ADC_InitStructure;
	/*Ê¹ÄÜÓÃµ½µÄÊ±ÖÓ×ÊÔ´*/
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA
//	 						|RCC_APB2Periph_GPIOB
//	 						|RCC_APB2Periph_GPIOC,
//							ENABLE);
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC×î´óÊ±¼ä²»ÄÜ³¬¹ı14M
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //Ê¹ÄÜDMA´«Êä    


	/* ADC1 ¶ÔÓ¦µÄ1¸ö²É¼¯Í¨µÀÅäÖÃ ------------------------------------*/
//   /*ÊäÈë¶Ë¿ÚAÅäÖÃ*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* ADC1 ÅäÖÃ ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;	                    //ÓÃÓÚ×ª»»µÄÍ¨µÀÊı
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 ¹æÔòÄ£Ê½Í¨µÀÅäÖÃ */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

//    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Æô¶¯ADC1 */
    ADC_Cmd(ADC1, ENABLE);


	// ÏÂÃæÊÇADC×Ô¶¯Ğ£×¼£¬¿ª»úºóĞèÖ´ĞĞÒ»´Î£¬±£Ö¤¾«¶È
    /* Ê¹ÄÜADC1¸´Î»Ğ£Õı*/
    ADC_ResetCalibration(ADC1);
    /* */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* ¿ªÊ¼ADC1¸´Î»Ğ£Õı */
    ADC_StartCalibration(ADC1);
    /* ¼ìÑéÊÇ·ñĞ£ÕıÍê±Ï */
    while(ADC_GetCalibrationStatus(ADC1));
	// ADC×Ô¶¯Ğ£×¼½áÊø---------------
//
//    /* ¿ªÆôADC1×ª»» */
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/****************************************************************************
* º¯ÊıÃû³Æ: DMAADCInit_Base
* ¹¦ÄÜÃèÊö: ADC1³õÊ¼»¯º¯Êı
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void DMAADCInit_Base(void)
{
    DMA_InitTypeDef DMA_InitStructure;

	/*DMA1Í¨µÀ£±ÅäÖÃ ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCConvertedValue[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;		  //dma´«Êä·½Ïòµ¥Ïò
    DMA_InitStructure.DMA_BufferSize = 80;					  //ÉèÖÃDMAÔÚ´«ÊäÊ±»º³åÇøµÄ³¤¶È
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //ÉèÖÃDMAµÄÍâÉèµİÔöÄ£Ê½
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		   //ÉèÖÃDMAµÄÄÚ´æµİÔöÄ£Ê½
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	 //×ª»»½á¹ûµÄÊı¾İ´óĞ¡
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;					  //ÉèÖÃDMAµÄ´«ÊäÄ£Ê½£ºÁ¬Ğø²»¶ÏµÄÑ­»·Ä£Ê½
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Ê¹ÄÜDMA1Í¨µÀ£±*/
    DMA_Cmd(DMA1_Channel1, ENABLE);

}
/****************************************************************************
* º¯ÊıÃû³Æ: ADCfliter()
* ¹¦ÄÜÃèÊö: ADC1ÂË²¨ı
* Èë¿Ú²ÎÊı: ÎŞ
* ³ö¿Ú²ÎÊı: ÎŞ
* Ê¹ÓÃËµÃ÷:
****************************************************************************/
void ADCfliter(u16 *p,u16 num)
{
    u16 i;
    u32 sum = 0;
	u32 Volume1,Volume2;
    for(i = 0;i < num;i++)
    {
        sum += p[i];
    }
    sum = sum/num;
    
	//Volume1 = sum*0.015625;//(sum/4096)*64;
	//Volume2=64-Volume1;
	Volume1 = sum*0.009765625;//(sum/4096)*40;
	Volume2=64-Volume1;
	
	if(Volume2<=0)
	{
		VolumeCtr.VolumeNow=0;
    }
	else if(Volume2>64)
	{
		VolumeCtr.VolumeNow=64;
    }
	else
	{
		VolumeCtr.VolumeNow=Volume2;
    }
		
}

/*******************************************************************************
* º¯ÊıÃû³Æ£ºInit_PlayStatus
* ¹¦ÄÜÃèÊö£º³õÊ¼»¯²¥·Å
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£ºÎŞ
********************************************************************************/
void Init_PlayStatus(void)
{
	VM8978Status.complete = 1;
	VM8978Status.mode = 0;
	VM8978Status.PlayStatus = 0;
//  InFiniteListCurPlay = InFiniteListHead;
}
/*******************************************************************************
* º¯ÊıÃû³Æ£ºInit_FreeList
* ¹¦ÄÜÃèÊö£º³õÊ¼»¯Á´±í
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£ºÎŞ
********************************************************************************/
void Init_FreeList(void)
{
	u8 i;
	_PlayICBBuf *Pt1 = (_PlayICBBuf *)0;
	_PlayICBBuf *Pt2 = (_PlayICBBuf *)0;

	InFiniteListHead = (_PlayICBBuf *)0;//´ı²¥·ÅÁĞ±íÍ·Ö¸Õë£¬
	InFiniteListTail =  (_PlayICBBuf *)0;//´ı²¥·ÅÁĞ±íÎ²Ö¸Õë

	Pt1 = &PlayIBC[0];
	Pt2 = &PlayIBC[1];

	Pt1->prev = (_PlayICBBuf *)0;
	for(i = 0;i < (INFINITEPLAYLISTDEPTH-1);i++){
		Pt1->next = Pt2;
		Pt2->prev = Pt1; 
		Pt1++;
		Pt2++;
	
	}

	Pt1->next = (_PlayICBBuf *)0;
    
    InFiniteListHead = &PlayIBC[0];
    InFiniteListTail = &PlayIBC[0];
	InFiniteListNullTail = &PlayIBC[INFINITEPLAYLISTDEPTH-1];
    InFiniteListNodeNum = 0;
	InFiniteListCurPlay = InFiniteListHead;  //³õÊ¼»¯µ±Ç°²¥·ÅÖ¸ÕëÎªÍ·Ö¸Õë
}

/*******************************************************************************
* º¯ÊıÃû³Æ£ºPushNode
* ¹¦ÄÜÃèÊö: ÏòÓïÒô²¥·ÅÁ´±íÑ¹ÈëÒ»¸öÓïÒô¿é
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£ºÎŞ
********************************************************************************/
u8 PushNode(_PlayICB *pt)
{
#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
#endif
	
	_PlayICBBuf *Ptmp;
    Ptmp = InFiniteListTail;
    if((*Ptmp).next != Null){
        (*Ptmp).segaddr = (*pt).segaddr;
        (*Ptmp).playinterval = (*pt).playinterval;
        (*Ptmp).playtimes = (*pt).playtimes;
        (*Ptmp).playtype = (*pt).playtype;
        (*Ptmp).playprior = (*pt).playprior;

				OS_ENTER_CRITICAL();
				InFiniteListNodeNum++;
				InFiniteListTail = InFiniteListTail->next;
				OS_EXIT_CRITICAL();
                                
        return 0;
    }
    return 0xFF;
}



/*******************************************************************************
* º¯ÊıÃû³Æ£ºDelNode
* ¹¦ÄÜÃèÊö: É¾³ıÓïÒô»º³åÇøÌØ¶¨µØÖ·½Úµã
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£º½«¸Ã½Úµã»¹µ½¿ÕbufferµÄÄ©¶Ë
********************************************************************************/
u8 DelNode(u16 segmentaddr)
{
//#if OS_CRITICAL_METHOD == 3                      
//    OS_CPU_SR  cpu_sr;
//#endif

	  _PlayICBBuf *ptmp = InFiniteListHead;
    u8 i,number,notmatch = 0,f=0;
    
    if(InFiniteListNodeNum == 0){
        return f=0;
    }
	 if(InFiniteListNodeNum == 1){
		if((*ptmp).segaddr ==segmentaddr){
//       OS_ENTER_CRITICAL();
			
			  (*ptmp).segaddr = 0;
			  (*ptmp).playinterval = 0;
    		(*ptmp).playtimes = 0;
    		(*ptmp).playtype = 0;
    		(*ptmp).playprior = 0;
			   //--É¾³ıÍ·ÔªËØ½Úµã
         InFiniteListHead = InFiniteListTail;
         InFiniteListCurPlay = InFiniteListHead;//²¥·ÅÖ¸Õë¸úËæÍ·Ö¸Õë
         InFiniteListNullTail->next = ptmp;   //É¾³ıÖ¸Õë·ÅÔÚ¿ÕÎªÖ¸ÕëºóÃæ
         ptmp->next = Null;
         ptmp->prev = InFiniteListNullTail;
		 InFiniteListNullTail = ptmp; 
			          
         InFiniteListNodeNum--;

//				 OS_EXIT_CRITICAL();	
			
				return f=1;	
		}	
	}
    number=InFiniteListNodeNum ;
	for(i = 0;i < (InFiniteListNodeNum );i++)
    {
		if((*ptmp).segaddr ==segmentaddr)
		{
//            OS_ENTER_CRITICAL();
						if(InFiniteListCurPlay==ptmp)  //ÒªÉ¾³ıµÄÖ¸ÕëÎªµ±Ç°²¥·ÅÖ¸Õë
						{
								if(ptmp==InFiniteListTail->prev) //µ±Ç°²¥·ÅÖ¸ÕëÎªÎ²Ö¸ÕëµÄÇ°Ò»¸öÖ¸Õë
								{
										InFiniteListCurPlay=InFiniteListHead;//µ±Ç°²¥·ÅÖ¸ÕëÉèÖÃÎªÍ·Ö¸Õë
								}
								else
								{
											InFiniteListCurPlay=ptmp->next;
								}
						}
						//´ıÉ¾³ıµÄ½ÚµãÇåÁã
						(*ptmp).segaddr = 0;
						(*ptmp).playinterval = 0;
						(*ptmp).playtimes = 0;
						(*ptmp).playtype = 0;
						(*ptmp).playprior = 0;
			
            if(ptmp==InFiniteListHead) //´ıÉ¾³ıµÄ½ÚµãÎªµÚÒ»¸ö½Úµã
            {
                ptmp->next->prev=NULL;  //µÚ¶ş¸ö½ÚµãÉèÎªÍ·½Úµã
                InFiniteListHead=ptmp->next;

            }
            else //´ıÉ¾³ıµÄ½Úµã´¦ÓÚÍ·Ö¸ÕëºÍÎªÖ¸ÕëÖ®¼ä
            {
                 ptmp->next->prev=ptmp->prev;  //´ıÉ¾³ı½ÚµãÇ°ÃæµÄ½ÚµãºÍºóÃæµÄ½Úµã´®Á¬ÆğÀ´
                 ptmp->prev->next=ptmp->next;
            }
						//¹é»¹É¾³ı½Úµã¿éµ½Î²Ö¸ÕëÖ®ºó
			InFiniteListNullTail->next = ptmp;
            ptmp->next = Null;
            ptmp->prev = InFiniteListNullTail;
			InFiniteListNullTail = ptmp; 

            InFiniteListNodeNum--;

//            OS_EXIT_CRITICAL();            
			return f=1;
		}
		else
		{
			notmatch++;
			ptmp=ptmp->next;
		}  
	}

    if(notmatch == number)
    {
        return f=2;
    }
		return f;
}

/*******************************************************************************
* º¯ÊıÃû³Æ£ºDel_AllNode
* ¹¦ÄÜÃèÊö: É¾³ıÎŞÏŞ´ÎÓïÒôÁ´±íÖĞµÄËùÓĞ½Úµã£¬¼´½ÚµãÇåÁã
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ê¹ÓÃËµÃ÷£ºÎŞ
********************************************************************************/
u8 Del_AllNode(void)
{
	  _PlayICBBuf *Ptmp;
		u8 i,j,err=0;
    Ptmp = InFiniteListHead;
		if(InFiniteListNodeNum<=0)
		{
				err=1;
			  return err;
		}
    else
		{
			j=InFiniteListNodeNum;
			for(i=0;i<j;i++)
			{
					
					(*Ptmp).segaddr = 0;
					(*Ptmp).playinterval = 0;
					(*Ptmp).playtimes = 0;
					(*Ptmp).playtype = 0;
					(*Ptmp).playprior =0;

					InFiniteListNodeNum--;
				  if(InFiniteListNodeNum==0)
					{
						return err=0;
					}
					Ptmp = Ptmp->next;
					
			}                           
    }
    return err;
}




/*******************************end of file***********************************/
