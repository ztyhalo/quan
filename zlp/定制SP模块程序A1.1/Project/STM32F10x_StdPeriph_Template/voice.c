
#include "include.h"

//******************************************* �궨��***************************
#define Null (_PlayICBBuf *)0

                                            
//******************************************��������***************************


//*****************************************��������****************************
_PlayICBBuf PlayIBC[INFINITEPLAYLISTDEPTH];//���в����б�

_PlayICBBuf *FiniteListHead = (_PlayICBBuf *)0;//���޴δ������б�ͷָ�룬
_PlayICBBuf *FiniteListTail =  (_PlayICBBuf *)0;//���޴δ������б�βָ��
_PlayICBBuf *FiniteListCurPlay =  (_PlayICBBuf *)0;//���޴ε�ǰ����������ָ��

_PlayICBBuf FiniteList;//���޴β��ű���
_PlayICBBuf FiniteListBuf;//���޴β��ű�������

_PlayICBBuf *InFiniteListHead = (_PlayICBBuf *)0;//���޴δ������б�ͷָ�룬
_PlayICBBuf *InFiniteListTail =  (_PlayICBBuf *)0;//���޴δ������б�βָ��
_PlayICBBuf *InFiniteListCurPlay =  (_PlayICBBuf *)0;//���޴ε�ǰ����������ָ��
_PlayICBBuf *InFiniteListNullTail = (_PlayICBBuf *)0;//���޴οսڵ�βָ��


u8 FiniteListNodeNum = 0;//���޴λ������ڵ���Ŀ
u8 InFiniteListNodeNum = 0;//���ߴλ������ڵ���Ŀ

u16 ADCConvertedValue[ADCBUFFER] = {0};

_VM8978Status VM8978Status;

_SoftTimer SoftTimer;
_VolumeCtr VolumeCtr;

OS_EVENT *MsgQueue;
void     *MsgQueueTbl[MSG_QUEUE_SIZE];


//******************************************��������***************************
void Init_FreeList(void);
//******************************************��������***************************
/****************************************************************************
* ��������: ADC1Init_Base
* ��������: ADC1��ʼ������
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��:
****************************************************************************/
void ADC1Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  ADC_InitTypeDef ADC_InitStructure;
	/*ʹ���õ���ʱ����Դ*/
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA
//	 						|RCC_APB2Periph_GPIOB
//	 						|RCC_APB2Periph_GPIOC,
//							ENABLE);
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC���ʱ�䲻�ܳ���14M
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����    


	/* ADC1 ��Ӧ��1���ɼ�ͨ������ ------------------------------------*/
//   /*����˿�A����*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* ADC1 ���� ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;	                    //����ת����ͨ����
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 ����ģʽͨ������ */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

//    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* ����ADC1 */
    ADC_Cmd(ADC1, ENABLE);


	// ������ADC�Զ�У׼����������ִ��һ�Σ���֤����
    /* ʹ��ADC1��λУ��*/
    ADC_ResetCalibration(ADC1);
    /* */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* ��ʼADC1��λУ�� */
    ADC_StartCalibration(ADC1);
    /* �����Ƿ�У����� */
    while(ADC_GetCalibrationStatus(ADC1));
	// ADC�Զ�У׼����---------------
//
//    /* ����ADC1ת�� */
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/****************************************************************************
* ��������: DMAADCInit_Base
* ��������: ADC1��ʼ������
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��:
****************************************************************************/
void DMAADCInit_Base(void)
{
    DMA_InitTypeDef DMA_InitStructure;

	/*DMA1ͨ�������� ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCConvertedValue[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;		  //dma���䷽����
    DMA_InitStructure.DMA_BufferSize = 80;					  //����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //����DMA���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		   //����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	 //ת����������ݴ�С
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;					  //����DMA�Ĵ���ģʽ���������ϵ�ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* ʹ��DMA1ͨ����*/
    DMA_Cmd(DMA1_Channel1, ENABLE);

}
/****************************************************************************
* ��������: ADCfliter()
* ��������: ADC1�˲��
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��:
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
* �������ƣ�Init_PlayStatus
* ������������ʼ������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************/
void Init_PlayStatus(void)
{
	VM8978Status.complete = 1;
	VM8978Status.mode = 0;
	VM8978Status.PlayStatus = 0;
//  InFiniteListCurPlay = InFiniteListHead;
}
/*******************************************************************************
* �������ƣ�Init_FreeList
* ������������ʼ������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************/
void Init_FreeList(void)
{
	u8 i;
	_PlayICBBuf *Pt1 = (_PlayICBBuf *)0;
	_PlayICBBuf *Pt2 = (_PlayICBBuf *)0;

	InFiniteListHead = (_PlayICBBuf *)0;//�������б�ͷָ�룬
	InFiniteListTail =  (_PlayICBBuf *)0;//�������б�βָ��

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
	InFiniteListCurPlay = InFiniteListHead;  //��ʼ����ǰ����ָ��Ϊͷָ��
}

/*******************************************************************************
* �������ƣ�PushNode
* ��������: ��������������ѹ��һ��������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
* �������ƣ�DelNode
* ��������: ɾ�������������ض���ַ�ڵ�
* ��ڲ�������
* ���ڲ�������
* ʹ��˵�������ýڵ㻹����buffer��ĩ��
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
			   //--ɾ��ͷԪ�ؽڵ�
         InFiniteListHead = InFiniteListTail;
         InFiniteListCurPlay = InFiniteListHead;//����ָ�����ͷָ��
         InFiniteListNullTail->next = ptmp;   //ɾ��ָ����ڿ�Ϊָ�����
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
						if(InFiniteListCurPlay==ptmp)  //Ҫɾ����ָ��Ϊ��ǰ����ָ��
						{
								if(ptmp==InFiniteListTail->prev) //��ǰ����ָ��Ϊβָ���ǰһ��ָ��
								{
										InFiniteListCurPlay=InFiniteListHead;//��ǰ����ָ������Ϊͷָ��
								}
								else
								{
											InFiniteListCurPlay=ptmp->next;
								}
						}
						//��ɾ���Ľڵ�����
						(*ptmp).segaddr = 0;
						(*ptmp).playinterval = 0;
						(*ptmp).playtimes = 0;
						(*ptmp).playtype = 0;
						(*ptmp).playprior = 0;
			
            if(ptmp==InFiniteListHead) //��ɾ���Ľڵ�Ϊ��һ���ڵ�
            {
                ptmp->next->prev=NULL;  //�ڶ����ڵ���Ϊͷ�ڵ�
                InFiniteListHead=ptmp->next;

            }
            else //��ɾ���Ľڵ㴦��ͷָ���Ϊָ��֮��
            {
                 ptmp->next->prev=ptmp->prev;  //��ɾ���ڵ�ǰ��Ľڵ�ͺ���Ľڵ㴮������
                 ptmp->prev->next=ptmp->next;
            }
						//�黹ɾ���ڵ�鵽βָ��֮��
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
* �������ƣ�Del_AllNode
* ��������: ɾ�����޴����������е����нڵ㣬���ڵ�����
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
