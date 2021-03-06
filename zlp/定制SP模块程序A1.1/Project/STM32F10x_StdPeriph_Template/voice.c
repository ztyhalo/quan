
#include "include.h"

//******************************************* 宏定义***************************
#define Null (_PlayICBBuf *)0

                                            
//******************************************常量定义***************************


//*****************************************变量定义****************************
_PlayICBBuf PlayIBC[INFINITEPLAYLISTDEPTH];//空闲播放列表

_PlayICBBuf *FiniteListHead = (_PlayICBBuf *)0;//有限次待播放列表头指针，
_PlayICBBuf *FiniteListTail =  (_PlayICBBuf *)0;//有限次待播放列表尾指针
_PlayICBBuf *FiniteListCurPlay =  (_PlayICBBuf *)0;//有限次当前播放语音块指针

_PlayICBBuf FiniteList;//有限次播放变量
_PlayICBBuf FiniteListBuf;//有限次播放变量缓存

_PlayICBBuf *InFiniteListHead = (_PlayICBBuf *)0;//无限次待播放列表头指针，
_PlayICBBuf *InFiniteListTail =  (_PlayICBBuf *)0;//无限次待播放列表尾指针
_PlayICBBuf *InFiniteListCurPlay =  (_PlayICBBuf *)0;//无限次当前播放语音块指针
_PlayICBBuf *InFiniteListNullTail = (_PlayICBBuf *)0;//无限次空节点尾指针


u8 FiniteListNodeNum = 0;//有限次缓冲区节点数目
u8 InFiniteListNodeNum = 0;//无线次缓冲区节点数目

u16 ADCConvertedValue[ADCBUFFER] = {0};

_VM8978Status VM8978Status;

_SoftTimer SoftTimer;
_VolumeCtr VolumeCtr;

OS_EVENT *MsgQueue;
void     *MsgQueueTbl[MSG_QUEUE_SIZE];


//******************************************函数声明***************************
void Init_FreeList(void);
//******************************************函数定义***************************
/****************************************************************************
* 函数名称: ADC1Init_Base
* 功能描述: ADC1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void ADC1Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  ADC_InitTypeDef ADC_InitStructure;
	/*使能用到的时钟资源*/
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA
//	 						|RCC_APB2Periph_GPIOB
//	 						|RCC_APB2Periph_GPIOC,
//							ENABLE);
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC最大时间不能超过14M
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输    


	/* ADC1 对应的1个采集通道配置 ------------------------------------*/
//   /*输入端口A配置*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* ADC1 配置 ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;	                    //用于转换的通道数
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 规则模式通道配置 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

//    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* 启动ADC1 */
    ADC_Cmd(ADC1, ENABLE);


	// 下面是ADC自动校准，开机后需执行一次，保证精度
    /* 使能ADC1复位校正*/
    ADC_ResetCalibration(ADC1);
    /* */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* 开始ADC1复位校正 */
    ADC_StartCalibration(ADC1);
    /* 检验是否校正完毕 */
    while(ADC_GetCalibrationStatus(ADC1));
	// ADC自动校准结束---------------
//
//    /* 开启ADC1转换 */
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/****************************************************************************
* 函数名称: DMAADCInit_Base
* 功能描述: ADC1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void DMAADCInit_Base(void)
{
    DMA_InitTypeDef DMA_InitStructure;

	/*DMA1通道１配置 ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCConvertedValue[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;		  //dma传输方向单向
    DMA_InitStructure.DMA_BufferSize = 80;					  //设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //设置DMA的外设递增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		   //设置DMA的内存递增模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	 //转换结果的数据大小
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;					  //设置DMA的传输模式：连续不断的循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* 使能DMA1通道１*/
    DMA_Cmd(DMA1_Channel1, ENABLE);

}
/****************************************************************************
* 函数名称: ADCfliter()
* 功能描述: ADC1滤波�
* 入口参数: 无
* 出口参数: 无
* 使用说明:
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
* 函数名称：Init_PlayStatus
* 功能描述：初始化播放
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************/
void Init_PlayStatus(void)
{
	VM8978Status.complete = 1;
	VM8978Status.mode = 0;
	VM8978Status.PlayStatus = 0;
//  InFiniteListCurPlay = InFiniteListHead;
}
/*******************************************************************************
* 函数名称：Init_FreeList
* 功能描述：初始化链表
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************/
void Init_FreeList(void)
{
	u8 i;
	_PlayICBBuf *Pt1 = (_PlayICBBuf *)0;
	_PlayICBBuf *Pt2 = (_PlayICBBuf *)0;

	InFiniteListHead = (_PlayICBBuf *)0;//待播放列表头指针，
	InFiniteListTail =  (_PlayICBBuf *)0;//待播放列表尾指针

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
	InFiniteListCurPlay = InFiniteListHead;  //初始化当前播放指针为头指针
}

/*******************************************************************************
* 函数名称：PushNode
* 功能描述: 向语音播放链表压入一个语音块
* 入口参数：无
* 出口参数：无
* 使用说明：无
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
* 函数名称：DelNode
* 功能描述: 删除语音缓冲区特定地址节点
* 入口参数：无
* 出口参数：无
* 使用说明：将该节点还到空buffer的末端
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
			   //--删除头元素节点
         InFiniteListHead = InFiniteListTail;
         InFiniteListCurPlay = InFiniteListHead;//播放指针跟随头指针
         InFiniteListNullTail->next = ptmp;   //删除指针放在空为指针后面
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
						if(InFiniteListCurPlay==ptmp)  //要删除的指针为当前播放指针
						{
								if(ptmp==InFiniteListTail->prev) //当前播放指针为尾指针的前一个指针
								{
										InFiniteListCurPlay=InFiniteListHead;//当前播放指针设置为头指针
								}
								else
								{
											InFiniteListCurPlay=ptmp->next;
								}
						}
						//待删除的节点清零
						(*ptmp).segaddr = 0;
						(*ptmp).playinterval = 0;
						(*ptmp).playtimes = 0;
						(*ptmp).playtype = 0;
						(*ptmp).playprior = 0;
			
            if(ptmp==InFiniteListHead) //待删除的节点为第一个节点
            {
                ptmp->next->prev=NULL;  //第二个节点设为头节点
                InFiniteListHead=ptmp->next;

            }
            else //待删除的节点处于头指针和为指针之间
            {
                 ptmp->next->prev=ptmp->prev;  //待删除节点前面的节点和后面的节点串连起来
                 ptmp->prev->next=ptmp->next;
            }
						//归还删除节点块到尾指针之后
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
* 函数名称：Del_AllNode
* 功能描述: 删除无限次语音链表中的所有节点，即节点清零
* 入口参数：无
* 出口参数：无
* 使用说明：无
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
