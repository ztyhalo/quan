
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
//----------------------------------- CAN接收buffer -----------------------------------------
CAN_FIFOStruct CANRxBuf =
{
    0,                                      // wr
    0,                                      // rd
    0,                                      // num
};

//----------------------------------- CAN发送buffer -----------------------------------------
CAN_FIFOStruct CANTxBuf =
{
    0,                                      // wr
    0,                                      // rd
    0,                                      // num
};
//----------------------------------- CAN重发送buffer -----------------------------------------
CAN_FIFOStruct CANRTxBuf =
{
	0,
	0,
	0,
};
_Playlistinfo Playlistinfo ;
//--------------函数声明区---------------------------------------------------
/*初始化CAN引脚*/
void  CAN1_GPIO_Config(void);
/*初始化CAN中断*/
void  CAN1_NVIC_Config(void);
/*CAN模式配置*/
void  CAN1_Mode_Config(void);
/*CAN过滤器配置*/
void  CAN1_Filter_Config(void);

//---------------函数实体定义区----------------------------------------------
/****************************************************************************
* 函数名称: CAN1Init
* 功能描述: CAN 初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void CAN1Init(void)
{
  CAN1_GPIO_Config();
	CAN1_NVIC_Config();
	CAN1_Mode_Config();
	CAN1_Filter_Config(); 
}

/*******************************************************************************************
* 函数名称： CAN1_GPIO_Config()
* 功能描述：  用于配置CAN收发引脚（CAN1_TX(PA.12),CAN1_RX(PA.11)）；        
* 入口参数：无
* 出口参数：无
* 使用说明：初始化调用
*******************************************************************************************/
void  CAN1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(CAN1_RCC, ENABLE);  		// 开启CAN1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);		//开启复用时钟功能

	/* Configure CAN1 pin: RX PA11*/									               // PA11
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
    
    /* Configure CAN1 pin: TX PA12*/									               // PA12
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   	GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
}
/*******************************************************************************************
* 函数名称： CAN1_NVIC_Config()
* 功能描述： 对CAN1的优先级进行设置         
* 入口参数：无
* 出口参数：无
* 使用说明：初始化调用
*******************************************************************************************/
void  CAN1_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
	/*中断设置*/
	
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;	               //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//不使用中断优先级嵌套。因为SysTick的中断优先级为0x0f
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//0级用于SysTick
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

}
/*******************************************************************************************
* 函数名称： CAN1_Mode_Config()
* 功能描述： CAN1通信参数设置      
* 入口参数：无
* 出口参数：无
* 使用说明：初始化调用
*******************************************************************************************/
void  CAN1_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN1通信参数设置**********************************/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
    //CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  自动离线管理
	///////////////////////////////////////////
	CAN_InitStructure.CAN_ABOM=ENABLE;				  //自动离线管理,如果ABOM位为’1’，bxCAN进入离线状态后，就自动开启恢复过程。
													  //如果ABOM位为’0’，软件必须先请求bxCAN进入然后再退出初始化模式，随后恢复过程才被开启。
	/////////////////////////////////////// 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
    CAN_InitStructure.CAN_TXFP=ENABLE;			   //MCR-TXFP  发送FIFO优先级 ENABLE-优先级由发送请求的顺序决定
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;		   //BTR-TS1 时间段1 占用了8个时间单元
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		   //BTR-TS1 时间段2 占用了7个时间单元
    CAN_InitStructure.CAN_Prescaler =24;   //波特率为100k/bps
	  CAN_Init(CAN1, &CAN_InitStructure);
/******************************************************************************************
*波特率计算公式
*				          Pclk1
* BRR = -----------------------------------------
*		(CAN_Prescaler + 1)*(1 + CAN_BS1 + CAN_BS2)
* 
*CAN_BS1一般取　8
*CAN_BS1一般取　7
******************************************************************************************/
}
/*******************************************************************************************
* 函数名称： CAN1_Filter_Config()
* 功能描述： CAN1过滤器初始化参数设置      
* 入口参数：无
* 出口参数：无
* 使用说明：初始化调用
*******************************************************************************************/
void  CAN1_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
/* 配置CAN 过滤器 共有14个,下面为第一个的配置 */
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

	/* 配置CAN 过滤器 共有14个,下面为第二个的配置 */
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

	/* 配置CAN 过滤器 共有14个,下面为第三个的配置 */
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
	
		/* 配置CAN 过滤器 共有14个,下面为第三个的配置 */
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

	
    /*CAN1（对应该板的CAN1口）通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP1|CAN_IT_FMP0, ENABLE);
	/*CAN1发送中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);	
}
/****************************************************************************
* 函数名称: FIFOInit()
* 功能描述: 初始化FIFO
* 入口参数: 无
* 出口参数: 无
* 使用说明: 该函数没有实现,需要在定义FIFO变量时初始化
****************************************************************************/
void CAN_FIFOFlush(CAN_FIFOStruct *pBuf)
{
    pBuf->wr  = 0;
    pBuf->rd  = 0;
    pBuf->num = 0;
}


/****************************************************************************
* 函数名称: FIFOGetOne()
* 功能描述: 给FIFO中放入一个(包)数据,成功返回0
* 入口参数: FIFO结构地址,待放入数据的地址
* 出口参数:
* 使用说明:
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
* 函数名称: CAN_FIFOGetOne()
* 功能描述: 从FIFO中取出一个(包)数据,成功返回0
* 入口参数: FIFO结构地址,取出数据要放到的地址
* 出口参数:
* 使用说明:
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
* 函数名称: CAN_FIFOCopyOne()
* 功能描述: 从FIFO中取出一个(包)数据,成功返回0
* 入口参数: FIFO结构地址,取出数据要放到的地址
* 出口参数:
* 使用说明:
****************************************************************************/
void CAN_FIFOCopyOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q)
{
    if(pBuf->num > 0){
        memcpy((s8*)q, (cs8*)&pBuf->buf[pBuf->rd], sizeof(CAN_FrameStruct));
    }
}
/****************************************************************************
* 函数名称: CAN_FIFODeleteOne()
* 功能描述: 从FIFO中删除一个(包)数据,成功返回0
* 入口参数: FIFO结构地址。
* 出口参数:
* 使用说明:
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
* 函数名称: FIFOPeek
* 功能描述: 偷看一眼FIFO中现在有几个数据(包)
* 入口参数: 无
* 出口参数: FIFO中实体的个数
* 使用说明:
****************************************************************************/
u8 CAN_FIFOPeek(CAN_FIFOStruct *pBuf)
{
    return pBuf->num;
}


/****************************************************************************
* 函数名称: CAN_RcvDataInt
* 功能描述: 从中断中接收CAN数据
* 入口参数: 无
* 出口参数: 无
* 使用说明: 在高优先级中断中调用
****************************************************************************/
void CAN_RcvDataInt(void)
{
	u8 i,j,framenum;
	CanRxMsg RxMessage;

	if((framenum = CAN_MessagePending(CAN1,CAN_FIFO0)) != 0){
		for(i = 0;i < framenum;i++){
			CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);			 //读取FIFO0内的报文		
			if(CANRxBuf.num < CAN_BUFFER_DEPTH){		 //将数据拷贝到接收缓冲中
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
			CANRxBufFilled1=1;    //CAN接收缓存1已填满
        }					
		}
		CAN_FIFORelease(CAN1,CAN_FIFO0);		                 //打开接收缓冲区0，可以接收新数据
	}

	if((framenum = CAN_MessagePending(CAN1,CAN_FIFO1)) != 0){
		for(i = 0;i < framenum;i++){
			
			CAN_Receive(CAN1,CAN_FIFO1, &RxMessage);			 //读取FIFO1内的报文
			if(CANRxBuf.num < CAN_BUFFER_DEPTH){		 //将数据拷贝到接收缓冲中
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
				CANRxBufFilled2=1;    //CAN接收缓存2已满
			}	
		}
		CAN_FIFORelease(CAN1,CAN_FIFO1);		                 //打开接收缓冲区1，可以接收新数据
	}
}
/****************************************************************************
* 函数名称:	CAN_SendData
* 功能描述: 非中断模式下发送CAN数据帧
* 入口参数: 无
* 出口参数: 0--发送失败,1--发送成功
* 使用说明: 不在中断函数下使用
****************************************************************************/
void CAN_SendData(CAN_FrameStruct *q)
{
	u16 i = 0;
	u8 TransmitMailbox = 0;
	CanTxMsg TxMessage;
        
	if((CAN_GetFlagStatus(CAN1,CAN_FLAG_EPV) == SET)||(CAN_GetFlagStatus(CAN1,CAN_FLAG_BOF) == SET)){
		CAN1Init();                                                        // 如果接收或者发送错误寄存器值超过最大值  初始化CAN模块
	}

	if((*q).msgflg == (CAN_ID_STD|CAN_RTR_DATA)){			                       //为标准帧
		TxMessage.StdId = (*q).id;
		TxMessage.ExtId = 0;
		TxMessage.IDE = CAN_ID_STD;
	}
	else{																	   //为扩展帧
		TxMessage.StdId = 0;										                               
		TxMessage.ExtId = (*q).id;
		TxMessage.IDE = CAN_ID_EXT;											  
	}
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = (*q).dlc;
	for(i = 0;i < (*q).dlc;i++){
		TxMessage.Data[i] = (*q).dat[i];
	}

	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);                                //发送邮箱的数量

	if(TransmitMailbox == CAN_NO_MB){							               //没有空闲邮箱时的处理
		for(i = 0;i < 10;i++){												   //等待100ms
			OSTimeDly(1);
			if((CAN_TransmitStatus(CAN1,TransmitMailbox) != CAN_NO_MB)){			   //在等待期间有空闲邮箱
				TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);					   //重新发送该帧
				break;
			}
		}
		if(i>=10){
			CAN1Init();													   //超过100ms依旧没有空闲邮箱，则初始化
			TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);						   //初始化后重新发送该帧
		}
	}
  	i = 0;
  	while(CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK){                     //超时等待20ms
    	i++;
		OSTimeDly(1);
		if(i > 2){
			break;
		}
  	}
//	CANTxBuf.buf->sndflg = 1;                                                  //已发送标识有效。
}
/****************************************************************************
* 函数名称: CAN_PlaySoundAnalyse
* 功能描述: 从CAN接收数据中解析要播放的文件
* 入口参数: 无
* 出口参数: 无
* 使用说明: �
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
// 	static u16 StopInstruct2[2]={0}; //进参数设置时使用
// 	static u16 StopInstruct3[2]={0}; //进参数设置时使用
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
			tmpSegment.playprior = 0;//0时为FIFO模式，非0为优先级放音模式(优先级取值范围：1~255)，数字越大优先级越高

			if(tmpSegment.playtype != 0)//重复播报压入链表
			{
				err=CAN_RxDateAnalyse(&tmpSegment);
				if(err==0)
				{
					OS_ENTER_CRITICAL();
					PushNode(&tmpSegment);									   //压入链表
					OS_EXIT_CRITICAL();
				}
				else
				{
					NoSameDate=1;       //接收数据与之前有相同的，不压入链表
				}
			}
			else//有限次播报拷贝
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
					NotPushBs=1;//拍下闭锁式，上位机连发两次481 1E/1F 01命令，根据老的SP只报1次，因此在第一遍还未报完时，将第二遍的语音屏蔽掉
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
			CAN_TxFrame(FRAMEID_LOWTOUP_PLAYACK ,TempArray,2);					   //&TempArray[0]   应答帧
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
			if((TempArray[0]==0x00)&&(TempArray[1]==0x00))//上位机会发481 00 00来停止语音播报（在闭锁被抬起时）
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
            CAN_TxFrame(FRAMEID_ONLINECHECKACK ,TempArray,0);					   //&TempArray[0]   应答帧
        }
    }
	if((*CanMsg).id ==FRAMEID_UPTOLOW_RESET) //接到0X5FF时，复位
	{
		wav_play_stop(); 
		sysreset();
	}
}
/*******************************************************************************
* 函数名称：CAN_TxFrame
* 功能描述：
* 入口参数：无
* 出口参数：无
* 使用说明：
********************************************************************************/
void CAN_TxFrame(u32 AnswerCode,u8 *p,u8 Dlc)
{
#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
#endif

    CAN_FrameStruct CAN_Msg_Tx ={0};
	
	CAN_Msg_Tx.msgflg = CAN_ID_STD|CAN_RTR_DATA;

	if(AnswerCode == FRAMEID_LOWTOUP_PLAYACK){								//播放应答帧
		CAN_Msg_Tx.id = FRAMEID_LOWTOUP_PLAYACK;
		CAN_Msg_Tx.dlc = 2;
		CAN_Msg_Tx.dat[0] = p[0];//*p
        CAN_Msg_Tx.dat[1] = p[1];//*(p+1)
	}
    else if(AnswerCode == FRAMEID_LOWTOUP_STOPACK ){							//设置应答帧

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
* 函数名称: CAN_RxDateAnalyse
* 功能描述: 判断CAN接收数据是否有重复的
* 入口参数: 无
* 出口参数: 无
* 使用说明: �
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
* 函数名称: CAN_RxStopDateAnalyse
* 功能描述: 判断CAN接收到的停止数据是否在有限次播放链表中
* 入口参数: 无
* 出口参数: 无
* 使用说明: ?
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


