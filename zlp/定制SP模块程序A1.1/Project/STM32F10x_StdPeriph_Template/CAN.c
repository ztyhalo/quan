
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
//----------------------------------- CAN����buffer -----------------------------------------
CAN_FIFOStruct CANRxBuf =
{
    0,                                      // wr
    0,                                      // rd
    0,                                      // num
};

//----------------------------------- CAN����buffer -----------------------------------------
CAN_FIFOStruct CANTxBuf =
{
    0,                                      // wr
    0,                                      // rd
    0,                                      // num
};
//----------------------------------- CAN�ط���buffer -----------------------------------------
CAN_FIFOStruct CANRTxBuf =
{
	0,
	0,
	0,
};
_Playlistinfo Playlistinfo ;
//--------------����������---------------------------------------------------
/*��ʼ��CAN����*/
void  CAN1_GPIO_Config(void);
/*��ʼ��CAN�ж�*/
void  CAN1_NVIC_Config(void);
/*CANģʽ����*/
void  CAN1_Mode_Config(void);
/*CAN����������*/
void  CAN1_Filter_Config(void);

//---------------����ʵ�嶨����----------------------------------------------
/****************************************************************************
* ��������: CAN1Init
* ��������: CAN ��ʼ������
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��:
****************************************************************************/
void CAN1Init(void)
{
  CAN1_GPIO_Config();
	CAN1_NVIC_Config();
	CAN1_Mode_Config();
	CAN1_Filter_Config(); 
}

/*******************************************************************************************
* �������ƣ� CAN1_GPIO_Config()
* ����������  ��������CAN�շ����ţ�CAN1_TX(PA.12),CAN1_RX(PA.11)����        
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������ʼ������
*******************************************************************************************/
void  CAN1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(CAN1_RCC, ENABLE);  		// ����CAN1��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);		//��������ʱ�ӹ���

	/* Configure CAN1 pin: RX PA11*/									               // PA11
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // ��������
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
    
    /* Configure CAN1 pin: TX PA12*/									               // PA12
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // �����������
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   	GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
}
/*******************************************************************************************
* �������ƣ� CAN1_NVIC_Config()
* ���������� ��CAN1�����ȼ���������         
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������ʼ������
*******************************************************************************************/
void  CAN1_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
	/*�ж�����*/
	
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;	               //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //�����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//��ʹ���ж����ȼ�Ƕ�ס���ΪSysTick���ж����ȼ�Ϊ0x0f
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//0������SysTick
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

}
/*******************************************************************************************
* �������ƣ� CAN1_Mode_Config()
* ���������� CAN1ͨ�Ų�������      
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������ʼ������
*******************************************************************************************/
void  CAN1_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN1ͨ�Ų�������**********************************/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
    //CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  �Զ����߹���
	///////////////////////////////////////////
	CAN_InitStructure.CAN_ABOM=ENABLE;				  //�Զ����߹���,���ABOMλΪ��1����bxCAN��������״̬�󣬾��Զ������ָ����̡�
													  //���ABOMλΪ��0�����������������bxCAN����Ȼ�����˳���ʼ��ģʽ�����ָ����̲ű�������
	/////////////////////////////////////// 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
    CAN_InitStructure.CAN_TXFP=ENABLE;			   //MCR-TXFP  ����FIFO���ȼ� ENABLE-���ȼ��ɷ��������˳�����
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;		   //BTR-TS1 ʱ���1 ռ����8��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		   //BTR-TS1 ʱ���2 ռ����7��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_Prescaler =24;   //������Ϊ100k/bps
	  CAN_Init(CAN1, &CAN_InitStructure);
/******************************************************************************************
*�����ʼ��㹫ʽ
*				          Pclk1
* BRR = -----------------------------------------
*		(CAN_Prescaler + 1)*(1 + CAN_BS1 + CAN_BS2)
* 
*CAN_BS1һ��ȡ��8
*CAN_BS1һ��ȡ��7
******************************************************************************************/
}
/*******************************************************************************************
* �������ƣ� CAN1_Filter_Config()
* ���������� CAN1��������ʼ����������      
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������ʼ������
*******************************************************************************************/
void  CAN1_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
/* ����CAN ������ ����14��,����Ϊ��һ�������� */
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

	/* ����CAN ������ ����14��,����Ϊ�ڶ��������� */
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

	/* ����CAN ������ ����14��,����Ϊ������������ */
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
	
		/* ����CAN ������ ����14��,����Ϊ������������ */
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

	
    /*CAN1����Ӧ�ð��CAN1�ڣ�ͨ���ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP1|CAN_IT_FMP0, ENABLE);
	/*CAN1�����ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);	
}
/****************************************************************************
* ��������: FIFOInit()
* ��������: ��ʼ��FIFO
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: �ú���û��ʵ��,��Ҫ�ڶ���FIFO����ʱ��ʼ��
****************************************************************************/
void CAN_FIFOFlush(CAN_FIFOStruct *pBuf)
{
    pBuf->wr  = 0;
    pBuf->rd  = 0;
    pBuf->num = 0;
}


/****************************************************************************
* ��������: FIFOGetOne()
* ��������: ��FIFO�з���һ��(��)����,�ɹ�����0
* ��ڲ���: FIFO�ṹ��ַ,���������ݵĵ�ַ
* ���ڲ���:
* ʹ��˵��:
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
* ��������: CAN_FIFOGetOne()
* ��������: ��FIFO��ȡ��һ��(��)����,�ɹ�����0
* ��ڲ���: FIFO�ṹ��ַ,ȡ������Ҫ�ŵ��ĵ�ַ
* ���ڲ���:
* ʹ��˵��:
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
* ��������: CAN_FIFOCopyOne()
* ��������: ��FIFO��ȡ��һ��(��)����,�ɹ�����0
* ��ڲ���: FIFO�ṹ��ַ,ȡ������Ҫ�ŵ��ĵ�ַ
* ���ڲ���:
* ʹ��˵��:
****************************************************************************/
void CAN_FIFOCopyOne(CAN_FIFOStruct *pBuf, CAN_FrameStruct *q)
{
    if(pBuf->num > 0){
        memcpy((s8*)q, (cs8*)&pBuf->buf[pBuf->rd], sizeof(CAN_FrameStruct));
    }
}
/****************************************************************************
* ��������: CAN_FIFODeleteOne()
* ��������: ��FIFO��ɾ��һ��(��)����,�ɹ�����0
* ��ڲ���: FIFO�ṹ��ַ��
* ���ڲ���:
* ʹ��˵��:
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
* ��������: FIFOPeek
* ��������: ͵��һ��FIFO�������м�������(��)
* ��ڲ���: ��
* ���ڲ���: FIFO��ʵ��ĸ���
* ʹ��˵��:
****************************************************************************/
u8 CAN_FIFOPeek(CAN_FIFOStruct *pBuf)
{
    return pBuf->num;
}


/****************************************************************************
* ��������: CAN_RcvDataInt
* ��������: ���ж��н���CAN����
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: �ڸ����ȼ��ж��е���
****************************************************************************/
void CAN_RcvDataInt(void)
{
	u8 i,j,framenum;
	CanRxMsg RxMessage;

	if((framenum = CAN_MessagePending(CAN1,CAN_FIFO0)) != 0){
		for(i = 0;i < framenum;i++){
			CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);			 //��ȡFIFO0�ڵı���		
			if(CANRxBuf.num < CAN_BUFFER_DEPTH){		 //�����ݿ��������ջ�����
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
			CANRxBufFilled1=1;    //CAN���ջ���1������
        }					
		}
		CAN_FIFORelease(CAN1,CAN_FIFO0);		                 //�򿪽��ջ�����0�����Խ���������
	}

	if((framenum = CAN_MessagePending(CAN1,CAN_FIFO1)) != 0){
		for(i = 0;i < framenum;i++){
			
			CAN_Receive(CAN1,CAN_FIFO1, &RxMessage);			 //��ȡFIFO1�ڵı���
			if(CANRxBuf.num < CAN_BUFFER_DEPTH){		 //�����ݿ��������ջ�����
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
				CANRxBufFilled2=1;    //CAN���ջ���2����
			}	
		}
		CAN_FIFORelease(CAN1,CAN_FIFO1);		                 //�򿪽��ջ�����1�����Խ���������
	}
}
/****************************************************************************
* ��������:	CAN_SendData
* ��������: ���ж�ģʽ�·���CAN����֡
* ��ڲ���: ��
* ���ڲ���: 0--����ʧ��,1--���ͳɹ�
* ʹ��˵��: �����жϺ�����ʹ��
****************************************************************************/
void CAN_SendData(CAN_FrameStruct *q)
{
	u16 i = 0;
	u8 TransmitMailbox = 0;
	CanTxMsg TxMessage;
        
	if((CAN_GetFlagStatus(CAN1,CAN_FLAG_EPV) == SET)||(CAN_GetFlagStatus(CAN1,CAN_FLAG_BOF) == SET)){
		CAN1Init();                                                        // ������ջ��߷��ʹ���Ĵ���ֵ�������ֵ  ��ʼ��CANģ��
	}

	if((*q).msgflg == (CAN_ID_STD|CAN_RTR_DATA)){			                       //Ϊ��׼֡
		TxMessage.StdId = (*q).id;
		TxMessage.ExtId = 0;
		TxMessage.IDE = CAN_ID_STD;
	}
	else{																	   //Ϊ��չ֡
		TxMessage.StdId = 0;										                               
		TxMessage.ExtId = (*q).id;
		TxMessage.IDE = CAN_ID_EXT;											  
	}
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = (*q).dlc;
	for(i = 0;i < (*q).dlc;i++){
		TxMessage.Data[i] = (*q).dat[i];
	}

	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);                                //�������������

	if(TransmitMailbox == CAN_NO_MB){							               //û�п�������ʱ�Ĵ���
		for(i = 0;i < 10;i++){												   //�ȴ�100ms
			OSTimeDly(1);
			if((CAN_TransmitStatus(CAN1,TransmitMailbox) != CAN_NO_MB)){			   //�ڵȴ��ڼ��п�������
				TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);					   //���·��͸�֡
				break;
			}
		}
		if(i>=10){
			CAN1Init();													   //����100ms����û�п������䣬���ʼ��
			TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);						   //��ʼ�������·��͸�֡
		}
	}
  	i = 0;
  	while(CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK){                     //��ʱ�ȴ�20ms
    	i++;
		OSTimeDly(1);
		if(i > 2){
			break;
		}
  	}
//	CANTxBuf.buf->sndflg = 1;                                                  //�ѷ��ͱ�ʶ��Ч��
}
/****************************************************************************
* ��������: CAN_PlaySoundAnalyse
* ��������: ��CAN���������н���Ҫ���ŵ��ļ�
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: �
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
// 	static u16 StopInstruct2[2]={0}; //����������ʱʹ��
// 	static u16 StopInstruct3[2]={0}; //����������ʱʹ��
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
			tmpSegment.playprior = 0;//0ʱΪFIFOģʽ����0Ϊ���ȼ�����ģʽ(���ȼ�ȡֵ��Χ��1~255)������Խ�����ȼ�Խ��

			if(tmpSegment.playtype != 0)//�ظ�����ѹ������
			{
				err=CAN_RxDateAnalyse(&tmpSegment);
				if(err==0)
				{
					OS_ENTER_CRITICAL();
					PushNode(&tmpSegment);									   //ѹ������
					OS_EXIT_CRITICAL();
				}
				else
				{
					NoSameDate=1;       //����������֮ǰ����ͬ�ģ���ѹ������
				}
			}
			else//���޴β�������
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
					NotPushBs=1;//���±���ʽ����λ����������481 1E/1F 01��������ϵ�SPֻ��1�Σ�����ڵ�һ�黹δ����ʱ�����ڶ�����������ε�
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
			CAN_TxFrame(FRAMEID_LOWTOUP_PLAYACK ,TempArray,2);					   //&TempArray[0]   Ӧ��֡
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
			if((TempArray[0]==0x00)&&(TempArray[1]==0x00))//��λ���ᷢ481 00 00��ֹͣ�����������ڱ�����̧��ʱ��
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
            CAN_TxFrame(FRAMEID_ONLINECHECKACK ,TempArray,0);					   //&TempArray[0]   Ӧ��֡
        }
    }
	if((*CanMsg).id ==FRAMEID_UPTOLOW_RESET) //�ӵ�0X5FFʱ����λ
	{
		wav_play_stop(); 
		sysreset();
	}
}
/*******************************************************************************
* �������ƣ�CAN_TxFrame
* ����������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵����
********************************************************************************/
void CAN_TxFrame(u32 AnswerCode,u8 *p,u8 Dlc)
{
#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
#endif

    CAN_FrameStruct CAN_Msg_Tx ={0};
	
	CAN_Msg_Tx.msgflg = CAN_ID_STD|CAN_RTR_DATA;

	if(AnswerCode == FRAMEID_LOWTOUP_PLAYACK){								//����Ӧ��֡
		CAN_Msg_Tx.id = FRAMEID_LOWTOUP_PLAYACK;
		CAN_Msg_Tx.dlc = 2;
		CAN_Msg_Tx.dat[0] = p[0];//*p
        CAN_Msg_Tx.dat[1] = p[1];//*(p+1)
	}
    else if(AnswerCode == FRAMEID_LOWTOUP_STOPACK ){							//����Ӧ��֡

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
* ��������: CAN_RxDateAnalyse
* ��������: �ж�CAN���������Ƿ����ظ���
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: �
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
* ��������: CAN_RxStopDateAnalyse
* ��������: �ж�CAN���յ���ֹͣ�����Ƿ������޴β���������
* ��ڲ���: ��
* ���ڲ���: ��
* ʹ��˵��: ?
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


