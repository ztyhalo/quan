#include "tag.h"
#include "anchor.h"
#include "address.h"
#include "BEB.h"
#include "crc.h"
#if defined(USE_OLED)
	#include "oled.h"
#endif

#define  FLASH_INIT_VALUE					0xFFFFFFFFFFFFFFFF   //FLASH��ʼ��ֵ


Tag_Config_s sTagConfig;
extern DWT_AddrStatus_s 	sDWT;

OS_EVENT  *TagCommSem;			//��ǩͨѶ�ź���
OS_EVENT  *ID_ConfigSem;		//ID�����ź���

uint16_t WaitForPower = 9000;		//�ϵ�ȴ�ʱ�䣺10s����ֹ�������ʱ����縺�ص���̫�󣬳䲻��ȥ

/*��ǩͨѶ֡����*/
uint8_t TxPollMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 'V', 'E', FRAME_POLL, 0, 0,};							//POLL֡
uint8_t RxRespMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 0xFF, 0xFF, FRAME_RESP, 0x02, 0, 0, 0, 0};	//RESP֡
uint8_t TxFinalMsg[] = {
												0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 'V', 'E', FRAME_FINAL, 
												0x00, 0x00, 0x00, 0x00, 0x00,
												0x00, 0x00, 0x00, 0x00, 0x00,
												0x00, 0x00, 0x00, 0x00, 0x00,
												0, 0
											 };																																								//FINAL֡
uint8_t Tag_CommDec[] = {0x00, 0x80, 0x00, 0xCA, 0xDE, 0x00, 0x00, FRAME_COMM_DEC, 0x00, 0x00};					//ͨѶ����֡

static uint8_t UsartFrameHeader[] = {0x41, 0x8C, 0, 0xFF, 0xFF, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, FRAME_CONFIG_ID};	//����--����֡֡ͷ
static uint8_t UsartFramePayLoad[1024];

static uint8_t RxBuffer[RX_BUF_LEN_MAX];
static uint32_t FrameSeqNum = 0;
static uint32_t TagStatusReg = 0;
											 
static uint64_t PollTxTS;
static uint64_t RespRxTS;
static uint64_t FinalTxTS;

/****************************64Bit FLASH����*************************************
* DWT_WriteToFlash
*  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  
* |     |          |                  |                   |
* |Bit63| Reserved |   Bit47--Bit32   |     Bit31--Bit0   |
* |_ _ _|_ _ _ _ _ |_ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _|
* 
* Bit31--Bit0 :32λCHIP ID
* Bit47--Bit32:16λ�̵�ַ
* Bit59--Bit56:ģ�鹤��ģʽ��0--��ǩ��1--�ӻ�վ��2--����վ��3--CLE
* Bit63:       ģ��̵�ַ��ʶλ��0--δ����̵�ַ��1--�ѷ���̵�ַ
*
* TAG_WriteToFlash
*  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
* |							 |              |              |              |             |
* | Bit63--Bit48 | Bit47--Bit40 | Bit39--Bit32 | Bit31--Bit16 | Bit15--Bit0 |
* |_ _ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _ _ _|
* 
* Bit15--Bit0:  Perm
* Bit31--Bit16: JobNum
* Bit39--Bit32: Age
* Bit47--Bit40: Sex
* Bit63--Bit48: MAC ID
*********************************************************************************/
uint64_t DWT_ReadFromFlash;
uint64_t DWT_WriteToFlash;

uint64_t TAG_ReadFromFlash;
uint64_t TAG_WriteToFlash;

uint16_t CollisionWindow = 1;				//������˳�ͻ����Cw
uint8_t TagExitWorkSpaceTimBegin;		//����ʧ�ܱ�־λ����������Tag�����ж���ʱ��
uint8_t TagExitWorkSpace;						//Tag�뿪�������־λ��0--Tag�ڹ������ڣ�1--Tag�ڹ�������

uint32_t TagExitWorkSpaceTIM;		//Tag�����ж���ʱ��
uint32_t TagCommCycleTIM;				//TagͨѶ���ڼ�ʱ��

uint16_t AncAddr_Center;
uint16_t AncAddr_CommBegin;
uint16_t AncAddr_Comm;

uint8_t TagCommToAncNum;			//����ͨѶ�У���ǩ��Ҫ��֮ͨѶ�Ļ�վ������

uint8_t KeepInConfig = 0;			//���ֱ�ǩ����ģʽ��־��1--��������ģʽ��0--�˳�����ģʽ

uint8_t TagUsartCrcRepeatNum = 0;	//CRC�������

/*******************************************************************************************
* �������ƣ�void Tag_Reset_Blink(void)
* ����������ϵͳ��λ��˸
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void Tag_Reset_Blink(void)
{
	uint8_t i;
	for(i = 0; i < 10; i++)
	{
		HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED3_PIN | LED4_PIN);
		OSTimeDly(100);
	}
}


/*******************************************************************************************
* �������ƣ�UWB_Tag(void *pdata)
* ����������UWB��Ϊ��ǩʱ��ִ�к���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
uint16_t CollisionWindowCnt = 0;

void UWB_Tag(void *pdata)
{
#if defined(USE_OLED)
	char ModelID[5];
#endif
	
	INT8U err;
	uint16_t RNG_Num;
	DWT_SetAddress16(sDWT.ShortAddr);

//	DWT_ConfigEventCounters(ENABLE);			//�����¼�������
	
	WriteToMsg(TxPollMsg, (uint8_t *)&sDWT.ShortAddr, FRAME_SOURCE_ADDR_IDX, 2, 0);			//����Poll����֡����ǩ��ַ
	WriteToMsg(TxFinalMsg, (uint8_t *)&sDWT.ShortAddr, FRAME_SOURCE_ADDR_IDX, 2, 0);		//����Final����֡����ǩ��ַ
	WriteToMsg(Tag_CommDec, (uint8_t *)&sDWT.ShortAddr, FRAME_DEC_TAG_ADDR_IDX, 2, 0);	//����ͨѶ����֡����ǩ��ַ
	
#if defined(USE_OLED)
	sprintf(ModelID, "%04d", sDWT.ShortAddr);
	OLED_ShowString(0, 0, "TagID:", 16);
	OLED_ShowString(48, 0, ModelID, 16);
#endif
	
	ID_ConfigSem = OSSemCreate(0);	//����ID�����ź���
	
	Tag_Reset_Blink();				//ϵͳ��˸ָʾ
	OSTimeDly(WaitForPower);	//��ʱ��������10s
	
	FLASH_ReadBytes(TAG_INFO_ADDR, SHORT_ID_ADDR_OFFECT, (uint8_t *)&DWT_ReadFromFlash, 8);
	if(((DWT_ReadFromFlash >> UWB_16BIT_ADDR_FLAG_IDX) & 0x0001) && (DWT_ReadFromFlash != FLASH_INIT_VALUE))		//MAC ID�����ã������ǩ����
	{
		TagCommSem = OSSemCreate(1);		//������ǩͨѶ�ź���
	}
	else
	{
		TagCommSem = OSSemCreate(1);		//������ǩͨѶ�ź���
		//����˯��ģʽ
	}

	for(;;)
	{
		if((TagExitWorkSpaceTimBegin == 0) || TagExitWorkSpace == 1)		//�����ɹ����߱�ǩ������
		{
//			DWT_EnterSleep();																		//DWM1000��ʼ˯��
			OSSemPend(TagCommSem,0,&err);
//			DWT_SpiCsWakeup(DummyBuffer, DUMMY_BUFFER_LEN);			//����DWT
			TagCommCycleTIM = (TagExitWorkSpace == 0 ? HAL_GetTick() + SYSTEM_COMM_CYCLE_MS : HAL_GetTick() + SYSTEM_COMM_CYCLE_LP_MS);		//ͨѶ���ڼ�ʱ������
		}
		
		HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED4_PIN);
		
		if(ListenChannel() == ChannelFree)	//�����ŵ��Ƿ����
		{
			if(CommDec())											//����ͨѶ��������ȡͨѶȨ
			{
				uint8_t i, j;
				
				if(TagExitWorkSpace == 1)
				{
					TagCommCycleTIM = TagCommCycleTIM + SYSTEM_COMM_CYCLE_MS - SYSTEM_COMM_CYCLE_LP_MS;			//����״̬ת��Ϊ����״̬������ͨѶ����
				}
				
				TagExitWorkSpaceTimBegin = 0;		//��ʼ������ʧ�ܱ�־λ
				TagExitWorkSpace = 0;						//��ǩ���빤����
				
				if(AncAddr_Center < 0x02)				//�жϱ�ǩ�Ƿ��ڱ߽�λ��
				{
					AncAddr_CommBegin = 0;																													//λ��ͷ�߽磬��ʼͨѶ��վIDΪ0
					TagCommToAncNum = (TAG_COMM_TO_ANC_NUM_MAX + AncAddr_Center) - 0x02;						//�����ڵ�ͨѶ��վ��������
				}
				else if(AncAddr_Center > (ANC_NUM - 2))
				{
					AncAddr_CommBegin = AncAddr_Center - 0x02;																			//λ��β�߽磬��ʼͨѶ��վID
					TagCommToAncNum = (TAG_COMM_TO_ANC_NUM_MAX + ANC_NUM) - AncAddr_Center - 0x02;	//�����ڵ�ͨѶ��վ��������
				}
				else
				{
					AncAddr_CommBegin = AncAddr_Center - 0x02;																			//δ����߽�λ�ã���ʼͨѶ��վID
					TagCommToAncNum = TAG_COMM_TO_ANC_NUM_MAX;																			//�����ڵ�ͨѶ��վ��������
				}
/**************************���Դ���****************************/
//				TagCommToAncNum = 1;
//				AncAddr_Comm = AncAddr_Center;
/**************************************************************/
				
				for(i = 0; i < TagCommToAncNum; i++)																							//׼���ͻ�վ���ν���ͨѶ
				{
					AncAddr_Comm = AncAddr_CommBegin + i;																						//�����վ��ַ
					
//					TimeDlyForUS_380 = 1;																													//��ʱ380us
					
					WriteToMsg(TxPollMsg, (uint8_t *)&AncAddr_Comm, FRAME_DEST_ADDR_IDX, 2, 0);			//����Poll֡��վ��ַ
					WriteToMsg(TxFinalMsg, (uint8_t *)&AncAddr_Comm, FRAME_DEST_ADDR_IDX, 2, 0);		//����Final֡��վ��ַ
					
					for(j = 0; j < TAG_COMM_TO_ANC_CNT; j++)																				//�͵�ǰ��վ���ж��ͨѶ
					{
						TAG_TOF();																																		//���в��ͨѶ
						if(j < (TAG_COMM_TO_ANC_CNT - 1))
						{
							OSTimeDly(2);																																//����͵�ǰ��վ�������һ��ͨѶ�����ӳ�1ms���ȴ���վ�����������
						}
					}
				}
			}//CommDec
			else
			{
				if(TagExitWorkSpaceTimBegin == 0)
				{
					TagExitWorkSpaceTimBegin = 1;		//��һ������ʧ��,��ʼ�����ж���ʱ
					TagExitWorkSpaceTIM = HAL_GetTick() + TAG_EXIT_WORKSPACE_JUDGE_TIME_MS;
				}
				if(TagExitWorkSpace == 0)					//����ڹ������ڣ���������ˣ����������ŵ�������˯��
				{
					CollisionWindow = (CollisionWindow * 2 >= (100 / CSMA_CD_MS) ? (100 / CSMA_CD_MS) : CollisionWindow * 2);
					CollisionWindowCnt = ((CollisionWindow == 100 / CSMA_CD_MS) ? CollisionWindowCnt + 1 : CollisionWindowCnt);
					RNG_Num = RNG_Get_RandomRange(0, CollisionWindow);							//��ȡ���������ֵ
					OSTimeDly(RNG_Num * CSMA_CD_MS);
				}
			}
		}//ListenChannel
		else
		{
			OSTimeDly(SYSTEM_COMM_CYCLE_MS);
		}
	}
}


/*******************************************************************************************
* �������ƣ�void TAG_TOF(void)
* ����������tof���
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TAG_TOF(void)
{
	int8_t StratTxStatu = -1;
	
	DWT_EnableFrameFilter(SYS_CFG_FFAD);					//��������֡
	DWT_SetAutoRxReEnable(ENABLE);								//�������Զ��ؿ�
	
	DWT_SetInterrupt(DWT_INT_ALL, DISABLE);				//�ر��жϣ�����ӻ�վ�л�����ǩʱ������������жϻָ���վ����

	DWT_SetRxAfterTxDelay(POLL_TX_TO_RESP_RX_DLY_UUS);		//���÷��ͺ������գ����趨�ӳ�ʱ�䣬��λ��uus
	DWT_SetRxTimeOut(RESP_RX_TIMEOUT_UUS);								//���ý��ճ�ʱʱ�䣬��λ��uus

//	TxPollMsg[ALL_MSG_SN_IDX] = FrameSeqNum;
	DWT_WriteTxData(sizeof(TxPollMsg), TxPollMsg, 0);			//��Poll�����ݴ���DW1000�����ڿ�������ʱ����ȥ
	DWT_WriteTxfCtrl(sizeof(TxPollMsg), 0, 0);						//���ó�����������ݳ���

	StratTxStatu = DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);		//�������ͣ�������ɺ�ȴ�һ��ʱ�俪�����գ��ȴ�ʱ����dwt_setrxaftertxdelay������

//	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//���ϲ�ѯоƬ״ֱ̬���������
//	{ };
	
//	FrameSeqNum++;
	
	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//���ϲ�ѯоƬ״ֱ̬���ɹ����ջ��߷���������߽��ճ�ʱ
	{ };
	
	if (TagStatusReg & SYS_STATUS_RXFCG)//����ɹ�����
	{
		uint16 frame_len;
		
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//����Ĵ�����־λ

		frame_len = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//��ý��յ������ݳ���
		if (frame_len <= RX_BUF_LEN)
		{
			DWT_ReadRxData(RxBuffer, frame_len, 0);   														//��ȡ��������
		}
		if (RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_RESP)									//�жϽ��յ��������Ƿ���response����
		{
			uint32 final_tx_time;
			
			PollTxTS = GetTxTimeStamp_u64();																			//���POLL����ʱ��T1
			RespRxTS = GetRxTimeStamp_u64();																			//���RESPONSE����ʱ��
			
			final_tx_time = (RespRxTS + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//����final������ʱ�䣬T5=T4+Treply2
			DWT_SetDelayedTRxTime(final_tx_time);																								//����final������ʱ��T5
			
			FinalTxTS = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;						//final��ʵ�ʷ���ʱ���Ǽ���ʱ����Ϸ�������delay
			
			FinalMsgSetTS(&TxFinalMsg[FINAL_MSG_POLL_TX_TS_IDX], PollTxTS);					//��T1��T4��T5д�뷢������
			FinalMsgSetTS(&TxFinalMsg[FINAL_MSG_RESP_RX_TS_IDX], RespRxTS);
			FinalMsgSetTS(&TxFinalMsg[FINAL_MSG_FINAL_TX_TS_IDX], FinalTxTS);
			
			TxFinalMsg[ALL_MSG_SN_IDX] = FrameSeqNum;
			
			DWT_WriteTxData(sizeof(TxFinalMsg), TxFinalMsg, 0);											//����������д��DW1000
			DWT_WriteTxfCtrl(sizeof(TxFinalMsg), 0, 1);															//�趨�������ݳ���	
			
			StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED);												//�趨Ϊ�ӳٷ���
			
			while (!(DWT_Read32BitReg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))						//���ϲ�ѯоƬ״ֱ̬���������
			{}
				
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_TXFRS);				//�����־λ
//			HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED3_PIN);					//LED�̵���˸
		}
	}
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS);
		DWT_RxReset();
	}
	
	DWT_SetAutoRxReEnable(DISABLE);
	DWT_ForceTRxOff();
	DWT_EnableFrameFilter(DISABLE);		//�ر�֡����
}


/*******************************************************************************************
* �������ƣ�uint8_t CommDec(void)
* ������������ǩ��λͨѶ����
* ��ڲ�������
* ���ڲ�����
* ʹ��˵������
********************************************************************************************/
uint32_t DWT_Statu_Dec;

uint8_t CommDec(void)
{
	int8_t StratTxStatu;
	uint16_t FrameLen;
//	DWT_EnableFrameFilter(SYS_CFG_FFAD);  			//ֻ��������֡
//	DWT_SetAutoRxReEnable(ENABLE);							//�����������Զ��ؿ�
	
	DWT_ForceTRxOff();
	
	DWT_SetRxAfterTxDelay(FRAME_COMM_DEC_TX_TO_RESP_RX_DLY_UUS);				//���÷�����ɺ��������ӳ�ʱ��
	DWT_SetRxTimeOut(FRAME_COMM_DEC_RESP_RX_TIMEOUT_UUS);								//���ճ�ʱʱ��
	
	DWT_WriteTxData(sizeof(Tag_CommDec), Tag_CommDec, 0);								//д�뷢������
	DWT_WriteTxfCtrl(sizeof(Tag_CommDec), 0, 1);												//�趨���ͳ���
	
//	DWT_Statu_Dec = DWT_Read32BitReg(SYS_STATUS_ID);
	
	StratTxStatu = DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);				//�������ͣ��ӳٽ���
	
//	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_TXFRS)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߷�������
//	{}
	
	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߷�������
	{}
	
//	DWT_EnableFrameFilter(DISABLE);  			//�ر������
//	DWT_SetAutoRxReEnable(DISABLE);				//�رս������Զ��ؿ�
	
	if(TagStatusReg & SYS_STATUS_RXFCG)
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//�����־λ
		
		FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;		//��ý������ݳ���
		if (FrameLen <= RX_BUF_LEN)
		{
			DWT_ReadRxData(RxBuffer, FrameLen, 0);															//��ȡ��������
			
			if(RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_COMM_DEC_RESP)			//�ж�֡����
			{
				ReadFromMsg((uint8_t *)&AncAddr_Center, RxBuffer, FRMAE_DEC_RESP_ANC_ADDR_IDX, 2, 0);		//��ȡ��վ��ַ
				return 1;
			}
		}
	}
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_TX);
//		DWT_RxReset();
		DWT_ForceTRxOff();
	}
	
	return 0;
}


/*******************************************************************************************
* �������ƣ�ChannelStatu ListenChannel(void)
* �����������ŵ�����
* ��ڲ�������
* ���ڲ�������ǰ�ŵ�״̬��ChannelFree -- 1��ChannelBusy -- 0
* ʹ��˵������
********************************************************************************************/
ChannelStatu ListenChannel(void)
{
	uint16_t RngNum;
	uint16_t FrameLen;
	uint8_t ListenCnt = 0;
	
	DWT_SetRxTimeOut(LISTEN_CHANNEL_TIME);				//�趨���ճ�ʱʱ��
	
	while(1)
	{
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);				//�򿪽���
		
		while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR_CLE)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
		{}
		
		if ((TagStatusReg & SYS_STATUS_RXRFTO) && ((TagStatusReg & SYS_STATUS_ALL_RX_GOOD) == 0))	//���ճ�ʱ���ҽ�������ǰ��δ���ڽ�������֡����ʾ�ŵ�����
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);		//�����־λ
			
			RngNum = RNG_Get_RandomRange(0, CollisionWindow);				//��ȡ�������ֵ
			OSTimeDly(RngNum * CSMA_CD_MS);
			
			DWT_RxEnable(DWT_START_RX_IMMEDIATE);										//�򿪽���
			
			while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR_CLE)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
			{}
				
			if ((TagStatusReg & SYS_STATUS_RXRFTO) && ((TagStatusReg & SYS_STATUS_ALL_RX_GOOD) == 0))												//���ճ�ʱ���ҽ�������ǰ��δ���ڽ�������֡����ʾ�ŵ�����
			{
				DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);	//�����־λ
				
				ListenCnt = 0;							//�����ͻ����
				CollisionWindow = (CollisionWindow / 2 == 0 ? 1 : CollisionWindow / 2);				//���;������ڴ�С
//				CollisionWindow /= 2;
				
				return ChannelFree;
			}
			else if(TagStatusReg & SYS_STATUS_RXFCG)																		
			{
				DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);	//�����־λ
				
				FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//���ݳ���
			
				if (FrameLen <= RX_BUF_LEN)
				{
					DWT_ReadRxData(RxBuffer, FrameLen, 0);																//��ȡ��������
					
					if((RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_COMM_DEC) || (RxBuffer[FRAME_16BIT_ADDR_TYPE_IDX] == FRAME_COMM_DEC_RESP))			//�Ƿ��յ�����֡���������ظ�֡
					{
						OSTimeDly(TAG_COMM_TO_ALL_ANC_TIME_MS);
					}
				}
			}
		}//�ŵ�����
		else
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_RX_GOOD);
			DWT_RxReset();
			TagExitWorkSpaceTimBegin = 0;			//
		}
		ListenCnt++;
		CollisionWindow = (CollisionWindow * 2 >= (100 / CSMA_CD_MS) ? (100 / CSMA_CD_MS) : CollisionWindow * 2);
		CollisionWindowCnt = ((CollisionWindow == 100 / CSMA_CD_MS) ? CollisionWindowCnt + 1 : CollisionWindowCnt);
		
		if(ListenCnt == 16)					//������ͻ16��
		{
			return ChannelBusy;
		}
	}
}



/*******************************************************************************************
* �������ƣ�void UWB_ID_Config(void *pdata)
* ������������ǩID���ú���--���룬���ģ�ɾ��
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UWB_ID_Config(void *pdata)
{
	INT8U err;
	
	for(;;)
	{
		if(KeepInConfig == 0)		//�ж��Ƿ񱣳�������ģʽ
		{
			OSSemPend(ID_ConfigSem, 0, &err);
		}
		
		DWT_ForceTRxOff();
		
		DWT_SetRxTimeOut(0);		//�趨���ճ�ʱʱ�䣬0--û�г�ʱʱ�䣬���޵ȴ�
		
		DWT_RxEnable(DWT_START_RX_IMMEDIATE);	//�򿪽�����
		
		while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
		{ }
		
		if ((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG)	//�ɹ�����
		{
			uint16_t FrameLen;
			uint16_t FrameCotrl;
			
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//�����־λ
			
			KeepInConfig = 1;	//��λ��־λ
			
			FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if(FrameLen <= RX_BUF_LEN_MAX)
			{
				uint16_t Crc, UsartLen;
				
				DWT_ReadRxData(RxBuffer, FrameLen, 0);	//��ȡ���߽�������
				FrameCotrl = *((uint16_t *)RxBuffer);
				
				if((FrameCotrl & FRAME_CTRL_BYTE_ADDR_LEN_BIT) == FRAME_CTRL_BYTE_DEST_ID_64BIT)	//64Bit����֡
				{
					uint64_t DestAddr;
					
					ReadFromMsg((uint8_t *)&DestAddr, RxBuffer, FRAME_64BIT_DEST_ADDR_IDX, 8, 0);		//��ȡ���յ�ַ
					
					if(sTagConfig.ChipID == DestAddr)	//���յ�ַ���Լ���
					{
						UsartLen = (((uint16_t)RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_LEN_IDX]) << 8) + RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_LEN_IDX + 1];	//��ȡUSART���ݳ���
						Crc = (((uint16_t)RxBuffer[FRAME_64BIT_DATA_IDX + UsartLen + 2]) << 8) + RxBuffer[FRAME_64BIT_DATA_IDX + UsartLen + 3];		//��ȡUSART CRCֵ
						
						if(CheckCRC(Crc, RxBuffer, FRAME_64BIT_DATA_IDX, UsartLen + 2))	//У��CRC
						{
							TAG_ConfigID(RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_TYPE_IDX]);
							TAG_ConfigID_ACK(RxBuffer[FRAME_64BIT_DATA_IDX + USART_FRAME_TYPE_IDX]);
						}
						else
						{
							if(TagUsartCrcRepeatNum < USART_FRAME_REPEAT_MAX)
							{
								TAG_ConfigID_ACK(USART_FRAME_REPEAT);
							}
							else
							{
								TAG_ConfigID_ACK(USART_FRAME_ERR);
							}
						}
					}						
				}
			}
		}
		else
		{
			DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR_CLE | SYS_STATUS_ALL_TX);
			DWT_ForceTRxOff();
		}
	}
}



/*******************************************************************************************
* �������ƣ�TAG_ConfigID(uint8_t frametype)
* ������������ǩ����ID��Ϣ
* ��ڲ�����frametype--���յ���֡����
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TAG_ConfigID(uint8_t frametype)
{
	assert_param(IS_USART_FRAME_TYPE(frametype));
	
	switch(frametype)
	{
		case USART_FRAME_SET_ALL:
			TAG_ReadAllConfigInfo();
		case USART_FRAME_SET_CHIPID:
			break;
		case USART_FRAME_SET_MACID:
		{
			ReadFromMsg((uint8_t *)&sTagConfig.MAC_ID, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_MACID_LEN, 1);	//��ȡ���洢MAC ID
			sDWT.ShortAddr = sTagConfig.MAC_ID;
			sDWT.ShortAddrFlag = 1;
			break;
		}
		case USART_FRAME_SET_NAME:
			ReadStringFromMsg(sTagConfig.Name, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);															//��ȡ���洢����
			break;
		case USART_FRAME_SET_SEX:
			ReadFromMsg((uint8_t *)&sTagConfig.Sex, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_SEX_LEN, 1);			//��ȡ���洢�Ա�
			break;
		case USART_FRAME_SET_AGE:
			ReadFromMsg((uint8_t *)&sTagConfig.Age, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_AGE_LEN, 1);			//��ȡ���洢����
			break;
		case USART_FRAME_SET_JOBNUM:
			ReadFromMsg((uint8_t *)&sTagConfig.JobNum, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX, CONFIG_JOBNUM_LEN, 1);//��ȡ���洢����
			break;
		case USART_FRAME_SET_DEPT:
			ReadStringFromMsg(sTagConfig.Department, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);												//��ȡ���洢����
			break;
		case USART_FRAME_SET_TITLE:
			ReadStringFromMsg(sTagConfig.Title, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);															//��ȡ���洢ְ��
			break;
		case USART_FRAME_SET_WORKSPACE:
			ReadStringFromMsg(sTagConfig.WorkSpace, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);													//��ȡ���洢������
			break;
		case USART_FRAME_SET_PERM:
			ReadStringFromMsg(sTagConfig.Perm, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);															//��ȡ���洢Ȩ��
			break;
		case USART_FRAME_DEINIT:
			KeepInConfig = 0;
			break;
	}
}



/*******************************************************************************************
* �������ƣ�TAG_ConfigID_ACK(uint8_t frametype)
* ������������ǩ����ID��Ϣʱ�ظ�ʱ����
* ��ڲ�����frametype--���յ���֡����
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TAG_ConfigID_ACK(uint8_t frametype)
{
	uint16_t DataPayLoadLength;
	uint16_t Crc;
	
	assert_param(IS_USART_FRAME_TYPE(frametype));
	
	WriteToMsg(UsartFrameHeader, (uint8_t *)&sDWT.ChipID, FRAME_64BIT_SOURCE_ADDR_TX_IDX, 8, 0);
	DWT_WriteTxData(sizeof(UsartFrameHeader) + 2, UsartFrameHeader, 0);
	UsartFramePayLoad[0] = 0x5A;		//����Э��֡ͷ
	
	switch(frametype)
	{
		case USART_FRAME_QUERY:
		{
			if(sDWT.ShortAddrFlag == 1)		//��ǩ�����ã��ظ�ȫ����Ϣ
			{
				uint8_t DataWriteOffect = USART_FRAME_CHIPID_IDX;	//��ʼ������д��ƫ����
				
				FLASH_ReadBytes(TAG_INFO_ADDR, MAC_ID_ADDR_OFFECT, (uint8_t *)&TAG_ReadFromFlash, 8);	//��ȡ�������
				
				sTagConfig.Perm[0] = TAG_ReadFromFlash >> UWB_PERM_IDX;
				sTagConfig.JobNum = TAG_ReadFromFlash >> UWB_JOBNUM_IDX;
				sTagConfig.Age = TAG_ReadFromFlash >> UWB_AGE_IDX;
				sTagConfig.Sex = TAG_ReadFromFlash >> UWB_SEX_IDX;
				sTagConfig.MAC_ID = TAG_ReadFromFlash >> UWB_MAC_ID_IDX;
				
				FLASH_ReadBytes(TAG_INFO_ADDR, CHIPID_ADDR_OFFECT, (uint8_t *)&sTagConfig.ChipID, CONFIG_CHIPID_LEN);									//��ȡsTagConfig.ChipID
				FLASH_ReadBytes(TAG_INFO_ADDR, NAME_ADDR_OFFECT, (uint8_t *)&sTagConfig.Name, CONFIG_NAME_LEN_MAX);										//��ȡsTagConfig.Name
				FLASH_ReadBytes(TAG_INFO_ADDR, DEPT_ADDR_OFFECT, (uint8_t *)&sTagConfig.Department, CONFIG_DEPT_LEN_MAX);							//��ȡsTagConfig.Department
				FLASH_ReadBytes(TAG_INFO_ADDR, TITLE_ADDR_OFFECT, (uint8_t *)&sTagConfig.Title, CONFIG_TITLE_LEN_MAX);								//��ȡsTagConfig.Title
				FLASH_ReadBytes(TAG_INFO_ADDR, WORK_SPACE_ADDR_OFFECT, (uint8_t *)&sTagConfig.WorkSpace, CONFIG_WORKSPACE_LEN_MAX);		//��ȡsTagConfig.WorkSpace
				
				UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_INFO_ALL;		//д�봮��֡���--�ظ�ȫ����Ϣ
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.ChipID, DataWriteOffect, CONFIG_CHIPID_LEN, 1);		//д��ChipID
				DataWriteOffect += CONFIG_CHIPID_LEN;																																		//��������д��ƫ����
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.MAC_ID, DataWriteOffect, CONFIG_MACID_LEN, 1);			//дMAC_ID
				DataWriteOffect += CONFIG_MACID_LEN;
				
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Name, DataWriteOffect);								//д���������������ݶ�ȡƫ����
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.Sex, DataWriteOffect, CONFIG_SEX_LEN, 1);					//д���Ա�
				DataWriteOffect += CONFIG_SEX_LEN;
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.Age, DataWriteOffect, CONFIG_AGE_LEN, 1);					//д������
				DataWriteOffect += CONFIG_AGE_LEN;
				
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.JobNum, DataWriteOffect, CONFIG_JOBNUM_LEN, 1);		//д�빤��
				DataWriteOffect += CONFIG_JOBNUM_LEN;
				
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Department, DataWriteOffect);					//д�벿�ţ��������ݶ�ȡƫ����
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Title, DataWriteOffect);							//д��ְ�񣬸������ݶ�ȡƫ����
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.WorkSpace, DataWriteOffect);					//д�빤�������������ݶ�ȡƫ����
				DataWriteOffect += WriteStringToMsg(UsartFramePayLoad, sTagConfig.Perm, DataWriteOffect);								//д��Ȩ�ޣ��������ݶ�ȡƫ����
				
				DataPayLoadLength = DataWriteOffect - USART_FRAME_DATA_IDX + 2;	//���ڸ��س��ȣ�����д��ƫ���� - ������ʼƫ���� + CRC����
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&DataPayLoadLength, USART_FRAME_LEN_IDX, 2, 1);		//д��֡���س���
				
				Crc = GetCRC(UsartFramePayLoad, 0, DataPayLoadLength + 2);
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, DataWriteOffect, 2, 1);		//д��CRC
			}
			else	//��ǩδ���ã����ظ�ChipID
			{
				UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_INFO_CHIPID;		//д�봮��֡���
				
				sTagConfig.ChipID = sDWT.ChipID;
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&sTagConfig.ChipID, USART_FRAME_CHIPID_IDX, CONFIG_CHIPID_LEN, 1);		//д��ChipID
				
				DataPayLoadLength = CONFIG_CHIPID_LEN + 2;	//���ڸ��س��ȣ�ChipID���� + CRC����
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&DataPayLoadLength, USART_FRAME_LEN_IDX, 2, 1);		//д��֡���س���
				
				Crc = GetCRC(UsartFramePayLoad, 0, DataPayLoadLength + 2);
				WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX + CONFIG_CHIPID_LEN, 2, 1);		//д��CRC
			}
			DWT_WriteTxData(DataPayLoadLength + 6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//д�뷢������
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 6, 0, 0);														//�趨���ͳ���
			break;
		}
		case USART_FRAME_REPEAT:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_REPEAT;		//д�봮��֡���--�ط�֡
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//д��CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//д�뷢������
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//�趨���ͳ���
		}
		case USART_FRAME_ERR:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_ERR;		//д�봮��֡���--CRC����֡
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//д��CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//д�뷢������
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//�趨���ͳ���
		}
		case USART_FRAME_DEINIT:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = USART_FRAME_DEINIT_ACK;		//д�봮��֡���--CRC����֡
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//д��CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//д�뷢������
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//�趨���ͳ���
		}
		default:
		{
			UsartFramePayLoad[USART_FRAME_TYPE_IDX] = frametype;		//д�봮��֡���--ACK�ظ�֡
			
			Crc = GetCRC(UsartFramePayLoad, 0, 2);
			WriteToMsg(UsartFramePayLoad, (uint8_t *)&Crc, USART_FRAME_CHIPID_IDX, 2, 1);		//д��CRC
			
			DWT_WriteTxData(6, UsartFramePayLoad, sizeof(UsartFrameHeader));								//д�뷢������
			DWT_WriteTxfCtrl(sizeof(UsartFrameHeader) + DataPayLoadLength + 4, 0, 0);														//�趨���ͳ���
		}
	}
	
	DWT_StartTx(DWT_START_TX_IMMEDIATE);
	
	while (!((TagStatusReg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))	//���ϲ�ѯоƬ״ֱ̬���������
	{ };
	
	DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//����Ĵ�����־λ
}


/*******************************************************************************************
* �������ƣ�TAG_ReadAllConfigInfo(void)
* ������������ǩ��������������Ϣ
* ��ڲ�����frametype--���յ���֡����
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TAG_ReadAllConfigInfo(void)
{
	uint8_t DataReadOffect = USART_FRAME_MACID_IDX;	//��ʼ�����ݶ�ȡƫ������Chip ID�����޸ģ��ʲ����ж�ȡ��
	
	ReadFromMsg((uint8_t *)&sTagConfig.MAC_ID, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_MACID_LEN, 1);		//��ȡ������MAC ID
	DataReadOffect += CONFIG_MACID_LEN;																																									//�������ݶ�ȡƫ����
	sDWT.ShortAddr = sTagConfig.MAC_ID;
	sDWT.ShortAddrFlag = 1;
	
	DataReadOffect += ReadStringFromMsg(sTagConfig.Name, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect);							//��ȡ�������������������ݶ�ȡƫ����
	
	ReadFromMsg((uint8_t *)&sTagConfig.Sex, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_SEX_LEN, 1);				//��ȡ�������Ա�
	DataReadOffect += CONFIG_SEX_LEN;
	
	ReadFromMsg((uint8_t *)&sTagConfig.Age, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_AGE_LEN, 1);				//��ȡ����������
	DataReadOffect += CONFIG_AGE_LEN;
	
	ReadFromMsg((uint8_t *)&sTagConfig.JobNum, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect, CONFIG_JOBNUM_LEN, 1);	//��ȡ�����湤��
	DataReadOffect += CONFIG_JOBNUM_LEN;
	
	DataReadOffect += ReadStringFromMsg(sTagConfig.Department, RxBuffer, FRAME_64BIT_DATA_IDX + DataReadOffect);				//��ȡ�����沿�ţ��������ݶ�ȡƫ����
	DataReadOffect += ReadStringFromMsg(sTagConfig.Title, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);				//��ȡ������ְ�񣬸������ݶ�ȡƫ����
	DataReadOffect += ReadStringFromMsg(sTagConfig.WorkSpace, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);		//��ȡ�����湤�������������ݶ�ȡƫ����
	
	ReadStringFromMsg(sTagConfig.Perm, RxBuffer, FRAME_64BIT_DATA_IDX + USART_FRAME_DATA_IDX);													//��ȡ������Ȩ��
}


/************************************��ǩ���ݴ洢˳��*************************************************************************
* 1. DWT_WriteToFlash -- sDWT.ShortAddr + sDWT.DWT_Mode + sDWT.ShortAddrFlag��64bit
* 2. sTagConfig.ChipID -- 64bit
* 3. TAG_WriteToFlash -- sTagConfig.Perm + sTagConfig.JobNum + sTagConfig.Age + sTagConfig.Sex + sTagConfig.MAC_ID��64bit
* 4. sTagConfig.Name -- 256bit
* 5. sTagConfig.Department -- 256bit
* 6. sTagConfig.Title -- 256bit
* 7. sTagConfig.WorkSpace -- 256bit
******************************************************************************************************************************/

/*******************************************************************************************
* �������ƣ�TAG_ReadAllConfigInfo(void)
* ������������ǩ��������������Ϣ
* ��ڲ�����frametype--���յ���֡����
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TAG_SaveConfigInfoToFlash(void)
{
	DWT_WriteToFlash = (((uint64_t)sDWT.ShortAddr << UWB_16BIT_ADDR_IDX) | \
											((uint64_t)sDWT.ShortAddrFlag << UWB_16BIT_ADDR_FLAG_IDX) | \
											((uint64_t)sDWT.DWT_Mode << UWB_MODE_IDX));	
	
	TAG_WriteToFlash = (((uint64_t)sTagConfig.Perm[0] << UWB_PERM_IDX) | \
											((uint64_t)sTagConfig.JobNum << UWB_JOBNUM_IDX) | \
											((uint64_t)sTagConfig.Age << UWB_AGE_IDX) | \
											((uint64_t)sTagConfig.Sex << UWB_SEX_IDX) | \
											((uint64_t)sTagConfig.MAC_ID << UWB_MAC_ID_IDX));
	
	FLASH_WriteBytes(TAG_INFO_ADDR, SHORT_ID_ADDR_OFFECT, &DWT_WriteToFlash, 1);																								//����sDWT����
	FLASH_WriteBytes(TAG_INFO_ADDR, CHIPID_ADDR_OFFECT, &sTagConfig.ChipID, CONFIG_CHIPID_LEN >> 3);														//����ChipID
	FLASH_WriteBytes(TAG_INFO_ADDR, MAC_ID_ADDR_OFFECT, &TAG_WriteToFlash, 1);																									//����sTagConfig�����������
	FLASH_WriteBytes(TAG_INFO_ADDR, NAME_ADDR_OFFECT, (uint64_t *)sTagConfig.Name, CONFIG_NAME_LEN_MAX >> 3);										//����sTagConfig.Name
	FLASH_WriteBytes(TAG_INFO_ADDR, DEPT_ADDR_OFFECT, (uint64_t *)sTagConfig.Department, CONFIG_DEPT_LEN_MAX >> 3);							//����sTagConfig.Department
	FLASH_WriteBytes(TAG_INFO_ADDR, TITLE_ADDR_OFFECT, (uint64_t *)sTagConfig.Title, CONFIG_TITLE_LEN_MAX >> 3);								//����sTagConfig.Title
	FLASH_WriteBytes(TAG_INFO_ADDR, WORK_SPACE_ADDR_OFFECT, (uint64_t *)sTagConfig.WorkSpace, CONFIG_WORKSPACE_LEN_MAX >> 3);		//����sTagConfig.WorkSpace
	
}
