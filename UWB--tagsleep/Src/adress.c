#include "address.h"

#define NO_FREE_ADDR				0xFFFE

uint8_t  const  AddrUnMapTbl[256] = {
    0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x00 to 0x0F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x10 to 0x1F                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x20 to 0x2F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x30 to 0x3F                             */
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x40 to 0x4F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x50 to 0x5F                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x60 to 0x6F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x70 to 0x7F                             */
    7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x80 to 0x8F                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0x90 to 0x9F                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xA0 to 0xAF                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xB0 to 0xBF                             */
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xC0 to 0xCF                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xD0 to 0xDF                             */
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,       /* 0xE0 to 0xEF                             */
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0        /* 0xF0 to 0xFF                             */
};

uint32_t AddrMapTbl[ADDR_16BIT_MAX_NUM / 8 + 1] = {
		0x00000001, 0x00000002, 0x00000004, 0x00000008, 			/* AddrFreeTbl: 0 �� 3	*/
		0x00000010, 0x00000020, 0x00000040, 0x00000080, 			/* AddrFreeTbl: 4 �� 7	*/
		0x00000100, 0x00000200, 0x00000400, 0x00000800, 			/* AddrFreeTbl: 8 ��11	*/
		0x00001000, 0x00002000, 0x00004000, 0x00008000, 			/* AddrFreeTbl: 12��15	*/
		0x00010000, 0x00020000, 0x00040000, 0x00080000, 			/* AddrFreeTbl: 16��19	*/
		0x00100000, 0x00200000, 0x00400000, 0x00800000, 			/* AddrFreeTbl: 20��23	*/
		0x01000000, 0x02000000, 0x04000000, 0x08000000, 			/* AddrFreeTbl: 24��27	*/
		0x10000000, 0x20000000, 0x40000000, 0x80000000, 			/* AddrFreeTbl: 28��31	*/
};

uint8_t AddrFreeTbl[ADDR_16BIT_MAX_NUM / 8 + 1];					//���е�ַ�б�
//uint8_t AddrRdyTbl[ADDR_16BIT_MAX_NUM / 8 + 1];

uint8_t UWB_ModelNum = 4;
uint8_t UWB_TagNum;
uint8_t UWB_AncNum;
uint8_t UWB_ModelNow;
uint8_t UWB_FirstSyncFlag = 0;

uint16_t AddrForAnc_M;

uint32_t AddrFreeGrp = 0xFFFFFFFF;

extern uint64_t AddrWriteToFlash;

static uint32 status_reg;
static uint16 FrameLen;
static uint8 rx_buffer[RX_BUF_LEN+4];

static uint64_t FrameRxTime, FrameTxTime;
	
extern DWT_AddrStatus_s sDWT;
DWT_AddrSta_s sDWT_AddrStatus[ADDR_16BIT_MAX_NUM];

extern uint8_t AllModelHaveAddr;


/*��վ*/
extern uint8 RxBeaconMsg[];
extern uint8 TxSyncMsg[];
extern uint8 RxSyncMsg[];

/*��ǩ*/
extern uint8 TxBeaconMsg[];

extern uint8 AddrEnsureMsg[17];
extern uint8 AddrEnsureMsgResp[21];

extern uint8_t Tag_AskForShortID[14];
extern uint8_t Anc_ConfigShortID[16];

extern uint8_t FirstSync[12];
extern uint8_t FirstSyncTest[13];

/**********************************************/
uint64_t SysTime = 0;
//uint32_t FrameSyncTxTime = 0;
//uint64_t SysTimeNow = 0;

/**********************************************/

/*******************************************************************************************
* �������ƣ�void Tag_AskFor16BitAddr(void)
* ����������16Bit��ַ����������ǩ�ͻ�վ�ࣩ
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void Tag_AskFor16BitAddr(void)
{
	DWT_SetRxAfterTxDelay(SHORT_ID_CONFIG_RX_DLY_UUS);					//���÷��ͺ������գ����趨�ӳ�ʱ�䣬��λ��uus
	DWT_SetRxTimeOut(SHORT_ID_CONFIG_RX_TIMEOUT_UUS);						//���ý��ճ�ʱʱ�䣬��λ��uus
	
	WriteToMsg(Tag_AskForShortID, (uint8_t *)&sDWT.ChipID, FRAME_SHORT_ID_ASK_SOURE_ADDR_IDX, 4); 		//д��32λCHIP ID 
	WriteToMsg(Anc_ConfigShortID, (uint8_t *)&sDWT.ChipID, FRAME_SHORT_ID_CONFIG_DEST_ADDR_IDX, 4);  	//д��̵�ַ�ظ�֡Ŀ���ַ
	
	DWT_WriteTxData(sizeof(Tag_AskForShortID), Tag_AskForShortID, 0);	//��Poll�����ݴ���DW1000�����ڿ�������ʱ����ȥ
	DWT_WriteTxfCtrl(sizeof(Tag_AskForShortID), 0, 0);								//���ó�����������ݳ���

	DWT_StartTx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);			//�������ͣ�������ɺ�ȴ�һ��ʱ�俪�����գ��ȴ�ʱ����DWT_SetRxAfterTxDelay������
	
	while (!((status_reg = DWT_Read32BitReg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߷�������
	{ };
	
	HAL_GPIO_TogglePin(LED1_PIN_Port, LED1_PIN);	//LED�����˸
	
	if (status_reg & SYS_STATUS_RXFCG)//���ճɹ�
	{
//		DWT_SetAutoRxReEnable(DISABLE);			//�رս������Զ��ؿ�
//		DWT_ForceTRxOff();
//		DWT_EnableFrameFilter(DISABLE);
		
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	//�����־λ
		
		FrameLen = DWT_Read32BitReg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;				//���ݳ���
		
		if (FrameLen <= RX_BUF_LEN)
		{
			DWT_ReadRxData(rx_buffer, FrameLen, 0);																//��ȡ��������
		}
		
		if (memcmp(rx_buffer, Anc_ConfigShortID, SHORT_ADDR_ASK_COMMON_LEN) == 0)  //�ж��Ƿ�Ϊ�̵�ַ�ظ�֡
		{
			sDWT.ShortAddr = (((uint16_t)rx_buffer[FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX + 1]) << 8) | rx_buffer[FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX];  //��ȡ�̵�ַ����ֵ
			sDWT.ShortAddrFlag = 1;
			
			AddrWriteToFlash = ((uint64_t)sDWT.ChipID << FLASH_ChipID_IDX) | \
												 ((uint64_t)sDWT.ShortAddr << FLASH_16BIT_ID_IDX) | \
												 ((uint64_t)sDWT.ShortAddrFlag << FLASH_16BIT_ID_FLAG_IDX) | \
												 (uint64_t)sDWT.DWT_Mode << FLASH_MODE_IDX;															//����豣������
			
			FLASH_WriteBytes((uint32_t)0, (uint8_t *)&AddrWriteToFlash, 1);													//������FLASH
			
			HAL_GPIO_WritePin(LED1_PIN_Port, LED1_PIN, GPIO_PIN_SET);																//LED1��

			OSTaskResume(UWB_MODECHOOSE_PRIO);
			OSTaskSuspend(OS_PRIO_SELF);
		}
	}//���ճɹ�
	else
	{
		DWT_Write32BitReg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
		DWT_RxReset();
		OSTimeDly(ASK_ADDR_DELAY_MS);//���߹̶�ʱ��
	}
}


/*******************************************************************************************
* �������ƣ�void CLE_AskFor16BitAddr(uint8_t *pdata)
* ����������16Bit��ַ����������CLE��
* ��ڲ�����*pdata�����յ�������֡��
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void CLE_AskFor16BitAddr(uint8_t *pdata)
{
	int8_t StratTxStatu = -1;		//�����������Ƿ�ɹ���-1��δ������0���ѿ���
	uint32_t ChipID;
	uint16_t FreeAddr16Bit = 0;
	int8_t i;
	
//	DWT_ForceTRxOff();
	
	for(i = 3; i >= 0; i--)
	{
		ChipID = (ChipID << 8) | pdata[FRAME_SHORT_ID_ASK_SOURE_ADDR_IDX + i];   //��ȡ32Bit��ַ
	}
	
	FreeAddr16Bit = GetFreeAddr();  //�ӿ��е�ַ��ȡ��һ����ַ��ȡ��˳���С����
	
//	sDWT_AddrStatus[FreeAddr16Bit].DWT_Mode = pdata[ADDRESS_ASK_MODE_ID_IDX]; //���������Ϣ
	sDWT_AddrStatus[FreeAddr16Bit].ChipID = ChipID;
	
	WriteToMsg(Anc_ConfigShortID, (uint8_t *)&ChipID, FRAME_SHORT_ID_CONFIG_DEST_ADDR_IDX, 4); 				//Ŀ���ַ--�������ߵ�ַ��32Bit��
	WriteToMsg(Anc_ConfigShortID, (uint8_t *)&FreeAddr16Bit, FRAME_SHORT_ID_CONFIG_16BIT_ID_IDX, 2);	//��������16Bit��ֵַ
	
	FrameRxTime = GetRxTimeStamp_u64();		//��ȡ����֡����ʱ��
	FrameTxTime = (FrameRxTime + (SHORT_ID_CONFIG_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//����ظ�֡����ʱ�䡣
	DWT_SetDelayedTRxTime(FrameTxTime);//���ûظ�֡����ʱ��
	
	DWT_WriteTxData(sizeof(Anc_ConfigShortID), Anc_ConfigShortID, 0);//д�뷢������
	DWT_WriteTxfCtrl(sizeof(Anc_ConfigShortID), 0, 1);//�趨���ͳ���
	
	while(StratTxStatu)
	{
		StratTxStatu = DWT_StartTx(DWT_START_TX_DELAYED);		//�ӳٷ���
	}
	
	while (!((status_reg = DWT_Read32BitReg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))//���ϲ�ѯоƬ״ֱ̬���������
	{ }
	
	UWB_ModelNow++;
}


/*******************************************************************************************
* �������ƣ�uint16_t GetFreeAddr(void)
* ������������ȡ���е�ַ
* ��ڲ�������
* ���ڲ�����AddrFree--16Bit��ַ
* ʹ��˵������
********************************************************************************************/
uint16_t GetFreeAddr(void)
{
	uint32_t AddrFreeGrpNow;
	uint16_t AddrFree;
	uint8_t AddrUnMapTblFoldNum = 0;
	uint8_t y;
	
	AddrFreeGrpNow = AddrFreeGrp;
	
	if(AddrFreeGrpNow == 0x0000)
	{
		return NO_FREE_ADDR;       //��ǰ�޿��е�ַ
	}
	
	while((uint8_t)AddrFreeGrpNow == 0)    //����AdrUnMapTbl�۵�����
	{
		AddrUnMapTblFoldNum++;
		AddrFreeGrpNow = AddrFreeGrpNow >> 8;
	}
	
	y = AddrUnMapTbl[(uint8_t)AddrFreeGrpNow] + (8 * AddrUnMapTblFoldNum);  //��ȡ��ǰ���е�ַ����Сֵ
	AddrFree = (INT8U)((y << 3) + AddrUnMapTbl[AddrFreeTbl[y]]);
	
	if((AddrFreeTbl[AddrFree >> 3] &= ~AddrMapTbl[(uint8_t)AddrFree & 0x07]) == 0) //������ȡ��ַ�ӿ��е�ַ�б������
	{
		AddrFreeGrp &= ~AddrMapTbl[AddrFree >> 3];
	}
	
	return AddrFree;
}

/*******************************************************************************************
* �������ƣ�void Anc_CLE_Release16BitAddr(uint16_t ShortAddr)
* ����������16Bit��ַ�ͷŴ���CLE��
* ��ڲ�����ShortAddr�����ͷŵ�16Bit��ַ��
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void Anc_CLE_Release16BitAddr(uint16_t ShortAddr)
{
	AddrFreeGrp |= AddrMapTbl[ShortAddr >> 3];
	AddrFreeTbl[ShortAddr >> 3] |= AddrMapTbl[ShortAddr & 0x07];		//�����ͷŵ�16Bit��ַ��ӵ����е�ַ�б���
	
	sDWT_AddrStatus[ShortAddr].DWT_Mode = 0;  //���16Bit��ַ��32λ��ַ������Ϣ
	sDWT_AddrStatus[ShortAddr].ChipID = 0;
}


