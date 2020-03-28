/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : i2s.c
* ��  �� :��ʼ��I2S�ӿ�
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
*********************************************************************************************
*/


#include "i2s.h"  

//����I2S_Standard:  @ref SPI_I2S_Standard  I2S��׼,
//I2S_Standard_Phillips,�����ֱ�׼;
//I2S_Standard_MSB,MSB�����׼(�Ҷ���);
//I2S_Standard_LSB,LSB�����׼(�����);
//I2S_Standard_PCMShort,I2S_Standard_PCMLong:PCM��׼
//����I2S_Mode:  @ref SPI_I2S_Mode  I2S_Mode_SlaveTx:�ӻ�����;I2S_Mode_SlaveRx:�ӻ�����;I2S_Mode_MasterTx:��������;I2S_Mode_MasterRx:��������;
//����I2S_Clock_Polarity   @ref SPI_I2S_Clock_Polarity:  I2S_CPOL_Low,ʱ�ӵ͵�ƽ��Ч;I2S_CPOL_High,ʱ�Ӹߵ�ƽ��Ч
//����I2S_DataFormat�� @ref SPI_I2S_Data_Format :���ݳ���,I2S_DataFormat_16b,16λ��׼;I2S_DataFormat_16bextended,16λ��չ(frame=32bit);I2S_DataFormat_24b,24λ;I2S_DataFormat_32b,32λ.
void I2S2_Init(u16 I2S_Standard,u16 I2S_Mode,u16 I2S_Clock_Polarity,u16 I2S_DataFormat)
{ 
	I2S_InitTypeDef I2S_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��SPI2ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	
	SPI_I2S_DeInit(SPI2);
	I2S_InitStructure.I2S_Mode=I2S_Mode;//IISģʽ
	I2S_InitStructure.I2S_Standard=I2S_Standard;//IIS��׼
	I2S_InitStructure.I2S_DataFormat=I2S_DataFormat;//IIS���ݳ���
	I2S_InitStructure.I2S_MCLKOutput=I2S_MCLKOutput_Enable;//��ʱ���������
	I2S_InitStructure.I2S_AudioFreq=I2S_AudioFreq_44k;//IISƵ������
	I2S_InitStructure.I2S_CPOL=I2S_Clock_Polarity;//����״̬ʱ�ӵ�ƽ
	I2S_Init(SPI2,&I2S_InitStructure);//��ʼ��IIS

	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);//SPI2 TX DMA����ʹ��.
	I2S_Cmd(SPI2,ENABLE);//SPI2 I2S ENʹ��.	
	
} 
//�����ʼ��㹫ʽ:Fs=I2SxCLK/[256*(2*I2SDIV+ODD)]
//I2SxCLK=(HSE/pllm)*PLLI2SN/PLLI2SR
//һ��HSE=8Mhz 
//pllm:��Sys_Clock_Set���õ�ʱ��ȷ����һ����8
//PLLI2SN:һ����192~432 
//PLLI2SR:2~7
//I2SDIV:2~255
//ODD:0/1
//I2S��Ƶϵ����@pllm=8,HSE=8Mhz,��vco����Ƶ��Ϊ1Mhz
//���ʽ:������/10,PLLI2SN,PLLI2SR,I2SDIV,ODD
const u16 I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz������
	{1102,429,4,19,0},		//11.025Khz������ 
	{1600,213,2,13,0},		//16Khz������
	{2205,429,4, 9,1},		//22.05Khz������
	{3200,213,2, 6,1},		//32Khz������
	{4410,271,2, 6,0},		//44.1Khz������
	{4800,258,3, 3,1},		//48Khz������
	{8820,316,2, 3,1},		//88.2Khz������
	{9600,344,2, 3,1},  	//96Khz������
	{17640,361,2,2,0},  	//176.4Khz������ 
	{19200,393,2,2,0},  	//192Khz������
};  

//I2S2 TX DMA����
//����Ϊ˫����ģʽ,������DMA��������ж�
//buf0:M0AR��ַ.
//buf1:M1AR��ַ.
//num:ÿ�δ���������
void I2S2_TX_DMA_Init(u8* buf0,u8 *buf1,u16 num)
{  
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //DMA1ʱ��
 
	
	DMA_DeInit(DMA1_Channel5);
	DMA_SetCurrDataCounter(DMA1_Channel5, 0);
	DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5);//??DMA2 channel1??
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&SPI2->DR);                 //�����ַΪ:(u32)&SPI2->DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buf0;                            //DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                           //������Ϊ���ݵ�Ŀ��
	DMA_InitStructure.DMA_BufferSize = num;                                      //���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //����Ĵ�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          //�洢�����ݳ��ȣ�16λ
	DMA_InitStructure.DMA_Mode =DMA_Mode_Circular ;//DMA_Mode_Normal;                              // ʹ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          //�����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 //��ֹ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	//DMA_Cmd(DMA1_Channel5, ENABLE);  
		
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx,ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); 
	

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;//�����ȼ�6
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
	
} 
void I2S2_DMA_Init(u8* buf0,u16 num)
{  
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //DMA1ʱ��
 
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;//�����ȼ�6
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����
	
	DMA_DeInit(DMA1_Channel5);
	DMA_SetCurrDataCounter(DMA1_Channel5, 0);
	DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5);//??DMA2 channel1??
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&SPI2->DR);                 //�����ַΪ:(u32)&SPI2->DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buf0;                            //DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                           //������Ϊ���ݵ�Ŀ��
	DMA_InitStructure.DMA_BufferSize = num;                                      //���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //����Ĵ�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          //�洢�����ݳ��ȣ�16λ
	DMA_InitStructure.DMA_Mode =DMA_Mode_Circular ;//DMA_Mode_Normal;                              // ʹ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          //�����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 //��ֹ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	 
		
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx,ENABLE);
	 
}
void reset_dma_buf(DMA_Channel_TypeDef * dma, void * buf, u16 size)
{
	dma->CMAR = (u32)buf;
	dma->CNDTR = size;
	
}
//I2S DMA�ص�����ָ��
void (*i2s_tx_callback)(void);	//TX�ص����� 

//DMA1_Stream4�жϷ�����
void DMA4_Channel5_IRQHandler(void)
{      
	if(DMA_GetITStatus(DMA1_IT_TC5)==SET)////DMA2_IT_TC1,������ɱ�־
	{ 
		DMA_ClearFlag(DMA1_FLAG_TC5);
      	i2s_tx_callback();	//ִ�лص�����,��ȡ���ݵȲ����������洦��  
	}   											 
}  
//I2S��ʼ����
//void I2S_Play_Start(void)
//{   	  
//	DMA_Cmd(DMA1_Channel5,ENABLE);//����DMA TX����,��ʼ���� 		
//}
void I2S_Play_Start(void)
{   
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx,ENABLE);
 	DMA_ClearFlag(DMA1_FLAG_TC5);
 	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); 
	DMA_Cmd(DMA1_Channel5, ENABLE); 
			
}
//�ر�I2S����
//void I2S_Play_Stop(void)
//{   
//	DMA_Cmd(DMA1_Channel5,DISABLE);//�ر�DMA,��������	 
//} 
void I2S_Play_Stop(void)
{   
	I2S_Cmd(SPI2, DISABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);//�ر�DMA,��������	 
}







