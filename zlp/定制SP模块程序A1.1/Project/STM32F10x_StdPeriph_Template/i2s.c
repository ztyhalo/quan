/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 文件名 : i2s.c
* 描  述 :初始化I2S接口
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
*********************************************************************************************
*/


#include "i2s.h"  

//参数I2S_Standard:  @ref SPI_I2S_Standard  I2S标准,
//I2S_Standard_Phillips,飞利浦标准;
//I2S_Standard_MSB,MSB对齐标准(右对齐);
//I2S_Standard_LSB,LSB对齐标准(左对齐);
//I2S_Standard_PCMShort,I2S_Standard_PCMLong:PCM标准
//参数I2S_Mode:  @ref SPI_I2S_Mode  I2S_Mode_SlaveTx:从机发送;I2S_Mode_SlaveRx:从机接收;I2S_Mode_MasterTx:主机发送;I2S_Mode_MasterRx:主机接收;
//参数I2S_Clock_Polarity   @ref SPI_I2S_Clock_Polarity:  I2S_CPOL_Low,时钟低电平有效;I2S_CPOL_High,时钟高电平有效
//参数I2S_DataFormat： @ref SPI_I2S_Data_Format :数据长度,I2S_DataFormat_16b,16位标准;I2S_DataFormat_16bextended,16位扩展(frame=32bit);I2S_DataFormat_24b,24位;I2S_DataFormat_32b,32位.
void I2S2_Init(u16 I2S_Standard,u16 I2S_Mode,u16 I2S_Clock_Polarity,u16 I2S_DataFormat)
{ 
	I2S_InitTypeDef I2S_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2时钟
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
	I2S_InitStructure.I2S_Mode=I2S_Mode;//IIS模式
	I2S_InitStructure.I2S_Standard=I2S_Standard;//IIS标准
	I2S_InitStructure.I2S_DataFormat=I2S_DataFormat;//IIS数据长度
	I2S_InitStructure.I2S_MCLKOutput=I2S_MCLKOutput_Enable;//主时钟输出启动
	I2S_InitStructure.I2S_AudioFreq=I2S_AudioFreq_44k;//IIS频率设置
	I2S_InitStructure.I2S_CPOL=I2S_Clock_Polarity;//空闲状态时钟电平
	I2S_Init(SPI2,&I2S_InitStructure);//初始化IIS

	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);//SPI2 TX DMA请求使能.
	I2S_Cmd(SPI2,ENABLE);//SPI2 I2S EN使能.	
	
} 
//采样率计算公式:Fs=I2SxCLK/[256*(2*I2SDIV+ODD)]
//I2SxCLK=(HSE/pllm)*PLLI2SN/PLLI2SR
//一般HSE=8Mhz 
//pllm:在Sys_Clock_Set设置的时候确定，一般是8
//PLLI2SN:一般是192~432 
//PLLI2SR:2~7
//I2SDIV:2~255
//ODD:0/1
//I2S分频系数表@pllm=8,HSE=8Mhz,即vco输入频率为1Mhz
//表格式:采样率/10,PLLI2SN,PLLI2SR,I2SDIV,ODD
const u16 I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz采样率
	{1102,429,4,19,0},		//11.025Khz采样率 
	{1600,213,2,13,0},		//16Khz采样率
	{2205,429,4, 9,1},		//22.05Khz采样率
	{3200,213,2, 6,1},		//32Khz采样率
	{4410,271,2, 6,0},		//44.1Khz采样率
	{4800,258,3, 3,1},		//48Khz采样率
	{8820,316,2, 3,1},		//88.2Khz采样率
	{9600,344,2, 3,1},  	//96Khz采样率
	{17640,361,2,2,0},  	//176.4Khz采样率 
	{19200,393,2,2,0},  	//192Khz采样率
};  

//I2S2 TX DMA配置
//设置为双缓冲模式,并开启DMA传输完成中断
//buf0:M0AR地址.
//buf1:M1AR地址.
//num:每次传输数据量
void I2S2_TX_DMA_Init(u8* buf0,u8 *buf1,u16 num)
{  
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //DMA1时钟
 
	
	DMA_DeInit(DMA1_Channel5);
	DMA_SetCurrDataCounter(DMA1_Channel5, 0);
	DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5);//??DMA2 channel1??
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&SPI2->DR);                 //外设地址为:(u32)&SPI2->DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buf0;                            //DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                           //外设作为数据的目的
	DMA_InitStructure.DMA_BufferSize = num;                                      //数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //外设寄存器地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          //存储器数据长度：16位
	DMA_InitStructure.DMA_Mode =DMA_Mode_Circular ;//DMA_Mode_Normal;                              // 使用循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          //高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 //禁止内存到内存传输
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	//DMA_Cmd(DMA1_Channel5, ENABLE);  
		
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx,ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); 
	

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;//子优先级6
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
	
} 
void I2S2_DMA_Init(u8* buf0,u16 num)
{  
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //DMA1时钟
 
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;//子优先级6
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
	
	DMA_DeInit(DMA1_Channel5);
	DMA_SetCurrDataCounter(DMA1_Channel5, 0);
	DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5);//??DMA2 channel1??
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&SPI2->DR);                 //外设地址为:(u32)&SPI2->DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buf0;                            //DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                           //外设作为数据的目的
	DMA_InitStructure.DMA_BufferSize = num;                                      //数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //外设寄存器地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          //存储器数据长度：16位
	DMA_InitStructure.DMA_Mode =DMA_Mode_Circular ;//DMA_Mode_Normal;                              // 使用循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          //高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 //禁止内存到内存传输
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	 
		
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx,ENABLE);
	 
}
void reset_dma_buf(DMA_Channel_TypeDef * dma, void * buf, u16 size)
{
	dma->CMAR = (u32)buf;
	dma->CNDTR = size;
	
}
//I2S DMA回调函数指针
void (*i2s_tx_callback)(void);	//TX回调函数 

//DMA1_Stream4中断服务函数
void DMA4_Channel5_IRQHandler(void)
{      
	if(DMA_GetITStatus(DMA1_IT_TC5)==SET)////DMA2_IT_TC1,传输完成标志
	{ 
		DMA_ClearFlag(DMA1_FLAG_TC5);
      	i2s_tx_callback();	//执行回调函数,读取数据等操作在这里面处理  
	}   											 
}  
//I2S开始播放
//void I2S_Play_Start(void)
//{   	  
//	DMA_Cmd(DMA1_Channel5,ENABLE);//开启DMA TX传输,开始播放 		
//}
void I2S_Play_Start(void)
{   
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx,ENABLE);
 	DMA_ClearFlag(DMA1_FLAG_TC5);
 	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); 
	DMA_Cmd(DMA1_Channel5, ENABLE); 
			
}
//关闭I2S播放
//void I2S_Play_Stop(void)
//{   
//	DMA_Cmd(DMA1_Channel5,DISABLE);//关闭DMA,结束播放	 
//} 
void I2S_Play_Stop(void)
{   
	I2S_Cmd(SPI2, DISABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);//关闭DMA,结束播放	 
}







