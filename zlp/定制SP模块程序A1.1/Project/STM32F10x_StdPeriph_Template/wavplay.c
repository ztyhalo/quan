/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 文件名 : wavplay.c
* 描  述 :播放声音的应用层函数
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
*********************************************************************************************
*/


//#include "wavplay.h" 
//#include "malloc.h"
//#include "ff.h"
//#include "i2s.h"
//#include "bsp_wm8978.h"
//#include "ucos_ii.h"
//#include  "config.h"

#include "include.h"

//extern u8 CORUSCATE;
//extern u8 InputErr;
//extern u8 Input;
void StartPlay(void);
//V1.0 说明
//1,支持16位/24位WAV文件播放
//2,最高可以支持到192K/24bit的WAV格式. 
////////////////////////////////////////////////////////////////////////////////// 	
 
__wavctrl wavctrl;		//WAV控制结构体
vu8 wavtransferend=0;	//i2s传输完成标志
vu8 wavwitchbuf=0;		//i2sbufx指示标志
 

static uint8_t 			Gauddatabuf[2][WAV_I2S_TX_DMA_BUFSIZE];  
static uint32_t         Gplyfilep = 0;

//u8 audiofilm[1024];
//u8 audiobuf1[WAV_I2S_TX_DMA_BUFSIZE];
//u8 audiobuf2[WAV_I2S_TX_DMA_BUFSIZE];

//音乐播放控制器
__audiodev audiodev;	  
 

//开始音频播放
void audio_start(void)
{
	audiodev.status=3<<0;//开始播放+非暂停
	I2S_Play_Start();
} 
//关闭音频播放
void audio_stop(void)
{
	audiodev.status=0;
	I2S_Play_Stop();
}  

 
//WAV解析初始化
//fname:文件路径+文件名
//wavx:wav 信息存放结构体指针
//返回值:0,成功;1,打开文件失败;2,非WAV文件;3,DATA区域未找到.
u8 wav_decode_init(u8* fname,__wavctrl* wavx)
{
	FIL*ftemp;
	u8 *buf; 
	u32 br=0;
	u8 res=0;
	
	ChunkRIFF *riff;
	ChunkFMT *fmt;
	ChunkFACT *fact;
	ChunkDATA *data;
	ftemp=(FIL*)mymalloc(SRAMIN,sizeof(FIL));
	buf=mymalloc(SRAMIN,512);
	if(ftemp&&buf)	//内存申请成功
	{
		res=f_open(ftemp,(TCHAR*)fname,FA_READ);//打开文件
		if(res==FR_OK)
		{
			f_read(ftemp,buf,512,&br);	//读取512字节在数据
			riff=(ChunkRIFF *)buf;		//获取RIFF块
			if(riff->Format==0X45564157)//是WAV文件
			{
				fmt=(ChunkFMT *)(buf+12);	//获取FMT块 
				fact=(ChunkFACT *)(buf+12+8+fmt->ChunkSize);//读取FACT块
				if(fact->ChunkID==0X74636166||fact->ChunkID==0X5453494C)wavx->datastart=12+8+fmt->ChunkSize+8+fact->ChunkSize;//具有fact/LIST块的时候(未测试)
				else wavx->datastart=12+8+fmt->ChunkSize;  
				data=(ChunkDATA *)(buf+wavx->datastart);	//读取DATA块
				if(data->ChunkID==0X61746164)//解析成功!
				{
					wavx->audioformat=fmt->AudioFormat;		//音频格式
					wavx->nchannels=fmt->NumOfChannels;		//通道数
					wavx->samplerate=fmt->SampleRate;		//采样率
					wavx->bitrate=fmt->ByteRate*8;			//得到位速
					wavx->blockalign=fmt->BlockAlign;		//块对齐
					wavx->bps=fmt->BitsPerSample;			//位数,16/24/32位
					
					wavx->datasize=data->ChunkSize;			//数据块大小
					wavx->datastart=wavx->datastart+8;		//数据流开始的地方. 
					 
				}else res=3;//data区域未找到.
			}else res=2;//非wav文件
			
		}else res=1;//打开文件错误
	}
	f_close(ftemp);
	myfree(SRAMIN,ftemp);//释放内存
	myfree(SRAMIN,buf); 
	return res;
}

//填充buf
//buf:数据区
//size:填充数据量
//bits:位数(16/24)
//返回值:读到的数据个数
u32 wav_buffill(u8 *buf,u16 size,u8 bits)
{
	u16 readlen=0;
	volatile u32 bread = 0;
	u16 i;
	u8 *p;
	if(bits==24)//24bit音频,需要处理一下
	{
		readlen=(size/4)*3;							//此次要读取的字节数
		f_read(audiodev.file,audiodev.tbuf,readlen,(UINT*)&bread);	//读取数据
        p=audiodev.tbuf;
		for(i=0;i<size;)
		{
			buf[i++]=p[1];
			buf[i]=p[2]; 
			i+=2;
			buf[i++]=p[0];
			p+=3;
		} 
		bread=(bread*4)/3;		//填充后的大小.
	}else 
	{
		f_read(audiodev.file,buf,size,(UINT*)&bread);//16bit音频,直接读取数据
		if(bread<size)//不够数据了,补充0
		{
			for(i=bread;i<size-bread;i++)buf[i]=0; 
		}
	}
	return bread;
}  
//WAV播放时,I2S DMA传输回调函数
//WAV播放时,I2S DMA传输回调函数
void wav_i2s_dma_tx_callback(void) 
{   
	wavtransferend=1;	
} 


//播放某个WAV文件
//fname:wav文件路径.
//返回值:
//KEY0_PRES:下一曲
//KEY1_PRES:上一曲
//其他:错误
u8 wav_play_song(u8* fname)
{	
	u8 res;  
	u32 fillnum;
	FIL file;
	u32 mark = 0;
	FRESULT fres = FR_OK;
	u32 stopplay = 0;

	res=wav_decode_init(fname,&wavctrl);//得到文件的信息
	if(res==1)//解析文件成功
	{
		res=0XFF;
		return res;
	}
	res=f_open(&file,(TCHAR*)fname,FA_READ);	//打开文件
	if(res==1)
	{
		res=0XFF;
		return res;
	}
	Gplyfilep = 0;
	
	f_lseek(&file, wavctrl.datastart);		//跳过文件头

	
	memset(&Gauddatabuf[0][0], 0x00, 2*WAV_I2S_TX_DMA_BUFSIZE);
	fres = f_read(&file, &Gauddatabuf[Gplyfilep][0], 2*WAV_I2S_TX_DMA_BUFSIZE, &fillnum);//填充buf1
	if(fres != FR_OK )
		return 0xff;
	if(fillnum <= WAV_I2S_TX_DMA_BUFSIZE)
	{
		mark = 1;
	}
	else if(fillnum < 2*WAV_I2S_TX_DMA_BUFSIZE)
	//if(fillnum < 2*WAV_I2S_TX_DMA_BUFSIZE)
	{
		fillnum -= WAV_I2S_TX_DMA_BUFSIZE;
		stopplay = 1;
	}
	else
	{
		fillnum = WAV_I2S_TX_DMA_BUFSIZE;
	}	
	reset_dma_buf(DMA1_Channel5, &Gauddatabuf[Gplyfilep][0], WAV_I2S_TX_DMA_BUFSIZE/2);
	ConfStartPlay(wavctrl.samplerate, wavctrl.nchannels);
		
	I2S_Play_Start();
	Gplyfilep++;

	while(1)
	{ 
		if(VM8978Status.complete == 1)//中途强制退出播放
     	{ 
			res=1;
			break;
     	}
		if(wavtransferend==1)  
		{
			if(mark)
			{
				break;
			}
			wavtransferend=0;
	
			reset_dma_buf(DMA1_Channel5, &Gauddatabuf[Gplyfilep][0],  WAV_I2S_TX_DMA_BUFSIZE/2);
				
			if(stopplay)
			{
				stopplay++;
				if(stopplay >= 2)
					mark = 1;
				  //break;
			}
										
			if(stopplay == 0)
			{
				Gplyfilep = !Gplyfilep;
				fres = f_read(&file, &Gauddatabuf[Gplyfilep][0], WAV_I2S_TX_DMA_BUFSIZE, &fillnum);//填充buf1
				if(fres != FR_OK )
					break;
			}
			
			if(fillnum != WAV_I2S_TX_DMA_BUFSIZE)//播放结束?
			{
				memset(&Gauddatabuf[Gplyfilep][fillnum], 0x00, WAV_I2S_TX_DMA_BUFSIZE-fillnum);
                mark = 1;
                //				if(stopplay == 0)
					//stopplay = 1;
				// break;
//				else
//				{					
//					Gplyfilep = !Gplyfilep;
//					memset(&Gauddatabuf[Gplyfilep][0], 0x00, WAV_I2S_TX_DMA_BUFSIZE);
//				}
				
			} 			
		}
	}


				
// 	while(1)
// 	{ 
// 		if(VM8978Status.complete == 1)//中途强制退出播放
//      	{ 
// 			res=1;
// 			break;
//      	}
// 		if(wavtransferend==1)  
// 		{
// //			if(mark)
// //			{
// //				break;
// //			}
// 			wavtransferend=0;
// 	
// 			reset_dma_buf(DMA1_Channel5, &Gauddatabuf[Gplyfilep][0],  WAV_I2S_TX_DMA_BUFSIZE/2);
// 				
// //			if(stopplay)
// //			{
// //				stopplay++;
// //				if(stopplay >= 2)
// //					mark = 1;
// //				  //break;
// //			}
// 										
// //			if(stopplay == 0)
// //			{
// 				Gplyfilep = !Gplyfilep;
// 				fres = f_read(&file, &Gauddatabuf[Gplyfilep][0], WAV_I2S_TX_DMA_BUFSIZE, &fillnum);//填充buf1
// 				if(fres != FR_OK )
// 					break;
// //			}
// 			
// 			if(fillnum != WAV_I2S_TX_DMA_BUFSIZE)//播放结束?
// 			{
// 				memset(&Gauddatabuf[Gplyfilep][fillnum], 0x00, WAV_I2S_TX_DMA_BUFSIZE-fillnum);
// //				if(stopplay == 0)
// 					//stopplay = 1;
// 				 break;
// //				else
// //				{					
// //					Gplyfilep = !Gplyfilep;
// //					memset(&Gauddatabuf[Gplyfilep][0], 0x00, WAV_I2S_TX_DMA_BUFSIZE);
// //				}
// 				
// 			} 			
// 		}
// 	}
	audio_stop();
	f_close(&file);

	return res;
}

//停播放某个WAV文件
//fname:wav文件路径.
//返回值:
//KEY0_PRES:下一曲
//KEY1_PRES:上一曲
//其他:错误
u8 wav_play_stop(void)
{
	audio_stop();
	return 0;
}
/********************************************************************************************
* 函数名称：StartPlay ()
* 功能描述：初始化语音播放芯片
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/

void StartPlay_Init(void)
{
	/* 如果已经再录音和放音状态，则需要先停止再开启 */
	i2s_tx_callback=wav_i2s_dma_tx_callback;			//回调函数指wav_i2s_dma_callback
	
	wm8978_Init();		/* 复位WM8978到复位状态 */
	I2S_Stop();			/* 停止I2S录音和放音 */
	
	/* 配置WM8978芯片，输入为DAC，输出为扬声器 */
	wm8978_Dac2Spk();

	/* 调节音量，左右相同音量 */	
	wm8978_ChangeVolume(volume, volume);	//范围：0~64
																				
	/* 配置WM8978音频接口为飞利浦标准I2S接口，16bit */					
	wm8978_CfgAudioIF(I2S_Standard_MSB, 16, I2S_Mode_MasterTx);	
	
	I2S2_DMA_Init(&Gauddatabuf[0][0], WAV_I2S_TX_DMA_BUFSIZE/2); //配置TX DMA


}
