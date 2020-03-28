/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : wavplay.c
* ��  �� :����������Ӧ�ò㺯��
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
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
//V1.0 ˵��
//1,֧��16λ/24λWAV�ļ�����
//2,��߿���֧�ֵ�192K/24bit��WAV��ʽ. 
////////////////////////////////////////////////////////////////////////////////// 	
 
__wavctrl wavctrl;		//WAV���ƽṹ��
vu8 wavtransferend=0;	//i2s������ɱ�־
vu8 wavwitchbuf=0;		//i2sbufxָʾ��־
 

static uint8_t 			Gauddatabuf[2][WAV_I2S_TX_DMA_BUFSIZE];  
static uint32_t         Gplyfilep = 0;

//u8 audiofilm[1024];
//u8 audiobuf1[WAV_I2S_TX_DMA_BUFSIZE];
//u8 audiobuf2[WAV_I2S_TX_DMA_BUFSIZE];

//���ֲ��ſ�����
__audiodev audiodev;	  
 

//��ʼ��Ƶ����
void audio_start(void)
{
	audiodev.status=3<<0;//��ʼ����+����ͣ
	I2S_Play_Start();
} 
//�ر���Ƶ����
void audio_stop(void)
{
	audiodev.status=0;
	I2S_Play_Stop();
}  

 
//WAV������ʼ��
//fname:�ļ�·��+�ļ���
//wavx:wav ��Ϣ��Žṹ��ָ��
//����ֵ:0,�ɹ�;1,���ļ�ʧ��;2,��WAV�ļ�;3,DATA����δ�ҵ�.
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
	if(ftemp&&buf)	//�ڴ�����ɹ�
	{
		res=f_open(ftemp,(TCHAR*)fname,FA_READ);//���ļ�
		if(res==FR_OK)
		{
			f_read(ftemp,buf,512,&br);	//��ȡ512�ֽ�������
			riff=(ChunkRIFF *)buf;		//��ȡRIFF��
			if(riff->Format==0X45564157)//��WAV�ļ�
			{
				fmt=(ChunkFMT *)(buf+12);	//��ȡFMT�� 
				fact=(ChunkFACT *)(buf+12+8+fmt->ChunkSize);//��ȡFACT��
				if(fact->ChunkID==0X74636166||fact->ChunkID==0X5453494C)wavx->datastart=12+8+fmt->ChunkSize+8+fact->ChunkSize;//����fact/LIST���ʱ��(δ����)
				else wavx->datastart=12+8+fmt->ChunkSize;  
				data=(ChunkDATA *)(buf+wavx->datastart);	//��ȡDATA��
				if(data->ChunkID==0X61746164)//�����ɹ�!
				{
					wavx->audioformat=fmt->AudioFormat;		//��Ƶ��ʽ
					wavx->nchannels=fmt->NumOfChannels;		//ͨ����
					wavx->samplerate=fmt->SampleRate;		//������
					wavx->bitrate=fmt->ByteRate*8;			//�õ�λ��
					wavx->blockalign=fmt->BlockAlign;		//�����
					wavx->bps=fmt->BitsPerSample;			//λ��,16/24/32λ
					
					wavx->datasize=data->ChunkSize;			//���ݿ��С
					wavx->datastart=wavx->datastart+8;		//��������ʼ�ĵط�. 
					 
				}else res=3;//data����δ�ҵ�.
			}else res=2;//��wav�ļ�
			
		}else res=1;//���ļ�����
	}
	f_close(ftemp);
	myfree(SRAMIN,ftemp);//�ͷ��ڴ�
	myfree(SRAMIN,buf); 
	return res;
}

//���buf
//buf:������
//size:���������
//bits:λ��(16/24)
//����ֵ:���������ݸ���
u32 wav_buffill(u8 *buf,u16 size,u8 bits)
{
	u16 readlen=0;
	volatile u32 bread = 0;
	u16 i;
	u8 *p;
	if(bits==24)//24bit��Ƶ,��Ҫ����һ��
	{
		readlen=(size/4)*3;							//�˴�Ҫ��ȡ���ֽ���
		f_read(audiodev.file,audiodev.tbuf,readlen,(UINT*)&bread);	//��ȡ����
        p=audiodev.tbuf;
		for(i=0;i<size;)
		{
			buf[i++]=p[1];
			buf[i]=p[2]; 
			i+=2;
			buf[i++]=p[0];
			p+=3;
		} 
		bread=(bread*4)/3;		//����Ĵ�С.
	}else 
	{
		f_read(audiodev.file,buf,size,(UINT*)&bread);//16bit��Ƶ,ֱ�Ӷ�ȡ����
		if(bread<size)//����������,����0
		{
			for(i=bread;i<size-bread;i++)buf[i]=0; 
		}
	}
	return bread;
}  
//WAV����ʱ,I2S DMA����ص�����
//WAV����ʱ,I2S DMA����ص�����
void wav_i2s_dma_tx_callback(void) 
{   
	wavtransferend=1;	
} 


//����ĳ��WAV�ļ�
//fname:wav�ļ�·��.
//����ֵ:
//KEY0_PRES:��һ��
//KEY1_PRES:��һ��
//����:����
u8 wav_play_song(u8* fname)
{	
	u8 res;  
	u32 fillnum;
	FIL file;
	u32 mark = 0;
	FRESULT fres = FR_OK;
	u32 stopplay = 0;

	res=wav_decode_init(fname,&wavctrl);//�õ��ļ�����Ϣ
	if(res==1)//�����ļ��ɹ�
	{
		res=0XFF;
		return res;
	}
	res=f_open(&file,(TCHAR*)fname,FA_READ);	//���ļ�
	if(res==1)
	{
		res=0XFF;
		return res;
	}
	Gplyfilep = 0;
	
	f_lseek(&file, wavctrl.datastart);		//�����ļ�ͷ

	
	memset(&Gauddatabuf[0][0], 0x00, 2*WAV_I2S_TX_DMA_BUFSIZE);
	fres = f_read(&file, &Gauddatabuf[Gplyfilep][0], 2*WAV_I2S_TX_DMA_BUFSIZE, &fillnum);//���buf1
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
		if(VM8978Status.complete == 1)//��;ǿ���˳�����
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
				fres = f_read(&file, &Gauddatabuf[Gplyfilep][0], WAV_I2S_TX_DMA_BUFSIZE, &fillnum);//���buf1
				if(fres != FR_OK )
					break;
			}
			
			if(fillnum != WAV_I2S_TX_DMA_BUFSIZE)//���Ž���?
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
// 		if(VM8978Status.complete == 1)//��;ǿ���˳�����
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
// 				fres = f_read(&file, &Gauddatabuf[Gplyfilep][0], WAV_I2S_TX_DMA_BUFSIZE, &fillnum);//���buf1
// 				if(fres != FR_OK )
// 					break;
// //			}
// 			
// 			if(fillnum != WAV_I2S_TX_DMA_BUFSIZE)//���Ž���?
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

//ͣ����ĳ��WAV�ļ�
//fname:wav�ļ�·��.
//����ֵ:
//KEY0_PRES:��һ��
//KEY1_PRES:��һ��
//����:����
u8 wav_play_stop(void)
{
	audio_stop();
	return 0;
}
/********************************************************************************************
* �������ƣ�StartPlay ()
* ������������ʼ����������оƬ
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/

void StartPlay_Init(void)
{
	/* ����Ѿ���¼���ͷ���״̬������Ҫ��ֹͣ�ٿ��� */
	i2s_tx_callback=wav_i2s_dma_tx_callback;			//�ص�����ָwav_i2s_dma_callback
	
	wm8978_Init();		/* ��λWM8978����λ״̬ */
	I2S_Stop();			/* ֹͣI2S¼���ͷ��� */
	
	/* ����WM8978оƬ������ΪDAC�����Ϊ������ */
	wm8978_Dac2Spk();

	/* ����������������ͬ���� */	
	wm8978_ChangeVolume(volume, volume);	//��Χ��0~64
																				
	/* ����WM8978��Ƶ�ӿ�Ϊ�����ֱ�׼I2S�ӿڣ�16bit */					
	wm8978_CfgAudioIF(I2S_Standard_MSB, 16, I2S_Mode_MasterTx);	
	
	I2S2_DMA_Init(&Gauddatabuf[0][0], WAV_I2S_TX_DMA_BUFSIZE/2); //����TX DMA


}
