/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : PLAYSOUND.c
* ��  �� :�����������񣬶�ȡSD�����ݣ����͸�WM8978
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
*********************************************************************************************
*/
#include "string.h"
#include "ucos_ii.h"
#include "stm32f10x.h"
#include "target.h"
#include "ff.h"
#include <diskio.h>
#include <stdio.h>
#include <string.h>
#include "wavplay.h"
#include "bsp_wm8978.h"
#include  "config.h"
#include "voice.h"


#define VM8978_IDEL                0
#define VM8978_BUSY                1
#define VM8978_INTERVAL            2
//-------------------------------����������-----------------------------------

FATFS fsw[2];//�߼����̹�����.	 
FIL filew;	  		//�ļ�1
FIL ftempw;	  		//�ļ�2.
UINT brw,bww;			//��д����
FILINFO fileinfow;	//�ļ���Ϣ
DIR dirw;  			//Ŀ¼
u8 *fatbufw;			//SD�����ݻ�����
FRESULT resw;

u8 Desiremusicnum = 0;
u8 musicnum;
//static char PlayingFilename[40]="0:/";

extern __wavctrl wavctrl;		//WAV���ƽṹ��
//-------------------------------����������------------------------------------

u8 ReadPlayMusic(u8 );
char *Fand_file ( char* path);
FRESULT scan_files ( char* );

/********************************************************************************************
* �������ƣ� TaskPLAYSOUND ()
* ����������������������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TaskPLAYSOUND(void *pdata)
{
	uint8_t PlayErr=0;
	StartPlay_Init();//��ʼ������оƬ
	PlayErr =f_mount(0,&fsw[0]);
	if(PlayErr!= FR_OK)
	{
		PlayErr=1;
		
    }
    VM8978Status.complete =1;
    for(;;)
    {		
        if(VM8978Status.complete == 0)//��ǰVM8978�޴���Ƭ��
        {									
			Desiremusicnum = VM8978Status.segaddr;
            PlayMusic(Desiremusicnum);
		}
		
		OSTimeDly(10);
    }
}

//����Ƭ�β���
u8 PlayMusic(u8 NI)
{
	uint8_t PlayErr=0;
	char *filename;
	char fileHead[40]="0:/";
	Desiremusicnum = NI;
	filename = Fand_file("0:/");
	if(	*filename == 0x00)
	{
		VM8978Status.complete = 1;//�ò�����ϱ�־
	}
	strcat(fileHead,filename);
	PlayErr = wav_play_song((u8*)fileHead);
	VM8978Status.complete = 1;//�ò�����ϱ�־
	return PlayErr;
}
//������λ��ָ��Ѱ��Ҫ���ŵ��ļ�
char *Fand_file ( char* path)
{
	uint8_t musicnumhigh;
    uint8_t musicnumlow;
    FRESULT res;
    FILINFO fno;
    DIR dir;
	uint32_t fname_test1;
	char *fn;
    
    #if _USE_LFN
    static char lfn[_MAX_LFN + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
    #endif
	fname_test1=fname_test1;
    res = f_opendir(&dir, path);                     //��Ŀ¼
    if (res == FR_OK) 
    {
        for (;;) 
        {
            res = f_readdir(&dir, &fno);             //��ȡĿ¼���ļ������ļ���
            if(res!=FR_OK||fno.fname[0]==0)
            {
                *fn=0;
                break;
            }
            fname_test1 = fno.fname[0];

    #if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;	//���ļ�����ȡ
    #else
            fn = fno.fname;
    #endif
            if((fno.fname[0]>='0')&&(fno.fname[0]<='9'))
            {
                musicnumhigh=fno.fname[0]-'0';
            } 
            if((fno.fname[0]>='A')&&(fno.fname[0]<='F'))
            {
                musicnumhigh=fno.fname[0]-'A'+10;
            }
            if((fno.fname[1]>='0')&&(fno.fname[1]<='9'))
            {
                musicnumlow=fno.fname[1]-'0';
            } 
            if((fno.fname[1]>='A')&&(fno.fname[1]<='F'))
            {
                musicnumlow=fno.fname[1]-'A'+10;
            }
            musicnum=(musicnumhigh<<4)|musicnumlow;
            if ((res != FR_OK) || (musicnum == Desiremusicnum)) //��ȡ������߶�ȡ���
            {								
                break;
            }		
      }
  }
    return fn;
}


