/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 文件名 : PLAYSOUND.c
* 描  述 :播放声音任务，读取SD卡数据，发送给WM8978
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
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
//-------------------------------变量声明区-----------------------------------

FATFS fsw[2];//逻辑磁盘工作区.	 
FIL filew;	  		//文件1
FIL ftempw;	  		//文件2.
UINT brw,bww;			//读写变量
FILINFO fileinfow;	//文件信息
DIR dirw;  			//目录
u8 *fatbufw;			//SD卡数据缓存区
FRESULT resw;

u8 Desiremusicnum = 0;
u8 musicnum;
//static char PlayingFilename[40]="0:/";

extern __wavctrl wavctrl;		//WAV控制结构体
//-------------------------------函数声明区------------------------------------

u8 ReadPlayMusic(u8 );
char *Fand_file ( char* path);
FRESULT scan_files ( char* );

/********************************************************************************************
* 函数名称： TaskPLAYSOUND ()
* 功能描述：语音播放任务
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TaskPLAYSOUND(void *pdata)
{
	uint8_t PlayErr=0;
	StartPlay_Init();//初始化播放芯片
	PlayErr =f_mount(0,&fsw[0]);
	if(PlayErr!= FR_OK)
	{
		PlayErr=1;
		
    }
    VM8978Status.complete =1;
    for(;;)
    {		
        if(VM8978Status.complete == 0)//当前VM8978无待放片段
        {									
			Desiremusicnum = VM8978Status.segaddr;
            PlayMusic(Desiremusicnum);
		}
		
		OSTimeDly(10);
    }
}

//语音片段播放
u8 PlayMusic(u8 NI)
{
	uint8_t PlayErr=0;
	char *filename;
	char fileHead[40]="0:/";
	Desiremusicnum = NI;
	filename = Fand_file("0:/");
	if(	*filename == 0x00)
	{
		VM8978Status.complete = 1;//置播放完毕标志
	}
	strcat(fileHead,filename);
	PlayErr = wav_play_song((u8*)fileHead);
	VM8978Status.complete = 1;//置播放完毕标志
	return PlayErr;
}
//根据上位机指令寻找要播放的文件
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
    res = f_opendir(&dir, path);                     //打开目录
    if (res == FR_OK) 
    {
        for (;;) 
        {
            res = f_readdir(&dir, &fno);             //读取目录下文件或者文件夹
            if(res!=FR_OK||fno.fname[0]==0)
            {
                *fn=0;
                break;
            }
            fname_test1 = fno.fname[0];

    #if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;	//长文件名提取
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
            if ((res != FR_OK) || (musicnum == Desiremusicnum)) //读取错误或者读取完毕
            {								
                break;
            }		
      }
  }
    return fn;
}


