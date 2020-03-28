/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 文件名 : task2.c
* 描  述 :控制光耦输入，即指示灯交替闪烁任务
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
*********************************************************************************************
*/
//#include "ucos_ii.h"
//#include "stm32f10x.h"
//#include "target.h"
#include "include.h"

#define IDEL                    0
#define FINITE_ONLY             1
#define INFINITE_ONLY           2
#define FINITE_INFINITE_BOTH    3

#define IDEL_PLAY               0
#define FINITE_PLAY             1
#define INFINITE_PLAY           2


/*******************************************************************************
* 函数名称：DealPlayInstrction 
* 功能描述：处理播放指令
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************/
void DealPlayInstrction(u16 *msg)
{
	static u8 DealResult;
    u8 instruct = (*msg&0x0300)>>8;
    u8 segid = (*msg&0x00FF);
    
	switch(instruct){
        case STOP_ALLFINITE://停止所有不一直播的
            if((VM8978Status.mode == 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//停止放音
                Init_PlayStatus();
            }
            //wav_play_stop();//停止放音

            FiniteList.segaddr = 0;//清空有限次播放列表
            FiniteList.playtype = 0;
            FiniteList.playtimes = 0;
            FiniteList.playprior = 0;
            FiniteList.playinterval = 0;

            FiniteListNodeNum = 0;

            //Init_PlayStatus();

            DealResult = STOP_ACK_OK;

        break;

        case STOP_SPCINFINITE://停止指定一直播的
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                if(VM8978Status.segaddr==segid )
                {
                    wav_play_stop();//停止放音
                    DelNode(segid);
                    Init_PlayStatus();
                    OSTimeDly(20);	
                }  
                else
                {
                    DealResult =  DelNode(segid);
                }
            }
            else
            {
                DealResult =  DelNode(segid);
            }
        break;
        case STOP_ALLINFINITE://停止所有一直播的
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//停止放音
                Init_PlayStatus();
            }
            
            Init_FreeList();
            
            
            DealResult = STOP_ACK_OK;
        break;
		default:

		break;
	
	}


}
/********************************************************************************************
* 函数名称：PlayCondition ()
* 功能描述：放音状态判定
* 入口参数：无
* 出口参数：放音模式
* 使用说明：无
********************************************************************************************/
u8 PlayCondition(void)
{
    if((FiniteListNodeNum > 0)&&(InFiniteListNodeNum == 0))//仅有限次播放列表非空
    {
        return FINITE_ONLY;
    }
    if((FiniteListNodeNum == 0)&&(InFiniteListNodeNum > 0))//仅重复播放列表非空
    {
        return INFINITE_ONLY;
    }
    if((FiniteListNodeNum > 0)&&(InFiniteListNodeNum > 0))//既有重复播报，又有有限次播报
    {
        return FINITE_INFINITE_BOTH;
    }
    return IDEL;
}
/********************************************************************************************
* 函数名称：CopyInfor
* 功能描述：拷贝声音片段信息到播放机
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void CopyInfor( _PlayICBBuf *point)
{
    VM8978Status.mode = (*point).playtype;
    VM8978Status.playtimes = (*point).playtimes;
    VM8978Status.timeinterval = (*point).playinterval;
    VM8978Status.segaddr = (*point).segaddr;
    VM8978Status.PlayStatus = 1;
}
/*******************************************************************************
* 函数名称：VM8978StartPlay
* 功能描述: VM8978开始播放
* 入口参数：无
* 出口参数：无
* 使用说明：
********************************************************************************/
void VM8978StartPlay(void)
{
    VM8978Status.complete = 0;
}


/********************************************************************************************
* 函数名称：TaskPlayLogic ()
* 功能描述：输出闪灯
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TaskPlayLogic(void *pdata)
{
	#if OS_CRITICAL_METHOD == 3                    
    OS_CPU_SR  cpu_sr = 0;
    #endif

	u8 PlayErr=0;
    char *filename;
    u8 prior;
	u8 err;
    u16* msg;
    u8 PlayJudgement = 0;
    u8 tmpjudgement;

    Init_FreeList();
    Init_PlayStatus();
    ADC1Init_Base();//初始化ADC
    DMAADCInit_Base();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    OSTimeDly(10);
	while(1)
    {
        msg = (u16 *)OSQAccept(MsgQueue,&err);
		if(msg != (void *)0)//播放列表维护
        {
			DealPlayInstrction(msg);
		}
        ADCfliter(ADCConvertedValue,ADCBUFFER);
        if(VolumeCtr.VolumeNow != VolumeCtr.VolumePre)//音量控制
        {
            VolumeCtr.VolumePre = VolumeCtr.VolumeNow;
            wm8978_ChangeVolume(VolumeCtr.VolumeNow,VolumeCtr.VolumeNow);
        }
        tmpjudgement = PlayCondition();
        switch(PlayJudgement)
        {
            case IDEL_PLAY:
                //tmpjudgement = PlayCondition();
                if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//两种语音都待播放
                {
                    CopyInfor(&FiniteList);//拷贝播放信息到播放ic，并开始播放
//                     VM8978StartPlay();
                    PlayJudgement = FINITE_PLAY;
                    break;
                }
                else if(tmpjudgement == INFINITE_ONLY)//只有重复播放语音
                {
                    CopyInfor(InFiniteListCurPlay);
                    InFiniteListCurPlay->blockactive = 1;
                    VM8978StartPlay();
                    OS_ENTER_CRITICAL();
                    if(InFiniteListCurPlay->next != InFiniteListTail)//没有到最后一个成员，则移向下一成员
                    {
                        InFiniteListCurPlay = InFiniteListCurPlay->next;
                    }
                    else//当前是最后一个成员，则移向队头
                    {
                        InFiniteListCurPlay = InFiniteListHead;
                    }
                    OS_EXIT_CRITICAL();

                    PlayJudgement = INFINITE_PLAY;
                    break;
                }
                Init_PlayStatus();//清空播放机信息
            break;
            case FINITE_PLAY:
                //tmpjudgement = PlayCondition();
                if(tmpjudgement == INFINITE_ONLY)//只剩重复播放类型语音·
                {
                    CopyInfor(InFiniteListCurPlay);
                    InFiniteListCurPlay->blockactive = 1;
                    VM8978StartPlay();

                    PlayJudgement = INFINITE_PLAY;
                    break;
                }
                else if(tmpjudgement == IDEL)//播放完成
                {
                    PlayJudgement = IDEL_PLAY;
                    break;
                }
                if(VM8978Status.complete == 1)//播放完毕处理
                {
                    if(FiniteList.playtimes > 0)//片段播放次数未清除
                    {
                        VM8978Status.complete = 0;
                        FiniteList.playtimes--;
                        CopyInfor(&FiniteList);//再次载入待播放语音
                        VM8978StartPlay();                     
                    }
                    else
                    {
                        FiniteListNodeNum = 0;
                        //PlayJudgement = IDEL_PLAY;
                        break;
                    }
                }
                if(FiniteList.segaddr != VM8978Status.segaddr)//接收到新的播放指令
                {
                    wav_play_stop();//停止放音
                    Init_PlayStatus();//清空播放机信息
                    OSTimeDly(20);	
                    CopyInfor(&FiniteList);//拷贝播放信息到播放ic，并开始播放
//                     VM8978StartPlay();

                    //VM8978Status.complete = 1;//强制认为播放完成
                }

            break;
            case INFINITE_PLAY:
                //tmpjudgement = PlayCondition();
                if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//两种语音都有，优先播放有限次语音
                {
                    wav_play_stop();//停止当前播放的
                    Init_PlayStatus();//清空播放机信息
                    OSTimeDly(20);
                    CopyInfor(&FiniteList);//拷贝播放信息到播放ic，并开始播放
//                     VM8978StartPlay();
                    PlayJudgement = FINITE_PLAY;                    
                    break;
                }
                else if(tmpjudgement == IDEL)
                {
                    wav_play_stop();//停止当前播放的
                    Init_PlayStatus();//清空播放机信息
                    PlayJudgement = IDEL_PLAY;
                    break;
                }
                if(VM8978Status.complete == 1)//循环取下一段语音
                {
                    if(SoftTimer.PlayIntervalTimer == 0)
                    {
                        VM8978Status.complete = 0;
                        OS_ENTER_CRITICAL();
                        if(InFiniteListCurPlay->next != InFiniteListTail)//没有到最后一个成员，则移向下一成员
                        {
                            InFiniteListCurPlay = InFiniteListCurPlay->next;
                        }   
                        else//当前是最后一个成员，则移向队头
                        {
                            InFiniteListCurPlay = InFiniteListHead;
                        }
                        OS_EXIT_CRITICAL();
                        CopyInfor(InFiniteListCurPlay);
                        //InFiniteListCurPlay->blockactive = 1;
                        VM8978StartPlay();                            
                    }
                }
            break;
            default:

            break;
        }
        OSTimeDly(10);	
    }
}
