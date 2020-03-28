/*
*********************************************************************************************
*                                   ÃÏΩÚª™ƒ˛µÁ◊””–œﬁπ´Àæ
*                                        «∂»Î Ωø™∑¢◊È
*
*                                
*
* Œƒº˛√˚ : Playlogic.c
* √Ë   ˆ :”Ô“Ùøÿ÷∆≤•∑≈¬ﬂº≠»ŒŒÒ
* ◊˜  ’ﬂ : 
* ∞Ê  ±æ : V1.0
* »’  ∆⁄ :
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
* ∫Ø ˝√˚≥∆£∫DealPlayInstrction 
* π¶ƒ‹√Ë ˆ£∫¥¶¿Ì≤•∑≈÷∏¡Ó
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫Œﬁ
*  π”√Àµ√˜£∫Œﬁ
********************************************************************************/
void DealPlayInstrction(u16 *msg)
{ 
#if OS_CRITICAL_METHOD == 3                      
    OS_CPU_SR  cpu_sr;
#endif
	static u8 instruct = 0;
	static u8 segid = 0;
    instruct = (u8)((*msg & 0x0300) >> 8);
    segid = (u8)(*msg & 0x00FF);
    
	switch(instruct)
	{
        case STOP_ALLFINITE://Õ£÷πÀ˘”–≤ª“ª÷±≤•µƒ
            if((VM8978Status.mode == 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//Õ£÷π∑≈“Ù
                Init_PlayStatus();
			}

            FiniteListBuf.segaddr = 0;//«Âø’”–œﬁ¥Œ≤•∑≈¡–±Ì
            FiniteListBuf.playtype = 0;
            FiniteListBuf.playtimes = 0;
            FiniteListBuf.playprior = 0;
            FiniteListBuf.playinterval = 0;

            FiniteListNodeNum = 0;

        break;

        case STOP_SPCINFINITE://Õ£÷π÷∏∂®“ª÷±≤•µƒ
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                if(VM8978Status.segaddr==segid )
                {
                    wav_play_stop();//Õ£÷π∑≈“Ù
					OS_ENTER_CRITICAL();
                    DelNode(segid);
					OS_EXIT_CRITICAL();
                    Init_PlayStatus();
                    OSTimeDly(20);	
                }  
                else
                {
                    OS_ENTER_CRITICAL();
					DelNode(segid);
					OS_EXIT_CRITICAL();
                }
            }
			else
            {
                OS_ENTER_CRITICAL();
				DelNode(segid);
				OS_EXIT_CRITICAL();
            }
        break;
        case STOP_ALLINFINITE://Õ£÷πÀ˘”–“ª÷±≤•µƒ
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//Õ£÷π∑≈“Ù
                Init_PlayStatus();
				OS_ENTER_CRITICAL();
				Del_AllNode();
				OS_EXIT_CRITICAL();							
            }
            
            Init_FreeList();
            
        break;
				
		default:
			break;
	}
}
/*******************************************************************************
* ∫Ø ˝√˚≥∆£∫DealPlayInstrction 
* π¶ƒ‹√Ë ˆ£∫¥¶¿Ì≤•∑≈÷∏¡Ó
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫Œﬁ
*  π”√Àµ√˜£∫Œﬁ
********************************************************************************/
// void DealPlayInstrction1(u8 temp)
// { 
// #if OS_CRITICAL_METHOD == 3                      
//     OS_CPU_SR  cpu_sr;
// #endif
// 	static u8 instruct = 0;
// //	static u8 segid = 0;
//     instruct = temp;
//     
// 	switch(instruct)
// 	{
//         case STOP_ALLFINITE://Õ£÷πÀ˘”–≤ª“ª÷±≤•µƒ

// 				wav_play_stop();//Õ£÷π∑≈“Ù
// 				Init_PlayStatus();

// 				FiniteListBuf.segaddr = 0;//«Âø’”–œﬁ¥Œ≤•∑≈¡–±Ì
// 				FiniteListBuf.playtype = 0;
// 				FiniteListBuf.playtimes = 0;
// 				FiniteListBuf.playprior = 0;
// 				FiniteListBuf.playinterval = 0;

// 				FiniteListNodeNum = 0;

//         break;

//         case STOP_ALLINFINITE://Õ£÷πÀ˘”–“ª÷±≤•µƒ

//                 wav_play_stop();//Õ£÷π∑≈“Ù
//                 Init_PlayStatus();
// 				OS_ENTER_CRITICAL();
// 				Del_AllNode();
// 				OS_EXIT_CRITICAL();							
//             
// 				Init_FreeList();
//             
//         break;
// 				
// 		default:
// 			break;
// 	}
// }
/********************************************************************************************
* ∫Ø ˝√˚≥∆£∫PlayCondition ()
* π¶ƒ‹√Ë ˆ£∫∑≈“Ù◊¥Ã¨≈–∂®
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫∑≈“Ùƒ£ Ω
*  π”√Àµ√˜£∫Œﬁ
********************************************************************************************/
u8 PlayCondition(void)
{	
	if((VM8978Status.allcomplete==1)&&(FiniteListBuf.blockvalid==0))
	{
        VM8978Status.allcomplete = 0;
		 if(FiniteList.blockvalid !=1)
        {
            FiniteListNodeNum=0;
        }
	}
	
    if((FiniteListNodeNum > 0)&&(InFiniteListNodeNum == 0))//Ωˆ”–œﬁ¥Œ≤•∑≈¡–±Ì∑«ø’
    {
        return FINITE_ONLY;
    }
    if((FiniteListNodeNum == 0)&&(InFiniteListNodeNum > 0))//Ωˆ÷ÿ∏¥≤•∑≈¡–±Ì∑«ø’
    {
        return INFINITE_ONLY;
    }
    if((FiniteListNodeNum > 0)&&(InFiniteListNodeNum > 0))//º»”–÷ÿ∏¥≤•±®£¨”÷”–”–œﬁ¥Œ≤•±®
    {
        return FINITE_INFINITE_BOTH;
    }
    

	return IDEL;
}
/********************************************************************************************
* ∫Ø ˝√˚≥∆£∫CopyInfor
* π¶ƒ‹√Ë ˆ£∫øΩ±¥…˘“Ù∆¨∂Œ–≈œ¢µΩ≤•∑≈ª˙
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫Œﬁ
*  π”√Àµ√˜£∫Œﬁ
********************************************************************************************/
void CopyInfor( _PlayICBBuf *point)
{
    VM8978Status.mode = (*point).playtype;
    VM8978Status.playtimes = (*point).playtimes;
    VM8978Status.timeinterval = (*point).playinterval;
    VM8978Status.segaddr = (*point).segaddr;
    VM8978Status.PlayStatus = 1;
	(*point).blockvalid=0;
}
/********************************************************************************************
* ∫Ø ˝√˚≥∆£∫CopyFinInfor
* π¶ƒ‹√Ë ˆ£∫øΩ±¥”–œﬁ¥Œ…˘“Ù∆¨∂Œ–≈œ¢µΩ”–œﬁ¥ŒΩ·ππÃÂª∫¥Ê÷–
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫Œﬁ
*  π”√Àµ√˜£∫Œﬁ
********************************************************************************************/
void CopyFinInfor( _PlayICBBuf *point)
{
	FiniteListBuf.segaddr = (*point).segaddr;
    FiniteListBuf.playtype = (*point).playtype;
    FiniteListBuf.playtimes = (*point).playtimes;
    FiniteListBuf.playprior = (*point).playprior;
    FiniteListBuf.playinterval = (*point).playinterval;
	FiniteListBuf.blockvalid = (*point).blockvalid;
}
/*******************************************************************************
* ∫Ø ˝√˚≥∆£∫VM8978StartPlay
* π¶ƒ‹√Ë ˆ: VM8978ø™ º≤•∑≈
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫Œﬁ
*  π”√Àµ√˜£∫
********************************************************************************/
void VM8978StartPlay(void)
{
    VM8978Status.complete = 0;
}


/********************************************************************************************
* ∫Ø ˝√˚≥∆£∫TaskPlayLogic ()
* π¶ƒ‹√Ë ˆ£∫ ‰≥ˆ…¡µ∆
* »Îø⁄≤Œ ˝£∫Œﬁ
* ≥ˆø⁄≤Œ ˝£∫Œﬁ
*  π”√Àµ√˜£∫Œﬁ
********************************************************************************************/
void TaskPlayLogic(void *pdata)
{
	#if OS_CRITICAL_METHOD == 3                    
    OS_CPU_SR  cpu_sr = 0;
    #endif

	u8 err;
    u16* msg;
    u8 PlayJudgement = 0;
    u8 tmpjudgement;
    u8 p[2] = {0};

    Init_PlayStatus();
    ADC1Init_Base();//≥ı ºªØADC
    DMAADCInit_Base();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    OSTimeDly(10);
	while(1)
    {
        msg = (u16 *)OSQAccept(MsgQueue,&err);
		if(msg != (void *)0)//≤•∑≈¡–±ÌŒ¨ª§
		{
			DealPlayInstrction(msg);
		}
        ADCfliter(ADCConvertedValue,ADCBUFFER);
        if(VolumeCtr.VolumeNow != VolumeCtr.VolumePre)//“Ù¡øøÿ÷∆
        {
			VolumeCtr.VolumePre = VolumeCtr.VolumeNow;
			wm8978_ChangeVolume(VolumeCtr.VolumeNow,VolumeCtr.VolumeNow);
        }
        tmpjudgement = PlayCondition();
        switch(PlayJudgement)
        {
            case IDEL_PLAY: 
				if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//¡Ω÷÷”Ô“Ù∂º¥˝≤•∑≈
				{
					CopyFinInfor(&FiniteList);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ª∫¥Ê÷–≤
					CopyInfor(&FiniteListBuf);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ic£
					PlayJudgement = FINITE_PLAY;
					break;
				}
				else if(tmpjudgement == INFINITE_ONLY)//÷ª”–÷ÿ∏¥≤•∑≈”Ô“Ù
				{
					InFiniteListCurPlay = InFiniteListHead;//≤•∑≈÷∏’Î∏˙ÀÊÕ∑÷∏’Î
					CopyInfor(InFiniteListCurPlay);
					PlayJudgement = INFINITE_PLAY;
					break;
				}
				Init_PlayStatus();//«Âø’≤•∑≈ª˙–≈œ¢
				break;
            case FINITE_PLAY:
                if(tmpjudgement == INFINITE_ONLY)//÷ª £÷ÿ∏¥≤•∑≈¿‡–Õ”Ô“Ù°§
                {
                    CopyInfor(InFiniteListCurPlay);
					PlayJudgement = INFINITE_PLAY;
                    break;
                }
                else if(tmpjudgement == IDEL)//≤•∑≈ÕÍ≥…
                {
                    PlayJudgement = IDEL_PLAY;
                    break;
                }
                if(VM8978Status.complete == 1)//≤•∑≈ÕÍ±œ¥¶¿Ì
                {   
 					if(FiniteList.blockvalid==1)
					{	
						CopyFinInfor(&FiniteList);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ª∫¥Ê÷–?
						CopyInfor(&FiniteListBuf);
						FiniteList.blockvalid=0;
					}
                    if(FiniteListBuf.playtimes > 0)//∆¨∂Œ≤•∑≈¥Œ ˝Œ¥«Â≥˝
                    {
                        VM8978Status.complete = 0;
                        FiniteListBuf.playtimes--;
                        CopyInfor(&FiniteListBuf);//‘Ÿ¥Œ‘ÿ»Î¥˝≤•∑≈”Ô“Ù
                        VM8978StartPlay();                     
                    }
                    else
                    {
                        CAN_TxFrame(FRAMEID_LOWTOUP_STOPACK,p,1);
						VM8978Status.allcomplete=1;//’˚∏ˆ”Ô“Ù∂ŒÕÍ»´≤•∑≈ÕÍ±œ
                        break;
                    }
					if((FiniteList.segaddr == VM8978Status.segaddr)&&(FiniteNodeSame==1))//20160926–ﬁ∏ƒ≥…œ‡Õ¨µƒ”Ô“Ù£¨µ•¥ ≤•∑≈ÕÍ≥…∫Û≤≈ƒ‹≤•∑≈–¬µƒ”Ô“Ù°£
					{
						wav_play_stop();//Õ£÷π∑≈“Ù
						Init_PlayStatus();//«Âø’≤•∑≈ª˙–≈œ¢
						OSTimeDly(20);	
						CopyFinInfor(&FiniteList);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ª∫¥Ê÷–¥
						CopyInfor(&FiniteListBuf);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ic£
						FiniteNodeSame=2; //≤•∑≈–¬µƒ≤•∑≈÷∏¡Ó
					}
                }
//                if(FiniteList.segaddr != VM8978Status.segaddr)//Ω” ’µΩ–¬µƒ≤•∑≈÷∏¡Ó
//                {
//                    wav_play_stop();//Õ£÷π∑≈“Ù
//                    Init_PlayStatus();//«Âø’≤•∑≈ª˙–≈œ¢
//                    OSTimeDly(20);
//					CopyFinInfor(&FiniteList);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ª∫¥Ê÷–
//                    CopyInfor(&FiniteListBuf);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ic≈
// 					FiniteList.blockvalid=0;
//                    break;
//                }
            break;
            case INFINITE_PLAY:
                if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//¡Ω÷÷”Ô“Ù∂º”–£¨”≈œ»≤•∑≈”–œﬁ¥Œ”Ô“Ù
                {
                    wav_play_stop();//Õ£÷πµ±«∞≤•∑≈µƒ
                    Init_PlayStatus();//«Âø’≤•∑≈ª˙–≈œ¢
                    OSTimeDly(20);
					CopyFinInfor(&FiniteList);
                    CopyInfor(&FiniteListBuf);//øΩ±¥≤•∑≈–≈œ¢µΩ≤•∑≈ic£¨≤¢ø™ º≤•∑≈
                    PlayJudgement = FINITE_PLAY;                    
                    break;
                }
                else if(tmpjudgement == IDEL)
                {
                    wav_play_stop();//Õ£÷πµ±«∞≤•∑≈µƒ
                    Init_PlayStatus();//«Âø’≤•∑≈ª˙–≈œ¢
                    PlayJudgement = IDEL_PLAY;
                    break;
                }
                if(VM8978Status.complete == 1)//—≠ª∑»°œ¬“ª∂Œ”Ô“Ù
                {
                    if(SoftTimer.PlayIntervalTimer == 0)
                    {
						CopyInfor(InFiniteListCurPlay);
                        VM8978StartPlay(); 
                        OS_ENTER_CRITICAL();
                        if(InFiniteListCurPlay->next != InFiniteListTail)//√ª”–µΩ◊Ó∫Û“ª∏ˆ≥…‘±£¨‘Ú“∆œÚœ¬“ª≥…‘±
                        {
                            InFiniteListCurPlay = InFiniteListCurPlay->next;
							VM8978StartPlay();
                        }   
                        else//µ±«∞ «◊Ó∫Û“ª∏ˆ≥…‘±£¨‘Ú“∆œÚ∂”Õ∑
                        {
                            SoftTimer.PlayIntervalTimer=PLAYALLDELAY;
							InFiniteListCurPlay = InFiniteListHead;
                        }
                        OS_EXIT_CRITICAL(); 

                    }
                }
            break;
				
			default:
				break;
        }
       OSTimeDly(10);	
    }
}
