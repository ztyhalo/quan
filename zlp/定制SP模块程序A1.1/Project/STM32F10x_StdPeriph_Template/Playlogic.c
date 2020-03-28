/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : Playlogic.c
* ��  �� :�������Ʋ����߼�����
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
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
* �������ƣ�DealPlayInstrction 
* ����������������ָ��
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
        case STOP_ALLFINITE://ֹͣ���в�һֱ����
            if((VM8978Status.mode == 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//ֹͣ����
                Init_PlayStatus();
			}

            FiniteListBuf.segaddr = 0;//������޴β����б�
            FiniteListBuf.playtype = 0;
            FiniteListBuf.playtimes = 0;
            FiniteListBuf.playprior = 0;
            FiniteListBuf.playinterval = 0;

            FiniteListNodeNum = 0;

        break;

        case STOP_SPCINFINITE://ָֹͣ��һֱ����
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                if(VM8978Status.segaddr==segid )
                {
                    wav_play_stop();//ֹͣ����
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
        case STOP_ALLINFINITE://ֹͣ����һֱ����
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//ֹͣ����
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
* �������ƣ�DealPlayInstrction 
* ����������������ָ��
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
//         case STOP_ALLFINITE://ֹͣ���в�һֱ����

// 				wav_play_stop();//ֹͣ����
// 				Init_PlayStatus();

// 				FiniteListBuf.segaddr = 0;//������޴β����б�
// 				FiniteListBuf.playtype = 0;
// 				FiniteListBuf.playtimes = 0;
// 				FiniteListBuf.playprior = 0;
// 				FiniteListBuf.playinterval = 0;

// 				FiniteListNodeNum = 0;

//         break;

//         case STOP_ALLINFINITE://ֹͣ����һֱ����

//                 wav_play_stop();//ֹͣ����
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
* �������ƣ�PlayCondition ()
* ��������������״̬�ж�
* ��ڲ�������
* ���ڲ���������ģʽ
* ʹ��˵������
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
	
    if((FiniteListNodeNum > 0)&&(InFiniteListNodeNum == 0))//�����޴β����б�ǿ�
    {
        return FINITE_ONLY;
    }
    if((FiniteListNodeNum == 0)&&(InFiniteListNodeNum > 0))//���ظ������б�ǿ�
    {
        return INFINITE_ONLY;
    }
    if((FiniteListNodeNum > 0)&&(InFiniteListNodeNum > 0))//�����ظ��������������޴β���
    {
        return FINITE_INFINITE_BOTH;
    }
    

	return IDEL;
}
/********************************************************************************************
* �������ƣ�CopyInfor
* ������������������Ƭ����Ϣ�����Ż�
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
* �������ƣ�CopyFinInfor
* �����������������޴�����Ƭ����Ϣ�����޴νṹ�建����
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
* �������ƣ�VM8978StartPlay
* ��������: VM8978��ʼ����
* ��ڲ�������
* ���ڲ�������
* ʹ��˵����
********************************************************************************/
void VM8978StartPlay(void)
{
    VM8978Status.complete = 0;
}


/********************************************************************************************
* �������ƣ�TaskPlayLogic ()
* �����������������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
    ADC1Init_Base();//��ʼ��ADC
    DMAADCInit_Base();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    OSTimeDly(10);
	while(1)
    {
        msg = (u16 *)OSQAccept(MsgQueue,&err);
		if(msg != (void *)0)//�����б�ά��
		{
			DealPlayInstrction(msg);
		}
        ADCfliter(ADCConvertedValue,ADCBUFFER);
        if(VolumeCtr.VolumeNow != VolumeCtr.VolumePre)//��������
        {
			VolumeCtr.VolumePre = VolumeCtr.VolumeNow;
			wm8978_ChangeVolume(VolumeCtr.VolumeNow,VolumeCtr.VolumeNow);
        }
        tmpjudgement = PlayCondition();
        switch(PlayJudgement)
        {
            case IDEL_PLAY: 
				if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//����������������
				{
					CopyFinInfor(&FiniteList);//����������Ϣ�����Ż����в
					CopyInfor(&FiniteListBuf);//����������Ϣ������ic�
					PlayJudgement = FINITE_PLAY;
					break;
				}
				else if(tmpjudgement == INFINITE_ONLY)//ֻ���ظ���������
				{
					InFiniteListCurPlay = InFiniteListHead;//����ָ�����ͷָ��
					CopyInfor(InFiniteListCurPlay);
					PlayJudgement = INFINITE_PLAY;
					break;
				}
				Init_PlayStatus();//��ղ��Ż���Ϣ
				break;
            case FINITE_PLAY:
                if(tmpjudgement == INFINITE_ONLY)//ֻʣ�ظ���������������
                {
                    CopyInfor(InFiniteListCurPlay);
					PlayJudgement = INFINITE_PLAY;
                    break;
                }
                else if(tmpjudgement == IDEL)//�������
                {
                    PlayJudgement = IDEL_PLAY;
                    break;
                }
                if(VM8978Status.complete == 1)//������ϴ���
                {   
 					if(FiniteList.blockvalid==1)
					{	
						CopyFinInfor(&FiniteList);//����������Ϣ�����Ż�����?
						CopyInfor(&FiniteListBuf);
						FiniteList.blockvalid=0;
					}
                    if(FiniteListBuf.playtimes > 0)//Ƭ�β��Ŵ���δ���
                    {
                        VM8978Status.complete = 0;
                        FiniteListBuf.playtimes--;
                        CopyInfor(&FiniteListBuf);//�ٴ��������������
                        VM8978StartPlay();                     
                    }
                    else
                    {
                        CAN_TxFrame(FRAMEID_LOWTOUP_STOPACK,p,1);
						VM8978Status.allcomplete=1;//������������ȫ�������
                        break;
                    }
					if((FiniteList.segaddr == VM8978Status.segaddr)&&(FiniteNodeSame==1))//20160926�޸ĳ���ͬ�����������ʲ�����ɺ���ܲ����µ�������
					{
						wav_play_stop();//ֹͣ����
						Init_PlayStatus();//��ղ��Ż���Ϣ
						OSTimeDly(20);	
						CopyFinInfor(&FiniteList);//����������Ϣ�����Ż����д
						CopyInfor(&FiniteListBuf);//����������Ϣ������ic�
						FiniteNodeSame=2; //�����µĲ���ָ��
					}
                }
//                if(FiniteList.segaddr != VM8978Status.segaddr)//���յ��µĲ���ָ��
//                {
//                    wav_play_stop();//ֹͣ����
//                    Init_PlayStatus();//��ղ��Ż���Ϣ
//                    OSTimeDly(20);
//					CopyFinInfor(&FiniteList);//����������Ϣ�����Ż�����
//                    CopyInfor(&FiniteListBuf);//����������Ϣ������ic�
// 					FiniteList.blockvalid=0;
//                    break;
//                }
            break;
            case INFINITE_PLAY:
                if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//�����������У����Ȳ������޴�����
                {
                    wav_play_stop();//ֹͣ��ǰ���ŵ�
                    Init_PlayStatus();//��ղ��Ż���Ϣ
                    OSTimeDly(20);
					CopyFinInfor(&FiniteList);
                    CopyInfor(&FiniteListBuf);//����������Ϣ������ic������ʼ����
                    PlayJudgement = FINITE_PLAY;                    
                    break;
                }
                else if(tmpjudgement == IDEL)
                {
                    wav_play_stop();//ֹͣ��ǰ���ŵ�
                    Init_PlayStatus();//��ղ��Ż���Ϣ
                    PlayJudgement = IDEL_PLAY;
                    break;
                }
                if(VM8978Status.complete == 1)//ѭ��ȡ��һ������
                {
                    if(SoftTimer.PlayIntervalTimer == 0)
                    {
						CopyInfor(InFiniteListCurPlay);
                        VM8978StartPlay(); 
                        OS_ENTER_CRITICAL();
                        if(InFiniteListCurPlay->next != InFiniteListTail)//û�е����һ����Ա����������һ��Ա
                        {
                            InFiniteListCurPlay = InFiniteListCurPlay->next;
							VM8978StartPlay();
                        }   
                        else//��ǰ�����һ����Ա���������ͷ
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
