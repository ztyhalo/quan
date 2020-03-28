/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* �ļ��� : task2.c
* ��  �� :���ƹ������룬��ָʾ�ƽ�����˸����
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
	static u8 DealResult;
    u8 instruct = (*msg&0x0300)>>8;
    u8 segid = (*msg&0x00FF);
    
	switch(instruct){
        case STOP_ALLFINITE://ֹͣ���в�һֱ����
            if((VM8978Status.mode == 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//ֹͣ����
                Init_PlayStatus();
            }
            //wav_play_stop();//ֹͣ����

            FiniteList.segaddr = 0;//������޴β����б�
            FiniteList.playtype = 0;
            FiniteList.playtimes = 0;
            FiniteList.playprior = 0;
            FiniteList.playinterval = 0;

            FiniteListNodeNum = 0;

            //Init_PlayStatus();

            DealResult = STOP_ACK_OK;

        break;

        case STOP_SPCINFINITE://ָֹͣ��һֱ����
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                if(VM8978Status.segaddr==segid )
                {
                    wav_play_stop();//ֹͣ����
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
        case STOP_ALLINFINITE://ֹͣ����һֱ����
            if((VM8978Status.mode != 0)&&(VM8978Status.complete == 0))
            {
                wav_play_stop();//ֹͣ����
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
* �������ƣ�PlayCondition ()
* ��������������״̬�ж�
* ��ڲ�������
* ���ڲ���������ģʽ
* ʹ��˵������
********************************************************************************************/
u8 PlayCondition(void)
{
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

	u8 PlayErr=0;
    char *filename;
    u8 prior;
	u8 err;
    u16* msg;
    u8 PlayJudgement = 0;
    u8 tmpjudgement;

    Init_FreeList();
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
                //tmpjudgement = PlayCondition();
                if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//����������������
                {
                    CopyInfor(&FiniteList);//����������Ϣ������ic������ʼ����
//                     VM8978StartPlay();
                    PlayJudgement = FINITE_PLAY;
                    break;
                }
                else if(tmpjudgement == INFINITE_ONLY)//ֻ���ظ���������
                {
                    CopyInfor(InFiniteListCurPlay);
                    InFiniteListCurPlay->blockactive = 1;
                    VM8978StartPlay();
                    OS_ENTER_CRITICAL();
                    if(InFiniteListCurPlay->next != InFiniteListTail)//û�е����һ����Ա����������һ��Ա
                    {
                        InFiniteListCurPlay = InFiniteListCurPlay->next;
                    }
                    else//��ǰ�����һ����Ա���������ͷ
                    {
                        InFiniteListCurPlay = InFiniteListHead;
                    }
                    OS_EXIT_CRITICAL();

                    PlayJudgement = INFINITE_PLAY;
                    break;
                }
                Init_PlayStatus();//��ղ��Ż���Ϣ
            break;
            case FINITE_PLAY:
                //tmpjudgement = PlayCondition();
                if(tmpjudgement == INFINITE_ONLY)//ֻʣ�ظ���������������
                {
                    CopyInfor(InFiniteListCurPlay);
                    InFiniteListCurPlay->blockactive = 1;
                    VM8978StartPlay();

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
                    if(FiniteList.playtimes > 0)//Ƭ�β��Ŵ���δ���
                    {
                        VM8978Status.complete = 0;
                        FiniteList.playtimes--;
                        CopyInfor(&FiniteList);//�ٴ��������������
                        VM8978StartPlay();                     
                    }
                    else
                    {
                        FiniteListNodeNum = 0;
                        //PlayJudgement = IDEL_PLAY;
                        break;
                    }
                }
                if(FiniteList.segaddr != VM8978Status.segaddr)//���յ��µĲ���ָ��
                {
                    wav_play_stop();//ֹͣ����
                    Init_PlayStatus();//��ղ��Ż���Ϣ
                    OSTimeDly(20);	
                    CopyInfor(&FiniteList);//����������Ϣ������ic������ʼ����
//                     VM8978StartPlay();

                    //VM8978Status.complete = 1;//ǿ����Ϊ�������
                }

            break;
            case INFINITE_PLAY:
                //tmpjudgement = PlayCondition();
                if((tmpjudgement == FINITE_ONLY)||(tmpjudgement == FINITE_INFINITE_BOTH))//�����������У����Ȳ������޴�����
                {
                    wav_play_stop();//ֹͣ��ǰ���ŵ�
                    Init_PlayStatus();//��ղ��Ż���Ϣ
                    OSTimeDly(20);
                    CopyInfor(&FiniteList);//����������Ϣ������ic������ʼ����
//                     VM8978StartPlay();
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
                        VM8978Status.complete = 0;
                        OS_ENTER_CRITICAL();
                        if(InFiniteListCurPlay->next != InFiniteListTail)//û�е����һ����Ա����������һ��Ա
                        {
                            InFiniteListCurPlay = InFiniteListCurPlay->next;
                        }   
                        else//��ǰ�����һ����Ա���������ͷ
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
