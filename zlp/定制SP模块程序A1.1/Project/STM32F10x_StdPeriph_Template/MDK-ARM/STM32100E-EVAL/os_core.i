#line 1 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"





















 

#line 1 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"




















 












 







 


#line 1 "..\\..\\..\\OS\\uCOS-II\\os_cfg.h"






















 





                                        










                                        













                                        





                                        
#line 72 "..\\..\\..\\OS\\uCOS-II\\os_cfg.h"


                                        
#line 82 "..\\..\\..\\OS\\uCOS-II\\os_cfg.h"


                                        
#line 92 "..\\..\\..\\OS\\uCOS-II\\os_cfg.h"


                                        





                                        






                                        
#line 117 "..\\..\\..\\OS\\uCOS-II\\os_cfg.h"


                                        
#line 126 "..\\..\\..\\OS\\uCOS-II\\os_cfg.h"


                                        






                                        






#line 46 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"
#line 1 "..\\..\\..\\OS\\uCOS-II\\Ports\\os_cpu.h"






















 
















 

typedef unsigned char  BOOLEAN;
typedef unsigned char  INT8U;                     
typedef signed   char  INT8S;                     
typedef unsigned short INT16U;                    
typedef signed   short INT16S;                    
typedef unsigned int   INT32U;                    
typedef signed   int   INT32S;                    
typedef float          FP32;                      
typedef double         FP64;                      

typedef unsigned int   OS_STK;                    
typedef unsigned int   OS_CPU_SR;                 





















 












 









 


OS_CPU_SR  OS_CPU_SR_Save(void);
void       OS_CPU_SR_Restore(OS_CPU_SR cpu_sr);


void       OSCtxSw(void);
void       OSIntCtxSw(void);
void       OSStartHighRdy(void);

void       OS_CPU_PendSVHandler(void);

                                                   





#line 47 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 




























#line 88 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"









 




 
#line 111 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"







 








 
#line 134 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"


                                             





 

























 











 









 









 









 









 












 









 


#line 254 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

#line 264 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"












#line 290 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

#line 297 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

#line 308 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"



#line 317 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"



#line 334 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 
#line 380 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

 




 


typedef struct os_event {
    INT8U    OSEventType;                     
    void    *OSEventPtr;                      
    INT16U   OSEventCnt;                      

    INT8U    OSEventGrp;                      
    INT8U    OSEventTbl[((31) / 8 + 1)];   






    INT8U    OSEventName[16];

} OS_EVENT;







 

#line 453 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

 




 


typedef struct os_mbox_data {
    void   *OSMsg;                          

    INT8U   OSEventTbl[((31) / 8 + 1)];  
    INT8U   OSEventGrp;                     




} OS_MBOX_DATA;






 

#line 502 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

 




 

#line 524 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 

#line 556 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 


typedef struct os_sem_data {
    INT16U  OSCnt;                           

    INT8U   OSEventTbl[((31) / 8 + 1)];   
    INT8U   OSEventGrp;                      




} OS_SEM_DATA;






 


typedef struct os_stk_data {
    INT32U  OSFree;                     
    INT32U  OSUsed;                     
} OS_STK_DATA;


 




 

typedef struct os_tcb {
    OS_STK          *OSTCBStkPtr;            


    void            *OSTCBExtPtr;            
    OS_STK          *OSTCBStkBottom;         
    INT32U           OSTCBStkSize;           
    INT16U           OSTCBOpt;               
    INT16U           OSTCBId;                


    struct os_tcb   *OSTCBNext;              
    struct os_tcb   *OSTCBPrev;              


    OS_EVENT        *OSTCBEventPtr;          







    void            *OSTCBMsg;               


#line 628 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

    INT16U           OSTCBDly;               
    INT8U            OSTCBStat;              
    INT8U            OSTCBStatPend;          
    INT8U            OSTCBPrio;              

    INT8U            OSTCBX;                 
    INT8U            OSTCBY;                 

    INT8U            OSTCBBitX;              
    INT8U            OSTCBBitY;              






    INT8U            OSTCBDelReq;            



    INT32U           OSTCBCtxSwCtr;          
    INT32U           OSTCBCyclesTot;         
    INT32U           OSTCBCyclesStart;       
    OS_STK          *OSTCBStkBase;           
    INT32U           OSTCBStkUsed;           



    INT8U            OSTCBTaskName[25];

} OS_TCB;

 




 

#line 699 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

 




 

  INT32U            OSCtxSwCtr;                


  OS_EVENT         *OSEventFreeList;           
  OS_EVENT          OSEventTbl[10]; 








  INT8U             OSCPUUsage;                
  INT32U            OSIdleCtrMax;              
  INT32U            OSIdleCtrRun;              
  BOOLEAN           OSStatRdy;                 
  OS_STK            OSTaskStatStk[128];       


  INT8U             OSIntNesting;              

  INT8U             OSLockNesting;             

  INT8U             OSPrioCur;                 
  INT8U             OSPrioHighRdy;             


  INT8U             OSRdyGrp;                         
  INT8U             OSRdyTbl[((31) / 8 + 1)];        





  BOOLEAN           OSRunning;                        

  INT8U             OSTaskCtr;                        

  volatile  INT32U  OSIdleCtr;                                  

  OS_STK            OSTaskIdleStk[128];       


  OS_TCB           *OSTCBCur;                         
  OS_TCB           *OSTCBFreeList;                    
  OS_TCB           *OSTCBHighRdy;                     
  OS_TCB           *OSTCBList;                        
  OS_TCB           *OSTCBPrioTbl[31 + 1]; 
  OS_TCB            OSTCBTbl[20 + 2u];    


  INT8U             OSTickStepState;           
















#line 790 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

extern  INT8U   const     OSUnMapTbl[256];           

 





 





 




INT8U         OSEventNameGet          (OS_EVENT        *pevent,
                                       INT8U           *pname,
                                       INT8U           *perr);

void          OSEventNameSet          (OS_EVENT        *pevent,
                                       INT8U           *pname,
                                       INT8U           *perr);


#line 826 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"







 

#line 880 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 




void         *OSMboxAccept            (OS_EVENT        *pevent);


OS_EVENT     *OSMboxCreate            (void            *pmsg);


OS_EVENT     *OSMboxDel               (OS_EVENT        *pevent,
                                       INT8U            opt,
                                       INT8U           *perr);


void         *OSMboxPend              (OS_EVENT        *pevent,
                                       INT16U           timeout,
                                       INT8U           *perr);


INT8U         OSMboxPendAbort         (OS_EVENT        *pevent,
                                       INT8U            opt,
                                       INT8U           *perr);



INT8U         OSMboxPost              (OS_EVENT        *pevent,
                                       void            *pmsg);



INT8U         OSMboxPostOpt           (OS_EVENT        *pevent,
                                       void            *pmsg,
                                       INT8U            opt);



INT8U         OSMboxQuery             (OS_EVENT        *pevent,
                                       OS_MBOX_DATA    *p_mbox_data);







 

#line 961 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 

#line 996 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

 




 

#line 1056 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

 




 



INT16U        OSSemAccept             (OS_EVENT        *pevent);


OS_EVENT     *OSSemCreate             (INT16U           cnt);


OS_EVENT     *OSSemDel                (OS_EVENT        *pevent,
                                       INT8U            opt,
                                       INT8U           *perr);


void          OSSemPend               (OS_EVENT        *pevent,
                                       INT16U           timeout,
                                       INT8U           *perr);


INT8U         OSSemPendAbort          (OS_EVENT        *pevent,
                                       INT8U            opt,
                                       INT8U           *perr);


INT8U         OSSemPost               (OS_EVENT        *pevent);


INT8U         OSSemQuery              (OS_EVENT        *pevent,
                                       OS_SEM_DATA     *p_sem_data);



void          OSSemSet                (OS_EVENT        *pevent,
                                       INT16U           cnt,
                                       INT8U           *perr);




 




 






INT8U         OSTaskCreate            (void           (*task)(void *p_arg),
                                       void            *p_arg,
                                       OS_STK          *ptos,
                                       INT8U            prio);



INT8U         OSTaskCreateExt         (void           (*task)(void *p_arg),
                                       void            *p_arg,
                                       OS_STK          *ptos,
                                       INT8U            prio,
                                       INT16U           id,
                                       OS_STK          *pbos,
                                       INT32U           stk_size,
                                       void            *pext,
                                       INT16U           opt);



INT8U         OSTaskDel               (INT8U            prio);
INT8U         OSTaskDelReq            (INT8U            prio);



INT8U         OSTaskNameGet           (INT8U            prio,
                                       INT8U           *pname,
                                       INT8U           *perr);

void          OSTaskNameSet           (INT8U            prio,
                                       INT8U           *pname,
                                       INT8U           *perr);



INT8U         OSTaskResume            (INT8U            prio);
INT8U         OSTaskSuspend           (INT8U            prio);



INT8U         OSTaskStkChk            (INT8U            prio,
                                       OS_STK_DATA     *p_stk_data);



INT8U         OSTaskQuery             (INT8U            prio,
                                       OS_TCB          *p_task_data);


 




 

void          OSTimeDly               (INT16U           ticks);


INT8U         OSTimeDlyHMSM           (INT8U            hours,
                                       INT8U            minutes,
                                       INT8U            seconds,
                                       INT16U           milli);











void          OSTimeTick              (void);





 

#line 1228 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





 

void          OSInit                  (void);

void          OSIntEnter              (void);
void          OSIntExit               (void);






void          OSStart                 (void);

void          OSStatInit              (void);

INT16U        OSVersion               (void);

 





 


void          OS_Dummy                (void);



INT8U         OS_EventTaskRdy         (OS_EVENT        *pevent,
                                       void            *pmsg,
                                       INT8U            msk,
                                       INT8U            pend_stat);

void          OS_EventTaskWait        (OS_EVENT        *pevent);

void          OS_EventTaskRemove      (OS_TCB          *ptcb,
                                       OS_EVENT        *pevent);

#line 1280 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"

void          OS_EventWaitListInit    (OS_EVENT        *pevent);







void          OS_MemClr               (INT8U           *pdest,
                                       INT16U           size);

void          OS_MemCopy              (INT8U           *pdest,
                                       INT8U           *psrc,
                                       INT16U           size);









void          OS_Sched                (void);


INT8U         OS_StrCopy              (INT8U           *pdest,
                                       INT8U           *psrc);

INT8U         OS_StrLen               (INT8U           *psrc);


void          OS_TaskIdle             (void            *p_arg);


void          OS_TaskStat             (void            *p_arg);



void          OS_TaskStkClr           (OS_STK          *pbos,
                                       INT32U           size,
                                       INT16U           opt);



void          OS_TaskStatStkChk       (void);


INT8U         OS_TCBInit              (INT8U            prio,
                                       OS_STK          *ptos,
                                       OS_STK          *pbos,
                                       INT16U           id,
                                       INT32U           stk_size,
                                       void            *pext,
                                       INT16U           opt);





 





 


void          OSDebugInit             (void);


void          OSInitHookBegin         (void);
void          OSInitHookEnd           (void);

void          OSTaskCreateHook        (OS_TCB          *ptcb);
void          OSTaskDelHook           (OS_TCB          *ptcb);

void          OSTaskIdleHook          (void);

void          OSTaskStatHook          (void);
OS_STK       *OSTaskStkInit           (void           (*task)(void *p_arg),
                                       void            *p_arg,
                                       OS_STK          *ptos,
                                       INT16U           opt);


void          OSTaskSwHook            (void);


void          OSTCBInitHook           (OS_TCB          *ptcb);





 





 

#line 1402 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"







 







 










 





 

#line 1446 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"






























 

#line 1484 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"


























 

#line 1522 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"














 

#line 1544 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"














 

#line 1570 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"






































 

#line 1616 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"






















 

#line 1646 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"



























































 





















 

#line 1773 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"






 


























#line 1813 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"



































 

#line 1925 "..\\..\\..\\OS\\uCOS-II\\Source\\ucos_ii.h"





#line 27 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"









 

INT8U  const  OSUnMapTbl[256] = {
    0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,        
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0         
};

 




 

static  void  OS_InitEventList(void);

static  void  OS_InitMisc(void);

static  void  OS_InitRdyList(void);

static  void  OS_InitTaskIdle(void);


static  void  OS_InitTaskStat(void);


static  void  OS_InitTCBList(void);

static  void  OS_SchedNew(void);

 

























 


INT8U  OSEventNameGet (OS_EVENT *pevent, INT8U *pname, INT8U *perr)
{
    INT8U      len;

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                     
        return (0);
    }
    if (pevent == (OS_EVENT *)0) {                
        *perr = 4u;
        return (0);
    }
    if (pname == (INT8U *)0) {                    
        *perr = 12u;
        return (0);
    }

    if (OSIntNesting > 0) {                       
        *perr  = 17u;
        return (0);
    }
    switch (pevent->OSEventType) {
        case 3u:
        case 4u:
        case 1u:
        case 2u:
             break;

        default:
             *perr = 1u;
             return (0);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    len   = OS_StrCopy(pname, pevent->OSEventName);    
    {OS_CPU_SR_Restore(cpu_sr);};
    *perr = 0u;
    return (len);
}


 

























 


void  OSEventNameSet (OS_EVENT *pevent, INT8U *pname, INT8U *perr)
{
    INT8U      len;

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                     
        return;
    }
    if (pevent == (OS_EVENT *)0) {                
        *perr = 4u;
        return;
    }
    if (pname == (INT8U *)0) {                    
        *perr = 12u;
        return;
    }

    if (OSIntNesting > 0) {                       
        *perr = 18u;
        return;
    }
    switch (pevent->OSEventType) {
        case 3u:
        case 4u:
        case 1u:
        case 2u:
             break;

        default:
             *perr = 1u;
             return;
    }
    {cpu_sr = OS_CPU_SR_Save();};
    len = OS_StrLen(pname);                            
    if (len > (16 - 1)) {              
        {OS_CPU_SR_Restore(cpu_sr);};
        *perr = 11u;
        return;
    }
    (void)OS_StrCopy(pevent->OSEventName, pname);      
    {OS_CPU_SR_Restore(cpu_sr);};
    *perr = 0u;
}


 





































































 
 
#line 539 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"

 











 

void  OSInit (void)
{
    OSInitHookBegin();                                            

    OS_InitMisc();                                                

    OS_InitRdyList();                                             

    OS_InitTCBList();                                             

    OS_InitEventList();                                           













    OS_InitTaskIdle();                                            

    OS_InitTaskStat();                                            






    OSInitHookEnd();                                              


    OSDebugInit();

}
 























 

void  OSIntEnter (void)
{
    if (OSRunning == 1u) {
        if (OSIntNesting < 255u) {
            OSIntNesting++;                       
        }
    }
}
 

















 

void  OSIntExit (void)
{

    OS_CPU_SR  cpu_sr = 0;




    if (OSRunning == 1u) {
        {cpu_sr = OS_CPU_SR_Save();};
        if (OSIntNesting > 0) {                             
            OSIntNesting--;
        }
        if (OSIntNesting == 0) {                            
            if (OSLockNesting == 0) {                       
                OS_SchedNew();
                if (OSPrioHighRdy != OSPrioCur) {           
                    OSTCBHighRdy  = OSTCBPrioTbl[OSPrioHighRdy];

                    OSTCBHighRdy->OSTCBCtxSwCtr++;          

                    OSCtxSwCtr++;                           
                    OSIntCtxSw();                           
                }
            }
        }
        {OS_CPU_SR_Restore(cpu_sr);};
    }
}
 














 

#line 713 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"

 













 

#line 759 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"

 


















 

void  OSStart (void)
{
    if (OSRunning == 0u) {
        OS_SchedNew();                                
        OSPrioCur     = OSPrioHighRdy;
        OSTCBHighRdy  = OSTCBPrioTbl[OSPrioHighRdy];  
        OSTCBCur      = OSTCBHighRdy;
        OSStartHighRdy();                             
    }
}
 


















 


void  OSStatInit (void)
{

    OS_CPU_SR  cpu_sr = 0;




    OSTimeDly(2);                                 
    {cpu_sr = OS_CPU_SR_Save();};
    OSIdleCtr    = 0L;                            
    {OS_CPU_SR_Restore(cpu_sr);};
    OSTimeDly(1000 / 10);             
    {cpu_sr = OS_CPU_SR_Save();};
    OSIdleCtrMax = OSIdleCtr;                     
    OSStatRdy    = 1u;
    {OS_CPU_SR_Restore(cpu_sr);};
}

 












 

void  OSTimeTick (void)
{
    OS_TCB    *ptcb;

    BOOLEAN    step;


    OS_CPU_SR  cpu_sr = 0;




#line 867 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"
    if (OSRunning == 1u) {

        switch (OSTickStepState) {                          
            case 0u:                          
                 step = 1u;
                 break;

            case 1u:                         
                 step = 0u;                           
                 break;

            case 2u:                         
                 step            = 1u;                 
                 OSTickStepState = 1u;
                 break;

            default:                                        
                 step            = 1u;
                 OSTickStepState = 0u;
                 break;
        }
        if (step == 0u) {                             
            return;
        }

        ptcb = OSTCBList;                                   
        while (ptcb->OSTCBPrio != (31)) {      
            {cpu_sr = OS_CPU_SR_Save();};
            if (ptcb->OSTCBDly != 0) {                      
                if (--ptcb->OSTCBDly == 0) {                
                                                            
                    if ((ptcb->OSTCBStat & (0x01u | 0x02u | 0x04u | 0x10u | 0x20u)) != 0x00u) {
                        ptcb->OSTCBStat  &= ~(INT8U)(0x01u | 0x02u | 0x04u | 0x10u | 0x20u);           
                        ptcb->OSTCBStatPend = 1u;                  
                    } else {
                        ptcb->OSTCBStatPend = 0u;
                    }

                    if ((ptcb->OSTCBStat & 0x08u) == 0x00u) {   
                        OSRdyGrp               |= ptcb->OSTCBBitY;              
                        OSRdyTbl[ptcb->OSTCBY] |= ptcb->OSTCBBitX;
                    }
                }
            }
            ptcb = ptcb->OSTCBNext;                         
            {OS_CPU_SR_Restore(cpu_sr);};
        }
    }
}

 












 

INT16U  OSVersion (void)
{
    return (286u);
}

 










 


void  OS_Dummy (void)
{
}


 


























 

INT8U  OS_EventTaskRdy (OS_EVENT *pevent, void *pmsg, INT8U msk, INT8U pend_stat)
{
    OS_TCB  *ptcb;
    INT8U    y;
    INT8U    x;
    INT8U    prio;






    y    = OSUnMapTbl[pevent->OSEventGrp];               
    x    = OSUnMapTbl[pevent->OSEventTbl[y]];
    prio = (INT8U)((y << 3) + x);                        
#line 1014 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"

    ptcb                  =  OSTCBPrioTbl[prio];         
    ptcb->OSTCBDly        =  0;                          

    ptcb->OSTCBMsg        =  pmsg;                       



    ptcb->OSTCBStat      &= ~msk;                        
    ptcb->OSTCBStatPend   =  pend_stat;                  
                                                         
    if ((ptcb->OSTCBStat &   0x08u) == 0x00u) {
        OSRdyGrp         |=  ptcb->OSTCBBitY;            
        OSRdyTbl[y]      |=  ptcb->OSTCBBitX;
    }

    OS_EventTaskRemove(ptcb, pevent);                    
#line 1037 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"

    return (prio);
}

 













 

void  OS_EventTaskWait (OS_EVENT *pevent)
{
    INT8U  y;


    OSTCBCur->OSTCBEventPtr               = pevent;                  

    pevent->OSEventTbl[OSTCBCur->OSTCBY] |= OSTCBCur->OSTCBBitX;     
    pevent->OSEventGrp                   |= OSTCBCur->OSTCBBitY;

    y             =  OSTCBCur->OSTCBY;             
    OSRdyTbl[y]  &= ~OSTCBCur->OSTCBBitX;
    if (OSRdyTbl[y] == 0) {
        OSRdyGrp &= ~OSTCBCur->OSTCBBitY;          
    }
}

 














 
#line 1117 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"
 














 

void  OS_EventTaskRemove (OS_TCB   *ptcb,
                          OS_EVENT *pevent)
{
    INT8U  y;


    y                       =  ptcb->OSTCBY;
    pevent->OSEventTbl[y]  &= ~ptcb->OSTCBBitX;          
    if (pevent->OSEventTbl[y] == 0) {
        pevent->OSEventGrp &= ~ptcb->OSTCBBitY;
    }
}

 














 
#line 1194 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"
 












 

void  OS_EventWaitListInit (OS_EVENT *pevent)
{

    INT8U  *ptbl;



    INT8U   i;


    pevent->OSEventGrp = 0;                       
    ptbl               = &pevent->OSEventTbl[0];

    for (i = 0; i < ((31) / 8 + 1); i++) {
        *ptbl++ = 0;
    }
}

 











 

static  void  OS_InitEventList (void)
{


    INT16U     i;
    OS_EVENT  *pevent1;
    OS_EVENT  *pevent2;


    OS_MemClr((INT8U *)&OSEventTbl[0], sizeof(OSEventTbl));  
    pevent1 = &OSEventTbl[0];
    pevent2 = &OSEventTbl[1];
    for (i = 0; i < (10 - 1); i++) {              
        pevent1->OSEventType    = 0u;
        pevent1->OSEventPtr     = pevent2;

        pevent1->OSEventName[0] = '?';                       
        pevent1->OSEventName[1] = (INT8U)0;

        pevent1++;
        pevent2++;
    }
    pevent1->OSEventType            = 0u;
    pevent1->OSEventPtr             = (OS_EVENT *)0;

    pevent1->OSEventName[0]         = '?';
    pevent1->OSEventName[1]         = (INT8U)0;

    OSEventFreeList                 = &OSEventTbl[0];
#line 1280 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"
}
 











 

static  void  OS_InitMisc (void)
{




    OSIntNesting  = 0;                                      
    OSLockNesting = 0;                                      

    OSTaskCtr     = 0;                                      

    OSRunning     = 0u;                               

    OSCtxSwCtr    = 0;                                      
    OSIdleCtr     = 0L;                                     


    OSIdleCtrRun  = 0L;
    OSIdleCtrMax  = 0L;
    OSStatRdy     = 0u;                               

}
 











 

static  void  OS_InitRdyList (void)
{
    INT8U    i;

    INT8U   *prdytbl;





    OSRdyGrp      = 0;                                      
    prdytbl       = &OSRdyTbl[0];
    for (i = 0; i < ((31) / 8 + 1); i++) {
        *prdytbl++ = 0;
    }

    OSPrioCur     = 0;
    OSPrioHighRdy = 0;

    OSTCBHighRdy  = (OS_TCB *)0;
    OSTCBCur      = (OS_TCB *)0;
}

 











 

static  void  OS_InitTaskIdle (void)
{

    INT8U  err;





    (void)OSTaskCreateExt(OS_TaskIdle,
                          (void *)0,                                  
                          &OSTaskIdleStk[128 - 1],  
                          (31),                          
                          65535u,
                          &OSTaskIdleStk[0],                          
                          128,
                          (void *)0,                                  
                          0x0001u | 0x0002u); 
#line 1410 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"


    OSTaskNameSet((31), (INT8U *)"uC/OS-II Idle", &err);





}
 











 


static  void  OS_InitTaskStat (void)
{

    INT8U  err;





    (void)OSTaskCreateExt(OS_TaskStat,
                          (void *)0,                                    
                          &OSTaskStatStk[128 - 1],    
                          (31 - 1),                            
                          65534u,
                          &OSTaskStatStk[0],                            
                          128,
                          (void *)0,                                    
                          0x0001u | 0x0002u);   
#line 1476 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"


    OSTaskNameSet((31 - 1), (INT8U *)"uC/OS-II Stat", &err);





}

 











 

static  void  OS_InitTCBList (void)
{
    INT8U    i;
    OS_TCB  *ptcb1;
    OS_TCB  *ptcb2;


    OS_MemClr((INT8U *)&OSTCBTbl[0],     sizeof(OSTCBTbl));       
    OS_MemClr((INT8U *)&OSTCBPrioTbl[0], sizeof(OSTCBPrioTbl));   
    ptcb1 = &OSTCBTbl[0];
    ptcb2 = &OSTCBTbl[1];
    for (i = 0; i < (20 + 2u - 1); i++) {   
        ptcb1->OSTCBNext = ptcb2;

        ptcb1->OSTCBTaskName[0] = '?';                            
        ptcb1->OSTCBTaskName[1] = (INT8U)0;

        ptcb1++;
        ptcb2++;
    }
    ptcb1->OSTCBNext = (OS_TCB *)0;                               

    ptcb1->OSTCBTaskName[0] = '?';                                
    ptcb1->OSTCBTaskName[1] = (INT8U)0;

    OSTCBList               = (OS_TCB *)0;                        
    OSTCBFreeList           = &OSTCBTbl[0];
}
 


















 

void  OS_MemClr (INT8U *pdest, INT16U size)
{
    while (size > 0) {
        *pdest++ = (INT8U)0;
        size--;
    }
}
 






















 

void  OS_MemCopy (INT8U *pdest, INT8U *psrc, INT16U size)
{
    while (size > 0) {
        *pdest++ = *psrc++;
        size--;
    }
}
 















 

void  OS_Sched (void)
{

    OS_CPU_SR  cpu_sr = 0;




    {cpu_sr = OS_CPU_SR_Save();};
    if (OSIntNesting == 0) {                            
        if (OSLockNesting == 0) {                       
            OS_SchedNew();
            if (OSPrioHighRdy != OSPrioCur) {           
                OSTCBHighRdy = OSTCBPrioTbl[OSPrioHighRdy];

                OSTCBHighRdy->OSTCBCtxSwCtr++;          

                OSCtxSwCtr++;                           
                OSCtxSw();                           
            }
        }
    }
    {OS_CPU_SR_Restore(cpu_sr);};
}
















 

static  void  OS_SchedNew (void)
{

    INT8U   y;


    y             = OSUnMapTbl[OSRdyGrp];
    OSPrioHighRdy = (INT8U)((y << 3) + OSUnMapTbl[OSRdyTbl[y]]);
#line 1673 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"
}

 

















 


INT8U  OS_StrCopy (INT8U *pdest, INT8U *psrc)
{
    INT8U  len;


    len = 0;
    while (*psrc != (INT8U)0) {
        *pdest++ = *psrc++;
        len++;
    }
    *pdest = (INT8U)0;
    return (len);
}

 














 


INT8U  OS_StrLen (INT8U *psrc)
{
    INT8U  len;


    len = 0;
    while (*psrc != (INT8U)0) {
        psrc++;
        len++;
    }
    return (len);
}

 



















 

void  OS_TaskIdle (void *p_arg)
{

    OS_CPU_SR  cpu_sr = 0;




    (void)p_arg;                                  
    for (;;) {
        {cpu_sr = OS_CPU_SR_Save();};
        OSIdleCtr++;
        {OS_CPU_SR_Restore(cpu_sr);};
        OSTaskIdleHook();                         
    }
}
 






















 


void  OS_TaskStat (void *p_arg)
{

    OS_CPU_SR  cpu_sr = 0;




    (void)p_arg;                                  
    while (OSStatRdy == 0u) {
        OSTimeDly(2 * 1000 / 10);     
    }
    OSIdleCtrMax /= 100L;
    if (OSIdleCtrMax == 0L) {
        OSCPUUsage = 0;
        (void)OSTaskSuspend(0xFFu);
    }
    for (;;) {
        {cpu_sr = OS_CPU_SR_Save();};
        OSIdleCtrRun = OSIdleCtr;                 
        OSIdleCtr    = 0L;                        
        {OS_CPU_SR_Restore(cpu_sr);};
        OSCPUUsage   = (INT8U)(100L - OSIdleCtrRun / OSIdleCtrMax);
        OSTaskStatHook();                         

        OS_TaskStatStkChk();                      

        OSTimeDly(1000 / 10);         
    }
}

 










 


void  OS_TaskStatStkChk (void)
{
    OS_TCB      *ptcb;
    OS_STK_DATA  stk_data;
    INT8U        err;
    INT8U        prio;


    for (prio = 0; prio <= (31); prio++) {
        err = OSTaskStkChk(prio, &stk_data);
        if (err == 0u) {
            ptcb = OSTCBPrioTbl[prio];
            if (ptcb != (OS_TCB *)0) {                                
                if (ptcb != ((OS_TCB *)1)) {                        


                    ptcb->OSTCBStkBase = ptcb->OSTCBStkBottom + ptcb->OSTCBStkSize;



                    ptcb->OSTCBStkUsed = stk_data.OSUsed;             

                }
            }
        }
    }
}

 









































 

INT8U  OS_TCBInit (INT8U prio, OS_STK *ptos, OS_STK *pbos, INT16U id, INT32U stk_size, void *pext, INT16U opt)
{
    OS_TCB    *ptcb;

    OS_CPU_SR  cpu_sr = 0;




    {cpu_sr = OS_CPU_SR_Save();};
    ptcb = OSTCBFreeList;                                   
    if (ptcb != (OS_TCB *)0) {
        OSTCBFreeList            = ptcb->OSTCBNext;         
        {OS_CPU_SR_Restore(cpu_sr);};
        ptcb->OSTCBStkPtr        = ptos;                    
        ptcb->OSTCBPrio          = prio;                    
        ptcb->OSTCBStat          = 0x00u;             
        ptcb->OSTCBStatPend      = 0u;         
        ptcb->OSTCBDly           = 0;                       


        ptcb->OSTCBExtPtr        = pext;                    
        ptcb->OSTCBStkSize       = stk_size;                
        ptcb->OSTCBStkBottom     = pbos;                    
        ptcb->OSTCBOpt           = opt;                     
        ptcb->OSTCBId            = id;                      
#line 1955 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"


        ptcb->OSTCBDelReq        = 0u;



        ptcb->OSTCBY             = (INT8U)(prio >> 3);           
        ptcb->OSTCBX             = (INT8U)(prio & 0x07);
        ptcb->OSTCBBitY          = (INT8U)(1 << ptcb->OSTCBY);
        ptcb->OSTCBBitX          = (INT8U)(1 << ptcb->OSTCBX);
#line 1971 "..\\..\\..\\OS\\uCOS-II\\Source\\os_core.c"


        ptcb->OSTCBEventPtr      = (OS_EVENT  *)0;          










        ptcb->OSTCBMsg       = (void *)0;                   



        ptcb->OSTCBCtxSwCtr    = 0L;                        
        ptcb->OSTCBCyclesStart = 0L;
        ptcb->OSTCBCyclesTot   = 0L;
        ptcb->OSTCBStkBase     = (OS_STK *)0;
        ptcb->OSTCBStkUsed     = 0L;



        ptcb->OSTCBTaskName[0] = '?';                       
        ptcb->OSTCBTaskName[1] = (INT8U)0;


        OSTCBInitHook(ptcb);

        OSTaskCreateHook(ptcb);                             

        {cpu_sr = OS_CPU_SR_Save();};
        OSTCBPrioTbl[prio] = ptcb;
        ptcb->OSTCBNext    = OSTCBList;                     
        ptcb->OSTCBPrev    = (OS_TCB *)0;
        if (OSTCBList != (OS_TCB *)0) {
            OSTCBList->OSTCBPrev = ptcb;
        }
        OSTCBList               = ptcb;
        OSRdyGrp               |= ptcb->OSTCBBitY;          
        OSRdyTbl[ptcb->OSTCBY] |= ptcb->OSTCBBitX;
        OSTaskCtr++;                                        
        {OS_CPU_SR_Restore(cpu_sr);};
        return (0u);
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    return (66u);
}
