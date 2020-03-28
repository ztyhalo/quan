#line 1 "..\\..\\..\\OS\\uCOS-II\\Source\\os_sem.c"





















 

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

 




 

extern  INT32U            OSCtxSwCtr;                


extern  OS_EVENT         *OSEventFreeList;           
extern  OS_EVENT          OSEventTbl[10]; 








extern  INT8U             OSCPUUsage;                
extern  INT32U            OSIdleCtrMax;              
extern  INT32U            OSIdleCtrRun;              
extern  BOOLEAN           OSStatRdy;                 
extern  OS_STK            OSTaskStatStk[128];       


extern  INT8U             OSIntNesting;              

extern  INT8U             OSLockNesting;             

extern  INT8U             OSPrioCur;                 
extern  INT8U             OSPrioHighRdy;             


extern  INT8U             OSRdyGrp;                         
extern  INT8U             OSRdyTbl[((31) / 8 + 1)];        





extern  BOOLEAN           OSRunning;                        

extern  INT8U             OSTaskCtr;                        

extern  volatile  INT32U  OSIdleCtr;                                  

extern  OS_STK            OSTaskIdleStk[128];       


extern  OS_TCB           *OSTCBCur;                         
extern  OS_TCB           *OSTCBFreeList;                    
extern  OS_TCB           *OSTCBHighRdy;                     
extern  OS_TCB           *OSTCBList;                        
extern  OS_TCB           *OSTCBPrioTbl[31 + 1]; 
extern  OS_TCB            OSTCBTbl[20 + 2u];    


extern  INT8U             OSTickStepState;           
















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





#line 26 "..\\..\\..\\OS\\uCOS-II\\Source\\os_sem.c"



 
















 


INT16U  OSSemAccept (OS_EVENT *pevent)
{
    INT16U     cnt;

    OS_CPU_SR  cpu_sr = 0;





    if (pevent == (OS_EVENT *)0) {                     
        return (0);
    }

    if (pevent->OSEventType != 3u) {    
        return (0);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    cnt = pevent->OSEventCnt;
    if (cnt > 0) {                                     
        pevent->OSEventCnt--;                          
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    return (cnt);                                      
}


 















 

OS_EVENT  *OSSemCreate (INT16U cnt)
{
    OS_EVENT  *pevent;

    OS_CPU_SR  cpu_sr = 0;




    if (OSIntNesting > 0) {                                 
        return ((OS_EVENT *)0);                             
    }
    {cpu_sr = OS_CPU_SR_Save();};
    pevent = OSEventFreeList;                               
    if (OSEventFreeList != (OS_EVENT *)0) {                 
        OSEventFreeList = (OS_EVENT *)OSEventFreeList->OSEventPtr;
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    if (pevent != (OS_EVENT *)0) {                          
        pevent->OSEventType    = 3u;
        pevent->OSEventCnt     = cnt;                       
        pevent->OSEventPtr     = (void *)0;                 

        pevent->OSEventName[0] = '?';                       
        pevent->OSEventName[1] = (INT8U)0;

        OS_EventWaitListInit(pevent);                       
    }
    return (pevent);
}

 



































 


OS_EVENT  *OSSemDel (OS_EVENT *pevent, INT8U opt, INT8U *perr)
{
    BOOLEAN    tasks_waiting;
    OS_EVENT  *pevent_return;

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                               
        return (pevent);
    }
    if (pevent == (OS_EVENT *)0) {                          
        *perr = 4u;
        return (pevent);
    }

    if (pevent->OSEventType != 3u) {         
        *perr = 1u;
        return (pevent);
    }
    if (OSIntNesting > 0) {                                 
        *perr = 15u;                              
        return (pevent);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    if (pevent->OSEventGrp != 0) {                          
        tasks_waiting = 1u;                            
    } else {
        tasks_waiting = 0u;                           
    }
    switch (opt) {
        case 0u:                                
             if (tasks_waiting == 0u) {

                 pevent->OSEventName[0] = '?';              
                 pevent->OSEventName[1] = (INT8U)0;

                 pevent->OSEventType    = 0u;
                 pevent->OSEventPtr     = OSEventFreeList;  
                 pevent->OSEventCnt     = 0;
                 OSEventFreeList        = pevent;           
                 {OS_CPU_SR_Restore(cpu_sr);};
                 *perr                  = 0u;
                 pevent_return          = (OS_EVENT *)0;    
             } else {
                 {OS_CPU_SR_Restore(cpu_sr);};
                 *perr                  = 73u;
                 pevent_return          = pevent;
             }
             break;

        case 1u:                                 
             while (pevent->OSEventGrp != 0) {              
                 (void)OS_EventTaskRdy(pevent, (void *)0, 0x01u, 0u);
             }

             pevent->OSEventName[0] = '?';                  
             pevent->OSEventName[1] = (INT8U)0;

             pevent->OSEventType    = 0u;
             pevent->OSEventPtr     = OSEventFreeList;      
             pevent->OSEventCnt     = 0;
             OSEventFreeList        = pevent;               
             {OS_CPU_SR_Restore(cpu_sr);};
             if (tasks_waiting == 1u) {                
                 OS_Sched();                                
             }
             *perr                  = 0u;
             pevent_return          = (OS_EVENT *)0;        
             break;

        default:
             {OS_CPU_SR_Restore(cpu_sr);};
             *perr                  = 7u;
             pevent_return          = pevent;
             break;
    }
    return (pevent_return);
}


 






























 
 
void  OSSemPend (OS_EVENT *pevent, INT16U timeout, INT8U *perr)
{

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                          
        return;
    }
    if (pevent == (OS_EVENT *)0) {                     
        *perr = 4u;
        return;
    }

    if (pevent->OSEventType != 3u) {    
        *perr = 1u;
        return;
    }
    if (OSIntNesting > 0) {                            
        *perr = 2u;                       
        return;
    }
    if (OSLockNesting > 0) {                           
        *perr = 13u;                    
        return;
    }
    {cpu_sr = OS_CPU_SR_Save();};
    if (pevent->OSEventCnt > 0) {                      
        pevent->OSEventCnt--;                          
        {OS_CPU_SR_Restore(cpu_sr);};
        *perr = 0u;
        return;
    }
                                                       
    OSTCBCur->OSTCBStat     |= 0x01u;            
    OSTCBCur->OSTCBStatPend  = 0u;
    OSTCBCur->OSTCBDly       = timeout;                
    OS_EventTaskWait(pevent);                          
    {OS_CPU_SR_Restore(cpu_sr);};
    OS_Sched();                                        
    {cpu_sr = OS_CPU_SR_Save();};
    switch (OSTCBCur->OSTCBStatPend) {                 
        case 0u:
             *perr = 0u;
             break;

        case 2u:
             *perr = 14u;                
             break;

        case 1u:
        default:        
             OS_EventTaskRemove(OSTCBCur, pevent);
             *perr = 10u;                   
             break;
    }
    OSTCBCur->OSTCBStat          =  0x00u;       
    OSTCBCur->OSTCBStatPend      =  0u;   
    OSTCBCur->OSTCBEventPtr      = (OS_EVENT  *)0;     



    {OS_CPU_SR_Restore(cpu_sr);};
}

 































 


INT8U  OSSemPendAbort (OS_EVENT *pevent, INT8U opt, INT8U *perr)
{
    INT8U      nbr_tasks;

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                          
        return (0);
    }
    if (pevent == (OS_EVENT *)0) {                     
        *perr = 4u;
        return (0);
    }

    if (pevent->OSEventType != 3u) {    
        *perr = 1u;
        return (0);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    if (pevent->OSEventGrp != 0) {                     
        nbr_tasks = 0;
        switch (opt) {
            case 1u:                
                 while (pevent->OSEventGrp != 0) {     
                     (void)OS_EventTaskRdy(pevent, (void *)0, 0x01u, 2u);
                     nbr_tasks++;
                 }
                 break;
                 
            case 0u:
            default:                                   
                 (void)OS_EventTaskRdy(pevent, (void *)0, 0x01u, 2u);
                 nbr_tasks++;
                 break;
        }
        {OS_CPU_SR_Restore(cpu_sr);};
        OS_Sched();                                    
        *perr = 14u;
        return (nbr_tasks);
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    *perr = 0u;
    return (0);                                        
}


 
















 

INT8U  OSSemPost (OS_EVENT *pevent)
{

    OS_CPU_SR  cpu_sr = 0;





    if (pevent == (OS_EVENT *)0) {                     
        return (4u);
    }

    if (pevent->OSEventType != 3u) {    
        return (1u);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    if (pevent->OSEventGrp != 0) {                     
                                                       
        (void)OS_EventTaskRdy(pevent, (void *)0, 0x01u, 0u);
        {OS_CPU_SR_Restore(cpu_sr);};
        OS_Sched();                                    
        return (0u);
    }
    if (pevent->OSEventCnt < 65535u) {                 
        pevent->OSEventCnt++;                          
        {OS_CPU_SR_Restore(cpu_sr);};
        return (0u);
    }
    {OS_CPU_SR_Restore(cpu_sr);};                                
    return (50u);
}

 

















 


INT8U  OSSemQuery (OS_EVENT *pevent, OS_SEM_DATA *p_sem_data)
{

    INT8U     *psrc;
    INT8U     *pdest;




    INT8U      i;

    OS_CPU_SR  cpu_sr = 0;





    if (pevent == (OS_EVENT *)0) {                          
        return (4u);
    }
    if (p_sem_data == (OS_SEM_DATA *)0) {                   
        return (9u);
    }

    if (pevent->OSEventType != 3u) {         
        return (1u);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    p_sem_data->OSEventGrp = pevent->OSEventGrp;            
    psrc                   = &pevent->OSEventTbl[0];
    pdest                  = &p_sem_data->OSEventTbl[0];
    for (i = 0; i < ((31) / 8 + 1); i++) {
        *pdest++ = *psrc++;
    }
    p_sem_data->OSCnt = pevent->OSEventCnt;                 
    {OS_CPU_SR_Restore(cpu_sr);};
    return (0u);
}


 






















 


void  OSSemSet (OS_EVENT *pevent, INT16U cnt, INT8U *perr)
{

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                          
        return;
    }
    if (pevent == (OS_EVENT *)0) {                     
        *perr = 4u;
        return;
    }

    if (pevent->OSEventType != 3u) {    
        *perr = 1u;
        return;
    }
    {cpu_sr = OS_CPU_SR_Save();};
    *perr = 0u;
    if (pevent->OSEventCnt > 0) {                      
        pevent->OSEventCnt = cnt;                      
    } else {                                           
        if (pevent->OSEventGrp == 0) {                 
            pevent->OSEventCnt = cnt;                  
        } else {
            *perr              = 73u;
        }
    }
    {OS_CPU_SR_Restore(cpu_sr);};
}


