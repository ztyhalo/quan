#line 1 "..\\..\\..\\OS\\uCOS-II\\Source\\os_task.c"





















 

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





#line 26 "..\\..\\..\\OS\\uCOS-II\\Source\\os_task.c"


 



















 

#line 173 "..\\..\\..\\OS\\uCOS-II\\Source\\os_task.c"
 






































 


INT8U  OSTaskCreate (void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT8U prio)
{
    OS_STK    *psp;
    INT8U      err;

    OS_CPU_SR  cpu_sr = 0;





    if (prio > 31) {              
        return (42u);
    }

    {cpu_sr = OS_CPU_SR_Save();};
    if (OSIntNesting > 0) {                   
        {OS_CPU_SR_Restore(cpu_sr);};
        return (60u);
    }
    if (OSTCBPrioTbl[prio] == (OS_TCB *)0) {  
        OSTCBPrioTbl[prio] = ((OS_TCB *)1); 
                                              
        {OS_CPU_SR_Restore(cpu_sr);};
        psp = OSTaskStkInit(task, p_arg, ptos, 0);               
        err = OS_TCBInit(prio, psp, (OS_STK *)0, 0, 0, (void *)0, 0);
        if (err == 0u) {
            if (OSRunning == 1u) {       
                OS_Sched();
            }
        } else {
            {cpu_sr = OS_CPU_SR_Save();};
            OSTCBPrioTbl[prio] = (OS_TCB *)0; 
            {OS_CPU_SR_Restore(cpu_sr);};
        }
        return (err);
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    return (40u);
}

 




































































 
 

INT8U  OSTaskCreateExt (void   (*task)(void *p_arg),
                        void    *p_arg,
                        OS_STK  *ptos,
                        INT8U    prio,
                        INT16U   id,
                        OS_STK  *pbos,
                        INT32U   stk_size,
                        void    *pext,
                        INT16U   opt)
{
    OS_STK    *psp;
    INT8U      err;

    OS_CPU_SR  cpu_sr = 0;





    if (prio > 31) {              
        return (42u);
    }

    {cpu_sr = OS_CPU_SR_Save();};
    if (OSIntNesting > 0) {                   
        {OS_CPU_SR_Restore(cpu_sr);};
        return (60u);
    }
    if (OSTCBPrioTbl[prio] == (OS_TCB *)0) {  
        OSTCBPrioTbl[prio] = ((OS_TCB *)1); 
                                              
        {OS_CPU_SR_Restore(cpu_sr);};


        OS_TaskStkClr(pbos, stk_size, opt);                     


        psp = OSTaskStkInit(task, p_arg, ptos, opt);            
        err = OS_TCBInit(prio, psp, pbos, id, stk_size, pext, opt);
        if (err == 0u) {
            if (OSRunning == 1u) {                         
                OS_Sched();
            }
        } else {
            {cpu_sr = OS_CPU_SR_Save();};
            OSTCBPrioTbl[prio] = (OS_TCB *)0;                   
            {OS_CPU_SR_Restore(cpu_sr);};
        }
        return (err);
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    return (40u);
}

 


































 


INT8U  OSTaskDel (INT8U prio)
{



    OS_TCB       *ptcb;

    OS_CPU_SR     cpu_sr = 0;




    if (OSIntNesting > 0) {                              
        return (64u);
    }
    if (prio == (31)) {                     
        return (62u);
    }

    if (prio >= 31) {                        
        if (prio != 0xFFu) {
            return (42u);
        }
    }


 
    {cpu_sr = OS_CPU_SR_Save();};
    if (prio == 0xFFu) {                          
        prio = OSTCBCur->OSTCBPrio;                      
    }
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                           
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);
    }
    if (ptcb == ((OS_TCB *)1)) {                       
        {OS_CPU_SR_Restore(cpu_sr);};
        return (61u);
    }

    OSRdyTbl[ptcb->OSTCBY] &= ~ptcb->OSTCBBitX;
    if (OSRdyTbl[ptcb->OSTCBY] == 0) {                   
        OSRdyGrp           &= ~ptcb->OSTCBBitY;
    }
    

    if (ptcb->OSTCBEventPtr != (OS_EVENT *)0) {
        OS_EventTaskRemove(ptcb, ptcb->OSTCBEventPtr);   
    }
#line 476 "..\\..\\..\\OS\\uCOS-II\\Source\\os_task.c"

#line 483 "..\\..\\..\\OS\\uCOS-II\\Source\\os_task.c"

    ptcb->OSTCBDly      = 0;                             
    ptcb->OSTCBStat     = 0x00u;                   
    ptcb->OSTCBStatPend = 0u;
    if (OSLockNesting < 255u) {                          
        OSLockNesting++;
    }
    {OS_CPU_SR_Restore(cpu_sr);};                                  
    OS_Dummy();                                          
    {cpu_sr = OS_CPU_SR_Save();};                                 
    if (OSLockNesting > 0) {                             
        OSLockNesting--;
    }
    OSTaskDelHook(ptcb);                                 
    OSTaskCtr--;                                         
    OSTCBPrioTbl[prio] = (OS_TCB *)0;                    
    if (ptcb->OSTCBPrev == (OS_TCB *)0) {                
        ptcb->OSTCBNext->OSTCBPrev = (OS_TCB *)0;
        OSTCBList                  = ptcb->OSTCBNext;
    } else {
        ptcb->OSTCBPrev->OSTCBNext = ptcb->OSTCBNext;
        ptcb->OSTCBNext->OSTCBPrev = ptcb->OSTCBPrev;
    }
    ptcb->OSTCBNext   = OSTCBFreeList;                   
    OSTCBFreeList     = ptcb;

    ptcb->OSTCBTaskName[0] = '?';                        
    ptcb->OSTCBTaskName[1] = (INT8U)0;

    {OS_CPU_SR_Restore(cpu_sr);};
    if (OSRunning == 1u) {
        OS_Sched();                                      
    }
    return (0u);
}

 












































 
 

INT8U  OSTaskDelReq (INT8U prio)
{
    INT8U      stat;
    OS_TCB    *ptcb;

    OS_CPU_SR  cpu_sr = 0;




    if (prio == (31)) {                             
        return (62u);
    }

    if (prio >= 31) {                                
        if (prio != 0xFFu) {
            return (42u);
        }
    }

    if (prio == 0xFFu) {                                  
        {cpu_sr = OS_CPU_SR_Save();};                                     
        stat = OSTCBCur->OSTCBDelReq;                            
        {OS_CPU_SR_Restore(cpu_sr);};
        return (stat);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                                   
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);                          
    }
    if (ptcb == ((OS_TCB *)1)) {                               
        {OS_CPU_SR_Restore(cpu_sr);};
        return (61u);
    }
    ptcb->OSTCBDelReq = 63u;                     
    {OS_CPU_SR_Restore(cpu_sr);};
    return (0u);
}

 























 


INT8U  OSTaskNameGet (INT8U prio, INT8U *pname, INT8U *perr)
{
    OS_TCB    *ptcb;
    INT8U      len;

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                             
        return (0);
    }
    if (prio > 31) {                          
        if (prio != 0xFFu) {
            *perr = 42u;                  
            return (0);
        }
    }
    if (pname == (INT8U *)0) {                            
        *perr = 12u;                        
        return (0);
    }

    if (OSIntNesting > 0) {                               
        *perr = 17u;
        return (0);
    }
    {cpu_sr = OS_CPU_SR_Save();};
    if (prio == 0xFFu) {                           
        prio = OSTCBCur->OSTCBPrio;
    }
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                            
        {OS_CPU_SR_Restore(cpu_sr);};                               
        *perr = 67u;
        return (0);
    }
    if (ptcb == ((OS_TCB *)1)) {                        
        {OS_CPU_SR_Restore(cpu_sr);};                               
        *perr = 67u;
        return (0);
    }
    len   = OS_StrCopy(pname, ptcb->OSTCBTaskName);       
    {OS_CPU_SR_Restore(cpu_sr);};
    *perr = 0u;
    return (len);
}


 

























 

void  OSTaskNameSet (INT8U prio, INT8U *pname, INT8U *perr)
{
    INT8U      len;
    OS_TCB    *ptcb;

    OS_CPU_SR  cpu_sr = 0;





    if (perr == (INT8U *)0) {                         
        return;
    }
    if (prio > 31) {                      
        if (prio != 0xFFu) {
            *perr = 42u;              
            return;
        }
    }
    if (pname == (INT8U *)0) {                        
        *perr = 12u;                    
        return;
    }

    if (OSIntNesting > 0) {                           
        *perr = 18u;
        return;
    }
    {cpu_sr = OS_CPU_SR_Save();};
    if (prio == 0xFFu) {                       
        prio = OSTCBCur->OSTCBPrio;
    }
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                        
        {OS_CPU_SR_Restore(cpu_sr);};                           
        *perr = 67u;
        return;
    }
    if (ptcb == ((OS_TCB *)1)) {                    
        {OS_CPU_SR_Restore(cpu_sr);};                           
        *perr = 67u;
        return;
    }
    len = OS_StrLen(pname);                           
    if (len > (25 - 1)) {              
        {OS_CPU_SR_Restore(cpu_sr);};
        *perr = 65u;
        return;
    }
    (void)OS_StrCopy(ptcb->OSTCBTaskName, pname);     
    {OS_CPU_SR_Restore(cpu_sr);};
    *perr = 0u;
}


 
















 


INT8U  OSTaskResume (INT8U prio)
{
    OS_TCB    *ptcb;

    OS_CPU_SR  cpu_sr = 0;





    if (prio >= 31) {                              
        return (42u);
    }

    {cpu_sr = OS_CPU_SR_Save();};
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                                 
        {OS_CPU_SR_Restore(cpu_sr);};
        return (70u);
    }
    if (ptcb == ((OS_TCB *)1)) {                             
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);
    }
    if ((ptcb->OSTCBStat & 0x08u) != 0x00u) {  
        ptcb->OSTCBStat &= ~(INT8U)0x08u;            
        if (ptcb->OSTCBStat == 0x00u) {                  
            if (ptcb->OSTCBDly == 0) {
                OSRdyGrp               |= ptcb->OSTCBBitY;     
                OSRdyTbl[ptcb->OSTCBY] |= ptcb->OSTCBBitX;
                {OS_CPU_SR_Restore(cpu_sr);};
                if (OSRunning == 1u) {
                    OS_Sched();                                
                }
            } else {
                {OS_CPU_SR_Restore(cpu_sr);};
            }
        } else {                                               
            {OS_CPU_SR_Restore(cpu_sr);};
        }
        return (0u);
    }
    {OS_CPU_SR_Restore(cpu_sr);};
    return (68u);
}

 


















 

INT8U  OSTaskStkChk (INT8U prio, OS_STK_DATA *p_stk_data)
{
    OS_TCB    *ptcb;
    OS_STK    *pchk;
    INT32U     nfree;
    INT32U     size;

    OS_CPU_SR  cpu_sr = 0;





    if (prio > 31) {                        
        if (prio != 0xFFu) {
            return (42u);
        }
    }
    if (p_stk_data == (OS_STK_DATA *)0) {               
        return (9u);
    }

    p_stk_data->OSFree = 0;                             
    p_stk_data->OSUsed = 0;
    {cpu_sr = OS_CPU_SR_Save();};
    if (prio == 0xFFu) {                         
        prio = OSTCBCur->OSTCBPrio;
    }
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                          
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);
    }
    if (ptcb == ((OS_TCB *)1)) {
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);
    }
    if ((ptcb->OSTCBOpt & 0x0001u) == 0) {  
        {OS_CPU_SR_Restore(cpu_sr);};
        return (69u);
    }
    nfree = 0;
    size  = ptcb->OSTCBStkSize;
    pchk  = ptcb->OSTCBStkBottom;
    {OS_CPU_SR_Restore(cpu_sr);};

    while (*pchk++ == (OS_STK)0) {                     
        nfree++;
    }





    p_stk_data->OSFree = nfree * sizeof(OS_STK);           
    p_stk_data->OSUsed = (size - nfree) * sizeof(OS_STK);  
    return (0u);
}

 





















 


INT8U  OSTaskSuspend (INT8U prio)
{
    BOOLEAN    self;
    OS_TCB    *ptcb;
    INT8U      y;

    OS_CPU_SR  cpu_sr = 0;





    if (prio == (31)) {                             
        return (71u);
    }
    if (prio >= 31) {                                
        if (prio != 0xFFu) {
            return (42u);
        }
    }

    {cpu_sr = OS_CPU_SR_Save();};
    if (prio == 0xFFu) {                                  
        prio = OSTCBCur->OSTCBPrio;
        self = 1u;
    } else if (prio == OSTCBCur->OSTCBPrio) {                    
        self = 1u;
    } else {
        self = 0u;                                         
    }
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                                   
        {OS_CPU_SR_Restore(cpu_sr);};
        return (72u);
    }
    if (ptcb == ((OS_TCB *)1)) {                               
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);
    }
    y            = ptcb->OSTCBY;
    OSRdyTbl[y] &= ~ptcb->OSTCBBitX;                             
    if (OSRdyTbl[y] == 0) {
        OSRdyGrp &= ~ptcb->OSTCBBitY;
    }
    ptcb->OSTCBStat |= 0x08u;                          
    {OS_CPU_SR_Restore(cpu_sr);};
    if (self == 1u) {                                       
        OS_Sched();                                              
    }
    return (0u);
}

 

















 


INT8U  OSTaskQuery (INT8U prio, OS_TCB *p_task_data)
{
    OS_TCB    *ptcb;

    OS_CPU_SR  cpu_sr = 0;





    if (prio > 31) {                  
        if (prio != 0xFFu) {
            return (42u);
        }
    }
    if (p_task_data == (OS_TCB *)0) {             
        return (9u);
    }

    {cpu_sr = OS_CPU_SR_Save();};
    if (prio == 0xFFu) {                   
        prio = OSTCBCur->OSTCBPrio;
    }
    ptcb = OSTCBPrioTbl[prio];
    if (ptcb == (OS_TCB *)0) {                    
        {OS_CPU_SR_Restore(cpu_sr);};
        return (41u);
    }
    if (ptcb == ((OS_TCB *)1)) {                
        {OS_CPU_SR_Restore(cpu_sr);};
        return (67u);
    }
                                                  
    OS_MemCopy((INT8U *)p_task_data, (INT8U *)ptcb, sizeof(OS_TCB));
    {OS_CPU_SR_Restore(cpu_sr);};
    return (0u);
}

 





















 

void  OS_TaskStkClr (OS_STK *pbos, INT32U size, INT16U opt)
{
    if ((opt & 0x0001u) != 0x0000) {        
        if ((opt & 0x0002u) != 0x0000) {    

            while (size > 0) {                          
                size--;
                *pbos++ = (OS_STK)0;                    
            }
#line 1091 "..\\..\\..\\OS\\uCOS-II\\Source\\os_task.c"
        }
    }
}

