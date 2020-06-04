/*
*********************************************************************************************************
*                                                uC/OS-II
*                                          The Real-Time Kernel
*                                  uC/OS-II Configuration File for V2.8x
*
*                               (c) Copyright 2005-2009, Micrium, Weston, FL
*                                          All Rights Reserved
*
*
* File    : OS_CFG.H
* By      : Jean J. Labrosse
* Version : V2.89
*
* LICENSING TERMS:
* ---------------
*   uC/OS-II is provided in source form for FREE evaluation, for educational use or for peaceful research.
* If you plan on using  uC/OS-II  in a commercial product you need to contact Micriµm to properly license
* its use in your product. We provide ALL the source code for your convenience and to help you experience
* uC/OS-II.   The fact that the  source is provided does  NOT  mean that you can use it without  paying a
* licensing fee.
*********************************************************************************************************
*/

#ifndef OS_CFG_H
#define OS_CFG_H


                                       /* ---------------------- MISCELLANEOUS ----------------------- */
#define OS_APP_HOOKS_EN           0u   /* Application-defined hooks are called from the uC/OS-II hooks */
#define OS_ARG_CHK_EN             0u   /* Enable (1) or Disable (0) argument checking                  *///作用:设定系统中是否使用参数检查功能。
#define OS_CPU_HOOKS_EN           1u   /* uC/OS-II hooks are found in the processor port files         *///作用：设定是否在文件OS_CPU_C.C中实现各钩子函数（Hook Function）.如果要实现钩子函数，则常量OS_CPU_HOOKS_EN必须设置为1.

#define OS_DEBUG_EN               0u   /* Enable(1) debug variables                                    */

#define OS_EVENT_MULTI_EN         1u   /* Include code for OSEventPendMulti()                          */
#define OS_EVENT_NAME_EN          0u   /* Enable names for Sem, Mutex, Mbox and Q                      */

#define OS_LOWEST_PRIO           10u   /* Defines the lowest priority that can be assigned ...         *///作用：设置程序中最低任务的优先级。
                                       /* ... MUST NEVER be higher than 254!                           */

#define OS_MAX_EVENTS            20u   /* Max. number of event control blocks in your application      *///作用：设置程序中可以具有事件控制块的最大数量...即程序设计中信号量，邮箱和消息队列的个数
#define OS_MAX_FLAGS              5u   /* Max. number of Event Flag Groups    in your application      *///作用：设定事件标志组的最大数目
#define OS_MAX_MEM_PART           5u   /* Max. number of memory partitions                             *///作用：设置系统中内存块的最大数目。
#define OS_MAX_QS                 5u   /* Max. number of queue control blocks in your application      *///作用：设置系统中具有消息队列的最大数目
#define OS_MAX_TASKS              8u   /* Max. number of tasks in your application, MUST be >= 2       *///作用：设置用户程序中可以使用的最多任务数。


#define OS_TICK_STEP_EN           1u   /* Enable tick stepping feature for uC/OS-View                  */
#define OS_TICKS_PER_SEC        100u   /* Set the number of ticks in one second                        *///作用：设置调用OSTimeTick()函数的频率，即时钟最小单位的设定。


                                       /* --------------------- TASK STACK SIZE ---------------------- */
#define OS_TASK_TMR_STK_SIZE    100u   /* Timer      task stack size (# of OS_STK wide entries)        */
#define OS_TASK_STAT_STK_SIZE   32u   /* Statistics task stack size (# of OS_STK wide entries)        *///作用：设定统计任务的任务堆栈容量。
#define OS_TASK_IDLE_STK_SIZE   64u   /* Idle       task stack size (# of OS_STK wide entries)        *///作用：设置UC/OS操作系统中空闲任务堆栈的容量


                                       /* --------------------- TASK MANAGEMENT ---------------------- */
#define OS_SCHED_LOCK_EN          1u   /* Include code for OSSchedLock() and OSSchedUnlock()           */ //作用：设定应用程序中是否使用关调度锁函数OSSchedLock()和开调度锁函数OSSchedUnlock()。
#define OS_TASK_CHANGE_PRIO_EN    0u   /*     Include code for OSTaskChangePrio()                      *///作用：设定程序中是否使用UC/OS的改变任务优先级函数OSTaskChangePrio().
#define OS_TASK_CREATE_EN         1u   /*     Include code for OSTaskCreate()                          *///作用：设定控制用户程序是否使用OSTaskCreate()函数
#define OS_TASK_CREATE_EXT_EN     1u   /*     Include code for OSTaskCreateExt()                       *///作用：设置程序中是否使用OSTaskCreateExt()
#define OS_TASK_DEL_EN            1u   /*     Include code for OSTaskDel()                             *///作用：设定程序中是否使用删除任务函数OSTaskDel()
#define OS_TASK_NAME_EN           0u   /*     Enable task names                                        */
#define OS_TASK_PROFILE_EN        1u   /*     Include variables in OS_TCB for profiling                */
#define OS_TASK_QUERY_EN          1u   /*     Include code for OSTaskQuery()                           *///作用：设定程序中是否需要使用获取任务信息函数OSTaskQuery。
#define OS_TASK_STAT_EN           0u   /*     Enable (1) or Disable(0) the statistics task             *///作用：设置系统是否使用UC/OS中的统计任务OSTaskStat()及其初始化函数。
#define OS_TASK_STAT_STK_CHK_EN   1u   /*     Check task stacks from statistic task                    */
#define OS_TASK_SUSPEND_EN        1u   /*     Include code for OSTaskSuspend() and OSTaskResume()      *///作用：设定程序中使用任务挂起和唤醒函数OSTaskSupend()和OSTaskResume().
#define OS_TASK_SW_HOOK_EN        1u   /*     Include code for OSTaskSwHook()                          */
#define OS_TASK_REG_TBL_SIZE      1u   /*     Size of task variables array (#of INT32U entries)        */


                                       /* ----------------------- EVENT FLAGS ------------------------ */
#define OS_FLAG_EN                1u   /* Enable (1) or Disable (0) code generation for EVENT FLAGS    *///作用：设定程序中是否使用事件标志组。
#define OS_FLAG_ACCEPT_EN         1u   /*     Include code for OSFlagAccept()                          *///作用：设定程序中是否需要使用OSFlagAccept()。
#define OS_FLAG_DEL_EN            1u   /*     Include code for OSFlagDel()                             *///作用：设定应用程序中是否需要使用OSFlagDel()函数。
#define OS_FLAG_NAME_EN           1u   /*     Enable names for event flag group                        */
#define OS_FLAG_QUERY_EN          1u   /*     Include code for OSFlagQuery()                           *///作用：设定程序中是否使用OSFlagQuery()函数
#define OS_FLAG_WAIT_CLR_EN       1u   /* Include code for Wait on Clear EVENT FLAGS                   */
#define OS_FLAGS_NBITS           16u   /* Size in #bits of OS_FLAGS data type (8, 16 or 32)            */


                                       /* -------------------- MESSAGE MAILBOXES --------------------- */
#define OS_MBOX_EN                0u   /* Enable (1) or Disable (0) code generation for MAILBOXES      *///作用：设置程序是否使用消息邮箱函数及其相关数据结构。
#define OS_MBOX_ACCEPT_EN         1u   /*     Include code for OSMboxAccept()                          *///作用：设定程序中是否需要使用OSMboxAccept()函数。
#define OS_MBOX_DEL_EN            1u   /*     Include code for OSMboxDel()                             *///作用：设定程序中是否使用OSMboxDel()函数
#define OS_MBOX_PEND_ABORT_EN     1u   /*     Include code for OSMboxPendAbort()                       */
#define OS_MBOX_POST_EN           1u   /*     Include code for OSMboxPost()                            *///作用：设定程序中是否使用OSMboxPost()函数
#define OS_MBOX_POST_OPT_EN       1u   /*     Include code for OSMboxPostOpt()                         *///作用：设定程序中是否使用OSMboxPostOpt()函数
#define OS_MBOX_QUERY_EN          1u   /*     Include code for OSMboxQuery()                           *///作用：设定程序中是否使用OSMboxQuery().


                                       /* --------------------- MEMORY MANAGEMENT -------------------- */
#define OS_MEM_EN                 0u   /* Enable (1) or Disable (0) code generation for MEMORY MANAGER *///作用：设置程序中是否使用内存块管理函数及其相关数据结构。
#define OS_MEM_NAME_EN            0u   /*     Enable memory partition names                            */
#define OS_MEM_QUERY_EN           0u   /*     Include code for OSMemQuery()                            *///作用：设定程序中是否使用OSMemQuery()函数(查询内存分区状态函数)


                                       /* ---------------- MUTUAL EXCLUSION SEMAPHORES --------------- */
#define OS_MUTEX_EN               1u   /* Enable (1) or Disable (0) code generation for MUTEX          *///作用：设定程序中是否使用互斥信号量
#define OS_MUTEX_ACCEPT_EN        1u   /*     Include code for OSMutexAccept()                         *///作用：设定程序中是否使用无等待获取互斥型信号量函数OSMutexAccept()
#define OS_MUTEX_DEL_EN           1u   /*     Include code for OSMutexDel()                            *///作用：设定程序中是否使用OSMutexDel()函数。
#define OS_MUTEX_QUERY_EN         1u   /*     Include code for OSMutexQuery()                          *///作用：设定程序中是否使用OSMutexQuery()函数。


                                       /* ---------------------- MESSAGE QUEUES ---------------------- */
#define OS_Q_EN                   1u   /* Enable (1) or Disable (0) code generation for QUEUES         *///作用：设定程序中是否使用消息队列函数及其相关数据结构
#define OS_Q_ACCEPT_EN            1u   /*     Include code for OSQAccept()                             *///作用：设定程序中是否使用OSQAccept()
#define OS_Q_DEL_EN               1u   /*     Include code for OSQDel()                                *///作用：设定程序中是否使用OSQDel()
#define OS_Q_FLUSH_EN             1u   /*     Include code for OSQFlush()                              *///作用：设定程序中是否使用OSQFlush()（清空消息队列函数）
#define OS_Q_PEND_ABORT_EN        1u   /*     Include code for OSQPendAbort()                          */
#define OS_Q_POST_EN              1u   /*     Include code for OSQPost()                               *///作用：设定程序中是否使用按FIFO规则向消息队列发送消息函数OSQPost()函数
#define OS_Q_POST_FRONT_EN        1u   /*     Include code for OSQPostFront()                          *///作用：设定程序中是否使用按LIFO规则向消息队列发送消息函数OSQPostFront()函数
#define OS_Q_POST_OPT_EN          1u   /*     Include code for OSQPostOpt()                            *///作用：设定程序中是否使用按FIFO或LIFO规则向消息队列发送消息函数OSQPostOpt().
#define OS_Q_QUERY_EN             1u   /*     Include code for OSQQuery()                              *///作用：设定程序中是否使用OSQQuery()函数。


                                       /* ------------------------ SEMAPHORES ------------------------ */
#define OS_SEM_EN                 1u   /* Enable (1) or Disable (0) code generation for SEMAPHORES     *///作用：设定程序中是否使用信号量管理函数和其相关数据结构
#define OS_SEM_ACCEPT_EN          1u   /*    Include code for OSSemAccept()                            *///作用：设定程序中是否需要使用无等待获取信号量函数OSSemAccept()
#define OS_SEM_DEL_EN             1u   /*    Include code for OSSemDel()                               *///作用：设定程序中是否需要使用删除信号量函数OSSemDel()
#define OS_SEM_PEND_ABORT_EN      1u   /*    Include code for OSSemPendAbort()                         */
#define OS_SEM_QUERY_EN           1u   /*    Include code for OSSemQuery()                             *///作用：应用系统是否需要使用查询信号量状态函数OSSemQuery()
#define OS_SEM_SET_EN             1u   /*    Include code for OSSemSet()                               */


                                       /* --------------------- TIME MANAGEMENT ---------------------- */
#define OS_TIME_DLY_HMSM_EN       1u   /*     Include code for OSTimeDlyHMSM()                         *///作用：设定程序中是否使用OSTimeDlyHMSM()函数。
#define OS_TIME_DLY_RESUME_EN     1u   /*     Include code for OSTimeDlyResume()                       *///作用：设定应用系统是否需要使用OSTimeDlyResume()函数。
#define OS_TIME_GET_SET_EN        1u   /*     Include code for OSTimeGet() and OSTimeSet()             *///作用：设定应用系统中是否使用OSTimeGet()函数。
#define OS_TIME_TICK_HOOK_EN      1u   /*     Include code for OSTimeTickHook()                        */


                                       /* --------------------- TIMER MANAGEMENT --------------------- */
#define OS_TMR_EN                 0u   /* Enable (1) or Disable (0) code generation for TIMERS         */
#define OS_TMR_CFG_MAX           16u   /*     Maximum number of timers                                 */
#define OS_TMR_CFG_NAME_EN        1u   /*     Determine timer names                                    */
#define OS_TMR_CFG_WHEEL_SIZE     8u   /*     Size of timer wheel (#Spokes)                            */
#define OS_TMR_CFG_TICKS_PER_SEC 10u   /*     Rate at which timer management task runs (Hz)            */

#endif
