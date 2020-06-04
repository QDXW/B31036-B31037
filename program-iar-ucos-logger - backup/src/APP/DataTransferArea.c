#include "DataTransferArea.h"

#include "GlobalVar.h"
#include "Memory.h"


/***********************************************************
本文件为数据传输区域
***********************************************************/

//=======================================================================
#define    N_MESSAGES     23
OS_EVENT   *MesQ;                   // 定义事件控制块指针 队列的事件控制块指针 用于存放创建的消息队列的指针
void       *MsgGrp[N_MESSAGES];     // 定义消息指针数组

//uint16_t   g_uTransData;

//=======================================================================
//=======================================================================
// 链表结构体
typedef struct LINK_LIST
{
    uint8_t *buff;      // 缓存空间
    uint8_t bytes;      // 缓存字节数
    struct LINK_LIST *next;
}LINK_LIST_T;

static LINK_LIST_T *LinkCreateNew(uint8_t uSize);
static uint16_t LinkGetLen(LINK_LIST_T *link);
static uint8_t LinkStoreData(LINK_LIST_T* *link,uint8_t *input,uint8_t bytes);
static uint8_t LinkReadData(LINK_LIST_T* *link,uint8_t *output);
static void LinkFree(LINK_LIST_T *del);
static void LinkDeleteAll(LINK_LIST_T* *head);
//=======================================================================
//=======================================================================
//模块数据收发链表相关
static LINK_LIST_T *pLinkRecHead;   // 接收数据的链表头
static LINK_LIST_T *pLinkSendHead;  // 发送数据的链表头

OS_EVENT *pLinkRecLock;  // 接收链表读写锁
OS_EVENT *pLinkSendLock; // 发送链表读写锁
//OS_EVENT *pRecordLock;   // 历史数据存储读取锁
//=======================================================================


/*
// 内存块定义
#define MEM_BLOCK_NUM      50
#define MEM_BLOCK_SIZE     32
OS_MEM        *pMemBank;         // 定义内存控制块指针，也即是指向内存分区的指针，创建一个
                                 // 内存分区时，返回值就是它  OS_MEM 内存控制块类型的指针
uint8_t       uMemBank[MEM_BLOCK_NUM][MEM_BLOCK_SIZE];  // 划分一个具有MEM_BLOCK_NUM个内存块，每个内存块长度是MEM_BLOCK_SIZE个字节的内存分区
//=======================================================================

// 内存块初始化
void MemBankInit(void)
{
    uint8_t err;
    pMemBank = OSMemCreate(uMemBank,MEM_BLOCK_NUM,MEM_BLOCK_SIZE,&err);    //创建动态内存区 函数参数为：IntPart为内存分区的起始地址

    //前面已经定义了uint8_t  uMemBank[MEM_BLOCK_NUM][MEM_BLOCK_SIZE];表示内存分区，用数组名表示起始地址
    //第二个参数MEM_BLOCK_NUM表示分区中内存块的数目，第三个参数MEM_BLOCK_SIZE表示每个内存块的字节数，最后&err为错误信息
}*/
//=======================================================================
// 系统信号初始化
void SysMesInit(void)
{
    //uint8_t err = 0;
    MesQ = OSQCreate(&MsgGrp[0],N_MESSAGES);       // 创建消息队列
    //pRecordLock = OSMutexCreate(4,&err);           // 历史数据存储读取锁
}

//=======================================================================
//=======================================================================
//=======================================================================

/******************************************************************************
* 名    称：CmLinkInit()
* 功    能：链表初始化
* 入口参数：
*           无
* 出口参数：无
* 范    例:
******************************************************************************/
void CmLinkInit(void)
{
    uint8_t err = 0;

    pLinkRecHead  = NULL;//LinkCreateNew(255);  // 创建接收链表
    pLinkSendHead = NULL;//LinkCreateNew(255);  // 创建发送链表

    pLinkRecLock  = OSMutexCreate(1,&err);
    pLinkSendLock = OSMutexCreate(2,&err);
}
/******************************************************************************
* 名    称：CmRecLinkLen()
* 功    能：读接收链表字节数
* 入口参数：
*           无
* 出口参数：链表字节数
* 范    例:
******************************************************************************/
uint16_t CmRecLinkLen(void)
{
    return LinkGetLen(pLinkRecHead);
}
/******************************************************************************
* 名    称：CmRecLinkLen()
* 功    能：读接收链表字节数
* 入口参数：
*           无
* 出口参数：链表字节数
* 范    例:
******************************************************************************/
uint16_t CmSendLinkLen(void)
{
    return LinkGetLen(pLinkSendHead);
}
/******************************************************************************
* 名    称：CmStoreRecLink()
* 功    能：数据存储到接收链表
* 入口参数：
*           *input     输入数据头指针
            bytes      字节数
* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t CmStoreRecLink(uint8_t *input,uint8_t bytes)
{
    uint8_t err = 0;
    //请求信号量
    OSMutexPend(pLinkRecLock,0,&err);
    err =  LinkStoreData(&pLinkRecHead,input,bytes);
    //释放信号量
    OSMutexPost(pLinkRecLock);
    return err;
}
/******************************************************************************
* 名    称：CmReadRecLink()
* 功    能：读接收链表
* 入口参数：
*           *output     待存放数据头指针
* 出口参数：被读取的字节数
* 范    例:
******************************************************************************/
uint8_t CmReadRecLink(uint8_t *output)
{
    uint8_t err = 0;
    //请求信号量
    OSMutexPend(pLinkRecLock,0,&err);
    err =  LinkReadData(&pLinkRecHead,output);
    //释放信号量
    OSMutexPost(pLinkRecLock);
    return err;
}
/******************************************************************************
* 名    称：CM_store_send_link()
* 功    能：数据存储到发送链表
* 入口参数：
*           *input     输入数据头指针
            bytes      字节数
* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t CmStoreSendLink(uint8_t *input,uint8_t bytes)
{
    uint8_t err = 0;
    //请求信号量
    OSMutexPend(pLinkSendLock,0,&err);
    err = LinkStoreData(&pLinkSendHead,input,bytes);
    //释放信号量
    OSMutexPost(pLinkSendLock);
    return err;
}
/******************************************************************************
* 名    称：CmReadSendLink()
* 功    能：读发送链表
* 入口参数：
*           *output     待存放数据头指针
* 出口参数：被读取的字节数
* 范    例:
******************************************************************************/
uint8_t CmReadSendLink(uint8_t *output)
{
    uint8_t err = 0;
    //请求信号量
    OSMutexPend(pLinkSendLock,0,&err);
    err =  LinkReadData(&pLinkSendHead,output);
    //释放信号量
    OSMutexPost(pLinkSendLock);
    return err;
}
/******************************************************************************
* 名    称：CmDelRecLink()
* 功    能：删除所有接收链表
* 入口参数：无
* 出口参数：无
* 范    例:
******************************************************************************/
void CmDelRecLink(void)
{
    uint8_t err = 0;
    //请求信号量
    OSMutexPend(pLinkRecLock,0,&err);
    LinkDeleteAll(&pLinkRecHead);
    //释放信号量
    OSMutexPost(pLinkRecLock);
}
/******************************************************************************
* 名    称：CmDelSendLink()
* 功    能：删除所有发送链表
* 入口参数：无
* 出口参数：无
* 范    例:
******************************************************************************/
void CmDelSendLink(void)
{
    uint8_t err = 0;
    //请求信号量
    OSMutexPend(pLinkSendLock,0,&err);
    LinkDeleteAll(&pLinkSendHead);
    //释放信号量
    OSMutexPost(pLinkSendLock);
}
//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================
/******************************************************************************
* 名    称：LinkStoreData()
* 功    能：往链表存数据
* 入口参数：
*           **link      链表头指针的指针
            *input      待存入数据指针
            bytes       待存入字节数

* 出口参数：0：存入数据失败，其他为存入字节数
* 范    例:LinkStoreData(link_recv_head,input,bytes);
******************************************************************************/
uint8_t LinkStoreData(LINK_LIST_T **link,uint8_t *input,uint8_t bytes)
{
    LINK_LIST_T *pnew,*ptail;//,*link;

    if(0==bytes)
    {
        return 0;
    }

    pnew = LinkCreateNew(bytes);//pnew = Link_Creat();  // 创建一个链表节点
    if(NULL!=pnew)
    {
        memcpy(pnew->buff,(const uint8_t*)input,bytes);
        pnew->bytes = bytes;// 存数据
        pnew->next  = NULL;

        if(NULL==*link)  // 链表没有任何节点
        {
            *link = pnew;
        }
        else
        {
            ptail = *link;

            while(NULL!=ptail->next)  // 找出尾节点
            {
                ptail = ptail->next;
            }

            ptail->next = pnew;
        }

        return bytes;
    }
    else
    {
        return 0;
    }
}
/******************************************************************************
* 名    称：LinkReadData()
* 功    能：往链表存数据
* 入口参数：
*           **link      链表头指针的指针
            *output     存放待读取数据指针

* 出口参数：0：读取数据失败或没有数据，其他为读取字节数
* 范    例:LinkReadData(link_recv_head,output);
******************************************************************************/
uint8_t LinkReadData(LINK_LIST_T **link,uint8_t *output)
{
    LINK_LIST_T *nlink;
    uint8_t rbytes;

    if(NULL==*link)
    {
        return 0;
    }

    nlink = (*link)->next;

    rbytes = (*link)->bytes;

    if(0!=rbytes)
    {
        memcpy((uint8_t*)output,(const uint8_t*)(*link)->buff,rbytes);
    }

    LinkFree(*link); // 释放已经被读取数据的节点空间

    *link = nlink;

    return rbytes;
}
/******************************************************************************
* 名    称：LinkGetLen()
* 功    能：获取链表的字节数
* 入口参数：
*           *link      待操作链表头

* 出口参数：链表字节数
* 范    例:
******************************************************************************/
uint16_t LinkGetLen(LINK_LIST_T *link)
{
    if(NULL==link)
    {
        return 0;
    }

    return link->bytes;
}
/******************************************************************************
* 名    称：LinkCreateNew()
* 功    能：创建链表
* 入口参数：
*           uSize      数据数量

* 出口参数：创建的链表指针
* 范    例:
******************************************************************************/
LINK_LIST_T *LinkCreateNew(uint8_t uSize)
{
    LINK_LIST_T *head=NULL;
    uint8_t *data=NULL;

    data = (uint8_t *)WMemMalloc(data,uSize*sizeof(uint8_t));
    if(NULL==data)
    {
        DEBUGOUT("链表数据失败\r\n");
        Reboot();// 标记重启芯片
        return NULL;
    }

    head = (LINK_LIST_T *)WMemMalloc(head,sizeof(LINK_LIST_T));

    if(NULL==head)
    {
        //free(data);
        WMemFree(data);
        DEBUGOUT("链表节点头失败\r\n");
        return NULL;
    }

    head->buff = data;
    head->next = NULL;
    head->bytes = 0x00;

    return head;
}

/******************************************************************************
* 名    称：Link_Creat()
* 功    能：插入链表节点
* 入口参数：
*           无

* 出口参数：创建的链表指针
* 范    例:
******************************************************************************/
/*void Link_Insert(LINK_LIST_T *head,LINK_LIST_T *pnew,uint8_t i)
{
    LINK_LIST_T *p;
    uint8_t j;

    p = head;

    for(j=0; j<i&&p!=NULL; j++)
    {
        p = p->next;
    }

    if(NULL==p)
    {
        return;
    }

    pnew->next = p->next;
    p->next = pnew;
}
*/
/******************************************************************************
* 名    称：Link_Delete()
* 功    能：删除链表一个节点
* 入口参数：
*           无

* 出口参数：创建的链表指针
* 范    例:
******************************************************************************/
/*void Link_Delete(LINK_LIST_T *head,uint8_t i)
{
    LINK_LIST_T *p,*del;
    uint8_t j;

    if(0==i)  // 删除的是头指针
    {
        return;
    }

    p = head;
    for(j=1;j<i&&p->next!=NULL;j++) // 将p指向要删除的第i个节点的前驱节点
    {
        p = p->next;
    }
    if(NULL==p->next) // 表明链表中的节点不存在
    {
        return;
    }
    del = p->next;  // del指向待删除的节点
    p->next = del->next;  // 删除节点

    free(del);  // 释放空间
}*/
/******************************************************************************
* 名    称：LinkFree()
* 功    能：释放链表一个节点空间
* 入口参数：
*           *del     待删除链表节点指针

* 出口参数：
* 范    例:
******************************************************************************/
void LinkFree(LINK_LIST_T *del)
{
    WMemFree(del->buff);
    WMemFree(del);  // 释放空间
}

/******************************************************************************
* 名    称：Link_Delete_All()
* 功    能：释放所有链表空间
* 入口参数：
*           **head     待删除链表头指针的指针

* 出口参数：创建的链表指针
* 范    例:
******************************************************************************/
void LinkDeleteAll(LINK_LIST_T **head)
{
    LINK_LIST_T *p,*del;

    p = *head;

    if(NULL!=*head)
    {
        while(NULL!=p->next)
        {
            del = p->next;
            p->next = del->next;
            LinkFree(del);// 删除节点
        }

        LinkFree(*head);// 删除节点
        *head = NULL;
    }
}
