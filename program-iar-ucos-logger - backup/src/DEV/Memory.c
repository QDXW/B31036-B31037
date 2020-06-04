#include "Memory.h"

#include "ucos_ii.h"

#include <string.h>
//#include <stdlib.h>

//#define NULL ((void *) 0)

#define MEM_BLOCK_SIZE          32                             // 内存块大小
#define MAX_MEM_SIZE            17408                          // 堆内存大小 17KB
#define MEM_ALLOC_TABLE_SIZE    (MAX_MEM_SIZE/MEM_BLOCK_SIZE)  // 内存表大小
static char memory_map[MEM_ALLOC_TABLE_SIZE]={0};   //内存表
static unsigned char memory_heap[MAX_MEM_SIZE];     //堆内存
  
/*
* @fn malloc
* @param unsigned int size 申请的内存大小
* @return 申请到的内存块指针
*/
/******************************************************************************
* 名    称：WMemMalloc()
* 功    能：申请动态内存
* 入口参数：
            *ptr         待申请动态内存的指针
            size         内存大小，字节
*
* 出口参数：申请成功返回指针，申请失败返回NULL
* 范    例:
******************************************************************************/
void *WMemMalloc(void *ptr,unsigned int size)
{
    int offset;
    int nmemb;
    int i;
    int vacantSize;

    OS_CPU_SR  cpu_sr;

    if(size==0)
    {
        return NULL;
    }

    if(NULL!=ptr)
    {
        ptr = WMemFree(ptr);  // 先释放内存，避免同一个指针重复申请内存空间
    }

    OS_ENTER_CRITICAL();

    nmemb=size/MEM_BLOCK_SIZE;  //需要的内存块个数
    if(size%MEM_BLOCK_SIZE)
    {
        nmemb++;
    }

    for(offset=MEM_ALLOC_TABLE_SIZE-1;offset>=0;offset--)  //从后往前，用掉的内存块在内存表表中置上块个数的标记
    {
        if(!memory_map[offset] && ((offset+nmemb)<=MEM_ALLOC_TABLE_SIZE))
        {
            vacantSize=0;
            for(vacantSize=0;vacantSize<nmemb && !memory_map[offset+vacantSize];vacantSize++);
            if(vacantSize==nmemb)
            {
                for(i=0;i<nmemb;i++)
                {
                    memory_map[offset+i]=nmemb;
                }
                memset(memory_heap+(offset*MEM_BLOCK_SIZE),0,size);

                OS_EXIT_CRITICAL();

                return memory_heap+(offset*MEM_BLOCK_SIZE);  //返回起始块的地址
            }
        }
    }
    OS_EXIT_CRITICAL();
    return NULL;
}
/*
* @fn free
* @param void *ptr 释放内存首地址
* @return void
*/
/******************************************************************************
* 名    称：WMemFree()
* 功    能：申请动态内存
* 入口参数：
            *ptr         已申请动态内存的指针
*
* 出口参数：返回NULL
* 范    例:
******************************************************************************/
void *WMemFree(void *ptr)
{
    int offset;
    int i;
    int index;
    int nmemb;

    OS_CPU_SR  cpu_sr;

    if(ptr==NULL)
    {
        return NULL;
    }

    if((int)ptr < (int)memory_heap)
    {
        return NULL;
    }

    OS_ENTER_CRITICAL();

    offset=(int)ptr-(int)memory_heap; //在内存表中的偏移
    if(offset<MAX_MEM_SIZE)
    {
        index=offset/MEM_BLOCK_SIZE;
        nmemb=memory_map[index];
        for(i=0;i<nmemb;i++)
        {
            memory_map[index+i]=0;  //释放的内存在表中置0
        }
    }

    OS_EXIT_CRITICAL();
    return NULL;
}
/******************************************************************************
* 名    称：FreeRamSpace()
* 功    能：统计动态内存剩余字节数
* 入口参数：
            无
*
* 出口参数：返回可用的内存总数
* 范    例:
******************************************************************************/
uint16_t FreeRamSpace(void)
{
    uint16_t uFreeSpace=0;
    uint16_t i;
    for(i=0;i<MEM_ALLOC_TABLE_SIZE;i++)
    {
        if(0==memory_map[i])
        {
            uFreeSpace++;
        }
    }

    return uFreeSpace*MEM_BLOCK_SIZE;
}

