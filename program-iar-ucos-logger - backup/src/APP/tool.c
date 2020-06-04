#include "tool.h"

#define ABS(cond) (cond>0?cond:-cond)

/****************************************************************************
* 名    称：ToolDelay()
* 功    能：短延时。
* 入口参数：
*           uInDelay     输入延时时间，多少个
* 出口参数：无
* 范    例: 无
****************************************************************************/
void ToolDelay(uint16_t uInDelay)
{
    // 10000约为3.1mS。
    volatile uint16_t uDelay;

    uDelay = uInDelay;

    while(uDelay--);
}
/****************************************************************************
* 名    NoTimeOut()
* 功    能：判断时间差
* 入口参数：
*         sStart：起始时间
*         sNow：  现在的时间
*         uGap:： 时间差值
* 出口参数：
          未达到时间差返回 NOTIMEOUT；达到时间差返回TIMEOUT
* 范    例: 无
****************************************************************************/
int8_t NoTimeOut(uint32_t sStart,uint32_t sNow,uint32_t uGap)
{
    // ucos的tick为32位整型最多计时到0xFFFFFFFF，防止溢出。判断起始时间和现在时间的大小
    if(((sNow>=sStart)?(sNow - sStart):(0xFFFFFFFF-sStart+sNow)) < (uGap/10))   //if((sNow - sStart)<(uGap/10))
    {
        return NOTIMEOUT;
    }

    return TIMEOUT;
}
/****************************************************************************
* 名    TimeOut()
* 功    能：判断时间差
* 入口参数：
*         sStart：起始时间
*         sNow：  现在的时间
*         uGap:： 时间差值
* 出口参数：
          达到时间差返回 1；未达到时间差返回0
* 范    例: 无
****************************************************************************/
int8_t TimeOut(uint32_t sStart,uint32_t sNow,uint32_t uGap)
{
    // ucos的tick为32位整型最多计时到0xFFFFFFFF，防止溢出。判断起始时间和现在时间的大小
    if(((sNow>=sStart)?(sNow - sStart):(0xFFFFFFFF-sStart+sNow)) > (uGap/10))   //if((sNow - sStart)<(uGap/10))
    {
        return TIMEOUT;
    }

    return NOTIMEOUT;
}
/******************************************************************************
* 名    称：Str2IP()
* 功    能：找出字符串中的IP地址并转换为数字
* 入口参数：
*           *str     输入字符串
            *ip      存放IP地址的数组指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Str2IP(const char* str,uint8_t *ip)
{
    uint8_t i,k;
    uint8_t iptemp;
    for(i=0;i<18;i++)
    {
        if('\"'==str[i])
        {
            break;
        }
    }
    for(k=0; k<4; k++)
    {
        i++;
        iptemp = str[i] - '0';
        i++;
        if('.'==str[i] || '\"'==str[i])
        {
            ip[k] = iptemp;
            continue;
        }
        else
        {
            iptemp = iptemp*10 + str[i] - '0';
        }
        i++;
        if('.'==str[i] || '\"'==str[i])
        {
            ip[k] = iptemp;
            continue;
        }
        else
        {
            iptemp = iptemp*10 + str[i] - '0';
            ip[k] = iptemp;
        }

        i++;
        if('.'==str[i] || '\"'==str[i])
        {
            continue;
        }
    }
}

/******************************************************************************
* 名    称：Str2UI()
* 功    能：找出字符串中的数字并转换为整型
* 入口参数：
*           *str     输入字符串
            *ip      存放IP地址的数组指针

* 出口参数：失败返回1
* 范    例:
******************************************************************************/
uint8_t Str2UI(const char* str,uint16_t *result)
{
    uint8_t i,k=1;
    uint16_t temp;

    for(i=0;i<20;i++)
    {
        if('\"'==str[i])
        {
            if(k)
            {
                k = 0;
                temp = 0;
                continue;
            }
            else
            {
                *result = temp;
                return 0;
            }
        }
        else if(str[i]>'9' || str[i]<'0')
        {
            continue;//return 1;
        }
        else
        {
            temp = temp*10 + (str[i]-'0');
        }
    }
    return 1;
}
/******************************************************************************
* 名    称：UI2Str()
* 功    能：找出字符串中的数字并转换为整型
* 入口参数：
*           in           输入整数
            *result      存放字符串的指针

* 出口参数：字符串指针
* 范    例:
******************************************************************************/
char *I2Str(const uint16_t in,char* result)
{
    char str[7]={'\0'};
    uint16_t temp;
    int8_t i,j;


    temp = ABS(in);
    /*if(in>99999)
    {
        return 1;
    }*/

    for(i=0;i<5;i++)
    {
        str[i] = (temp%10)+'0';
        temp /= 10;
        if(0==temp)
        {
            break;
        }
    }

    /*if(in<0)
    {
        str[++i] = '-';
    }*/

    for(j=0;i>=0;i--)
    {
        result[j++] = str[i];
    }

    result[j] = '\0';

    return result;
}
/******************************************************************************
* 名    称：UI2Str()
* 功    能：找出字符串中的数字并转换为整型
* 入口参数：
*           in           输入整数
            *result      存放字符串的指针

* 出口参数：字符串指针
* 范    例:
******************************************************************************/
/*char * strset(char *s, char c,uint8_t byts)
{
    uint8_t i;

    for(i=0;i<bytes;i++)
    {
        s[
    }
}*/
