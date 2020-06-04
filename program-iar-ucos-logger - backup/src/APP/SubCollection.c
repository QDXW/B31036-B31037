/**
 * FileName:
 * Company: pinnenger
 * @author chenyang
 * @date 2017-07-30下午11:59:59
 * @version 1.0
 * Description:键盘音效
 *
 * ......................我佛慈悲......................
 *                       _oo0oo_
 *                      o8888888o
 *                      88" . "88
 *                      (| -_- |)
 *                      0\  =  /0
 *                    ___/`---'\___
 *                  .' \\|     |// '.
 *                 / \\|||  :  |||// \
 *                / _||||| -卍-|||||- \
 *               |   | \\\  -  /// |   |
 *               | \_|  ''\---/''  |_/ |
 *               \  .-\__  '-'  ___/-. /
 *             ___'. .'  /--.--\  `. .'___
 *          ."" '<  `.___\_<|>_/___.' >' "".
 *         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *         \  \ `_.   \_ __\ /__ _/   .-` /  /
 *     =====`-.____`.___ \_____/___.-`___.-'=====
 *                       `=---='
 *
 *..................佛祖开光 ,永无BUG...................
 *
 */
#include "SubCollection.h"

#include <stdio.h>
#include <time.h>
#include "IEC104.h"
#include "GlobalVar.h"
#include "WatchDog.h"

#include "DataFlash.h"
#include "Record.h"
#include "InEEPROM.h"
#include "CM.h"

#define OK 0
#define ERR -1
#define INTERVAL_TIME_RANG 150 	//平台指定时间戳前后两分半分钟内
#define NO_DATA -2 //遍历历史数据查找不到需要补采时标前后2.5分钟范围内的数据
#define FEED_WATCH_DOG()   FEED_DOG()    // 喂狗

extern uint8_t g_TimeMarker[7];
/******************************************************************************
* 名    称：tMarkCheck()
* 功    能：检测读取到的历史数据时标是否在需要补采的时标范围内
* 入口参数：
*			char menAddr[]		读取的历史数据中的头标和时标
*			time_t targetTime	平台下发的需要补采的时标,已经换算成经过的秒数
*
* 出口参数：
* 范    例:
* ADD BY: chenyang
******************************************************************************/
int32_t tMarkCheck(uint8_t menAddr[],time_t targetTime )
{
    struct tm src = {0};
    time_t recordTime;
    float result;

    src.tm_sec = menAddr[7];
    src.tm_min = menAddr[6];
    src.tm_hour = menAddr[5];
    src.tm_mday = menAddr[4]&0x1F;//日的高三位是星期，这里只取低五位
    src.tm_wday = menAddr[4]&0xE0;//只取高三位
    src.tm_mon = menAddr[3]-1;
    src.tm_year = menAddr[2]+100;//！！！特别注意：这里是从1900经过的年数，此处要与dst计算差值可以忽略具体年份
    src.tm_isdst = 0;   //不实行夏令时
    recordTime = mktime(&src);

    if(targetTime>=recordTime)
    {
        result = difftime(targetTime,recordTime);    //计算平台下发时间戳与日志中时间戳的间隔差
    }else{
        result = difftime(recordTime,targetTime);    // 更大的时间放在第一个入参
    }
    if(result<=540  && result>=0)
    {
        /*
        g_TimeMarker[0]=menAddr[7]*1000%256;
        g_TimeMarker[1]=menAddr[7]*1000/256;
        g_TimeMarker[2]=menAddr[6];
        g_TimeMarker[3]=menAddr[5];
        g_TimeMarker[4]=menAddr[4];
        g_TimeMarker[5]=menAddr[3]-1;   //上报平台时，月份从0开始计算
        g_TimeMarker[6]=menAddr[2];
        */
//        DEBUGOUT("\\\\\\\\\\\\\\\\\\\\\\\ tMarkCheck Success \\\\\\\\\\\\\\\\\n");
    	return OK;
    }
    return NO_DATA;
}
/******************************************************************************
* 名    称：hMarkCheck()
* 功    能：检测读取到的历史数据起始地址是否为头标
* 入口参数：
*		char array[]	从历史数据中读取的前八个字符，包含了头标和时标
*
* 出口参数：
* 范    例:
* ADD BY: chenyang
******************************************************************************/
int32_t hMarkCheck(uint8_t array[])
{
    if((array[0] == 0xAA)&&(array[1] == 0x55))
    {
        return OK;
    }
    return ERR;
}
/******************************************************************************
* 名    CheckTarget()
* 功    能：检测读取到的历史数据起始地址是否为头标
* 入口参数：
*			char array[]	从历史数据中读取的前八个字符，包含了头标和时标
*
* 出口参数：
* 范    例:
* ADD BY: chenyang
******************************************************************************/
int CheckTarget(uint32_t uReadFlash,time_t targetTime,uint32_t uFrameLenth,uint32_t uTimes,uint32_t *uNeedReadFlash)
{
    int32_t iRecord = NO_DATA;
    uint8_t uSubCheckAddr[SUBCHECKSIZE] = {0};
    uint32_t i=0;
    DataFlash_Read(uReadFlash,uSubCheckAddr,SUBCHECKSIZE); //从历史数据存储的起始位置拷贝头标与时标（2+6）到缓存中    
//    DEBUGOUT("@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@*********First before markCheck****\n");
/*    for (uint8_t j = 0; j < SUBCHECKSIZE; ++j)
    {
        DEBUGOUT("%x ", uSubCheckAddr[j]);
    }
    DEBUGOUT("\n@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@*********First before markCheck****\n");
    */
    //Delay(1);
    for(; i<uTimes && uReadFlash<=(DATAFLASH_RECORD_END-uFrameLenth);i++ )
    {
        if(OK == hMarkCheck(uSubCheckAddr))
        {
 //           DEBUGOUT("\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ hMarkCheck Success \\\\\\\\\\\\\\\\\\\\\\\\\n");
            if(OK == tMarkCheck(uSubCheckAddr ,targetTime))
            {
                
                iRecord = i;
                *uNeedReadFlash = uReadFlash;
  //              DEBUGOUT("@@@@@@@@@@@@@@@@@@@@@@@@@@ tMarkCheck: %d @@@@@@@@@@@@@@@@@@@@@@@@@\n", iRecord);
                return iRecord;
            }
        }
        uReadFlash = uReadFlash + uFrameLenth;
        if(i/20)
        {
            FEED_WATCH_DOG();  // 喂狗
        }
        DataFlash_Read(uReadFlash,uSubCheckAddr,SUBCHECKSIZE); //从历史数据存储的起始位置拷贝头标与时标（2+6）到缓存中
/*        DEBUGOUT("@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@\n");
        for (uint8_t j = 0; j < SUBCHECKSIZE; ++j)
        {
            DEBUGOUT("%x ", uSubCheckAddr[j]);
        }
        DEBUGOUT("\n@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@\n");
        */
    }
    return iRecord;

}
/******************************************************************************
* 名    称： CollectData()
* 功    能：查找平台需要补采的数据
* 入口参数：
*	    uint16_t *uSubCheckAddr    从历史数据中读取的头标和时标数据
*           IEC104_MAIN_T *pA         IEC信息指针
*
* 出口参数：uint32_t uRecord 需要上报的时间戳在历史数据中的第几帧
* 范    例:
* ADD BY: chenyang
******************************************************************************/
int32_t CollectData(IEC104_MAIN_T *pA, uint32_t *uNeedReadFlash)
{

    int32_t iRecord = NO_DATA;
    uint8_t uRes;
    uint32_t uFrameLenth,uOver,uTimes;
    uint32_t uFirstReadFlash,ret1;
    time_t targetTime,tCurrentTime;
    //time_t inteval;

    struct tm dst = {0};
    struct tm cuTm = {0};
    float result = 0;
//    IEC104_MAIN_T *recvpkt = pA;
    SYSTEMTIME *pTime;
    
    pTime = RealTimeGet();

    /*************************************************************
    //由于平台入库时间偏移了5分钟，所以需要将补采的数据向前推移5分钟
    dst.tm_sec = 0;
    dst.tm_min = 5;	//5分钟偏移
    dst.tm_hour = 0;
    dst.tm_mday = 0;
    dst.tm_wday = 0;
    dst.tm_mon = 0;
    dst.tm_year = 0;
    dst.tm_isdst = 0;   //不实行夏令时
    inteval = mktime(&dst);
    *************************************************************/
    //禅道单890，补采第一帧时标错误
/*    DEBUGOUT("******************************************************************\n");
    DEBUGOUT("%x %x %x %x %x %x %x\n", recvpkt->recv.format.data[1],
                                       recvpkt->recv.format.data[2],
                                       recvpkt->recv.format.data[3],
                                       recvpkt->recv.format.data[4],
                                       recvpkt->recv.format.data[5],
                                       recvpkt->recv.format.data[6],
                                       recvpkt->recv.format.data[7]);
    DEBUGOUT("******************************************************************\n");
*/

    //需求文档中日期是7个字符，毫秒分时日月年
//    dst.tm_sec = (recvpkt->recv.format.data[1]+recvpkt->recv.format.data[2]*256)/1000;	//低端模式
//    dst.tm_min = recvpkt->recv.format.data[3];
//    dst.tm_hour = recvpkt->recv.format.data[4];
//    dst.tm_mday = recvpkt->recv.format.data[5]&0x1F;
//    dst.tm_wday = recvpkt->recv.format.data[5]&0xE0;
//    dst.tm_mon = recvpkt->recv.format.data[6];
//    dst.tm_year = recvpkt->recv.format.data[7]+100;//！！！特别注意：这里是从1900经过的年数，此处要与src计算差值可以忽略具体年份
//    dst.tm_isdst = 0;   //不实行夏令时

    dst.tm_sec = (g_TimeMarker[0]+g_TimeMarker[1]*256)/1000;	//低端模式
    dst.tm_min  = g_TimeMarker[2];
    dst.tm_hour = g_TimeMarker[3];
    dst.tm_mday = g_TimeMarker[4]&0x1F;
    dst.tm_wday = g_TimeMarker[4]&0xE0;
    dst.tm_mon  = g_TimeMarker[5];
    dst.tm_year = g_TimeMarker[6]+100;//！！！特别注意：这里是从1900经过的年数，此处要与src计算差值可以忽略具体年份
    dst.tm_isdst = 0;   //不实行夏令时
    targetTime = mktime(&dst);
    struct tm*tm_tar = localtime(&targetTime);
    
    DEBUGOUT("start-0: %d-%d-%d %d:%d:%d\n",tm_tar->tm_year+1900, 
                                            tm_tar->tm_mon+1,
                                            tm_tar->tm_mday, 
                                            tm_tar->tm_hour,
                                            tm_tar->tm_min, 
                                            tm_tar->tm_sec);

    uFrameLenth = g_DeviceSouth.yx_sum + g_DeviceSouth.yc_sum*4+8;
    if(uFrameLenth == 8)
    {
        return iRecord;
    }

    if(0 == sRecord.uExist)//查找最后一条数据
    {
        uRes = EepReadData(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);// 数采读取
        if(0 == uRes)
        {
            sRecord.uFlashAddr = SearchRecordLastAddr(1,uFrameLenth);
        }
    }

    cuTm.tm_sec = pTime->Second;
    cuTm.tm_min = pTime->Minute;
    cuTm.tm_hour = pTime->Hour;
    cuTm.tm_mday = pTime->Date;
    cuTm.tm_mon = pTime->Month-1;
    cuTm.tm_year = pTime->Year+100;//！！！特别注意：这里是从1900经过的年数，此处要与src计算差值可以忽略具体年份
    cuTm.tm_isdst = 0;   //不实行夏令时
    tCurrentTime = mktime(&cuTm);

    //5天所经过的432000秒数
    if(tCurrentTime<targetTime||tCurrentTime>targetTime+432000){
       return iRecord;
    }

    result = difftime(tCurrentTime,targetTime);
    ret1 = (uint32_t)((result/(2*INTERVAL_TIME_RANG)))*uFrameLenth; //历史数据的相应帧数计算
    if(ret1<=sRecord.uFlashAddr)
    {

        DEBUGOUT("ret1<=sRecord.uFlashAddr");
        uFirstReadFlash = sRecord.uFlashAddr-(uint32_t)((result/(2*INTERVAL_TIME_RANG)))*uFrameLenth;
    }else
    {
        DEBUGOUT("ret1>sRecord.uFlashAddr");
        uOver = ret1 - (sRecord.uFlashAddr-DATAFLASH_RECORD_HEAD);
    }
    if(uFirstReadFlash>=DATAFLASH_RECORD_HEAD && ret1<=sRecord.uFlashAddr)
    {
//        DEBUGOUT("&&&&&&&&&&&&&&&&&&&&&&&&&&&uFirstReadFlash>=DATAFLASH_RECORD_HEAD && ret1<=sRecord.uFlashAddr&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
        uTimes = (uint32_t)result/(2*INTERVAL_TIME_RANG);
        iRecord = CheckTarget(uFirstReadFlash,targetTime,uFrameLenth,uTimes,uNeedReadFlash);
    }else
    {
        //当目标地址小于起始地址，需要跳到尾部
        if(ret1<=sRecord.uFlashAddr)
        {
            uOver = DATAFLASH_RECORD_HEAD - uFirstReadFlash;    //计算推算出的地址与dataflash的起始地址差值
        }

        uFirstReadFlash = DATAFLASH_RECORD_END-uOver-(DATAFLASH_RECORD_END-DATAFLASH_RECORD_HEAD)%uFrameLenth;
        uTimes = uOver/uFrameLenth;
        iRecord = CheckTarget(uFirstReadFlash,targetTime,uFrameLenth,uTimes,uNeedReadFlash);
        if(NO_DATA == iRecord)
        {
            uFirstReadFlash = DATAFLASH_RECORD_HEAD;
            uTimes = (sRecord.uFlashAddr-DATAFLASH_RECORD_HEAD)/uFrameLenth;
            iRecord = CheckTarget(uFirstReadFlash,targetTime,uFrameLenth,uTimes,uNeedReadFlash);
        }
    }
    return iRecord;
}
