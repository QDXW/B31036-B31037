#ifndef __SUB_COLLECTION_H_
#define __SUB_COLLECTION_H_
#include <stdint.h>
#include "IEC104.h"
#include <time.h>

#define SUBCHECKSIZE 8	        //检测时标时从flash中读取的长度
#define YXBUFFSIZE 67	        //用于从历史数据中读取需要上报的补采遥信数据
#define YCBUFFSIZE 46*4+8	        //用于从历史数据中读取需要上报的补采遥测数据
#define INTERVAL_TIME_RANG 150 	//平台指定时间戳前后两分半分钟内
#define NO_DATA -2              //遍历历史数据查找不到需要补采时标前后2.5分钟范围内的数据


extern int32_t tMarkCheck(uint8_t menAddr[],time_t targetTime );

extern int32_t hMarkCheck(uint8_t array[]);

extern int32_t CollectData(IEC104_MAIN_T *pA, uint32_t *uNeedReadFlash);





#endif
