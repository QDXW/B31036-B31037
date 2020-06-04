#ifndef __IEC104_H_
#define __IEC104_H_

#include "ucos_ii.h"

#include <stdint.h>
//=================================================================

#define TaskIec104ProcessStkSize   256   // 查询任务堆栈大小
extern OS_STK TaskIec104ProcessStk[TaskIec104ProcessStkSize];	      // 定义任务堆栈大小

//=================================================================
//--------------------------------------
// 监视方向的过程信息
#define  M_SP_NA_1         0x01  //1    // 单点信息
#define  M_DP_NA_1         0x03  //3    // 双点信息
#define  M_ME_NA_1         0x09  //9    // 测量值，归一化值
#define  M_ME_NC_1         0x0D  //13   // 测量值，短浮点数
#define  M_IT_NA_1         0x0F  //15   // 累计值
#define  M_SP_TB_1         0x1E  //30   // 带时标CP56Time2a的单点信息
#define  M_DP_TB_1         0x1F  //31   // 带时标CP56Time2a的双点信息
#define  M_ME_TF_1         0x24  //36   // 带时标CP56Time2a的测量值，短浮点数
#define  M_IT_TB_1         0x25  //37   // 带时标CP56Time2a的累计量
//-----------------------------------------
// 控制方向的过程信息
#define  C_SC_NA_1         0x2D  //45   // 单点信息
#define  C_DC_NA_1         0x2E  //46   // 双点信息
#define  C_SE_NA_1         0x30  //48   // 设定命令，归一化值
#define  C_SE_NB_1         0x31  //49   // 设定命令，标度化值
#define  C_SE_NC_1         0x32  //50   // 设定命令，短浮点数
//------------------------------------------
// 控制方向的系统信息
#define  C_IC_NA_1         0x64  //100  // 总召唤命令
#define  C_CI_NA_1         0x65  //101  // 电度总召唤命令
#define  C_CS_NA_1         0x67  //103  // 时钟同步命令

//#define  P_DT1000_UPDATA   0xF0  //179   //表计升级
#define  P_FILE_INOUT      0xF0  //179   //文件导入导出

#define  P_UPDATA          0xB4  //180  // 升级
#define  P_SET_IP          0xB5  //181  // 设备IP配置
#define  P_MODBUS_ENDIAN   0xB6  //182  // MODBUS大小端配置
#define  P_MODBUS_BAUD     0xB7  //183  // MODBUS波特率配置
#define  P_ADDR            0xB8  //184  // 设备公共地址配置
#define  P_VERSION         0xB9  //185  // 版本查询
#define  P_ROLLBAOCK       0xBA  //186  // 版本回滚
#define  P_INPUT_TABLE     0xBB  //187  // 点表导入
#define  P_REPORT_NUM      0xBC  //188  // 上报号码配置
#define  P_INIT_FINISH     0xBD  //189  // 设备初始化是否完成
#define  P_TIME_COLLECT    0xBE  //190  // 带时标总召唤
#define  P_ERR_PROCESS     0xBF  //191  // 异常处理
#define  P_DEV_INFO        0xC0  //192  // 设备基本信息
#define  P_STATION_INFO    0xC1  //193  // 站点基本信息
#define  P_SERVICE_INFO    0xC2  //194  // 平台基本信息
#define  P_CREATE_STATION  0xC3  //195  // 建站指令
#define  P_SOUTH_INFO      0xC4  //196  // 下联设备信息
#define  P_TABLE           0xC5  //197  // 导表指令
#define  P_COM_MODE        0xC6  //198  // 通信方式
#define  P_HW_INFO         0xC7  //199  // 华为设备信息
#define  P_SERVICE_IP      0xC8  //200  // 服务器IP/域名
#define  P_SERVICE_PORT    0xC9  //201  // 服务器端口
#define  P_LOG             0xCA  //202  // 日志处理
#define  P_LOCATION        0xCB  //203  // 经纬度
#define  P_SOUTH_STATUS    0xCC  //204  // 下联设备状态
#define  P_CL_ESN          0xD0  //208  // 数采ESN配置      CL=CLEAN LOGGER
#define  P_CL_TYPE         0xD1  //209  // 数采类型配置
#define  P_CL_MODEL        0xD2  //210  // 数采型号配置
#define  P_CL_NAME         0xD3  //211  // 数采设备名称配置
#define  P_HW_ESN          0xD4  //212  // 下联华为设备ESN更新
#define  P_HW_DEL          0xD5  //213  // 下联华为设备删除
#define  P_SIM_ID          0xD6  //214  // SIM卡ID上报
#define  P_WL_QUALITY      0xD7  //215  // 无线功率上报
#define  P_MAX_DEVICE      0xD8  //216  // 下联设备最大值上报
#define  p_SD              0xD9  //平台设点
#define  P_CL_RESET        0xFA  //250  // 恢复出厂设置
#define  P_CL_REBOOT       0xFB  //251  // 设备复位   ---1.1.7 新增 ---刘京
//------------------------------------------
//子功能码subfunction
#define  S_FILE_IMPORT   0x01
#define  S_FILE_EXPORT   0x33


//------------------------------------------
// 传输原因
#define   R_ROUND              0x01   // 周期、循环
#define   R_BACK_SCAN          0x02   // 背景扫描
#define   R_BURST              0x03   // 突发、自发上传
#define   R_INIT               0x04   // 初始化
#define   R_ASK                0x05   // 请求或被请求
#define   R_ACTIVE             0x06   // 激活
#define   R_ACTIVE_ACK         0x07   // 激活确认
#define   R_ACTIVE_STOP        0x08   // 停止激活
#define   R_ACTIVE_STOP_ACK    0x09   // 停止激活确认
#define   R_ACTIVE_END         0x0A   // 激活结束
#define   R_COLLECT_ACK        0x14   // 响应总召唤

#define   R_TRANS_START        0x80   // 开始传输
#define   R_TRANS_START_ACK    0x81   // 开始传输确认
#define   R_DATA_TRANS         0x82   // 数据传输
#define   R_DATA_RETRY         0x83   // 数据重发
#define   R_TRANS_STOP         0x84   // 停止传输
#define   R_TRANS_STOP_ACK     0x85   // 停止传输确认
#define   R_TRANS_FINISH       0x86   // 传输完成
#define   R_TRANS_FINISH_ACK   0x87   // 传输完成确认
#define   R_INPUT_GLO_INFO     0x88   // 导入全局信息
#define   R_INPUT_TYPE_INFO    0x89   // 导入设备类型信息
#define   R_INPUT_DEV_INFO     0x8A   // 导入设备信息
#define   R_STATION_START      0x8B   // 建站启动
#define   R_TABLE_START        0x8C   // 导表启动
#define   R_STOP               0x8D   // 停止
#define   R_STOP_ACK           0x8E   // 停止确认
#define   R_RECOLLECT_END      0x8F   // 补采结束
#define   R_INQUIRE            0x90   // 查询
#define   R_INQUIRE_SUC        0x91   // 查询成功
#define   R_SETTING            0x92   // 设置
#define   R_SET_SUC            0x93   // 设置成功
#define   R_INFO_REPORT        0x94   // 信息上报
#define   R_INFO_REPORT_END    0x95   // 上报结束
//#define   0x96   //
//#define   0x97   //
#define   R_INPUT_GLO_ACK      0x98   // 导入全局信息确认
#define   R_INPUT_TYPE_ACK     0x99   // 导入设备类型信息确认
#define   R_INPUT_DEV_ACK      0x9A   // 导入设备信息确认
#define   R_INTERCHANGE_ERR    0xEE   // 交互异常

//==================================================
//==================================================
// IEC104帧起始为0x68
#define IEC104_HEAD			    0x68
// U帧，S帧
#define IEC104_U_MAK			0x03
#define IEC104_U_STARTDT		0x04
#define IEC104_U_STARTDT_ACK	0x08
#define IEC104_U_STOPDT		    0x10
#define IEC104_U_STOPDT_ACK	    0x20
#define IEC104_U_TESTFR		    0x40
#define IEC104_U_TESTFR_ACK	    0x80
#define IEC104_S_MAK			0x01

//==================================================
//==================================================
// 信息点类型
#define TYPE_YX      0x02    // 遥信
#define TYPE_YC      0x01    // 遥测
#define TYPE_YK      0x04    // 遥控
#define TYPE_SD      0x07    // 设点
#define TYPE_DD      0x03    // 电度
#define TYPE_GJ      0x09    // 告警

#define T_UINT16     0x01    // 无符号16位数据
#define T_STRING     0x02    // 字符串
#define T_UINT32     0x03    // 无符号32位数据
#define T_INT16      0x04    // 有符号16位数据
#define T_INT32      0x05    // 有符号32位数据
#define T_NULLDATA   0x06    // 空值
#define T_EPOCHTIME  0x07    // 时间类型
#define T_BIT        0x08    // 为位类型
#define T_FLOAT      0x09    // 为浮点类型
//==================================================
//==================================================
typedef union
{
    struct
    {
        uint8_t start;   		// 启动符
        uint8_t len;     		// 长度
        uint8_t SseqL;   		// 发送序列低位
        uint8_t SseqH;   		// 发送序列高位
        uint8_t RseqL;   		// 接收序列低位
        uint8_t RseqH;   		// 接收序列高位
        uint8_t type;    		// 类型标识
        uint8_t limit;   		// 可变结构限定词
        uint8_t reasonL; 		// 传输原因低位
        uint8_t reasonH; 		// 传输原因高位
        uint8_t addrL;   		// 公共地址低位
        uint8_t addrH;   		// 公共地址高位
        uint8_t maddrL;  		// 信息体地址低位
        uint8_t maddrM;  		// 信息体地址中位
        uint8_t maddrH;  		// 信息体地址高位
        uint8_t data[240]; 		// 数据
    }format;
    uint8_t buff[255];
}IEC_FORMAT_T;

typedef struct
{
    uint8_t      FrameCommand;    // 帧格式 I帧，U帧，S帧
    uint8_t      SendBytes;       // 发送字节数
    IEC_FORMAT_T send;            // uint8_t   send_buff[255];  // 发送的数据数组
    IEC_FORMAT_T recv;            // uint8_t   rec_buff[255];   // 接收的数据数组
}IEC104_MAIN_T;
//========================================================================
// 统计各个点表的遥信，遥测总数
typedef struct
{
    uint16_t uYxSum;
    uint16_t uYcSum;
    uint16_t uYkSum;
    uint16_t uSdSum;
}IEC104_COUNT_T;
//========================================================================
extern IEC104_COUNT_T g_sIecPointCount[];  // 点表信息点统计
//========================================================================
//extern uint8_t  IEC104_DATA_YX[];   // IEC104数据-遥信指针，动态申请空间
//extern float    IEC104_DATA_YC[];   // IEC104数据-遥测指针，动态申请空间

extern uint8_t  *IEC104_DATA_YX;   // IEC104数据-遥信指针，动态申请空间
extern uint32_t *IEC104_DATA_YC;   // IEC104数据-遥测指针，动态申请空间
extern uint8_t  *IEC104_DATA_YK;   // IEC104数据-遥控指针，动态申请空间
extern uint32_t *IEC104_DATA_SD;   // IEC104数据-设点指针，动态申请空间
extern uint32_t *IEC104_DATA_DD;   // IEC104数据-电度指针，动态申请空间
//extern static uint8_t  s_uSouthReadSd; //南向读设点
extern IEC104_MAIN_T g_sIEC;
extern uint16_t      g_IecRecSeq;          // 接收序列
extern uint16_t      g_IecSendSeq;         // 发送序列
//==================================================
//以下用于总召
#define COLLECT_NULL           0   // 无总召
#define COLLECT_START          1   // 总召开始
#define COLLECT_YX             2   // 发送遥信数据
#define COLLECT_YC             3   // 发送遥测数据
#define COLLECT_END            9   // 总召结束
#define COLLECT_DD_START       10  // 电度总召
#define COLLECT_DD_END         11  // 电度总召结束

//以下用于补采
#define SUBCOLLECT_NULL        0   // 无补采

#define SUBCOLLECT_START       21  // 补采开始
#define SUBCOLLECT_YX          22  // 发送补采遥信数据
#define SUBCOLLECT_YC          23  // 发送遥测数据
#define SUBCOLLECT_END         25  // 结束补采总召唤
#define SUBCOLLECT_CHECK	   26  // 平台查询是否还存在历史数据
#define SUBCOLLECT_END_CONFIRM 27  // 补采结束


#define NORTH_CMD_READSD  (5)


typedef struct
{
    uint8_t running;     // IEC104状态
    uint8_t collect;     // 总召状态
    uint8_t dd_collect;  // 电度总召
    uint8_t uRelAddr;    // 在上报总召数据的相对设备地址
    uint8_t subcollect;  // 补采状态
    uint8_t logext;
} IEC_RUNNING_T;

extern IEC_RUNNING_T g_sIecRun;
extern IEC_RUNNING_T iec_subcollect_run;
//====================================================================================================
//====================================================================================================
//====================================================================================================
extern void TaskIec104Process(void *p);
//extern void Iec104CreateFrameI(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC104_MAIN_T *pA);
extern void IecCreateFrameI(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC_FORMAT_T *pA);
extern void IecReportLogInfo(uint8_t uReason);

//====================================================================================================
//====================================================================================================
//====================================================================================================

//======================================================================
extern void ReportCtrlClear(uint8_t uCtrl);
// 设置上报控制
extern void ReportCtrlSet(uint8_t uCtrl);
// 读上报控制
extern uint8_t ReportCtrlRead(uint8_t uCtrl);
#define REPORT_HW_DEVICE       0x01
#define REPORT_OTHERS          0x02
#define REPORT_HW_SOFT         0X04
//extern uint8_t  g_uNextFrame; // 全局变量，在IEC104.c中标记。
//extern uint8_t uPdateMark;     //升级标志，用于复位后判断是否继续
//======================================================================

#endif
