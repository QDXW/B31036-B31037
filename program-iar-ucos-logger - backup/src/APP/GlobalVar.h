#ifndef __GLOBAL_VAR_H__
#define __GLOBAL_VAR_H__

#include <stdint.h>
#include <stdio.h>
#include "tool.h"
#include "Update.h"
#include "log.h"

//----------------------------------------------------------
#define DEBUG_PRINT  (1)    // DEBUG模式，可以串口0打印输出信息

#if (1==DEBUG_PRINT)
#define DEBUGOUT(...) printf(__VA_ARGS__)

#else
#define DEBUGOUT(...) ;
#endif
//==============================================================================
// 定义数采型号
#define  Logger_C        (1)
#define  Logger_CP       (2)

#define  LOGGER_TYPE    Logger_CP
//==============================================================================

//==============================================================================
#define MAX_device      (10)  // 最多南向设备数量
#define USE_485_DEBUG   (1)  // 1：使用南向485口作为调试串口；0：南向485口仅作为南向通讯串口
//==============================================================================
// 数据在EEPROM中的存储地址
#define  EEP_LOGGER_INF_HEAD          0x00000044   // 数采本机存储起始地址   eeprom起始64字节不能存储
#define  EEP_LOGGER_DEVICE_ESN_HEAD   0x00000100   // 设备ESN存储起始地址
#define  EEP_LOGGER_DEVICE_INF_HEAD   0x00000300   // 设备信息和点表信息存储起始地址
#define  EEP_LOGGER_UPDATA_HEAD       0x00000500   // 数采本机升级标识存储起始地址
#define  EEP_LOGGER_104_RECORD_HEAD   0x00000550   // IEC104历史数据最后一条DataFlash存储地址
#define  EEP_LOGGER_LOG_LADDR_HEAD    0x00000560   // 日志最后一条存储地址
#define  EEP_LOGGER_LOG_HEAD          0x00000570   // 日志存储起始地址
//#define  EEP_TABLE_INFO_HEAD          0x00000580   // 预置点表信息点表编码
#define  EEP_DT1000_UPDATA_HEAD       0x00000580   // 表计升级标识存储起始地址

#define  EEP_LOGGER_DEVICE_SOFT_HEAD  0x00000600    //设备软件版本号存储起始地址
#define  EEP_ALARM_HEAD               0x00000800   // 2KB起始点，存储告警信息
#define  EEP_DT1000UPDATA_SN          0x00000E00   //存储成功的序号--4个字节
#define  EEP_DT1000UPDATALEN_SN       0x00000E10   //存储成功数据长度---8个字节

//------------------------------------------------------------------------------
// 数据在DataFlash中的存储地址
#define  DATAFLASH_UPDATA_HEAD        0x000000     // 升级程序存放在DataFlash的起始地址  占用96KB
#define  DATAFLASH_POINT_HEAD         0x018000     // 点表存放在DataFlash的起始地址      占用8KB

#define  DATAFLASH_LOG_ONE            0x01A000     // 南向日志第一天存放起始地址
#define  DATAFLASH_LOG_TWO            0x01C000     // 南向日志第一天存放起始地址
#define  DATAFLASH_LOG_THREE          0x01E000     // 南向日志第一天存放起始地址
#define  DATAFLASH_LOG_END            0x020000     // 南向日志存放的结束地址
#define  DATAFLASH_LOGGER_INFO        0x022000     // 数采基本信息备份
#define  DATAFLASH_DT1000_LOG         0x023000     //存放表计日志
#define  DATAFLASH_DT1000_UPDATA_HEAD 0x050000     //表计升级程序存放在falsh的起始地址占用128KB，到0x070000为A面,0x070000到0x090000为B面
#define  DATAFLASH_RECORD_HEAD        0x11A000     // 0x01E000历史数据存放在DataFlash的起始地址
#define  DATAFLASH_RECORD_END         0x7FFFFF     // 历史数据存放在DataFlash的结束地址
//==============================================================================
// 读EEP数据类型
#define EEP_LOGGER_INF     1    // 读取数采基本信息
#define EEP_DEVICE_SOUTH   2    // 读取南向设备信息
#define EEP_DEVICE_ESN     3    // 读取南向设备ESN
#define EEP_UPDATA         4    // 读取升级信息
#define EEP_ALARM          5    // 读取南向告警信息
#define EEP_DEVICE_SOFT    6    // 读取南向设备软件信息
//#define EEP_TABLE_SEQ      7    // 读取预置点表信息点表编码
#define EEP_DT1000_UPDATA  7   //读取表计升级信息

//==============================================================================
//==============================================================================
/*******************************************************************************
*数采本机信息
*需要存储
*占用空间：88字节
*存储EEPROM起始地址：0x00000050~0x000000AF
*******************************************************************************/
typedef struct
{
    uint16_t  inquire_interval_time;   // 查询间隔时间
    char      esn[20];                 // 数采ESN号
    char      name[20];                // 数采名称
    char      model[20];               // 数采型号
    char      type[20];                // 数采类型
    uint16_t  server_port;             // 服务器端口
    char      server_domain[34];       // 服务器域名 域名最多30字节，"域名"然后\0一共53字节

    uint8_t   ADDR;                    // 设备地址
    char      phonenum[11];            // 手机号码
    uint8_t   IP[4];                   // IP地址
    //uint8_t   reserve;                 // 占位

    uint16_t  CRC;                     // 存储数据的CRC校验
}LOGGER_INF_T;
//==============================================================================

/*******************************************************************************
*设备ESN号信息
*需要存储
*占用空间：304字节
*存储EEPROM起始地址：0x000000B0~0x000001DF
*******************************************************************************/
typedef struct
{
    char  cDeviceEsn[MAX_device][20];       // 南向设备
	uint8_t uEsnMark[MAX_device+1];			 //分配地址与未分配地址的表计

    //uint16_t  reserve;                 // 预留
    uint16_t  CRC;                     // 存储数据的CRC校验
}LOGGER_DEVICE_ESN_T;

/*******************************************************************************
*设备设备软件号信息
*需要存储
*占用空间：304字节
*存储EEPROM起始地址：0x000000B0~0x000001DF
*******************************************************************************/
typedef struct
{
    uint8_t  cDeviceSoft[MAX_device][17];       // Cplus为3台南向设备
    uint8_t  uSoftMark[MAX_device];
    //uint16_t  reserve;                 // 预留
    uint16_t  CRC;                     // 存储数据的CRC校验
}LOGGER_DEVICE_SOFT_T;

/*******************************************************************************
*设备信息
*不需要存储
*占用空间：20字节
*******************************************************************************/
#define  BIG_ENDIAN      0
#define  LITTLE_ENDIAN   1
typedef struct
{
    uint8_t   addr;               // 设备地址
    uint8_t   rel_num;            // 相对点表号--相对存储到点表信息的
    uint16_t  protocol_num;       // 绝对点表号--管理系统下发的点表号
    uint8_t   baud_rate;          // 通讯波特率：1:2400；2:4800；3:9600；4:19200；5:38400；6:115200
    uint8_t   protocol_type;      // 协议类型,1:华为MODBUS；2：标准MODBUS
    uint8_t   big_little_endian;  // 数据大小端
    uint8_t   reserve;            // 预留占位
    uint16_t  yx_start_addr;      // 遥信起始地址
    uint16_t  yc_start_addr;      // 遥测起始地址
    uint16_t  yk_start_addr;      // 遥控起始地址
    uint16_t  sd_start_addr;      // 设点起始地址
    uint16_t  dd_start_addr;      // 电度起始地址
}LOGGER_DEVICE_INF_T;

/*******************************************************************************
*点表信息汇总。
*不需要存储
*占用空间：12字节
*******************************************************************************/
typedef struct
{
    uint16_t  protocol_num;       // 点表号
    uint8_t   protocol_type;      // 协议类型
    uint8_t   alarm_sum;          // 告警点总数
    uint16_t  reserve2;           // 预留
    uint16_t  mess_point_sum;     // 信息点总数

    uint32_t  flash_addr;         // 点表Dataflash存储地址
}LOGGER_PROTOCOL_INF_T;

/*******************************************************************************
*设备信息
*需要存储
*占用空间：484字节
*存储EEPROM起始地址：0x000001E0~0x000003C3
*******************************************************************************/
typedef struct
{
    LOGGER_DEVICE_INF_T      device_inf[MAX_device]; // 南向设备
    LOGGER_PROTOCOL_INF_T    protocol[MAX_device];   // 南向设备

    uint8_t   reserve;           // 占位
    uint8_t   device_sum;              // 设备总数
    uint16_t  yx_sum;                  // 遥信总数
    uint16_t  yc_sum;                  // 遥测总数
    uint16_t  yk_sum;                  // 遥控总数
    uint16_t  sd_sum;                  // 设点总数
    uint16_t  dd_sum;                  // 电度总数

    uint16_t  reserve2;           // 占位
    uint16_t  CRC;                // 存储数据的CRC校验
}LOGGER_DEVICE_PROTOCOL_T;

//==============================================================================
//==============================================================================
/*******************************************************************************
*点表单点信息
*不需要存储
*占用空间：4字节
*******************************************************************************/
typedef union
{
    uint8_t type_type;
    struct
    {
        uint8_t  mess:4;  // 信息点类型 遥控，遥测，遥信，设点，电度
        uint8_t  data:4;  // 数据类型
    }type;
}LOG_MESS_TYPE_T;
typedef struct
{
    uint16_t         reg_addr;             // MODBUS寄存器地址
    LOG_MESS_TYPE_T  reg_type;             // 信息点类型 遥控，遥测，遥信，设点，电度
    uint8_t          reg_count;            // 数据长度
}LOGGER_MODBUS_REG_T;

//==============================================================================
//==============================================================================
/*******************************************************************************
*IEC104点表信息
*不需要存储
*占用空间：4字节
*******************************************************************************/
// 遥信
typedef struct
{
    uint16_t data;
    //uint16_t  reserve;
}IEC104_YX;

// 遥测
typedef struct
{
    Float_Uint_Convert data;
}IEC104_YC;

// 遥控
typedef struct
{
    uint16_t data;
    //uint16_t  reserve;
}IEC104_YK;

// 设点
typedef struct
{
    uint16_t data;
    //uint16_t  reserve;
}IEC104_SD;

// 电度
typedef struct
{
    Float_Uint_Convert data;
}IEC104_DD;

//==============================================================================
// 用于南向设备告警
typedef struct
{
    uint16_t     mdbus_addr;          // 告警MODBUS地址
    uint16_t     alarm_value;         // 告警值
}SOUTH_ALARM_T;

typedef struct
{
    uint16_t    dev_lost;        // 南向设备断线，一位代表一台设备
    uint16_t    log_alarm;       // 数采本机异常，一位代表一个异常状态
    //uint16_t    dev_alarm_sum;   // 南向告警点总数
}LOGGER_ALARM_T;
//==============================================================================
//==============================================================================

// 运行状态
enum  LOGGER_RUNNING_STATE
{
    RUNNING_EMPTY          = 0,    // 新，未开站
    //RUNNING_SEARCH_PREPARE = 1,    // 准备搜索华为设备
    RUNNING_SEARCH_HW      = 1,    // 搜索华为设备
    RUNNING_SEARCH_END     = 2,    // 搜索华为设备结束

    RUNNING_INPUT_START    = 3,    // 启动导表C5
    RUNNING_INPUT_SOUTH    = 4,    // 导入下联设备信息C4 92
    RUNNING_INPUT_GOLB     = 5,    // 导入全局信息BB 88
    RUNNING_INPUT_TABLE    = 6,    // 导入点表BB 89
    RUNNING_INPUT_104      = 7,    // 导入设备104表信息BB 8A
    //RUNNING_INPUT_INFO     = 4,    // 导入信息过程中，包括全局信息，设备信息，点表信息等，在导入信息过程中，南向查询暂停

    RUNNING_WORK_READ      = 10,   // 已经开站完成，正常工作
};//LOGGER_RUNNING_STATE_E;

// 通讯模块连接服务器状态
enum NORTH_RUNNING_STATE
{
    NORTH_DISCON    = 0,   // 北向断联
    NORTH_CMERR     = 1,   // 启动发送数据没有收到模块返回>
    NORTH_RDY       = 2,   // 模块准备好
    NORTH_CONNECT   = 3,   // 模块连接中
    NORTH_POWERDOWN = 4,   // 模块关机
    NORTH_OK        = 5,   // 连接到服务器并已经上报数采信息，完成对时
};

//------------------------------------------------
// 运行故障
enum LOGGER_RUNNING_ERR
{
    err_power_off = 0x0001,   // 断电
};//LOGGER_RUNNING_ERR_E;


//==============================================================================
// 运行状态，不需要存储
typedef struct
{
    uint8_t  run_status;      // 工作状态
    uint8_t  update;          // 切换到升级模式，0x0A：近端升级；0xA0：远程升级
    uint16_t err;             // 故障编码存储
    uint16_t err_lost;        // 南向设备丢失
    uint16_t uDevExist;       // 南向有效设备标记，一位一个
    uint8_t  IP[4];           // 数采IP地址
    char     IMSI[15];        // 国际移动台设备表示（IMSI）
    uint8_t  north_status;    // 北向连接状态
    uint8_t  uGPS;            // GPS状态，1:搜索到GPS;0:没有搜索到GPS
    uint8_t  uCom;            // 南向通讯状态，bit：2发送数据，bit:1收到数据，bit3：延时中，
    uint16_t uCMType;        // 通信模块类型
    uint8_t  uCSQ;           // 无线功率
    //uint8_t  uDt1000_update;  //表计升级状态
    uint8_t  uFileIO_status;  // 南向文件导入导出状态

}LOGGER_RUNNING_T;
//==============================================================================
typedef struct
{
    uint8_t   updata_mark;      // 升级标志0xAA升级；0xBB回滚；0x55无
    uint8_t   version[17];      // 程序版本
    uint8_t   uDevAddr;        //设备地址
    uint8_t   uData;           // 日期
    uint32_t  nDataLen;
    uint16_t  frame_sum;        // 升级总帧数，256字节一帧
    uint16_t  CRC;              // 数据CRC校验
}DT1000UPDATA_MARK_T;

//==============================================================================
//==============================================================================
//==============================================================================
// 运行数据，不存储
extern LOGGER_RUNNING_T          g_LoggerRun;    // 数采运行参数
//-----------------------------------------------------------------
// 参数数据，存储于内部eeprom
extern LOGGER_DEVICE_PROTOCOL_T  g_DeviceSouth;  // 南向设备和点表信息
extern LOGGER_INF_T              g_LoggerInfo;    // 数采信息
extern LOGGER_DEVICE_ESN_T       g_DeviceEsn;    // 南向设备ESN
extern LOGGER_DEVICE_SOFT_T      g_DeviceSoft;    // 南向设备软件版本号



// 参数数据，存储于外部DataFlash
extern LOGGER_MODBUS_REG_T       *g_pRegPoint[MAX_device]; // 点表信息

// 南向告警信息，暂定存储于内部eeprom
extern SOUTH_ALARM_T             *g_psSouthAlarm[MAX_device];
extern SOUTH_ALARM_T             *g_psSouthAlarmCopy[MAX_device];
extern LOGGER_ALARM_T            g_LoggerAlarm;

// 升级信息，存储于每部eeprom
extern UPDATA_MARK_T             g_LoggerUpdate;
extern DT1000UPDATA_MARK_T        g_DT1000Updata;

//extern uint16_t g_PerSetTableResq;             //点表编码
extern uint8_t SouthSwich;
extern uint8_t SdContinuousAddr;
extern uint32_t g_South_Action_Newtime;
extern uint32_t gImport_Table_time;

//==============================================================================
extern uint8_t GetVerType(void);
extern uint8_t GetVerS1(void);
extern uint8_t GetVerS2(void);
extern uint16_t GetVerS3(void);
//extern void PrintThisInfo(void);
extern void SetLoggerDefInfo(void);
extern uint16_t ReadEepData(uint8_t t);
extern uint16_t SaveEepData(uint8_t t);
extern void AllReset(uint8_t uSave);
extern void Reboot(void);

#endif
