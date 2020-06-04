/**
 *���ļ�����ȫ�ֱ���
 *
 */

#include "chip.h"
#include "GlobalVar.h"
#include "InEeprom.h"

#include "string.h"
#include "RealTime.h"
#include "Update.h"
#include "Memory.h"
#include "IEC104.h"
#include "WatchDog.h"

#include "Record.h"
//==============================================================================
/*
AB.CD.EF

A����ʾ�����汾���ͣ�A-��ʽ�����棬B-���԰�
B��Ӳ���汾�ţ�1-����A�� 2-����B�� 3-�������� 4-��Դ����
CD��������汾��00-FF
EF������С�汾��00-FF
*/
// ���ɰ汾��
#define G_VER_TYPE  0xC3    // ���԰� + ����B�汾
#define G_VER_S1    0x10    // ������汾
#define G_VER_S2    0x24    // ����С�汾  С�汾�����λ   ���λ1:��A�棻0:��B��
#define G_VER_S3    0x0001  // �����ڲ��汾
//==============================================================================
// ����Ĭ���豸��Ϣ
#define DEFAULT_inquire      300                    // ��ѯ������ʱ�䣬5����=300�� = 30000��10ms
#define DEFAULT_MODEL        "C8000"                // Ĭ���ͺ�
#define DEFAULT_NAME         "icleanlogger"         // Ĭ������
#define DEFAULT_TYPE         "iCleanLogger-c1"      // Ĭ������
#define DEFAULT_ESN          "CLB1705HZ010001RB"    // ����ESN��
#define DEFAULT_port          12434                 // Ĭ�����Ӷ˿�
#define DEFAULT_domain        "\"221.12.173.94\""   // Ĭ�Ϸ���������
#define DEFAULT_phonenum      "15600005678"         // ����Ĭ���ϱ�����
#define DEFAULT_ADDR          1                     // Ĭ���豸��ַ
#define DEFAULT_device_sum    0                     // Ĭ���豸����
#define DEFAULT_yx_sum        0                     // Ĭ��ң�ŵ�����
#define DEFAULT_yc_sum        0                     // Ĭ��ң�������
#define DEFAULT_yk_sum        0                     // Ĭ��ң�ص�����
#define DEFAULT_sd_sum        0                     // Ĭ����������
#define DEFAULT_dd_sum        0                     // Ĭ�ϵ�ȵ�����
//==============================================================================
LOGGER_RUNNING_T           g_LoggerRun={0};
LOGGER_INF_T               g_LoggerInfo={0};
LOGGER_DEVICE_PROTOCOL_T   g_DeviceSouth={0};
LOGGER_DEVICE_ESN_T        g_DeviceEsn={0};

LOGGER_DEVICE_SOFT_T       g_DeviceSoft={0};
UPDATA_MARK_T              g_LoggerUpdate={0};
DT1000UPDATA_MARK_T        g_DT1000Updata={0};


LOGGER_MODBUS_REG_T        *g_pRegPoint[MAX_device]={0,0,0,0,0,0,0,0,0,0};

SOUTH_ALARM_T              *g_psSouthAlarm[MAX_device]={0,0,0,0,0,0,0,0,0,0};
SOUTH_ALARM_T              *g_psSouthAlarmCopy[MAX_device]={0,0,0,0,0,0,0,0,0,0};
LOGGER_ALARM_T             g_LoggerAlarm={0};

//uint16_t g_PerSetTableResq = 0;             //�������
uint8_t SouthSwich=0;
uint8_t SdContinuousAddr = 0;     //ң������ַ�Ƿ�������


//==============================================================================
// ��ȡ�汾��
uint8_t GetVerType(void)
{
    return G_VER_TYPE;
}
uint8_t GetVerS1(void)
{
    return G_VER_S1;
}
uint8_t GetVerS2(void)
{
    return G_VER_S2;
}
uint16_t GetVerS3(void)
{
    return G_VER_S3;
}

//==============================================================================
//==============================================================================
/****************************************************************************
* ��    �ƣ�SetDefInfo()
* ��    �ܣ���ʼ��ΪĬ�����ݡ�
* ��ڲ�������
* ���ڲ������洢�ɹ����ش洢�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
void SetLoggerDefInfo(void)
{
    memset(g_LoggerInfo.esn,0,20);
    strncpy(g_LoggerInfo.esn,DEFAULT_ESN,17);      // ESN��
    strncpy(g_LoggerInfo.name,DEFAULT_NAME,20);    // ���ƣ�ʹ��ESN����ΪĬ������
    strncpy(g_LoggerInfo.model,DEFAULT_MODEL,20);  // �ͺ�
    strncpy(g_LoggerInfo.type,DEFAULT_TYPE,20);    // ����

    g_LoggerInfo.server_port = DEFAULT_port;   // �������˿�

    strncpy(g_LoggerInfo.phonenum,DEFAULT_phonenum,11);    // �ϱ��ֻ�����

    memset(g_LoggerInfo.server_domain,0,30);
    strcpy(g_LoggerInfo.server_domain,DEFAULT_domain); // ����������  strncpy(g_LoggerInfo.server_domain,DEFAULT_domain,17); // ����������
    g_LoggerInfo.ADDR = DEFAULT_ADDR;// ���ɵ�ַ

    g_LoggerInfo.IP[0] = 0;
    g_LoggerInfo.IP[1] = 0;
    g_LoggerInfo.IP[2] = 0;
    g_LoggerInfo.IP[3] = 0;
    g_LoggerInfo.inquire_interval_time = DEFAULT_inquire;  //��ѯ������ʱ�䣬5����=30000��10����
}
/****************************************************************************
* ��    �ƣ�Save_eep_data()
* ��    �ܣ��洢�ڲ�EEPROM���ݡ�
* ��ڲ�����
*           t         ���洢���������� 1��Logger_inf��2��Device_south��3��Device_esn��4��Logger_updata

* ���ڲ������洢�ɹ����ش洢�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint16_t ReadEepData(uint8_t t)
{
    uint16_t result;
    uint8_t  i;
    uint8_t  space;
    uint32_t eep_addr;
    switch(t)
    {
    case EEP_LOGGER_INF:
        result = EepReadData(EEP_LOGGER_INF_HEAD,(uint8_t *)&g_LoggerInfo,sizeof(g_LoggerInfo),&g_LoggerInfo.CRC);// ���ɶ�ȡ
        break;

    case EEP_DEVICE_SOUTH:
        for(i=0;i<MAX_device;i++)
        {
            g_DeviceSouth.device_inf[i].addr = 0;
        }
        result = EepReadData(EEP_LOGGER_DEVICE_INF_HEAD,(uint8_t *)&g_DeviceSouth,sizeof(g_DeviceSouth),&g_DeviceSouth.CRC);// �豸��Ϣ�͵����Ϣ��ȡ
        break;

    case EEP_DEVICE_ESN:
        result = EepReadData(EEP_LOGGER_DEVICE_ESN_HEAD,(uint8_t *)&g_DeviceEsn,sizeof(LOGGER_DEVICE_ESN_T),&g_DeviceEsn.CRC);// �豸ESN��Ϣ��ȡ
        break;
    case EEP_DEVICE_SOFT:
        result = EepReadData(EEP_LOGGER_DEVICE_SOFT_HEAD,(uint8_t *)&g_DeviceSoft,sizeof(LOGGER_DEVICE_SOFT_T),&g_DeviceSoft.CRC);// �豸ESN��Ϣ�洢
        break;

    case EEP_UPDATA:
        result = EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&g_LoggerUpdate,sizeof(UPDATA_MARK_T),&g_LoggerUpdate.CRC);// ������Ϣ��ȡ
        break;

    case EEP_ALARM:
        result = EepReadData(EEP_ALARM_HEAD,(uint8_t *)&g_LoggerAlarm.dev_lost,sizeof(g_LoggerAlarm),NULL);

        g_LoggerRun.err      = g_LoggerAlarm.log_alarm;
        g_LoggerRun.err_lost = g_LoggerAlarm.dev_lost;

        eep_addr = EEP_ALARM_HEAD + sizeof(g_LoggerAlarm);
        for(i=0; i<MAX_device&&i<g_DeviceSouth.device_sum; i++)
        {
            space = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[i].rel_num].alarm_sum * sizeof(SOUTH_ALARM_T);
            if(space)
            {
                /*if(NULL!=g_psSouthAlarm[i])
                {
                    g_psSouthAlarm[i] = WMemFree(g_psSouthAlarm[i]);
                }
                if(NULL!=g_psSouthAlarmCopy[i])
                {
                    g_psSouthAlarmCopy[i] = WMemFree(g_psSouthAlarmCopy[i]);
                }*/
			    FEED_DOG();

                g_psSouthAlarm[i] = (SOUTH_ALARM_T*)WMemMalloc(g_psSouthAlarm[i],space);   // ����ռ�
                g_psSouthAlarmCopy[i] = (SOUTH_ALARM_T*)WMemMalloc(g_psSouthAlarmCopy[i],space);   // ����ռ�

                if(NULL!=g_psSouthAlarm[i] && NULL!=g_psSouthAlarmCopy[i])
                {
                    result = EepReadData(eep_addr,(uint8_t *)&g_psSouthAlarm[i][0],space,NULL);
                    result = EepReadData(eep_addr,(uint8_t *)&g_psSouthAlarmCopy[i][0],space,NULL);
                    eep_addr += space;
                }
                else
                {
                    DEBUGOUT("�澯����ռ�ʧ��-%d\r\n",i);
                }
            }
        }
        break;

   case EEP_DT1000_UPDATA:
        result = EepReadData(EEP_DT1000_UPDATA_HEAD,(uint8_t *)&g_DT1000Updata,sizeof(DT1000UPDATA_MARK_T),&g_DT1000Updata.CRC);// �豸ESN��Ϣ�洢
        break;
    }

    return result;
}
/****************************************************************************
* ��    �ƣ�SaveEepData()
* ��    �ܣ��洢�ڲ�EEPROM���ݡ�
* ��ڲ�����
*           t         �洢���������� 1��Logger_inf��2��Device_south��3��Device_esn��4��Logger_updata

* ���ڲ������洢�ɹ����ش洢�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint16_t  SaveEepData(uint8_t t)
{
    uint16_t result;
    uint8_t  i;
    uint8_t  space;
    uint32_t eep_addr;

    switch(t)
    {
    case EEP_LOGGER_INF:
        result = EepSavedata(EEP_LOGGER_INF_HEAD,(uint8_t *)&g_LoggerInfo,sizeof(g_LoggerInfo),&g_LoggerInfo.CRC);// �洢
        break;

    case EEP_DEVICE_SOUTH:
        result = EepSavedata(EEP_LOGGER_DEVICE_INF_HEAD,(uint8_t *)&g_DeviceSouth,sizeof(g_DeviceSouth),&g_DeviceSouth.CRC);// �豸��Ϣ�͵����Ϣ�洢
        break;

    case EEP_DEVICE_ESN:
        result = EepSavedata(EEP_LOGGER_DEVICE_ESN_HEAD,(uint8_t *)&g_DeviceEsn,sizeof(LOGGER_DEVICE_ESN_T),&g_DeviceEsn.CRC);// �豸ESN��Ϣ�洢
        break;

    case EEP_UPDATA:
        result = EepSavedata(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&g_LoggerUpdate,sizeof(UPDATA_MARK_T),&g_LoggerUpdate.CRC);// ������Ϣ�洢
        break;

    case EEP_ALARM:
        result = EepSavedata(EEP_ALARM_HEAD,(uint8_t *)&g_LoggerAlarm.dev_lost,sizeof(g_LoggerAlarm),NULL);
        eep_addr = EEP_ALARM_HEAD + sizeof(g_LoggerAlarm);
        for(i=0;i<MAX_device&&i<g_DeviceSouth.device_sum;i++)
        {
            space = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[i].rel_num].alarm_sum * sizeof(SOUTH_ALARM_T);
            if(space)
            {
				FEED_DOG();
				if(NULL!=g_psSouthAlarm[i])
                {
                    result = EepSavedata(eep_addr,(uint8_t *)&g_psSouthAlarm[i][0],space,NULL);
                    eep_addr += space;
                }
            }
        }
        break;

    case EEP_DEVICE_SOFT:
        result = EepSavedata(EEP_LOGGER_DEVICE_SOFT_HEAD,(uint8_t *)&g_DeviceSoft,sizeof(LOGGER_DEVICE_SOFT_T),&g_DeviceSoft.CRC);// �豸ESN��Ϣ�洢
        break;

    case EEP_DT1000_UPDATA:
        result = EepSavedata(EEP_DT1000_UPDATA_HEAD,(uint8_t *)&g_DT1000Updata,sizeof(DT1000UPDATA_MARK_T),&g_DT1000Updata.CRC);// �豸ESN��Ϣ�洢
        break;

    }

    return result;
}
/****************************************************************************
* ��    �ƣ�AllReset()
* ��    �ܣ��ָ��������á�
* ��ڲ�����
*           uSave     1:�洢    0:���洢

* ���ڲ������洢�ɹ����ش洢�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
void AllReset(uint8_t uSave)
{
    uint8_t i;

    g_DeviceSouth.device_sum = 0x00;
    g_DeviceSouth.yx_sum     = 0x00;
    g_DeviceSouth.yc_sum     = 0x00;
    g_DeviceSouth.yk_sum     = 0x00;
    g_DeviceSouth.sd_sum     = 0x00;
    g_DeviceSouth.dd_sum     = 0x00;


    for(i=0; i<MAX_device; i++)
    {
        g_DeviceSouth.device_inf[i].addr = 0;
        g_DeviceSouth.device_inf[i].rel_num = 0;
        g_DeviceSouth.device_inf[i].protocol_num = 0;
        g_DeviceSouth.device_inf[i].baud_rate = 3;    //1:2400��2:4800��3:9600��4:19200��5:38400��6:115200
        g_DeviceSouth.device_inf[i].protocol_type = 2;
        g_DeviceSouth.device_inf[i].big_little_endian = BIG_ENDIAN;
        g_DeviceSouth.device_inf[i].reserve = 0;
        g_DeviceSouth.device_inf[i].yx_start_addr = 0;
        g_DeviceSouth.device_inf[i].yc_start_addr = 0;
        g_DeviceSouth.device_inf[i].yk_start_addr = 0;
        g_DeviceSouth.device_inf[i].sd_start_addr = 0;
        g_DeviceSouth.device_inf[i].dd_start_addr = 0;

        g_DeviceSouth.protocol[i].protocol_num = 0;
        g_DeviceSouth.protocol[i].protocol_type = 2;
        g_DeviceSouth.protocol[i].alarm_sum = 0;
        g_DeviceSouth.protocol[i].reserve2 = 0;
        g_DeviceSouth.protocol[i].mess_point_sum = 0;
        g_DeviceSouth.protocol[i].flash_addr = 0;

        //---------------------------------------------------
        memset(g_DeviceEsn.cDeviceEsn[i],0,20);
		memset(&g_DeviceEsn.uEsnMark[i],0,1);
		memset(g_DeviceSoft.cDeviceSoft[i],0,17);
        //---------------------------------------------------
        // �ͷŸ澯��ռ�

        g_psSouthAlarm[i] = WMemFree(g_psSouthAlarm[i]);

        g_psSouthAlarmCopy[i] = WMemFree(g_psSouthAlarmCopy[i]);

        //---------------------------------------------------
        // �ͷŵ������ռ�

        g_pRegPoint[i] = WMemFree(g_pRegPoint[i]);

    }

    //---------------------------------------------------
    g_LoggerAlarm.dev_lost  = 0x00;
    g_LoggerRun.err_lost    = 0x00;
    g_LoggerAlarm.log_alarm = 0x00;
    //---------------------------------------------------

    if(uSave)
    {
        SaveEepData(EEP_DEVICE_ESN);
        SaveEepData(EEP_DEVICE_SOFT);
        SaveEepData(EEP_DEVICE_SOUTH);
        SaveEepData(EEP_ALARM);
    }


    // �ͷ�IEC104���ݿռ�

    IEC104_DATA_YX = WMemFree(IEC104_DATA_YX);

    IEC104_DATA_YC = WMemFree(IEC104_DATA_YC);

    IEC104_DATA_YK = WMemFree(IEC104_DATA_YK);

    IEC104_DATA_SD = WMemFree(IEC104_DATA_SD);

    IEC104_DATA_DD = WMemFree(IEC104_DATA_DD);


    RecordInit(1);   // ��ʷ���ݴ洢����
    //SaveLogInit(1);  //��־�洢����

    if(COLLECT_NULL!=g_sIecRun.collect)
    {
        g_sIecRun.collect = COLLECT_END;      // // ���ٽ���
    }

    if(COLLECT_NULL!=g_sIecRun.dd_collect)
    {
        g_sIecRun.dd_collect = COLLECT_DD_END;   // // ���ٽ���
    }
    if(COLLECT_NULL!=g_sIecRun.subcollect)
    {
        g_sIecRun.subcollect = SUBCOLLECT_END;
    }

    ReportCtrlClear(0xFF);


    g_LoggerRun.run_status = RUNNING_EMPTY;
}


/****************************************************************************
* ��    �ƣ�Reboot()
* ��    �ܣ���λ��
* ��ڲ�����
*

* ���ڲ�������
* ��    ��: ��
****************************************************************************/

void Reboot(void)
{
    // reset the mcu
    __DSB();  // Ensure all outstanding memory accesses included   buffered write are completed before reset
    SCB->AIRCR  = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |  SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();                   // Ensure completion of memory access
    
    for(;;)                    // wait until reset
    {
        __NOP();
    }
}