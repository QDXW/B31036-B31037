
#include "chip.h"

#include "KEY_SCAN.h"

#include "SWITCH_SCAN.h"
#include "GPIO.h"
#include "GlobalVar.h"
#include "InEEPROM.h"
//==============================================================================
//端口定义
//==============================================================================
//#define LOAD_165_L              GPIO1PIN_P3B_PUT(0)    // 74HC165 信号数据加载 PL
//#define LOAD_165_H              GPIO1PIN_P3B_PUT(1)    // 74HC165 信号数据加载 PL  LDB6
//#define CLK_165_L               GPIO1PIN_P3C_PUT(0)    // 74HC165 读时钟  CLK
//#define CLK_165_H               GPIO1PIN_P3C_PUT(1)    // 74HC165 读时钟  CLK  LDB7
//#define INPUT_165               GPIO1PIN_P3A_GET       // 74HC165 数据输入     LDB5
//#define KEY_Enable              GPIO1PIN_P3A_INIT(INPUT_LOW)  // 允许165输入
//#define KEY_Disable             GPIO1PIN_P3A_INIT(OUTPUT)// 禁止165输入

#define  Serial165LoadIO(x)        GPIO_SetBit(2,17,x)
#define  Serial165ClkIO(x)         GPIO_SetBit(1,30,x)
#define  Serial165InIO()           (Chip_GPIO_ReadPortBit(LPC_GPIO,2,18))


#define S01_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,22))
#define S02_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,21))
#define S03_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,20))
#define S04_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1, 3))
#define S05_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,14))
#define S06_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,13))
#define S07_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,30))
#define S08_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,12))
#define S09_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,11))
#define S10_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,11))
#define S11_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,22))
#define S12_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,19))
#define S13_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,21))
#define S14_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,18))
#define S15_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,17))
#define S16_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,16))

//----------------按键变量---------------------
SWITCHINT  SwitchDown =0;  // 按下触发
SWITCHINT  SwitchUp   =0;  // 松开触发
SWITCHINT  SwitchOn   =0;  // 连续按住


static uint16_t Read165(uint8_t n);

// LED等和按键IO口初始化
void SWITCH_IO_Init(void)
{
    /*// S01 - S16
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 22, 0x90);  // S01
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 21, 0x90);  // S02
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 20, 0x90);  // S03
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  3, 0x90);  // S04
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, 0x91);  // S05
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, 0x91);  // S06
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 30, 0x90);  // S07
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, 0x91);  // S08
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 11, 0x90);  // S09
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, 0x91);  // S10
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, 0x90);  // S11
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 19, 0x90);  // S12
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 21, 0x90);  // S13
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 18, 0x90);  // S14
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 17, 0x90);  // S15
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 16, 0x90);  // S16

    // S01 - S16
    GPIO_SetDir(2, 22, Input);  // S01
    GPIO_SetDir(2, 21, Input);  // S02
    GPIO_SetDir(2, 20, Input);  // S03
    GPIO_SetDir(1,  3, Input);  // S04
    GPIO_SetDir(0, 14, Input);  // S05
    GPIO_SetDir(0, 13, Input);  // S06
    GPIO_SetDir(1, 30, Input);  // S07
    GPIO_SetDir(0, 12, Input);  // S08
    GPIO_SetDir(1, 11, Input);  // S09
    GPIO_SetDir(0, 11, Input);  // S10
    GPIO_SetDir(0, 22, Input);  // S11
    GPIO_SetDir(2, 19, Input);  // S12
    GPIO_SetDir(1, 21, Input);  // S13
    GPIO_SetDir(2, 18, Input);  // S14
    GPIO_SetDir(2, 17, Input);  // S15
    GPIO_SetDir(2, 16, Input);  // S16
    */

    GPIO_SetDir(2, 17, Output);  // LOAD
    GPIO_SetDir(2, 18, Input);   // DATA
    GPIO_SetDir(1, 30, Output);  // CLK

    Serial165LoadIO(1);
    Serial165LoadIO(0);
    Serial165LoadIO(1);

    Serial165ClkIO(1);
    Serial165ClkIO(0);
    Serial165ClkIO(1);
}

/*******************************************************************************
* 名称：按键扫描子程序
* 输入：无
* 输出：按键值，存储在全局变量中
* 涉及变量：KeyDown,Keyup,Keyon
*******************************************************************************/
void SwitchScan(void)
{
    /*
    uint8_t ReadData = P2IN^0xff;   // 读键值
    //Trg = ReadData & (ReadData^Cont);
    Trg = ReadData&(~Cont);               // 得到按下触发值
    //Release=  (ReadData ^ Trg ^ Cont);
    Release = Cont &(~ReadData);          // 得到释放触发值
    Cont = ReadData;                      // 得到所有未释放的键值
    */
    //以下部分参考：http://blog.csdn.net/xuechaojie/article/details/6761772
    static SWITCHINT   PreKey=0x00;      // 记录上次KeyScan()读取的IO口键值
    SWITCHINT           CurrReadKey;      // 记录本次KeyScan()读取的IO口键值
    SWITCHINT           NowKey;           // 记录本次经过消抖处理后的有效按键值

    static uint8_t  uDelay;
    //--------------------------------------------------------------------------
    /*若需要消除长按键的松开触发，则取消本部分的注释
    static uint8_t LongKey=0;         // 长按键库
    static uint8_t X=0;               // 新按下的单键位
    static uint16_t  LongTime=0;        // 长按键计时，记录主循环次数
    */
    //--------------------------------------------------------------------------
    //if(0==GetRunKeyBoardOn())
    {
       // return;
    }
    if(KeyOn & Key_button)
    {
        SetRunKeyBoardOn(1);
    }
    else
    {
        SetRunKeyBoardOn(0);
        return;
    }


    CurrReadKey = Read165(2);   // 读两片74HC165按键

    if(0xFFFF==CurrReadKey)  // 键盘被移除
    {
        if(0xFFFF!=PreKey)
        {
            /*if(SET_OK==SetStateLastSwitch(SwitchOn))
            {
                SaveManual();
            }*/

            PreKey = 0xFFFF;
        }
        uDelay = 0;
        return;
    }
    else
    {
        if(uDelay<10)
        {
            uDelay++;
            return;
        }
    }

    /*
    #define FIRE_IO              GPIO1PIN_P0A_GET   //(KeyOn & BIT6)   // 消防信号输入端口
    #define Feedback_I_ON        GPIO1PIN_NP02_GET  // I路合闸
    #define Feedback_II_ON       GPIO1PIN_NP04_GET  // II路合闸
    #define Feedback_OFF         GPIO1PIN_NP00_GET  // 分闸
    */
/*
    CurrReadKey = S16_INPUT;
    CurrReadKey = CurrReadKey<<1 | S15_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S14_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S13_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S12_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S11_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S10_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S09_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S08_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S07_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S06_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S05_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S04_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S03_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S02_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S01_INPUT;   //
*/

    /***************************************************************************
    * 消抖原理很简单：
    * 如果上次LastReadKey和当前CurReadKey读取的键值都为1，那么当前有效键值CurrKey一定为1
    * 如果上次和当前读取的键值不一样的话，则与上次有效键值LastKey保持一致。
    * 通过这种方式来进行消抖，抖动时间必须小于keyscan()定时扫描周期，否则会出现错误。
    ***************************************************************************/
    NowKey = (CurrReadKey & PreKey) | (SwitchOn & (CurrReadKey ^ PreKey));

    //记录按键按下及释放
    SwitchDown = (~SwitchOn) & NowKey;    // 按下按键触发置位
    SwitchUp   = SwitchOn & (~NowKey);    // 松开按键触发置位

    //-------------------------消除长按键的松开触发-----------------------------
    /* 若需要消除长按键的松开触发，则取消本部分的注释
    LongTime++;
    if(KeyDown)
    {
        X = KeyDown ^ KeyOn;      // 读取新按下的键值，KeyOn位之前已按下的键值，KeyDown为新的键值，它们不同的位就是新的键值
        LongTime = 0;
    }
    if(LongTime>30000)            // 计时，单位时间为主循环的时间
    {
        LongKey |= X;             // 保存入长按键库
        X = 0;
        LongTime = 0;
    }
    //KeyUp = (KeyOn & (~NowKey)) & (~LongKey);  // 若是长按键，则没有松开触发
    KeyUp &= ~LongKey;            // 删除长按键松开触发位
    LongKey &= NowKey;            // 从长按键库删除已松开键
    */
    //--------------------------------------------------------------------------

    PreKey = CurrReadKey;         // 更新历史按键
    SwitchOn = NowKey;

    SetRunKeyboard(SwitchOn);
}



/*******************************************************************************
* 名称：读165
* 输入：165级联片数
* 输出：按键值，存储在全局变量中
* 涉及变量：KeyDown,Keyup,Keyon
*******************************************************************************/
static uint16_t Read165(uint8_t n)               // 74HC165
{
    uint8_t read[2]={0,0};    // 信号变量暂存
    uint16_t readback;
    uint8_t  i,j;

    //KEY_Enable;
    // 读取级联74HC165。
    Serial165LoadIO(1);//LOAD_165_L;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    Serial165LoadIO(0);//LOAD_165_H;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    Serial165LoadIO(1);


    asm("nop");
    asm("nop");
    Serial165ClkIO(0);//CLK_165_L;


    for(j=0;j<n;j++)
    {
        read[j] = 0;
        for(i=0;i<8;i++)
        {
            read[j] <<= 1;
            read[j] |= Serial165InIO();//INPUT_165;

            Serial165ClkIO(0);//CLK_165_L;
            asm("nop");
            asm("nop");
            Serial165ClkIO(1);//CLK_165_H;
            asm("nop");
            asm("nop");
        }
    }

    readback = read[0] | (read[1]<<8);//((read&0xff)<<8) | (read>>8);
    //KEY_Disable;

    return readback;
}

