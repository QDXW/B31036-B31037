
#include "chip.h"


#include "KEY_SCAN.h"
#include "GPIO.h"


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


#define KEY1_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,1,25))
#define KEY2_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,1,0))
#define KEY3_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,1,16))
#define KEY4_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,0,17))
#define SCARM_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,13))
#define BUTTON_DETECT (Chip_GPIO_ReadPortBit(LPC_GPIO,2,19))

//----------------按键变量---------------------
KEYINT  KeyDown =0;  // 按下触发
KEYINT  KeyUp   =0;  // 松开触发
KEYINT  KeyOn   =0;  // 连续按住


//static uint16_t Read(uint8_t n);

// LED等和按键IO口初始化
void LED_KEY_IO_Init(void)
{
    GPIO_SetDir(1,26,Output);  // LED1
    GPIO_SetDir(1,27,Output);  // LED2
    GPIO_SetDir(1,4,Output);   // LED3

    LED1(ON);
    LED2(OF);
    LED3(OF);
    
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 25, 0xd0);  // 输入翻转
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  0, 0xd0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 16, 0xd0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, 0xd0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 13, 0xd0);

    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 19, 0x88);  // 输入下拉

    GPIO_SetDir(1,25,Input);  // KEY1
    GPIO_SetDir(1, 0,Input);  // KEY2
    GPIO_SetDir(1,16,Input);  // KEY3
    GPIO_SetDir(0,17,Input);  // KEY4
    GPIO_SetDir(1,13,Input);  // SCARM  急停
    GPIO_SetDir(2,19,Input);  // 按键板检测
}

/*******************************************************************************
* 名称：按键扫描子程序
* 输入：无
* 输出：按键值，存储在全局变量中
* 涉及变量：KeyDown,Keyup,Keyon
*******************************************************************************/
void KeyScan(void)
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
    static KEYINT  PreKey=0x00;      // 记录上次KeyScan()读取的IO口键值
    KEYINT          CurrReadKey;      // 记录本次KeyScan()读取的IO口键值
    KEYINT          NowKey;           // 记录本次经过消抖处理后的有效按键值
    //--------------------------------------------------------------------------
    /*若需要消除长按键的松开触发，则取消本部分的注释
    static uint8_t LongKey=0;         // 长按键库
    static uint8_t X=0;               // 新按下的单键位
    static uint16_t  LongTime=0;        // 长按键计时，记录主循环次数
    */
    //--------------------------------------------------------------------------


    //CurrReadKey = Read(2);   // 读两片74HC165按键

    /*
    #define FIRE_IO              GPIO1PIN_P0A_GET   //(KeyOn & BIT6)   // 消防信号输入端口
    #define Feedback_I_ON        GPIO1PIN_NP02_GET  // I路合闸
    #define Feedback_II_ON       GPIO1PIN_NP04_GET  // II路合闸
    #define Feedback_OFF         GPIO1PIN_NP00_GET  // 分闸
    */
    
    CurrReadKey = KEY4_INPUT;
    CurrReadKey = CurrReadKey<<1 | KEY3_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | KEY2_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | KEY1_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | SCARM_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | BUTTON_DETECT;

    /***************************************************************************
    * 消抖原理很简单：
    * 如果上次LastReadKey和当前CurReadKey读取的键值都为1，那么当前有效键值CurrKey一定为1
    * 如果上次和当前读取的键值不一样的话，则与上次有效键值LastKey保持一致。
    * 通过这种方式来进行消抖，抖动时间必须小于keyscan()定时扫描周期，否则会出现错误。
    ***************************************************************************/
    NowKey = (CurrReadKey & PreKey) | (KeyOn & (CurrReadKey ^ PreKey));

    //记录按键按下及释放
    KeyDown = (~KeyOn) & NowKey;    // 按下按键触发置位
    KeyUp   = KeyOn & (~NowKey);    // 松开按键触发置位

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
    KeyOn = NowKey;
}



/*******************************************************************************
* 名称：读165
* 输入：165级联片数
* 输出：按键值，存储在全局变量中
* 涉及变量：KeyDown,Keyup,Keyon
*******************************************************************************/
/*static uint16_t Read(uint8_t n)               // 74HC165
{
    uint8_t read[2];    // 信号变量暂存
    uint16_t readback;
    uint8_t  i,j;

    KEY_Enable;
    // 读取级联74HC165。
    LOAD_165_L;
    asm("nop");
    asm("nop");
    asm("nop");
    LOAD_165_H;

    asm("nop");
    asm("nop");
    CLK_165_L;


    for(j=0;j<n;j++)
    {
        read[j] = 0;
        for(i=0;i<8;i++)
        {
            read[j] <<= 1;
            read[j] |= INPUT_165;

            CLK_165_L;
            asm("nop");
            asm("nop");
            CLK_165_H;
            asm("nop");
            asm("nop");
        }
    }

    readback = read[0] | (read[1]<<8);//((read&0xff)<<8) | (read>>8);
    KEY_Disable;

    return readback;
}*/
