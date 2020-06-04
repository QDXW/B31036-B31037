#include "chip.h"
#include "IIC.h"


#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000
#define I2C_DEFAULT_SPEED    SPEED_100KHZ
#define I2C_FASTPLUS_BIT     0

#if (I2C_DEFAULT_SPEED > SPEED_400KHZ)
#undef  I2C_FASTPLUS_BIT
#define I2C_FASTPLUS_BIT IOCON_FASTI2C_EN
#endif

#ifdef DEBUG_ENABLE
static const char menu[] =
	"**************** I2C Demo Menu ****************\r\n"
	"\t0: Exit Demo\r\n"
	"\t1: Select I2C peripheral [\033[1;32mI2C%d\033[0;37m]\r\n"
	"\t2: Toggle mode POLLING/INTERRUPT [\033[1;32m%s\033[0;37m]\r\n"
	"\t3: Probe for Slave devices\r\n"
	"\t4: Read slave data\r\n"
	"\t5: Write slave data\r\n"
	"\t6: Write/Read slave data\r\n";
#endif

static int mode_poll;	/* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C;	/* Currently active I2C device */
static volatile uint32_t time_out;     // 超时

#define I2C_SLAVE_ADDR       0xd0//0x5A


/* Data area for slave operations */

//static volatile uint32_t tick_cnt;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
#define mSLAVE_ACTIVE(iic) (((iic)->flags & 0xFF00) != 0)
struct i2c_interface {
	LPC_I2C_T *ip;		/* IP base address of the I2C device */
	CHIP_SYSCTL_CLOCK_T clk;	/* Clock used by I2C */
	I2C_EVENTHANDLER_T mEvent;	/* Current active Master event handler */
	I2C_EVENTHANDLER_T sEvent;	/* Slave transfer events */
	I2C_XFER_T *mXfer;	/* Current active xfer pointer */
	I2C_XFER_T *sXfer;	/* Pointer to store xfer when bus is busy */
	uint32_t flags;		/* Flags used by I2C master and slave */
};

static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	}
	else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

 /* I2C interfaces */
static struct i2c_interface i2c[I2C_NUM_INTERFACE] = {
	{LPC_I2C0, SYSCTL_CLOCK_I2C0, Chip_I2C_EventHandler, NULL, NULL, NULL, 0},
	{LPC_I2C1, SYSCTL_CLOCK_I2C1, Chip_I2C_EventHandler, NULL, NULL, NULL, 0}
};

/* Enable I2C and start master transfer */
STATIC INLINE void mstartMasterXfer(LPC_I2C_T *pI2C)
{
	/* Reset STA, STO, SI */
	pI2C->CONCLR = I2C_CON_SI | I2C_CON_STO | I2C_CON_STA | I2C_CON_AA;

	/* Enter to Master Transmitter mode */
	pI2C->CONSET = I2C_CON_I2EN | I2C_CON_STA;
}

/* Check if I2C bus is free */
STATIC INLINE int misI2CBusFree(LPC_I2C_T *pI2C)
{
	return !(pI2C->CONSET & I2C_CON_STO);
}

/* Enable I2C and enable slave transfers */
STATIC INLINE void mstartSlaverXfer(LPC_I2C_T *pI2C)
{
	/* Reset STA, STO, SI */
	pI2C->CONCLR = I2C_CON_SI | I2C_CON_STO | I2C_CON_STA;

	/* Enter to Master Transmitter mode */
	pI2C->CONSET = I2C_CON_I2EN | I2C_CON_AA;
}

/* Transmit and Receive data in master mode */
int I2C_MasterTransfer(I2C_ID_T id, I2C_XFER_T *xfer)
{
	struct i2c_interface *miic = &i2c[id];

	miic->mEvent(id, I2C_EVENT_LOCK);
	xfer->status = I2C_STATUS_BUSY;
	miic->mXfer = xfer;

	/* If slave xfer not in progress */
	if (!miic->sXfer) {
		mstartMasterXfer(miic->ip);
	}
	miic->mEvent(id, I2C_EVENT_WAIT);
	miic->mXfer = 0;

	/* Wait for stop condition to appear on bus */
	while (!misI2CBusFree(miic->ip)) {}

	/* Start slave if one is active */
	if (mSLAVE_ACTIVE(miic)) {
		mstartSlaverXfer(miic->ip);
	}

	miic->mEvent(id, I2C_EVENT_UNLOCK);
	return (int) xfer->status;
}

//=====================================================================================================
static void Init_I2C_PinMux(void)
{
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, 0x1);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, 0x1);
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if (!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(I2C0_IRQn);
	}
	else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(I2C0_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
void i2c_app_init(I2C_ID_T id, int speed)
{
	Init_I2C_PinMux();

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}
//================================================================================================
void IIC_Init(void)
{
    i2c_app_init(I2C0, I2C_DEFAULT_SPEED);
}
uint8_t IIC_read(uint8_t *buff,uint8_t len)
{
    //uint8_t tmp;
    //tmp = Chip_I2C_MasterRead(i2cDev, I2C_SLAVE_ADDR, buff, len);
    /*int Chip_I2C_MasterRead(I2C_ID_T id, uint8_t slaveAddr, uint8_t *buff, int len)
    {
        I2C_XFER_T xfer = {0};
        xfer.slaveAddr = slaveAddr;
        xfer.rxBuff = buff;
        xfer.rxSz = len;
        while (Chip_I2C_MasterTransfer(id, &xfer) == I2C_STATUS_ARBLOST) {}
        return len - xfer.rxSz;
    }*/

    I2C_XFER_T xfer = {0};
    xfer.slaveAddr = I2C_SLAVE_ADDR;
    xfer.rxBuff = buff;
    xfer.rxSz = len;
    while (I2C_MasterTransfer(i2cDev, &xfer) == I2C_STATUS_ARBLOST)
    {

    }
    return len - xfer.rxSz;
}

uint8_t IIC_write(uint8_t *buff,uint8_t len)
{
    //uint8_t tmp;
    //tmp = Chip_I2C_MasterSend(i2cDev, I2C_SLAVE_ADDR, buff, len);
    /* Master tx only */
    /*int Chip_I2C_MasterSend(I2C_ID_T id, uint8_t slaveAddr, const uint8_t *buff, uint8_t len)
    {
        I2C_XFER_T xfer = {0};
        xfer.slaveAddr = slaveAddr;
        xfer.txBuff = buff;
        xfer.txSz = len;
        while (Chip_I2C_MasterTransfer(id, &xfer) == I2C_STATUS_ARBLOST) {}
        return len - xfer.txSz;
    }*/
    I2C_XFER_T xfer = {0};
    xfer.slaveAddr = I2C_SLAVE_ADDR;
    xfer.txBuff = buff;
    xfer.txSz = len;
    while (I2C_MasterTransfer(i2cDev, &xfer) == I2C_STATUS_ARBLOST)
    {

    }
    return len - xfer.txSz;
}


