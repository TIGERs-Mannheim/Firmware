#include "io_tca9539.h"

#define I2C_ADDR_TCA9539	0x74

#define TCA9539_REG_OUTPUT_PORT_0			0x02
#define TCA9539_REG_CONFIGURATION_PORT_0	0x06

static void ioTask(void* pParams);
static void configure(IoTCA9539* pIo);
static void gpioWrite(GPIOPinInterface* pGPIO, uint8_t level);
static uint8_t gpioRead(GPIOPinInterface* pGPIO);
static void gpioConfigure(GPIOPinInterface* pGPIO, uint8_t isOutput);

void IoTCA9539Init(IoTCA9539* pIo, I2C* pBus, GPIOPin rstPin, tprio_t prio)
{
	chMtxObjectInit(&pIo->writeMutex);

	pIo->pBus = pBus;
	pIo->rstPin = rstPin;

	// Pull reset low
	GPIOPinReset(rstPin);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(rstPin.pPort, rstPin.pin, &gpioInit); // SMB-RST pin for TCA9539

	configure(pIo);

	pIo->pTask = chThdCreateStatic(pIo->waTask, sizeof(pIo->waTask), prio, &ioTask, pIo);
}

void IoTCA9539GPIOPinInit(GPIOPinTCA9539* pPin, IoTCA9539* pIo, uint16_t pinMask)
{
	pPin->pTca = pIo;
	pPin->pinMask = pinMask;
	pPin->pin.write = &gpioWrite;
	pPin->pin.read = &gpioRead;
	pPin->pin.configure = &gpioConfigure;
}

void IoTCA9539SetAsInput(IoTCA9539* pIo, uint16_t set, uint16_t reset)
{
	chMtxLock(&pIo->writeMutex);

	pIo->cfgReg.u16 &= ~reset;
	pIo->cfgReg.u16 |= set;

	chMtxUnlock(&pIo->writeMutex);

	chEvtSignal(pIo->pTask, EVENT_MASK(0));
}

void IoTCA9539SetOutLevel(IoTCA9539* pIo, uint16_t set, uint16_t reset)
{
	chMtxLock(&pIo->writeMutex);

	pIo->outReg.u16 &= ~reset;
	pIo->outReg.u16 |= set;

	chMtxUnlock(&pIo->writeMutex);

	chEvtSignal(pIo->pTask, EVENT_MASK(0));
}

static void updateCfgReg(IoTCA9539* pIo)
{
	uint8_t* pTx;
	I2CAcquire(pIo->pBus, &pTx, 0);

	pTx[0] = TCA9539_REG_CONFIGURATION_PORT_0;
	chMtxLock(&pIo->writeMutex);
	pTx[1] = pIo->cfgReg.u8[0];
	pTx[2] = pIo->cfgReg.u8[1];
	chMtxUnlock(&pIo->writeMutex);

	pIo->comStatus = I2CWrite(pIo->pBus, I2C_ADDR_TCA9539, 3);
	if(pIo->comStatus)
	{
		// comm failed, at least keep it in reset (if possible)
		GPIOPinReset(pIo->rstPin);
	}

	I2CRelease(pIo->pBus);
}

static void updateOutReg(IoTCA9539* pIo)
{
	uint8_t* pTx;
	I2CAcquire(pIo->pBus, &pTx, 0);

	pTx[0] = TCA9539_REG_OUTPUT_PORT_0;
	chMtxLock(&pIo->writeMutex);
	pTx[1] = pIo->outReg.u8[0];
	pTx[2] = pIo->outReg.u8[1];
	chMtxUnlock(&pIo->writeMutex);

	pIo->comStatus = I2CWrite(pIo->pBus, I2C_ADDR_TCA9539, 3);
	if(pIo->comStatus)
	{
		// comm failed, at least keep it in reset (if possible)
		GPIOPinReset(pIo->rstPin);
	}

	I2CRelease(pIo->pBus);
}

static void configure(IoTCA9539* pIo)
{
	// Pull reset low
	GPIOPinReset(pIo->rstPin);

	chThdSleepMilliseconds(10);

	// Get TCA9539 out of reset
	GPIOPinSet(pIo->rstPin);

	chThdSleepMilliseconds(10);

	// Configure all pins as input by default
	pIo->outReg.u16 = 0;
	pIo->cfgReg.u16 = 0xFFFF;

	updateOutReg(pIo);
	updateCfgReg(pIo);
}

static void ioTask(void* pParams)
{
	IoTCA9539* pIo = (IoTCA9539*)pParams;

	chRegSetThreadName("IO_TCA9539");

	while(1)
	{
		chEvtWaitAny(ALL_EVENTS);

		updateOutReg(pIo);
		updateCfgReg(pIo);
	}
}

static void gpioWrite(GPIOPinInterface* pGPIO, uint8_t level)
{
	GPIOPinTCA9539* pPin = (GPIOPinTCA9539*)pGPIO;

	if(level)
		IoTCA9539SetOutLevel(pPin->pTca, pPin->pinMask, 0);
	else
		IoTCA9539SetOutLevel(pPin->pTca, 0, pPin->pinMask);
}

static uint8_t gpioRead(GPIOPinInterface*)
{
	return 0; // not supported by this interface
}

static void gpioConfigure(GPIOPinInterface* pGPIO, uint8_t isOutput)
{
	GPIOPinTCA9539* pPin = (GPIOPinTCA9539*)pGPIO;

	if(isOutput)
		IoTCA9539SetAsInput(pPin->pTca, 0, pPin->pinMask);
	else
		IoTCA9539SetAsInput(pPin->pTca, pPin->pinMask, 0);
}
