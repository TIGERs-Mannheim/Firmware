/*
 * spi6.c
 *
 *  Created on: 26.10.2015
 *      Author: AndreR
 */

/*
Maxima calculations:
CLK : 54e6 $
t_kick : (16*2)*16*16/CLK;
t_touch : (8?)*80*32/CLK;
t_pwr : (16*3)*16*32/CLK;
t_mpu : 128*64/CLK;
t_opt : 120*32/CLK+120e-6;
t_kick + t_touch + t_pwr + t_mpu + t_opt;
t_kick + t_pwr + t_mpu;
t_touch + t_opt*2;

SPI2:
NRF24

SPI4:
Touch, AD7843, 0.01-2MHz, 1read => 10byte => 80clks
ADNS9800, 2MHz, 1 read => 15byte => 120clks + 120us

SPI6:
Kicker, ADC122S051, 200-500kSPS, 3.2-8MHz, 1 value => 16clks
PRW ADC, ADC124S021, 50-200kSPS, 0.8-3.2MHz, 1 value => 16clks
MPU-6000, 1MHz (all reg), 20MHz (irq, read), 1 read => 16byte => 128clks

SPI div<->Freq:
256: 0,2109375
128: 0,421875
 64: 0,84375
 32: 1,6875
 16: 3,375
  8: 6,75
  4: 13,5
 */

#include "spi6.h"

#include "util/init_hal.h"
#include "util/sys_time.h"
#include "hal/kicker.h"
#include "constants.h"
#include "power.h"

// V_adc * resolution * voltage divider
// 3.3*1/2^12*530/100.0 =
#define ADC_BAT_TO_VOLTAGE 0.00427001953125f

// V_adc * resolution * shunt monitor * shunt
// 3.3*1/2^12*1/60*1/0.001 =
#define ADC_SHUNT_TO_CURRENT 0.013427734375f

// 3.3*1/2^12 =
#define ADC_IR_TO_VOLTAGE 0.0008056640625f

#define ADC_CAP_TO_VOLTAGE 0.06464419157608696f
//#define ADC_CAP_TO_VOLTAGE 0.049560546875f
#define ADC_CHG_TO_CURRENT 0.00732421875f

#define SPI6_ACC_UNIT_TO_MS2 (5.9875488281250003e-4f)
#define SPI6_GYR_UNIT_TO_RADS (0.0010652644360317f)

#define BARRIER_OFF()	(GPIOReset(GPIOB, GPIO_PIN_7))
#define BARRIER_ON()	(GPIOSet(GPIOB, GPIO_PIN_7))

SPI6Global spi6 __attribute__((section(".dtc")));

void DMA2_Stream6_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPISyncDmaRxInterrupt(&spi6.bus);

	CH_IRQ_EPILOGUE();
}

void SPI6Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;

	spi6.power.slave.cpha = 0;
	spi6.power.slave.cpol = 0;
	spi6.power.slave.csPin = GPIO_PIN_1;
	spi6.power.slave.pCSPort = GPIOG;
	spi6.power.slave.prescaler = SPI_CR1_BRDIV16;	// DIV4 is possible, but far outside specs
	spi6.power.slave.timeoutTicks = MS2ST(20);

	spi6.kick.slave.cpha = 0;
	spi6.kick.slave.cpol = 0;
	spi6.kick.slave.csPin = GPIO_PIN_1;
	spi6.kick.slave.pCSPort = GPIOF;
	spi6.kick.slave.prescaler = SPI_CR1_BRDIV16;
	spi6.kick.slave.timeoutTicks = MS2ST(20);

	spi6.imu.slave.cpol = 0;
	spi6.imu.slave.cpha = 0;
	spi6.imu.slave.csPin = GPIO_PIN_11;
	spi6.imu.slave.pCSPort = GPIOG;
	spi6.imu.slave.timeoutTicks = MS2ST(20);
	spi6.imu.slave.prescaler = SPI_CR1_BRDIV128;

	SPISyncData spiData;
	spiData.dmaChRx = 6;
	spiData.dmaChTx = 5;
	spiData.dmaPrio = 1;
	spiData.pDma = DMA2;
	spiData.pRegister = SPI6;
	SPISyncInit(&spi6.bus, &spiData);
	SPISyncSlaveInit(&spi6.bus, &spi6.power.slave);
	SPISyncSlaveInit(&spi6.bus, &spi6.kick.slave);
	SPISyncSlaveInit(&spi6.bus, &spi6.imu.slave);

	// init GPIO
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOG, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, &gpioInit); // MISO, SCK, MOSI

	// configure NVIC
	NVICEnableIRQ(DMA2_Stream6_IRQn, IRQL_SPI6);

	// setup PWR ADC tx data
	for(uint16_t i = 0; i < SPI6_PWR_ADC_SAMPLES; i++)
	{
		spi6.power.tx[i*2] = 1 << 3;
	}
	for(uint16_t i = SPI6_PWR_ADC_SAMPLES; i < 2*SPI6_PWR_ADC_SAMPLES; i++)
	{
		spi6.power.tx[i*2] = 2 << 3;
	}
	for(uint16_t i = 2*SPI6_PWR_ADC_SAMPLES; i < 3*SPI6_PWR_ADC_SAMPLES; i++)
	{
		spi6.power.tx[i*2] = 3 << 3;
	}

	// setup kicker ADC tx data
	for(uint16_t i = 0; i < SPI6_KICK_SAMPLES; i++)
	{
		spi6.kick.tx[i*2] = 0 << 3;
	}
	for(uint16_t i = SPI6_KICK_SAMPLES; i < 2*SPI6_KICK_SAMPLES; i++)
	{
		spi6.kick.tx[i*2] = 1 << 3;
	}

	// Barrier
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOB, GPIO_PIN_7, &gpioInit);

	BARRIER_OFF();
}

static void pwrUpdate()
{
	SPISyncTransfer(&spi6.power.slave, spi6.power.tx, (uint8_t*)spi6.power.rx, SPI6_PWR_ADC_SAMPLES*3*2);

	uint32_t batSum = 0;
	uint32_t curSum = 0;
	uint32_t irSum = 0;
	uint8_t validBatSamples = 0;
	uint8_t validCurSamples = 0;
	uint8_t validIrSamples = 0;

	// sum up measurements, we ignore the first two measurements
	// the sampling capacitor has to settle in that time
	for(uint16_t i = 2; i < SPI6_PWR_ADC_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi6.power.rx[i]);
		if(sample < 4096)
		{
			curSum += sample;
			++validCurSamples;
		}
	}
	for(uint16_t i = SPI6_PWR_ADC_SAMPLES+2; i < 2*SPI6_PWR_ADC_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi6.power.rx[i]);
		if(sample < 4096)
		{
			irSum += sample;
			++validIrSamples;
		}
	}
	for(uint16_t i = 2*SPI6_PWR_ADC_SAMPLES+2; i < 3*SPI6_PWR_ADC_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi6.power.rx[i]);
		if(sample < 4096)
		{
			batSum += sample;
			++validBatSamples;
		}
	}

	if(validBatSamples > 3)
		power.vBat = batSum*(ADC_BAT_TO_VOLTAGE/validBatSamples)-0.05f;

	if(validCurSamples > 3)
		power.iCur = curSum*(ADC_SHUNT_TO_CURRENT/validCurSamples)-0.08f;

	power.measTimestamp = SysTimeUSec();
	spi6.vIr = irSum*(ADC_IR_TO_VOLTAGE/validIrSamples);
}

static void kickUpdate()
{
	SPISyncTransfer(&spi6.kick.slave, spi6.kick.tx, (uint8_t*)spi6.kick.rx, 4*SPI6_KICK_SAMPLES);

	uint32_t ch1Sum = 0;
	uint32_t ch2Sum = 0;
	uint8_t validCh1Samples = 0;
	uint8_t validCh2Samples = 0;

	// sum up measurements, we ignore the first two measurements
	// the sampling capacitor has to settle in that time
	for(uint16_t i = 2; i < SPI6_KICK_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi6.kick.rx[i]);
		if(sample < 4096)
		{
			ch1Sum += sample;
			++validCh1Samples;
		}
	}
	for(uint16_t i = SPI6_KICK_SAMPLES+2; i < 2*SPI6_KICK_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi6.kick.rx[i]);
		if(sample < 4096)
		{
			ch2Sum += sample;
			++validCh2Samples;
		}
	}

	// we need at least 4 valid samples for a reasonable measurement
	if(validCh1Samples > 3 && validCh2Samples > 3)
	{
		float ch1 = ch1Sum/(validCh1Samples*4096.0f);
		float ch2 = ch2Sum/(validCh2Samples*4096.0f);

		KickerADCUpdate(ch1, ch2);
	}
}

#define MPU6000_READ	0x80
#define MPU6000_WRITE	0x00

#define MPU6000_CONFIG				26
#define MPU6000_GYRO_CONFIG			27
#define MPU6000_ACCEL_CONFIG		28
#define MPU6000_PWR_MGMT_1			107
#define MPU6000_SIGNAL_PATH_RESET	104
#define MPU6000_WHOAMI				117

static void imuInit()
{
	chThdSleepMilliseconds(100);

	spi6.imu.tx[0] = MPU6000_PWR_MGMT_1 | MPU6000_WRITE;
	spi6.imu.tx[1] = 0x80;
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, 0, 2);
	chThdSleepMilliseconds(100);

	spi6.imu.tx[0] = MPU6000_SIGNAL_PATH_RESET | MPU6000_WRITE;
	spi6.imu.tx[1] = 0x07;	// GYRO_RST, ACCEL_RST, TEMP_RST
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, 0, 2);
	chThdSleepMilliseconds(100);

	spi6.imu.tx[0] = MPU6000_PWR_MGMT_1 | MPU6000_WRITE;
	spi6.imu.tx[1] = 0x01; // select gyr X clock as PLL source
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, 0, 2);

	spi6.imu.tx[0] = MPU6000_GYRO_CONFIG | MPU6000_WRITE;
	spi6.imu.tx[1] = 0x18; // +-2000dps full scale
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, 0, 2);

	spi6.imu.tx[0] = MPU6000_ACCEL_CONFIG | MPU6000_WRITE;
	spi6.imu.tx[1] = 0x00; // +-2g full scale
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, 0, 2);

	spi6.imu.tx[0] = MPU6000_CONFIG | MPU6000_WRITE;
//	spi6.imu.tx[1] = 0x00; // Acc BW 260Hz/0.0ms, Gyr BW 256Hz/0.98ms
	spi6.imu.tx[1] = 0x01; // Acc BW 184Hz/2.0ms, Gyr BW 188Hz/1.9ms
//	spi6.imu.tx[1] = 0x03; // Acc BW 44Hz/4.9ms, Gyr BW 42Hz/4.8ms
//	spi6.imu.tx[1] = 0x04; // Acc BW 21Hz/8.5ms, Gyr BW 20Hz/8.3ms
//	spi6.imu.tx[1] = 0x05; // Acc BW 10Hz/13.8ms, Gyr BW 10Hz/13.4ms
//	spi6.imu.tx[1] = 0x06; // Acc BW 5Hz/19.0ms, Gyr BW 5Hz/18.6ms
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, 0, 2);

	spi6.imu.slave.prescaler = SPI_CR1_BRDIV8;
}

static void imuUpdate()
{
	spi6.imu.tx[0] = 59 | MPU6000_READ; // start at ACCEL_XOUT_H
	SPISyncTransfer(&spi6.imu.slave, spi6.imu.tx, spi6.imu.rx, 15);

	int16_t acc[3];
	acc[0] = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[1]));
	acc[1] = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[3]));
	acc[2] = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[5]));

	int16_t temp;
	temp = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[7]));

	int16_t gyr[3];
	gyr[0] = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[9]));
	gyr[1] = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[11]));
	gyr[2] = (int16_t)__REV16(*((uint16_t*)&spi6.imu.rx[13]));

	for(uint8_t i = 0; i < 3; i++)
	{
		spi6.imu.gyr[i] = gyr[i]*SPI6_GYR_UNIT_TO_RADS;
		spi6.imu.acc[i] = acc[i]*SPI6_ACC_UNIT_TO_MS2;
	}

	spi6.imu.temp = temp/340.0f + 36.53f;
}

void SPI6Task(void* params)
{
	(void)params;

	chRegSetThreadName("SPI6");

	imuInit();

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(500);

	uint8_t irOn = 0;

	while(1)
	{
		uint32_t start = SysTimeUSec();
		pwrUpdate();
		uint32_t end = SysTimeUSec();
		spi6.power.sampleTime = end-start;

		start = SysTimeUSec();
		kickUpdate();
		end = SysTimeUSec();
		spi6.kick.sampleTime = end-start;

		start = SysTimeUSec();
		imuUpdate();
		end = SysTimeUSec();
		spi6.imu.sampleTime = end-start;

		if(irOn == 0)
		{
			spi6.vIrOff = spi6.vIr;

			BARRIER_ON();
			irOn = 1;
		}
		else
		{
			spi6.vIrOn = spi6.vIr;

			KickerIrUpdate(spi6.vIrOn, spi6.vIrOff);

			BARRIER_OFF();
			irOn = 0;
		}

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(500);
	}
}
