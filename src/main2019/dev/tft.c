#include "tft.h"
#include "hal/init_hal.h"
#include "ch.h"

static void tftReset(uint8_t reset);
static void tftEnable(uint8_t enable);

static void regInit()
{
	// Hardware reset
	tftReset(0);
	chThdSleepMilliseconds(1);
	tftReset(1);
	chThdSleepMilliseconds(10);
	tftReset(0);
	chThdSleepMilliseconds(120);

	*TFT_CMD_REG = 0x11;	// exit sleep
	chThdSleepMilliseconds(5);

	*TFT_CMD_REG = 0xCF;	// Power Control B
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x81;
	*TFT_DATA_REG = 0xC0;

	*TFT_CMD_REG = 0xED;	// Power On Sequence Control
	*TFT_DATA_REG = 0x64;
	*TFT_DATA_REG = 0x03;
	*TFT_DATA_REG = 0x12;
	*TFT_DATA_REG = 0x81;

	*TFT_CMD_REG = 0xE8;	// Driver Timing Control A
	*TFT_DATA_REG = 0x85;
	*TFT_DATA_REG = 0x01;
	*TFT_DATA_REG = 0x79;

	*TFT_CMD_REG = 0xCB;	// Power Control A
	*TFT_DATA_REG = 0x39;
	*TFT_DATA_REG = 0x2C;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x34;
	*TFT_DATA_REG = 0x02;

	*TFT_CMD_REG = 0xF7;	// Pump Ratio Control
	*TFT_DATA_REG = 0x20;

	*TFT_CMD_REG = 0xEA;	// Driver Timing Control B
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0xB1;	// Frame Rate Control
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x1B;

	*TFT_CMD_REG = 0xB6;	// Display Function Control
	*TFT_DATA_REG = 0x0A;
	*TFT_DATA_REG = 0x82;

	*TFT_CMD_REG = 0xC0;	// power control 1
	*TFT_DATA_REG = 0x05;

	*TFT_CMD_REG = 0xC1;	// power control 2
	*TFT_DATA_REG = 0x11;

	*TFT_CMD_REG = 0xC5;	// VCOM control 1
	*TFT_DATA_REG = 0x45;
	*TFT_DATA_REG = 0x45;

	*TFT_CMD_REG = 0xC7;	// VCOM control 2
	*TFT_DATA_REG = 0xA2;

	*TFT_CMD_REG = 0x36;	// Memory Access Control
	*TFT_DATA_REG = 0x48;

	*TFT_CMD_REG = 0xF2;	// Enable 3G
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0x26;	// Gamma Set
	*TFT_DATA_REG = 0x01;

	*TFT_CMD_REG = 0xE0;	// Positive Gamma Correction
	*TFT_DATA_REG = 0x0F;
	*TFT_DATA_REG = 0x26;
	*TFT_DATA_REG = 0x24;
	*TFT_DATA_REG = 0x0B;
	*TFT_DATA_REG = 0x0E;
	*TFT_DATA_REG = 0x08;
	*TFT_DATA_REG = 0x4B;
	*TFT_DATA_REG = 0xA8;
	*TFT_DATA_REG = 0x3B;
	*TFT_DATA_REG = 0x0A;
	*TFT_DATA_REG = 0x14;
	*TFT_DATA_REG = 0x06;
	*TFT_DATA_REG = 0x10;
	*TFT_DATA_REG = 0x09;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0xE1;	// Negative Gamma Correction
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x1C;
	*TFT_DATA_REG = 0x20;
	*TFT_DATA_REG = 0x04;
	*TFT_DATA_REG = 0x10;
	*TFT_DATA_REG = 0x08;
	*TFT_DATA_REG = 0x34;
	*TFT_DATA_REG = 0x47;
	*TFT_DATA_REG = 0x44;
	*TFT_DATA_REG = 0x05;
	*TFT_DATA_REG = 0x0B;
	*TFT_DATA_REG = 0x09;
	*TFT_DATA_REG = 0x2F;
	*TFT_DATA_REG = 0x36;
	*TFT_DATA_REG = 0x0F;

	*TFT_CMD_REG = 0x2A;	// Column Address Set
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0xEF;

	*TFT_CMD_REG = 0x2B;	// Page Address Set
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x01;
	*TFT_DATA_REG = 0x3F;

	*TFT_CMD_REG = 0x3A;	// Pixel Format Set
	*TFT_DATA_REG = 0x55;

	*TFT_CMD_REG = 0xF6;	// Interface Control
	*TFT_DATA_REG = 0x01;
	*TFT_DATA_REG = 0x30;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0x29;	// Display On

	*TFT_CMD_REG = 0x2C;	// Memory Write
}

void DevTFTInit()
{
	// FMC pins
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_100MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.alternate = 12; // FMC

	GPIOInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 |
			GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit);

	GPIOInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
			GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit);

	GPIOInit(GPIOF, GPIO_PIN_13, &gpioInit);

	// LCD_ON and LCD_RESET
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 0;

	GPIOReset(GPIOF, GPIO_PIN_15);
	GPIOReset(GPIOG, GPIO_PIN_1);

	GPIOInit(GPIOF, GPIO_PIN_15, &gpioInit);
	GPIOInit(GPIOG, GPIO_PIN_1, &gpioInit);

	// configure FMC
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
	__DSB();

	FMC_Bank1_R->BTCR[0] = FMC_BCRx_MWID_0 | FMC_BCRx_WREN | FMC_BCR1_WFDIS | 0x80;
	FMC_Bank1_R->BTCR[1] = (5 << FMC_BTRx_DATAST_Pos);	// DataSetupTime = 5
	FMC_Bank1_R->BTCR[0] |= FMC_BCRx_MBKEN | FMC_BCR1_FMCEN;

	regInit();

	chThdSleepMilliseconds(120);

	tftEnable(1);
}

static void tftReset(uint8_t reset)
{
	if(reset)
		GPIOReset(GPIOG, GPIO_PIN_1);
	else
		GPIOSet(GPIOG, GPIO_PIN_1);
}

static void tftEnable(uint8_t enable)
{
	if(enable)
		GPIOSet(GPIOF, GPIO_PIN_15);
	else
		GPIOReset(GPIOF, GPIO_PIN_15);
}
