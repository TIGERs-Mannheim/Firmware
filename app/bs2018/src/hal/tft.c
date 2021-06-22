/*
 * tft.c
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#include "tft.h"
#include "util/init_hal.h"

// convert 0x00RRGGBB to RGB565 format, max value for R, G and B is 0x3F (6Bit)
#define TFT_COLOR(rgb) ((uint16_t)((rgb & 0x3E0000) >> 6) | (rgb & 0x3F00) >> 3 | (rgb & 0x3E) >> 1)

// Register Definitions
#define TFT_REG_SOFT_RESET					0x01
#define TFT_REG_SET_DISPLAY_ON				0x29
#define TFT_REG_SET_COLUMN_ADDRESS			0x2A
#define TFT_REG_SET_PAGE_ADDRESS			0x2B
#define TFT_REG_WRITE_MEMORY_START			0x2C
#define TFT_REG_SET_ADDRESS_MODE			0x36
#define TFT_REG_EXIT_IDLE_MODE				0x38
#define TFT_REG_WRITE_MEMORY_CONTINUE		0x3C
#define TFT_REG_SET_LCD_MODE				0xB0
#define TFT_REG_SET_HORI_PERIOD				0xB4
#define TFT_REG_SET_VERT_PERIOD				0xB6
#define TFT_REG_SET_GPIO_CONF				0xB8
#define TFT_REG_SET_GPIO_VALUE				0xBA
#define TFT_REG_SET_PWM_CONF				0xBE
#define TFT_REG_SET_PLL						0xE0
#define TFT_REG_SET_PLL_MN					0xE2
#define TFT_REG_SET_LSHIFT_FREQ				0xE6
#define TFT_REG_SET_PIXEL_DATA_INTERFACE	0xF0
#define TFT_REG_SET_ADDR_MODE				0x36
#define TFT_REG_SET_COL_ADDR				0x2A
#define TFT_REG_SET_PG_ADDR					0x2B

static void regInit()
{
	// Hardware reset
	TFTReset(0);
	chThdSleepMilliseconds(1);
	TFTReset(1);
	chThdSleepMilliseconds(10);
	TFTReset(0);
	chThdSleepMilliseconds(120);

	*TFT_CMD_REG = 0xE2;	//PLL multiplier, set PLL clock to 120M
	*TFT_DATA_REG = 0x1E;	   //N=0x36 for 6.5M, 0x23 for 10M crystal
	*TFT_DATA_REG = 0x02;
	*TFT_DATA_REG = 0x54;

	*TFT_CMD_REG = 0xE0;	// PLL enable
	*TFT_DATA_REG = 0x01;
	chThdSleepMilliseconds(10);

	*TFT_CMD_REG = 0xE0;
	*TFT_DATA_REG = 0x03;
	chThdSleepMilliseconds(10);

	*TFT_CMD_REG = 0x01;	// software reset
	chThdSleepMilliseconds(100);

	*TFT_CMD_REG = 0xE6;		//PLL setting for PCLK, depends on resolution
	*TFT_DATA_REG = 0x04;
	*TFT_DATA_REG = 0x6F;
	*TFT_DATA_REG = 0x46;

	FMC_Bank1->BTCR[1] = (3 << 8);	// DataSetupTime = 3+1 HCLK

	*TFT_CMD_REG = 0xB0;	//LCD SPECIFICATION
	*TFT_DATA_REG = 0x24;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x03;	//Set HDP	799
	*TFT_DATA_REG = 0x1F;
	*TFT_DATA_REG = 0x01;	//Set VDP	479
	*TFT_DATA_REG = 0xDF;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0xB4;	//HSYNC
	*TFT_DATA_REG = 0x03;	//Set HT	928
	*TFT_DATA_REG = 0xA0;
	*TFT_DATA_REG = 0x00;	//Set HPS	46
	*TFT_DATA_REG = 0x2E;
	*TFT_DATA_REG = 0x30;	//Set HPW	48
	*TFT_DATA_REG = 0x00;	//Set LPS	15
	*TFT_DATA_REG = 0x0F;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0xB6;	//VSYNC
	*TFT_DATA_REG = 0x02;	//Set VT	525
	*TFT_DATA_REG = 0x0D;
	*TFT_DATA_REG = 0x00;	//Set VPS	16
	*TFT_DATA_REG = 0x10;
	*TFT_DATA_REG = 0x10;	//Set VPW	16
	*TFT_DATA_REG = 0x00;	//Set FPS	8
	*TFT_DATA_REG = 0x08;

	*TFT_CMD_REG = 0xBA;
	*TFT_DATA_REG = 0x0F;	//GPIO[3:0] out 1

	*TFT_CMD_REG = 0xB8;
	*TFT_DATA_REG = 0x07;	//GPIO3=input, GPIO[2:0]=output
	*TFT_DATA_REG = 0x01;	//GPIO0 normal

	*TFT_CMD_REG = 0x36;	//rotation
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0xF0;	//pixel data interface
	*TFT_DATA_REG = 0x03;

	chThdSleepMilliseconds(1);

	*TFT_CMD_REG = 0x2A;	// Column Address Set
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x03;
	*TFT_DATA_REG = 0x1F;

	*TFT_CMD_REG = 0x2B;	// Page Address Set
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x01;
	*TFT_DATA_REG = 0xDF;

	*TFT_CMD_REG = 0x2C;	// Write Memory Start

	*TFT_CMD_REG = 0x29;	//display on

	*TFT_CMD_REG = 0xBE;	//set PWM for B/L
	*TFT_DATA_REG = 0x06;
	*TFT_DATA_REG = 0xF0;

//	*TFT_DATA_REG = 0x01;
	*TFT_DATA_REG = 0x00;

	*TFT_DATA_REG = 0xF0;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0xD0;	// Dynamic backlight configuration
//	*TFT_DATA_REG = 0x0D;
	*TFT_DATA_REG = 0x00;

	*TFT_CMD_REG = 0x2C;	// Write Memory Start
}

void TFTInit()
{
	// FMC pins
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_100MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.alternate = 12; // FMC

	GPIOInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 |
			GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit);

	GPIOInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
			GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit);

	// LCD_ON and LCD_RESET
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 0;

	GPIOReset(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

	GPIOInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, &gpioInit);

	// configure FMC
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	FMC_Bank1->BTCR[0] = FMC_BCR1_MWID_0 | FMC_BCR1_WREN | FMC_BCR1_WFDIS;
	FMC_Bank1->BTCR[1] = 0x0FFFFFFF; // use default settings, LCD PLL not set
	FMC_Bank1->BTCR[0] |= FMC_BCR1_MBKEN;

	regInit();

	TFTEnable(1);

	for(uint32_t i = 0; i < 800*480; i++)
		*TFT_DATA_REG = TFT_COLOR(0xFF0000);
}

void TFTReset(uint8_t reset)
{
	if(reset)
		GPIOReset(GPIOA, GPIO_PIN_9);
	else
		GPIOSet(GPIOA, GPIO_PIN_9);
}

void TFTEnable(uint8_t enable)
{
	if(enable)
		GPIOSet(GPIOA, GPIO_PIN_10);
	else
		GPIOReset(GPIOA, GPIO_PIN_10);
}

void TFTWritePixel(uint32_t color)
{
	*TFT_DATA_REG = TFT_COLOR(color);
}

void TFTSetBrightness(uint8_t level)
{
	// Set PWM config
	*TFT_CMD_REG = TFT_REG_SET_PWM_CONF;
	*TFT_DATA_REG = 0x0E;  // frequency
	*TFT_DATA_REG = level; //  PWM duty cycle
	*TFT_DATA_REG = 0x01;  //  PWM enabled & PWM controlled by host
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x00;
	*TFT_DATA_REG = 0x0F;
}
