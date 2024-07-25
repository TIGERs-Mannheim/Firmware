/*
 * init_hal.h
 *
 *  Created on: 30.07.2014
 *      Author: AndreR
 */

#ifndef INIT_HAL_H_
#define INIT_HAL_H_

#include "cmparams.h"

#if !defined(USART_ISR_TXE_TXFNF) && defined(USART_ISR_TXE)
#define USART_ISR_TXE_TXFNF USART_ISR_TXE
#endif

#if !defined(USART_ISR_RXNE_RXFNE) && defined(USART_ISR_RXNE)
#define USART_ISR_RXNE_RXFNE USART_ISR_RXNE
#endif

#if !defined(GPIO_MODER_MODE0) && defined(GPIO_MODER_MODER0)
#define GPIO_MODER_MODE0 GPIO_MODER_MODER0
#endif

#if !defined(GPIO_OSPEEDR_OSPEED0) && defined(GPIO_OSPEEDR_OSPEEDR0)
#define GPIO_OSPEEDR_OSPEED0 GPIO_OSPEEDR_OSPEEDR0
#endif

#if !defined(GPIO_PUPDR_PUPD0) && defined(GPIO_PUPDR_PUPDR0)
#define GPIO_PUPDR_PUPD0 GPIO_PUPDR_PUPDR0
#endif

//#################### GPIO ###################
#define GPIO_PIN_0			0x0001
#define GPIO_PIN_1			0x0002
#define GPIO_PIN_2			0x0004
#define GPIO_PIN_3			0x0008
#define GPIO_PIN_4			0x0010
#define GPIO_PIN_5			0x0020
#define GPIO_PIN_6			0x0040
#define GPIO_PIN_7			0x0080
#define GPIO_PIN_8			0x0100
#define GPIO_PIN_9			0x0200
#define GPIO_PIN_10			0x0400
#define GPIO_PIN_11			0x0800
#define GPIO_PIN_12			0x1000
#define GPIO_PIN_13			0x2000
#define GPIO_PIN_14			0x4000
#define GPIO_PIN_15			0x8000

#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_EXTI		4
#define GPIO_MODE_EXTI_AF	5

#define GPIO_OTYPE_PUSH_PULL	0
#define GPIO_OTYPE_OPEN_DRAIN	1

#ifdef STM32F30X
#define GPIO_OSPEED_F3_2MHZ		0
#define GPIO_OSPEED_F3_10MHZ	1
#define GPIO_OSPEED_F3_50MHZ	3
#endif

#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
#define GPIO_OSPEED_2MHZ	0
#define GPIO_OSPEED_25MHZ	1
#define GPIO_OSPEED_50MHZ	2
#define GPIO_OSPEED_100MHZ	3	// active compensation cell!
#endif

#define GPIO_PUPD_NONE		0
#define GPIO_PUPD_UP		1
#define GPIO_PUPD_DOWN		2

#define GPIO_EXTI_TRIG_RISING 	0x01
#define GPIO_EXTI_TRIG_FALLING	0x02
#define GPIO_EXTI_TRIG_RISING_FALLING	(GPIO_EXTI_TRIG_RISING | GPIO_EXTI_TRIG_FALLING)

typedef struct _GPIOPin
{
	GPIO_TypeDef* pPort;
	uint16_t pin;
} GPIOPin;

typedef struct _GPIOPinAlt
{
	GPIO_TypeDef* pPort;
	uint16_t pin;
	uint16_t alternate;
} GPIOPinAlt;

typedef struct _GPIOInit
{
	uint32_t mode;
	uint32_t otype;
	uint32_t ospeed;	// not used in input and analog mode
	uint32_t pupd;
	uint32_t alternate;
	uint32_t extiTrigger;
} GPIOInitData;

void GPIOInit(GPIO_TypeDef* pGPIO, uint16_t pins, GPIOInitData* pInit);
void GPIODeInit(GPIO_TypeDef* pGPIO, uint16_t pins);
#define GPIOSet(pGPIO, mask) (pGPIO->BSRR = ((mask) & 0xFFFF))
#define GPIOReset(pGPIO, mask) (pGPIO->BSRR = (((uint32_t)mask) << 16))
#define GPIOPinSet(gpio) (GPIOSet(gpio.pPort, gpio.pin))
#define GPIOPinReset(gpio) (GPIOReset(gpio.pPort, gpio.pin))
#define GPIOPinIsSet(gpio) (((gpio.pPort->BSRR & gpio.pin) != 0) ? 1 : 0)
#define GPIOPinRead(gpio) (((gpio.pPort->IDR & gpio.pin) != 0) ? 1 : 0)

//#################### DMA ###################
#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
#define DMA_MBURST_SINGLE	0
#define DMA_MBURST_INCR4	DMA_SxCR_MBURST_0
#define DMA_MBURST_INCR8	DMA_SxCR_MBURST_1
#define DMA_MBURST_INCR16	(DMA_SxCR_MBURST_0 | DMA_SxCR_MBURST_1)

#define DMA_PBURST_SINGLE	0
#define DMA_PBURST_INCR4	DMA_SxCR_PBURST_0
#define DMA_PBURST_INCR8	DMA_SxCR_PBURST_1
#define DMA_PBURST_INCR16	(DMA_SxCR_PBURST_0 | DMA_SxCR_PBURST_1)

#define DMA_PL_LOW			0
#define DMA_PL_MEDIUM		DMA_SxCR_PL_0
#define DMA_PL_HIGH			DMA_SxCR_PL_1
#define DMA_PL_VERY_HIGH	(DMA_SxCR_PL_0 | DMA_SxCR_PL_1)

#define DMA_MSIZE_1BYTE		0
#define DMA_MSIZE_2BYTE		DMA_SxCR_MSIZE_0
#define DMA_MSIZE_4BYTE		DMA_SxCR_MSIZE_1

#define DMA_PSIZE_1BYTE		0
#define DMA_PSIZE_2BYTE		DMA_SxCR_PSIZE_0
#define DMA_PSIZE_4BYTE		DMA_SxCR_PSIZE_1

#define DMA_DIR_PERIPH2MEM	0
#define DMA_DIR_MEM2PERIPH	DMA_SxCR_DIR_0
#define DMA_DIR_MEM2MEM		DMA_SxCR_DIR_1

#define DMA_FTH_1QUARTER	0
#define DMA_FTH_HALF		DMA_SxFCR_FTH_0
#define DMA_FTH_3QUARTER	DMA_SxFCR_FTH_1
#define DMA_FTH_FULL		(DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1)

#ifdef STM32H7XX
#define DMA_CHSEL(ch)		(0)
#else
#define DMA_CHSEL(ch)		(ch << 25)
#endif

/**
 * pDMA: DMA1 or DMA2
 * stream: 0 - 7
 */
DMA_Stream_TypeDef* DMAGenerateStreamTypeDef(DMA_TypeDef* pDMA, uint32_t stream);
volatile uint32_t* DMAGenerateIFCR(DMA_TypeDef* pDMA, uint32_t stream);
volatile uint32_t* DMAGenerateISR(DMA_TypeDef* pDMA, uint32_t stream);
uint32_t DMAGenerateMask(uint32_t flags, uint32_t stream);

#define DMAGenerateMaskAll(s)			DMAGenerateMask(0x3D, s)
#define DMAGenerateMaskTCIF(s)			DMAGenerateMask(0x20, s)
#define DMAGenerateMaskTEIForFEIF(s)	DMAGenerateMask(0x09, s)

#else

#define DMA_PL_LOW			0
#define DMA_PL_MEDIUM		DMA_CCR_PL_0
#define DMA_PL_HIGH			DMA_CCR_PL_1
#define DMA_PL_VERY_HIGH	(DMA_CCR_PL_0 | DMA_CCR_PL_1)

#define DMA_MSIZE_1BYTE		0
#define DMA_MSIZE_2BYTE		DMA_CCR_MSIZE_0
#define DMA_MSIZE_4BYTE		DMA_CCR_MSIZE_1

#define DMA_PSIZE_1BYTE		0
#define DMA_PSIZE_2BYTE		DMA_CCR_PSIZE_0
#define DMA_PSIZE_4BYTE		DMA_CCR_PSIZE_1

#define DMA_DIR_PERIPH2MEM	0
#define DMA_DIR_MEM2PERIPH	DMA_CCR_DIR

#endif

//#################### NVIC ###################
void NVICEnableIRQ(uint32_t irqn, uint32_t irql);

//#################### USART ###################
uint16_t USARTCalcBRR(uint32_t baudRate, uint32_t peripheralClock, uint8_t over8);

//#################### SYSCLK ###################
typedef struct _SystemClockInfo
{
	uint32_t SYSClk;
	uint32_t APB1PeriphClk;
	uint32_t APB1TimerClk;
	uint32_t APB2PeriphClk;
	uint32_t APB2TimerClk;
#ifdef STM32H7XX
	uint32_t APB3PeriphClk;
	uint32_t H3Clk;
	uint32_t PLL1QClk;
#endif
} SystemClockInfo;

extern SystemClockInfo systemClockInfo;

#ifndef STM32H7XX
typedef struct _SystemClockInitData
{
#if defined(STM32F4XX) || defined(STM32F7XX)
	struct _pll
	{
		uint32_t M;
		uint32_t N;
		uint32_t P;
		uint32_t Q;
	} pll;

	uint32_t RTCDiv;	// must be 1MHz
#else
	uint32_t pllMul;
#endif

	uint32_t APB1Div;	// 42MHz max
	uint32_t APB2Div;	// 84MHz max
	uint32_t flashLatency;
	uint8_t HSEBypass;
	uint32_t sysTickFreq;
} SystemClockInitData;


void SystemClockInit(SystemClockInitData* pInit, uint32_t sysClk);
#endif

#ifdef STM32F7XX
static inline void SystemCleanInvalidateDCache(void* pData, uint32_t numBytes)
{
	uint32_t buffAddr = (uint32_t)pData;
	buffAddr &= 0xFFFFFFE0;	// align to 32 byte boundary

	// flush cache lines that will be updated in a moment by DMA
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buffAddr, numBytes+32);
}
#endif

uint32_t chThdGetFreeStack(void* pThread);

//#################### SPI ###################
#ifdef STM32H7XX
#define SPI_CFG1_BRDIV2		(0 << 28)
#define SPI_CFG1_BRDIV4		(1 << 28)
#define SPI_CFG1_BRDIV8		(2 << 28)
#define SPI_CFG1_BRDIV16	(3 << 28)
#define SPI_CFG1_BRDIV32	(4 << 28)
#define SPI_CFG1_BRDIV64	(5 << 28)
#define SPI_CFG1_BRDIV128	(6 << 28)
#define SPI_CFG1_BRDIV256	(7 << 28)
#else
#define SPI_CR1_BRDIV2		(0 << 3)
#define SPI_CR1_BRDIV4		(1 << 3)
#define SPI_CR1_BRDIV8		(2 << 3)
#define SPI_CR1_BRDIV16		(3 << 3)
#define SPI_CR1_BRDIV32		(4 << 3)
#define SPI_CR1_BRDIV64		(5 << 3)
#define SPI_CR1_BRDIV128	(6 << 3)
#define SPI_CR1_BRDIV256	(7 << 3)
#endif

#ifndef STM32F407xx
#define ADC_SampleTime_1Cycles5                    ((uint8_t)0x00)   /*!<  ADC sampling time 1.5 cycle */
#define ADC_SampleTime_2Cycles5                    ((uint8_t)0x01)   /*!<  ADC sampling time 2.5 cycles */
#define ADC_SampleTime_4Cycles5                    ((uint8_t)0x02)   /*!<  ADC sampling time 4.5 cycles */
#define ADC_SampleTime_7Cycles5                    ((uint8_t)0x03)   /*!<  ADC sampling time 7.5 cycles */
#define ADC_SampleTime_19Cycles5                   ((uint8_t)0x04)   /*!<  ADC sampling time 19.5 cycles */
#define ADC_SampleTime_61Cycles5                   ((uint8_t)0x05)   /*!<  ADC sampling time 61.5 cycles */
#define ADC_SampleTime_181Cycles5                  ((uint8_t)0x06)   /*!<  ADC sampling time 181.5 cycles */
#define ADC_SampleTime_601Cycles5                  ((uint8_t)0x07)   /*!<  ADC sampling time 601.5 cycles */

// Rank = sequence position: 1-16
// ADC_Channel: 1-18
void ADCRegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
#endif

//#################### ETH ###################
#define ETH_DMATXDESC_CHECKSUMBYPASS             ((uint32_t)0x00000000)   /*!< Checksum engine bypass */
#define ETH_DMATXDESC_CHECKSUMIPV4HEADER         ((uint32_t)0x00400000)   /*!< IPv4 header checksum insertion  */
#define ETH_DMATXDESC_CHECKSUMTCPUDPICMPSEGMENT  ((uint32_t)0x00800000)   /*!< TCP/UDP/ICMP checksum insertion. Pseudo header checksum is assumed to be present */
#define ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL     ((uint32_t)0x00C00000)   /*!< TCP/UDP/ICMP checksum fully in hardware including pseudo header */

/*
   DMA Tx Desciptor
  -----------------------------------------------------------------------------------------------
  TDES0 | OWN(31) | CTRL[30:26] | Reserved[25:24] | CTRL[23:20] | Reserved[19:17] | Status[16:0] |
  -----------------------------------------------------------------------------------------------
  TDES1 | Reserved[31:29] | Buffer2 ByteCount[28:16] | Reserved[15:13] | Buffer1 ByteCount[12:0] |
  -----------------------------------------------------------------------------------------------
  TDES2 |                         Buffer1 Address [31:0]                                         |
  -----------------------------------------------------------------------------------------------
  TDES3 |                   Buffer2 Address [31:0] / Next Descriptor Address [31:0]              |
  -----------------------------------------------------------------------------------------------
*/

/**
  * @brief  Bit definition of TDES0 register: DMA Tx descriptor status register
  */
#define ETH_DMATXDESC_OWN                     ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMATXDESC_IC                      ((uint32_t)0x40000000)  /*!< Interrupt on Completion */
#define ETH_DMATXDESC_LS                      ((uint32_t)0x20000000)  /*!< Last Segment */
#define ETH_DMATXDESC_FS                      ((uint32_t)0x10000000)  /*!< First Segment */
#define ETH_DMATXDESC_DC                      ((uint32_t)0x08000000)  /*!< Disable CRC */
#define ETH_DMATXDESC_DP                      ((uint32_t)0x04000000)  /*!< Disable Padding */
#define ETH_DMATXDESC_TTSE                    ((uint32_t)0x02000000)  /*!< Transmit Time Stamp Enable */
#define ETH_DMATXDESC_CIC                     ((uint32_t)0x00C00000)  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATXDESC_CIC_BYPASS              ((uint32_t)0x00000000)  /*!< Do Nothing: Checksum Engine is bypassed */
#define ETH_DMATXDESC_CIC_IPV4HEADER          ((uint32_t)0x00400000)  /*!< IPV4 header Checksum Insertion */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_SEGMENT  ((uint32_t)0x00800000)  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_FULL     ((uint32_t)0x00C00000)  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */
#define ETH_DMATXDESC_TER                     ((uint32_t)0x00200000)  /*!< Transmit End of Ring */
#define ETH_DMATXDESC_TCH                     ((uint32_t)0x00100000)  /*!< Second Address Chained */
#define ETH_DMATXDESC_TTSS                    ((uint32_t)0x00020000)  /*!< Tx Time Stamp Status */
#define ETH_DMATXDESC_IHE                     ((uint32_t)0x00010000)  /*!< IP Header Error */
#define ETH_DMATXDESC_ES                      ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATXDESC_JT                      ((uint32_t)0x00004000)  /*!< Jabber Timeout */
#define ETH_DMATXDESC_FF                      ((uint32_t)0x00002000)  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATXDESC_PCE                     ((uint32_t)0x00001000)  /*!< Payload Checksum Error */
#define ETH_DMATXDESC_LCA                     ((uint32_t)0x00000800)  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATXDESC_NC                      ((uint32_t)0x00000400)  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATXDESC_LCO                     ((uint32_t)0x00000200)  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATXDESC_EC                      ((uint32_t)0x00000100)  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATXDESC_VF                      ((uint32_t)0x00000080)  /*!< VLAN Frame */
#define ETH_DMATXDESC_CC                      ((uint32_t)0x00000078)  /*!< Collision Count */
#define ETH_DMATXDESC_ED                      ((uint32_t)0x00000004)  /*!< Excessive Deferral */
#define ETH_DMATXDESC_UF                      ((uint32_t)0x00000002)  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATXDESC_DB                      ((uint32_t)0x00000001)  /*!< Deferred Bit */

/**
  * @brief  Bit definition of TDES1 register
  */
#define ETH_DMATXDESC_TBS2  ((uint32_t)0x1FFF0000)  /*!< Transmit Buffer2 Size */
#define ETH_DMATXDESC_TBS1  ((uint32_t)0x00001FFF)  /*!< Transmit Buffer1 Size */

/**
  * @brief  Bit definition of TDES2 register
  */
#define ETH_DMATXDESC_B1AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer1 Address Pointer */

/**
  * @brief  Bit definition of TDES3 register
  */
#define ETH_DMATXDESC_B2AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer2 Address Pointer */

  /*---------------------------------------------------------------------------------------------
  TDES6 |                         Transmit Time Stamp Low [31:0]                                 |
  -----------------------------------------------------------------------------------------------
  TDES7 |                         Transmit Time Stamp High [31:0]                                |
  ----------------------------------------------------------------------------------------------*/

/* Bit definition of TDES6 register */
 #define ETH_DMAPTPTXDESC_TTSL  ((uint32_t)0xFFFFFFFF)  /* Transmit Time Stamp Low */

/* Bit definition of TDES7 register */
 #define ETH_DMAPTPTXDESC_TTSH  ((uint32_t)0xFFFFFFFF)  /* Transmit Time Stamp High */

/**
  * @}
  */


/** @defgroup ETH_DMA_Rx_descriptor
  * @{
  */

/*
  DMA Rx Descriptor
  --------------------------------------------------------------------------------------------------------------------
  RDES0 | OWN(31) |                                             Status [30:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES1 | CTRL(31) | Reserved[30:29] | Buffer2 ByteCount[28:16] | CTRL[15:14] | Reserved(13) | Buffer1 ByteCount[12:0] |
  ---------------------------------------------------------------------------------------------------------------------
  RDES2 |                                       Buffer1 Address [31:0]                                                 |
  ---------------------------------------------------------------------------------------------------------------------
  RDES3 |                          Buffer2 Address [31:0] / Next Descriptor Address [31:0]                             |
  ---------------------------------------------------------------------------------------------------------------------
*/

/**
  * @brief  Bit definition of RDES0 register: DMA Rx descriptor status register
  */
#define ETH_DMARXDESC_OWN         ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARXDESC_AFM         ((uint32_t)0x40000000)  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARXDESC_FL          ((uint32_t)0x3FFF0000)  /*!< Receive descriptor frame length  */
#define ETH_DMARXDESC_ES          ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARXDESC_DE          ((uint32_t)0x00004000)  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARXDESC_SAF         ((uint32_t)0x00002000)  /*!< SA Filter Fail for the received frame */
#define ETH_DMARXDESC_LE          ((uint32_t)0x00001000)  /*!< Frame size not matching with length field */
#define ETH_DMARXDESC_OE          ((uint32_t)0x00000800)  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARXDESC_VLAN        ((uint32_t)0x00000400)  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARXDESC_FS          ((uint32_t)0x00000200)  /*!< First descriptor of the frame  */
#define ETH_DMARXDESC_LS          ((uint32_t)0x00000100)  /*!< Last descriptor of the frame  */
#define ETH_DMARXDESC_IPV4HCE     ((uint32_t)0x00000080)  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */
#define ETH_DMARXDESC_LC          ((uint32_t)0x00000040)  /*!< Late collision occurred during reception   */
#define ETH_DMARXDESC_FT          ((uint32_t)0x00000020)  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARXDESC_RWT         ((uint32_t)0x00000010)  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARXDESC_RE          ((uint32_t)0x00000008)  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARXDESC_DBE         ((uint32_t)0x00000004)  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARXDESC_CE          ((uint32_t)0x00000002)  /*!< CRC error */
#define ETH_DMARXDESC_MAMPCE      ((uint32_t)0x00000001)  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

/**
  * @brief  Bit definition of RDES1 register
  */
#define ETH_DMARXDESC_DIC   ((uint32_t)0x80000000)  /*!< Disable Interrupt on Completion */
#define ETH_DMARXDESC_RBS2  ((uint32_t)0x1FFF0000)  /*!< Receive Buffer2 Size */
#define ETH_DMARXDESC_RER   ((uint32_t)0x00008000)  /*!< Receive End of Ring */
#define ETH_DMARXDESC_RCH   ((uint32_t)0x00004000)  /*!< Second Address Chained */
#define ETH_DMARXDESC_RBS1  ((uint32_t)0x00001FFF)  /*!< Receive Buffer1 Size */

/**
  * @brief  Bit definition of RDES2 register
  */
#define ETH_DMARXDESC_B1AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer1 Address Pointer */

/**
  * @brief  Bit definition of RDES3 register
  */
#define ETH_DMARXDESC_B2AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer2 Address Pointer */

/*---------------------------------------------------------------------------------------------------------------------
  RDES4 |                   Reserved[31:15]              |             Extended Status [14:0]                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES5 |                                            Reserved[31:0]                                                    |
  ---------------------------------------------------------------------------------------------------------------------
  RDES6 |                                       Receive Time Stamp Low [31:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES7 |                                       Receive Time Stamp High [31:0]                                         |
  --------------------------------------------------------------------------------------------------------------------*/

/* Bit definition of RDES4 register */
#define ETH_DMAPTPRXDESC_PTPV     ((uint32_t)0x00002000)  /* PTP Version */
#define ETH_DMAPTPRXDESC_PTPFT    ((uint32_t)0x00001000)  /* PTP Frame Type */
#define ETH_DMAPTPRXDESC_PTPMT    ((uint32_t)0x00000F00)  /* PTP Message Type */
#define ETH_DMAPTPRXDESC_PTPMT_SYNC                      ((uint32_t)0x00000100)  /* SYNC message (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_FOLLOWUP                  ((uint32_t)0x00000200)  /* FollowUp message (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_DELAYREQ                  ((uint32_t)0x00000300)  /* DelayReq message (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_DELAYRESP                 ((uint32_t)0x00000400)  /* DelayResp message (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_PDELAYREQ_ANNOUNCE        ((uint32_t)0x00000500)  /* PdelayReq message (peer-to-peer transparent clock) or Announce message (Ordinary or Boundary clock) */
#define ETH_DMAPTPRXDESC_PTPMT_PDELAYRESP_MANAG          ((uint32_t)0x00000600)  /* PdelayResp message (peer-to-peer transparent clock) or Management message (Ordinary or Boundary clock)  */
#define ETH_DMAPTPRXDESC_PTPMT_PDELAYRESPFOLLOWUP_SIGNAL ((uint32_t)0x00000700)  /* PdelayRespFollowUp message (peer-to-peer transparent clock) or Signaling message (Ordinary or Boundary clock) */
#define ETH_DMAPTPRXDESC_IPV6PR   ((uint32_t)0x00000080)  /* IPv6 Packet Received */
#define ETH_DMAPTPRXDESC_IPV4PR   ((uint32_t)0x00000040)  /* IPv4 Packet Received */
#define ETH_DMAPTPRXDESC_IPCB  ((uint32_t)0x00000020)  /* IP Checksum Bypassed */
#define ETH_DMAPTPRXDESC_IPPE  ((uint32_t)0x00000010)  /* IP Payload Error */
#define ETH_DMAPTPRXDESC_IPHE  ((uint32_t)0x00000008)  /* IP Header Error */
#define ETH_DMAPTPRXDESC_IPPT  ((uint32_t)0x00000007)  /* IP Payload Type */
#define ETH_DMAPTPRXDESC_IPPT_UDP                 ((uint32_t)0x00000001)  /* UDP payload encapsulated in the IP datagram */
#define ETH_DMAPTPRXDESC_IPPT_TCP                 ((uint32_t)0x00000002)  /* TCP payload encapsulated in the IP datagram */
#define ETH_DMAPTPRXDESC_IPPT_ICMP                ((uint32_t)0x00000003)  /* ICMP payload encapsulated in the IP datagram */

/* Bit definition of RDES6 register */
#define ETH_DMAPTPRXDESC_RTSL  ((uint32_t)0xFFFFFFFF)  /* Receive Time Stamp Low */

/* Bit definition of RDES7 register */
#define ETH_DMAPTPRXDESC_RTSH  ((uint32_t)0xFFFFFFFF)  /* Receive Time Stamp High */

/* ETHERNET MAC address offsets */
#define ETH_MAC_ADDR_HBASE    (uint32_t)(ETH_MAC_BASE + (uint32_t)0x40)  /* ETHERNET MAC address high offset */
#define ETH_MAC_ADDR_LBASE    (uint32_t)(ETH_MAC_BASE + (uint32_t)0x44)  /* ETHERNET MAC address low offset */


#endif /* INIT_HAL_H_ */
