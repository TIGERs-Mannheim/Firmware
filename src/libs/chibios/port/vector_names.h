/*
 * vector_names.h
 *
 *  Created on: 23.10.2015
 *      Author: AndreR
 */

#ifndef VECTOR_NAMES_H_
#define VECTOR_NAMES_H_

#ifdef STM32F4XX
#define Vector40   WWDG_IRQHandler
#define Vector44   PVD_IRQHandler
#define Vector48   TAMP_STAMP_IRQHandler
#define Vector4C   RTC_WKUP_IRQHandler
#define Vector50   FLASH_IRQHandler
#define Vector54   RCC_IRQHandler
#define Vector58   EXTI0_IRQHandler
#define Vector5C   EXTI1_IRQHandler
#define Vector60   EXTI2_IRQHandler
#define Vector64   EXTI3_IRQHandler
#define Vector68   EXTI4_IRQHandler
#define Vector6C   DMA1_Stream0_IRQHandler
#define Vector70   DMA1_Stream1_IRQHandler
#define Vector74   DMA1_Stream2_IRQHandler
#define Vector78   DMA1_Stream3_IRQHandler
#define Vector7C   DMA1_Stream4_IRQHandler
#define Vector80   DMA1_Stream5_IRQHandler
#define Vector84   DMA1_Stream6_IRQHandler
#define Vector88   ADC_IRQHandler
#define Vector8C   CAN1_TX_IRQHandler
#define Vector90   CAN1_RX0_IRQHandler
#define Vector94   CAN1_RX1_IRQHandler
#define Vector98   CAN1_SCE_IRQHandler
#define Vector9C   EXTI9_5_IRQHandler
#define VectorA0   TIM1_BRK_TIM9_IRQHandler
#define VectorA4   TIM1_UP_TIM10_IRQHandler
#define VectorA8   TIM1_TRG_COM_TIM11_IRQHandler
#define VectorAC   TIM1_CC_IRQHandler
#define VectorB0   TIM2_IRQHandler
#define VectorB4   TIM3_IRQHandler
#define VectorB8   TIM4_IRQHandler
#define VectorBC   I2C1_EV_IRQHandler
#define VectorC0   I2C1_ER_IRQHandler
#define VectorC4   I2C2_EV_IRQHandler
#define VectorC8   I2C2_ER_IRQHandler
#define VectorCC   SPI1_IRQHandler
#define VectorD0   SPI2_IRQHandler
#define VectorD4   USART1_IRQHandler
#define VectorD8   USART2_IRQHandler
#define VectorDC   USART3_IRQHandler
#define VectorE0   EXTI15_10_IRQHandler
#define VectorE4   RTC_Alarm_IRQHandler
#define VectorE8   OTG_FS_WKUP_IRQHandler
#define VectorEC   TIM8_BRK_TIM12_IRQHandler
#define VectorF0   TIM8_UP_TIM13_IRQHandler
#define VectorF4   TIM8_TRG_COM_TIM14_IRQHandler
#define VectorF8   TIM8_CC_IRQHandler
#define VectorFC   DMA1_Stream7_IRQHandler
#define Vector100  FSMC_IRQHandler
#define Vector104  SDIO_IRQHandler
#define Vector108  TIM5_IRQHandler
#define Vector10C  SPI3_IRQHandler
#define Vector110  UART4_IRQHandler
#define Vector114  UART5_IRQHandler
#define Vector118  TIM6_DAC_IRQHandler
#define Vector11C  TIM7_IRQHandler
#define Vector120  DMA2_Stream0_IRQHandler
#define Vector124  DMA2_Stream1_IRQHandler
#define Vector128  DMA2_Stream2_IRQHandler
#define Vector12C  DMA2_Stream3_IRQHandler
#define Vector130  DMA2_Stream4_IRQHandler
#define Vector134  ETH_IRQHandler
#define Vector138  ETH_WKUP_IRQHandler
#define Vector13C  CAN2_TX_IRQHandler
#define Vector140  CAN2_RX0_IRQHandler
#define Vector144  CAN2_RX1_IRQHandler
#define Vector148  CAN2_SCE_IRQHandler
#define Vector14C  OTG_FS_IRQHandler
#define Vector150  DMA2_Stream5_IRQHandler
#define Vector154  DMA2_Stream6_IRQHandler
#define Vector158  DMA2_Stream7_IRQHandler
#define Vector15C  USART6_IRQHandler
#define Vector160  I2C3_EV_IRQHandler
#define Vector164  I2C3_ER_IRQHandler
#define Vector168  OTG_HS_EP1_OUT_IRQHandler
#define Vector16C  OTG_HS_EP1_IN_IRQHandler
#define Vector170  OTG_HS_WKUP_IRQHandler
#define Vector174  OTG_HS_IRQHandler
#define Vector178  DCMI_IRQHandler
#define Vector17C  CRYP_IRQHandler
#define Vector180  HASH_RNG_IRQHandler
#define Vector184  FPU_IRQHandler
#endif

#ifdef STM32F30X
#define Vector40   WWDG_IRQHandler
#define Vector44   PVD_IRQHandler
#define Vector48   TAMPER_STAMP_IRQHandler
#define Vector4C   RTC_WKUP_IRQHandler
#define Vector50   FLASH_IRQHandler
#define Vector54   RCC_IRQHandler
#define Vector58   EXTI0_IRQHandler
#define Vector5C   EXTI1_IRQHandler
#define Vector60   EXTI2_TS_IRQHandler
#define Vector64   EXTI3_IRQHandler
#define Vector68   EXTI4_IRQHandler
#define Vector6C   DMA1_Channel1_IRQHandler
#define Vector70   DMA1_Channel2_IRQHandler
#define Vector74   DMA1_Channel3_IRQHandler
#define Vector78   DMA1_Channel4_IRQHandler
#define Vector7C   DMA1_Channel5_IRQHandler
#define Vector80   DMA1_Channel6_IRQHandler
#define Vector84   DMA1_Channel7_IRQHandler
#define Vector88   ADC1_2_IRQHandler
#define Vector8C   USB_HP_CAN1_TX_IRQHandler
#define Vector90   USB_LP_CAN1_RX0_IRQHandler
#define Vector94   CAN1_RX1_IRQHandler
#define Vector98   CAN1_SCE_IRQHandler
#define Vector9C   EXTI9_5_IRQHandler
#define VectorA0   TIM1_BRK_TIM15_IRQHandler
#define VectorA4   TIM1_UP_TIM16_IRQHandler
#define VectorA8   TIM1_TRG_COM_TIM17_IRQHandler
#define VectorAC   TIM1_CC_IRQHandler
#define VectorB0   TIM2_IRQHandler
#define VectorB4   TIM3_IRQHandler
#define VectorB8   TIM4_IRQHandler
#define VectorBC   I2C1_EV_IRQHandler
#define VectorC0   I2C1_ER_IRQHandler
#define VectorC4   I2C2_EV_IRQHandler
#define VectorC8   I2C2_ER_IRQHandler
#define VectorCC   SPI1_IRQHandler
#define VectorD0   SPI2_IRQHandler
#define VectorD4   USART1_IRQHandler
#define VectorD8   USART2_IRQHandler
#define VectorDC   USART3_IRQHandler
#define VectorE0   EXTI15_10_IRQHandler
#define VectorE4   RTC_Alarm_IRQHandler
#define VectorE8   USBWakeUp_IRQHandler
#define VectorEC   TIM8_BRK_IRQHandler
#define VectorF0   TIM8_UP_IRQHandler
#define VectorF4   TIM8_TRG_COM_IRQHandler
#define VectorF8   TIM8_CC_IRQHandler
#define VectorFC   ADC3_IRQHandler
#define Vector10C  SPI3_IRQHandler
#define Vector110  UART4_IRQHandler
#define Vector114  UART5_IRQHandler
#define Vector118  TIM6_DAC_IRQHandler
#define Vector11C  TIM7_IRQHandler
#define Vector120  DMA2_Channel1_IRQHandler
#define Vector124  DMA2_Channel2_IRQHandler
#define Vector128  DMA2_Channel3_IRQHandler
#define Vector12C  DMA2_Channel4_IRQHandler
#define Vector130  DMA2_Channel5_IRQHandler
#define Vector134  ADC4_IRQHandler
#define Vector140  COMP1_2_3_IRQHandler
#define Vector144  COMP4_5_6_IRQHandler
#define Vector148  COMP7_IRQHandler
#define Vector168  USB_HP_IRQHandler
#define Vector16C  USB_LP_IRQHandler
#define Vector170  USBWakeUp_RMP_IRQHandler
#define Vector184  FPU_IRQHandler
#endif

#ifdef STM32F7XX
#define Vector40   WWDG_IRQHandler
#define Vector44   PVD_IRQHandler
#define Vector48   TAMP_STAMP_IRQHandler
#define Vector4C   RTC_WKUP_IRQHandler
#define Vector50   FLASH_IRQHandler
#define Vector54   RCC_IRQHandler
#define Vector58   EXTI0_IRQHandler
#define Vector5C   EXTI1_IRQHandler
#define Vector60   EXTI2_IRQHandler
#define Vector64   EXTI3_IRQHandler
#define Vector68   EXTI4_IRQHandler
#define Vector6C   DMA1_Stream0_IRQHandler
#define Vector70   DMA1_Stream1_IRQHandler
#define Vector74   DMA1_Stream2_IRQHandler
#define Vector78   DMA1_Stream3_IRQHandler
#define Vector7C   DMA1_Stream4_IRQHandler
#define Vector80   DMA1_Stream5_IRQHandler
#define Vector84   DMA1_Stream6_IRQHandler
#define Vector88   ADC_IRQHandler
#define Vector8C   CAN1_TX_IRQHandler
#define Vector90   CAN1_RX0_IRQHandler
#define Vector94   CAN1_RX1_IRQHandler
#define Vector98   CAN1_SCE_IRQHandler
#define Vector9C   EXTI9_5_IRQHandler
#define VectorA0   TIM1_BRK_TIM9_IRQHandler
#define VectorA4   TIM1_UP_TIM10_IRQHandler
#define VectorA8   TIM1_TRG_COM_TIM11_IRQHandler
#define VectorAC   TIM1_CC_IRQHandler
#define VectorB0   TIM2_IRQHandler
#define VectorB4   TIM3_IRQHandler
#define VectorB8   TIM4_IRQHandler
#define VectorBC   I2C1_EV_IRQHandler
#define VectorC0   I2C1_ER_IRQHandler
#define VectorC4   I2C2_EV_IRQHandler
#define VectorC8   I2C2_ER_IRQHandler
#define VectorCC   SPI1_IRQHandler
#define VectorD0   SPI2_IRQHandler
#define VectorD4   USART1_IRQHandler
#define VectorD8   USART2_IRQHandler
#define VectorDC   USART3_IRQHandler
#define VectorE0   EXTI15_10_IRQHandler
#define VectorE4   RTC_Alarm_IRQHandler
#define VectorE8   OTG_FS_WKUP_IRQHandler
#define VectorEC   TIM8_BRK_TIM12_IRQHandler
#define VectorF0   TIM8_UP_TIM13_IRQHandler
#define VectorF4   TIM8_TRG_COM_TIM14_IRQHandler
#define VectorF8   TIM8_CC_IRQHandler
#define VectorFC   DMA1_Stream7_IRQHandler
#define Vector100  FSMC_IRQHandler
#define Vector104  SDMMC1_IRQHandler
#define Vector108  TIM5_IRQHandler
#define Vector10C  SPI3_IRQHandler
#define Vector110  UART4_IRQHandler
#define Vector114  UART5_IRQHandler
#define Vector118  TIM6_DAC_IRQHandler
#define Vector11C  TIM7_IRQHandler
#define Vector120  DMA2_Stream0_IRQHandler
#define Vector124  DMA2_Stream1_IRQHandler
#define Vector128  DMA2_Stream2_IRQHandler
#define Vector12C  DMA2_Stream3_IRQHandler
#define Vector130  DMA2_Stream4_IRQHandler
#define Vector134  ETH_IRQHandler
#define Vector138  ETH_WKUP_IRQHandler
#define Vector13C  CAN2_TX_IRQHandler
#define Vector140  CAN2_RX0_IRQHandler
#define Vector144  CAN2_RX1_IRQHandler
#define Vector148  CAN2_SCE_IRQHandler
#define Vector14C  OTG_FS_IRQHandler
#define Vector150  DMA2_Stream5_IRQHandler
#define Vector154  DMA2_Stream6_IRQHandler
#define Vector158  DMA2_Stream7_IRQHandler
#define Vector15C  USART6_IRQHandler
#define Vector160  I2C3_EV_IRQHandler
#define Vector164  I2C3_ER_IRQHandler
#define Vector168  OTG_HS_EP1_OUT_IRQHandler
#define Vector16C  OTG_HS_EP1_IN_IRQHandler
#define Vector170  OTG_HS_WKUP_IRQHandler
#define Vector174  OTG_HS_IRQHandler
#define Vector178  DCMI_IRQHandler
#define Vector17C  CRYP_IRQHandler
#define Vector180  HASH_RNG_IRQHandler
#define Vector184  FPU_IRQHandler
#define Vector188  UART7_IRQHandler
#define Vector18C  UART8_IRQHandler
#define Vector190  SPI4_IRQHandler
#define Vector194  SPI5_IRQHandler
#define Vector198  SPI6_IRQHandler
#define Vector19C  SAI1_IRQHandler
#define Vector1A0  LCD_TFT_IRQHandler
#define Vector1A4  LCD_TFT_ER_IRQHandler
#define Vector1A8  DMA2D_IRQHandler
#define Vector1AC  SAI2_IRQHandler
#define Vector1B0  QUAD_SPI_IRQHandler
#define Vector1B4  LP_TIMER1_IRQHandler
#define Vector1B8  HDMI_CEC_IRQHandler
#define Vector1BC  I2C4_EV_IRQHandler
#define Vector1C0  I2C4_ER_IRQHandler
#define Vector1C4  SPDIFRX_IRQHandler
#endif

#ifdef STM32H7XX
#define Vector40   WWDG_IRQHandler                   /* Window WatchDog              */
#define Vector44   PVD_AVD_IRQHandler                /* PVD/AVD through EXTI Line detection */
#define Vector48   TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
#define Vector4C   RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
#define Vector50   FLASH_IRQHandler                  /* FLASH                        */
#define Vector54   RCC_IRQHandler                    /* RCC                          */
#define Vector58   EXTI0_IRQHandler                  /* EXTI Line0                   */
#define Vector5C   EXTI1_IRQHandler                  /* EXTI Line1                   */
#define Vector60   EXTI2_IRQHandler                  /* EXTI Line2                   */
#define Vector64   EXTI3_IRQHandler                  /* EXTI Line3                   */
#define Vector68   EXTI4_IRQHandler                  /* EXTI Line4                   */
#define Vector6C   DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
#define Vector70   DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
#define Vector74   DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
#define Vector78   DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
#define Vector7C   DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
#define Vector80   DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
#define Vector84   DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
#define Vector88   ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
#define Vector8C   FDCAN1_IT0_IRQHandler             /* FDCAN1 interrupt line 0      */
#define Vector90   FDCAN2_IT0_IRQHandler             /* FDCAN2 interrupt line 0      */
#define Vector94   FDCAN1_IT1_IRQHandler             /* FDCAN1 interrupt line 1      */
#define Vector98   FDCAN2_IT1_IRQHandler             /* FDCAN2 interrupt line 1      */
#define Vector9C   EXTI9_5_IRQHandler                /* External Line[9:5]s          */
#define VectorA0   TIM1_BRK_IRQHandler               /* TIM1 Break interrupt         */
#define VectorA4   TIM1_UP_IRQHandler                /* TIM1 Update interrupt        */
#define VectorA8   TIM1_TRG_COM_IRQHandler           /* TIM1 Trigger and Commutation interrupt */
#define VectorAC   TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
#define VectorB0   TIM2_IRQHandler                   /* TIM2                         */
#define VectorB4   TIM3_IRQHandler                   /* TIM3                         */
#define VectorB8   TIM4_IRQHandler                   /* TIM4                         */
#define VectorBC   I2C1_EV_IRQHandler                /* I2C1 Event                   */
#define VectorC0   I2C1_ER_IRQHandler                /* I2C1 Error                   */
#define VectorC4   I2C2_EV_IRQHandler                /* I2C2 Event                   */
#define VectorC8   I2C2_ER_IRQHandler                /* I2C2 Error                   */
#define VectorCC   SPI1_IRQHandler                   /* SPI1                         */
#define VectorD0   SPI2_IRQHandler                   /* SPI2                         */
#define VectorD4   USART1_IRQHandler                 /* USART1                       */
#define VectorD8   USART2_IRQHandler                 /* USART2                       */
#define VectorDC   USART3_IRQHandler                 /* USART3                       */
#define VectorE0   EXTI15_10_IRQHandler              /* External Line[15:10]s        */
#define VectorE4   RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
//#define VectorE8   0                                 /* Reserved                     */
#define VectorEC   TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
#define VectorF0   TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
#define VectorF4   TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
#define VectorF8   TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
#define VectorFC   DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
#define Vector100  FMC_IRQHandler                    /* FMC                          */
#define Vector104  SDMMC1_IRQHandler                 /* SDMMC1                       */
#define Vector108  TIM5_IRQHandler                   /* TIM5                         */
#define Vector10C  SPI3_IRQHandler                   /* SPI3                         */
#define Vector110  UART4_IRQHandler                  /* UART4                        */
#define Vector114  UART5_IRQHandler                  /* UART5                        */
#define Vector118  TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
#define Vector11C  TIM7_IRQHandler                   /* TIM7                         */
#define Vector120  DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
#define Vector124  DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
#define Vector128  DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
#define Vector12C  DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
#define Vector130  DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
#define Vector134  ETH_IRQHandler                    /* Ethernet                     */
#define Vector138  ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
#define Vector13C  FDCAN_CAL_IRQHandler              /* FDCAN calibration unit interrupt*/
//#define Vector140  0                                 /* Reserved                     */
//#define Vector144  0                                 /* Reserved                     */
//#define Vector148  0                                 /* Reserved                     */
//#define Vector14C  0                                 /* Reserved                     */
#define Vector150  DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
#define Vector154  DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
#define Vector158  DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
#define Vector15C  USART6_IRQHandler                 /* USART6                       */
#define Vector160  I2C3_EV_IRQHandler                /* I2C3 event                   */
#define Vector164  I2C3_ER_IRQHandler                /* I2C3 error                   */
#define Vector168  OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
#define Vector16C  OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
#define Vector170  OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
#define Vector174  OTG_HS_IRQHandler                 /* USB OTG HS                   */
#define Vector178  DCMI_IRQHandler                   /* DCMI                         */
//#define Vector17C  0                                 /* Reserved                     */
#define Vector180  RNG_IRQHandler                    /* Rng                          */
#define Vector184  FPU_IRQHandler                    /* FPU                          */
#define Vector188  UART7_IRQHandler                  /* UART7                        */
#define Vector18C  UART8_IRQHandler                  /* UART8                        */
#define Vector190  SPI4_IRQHandler                   /* SPI4                         */
#define Vector194  SPI5_IRQHandler                   /* SPI5                         */
#define Vector198  SPI6_IRQHandler                   /* SPI6                         */
#define Vector19C  SAI1_IRQHandler                   /* SAI1                         */
#define Vector1A0  LTDC_IRQHandler                   /* LTDC                         */
#define Vector1A4  LTDC_ER_IRQHandler                /* LTDC error                   */
#define Vector1A8  DMA2D_IRQHandler                  /* DMA2D                        */
#define Vector1AC  SAI2_IRQHandler                   /* SAI2                         */
#define Vector1B0  QUADSPI_IRQHandler                /* QUADSPI                      */
#define Vector1B4  LPTIM1_IRQHandler                 /* LPTIM1                       */
#define Vector1B8  CEC_IRQHandler                    /* HDMI_CEC                     */
#define Vector1BC  I2C4_EV_IRQHandler                /* I2C4 Event                   */
#define Vector1C0  I2C4_ER_IRQHandler                /* I2C4 Error                   */
#define Vector1C4  SPDIF_RX_IRQHandler               /* SPDIF_RX                     */
#define Vector1C8  OTG_FS_EP1_OUT_IRQHandler         /* USB OTG FS End Point 1 Out   */
#define Vector1CC  OTG_FS_EP1_IN_IRQHandler          /* USB OTG FS End Point 1 In    */
#define Vector1D0  OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI */
#define Vector1D4  OTG_FS_IRQHandler                 /* USB OTG FS                   */
#define Vector1D8  DMAMUX1_OVR_IRQHandler            /* DMAMUX1 Overrun interrupt    */
#define Vector1DC  HRTIM1_Master_IRQHandler          /* HRTIM Master Timer global Interrupt */
#define Vector1E0  HRTIM1_TIMA_IRQHandler            /* HRTIM Timer A global Interrupt */
#define Vector1E4  HRTIM1_TIMB_IRQHandler            /* HRTIM Timer B global Interrupt */
#define Vector1E8  HRTIM1_TIMC_IRQHandler            /* HRTIM Timer C global Interrupt */
#define Vector1EC  HRTIM1_TIMD_IRQHandler            /* HRTIM Timer D global Interrupt */
#define Vector1F0  HRTIM1_TIME_IRQHandler            /* HRTIM Timer E global Interrupt */
#define Vector1F4  HRTIM1_FLT_IRQHandler             /* HRTIM Fault global Interrupt   */
#define Vector1F8  DFSDM1_FLT0_IRQHandler            /* DFSDM Filter0 Interrupt        */
#define Vector1FC  DFSDM1_FLT1_IRQHandler            /* DFSDM Filter1 Interrupt        */
#define Vector200  DFSDM1_FLT2_IRQHandler            /* DFSDM Filter2 Interrupt        */
#define Vector204  DFSDM1_FLT3_IRQHandler            /* DFSDM Filter3 Interrupt        */
#define Vector208  SAI3_IRQHandler                   /* SAI3 global Interrupt          */
#define Vector20C  SWPMI1_IRQHandler                 /* Serial Wire Interface 1 global interrupt */
#define Vector210  TIM15_IRQHandler                  /* TIM15 global Interrupt      */
#define Vector214  TIM16_IRQHandler                  /* TIM16 global Interrupt      */
#define Vector218  TIM17_IRQHandler                  /* TIM17 global Interrupt      */
#define Vector21C  MDIOS_WKUP_IRQHandler             /* MDIOS Wakeup  Interrupt     */
#define Vector220  MDIOS_IRQHandler                  /* MDIOS global Interrupt      */
#define Vector224  JPEG_IRQHandler                   /* JPEG global Interrupt       */
#define Vector228  MDMA_IRQHandler                   /* MDMA global Interrupt       */
//#define Vector22C  0                                 /* Reserved                    */
#define Vector230  SDMMC2_IRQHandler                 /* SDMMC2 global Interrupt     */
#define Vector234  HSEM1_IRQHandler                  /* HSEM1 global Interrupt      */
//#define Vector238  0                                 /* Reserved                    */
#define Vector23C  ADC3_IRQHandler                   /* ADC3 global Interrupt       */
#define Vector240  DMAMUX2_OVR_IRQHandler            /* DMAMUX Overrun interrupt    */
#define Vector244  BDMA_Channel0_IRQHandler          /* BDMA Channel 0 global Interrupt */
#define Vector248  BDMA_Channel1_IRQHandler          /* BDMA Channel 1 global Interrupt */
#define Vector24C  BDMA_Channel2_IRQHandler          /* BDMA Channel 2 global Interrupt */
#define Vector250  BDMA_Channel3_IRQHandler          /* BDMA Channel 3 global Interrupt */
#define Vector254  BDMA_Channel4_IRQHandler          /* BDMA Channel 4 global Interrupt */
#define Vector258  BDMA_Channel5_IRQHandler          /* BDMA Channel 5 global Interrupt */
#define Vector25C  BDMA_Channel6_IRQHandler          /* BDMA Channel 6 global Interrupt */
#define Vector260  BDMA_Channel7_IRQHandler          /* BDMA Channel 7 global Interrupt */
#define Vector264  COMP1_IRQHandler                  /* COMP1 global Interrupt     */
#define Vector268  LPTIM2_IRQHandler                 /* LP TIM2 global interrupt   */
#define Vector26C  LPTIM3_IRQHandler                 /* LP TIM3 global interrupt   */
#define Vector270  LPTIM4_IRQHandler                 /* LP TIM4 global interrupt   */
#define Vector274  LPTIM5_IRQHandler                 /* LP TIM5 global interrupt   */
#define Vector278  LPUART1_IRQHandler                /* LP UART1 interrupt         */
#define Vector27C  WWDG_RST_IRQHandler               /* Reserved                   */
#define Vector280  CRS_IRQHandler                    /* Clock Recovery Global Interrupt */
#define Vector284  RAMECC_IRQHandler                 /* Reserved                   */
#define Vector288  SAI4_IRQHandler                   /* SAI4 global interrupt      */
//#define Vector28C  0                                 /* Reserved                   */
//#define Vector290  0                                 /* Reserved                   */
#define Vector294  WAKEUP_PIN_IRQHandler             /* Interrupt for all 6 wake-up pins */
#endif

#endif /* VECTOR_NAMES_H_ */
