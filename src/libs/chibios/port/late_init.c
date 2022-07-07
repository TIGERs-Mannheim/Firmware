/*
 * late_init.c
 *
 *  Created on: 23.10.2015
 *      Author: AndreR
 */

#include <stdint.h>
#include "ch.h"

// Setting up CCM, ITCM and DTCM here.

#if defined(STM32F7XX) || defined(STM32H7XX)
extern uint32_t _dtc_bss_start;
extern uint32_t _dtc_bss_end;

extern uint32_t _dtc_data_start;
extern uint32_t _dtc_data_end;
extern uint32_t _dtc_data_load;

extern uint32_t _itc_text_start;
extern uint32_t _itc_text_end;
extern uint32_t _itc_text_load;

extern uint32_t __main_stack_base__;
extern uint32_t __process_stack_end__;
#else
extern uint32_t _cmm_start;
extern uint32_t _cmm_end;

extern uint32_t _ccmtext_start;
extern uint32_t _ccmtext_end;
extern uint32_t _ccmtext_data;
#endif

void __late_init()
{
#if defined(STM32F7XX) || defined(STM32H7XX)
	{
		uint32_t *tp, *dp;
		tp = &_dtc_data_load;
		dp = &_dtc_data_start;
		while(dp < &_dtc_data_end)
			*dp++ = *tp++;
	}

	{
		uint32_t *tp, *dp;
		tp = &_itc_text_load;
		dp = &_itc_text_start;
		while(dp < &_itc_text_end)
			*dp++ = *tp++;
	}

	{
		uint32_t *dp;
		dp = &_dtc_bss_start;
		while(dp < &_dtc_bss_end)
			*dp++ = 0;
	}

	{
		uint32_t *dp;
		dp = &__main_stack_base__;
		while(dp < &__process_stack_end__)
			*dp++ = 0x55555555;
	}
#else
	{
		uint32_t *tp, *dp;
		tp = &_ccmtext_data;
		dp = &_ccmtext_start;
		while(dp < &_ccmtext_end)
			*dp++ = *tp++;
	}

	{
		uint32_t *dp;
		dp = &_cmm_start;
		while(dp < &_cmm_end)
			*dp++ = 0;
	}
#endif
}

CH_IRQ_HANDLER(SysTick_Handler)
{
	CH_IRQ_PROLOGUE();

	chSysLockFromISR();
	chSysTimerHandlerI();
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}
