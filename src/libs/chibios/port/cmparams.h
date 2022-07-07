/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    STM32F4xx/cmparams.h
 * @brief   ARM Cortex-M4 parameters for the STM32F4xx.
 *
 * @defgroup ARMCMx_STM32F4xx STM32F4xx Specific Parameters
 * @ingroup ARMCMx_SPECIFIC
 * @details This file contains the Cortex-M4 specific parameters for the
 *          STM32F4xx platform.
 * @{
 */

#ifndef _CMPARAMS_H_
#define _CMPARAMS_H_

/**
 * @brief   Cortex core model.
 */
#define CORTEX_MODEL            4

/**
 * @brief   Floating Point unit presence.
 */
#define CORTEX_HAS_FPU          1

/**
 * @brief   Number of bits in priority masks.
 */
#define CORTEX_PRIORITY_BITS    4

/**
 * @brief   Number of interrupt vectors.
 * @note    This number does not include the 16 system vectors and must be
 *          rounded to a multiple of 8.
 */
#if defined(STM32F30X) || defined(STM32F4XX)
#define CORTEX_NUM_VECTORS      88
#endif

#ifdef STM32F7XX
#define CORTEX_NUM_VECTORS      104
#endif

#ifdef STM32H7XX
#define CORTEX_NUM_VECTORS      152
#endif

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/* Including the device CMSIS header. Note, we are not using the definitions
   from this header because we need this file to be usable also from
   assembler source files. We verify that the info matches instead.*/
#ifdef STM32F30X
#include "stm32f30x.h"
#endif

#ifdef STM32F4XX
#include "stm32f407xx.h"
#endif

#ifdef STM32F7XX
#include "stm32f746xx.h"
#endif

#ifdef STM32H7XX
#include "stm32h743xx.h"
#endif

//#if CORTEX_MODEL != __CORTEX_M
//#error "CMSIS __CORTEX_M mismatch"
//#endif
//
//#if CORTEX_HAS_FPU != __FPU_PRESENT
//#error "CMSIS __FPU_PRESENT mismatch"
//#endif
//
//#if CORTEX_PRIORITY_BITS != __NVIC_PRIO_BITS
//#error "CMSIS __NVIC_PRIO_BITS mismatch"
//#endif

#endif /* !defined(_FROM_ASM_) */

#endif /* _CMPARAMS_H_ */

/** @} */
