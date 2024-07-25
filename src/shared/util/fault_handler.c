#include "fault_handler.h"
#include "ch.h"
#include <stdio.h>
#include <stdarg.h>

#pragma GCC optimize ("O0")

void NMI_Handler() __attribute__((naked));
void HardFault_Handler() __attribute__((naked));
void MemManage_Handler() __attribute__((naked));
void BusFault_Handler() __attribute__((naked));
void UsageFault_Handler() __attribute__((naked));
void DebugMon_Handler() __attribute__((naked));

FaultHandler faultHandlerObj;

void FaultHandlerInit(USART_TypeDef* pPrintUsart, FaultHandlerEmergencyStopFunc emergencyStopFunc)
{
	faultHandlerObj.pPrintUart = pPrintUsart;
	faultHandlerObj.pEmergencyStopFunc = emergencyStopFunc;
}

typedef struct _SCBDebug
{
	struct _MMFSR
	{
		uint8_t IACCVIOL;
		uint8_t DACCVIOL;
		uint8_t MUNSTKERR;
		uint8_t MSTKERR;
		uint8_t MLSPERR;
		uint8_t MMARVALID;
	} MMFSR;

	struct _BFSR
	{
		uint8_t IBUSERR;
		uint8_t PRECISERR;
		uint8_t IMPRECISERR;
		uint8_t UNSTKERR;
		uint8_t STKERR;
		uint8_t LSPERR;
		uint8_t BFARVALID;
	} BFSR;

	struct _UFSR
	{
		uint8_t UNDEFINSTR;
		uint8_t INVSTATE;
		uint8_t INVPC;
		uint8_t NOCP;
		uint8_t UNALIGNED;
		uint8_t DIVBYZERO;
	} UFSR;

	struct _HFSR
	{
		uint8_t VECTTBL;
		uint8_t FORCED;
	} HFSR;

	struct _ABFSR
	{
		uint8_t ITCM;
		uint8_t DTCM;
		uint8_t AHBP;
		uint8_t AXIM;
		uint8_t EPPB;
		uint8_t AXIMTYPE;
	} ABFSR;

	uint32_t MMFAR;
	uint32_t BFAR;
	uint32_t AFSR;
} SCBDebug;

static inline SCBDebug getSCBDebug()
{
	uint32_t cfsr = SCB->CFSR;
	uint32_t hfsr = SCB->HFSR;
	uint32_t abfsr = SCB->ABFSR;
	SCBDebug res;

	res.MMFSR.IACCVIOL = (cfsr >> 0) & 0x01;
	res.MMFSR.DACCVIOL = (cfsr >> 1) & 0x01;
	res.MMFSR.MUNSTKERR = (cfsr >> 3) & 0x01;
	res.MMFSR.MSTKERR = (cfsr >> 4) & 0x01;
	res.MMFSR.MLSPERR = (cfsr >> 5) & 0x01;
	res.MMFSR.MMARVALID = (cfsr >> 7) & 0x01;

	res.BFSR.IBUSERR = (cfsr >> 8) & 0x01;
	res.BFSR.PRECISERR = (cfsr >> 9) & 0x01;
	res.BFSR.IMPRECISERR = (cfsr >> 10) & 0x01;
	res.BFSR.UNSTKERR = (cfsr >> 11) & 0x01;
	res.BFSR.STKERR = (cfsr >> 12) & 0x01;
	res.BFSR.LSPERR = (cfsr >> 13) & 0x01;
	res.BFSR.BFARVALID = (cfsr >> 15) & 0x01;

	res.UFSR.UNDEFINSTR = (cfsr >> 16) & 0x01;
	res.UFSR.INVSTATE = (cfsr >> 17) & 0x01;
	res.UFSR.INVPC = (cfsr >> 18) & 0x01;
	res.UFSR.NOCP = (cfsr >> 19) & 0x01;
	res.UFSR.UNALIGNED = (cfsr >> 24) & 0x01;
	res.UFSR.DIVBYZERO = (cfsr >> 25) & 0x01;

	res.HFSR.VECTTBL = (hfsr >> 1) & 0x01;
	res.HFSR.FORCED = (hfsr >> 30) & 0x01;

	res.ABFSR.ITCM = (abfsr >> 0) & 0x01;
	res.ABFSR.DTCM = (abfsr >> 1) & 0x01;
	res.ABFSR.AHBP = (abfsr >> 2) & 0x01;
	res.ABFSR.AXIM = (abfsr >> 3) & 0x01;
	res.ABFSR.EPPB = (abfsr >> 4) & 0x01;
	res.ABFSR.AXIMTYPE = (abfsr >> 8) & 0x03;

	res.MMFAR = SCB->MMFAR;
	res.BFAR = SCB->BFAR;
	res.AFSR = SCB->AFSR;

	return res;
}

static uint8_t initFaultPrint()
{
	if(faultHandlerObj.pPrintUart)
	{
		faultHandlerObj.pPrintUart->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_RXNEIE);	// disable IRQs
		faultHandlerObj.pPrintUart->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
		return 1;
	}

	return 0;
}

static int32_t faultPrint(const char* fmt, ...)
{
	va_list args;
	int32_t bytesWritten;
	static char printBuf[128];

	USART_TypeDef* pFaultUart = faultHandlerObj.pPrintUart;

	va_start(args, fmt);
	bytesWritten = vsnprintf(printBuf, 128, fmt, args);
	va_end(args);

	for(int32_t i = 0; i < bytesWritten; i++)
	{
		while(!(pFaultUart->ISR & USART_ISR_TXE_TXFNF))
			asm volatile("nop");

		pFaultUart->TDR = printBuf[i];
	}

	return bytesWritten;
}

static void waitForKey()
{
	USART_TypeDef* pFaultUart = faultHandlerObj.pPrintUart;

	while(!(pFaultUart->ISR & USART_ISR_RXNE_RXFNE))
	{
		asm volatile("nop");
	}

	pFaultUart->RDR;
}

static void emergencyStop()
{
	if(faultHandlerObj.pEmergencyStopFunc)
	{
		(*faultHandlerObj.pEmergencyStopFunc)();
	}
}

static const char* tStateNames[] = { CH_STATE_NAMES };

void osFault()
{
	emergencyStop();

	if(initFaultPrint())
	{
		while(1)
		{
			faultPrint("\r\n--- OS Fault ---\r\n");
			faultPrint("Reason: %s\r\n", ch0.dbg.panic_msg);
			faultPrint("\r\nSystem execution stopped...\r\n");

			faultPrint("Listing active IRQs...\r\n");

			for(int32_t i = -15; i < 0; i++)
			{
				faultPrint("%d: %u\r\n", i, NVIC_GetPriority(i));
			}

			int32_t numIrqs = sizeof(NVIC->IP)/sizeof(NVIC->IP[0]);

			for(int32_t i = 0; i < numIrqs; i++)
			{
				uint32_t enabled = NVIC->ISER[(((uint32_t)(int32_t)i) >> 5UL)] & ((uint32_t)(1UL << (((uint32_t)(int32_t)i) & 0x1FUL)));

				if(enabled)
				{
					faultPrint("%d: %u\r\n", i, NVIC_GetPriority(i));
				}
			}

			// Ready List Check
			os_instance_t *oip = currcore;
			cnt_t n;

			/* Ready List integrity check.*/
			ch_priority_queue_t *pqp;

			/* Scanning the ready list forward.*/
			n = (cnt_t) 0;
			pqp = oip->rlist.pqueue.next;
			while(pqp != &oip->rlist.pqueue)
			{
				n++;
				pqp = pqp->next;
			}

			/* Scanning the ready list backward.*/
			pqp = oip->rlist.pqueue.prev;
			while(pqp != &oip->rlist.pqueue)
			{
				n--;
				pqp = pqp->prev;
			}

			/* The number of elements must match.*/
			if(n != (cnt_t) 0)
			{
				faultPrint("Integrity check failed\r\n");
			}
			else
			{
				faultPrint("Integrity check OK\r\n");

				faultPrint("Stack free...\r\n");
				faultPrint("IRQ: %u\r\n", chThdGetFreeStack(0));

				thread_t *pThread;
				uint8_t *p;

				p = (uint8_t*) REG_HEADER(currcore)->next;
				pThread = threadref((p - __CH_OFFSETOF(thread_t, rqueue)));

				while(pThread != 0)
				{
					const char* pName;
					if(pThread->name)
						pName = pThread->name;
					else
						pName = "Unknown";

					faultPrint("%-16s %u %-10s\r\n", pName, chThdGetFreeStack(pThread), tStateNames[pThread->state]);

					ch_queue_t *nqp;

					/* Next element in the registry queue.*/
					nqp = pThread->rqueue.next;
					if(nqp == REG_HEADER(currcore))
					{
						break;
					}
					else
					{
						uint8_t *p = (uint8_t*) nqp;
						pThread = threadref((p - __CH_OFFSETOF(thread_t, rqueue)));
					}
				}
			}

			waitForKey();
		}
	}
	else
	{
		asm volatile("BKPT #01");
		while(1);
	}
}

void faultHandler(unsigned int* args) __attribute__((used));

void faultHandler(unsigned int* args)
{
	unsigned int r0 = args[0];
	unsigned int r1 = args[1];
	unsigned int r2 = args[2];
	unsigned int r3 = args[3];
	unsigned int r12 = args[4];
	unsigned int lr = args[5];
	unsigned int pc = args[6];
	unsigned int psr = args[7];

	SCBDebug cfsr = getSCBDebug();

	emergencyStop();

	if(initFaultPrint())
	{
		while(1)
		{
			faultPrint("--- System Fault ---\r\n");
			faultPrint("R0:  0x%08X\r\n", r0);
			faultPrint("R1:  0x%08X\r\n", r1);
			faultPrint("R2:  0x%08X\r\n", r2);
			faultPrint("R3:  0x%08X\r\n", r3);
			faultPrint("R12: 0x%08X\r\n", r12);
			faultPrint("LR:  0x%08X\r\n", lr);
			faultPrint("PC:  0x%08X\r\n", pc);
			faultPrint("PSR: 0x%08X\r\n\r\n", psr);

			faultPrint("MMFAR: 0x%08X\r\n", cfsr.MMFAR);
			faultPrint("BFAR:  0x%08X\r\n", cfsr.BFAR);
			faultPrint("AFSR:  0x%08X\r\n\r\n", cfsr.AFSR);

			faultPrint("- MMFSR -\r\n");
			faultPrint("IACCVIOL:    0x%02hX\r\n", (uint16_t)cfsr.MMFSR.IACCVIOL);
			faultPrint("DACCVIOL:    0x%02hX\r\n", (uint16_t)cfsr.MMFSR.DACCVIOL);
			faultPrint("MUNSTKERR:   0x%02hX\r\n", (uint16_t)cfsr.MMFSR.MUNSTKERR);
			faultPrint("MSTKERR:     0x%02hX\r\n", (uint16_t)cfsr.MMFSR.MSTKERR);
			faultPrint("MLSPERR:     0x%02hX\r\n", (uint16_t)cfsr.MMFSR.MLSPERR);
			faultPrint("MMFARVALID:  0x%02hX\r\n\r\n", (uint16_t)cfsr.MMFSR.MMARVALID);

			faultPrint("- BFSR -\r\n");
			faultPrint("IBUSERR:     0x%02hX\r\n", (uint16_t)cfsr.BFSR.IBUSERR);
			faultPrint("PRECISERR:   0x%02hX\r\n", (uint16_t)cfsr.BFSR.PRECISERR);
			faultPrint("IMPRECISERR: 0x%02hX\r\n", (uint16_t)cfsr.BFSR.IMPRECISERR);
			faultPrint("UNSTKERR:    0x%02hX\r\n", (uint16_t)cfsr.BFSR.UNSTKERR);
			faultPrint("STKERR:      0x%02hX\r\n", (uint16_t)cfsr.BFSR.STKERR);
			faultPrint("LSPERR:      0x%02hX\r\n", (uint16_t)cfsr.BFSR.LSPERR);
			faultPrint("BFARVALID:   0x%02hX\r\n\r\n", (uint16_t)cfsr.BFSR.BFARVALID);

			faultPrint("- UFSR -\r\n");
			faultPrint("UNDEFINSTR:  0x%02hX\r\n", (uint16_t)cfsr.UFSR.UNDEFINSTR);
			faultPrint("INVSTATE:    0x%02hX\r\n", (uint16_t)cfsr.UFSR.INVSTATE);
			faultPrint("INVPC:       0x%02hX\r\n", (uint16_t)cfsr.UFSR.INVPC);
			faultPrint("NOCP:        0x%02hX\r\n", (uint16_t)cfsr.UFSR.NOCP);
			faultPrint("UNALIGNED:   0x%02hX\r\n", (uint16_t)cfsr.UFSR.UNALIGNED);
			faultPrint("DIVBYZERO:   0x%02hX\r\n\r\n", (uint16_t)cfsr.UFSR.DIVBYZERO);

			faultPrint("- HFSR -\r\n");
			faultPrint("VECTTBL:    0x%02hX\r\n", (uint16_t)cfsr.HFSR.VECTTBL);
			faultPrint("FORCED:     0x%02hX\r\n\r\n", (uint16_t)cfsr.HFSR.FORCED);

			faultPrint("- ABFSR -\r\n");
			faultPrint("ITCM:       0x%02hX\r\n", (uint16_t)cfsr.ABFSR.ITCM);
			faultPrint("DTCM:       0x%02hX\r\n", (uint16_t)cfsr.ABFSR.DTCM);
			faultPrint("AHBP:       0x%02hX\r\n", (uint16_t)cfsr.ABFSR.AHBP);
			faultPrint("AXIM:       0x%02hX\r\n", (uint16_t)cfsr.ABFSR.AXIM);
			faultPrint("EPPB:       0x%02hX\r\n", (uint16_t)cfsr.ABFSR.EPPB);
			faultPrint("AXIMTYPE:   0x%02hX\r\n\r\n", (uint16_t)cfsr.ABFSR.AXIMTYPE);

#ifdef STM32H7XX
			faultPrint("- FLASH -\r\n");
			faultPrint("SR1:        0x%08X\r\n", FLASH->SR1);
			faultPrint("SR2:        0x%08X\r\n", FLASH->SR2);
#endif

			faultPrint("Dump complete. System execution stopped...\r\n");

			waitForKey();
		}
	}
	else
	{
		asm volatile("BKPT #01");
		while(1);
	}
}

void NMI_Handler()
{
	asm volatile("TST LR, #4");
	asm volatile("ITE EQ");
	asm volatile("MRSEQ R0, MSP");
	asm volatile("MRSNE R0, PSP");
	asm volatile("B faultHandler");
}

void HardFault_Handler()
{
	asm volatile("TST LR, #4");
	asm volatile("ITE EQ");
	asm volatile("MRSEQ R0, MSP");
	asm volatile("MRSNE R0, PSP");
	asm volatile("B faultHandler");
}

void MemManage_Handler()
{
	asm volatile("TST LR, #4");
	asm volatile("ITE EQ");
	asm volatile("MRSEQ R0, MSP");
	asm volatile("MRSNE R0, PSP");
	asm volatile("B faultHandler");
}

void BusFault_Handler()
{
	asm volatile("TST LR, #4");
	asm volatile("ITE EQ");
	asm volatile("MRSEQ R0, MSP");
	asm volatile("MRSNE R0, PSP");
	asm volatile("B faultHandler");
}

void UsageFault_Handler()
{
	asm volatile("TST LR, #4");
	asm volatile("ITE EQ");
	asm volatile("MRSEQ R0, MSP");
	asm volatile("MRSNE R0, PSP");
	asm volatile("B faultHandler");
}

void DebugMon_Handler()
{
	asm volatile("TST LR, #4");
	asm volatile("ITE EQ");
	asm volatile("MRSEQ R0, MSP");
	asm volatile("MRSNE R0, PSP");
	asm volatile("B faultHandler");
}
