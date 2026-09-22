/*
 * fault_handler.c
 *
 * HardFault capture with a crash log that survives a warm reset.
 *
 * The Cortex-M3 pushes R0-R3, R12, LR, PC and xPSR onto the active stack before
 * entering the handler, so the faulting address is already in memory; the only
 * work is finding the frame and copying the fault status registers out of it.
 */

#include "main.h"
#include <stdint.h>

#define CRASH_LOG_MAGIC 0xDEADC0DEu

typedef struct {
    uint32_t magic;
    uint32_t pc; /* address of the faulting instruction */
    uint32_t lr; /* return address of the faulting function */
    uint32_t psr;
    uint32_t cfsr; /* MMFSR | BFSR | UFSR */
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
} crash_log_t;

/* .noinit is excluded from the startup zeroing loop, so this keeps its value
   across a reset as long as RAM stays powered. */
__attribute__((section(".noinit"))) volatile crash_log_t crash_log;

static void enter_safe_state(void) {
    TIM2->CCR1 = 0xFFFFUL; /* SSR drive is inverted: high is off */
    TIM1->CCR1 = 0UL;      /* buzzer silent */
}

/* Called from HardFault_Handler with the stacked exception frame in R0. */
void hard_fault_report(uint32_t* frame) {
    enter_safe_state();

    crash_log.pc = frame[6];
    crash_log.lr = frame[5];
    crash_log.psr = frame[7];
    crash_log.cfsr = SCB->CFSR;
    crash_log.hfsr = SCB->HFSR;
    crash_log.bfar = SCB->BFAR;
    crash_log.mmfar = SCB->MMFAR;
    crash_log.magic = CRASH_LOG_MAGIC; /* last, so a partial write is not trusted */

    /* BKPT with no debugger attached escalates to another fault, so guard it. */
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        __asm volatile("bkpt #0");
    }

    /* Deliberately no reset here: the watchdog reboots us, and the log survives. */
    while (1) {
    }
}

__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile("tst lr, #4          \n" /* EXC_RETURN bit 2: 0 = MSP, 1 = PSP */
                   "ite eq              \n"
                   "mrseq r0, msp       \n"
                   "mrsne r0, psp       \n"
                   "b hard_fault_report \n");
}

uint32_t crash_log_read(uint32_t* pc, uint32_t* lr, uint32_t* cfsr) {
    if (crash_log.magic != CRASH_LOG_MAGIC) {
        return 0U;
    }
    *pc = crash_log.pc;
    *lr = crash_log.lr;
    *cfsr = crash_log.cfsr;
    return 1U;
}

void crash_log_clear(void) {
    crash_log.magic = 0U;
}
