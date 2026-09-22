/*
 * fault_handler.h
 *
 *  Crash log captured by HardFault_Handler, retained across a warm reset.
 */

#ifndef CORE_INC_FAULT_HANDLER_H_
#define CORE_INC_FAULT_HANDLER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 0 when no crash is recorded, otherwise fills the outputs. */
uint32_t crash_log_read(uint32_t* pc, uint32_t* lr, uint32_t* cfsr);

void crash_log_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_FAULT_HANDLER_H_ */
