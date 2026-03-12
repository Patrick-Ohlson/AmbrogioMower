/* utils.h — Shared utility helpers for TestFirmware
 *
 * Delay routines, hex print functions, and FRT (Free-Running Timer) helpers.
 * Used across all test modules.
 */
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

/* Hex digit lookup table (defined in utils.c) */
extern char HEX[];

/* --- Delay functions --- */
void delay(int counter);
void delays(void);
void delay_ms(unsigned int delay);

/* --- FRT (Free-Running Timer) helpers --- */
void init_frt(void);
void delay_frt(uint32_t delay_ticks);
void delay_ms_frt(uint32_t ms);
void FRT_delay_sec(uint16_t sec);

/* --- Interrupt control --- */
void DI(void);      /* orc #0x80,ccr  — disable all maskable IRQs */
void EI(void);      /* andc #0x7F,ccr — enable IRQs */

/* --- Hex print helpers (output via PutChar/SCI) --- */
void print_hex_nibble(uint8_t n);
void print_hex8(uint8_t val);
void print_hex16(uint16_t val);
void print_hex32(uint32_t val);
void print_addr(const volatile void *ptr);

#endif /* UTILS_H */
