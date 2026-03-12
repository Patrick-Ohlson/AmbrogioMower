/* test_memory.h — Memory variable test (.data, .bss, .rodata) */
#ifndef TEST_MEMORY_H
#define TEST_MEMORY_H

#include <stdint.h>

void Memory_Test(void);

/* Print helpers for pass/fail result lines */
void print_test_u8(const char *label, volatile uint8_t *var, uint8_t expect);
void print_test_u16(const char *label, volatile uint16_t *var, uint16_t expect);
void print_test_u32(const char *label, volatile uint32_t *var, uint32_t expect);

#endif /* TEST_MEMORY_H */
