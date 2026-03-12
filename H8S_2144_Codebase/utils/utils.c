/* utils.c — Shared utility helpers for TestFirmware
 *
 * Delay routines, hex print functions, and FRT (Free-Running Timer) helpers.
 */
#include "utils.h"
#include "../iodefine.h"
#include "../sci.h"

/* Direct register access for MSTPCR (word-wide to avoid
 * H8S bit-field read-modify-write issues with GCC) */
#define MSTPCR_WORD     (*(volatile unsigned short *)0xFFFF86)

/* FRT register definitions */
#define FRT_TCSR_BYTE (*(volatile uint8_t *)0xFFFF91)
#define FRT_FRC       (*(volatile uint16_t *)0xFFFF92)

/* FRT timing constants */
#define SYSTEM_CLOCK_HZ 20000000
#define PRESCALER 30
#define FRT_TICK_RATE (SYSTEM_CLOCK_HZ / PRESCALER)
#define TICKS_PER_MS (FRT_TICK_RATE / 1000)

/* Hex digit lookup table */
char HEX[] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
  };


/* ================================================================
 * Delay functions
 * ================================================================ */
void delay(int counter){
	unsigned int index,i;
	for(i=0;i<counter;i++){
		for(index=0;index<50;index++){
			delay_ms(1000);
		}
	}
}

void delays(){
	delay(4);
}

void delay_ms(unsigned int delay)
{
	unsigned long scaler;
	unsigned long index;

	scaler = 925UL * delay;
	for(index=0;index<scaler;index++)
	{
	   asm("nop");
		 asm("nop");
		 asm("nop");
		 asm("nop");
		 asm("nop");
	}
}


/* ================================================================
 * FRT (Free-Running Timer) helpers
 * ================================================================ */
void init_frt(void) {
    /* Enable Free-Running Timer (clear MSTP13) */
    MSTPCR_WORD &= ~(1 << 13);

    /* Set Timer Control Register to 0xE2 (phi / 64, 3.2us per tick) */
    FRT_TCSR_BYTE = 0xE2;

    /* Reset the Free-Running Counter */
    FRT_FRC = 0x0000;
}

void delay_frt(uint32_t delay_ticks) {
    uint16_t frc_start, frc_now;
    uint32_t elapsed_ticks = 0, overflow_count = 0;

    /* Reset FRC counter */
    FRT_FRC = 0x0000;
    frc_start = FRT_FRC;

    /* Clear OVF flag */
    FRT_TCSR_BYTE &= ~(1 << 1);

    while (elapsed_ticks < delay_ticks) {
        frc_now = FRT_FRC;

        if (FRT_TCSR_BYTE & (1 << 1)) {
            FRT_TCSR_BYTE&= ~(1 << 1);
            overflow_count += 0x10000;
        }

        elapsed_ticks = overflow_count + (uint32_t)(frc_now - frc_start);
    }
}

void delay_ms_frt(uint32_t ms) {
    uint32_t ticks_needed = ms * TICKS_PER_MS;
    delay_frt(ticks_needed);
}

void FRT_delay_sec(uint16_t sec) {
    while (sec--) {
        delay_ms_frt(1000);
    }
}


/* ================================================================
 * Interrupt control
 * ================================================================ */
void DI(void)
{
    __asm__ volatile ("orc #0x80,ccr");   /* Set I bit -> mask all IRQs */
}

void EI(void)
{
    __asm__ volatile ("andc #0x7F,ccr");  /* Clear I bit -> enable IRQs */
}


/* ================================================================
 * Hex print helpers (output via PutChar/SCI)
 * ================================================================ */
void print_hex_nibble(uint8_t n)
{
	PutChar(HEX[n & 0x0F]);
}

void print_hex8(uint8_t val)
{
	print_hex_nibble(val >> 4);
	print_hex_nibble(val);
}

void print_hex16(uint16_t val)
{
	print_hex8((uint8_t)(val >> 8));
	print_hex8((uint8_t)(val & 0xFF));
}

void print_hex32(uint32_t val)
{
	print_hex16((uint16_t)(val >> 16));
	print_hex16((uint16_t)(val & 0xFFFF));
}

/* Print address of a variable (24-bit H8S address space) */
void print_addr(const volatile void *ptr)
{
	uint32_t a = (uint32_t)ptr;
	PutChar('0'); PutChar('x');
	print_hex8((uint8_t)(a >> 16));
	print_hex16((uint16_t)(a & 0xFFFF));
}
