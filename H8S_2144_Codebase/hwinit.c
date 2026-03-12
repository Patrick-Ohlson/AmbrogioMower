/*
 * hwinit.c - Hardware initialization for L200 mower (H8S/2144)
 *
 * Clean-room clone of SetupHW_Q (0x121A4) from original firmware.
 * This function configures every GPIO port, peripheral clock gate,
 * SCI2, FRT, and D/A converter to match the original mower state
 * at boot time.
 *
 * Register addresses are accessed via iodefine.h structures.
 * See STATES/memory_map.md for full I/O register map.
 *
 * Original firmware call chain:
 *   RealBoot (0x1A4) -> OldBoot (0xBF0) -> SetupHW_Q (0x121A4)
 *
 * Firmware traceability:
 *   SetupHW_Q  @ 0x121A4  (body 0x121A4 - 0x122A6)
 */

#include <stdint.h>
#include "inlines.h"
#include "edk2215def.h"
#include "iodefine.h"
#include "typedefine.h"
#include "lcd/lcd.h"
#include "motor/motor.h"

/* Forward declarations */
void HardwareSetup(void);
void hw_initialise(void);

/*
 * hw_initialise - Entry point called from start.S (_hw_initialise)
 *
 * This is the first C function called after the stack pointer is set.
 * Calls HardwareSetup() which clones the original firmware's SetupHW_Q.
 */
void hw_initialise(void)
{
	HardwareSetup();
}

/*
 * HardwareSetup - Complete H8S/2144 peripheral initialization
 *
 * Clone of SetupHW_Q (0x121A4) from original firmware.
 * Configures all GPIO ports, peripheral clocks, SCI2, FRT, and DAC
 * to the exact state the mower firmware expects at startup.
 *
 * Register init order matches the original firmware exactly.
 */
void HardwareSetup(void)
{
	uint8_t tmp;

	/* ================================================================
	 * 1. Module Stop Control Register (MSTPCR) - 0xFFFF86
	 *    Enable selected peripheral clocks.
	 *    0x403F = 0100_0000_0011_1111
	 *    Bit 14: Stop TMR0/1 (8-bit timers)
	 *    Bits 5-0: Stop various modules
	 *    Clears bits for: SCI0, SCI1, SCI2, FRT, A/D, D/A, I2C, etc.
	 * ================================================================ */
	MSTPCR.WORD = 0x403F;

	/* ================================================================
	 * 2. GPIO Port Configuration
	 *    Each port has: DDR (direction), DR (data), PCR (pull-up)
	 *    DDR: 0=input, 1=output
	 *    DR:  output latch value
	 *    PCR: 1=enable internal pull-up (input pins only)
	 *
	 *    Mower pin assignments (from Ghidra RE):
	 *    See STATES/memory_map.md "GPIO Pin Assignments" section
	 * ================================================================ */

	/* --- Port 1 (0xFFFFB0-B2) - Address bus A0-A7 / General I/O ---
	 *   DDR=0x20: bit5 output (P1.5 = charging relay), rest input
	 *   DR=0x00:  all outputs LOW (relay off at startup)
	 *   PCR=0x5F: pull-ups on P1.0-4,6 (0101_1111)
	 *             P1.0=bumper?, P1.1-4=sensors, P1.6=sensor
	 */
	P1.DR.BYTE = 0x00;
	P1.DDR     = 0x20;       /* P1.5 output (charge relay) */
	P1.PCR.BYTE = 0x5F;      /* Pull-ups on input pins */

	/* --- Port 2 (0xFFFFB1,B3) - Address bus A8-A15 / General I/O ---
	 *   DDR=0xE7: bits 7,6,5,2,1,0 output (1110_0111)
	 *   DR=0x00:  all outputs LOW
	 *   Note: Ghidra shows P1_wk2 (0xFFFFB1) = P2.DDR
	 */
	P2.DR.BYTE = 0x00;
	P2.DDR     = 0xE7;       /* Most pins output */

	/* --- Port 3 (0xFFFFB4-B6) - LCD interface ---
	 *   DDR=0xF7: all output except P3.3 (1111_0111)
	 *     P3.0=LCD_RS, P3.1=LCD_RW, P3.2=LCD_E
	 *     P3.4-7=LCD_D4-D7 (4-bit data bus)
	 *     P3.3=input (unused/reserved)
	 *   DR=0x00:  all outputs LOW (LCD idle state)
	 */
	P3.DR.BYTE = 0x00;
	P3.DDR     = 0xF7;       /* LCD port: all output except P3.3 */

	/* --- Port 4 (0xFFFFB5,B7) - Blade motor / misc ---
	 *   DDR=0x23: bits 5,1,0 output (0010_0011)
	 *     P4.0=blade motor enable
	 *     P4.1=blade motor direction?
	 *     P4.5=LED/status?
	 *   DR=0x20:  P4.5 HIGH at startup (0010_0000)
	 *   Note: Ghidra shows P3_wk2 (0xFFFFB5) = P4.DDR
	 */
	P4.DR.BYTE = 0x20;
	P4.DDR     = 0x23;       /* P4.0,1,5 output */

	/* --- Port 5 (0xFFFFB8-BA) - Wheel motor PWM ---
	 *   DDR=0x04: bit2 output (0000_0100)
	 *     P5.2=motor enable/direction?
	 *   DR=0x00:  all outputs LOW (motors off)
	 */
	P5.DR.BYTE = 0x00;
	P5.DDR     = 0x04;       /* P5.2 output */

	/* --- Port 6 (0xFFFFB9,BB) - Motor control / wire sensor ---
	 *   DDR=0x00: all input (0000_0000)
	 *     P6.2,3=motor1 IN1/IN2 (configured later by motor driver)
	 *     P6.5,6=motor enables (configured later)
	 *   DR=0x00:  all LOW
	 *   Note: Ghidra shows P5_wk (0xFFFFB9) = P6.DDR
	 */
	P6.DR.BYTE = 0x00;
	P6.DDR     = 0x00;       /* All input at init */

	/* --- Port 8 (0xFFFFBD-BF) - I2C / bumper / misc ---
	 *   DDR=0x0F: bits 3-0 output (0000_1111)
	 *     P8.0-3=output (LED/status/control)
	 *     P8.4-7=input (bumper sensors, etc.)
	 *   DR=0x00:  all outputs LOW
	 */
	P8.DR.BYTE = 0x00;
	P8.DDR     = 0x0F;       /* P8.0-3 output */

	/* --- Port 9 (0xFFFFC0-C1) - Motor enables / misc ---
	 *   DDR=0x23: bits 5,1,0 output (0010_0011)
	 *     P9.0=M2_IN1, P9.1=M2_IN2
	 *     P9.2=M1_EN (input - not set here)
	 *     P9.3=M2_EN (input - not set here)
	 *     P9.5=output (misc control)
	 *   DR=0x20:  P9.5 HIGH at startup (0010_0000)
	 */
	P9.DR.BYTE = 0x20;
	P9.DDR     = 0x23;       /* P9.0,1,5 output */

	/* --- Port A (0xFFFFAA-AB) - Extended address / open-drain ---
	 *   ODR=0xC0: bits 7,6 open-drain (1100_0000)
	 *     PA.6,PA.7 = open-drain I/O (purpose TBD)
	 *       NOT I2C Bus 1 (that's P8.6=SDA, P9.7=SCL)
	 *       May be I2C Bus 2 (accelerometer, oBladeMotorFuncs2)
	 *       or H8S hardware IIC pins (IICE disabled in STCR)
	 *       Needs Ghidra analysis of blade_v2_* functions to confirm
	 *   DDR=0x03: bits 1,0 output (0000_0011)
	 *     PA.0,1=address/control outputs
	 */
	PA.ODR.BYTE = 0xC0;      /* PA.6,7 open-drain (I2C) */
	PA.EQU.DDR  = 0x03;      /* PA.0,1 output */

	/* --- Port B (0xFFFFBC-BE) - Open-drain / misc ---
	 *   ODR (0xFFFFBC)=0xFF: all pins open-drain
	 *   DDR (0xFFFFBE)=0x00: all input
	 *   Note: Ghidra shows P6_wk2=PB.ODR, P7_PIN_BYTE=PB.DDR
	 */
	PB.ODR.BYTE = 0xFF;      /* All open-drain */
	PB.DDR      = 0x00;      /* All input */

	/* ================================================================
	 * 3. Low-Power Control
	 *    LPWRCR (0xFFFF85) |= 0x10: enable sub-clock output
	 *    Bit 4: LSON = 1 (sub-clock oscillator on)
	 * ================================================================ */
	tmp = LPWRCR.BYTE;
	LPWRCR.BYTE = tmp | 0x10;

	/* ================================================================
	 * 4. STCR - Serial Timer Control Register (0xFFFFC3)
	 *    Original firmware (SetupHW_Q) does:
	 *      STCR &= 0xFC  (clear bits 1:0 - ICKS clock select)
	 *      STCR |= 0x10  (set bit 4 - IICE/PWMX bus enable)
	 *
	 *    Clear ICKS bits here.  IICE (bit 4) for PWMX access and
	 *    ICKS0 (bit 0) for TMR clock are set later by motor_Init()
	 *    and motor_BladeInit(), which maintain an stcr_shadow to
	 *    avoid read-modify-write hazards during motor EMI.
	 *
	 *    NOTE: On H8S/2144, IICE enables the PWMX D/A-PWM module
	 *    bus (not hardware I2C on SCI1 pins).  Bit-bang I2C on
	 *    P8.6/P9.7 works independently of IICE.
	 * ================================================================ */
	STCR.BYTE = 0x00;        /* Start clean.  motor_Init() will set
	                           * IICE (0x10) and ICKS0 (0x01) via
	                           * shadow register — never read-modify-write.
	                           */

	/* ================================================================
	 * 5. SCI2 Configuration (0xFFFFA0-A6)
	 *    SCI2 is used for the boundary wire signal receiver.
	 *
	 *    Sequence from original firmware:
	 *      SCMR = 3          (smart-card mode register)
	 *      SMR  = 0x3F       (async, 8-bit, parity, 2-stop, /64)
	 *      SCMR = 3
	 *      SCMR = 2
	 *      SMR  = 2          (final: async, 8N1, /8)
	 *      SCMR = 2
	 *
	 *    Note: the repeated writes are the original firmware's
	 *    initialization dance - possibly required for H8S SCI
	 *    register synchronization.
	 * ================================================================ */
	SCI2.SCMR.BYTE = 0x03;
	SCI2.SMR.BYTE  = 0x3F;
	SCI2.SCMR.BYTE = 0x03;
	SCI2.SCMR.BYTE = 0x02;
	SCI2.SMR.BYTE  = 0x02;
	SCI2.SCMR.BYTE = 0x02;

	/* ================================================================
	 * 6. Free-Running Timer (FRT) - 0xFFFF90-9E
	 *    TCR (0xFFFF96) = 0xE2
	 *      Bits 7-6: IEDG = 11 (both edges capture)
	 *      Bits 1-0: CKS = 10 (internal clock / 8 = 2.5 MHz)
	 *    Used for motor PWM timing and general delays.
	 * ================================================================ */
	FRT.TCR.BYTE = 0xE2;

	/* ================================================================
	 * 7. D/A Converter (0xFFFFF8-FA)
	 *    DADR0 = 0x00   (channel 0 output = 0V)
	 *    DADR1 = 0x00   (channel 1 output = 0V)
	 *    DACR  = 0xC0   (bits 7,6 = 1: enable both DA channels)
	 *    Used for analog reference voltages (wire signal threshold?)
	 * ================================================================ */
	DA.DADR0      = 0x00;
	DA.DADR1      = 0x00;
	DA.DACR.BYTE  = 0xC0;    /* Enable both D/A channels */
}

/* ================================================================
 * Catch-all handler for unused interrupt vectors.
 * Referenced by vects.c for all unassigned vector table entries.
 *
 * Instead of silently hanging, display the return address on the
 * LCD so we can identify which interrupt fired.  The H8S pushes
 * CCR (1 byte) + PC (4 bytes) on the stack before entering the
 * ISR, so the return PC is at SP+2 (after CCR padding on H8S/2600).
 *
 * LCD row 0: "UNDEF IRQ!"
 * LCD row 1: "PC=0x" + 8-digit hex return address
 * ================================================================ */
void UNDEFINED_ISR(void) __attribute__((interrupt_handler));
void UNDEFINED_ISR(void)
{
	static const char hex[] = "0123456789ABCDEF";
	char buf[16];
	uint32_t pc;
	uint8_t *sp;
	int i;

	/* Read SP to find the return address.
	 * H8S/2600 ISR stack frame: [SP+0]=CCR(padded to word), [SP+2..5]=PC
	 * The compiler may have pushed additional regs, but __attribute__
	 * ((interrupt_handler)) means we're at the raw ISR entry. */
	__asm__ volatile ("mov.l er7, %0" : "=r"(sp));
	/* Return PC is at offset +2 (after 2-byte CCR) in 2600 mode */
	pc = ((uint32_t)sp[2] << 24) | ((uint32_t)sp[3] << 16) |
	     ((uint32_t)sp[4] << 8)  |  (uint32_t)sp[5];

	/* Format "PC=0xXXXXXXXX" into buf */
	buf[0]  = 'P'; buf[1] = 'C'; buf[2] = '=';
	buf[3]  = '0'; buf[4] = 'x';
	for (i = 0; i < 8; i++)
		buf[5 + i] = hex[(pc >> (28 - 4*i)) & 0xF];
	buf[13] = '\0';

	lcd_Print(0, "UNDEF IRQ!");
	lcd_Print(1, "%s", buf);

	/* Halt — motor safety: cut power (shadow write) */
	shadow_p4_dr &= (uint8_t)~MOTOR_P4_POWER;
	P4.DR.BYTE = shadow_p4_dr;

	while(1);
}

/* ================================================================
 * FRT (Free-Running Timer) ISR stubs
 *
 * motor_LeftForward() enables TIER bit 5 (ICIA, encoder capture A)
 * motor_RightForward() enables TIER bit 7 (ICIC, encoder capture C)
 *
 * If the encoder pin has noise or an edge fires, the ISR triggers.
 * Without a proper handler, the default UNDEFINED_ISR (while(1))
 * hangs the MCU.
 *
 * These stubs clear the interrupt flag in TCSR and disable the
 * interrupt source in TIER so it doesn't re-fire.  We don't use
 * encoder feedback in the test firmware, so disabling is safe.
 *
 * Original firmware vectors (L200 build):
 *   ICIA (vec 48) = encoder capture A handler
 *   ICIB (vec 49) = encoder capture B handler
 *   ICIC (vec 50) = encoder capture C handler
 *   FOVI (vec 54) = FRT overflow handler
 * ================================================================ */
/* EMI-safe FRT ISR stubs:
 * Clear the source flag via standard H8S read-then-clear-write,
 * then KILL ALL FRT interrupts with a direct write (no RMW on TIER).
 * Sync the shadow so motor_*Forward() can re-enable cleanly later. */

void ICIA_FRT(void) __attribute__((interrupt_handler));
void ICIA_FRT(void)
{
	FRT.TCSR.BYTE &= (uint8_t)~0x20;   /* Clear ICFA flag (H8S sequence) */
	shadow_frt_tier = 0;                 /* Sync shadow */
	FRT.TIER.BYTE = 0;                  /* Kill ALL FRT IRQs (no RMW) */
}

void ICIB_FRT(void) __attribute__((interrupt_handler));
void ICIB_FRT(void)
{
	FRT.TCSR.BYTE &= (uint8_t)~0x40;   /* Clear ICFB flag */
	shadow_frt_tier = 0;
	FRT.TIER.BYTE = 0;
}

void ICIC_FRT(void) __attribute__((interrupt_handler));
void ICIC_FRT(void)
{
	FRT.TCSR.BYTE &= (uint8_t)~0x80;   /* Clear ICFC flag */
	shadow_frt_tier = 0;
	FRT.TIER.BYTE = 0;
}

void FOVI_FRT(void) __attribute__((interrupt_handler));
void FOVI_FRT(void)
{
	FRT.TCSR.BYTE &= (uint8_t)~0x02;   /* Clear OVF flag */
	shadow_frt_tier = 0;
	FRT.TIER.BYTE = 0;
}
