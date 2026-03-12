/* test_i2c.c — I2C bus diagnostic test
 *
 * Low-level I2C bus diagnostic for both buses:
 *
 * Bus 1 (SCL=P8.6, SDA=P9.7, split-port):
 *   1. P8/P9 register dump (SCL/SDA pin state)
 *   2. PA.6/PA.7 info dump (open-drain, NOT I2C Bus 2)
 *   3. Re-init I2C bus, check for errors
 *   4. SCL (P8.6) pin toggle test
 *   5. SDA (P9.7) pin toggle test
 *   6. Full I2C bus scan (0x08..0x77)
 *   7. Direct DS1307 probe at address 0xD0
 *   8. Manual I2C transaction trace
 *   9. Re-init Bus 1 for clean state
 *
 * Bus 2 (SCL=PB.7, SDA=PB.6, same-port open-drain):
 *  10. Init Bus 2, pin idle check, accel probe, full scan
 */
#include "test_i2c.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../i2c/i2c.h"
#include "../rtc/rtc.h"
#include "../motor/motor.h"
#include "../utils/utils.h"


/* ================================================================
 * I2C Bus 1 initialization for RTC (DS1307) + EEPROM
 *
 * Pin assignments confirmed by vtable dispatch analysis:
 *   i2c_ClockHigh/Low use vtable[0,1] → P8.6 functions → SCL = P8.6
 *   i2c_DataHigh/Low  use vtable[2,3] → P9.7 functions → SDA = P9.7
 *
 * Original firmware pin functions (i2c_bus1_* at 0x122A8-0x12374):
 *   i2c_bus1_ReleaseSCL  (0x122A8): bShadow_P8_DDR &= ~0x40; P8_DDR = shadow
 *   i2c_bus1_DriveSCL_Low(0x122D0): P8_DR &= 0xBF; bShadow_P8_DDR |= 0x40
 *   i2c_bus1_ReleaseSDA  (0x12306): bShadow_P9_DDR &= ~0x80; P9_DDR = shadow
 *   i2c_bus1_DriveSDA_Low(0x1232E): P9_DR &= 0x7F; bShadow_P9_DDR |= 0x80
 *   i2c_bus1_ReadSCL     (0x12364): return (P8_DR >> 6) & 1
 *   i2c_bus1_ReadSDA     (0x12374): return (P9_DR >> 7)
 *
 * Vtable at ROM 0x1A270, context at RAM 0xFFE096 (startvars init).
 *
 * NOTE: SCL and SDA are on DIFFERENT ports! The i2c_bus_t struct
 * uses split-port design with separate register pointers for each pin.
 *
 * Shadow registers (pointer-based, shared with motor.h):
 *   scl_ddr_shadow -> &shadow_p8_ddr (global, init 0x0F)
 *   sda_ddr_shadow -> &shadow_p9_ddr (global, init 0x23)
 *   scl_dr_shadow  -> &shadow_p8_dr  (global, I2C/misc outputs)
 *   sda_dr_shadow  -> &shadow_p9_dr  (global, shares motor IN2 pins!)
 *
 * P9.DR shares motor direction pins (P9.0/P9.1) and blade power (P9.5),
 * so SDA DR shadow is REQUIRED to prevent RMW corruption during motor EMI.
 * ================================================================ */
void i2c_bus1_init(void)
{
	i2c_bus_t bus1;

	/* SCL = P8.6  (confirmed: vtable[0]=ReleaseSCL → i2c_bus1_ReleaseSCL on P8.6)
	 * DDR/DR shadows point to motor.h globals — single authoritative copy.
	 * P8.DR shares misc outputs but is safe during motor EMI (no IN2 bits).
	 * We still shadow it for consistency with the shadow architecture. */
	bus1.scl_ddr        = (volatile uint8_t *)0xFFFFBD;  /* P8.DDR */
	bus1.scl_dr         = &P8.DR.BYTE;
	bus1.scl_pin        = &P8.DR.BYTE;    /* P8 DR is readable for pin state */
	bus1.scl_mask       = 0x40;            /* bit 6 */
	bus1.scl_ddr_shadow = &shadow_p8_ddr;  /* -> motor.h global (0x0F init) */
	bus1.scl_dr_shadow  = &shadow_p8_dr;   /* -> motor.h global */

	/* SDA = P9.7  (confirmed: vtable[2]=ReleaseSDA → i2c_bus1_ReleaseSDA on P9.7)
	 * P9.DR shares motor IN2 (P9.0/P9.1) and blade power (P9.5).
	 * DR shadow is REQUIRED to avoid RMW corruption during motor EMI. */
	bus1.sda_ddr        = &P9.DDR;
	bus1.sda_dr         = &P9.DR.BYTE;
	bus1.sda_pin        = &P9.DR.BYTE;    /* P9 DR is readable for pin state */
	bus1.sda_mask       = 0x80;            /* bit 7 */
	bus1.sda_ddr_shadow = &shadow_p9_ddr;  /* -> motor.h global (0x23 init) */
	bus1.sda_dr_shadow  = &shadow_p9_dr;   /* -> motor.h global */

	bus1.error = 0;

	i2c_Init(I2C_BUS1, &bus1);
	rtc_Init();
}

/* ----------------------------------------------------------------
 * i2c_bus2_init - Initialize I2C Bus 2 (accelerometer on Port B)
 *
 * Bus 2 uses Port B open-drain for both SCL (PB.7) and SDA (PB.6).
 * Pin assignment confirmed by vtable dispatch: vtable[0,1] (SCL ops)
 * → bit 7 functions; vtable[2,3] (SDA ops) → bit 6 functions.
 * Since both pins share PB.DDR, both DDR shadow pointers point to
 * the same variable — automatic sync.
 *
 * Register map (from Ghidra i2c_bus2_* decompilation):
 *   PB.ODR (0xFFFFBC) — data / open-drain control (0=low, 1=release)
 *   PB.DDR (0xFFFFBE) — direction (1=output, 0=input/float)
 *   PIN readback (0xFFFFBD) — H8S DDR read returns PIN state
 *
 * HardwareSetup defaults: PB.ODR=0xFF, PB.DDR=0x00 (all input, OD=1)
 * ---------------------------------------------------------------- */
void i2c_bus2_init(void)
{
	i2c_bus_t bus2;

	/* Port B DDR shadow — local to Bus 2 (PB not used by motor driver).
	 * Both SDA and SCL DDR shadow pointers point to the SAME variable,
	 * so modifying one automatically keeps the other in sync. */
	static uint8_t shadow_pb_ddr = 0x00;  /* PB.DDR base from HardwareSetup */

	/* SCL = PB.7  (confirmed: vtable[0]=ReleaseSCL → i2c_bus2_ReleaseSCL on bit 7)
	 * PB.ODR is readable and doesn't share motor-critical bits,
	 * so DR shadow is NULL — direct RMW on PB.ODR is safe. */
	bus2.scl_ddr        = (volatile uint8_t *)0xFFFFBE;  /* PB.DDR */
	bus2.scl_dr         = (volatile uint8_t *)0xFFFFBC;  /* PB.ODR */
	bus2.scl_pin        = (volatile uint8_t *)0xFFFFBD;  /* PIN readback */
	bus2.scl_mask       = 0x80;            /* bit 7 */
	bus2.scl_ddr_shadow = &shadow_pb_ddr;  /* -> shared PB.DDR shadow */
	bus2.scl_dr_shadow  = (uint8_t *)0;    /* NULL = direct RMW OK */

	/* SDA = PB.6  (confirmed: vtable[2]=ReleaseSDA → i2c_bus2_ReleaseSDA on bit 6)
	 * Same port — DDR shadow pointers share same variable. */
	bus2.sda_ddr        = (volatile uint8_t *)0xFFFFBE;  /* PB.DDR — SAME */
	bus2.sda_dr         = (volatile uint8_t *)0xFFFFBC;  /* PB.ODR — SAME */
	bus2.sda_pin        = (volatile uint8_t *)0xFFFFBD;  /* PIN readback — SAME */
	bus2.sda_mask       = 0x40;            /* bit 6 */
	bus2.sda_ddr_shadow = &shadow_pb_ddr;  /* -> SAME shared PB.DDR shadow */
	bus2.sda_dr_shadow  = (uint8_t *)0;    /* NULL = direct RMW OK */

	bus2.error = 0;

	i2c_Init(I2C_BUS2, &bus2);
}


/* ================================================================
 * I2C_Test()
 * ================================================================ */
void I2C_Test(void)
{
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;
	uint8_t pin_val;
	uint8_t found_count = 0;
	uint8_t addr;
	uint8_t err;

	SendString((unsigned char *)"\r\n");
	SendString((unsigned char *)"========================================\r\n");
	SendString((unsigned char *)"  I2C Bus Diagnostic\r\n");
	SendString((unsigned char *)"========================================\r\n");

	/* --- Section 1: I2C Bus 1 pin state (SCL=P8.6, SDA=P9.7) --- */
	SendString((unsigned char *)"\r\n--- I2C Bus 1 pins (SCL=P8.6, SDA=P9.7) ---\r\n");

	pin_val = P8.DR.BYTE;
	SendString((unsigned char *)"  P8.DR  = 0x");
	print_hex8(pin_val);
	SendString((unsigned char *)"  SCL(bit6)=");
	PutChar((pin_val & 0x40) ? '1' : '0');
	SendString((unsigned char *)"\r\n");

	pin_val = P9.DR.BYTE;
	SendString((unsigned char *)"  P9.DR  = 0x");
	print_hex8(pin_val);
	SendString((unsigned char *)"  SDA(bit7)=");
	PutChar((pin_val & 0x80) ? '1' : '0');
	SendString((unsigned char *)"\r\n");

	/* STCR — check IICE bit */
	SendString((unsigned char *)"  STCR   = 0x");
	print_hex8(STCR.BYTE);
	if (STCR.BYTE & 0x10) {
		SendString((unsigned char *)"  IICE=1 (HW I2C ON - may conflict!)");
	} else {
		SendString((unsigned char *)"  IICE=0 (bit-bang OK)");
	}
	SendString((unsigned char *)"\r\n");

	/* --- Section 2: PA.6/PA.7 info (NOT I2C Bus 1) --- */
	SendString((unsigned char *)"\r\n--- PA.6/PA.7 info (NOT I2C Bus 1) ---\r\n");
	SendString((unsigned char *)"  PA.ODR = 0x");
	print_hex8(PA.ODR.BYTE);
	if ((PA.ODR.BYTE & 0xC0) == 0xC0)
		SendString((unsigned char *)"  bits 6,7 open-drain");
	SendString((unsigned char *)"\r\n");
	pin_val = PA.EQU.PIN.BYTE;
	SendString((unsigned char *)"  PA.PIN = 0x");
	print_hex8(pin_val);
	SendString((unsigned char *)"  PA.6=");
	PutChar((pin_val & 0x40) ? '1' : '0');
	SendString((unsigned char *)" PA.7=");
	PutChar((pin_val & 0x80) ? '1' : '0');
	SendString((unsigned char *)"\r\n");
	SendString((unsigned char *)"  (May be I2C Bus 2 / accel or HW IIC)\r\n");

	/* --- Section 3: Pre-I2C init (fDoPorts1 equivalent) --- */
	SendString((unsigned char *)"\r\n--- Pre-I2C init (fDoPorts1 equivalent) ---\r\n");
	SendString((unsigned char *)"  DA_DADR0 was: 0x");
	print_hex8(DA.DADR0);
	SendString((unsigned char *)"\r\n");
	DA.DADR0 = 0xFF;   /* Full-scale DAC output — matches fDoPorts1 */
	delay_ms(50);       /* Power-up settling (original uses ~15ms) */
	SendString((unsigned char *)"  DA_DADR0 now: 0xFF\r\n");

	/* Also check SCL/SDA after DAC change */
	{
		uint8_t scl_after = (P8.DR.BYTE & 0x40) ? 1 : 0;
		uint8_t sda_after = (P9.DR.BYTE & 0x80) ? 1 : 0;
		SendString((unsigned char *)"  After DAC: SCL(P8.6)=");
		PutChar('0' + scl_after);
		SendString((unsigned char *)" SDA(P9.7)=");
		PutChar('0' + sda_after);
		SendString((unsigned char *)"\r\n");
	}

	/* --- Section 4: Re-init I2C Bus 1 --- */
	SendString((unsigned char *)"\r\n--- Re-init I2C Bus 1 ---\r\n");
	i2c_ClearError(I2C_BUS1);
	i2c_bus1_init();

	err = i2c_GetError(I2C_BUS1);
	SendString((unsigned char *)"  i2c_GetError = 0x");
	print_hex8(err);
	if (err == 0) {
		SendString((unsigned char *)"  [PASS] no errors");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] init error");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* Check P8/P9 pin states after init (both should be HIGH = released) */
	{
		uint8_t scl_bit = (P8.DR.BYTE & 0x40) ? 1 : 0;
		uint8_t sda_bit = (P9.DR.BYTE & 0x80) ? 1 : 0;
		SendString((unsigned char *)"  After init: SCL(P8.6)=");
		PutChar('0' + scl_bit);
		SendString((unsigned char *)" SDA(P9.7)=");
		PutChar('0' + sda_bit);
		if (sda_bit && scl_bit) {
			SendString((unsigned char *)"  [PASS] both HIGH");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL]");
			if (!sda_bit) SendString((unsigned char *)" SDA LOW");
			if (!scl_bit) SendString((unsigned char *)" SCL LOW");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Section 5: SCL (P8.6) toggle test --- */
	SendString((unsigned char *)"\r\n--- SCL (P8.6) toggle test ---\r\n");
	{
		volatile uint8_t *p8_ddr = (volatile uint8_t *)0xFFFFBD;

		/* Release SCL: DDR bit 6 = 0 (input, pull-up brings HIGH) */
		shadow_p8_ddr &= ~0x40;
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);
		pin_val = P8.DR.BYTE;
		SendString((unsigned char *)"  Release: SCL=");
		PutChar((pin_val & 0x40) ? '1' : '0');
		if (pin_val & 0x40) {
			SendString((unsigned char *)"  [PASS] HIGH");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] stays LOW");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");

		/* Drive SCL low: DR=0 via shadow, then DDR=output */
		shadow_p8_dr &= ~0x40;
		P8.DR.BYTE = shadow_p8_dr;
		shadow_p8_ddr |= 0x40;
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);
		pin_val = P8.DR.BYTE;
		SendString((unsigned char *)"  Drive:   SCL=");
		PutChar((pin_val & 0x40) ? '1' : '0');
		if (!(pin_val & 0x40)) {
			SendString((unsigned char *)"  [PASS] LOW");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] still HIGH");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");

		/* Release SCL again */
		shadow_p8_ddr &= ~0x40;
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);
		pin_val = P8.DR.BYTE;
		SendString((unsigned char *)"  Release: SCL=");
		PutChar((pin_val & 0x40) ? '1' : '0');
		if (pin_val & 0x40) {
			SendString((unsigned char *)"  [PASS] back HIGH");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] stuck LOW");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Section 6: SDA (P9.7) toggle test --- */
	SendString((unsigned char *)"\r\n--- SDA (P9.7) toggle test ---\r\n");
	{
		/* Release SDA: DDR bit 7 = 0 (input, pull-up brings HIGH) */
		shadow_p9_ddr &= ~0x80;
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);
		pin_val = P9.DR.BYTE;
		SendString((unsigned char *)"  Release: SDA=");
		PutChar((pin_val & 0x80) ? '1' : '0');
		if (pin_val & 0x80) {
			SendString((unsigned char *)"  [PASS] HIGH");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] stays LOW");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");

		/* Drive SDA low: DR=0 via shadow, then DDR=output */
		shadow_p9_dr &= ~0x80;
		P9.DR.BYTE = shadow_p9_dr;
		shadow_p9_ddr |= 0x80;
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);
		pin_val = P9.DR.BYTE;
		SendString((unsigned char *)"  Drive:   SDA=");
		PutChar((pin_val & 0x80) ? '1' : '0');
		if (!(pin_val & 0x80)) {
			SendString((unsigned char *)"  [PASS] LOW");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] still HIGH");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");

		/* Release SDA again */
		shadow_p9_ddr &= ~0x80;
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);
		pin_val = P9.DR.BYTE;
		SendString((unsigned char *)"  Release: SDA=");
		PutChar((pin_val & 0x80) ? '1' : '0');
		if (pin_val & 0x80) {
			SendString((unsigned char *)"  [PASS] back HIGH");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] stuck LOW");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Section 7: Manual I2C transaction trace --- */
	SendString((unsigned char *)"\r\n--- Manual I2C trace: START + 0xD0 ---\r\n");
	{
		volatile uint8_t *p8_ddr = (volatile uint8_t *)0xFFFFBD;
		uint8_t sda, scl;
		uint8_t byte_out = 0xD0; /* DS1307 write addr */
		uint8_t bit_i;
		uint8_t ack_bit;

		/* Helper macro: read bus state (SCL=P8.6, SDA=P9.7) */
		#define READ_BUS() \
			scl = (P8.DR.BYTE & 0x40) ? 1 : 0; \
			sda = (P9.DR.BYTE & 0x80) ? 1 : 0

		/* Step 0: Ensure idle (both released / HIGH) */
		shadow_p8_ddr &= ~0x40;       /* SCL = input (release P8.6) */
		*p8_ddr = shadow_p8_ddr;
		shadow_p9_ddr &= ~0x80;       /* SDA = input (release P9.7) */
		P9.DDR = shadow_p9_ddr;
		delay_ms(2);
		READ_BUS();
		SendString((unsigned char *)"  Idle:     SDA=");
		PutChar('0' + sda);
		SendString((unsigned char *)" SCL=");
		PutChar('0' + scl);
		if (sda && scl)
			SendString((unsigned char *)" OK");
		else
			SendString((unsigned char *)" BAD(expect 1,1)");
		SendString((unsigned char *)"\r\n");

		/* Step 1: START — pull SDA(P9.7) low while SCL(P8.6) high */
		shadow_p9_dr &= ~0x80;        /* SDA DR=0 via shadow */
		P9.DR.BYTE = shadow_p9_dr;
		shadow_p9_ddr |= 0x80;        /* SDA = output (drives P9.7 low) */
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);
		READ_BUS();
		SendString((unsigned char *)"  START:    SDA=");
		PutChar('0' + sda);
		SendString((unsigned char *)" SCL=");
		PutChar('0' + scl);
		if (!sda && scl)
			SendString((unsigned char *)" OK(SDA fell)");
		else
			SendString((unsigned char *)" BAD(expect 0,1)");
		SendString((unsigned char *)"\r\n");

		/* Step 2: Pull SCL(P8.6) low (begin clocking) */
		shadow_p8_dr &= ~0x40;        /* SCL DR=0 via shadow */
		P8.DR.BYTE = shadow_p8_dr;
		shadow_p8_ddr |= 0x40;        /* SCL = output (drives P8.6 low) */
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);
		READ_BUS();
		SendString((unsigned char *)"  SCL low:  SDA=");
		PutChar('0' + sda);
		SendString((unsigned char *)" SCL=");
		PutChar('0' + scl);
		SendString((unsigned char *)"\r\n");

		/* Step 3: Clock out 8 bits of 0xD0, MSB first */
		SendString((unsigned char *)"  Bits[7:0]: ");
		for (bit_i = 0; bit_i < 8; bit_i++) {
			/* Set SDA(P9.7) for this bit */
			if (byte_out & 0x80) {
				/* SDA = 1 (release: DDR to input) */
				shadow_p9_ddr &= ~0x80;
				P9.DDR = shadow_p9_ddr;
			} else {
				/* SDA = 0 (drive low: DR=0 via shadow, then DDR=output) */
				shadow_p9_dr &= ~0x80;
				P9.DR.BYTE = shadow_p9_dr;
				shadow_p9_ddr |= 0x80;
				P9.DDR = shadow_p9_ddr;
			}
			delay_ms(1);

			/* SCL(P8.6) high (release: DDR to input) */
			shadow_p8_ddr &= ~0x40;
			*p8_ddr = shadow_p8_ddr;
			delay_ms(1);

			/* Read SDA at SCL high */
			READ_BUS();
			PutChar('0' + sda);

			/* SCL(P8.6) low (drive: DR=0 via shadow, DDR=output) */
			shadow_p8_dr &= ~0x40;
			P8.DR.BYTE = shadow_p8_dr;
			shadow_p8_ddr |= 0x40;
			*p8_ddr = shadow_p8_ddr;
			delay_ms(1);

			byte_out <<= 1;
		}
		SendString((unsigned char *)" (expect 11010000)\r\n");

		/* Step 4: ACK clock — release SDA(P9.7), clock once, read ACK */
		shadow_p9_ddr &= ~0x80;       /* Release SDA(P9.7) for slave */
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);

		/* SCL(P8.6) high (release) */
		shadow_p8_ddr &= ~0x40;
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);

		READ_BUS();
		ack_bit = sda;
		SendString((unsigned char *)"  ACK clk:  SDA=");
		PutChar('0' + sda);
		SendString((unsigned char *)" SCL=");
		PutChar('0' + scl);
		if (!ack_bit)
			SendString((unsigned char *)" ACK! (device present)");
		else
			SendString((unsigned char *)" NACK (no device)");
		SendString((unsigned char *)"\r\n");

		/* SCL(P8.6) low (drive) */
		shadow_p8_dr &= ~0x40;
		P8.DR.BYTE = shadow_p8_dr;
		shadow_p8_ddr |= 0x40;
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);

		/* Step 5: STOP — SDA(P9.7) low, SCL(P8.6) high, then SDA(P9.7) high */
		shadow_p9_dr &= ~0x80;
		P9.DR.BYTE = shadow_p9_dr;
		shadow_p9_ddr |= 0x80;
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);

		shadow_p8_ddr &= ~0x40;
		*p8_ddr = shadow_p8_ddr;
		delay_ms(1);

		shadow_p9_ddr &= ~0x80;
		P9.DDR = shadow_p9_ddr;
		delay_ms(1);

		READ_BUS();
		SendString((unsigned char *)"  STOP:     SDA=");
		PutChar('0' + sda);
		SendString((unsigned char *)" SCL=");
		PutChar('0' + scl);
		SendString((unsigned char *)"\r\n");

		if (!ack_bit) {
			pass_count++;
		} else {
			fail_count++;
		}

		#undef READ_BUS
	}

	/* --- Section 8: I2C bus scan --- */
	SendString((unsigned char *)"\r\n--- I2C bus scan (7-bit addresses) ---\r\n");

	/* Re-init bus clean before scan */
	i2c_ClearError(I2C_BUS1);
	i2c_bus1_init();

	found_count = 0;
	SendString((unsigned char *)"     0  1  2  3  4  5  6  7");
	SendString((unsigned char *)"  8  9  A  B  C  D  E  F\r\n");

	for (addr = 0; addr < 0x80; addr++) {
		/* Row header */
		if ((addr & 0x0F) == 0) {
			SendString((unsigned char *)"  ");
			print_hex8(addr);
			SendString((unsigned char *)":");
		}

		if (addr < 0x08 || addr > 0x77) {
			/* Reserved address ranges */
			SendString((unsigned char *)" ..");
		} else {
			uint8_t ack;
			i2c_ClearError(I2C_BUS1);
			i2c_Start(I2C_BUS1);
			ack = i2c_WriteByte(I2C_BUS1, (uint8_t)(addr << 1));
			i2c_Stop(I2C_BUS1);

			if (ack && (i2c_GetError(I2C_BUS1) == 0)) {
				SendString((unsigned char *)" ");
				print_hex8(addr);
				found_count++;
			} else {
				SendString((unsigned char *)" --");
			}
		}

		if ((addr & 0x0F) == 0x0F) {
			SendString((unsigned char *)"\r\n");
		}
	}

	SendString((unsigned char *)"\r\n  Found ");
	print_hex8(found_count);
	SendString((unsigned char *)" device(s)\r\n");

	if (found_count > 0) {
		SendString((unsigned char *)"  Known 7-bit addresses:\r\n");
		SendString((unsigned char *)"    0x50-57 = 24Cxx EEPROM\r\n");
		SendString((unsigned char *)"    0x68    = DS1307 RTC\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  No devices found on bus!\r\n");
		fail_count++;
	}

	/* --- Section 9: Direct DS1307 probe --- */
	SendString((unsigned char *)"\r\n--- Direct DS1307 probe (0xD0) ---\r\n");
	{
		uint8_t ack;
		i2c_ClearError(I2C_BUS1);
		i2c_Start(I2C_BUS1);
		ack = i2c_WriteByte(I2C_BUS1, 0xD0);  /* DS1307 write addr */
		err = i2c_GetError(I2C_BUS1);
		i2c_Stop(I2C_BUS1);

		SendString((unsigned char *)"  START + 0xD0: ");
		if (ack)
			SendString((unsigned char *)"ACK");
		else
			SendString((unsigned char *)"NACK");
		SendString((unsigned char *)"  err=0x");
		print_hex8(err);
		if (ack && err == 0) {
			SendString((unsigned char *)"  [PASS]");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* Restore Bus 1 to clean idle state */
	i2c_ClearError(I2C_BUS1);
	i2c_bus1_init();

	/* --- Section 10: I2C Bus 2 probe (accelerometer bus) --- */
	SendString((unsigned char *)"\r\n--- I2C Bus 2 (accel) driver test ---\r\n");
	SendString((unsigned char *)"  PB: SCL=PB.7, SDA=PB.6 (open-drain, shared DDR)\r\n");
	{
		volatile uint8_t *p8_pin = (volatile uint8_t *)0xFFFFBD;
		uint8_t pin_reg, sda, scl;
		uint8_t bus2_found = 0;
		uint8_t bus2_err;

		/* Init Bus 2 through driver (shared DDR auto-detected) */
		i2c_bus2_init();
		bus2_err = i2c_GetError(I2C_BUS2);

		/* Quick pin check — read idle state after driver init */
		pin_reg = *p8_pin;
		scl = (pin_reg >> 7) & 1;
		sda = (pin_reg >> 6) & 1;
		SendString((unsigned char *)"  Idle: SDA=");
		PutChar('0' + sda);
		SendString((unsigned char *)" SCL=");
		PutChar('0' + scl);
		SendString((unsigned char *)" err=0x");
		print_hex8(bus2_err);
		if (sda && scl && bus2_err == 0) {
			SendString((unsigned char *)" OK\r\n");
			pass_count++;
		} else {
			SendString((unsigned char *)" FAIL\r\n");
			fail_count++;
		}

		/* Common accelerometer addresses (7-bit) */
		{
			static const uint8_t accel_addrs[] = {
				0x1C, /* MMA7455L */
				0x1D, /* MMA7455L alt */
				0x18, /* LIS3DH */
				0x19, /* LIS3DH alt */
				0x4C, /* MMA8451Q */
				0x4D, /* MMA8451Q alt */
				0x53, /* ADXL345 */
				0x00  /* sentinel */
			};
			uint8_t ai;

			SendString((unsigned char *)"  Accel addresses:\r\n");
			for (ai = 0; accel_addrs[ai] != 0x00; ai++) {
				uint8_t a = accel_addrs[ai];
				SendString((unsigned char *)"    0x");
				print_hex8(a);
				SendString((unsigned char *)": ");

				i2c_Start(I2C_BUS2);
				if (i2c_WriteByte(I2C_BUS2, (uint8_t)(a << 1))) {
					SendString((unsigned char *)"ACK!\r\n");
					bus2_found++;
				} else {
					SendString((unsigned char *)"NACK\r\n");
				}
				i2c_Stop(I2C_BUS2);
			}
		}

		/* Full bus scan via driver 0x08-0x77 */
		SendString((unsigned char *)"  Full scan 0x08-0x77: ");
		for (addr = 0x08; addr <= 0x77; addr++) {
			i2c_Start(I2C_BUS2);
			if (i2c_WriteByte(I2C_BUS2, (uint8_t)(addr << 1))) {
				SendString((unsigned char *)"0x");
				print_hex8(addr);
				SendString((unsigned char *)" ");
				bus2_found++;
			}
			i2c_Stop(I2C_BUS2);
		}
		if (bus2_found == 0)
			SendString((unsigned char *)"none");
		SendString((unsigned char *)"\r\n");

		SendString((unsigned char *)"  Bus 2 found: ");
		print_hex8(bus2_found);
		SendString((unsigned char *)" device(s)\r\n");

		if (bus2_found > 0)
			pass_count++;
		else
			fail_count++;
	}

	/* --- Summary --- */
	SendString((unsigned char *)"\r\n========================================\r\n");
	SendString((unsigned char *)"  Results: ");
	print_hex8(pass_count);
	SendString((unsigned char *)" passed, ");
	print_hex8(fail_count);
	SendString((unsigned char *)" failed\r\n");
	if (fail_count == 0)
		SendString((unsigned char *)"  ALL TESTS PASSED\r\n");
	else
		SendString((unsigned char *)"  *** FAILURES DETECTED ***\r\n");
	SendString((unsigned char *)"========================================\r\n");
}
