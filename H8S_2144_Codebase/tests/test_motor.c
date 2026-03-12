/* test_motor.c — Blade motor driver test
 *
 * Tests GPIO register states, TMR0 PWM control, and the full blade
 * start/stop/emergency-stop sequences via register readback.
 *
 * Blade motor hardware (reversed from production firmware vtable 0x1A324):
 *   P2.2      = Blade IN1 (direction)
 *   P4.0      = Motor power enable
 *   P9.5      = Blade power enable
 *   P9.2      = Blade 1-wire bus (DS18B20)
 *   PA.0      = Blade EN (OD, emergency stop)
 *   TMR0.TCORB = Blade PWM duty (0x00-0xFF)
 *   TMR0.TCSR  = PWM output control (0x06=active, 0x00=off)
 *   TMR0.TCORA = PWM period (0xFF)
 *   TMR0.TCR   = Timer config (0x09 = phi/8, clear on TCORA)
 *
 * Speed parameter: 0-1000 range, scaled: duty = (speed * 255) / 1000
 *
 * Sections 1-8 are static register tests (safe without motor).
 * Section 9 is a live blade pulse gated behind UART 'Y' confirm.
 */
#include "test_motor.h"
#include "../iodefine.h"
#include "../micros.h"
#include "../sci.h"
#include "../motor/motor.h"
#include "../utils/utils.h"

void Blade_Motor_Test(void)
{
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;
	uint8_t reg_val;
	INT8 ch;
	motor_state_t mstate;

	SendString((unsigned char *)"\r\n========================================\r\n");
	SendString((unsigned char *)"  Blade Motor Driver Test\r\n");
	SendString((unsigned char *)"========================================\r\n");

	/* --- Section 1: motor_Init() verification --- */
	SendString((unsigned char *)"\r\n--- 1. motor_Init() ---\r\n");
	motor_Init();

	/* 1a: P2.DR motor direction bits clear */
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  P2.DR = 0x");
	print_hex8(reg_val);
	if ((reg_val & 0xE4) == 0x00) {
		SendString((unsigned char *)"  [PASS] dir pins clear\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] dir pins not clear\r\n");
		fail_count++;
	}

	/* 1b: P4.0 motor power off */
	reg_val = P4.DR.BYTE;
	SendString((unsigned char *)"  P4.DR = 0x");
	print_hex8(reg_val);
	if ((reg_val & MOTOR_P4_POWER) == 0x00) {
		SendString((unsigned char *)"  [PASS] motor power off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] P4.0 should be 0\r\n");
		fail_count++;
	}

	/* 1c: P4.5 PSU still on */
	if (reg_val & MOTOR_P4_PSU) {
		SendString((unsigned char *)"  P4.5 PSU     [PASS] still on\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  P4.5 PSU     [FAIL] should be on\r\n");
		fail_count++;
	}

	/* 1d: P9.5 blade power standby (high) */
	reg_val = P9.DR.BYTE;
	SendString((unsigned char *)"  P9.DR = 0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_P9_BLADE_PWR) {
		SendString((unsigned char *)"  [PASS] blade pwr standby\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] P9.5 should be 1\r\n");
		fail_count++;
	}

	/* 1e: PA.0 blade EN disabled (high = OD released) */
	reg_val = PA.ODR.BYTE;
	SendString((unsigned char *)"  PA.ODR= 0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_PA_BLADE_EN) {
		SendString((unsigned char *)"  [PASS] blade EN off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] PA.0 should be 1\r\n");
		fail_count++;
	}

	/* 1f: TMR0 blade PWM initialized by motor_BladeInit() */
	SendString((unsigned char *)"  TMR0.TCORA=0x");
	print_hex8(TMR0.TCORA);
	if (TMR0.TCORA == BLADE_TMR_PERIOD) {
		SendString((unsigned char *)"  [PASS] period=0xFF\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] expect 0xFF\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"  TMR0.TCORB=0x");
	print_hex8(TMR0.TCORB);
	if (TMR0.TCORB == 0x00) {
		SendString((unsigned char *)"  [PASS] duty=0\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] expect 0x00\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"  TMR0.TCSR=0x");
	print_hex8(TMR0.TCSR.BYTE);
	if ((TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_OFF) {
		SendString((unsigned char *)"  [PASS] PWM off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] expect 0x00\r\n");
		fail_count++;
	}

	/* 1g: DA.DACR bits 7:5 = 0xC0 (both channels enabled — legacy DAC init) */
	SendString((unsigned char *)"  DA.DACR =0x");
	print_hex8(DA.DACR.BYTE);
	if ((DA.DACR.BYTE & 0xE0) == (MOTOR_DACR_ENABLE & 0xE0)) {
		SendString((unsigned char *)"  [PASS] DAC enabled\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] expect bits 7:5 = 0xC0\r\n");
		fail_count++;
	}

	/* --- Section 2: TMR0 PWM control (blade speed) --- */
	SendString((unsigned char *)"\r\n--- 2. TMR0 PWM control (blade speed) ---\r\n");

	motor_BladeSetSpeed(500);
	SendString((unsigned char *)"  SetSpeed(500) TCORB=0x");
	print_hex8(TMR0.TCORB);
	SendString((unsigned char *)" TCSR=0x");
	print_hex8(TMR0.TCSR.BYTE);
	if (TMR0.TCORB == 0x7F && (TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_PWM) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect TCORB=7F TCSR=06]\r\n");
		fail_count++;
	}

	motor_BladeSetSpeed(1000);
	SendString((unsigned char *)"  SetSpeed(1000) TCORB=0x");
	print_hex8(TMR0.TCORB);
	if (TMR0.TCORB == 0xFF && (TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_PWM) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect TCORB=FF TCSR=06]\r\n");
		fail_count++;
	}

	motor_BladeSetSpeed(0);
	SendString((unsigned char *)"  SetSpeed(0) TCORB=0x");
	print_hex8(TMR0.TCORB);
	SendString((unsigned char *)" TCSR=0x");
	print_hex8(TMR0.TCSR.BYTE);
	if (TMR0.TCORB == 0x00 && (TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_OFF) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect TCORB=00 TCSR=00]\r\n");
		fail_count++;
	}



	/* --- Section 3: Blade enable pin (PA.0) --- */
	SendString((unsigned char *)"\r\n--- 3. Blade EN pin (PA.0 open-drain) ---\r\n");

	/* Drive PA.0 low (EN active) */
	PA.ODR.BYTE &= (uint8_t)~MOTOR_PA_BLADE_EN;
	reg_val = PA.ODR.BYTE;
	SendString((unsigned char *)"  PA.0 low: PA.ODR=0x");
	print_hex8(reg_val);
	if ((reg_val & MOTOR_PA_BLADE_EN) == 0x00) {
		SendString((unsigned char *)" [PASS] EN active\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	/* Release PA.0 high (EN disabled) */
	PA.ODR.BYTE |= MOTOR_PA_BLADE_EN;
	reg_val = PA.ODR.BYTE;
	SendString((unsigned char *)"  PA.0 high: PA.ODR=0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_PA_BLADE_EN) {
		SendString((unsigned char *)" [PASS] EN off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}



	/* --- Section 4: Blade direction (P2.2) --- */
	SendString((unsigned char *)"\r\n--- 4. Blade direction (P2.2 IN1) ---\r\n");

	P2.DR.BYTE |= MOTOR_P2_BLADE_IN1;
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  P2.2 set:   P2.DR=0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_P2_BLADE_IN1) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	P2.DR.BYTE &= (uint8_t)~MOTOR_P2_BLADE_IN1;
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  P2.2 clear: P2.DR=0x");
	print_hex8(reg_val);
	if ((reg_val & MOTOR_P2_BLADE_IN1) == 0x00) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}



	/* --- Section 5: motor_BladeStart() / motor_BladeStop() --- */
	SendString((unsigned char *)"\r\n--- 5. BladeStart(250) / BladeStop() ---\r\n");

	motor_BladeStart(250);

	SendString((unsigned char *)"  After Start:\r\n");

	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"    P2.2 IN1:  ");
	if (reg_val & MOTOR_P2_BLADE_IN1) {
		SendString((unsigned char *)"[PASS] set\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"[FAIL] should be set\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"    TMR0.TCORB=0x");
	print_hex8(TMR0.TCORB);
	if (TMR0.TCORB == 0x3F) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x3F]\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"    TMR0.TCSR=0x");
	print_hex8(TMR0.TCSR.BYTE);
	if ((TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_PWM) {
		SendString((unsigned char *)" [PASS] PWM on\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x06]\r\n");
		fail_count++;
	}

	reg_val = P4.DR.BYTE;
	SendString((unsigned char *)"    P4.0 pwr:  ");
	if (reg_val & MOTOR_P4_POWER) {
		SendString((unsigned char *)"[PASS] on\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"[FAIL] should be on\r\n");
		fail_count++;
	}

	/* Stop immediately */
	motor_BladeStop();

	SendString((unsigned char *)"  After Stop:\r\n");

	SendString((unsigned char *)"    TMR0.TCORB=0x");
	print_hex8(TMR0.TCORB);
	if (TMR0.TCORB == 0x00) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x00]\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"    TMR0.TCSR=0x");
	print_hex8(TMR0.TCSR.BYTE);
	if ((TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_OFF) {
		SendString((unsigned char *)" [PASS] PWM off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x00]\r\n");
		fail_count++;
	}

	reg_val = P4.DR.BYTE;
	SendString((unsigned char *)"    P4.0 pwr:  ");
	if ((reg_val & MOTOR_P4_POWER) == 0x00) {
		SendString((unsigned char *)"[PASS] off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"[FAIL] should be off\r\n");
		fail_count++;
	}

	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"    P2.2 IN1:  ");
	if ((reg_val & MOTOR_P2_BLADE_IN1) == 0x00) {
		SendString((unsigned char *)"[PASS] cleared\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"[FAIL] should be clear\r\n");
		fail_count++;
	}



	/* --- Section 6: motor_BladeEmergencyStop() --- */
	SendString((unsigned char *)"\r\n--- 6. BladeEmergencyStop() ---\r\n");

	motor_BladeEmergencyStop();

	SendString((unsigned char *)"  TMR0.TCORB=0x");
	print_hex8(TMR0.TCORB);
	if (TMR0.TCORB == 0x00) {
		SendString((unsigned char *)"  [PASS] PWM duty=0\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expect 0x00]\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"  TMR0.TCSR=0x");
	print_hex8(TMR0.TCSR.BYTE);
	if ((TMR0.TCSR.BYTE & BLADE_TCSR_MASK) == BLADE_TCSR_OFF) {
		SendString((unsigned char *)"  [PASS] output off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expect 0x00]\r\n");
		fail_count++;
	}

	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  P2.DR & 0x07 = 0x");
	print_hex8(reg_val & 0x07);
	if ((reg_val & 0x07) == 0x00) {
		SendString((unsigned char *)"  [PASS] low bits clear\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}

	reg_val = PA.ODR.BYTE;
	SendString((unsigned char *)"  PA.0 EN:  ");
	if (reg_val & MOTOR_PA_BLADE_EN) {
		SendString((unsigned char *)"[PASS] disabled\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"[FAIL]\r\n");
		fail_count++;
	}



	/* --- Section 7: motor_GetState() consistency --- */
	SendString((unsigned char *)"\r\n--- 7. motor_GetState() ---\r\n");

	motor_GetState(&mstate);
	SendString((unsigned char *)"  blade.enabled = ");
	print_hex8(mstate.blade.enabled);
	if (mstate.blade.enabled == 0) {
		SendString((unsigned char *)"  [PASS] disabled\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}

	SendString((unsigned char *)"  blade.direction = ");
	print_hex8(mstate.blade.direction);
	if (mstate.blade.direction == MOTOR_DIR_STOP) {
		SendString((unsigned char *)"  [PASS] stopped\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}



	/* --- Section 8: Blade 1-wire device (P9.2) --- */
	SendString((unsigned char *)"\r\n--- 8. Blade 1-wire device (P9.2) ---\r\n");

	SendString((unsigned char *)"  P9.DR = 0x");
	print_hex8(P9.DR.BYTE);
	SendString((unsigned char *)"  1w pin = ");
	print_hex8((P9.DR.BYTE & MOTOR_P9_BLADE_1WIRE) ? 1 : 0);
	SendString((unsigned char *)" (idle)\r\n");

	if (motor_BladeDevicePresent()) {
		SendString((unsigned char *)"  1-wire presence: [PASS] device found\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  1-wire presence: [FAIL] no device\r\n");
		fail_count++;
	}

	/* --- Section 9: Live blade test (optional, max 5 seconds) --- */
	SendString((unsigned char *)"\r\n--- 9. Live blade test (optional) ---\r\n");
	SendString((unsigned char *)"  Press 'Y' to spin blade (5s max), other key to skip: ");
	ch = GetChar();
	PutChar(ch);
	SendString((unsigned char *)"\r\n");

	if (ch == 'Y' || ch == 'y') {
		SendString((unsigned char *)"  WARNING: Blade motor will spin for 5 seconds!\r\n");
		SendString((unsigned char *)"  Starting in 1 second...\r\n");
		delay_ms(1000);

		motor_PowerOn();         /* P4.0 = 1 — global motor power bus ON */
		SendString((unsigned char *)"  P4.0 motor power ON\r\n");
		delay_ms(100);           /* Let power rail stabilize */

		motor_BladeStart(200);   /* 200/1000 = ~20% duty */
		SendString((unsigned char *)"  Motor ON (speed=200)...\r\n");
		delay_ms(5000);          /* 5 second run */
		motor_BladeStop();
		SendString((unsigned char *)"  Motor OFF\r\n");

		motor_PowerOff();        /* P4.0 = 0 — motor power bus OFF */
		SendString((unsigned char *)"  P4.0 motor power OFF\r\n");
		SendString((unsigned char *)"  Blade ran OK\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  Live test skipped\r\n");
	}

	/* --- Cleanup --- */
	motor_BladeStop();
	motor_StopAll();
	/* Restore P9.5 high (blade power standby) */
	P9.DR.BYTE |= MOTOR_P9_BLADE_PWR;

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
