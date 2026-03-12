/* test_misc.c — Misc driver test
 *
 * Tests LED toggle, backlight, DAC output, shutoff counter.
 *
 * Hardware pins:
 *   P9.5 = Status LED
 *   P4.5 = LCD backlight / power enable
 *   DA_DADR0 = Blade motor DAC
 */
#include "test_misc.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../misc/misc.h"
#include "../utils/utils.h"

void Misc_Test(void)
{
	uint8_t i;
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;

	SendString((unsigned char *)"\r\n");
	SendString((unsigned char *)"========================================\r\n");
	SendString((unsigned char *)"  Misc Driver Test\r\n");
	SendString((unsigned char *)"========================================\r\n");

	/* --- Test 1: LED toggle (visual) --- */
	SendString((unsigned char *)"\r\n--- LED toggle (P9.5) ---\r\n");
	SendString((unsigned char *)"  Blinking LED 6 times...\r\n");
	for (i = 0; i < 6; i++) {
		misc_BlinkLed();
		delay_ms(300);
	}
	SendString((unsigned char *)"  LED blink done (check visually)\r\n");

	/* --- Test 2: LED set on/off --- */
	SendString((unsigned char *)"\r\n--- LED set on/off ---\r\n");

	misc_SetLed(1);
	SendString((unsigned char *)"  misc_SetLed(1) -> P9.DR = 0x");
	print_hex8(P9.DR.BYTE);
	if (P9.DR.BYTE & 0x20) {
		SendString((unsigned char *)"  [PASS] bit5=1");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL bit5 should be 1]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	misc_SetLed(0);
	SendString((unsigned char *)"  misc_SetLed(0) -> P9.DR = 0x");
	print_hex8(P9.DR.BYTE);
	if (!(P9.DR.BYTE & 0x20)) {
		SendString((unsigned char *)"  [PASS] bit5=0");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL bit5 should be 0]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* --- Test 3: Backlight off/on (visual + register) --- */
	SendString((unsigned char *)"\r\n--- Backlight (P4.5) ---\r\n");

	misc_BacklightOff();
	SendString((unsigned char *)"  BacklightOff -> P4.DR = 0x");
	print_hex8(P4.DR.BYTE);
	if (!(P4.DR.BYTE & 0x20)) {
		SendString((unsigned char *)"  [PASS] bit5=0");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL bit5 should be 0]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	delay_ms(500);  /* Brief visible backlight-off pause */

	misc_BacklightOn();
	SendString((unsigned char *)"  BacklightOn  -> P4.DR = 0x");
	print_hex8(P4.DR.BYTE);
	if (P4.DR.BYTE & 0x20) {
		SendString((unsigned char *)"  [PASS] bit5=1");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL bit5 should be 1]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* --- Test 4: DAC output --- */
	SendString((unsigned char *)"\r\n--- DAC (DA_DADR0) ---\r\n");

	misc_SetDAC(0xFF);
	SendString((unsigned char *)"  SetDAC(0xFF) -> DA_DADR0 = 0x");
	print_hex8(DA.DADR0);
	if (DA.DADR0 == 0xFF) {
		SendString((unsigned char *)"  [PASS]");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expected 0xFF]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	misc_SetDAC(0x80);
	SendString((unsigned char *)"  SetDAC(0x80) -> DA_DADR0 = 0x");
	print_hex8(DA.DADR0);
	if (DA.DADR0 == 0x80) {
		SendString((unsigned char *)"  [PASS]");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expected 0x80]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	misc_SetDAC(0x00);
	SendString((unsigned char *)"  SetDAC(0x00) -> DA_DADR0 = 0x");
	print_hex8(DA.DADR0);
	if (DA.DADR0 == 0x00) {
		SendString((unsigned char *)"  [PASS]");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expected 0x00]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* Restore DAC to safe brake value */
	misc_SetDAC(0xFF);

	/* --- Test 5: Shutoff counter logic --- */
	SendString((unsigned char *)"\r\n--- Shutoff counter ---\r\n");

	misc_ShutoffReset();
	SendString((unsigned char *)"  ShutoffReset() called (counter=3000)\r\n");

	/* Tick once — should not be in shutoff yet */
	{
		uint8_t result = misc_ShutoffTick();
		SendString((unsigned char *)"  ShutoffTick() = ");
		print_hex8(result);
		if (result == 0) {
			SendString((unsigned char *)"  [PASS] not in shutoff");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL expected 0]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* Burn down counter to 0 by ticking 2999 more times */
	SendString((unsigned char *)"  Ticking 2999 more times...\r\n");
	for (i = 0; i < 255; i++) {      /* Can't loop 2999 in uint8, */
		uint16_t j;                   /* use nested loop */
		for (j = 0; j < 11; j++) {
			misc_ShutoffTick();
		}
	}
	/* 255*11 = 2805, need 2999-2805 = 194 more */
	{
		uint16_t j;
		for (j = 0; j < 194; j++) {
			misc_ShutoffTick();
		}
	}

	/* Now counter should be at 0, next tick should return 1 (shutoff active) */
	{
		uint8_t result = misc_ShutoffTick();
		SendString((unsigned char *)"  ShutoffTick() after 3000 = ");
		print_hex8(result);
		if (result == 1) {
			SendString((unsigned char *)"  [PASS] shutoff active");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL expected 1]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* Reset back to normal for continued operation */
	misc_ShutoffReset();
	misc_BacklightOn();
	SendString((unsigned char *)"  ShutoffReset() — restored normal operation\r\n");

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
