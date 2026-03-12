/* test_keyboard.c -- 5-key matrix keypad test
 *
 * Tests the keyboard matrix via P8[3:0] columns / P1[4:0] rows:
 *   1. GPIO config verify (P1 pull-ups, P8 DDR shadow)
 *   2. Raw scan display (press a key within 10s)
 *   3. Interactive LCD test (20s live display)
 *
 * Hardware: 5-key keypad on P8[3:0] columns, P1[4:0] rows
 */
#include "test_keyboard.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../keyboard/keyboard.h"
#include "../motor/motor.h"
#include "../lcd/lcd.h"
#include "../timer/timer.h"
#include "../utils/utils.h"

/* Key code to display name */
static const char *key_name(uint8_t key)
{
	switch (key) {
	case KEY_ENTER: return "ENTER";
	case KEY_DOWN:  return "DOWN ";
	case KEY_HOME:  return "HOME ";
	case KEY_PAUSE: return "PAUSE";
	case KEY_UP:    return "UP   ";
	default:        return "NONE ";
	}
}

void Keyboard_Test(void)
{
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;

	SendString((unsigned char *)"\r\n");
	SendString((unsigned char *)"========================================\r\n");
	SendString((unsigned char *)"  Keyboard (5-key Matrix) Test\r\n");
	SendString((unsigned char *)"========================================\r\n");

	kbd_Init();

	/* --- Section 1: GPIO config verify --- */
	SendString((unsigned char *)"\r\n--- 1. GPIO config ---\r\n");
	{
		uint8_t pcr = P1.PCR.BYTE;
		uint8_t ddr = shadow_p8_ddr;

		SendString((unsigned char *)"  P1.PCR = 0x");
		print_hex8(pcr);
		if ((pcr & 0x1F) == 0x1F) {
			SendString((unsigned char *)"  [PASS] pull-ups on P1[4:0]\r\n");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] expected pull-ups 0x1F on lower 5 bits\r\n");
			fail_count++;
		}

		SendString((unsigned char *)"  shadow_p8_ddr = 0x");
		print_hex8(ddr);
		if ((ddr & 0x0F) == 0x0F) {
			SendString((unsigned char *)"  [PASS] P8[3:0] output\r\n");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] expected 0x0F lower nibble\r\n");
			fail_count++;
		}
	}

	/* --- Section 2: Raw scan display --- */
	SendString((unsigned char *)"\r\n--- 2. Raw scan (press a key within 10s) ---\r\n");
	{
		uint32_t start = GetSystemCounter();
		uint8_t  got_key = 0;

		while ((GetSystemCounter() - start) < 1000) {  /* 10 seconds */
			uint8_t key = kbd_Scan();
			if (key != KEY_NONE) {
				uint32_t raw = kbd_GetLastScan();
				SendString((unsigned char *)"  Scan result: 0x");
				print_hex32(raw);
				SendString((unsigned char *)"  Key: ");
				SendString((unsigned char *)key_name(key));
				SendString((unsigned char *)"  [PASS]\r\n");
				pass_count++;
				got_key = 1;
				/* Wait for release */
				while (kbd_Scan() != KEY_NONE)
					;
				break;
			}
		}

		if (!got_key) {
			SendString((unsigned char *)"  No key pressed (timeout)  [SKIP]\r\n");
		}
	}

	/* --- Section 3: Interactive LCD test (20s) --- */
	SendString((unsigned char *)"\r\n--- 3. Interactive LCD test (20s) ---\r\n");
	SendString((unsigned char *)"  Press keys — LCD shows name + count\r\n");
	{
		uint32_t start_tick = GetSystemCounter();
		uint32_t last_update = 0;
		uint16_t press_count = 0;
		uint8_t  last_key = KEY_NONE;
		uint16_t remaining;

		kbd_Init();  /* Reset debounce state */
		lcd_Clear();

		while (1) {
			uint32_t now = GetSystemCounter();
			if ((now - start_tick) >= 2000)  /* 20 seconds */
				break;

			/* Poll for debounced keypress */
			{
				uint8_t key = kbd_GetKey();
				if (key != KEY_NONE) {
					last_key = key;
					press_count++;
					/* Echo to serial */
					SendString((unsigned char *)"  Key: ");
					SendString((unsigned char *)key_name(key));
					SendString((unsigned char *)" (#");
					print_hex8((uint8_t)(press_count & 0xFF));
					SendString((unsigned char *)")\r\n");
				}
			}

			/* Update LCD every ~100ms */
			if ((now - last_update) >= 10) {
				last_update = now;
				remaining = (uint16_t)((2000 - (now - start_tick)) / 100);

				lcd_Print(0, "Key: %s", key_name(last_key));
				lcd_Print(1, "N:%-5d  %2us", press_count, remaining);
			}
		}

		lcd_Clear();
		SendString((unsigned char *)"  LCD interactive test complete.\r\n");
		SendString((unsigned char *)"  Total presses: ");
		print_hex8((uint8_t)(press_count & 0xFF));
		SendString((unsigned char *)"\r\n");

		if (press_count > 0) {
			SendString((unsigned char *)"  [PASS] keys detected\r\n");
			pass_count++;
		} else {
			SendString((unsigned char *)"  No keys pressed  [SKIP]\r\n");
		}
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
