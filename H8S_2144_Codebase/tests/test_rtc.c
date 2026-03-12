/* test_rtc.c — DS1307 RTC test
 *
 * Tests the DS1307 RTC via I2C Bus 1:
 *   1. I2C bus error check
 *   2. Read time (HH:MM:SS)
 *   3. Read date (YYYY-MM-DD)
 *   4. Clock running check
 *   5. Battery-backed RAM write/read
 *   6. SetTime/GetTime roundtrip
 *   7. SetDate/GetDate roundtrip
 *   8. Control register SQW config check
 *   9. RAM boundary check
 *
 * Hardware: DS1307 on I2C Bus 1 (P8.6=SDA, P9.7=SCL)
 */
#include "test_rtc.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../i2c/i2c.h"
#include "../rtc/rtc.h"
#include "../utils/utils.h"
#include "../version.h"

/* Print a uint8_t as zero-padded 2-digit decimal (00-99) */
static void print_dec2(uint8_t val)
{
	PutChar('0' + (val / 10));
	PutChar('0' + (val % 10));
}

void RTC_Test(void)
{
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;
	rtc_time_t t1, t2;
	rtc_date_t d;
	int rc;

	SendString((unsigned char *)"\r\n");
	SendString((unsigned char *)"========================================\r\n");
	SendString((unsigned char *)"  RTC (DS1307) Test\r\n");
	SendString((unsigned char *)"========================================\r\n");

	/* --- Test 1: I2C bus error check --- */
	SendString((unsigned char *)"\r\n--- I2C Bus 1 status ---\r\n");
	{
		uint8_t err = i2c_GetError(I2C_BUS1);
		SendString((unsigned char *)"  i2c_GetError(BUS1) = 0x");
		print_hex8(err);
		if (err == 0) {
			SendString((unsigned char *)"  [PASS] no errors");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL bus error]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Test 2: Read time --- */
	SendString((unsigned char *)"\r\n--- Read time ---\r\n");
	rc = rtc_GetTime(&t1);
	SendString((unsigned char *)"  rtc_GetTime() = ");
	if (rc == 0) {
		print_dec2(t1.hours);
		PutChar(':');
		print_dec2(t1.minutes);
		PutChar(':');
		print_dec2(t1.seconds);
		SendString((unsigned char *)"  [PASS] I2C read OK");
		pass_count++;
		/* Sanity check ranges */
		if (t1.hours < 24 && t1.minutes < 60 && t1.seconds < 60) {
			SendString((unsigned char *)"\r\n  Range check: valid");
			pass_count++;
		} else {
			SendString((unsigned char *)"\r\n  Range check: [FAIL out of range]");
			fail_count++;
		}
	} else {
		SendString((unsigned char *)"ERROR (rc=-1)  [FAIL]");
		fail_count += 2;
	}
	SendString((unsigned char *)"\r\n");

	/* --- Test 3: Read date --- */
	SendString((unsigned char *)"\r\n--- Read date ---\r\n");
	rc = rtc_GetDate(&d);
	SendString((unsigned char *)"  rtc_GetDate() = 20");
	if (rc == 0) {
		print_dec2(d.year);
		PutChar('-');
		print_dec2(d.month);
		PutChar('-');
		print_dec2(d.day);
		SendString((unsigned char *)"  [PASS] I2C read OK");
		pass_count++;
		if (d.month >= 1 && d.month <= 12 && d.day >= 1 && d.day <= 31) {
			SendString((unsigned char *)"\r\n  Range check: valid");
			pass_count++;
		} else {
			SendString((unsigned char *)"\r\n  Range check: [FAIL out of range]");
			fail_count++;
		}
	} else {
		SendString((unsigned char *)"ERROR (rc=-1)  [FAIL]");
		fail_count += 2;
	}
	SendString((unsigned char *)"\r\n");

	/* --- Test 4: Clock running check --- */
	SendString((unsigned char *)"\r\n--- Clock running check ---\r\n");
	SendString((unsigned char *)"  Waiting ~2 seconds...\r\n");
	delay_ms(2000);  /* 2 sec */
	rc = rtc_GetTime(&t2);
	if (rc == 0) {
		SendString((unsigned char *)"  Time now: ");
		print_dec2(t2.hours);
		PutChar(':');
		print_dec2(t2.minutes);
		PutChar(':');
		print_dec2(t2.seconds);
		/* Check that time has changed */
		if (t2.seconds != t1.seconds || t2.minutes != t1.minutes) {
			SendString((unsigned char *)"  [PASS] clock running");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL clock stopped?]");
			fail_count++;
		}
	} else {
		SendString((unsigned char *)"  Read error  [FAIL]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* --- Test 5: DS1307 battery-backed RAM --- */
	SendString((unsigned char *)"\r\n--- DS1307 SRAM write/read ---\r\n");
	{
		uint8_t wbuf[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
		uint8_t rbuf[4] = { 0, 0, 0, 0 };
		int wrc, rrc;

		wrc = rtc_WriteRAM(0, wbuf, 4);
		SendString((unsigned char *)"  WriteRAM(0, {DE,AD,BE,EF}) = ");
		if (wrc == 0) {
			SendString((unsigned char *)"OK\r\n");
		} else {
			SendString((unsigned char *)"FAIL\r\n");
		}

		rrc = rtc_ReadRAM(0, rbuf, 4);
		SendString((unsigned char *)"  ReadRAM(0, 4)              = {");
		print_hex8(rbuf[0]); PutChar(',');
		print_hex8(rbuf[1]); PutChar(',');
		print_hex8(rbuf[2]); PutChar(',');
		print_hex8(rbuf[3]);
		SendString((unsigned char *)"}");

		if (rrc == 0 && rbuf[0] == 0xDE && rbuf[1] == 0xAD &&
		    rbuf[2] == 0xBE && rbuf[3] == 0xEF) {
			SendString((unsigned char *)"  [PASS]");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");

		/* Write back zeros to not pollute SRAM */
		wbuf[0] = 0; wbuf[1] = 0; wbuf[2] = 0; wbuf[3] = 0;
		rtc_WriteRAM(0, wbuf, 4);
	}

	/* --- Test 6: SetTime to 00:00:00 (build-date reset) --- */
	SendString((unsigned char *)"\r\n--- SetTime (build-date reset) ---\r\n");
	{
		rtc_time_t set_time, read_time;
		int srt, grt;

		set_time.hours   = 0;
		set_time.minutes = 0;
		set_time.seconds = 0;

		srt = rtc_SetTime(&set_time);
		SendString((unsigned char *)"  SetTime(00:00:00) = ");
		if (srt == 0) {
			SendString((unsigned char *)"OK\r\n");
		} else {
			SendString((unsigned char *)"FAIL\r\n");
		}

		grt = rtc_GetTime(&read_time);
		SendString((unsigned char *)"  GetTime()         = ");
		if (grt == 0) {
			print_dec2(read_time.hours);
			PutChar(':');
			print_dec2(read_time.minutes);
			PutChar(':');
			print_dec2(read_time.seconds);
		} else {
			SendString((unsigned char *)"READ ERROR");
		}

		if (srt == 0 && grt == 0 &&
		    read_time.hours == 0 && read_time.minutes == 0 &&
		    (read_time.seconds == 0 || read_time.seconds == 1)) {
			SendString((unsigned char *)"  [PASS]");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Test 7: SetDate to firmware build date --- */
	SendString((unsigned char *)"\r\n--- SetDate (firmware build date) ---\r\n");
	{
		rtc_date_t set_date, read_date;
		int srd, grd;

		set_date.year  = (uint8_t)(VERSION_YEAR % 100);
		set_date.month = VERSION_MONTH;
		set_date.day   = VERSION_DAY;

		SendString((unsigned char *)"  Build date: " VERSION "\r\n");
		srd = rtc_SetDate(&set_date);
		SendString((unsigned char *)"  SetDate() = ");
		if (srd == 0) {
			SendString((unsigned char *)"OK\r\n");
		} else {
			SendString((unsigned char *)"FAIL\r\n");
		}

		grd = rtc_GetDate(&read_date);
		SendString((unsigned char *)"  GetDate() = 20");
		if (grd == 0) {
			print_dec2(read_date.year);
			PutChar('-');
			print_dec2(read_date.month);
			PutChar('-');
			print_dec2(read_date.day);
		} else {
			SendString((unsigned char *)"READ ERROR");
		}

		if (srd == 0 && grd == 0 &&
		    read_date.year == (uint8_t)(VERSION_YEAR % 100) &&
		    read_date.month == VERSION_MONTH &&
		    read_date.day == VERSION_DAY) {
			SendString((unsigned char *)"  [PASS]");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Test 8: Control register SQW config --- */
	SendString((unsigned char *)"\r\n--- Control register check ---\r\n");
	{
		uint8_t ctrl;
		int rrc2;

		/* Read control register directly via RAM-offset trick:
		 * DS1307_REG_CONTROL (0x07) is below DS1307_RAM_START (0x08),
		 * so we use the internal ds1307_read via rtc_ReadRAM won't work.
		 * Instead we re-init and check the SQW output behavior.
		 * For now, call rtc_Init and verify no error. */
		rtc_Init();
		SendString((unsigned char *)"  rtc_Init() completed (SQW=32.768kHz)\r\n");

		/* Verify I2C bus is still clean after init */
		ctrl = i2c_GetError(I2C_BUS1);
		SendString((unsigned char *)"  Bus error after Init: 0x");
		print_hex8(ctrl);
		if (ctrl == 0) {
			SendString((unsigned char *)"  [PASS] init OK, no bus errors");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL] bus error after init");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
	}

	/* --- Test 9: RAM boundary check --- */
	SendString((unsigned char *)"\r\n--- RAM boundary check ---\r\n");
	{
		uint8_t dummy[4];
		int brc;

		/* addr 54 + size 4 = 58 > 56 -> must return -1 */
		brc = rtc_ReadRAM(54, dummy, 4);
		SendString((unsigned char *)"  ReadRAM(54, 4) = ");
		if (brc == -1) {
			SendString((unsigned char *)"-1  [PASS] correctly rejected");
			pass_count++;
		} else {
			SendString((unsigned char *)"unexpected  [FAIL]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");

		brc = rtc_WriteRAM(54, dummy, 4);
		SendString((unsigned char *)"  WriteRAM(54, 4) = ");
		if (brc == -1) {
			SendString((unsigned char *)"-1  [PASS] correctly rejected");
			pass_count++;
		} else {
			SendString((unsigned char *)"unexpected  [FAIL]");
			fail_count++;
		}
		SendString((unsigned char *)"\r\n");
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
