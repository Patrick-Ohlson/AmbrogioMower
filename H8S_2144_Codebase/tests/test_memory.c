/* test_memory.c — Memory variable test (.data, .bss, .rodata)
 *
 * Verifies that the startup code in start.S correctly initializes:
 *   1. .data section — ROM-to-RAM copy (initialized globals)
 *   2. .bss section  — zero fill (uninitialized globals)
 *   3. .rodata       — const data readable from ROM
 *   4. RAM write/read — modify a variable and read it back
 *
 * Memory test variables are defined in main.c (extern here).
 */
#include "test_memory.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../utils/utils.h"

/* Memory test variables (defined in main.c) */
extern uint8_t  mem_test_u8;
extern uint16_t mem_test_u16;
extern uint32_t mem_test_u32;
extern char     mem_test_str[];
extern uint8_t  bss_test_u8;
extern uint16_t bss_test_u16;
extern uint32_t bss_test_u32;
extern char     bss_test_buf[];

/* .rodata (const) — defined here since they're only used by this test */
static const char rodata_test[] = "ROM_OK";
static const uint32_t rodata_magic = 0x12345678;


/* ================================================================
 * Print helpers for pass/fail result lines
 * ================================================================ */
void print_test_u8(const char *label, volatile uint8_t *var, uint8_t expect)
{
	SendString((unsigned char *)"  ");
	SendString((unsigned char *)label);
	SendString((unsigned char *)" @ ");
	print_addr(var);
	SendString((unsigned char *)" = 0x");
	print_hex8(*var);
	if (*var == expect)
		SendString((unsigned char *)"  [PASS]");
	else {
		SendString((unsigned char *)"  [FAIL expected 0x");
		print_hex8(expect);
		SendString((unsigned char *)"]");
	}
	SendString((unsigned char *)"\r\n");
}

void print_test_u16(const char *label, volatile uint16_t *var, uint16_t expect)
{
	SendString((unsigned char *)"  ");
	SendString((unsigned char *)label);
	SendString((unsigned char *)" @ ");
	print_addr(var);
	SendString((unsigned char *)" = 0x");
	print_hex16(*var);
	if (*var == expect)
		SendString((unsigned char *)"  [PASS]");
	else {
		SendString((unsigned char *)"  [FAIL expected 0x");
		print_hex16(expect);
		SendString((unsigned char *)"]");
	}
	SendString((unsigned char *)"\r\n");
}

void print_test_u32(const char *label, volatile uint32_t *var, uint32_t expect)
{
	SendString((unsigned char *)"  ");
	SendString((unsigned char *)label);
	SendString((unsigned char *)" @ ");
	print_addr(var);
	SendString((unsigned char *)" = 0x");
	print_hex32(*var);
	if (*var == expect)
		SendString((unsigned char *)"  [PASS]");
	else {
		SendString((unsigned char *)"  [FAIL expected 0x");
		print_hex32(expect);
		SendString((unsigned char *)"]");
	}
	SendString((unsigned char *)"\r\n");
}


/* ================================================================
 * Memory_Test()
 * ================================================================ */
void Memory_Test(void)
{
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;
	uint8_t i;

	SendString((unsigned char *)"\r\n");
	SendString((unsigned char *)"========================================\r\n");
	SendString((unsigned char *)"  Memory Variable Test\r\n");
	SendString((unsigned char *)"========================================\r\n");

	/* --- Section 1: .data (initialized globals, ROM->RAM copy) --- */
	SendString((unsigned char *)"\r\n--- .data section (ROM->RAM copy) ---\r\n");

	print_test_u8 ("mem_test_u8 ", &mem_test_u8,  0xA5);
	if (mem_test_u8 == 0xA5) pass_count++; else fail_count++;

	print_test_u16("mem_test_u16", &mem_test_u16, 0xBEEF);
	if (mem_test_u16 == 0xBEEF) pass_count++; else fail_count++;

	print_test_u32("mem_test_u32", &mem_test_u32, 0xDEADCAFE);
	if (mem_test_u32 == 0xDEADCAFE) pass_count++; else fail_count++;

	SendString((unsigned char *)"  mem_test_str @ ");
	print_addr(mem_test_str);
	SendString((unsigned char *)" = \"");
	SendString((unsigned char *)mem_test_str);
	SendString((unsigned char *)"\"");
	if (mem_test_str[0] == 'H' && mem_test_str[3] == '/' &&
	    mem_test_str[7] == '4' && mem_test_str[8] == '\0') {
		SendString((unsigned char *)"  [PASS]");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expected \"H8S/2144\"]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* Modify mem_test_str to prove it's in RAM, not ROM */
	mem_test_str[0] = 'R';
	mem_test_str[1] = 'A';
	mem_test_str[2] = 'M';
	mem_test_str[3] = '_';
	mem_test_str[4] = 'O';
	mem_test_str[5] = 'K';
	mem_test_str[6] = '!';
	mem_test_str[7] = '\0';
	SendString((unsigned char *)"  str modify  @ ");
	print_addr(mem_test_str);
	SendString((unsigned char *)" = \"");
	SendString((unsigned char *)mem_test_str);
	SendString((unsigned char *)"\"");
	if (mem_test_str[0] == 'R' && mem_test_str[2] == 'M' &&
	    mem_test_str[4] == 'O' && mem_test_str[6] == '!') {
		SendString((unsigned char *)"  [PASS] writable RAM");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL stuck in ROM?]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	/* --- Section 2: .bss (uninitialized globals, zero fill) --- */
	SendString((unsigned char *)"\r\n--- .bss section (zero fill) ---\r\n");

	print_test_u8 ("bss_test_u8 ", &bss_test_u8,  0x00);
	if (bss_test_u8 == 0x00) pass_count++; else fail_count++;

	print_test_u16("bss_test_u16", &bss_test_u16, 0x0000);
	if (bss_test_u16 == 0x0000) pass_count++; else fail_count++;

	print_test_u32("bss_test_u32", &bss_test_u32, 0x00000000);
	if (bss_test_u32 == 0x00000000) pass_count++; else fail_count++;

	/* Check all bytes of bss buffer are zero */
	SendString((unsigned char *)"  bss_test_buf[8] @ ");
	print_addr(bss_test_buf);
	SendString((unsigned char *)" = {");
	{
		uint8_t all_zero = 1;
		for (i = 0; i < 8; i++) {
			if (i > 0) PutChar(',');
			print_hex8(bss_test_buf[i]);
			if (bss_test_buf[i] != 0) all_zero = 0;
		}
		SendString((unsigned char *)"}");
		if (all_zero) {
			SendString((unsigned char *)"  [PASS]");
			pass_count++;
		} else {
			SendString((unsigned char *)"  [FAIL expected all 00]");
			fail_count++;
		}
	}
	SendString((unsigned char *)"\r\n");

	/* --- Section 3: .rodata (const data in ROM) --- */
	SendString((unsigned char *)"\r\n--- .rodata section (const in ROM) ---\r\n");

	SendString((unsigned char *)"  rodata_test @ ");
	print_addr(rodata_test);
	SendString((unsigned char *)" = \"");
	SendString((unsigned char *)rodata_test);
	SendString((unsigned char *)"\"");
	if (rodata_test[0] == 'R' && rodata_test[3] == '_' &&
	    rodata_test[5] == 'K' && rodata_test[6] == '\0') {
		SendString((unsigned char *)"  [PASS]");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL expected \"ROM_OK\"]");
		fail_count++;
	}
	SendString((unsigned char *)"\r\n");

	print_test_u32("rodata_magic", (volatile uint32_t *)&rodata_magic, 0x12345678);
	if (rodata_magic == 0x12345678) pass_count++; else fail_count++;

	/* --- Section 4: RAM write/read test --- */
	SendString((unsigned char *)"\r\n--- RAM write/read test ---\r\n");

	/* Write a known pattern to a .data variable, read it back */
	mem_test_u32 = 0x01020304;
	print_test_u32("write 0x01020304", &mem_test_u32, 0x01020304);
	if (mem_test_u32 == 0x01020304) pass_count++; else fail_count++;

	/* Write complement pattern */
	mem_test_u32 = 0xFEFDFCFB;
	print_test_u32("write 0xFEFDFCFB", &mem_test_u32, 0xFEFDFCFB);
	if (mem_test_u32 == 0xFEFDFCFB) pass_count++; else fail_count++;

	/* Write to .bss variable */
	bss_test_u16 = 0x55AA;
	print_test_u16("bss write 0x55AA", &bss_test_u16, 0x55AA);
	if (bss_test_u16 == 0x55AA) pass_count++; else fail_count++;

	/* --- Restore all test variables to original values ---
	 * Without reset, .bss and .data tests fail on re-run
	 * because start.S only initializes them once at boot. */
	mem_test_u8  = 0xA5;
	mem_test_u16 = 0xBEEF;
	mem_test_u32 = 0xDEADCAFE;
	mem_test_str[0] = 'H'; mem_test_str[1] = '8';
	mem_test_str[2] = 'S'; mem_test_str[3] = '/';
	mem_test_str[4] = '2'; mem_test_str[5] = '1';
	mem_test_str[6] = '4'; mem_test_str[7] = '4';
	mem_test_str[8] = '\0';
	bss_test_u8  = 0x00;
	bss_test_u16 = 0x0000;
	bss_test_u32 = 0x00000000;
	for (i = 0; i < 8; i++) bss_test_buf[i] = 0;
	SendString((unsigned char *)"\r\n  (test variables restored for re-run)\r\n");

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

	/* Show linker symbols for reference */
	{
		extern char data[], edata[], bss[], ebss[], mdata[];
		SendString((unsigned char *)"\r\n  Linker symbols:\r\n");
		SendString((unsigned char *)"    _data  = "); print_addr(data);  SendString((unsigned char *)"\r\n");
		SendString((unsigned char *)"    _edata = "); print_addr(edata); SendString((unsigned char *)"\r\n");
		SendString((unsigned char *)"    _mdata = "); print_addr(mdata); SendString((unsigned char *)"\r\n");
		SendString((unsigned char *)"    _bss   = "); print_addr(bss);   SendString((unsigned char *)"\r\n");
		SendString((unsigned char *)"    _ebss  = "); print_addr(ebss);  SendString((unsigned char *)"\r\n");
		SendString((unsigned char *)"    .data size = 0x");
		print_hex16((uint16_t)(edata - data));
		SendString((unsigned char *)" bytes\r\n");
		SendString((unsigned char *)"    .bss  size = 0x");
		print_hex16((uint16_t)(ebss - bss));
		SendString((unsigned char *)" bytes\r\n");
	}
}
