/* test_eeprom.c — 24C16 EEPROM driver diagnostic
 *
 * Tests the EEPROM driver (eeprom.c) on I2C Bus 1 (shared with RTC).
 *
 * Test sections:
 *   1. EEPROM detection — ACK poll at base address 0xA0
 *   2. Single byte write/read at test area (0x700)
 *   3. Multi-byte write/read within one page
 *   4. Write/read across 16-byte page boundary
 *   5. Write/read across 256-byte block boundary (0x0FF..0x100)
 *   6. Config checksum computation (offline, no I2C needed)
 *   7. Config structure size verification
 *   8. Read existing config header from primary copy (0x000)
 *   9. Read existing config header from backup copy (0x400)
 *  10. Full config validate on primary and backup
 *
 * SAFETY: Write tests use addresses 0x700-0x72F which are well
 * outside both config regions (primary 0x000..0x0B3, backup 0x400..0x4B3).
 * Original data at test addresses is saved and restored after each test.
 *
 * Requires: I2C Bus 1 initialized (i2c_bus1_init from test_i2c.c)
 */
#include "test_eeprom.h"
#include "../eeprom/eeprom.h"
#include "../eeprom/config.h"
#include "../i2c/i2c.h"
#include "../sci.h"
#include "../utils/utils.h"

/* ---- Test area in EEPROM (safe zone, away from config) ---- */
#define TEST_ADDR_BASE      0x700   /* Start of test area */
#define TEST_ADDR_PAGE_CROSS 0x70C  /* 4 bytes before 16-byte boundary (0x710) */
#define TEST_ADDR_BLOCK_CROSS 0x0FC /* 4 bytes before 256-byte boundary */

/* ---- Helpers ---- */

static uint16_t pass_count;
static uint16_t fail_count;

static void report_pass(const char *msg)
{
    SendString((unsigned char *)"  [PASS] ");
    SendString((unsigned char *)msg);
    SendString((unsigned char *)"\r\n");
    pass_count++;
}

static void report_fail(const char *msg)
{
    SendString((unsigned char *)"  [FAIL] ");
    SendString((unsigned char *)msg);
    SendString((unsigned char *)"\r\n");
    fail_count++;
}

/* Print "0xHH " for a byte value */
static void print_byte(uint8_t val)
{
    SendString((unsigned char *)"0x");
    print_hex8(val);
}

/* Hex dump a buffer: "  HH HH HH ..." */
static void dump_buf(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    SendString((unsigned char *)"  ");
    for (i = 0; i < len; i++) {
        print_hex8(buf[i]);
        PutChar(' ');
    }
    SendString((unsigned char *)"\r\n");
}

/* Compare two buffers, return 1 if equal */
static int buf_equal(const uint8_t *a, const uint8_t *b, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        if (a[i] != b[i]) return 0;
    }
    return 1;
}

/* ================================================================
 * EEPROM_Test — Main test entry point
 * ================================================================ */
void EEPROM_Test(void)
{
    pass_count = 0;
    fail_count = 0;

    SendString((unsigned char *)"\r\n========================================\r\n");
    SendString((unsigned char *)"  EEPROM (24C16) Driver Diagnostic\r\n");
    SendString((unsigned char *)"========================================\r\n");

    /* ---- Section 1: EEPROM detection ---- */
    SendString((unsigned char *)"\r\n--- 1. EEPROM detection (ACK poll) ---\r\n");
    {
        int result;

        /* Poll at address 0x000 (device 0xA0) */
        result = eeprom_Poll(0x000);
        SendString((unsigned char *)"  Poll addr 0x000 (dev 0xA0): ");
        if (result == 0) {
            report_pass("EEPROM ACK received");
        } else {
            report_fail("EEPROM did not respond (NACK/timeout)");
            SendString((unsigned char *)"  *** EEPROM not detected — aborting remaining tests ***\r\n");
            goto test_summary;
        }

        /* Also poll at address 0x700 (device 0xAE — block 7) */
        result = eeprom_Poll(0x700);
        SendString((unsigned char *)"  Poll addr 0x700 (dev 0xAE): ");
        if (result == 0) {
            report_pass("Block 7 ACK received");
        } else {
            report_fail("Block 7 NACK — EEPROM may be < 2KB");
        }
    }

    /* ---- Section 2: Single byte write/read ---- */
    SendString((unsigned char *)"\r\n--- 2. Single byte write/read ---\r\n");
    {
        uint8_t save_byte;
        uint8_t write_val = 0xA5;
        uint8_t read_val = 0x00;

        /* Save original value */
        if (!eeprom_Read(TEST_ADDR_BASE, &save_byte, 1)) {
            report_fail("Could not read original byte at 0x700");
            goto sect3;
        }
        SendString((unsigned char *)"  Saved original at 0x700: ");
        print_byte(save_byte);
        SendString((unsigned char *)"\r\n");

        /* Write test pattern */
        if (!eeprom_Write(TEST_ADDR_BASE, &write_val, 1)) {
            report_fail("Write 0xA5 to 0x700 failed");
            goto sect3;
        }
        delay_ms(10);   /* Extra settling time */

        /* Read back */
        if (!eeprom_Read(TEST_ADDR_BASE, &read_val, 1)) {
            report_fail("Read back from 0x700 failed");
            goto restore2;
        }

        SendString((unsigned char *)"  Wrote: ");
        print_byte(write_val);
        SendString((unsigned char *)"  Read: ");
        print_byte(read_val);
        SendString((unsigned char *)"\r\n");

        if (read_val == write_val) {
            report_pass("Single byte write/read verified");
        } else {
            report_fail("Data mismatch on single byte");
        }

        /* Write complement to verify we're not reading stale data */
        write_val = 0x5A;
        eeprom_Write(TEST_ADDR_BASE, &write_val, 1);
        delay_ms(10);
        eeprom_Read(TEST_ADDR_BASE, &read_val, 1);
        SendString((unsigned char *)"  Complement: wrote ");
        print_byte(write_val);
        SendString((unsigned char *)" read ");
        print_byte(read_val);
        SendString((unsigned char *)"\r\n");
        if (read_val == 0x5A) {
            report_pass("Complement verify OK");
        } else {
            report_fail("Complement mismatch — stuck bit?");
        }

restore2:
        /* Restore original value */
        eeprom_Write(TEST_ADDR_BASE, &save_byte, 1);
        delay_ms(10);
    }

sect3:
    /* ---- Section 3: Multi-byte write/read within one page ---- */
    SendString((unsigned char *)"\r\n--- 3. Multi-byte write/read (same page) ---\r\n");
    {
        uint8_t save_buf[8];
        uint8_t write_buf[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
        uint8_t read_buf[8];
        uint16_t addr = TEST_ADDR_BASE;   /* 0x700, within page 0x700-0x70F */

        /* Save original */
        if (!eeprom_Read(addr, save_buf, 8)) {
            report_fail("Could not save 8 bytes at 0x700");
            goto sect4;
        }

        /* Write 8-byte pattern */
        if (!eeprom_Write(addr, write_buf, 8)) {
            report_fail("Write 8 bytes to 0x700 failed");
            goto restore3;
        }
        delay_ms(10);

        /* Read back */
        if (!eeprom_Read(addr, read_buf, 8)) {
            report_fail("Read back 8 bytes from 0x700 failed");
            goto restore3;
        }

        SendString((unsigned char *)"  Wrote:");
        dump_buf(write_buf, 8);
        SendString((unsigned char *)"  Read: ");
        dump_buf(read_buf, 8);

        if (buf_equal(write_buf, read_buf, 8)) {
            report_pass("8-byte write/read within page OK");
        } else {
            report_fail("8-byte data mismatch");
        }

restore3:
        eeprom_Write(addr, save_buf, 8);
        delay_ms(10);
    }

sect4:
    /* ---- Section 4: Page boundary crossing (16-byte page) ---- */
    SendString((unsigned char *)"\r\n--- 4. Write/read across 16-byte page boundary ---\r\n");
    {
        /* Write 8 bytes starting at 0x70C, which crosses the 0x710 page boundary:
         * bytes 0-3 go to page 0x700-0x70F, bytes 4-7 go to page 0x710-0x71F.
         * The driver must split this into two page writes. */
        uint8_t save_buf[8];
        uint8_t write_buf[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        uint8_t read_buf[8];
        uint16_t addr = TEST_ADDR_PAGE_CROSS;  /* 0x70C */

        SendString((unsigned char *)"  Addr 0x70C..0x713 (crosses page at 0x710)\r\n");

        /* Save */
        if (!eeprom_Read(addr, save_buf, 8)) {
            report_fail("Could not save original at 0x70C");
            goto sect5;
        }

        /* Write across boundary */
        if (!eeprom_Write(addr, write_buf, 8)) {
            report_fail("Page-crossing write failed");
            goto restore4;
        }
        delay_ms(10);

        /* Read back */
        if (!eeprom_Read(addr, read_buf, 8)) {
            report_fail("Page-crossing read back failed");
            goto restore4;
        }

        SendString((unsigned char *)"  Wrote:");
        dump_buf(write_buf, 8);
        SendString((unsigned char *)"  Read: ");
        dump_buf(read_buf, 8);

        if (buf_equal(write_buf, read_buf, 8)) {
            report_pass("Page boundary crossing write/read OK");
        } else {
            report_fail("Page boundary data mismatch — page split bug?");
        }

restore4:
        eeprom_Write(addr, save_buf, 8);
        delay_ms(10);
    }

sect5:
    /* ---- Section 5: Block boundary crossing (256-byte block) ---- */
    SendString((unsigned char *)"\r\n--- 5. Write/read across 256-byte block boundary ---\r\n");
    {
        /* Write 8 bytes at 0x0FC..0x103, crossing the 0x100 block boundary.
         * Device address changes from 0xA0 (block 0) to 0xA2 (block 1).
         * This tests the block-crossing logic in eeprom_Read/Write. */
        uint8_t save_buf[8];
        uint8_t write_buf[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01, 0x02};
        uint8_t read_buf[8];
        uint16_t addr = TEST_ADDR_BLOCK_CROSS;  /* 0x0FC */

        SendString((unsigned char *)"  Addr 0x0FC..0x103 (crosses block at 0x100)\r\n");
        SendString((unsigned char *)"  Block 0 (dev 0xA0) -> Block 1 (dev 0xA2)\r\n");

        /* Save */
        if (!eeprom_Read(addr, save_buf, 8)) {
            report_fail("Could not save original at 0x0FC");
            goto sect6;
        }

        /* Write across block boundary */
        if (!eeprom_Write(addr, write_buf, 8)) {
            report_fail("Block-crossing write failed");
            goto restore5;
        }
        delay_ms(10);

        /* Read back */
        if (!eeprom_Read(addr, read_buf, 8)) {
            report_fail("Block-crossing read back failed");
            goto restore5;
        }

        SendString((unsigned char *)"  Wrote:");
        dump_buf(write_buf, 8);
        SendString((unsigned char *)"  Read: ");
        dump_buf(read_buf, 8);

        if (buf_equal(write_buf, read_buf, 8)) {
            report_pass("Block boundary crossing write/read OK");
        } else {
            report_fail("Block boundary data mismatch — block split bug?");
        }

restore5:
        eeprom_Write(addr, save_buf, 8);
        delay_ms(10);
    }

sect6:
    /* ---- Section 6: Config checksum computation ---- */
    SendString((unsigned char *)"\r\n--- 6. Config checksum (offline) ---\r\n");
    {
        /* Test vector: checksum of header {0x01, 0x0B}
         * init=0, sum = 0 + 0x01 + 0x0B = 0x0C, ~0x0C = 0xF3 */
        uint8_t hdr[2] = {0x01, 0x0B};
        uint8_t chk;

        chk = eeprom_ConfigChecksum(0, hdr, 2);
        SendString((unsigned char *)"  Checksum({0x01, 0x0B}, init=0): ");
        print_byte(chk);
        SendString((unsigned char *)" (expect 0xF3)\r\n");
        if (chk == 0xF3) {
            report_pass("Header checksum correct");
        } else {
            report_fail("Header checksum wrong");
        }

        /* Test: all zeros, size 4 -> ~(0+0+0+0+0) = ~0 = 0xFF */
        {
            uint8_t zeros[4] = {0, 0, 0, 0};
            chk = eeprom_ConfigChecksum(0, zeros, 4);
            SendString((unsigned char *)"  Checksum(all zeros, 4): ");
            print_byte(chk);
            SendString((unsigned char *)" (expect 0xFF)\r\n");
            if (chk == 0xFF) {
                report_pass("Zero data checksum correct");
            } else {
                report_fail("Zero data checksum wrong");
            }
        }

        /* Test: all 0xFF, size 2, init=0 -> ~(0 + 0xFF + 0xFF) = ~0xFE = 0x01 */
        {
            uint8_t ffs[2] = {0xFF, 0xFF};
            chk = eeprom_ConfigChecksum(0, ffs, 2);
            SendString((unsigned char *)"  Checksum({FF,FF}, 2): ");
            print_byte(chk);
            SendString((unsigned char *)" (expect 0x01)\r\n");
            if (chk == 0x01) {
                report_pass("0xFF data checksum correct");
            } else {
                report_fail("0xFF data checksum wrong");
            }
        }

        /* Test: chaining — init=0x10, data={0x20} -> ~(0x10+0x20) = ~0x30 = 0xCF */
        {
            uint8_t dat[1] = {0x20};
            chk = eeprom_ConfigChecksum(0x10, dat, 1);
            SendString((unsigned char *)"  Checksum({0x20}, init=0x10): ");
            print_byte(chk);
            SendString((unsigned char *)" (expect 0xCF)\r\n");
            if (chk == 0xCF) {
                report_pass("Chained checksum correct");
            } else {
                report_fail("Chained checksum wrong");
            }
        }
    }

    /* ---- Section 7: Config structure size ---- */
    SendString((unsigned char *)"\r\n--- 7. Config structure size ---\r\n");
    {
        uint16_t sz = (uint16_t)sizeof(mower_config_t);
        SendString((unsigned char *)"  sizeof(mower_config_t) = 0x");
        print_hex16(sz);
        SendString((unsigned char *)" (expect 0x00B0 = 176)\r\n");
        if (sz == 0xB0) {
            report_pass("Config struct size correct (0xB0)");
        } else {
            report_fail("Config struct size wrong — packing error");
        }
    }

    /* ---- Section 8: Read primary config header ---- */
    SendString((unsigned char *)"\r\n--- 8. Primary config header (EEPROM 0x000) ---\r\n");
    {
        uint8_t hdr[3];
        uint8_t computed;

        if (!eeprom_Read(0x000, hdr, 3)) {
            report_fail("Could not read primary header");
            goto sect9;
        }

        SendString((unsigned char *)"  Header bytes: ");
        print_byte(hdr[0]);
        PutChar(' ');
        print_byte(hdr[1]);
        PutChar(' ');
        print_byte(hdr[2]);
        SendString((unsigned char *)"\r\n");

        SendString((unsigned char *)"  Version: ");
        print_byte(hdr[0]);
        SendString((unsigned char *)" (expect 0x02)\r\n");

        SendString((unsigned char *)"  Size:    ");
        print_byte(hdr[1]);
        SendString((unsigned char *)" (expect 0x09)\r\n");

        computed = eeprom_ConfigChecksum(0, hdr, 2);
        SendString((unsigned char *)"  Hdr chk: ");
        print_byte(hdr[2]);
        SendString((unsigned char *)" (computed: ");
        print_byte(computed);
        SendString((unsigned char *)")\r\n");

        if (computed == hdr[2] && hdr[0] == 0x02 && hdr[1] == 0x09) {
            report_pass("Primary header valid (v0x02, sz0x09, chk OK)");
        } else if (hdr[0] == 0xFF && hdr[1] == 0xFF && hdr[2] == 0xFF) {
            SendString((unsigned char *)"  (Blank/erased EEPROM — all 0xFF)\r\n");
            report_fail("Primary header blank — no config stored");
        } else {
            report_fail("Primary header invalid or corrupt");
        }
    }

sect9:
    /* ---- Section 9: Read backup config header ---- */
    SendString((unsigned char *)"\r\n--- 9. Backup config header (EEPROM 0x400) ---\r\n");
    {
        uint8_t hdr[3];
        uint8_t computed;

        if (!eeprom_Read(0x400, hdr, 3)) {
            report_fail("Could not read backup header");
            goto sect10;
        }

        SendString((unsigned char *)"  Header bytes: ");
        print_byte(hdr[0]);
        PutChar(' ');
        print_byte(hdr[1]);
        PutChar(' ');
        print_byte(hdr[2]);
        SendString((unsigned char *)"\r\n");

        computed = eeprom_ConfigChecksum(0, hdr, 2);
        SendString((unsigned char *)"  Hdr chk: ");
        print_byte(hdr[2]);
        SendString((unsigned char *)" (computed: ");
        print_byte(computed);
        SendString((unsigned char *)")\r\n");

        if (computed == hdr[2] && hdr[0] == 0x02 && hdr[1] == 0x09) {
            report_pass("Backup header valid (v0x02, sz0x09, chk OK)");
        } else if (hdr[0] == 0xFF && hdr[1] == 0xFF && hdr[2] == 0xFF) {
            SendString((unsigned char *)"  (Blank/erased EEPROM — all 0xFF)\r\n");
            report_fail("Backup header blank — no config stored");
        } else {
            report_fail("Backup header invalid or corrupt");
        }
    }

sect10:
    /* ---- Section 10: Full config validation ---- */
    SendString((unsigned char *)"\r\n--- 10. Full config validation ---\r\n");
    {
        uint8_t config_buf[0xB0];
        int result;
        const char *err_names[] = {
            "OK", "HDR_CHKSUM", "VERSION", "SIZE", "DATA_CHKSUM", "FACTORY"
        };

        /* Validate primary */
        result = eeprom_ConfigValidate(0x000, config_buf);
        SendString((unsigned char *)"  Primary (0x000): result=");
        if (result >= 0 && result <= 5) {
            SendString((unsigned char *)err_names[result]);
        } else {
            print_byte((uint8_t)result);
        }
        SendString((unsigned char *)"\r\n");

        if (result == EEPROM_CFG_OK) {
            report_pass("Primary config validates OK");
            /* Print a few known fields for sanity check */
            SendString((unsigned char *)"    battery_type (+0x66): ");
            print_byte(config_buf[0x66]);
            SendString((unsigned char *)" ('");
            PutChar(config_buf[0x66]);
            SendString((unsigned char *)"')\r\n");
            SendString((unsigned char *)"    config_bits  (+0xAB): ");
            print_byte(config_buf[0xAB]);
            SendString((unsigned char *)"\r\n");
            SendString((unsigned char *)"    feature_flags(+0xAC): ");
            print_byte(config_buf[0xAC]);
            SendString((unsigned char *)"\r\n");
        } else {
            report_fail("Primary config validation failed");
        }

        /* Validate backup */
        result = eeprom_ConfigValidate(0x400, config_buf);
        SendString((unsigned char *)"  Backup  (0x400): result=");
        if (result >= 0 && result <= 5) {
            SendString((unsigned char *)err_names[result]);
        } else {
            print_byte((uint8_t)result);
        }
        SendString((unsigned char *)"\r\n");

        if (result == EEPROM_CFG_OK) {
            report_pass("Backup config validates OK");
        } else {
            report_fail("Backup config validation failed");
        }
    }

    /* ---- Summary ---- */
test_summary:
    SendString((unsigned char *)"\r\n========================================\r\n");
    SendString((unsigned char *)"  EEPROM Test Summary: ");
    print_hex8((uint8_t)pass_count);
    SendString((unsigned char *)" passed, ");
    print_hex8((uint8_t)fail_count);
    SendString((unsigned char *)" failed\r\n");
    SendString((unsigned char *)"========================================\r\n");
}
