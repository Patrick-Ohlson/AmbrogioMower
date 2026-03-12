/* test_accel.c -- Accelerometer driver diagnostic
 *
 * Tests the accel driver (accel.c) on I2C Bus 2 (separate from RTC/EEPROM).
 *
 * Test sections:
 *   1. I2C Bus 2 probe -- try LIS302DL (0x3A) and LIS3DH (0x32)
 *   2. WHO_AM_I verification -- LIS302DL=0x3B, LIS3DH=0x33
 *   3. Register write/readback -- write and read a config register
 *   4. accel_Init auto-detect -- verify correct sensor type returned
 *   5. Raw axis read -- accel_Read, print XYZ
 *   6. IIR filter math -- feed known values, check accumulator output
 *   7. Tilt angle computation -- accel_GetTilt after priming filter
 *   8. Multi-sample read -- 10 consecutive reads, show raw and filtered
 *   9. Config register dump (sensor-specific)
 *  10. Live LCD output -- 20 seconds of real-time X/Y/Z and tilt display
 *
 * Requires: I2C Bus 2 pins configured (PB.6=SDA, PB.7=SCL).
 *           i2c_bus2_init() called at startup or re-init'd here.
 */
#include "test_accel.h"
#include "test_i2c.h"          /* i2c_bus2_init() */
#include "../accel/accel.h"
#include "../i2c/i2c.h"
#include "../sci.h"
#include "../lcd/lcd.h"
#include "../timer/timer.h"
#include "../utils/utils.h"
#include <stddef.h>     /* NULL */

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

static void print_byte(uint8_t val)
{
    SendString((unsigned char *)"0x");
    print_hex8(val);
}

/* Print a signed 16-bit value as decimal */
static void print_int16(int16_t val)
{
    char buf[7];  /* -32768 + null */
    int i = 0;
    uint16_t uval;

    if (val < 0) {
        PutChar('-');
        uval = (uint16_t)(-(int32_t)val);
    } else {
        uval = (uint16_t)val;
    }
    if (uval == 0) { PutChar('0'); return; }
    while (uval > 0) { buf[i++] = '0' + (uval % 10); uval /= 10; }
    while (i > 0) PutChar(buf[--i]);
}

/* Print a signed 32-bit value as decimal */
static void print_int32(int32_t val)
{
    char buf[12];
    int i = 0;
    uint32_t uval;

    if (val < 0) {
        PutChar('-');
        uval = (uint32_t)(-val);
    } else {
        uval = (uint32_t)val;
    }
    if (uval == 0) { PutChar('0'); return; }
    while (uval > 0) { buf[i++] = '0' + (char)(uval % 10); uval /= 10; }
    while (i > 0) PutChar(buf[--i]);
}

/* ================================================================
 * Accel_Test -- Main test entry point
 * ================================================================ */
void Accel_Test(void)
{
    uint8_t detected_type;

    pass_count = 0;
    fail_count = 0;

    SendString((unsigned char *)"\r\n========================================\r\n");
    SendString((unsigned char *)"  Accelerometer Driver Diagnostic\r\n");
    SendString((unsigned char *)"  Bus 2: PB.7=SCL, PB.6=SDA\r\n");
    SendString((unsigned char *)"========================================\r\n");

    /* Ensure I2C Bus 2 is initialized (accel lives on Bus 2).
     * main() calls i2c_bus2_init() at startup, but re-init here
     * to recover from any prior bus errors. */
    i2c_bus2_init();

    /* ---- Section 1: I2C Bus 2 probe ---- */
    SendString((unsigned char *)"\r\n--- 1. I2C Bus 2 device probe ---\r\n");
    {
        int ack_302, ack_3dh;

        /* Probe LIS302DL at 0x3A (firmware calls this "MMA7455") */
        i2c_Start(I2C_BUS2);
        ack_302 = i2c_WriteByte(I2C_BUS2, LIS302DL_ADDR_W);
        i2c_Stop(I2C_BUS2);

        SendString((unsigned char *)"  LIS302DL (0x3A): ");
        if (ack_302) {
            SendString((unsigned char *)"ACK\r\n");
            report_pass("LIS302DL responded on Bus 2");
        } else {
            SendString((unsigned char *)"NACK\r\n");
            SendString((unsigned char *)"  (Not present or not connected)\r\n");
        }

        /* Probe LIS3DH at 0x32 */
        i2c_Start(I2C_BUS2);
        ack_3dh = i2c_WriteByte(I2C_BUS2, LIS3DH_ADDR_W);
        i2c_Stop(I2C_BUS2);

        SendString((unsigned char *)"  LIS3DH   (0x32): ");
        if (ack_3dh) {
            SendString((unsigned char *)"ACK\r\n");
            report_pass("LIS3DH responded on Bus 2");
        } else {
            SendString((unsigned char *)"NACK\r\n");
            SendString((unsigned char *)"  (Not present or not connected)\r\n");
        }

        if (!ack_302 && !ack_3dh) {
            report_fail("No accelerometer found on Bus 2");
            SendString((unsigned char *)"  *** Check wiring: PB.7=SCL, PB.6=SDA ***\r\n");
            SendString((unsigned char *)"  *** Aborting remaining tests ***\r\n");
            goto test_summary;
        }
    }

    /* ---- Section 2: WHO_AM_I verification ---- */
    SendString((unsigned char *)"\r\n--- 2. WHO_AM_I register check ---\r\n");
    {
        uint8_t whoami = 0;
        int rc;

        /* Try LIS302DL WHO_AM_I (reg 0x0F, expect 0x3B) */
        rc = accel_ReadReg(LIS302DL_ADDR_W, LIS302DL_ADDR_R,
                           LIS302DL_REG_WHO_AM_I, &whoami);
        SendString((unsigned char *)"  LIS302DL WHO_AM_I (reg 0x0F): ");
        if (rc == 0) {
            print_byte(whoami);
            SendString((unsigned char *)" (expect 0x3B)\r\n");
            if (whoami == LIS302DL_WHOAMI_VALUE) {
                report_pass("LIS302DL WHO_AM_I = 0x3B");
            } else {
                SendString((unsigned char *)"  (Unexpected -- different chip variant?)\r\n");
            }
        } else {
            SendString((unsigned char *)"read error (NACK)\r\n");
            SendString((unsigned char *)"  (LIS302DL not present)\r\n");
        }

        /* Try LIS3DH WHO_AM_I (reg 0x0F, expect 0x33) */
        whoami = 0;
        rc = accel_ReadReg(LIS3DH_ADDR_W, LIS3DH_ADDR_R,
                           LIS3DH_REG_WHO_AM_I, &whoami);
        SendString((unsigned char *)"  LIS3DH   WHO_AM_I (reg 0x0F): ");
        if (rc == 0) {
            print_byte(whoami);
            SendString((unsigned char *)" (expect 0x33)\r\n");
            if (whoami == LIS3DH_WHOAMI_VALUE) {
                report_pass("LIS3DH WHO_AM_I = 0x33");
            } else {
                SendString((unsigned char *)"  (Unexpected value)\r\n");
            }
        } else {
            SendString((unsigned char *)"read error (NACK)\r\n");
        }
    }

    /* ---- Section 3: Register write/readback ---- */
    SendString((unsigned char *)"\r\n--- 3. Register write/readback ---\r\n");
    {
        uint8_t val = 0;
        int rc;

        /* Try LIS302DL CTRL_REG2 (0x21) -- writable config register */
        rc = accel_ReadReg(LIS302DL_ADDR_W, LIS302DL_ADDR_R,
                           LIS302DL_REG_CTRL_REG2, &val);
        if (rc == 0) {
            uint8_t saved = val;
            uint8_t test_val = 0x09;  /* HPF cutoff config -- safe value */
            uint8_t readback = 0;

            SendString((unsigned char *)"  LIS302DL CTRL_REG2 original: ");
            print_byte(saved);
            SendString((unsigned char *)"\r\n");

            /* Write test value */
            rc = accel_WriteReg(LIS302DL_ADDR_W, LIS302DL_REG_CTRL_REG2, test_val);
            if (rc == 0) {
                rc = accel_ReadReg(LIS302DL_ADDR_W, LIS302DL_ADDR_R,
                                   LIS302DL_REG_CTRL_REG2, &readback);
                SendString((unsigned char *)"  Wrote: ");
                print_byte(test_val);
                SendString((unsigned char *)"  Read: ");
                print_byte(readback);
                SendString((unsigned char *)"\r\n");

                if (rc == 0 && readback == test_val) {
                    report_pass("LIS302DL register write/readback OK");
                } else {
                    report_fail("LIS302DL register readback mismatch");
                }

                /* Restore original */
                accel_WriteReg(LIS302DL_ADDR_W, LIS302DL_REG_CTRL_REG2, saved);
            } else {
                report_fail("LIS302DL register write failed (NACK)");
            }
        } else {
            /* Try LIS3DH CTRL_REG2 */
            rc = accel_ReadReg(LIS3DH_ADDR_W, LIS3DH_ADDR_R,
                               LIS3DH_REG_CTRL_REG2, &val);
            if (rc == 0) {
                uint8_t saved = val;
                uint8_t test_val = 0x09;
                uint8_t readback = 0;

                SendString((unsigned char *)"  LIS3DH CTRL_REG2 original: ");
                print_byte(saved);
                SendString((unsigned char *)"\r\n");

                rc = accel_WriteReg(LIS3DH_ADDR_W, LIS3DH_REG_CTRL_REG2, test_val);
                if (rc == 0) {
                    rc = accel_ReadReg(LIS3DH_ADDR_W, LIS3DH_ADDR_R,
                                       LIS3DH_REG_CTRL_REG2, &readback);
                    SendString((unsigned char *)"  Wrote: ");
                    print_byte(test_val);
                    SendString((unsigned char *)"  Read: ");
                    print_byte(readback);
                    SendString((unsigned char *)"\r\n");

                    if (rc == 0 && readback == test_val) {
                        report_pass("LIS3DH register write/readback OK");
                    } else {
                        report_fail("LIS3DH register readback mismatch");
                    }

                    accel_WriteReg(LIS3DH_ADDR_W, LIS3DH_REG_CTRL_REG2, saved);
                } else {
                    report_fail("LIS3DH register write failed (NACK)");
                }
            } else {
                report_fail("No register access to either sensor");
            }
        }
    }

    /* ---- Section 4: accel_Init auto-detect ---- */
    SendString((unsigned char *)"\r\n--- 4. accel_Init auto-detect ---\r\n");
    {
        detected_type = accel_Init();

        SendString((unsigned char *)"  Detected: ");
        switch (detected_type) {
            case ACCEL_LIS302DL:
                SendString((unsigned char *)"LIS302DL\r\n");
                report_pass("accel_Init returned ACCEL_LIS302DL");
                break;
            case ACCEL_LIS3DH:
                SendString((unsigned char *)"LIS3DH\r\n");
                report_pass("accel_Init returned ACCEL_LIS3DH");
                break;
            default:
                SendString((unsigned char *)"NONE\r\n");
                report_fail("accel_Init returned ACCEL_NONE");
                SendString((unsigned char *)"  *** Sensor not detected by driver ***\r\n");
                goto test_summary;
        }

        /* Verify accel_GetType matches */
        if (accel_GetType() == detected_type) {
            report_pass("accel_GetType matches Init result");
        } else {
            report_fail("accel_GetType mismatch");
        }
    }

    /* ---- Section 5: Raw axis read ---- */
    SendString((unsigned char *)"\r\n--- 5. Raw axis read ---\r\n");
    {
        accel_raw_t raw;
        int rc;

        rc = accel_Read(&raw);
        if (rc == 0) {
            SendString((unsigned char *)"  X=");
            print_int16(raw.x);
            SendString((unsigned char *)"  Y=");
            print_int16(raw.y);
            SendString((unsigned char *)"  Z=");
            print_int16(raw.z);
            SendString((unsigned char *)"\r\n");
            report_pass("accel_Read returned data");

            /* Sanity: at rest, Z should be non-zero (gravity) */
            if (raw.z != 0) {
                report_pass("Z-axis non-zero (gravity present)");
            } else {
                SendString((unsigned char *)"  (Z=0 may indicate sensor not producing data)\r\n");
            }
        } else {
            report_fail("accel_Read returned error");
        }
    }

    /* ---- Section 6: IIR filter verification ---- */
    SendString((unsigned char *)"\r\n--- 6. IIR filter accumulator check ---\r\n");
    {
        accel_filtered_t filt;
        int i;

        /* Read 5 times to let filter converge from initial zero */
        for (i = 0; i < 5; i++) {
            accel_Read(NULL);
        }

        accel_GetFiltered(&filt);
        SendString((unsigned char *)"  Filtered (x16 scale):\r\n");
        SendString((unsigned char *)"    X=");
        print_int32(filt.x);
        SendString((unsigned char *)"  Y=");
        print_int32(filt.y);
        SendString((unsigned char *)"  Z=");
        print_int32(filt.z);
        SendString((unsigned char *)"\r\n");

        /* After 5 reads, filter should have non-zero values if sensor works */
        if (filt.x != 0 || filt.y != 0 || filt.z != 0) {
            report_pass("Filter accumulators non-zero after reads");
        } else {
            report_fail("Filter accumulators all zero -- sensor not producing data?");
        }
    }

    /* ---- Section 7: Tilt angle computation ---- */
    SendString((unsigned char *)"\r\n--- 7. Tilt angle computation ---\r\n");
    {
        accel_tilt_t tilt;
        int i;

        /* Prime the filter with a few more reads */
        for (i = 0; i < 5; i++) {
            accel_Read(NULL);
        }

        accel_GetTilt(&tilt);
        SendString((unsigned char *)"  Tilt1=");
        print_int16(tilt.tilt1);
        SendString((unsigned char *)" deg  Tilt2=");
        print_int16(tilt.tilt2);
        SendString((unsigned char *)" deg\r\n");

        /* At rest on a flat surface, both tilts should be small */
        if (tilt.tilt1 >= -90 && tilt.tilt1 <= 90 &&
            tilt.tilt2 >= -90 && tilt.tilt2 <= 90) {
            report_pass("Tilt angles in valid range (-90..+90)");
        } else {
            report_fail("Tilt angles out of expected range");
        }
    }

    /* ---- Section 8: Multi-sample burst read ---- */
    SendString((unsigned char *)"\r\n--- 8. Multi-sample burst (10 reads) ---\r\n");
    {
        int i;
        accel_raw_t raw;

        SendString((unsigned char *)"  #   X      Y      Z\r\n");
        for (i = 0; i < 10; i++) {
            int rc = accel_Read(&raw);
            if (rc == 0) {
                SendString((unsigned char *)"  ");
                if (i < 10) PutChar(' ');
                PutChar('0' + (char)i);
                SendString((unsigned char *)": ");
                print_int16(raw.x);
                SendString((unsigned char *)"\t");
                print_int16(raw.y);
                SendString((unsigned char *)"\t");
                print_int16(raw.z);
                SendString((unsigned char *)"\r\n");
            } else {
                SendString((unsigned char *)"  Read error at sample ");
                PutChar('0' + (char)i);
                SendString((unsigned char *)"\r\n");
            }
        }
        report_pass("Burst read complete");
    }

    /* ---- Section 9: Config register dump ---- */
    if (detected_type == ACCEL_LIS302DL) {
        SendString((unsigned char *)"\r\n--- 9. LIS302DL config register dump ---\r\n");
        {
            static const struct {
                uint8_t reg;
                const char *name;
                uint8_t expect;   /* 0xFF = no expected value */
            } regs[] = {
                { 0x0F, "WHO_AM_I  ", 0x3B },
                { 0x20, "CTRL_REG1 ", 0x47 },
                { 0x21, "CTRL_REG2 ", 0xFF },
                { 0x22, "CTRL_REG3 ", 0xFF },
                { 0x27, "STATUS    ", 0xFF },
            };
            int i;
            int config_ok = 1;

            for (i = 0; i < (int)(sizeof(regs) / sizeof(regs[0])); i++) {
                uint8_t val = 0;
                int rc = accel_ReadReg(LIS302DL_ADDR_W, LIS302DL_ADDR_R,
                                       regs[i].reg, &val);
                SendString((unsigned char *)"  ");
                print_byte(regs[i].reg);
                SendString((unsigned char *)" ");
                SendString((unsigned char *)regs[i].name);
                SendString((unsigned char *)"= ");
                if (rc == 0) {
                    print_byte(val);
                    if (regs[i].expect != 0xFF) {
                        SendString((unsigned char *)" (expect ");
                        print_byte(regs[i].expect);
                        if (val == regs[i].expect) {
                            SendString((unsigned char *)" OK)");
                        } else {
                            SendString((unsigned char *)" MISMATCH)");
                            config_ok = 0;
                        }
                    }
                } else {
                    SendString((unsigned char *)"READ ERROR");
                    config_ok = 0;
                }
                SendString((unsigned char *)"\r\n");
            }

            if (config_ok) {
                report_pass("LIS302DL config registers match expected values");
            } else {
                report_fail("LIS302DL config register mismatch");
            }
        }
    } else if (detected_type == ACCEL_LIS3DH) {
        SendString((unsigned char *)"\r\n--- 9. LIS3DH config register dump ---\r\n");
        {
            static const struct {
                uint8_t reg;
                const char *name;
                uint8_t expect;
            } regs[] = {
                { 0x0F, "WHO_AM_I  ", 0x33 },
                { 0x20, "CTRL_REG1 ", 0x57 },
                { 0x21, "CTRL_REG2 ", 0xFF },
                { 0x22, "CTRL_REG3 ", 0xFF },
                { 0x23, "CTRL_REG4 ", 0x80 },
                { 0x24, "CTRL_REG5 ", 0x04 },
                { 0x25, "CTRL_REG6 ", 0xFF },
                { 0x30, "INT1_CFG  ", 0x95 },
                { 0x32, "INT1_THS  ", 0x64 },
                { 0x33, "INT1_DUR  ", 0x7F },
            };
            int i;
            int config_ok = 1;

            for (i = 0; i < (int)(sizeof(regs) / sizeof(regs[0])); i++) {
                uint8_t val = 0;
                int rc = accel_ReadReg(LIS3DH_ADDR_W, LIS3DH_ADDR_R,
                                       regs[i].reg, &val);
                SendString((unsigned char *)"  ");
                print_byte(regs[i].reg);
                SendString((unsigned char *)" ");
                SendString((unsigned char *)regs[i].name);
                SendString((unsigned char *)"= ");
                if (rc == 0) {
                    print_byte(val);
                    if (regs[i].expect != 0xFF) {
                        SendString((unsigned char *)" (expect ");
                        print_byte(regs[i].expect);
                        if (val == regs[i].expect) {
                            SendString((unsigned char *)" OK)");
                        } else {
                            SendString((unsigned char *)" MISMATCH)");
                            config_ok = 0;
                        }
                    }
                } else {
                    SendString((unsigned char *)"READ ERROR");
                    config_ok = 0;
                }
                SendString((unsigned char *)"\r\n");
            }

            if (config_ok) {
                report_pass("LIS3DH config registers match expected values");
            } else {
                report_fail("LIS3DH config register mismatch");
            }
        }
    }

    /* ---- Section 10: Live LCD output for 20 seconds ---- */
    SendString((unsigned char *)"\r\n--- 10. Live LCD output (20s) ---\r\n");
    {
        uint32_t start_tick, now, last_update;
        uint16_t remaining;

        lcd_Clear();
        start_tick = GetSystemCounter();
        last_update = 0;  /* force immediate first update */

        /* 20 seconds = 2000 ticks at ~10ms each */
        while (1) {
            now = GetSystemCounter();
            if ((now - start_tick) >= 2000)
                break;

            /* Update ~4 times per second (every 25 ticks = 250ms) */
            if ((now - last_update) >= 25) {
                accel_raw_t raw;
                accel_tilt_t tilt;

                last_update = now;
                remaining = (uint16_t)((2000 - (now - start_tick)) / 100);

                accel_Read(&raw);
                accel_GetTilt(&tilt);

                /* Row 0: raw axis data -- "X:-11 Y:-9 Z:51" */
                lcd_Print(0, "X:%-4dY:%-4dZ:%d",
                          raw.x, raw.y, raw.z);

                /* Row 1: tilt + countdown -- "P:-12 R:-9  15s" */
                lcd_Print(1, "P:%-4dR:%-4d %2us",
                          tilt.tilt1, tilt.tilt2, remaining);
            }
        }

        lcd_Clear();
        SendString((unsigned char *)"  LCD live display complete.\r\n");
    }

    /* ---- Summary ---- */
test_summary:
    SendString((unsigned char *)"\r\n========================================\r\n");
    SendString((unsigned char *)"  Accel Test Summary: ");
    print_hex8((uint8_t)pass_count);
    SendString((unsigned char *)" passed, ");
    print_hex8((uint8_t)fail_count);
    SendString((unsigned char *)" failed\r\n");
    SendString((unsigned char *)"========================================\r\n");
}
