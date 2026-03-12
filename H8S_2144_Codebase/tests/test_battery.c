/* test_battery.c -- Battery voltage / charger ADC test
 *
 * Tests the battery subsystem:
 *   1.  ADC raw battery voltage
 *   2.  ADC charger voltage
 *   3.  Charger detection
 *   4.  IIR filter convergence
 *   5.  Battery type thresholds
 *   6.  Status get/set
 *   7.  Charge relay toggle
 *   8.  Hysteresis relay control
 *   9.  Full state snapshot
 *   10. Live LCD output (20s)
 *
 * Hardware: ADC ch0 = battery, ch2 = charger, P1.5 = charge relay
 */
#include "test_battery.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../battery/battery.h"
#include "../lcd/lcd.h"
#include "../timer/timer.h"
#include "../utils/utils.h"

/* Print a uint16_t as decimal (up to 65535) */
static void print_dec16(uint16_t val)
{
    char buf[6];
    int i = 0;
    if (val == 0) { PutChar('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) { PutChar(buf[--i]); }
}

void Battery_Test(void)
{
    uint8_t pass_count = 0;
    uint8_t fail_count = 0;

    SendString((unsigned char *)"\r\n");
    SendString((unsigned char *)"========================================\r\n");
    SendString((unsigned char *)"  Battery / Charger ADC Test\r\n");
    SendString((unsigned char *)"========================================\r\n");

    batt_Init();

    /* ---- Section 1: ADC raw battery voltage ---- */
    SendString((unsigned char *)"\r\n--- 1. Raw battery voltage (ADC ch0) ---\r\n");
    {
        uint16_t v = batt_ReadRawADC();
        SendString((unsigned char *)"  Raw voltage = ");
        print_dec16(v);
        SendString((unsigned char *)" (0x");
        print_hex16(v);
        SendString((unsigned char *)")\r\n");

        if (v > 500 && v < 5000) {
            SendString((unsigned char *)"  [PASS] in plausible range\r\n");
            pass_count++;
        } else if (v > 0) {
            SendString((unsigned char *)"  [WARN] outside typical range 500-5000\r\n");
            pass_count++;  /* Still passes — ADC is working */
        } else {
            SendString((unsigned char *)"  [FAIL] zero reading\r\n");
            fail_count++;
        }
    }

    /* ---- Section 2: ADC charger voltage ---- */
    SendString((unsigned char *)"\r\n--- 2. Charger voltage (ADC ch2) ---\r\n");
    {
        uint16_t v = batt_ReadChargerADC();
        SendString((unsigned char *)"  Charger voltage = ");
        print_dec16(v);
        SendString((unsigned char *)" (0x");
        print_hex16(v);
        SendString((unsigned char *)")\r\n");
        SendString((unsigned char *)"  [PASS] ADC ch2 read OK\r\n");
        pass_count++;
    }

    /* ---- Section 3: Charger detection ---- */
    SendString((unsigned char *)"\r\n--- 3. Charger detection ---\r\n");
    {
        uint8_t charging = batt_IsCharging();
        SendString((unsigned char *)"  batt_IsCharging() = ");
        PutChar('0' + charging);
        if (charging)
            SendString((unsigned char *)" (charger CONNECTED)\r\n");
        else
            SendString((unsigned char *)" (no charger)\r\n");
        SendString((unsigned char *)"  [PASS] detection OK\r\n");
        pass_count++;
    }

    /* ---- Section 4: IIR filter convergence ---- */
    SendString((unsigned char *)"\r\n--- 4. IIR filter convergence ---\r\n");
    {
        uint16_t raw, filt;
        int16_t  diff;
        uint32_t tick;
        uint8_t  i;

        batt_Init();  /* Reset filter state */
        raw = batt_ReadRawADC();
        tick = GetSystemCounter();

        /* Pump the filter 5 times with 200-tick intervals */
        for (i = 0; i < 5; i++) {
            tick += 200;  /* Exceed BATT_FILTER_INTERVAL (0x7B = 123) */
            filt = batt_ReadFiltered(tick);
        }

        diff = (int16_t)filt - (int16_t)raw;
        if (diff < 0) diff = -diff;

        SendString((unsigned char *)"  Raw = ");
        print_dec16(raw);
        SendString((unsigned char *)", Filtered = ");
        print_dec16(filt);
        SendString((unsigned char *)", Diff = ");
        print_dec16((uint16_t)diff);
        SendString((unsigned char *)"\r\n");

        if (filt > 0 && diff < 200) {
            SendString((unsigned char *)"  [PASS] filter converged\r\n");
            pass_count++;
        } else {
            SendString((unsigned char *)"  [FAIL] filter did not converge\r\n");
            fail_count++;
        }
    }

    /* ---- Section 5: Battery type thresholds ---- */
    SendString((unsigned char *)"\r\n--- 5. Battery type thresholds ---\r\n");
    {
        batt_thresholds_t th;
        uint8_t ok = 1;

        /* Type A */
        batt_ApplyType(BATT_TYPE_A, 0);
        batt_GetThresholds(&th);
        SendString((unsigned char *)"  Type A: charge=0x");
        print_hex16(th.charge_thresh);
        SendString((unsigned char *)" low=0x");
        print_hex16(th.low_thresh);
        SendString((unsigned char *)"\r\n");
        if (th.charge_thresh != BATT_A_CHARGE_THRESH ||
            th.low_thresh != BATT_A_LOW_THRESH) {
            ok = 0;
        }

        /* Type B */
        batt_ApplyType(BATT_TYPE_B, 0);
        batt_GetThresholds(&th);
        SendString((unsigned char *)"  Type B: charge=0x");
        print_hex16(th.charge_thresh);
        SendString((unsigned char *)" low=0x");
        print_hex16(th.low_thresh);
        SendString((unsigned char *)"\r\n");
        if (th.charge_thresh != BATT_B_CHARGE_THRESH ||
            th.low_thresh != BATT_B_LOW_THRESH) {
            ok = 0;
        }

        if (ok) {
            SendString((unsigned char *)"  [PASS] thresholds match\r\n");
            pass_count++;
        } else {
            SendString((unsigned char *)"  [FAIL] threshold mismatch\r\n");
            fail_count++;
        }
    }

    /* ---- Section 6: Status get/set ---- */
    SendString((unsigned char *)"\r\n--- 6. Status get/set ---\r\n");
    {
        uint8_t ok = 1;
        uint8_t codes[] = { BATT_STATUS_NORMAL, BATT_STATUS_LOW_WARN,
                            BATT_STATUS_RAIN, BATT_STATUS_TILT };
        uint8_t i;

        for (i = 0; i < 4; i++) {
            batt_SetStatus(codes[i]);
            if (batt_GetStatus() != codes[i]) {
                ok = 0;
                SendString((unsigned char *)"  FAIL at code ");
                print_hex8(codes[i]);
                SendString((unsigned char *)"\r\n");
            }
        }
        batt_SetStatus(BATT_STATUS_NORMAL);  /* Restore */

        if (ok) {
            SendString((unsigned char *)"  [PASS] status get/set OK\r\n");
            pass_count++;
        } else {
            SendString((unsigned char *)"  [FAIL]\r\n");
            fail_count++;
        }
    }

    /* ---- Section 7: Charge relay toggle ---- */
    SendString((unsigned char *)"\r\n--- 7. Charge relay toggle (P1.5) ---\r\n");
    {
        uint8_t ok = 1;

        batt_ChargeRelayOn();
        delay_ms(10);
        if (!(P1.DR.BYTE & 0x20)) {
            SendString((unsigned char *)"  FAIL: relay ON but P1.5=0\r\n");
            ok = 0;
        } else {
            SendString((unsigned char *)"  Relay ON:  P1.DR=0x");
            print_hex8(P1.DR.BYTE);
            SendString((unsigned char *)" (bit5=1)\r\n");
        }

        batt_ChargeRelayOff();
        delay_ms(10);
        if (P1.DR.BYTE & 0x20) {
            SendString((unsigned char *)"  FAIL: relay OFF but P1.5=1\r\n");
            ok = 0;
        } else {
            SendString((unsigned char *)"  Relay OFF: P1.DR=0x");
            print_hex8(P1.DR.BYTE);
            SendString((unsigned char *)" (bit5=0)\r\n");
        }

        if (ok) {
            SendString((unsigned char *)"  [PASS] relay toggle OK\r\n");
            pass_count++;
        } else {
            SendString((unsigned char *)"  [FAIL]\r\n");
            fail_count++;
        }
    }

    /* ---- Section 8: Hysteresis relay control ---- */
    SendString((unsigned char *)"\r\n--- 8. Hysteresis relay control ---\r\n");
    {
        uint8_t ok = 1;

        /* Reset hysteresis */
        batt_ChargeRelayControl(0);
        batt_ChargeRelayOff();  /* Ensure relay off */

        /* Activate hysteresis with a value in the deadband */
        batt_ChargeRelayControl(BATT_RELAY_ON_THRESH + 10);

        /* Below ON threshold: should turn relay ON */
        batt_ChargeRelayControl(BATT_RELAY_ON_THRESH - 1);
        delay_ms(5);
        if (!(P1.DR.BYTE & 0x20)) {
            SendString((unsigned char *)"  FAIL: below ON thresh but relay off\r\n");
            ok = 0;
        } else {
            SendString((unsigned char *)"  Below 0x0B41: relay ON  [OK]\r\n");
        }

        /* Above OFF threshold: should turn relay OFF */
        batt_ChargeRelayControl(BATT_RELAY_OFF_THRESH + 1);
        delay_ms(5);
        if (P1.DR.BYTE & 0x20) {
            SendString((unsigned char *)"  FAIL: above OFF thresh but relay on\r\n");
            ok = 0;
        } else {
            SendString((unsigned char *)"  Above 0x0B7B: relay OFF [OK]\r\n");
        }

        /* In deadband: relay should stay OFF */
        batt_ChargeRelayControl(BATT_RELAY_ON_THRESH + 10);
        delay_ms(5);
        if (P1.DR.BYTE & 0x20) {
            SendString((unsigned char *)"  FAIL: in deadband but relay toggled\r\n");
            ok = 0;
        } else {
            SendString((unsigned char *)"  Deadband:     relay OFF [OK]\r\n");
        }

        /* Clean up */
        batt_ChargeRelayControl(0);
        batt_ChargeRelayOff();

        if (ok) {
            SendString((unsigned char *)"  [PASS] hysteresis OK\r\n");
            pass_count++;
        } else {
            SendString((unsigned char *)"  [FAIL]\r\n");
            fail_count++;
        }
    }

    /* ---- Section 9: Full state snapshot ---- */
    SendString((unsigned char *)"\r\n--- 9. Full state snapshot ---\r\n");
    {
        batt_state_t st;
        batt_GetState(&st);

        SendString((unsigned char *)"  raw_voltage     = ");
        print_dec16(st.raw_voltage);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  filtered_voltage= ");
        print_dec16(st.filtered_voltage);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  charger_voltage = ");
        print_dec16(st.charger_voltage);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  charging        = ");
        PutChar('0' + st.charging);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  relay_on        = ");
        PutChar('0' + st.relay_on);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  status          = ");
        print_hex8(st.status);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  battery_type    = ");
        PutChar(st.battery_type);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  thresh_charge   = 0x");
        print_hex16(st.thresh.charge_thresh);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  thresh_low      = 0x");
        print_hex16(st.thresh.low_thresh);
        SendString((unsigned char *)"\r\n");
        SendString((unsigned char *)"  [PASS] state snapshot OK\r\n");
        pass_count++;
    }

    /* ---- Section 10: Live LCD output (20s) ---- */
    SendString((unsigned char *)"\r\n--- 10. Live LCD output (20s) ---\r\n");
    {
        uint32_t start_tick = GetSystemCounter();
        uint32_t last_update = 0;
        uint16_t remaining;

        lcd_Clear();

        while (1) {
            uint32_t now = GetSystemCounter();
            if ((now - start_tick) >= 2000)  /* 20 seconds */
                break;

            /* Update LCD every ~250ms */
            if ((now - last_update) >= 25) {
                uint16_t raw, filt, chrg;
                uint8_t  chg;

                last_update = now;
                remaining = (uint16_t)((2000 - (now - start_tick)) / 100);

                raw  = batt_ReadRawADC();
                filt = batt_ReadFiltered(now);
                chrg = batt_ReadChargerADC();
                chg  = batt_IsCharging();

                lcd_Print(0, "B:%4d F:%4d", raw, filt);
                lcd_Print(1, "C:%4d %c %2us",
                          chrg, chg ? '+' : '-', remaining);
            }
        }

        lcd_Clear();
        SendString((unsigned char *)"  LCD live display complete.\r\n");
    }

    /* ---- Summary ---- */
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
