/*
 * battery.c - Battery Voltage / Charger Driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_Charge.md
 * and STATES/state_MainMower.md
 *
 * ADC hardware: H8S/2144 on-chip 10-bit ADC with successive approximation.
 * Results are left-justified in 16-bit data registers. The firmware scales
 * readings as: scaled = (raw >> 6) * 5, giving range 0–5115.
 *
 * The battery voltage is filtered through a first-order IIR low-pass filter
 * to reject noise. The charger is detected by comparing ADC channel 2
 * against a fixed threshold (0x833).
 *
 * Original firmware functions:
 *   readADC_Battery          (0x11F2A) -> batt_ReadRawADC()
 *   CheckCharging_GPIO       (0x11F60) -> batt_ReadChargerADC()
 *   CheckCharging            (0x0477A) -> batt_IsCharging()
 *   readBatteryVoltageFiltered (0x077BC) -> batt_ReadFiltered()
 *   config_ApplyBatteryType  (0x03F94) -> batt_ApplyType()
 *   setBatteryStatus         (0x0195E) -> batt_SetStatus()
 *   readRainSensor           (0x078A4) -> batt_ReadRain()
 *   sub_7856                 (0x07856) -> batt_ChargeRelayControl()
 */

#include "battery.h"
#include "../iodefine.h"

/* ---- Internal ADC helper ----
 *
 * Both readADC_Battery and CheckCharging_GPIO use the same pattern:
 *   1. Write channel select + start bit to AD_ADCSR
 *   2. Poll AD_ADCSR bit 7 (ADF) until set
 *   3. Read 16-bit result from AD_ADDRx
 *   4. Scale: (raw >> 6) * 5
 *
 * The decompiler couldn't resolve the poll loop (showed "while(true)")
 * because the ADF flag is set by hardware, invisible to static analysis.
 *
 * Firmware: readADC_Battery (0x11F2A) uses channel 0 → AD_ADDRA
 *           CheckCharging_GPIO (0x11F60) uses channel 2 → AD_ADDRC
 */

/* ---- Module state ---- */

/* IIR filter accumulator (fixed-point with BATT_FILTER_FRAC_BITS bits).
 * 0 = not yet initialized (first call triggers raw ADC seed).
 * Firmware: dword_FFE988 */
static uint32_t filter_accum = 0;

/* Tick of last filter update (for rate limiting).
 * Firmware: dword_FFE98C */
static uint32_t filter_tick = 0;

/* Active voltage thresholds (set by batt_ApplyType).
 * Firmware: word_FFEB58 (charge), word_FFEB5A (low) */
static uint16_t thresh_charge = BATT_B_CHARGE_THRESH;
static uint16_t thresh_low    = BATT_B_LOW_THRESH;

/* Battery status code.
 * Firmware: bBatteryStatus (byte_FFE06C) */
static uint8_t battery_status = BATT_STATUS_NORMAL;

/* Battery type character.
 * Firmware: g_eepromConfig+0x66 */
static uint8_t battery_type = BATT_TYPE_B;

/* Charge relay hysteresis state.
 * Firmware: byte_FFE990 */
static uint8_t relay_hysteresis = 0;

/* Charge relay physical state tracking */
static uint8_t relay_on = 0;

/* Rain sensor flag.
 * Firmware: wChargeEventState1 — we track a simplified version. */
static uint8_t rain_flag = 0;

/* ====================================================================
 * Internal: ADC Read
 * ==================================================================== */

/*
 * adc_read_channel - Read a single ADC channel with scaling
 *
 * @param  ch_select  AD_ADCSR value to start conversion (channel + flags)
 * @param  result_reg Pointer to the 16-bit ADC result register
 * @return Scaled ADC value: (raw >> 6) * 5
 */
static uint16_t adc_read_channel(uint8_t ch_select,
                                  volatile unsigned int *result_reg)
{
    uint16_t raw;

    /* Start single conversion on selected channel */
    AD.ADCSR.BYTE = ch_select;

    /* Wait for ADF (bit 7) to indicate conversion complete.
     * Firmware: polls AD_ADCSR in a tight loop.
     * The decompiler couldn't resolve this because ADF is set
     * by hardware asynchronously. */
    while ((AD.ADCSR.BYTE & BATT_ADC_ADF_MASK) == 0)
        ;

    /* Read 16-bit result (10-bit value, left-justified) */
    raw = *result_reg;

    /* Scale: shift right to get 10-bit value, multiply by 5.
     * This gives range 0–5115 for the full ADC range. */
    return (uint16_t)((raw >> BATT_ADC_SHIFT) * BATT_ADC_SCALE);
}

/* ====================================================================
 * Public API
 * ==================================================================== */

void batt_Init(void)
{
    /* Clear IIR filter state */
    filter_accum = 0;
    filter_tick  = 0;

    /* Default thresholds: battery type B */
    thresh_charge = BATT_B_CHARGE_THRESH;
    thresh_low    = BATT_B_LOW_THRESH;
    battery_type  = BATT_TYPE_B;

    /* Clear status */
    battery_status = BATT_STATUS_NORMAL;

    /* Reset charge relay hysteresis */
    relay_hysteresis = 0;

    /* Ensure charge relay is off at startup */
    P1.DR.BYTE &= (uint8_t)~BATT_P1_CHARGE_RELAY;
    relay_on = 0;

    /* Clear rain flag */
    rain_flag = 0;
}

/*
 * batt_ReadRawADC - Read raw battery voltage (ADC channel 0)
 *
 * Firmware: readADC_Battery (0x11F2A)
 *
 * Ghidra decompilation (reconstructed from hex — decompiler showed
 * infinite loop due to ADF flag being hardware-set):
 *   AD_ADCSR = 0x20;            // Start channel 0 conversion
 *   while (!(AD_ADCSR & 0x80)); // Wait for ADF
 *   raw = AD_ADDRA;             // Read 16-bit result
 *   return (raw >> 6) * 5;      // Scale to voltage units
 */
uint16_t batt_ReadRawADC(void)
{
    return adc_read_channel(BATT_ADC_CH_BATTERY, &AD.ADDRA);
}

/*
 * batt_ReadChargerADC - Read charger voltage (ADC channel 2)
 *
 * Firmware: CheckCharging_GPIO (0x11F60)
 *
 * Same pattern as readADC_Battery but channel 2 → AD_ADDRC.
 */
uint16_t batt_ReadChargerADC(void)
{
    return adc_read_channel(BATT_ADC_CH_CHARGER, &AD.ADDRC);
}

/*
 * batt_IsCharging - Check if charger is connected
 *
 * Firmware: CheckCharging (0x477A)
 *
 * Ghidra decompilation:
 *   uVar1 = CheckCharging_GPIO();
 *   return 0x833 < uVar1;
 */
uint8_t batt_IsCharging(void)
{
    uint16_t charger_v = batt_ReadChargerADC();
    return (charger_v > BATT_CHARGER_THRESHOLD) ? 1 : 0;
}

/*
 * batt_ReadFiltered - Read IIR-filtered battery voltage
 *
 * Firmware: readBatteryVoltageFiltered (0x77BC)
 *
 * Ghidra decompilation:
 *   if (dword_FFE988 == 0) {
 *       uVar1 = readADC_Battery();
 *       dword_FFE988 = (uVar1 & 0xFFFF) << 4;     // seed filter
 *       dword_FFE98C = counter2;
 *   }
 *   if (0x7B < (counter2 - dword_FFE98C)) {        // rate limit
 *       iVar2 = tm_sub_E9C(dword_FFE988, 0x1D);    // acc * 29/30
 *       uVar1 = readADC_Battery();
 *       dword_FFE988 = tm_sub_E4A(
 *           (uVar1 & 0xFFFF) * 0x10 + iVar2, 0x1E); // (new*16 + old*29) / 30
 *       dword_FFE98C = counter2;
 *   }
 *   return dword_FFE988 >> 4;                       // output
 *
 * The filter is a first-order IIR (exponential moving average):
 *   accumulator = (accumulator * 29 + new_sample * 16) / 30
 *   output = accumulator >> 4
 *
 * The fixed-point representation with 4 fractional bits gives
 * sub-LSB resolution for smooth filtering.
 */
uint16_t batt_ReadFiltered(uint32_t tick)
{
    uint16_t raw;
    uint32_t decayed;
    uint32_t new_term;

    /* First call: seed filter from raw ADC */
    if (filter_accum == 0) {
        raw = batt_ReadRawADC();
        filter_accum = (uint32_t)raw << BATT_FILTER_FRAC_BITS;
        filter_tick = tick;
    }

    /* Rate limit: only update every BATT_FILTER_INTERVAL ticks */
    if ((tick - filter_tick) > BATT_FILTER_INTERVAL) {
        /* IIR decay: multiply accumulator by 29/30 */
        decayed = (filter_accum * BATT_FILTER_DECAY_NUM);

        /* New sample: shift left by frac bits (× 16) */
        raw = batt_ReadRawADC();
        new_term = (uint32_t)raw << BATT_FILTER_FRAC_BITS;

        /* Combine and normalize: (old*29 + new*16) / 30 */
        filter_accum = (decayed + new_term) / BATT_FILTER_DECAY_DEN;

        filter_tick = tick;
    }

    /* Return integer part (remove fractional bits) */
    return (uint16_t)(filter_accum >> BATT_FILTER_FRAC_BITS);
}

/*
 * batt_ApplyType - Configure voltage thresholds from battery type
 *
 * Firmware: config_ApplyBatteryType (0x3F94)
 *
 * Ghidra decompilation:
 *   if (battery_type == 'A') {
 *       word_FFEB58 = word_184AA;           // 0x0906 — type A charge
 *       word_FFEB5A = word_184A6;           // 0x094C — type A low
 *       if (hi_batt_sel)
 *           word_FFEB5A = word_184A4;       // 0x094C — type A low (alt)
 *   } else if (battery_type == 'B') {
 *       word_FFEB58 = word_184A8;           // 0x0988 — type B charge
 *       word_FFEB5A = word_184A2;           // 0x09C4 — type B low
 *       if (hi_batt_sel)
 *           word_FFEB5A = word_184A0;       // 0x09C4 — type B low (alt)
 *   }
 */
void batt_ApplyType(uint8_t type, uint8_t hi_batt_sel)
{
    battery_type = type;

    if (type == BATT_TYPE_A) {
        thresh_charge = BATT_A_CHARGE_THRESH;
        thresh_low    = BATT_A_LOW_THRESH;
        if (hi_batt_sel) {
            thresh_low = BATT_A_LOW_ALT_THRESH;
        }
    } else if (type == BATT_TYPE_B) {
        thresh_charge = BATT_B_CHARGE_THRESH;
        thresh_low    = BATT_B_LOW_THRESH;
        if (hi_batt_sel) {
            thresh_low = BATT_B_LOW_ALT_THRESH;
        }
    }
    /* Unknown type: keep existing thresholds */
}

void batt_GetThresholds(batt_thresholds_t *thresh)
{
    thresh->charge_thresh = thresh_charge;
    thresh->low_thresh    = thresh_low;
}

/*
 * batt_SetStatus - Set battery status code
 *
 * Firmware: setBatteryStatus (0x195E)
 *
 * Ghidra decompilation:
 *   if (bBatteryStatus != param_1) {
 *       byte_FFE082 = 1;        // motor control speed reset flag
 *       word_FFE864 = 0;        // clear movement counter
 *       bBatteryStatus = param_1;
 *       // also posts wire steering params based on compass flag
 *   }
 *
 * The original firmware also interacts with the AO framework
 * (posting wire steering params) — that part is omitted here
 * as it's application-level behavior.
 */
void batt_SetStatus(uint8_t status)
{
    battery_status = status;
}

uint8_t batt_GetStatus(void)
{
    return battery_status;
}

/*
 * batt_ChargeRelayOn / batt_ChargeRelayOff - Direct relay control
 *
 * Uses direct P1.DR read-modify-write (no shadow register needed).
 * P1.DDR = 0x20 — bit 5 is the ONLY output on Port 1. All other bits
 * are inputs (keyboard rows, sensors), so EMI-corrupted reads cannot
 * damage other outputs. Original firmware (sub_7856 at 0x07856) also
 * uses direct P1_DR access for the same reason.
 *
 * Firmware: P1_DR bit 5 set/clear in doFullChargeCycle (0x64BA)
 */
void batt_ChargeRelayOn(void)
{
    P1.DR.BYTE |= BATT_P1_CHARGE_RELAY;
    relay_on = 1;
}

void batt_ChargeRelayOff(void)
{
    P1.DR.BYTE &= (uint8_t)~BATT_P1_CHARGE_RELAY;
    relay_on = 0;
}

/*
 * batt_ChargeRelayControl - Hysteresis charge relay control
 *
 * Firmware: sub_7856 (0x7856)
 *
 * Ghidra decompilation:
 *   if (voltage == 0) {
 *       byte_FFE990 = 0;        // reset hysteresis
 *   } else if (byte_FFE990 == 1) {
 *       if (voltage > 0xB7B) {
 *           P1_DR &= ~0x20;     // relay OFF (full)
 *       } else if (voltage < 0xB41) {
 *           P1_DR |= 0x20;      // relay ON (charging)
 *       }
 *       // Between 0xB41 and 0xB7B: no change (deadband)
 *   }
 *
 * The hysteresis prevents rapid relay toggling when the voltage
 * is near the threshold. The ~58 unit deadband (2939 - 2881)
 * corresponds to approximately 0.3V of hysteresis.
 */
void batt_ChargeRelayControl(uint16_t voltage)
{
    if (voltage == 0) {
        /* Reset hysteresis state */
        relay_hysteresis = 0;
        return;
    }

    if (relay_hysteresis != 1) {
        /* Hysteresis not yet active — activate it.
         * On first call with non-zero voltage, start controlling. */
        relay_hysteresis = 1;
    }

    if (voltage > BATT_RELAY_OFF_THRESH) {
        /* Battery full — disable charge relay */
        P1.DR.BYTE &= (uint8_t)~BATT_P1_CHARGE_RELAY;
        relay_on = 0;
    } else if (voltage < BATT_RELAY_ON_THRESH) {
        /* Battery needs charging — enable charge relay */
        P1.DR.BYTE |= BATT_P1_CHARGE_RELAY;
        relay_on = 1;
    }
    /* Between thresholds: no change (deadband) */
}

uint8_t batt_IsLowBattery(uint16_t voltage)
{
    return (voltage < thresh_low) ? 1 : 0;
}

uint8_t batt_NeedsCharge(uint16_t voltage)
{
    return (voltage < thresh_charge) ? 1 : 0;
}

/*
 * batt_ReadRain - Read rain sensor status
 *
 * Firmware: readRainSensor (0x78A4)
 *
 * Ghidra decompilation:
 *   return wChargeEventState1 & 0xFF01;
 *
 * The rain sensor state is maintained in a charge event state word.
 * The firmware reads bits 0 and 8-15 to determine rain status.
 * For this standalone driver, we expose a simple flag that should
 * be updated by the application layer from the charge event system.
 *
 * TODO: Wire this to the actual charge event state when the
 * charge AO module is available.
 */
uint8_t batt_ReadRain(void)
{
    return rain_flag;
}

void batt_GetState(batt_state_t *state)
{
    state->raw_voltage      = batt_ReadRawADC();
    state->filtered_voltage = (uint16_t)(filter_accum >> BATT_FILTER_FRAC_BITS);
    state->charger_voltage  = batt_ReadChargerADC();
    state->filter_accum     = filter_accum;
    state->filter_tick      = filter_tick;
    state->charging         = batt_IsCharging();
    state->relay_on         = relay_on;
    state->status           = battery_status;
    state->battery_type     = battery_type;
    state->thresh.charge_thresh = thresh_charge;
    state->thresh.low_thresh    = thresh_low;
}
