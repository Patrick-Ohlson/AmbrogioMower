/*
 * battery.h - Battery Voltage / Charger Driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_Charge.md
 * and STATES/state_MainMower.md
 *
 * The L200 battery subsystem provides:
 *   - Battery voltage reading via ADC channel 0 (AD_ADDRA)
 *   - Charger voltage detection via ADC channel 2 (AD_ADDRC)
 *   - Filtered (IIR low-pass) battery voltage tracking
 *   - Charge relay control via P1.5
 *   - Battery type configuration (type A or B thresholds)
 *   - Rain sensor status reading
 *   - Charge relay hysteresis control
 *
 * ADC scaling: the H8S/2144 has a 10-bit ADC with result left-justified
 * in a 16-bit register. The firmware scales: result = (raw >> 6) * 5,
 * giving a range of 0–5115 (0x13FB).
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

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

/* ====================================================================
 * ADC Configuration
 *
 * Battery voltage: ADC channel 0, read from AD_ADDRA (0xFFFFE0)
 * Charger voltage: ADC channel 2, read from AD_ADDRC (0xFFFFE4)
 *
 * AD_ADCSR start-conversion values:
 *   0x20 = single conversion, channel 0 (battery)
 *   0x22 = single conversion, channel 2 (charger)
 *
 * Result scaling:
 *   raw = AD_ADDRx (16-bit, 10-bit left-justified)
 *   scaled = (raw >> 6) * 5
 *   Range: 0 to 5115 (0x13FB)
 *
 * Firmware: readADC_Battery (0x11F2A), CheckCharging_GPIO (0x11F60)
 * ==================================================================== */

#define BATT_ADC_CH_BATTERY     0x20    /* AD_ADCSR: channel 0, start conv */
#define BATT_ADC_CH_CHARGER     0x22    /* AD_ADCSR: channel 2, start conv */
#define BATT_ADC_ADF_MASK       0x80    /* AD_ADCSR bit 7: conversion done */
#define BATT_ADC_SHIFT          6       /* Right-shift for 10-bit extraction */
#define BATT_ADC_SCALE          5       /* Multiply factor after shift */

/* ====================================================================
 * Charger Detection Threshold
 *
 * CheckCharging() reads ADC channel 2 and compares against 0x833 (2099).
 * If charger voltage > 0x833, charger is connected.
 *
 * Firmware: CheckCharging (0x477A)
 * ==================================================================== */

#define BATT_CHARGER_THRESHOLD  0x0833  /* ADC value above = charger present */

/* ====================================================================
 * Charge Relay Hysteresis Thresholds
 *
 * sub_7856 controls the charge relay (P1.5) with hysteresis:
 *   voltage > 0xB7B (2939) → disable relay (battery full)
 *   voltage < 0xB41 (2881) → enable relay (needs charging)
 *   Between 0xB41 and 0xB7B → no change (deadband)
 *
 * Firmware: sub_7856 (0x7856)
 * ==================================================================== */

#define BATT_RELAY_OFF_THRESH   0x0B7B  /* 2939 — disable relay (full) */
#define BATT_RELAY_ON_THRESH    0x0B41  /* 2881 — enable relay (charging) */

/* ====================================================================
 * Charge Complete Voltage Target
 *
 * Charge cycle ends when filtered voltage >= this value.
 * From ROM table at 0x184AC.
 *
 * Firmware: word_184AC in doFullChargeCycle (0x64BA)
 * ==================================================================== */

#define BATT_CHARGE_COMPLETE    0x0B54  /* 2900 — charge complete target */

/* ====================================================================
 * Battery Type Voltage Thresholds
 *
 * config_ApplyBatteryType (0x3F94) selects thresholds based on the
 * battery_type field ('A' or 'B') in EEPROM config at offset +0x66.
 *
 * The featureFlags bit 3 (CFG_FEAT_HI_BATT_SEL) selects between
 * normal and alternate low-battery thresholds.
 *
 * ROM table at 0x184A0 (all values are scaled ADC units):
 *
 *   word_184A0 = 0x09C4 (2500) — Battery B alt low threshold
 *   word_184A2 = 0x09C4 (2500) — Battery B normal low threshold
 *   word_184A4 = 0x094C (2380) — Battery A alt low threshold
 *   word_184A6 = 0x094C (2380) — Battery A normal low threshold
 *   word_184A8 = 0x0988 (2440) — Battery B charge threshold
 *   word_184AA = 0x0906 (2310) — Battery A charge threshold
 *
 * After applying type:
 *   word_FFEB58 = charge threshold (when to return for charging)
 *   word_FFEB5A = low battery threshold (critical low warning)
 *
 * Firmware: config_ApplyBatteryType (0x3F94)
 * ==================================================================== */

/* Battery type A thresholds */
#define BATT_A_CHARGE_THRESH    0x0906  /* 2310 — type A return-to-charge */
#define BATT_A_LOW_THRESH       0x094C  /* 2380 — type A low battery */
#define BATT_A_LOW_ALT_THRESH   0x094C  /* 2380 — type A low (alt select) */

/* Battery type B thresholds */
#define BATT_B_CHARGE_THRESH    0x0988  /* 2440 — type B return-to-charge */
#define BATT_B_LOW_THRESH       0x09C4  /* 2500 — type B low battery */
#define BATT_B_LOW_ALT_THRESH   0x09C4  /* 2500 — type B low (alt select) */

/* Battery type codes (from EEPROM config +0x66) */
#define BATT_TYPE_A             'A'     /* 0x41 */
#define BATT_TYPE_B             'B'     /* 0x42 — factory default */

/* ====================================================================
 * Filtered Voltage IIR Parameters
 *
 * readBatteryVoltageFiltered (0x77BC) implements a first-order IIR
 * low-pass filter on the battery ADC reading:
 *
 *   accumulator = (accumulator * 29/30) + (new_sample * 16)
 *   output = accumulator >> 4
 *
 * Rate-limited: only updates when (tick - last_tick) > 0x7B (123 ticks).
 * On first call, initializes accumulator = raw_adc << 4.
 *
 * Firmware: readBatteryVoltageFiltered (0x77BC)
 *   dword_FFE988 = accumulator (fixed-point, 4 fractional bits)
 *   dword_FFE98C = last sample tick
 *   tm_sub_E9C(acc, 0x1D) = acc * 29 / 30 (IIR decay)
 *   tm_sub_E4A(sum, 0x1E) = sum / 30 (IIR normalize)
 * ==================================================================== */

#define BATT_FILTER_INTERVAL    0x7B    /* 123 ticks between samples */
#define BATT_FILTER_FRAC_BITS   4       /* Fixed-point fractional bits */
#define BATT_FILTER_DECAY_NUM   29      /* IIR decay numerator (0x1D) */
#define BATT_FILTER_DECAY_DEN   30      /* IIR decay denominator (0x1E) */

/* ====================================================================
 * Battery Status Codes
 *
 * bBatteryStatus (byte_FFE06C) tracks the mower's battery/charging state.
 * Used by MainMower to decide whether to mow, charge, or return home.
 *
 * Firmware: setBatteryStatus (0x195E), state_MainMower.md
 * ==================================================================== */

#define BATT_STATUS_NORMAL      0       /* Normal operation — continue mowing */
#define BATT_STATUS_WAS_CHARGING 1      /* Was charging — resume from charge */
#define BATT_STATUS_LOW_WARN    2       /* Low battery warning */
#define BATT_STATUS_NEAR_ZONE   3       /* Near charge zone — approach station */
#define BATT_STATUS_TIMEOUT     5       /* Timeout — force return to charge */
#define BATT_STATUS_FULL_START  6       /* Full charge at startup */
#define BATT_STATUS_RAIN        7       /* Rain detected — return to station */
#define BATT_STATUS_JUST_CHARGED 8      /* Just charged — fresh start */
#define BATT_STATUS_TILT        9       /* Tilt/lift safety stop */

/* ====================================================================
 * Charge Relay GPIO
 *
 * Charge relay is on P1.5:
 *   P1.DR |= 0x20  → relay ON (charging enabled)
 *   P1.DR &= ~0x20 → relay OFF (charging disabled)
 *
 * Firmware: P1_DR bit 5 in doFullChargeCycle and sub_7856
 * ==================================================================== */

#define BATT_P1_CHARGE_RELAY    0x20    /* P1.5 — charge relay control */

/* ====================================================================
 * Charge Timing
 *
 * Max charge time: 0xF5371 ticks (~1,004,401 ticks)
 * At ~100 Hz main loop = ~2.8 hours
 * (state_Charge.md says ~28 hours — tick rate may be 10 Hz)
 *
 * Charge test wait: 0x7C (124) ticks after connecting
 *
 * Firmware: doFullChargeCycle (0x64BA)
 * ==================================================================== */

#define BATT_CHARGE_MAX_TICKS   0xF5371 /* Max charge duration (ticks) */
#define BATT_CHARGE_TEST_TICKS  0x7C    /* Charge test wait period */
#define BATT_CHARGE_STALL_TICKS 0x28DE8 /* No voltage rise timeout */

/* ====================================================================
 * Types
 * ==================================================================== */

/* Battery voltage thresholds (set by batt_ApplyType) */
typedef struct {
    uint16_t charge_thresh;     /* Return-to-charge threshold (word_FFEB58) */
    uint16_t low_thresh;        /* Low battery threshold (word_FFEB5A) */
} batt_thresholds_t;

/* Battery/charger subsystem state */
typedef struct {
    uint16_t raw_voltage;       /* Last raw ADC battery voltage (scaled) */
    uint16_t filtered_voltage;  /* IIR filtered battery voltage */
    uint16_t charger_voltage;   /* Last charger ADC reading (scaled) */
    uint32_t filter_accum;      /* IIR filter accumulator (fixed-point) */
    uint32_t filter_tick;       /* Tick of last filter update */
    uint8_t  charging;          /* 1 = charger connected (ADC > threshold) */
    uint8_t  relay_on;          /* 1 = charge relay enabled (P1.5 high) */
    uint8_t  status;            /* Battery status code (BATT_STATUS_*) */
    uint8_t  battery_type;      /* Battery type: 'A' or 'B' */
    batt_thresholds_t thresh;   /* Active voltage thresholds */
} batt_state_t;

/* ====================================================================
 * Public API
 * ==================================================================== */

/*
 * batt_Init - Initialize battery/charger subsystem
 *
 * Clears filter state, sets default thresholds (type B),
 * and disables the charge relay.
 *
 * Call once at startup.
 */
void batt_Init(void);

/*
 * batt_ReadRawADC - Read raw battery voltage from ADC channel 0
 *
 * Starts a single ADC conversion on channel 0, waits for completion,
 * reads the 16-bit result and scales: (raw >> 6) * 5.
 *
 * @return  Scaled ADC value (0–5115)
 *
 * Firmware: readADC_Battery (0x11F2A)
 */
uint16_t batt_ReadRawADC(void);

/*
 * batt_ReadChargerADC - Read charger voltage from ADC channel 2
 *
 * Same as batt_ReadRawADC but uses ADC channel 2 (AD_ADDRC).
 *
 * @return  Scaled ADC value (0–5115)
 *
 * Firmware: CheckCharging_GPIO (0x11F60)
 */
uint16_t batt_ReadChargerADC(void);

/*
 * batt_IsCharging - Check if charger is connected
 *
 * Reads charger ADC and compares against threshold (0x833).
 *
 * @return  1 if charger connected, 0 if not
 *
 * Firmware: CheckCharging (0x477A)
 */
uint8_t batt_IsCharging(void);

/*
 * batt_ReadFiltered - Read IIR-filtered battery voltage
 *
 * Returns the filtered battery voltage. The filter is updated
 * at most every BATT_FILTER_INTERVAL ticks (0x7B = 123).
 * On first call, initializes the filter from a raw ADC read.
 *
 * IIR filter: output = ((accumulator * 29) + (new_sample * 16)) / 30
 * This provides ~30-sample exponential smoothing.
 *
 * @param tick  Current system tick counter
 * @return  Filtered voltage (same scale as raw ADC: 0–5115)
 *
 * Firmware: readBatteryVoltageFiltered (0x77BC)
 */
uint16_t batt_ReadFiltered(uint32_t tick);

/*
 * batt_ApplyType - Configure voltage thresholds from battery type
 *
 * Sets the charge and low-battery voltage thresholds based on the
 * battery type ('A' or 'B') and the high-battery-select feature flag.
 *
 * @param type         Battery type: BATT_TYPE_A ('A') or BATT_TYPE_B ('B')
 * @param hi_batt_sel  1 = alternate threshold, 0 = normal threshold
 *                     (from featureFlags bit 3: CFG_FEAT_HI_BATT_SEL)
 *
 * Firmware: config_ApplyBatteryType (0x3F94)
 */
void batt_ApplyType(uint8_t type, uint8_t hi_batt_sel);

/*
 * batt_GetThresholds - Get current voltage thresholds
 *
 * @param thresh  Pointer to batt_thresholds_t to populate
 */
void batt_GetThresholds(batt_thresholds_t *thresh);

/*
 * batt_SetStatus - Set battery status code
 *
 * Updates the battery status byte. When status changes, resets
 * the motor control speed reference.
 *
 * @param status  New status code (BATT_STATUS_*)
 *
 * Firmware: setBatteryStatus (0x195E)
 */
void batt_SetStatus(uint8_t status);

/*
 * batt_GetStatus - Get current battery status code
 *
 * @return  Current status (BATT_STATUS_*)
 */
uint8_t batt_GetStatus(void);

/*
 * batt_ChargeRelayOn - Enable charge relay
 *
 * Sets P1.5 high to close the charge relay.
 *
 * Firmware: P1_DR |= 0x20 in doFullChargeCycle
 */
void batt_ChargeRelayOn(void);

/*
 * batt_ChargeRelayOff - Disable charge relay
 *
 * Clears P1.5 to open the charge relay.
 *
 * Firmware: P1_DR &= ~0x20 in doFullChargeCycle
 */
void batt_ChargeRelayOff(void);

/*
 * batt_ChargeRelayControl - Hysteresis charge relay control
 *
 * Automatic charge relay management based on battery voltage:
 *   voltage > BATT_RELAY_OFF_THRESH (0xB7B) → relay OFF (full)
 *   voltage < BATT_RELAY_ON_THRESH  (0xB41) → relay ON  (charging)
 *   Between thresholds → no change (deadband)
 *
 * Pass voltage = 0 to reset the hysteresis controller state.
 *
 * @param voltage  Current filtered battery voltage, or 0 to reset
 *
 * Firmware: sub_7856 (0x7856)
 */
void batt_ChargeRelayControl(uint16_t voltage);

/*
 * batt_IsLowBattery - Check if battery is below low threshold
 *
 * Compares the given voltage against the active low-battery threshold
 * (set by batt_ApplyType).
 *
 * @param voltage  Battery voltage (from batt_ReadFiltered)
 * @return  1 if voltage < low_thresh, 0 if OK
 */
uint8_t batt_IsLowBattery(uint16_t voltage);

/*
 * batt_NeedsCharge - Check if battery needs charging
 *
 * Compares the given voltage against the active charge threshold
 * (set by batt_ApplyType).
 *
 * @param voltage  Battery voltage (from batt_ReadFiltered)
 * @return  1 if voltage < charge_thresh, 0 if OK
 */
uint8_t batt_NeedsCharge(uint16_t voltage);

/*
 * batt_ReadRain - Read rain sensor status
 *
 * Returns the rain sensor state from the charge event flags.
 *
 * @return  Non-zero if rain detected, 0 if dry
 *
 * Firmware: readRainSensor (0x78A4)
 */
uint8_t batt_ReadRain(void);

/*
 * batt_GetState - Get full battery/charger state (for diagnostics)
 *
 * @param state  Pointer to batt_state_t to populate
 */
void batt_GetState(batt_state_t *state);

#endif /* BATTERY_H */
