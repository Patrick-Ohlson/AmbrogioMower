# Battery / Charger Driver — battery

## Module Summary

Low-level hardware driver for the L200's battery voltage monitoring and charger
detection subsystem. Provides raw and IIR-filtered ADC battery voltage readings,
charger presence detection, charge relay control with hysteresis, battery type
threshold configuration, battery status tracking, and rain sensor status.

| Property | Value |
|----------|-------|
| Battery ADC | Channel 0, AD_ADDRA (0xFFFFE0) |
| Charger ADC | Channel 2, AD_ADDRC (0xFFFFE4) |
| ADC scaling | (raw >> 6) × 5, range 0–5115 |
| Charge relay | P1.5 (active-high) |
| Battery types | 'A' (lower thresholds) / 'B' (higher, factory default) |
| IIR filter | First-order, 29/30 decay, 4 fractional bits |
| Source doc | `STATES/state_Charge.md`, `STATES/state_MainMower.md` |

## Files

| File | Description |
|------|-------------|
| `battery.h` | Types, ADC/threshold constants, API declarations |
| `battery.c` | Implementation (ADC, filter, relay, status) |
| `battery_doc.md` | This file |

## How to Include

```c
#include "battery/battery.h"
```

```makefile
SRCS += battery/battery.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | Fixed-width types |
| `iodefine.h` | AD, P1 register access |

## Architecture

### ADC Reading Pipeline

```
batt_ReadRawADC() / batt_ReadChargerADC()
      │
      ▼
AD_ADCSR = channel select   (0x20 = ch0, 0x22 = ch2)
      │
      ▼
Poll AD_ADCSR bit 7 (ADF)   ← Hardware sets when conversion done
      │
      ▼
Read AD_ADDRx (16-bit)       ← 10-bit result, left-justified
      │
      ▼
Scale: (raw >> 6) × 5        ← Range 0–5115 (0x0000–0x13FB)
      │
      ▼
Return scaled value
```

### ADC Voltage Mapping

```
 ADC raw (10-bit left-justified in 16-bit register)
 ┌─────────────────────────────────────────────┐
 │ D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 │ 0...0 │
 │ ←── 10-bit result ──────────────────→ │ pad   │
 └─────────────────────────────────────────────┘
   >> 6  →  10-bit value (0–1023)
   × 5   →  scaled (0–5115)

 Approximate voltage mapping (5V reference):
   scaled 0    → 0.0V
   scaled 2310 → ~2.3V  (type A charge threshold)
   scaled 2500 → ~2.4V  (type B low battery)
   scaled 2900 → ~2.8V  (charge complete)
   scaled 5115 → ~5.0V  (full scale)
```

### IIR Low-Pass Filter

```
batt_ReadFiltered(tick)
      │
      ├── First call (accumulator == 0)?
      │     │
      │     ▼ YES
      │   raw = batt_ReadRawADC()
      │   accumulator = raw << 4         ← Seed with 4 fractional bits
      │   last_tick = tick
      │
      ├── (tick - last_tick) > 0x7B?     ← Rate limit: 123 ticks
      │     │
      │     ▼ YES
      │   decayed = accumulator × 29     ← IIR decay (numerator)
      │   raw = batt_ReadRawADC()
      │   new_term = raw << 4            ← New sample × 16
      │   accumulator = (decayed + new_term) / 30
      │   last_tick = tick               ← IIR normalize (denominator)
      │
      ▼
return accumulator >> 4                  ← Remove fractional bits

Filter characteristics:
  Time constant: ~30 samples
  Update rate: every 123 ticks
  Effective smoothing window: ~30 × 123 = 3690 ticks
  Fixed-point: 4 fractional bits (×16 resolution)
```

### Charger Detection

```
batt_IsCharging()
      │
      ▼
charger_v = batt_ReadChargerADC()    ← ADC channel 2
      │
      ▼
charger_v > 0x0833 (2099)?
      │
      ├── YES → return 1 (charger connected)
      └── NO  → return 0 (no charger)
```

### Charge Relay Hysteresis

```
batt_ChargeRelayControl(voltage)
      │
      ├── voltage == 0?
      │     ▼ YES
      │   Reset hysteresis state → return
      │
      ├── voltage > 0x0B7B (2939)?
      │     ▼ YES
      │   P1.5 = 0 (relay OFF, battery full)
      │
      ├── voltage < 0x0B41 (2881)?
      │     ▼ YES
      │   P1.5 = 1 (relay ON, needs charge)
      │
      └── Between thresholds?
            ▼
          No change (deadband prevents oscillation)

                 Relay State Diagram
 ┌──────────────────────────────────────────────────┐
 │                                                  │
 │   OFF ◄──────── voltage > 2939 ──────────┐      │
 │    │                                      │      │
 │    │         ┌── DEADBAND ──┐             │      │
 │    │         │  2881–2939   │             │      │
 │    │         │  (no change) │             │      │
 │    │         └──────────────┘             │      │
 │    │                                      │      │
 │    └── voltage < 2881 ──────────────► ON ─┘      │
 │                                                  │
 └──────────────────────────────────────────────────┘
```

### Battery Type Configuration

```
batt_ApplyType(type, hi_batt_sel)
      │
      ├── type == 'A' (0x41)?
      │     charge_thresh = 0x0906 (2310)
      │     low_thresh    = 0x094C (2380)
      │     if (hi_batt_sel)
      │         low_thresh = 0x094C (2380)   ← Same for type A
      │
      └── type == 'B' (0x42)?
            charge_thresh = 0x0988 (2440)
            low_thresh    = 0x09C4 (2500)
            if (hi_batt_sel)
                low_thresh = 0x09C4 (2500)   ← Same for type B

ROM Threshold Table (0x184A0):
 ┌─────────┬──────────────────┬───────────┬───────────┐
 │ Address │ Value            │ Scaled    │ Purpose   │
 ├─────────┼──────────────────┼───────────┼───────────┤
 │ 0x184A0 │ 0x09C4           │ 2500      │ B alt low │
 │ 0x184A2 │ 0x09C4           │ 2500      │ B low     │
 │ 0x184A4 │ 0x094C           │ 2380      │ A alt low │
 │ 0x184A6 │ 0x094C           │ 2380      │ A low     │
 │ 0x184A8 │ 0x0988           │ 2440      │ B charge  │
 │ 0x184AA │ 0x0906           │ 2310      │ A charge  │
 │ 0x184AC │ 0x0B54           │ 2900      │ Complete  │
 └─────────┴──────────────────┴───────────┴───────────┘
```

### Battery Status State Diagram

```
 ┌───────────────────────────────────────────────────────────────┐
 │                    Battery Status Codes                       │
 ├───────┬──────────────────────────────────────────────────────┤
 │  0    │ NORMAL — continue mowing                             │
 │  1    │ WAS_CHARGING — resume from charge station            │
 │  2    │ LOW_WARN — low battery warning, return to charge     │
 │  3    │ NEAR_ZONE — approaching charge station               │
 │  5    │ TIMEOUT — forced return due to timeout               │
 │  6    │ FULL_START — starting after full charge              │
 │  7    │ RAIN — rain detected, return to station              │
 │  8    │ JUST_CHARGED — freshly charged, ready to mow        │
 │  9    │ TILT — tilt/lift safety triggered                    │
 └───────┴──────────────────────────────────────────────────────┘
```

## API Reference

### Initialization

#### `void batt_Init(void)`

Initialize battery/charger subsystem. Clears IIR filter state, sets default
thresholds (type B), disables charge relay (P1.5 = 0), resets status to NORMAL.

Call once at startup.

### ADC Readings

| Function | Description |
|----------|-------------|
| `batt_ReadRawADC()` | Read battery voltage, ADC ch0 → scaled (0–5115) |
| `batt_ReadChargerADC()` | Read charger voltage, ADC ch2 → scaled (0–5115) |
| `batt_IsCharging()` | 1 = charger connected (ADC > 0x833), 0 = absent |
| `batt_ReadFiltered(tick)` | IIR-filtered battery voltage, rate-limited |

### Battery Type Configuration

| Function | Description |
|----------|-------------|
| `batt_ApplyType(type, hi_batt_sel)` | Set voltage thresholds for type A/B |
| `batt_GetThresholds(thresh)` | Read current charge/low thresholds |
| `batt_IsLowBattery(voltage)` | 1 = below low threshold |
| `batt_NeedsCharge(voltage)` | 1 = below charge threshold |

### Battery Status

| Function | Description |
|----------|-------------|
| `batt_SetStatus(status)` | Set battery status code (BATT_STATUS_*) |
| `batt_GetStatus()` | Get current battery status code |

### Charge Relay Control

| Function | Description |
|----------|-------------|
| `batt_ChargeRelayOn()` | P1.5 = 1, close relay |
| `batt_ChargeRelayOff()` | P1.5 = 0, open relay |
| `batt_ChargeRelayControl(v)` | Hysteresis: OFF > 0xB7B, ON < 0xB41 |

### Diagnostics

| Function | Description |
|----------|-------------|
| `batt_ReadRain()` | Rain sensor flag (stub — needs charge AO) |
| `batt_GetState(state)` | Fill batt_state_t with full subsystem snapshot |

## Register Map

| Register | Address | Bits Used | Function |
|----------|---------|-----------|----------|
| AD.ADCSR | 0xFFFFE8 | 8-bit | ADC control/status (start, channel, ADF) |
| AD.ADDRA | 0xFFFFE0 | 16-bit | Battery ADC result (ch0, left-justified) |
| AD.ADDRC | 0xFFFFE4 | 16-bit | Charger ADC result (ch2, left-justified) |
| P1.DR | 0xFFFFAC | bit 5 | Charge relay control (active-high) |

## RAM Variable Map

| Our Variable | Firmware Address | Type | Description |
|-------------|-----------------|------|-------------|
| `filter_accum` | dword_FFE988 | uint32 | IIR filter accumulator (fixed-point) |
| `filter_tick` | dword_FFE98C | uint32 | Last filter update tick |
| `thresh_charge` | word_FFEB58 | uint16 | Active charge threshold |
| `thresh_low` | word_FFEB5A | uint16 | Active low-battery threshold |
| `battery_status` | byte_FFE06C | uint8 | Battery status code |
| `relay_hysteresis` | byte_FFE990 | uint8 | Relay hysteresis state flag |

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `batt_Init` | (composite) | — |
| `batt_ReadRawADC` | `readADC_Battery` | 0x11F2A |
| `batt_ReadChargerADC` | `CheckCharging_GPIO` | 0x11F60 |
| `batt_IsCharging` | `CheckCharging` | 0x0477A |
| `batt_ReadFiltered` | `readBatteryVoltageFiltered` | 0x077BC |
| `batt_ApplyType` | `config_ApplyBatteryType` | 0x03F94 |
| `batt_SetStatus` | `setBatteryStatus` | 0x0195E |
| `batt_ReadRain` | `readRainSensor` | 0x078A4 |
| `batt_ChargeRelayControl` | `sub_7856` | 0x07856 |
| `batt_ChargeRelayOn/Off` | (inline in `doFullChargeCycle`) | 0x064BA |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| Full charge cycle state machine | Application-level (AO framework) | `doFullChargeCycle` (0x64BA) |
| Charge event state tracking | AO framework events | wChargeEventState1 |
| Motor speed reset on status change | Cross-module dependency | `setBatteryStatus` (0x195E) |
| Battery temperature reading | ADC + calibration — separate module | 0xA0CC–0xA0FC |
| Compass-based wire steering on status | AO framework post | `setBatteryStatus` |
| Current sense monitoring | Per-motor module (motor driver) | motor.c |
| EEPROM battery type persistence | Handled by eeprom/config module | config +0x66 |

## Usage Example

```c
#include "battery/battery.h"

/* At startup */
batt_Init();

/* Configure battery type from EEPROM config */
batt_ApplyType('B', 0);     /* Type B battery, normal thresholds */

/* Periodic monitoring (in main loop) */
uint16_t voltage = batt_ReadFiltered(system_tick);

/* Check battery levels */
if (batt_IsLowBattery(voltage)) {
    /* Critical low — emergency return */
    batt_SetStatus(BATT_STATUS_LOW_WARN);
}
if (batt_NeedsCharge(voltage)) {
    /* Time to head home for charging */
    batt_SetStatus(BATT_STATUS_NEAR_ZONE);
}

/* At charge station — check if charger is connected */
if (batt_IsCharging()) {
    batt_ChargeRelayOn();
    /* Wait for charge... */
    batt_ChargeRelayControl(voltage);  /* Hysteresis management */
}

/* Check if charge is complete */
if (voltage >= BATT_CHARGE_COMPLETE) {
    batt_ChargeRelayOff();
    batt_SetStatus(BATT_STATUS_JUST_CHARGED);
}

/* Rain detection */
if (batt_ReadRain()) {
    batt_SetStatus(BATT_STATUS_RAIN);
}

/* Diagnostics: full state dump */
batt_state_t state;
batt_GetState(&state);
/* state.raw_voltage, state.filtered_voltage, state.charging, ... */
```
