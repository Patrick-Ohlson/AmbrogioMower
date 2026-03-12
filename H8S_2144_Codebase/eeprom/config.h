/*
 * config.h - L200 Mower EEPROM Configuration Structure
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md §4.8
 *
 * The configuration block is 0xB0 (176) bytes stored in EEPROM at offset +3
 * (after the 3-byte header: version, size, header_checksum).
 * In RAM it resides at 0xFFEB60 — 0xFFEC0F.
 *
 * Two copies are kept in EEPROM for redundancy:
 *   Primary:  EEPROM 0x000 (header) + 0x003 (data)
 *   Backup:   EEPROM 0x400 (header) + 0x403 (data)
 *
 * Original firmware labels:
 *   RAM base:  word_FFEB60
 *   Validate:  config_Validate / fCheckEprom (0x4002)
 *   Defaults:  fSetParameterToDefault (0x3F44)
 *   Apply:     config_ApplyBatteryType (0x3F94)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ====================================================================
 * Configuration Bit Flag Definitions
 *
 * These correspond to the 4 flag bytes at offsets +0xAB through +0xAE
 * of the config block. The firmware references these extensively
 * (31, 21, 21, and 17 cross-references respectively).
 * ==================================================================== */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * bConfigBits (offset +0xAB, RAM 0xFFEC0B)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Main configuration flags.  Factory default: 0x81
 * Firmware cross-references: 41 (31 reads, 10 writes)
 *
 *   Bit 7-6: Rain sensor mode (2-bit field)
 *            00 = Disabled
 *            01 = Pause mowing when rain detected
 *            10 = Enabled — full rain response (default)
 *            Firmware: readRainSensor (0x78A4) reads rain GPIO;
 *                      wireFollowStateMachine (0x5EBE) checks mode
 *                      to decide mow-abort vs continue.
 *   Bit 5-4: Compass mode (2-bit field)
 *            00 = Disabled
 *            10 = Enabled — mower uses compass for navigation
 *   Bit 2:   Clock display format
 *            0 = 24-hour format ("%02d:%02d")
 *            1 = 12-hour format ("%02d:%02d%c" with AM/PM suffix)
 *            Firmware: sub_E8FC (0xE8FC) — LCD time display
 *   Bit 0:   Low-battery voltage check enable
 *            0 = Skip low-battery check during wire follow
 *            1 = Enable low-battery check (default)
 *            Firmware: wireFollowStateMachine (0x5EBE) — gate on
 *                      readBatteryVoltageFiltered() comparison
 */
#define CFG_RAIN_MASK           0xC0    /* Bits 7:6 — rain sensor mode field */
#define CFG_RAIN_DISABLED       0x00    /* Rain sensor off */
#define CFG_RAIN_PAUSE          0x40    /* Pause on rain */
#define CFG_RAIN_ENABLED        0x80    /* Rain sensor active (default) */
#define CFG_RAIN_SHIFT          6       /* Bit position for rain field */

#define CFG_COMPASS_MASK        0x30    /* Bits 5:4 — compass mode field */
#define CFG_COMPASS_DISABLED    0x00    /* Compass off */
#define CFG_COMPASS_ENABLED     0x20    /* Compass active */
#define CFG_COMPASS_SHIFT       4       /* Bit position for compass field */

#define CFG_CLOCK_12H           0x04    /* Bit 2 — 12-hour clock display */
#define CFG_LOW_BATT_CHECK      0x01    /* Bit 0 — low-battery check enable */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * featureFlags (offset +0xAC, RAM 0xFFEC0C)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Feature enable flags.  Factory default: 0x03
 * Firmware cross-references: 26 (21 reads, 5 writes)
 *
 *   Bit 4:   Force charge mode — bypass normal charge entry logic
 *            Firmware: doFullChargeCycle (0x64BA) checks this to
 *                      determine whether to force immediate charge.
 *   Bit 3:   High-battery voltage threshold select
 *            0 = Standard threshold
 *            1 = Alternate (higher) threshold
 *            Firmware: config_ApplyBatteryType (0x3F94) — selects
 *                      between word_184A2/word_184A0 for type B, or
 *                      word_184A6/word_184A4 for type A.
 *   Bit 1:   Charge zone tracking — remember which zone the mower
 *            was in when returning to charge station.
 *   Bit 0:   Feature base flag (set in factory default)
 */
#define CFG_FEAT_FORCE_CHARGE   0x10    /* Bit 4 — force charge mode */
#define CFG_FEAT_HI_BATT_SEL    0x08    /* Bit 3 — high battery threshold */
#define CFG_FEAT_ZONE_TRACK     0x02    /* Bit 1 — charge zone tracking */
#define CFG_FEAT_BIT0           0x01    /* Bit 0 — set in default */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * extFeatureFlags (offset +0xAD, RAM 0xFFEC0D)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Extended feature flags.  Factory default: 0x00
 * Firmware cross-references: 26 (21 reads, 5 writes)
 *
 *   Bit 5:   Closed-area wire steering — when battery status changes,
 *            passes non-zero distance (400) to wire steering params
 *            for closed-area navigation.
 *            Firmware: setBatteryStatus (0x195E)
 *   Bit 4:   Force charge override (extended)
 *            Firmware: config_ApplyBatteryType (0x3F94)
 *   Bit 3:   High-battery threshold select (extended)
 *            Same as CFG_FEAT_HI_BATT_SEL but in this register.
 *            Firmware: config_ApplyBatteryType (0x3F94) — checked
 *                      after battery type 'A'/'B' table lookup.
 *   Bits 2:1: Wire follow speed selector (2-bit field)
 *            Value 0 = no speed override (uVar6 = 0)
 *            Value 1-3 = index into ROM speed table at 0x18C88
 *            Firmware: wireFollowStateMachine (0x5EBE) —
 *                      ((byte >> 1) & 3) - 1 selects timeout.
 *   Bit 0:   Safety handle — require handle press to start mowing
 *            Firmware: sub_4534 (0x4534) — time display/rain check
 */
#define CFG_EXT_CLOSED_WIRE     0x20    /* Bit 5 — closed area wire steering */
#define CFG_EXT_FORCE_CHARGE    0x10    /* Bit 4 — forced charge override */
#define CFG_EXT_HI_BATT_SEL     0x08    /* Bit 3 — alt high battery */
#define CFG_EXT_WIRE_SPEED_MASK 0x06    /* Bits 2:1 — wire follow speed */
#define CFG_EXT_WIRE_SPEED_SHIFT 1      /* Bit position for wire speed field */
#define CFG_EXT_SPIRAL          0x04    /* Bit 2 — spiral mow (alias) */
#define CFG_EXT_RAPID_RETURN    0x02    /* Bit 1 — rapid return (alias) */
#define CFG_EXT_SAFETY_HANDLE   0x01    /* Bit 0 — safety handle required */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * extConfigFlags2 (offset +0xAE, RAM 0xFFEC0E)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Extended configuration flags 2.  Factory default: 0x60
 * Firmware cross-references: 17 (10 reads, 7 writes)
 *
 *   Bit 6:   Border blade — blade runs along perimeter wire edge
 *   Bit 5:   Closed area work mode
 *            0 = Open/automatic — mower roams freely
 *            1 = Closed area — mower stays within a defined zone
 *            Firmware: idleFunc (0x47D8) — tested alongside bit 4
 *                      to determine navigation strategy.
 *   Bit 4:   Alarm/sound enable (tested in idleFunc and menu functions)
 *   Bits 3-0: Additional flags (exact purpose TBD from further RE)
 */
#define CFG_EXT2_BORDER_BLADE   0x40    /* Bit 6 — border blade enable */
#define CFG_EXT2_CLOSED_AREA    0x20    /* Bit 5 — closed area work mode */
#define CFG_EXT2_ALARM          0x10    /* Bit 4 — alarm/sound enable */

/* --- Area flags (used in area_flags[] entries) ---
 *
 * Per-area configuration byte (offsets +0x03, +0x07, +0x0B)
 *
 *   Bit 7: Mow direction (0 = default, 1 = reverse)
 *   Bit 6: Bounce on wire (0 = normal return, 1 = bounce at wire)
 */
#define AREA_FLAG_DIRECTION     0x80    /* Bit 7 — mow direction reverse */
#define AREA_FLAG_BOUNCE        0x40    /* Bit 6 — bounce on wire */

/* Battery type codes (offset +0x66) */
#define BATTERY_TYPE_A          0x41    /* 'A' — battery pack type A */
#define BATTERY_TYPE_B          0x42    /* 'B' — battery pack type B (default) */

/* ====================================================================
 * Config Bit Helpers — Generic
 *
 * Single-bit helpers use bitwise OR/AND/test.
 * Multi-bit field helpers use mask-and-shift for 2+ bit fields.
 *
 * Convention:
 *   cfg_SetBit(reg, mask)       — set bit(s) in a flag byte
 *   cfg_ClearBit(reg, mask)     — clear bit(s) in a flag byte
 *   cfg_TestBit(reg, mask)      — non-zero if any bit in mask is set
 *   cfg_GetField(reg, mask, sh) — extract multi-bit field value
 *   cfg_SetField(reg, mask, sh, val) — write multi-bit field value
 * ==================================================================== */

#define cfg_SetBit(reg, mask)              ((reg) |= (uint8_t)(mask))
#define cfg_ClearBit(reg, mask)            ((reg) &= (uint8_t)~(mask))
#define cfg_TestBit(reg, mask)             ((reg) & (uint8_t)(mask))
#define cfg_GetField(reg, mask, shift)     (((reg) & (uint8_t)(mask)) >> (shift))
#define cfg_SetField(reg, mask, shift, v)  \
    ((reg) = ((reg) & (uint8_t)~(mask)) | (uint8_t)(((v) << (shift)) & (mask)))

/* ====================================================================
 * Config Bit Helpers — bConfigBits (+0xAB)
 *
 * Usage:
 *   cfg_GetRainMode(&cfg)         -> CFG_RAIN_DISABLED/PAUSE/ENABLED
 *   cfg_SetRainMode(&cfg, mode)   <- CFG_RAIN_DISABLED/PAUSE/ENABLED
 *   cfg_GetCompassMode(&cfg)      -> CFG_COMPASS_DISABLED/ENABLED
 *   cfg_SetCompassMode(&cfg, m)   <- CFG_COMPASS_DISABLED/ENABLED
 *   cfg_Is12HourClock(&cfg)       -> 0 or non-zero
 *   cfg_SetClockFormat(&cfg, b)   <- 0=24h, non-zero=12h
 *   cfg_IsLowBattCheckEnabled(&cfg) -> 0 or non-zero
 * ==================================================================== */

/* Rain sensor mode (2-bit field, bits 7:6) */
#define cfg_GetRainMode(c)     ((c)->config_bits & CFG_RAIN_MASK)
#define cfg_SetRainMode(c, m)  cfg_SetField((c)->config_bits, CFG_RAIN_MASK, CFG_RAIN_SHIFT, \
                                            ((m) >> CFG_RAIN_SHIFT))

/* Compass mode (2-bit field, bits 5:4) */
#define cfg_GetCompassMode(c)     ((c)->config_bits & CFG_COMPASS_MASK)
#define cfg_SetCompassMode(c, m)  cfg_SetField((c)->config_bits, CFG_COMPASS_MASK, \
                                               CFG_COMPASS_SHIFT, ((m) >> CFG_COMPASS_SHIFT))

/* Clock format: 12h vs 24h (single bit 2) */
#define cfg_Is12HourClock(c)        cfg_TestBit((c)->config_bits, CFG_CLOCK_12H)
#define cfg_SetClockFormat(c, en)   do { if (en) cfg_SetBit((c)->config_bits, CFG_CLOCK_12H); \
                                         else cfg_ClearBit((c)->config_bits, CFG_CLOCK_12H); } while(0)

/* Low-battery voltage check (single bit 0) */
#define cfg_IsLowBattCheckEnabled(c)   cfg_TestBit((c)->config_bits, CFG_LOW_BATT_CHECK)
#define cfg_SetLowBattCheck(c, en)     do { if (en) cfg_SetBit((c)->config_bits, CFG_LOW_BATT_CHECK); \
                                            else cfg_ClearBit((c)->config_bits, CFG_LOW_BATT_CHECK); } while(0)

/* ====================================================================
 * Config Bit Helpers — featureFlags (+0xAC)
 *
 * Usage:
 *   cfg_IsForceCharge(&cfg)       -> 0 or non-zero
 *   cfg_SetForceCharge(&cfg, en)  <- 0=off, non-zero=on
 *   cfg_IsHiBattSel(&cfg)         -> 0 or non-zero
 *   cfg_IsZoneTrack(&cfg)         -> 0 or non-zero
 * ==================================================================== */

#define cfg_IsForceCharge(c)       cfg_TestBit((c)->feature_flags, CFG_FEAT_FORCE_CHARGE)
#define cfg_SetForceCharge(c, en)  do { if (en) cfg_SetBit((c)->feature_flags, CFG_FEAT_FORCE_CHARGE); \
                                        else cfg_ClearBit((c)->feature_flags, CFG_FEAT_FORCE_CHARGE); } while(0)

#define cfg_IsHiBattSel(c)         cfg_TestBit((c)->feature_flags, CFG_FEAT_HI_BATT_SEL)
#define cfg_SetHiBattSel(c, en)    do { if (en) cfg_SetBit((c)->feature_flags, CFG_FEAT_HI_BATT_SEL); \
                                        else cfg_ClearBit((c)->feature_flags, CFG_FEAT_HI_BATT_SEL); } while(0)

#define cfg_IsZoneTrack(c)         cfg_TestBit((c)->feature_flags, CFG_FEAT_ZONE_TRACK)
#define cfg_SetZoneTrack(c, en)    do { if (en) cfg_SetBit((c)->feature_flags, CFG_FEAT_ZONE_TRACK); \
                                        else cfg_ClearBit((c)->feature_flags, CFG_FEAT_ZONE_TRACK); } while(0)

/* ====================================================================
 * Config Bit Helpers — extFeatureFlags (+0xAD)
 *
 * Usage:
 *   cfg_IsClosedWire(&cfg)            -> 0 or non-zero
 *   cfg_IsExtForceCharge(&cfg)        -> 0 or non-zero
 *   cfg_IsExtHiBattSel(&cfg)          -> 0 or non-zero
 *   cfg_GetWireFollowSpeed(&cfg)      -> 0-3 (speed index)
 *   cfg_SetWireFollowSpeed(&cfg, v)   <- 0-3
 *   cfg_IsSafetyHandle(&cfg)          -> 0 or non-zero
 * ==================================================================== */

#define cfg_IsClosedWire(c)           cfg_TestBit((c)->ext_feature_flags, CFG_EXT_CLOSED_WIRE)
#define cfg_SetClosedWire(c, en)      do { if (en) cfg_SetBit((c)->ext_feature_flags, CFG_EXT_CLOSED_WIRE); \
                                           else cfg_ClearBit((c)->ext_feature_flags, CFG_EXT_CLOSED_WIRE); } while(0)

#define cfg_IsExtForceCharge(c)       cfg_TestBit((c)->ext_feature_flags, CFG_EXT_FORCE_CHARGE)
#define cfg_SetExtForceCharge(c, en)  do { if (en) cfg_SetBit((c)->ext_feature_flags, CFG_EXT_FORCE_CHARGE); \
                                           else cfg_ClearBit((c)->ext_feature_flags, CFG_EXT_FORCE_CHARGE); } while(0)

#define cfg_IsExtHiBattSel(c)         cfg_TestBit((c)->ext_feature_flags, CFG_EXT_HI_BATT_SEL)
#define cfg_SetExtHiBattSel(c, en)    do { if (en) cfg_SetBit((c)->ext_feature_flags, CFG_EXT_HI_BATT_SEL); \
                                           else cfg_ClearBit((c)->ext_feature_flags, CFG_EXT_HI_BATT_SEL); } while(0)

/* Wire follow speed (2-bit field, bits 2:1) — 0=default, 1-3=ROM speed table */
#define cfg_GetWireFollowSpeed(c)     cfg_GetField((c)->ext_feature_flags, \
                                                   CFG_EXT_WIRE_SPEED_MASK, CFG_EXT_WIRE_SPEED_SHIFT)
#define cfg_SetWireFollowSpeed(c, v)  cfg_SetField((c)->ext_feature_flags, \
                                                   CFG_EXT_WIRE_SPEED_MASK, CFG_EXT_WIRE_SPEED_SHIFT, (v))

#define cfg_IsSafetyHandle(c)         cfg_TestBit((c)->ext_feature_flags, CFG_EXT_SAFETY_HANDLE)
#define cfg_SetSafetyHandle(c, en)    do { if (en) cfg_SetBit((c)->ext_feature_flags, CFG_EXT_SAFETY_HANDLE); \
                                           else cfg_ClearBit((c)->ext_feature_flags, CFG_EXT_SAFETY_HANDLE); } while(0)

/* ====================================================================
 * Config Bit Helpers — extConfigFlags2 (+0xAE)
 *
 * Usage:
 *   cfg_IsBorderBlade(&cfg)       -> 0 or non-zero
 *   cfg_SetBorderBlade(&cfg, en)  <- 0=off, non-zero=on
 *   cfg_IsClosedArea(&cfg)        -> 0 or non-zero
 *   cfg_SetClosedArea(&cfg, en)   <- 0=off, non-zero=on
 *   cfg_IsAlarm(&cfg)             -> 0 or non-zero
 *   cfg_SetAlarm(&cfg, en)        <- 0=off, non-zero=on
 * ==================================================================== */

#define cfg_IsBorderBlade(c)       cfg_TestBit((c)->ext_config_flags2, CFG_EXT2_BORDER_BLADE)
#define cfg_SetBorderBlade(c, en)  do { if (en) cfg_SetBit((c)->ext_config_flags2, CFG_EXT2_BORDER_BLADE); \
                                        else cfg_ClearBit((c)->ext_config_flags2, CFG_EXT2_BORDER_BLADE); } while(0)

#define cfg_IsClosedArea(c)        cfg_TestBit((c)->ext_config_flags2, CFG_EXT2_CLOSED_AREA)
#define cfg_SetClosedArea(c, en)   do { if (en) cfg_SetBit((c)->ext_config_flags2, CFG_EXT2_CLOSED_AREA); \
                                        else cfg_ClearBit((c)->ext_config_flags2, CFG_EXT2_CLOSED_AREA); } while(0)

#define cfg_IsAlarm(c)             cfg_TestBit((c)->ext_config_flags2, CFG_EXT2_ALARM)
#define cfg_SetAlarm(c, en)        do { if (en) cfg_SetBit((c)->ext_config_flags2, CFG_EXT2_ALARM); \
                                        else cfg_ClearBit((c)->ext_config_flags2, CFG_EXT2_ALARM); } while(0)

/* ====================================================================
 * Config Bit Helpers — Area Flags (+0x03, +0x07, +0x0B)
 * ==================================================================== */

#define cfg_IsAreaReverse(flags)       cfg_TestBit((flags), AREA_FLAG_DIRECTION)
#define cfg_SetAreaReverse(flags, en)  do { if (en) cfg_SetBit((flags), AREA_FLAG_DIRECTION); \
                                            else cfg_ClearBit((flags), AREA_FLAG_DIRECTION); } while(0)

#define cfg_IsAreaBounce(flags)        cfg_TestBit((flags), AREA_FLAG_BOUNCE)
#define cfg_SetAreaBounce(flags, en)   do { if (en) cfg_SetBit((flags), AREA_FLAG_BOUNCE); \
                                            else cfg_ClearBit((flags), AREA_FLAG_BOUNCE); } while(0)

/* ====================================================================
 * Packed Date Format
 *
 * Event timestamps use a 3-byte packed date: 2 bytes (word) + 1 byte.
 * The exact encoding is not fully documented — likely BCD or packed
 * day/month/year fields matching the DS1307 RTC format.
 * ==================================================================== */

typedef struct __attribute__((packed)) {
    uint16_t date_word;             /* Packed date (high part) */
    uint8_t  date_byte;             /* Packed date (low part / extra) */
} packed_date_t;

/* ====================================================================
 * Main Configuration Structure — mower_config_t
 *
 * Total size: 0xB0 (176) bytes
 * RAM location: 0xFFEB60 — 0xFFEC0F
 * EEPROM location: base+3 (after 3-byte header)
 *
 * This struct maps every byte of the config block. Fields documented
 * in the firmware are given meaningful names; undocumented regions are
 * marked as reserved[] with their offset range noted.
 *
 * All multi-byte values are big-endian (Renesas H8S native byte order).
 *
 * Firmware reference: state_i2c.md §4.8
 * ==================================================================== */

/*
 * The structure maps every byte 0x00..0xAF exactly as stored in
 * EEPROM. Undocumented gaps are kept as reserved[] arrays so the struct
 * size stays at exactly 0xB0 bytes and every field sits at its correct
 * offset for direct memory overlay.
 */
typedef struct __attribute__((packed)) {

    /* ================================================================
     * SCHEDULE & AREA CONFIGURATION  (offsets +0x00 to +0x1F)
     * ================================================================ */

    /* +0x00 (word): Schedule table base address
     *   RAM: word_FFEB60. Set to 0x0000 by factory defaults.
     *   Used as base pointer for schedule time lookups. */
    uint16_t schedule_base;

    /* +0x02 (byte): Secondary area 1 mowing percentage
     *   Range 0–100 in steps of 10. Determines what fraction of mow
     *   time is spent in secondary area 1 vs the primary area.
     *   RAM: byte_FFEB62. Factory default: 0 (all time in primary). */
    uint8_t  area1_pct;

    /* +0x03 (byte): Area 0 behaviour flags
     *   bit7 = mow direction (0=normal, 1=reverse)
     *   bit6 = bounce on perimeter wire (0=normal return, 1=bounce)
     *   RAM: byte_FFEB63. Factory default: 0. */
    uint8_t  area0_flags;

    /* +0x04..+0x05: Undocumented (likely area 0 extended params)
     *   Not referenced by known firmware functions. */
    uint8_t  _reserved_04[2];

    /* +0x06 (byte): Secondary area 2 mowing percentage (0–100, step 10)
     *   RAM: byte_FFEB66. Factory default: 0. */
    uint8_t  area2_pct;

    /* +0x07 (byte): Area 1 behaviour flags (same layout as area0_flags)
     *   RAM: byte_FFEB67. Factory default: 0. */
    uint8_t  area1_flags;

    /* +0x08..+0x09: Undocumented (likely area 1 extended params) */
    uint8_t  _reserved_08[2];

    /* +0x0A (byte): Secondary area 3 mowing percentage (0–100, step 10)
     *   RAM: byte_FFEB6A. Factory default: 0. */
    uint8_t  area3_pct;

    /* +0x0B (byte): Area 2 behaviour flags (same layout as area0_flags)
     *   RAM: byte_FFEB6B. Factory default: 0. */
    uint8_t  area2_flags;

    /* +0x0C (word): Compass calibration data
     *   RAM: word_FFEB6C. Factory default: 0.
     *   Stored as a 16-bit value; exact encoding depends on compass module. */
    uint16_t compass_cal;

    /* +0x0E..+0x11: Undocumented (4 bytes — possibly more compass data
     *   or area 3 extended params) */
    uint8_t  _reserved_0E[4];

    /* +0x12 (byte): Charge zone boundary distance
     *   RAM: aChargeZoneData. Factory default: 0x7F (127).
     *   Defines the distance/boundary for the charging zone. */
    uint8_t  charge_zone;

    /* +0x13 (byte): Timer/distance parameter 1
     *   RAM: byte_FFEB73. Factory default: 0x08 (8).
     *   Used in mow-time or wire-follow distance calculations. */
    uint8_t  timer_param1;

    /* +0x14 (byte): Timer/distance parameter 2
     *   RAM: byte_FFEB74. Factory default: 0x1E (30).
     *   Used in mow-time or wire-follow distance calculations. */
    uint8_t  timer_param2;

    /* +0x15..+0x18: Undocumented (4 bytes — possibly additional
     *   timer/distance parameters for other areas) */
    uint8_t  _reserved_15[4];

    /* +0x19 (byte): Timer/distance parameter 3
     *   RAM: byte_FFEB79. Factory default: 0x16 (22). */
    uint8_t  timer_param3;

    /* +0x1A (byte): Timer/distance parameter 4
     *   RAM: byte_FFEB7A. Factory default: 0x1E (30). */
    uint8_t  timer_param4;

    /* +0x1B..+0x1F: Undocumented (5 bytes — end of schedule/area block) */
    uint8_t  _reserved_1B[5];

    /* ================================================================
     * EXTENDED SCHEDULE TABLE  (offsets +0x20 to +0x4F)
     *
     * The mower supports up to 5 daily time slots across 4 areas.
     * Each entry is 2 bytes (mow time in minutes).
     * Total: 5 slots × 4 areas × 2 bytes = 40 bytes.
     * ================================================================ */

    /* +0x20 (40 bytes): Schedule time table
     *   RAM: unk_FFEB80. Layout: schedule[slot][area] as uint16_t.
     *   slot 0–4 = day-of-week groups, area 0–3 = mowing areas.
     *   Each uint16_t is mow duration in minutes for that slot/area. */
    uint8_t  schedule_table[40];

    /* +0x48 (3 bytes): Schedule timestamp — when schedule was last set
     *   Packed date format (word + byte), likely matches RTC date format. */
    packed_date_t schedule_date;

    /* +0x4B (4 bytes): Schedule area priority — one byte per area
     *   Determines which area is mowed first when multiple are active.
     *   Higher value = higher priority. */
    uint8_t  area_priority[4];

    /* +0x4F (1 byte): Undocumented (gap between schedule and stats) */
    uint8_t  _reserved_4F;

    /* ================================================================
     * RUNTIME STATISTICS  (offsets +0x50 to +0x63)
     *
     * Lifetime and per-session counters. These accumulate during
     * normal operation and are saved to EEPROM periodically.
     * ================================================================ */

    /* +0x50 (dword): Total work time in minutes (lifetime)
     *   RAM: lStatusTotalWorktime (0xFFEBB0).
     *   Displayed in service menu as hours:minutes ("%lu:%02lu"). */
    uint32_t total_work_time;

    /* +0x54 (word): User password — 4-digit numeric code
     *   RAM: word_FFEBB4. Factory default: 0 (no password).
     *   Stored as binary uint16, displayed as 4 decimal digits. */
    uint16_t password;

    /* +0x56 (word): Speed parameter — wheel motor speed setting
     *   RAM: word_FFEBB6. Accessible from service menu.
     *   Factory default: 0. */
    uint16_t speed_param;

    /* +0x58 (word): Blade high RPM threshold
     *   RAM: word_FFEBB8. Accessible from service menu.
     *   Factory default: 0. */
    uint16_t blade_high_rpm;

    /* +0x5A (word): Last session work time in minutes
     *   RAM: lStatusLastWorkTime (0xFFEBBA).
     *   Reset at start of each mow session. */
    uint16_t last_work_time;

    /* +0x5C (word): Current mow time accumulator (minutes)
     *   RAM: word_FFEBBC. 13 firmware cross-references.
     *   Also backed up to DS1307 battery-backed SRAM for crash recovery. */
    uint16_t mow_time_accum;

    /* +0x5E (word): Charge time counter (minutes, lifetime)
     *   RAM: wStatusChargeTime (0xFFEBBE). */
    uint16_t charge_time;

    /* +0x60 (word): Number of charge sessions (lifetime)
     *   RAM: word_FFEBC0. */
    uint16_t charge_count;

    /* +0x62 (byte): Language index for UI strings (0–11)
     *   RAM: byte_FFEBC2. 15 firmware cross-references.
     *   Selects the localization table for LCD display messages.
     *   Factory default: 0 (first language, typically English/Italian). */
    uint8_t  language;

    /* +0x63 (byte): Current schedule area (1–4, 0xFF = none active)
     *   RAM: byte_FFEBC3.
     *   Tracks which area the mower is currently assigned to mow. */
    uint8_t  current_area;

    /* ================================================================
     * CONFIGURATION PARAMETERS  (offsets +0x64 to +0x66)
     * ================================================================ */

    /* +0x64 (byte): Maximum follow-wire distance/time
     *   RAM: byte_FFEBC4. Factory default: 0x0D (13).
     *   Limits how far the mower follows the perimeter wire before
     *   giving up and declaring an error. */
    uint8_t  max_follow_wire;

    /* +0x65 (byte): Noise control level
     *   RAM: byte_FFEBC5. Factory default: 0x1C (28).
     *   Threshold for wire signal noise rejection. Higher = more
     *   noise tolerance but less sensitivity. */
    uint8_t  noise_level;

    /* +0x66 (byte): Battery type identifier
     *   RAM: byte_FFEBC6. Factory default: 0x42 ('B').
     *   'A' (0x41) = battery type A — different voltage thresholds
     *   'B' (0x42) = battery type B — default voltage thresholds
     *   Used by config_ApplyBatteryType() to set charge voltage limits
     *   stored at RAM 0xFFEB58/0xFFEB5A (NOT in config struct). */
    uint8_t  battery_type;

    /* ================================================================
     * UNDOCUMENTED GAP  (offsets +0x67 to +0x76, 16 bytes)
     *
     * These bytes are part of the config block but have no documented
     * purpose in the known firmware. May include additional config
     * params, sensor calibration, or unused padding.
     * ================================================================ */
    uint8_t  _reserved_67[16];

    /* ================================================================
     * TEMPERATURE MAX STATISTICS  (offsets +0x77 to +0x7C)
     *
     * Peak temperature readings for each motor and driver board.
     * These are high-water marks that only increase (reset on
     * "clear statistics" operation). Raw ADC or scaled values.
     * ================================================================ */

    /* +0x77 (byte): Left wheel motor — peak temperature
     *   RAM: bStatusTemp2 (0xFFEBD7). */
    uint8_t  temp_max_left_motor;

    /* +0x78 (byte): Blade motor — peak temperature
     *   RAM: bStatusTemp (0xFFEBD8). */
    uint8_t  temp_max_blade_motor;

    /* +0x79 (byte): Right wheel motor — peak temperature
     *   RAM: bStatusTemperatureMax (0xFFEBD9). */
    uint8_t  temp_max_right_motor;

    /* +0x7A (byte): Left wheel driver board — peak temperature
     *   RAM: bMaxLeftDriverTemp (0xFFEBDA). */
    uint8_t  temp_max_left_driver;

    /* +0x7B (byte): Right wheel driver board — peak temperature
     *   RAM: bMaxRightDriverTemp (0xFFEBDB). */
    uint8_t  temp_max_right_driver;

    /* +0x7C (byte): Blade driver board — peak temperature
     *   RAM: bMaxBladeDriverTemp (0xFFEBDC). */
    uint8_t  temp_max_blade_driver;

    /* ================================================================
     * ERROR COUNTERS  (offsets +0x7D to +0x89, 13 bytes)
     *
     * Each counter is 1 byte, saturating at 0xFF (never wraps).
     * Paired with a 3-byte timestamp in the Event Timestamps section.
     *
     * NOTE: Some Ghidra/IDA label names are shifted by one position
     * relative to their actual function (e.g., bStatusLowBattery
     * actually holds the high-battery counter). The comments here
     * reflect the ACTUAL purpose, not the possibly-misleading label.
     * ================================================================ */

    /* +0x7D (byte): Failed charge count — initial charge test failure
     *   RAM: bStatus1 (0xFFEBDD). */
    uint8_t  err_charge_init;

    /* +0x7E (byte): Failed charge count — during charge cycle
     *   RAM: bStatusFailedCharge (0xFFEBDE). */
    uint8_t  err_charge_cycle;

    /* +0x7F (byte): Clock (RTC) error count
     *   RAM: bStatusClockError (0xFFEBDF). */
    uint8_t  err_clock;

    /* +0x80 (byte): Forward steering error count
     *   RAM: bStatus4 (0xFFEBE0). */
    uint8_t  err_steering;

    /* +0x81 (byte): Low battery event count
     *   RAM: bStatus5 (0xFFEBE1). */
    uint8_t  err_low_battery;

    /* +0x82 (byte): Checksum error count (EEPROM corruption detected)
     *   RAM: bStatusBlackout (0xFFEBE2). */
    uint8_t  err_checksum;

    /* +0x83 (byte): High battery event count (overvoltage)
     *   RAM: bStatusLowBattery (0xFFEBE3).
     *   Note: Ghidra label is misleading — this is the HIGH battery counter. */
    uint8_t  err_high_battery;

    /* +0x84 (byte): Out of border count (wire boundary violation)
     *   RAM: bStatusHighBattery (0xFFEBE4).
     *   Note: Ghidra label is misleading — this is the out-of-border counter. */
    uint8_t  err_out_of_border;

    /* +0x85 (byte): Blocked count (mower stuck / obstacle)
     *   RAM: bStatusOutOfBorder (0xFFEBE5).
     *   Note: Ghidra label is misleading — this is the blocked counter. */
    uint8_t  err_blocked;

    /* +0x86 (byte): Right wheel motor error count
     *   RAM: bStatusBlocked (0xFFEBE6). */
    uint8_t  err_right_motor;

    /* +0x87 (byte): Blade motor error count
     *   RAM: bStatusLeftMotor (0xFFEBE7).
     *   Note: Ghidra label is misleading — this is the blade error counter. */
    uint8_t  err_blade;

    /* +0x88 (byte): Left wheel motor error count
     *   RAM: bStatusRightMotor (0xFFEBE8).
     *   Note: Ghidra label is misleading — this is the left motor counter. */
    uint8_t  err_left_motor;

    /* +0x89 (byte): Clear-errors timestamp counter
     *   RAM: bStatusBlade (0xFFEBE9).
     *   Incremented each time the "clear errors" service function is used. */
    uint8_t  err_clear_count;

    /* ================================================================
     * EVENT TIMESTAMPS  (offsets +0x8A to +0xAA, 33 bytes)
     *
     * Each timestamp is a 3-byte packed date (packed_date_t).
     * These record when each error type last occurred.
     * Paired with the error counters above.
     * ================================================================ */

    /* +0x8A (3B): First charge date — set once at initial setup,
     *   never overwritten. RAM: lStatusFirstCharge (0xFFEBEA). */
    packed_date_t date_first_charge;

    /* +0x8D (3B): Last low battery event date.
     *   Paired with err_low_battery (+0x81). */
    packed_date_t date_low_battery;

    /* +0x90 (3B): Daily report / SMS notification date.
     *   Used by the GSM/reporting subsystem (if equipped). */
    packed_date_t date_daily_report;

    /* +0x93 (3B): Last checksum error date.
     *   Paired with err_checksum (+0x82). */
    packed_date_t date_checksum_err;

    /* +0x96 (3B): Last high battery event date.
     *   Paired with err_high_battery (+0x83). */
    packed_date_t date_high_battery;

    /* +0x99 (3B): Last out-of-border event date.
     *   Paired with err_out_of_border (+0x84). */
    packed_date_t date_out_of_border;

    /* +0x9C (3B): Last blocked event date.
     *   Paired with err_blocked (+0x85). */
    packed_date_t date_blocked;

    /* +0x9F (3B): Last right motor error date.
     *   Paired with err_right_motor (+0x86). */
    packed_date_t date_right_motor;

    /* +0xA2 (3B): Last blade motor error date.
     *   Paired with err_blade (+0x87). */
    packed_date_t date_blade;

    /* +0xA5 (3B): Last left motor error date.
     *   Paired with err_left_motor (+0x88). */
    packed_date_t date_left_motor;

    /* +0xA8 (3B): Last "clear errors" date.
     *   Paired with err_clear_count (+0x89). */
    packed_date_t date_clear_errors;

    /* ================================================================
     * CONFIGURATION BIT FLAGS  (offsets +0xAB to +0xAE)
     *
     * Four flag bytes controlling major mower behaviour. These are
     * among the most heavily referenced values in the firmware
     * (41 + 26 + 26 + 17 = 110 total cross-references).
     *
     * See CFG_* / CFG_FEAT_* / CFG_EXT_* / CFG_EXT2_* macros for
     * individual bit definitions.
     *
     * Use cfg_Get/cfg_Set/cfg_Is helper macros for safe access:
     *   cfg_GetRainMode(c), cfg_SetRainMode(c, CFG_RAIN_ENABLED)
     *   cfg_IsForceCharge(c), cfg_SetForceCharge(c, 1)
     *   cfg_IsBorderBlade(c), cfg_SetBorderBlade(c, 0)
     *   etc. -- see full list above the struct definition.
     * ================================================================ */

    /* +0xAB (byte): Main config flags — 41 firmware cross-references
     *   RAM: bConfigBits (0xFFEC0B). Factory default: 0x81.
     *   [7:6] Rain mode  [5:4] Compass  [2] Clock 12h  [0] LowBatt check
     *   Helpers: cfg_GetRainMode, cfg_GetCompassMode, cfg_Is12HourClock,
     *            cfg_IsLowBattCheckEnabled */
    uint8_t  config_bits;

    /* +0xAC (byte): Feature enable flags — 26 firmware cross-references
     *   RAM: byte_FFEC0C. Factory default: 0x03.
     *   [4] Force charge  [3] HiBatt select  [1] Zone track  [0] base
     *   Helpers: cfg_IsForceCharge, cfg_IsHiBattSel, cfg_IsZoneTrack */
    uint8_t  feature_flags;

    /* +0xAD (byte): Extended feature flags — 26 firmware cross-references
     *   RAM: byte_FFEC0D. Factory default: 0x00.
     *   [5] Closed wire  [4] Force charge  [3] HiBatt  [2:1] Wire speed  [0] Safety
     *   Helpers: cfg_IsClosedWire, cfg_GetWireFollowSpeed, cfg_IsSafetyHandle */
    uint8_t  ext_feature_flags;

    /* +0xAE (byte): Extended config flags 2 — 17 firmware cross-references
     *   RAM: bConfigBits_0 (0xFFEC0E). Factory default: 0x60.
     *   [6] Border blade  [5] Closed area  [4] Alarm
     *   Helpers: cfg_IsBorderBlade, cfg_IsClosedArea, cfg_IsAlarm */
    uint8_t  ext_config_flags2;

    /* +0xAF (1 byte): Undocumented trailing byte
     *   Completes the 0xB0-byte block. */
    uint8_t  _reserved_AF;

} mower_config_t;

/* Compile-time size check: config block must be exactly 0xB0 bytes */
typedef char _config_size_check[(sizeof(mower_config_t) == 0xB0) ? 1 : -1];

/* ====================================================================
 * Factory Default Instance
 *
 * Provides a const default configuration matching the original firmware's
 * fSetParameterToDefault (0x3F44). Use config_LoadDefaults() to copy
 * this into a working config instance.
 *
 * Firmware: fSetParameterToDefault (0x3F44), byte_184BC/byte_184BD
 * ==================================================================== */

extern const mower_config_t CONFIG_FACTORY_DEFAULTS;

/* ====================================================================
 * Public API
 * ==================================================================== */

/*
 * config_LoadDefaults - Copy factory defaults into a config struct
 *
 * @param cfg  Pointer to config struct to initialize
 *
 * Firmware: fSetParameterToDefault (0x3F44)
 */
void config_LoadDefaults(mower_config_t *cfg);

/*
 * config_Load - Load and validate config from EEPROM
 *
 * Tries primary copy first (EEPROM 0x000), then backup (0x400).
 * If both fail, loads factory defaults.
 *
 * @param cfg  Pointer to config struct to populate
 * @return     0 = loaded from primary, 1 = loaded from backup,
 *             -1 = both corrupt (factory defaults loaded)
 *
 * Firmware: fCheckEprom (0x4002) — called during startup
 */
int config_Load(mower_config_t *cfg);

/*
 * config_Save - Write config to both EEPROM copies with checksums
 *
 * Writes the 3-byte header (version, size, header_checksum) followed
 * by the 0xB0 config data and data_checksum to both primary and
 * backup EEPROM locations.
 *
 * @param cfg  Pointer to config struct to save
 * @return     1 = success, 0 = EEPROM write failure
 *
 * Firmware: config_Write / fSaveConfig
 */
int config_Save(const mower_config_t *cfg);

#endif /* CONFIG_H */
