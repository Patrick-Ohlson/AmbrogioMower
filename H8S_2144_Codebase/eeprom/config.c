/*
 * config.c - L200 Mower EEPROM Configuration Manager
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md §4.4-4.8
 *
 * Provides factory defaults, EEPROM load/save with dual-copy redundancy,
 * and checksum validation.
 *
 * Original firmware functions:
 *   config_FactoryDefaults  (0x1844) -> config_LoadDefaults()
 *   config_InitFromEEPROM   (0x40AE) -> config_Load()
 *   config_SaveBothCopies   (0x4120) -> config_Save()
 *   config_SaveToEEPROM     (0x3F1E) -> (internal)
 */

#include "config.h"
#include "eeprom.h"
#include <string.h>     /* memcpy, memset */

/* ---- EEPROM header constants ---- */

/* Version and size bytes written to the 3-byte EEPROM header.
 * Decompiled firmware (ROM 0x184BC): version=0x01, size=0x0B
 * Actual EEPROM on target mower:    version=0x02, size=0x09
 * Using values from actual hardware — different firmware version. */
#define CONFIG_VERSION_BYTE     0x02
#define CONFIG_SIZE_BYTE        0x09

/* ====================================================================
 * Factory Defaults
 *
 * Matches config_FactoryDefaults (0x1844 / SetStartConfig):
 *   1. memset entire 0xB0-byte block to 0
 *   2. Set specific fields to non-zero defaults
 *
 * The firmware clears the block first via tm_ClearMemory(&g_eepromConfig, 0)
 * with size 0xB0 in r2, then writes individual fields by absolute address.
 *
 * Verified field offsets against Ghidra disassembly:
 *   g_eepromConfig._18_1_  = 0x7F  -> +0x12 charge_zone
 *   g_eepromConfig._19_1_  = 0x08  -> +0x13 timer_param1
 *   g_eepromConfig._20_1_  = 0x1E  -> +0x14 timer_param2
 *   g_eepromConfig._25_1_  = 0x16  -> +0x19 timer_param3
 *   g_eepromConfig._26_1_  = 0x1E  -> +0x1A timer_param4
 *   g_eepromConfig._100_1_ = 0x0D  -> +0x64 max_follow_wire
 *   g_eepromConfig._101_1_ = 0x1C  -> +0x65 noise_level
 *   g_eepromConfig._102_1_ = 0x42  -> +0x66 battery_type
 *   g_eepromConfig._171_1_ = 0x81  -> +0xAB config_bits
 *   g_eepromConfig._172_1_ = 0x03  -> +0xAC feature_flags
 *   g_eepromConfig._174_1_ = 0x60  -> +0xAE ext_config_flags2
 * ==================================================================== */

const mower_config_t CONFIG_FACTORY_DEFAULTS = {

    /* ---- Schedule & Area Configuration (+0x00 to +0x1F) ---- */

    /* +0x00 */ .schedule_base   = 0x0000,
    /* +0x02 */ .area1_pct       = 0,
    /* +0x03 */ .area0_flags     = 0,
    /* +0x04 */ ._reserved_04    = {0, 0},
    /* +0x06 */ .area2_pct       = 0,
    /* +0x07 */ .area1_flags     = 0,
    /* +0x08 */ ._reserved_08    = {0, 0},
    /* +0x0A */ .area3_pct       = 0,
    /* +0x0B */ .area2_flags     = 0,
    /* +0x0C */ .compass_cal     = 0x0000,
    /* +0x0E */ ._reserved_0E    = {0, 0, 0, 0},
    /* +0x12 */ .charge_zone     = 0x7F,    /* Charge zone boundary distance */
    /* +0x13 */ .timer_param1    = 0x08,    /* Timer/distance parameter 1 (8) */
    /* +0x14 */ .timer_param2    = 0x1E,    /* Timer/distance parameter 2 (30) */
    /* +0x15 */ ._reserved_15    = {0, 0, 0, 0},
    /* +0x19 */ .timer_param3    = 0x16,    /* Timer/distance parameter 3 (22) */
    /* +0x1A */ .timer_param4    = 0x1E,    /* Timer/distance parameter 4 (30) */
    /* +0x1B */ ._reserved_1B    = {0, 0, 0, 0, 0},

    /* ---- Extended Schedule Table (+0x20 to +0x4F) ---- */

    /* +0x20 */ .schedule_table  = {0},     /* All 40 bytes zeroed */
    /* +0x48 */ .schedule_date   = {0, 0},
    /* +0x4B */ .area_priority   = {0, 0, 0, 0},
    /* +0x4F */ ._reserved_4F    = 0,

    /* ---- Runtime Statistics (+0x50 to +0x63) ---- */

    /* +0x50 */ .total_work_time = 0,
    /* +0x54 */ .password         = 0,
    /* +0x56 */ .speed_param     = 0,
    /* +0x58 */ .blade_high_rpm  = 0,
    /* +0x5A */ .last_work_time  = 0,
    /* +0x5C */ .mow_time_accum  = 0,
    /* +0x5E */ .charge_time     = 0,
    /* +0x60 */ .charge_count    = 0,
    /* +0x62 */ .language         = 0,       /* Default language (English/Italian) */
    /* +0x63 */ .current_area    = 0,

    /* ---- Configuration Parameters (+0x64 to +0x66) ---- */

    /* +0x64 */ .max_follow_wire = 0x0D,    /* Max wire follow distance (13) */
    /* +0x65 */ .noise_level     = 0x1C,    /* Noise control threshold (28) */
    /* +0x66 */ .battery_type    = 0x42,    /* Battery type 'B' (standard) */

    /* ---- Gap (+0x67 to +0x76) ---- */

    /* +0x67 */ ._reserved_67    = {0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0},

    /* ---- Temperature Max Statistics (+0x77 to +0x7C) ---- */

    /* +0x77 */ .temp_max_left_motor   = 0,
    /* +0x78 */ .temp_max_blade_motor  = 0,
    /* +0x79 */ .temp_max_right_motor  = 0,
    /* +0x7A */ .temp_max_left_driver  = 0,
    /* +0x7B */ .temp_max_right_driver = 0,
    /* +0x7C */ .temp_max_blade_driver = 0,

    /* ---- Error Counters (+0x7D to +0x89) ---- */

    /* +0x7D */ .err_charge_init  = 0,
    /* +0x7E */ .err_charge_cycle = 0,
    /* +0x7F */ .err_clock        = 0,
    /* +0x80 */ .err_steering     = 0,
    /* +0x81 */ .err_low_battery  = 0,
    /* +0x82 */ .err_checksum     = 0,
    /* +0x83 */ .err_high_battery = 0,
    /* +0x84 */ .err_out_of_border = 0,
    /* +0x85 */ .err_blocked      = 0,
    /* +0x86 */ .err_right_motor  = 0,
    /* +0x87 */ .err_blade        = 0,
    /* +0x88 */ .err_left_motor   = 0,
    /* +0x89 */ .err_clear_count  = 0,

    /* ---- Event Timestamps (+0x8A to +0xAA) ---- */

    /* +0x8A */ .date_first_charge  = {0, 0},
    /* +0x8D */ .date_low_battery   = {0, 0},
    /* +0x90 */ .date_daily_report  = {0, 0},
    /* +0x93 */ .date_checksum_err  = {0, 0},
    /* +0x96 */ .date_high_battery  = {0, 0},
    /* +0x99 */ .date_out_of_border = {0, 0},
    /* +0x9C */ .date_blocked       = {0, 0},
    /* +0x9F */ .date_right_motor   = {0, 0},
    /* +0xA2 */ .date_blade         = {0, 0},
    /* +0xA5 */ .date_left_motor    = {0, 0},
    /* +0xA8 */ .date_clear_errors  = {0, 0},

    /* ---- Configuration Bit Flags (+0xAB to +0xAE) ---- */

    /* +0xAB */ .config_bits       = 0x81,  /* Rain sensor enabled + bit0 */
    /* +0xAC */ .feature_flags     = 0x03,  /* Zone tracking + bit0 */
    /* +0xAD */ .ext_feature_flags = 0x00,  /* No extended features */
    /* +0xAE */ .ext_config_flags2 = 0x60,  /* Border blade + closed area */

    /* +0xAF */ ._reserved_AF      = 0
};

/* ====================================================================
 * Internal: Write config + header to one EEPROM copy
 *
 * EEPROM layout at base_addr:
 *   +0x00: version byte   (CONFIG_VERSION_BYTE = 0x01)
 *   +0x01: size byte      (CONFIG_SIZE_BYTE = 0x0B)
 *   +0x02: header_chk     (checksum of version + size)
 *   +0x03: config data    (0xB0 bytes)
 *   +0xB3: data_chk       (checksum of config data)
 *
 * Firmware: config_SaveToEEPROM (0x3F1E / tm_sub_3F1E)
 * ==================================================================== */

static int config_write_copy(uint16_t base_addr, const mower_config_t *cfg)
{
    uint8_t header[3];
    uint8_t data_chk;

    /* Build 3-byte header */
    header[0] = CONFIG_VERSION_BYTE;
    header[1] = CONFIG_SIZE_BYTE;
    header[2] = eeprom_ConfigChecksum(0, header, 2);

    /* Compute data checksum over entire 0xB0-byte config block */
    data_chk = eeprom_ConfigChecksum(0, (const uint8_t *)cfg,
                                      EEPROM_CONFIG_DATA_SIZE);

    /* Write header (3 bytes at base) */
    if (!eeprom_WriteBlock(base_addr, header, 3))
        return 0;

    /* Write config data (0xB0 bytes at base+3) */
    if (!eeprom_WriteBlock(base_addr + EEPROM_CONFIG_HEADER,
                           (const uint8_t *)cfg, EEPROM_CONFIG_DATA_SIZE))
        return 0;

    /* Write data checksum (1 byte at base+0xB3) */
    if (!eeprom_WriteBlock(base_addr + EEPROM_CONFIG_HEADER + EEPROM_CONFIG_DATA_SIZE,
                           &data_chk, 1))
        return 0;

    return 1;
}

/* ====================================================================
 * Public API
 * ==================================================================== */

void config_LoadDefaults(mower_config_t *cfg)
{
    memcpy(cfg, &CONFIG_FACTORY_DEFAULTS, sizeof(mower_config_t));
}

int config_Load(mower_config_t *cfg)
{
    int result;

    /* Try primary config at EEPROM 0x000
     * Firmware tries backup first (0x400), but for our clean implementation
     * we try primary first — the important thing is both are checked.
     *
     * Firmware: config_InitFromEEPROM (0x40AE) */

    result = eeprom_ConfigValidate(EEPROM_CONFIG_PRIMARY, (uint8_t *)cfg);
    if (result == EEPROM_CFG_OK)
        return 0;   /* Loaded from primary */

    /* Primary failed — try backup at EEPROM 0x400 */
    result = eeprom_ConfigValidate(EEPROM_CONFIG_BACKUP, (uint8_t *)cfg);
    if (result == EEPROM_CFG_OK)
        return 1;   /* Loaded from backup */

    /* Both copies corrupt or marked as factory — load defaults */
    config_LoadDefaults(cfg);
    return -1;
}

int config_Save(const mower_config_t *cfg)
{
    /* Write backup copy first, then primary — same order as firmware.
     * Firmware: config_SaveBothCopies (0x4120) writes 0x400 then 0x000. */

    if (!config_write_copy(EEPROM_CONFIG_BACKUP, cfg))
        return 0;

    if (!config_write_copy(EEPROM_CONFIG_PRIMARY, cfg))
        return 0;

    return 1;   /* Both copies written successfully */
}
