/*
 * eeprom.c - 24Cxx I2C EEPROM driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * 24Cxx addressing:
 *   Device address byte: 1010 A2 A1 A0 R/W
 *   For 24C16: A2-A0 come from address bits 10:8 (block select)
 *   Byte address: address bits 7:0 within each 256-byte block
 *
 * Write operations are limited to 16-byte page boundaries.
 * After each page write, the EEPROM needs ~5ms for internal write cycle.
 * ACK polling is used to detect write completion.
 */

#include "eeprom.h"
#include "../i2c/i2c.h"

/* EEPROM and RTC share I2C Bus 1 (firmware: ptrBladeJumpTable1) */
#define EE_I2C_BUS  I2C_BUS1

/* ---- Config version constants ---- */
/* Decompiled firmware (ROM 0x184BC): version=0x01, size=0x0B
 * Actual EEPROM on target mower:    version=0x02, size=0x09
 * Using values from actual hardware — different firmware version. */
#define CONFIG_VERSION_BYTE     0x02
#define CONFIG_SIZE_BYTE        0x09

/* ---- Internal helpers ---- */

/*
 * Compute the I2C device address byte for a given EEPROM address.
 * Bits 10:8 of the EEPROM address become A2:A0 in the device address.
 */
static uint8_t eeprom_device_addr(uint16_t addr, uint8_t rw)
{
    uint8_t block = (uint8_t)((addr >> 8) & 0x07);
    return EEPROM_ADDR_BASE | (block << 1) | (rw & 1);
}

/* ==================================================================
 * Public API
 * ================================================================== */

/*
 * eeprom_Poll - ACK polling for write completion
 *
 * Firmware: eeprom_Poll (0x151B2)
 * Repeatedly sends START + device address until ACK received.
 */
int eeprom_Poll(uint16_t addr)
{
    uint16_t retry;
    uint8_t dev_addr_w = eeprom_device_addr(addr, 0);

    for (retry = 0; retry < 1000; retry++) {
        i2c_Start(EE_I2C_BUS);
        if (i2c_WriteByte(EE_I2C_BUS, dev_addr_w)) {
            i2c_Stop(EE_I2C_BUS);
            return 0;   /* ACK received — ready */
        }
        i2c_Stop(EE_I2C_BUS);
    }

    return -1;  /* Timeout */
}

/*
 * eeprom_Read - Read N bytes, handling 256-byte block crossings
 *
 * Firmware: eeprom_Read (0x151F8)
 */
int eeprom_Read(uint16_t addr, uint8_t *buf, uint16_t size)
{
    uint16_t chunk;
    uint16_t remaining = size;
    uint16_t offset = 0;

    while (remaining > 0) {
        /* Calculate bytes remaining in current 256-byte block */
        chunk = EEPROM_BLOCK_SIZE - (addr & 0xFF);
        if (chunk > remaining)
            chunk = remaining;

        /* ACK poll — wait for any previous write to complete */
        if (eeprom_Poll(addr) != 0)
            return 0;

        /* Set address: START -> device_addr_W -> byte_addr */
        i2c_Start(EE_I2C_BUS);
        if (!i2c_WriteByte(EE_I2C_BUS, eeprom_device_addr(addr, 0))) {
            i2c_Stop(EE_I2C_BUS);
            return 0;
        }
        if (!i2c_WriteByte(EE_I2C_BUS, (uint8_t)(addr & 0xFF))) {
            i2c_Stop(EE_I2C_BUS);
            return 0;
        }

        /* Repeated start for read: rSTART -> device_addr_R */
        i2c_Start(EE_I2C_BUS);
        if (!i2c_WriteByte(EE_I2C_BUS, eeprom_device_addr(addr, 1))) {
            i2c_Stop(EE_I2C_BUS);
            return 0;
        }

        /* Read bytes with ACK, NACK on last */
        {
            uint16_t i;
            for (i = 0; i < chunk; i++) {
                buf[offset + i] = i2c_ReadByte(EE_I2C_BUS, i == chunk - 1);
            }
        }

        i2c_Stop(EE_I2C_BUS);

        offset += chunk;
        addr += chunk;
        remaining -= chunk;
    }

    return 1;   /* Success */
}

/*
 * eeprom_Write - Write N bytes, handling 16-byte page boundaries
 *
 * Firmware: eeprom_Write (0x152D8)
 */
int eeprom_Write(uint16_t addr, const uint8_t *buf, uint16_t size)
{
    uint16_t chunk;
    uint16_t remaining = size;
    uint16_t offset = 0;

    while (remaining > 0) {
        /* Calculate bytes remaining in current 16-byte page */
        chunk = EEPROM_PAGE_SIZE - (addr & (EEPROM_PAGE_SIZE - 1));
        if (chunk > remaining)
            chunk = remaining;

        /* ACK poll — wait for any previous write to complete */
        if (eeprom_Poll(addr) != 0)
            return 0;

        /* START -> device_addr_W -> byte_addr -> data */
        i2c_Start(EE_I2C_BUS);
        if (!i2c_WriteByte(EE_I2C_BUS, eeprom_device_addr(addr, 0))) {
            i2c_Stop(EE_I2C_BUS);
            return 0;
        }
        if (!i2c_WriteByte(EE_I2C_BUS, (uint8_t)(addr & 0xFF))) {
            i2c_Stop(EE_I2C_BUS);
            return 0;
        }

        /* Write page data */
        {
            uint16_t i;
            for (i = 0; i < chunk; i++) {
                if (!i2c_WriteByte(EE_I2C_BUS, buf[offset + i])) {
                    i2c_Stop(EE_I2C_BUS);
                    return 0;
                }
            }
        }

        i2c_Stop(EE_I2C_BUS);

        offset += chunk;
        addr += chunk;
        remaining -= chunk;
    }

    return 1;   /* Success */
}

/*
 * eeprom_ReadBlock - Read wrapper with error reporting
 *
 * Firmware: eeprom_ReadBlock / checkEEprom (0xB51A)
 * On failure, original firmware calls DebugPrint("EEPROM ERROR")
 */
int eeprom_ReadBlock(uint16_t addr, uint8_t *buf, uint16_t size)
{
    int result = eeprom_Read(addr, buf, size);
    /* TODO: Call DebugPrint("EEPROM ERROR") on failure if debug output available */
    return result;
}

/*
 * eeprom_WriteBlock - Write wrapper with error reporting
 *
 * Firmware: eeprom_WriteBlock (0xB548)
 */
int eeprom_WriteBlock(uint16_t addr, const uint8_t *buf, uint16_t size)
{
    int result = eeprom_Write(addr, buf, size);
    /* TODO: Call DebugPrint("EEPROM ERROR") on failure if debug output available */
    return result;
}

/*
 * eeprom_ConfigChecksum - Additive checksum with one's complement
 *
 * Firmware: config_Checksum (0x3EC8)
 *
 * uint8_t config_Checksum(uint8_t init, uint8_t *data, int size) {
 *     for (int i = 0; i < size; i++)
 *         init += data[i];
 *     return ~init;
 * }
 */
uint8_t eeprom_ConfigChecksum(uint8_t init, const uint8_t *data, uint16_t size)
{
    uint16_t i;

    for (i = 0; i < size; i++)
        init += data[i];

    return (uint8_t)~init;
}

/*
 * eeprom_ConfigValidate - Validate EEPROM config at given base address
 *
 * Firmware: config_Validate / fCheckEprom (0x4002)
 *
 * EEPROM layout at base_addr:
 *   +0x00: version byte   (must match CONFIG_VERSION_BYTE)
 *   +0x01: size byte      (must match CONFIG_SIZE_BYTE)
 *   +0x02: header_chk     (checksum of version + size)
 *   +0x03: config data    (0xB0 bytes)
 *   +0xB3: data_chk       (checksum of config data)
 */
int eeprom_ConfigValidate(uint16_t base_addr, uint8_t *buf)
{
    uint8_t header[3];
    uint8_t data_chk;
    uint8_t computed;

    /* Read 3-byte header */
    if (!eeprom_ReadBlock(base_addr, header, 3))
        return EEPROM_CFG_HDR_CHKSUM;

    /* Check header checksum: ~(0 + version + size) */
    computed = eeprom_ConfigChecksum(0, header, 2);
    if (computed != header[2])
        return EEPROM_CFG_HDR_CHKSUM;

    /* Check version byte */
    if (header[0] != CONFIG_VERSION_BYTE)
        return EEPROM_CFG_VERSION;

    /* Check size byte */
    if (header[1] != CONFIG_SIZE_BYTE)
        return EEPROM_CFG_SIZE;

    /* Read config data block */
    if (!eeprom_ReadBlock(base_addr + EEPROM_CONFIG_HEADER,
                          buf, EEPROM_CONFIG_DATA_SIZE))
        return EEPROM_CFG_DATA_CHKSUM;

    /* Read data checksum byte */
    if (!eeprom_ReadBlock(base_addr + EEPROM_CONFIG_HEADER + EEPROM_CONFIG_DATA_SIZE,
                          &data_chk, 1))
        return EEPROM_CFG_DATA_CHKSUM;

    /* Validate data checksum */
    computed = eeprom_ConfigChecksum(0, buf, EEPROM_CONFIG_DATA_SIZE);
    if (computed != data_chk)
        return EEPROM_CFG_DATA_CHKSUM;

    /* Check for factory default marker (all zeros or specific pattern) */
    /* Firmware returns 5 if config block indicates factory defaults needed */

    return EEPROM_CFG_OK;
}
