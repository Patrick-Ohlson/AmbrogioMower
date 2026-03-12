/*
 * eeprom.h - 24Cxx I2C EEPROM driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * Supports 24Cxx series EEPROM (likely 24C16 or larger) with:
 *   - 16-byte page write boundaries
 *   - 256-byte page crossing (address bits 8-10 in device address)
 *   - ACK polling for write completion
 *
 * Original firmware functions:
 *   eeprom_Poll       (0x151B2) -> eeprom_Poll()
 *   eeprom_Read       (0x151F8) -> eeprom_Read()
 *   eeprom_Write      (0x152D8) -> eeprom_Write()
 *   eeprom_ReadBlock  (0xB51A)  -> eeprom_ReadBlock()
 *   eeprom_WriteBlock (0xB548)  -> eeprom_WriteBlock()
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

/* ---- EEPROM I2C base address ---- */
#define EEPROM_ADDR_BASE    0xA0    /* 7-bit addr 0x50, shifted: 0xA0/0xA1 */

/* ---- EEPROM geometry ---- */
#define EEPROM_PAGE_SIZE    16      /* Write page size in bytes */
#define EEPROM_BLOCK_SIZE   256     /* Address block size (1 device addr bit) */

/* ---- Config system constants ---- */
#define EEPROM_CONFIG_PRIMARY   0x000   /* Primary config copy offset */
#define EEPROM_CONFIG_BACKUP    0x400   /* Backup config copy offset */
#define EEPROM_CONFIG_HEADER    3       /* Header size: version + size + checksum */
#define EEPROM_CONFIG_DATA_SIZE 0xB0    /* Config data block size (176 bytes) */

/* ---- Config validation error codes ---- */
#define EEPROM_CFG_OK           0   /* Config valid */
#define EEPROM_CFG_HDR_CHKSUM   1   /* Header checksum bad */
#define EEPROM_CFG_VERSION      2   /* Version byte mismatch */
#define EEPROM_CFG_SIZE         3   /* Size byte mismatch */
#define EEPROM_CFG_DATA_CHKSUM  4   /* Data checksum bad */
#define EEPROM_CFG_FACTORY      5   /* Factory default marker */

/* ---- Public API ---- */

/*
 * eeprom_Poll - ACK polling for write completion
 *
 * After a write, the EEPROM enters an internal write cycle (~5ms).
 * This function repeatedly sends the device address until the EEPROM
 * responds with ACK, indicating the write is complete.
 *
 * @param addr  EEPROM address (used to compute device address byte)
 * @return      0 on success, -1 on timeout
 *
 * Firmware: eeprom_Poll (0x151B2)
 */
int eeprom_Poll(uint16_t addr);

/*
 * eeprom_Read - Read N bytes from EEPROM
 *
 * Handles 256-byte page crossing by splitting reads across device
 * address boundaries. Protocol per page:
 *   START -> device_addr_W -> byte_addr -> rSTART -> device_addr_R -> read N -> STOP
 *
 * @param addr  EEPROM byte address (0x000 - 0x7FF+)
 * @param buf   Buffer to receive data
 * @param size  Number of bytes to read
 * @return      1 on success, 0 on failure
 *
 * Firmware: eeprom_Read (0x151F8)
 */
int eeprom_Read(uint16_t addr, uint8_t *buf, uint16_t size);

/*
 * eeprom_Write - Write N bytes to EEPROM
 *
 * Handles 16-byte page alignment: splits writes at page boundaries.
 * Performs ACK polling before each page write.
 * Protocol per page:
 *   Poll -> START -> device_addr_W -> byte_addr -> write up to 16 bytes -> STOP
 *
 * @param addr  EEPROM byte address
 * @param buf   Data to write
 * @param size  Number of bytes to write
 * @return      1 on success, 0 on failure
 *
 * Firmware: eeprom_Write (0x152D8)
 */
int eeprom_Write(uint16_t addr, const uint8_t *buf, uint16_t size);

/*
 * eeprom_ReadBlock - Read with error reporting
 *
 * Wrapper around eeprom_Read that provides error reporting
 * (original firmware prints "EEPROM ERROR" on failure).
 *
 * @param addr  EEPROM byte address
 * @param buf   Buffer to receive data
 * @param size  Number of bytes to read
 * @return      1 on success, 0 on failure
 *
 * Firmware: eeprom_ReadBlock (0xB51A)
 */
int eeprom_ReadBlock(uint16_t addr, uint8_t *buf, uint16_t size);

/*
 * eeprom_WriteBlock - Write with error reporting
 *
 * Wrapper around eeprom_Write with error reporting.
 *
 * @param addr  EEPROM byte address
 * @param buf   Data to write
 * @param size  Number of bytes to write
 * @return      1 on success, 0 on failure
 *
 * Firmware: eeprom_WriteBlock (0xB548)
 */
int eeprom_WriteBlock(uint16_t addr, const uint8_t *buf, uint16_t size);

/*
 * eeprom_ConfigChecksum - Compute additive checksum with one's complement
 *
 * @param init  Initial checksum value (for chaining)
 * @param data  Data to checksum
 * @param size  Number of bytes
 * @return      ~(init + sum(data))
 *
 * Firmware: config_Checksum (0x3EC8)
 */
uint8_t eeprom_ConfigChecksum(uint8_t init, const uint8_t *data, uint16_t size);

/*
 * eeprom_ConfigValidate - Validate config at a given EEPROM base address
 *
 * Reads the 3-byte header (version, size, header checksum) and the
 * config data block, then validates checksums, version, and size.
 *
 * @param base_addr  EEPROM base address (0x000 or 0x400)
 * @param buf        Buffer to receive config data (must be >= EEPROM_CONFIG_DATA_SIZE)
 * @return           EEPROM_CFG_OK (0) or error code 1-5
 *
 * Firmware: config_Validate (0x4002)
 */
int eeprom_ConfigValidate(uint16_t base_addr, uint8_t *buf);

#endif /* EEPROM_H */
