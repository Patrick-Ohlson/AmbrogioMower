/*
 * lcd.c - HD44780 LCD driver (4-bit parallel via Port 3)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_Display.md
 *
 * Port 3 register layout (from iodefine.h):
 *   P3.DDR     = 0xFFFFB4  (Data Direction Register)
 *   P3.DR.BYTE = 0xFFFFB6  (Data Register)
 *   P3.DR.BIT.B0 = RS, B1 = RW, B2 = E, B4-B7 = D4-D7
 *
 * DDR modes:
 *   Write: 0xF7 (1111_0111) - P3.0-2,4-7 output; P3.3 input
 *   Read:  0x07 (0000_0111) - P3.0-2 output; P3.3-7 input
 */

#include "lcd.h"
#include "../config.h"
#include "../iodefine.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if SIMULATION
/* Simulation mode: all LCD functions are no-ops to avoid P3 register access */
void lcd_Init(void) {}
void lcd_Clear(void) {}
void lcd_SetRow(uint8_t row) { (void)row; }
void lcd_WriteCommand(uint8_t cmd) { (void)cmd; }
void lcd_WriteString(const char *str) { (void)str; }
void lcd_Print(uint8_t row, const char *fmt, ...) { (void)row; (void)fmt; }
void lcd_PrintCenter(uint8_t row, const char *fmt, ...) { (void)row; (void)fmt; }
#else /* !SIMULATION */

/* ---- DDR configuration values ---- */
#define P3DDR_WRITE     0xF7    /* upper nibble + control as output, P3.3 input */
#define P3DDR_READ      0x07    /* only control bits as output (for BF read)    */

/* ---- Control bit masks ---- */
#define LCD_RS_BIT      0x01    /* P3.0 */
#define LCD_RW_BIT      0x02    /* P3.1 */
#define LCD_E_BIT       0x04    /* P3.2 */
#define LCD_DATA_MASK   0xF0    /* P3.4-P3.7 */

/* ---- DDRAM row address lookup ---- */
static const uint8_t row_addr[4] = {
    LCD_ROW0_ADDR,  /* 0x00 */
    LCD_ROW1_ADDR,  /* 0x40 */
    LCD_ROW2_ADDR,  /* 0x10 */
    LCD_ROW3_ADDR   /* 0x50 */
};

/* ==================================================================
 * Static helpers - GPIO bit manipulation
 * These map 1:1 to the original firmware's hardware layer functions
 * ================================================================== */

/*
 * Set RS (Register Select) pin - P3.0
 * 0 = command register, 1 = data register
 *
 * Firmware: lcd_SetRS (0x146E6)
 */
static void lcd_SetRS(uint8_t val)
{
    if (val)
        P3.DR.BYTE |= LCD_RS_BIT;
    else
        P3.DR.BYTE &= ~LCD_RS_BIT;
}

/*
 * Set RW (Read/Write) pin - P3.1
 * 0 = write, 1 = read
 *
 * Firmware: lcd_SetRW (0x146EE)
 */
static void lcd_SetRW(uint8_t val)
{
    if (val)
        P3.DR.BYTE |= LCD_RW_BIT;
    else
        P3.DR.BYTE &= ~LCD_RW_BIT;
}

/*
 * Set E (Enable) pin - P3.2
 * Data is latched on the falling edge (HIGH -> LOW)
 *
 * Firmware: lcd_SetE (0x14728)
 */
static void lcd_SetE(uint8_t val)
{
    if (val)
        P3.DR.BYTE |= LCD_E_BIT;
    else
        P3.DR.BYTE &= ~LCD_E_BIT;
}

/*
 * Write upper nibble to data pins P3[7:4]
 * Only the upper 4 bits of 'nibble' are used.
 *
 * Firmware: lcd_WriteNibble (0x14748)
 */
static void lcd_WriteNibble(uint8_t nibble)
{
    uint8_t dr = P3.DR.BYTE;
    dr = (dr & ~LCD_DATA_MASK) | (nibble & LCD_DATA_MASK);
    P3.DR.BYTE = dr;
}

/*
 * Read upper nibble from data pins P3[7:4]
 * Returns value in upper nibble (bits 7:4).
 *
 * Firmware: lcd_ReadNibble (0x14768)
 */
static uint8_t lcd_ReadNibble(void)
{
    return P3.DR.BYTE & LCD_DATA_MASK;
}

/*
 * Check HD44780 busy flag (BF = D7)
 * Returns 1 if LCD is ready (BF=0), 0 if still busy.
 *
 * Protocol:
 *   1. RS=0, RW=1
 *   2. Set P3DDR to read mode (upper nibble as input)
 *   3. Strobe E high, read D7 (BF), strobe E low
 *   4. Strobe E again for low nibble (discarded)
 *   5. Restore P3DDR to write mode
 *
 * Firmware: lcd_CheckBusyFlag (0x146F6)
 */
static uint8_t lcd_CheckBusy(void)
{
    uint8_t val;

    lcd_SetRS(0);
    lcd_SetRW(1);

    /* Switch upper nibble to input for reading */
    P3.DDR = P3DDR_READ;

    /* First strobe: read high nibble (D7=BF, D6-D4) */
    lcd_SetE(1);
    val = lcd_ReadNibble();
    lcd_SetE(0);

    /* Second strobe: read low nibble (discarded) */
    lcd_SetE(1);
    lcd_SetE(0);

    /* Restore DDR to write mode */
    P3.DDR = P3DDR_WRITE;

    /* BF is in bit 7: return 1 if ready (BF=0) */
    return (val & 0x80) == 0;
}

/*
 * Wait until the LCD is no longer busy.
 * Polls the busy flag in a tight loop.
 */
static void lcd_WaitReady(void)
{
    while (!lcd_CheckBusy())
        ;
}

/*
 * Send a single nibble with E strobe (write mode).
 * Assumes RS and RW are already set by caller.
 */
static void lcd_StrobeNibble(uint8_t nibble)
{
    lcd_WriteNibble(nibble);
    lcd_SetE(1);
    lcd_SetE(0);
}

/* ==================================================================
 * Public API
 * ================================================================== */

/*
 * lcd_WriteCommand - Send a command byte (RS=0)
 *
 * Waits for busy flag, then sends high nibble followed by low nibble,
 * each with an E strobe.
 *
 * Firmware: lcd_WriteCommand (0x14782)
 */
void lcd_WriteCommand(uint8_t cmd)
{
    lcd_WaitReady();
    lcd_SetRS(0);
    lcd_SetRW(0);
    lcd_StrobeNibble(cmd & 0xF0);           /* high nibble */
    lcd_StrobeNibble((cmd << 4) & 0xF0);    /* low nibble  */
}

/*
 * lcd_Clear - Clear display
 *
 * Firmware: lcd_Clear (0xB0FC) -> lcd_ClearDisplay (0x147B4)
 */
void lcd_Clear(void)
{
    lcd_WriteCommand(LCD_CMD_CLEAR);
}

/*
 * lcd_SetRow - Set DDRAM address for row
 *
 * @param row  Row number 0-3 (normally 0-1 for 16x2)
 *
 * Firmware: lcd_SetRowAddress (0x147BC)
 */
void lcd_SetRow(uint8_t row)
{
    if (row > 3)
        row = 0;
    lcd_WriteCommand(0x80 | row_addr[row]);
}

/*
 * lcd_WriteString - Write string to current DDRAM position (RS=1)
 *
 * Each character: wait busy, RS=1, RW=0, send high+low nibble with strobe.
 *
 * Firmware: lcd_WriteString (0x147F8)
 */
void lcd_WriteString(const char *str)
{
    while (*str) {
        uint8_t ch = (uint8_t)*str++;
        lcd_WaitReady();
        lcd_SetRS(1);
        lcd_SetRW(0);
        lcd_StrobeNibble(ch & 0xF0);           /* high nibble */
        lcd_StrobeNibble((ch << 4) & 0xF0);    /* low nibble  */
    }
}

/*
 * lcd_Init - Full LCD initialization
 *
 * Sets up Port 3 DDR for LCD output, then sends the HD44780
 * initialization sequence for 4-bit mode, 2-line, 5x8 font.
 *
 * Init sequence (from firmware at 0x14858):
 *   1. 0x20 - Function Set: DL=0 (4-bit mode)
 *   2. 0x28 - Function Set: DL=0, N=1 (2-line), F=0 (5x8)
 *   3. 0x0C - Display Control: D=1 (on), C=0, B=0
 *   4. 0x06 - Entry Mode: I/D=1 (increment), S=0
 *   5. 0x01 - Clear Display
 *
 * Firmware: lcd_FullInit (0xB0F4) -> lcd_InitSequence (0x14858)
 */
void lcd_Init(void)
{
    /* Configure Port 3 for LCD write mode */
    P3.DDR = P3DDR_WRITE;

    /* HD44780 initialization sequence */
    lcd_WriteCommand(LCD_CMD_FUNC_4BIT);    /* 0x20: 4-bit mode      */
    lcd_WriteCommand(LCD_CMD_FUNC_2LINE);   /* 0x28: 2-line, 5x8     */
    lcd_WriteCommand(LCD_CMD_DISPLAY_ON);   /* 0x0C: display on      */
    lcd_WriteCommand(LCD_CMD_ENTRY_MODE);   /* 0x06: auto-increment  */
    lcd_WriteCommand(LCD_CMD_CLEAR);        /* 0x01: clear + home    */
}

/*
 * lcd_Print - Format and write to a specific row
 *
 * Pipeline:
 *   1. vsprintf into 17-byte buffer (16 chars + null)
 *   2. Pad to exactly LCD_COLS characters with spaces
 *   3. Set DDRAM address for the requested row
 *   4. Write the padded string
 *
 * Always overwrites the entire row (no partial updates).
 *
 * Firmware: tm_DisplayWrite (0xB168) -> lcd_FormatAndWrite (0xB104)
 */
void lcd_Print(uint8_t row, const char *fmt, ...)
{
    char buf[LCD_COLS + 1];     /* 17 bytes: 16 chars + null */
    va_list ap;
    int len;
    int i;

    /* Format the string */
    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    /* Clamp length */
    if (len < 0)
        len = 0;
    if (len > LCD_COLS)
        len = LCD_COLS;

    /* Pad remaining columns with spaces */
    for (i = len; i < LCD_COLS; i++)
        buf[i] = ' ';
    buf[LCD_COLS] = '\0';

    /* Set row and write */
    lcd_SetRow(row);
    lcd_WriteString(buf);
}

/*
 * lcd_PrintCenter - Format and write a string centered on a row
 *
 * Formats via vsnprintf, then pads with leading and trailing spaces.
 */
void lcd_PrintCenter(uint8_t row, const char *fmt, ...)
{
    char tmp[LCD_COLS + 1];
    char buf[LCD_COLS + 1];
    va_list ap;
    int len, pad, i;

    /* Format the string */
    va_start(ap, fmt);
    len = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if (len < 0)
        len = 0;
    if (len > LCD_COLS)
        len = LCD_COLS;

    /* Leading spaces */
    pad = (LCD_COLS - len) / 2;
    for (i = 0; i < pad; i++)
        buf[i] = ' ';

    /* Copy text */
    for (i = 0; i < len; i++)
        buf[pad + i] = tmp[i];

    /* Trailing spaces */
    for (i = pad + len; i < LCD_COLS; i++)
        buf[i] = ' ';
    buf[LCD_COLS] = '\0';

    lcd_SetRow(row);
    lcd_WriteString(buf);
}

#endif /* !SIMULATION */
