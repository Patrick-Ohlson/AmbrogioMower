/*
 * lcd.h - HD44780 LCD driver (4-bit parallel via Port 3)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_Display.md
 *
 * Hardware: 16x2 HD44780-compatible character LCD
 * Interface: 4-bit parallel GPIO on Port 3
 *   P3.0 = RS   (Register Select: 0=command, 1=data)
 *   P3.1 = RW   (Read/Write: 0=write, 1=read)
 *   P3.2 = E    (Enable strobe: HIGH->LOW clocks data)
 *   P3.3 = unused (input)
 *   P3.4-P3.7 = D4-D7 (data nibble, bidirectional)
 *
 * Original firmware functions:
 *   lcd_FullInit      (0xB0F4)   -> lcd_Init()
 *   lcd_Clear         (0xB0FC)   -> lcd_Clear()
 *   lcd_WriteCommand  (0x14782)  -> lcd_WriteCommand()
 *   lcd_WriteString   (0x147F8)  -> lcd_WriteString()
 *   lcd_SetRowAddress (0x147BC)  -> lcd_SetRow()
 *   lcd_InitSequence  (0x14858)  -> lcd_Init() internal
 *   lcd_FormatAndWrite(0xB104)   -> lcd_Print()
 *   tm_DisplayWrite   (0xB168)   -> lcd_Print()
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

/* ---- Display geometry ---- */
#define LCD_COLS    16
#define LCD_ROWS    2

/* ---- HD44780 commands ---- */
#define LCD_CMD_CLEAR       0x01
#define LCD_CMD_HOME        0x02
#define LCD_CMD_ENTRY_MODE  0x06    /* I/D=1 (increment), S=0 (no shift) */
#define LCD_CMD_DISPLAY_ON  0x0C    /* D=1 (on), C=0 (no cursor), B=0 (no blink) */
#define LCD_CMD_FUNC_4BIT   0x20    /* DL=0 (4-bit mode) */
#define LCD_CMD_FUNC_2LINE  0x28    /* DL=0, N=1 (2-line), F=0 (5x8 font) */

/* ---- DDRAM row base addresses ---- */
#define LCD_ROW0_ADDR   0x00
#define LCD_ROW1_ADDR   0x40
#define LCD_ROW2_ADDR   0x10        /* for 20x4 displays */
#define LCD_ROW3_ADDR   0x50        /* for 20x4 displays */

/* ---- Public API ---- */

/*
 * lcd_Init - Full LCD initialization
 *
 * Configures Port 3 DDR for LCD output and sends the HD44780
 * initialization sequence (4-bit mode, 2-line, display on).
 * Must be called once at startup before any other lcd_* functions.
 *
 * Firmware equivalent: lcd_FullInit (0xB0F4) -> lcd_InitSequence (0x14858)
 */
void lcd_Init(void);

/*
 * lcd_Clear - Clear display and return cursor home
 *
 * Sends HD44780 clear command (0x01). Takes ~1.52ms to execute.
 *
 * Firmware equivalent: lcd_Clear (0xB0FC) -> lcd_ClearDisplay (0x147B4)
 */
void lcd_Clear(void);

/*
 * lcd_SetRow - Set DDRAM address to beginning of specified row
 *
 * @param row  Row number (0-3, normally 0-1 for 16x2)
 *
 * Firmware equivalent: lcd_SetRowAddress (0x147BC)
 */
void lcd_SetRow(uint8_t row);

/*
 * lcd_WriteCommand - Send a command byte to the LCD
 *
 * Waits for busy flag clear, then sends command with RS=0.
 *
 * @param cmd  HD44780 command byte
 *
 * Firmware equivalent: lcd_WriteCommand (0x14782)
 */
void lcd_WriteCommand(uint8_t cmd);

/*
 * lcd_WriteString - Write a null-terminated string to current DDRAM position
 *
 * Sends each character with RS=1 (data mode). Does NOT pad or truncate.
 * Caller should set row address first with lcd_SetRow().
 *
 * @param str  Null-terminated string to write
 *
 * Firmware equivalent: lcd_WriteString (0x147F8)
 */
void lcd_WriteString(const char *str);

/*
 * lcd_Print - Format and write a string to a specific LCD row
 *
 * Combines sprintf formatting, 16-char padding, row addressing and
 * string output in one call. Always writes exactly LCD_COLS characters
 * (pads short strings with spaces, truncates long strings).
 *
 * @param row  Row number (0-1)
 * @param fmt  printf-style format string
 * @param ...  Format arguments
 *
 * Firmware equivalent: tm_DisplayWrite (0xB168) -> lcd_FormatAndWrite (0xB104)
 */
void lcd_Print(uint8_t row, const char *fmt, ...);

/*
 * lcd_PrintCenter - Format and write a string centered on a specific LCD row
 *
 * Supports printf-style format strings. Pads with spaces on both sides
 * to center the text within LCD_COLS. Truncates if longer than LCD_COLS.
 *
 * @param row  Row number (0-1)
 * @param fmt  printf-style format string
 * @param ...  Format arguments
 */
void lcd_PrintCenter(uint8_t row, const char *fmt, ...);

#endif /* LCD_H */
