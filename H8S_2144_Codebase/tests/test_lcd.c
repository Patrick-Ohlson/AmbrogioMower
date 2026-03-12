/* test_lcd.c — LCD display test */
#include "test_lcd.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../lcd/lcd.h"
#include "../utils/utils.h"

void LCD_Test(void)
{
	SendString((unsigned char *)"\r\nLCD Test in progress:\r\n");

	lcd_Clear();
	delay_ms(500);

	lcd_Print(0, "LCD Test Line 1");
	lcd_Print(1, "HD44780 4-bit OK");
	SendString((unsigned char *)"Wrote test text to LCD rows 0+1\r\n");

	delay_ms(2000);

	/* Test cursor addressing */
	lcd_Clear();
	lcd_SetRow(0);
	lcd_WriteString("Row 0 direct");
	lcd_SetRow(1);
	lcd_WriteString("Row 1 direct");
	SendString((unsigned char *)"Wrote direct strings to LCD\r\n");

	delay_ms(2000);

	/* Test formatted print with counter */
	{
		uint8_t i;
		for (i = 0; i < 5; i++) {
			lcd_Print(0, "Counter: %d", i);
			lcd_Print(1, "LCD test %d/4", i);
			delay_ms(1000);
		}
	}

	lcd_Print(0, "LCD Test Done   ");
	lcd_Print(1, "                ");
	SendString((unsigned char *)"LCD test complete.\r\n");
}
