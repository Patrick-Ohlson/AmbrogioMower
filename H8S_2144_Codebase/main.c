/****************************************************************
KPIT Cummins Infosystems Ltd, Pune, India. - 10-Mar-2003.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************/

//-----------------Include Common Header Files--------------------------
#include "config.h"
#include "iodefine.h"
#include "micros.h"
#include "edk2215def.h"
#include "inlines.h"
#include <stdio.h>
#include "hardware.h"
#include "timer/timer.h"
#include "sci.h"

#include <stdint.h>
#include "lcd/lcd.h"
#include "misc/misc.h"
#include "i2c/i2c.h"
#include "rtc/rtc.h"
#include "motor/motor.h"
#include "version.h"

/* Utility helpers (delay_ms, print_hex*, FRT) */
#include "utils/utils.h"

/* Test modules */
#include "tests/test_sci.h"
#include "tests/test_statics.h"
#include "tests/test_i2c.h"
#include "tests/test_rtc.h"
#include "tests/test_memory.h"
#include "tests/test_misc.h"
#include "tests/test_motor.h"
#include "tests/test_drive.h"
#include "tests/test_lcd.h"
#include "tests/test_timer.h"
#include "tests/test_steering.h"
#include "tests/test_eeprom.h"
#include "tests/test_accel.h"
#include "keyboard/keyboard.h"
#include "tests/test_keyboard.h"
#include "battery/battery.h"
#include "tests/test_battery.h"
#include "wire/wire.h"
#include "tests/test_wire.h"
#include "bump/bump.h"
#include "tests/test_bump.h"
#include "active_object/ao.h"
#include "tests/test_ao.h"
#include "tests/demo.h"
#include "tests/demo_config.h"
#include "tests/remote.h"

//-----------------Function Declarations--------------------------------
int main(void);
unsigned char getch2(void);
void uart_init(void);
int flushread(void);
void Test_Menu(void);
void String_Output(void);
void Start_Monitor(void);

/* Trace32 monitor integration — symbols defined in monitor_asm.S / monitor.c */
#include "monitor.h"

//-----------------Define Globals----------------------------------------
char ucStr[] = "UART test\0";	/* Global Variables */
UINT8 flag;

/* ================================================================
 * Memory Test Variables
 *
 * These test that start.S correctly:
 *   1. Copies .data from ROM (_mdata) to RAM (_data.._edata)
 *   2. Zeroes .bss (_bss.._ebss)
 *
 * .data (initialized) — ROM load addr at _mdata, RAM at 0xFFE080+
 * .bss  (uninitialized) — RAM, must be zero on startup
 * .rodata (const) — stays in ROM (flash)
 * ================================================================ */

/* .data section: initialized globals, should be copied from ROM to RAM */
uint8_t  mem_test_u8   = 0xA5;          /* should read 0xA5 */
uint16_t mem_test_u16  = 0xBEEF;        /* should read 0xBEEF */
uint32_t mem_test_u32  = 0xDEADCAFE;    /* should read 0xDEADCAFE */
char     mem_test_str[] = "H8S/2144";   /* should read "H8S/2144" */

/* .bss section: uninitialized globals, should be zeroed by start.S */
uint8_t  bss_test_u8;                   /* should be 0x00 */
uint16_t bss_test_u16;                  /* should be 0x0000 */
uint32_t bss_test_u32;                  /* should be 0x00000000 */
char     bss_test_buf[8];               /* should be all zeroes */

//------------------ M A I N ---------------------------------------------
int main(void)						/* Initialise the serial port and display the menu */
{

#if !SIMULATION
lcd_Init();
//#endif
misc_InitPorts();
i2c_bus1_init();
i2c_bus2_init();
batt_Init();
wire_Init();
bump_Init();
timer_HW_Init();
//#if !SIMULATION
lcd_PrintCenter(0, ROBOT_NAME);
lcd_PrintCenter(1, "V%d-%02d R%d", VERSION_YEAR, VERSION_MONTH, REVISION);
InitSci2();    /* SCI1: interrupt TX + polled RX, ring buffers, BRR=8 */
flushread();
#endif
Test_Menu();
while (1)
{
	SendString((unsigned char *)"* Invalid Selection! Please try again! *");


//use getch to get a character from the serial port
unsigned char c = (unsigned char)getch2();



}



return 0;
}






int flushread(void)
{
	if (SCI1.SSR.BIT.RDRF!=0)
	{
		SCI1.SSR.BIT.RDRF=0;/* code */
	}
}


/***********************************************************************
 *  Start_Monitor — Enter Trace32 ROM monitor mode.
 *
 *  Sets persistent magic word (survives soft reset), sets runtime flag
 *  for ISR wrappers, drains TX, then executes TRAP #2 to enter the
 *  monitor with a proper exception frame (CCR + PC on stack).
 *
 *  Using TRAP instead of JMP means Trace32 sees a valid PC and full
 *  register context.  TRAP #2 → vector 10 → mon_trap entry.
 *  Does not return.
 ***********************************************************************/
void Start_Monitor(void)
{
	/* Set persistent magic so monitor restarts after soft reset */
	mon_magic = MON_MAGIC_VALUE;

	/* Send message BEFORE setting mon_active — PutChar guard
	 * blocks ALL serial output when mon_active is set. */
	SendString((unsigned char *)"\r\nEntering Trace32 Monitor...\r\n");

	/* Wait for TX ring buffer to drain completely.
	 * TXI ISR sends bytes; we poll TEND for the last byte. */
	while (SCI1.SSR.BIT.TEND == 0) {}

	/* NOW set runtime flag and disable firmware SCI interrupts.
	 * Order matters: mon_active=1 makes PutChar a no-op,
	 * then disabling TIE/RIE ensures no ISR touches SCI1. */
	mon_active = 1;
	SCI1.SCR.BIT.TIE = 0;
	SCI1.SCR.BIT.RIE = 0;

	/* Enter monitor via TRAP #2 — builds exception frame with valid
	 * PC + CCR on stack.  Vector 10 → mon_trap → _mon_breakp0.
	 * The monitor's prepcom reinits SCI for its own polled protocol.
	 *
	 * NOTE: Without Trace32 connected, the monitor blocks forever
	 * on mon_readbyte() waiting for a host command.  This is normal.
	 * Reset the board to return to the menu. */
	__asm__ volatile ("trapa #2");
}


unsigned char getch2(void)
{
	if(SCI1.SSR.BIT.ORER==1)SCI1.SSR.BIT.ORER=0;	// Clear the flag because the receiving error flag does not perform the reception operation.
	if(SCI1.SSR.BIT.FER==1)SCI1.SSR.BIT.FER=0;		// (Ignoring the error)
	if(SCI1.SSR.BIT.PER==1)SCI1.SSR.BIT.PER=0;

	while(SCI1.SSR.BIT.RDRF==0);	// Wait until you receive it

	SCI1.SSR.BIT.RDRF=0;

	return (unsigned char)SCI1.RDR;
}

// If you receive something, return 1
unsigned char keyhit(void)
{
	return (unsigned char)SCI1.SSR.BIT.RDRF;
}

/*
void puthex(unsigned char a)
{
	putch(tochar((a&0xF0)>>4));
	putch(tochar(a&0x0F));
}

unsigned char tochar(unsigned char a)
{
	if(a<10) a=a+0x30;
	else a=a+0x41-10;
	return a;
}

void puthexshort(unsigned short a)
{
	puthex((a>>8)&0xFF);
	puthex((a&0xFF));
}

unsigned short getshort(void)
{
	unsigned short data;
	data =((unsigned short)getch()<<8);
	data+=((unsigned short)getch()   );
	return data;
}

void putshort(unsigned short data)
{
	putch((data>>8)&0xFF);
	putch((data   )&0xFF);
}

void putdecimal(unsigned short data)
{
	putch(tochar(data/10000));
	data=data%10000;
	putch(tochar(data/1000));
	data=data%1000;
	putch(tochar(data/100));
	data=data%100;
	putch(tochar(data/10));
	data=data%10;
	putch(tochar(data));
}
*/



//UARTEND


#ifndef ROMSTART_DISABLED
//------------------ T E S T  M E N U -------------------------------------1


/*
 * demo_wait_for_key — Poll for keypress with auto-start timeout.
 *
 * Returns the key pressed, or 0 if the timeout expires (triggers demo).
 * Timeout is DEMO_AUTOSTART_SECONDS.  LCD shows countdown on row 1.
 * Any keypress during countdown cancels auto-start and returns the key.
 */
static INT8 demo_wait_for_key(void)
{
    uint32_t ticks_per_sec;
    uint32_t deadline;
    uint32_t now;
    uint8_t  last_shown = 0xFF;
    uint8_t  remaining;

    ticks_per_sec = 100;    /* SystemCounter increments ~100/sec (10ms tick) */
    deadline = GetSystemCounter() + (uint32_t)DEMO_AUTOSTART_SECONDS * ticks_per_sec;

    while (1) {
        now = GetSystemCounter();
        if (now >= deadline) {
            return 0;       /* Timeout — auto-start demo */
        }

        /* Show countdown on LCD row 1 */
        remaining = (uint8_t)((deadline - now) / ticks_per_sec);
        if (remaining != last_shown) {
            lcd_Print(1, "Demo in %us...", remaining);
            last_shown = remaining;
        }

        /* Blink LED to show we're alive */
        if ((now & 0x3F) == 0) {
            misc_BlinkLed();
        }

        /* Check for keypress (non-blocking) */
        if (keyhit()) {
            lcd_Print(1, "                ");
            return (INT8)GetChar();
        }
    }
}

void Test_Menu(void)				/* Displays the Test Menu and calls the respective function*/
{
   INT8  cSelection;
   UINT32 ulDelay;
   uint8_t first_run = 1;

	while(1)
	{

#if SIMULATION
	 /* Simulation: no LCD/UART, launch AUTOSTART_MODULE directly */
	 cSelection = AUTOSTART_MODULE;
#else
	 String_Output();

	 /* First iteration: auto-start countdown or straight to menu.
	  * Subsequent iterations: block on GetChar() normally. */
	 if (first_run) {
	     first_run = 0;
#if AUTOSTART_ENABLE
	     cSelection = demo_wait_for_key();
	     if (cSelection == 0) {
	         /* Timeout — auto-start */
	         SendString((unsigned char *)"\r\n\r\n>>> Auto-starting module '");
	         PutChar(AUTOSTART_MODULE);
	         SendString((unsigned char *)"' <<<\r\n");
	         cSelection = AUTOSTART_MODULE;
	     }
#else
	     /* No autostart: go straight to menu */
	     cSelection = GetChar();
#endif
	 } else {
	     cSelection = GetChar();
	 }
	 PutChar(cSelection);
#endif /* SIMULATION */

	 for (ulDelay=0; ulDelay<100000; ulDelay++);
	 switch (cSelection)
	 {
	   case '1': Drive_Motor_Test();
	             break;
	   case '2': Blade_Motor_Test();
	             break;
	   case '3': Timer_Test();
	             break;
       case '4': SCI_Test();
	             break;
	   case '5': LCD_Test();
	             break;
	   case '6': Memory_Test();
	             break;
	   case '7': Misc_Test();
	             break;
	   case '8': RTC_Test();
	             break;
	   case '9': I2C_Test();
	             break;
	   case 'A':
	   case 'a': Steering_Test();
	             break;
	   case 'B':
	   case 'b': Demo_Run();
	             break;
	   case 'C':
	   case 'c': EEPROM_Test();
	             break;
	   case 'D':
	   case 'd': Accel_Test();
	             break;
	   case 'E':
	   case 'e': Keyboard_Test();
	             break;
	   case 'F':
	   case 'f': Battery_Test();
	             break;
	   case 'G':
	   case 'g': Wire_Test();
	             break;
	   case 'H':
	   case 'h': Bump_Test();
	             break;
	   case 'I':
	   case 'i': AO_Test();
	             break;

	   case 'J':
	   case 'j': Start_Monitor();
	             break;  /* never reached */

	   case 'K':
	   case 'k': Remote_Run();
	             break;

	   default: SendString((unsigned char *)"\r\n");
	            SendString((unsigned char *)"\r\n");
	            SendString((unsigned char *)"* Invalid Selection! Please try again! *");
	            SendString((unsigned char *)"\r\n");
	            SendString((unsigned char *)"\r\n");
	 }

   }

}

/* Print an unsigned int as decimal (up to 65535) */
static void print_dec(unsigned int val)
{
    char buf[6];  /* max 5 digits + null */
    int i = 0;
    if (val == 0) { PutChar('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) { PutChar(buf[--i]); }
}

void String_Output(void)
{

    {
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      ***********************");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      ");
     SendString((unsigned char *)ROBOT_NAME);
     SendString((unsigned char *)" v");
     SendString((unsigned char *)VERSION);
     SendString((unsigned char *)" r");
     print_dec(REVISION);
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      ***********************");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"          Menu Selection     ");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      * ------------------- *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      *   1. Motor Test     *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      *   2. Blade Motor    *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      *   3. Timer Demo     *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      *   4. SCI  Demo      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   5. LCD  Test      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   6. Memory Test    *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   7. Misc Test      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   8. RTC  Test      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   9. I2C  Diag      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   A. Steering Test  *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   B. Demo Mode      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   C. EEPROM Test    *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   D. Accel Test     *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   E. Keyboard Test  *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   F. Battery Test   *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   G. Wire Test      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   H. Bump Test      *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   I. AO Framework   *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   J. Start Monitor  *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
	 SendString((unsigned char *)"      *   K. Remote Control *");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"      ***********************");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"\r\n");
     SendString((unsigned char *)"Enter Test Selection: ");
     }

}





#endif
