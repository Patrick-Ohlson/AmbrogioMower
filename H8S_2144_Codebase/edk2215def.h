/****************************************************************
KPIT Cummins Infosystems Ltd, Pune, India. - 10-Mar-2003.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************/
#ifdef ROMSTART

/* Timer setup for EDK2215 */

#define INTERRUPT_MODE              P_SYSTEM.SYSCR.BIT.INTM
#define TIMER_INT_PRIORITY          P_INT.IPRF.BIT.TPU1

#define TIMER_COMPARE_REGISTER   	P_TPU_1.TGRB_1      		/* Duty of LED on TIOCB1 controlled by TGRB_1 compare match */
#define TIMER_ENABLE_BIT	    	P_SYSTEM.MSTPCRA.BIT.MSTPA5   /* Enable Timer */
#define TIMER_COUNT_ENABLE_BIT	    P_TPU.TSTR.BIT.CST1		/* Stop/Start bit for TCNT_1 */

#define TIMER_TCR				    P_TPU_1.TCR_1.BYTE       	/* Timer Control Register */
#define TIMER_MODE_TMDR			    P_TPU_1.TMDR_1.BYTE		/* Timer Mode Register */
#define TIMER_OUTPUT_TIOR		    P_TPU_1.TIOR_1.BYTE		/* Timer I/O used to set o/p high at compare match */

#define TIMER_UPPER_LIMIT		    P_TPU_1.TGRA_1			/* Used to set the value of TGRA_1 for max count value */

/* Initial value setup for registers */

#define MODE_2                            0x2
#define ONE                               0x1
#define COMPARE_VALUE				0x7fff			/* Initial Compare match value */
#define SET_BIT_HIGH				1
#define SET_BIT_LOW			     	0
#define SET_BYTE_HIGH				0xFF
#define SET_BYTE_LOW				0x00
#define SETUP_TCR					0x23				/* TCNT cleared by OVF interrupt requests (OVI) & Internal clock: counts on �/8192 */
#define SET_MODE_PWM				0xC3				/* Sets PWM mode to 2 */
#define TIMER_OUTPUT_TIORL_VALUE	0x28				/* O/P for TIOCA2 set initially 0, high at compare match */
#define TIMER_UPPER_LIMIT_VALUE		0xffff			/* Max count Value for TGRA_1 */


/* Interrupt Setup */

#define TIMER_OVERFLOW_BIT		    P_TPU_1.TIER_1.BIT.TCIEV
#define TIMER_INT_COMPARE_BIT	    P_TPU_1.TIER_1.BIT.TGIEB

#define TIMER_STATUS		     	P_TPU_1.TSR_1.BYTE
#define TIMER_INT_FLAG		    	P_TPU_1.TSR_1.BIT.TGFB

#define CLEAR_INTERRUPT_FLAGS	    0xC0

#define COMPARE_DECREMENT	    	0x400


#define LED_PORT_DDR    	    	P_PORT.P1DDR.BYTE
#define LED_PORT_DR       	    	P_PORT.P1DR.BYTE

#define SET_LED_PORT_BITS  		    0x60            			/* Port1 (0110 0000) bits 5 & 6 */

#define LED_BIT          		    0x40       				/* Set bit 6 of Port 1 to an output */
#define INITIAL_LED_PORT_VALUE 	    0x00

//#ifdef ROMSTART

/* Serial Port setup */
#define SERIAL_PORT_BIT_TX		    P_PORT.PADR.BIT.PA1DR
#define TXD_BIT				        1

#define SERIAL_PORT_BIT_RX		    P_PORT.PADR.BIT.PA2DR
#define RXD_BIT			     	    1
#endif

