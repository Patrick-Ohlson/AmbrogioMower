/****************************************************************
KPIT Cummins Infosystems Ltd, Pune, India. - 10-Mar-2003.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************/

#include <stdint.h>
#include "config.h"
#include "iodefine.h"
#include "edk2215def.h"
#include "sci.h"
#include "micros.h"

/* Trace32 monitor flag — defined in monitor.S (.bss) */
extern volatile uint8_t mon_active;
#define SET_BIT_HIGH				1
#define SET_BIT_LOW					0
#define SET_BYTE_HIGH				0xFF
#define SET_BYTE_LOW				0x00

/* ================================================================
 * Phase 2a — Hybrid: Interrupt-driven TX + Polled RX
 *
 * TX (interrupt-driven, proven working):
 *   PutChar() -> push to TX ring buffer -> enable TIE
 *   TXI1 ISR -> pop from ring buffer -> TDR -> clear TDRE
 *   Buffer empty in ISR -> disable TIE
 *   Fixes polled PutChar hangs under motor EMI.
 *
 * RX (polled — interrupt RX failed, cause unknown):
 *   GetChar() polls RDRF, reads RDR, clears RDRF.
 *   Known working from Phase 1.
 *   RIE is NOT enabled, so RXI1/ERI1 ISRs do not fire.
 *   Error flags (ORER/FER/PER) are checked inline in polled reads.
 *
 * TODO: Debug why RXI1 ISR never fires despite TXI1 working.
 *       RX ring buffer code and ISR kept for future investigation.
 *
 * TX ring buffer is lock-free SPSC (single-producer, single-consumer):
 *   TX: main context pushes (head), TXI ISR pops (tail)
 * Separate read/write indices on 8-bit vars = naturally atomic on H8S.
 * ================================================================ */

#if SIMULATION
/* Simulation mode: all UART functions are no-ops to avoid SCI register access */
void InitSci2(void) {}
unsigned char GetChar(void) { return '\r'; }
void PutChar(unsigned char Data) { (void)Data; }
void PutStr(unsigned char *String) { (void)String; }
void SendByte(unsigned char b) { (void)b; }
void SendString(char *str) { (void)str; }
unsigned short GetByte(unsigned long timeout) { (void)timeout; return 0xFFFF; }
unsigned char RxByteWaiting(void) { return 0; }
void PurgeComms(unsigned long timeout) { (void)timeout; }
void SCI1_ClearErrors(void) {}
void sci1_tx_flush(void) {}
unsigned char sci1_rx_available(void) { return 0; }
void sci1_rx_flush(void) {}
void TXI1_SCI1(void) __attribute__((interrupt_handler));
void TXI1_SCI1(void) {}
void RXI1_SCI1(void) __attribute__((interrupt_handler));
void RXI1_SCI1(void) {}
void ERI1_SCI1(void) __attribute__((interrupt_handler));
void ERI1_SCI1(void) {}
#else /* !SIMULATION */

/* Ring buffer sizes — MUST be power of 2 for fast index masking */
#define SCI1_TX_BUF_SIZE  64
#define SCI1_RX_BUF_SIZE  64
#define TX_MASK           (SCI1_TX_BUF_SIZE - 1)
#define RX_MASK           (SCI1_RX_BUF_SIZE - 1)

#define RBUF_NEXT(idx, mask)  (((idx) + 1) & (mask))

/* TX ring buffer — main context writes head, TXI ISR reads tail */
static volatile uint8_t sci1_tx_data[SCI1_TX_BUF_SIZE];
static volatile uint8_t sci1_tx_head;   /* next write position   */
static volatile uint8_t sci1_tx_tail;   /* next read position    */

/* RX ring buffer — RXI ISR writes head, main context reads tail */
static volatile uint8_t sci1_rx_data[SCI1_RX_BUF_SIZE];
static volatile uint8_t sci1_rx_head;   /* next write position   */
static volatile uint8_t sci1_rx_tail;   /* next read position    */


/* ---- TX ring buffer helpers (inline) ---- */

static inline uint8_t tx_buf_full(void) {
	return (RBUF_NEXT(sci1_tx_head, TX_MASK) == sci1_tx_tail);
}

static inline uint8_t tx_buf_empty(void) {
	return (sci1_tx_head == sci1_tx_tail);
}

static inline void tx_buf_push(uint8_t byte) {
	sci1_tx_data[sci1_tx_head] = byte;
	sci1_tx_head = RBUF_NEXT(sci1_tx_head, TX_MASK);
}

static inline int16_t tx_buf_pop(void) {
	uint8_t byte;
	if (sci1_tx_head == sci1_tx_tail) return -1;
	byte = sci1_tx_data[sci1_tx_tail];
	sci1_tx_tail = RBUF_NEXT(sci1_tx_tail, TX_MASK);
	return (int16_t)byte;
}


/* ---- RX ring buffer helpers (inline) ---- */

static inline uint8_t rx_buf_full(void) {
	return (RBUF_NEXT(sci1_rx_head, RX_MASK) == sci1_rx_tail);
}

static inline uint8_t rx_buf_empty(void) {
	return (sci1_rx_head == sci1_rx_tail);
}

static inline void rx_buf_push(uint8_t byte) {
	sci1_rx_data[sci1_rx_head] = byte;
	sci1_rx_head = RBUF_NEXT(sci1_rx_head, RX_MASK);
}

static inline int16_t rx_buf_pop(void) {
	uint8_t byte;
	if (sci1_rx_head == sci1_rx_tail) return -1;
	byte = sci1_rx_data[sci1_rx_tail];
	sci1_rx_tail = RBUF_NEXT(sci1_rx_tail, RX_MASK);
	return (int16_t)byte;
}


/* ================================================================
 * InitSCI — KPIT EDK board SCI init (NOT compiled in ROMSTART builds)
 * Kept for reference / non-ROM builds only.
 * ================================================================ */
#ifndef ROMSTART
unsigned char InitSCI( struct SCI_Init_Params SCI_Config )
{
	UINT16  	Wait = INTERVAL,
					Error = SCI_OK;

	volatile unsigned char	SmrVal = SET_BYTE_LOW;

	/* Clear the SCI2 module stop control bit */
	Serial_Port_Enable_Bit = SET_BIT_LOW; //
	//MSTPCR.WORD = 0x6B3F;
	/* Clear TE & RE bits to 0 in SCR */
	Serial_Port_Control_Register.BYTE = SET_BYTE_LOW;

	/* Clear the error flags */
	Serial_Port_Status_Register.BYTE &= ~ (SSR_PER | SSR_FER | SSR_ORER);

	/* Set the Parity setting SMR bits */

	switch ( SCI_Config.Parity )
	{
	case P_EVEN :	SmrVal |= ( SMR_PAR_ENABLE | SMR_PAR_EVEN );
					break;
	case P_ODD :	SmrVal |= ( SMR_PAR_ENABLE | SMR_PAR_ODD );
					break;
	case P_NONE :	SmrVal |= SMR_PAR_DISABLE;
					break;
	default :		Error = SCI_ERR;
					break;
	}

  	/* Set the Stop bits setting SMR bits */
	switch ( SCI_Config.Stops )
	{
	case 1 :	SmrVal |= SMR_1_STOP;
				break;
	case 2 :	SmrVal |= SMR_2_STOP;
				break;
	default :	Error = SCI_ERR;
				break;
	}

  	/* Set the Data length setting SMR bits */
	switch ( SCI_Config.Length )
	{
	case 7 :	SmrVal |= SMR_7_CHAR;
				break;
	case 8 :	SmrVal |= SMR_8_CHAR;
				break;
	default :	Error = SCI_ERR;
				break;
	}

	/* Now set the SMR */
	Serial_Port_Mode_Register.BYTE = SmrVal;

  	/* Set the Baud Rate setting BRR bits */
  	Serial_Port_Baud_Rate_Register = SCI_Config.Baud;

  	/* Short delay to let things settle! */
	while(Wait--);

	/* Enable transmission and reception */
	Serial_Port_Control_Register.BYTE |= ( SCR_TE | SCR_RE );

	return (Error);
}

#endif


/***********************************************************************
 *
 *  GetChar — Receive a single character from SCI1.
 *
 *  Phase 2a: polled RX — waits for RDRF, reads RDR, clears RDRF.
 *  Handles error flags inline (ORER/FER/PER) since RIE is disabled
 *  and ERI1 ISR does not fire.
 *  Blocks until a valid byte is received.
 *
 ***********************************************************************/
unsigned char GetChar(void)
{
	volatile uint8_t ssr;
	uint8_t byte;

	while (1) {
		ssr = SCI1.SSR.BYTE;

		/* Clear any error flags — ORER blocks further reception.
		 * Must clear errors BEFORE checking RDRF so SCI can resume. */
		if (ssr & (SSR_ORER | SSR_FER | SSR_PER)) {
			SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
			continue;
		}

		/* Check if received data is ready */
		if (ssr & SSR_RDRF) {
			byte = SCI1.RDR;
			/* Clear RDRF: sequence is read SSR (done above), read RDR (done),
			 * then write 0 to RDRF bit. Belt-and-suspenders. */
			SCI1.SSR.BYTE &= (unsigned char)~SSR_RDRF;
			return byte;
		}
	}
}


/***********************************************************************
 *
 *  PutChar — Send a single character through SCI1.
 *
 *  Phase 2: pushes to TX ring buffer, enables TXI interrupt.
 *  TXI1 ISR drains the buffer byte-by-byte into SCI1.TDR.
 *  Non-blocking unless buffer is full (spins waiting for ISR to drain).
 *
 *  This is the core fix for motor EMI hangs: the main thread never
 *  busy-waits on TDRE/TEND — the ISR handles all SCI register access.
 *
 ***********************************************************************/
void PutChar(unsigned char Data)
{
	/* Block firmware TX while Trace32 monitor owns SCI1.
	 * Without this guard, timer ISRs calling PutChar re-enable TIE,
	 * and the TXI ISR injects firmware bytes into the monitor's
	 * polled protocol stream — the root cause of protocol corruption. */
	if (mon_active) return;

	/* Spin if TX buffer is full — TXI ISR will drain it.
	 * Safety: ensure TXI is enabled so buffer can actually drain.
	 * Protects against edge case where TIE was cleared between
	 * our push and the ISR check. */
	while (tx_buf_full()) {
		SCI1.SCR.BYTE |= SCR_TIE;
	}

	tx_buf_push(Data);

	/* Enable TX interrupt — if TDRE is already set, TXI fires immediately.
	 * Read-modify-write of SCR is safe: only TXI ISR touches TIE bit,
	 * and it only clears TIE (never sets other bits). */
	SCI1.SCR.BYTE |= SCR_TIE;
}


/***********************************************************************
 *
 *  PutStr — Send a null-terminated string through SCI1.
 *
 ***********************************************************************/
void PutStr(unsigned char* String)
{
	while ( *String != 0 )         /* While not end of string */
		PutChar( *String++);       /* output character and increment string */
}


/***********************************************************************
 *
 *  InitSci2 — Initialize SCI1: interrupt TX + polled RX.
 *
 *  Phase 2a: TX ring buffer drained by TXI1 ISR (proven working).
 *  RX is polled — GetChar/GetByte check RDRF directly.
 *  RIE is NOT enabled (RXI1/ERI1 ISRs do not fire).
 *  TIE is enabled on demand by PutChar when data is queued.
 *
 *  Baud rate: ~57600 bps (BRR=8, 8-N-1)
 *
 ***********************************************************************/
void InitSci2 (void)
{
	volatile unsigned long bit_delay;

	/* Initialize TX ring buffer */
	sci1_tx_head = 0;
	sci1_tx_tail = 0;

	/* RX ring buffer not used in polled mode, but reset anyway */
	sci1_rx_head = 0;
	sci1_rx_tail = 0;

	/* Enable SCI channel in module stop register */
	#if SCI_CH_NUM == 0
	MSTPCR.BIT._SCI0 = 0;
	#elif SCI_CH_NUM == 1
	MSTPCR.BIT._SCI1 = 0;
	#elif SCI_CH_NUM == 2
	MSTPCR.BIT._SCI2= 0;
	#endif

	/* Disable all SCI1 functions during configuration.
	 * Preserve RIE if Trace32 monitor is active — the monitor's
	 * prepgo sets RIE so Trace32 can break into running code via
	 * the RXI1 wrapper ISR. */
	if (mon_active)
		SCI_SCR.BYTE = SCR_RIE;
	else
		SCI_SCR.BYTE = 0;

	/* Async mode, 8-N-1, system clock /1 (n=0) */
	SCI_SMR.BYTE = 0;

	/* Smart Card mode disabled */
	SCI_SCMR.BYTE = 0;

	/* Baud rate: BRR=8 — empirical value that works at 57600 baud
	 * on this hardware.  Matches uart_init().  Crystal frequency
	 * may not be 14.7456 MHz (see uart_difference.md). */
	SCI_BRR = 8;

	/* Wait at least 1 bit time for baud rate generator to stabilize */
	for (bit_delay=0; bit_delay<100000; bit_delay++);

	/* Clear any pending error flags */
	SCI_SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);

	/* Enable TX and RX.  Preserve RIE for Trace32 monitor break-in. */
	if (mon_active)
		SCI_SCR.BYTE = SCR_TE | SCR_RE | SCR_RIE;
	else
		SCI_SCR.BYTE = SCR_TE | SCR_RE;
}


/***********************************************************************
 *
 *  SendByte — Send a single byte through SCI1.
 *
 *  Phase 2: delegates to PutChar (same ring buffer TX path).
 *
 ***********************************************************************/
void SendByte (unsigned char b)
{
	PutChar(b);
}


/***********************************************************************
 *
 *  SendString — Send a null-terminated string through SCI1.
 *
 ***********************************************************************/
void SendString (char* str)
{
	while (*str != 0) {
		PutChar((unsigned char)*str++);
	}
}


/***********************************************************************
 *
 *  GetByte — Receive a byte with timeout.
 *
 *  Returns the byte (0x0000-0x00FF) or 0xFFFF on timeout.
 *  Phase 2a: polled RX — waits for RDRF with countdown timeout.
 *  Clears error flags inline (ORER blocks further reception).
 *
 ***********************************************************************/
unsigned short GetByte (unsigned long timeout)
{
	volatile uint8_t ssr;
	uint8_t byte;

	while (1) {
		ssr = SCI1.SSR.BYTE;

		/* Clear error flags so SCI can resume */
		if (ssr & (SSR_ORER | SSR_FER | SSR_PER)) {
			SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
			continue;
		}

		/* Data ready? */
		if (ssr & SSR_RDRF) {
			byte = SCI1.RDR;
			SCI1.SSR.BYTE &= (unsigned char)~SSR_RDRF;
			return (unsigned short)byte;
		}

		/* Timeout countdown */
		if (timeout == 0) return 0xFFFF;
		timeout--;
	}
}


/***********************************************************************
 *
 *  RxByteWaiting — Check if RX data is available.
 *
 *  Phase 2a: checks RDRF hardware flag directly (polled RX).
 *  Returns non-zero if a received byte is waiting in RDR.
 *
 ***********************************************************************/
unsigned char RxByteWaiting (void)
{
	return (SCI1.SSR.BYTE & SSR_RDRF) ? 1 : 0;
}


/***********************************************************************
 *
 *  PurgeComms — Discard any pending RX data and errors.
 *
 *  Phase 2a: reads/discards from RDRF directly (polled RX).
 *  Also clears any pending error flags.
 *
 ***********************************************************************/
void PurgeComms (unsigned long timeout)
{
	volatile unsigned long delay;
	volatile uint8_t dummy;

	/* Clear any error flags first */
	SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);

	/* Read and discard any pending byte */
	if (SCI1.SSR.BYTE & SSR_RDRF) {
		dummy = SCI1.RDR;
		SCI1.SSR.BYTE &= (unsigned char)~SSR_RDRF;
		(void)dummy;
	}

	/* Wait for any trailing bytes to arrive */
	for (delay = 0; delay < timeout; delay++);

	/* Purge again to catch late arrivals */
	SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
	if (SCI1.SSR.BYTE & SSR_RDRF) {
		dummy = SCI1.RDR;
		SCI1.SSR.BYTE &= (unsigned char)~SSR_RDRF;
		(void)dummy;
	}
}


/* ================================================================
 * SCI1 Interrupt Service Routines
 * ================================================================ */


/***********************************************************************
 *
 *  TXI1_SCI1 — SCI1 Transmit Data Register Empty interrupt (vector 86)
 *
 *  Called when TDRE=1 and TIE=1.  Pops next byte from TX ring buffer
 *  and loads it into TDR.  If buffer is empty, disables TIE to stop
 *  further TXI interrupts until PutChar queues more data.
 *
 *  Matches original firmware's WriteByteUART (0x12ECC):
 *    pop byte -> TDR, clear TDRE; if empty -> SCR &= ~TIE
 *
 ***********************************************************************/
void TXI1_SCI1(void) __attribute__((interrupt_handler));
void TXI1_SCI1(void)
{
	int16_t byte;

	byte = tx_buf_pop();
	if (byte < 0) {
		/* TX ring buffer empty — disable TXI until PutChar re-enables */
		SCI1.SCR.BYTE &= (unsigned char)~SCR_TIE;
	} else {
		/* Load next byte into transmit data register */
		SCI1.TDR = (unsigned char)byte;
		/* Clear TDRE flag: read SSR (implicit in &=), write 0 to bit 7.
		 * This tells the SCI hardware that TDR now has new data. */
		SCI1.SSR.BYTE &= (unsigned char)~SSR_TDRE;
	}
}


/***********************************************************************
 *
 *  RXI1_SCI1 — SCI1 Receive Data Register Full interrupt (vector 85)
 *
 *  CURRENTLY INACTIVE — RIE is not enabled in Phase 2a (polled RX).
 *  Kept in code and vector table for future debugging.
 *
 *  When enabled (RIE=1): called when RDRF=1.  Reads received byte
 *  from RDR and pushes it into the RX ring buffer.
 *
 ***********************************************************************/
void RXI1_SCI1(void) __attribute__((interrupt_handler));
void RXI1_SCI1(void)
{
	volatile uint8_t ssr;
	uint8_t byte;

	/* H8S RDRF clearing sequence:
	 *   Step 1: read SSR with RDRF=1  (latches clear-enable)
	 *   Step 2: read RDR              (auto-clears RDRF)
	 * Order matters — reading RDR first does NOT clear RDRF.
	 * Belt-and-suspenders: also write 0 to RDRF bit. */
	ssr = SCI1.SSR.BYTE;              /* Step 1: read SSR (RDRF=1) */
	byte = SCI1.RDR;                  /* Step 2: read RDR -> clears RDRF */
	(void)ssr;

	/* Push to RX ring buffer if space available */
	if (!rx_buf_full()) {
		rx_buf_push(byte);
	}
	/* If buffer full, byte is silently dropped */
}


/***********************************************************************
 *
 *  ERI1_SCI1 — SCI1 Receive Error interrupt (vector 84)
 *
 *  CURRENTLY INACTIVE — RIE is not enabled in Phase 2a (polled RX).
 *  Errors are handled inline in GetChar/GetByte.
 *  Kept in code and vector table for future debugging.
 *
 *  When enabled (RIE=1): clears ORER/FER/PER error flags.
 *
 ***********************************************************************/
void ERI1_SCI1(void) __attribute__((interrupt_handler));
void ERI1_SCI1(void)
{
	/* Clear all error flags: ORER, FER, PER.
	 * Do NOT read RDR here — if RDRF is also set, reading RDR
	 * would steal the byte from the pending RXI interrupt.
	 * The H8S ERI vector (84) has higher priority than RXI (85),
	 * so ERI always fires first when both error and RDRF are pending.
	 * Let RXI handle RDR reading. */
	SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
}


/* ================================================================
 * Utility functions
 * ================================================================ */


/***********************************************************************
 *
 *  SCI1_ClearErrors — Manually clear SCI1 error flags.
 *
 *  Safe to call anytime.  With Phase 2 (RIE enabled), ERI1_SCI1
 *  handles this automatically, but this function remains available
 *  for explicit clearing before/after motor operations.
 *
 ***********************************************************************/
void SCI1_ClearErrors(void)
{
	if (SCI1.SSR.BYTE & (SSR_ORER | SSR_FER | SSR_PER)) {
		SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
	}
}


/***********************************************************************
 *
 *  sci1_tx_flush — Block until all TX data has been physically sent.
 *
 *  Waits for TX ring buffer to drain (TXI ISR), then waits for
 *  TEND=1 (shift register empty = last byte fully transmitted).
 *  Use before operations that need all serial output complete.
 *
 ***********************************************************************/
void sci1_tx_flush(void)
{
	volatile uint32_t timeout;

	/* Wait until TX ring buffer is empty.
	 * Timeout prevents infinite hang if EMI corrupted SCI
	 * and TXI can no longer drain the buffer. */
	timeout = 500000UL;
	while (!tx_buf_empty() && --timeout) {
		/* Safety: ensure TXI is running */
		SCI1.SCR.BYTE |= SCR_TIE;
	}

	/* Wait for last byte to physically complete transmission.
	 * TEND=1 means both TDR and shift register are empty.
	 * Timeout in case SCI hardware is in a bad state. */
	timeout = 200000UL;
	while (!(SCI1.SSR.BIT.TEND) && --timeout) {
		;
	}
}


/***********************************************************************
 *
 *  sci1_rx_available — Check if RX data is available.
 *
 *  Phase 2a: checks RDRF hardware flag directly (polled RX).
 *
 ***********************************************************************/
unsigned char sci1_rx_available(void)
{
	return (SCI1.SSR.BYTE & SSR_RDRF) ? 1 : 0;
}


/***********************************************************************
 *
 *  sci1_rx_flush — Discard all data in the RX ring buffer.
 *
 ***********************************************************************/
void sci1_rx_flush(void)
{
	sci1_rx_tail = sci1_rx_head;
}

#endif /* !SIMULATION */
