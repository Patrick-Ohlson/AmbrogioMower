/***********************************************************************
 *
 *  monitor.h — Trace32 ROM Monitor for H8S/2144 (C implementation)
 *
 *  Shared constants, register definitions, stack frame layout,
 *  and function prototypes for monitor.c and monitor_asm.S.
 *
 *  Replaces the monolithic monitor.S with a three-layer split:
 *    monitor_asm.S — assembly stubs (register save/restore, RTE, ISR)
 *    monitor.c     — protocol handler (all commands in C)
 *    monitor.h     — this file (shared definitions)
 *
 ***********************************************************************/

#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

/* ================================================================
 * Constants
 * ================================================================ */

/* Persistent magic word for soft-reset re-entry.
 * 0x4D4F4E21 = "MON!" in ASCII.
 * Set by Start_Monitor(), checked by start.S on reset. */
#define MON_MAGIC_VALUE  0x4D4F4E21

/* ================================================================
 * SCI1 Register Definitions (direct polling)
 *
 * The monitor uses polled SCI1 I/O, bypassing the firmware's
 * interrupt-driven ring buffer.  These addresses match
 * mon_scibase (0xFFFF88) in the config table.
 * ================================================================ */

#define MON_SCI_BASE     0xFFFF88
#define MON_SCI_SMR      (*(volatile uint8_t  *)(MON_SCI_BASE + 0))
#define MON_SCI_BRR      (*(volatile uint8_t  *)(MON_SCI_BASE + 1))
#define MON_SCI_SCR      (*(volatile uint8_t  *)(MON_SCI_BASE + 2))
#define MON_SCI_TDR      (*(volatile uint8_t  *)(MON_SCI_BASE + 3))
#define MON_SCI_SSR      (*(volatile uint8_t  *)(MON_SCI_BASE + 4))
#define MON_SCI_RDR      (*(volatile uint8_t  *)(MON_SCI_BASE + 5))

/* SSR bit masks */
#define MON_SSR_TDRE     0x80   /* Transmit Data Register Empty */
#define MON_SSR_RDRF     0x40   /* Receive Data Register Full */
#define MON_SSR_ORER     0x20   /* Overrun Error */
#define MON_SSR_FER      0x10   /* Framing Error */
#define MON_SSR_PER      0x08   /* Parity Error */
#define MON_SSR_TEND     0x04   /* Transmit End */

/* SCR bit masks */
#define MON_SCR_TIE      0x80   /* Transmit Interrupt Enable */
#define MON_SCR_RIE      0x40   /* Receive Interrupt Enable */
#define MON_SCR_TE       0x20   /* Transmit Enable */
#define MON_SCR_RE       0x10   /* Receive Enable */

/* ================================================================
 * Protocol Constants
 * ================================================================ */

#define MON_SYNC_HOST_READY   0x3F   /* Host → Monitor: ready */
#define MON_SYNC_MON_READY    0x20   /* Monitor → Host: ready */

/* Command types (r3l in assembly) */
#define MON_CMD_NOP           0      /* No operation */
#define MON_CMD_READ_MEM      1      /* Read memory */
#define MON_CMD_WRITE_MEM     2      /* Write memory */
#define MON_CMD_READ_REGS     3      /* Read CPU registers */
#define MON_CMD_WRITE_REG     4      /* Write CPU register */
#define MON_CMD_GO            5      /* Go (run) */
#define MON_CMD_STEP          6      /* Single step */
#define MON_CMD_GORAM         7      /* Go to RAM (hot patch) */

/* Entry reasons (pushed on stack by _mon_breakp / _mon_trap) */
#define MON_REASON_BREAKP     0x1000
#define MON_REASON_TRAP       0x3000

/* ================================================================
 * Stack Frame Layout
 *
 * Built by _mon_breakp0 in monitor_asm.S:
 *   Hardware exception pushes CCR+PC (4 bytes).
 *   _mon_breakp/_mon_trap pushes ER0 + reason (8 bytes).
 *   _mon_breakp0 pushes ER1-ER6 (24 bytes).
 *
 * Total frame: 36 bytes.
 *
 * Push order (top of stack first):
 *   ER6, ER5, ER4, ER3, ER2, ER1, [reason, ER0], CCR+PC
 * ================================================================ */

typedef struct {
    uint32_t er6;           /* SP+0   — saved by _mon_breakp0 */
    uint32_t er5;           /* SP+4   — saved by _mon_breakp0 */
    uint32_t er4;           /* SP+8   — saved by _mon_breakp0 */
    uint32_t er3;           /* SP+12  — saved by _mon_breakp0 */
    uint32_t er2;           /* SP+16  — saved by _mon_breakp0 */
    uint32_t er1;           /* SP+20  — saved by _mon_breakp0 */
    uint32_t reason;        /* SP+24  — pushed by _mon_breakp/_mon_trap */
    uint32_t er0;           /* SP+28  — pushed by _mon_breakp/_mon_trap */
    uint32_t ccr_pc;        /* SP+32  — hardware exception frame */
} mon_frame_t;

/* ================================================================
 * Trace32 Register Numbering
 *
 * Protocol index → register mapping (matches readreg output order):
 *   0=ER7/SP, 1=MACH, 2=MACL, 3=ER6, 4=ER5, 5=ER4,
 *   6=ER3,    7=ER2,  8=ER1,  9=ER0, 10=EXR, 11=CCR+PC
 * ================================================================ */

#define MON_REG_ER7     0    /* SP — direct register, not on stack */
#define MON_REG_MACH    1    /* dummy 0 on H8S/2144 (no MAC unit) */
#define MON_REG_MACL    2    /* dummy 0 on H8S/2144 (no MAC unit) */
#define MON_REG_ER6     3
#define MON_REG_ER5     4
#define MON_REG_ER4     5
#define MON_REG_ER3     6
#define MON_REG_ER2     7
#define MON_REG_ER1     8
#define MON_REG_ER0     9
#define MON_REG_EXR    10    /* dummy 0 on H8S/2144 (int mode 0) */
#define MON_REG_CCRPC  11    /* combined CCR (high byte) + PC (24-bit) */
#define MON_REG_COUNT  12

/* ================================================================
 * Go Command Register Buffer
 *
 * C fills this array, assembly reads it to push registers and RTE.
 * Index: [0]=CCR+PC, [1]=EXR, [2]=ER0, [3]=ER1, [4]=ER2,
 *        [5]=ER3, [6]=ER4, [7]=ER5, [8]=ER6
 * ================================================================ */

extern volatile uint32_t mon_go_regs[9];

/* Return value from mon_protocol_loop() to signal Go command */
#define MON_ACTION_GO    1

/* ================================================================
 * Global Variables (defined in monitor_asm.S)
 * ================================================================ */

/* Runtime flag: 0=firmware mode, 1=monitor active.
 * Checked by ISR wrappers and PutChar guard.
 * Placed in .bss (zeroed on cold boot). */
extern volatile uint8_t  mon_active;

/* Persistent magic word for soft-reset re-entry.
 * Placed in .noinit (NOT zeroed by start.S).
 * Set to MON_MAGIC_VALUE by Start_Monitor(). */
extern volatile uint32_t mon_magic;

/* Saved user SP for Go command.
 * Initialized from stack frame at protocol loop entry.
 * Updated by Write Register for ER7.
 * Read by _mon_go_asm to set SP before push/pop/RTE. */
extern volatile uint32_t mon_saved_sp;

/* ================================================================
 * Function Prototypes
 * ================================================================ */

/* Assembly entry point — called from start.S on reset/magic re-entry */
extern void mon_start(void);

/* C protocol loop — called from _mon_breakp0 in assembly.
 * Returns MON_ACTION_GO when Go/Step command received (assembly
 * then reads mon_go_regs[] and does RTE).
 * For all other commands, loops forever (never returns). */
uint8_t mon_protocol_loop(mon_frame_t *frame, uint8_t entry_reason);

/* Assembly Go stub — called from monitor.c when Go/Step command
 * is received.  Reads mon_go_regs[], pushes to stack, clears
 * mon_active/mon_magic/RIE, pops registers, and executes RTE.
 * Does not return. */
extern void mon_go_asm(void) __attribute__((noreturn));

#endif /* MONITOR_H */
