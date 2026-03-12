/***********************************************************************
 *
 *  monitor.c — Trace32 ROM Monitor Protocol Handler (C implementation)
 *
 *  Implements the Lauterbach Trace32 ROM monitor V4.3 serial protocol
 *  for the H8S/2144 MCU.  Replaces the protocol logic from monitor.S
 *  with readable C code, while assembly stubs in monitor_asm.S handle
 *  register save/restore, RTE, and ISR wrappers.
 *
 *  SCI1 I/O is done by direct polled register access, bypassing the
 *  firmware's interrupt-driven ring buffer (PutChar/TXI).  A guard in
 *  PutChar() (sci.c) prevents TXI from firing while the monitor is
 *  active — this was the root cause of protocol corruption.
 *
 *  Protocol reference: monitor.S comments + Lauterbach documentation.
 *
 ***********************************************************************/

#include "monitor.h"

/* ================================================================
 * SCI1 Polled I/O
 *
 * These functions match the assembly readbyte/writebyte macros
 * exactly, including the TE toggle per byte in writebyte.
 * ================================================================ */

/**
 * Read one byte from SCI1 (polled).
 * Blocks until RDRF is set, then reads RDR and clears RDRF.
 * Matches assembly: btst #6,@er5 / mov.b @er4,r0l / bclr #6,@er5
 */
static uint8_t mon_readbyte(void)
{
    uint8_t b;

    /* Clear any error flags first — ORER blocks further reception */
    if (MON_SCI_SSR & (MON_SSR_ORER | MON_SSR_FER | MON_SSR_PER)) {
        MON_SCI_SSR &= (uint8_t)~(MON_SSR_ORER | MON_SSR_FER | MON_SSR_PER);
    }

    while (!(MON_SCI_SSR & MON_SSR_RDRF)) {
        /* Clear errors that may arrive while waiting */
        if (MON_SCI_SSR & (MON_SSR_ORER | MON_SSR_FER | MON_SSR_PER)) {
            MON_SCI_SSR &= (uint8_t)~(MON_SSR_ORER | MON_SSR_FER | MON_SSR_PER);
        }
    }
    b = MON_SCI_RDR;
    MON_SCI_SSR &= (uint8_t)~MON_SSR_RDRF;
    return b;
}

/**
 * Write one byte to SCI1 (polled).
 * TE toggle per byte: set TE → poll TDRE → write TDR → clear TDRE →
 * poll TEND → clear TE.
 *
 * Matches assembly:
 *   bset #5,@er4          ; TE = 1
 *   btst #7,@er5 / beq   ; poll TDRE
 *   mov.b r0l,@er6        ; write TDR
 *   bclr #7,@er5          ; clear TDRE
 *   btst #2,@er5 / beq    ; poll TEND
 *   bclr #5,@er4          ; TE = 0
 *
 * Note: The assembly manipulates SCR via er4 (SCI base+2→SMR, then
 * sub 3 to get SCR). The TE toggle per byte is unusual but Trace32
 * may depend on this timing for byte framing.
 */
static void mon_writebyte(uint8_t b)
{
    MON_SCI_SCR |= MON_SCR_TE;                           /* set TE */
    while (!(MON_SCI_SSR & MON_SSR_TDRE)) {}             /* poll TDRE */
    MON_SCI_TDR = b;                                     /* write TDR */
    MON_SCI_SSR &= (uint8_t)~MON_SSR_TDRE;               /* clear TDRE */
    while (!(MON_SCI_SSR & MON_SSR_TEND)) {}              /* poll TEND */
    MON_SCI_SCR &= (uint8_t)~MON_SCR_TE;                 /* clear TE */
}

/**
 * Initialize SCI1 for monitor polled protocol.
 * Matches assembly prepcom macro:
 *   SCR=0, SMR=0, BRR=8, delay, SCR=TE|RE
 *
 * CRITICAL: Also disables TIE — prevents TXI ISR from firing.
 */
static void mon_prepcom(void)
{
    MON_SCI_SCR = 0;                       /* disable everything (incl TIE/RIE) */
    MON_SCI_SMR = 0;                       /* async 8N1, system clock /1 */
    MON_SCI_BRR = 8;                       /* 57600 baud (matches InitSci2) */

    /* Wait at least 1 bit time for BRR to stabilize.
     * Assembly uses implicit delay from subsequent instructions.
     * We use a short spin loop. */
    {
        volatile uint16_t i;
        for (i = 0; i < 1000; i++) {}
    }

    /* Clear any pending errors */
    MON_SCI_SSR &= (uint8_t)~(MON_SSR_ORER | MON_SSR_FER | MON_SSR_PER);

    /* Enable RX only — TE is toggled per-byte in writebyte.
     * Must match assembly prepcom which only sets RE here.
     * writebyte handles the TE 0→1 transition per byte. */
    MON_SCI_SCR = MON_SCR_RE;
}

/**
 * Flush any pending RX byte from SCI1.
 * Matches assembly flushread macro.
 */
static void mon_flushread(void)
{
    if (MON_SCI_SSR & MON_SSR_RDRF) {
        (void)MON_SCI_RDR;                 /* read and discard */
        MON_SCI_SSR &= (uint8_t)~MON_SSR_RDRF;
    }
}


/* ================================================================
 * Checksum
 *
 * 16-bit accumulator.  Each byte is zero-extended to 16 bits and
 * added: extu.w r0; add.w r0,e3.
 * ================================================================ */

typedef struct {
    uint16_t val;
} mon_chk_t;

static void chk_reset(mon_chk_t *c)                { c->val = 0; }
static void chk_add(mon_chk_t *c, uint8_t b)       { c->val = (uint16_t)(c->val + b); }
static uint8_t chk_hi(const mon_chk_t *c)           { return (uint8_t)(c->val >> 8); }
static uint8_t chk_lo(const mon_chk_t *c)           { return (uint8_t)(c->val & 0xFF); }


/* ================================================================
 * Checksum-wrapped TX/RX helpers
 * ================================================================ */

/** Send byte and add to checksum. */
static void mon_tx(uint8_t b, mon_chk_t *c)
{
    chk_add(c, b);
    mon_writebyte(b);
}

/** Receive byte and add to checksum. */
static uint8_t mon_rx(mon_chk_t *c)
{
    uint8_t b = mon_readbyte();
    chk_add(c, b);
    return b;
}

/** Send 32-bit value as 4 bytes big-endian, adding each to checksum.
 *  Matches assembly writelong. */
static void mon_tx_long(uint32_t val, mon_chk_t *c)
{
    mon_tx((uint8_t)(val >> 24), c);
    mon_tx((uint8_t)(val >> 16), c);
    mon_tx((uint8_t)(val >>  8), c);
    mon_tx((uint8_t)(val >>  0), c);
}

/** Receive 32-bit value as 4 bytes big-endian, adding each to checksum.
 *  Matches assembly readlong. */
static uint32_t mon_rx_long(mon_chk_t *c)
{
    uint32_t val;
    val  = (uint32_t)mon_rx(c) << 24;
    val |= (uint32_t)mon_rx(c) << 16;
    val |= (uint32_t)mon_rx(c) <<  8;
    val |= (uint32_t)mon_rx(c);
    return val;
}


/* ================================================================
 * Data Checksum Verification
 *
 * After a data block, the host sends 2 checksum bytes.
 * Assembly: XOR read_hi with chk_hi, XOR read_lo with chk_lo.
 * Both must be zero.
 *
 * Returns 1 on success, 0 on failure.
 * ================================================================ */

static uint8_t mon_verify_data_checksum(mon_chk_t *c)
{
    uint8_t chk_h;
    uint8_t chk_l;

    chk_h = mon_readbyte();
    if (chk_h != chk_hi(c))
        return 0;

    chk_l = mon_readbyte();
    if (chk_l != chk_lo(c))
        return 0;

    return 1;
}


/* ================================================================
 * Sync Handshakes
 * ================================================================ */

/**
 * checkandreply pattern: verify data checksum, then send sync + ack.
 * Matches assembly checkandreply routine.
 *
 * After checkandreply, reset checksum and send data_len + msg_id.
 *
 * Returns 1 on success, 0 on failure (checksum or sync).
 */
static uint8_t mon_checkandreply(uint8_t data_len, uint8_t msg_id,
                                  mon_chk_t *c)
{
    uint8_t ack;

    /* Verify data checksum (2 bytes from host) */
    if (!mon_verify_data_checksum(c))
        return 0;

    /* Send sync */
    mon_writebyte(MON_SYNC_MON_READY);     /* TX 0x20 */

    /* Expect host ack */
    ack = mon_readbyte();
    if (ack != MON_SYNC_HOST_READY)        /* expect 0x3F */
        return 0;

    /* Reset checksum for response */
    chk_reset(c);

    /* Send data length and message ID */
    mon_tx(data_len, c);
    mon_tx(msg_id, c);

    return 1;
}

/**
 * Send checksum footer (mon_mainlp1 pattern).
 * TX checksum high byte, then low byte.
 */
static void mon_send_checksum_footer(mon_chk_t *c)
{
    mon_writebyte(chk_hi(c));
    mon_writebyte(chk_lo(c));
}


/* ================================================================
 * Register Access Helpers
 *
 * Map Trace32 register numbers to mon_frame_t fields.
 * ================================================================ */

/**
 * Read a register value by Trace32 protocol number.
 * frame points to the saved registers on the stack.
 * Returns the 32-bit register value.
 */
static uint32_t mon_read_reg(const mon_frame_t *frame, uint8_t reg_num)
{
    switch (reg_num) {
    case MON_REG_ER7:   /* SP — tracked in mon_saved_sp */
        return mon_saved_sp;
    case MON_REG_MACH:  return 0;   /* dummy on H8S/2144 */
    case MON_REG_MACL:  return 0;   /* dummy on H8S/2144 */
    case MON_REG_ER6:   return frame->er6;
    case MON_REG_ER5:   return frame->er5;
    case MON_REG_ER4:   return frame->er4;
    case MON_REG_ER3:   return frame->er3;
    case MON_REG_ER2:   return frame->er2;
    case MON_REG_ER1:   return frame->er1;
    case MON_REG_ER0:   return frame->er0;
    case MON_REG_EXR:   return 0;   /* dummy on H8S/2144 (int mode 0) */
    case MON_REG_CCRPC: return frame->ccr_pc;
    default:            return 0;
    }
}

/**
 * Write a register value by Trace32 protocol number.
 * Modifies the saved register on the stack (effective on RTE).
 */
static void mon_write_reg(mon_frame_t *frame, uint8_t reg_num, uint32_t val)
{
    switch (reg_num) {
    case MON_REG_ER7:
        /* Store desired SP — _mon_go_asm uses mon_saved_sp to
         * set ER7 before push/pop/RTE on Go command. */
        mon_saved_sp = val;
        break;
    case MON_REG_MACH:  break;  /* no MACH on H8S/2144 */
    case MON_REG_MACL:  break;  /* no MACL on H8S/2144 */
    case MON_REG_ER6:   frame->er6    = val; break;
    case MON_REG_ER5:   frame->er5    = val; break;
    case MON_REG_ER4:   frame->er4    = val; break;
    case MON_REG_ER3:   frame->er3    = val; break;
    case MON_REG_ER2:   frame->er2    = val; break;
    case MON_REG_ER1:   frame->er1    = val; break;
    case MON_REG_ER0:   frame->er0    = val; break;
    case MON_REG_EXR:   break;  /* no EXR on H8S/2144 (int mode 0) */
    case MON_REG_CCRPC: frame->ccr_pc = val; break;
    default:            break;
    }
}


/* ================================================================
 * Command Handlers
 * ================================================================ */

/**
 * Command 1 — Read Memory
 *
 * Protocol (after param checksum verified):
 *   RX: data_chk_high, data_chk_low  (verify)
 *   TX: 0x20 sync, RX: expect 0x3F
 *   Reset checksum
 *   TX: data_length, msg_id
 *   TX: memory[addr..addr+len-1]  (each byte checksummed)
 *   TX: checksum_high, checksum_low  (mon_mainlp1)
 *   → back to mon_mainlp
 *
 * Access width: bit1=word, bit2=long, falls back to byte.
 */
static void cmd_read_mem(uint8_t msg_id, uint16_t access_width,
                          uint32_t addr, uint8_t data_len, mon_chk_t *c)
{
    uint8_t width;
    uint8_t use_long;
    uint8_t use_word;
    uint8_t count;
    volatile uint8_t  *p8;
    volatile uint16_t *p16;
    volatile uint32_t *p32;
    uint32_t val32;
    uint16_t val16;

    /* Verify data checksum (host sends 2 bytes after params) */
    if (!mon_verify_data_checksum(c))
        return;  /* checksum fail → resync (caller goes to mon_mainlp) */

    /* Send sync */
    mon_writebyte(MON_SYNC_MON_READY);
    if (mon_readbyte() != MON_SYNC_HOST_READY)
        return;  /* host not ready → resync */

    /* Reset checksum for response */
    chk_reset(c);

    /* Send data length and message ID */
    mon_tx(data_len, c);
    mon_tx(msg_id, c);

    /* Determine access mode.
     * Assembly: btst #2 → long; btst #1 → word; else byte.
     * Falls back to byte if alignment is wrong. */
    width = (uint8_t)(access_width & 0xFF);
    use_long = (width & 0x04) && !(addr & 0x03) && !(data_len & 0x03);
    use_word = !use_long && (width & 0x02) && !(addr & 0x01) && !(data_len & 0x01);

    p8  = (volatile uint8_t *)addr;
    p16 = (volatile uint16_t *)addr;
    p32 = (volatile uint32_t *)addr;

    if (use_long) {
        /* Long-word access: read 4 bytes per iteration */
        count = data_len >> 2;
        while (count--) {
            val32 = *p32++;
            mon_tx((uint8_t)(val32 >> 24), c);
            mon_tx((uint8_t)(val32 >> 16), c);
            mon_tx((uint8_t)(val32 >>  8), c);
            mon_tx((uint8_t)(val32 >>  0), c);
        }
    } else if (use_word) {
        /* Word access: read 2 bytes per iteration */
        count = data_len >> 1;
        while (count--) {
            val16 = *p16++;
            mon_tx((uint8_t)(val16 >> 8), c);
            mon_tx((uint8_t)(val16 >> 0), c);
        }
    } else {
        /* Byte access */
        count = data_len;
        while (count--) {
            mon_tx(*p8++, c);
        }
    }

    /* Send checksum footer (mon_mainlp1) */
    mon_send_checksum_footer(c);
}


/**
 * Command 2 — Write Memory
 *
 * Protocol (after param checksum verified):
 *   RX: data_length bytes → write to memory[addr..], checksum each
 *   RX: data_chk_high, data_chk_low  (verify)
 *   → back to mon_mainlp2 (send status)
 *
 * Returns 1 to signal caller to go to mon_mainlp2, 0 for resync.
 */
static uint8_t cmd_write_mem(uint16_t access_width, uint32_t addr,
                              uint8_t data_len, mon_chk_t *c)
{
    uint8_t width = (uint8_t)(access_width & 0xFF);
    uint8_t use_long = (width & 0x04) && !(addr & 0x03) && !(data_len & 0x03);
    uint8_t use_word = !use_long && (width & 0x02) && !(addr & 0x01) && !(data_len & 0x01);

    volatile uint8_t  *p8  = (volatile uint8_t *)addr;
    volatile uint16_t *p16 = (volatile uint16_t *)addr;
    volatile uint32_t *p32 = (volatile uint32_t *)addr;

    if (use_long) {
        uint8_t count = data_len >> 2;
        while (count--) {
            uint32_t val = mon_rx_long(c);
            *p32++ = val;
        }
    } else if (use_word) {
        uint8_t count = data_len >> 1;
        while (count--) {
            uint16_t val;
            val  = (uint16_t)mon_rx(c) << 8;
            val |= (uint16_t)mon_rx(c);
            *p16++ = val;
        }
    } else {
        uint8_t count = data_len;
        while (count--) {
            *p8++ = mon_rx(c);
        }
    }

    /* mon_mainlp3: verify data checksum */
    if (!mon_verify_data_checksum(c))
        return 0;  /* checksum fail → resync */

    return 1;  /* success → caller sends status (mon_mainlp2) */
}


/**
 * Command 3 — Read Registers
 *
 * Protocol:
 *   checkandreply (verify checksum, send sync+ack, send len+id)
 *   If data_len != 36: send ER7, MACH(0), MACL(0) = 12 bytes
 *   If data_len == 36: send ER6,ER5,ER4,ER3,ER2,ER1,ER0,EXR(0),CCR+PC
 *   TX: checksum_high, checksum_low  (mon_mainlp1)
 *   → back to mon_mainlp
 */
static void cmd_read_regs(mon_frame_t *frame, uint8_t msg_id,
                           uint8_t data_len, mon_chk_t *c)
{
    if (!mon_checkandreply(data_len, msg_id, c))
        return;  /* checksum/sync fail → resync */

    if (data_len != 36) {
        /* Short read: ER7(SP), MACH(0), MACL(0) = 12 bytes */
        mon_tx_long(mon_read_reg(frame, MON_REG_ER7), c);    /* SP */
        mon_tx_long(0, c);                                     /* dummy MACH */
        mon_tx_long(0, c);                                     /* dummy MACL */
    } else {
        /* Full read: ER6,ER5,ER4,ER3,ER2,ER1,ER0,EXR(0),CCR+PC = 36 bytes */
        mon_tx_long(frame->er6,    c);
        mon_tx_long(frame->er5,    c);
        mon_tx_long(frame->er4,    c);
        mon_tx_long(frame->er3,    c);
        mon_tx_long(frame->er2,    c);
        mon_tx_long(frame->er1,    c);
        mon_tx_long(frame->er0,    c);
        mon_tx_long(0,             c);    /* dummy EXR (H8S/2144, int mode 0) */
        mon_tx_long(frame->ccr_pc, c);
    }

    /* Send checksum footer (mon_mainlp1) */
    mon_send_checksum_footer(c);
}


/**
 * Command 4 — Write Register
 *
 * Protocol:
 *   If data_len == 0: → mon_mainlp3 (just verify checksum)
 *   If data_len != 4: → resync
 *   RX: 4 value bytes → checksum each
 *   If reg_num >= 12: ignore → mon_mainlp3
 *   Else: store in frame → mon_mainlp3
 *
 * Returns 1 for mon_mainlp2, 0 for resync.
 */
static uint8_t cmd_write_reg(mon_frame_t *frame, uint32_t reg_num,
                              uint8_t data_len, mon_chk_t *c)
{
    uint32_t val;

    if (data_len == 0) {
        /* No data — just verify checksum (mon_mainlp3) */
        return mon_verify_data_checksum(c);
    }

    if (data_len != 4) {
        /* Illegal packet length → resync */
        return 0;
    }

    /* Read 4-byte register value */
    val = mon_rx_long(c);

    /* Range check — register number is in addr field (er2) */
    if (reg_num < MON_REG_COUNT) {
        mon_write_reg(frame, (uint8_t)reg_num, val);
    }
    /* else: out of range, ignore value */

    /* mon_mainlp3: verify data checksum */
    return mon_verify_data_checksum(c);
}


/**
 * Command 5/6 — Go / Step
 *
 * Protocol:
 *   RX: CCR+PC (4 bytes)  → mon_go_regs[0]
 *   RX: EXR    (4 bytes)  → mon_go_regs[1]
 *   RX: ER0    (4 bytes)  → mon_go_regs[2]
 *   RX: ER1    (4 bytes)  → mon_go_regs[3]
 *   RX: ER2    (4 bytes)  → mon_go_regs[4]
 *   RX: ER3    (4 bytes)  → mon_go_regs[5]
 *   RX: ER4    (4 bytes)  → mon_go_regs[6]
 *   RX: ER5    (4 bytes)  → mon_go_regs[7]
 *   RX: ER6    (4 bytes)  → mon_go_regs[8]
 *   RX: data_chk_high, data_chk_low  (verify)
 *   TX: 0x20 sync, RX: expect 0x3F
 *   TX: size=0, msg_id, checksum_high, checksum_low
 *   → assembly: push registers, clear mon_active/magic/RIE, RTE
 *
 * Returns 1 to signal Go (assembly takes over), 0 for resync.
 */
static uint8_t cmd_go(uint8_t msg_id, mon_chk_t *c)
{
    uint8_t i;
    uint8_t chk_h;
    uint8_t chk_l;
    mon_chk_t ack_chk;

    /* Read 9 × 4 = 36 bytes of register data */
    for (i = 0; i < 9; i++) {
        mon_go_regs[i] = mon_rx_long(c);
    }

    /* Verify data checksum */
    chk_h = mon_readbyte();
    if (chk_h != chk_hi(c))
        return 0;  /* checksum fail → resync (assembly handles stack cleanup) */

    chk_l = mon_readbyte();
    if (chk_l != chk_lo(c))
        return 0;

    /* Send sync */
    mon_writebyte(MON_SYNC_MON_READY);
    if (mon_readbyte() != MON_SYNC_HOST_READY)
        return 0;

    /* Send final ack: size=0, msg_id, checksum */
    chk_reset(&ack_chk);
    mon_tx(0, &ack_chk);               /* size = 0 */
    mon_tx(msg_id, &ack_chk);          /* message ID */
    mon_send_checksum_footer(&ack_chk); /* checksum high + low */

    return 1;  /* signal Go to caller */
}


/**
 * Command 0/7 — NOP / GoRAM
 *
 * Protocol (checkandreply pattern):
 *   RX: data_chk_high, data_chk_low  (verify)
 *   TX: 0x20 sync, RX: expect 0x3F
 *   TX: data_len=0, msg_id
 *   TX: checksum_high, checksum_low  (mon_mainlp1)
 *   → back to mon_mainlp
 *
 * GoRAM (type 7) sends checksum high before branching in assembly,
 * but for the simulator/C version, both NOP and GoRAM are handled
 * the same way (no hot-patch support needed).
 */
static void cmd_nop_goram(uint8_t msg_id, mon_chk_t *c)
{
    /* checkandreply: verify checksum, send sync + ack + header */
    if (!mon_checkandreply(0, msg_id, c))
        return;

    /* Send checksum footer (mon_mainlp1) */
    mon_send_checksum_footer(c);
}


/* ================================================================
 * Main Protocol Loop
 *
 * Called from _mon_breakp0 in monitor_asm.S after registers are
 * saved to the stack.  frame points to the saved register frame.
 * entry_reason is the high byte of the reason code:
 *   0x10 = breakpoint, 0x30 = trap, 0x40 = reset
 *
 * This function loops forever for normal commands.  When a Go/Step
 * command is received, it calls mon_go_asm() which never returns
 * (it does RTE to user code).
 * ================================================================ */

uint8_t mon_protocol_loop(mon_frame_t *frame, uint8_t entry_reason)
{
    mon_chk_t chk;
    uint8_t msg_id = entry_reason;  /* r3h equivalent */
    uint16_t burst_i;

    /* Per-iteration variables (C89: declare at top) */
    uint8_t sync_byte;
    uint8_t size_byte;
    uint8_t cmd_byte;
    uint8_t cmd_type;
    uint8_t access_class;
    uint8_t access_width_byte;
    uint16_t access_width;
    uint32_t addr;
    uint8_t data_len;
    uint8_t param_chk;

    /* Save user's SP from the exception frame.
     * frame points to the bottom of the 36-byte mon_frame_t;
     * the user's original SP was just above it. */
    mon_saved_sp = (uint32_t)frame + sizeof(mon_frame_t);

    /* SCI1 prepcom is now done in assembly (monitor_asm.S) before
     * calling this function, matching original monitor.S flow.
     * SCI1 is already in polled mode: SCR=RE, 57600 baud. */

    /* Send 256 × 0xFF burst to flush host receiver.
     * Skip for breakpoint re-entry (reason 0x10). */
    if (entry_reason != 0x10) {
        for (burst_i = 0; burst_i < 256; burst_i++) {
            mon_writebyte(0xFF);
        }
    }

    /* Flush any pending RX */
    mon_flushread();

    /* === Main loop === */
    for (;;) {

        /* --- mon_mainlp2: send status to host --- */

        mon_writebyte(0x00);                    /* status byte */
        mon_writebyte(MON_SYNC_MON_READY);      /* sync 0x20 */

        /* Wait for host ack 0x3F */
        if (mon_readbyte() != MON_SYNC_HOST_READY) {
            /* Host not ready — go to mon_mainlp (skip status next time) */
            goto mon_mainlp;
        }

        /* Send status message: size=0, msg_id, checksum */
        chk_reset(&chk);
        mon_tx(0x00, &chk);                    /* size = 0 */
        mon_tx(msg_id, &chk);                  /* message ID */
        mon_send_checksum_footer(&chk);         /* checksum high + low */

mon_mainlp:
        /* --- mon_mainlp: wait for host command --- */

        chk_reset(&chk);

        /* Wait for host sync 0x20 */
        do {
            sync_byte = mon_readbyte();
        } while (sync_byte != MON_SYNC_MON_READY);  /* 0x20 from host */

        /* Acknowledge */
        mon_writebyte(MON_SYNC_HOST_READY);     /* TX 0x3F */

        /* Read header: size, cmd, type */
        size_byte = mon_rx(&chk);
        (void)size_byte;  /* stored but unused for routing */

        cmd_byte = mon_rx(&chk);
        msg_id = cmd_byte & 0x0F;               /* message ID */

        cmd_type = mon_rx(&chk);

        /* NOP or GoRAM — no params */
        if (cmd_type == MON_CMD_NOP || cmd_type == MON_CMD_GORAM) {
            cmd_nop_goram(msg_id, &chk);
            goto mon_mainlp;  /* asm: bra mon_mainlp (skip status) */
        }

        /* Out of range → ignore, re-enter mainlp */
        if (cmd_type > MON_CMD_GORAM) {
            goto mon_mainlp;  /* asm: bgt mon_mainlp */
        }

        /* Read standard parameters (cmd_type 1-6) */
        access_class = mon_rx(&chk);    /* access class */
        (void)access_class;

        access_width_byte = mon_rx(&chk);
        access_width = access_width_byte;

        /* 4-byte address (big-endian) */
        addr = 0;
        addr  = (uint32_t)mon_rx(&chk) << 24;
        addr |= (uint32_t)mon_rx(&chk) << 16;
        addr |= (uint32_t)mon_rx(&chk) <<  8;
        addr |= (uint32_t)mon_rx(&chk);

        /* Data length */
        data_len = mon_rx(&chk);
        if (data_len == 0xFF) {
            goto mon_mainlp;  /* asm: beq mon_mainlp */
        }

        /* Parameter checksum verification.
         * Assembly: compare low_byte(checksum) with received byte.
         * Then add param_chk to running checksum. */
        param_chk = mon_readbyte();
        if (param_chk != chk_lo(&chk)) {
            goto mon_mainlp;  /* asm: bne mon_mainlp */
        }
        chk_add(&chk, param_chk);

        /* --- Dispatch command --- */
        switch (cmd_type) {

        case MON_CMD_READ_MEM:   /* 1 */
            cmd_read_mem(msg_id, access_width, addr, data_len, &chk);
            goto mon_mainlp;  /* asm: bra mon_mainlp1 → mon_mainlp */

        case MON_CMD_WRITE_MEM:  /* 2 */
            if (cmd_write_mem(access_width, addr, data_len, &chk)) {
                continue;  /* success → mon_mainlp2 (asm: bra mon_mainlp2) */
            }
            goto mon_mainlp;  /* fail → mon_mainlp */

        case MON_CMD_READ_REGS:  /* 3 */
            cmd_read_regs(frame, msg_id, data_len, &chk);
            goto mon_mainlp;  /* asm: bra mon_mainlp1 → mon_mainlp */

        case MON_CMD_WRITE_REG:  /* 4 */
            if (cmd_write_reg(frame, addr, data_len, &chk)) {
                continue;  /* success → mon_mainlp2 (asm: bra mon_mainlp2) */
            }
            goto mon_mainlp;  /* fail → mon_mainlp */

        case MON_CMD_GO:         /* 5 */
        case MON_CMD_STEP:       /* 6 */
            if (cmd_go(msg_id, &chk)) {
                /* Go command succeeded — assembly takes over.
                 * mon_go_asm() pushes registers, clears flags, RTE.
                 * Does not return. */
                mon_go_asm();
                /* NOTREACHED */
            }
            goto mon_mainlp;  /* checksum fail → mon_mainlp */

        default:
            goto mon_mainlp;  /* unknown → mon_mainlp */
        }

        /* Write commands that succeed use 'continue' above to go
         * to mon_mainlp2 (send status before accepting next command).
         *
         * All other paths use 'goto mon_mainlp' to skip the status
         * message and wait directly for the next host command.
         * This matches assembly: read/NOP → mon_mainlp,
         * write OK → mon_mainlp2, errors → mon_mainlp. */
    }

    /* NOTREACHED */
    return 0;
}
