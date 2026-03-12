.h8300s
.altmacro
;----------------------------------------------------------------------
;
;	ROM Monitor Assembly Stubs for H8S/2144
;
;	Minimal assembly that handles what C cannot:
;	  - Register save/restore (push/pop ER1-ER6)
;	  - Exception return (RTE)
;	  - ISR wrappers (mon_rxi1_wrapper, mon_eri1_wrapper)
;	  - Config table (monitor identification)
;	  - Go command: push registers from mon_go_regs[], RTE
;
;	Protocol logic is implemented in monitor.c.
;
;	This file replaces the monolithic monitor.S.
;	All external symbol names are preserved for compatibility
;	with vects.c and start.S.
;
;----------------------------------------------------------------------

;----------------------------------------------------------------------
;  Processor Configuration
;----------------------------------------------------------------------

.equ serial, 1
.equ advanced, 1

;----------------------------------------------------------------------
;  H8S/2144 Register Addresses used by monitor
;----------------------------------------------------------------------

.equ ICRC,       0xFFFEEA	; IRQ Control C (SCI priority)

;----------------------------------------------------------------------
;  External symbols
;----------------------------------------------------------------------

	; C protocol handler (monitor.c)
	.extern _mon_protocol_loop

	; Go command assembly stub
	; (mon_go_regs[] is defined below in .bss)

	; Firmware ISR handlers (sci.c)
	.extern _RXI1_SCI1
	.extern _ERI1_SCI1

	; (LCD calls removed — follow original monitor.S flow)

;----------------------------------------------------------------------
;  Global symbols (used by vects.c, start.S, main.c, monitor.c)
;----------------------------------------------------------------------

	.global _mon_start
	.global _mon_breakp
	.global _mon_trap
	.global _mon_breakp0
	.global _mon_rxi1_wrapper
	.global _mon_eri1_wrapper
	.global _mon_active
	.global _mon_magic
	.global _mon_go_regs
	.global _mon_go_asm


;----------------------------------------------------------------------
;
;	Monitor Configuration Table
;
;	Trace32 may read these at specific offsets relative to
;	mon_start, so keep the layout identical to original monitor.S.
;
;----------------------------------------------------------------------
	.align	4

mon_proctype:	.byte	0x5		; processor type 0x5 = H8S/21xx
mon_imask0:	.byte	0x3		; bit0=I, bit1=UI
mon_imask1:	.byte	0x7		; bit2-0: I2-I0
	.byte	0x0			; reserved

mon_scichannel:	.byte	0x1		; SCI Channel 1
mon_scibrr:	.byte	0x08		; BRR=8 → 57600 baud
	.byte	0x0			; reserved
	.byte	0x0			; reserved

mon_scibase:	.long	0xffff88	; SCI1 base address

mon_pllfactor:	.byte	0x3		; PLL off


;----------------------------------------------------------------------
;
;	Entry Jump Table
;
;	These entry points are referenced by vects.c (vector table)
;	and start.S (soft-reset re-entry).
;
;----------------------------------------------------------------------

;----------------------------------------------------------------------
; _mon_start — Entry by reset / soft-reset re-entry
;
; Called from start.S when mon_magic matches.
; R3H = 0x40 (reset entry reason).
;----------------------------------------------------------------------
_mon_start:
	mov.b	#0x40,r3h		; entry by reset
	bra	mon_startr:16

;----------------------------------------------------------------------
; _mon_breakp — Entry by breakpoint (NMI, ABRK, address break)
;
; Hardware has already pushed CCR+PC on stack.
; We push ER0 and reason 0x1000, then jump to _mon_breakp0.
;----------------------------------------------------------------------
_mon_breakp:
	push.l	er0
	mov.l	#0x1000,er0
	push.l	er0
	bra	_mon_breakp0:16		; entry by breakpoint

;----------------------------------------------------------------------
; _mon_trap — Entry by TRAP #2 (from Start_Monitor)
;
; Hardware has already pushed CCR+PC on stack.
; We push ER0 and reason 0x3000, then jump to _mon_breakp0.
;----------------------------------------------------------------------
_mon_trap:
	push.l	er0
	mov.l	#0x3000,er0
	push.l	er0
	bra	_mon_breakp0:16		; entry by trap


;----------------------------------------------------------------------
;
;	Monitor Register Save and C Entry
;
;----------------------------------------------------------------------

;----------------------------------------------------------------------
; mon_startr — Common path after _mon_start
;
; For reset entry, we don't have a valid stack frame from an
; exception.  We need to create a dummy frame for the C code.
; Push dummy ER0, reason, ER1-ER6 to build mon_frame_t on stack.
;----------------------------------------------------------------------
mon_startr:
	nop

	; For reset entry, build a dummy exception frame:
	; Push dummy CCR+PC (0x00000000)
	sub.l	er0,er0
	push.l	er0			; dummy CCR+PC (SP+32)

	; Push dummy ER0 and reason
	push.l	er0			; dummy ER0 (SP+28)
	mov.l	#0x4000,er0		; reset reason (0x40 << 8)
	push.l	er0			; reason (SP+24)

	; Push ER1-ER6
	push.l	er1			; SP+20
	push.l	er2			; SP+16
	push.l	er3			; SP+12
	push.l	er4			; SP+8
	push.l	er5			; SP+4
	push.l	er6			; SP+0

	; Set interrupt mask (same as breakp1)
	stc	ccr,r0l
	mov.b	@mon_imask0,r0h
	btst	#0,r0h
	beq	.Lstart_noI
	bset	#7,r0l
	bra	.Lstart_UI
.Lstart_noI:
	bclr	#7,r0l
.Lstart_UI:
	btst	#1,r0h
	beq	.Lstart_noUI
	bset	#6,r0l
	bra	.Lstart_call
.Lstart_noUI:
	bclr	#6,r0l
.Lstart_call:
	ldc	r0l,ccr

	; Clear mon_magic
	sub.l	er0,er0
	mov.l	er0,@_mon_magic:32

	; --- SCI1 prepcom (same as breakp0 path) ---
	mov.l	@mon_scibase,er2
	sub.b	r0l,r0l
	mov.b	r0l,@(2,er2)		; SCR = 0
	mov.b	r0l,@(0,er2)		; SMR = 0
	mov.b	@mon_scibrr,r0l
	mov.b	r0l,@(1,er2)		; BRR
	mov.w	#400,r0
.Lstart_brrwait:
	nop
	dec.w	#1,r0
	bne	.Lstart_brrwait
	mov.b	@(4,er2),r0l
	and.b	#0xC7,r0l
	mov.b	r0l,@(4,er2)		; clear SSR errors
	mov.b	#0x10,r0l
	mov.b	r0l,@(2,er2)		; SCR = RE

	; Call C protocol loop
	; arg1 = frame pointer (current SP = &mon_frame_t)
	; arg2 = entry reason (0x40 for reset)
	mov.l	er7,er0			; er0 = frame pointer
	mov.b	#0x40,r1l		; r1l = entry_reason
	extu.w	r1
	extu.l	er1
	jsr	@_mon_protocol_loop

	; Should never return — but if it does, infinite loop
.Lstart_hang:
	bra	.Lstart_hang:8


;----------------------------------------------------------------------
; _mon_breakp0 — Common register save for breakpoint/trap entry
;
; Stack at entry:
;   SP+0  = reason (4 bytes)
;   SP+4  = saved ER0 (4 bytes)
;   SP+8  = saved CCR+PC (4 bytes, hardware exception frame)
;
; We push ER1-ER6 to complete the mon_frame_t layout.
;----------------------------------------------------------------------
_mon_breakp0:
	push.l	er1			; SP+20
	push.l	er2			; SP+16
	push.l	er3			; SP+12
	push.l	er4			; SP+8
	push.l	er5			; SP+4
	push.l	er6			; SP+0

	; Set interrupt mask from mon_imask0
breakp1:
	stc	ccr,r0l
	mov.b	@mon_imask0,r0h
	btst	#0,r0h
	beq	breakp11:8
	bset	#7,r0l
	bra	breakp12:8
breakp11:
	bclr	#7,r0l
breakp12:
	btst	#1,r0h
	beq	breakp13:8
	bset	#6,r0l
	bra	breakp14:8
breakp13:
	bclr	#6,r0l
breakp14:
	ldc	r0l,ccr			; set I- and UI-bit to mon_imask0

	; Get entry reason
	mov.l	@(24,er7),er3		; reason code

	; Clear mon_magic — prevent reset-loop
	sub.l	er0,er0
	mov.l	er0,@_mon_magic:32

	; --- SCI1 prepcom: reinitialise SCI for polled monitor protocol ---
	; Matches original monitor.S flow: prepcom runs in assembly
	; BEFORE calling the C protocol loop.
	;
	; Uses mon_scibase (0xFFFF88) from config table.
	; Register usage: er2 = SCI base, r0l/r0h = scratch
	mov.l	@mon_scibase,er2	; er2 = 0xFFFF88

	; SCR = 0  (disable TE/RE/TIE/RIE)
	sub.b	r0l,r0l
	mov.b	r0l,@(2,er2)		; SCR offset +2

	; SMR = 0  (async, 8N1, clock /1)
	mov.b	r0l,@(0,er2)		; SMR offset +0

	; BRR = mon_scibrr  (8 → 57600 baud at 18.432MHz)
	mov.b	@mon_scibrr,r0l
	mov.b	r0l,@(1,er2)		; BRR offset +1

	; Wait at least 1 bit time for BRR to take effect
	; (original monitor.S used ~32 NOPs worth of delay)
	mov.w	#400,r0
.Lbrk_brrwait:
	nop
	dec.w	#1,r0
	bne	.Lbrk_brrwait

	; Clear error flags in SSR (ORER|FER|PER = bits 5,4,3)
	mov.b	@(4,er2),r0l		; read SSR
	and.b	#0xC7,r0l		; clear bits 5,4,3
	mov.b	r0l,@(4,er2)		; write SSR

	; SCR = RE  (receive enable only; TE toggled per-byte by writebyte)
	mov.b	#0x10,r0l		; RE = bit 4
	mov.b	r0l,@(2,er2)		; SCR offset +2

	; Call C protocol loop
	; arg1 = frame pointer (current SP = &mon_frame_t)
	; arg2 = entry reason high byte (r3h: 0x10=breakp, 0x30=trap)
	mov.l	er7,er0			; er0 = frame pointer
	mov.b	r3h,r1l			; r1l = entry_reason
	extu.w	r1
	extu.l	er1
	jsr	@_mon_protocol_loop

	; Should never return — but if it does, infinite loop
.Lbrk_hang:
	bra	.Lbrk_hang:8


;----------------------------------------------------------------------
;
;	Go Command — Assembly Register Restore + RTE
;
;	Called from monitor.c when Go/Step command is received.
;	Reads register values from mon_go_regs[] array, pushes them
;	onto stack in the correct order, clears monitor flags, and
;	executes RTE to resume user code.
;
;	mon_go_regs[9] layout:
;	  [0]=CCR+PC  [1]=EXR  [2]=ER0  [3]=ER1  [4]=ER2
;	  [5]=ER3     [6]=ER4  [7]=ER5  [8]=ER6
;
;----------------------------------------------------------------------
_mon_go_asm:
	; First, we need to build a proper stack frame for RTE.
	; The H8S/2144 (proctype 5) does NOT use EXR on the exception
	; stack frame (interrupt control mode 0).
	;
	; Stack frame for RTE (H8S/2144):
	;   SP+0  = ER6 (restored by pop)
	;   SP+4  = ER5
	;   SP+8  = ER4
	;   SP+12 = ER3
	;   SP+16 = ER2
	;   SP+20 = ER1
	;   SP+24 = ER0
	;   SP+28 = CCR+PC (consumed by RTE)

	mov.l	#_mon_go_regs,er2

	; Push CCR+PC (RTE will pop this)
	mov.l	@(0,er2),er0		; [0] = CCR+PC
	push.l	er0

	; Push ER0 (will be popped last)
	mov.l	@(8,er2),er0		; [2] = ER0
	push.l	er0

	; Push ER1
	mov.l	@(12,er2),er0		; [3] = ER1
	push.l	er0

	; Push ER2
	mov.l	@(16,er2),er0		; [4] = ER2
	push.l	er0

	; Push ER3
	mov.l	@(20,er2),er0		; [5] = ER3
	push.l	er0

	; Push ER4
	mov.l	@(24,er2),er0		; [6] = ER4
	push.l	er0

	; Push ER5
	mov.l	@(28,er2),er0		; [7] = ER5
	push.l	er0

	; Push ER6
	mov.l	@(32,er2),er0		; [8] = ER6
	push.l	er0

#ifdef INTEGRATED
	; --- Integrated mode: prepare for return to firmware ---

	; Set SCI1 RIE for Trace32 break-in capability
	; (prepgo equivalent — set SCI interrupt priority + RIE)
	; H8S/21xx: set ICRC bit for SCI1 high priority
	mov.b	@mon_scichannel,r0h
	mov.b	#7,r0l
	sub.b	r0h,r0l			; r0l = 7 - channel = ICRC bit#
	bset	r0l,@ICRC		; SCI1 interrupt to high priority

	; Enable RIE for break-in
	mov.l	@mon_scibase,er0
	add.b	#2,r0l			; er0 → SCR
	bset	#6,@er0			; SCR.RIE = 1

	; Now UNDO the RIE — in integrated mode, we don't support
	; break-in via serial (firmware also uses SCI1).
	; Clear RIE, mon_active, mon_magic before returning to user code.
	bclr	#6,@er0			; SCR.RIE = 0

	; Clear runtime flag — firmware ISRs handle SCI1 normally
	sub.b	r0l,r0l
	mov.b	r0l,@_mon_active:32

	; Clear magic word — prevent reset-loop if user code crashes
	sub.l	er0,er0
	mov.l	er0,@_mon_magic:32
#endif

	; Pop registers and RTE
	pop.l	er6
	pop.l	er5
	pop.l	er4
	pop.l	er3
	pop.l	er2
	pop.l	er1
	pop.l	er0
	rte				; pop CCR+PC, resume user code


;----------------------------------------------------------------------
;
;	Integrated-mode ISR Wrappers
;
;	When INTEGRATED is defined, these wrappers check mon_active
;	to route SCI1 interrupts to either the monitor or the firmware.
;
;----------------------------------------------------------------------

#ifdef INTEGRATED

;----------------------------------------------------------------------
; RXI1 wrapper: if monitor active → enter monitor (breakp path)
;               else             → call firmware RXI1_SCI1
;----------------------------------------------------------------------
_mon_rxi1_wrapper:
	push.l	er0
	mov.b	@_mon_active:32,r0l
	beq	.Lrxi1_fw
	; Monitor path — mimic breakp entry: ER0 saved, push reason 0x1000
	mov.l	#0x1000,er0
	push.l	er0
	bra	_mon_breakp0:16
.Lrxi1_fw:
	pop.l	er0
	jmp	@_RXI1_SCI1

;----------------------------------------------------------------------
; ERI1 wrapper: if monitor active → enter monitor (trap path)
;               else             → call firmware ERI1_SCI1
;----------------------------------------------------------------------
_mon_eri1_wrapper:
	push.l	er0
	mov.b	@_mon_active:32,r0l
	beq	.Leri1_fw
	; Monitor path — mimic trap entry: ER0 saved, push reason 0x3000
	mov.l	#0x3000,er0
	push.l	er0
	bra	_mon_breakp0:16
.Leri1_fw:
	pop.l	er0
	jmp	@_ERI1_SCI1

#endif /* INTEGRATED */


;----------------------------------------------------------------------
;  (LCD debug strings removed — follow original monitor.S flow)
;----------------------------------------------------------------------


;----------------------------------------------------------------------
;
;	Data Sections
;
;----------------------------------------------------------------------

;----------------------------------------------------------------------
; mon_active — runtime flag: 0=firmware, 1=monitor active
; Placed in .bss (zeroed on cold boot = firmware mode).
;----------------------------------------------------------------------
	.section .bss
	.align	2
_mon_active:
	.byte	0

;----------------------------------------------------------------------
; mon_go_regs — Go command register buffer (C fills, assembly reads)
; 9 × 4 = 36 bytes.
; Placed in .bss.
;----------------------------------------------------------------------
	.align	4
_mon_go_regs:
	.space	36			; 9 × 4 bytes

;----------------------------------------------------------------------
; mon_magic — persistent magic word for soft-reset re-entry.
; Placed in .noinit (NOT zeroed by start.S).
; Value 0x4D4F4E21 = "MON!" means monitor was active before reset.
;----------------------------------------------------------------------
	.section .noinit,"aw",@nobits
	.align	4
_mon_magic:
	.long	0
