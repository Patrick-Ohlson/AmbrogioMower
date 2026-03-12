.h8300s
.altmacro
;----------------------------------------------------------------------
;
;	ROM Monitor for the H8/300H- and H8S-Family
;
;	96/11/26	V1.0
;	97/06/20	V2.0	Protocol change
;	97/11/11	V3.0	Implementation of H8S
;				and protocol change
;	98/11/03	V3.1	Serial line communication version
;	99/01/27	V3.2	Address layout change
;	99/05/17	V3.3	SCI init now o.k. for H8S/21xx
;	99/06/08	V4.0	SCI init change
;				Implementation of processor-types 0x6, 0x7
;	99/07/26	V4.1	Implementation of address breaks with H8S/21xx
;	99/08/04	V4.2	Oncip PLL setting included
;	03/09/09	V4.3	EXR handling was wrong for processor-type 0x7
;
;	NOTE: This source file is mainly included for
;	      reference, changes are usually not required.
;
;	Monitor Variants:
;			(esi) defined: 8-bit ESI Monitor
;			(esi) and (bus16) defined: 16-bit ESI Monitor
;			(serial) defined: Serial-Line Monitor
;			(normal) defined: Vector-Table for normal addressing mode
;			(advanced) defined: Vector-Table for advanced addressing mode
;
;----------------------------------------------------------------------
;
;	Global Register Usage:
;
;	ER7	Stack
;	ER6	Communication Write Port
;	ER5	Status Port
;	ER4	Communication Read Port
;
;	R3H	Message ID
;	E3	Checksum
;
;	ER0	Temp
;
;----------------------------------------------------------------------

;----------------------------------------------------------------------
;  H8S/2144 Register Addresses (processor-type 0x5)
;
;  Cross-referenced with H8S/2144 Hardware Manual Appendix B.
;  See h8s2144_registers.md for bit-field details.
;  Names match datasheet register names wherever possible.
;
;  Fixes applied to original Ghidra export:
;    KBCOMP_BYTE  (0xFFFEE4) → IIC1_ICCR  (was IIC1 control, not KBC)
;    FLASH_EBR2   (0xFFFF83) → SYSCR2     (System Control 2, not flash)
;    INTC_wk1     (0xFFFEEE) → DTCERA     (DTC Enable Register A)
;    PB_DDR       (0xFFFFBE) → removed    (0xFFFFBE is P7PIN, not PB DDR)
;    PWMX_REGS*   (0xFFFFA0) → removed    (collided with SCI2, wrong chip)
;----------------------------------------------------------------------

;--- IIC Bus Interface (I2C) ---
; IIC0: 0xFFFEE0-E3 (not defined here — used via bit-banged vtable)
; IIC1: 0xFFFEE4-E7
.equ IIC1_ICCR,  0xFFFEE4  ; I2C Control 1 (ICE,IEIC,MST,TRS,CKS,BBSY,SCP)
                            ;   was KBCOMP_BYTE — actually IIC1, not keyboard

;--- Interrupt Controller ---
.equ ICRA,       0xFFFEE8  ; IRQ Control A (IRQ0-7 priority, 1=high)
.equ ICRB,       0xFFFEE9  ; IRQ Control B (DTC/WDT,ADC,FRT,TMR0/1,TMRXY,HIF)
.equ ICRC,       0xFFFEEA  ; IRQ Control C (SCI0/1/2, IIC0/1 priority)
.equ ISR,        0xFFFEEB  ; IRQ Status (IRQ7F-IRQ0F, write 0 to clear)
.equ ISCRH,      0xFFFEEC  ; IRQ Sense High (IRQ4-7: 00=low,01=fall,10=rise,11=both)
.equ ISCRL,      0xFFFEED  ; IRQ Sense Low  (IRQ0-3: same encoding)
.equ ISCR_WORD,  0xFFFEEC  ; IRQ Sense (word access = ISCRH:ISCRL)
.equ IER,        0xFFFFC2  ; IRQ Enable (IRQ7E-IRQ0E, 1=enabled)

;--- DTC (Data Transfer Controller) ---
.equ DTCERA,     0xFFFEEE  ; DTC Enable Register A
                            ;   was INTC_wk1 — misidentified as INTC padding

;--- Address Break (debug, used by Trace32 monitor) ---
.equ ABRKCR,     0xFFFEF4  ; Address Break Control Register
.equ BARA,       0xFFFEF5  ; Address Break Register A (address byte 2)
.equ BARB,       0xFFFEF6  ; Address Break Register B (address byte 1)
.equ BARC,       0xFFFEF7  ; Address Break Register C (address byte 0)

;--- Flash Memory ---
.equ FLMCR1,     0xFFFF80  ; Flash Control 1 (FWE,SWE,EV,PV,E,P)
.equ FLMCR2,     0xFFFF81  ; Flash Control 2 (FLER,ESU,PSU)
.equ EBR,        0xFFFF82  ; Erase Block Register (EB7-EB0)

;--- System Control ---
.equ SYSCR2,     0xFFFF83  ; System Control 2 (KWUL,P6PUE,SDE,CS4E,CS3E)
                            ;   was FLASH_EBR2 — not a flash register
.equ SBYCR,      0xFFFF84  ; Standby Control (SSBY,STS2-0,SCK2-0)
.equ LPWRCR,     0xFFFF85  ; Low Power Control (DTON,LSON,NESEL,EXCLE)
.equ MSTPCRH,    0xFFFF86  ; Module Stop High (DTC,ADC,FRT,TMR01,TMRXY,PWM,_,HIF)
.equ MSTPCRL,    0xFFFF87  ; Module Stop Low  (SCI0,SCI1,SCI2,IIC0,IIC1,KBC,_,_)
.equ MSTPCR_WORD,0xFFFF86  ; Module Stop (word access = MSTPCRH:MSTPCRL)
.equ STCR,       0xFFFFC3  ; Serial Timer Control (IICS,IICX,IICE,FLSHE)
.equ SYSCR,      0xFFFFC4  ; System Control (CS2E,IOSE,INTM,XRST,NMIEG,HIE,RAME)
.equ MDCR,       0xFFFFC5  ; Mode Control (R only: EXPE,MDS1,MDS0)

;--- SCI1 (Serial Comm Interface 1 — Trace32 monitor channel) ---
; Base 0xFFFF88.  BRR=8 → ~51200 baud (works at 57600 on this crystal).
.equ SCI1_SMR,   0xFFFF88  ; Serial Mode 1 (C/A,CHR,PE,O/E,STOP,MP,CKS)
.equ SCI1_BRR,   0xFFFF89  ; Bit Rate 1
.equ SCI1_SCR,   0xFFFF8A  ; Serial Control 1 (TIE,RIE,TE,RE,MPIE,TEIE,CKE)
.equ SCI1_TDR,   0xFFFF8B  ; Transmit Data 1
.equ SCI1_SSR,   0xFFFF8C  ; Serial Status 1 (TDRE,RDRF,ORER,FER,PER,TEND)
.equ SCI1_RDR,   0xFFFF8D  ; Receive Data 1
.equ SCI1_SCMR,  0xFFFF8E  ; Smart Card Mode 1

;--- FRT (16-bit Free Running Timer) ---
; Base 0xFFFF90.  Used for input capture (wire signal timing) in firmware.
.equ FRT_TIER,   0xFFFF90  ; Timer Interrupt Enable (ICIEA-D,OCIEA-B,OVIE)
.equ FRT_TCSR,   0xFFFF91  ; Timer Control/Status (ICFA-D,OCFA-B,OVF,CCLRA)
.equ FRT_FRC,    0xFFFF92  ; Free Running Counter (16-bit, R/W)
.equ FRT_OCRA,   0xFFFF94  ; Output Compare A (16-bit)
.equ FRT_TCR,    0xFFFF96  ; Timer Control (IEDGx,BUFEx,CKS)
.equ FRT_TOCR,   0xFFFF97  ; Timer Output Compare Control
.equ FRT_ICRA,   0xFFFF98  ; Input Capture A (16-bit, R)
.equ FRT_ICRB,   0xFFFF9A  ; Input Capture B (16-bit, R)
.equ FRT_ICRC,   0xFFFF9C  ; Input Capture C (16-bit, R)
.equ FRT_ICRD,   0xFFFF9E  ; Input Capture D (16-bit, R)

;--- SCI2 (Serial Comm Interface 2) ---
; Base 0xFFFFA0.  Same register layout as SCI0/SCI1.
; NOTE: Old PWMX_REGS* equates at 0xFFFFA0-A6 were WRONG —
;   they collided with SCI2.  PWM is at 0xFFFFD2-D6 (not defined here).
;   The PWMX equates were from a different H8S variant (DA converter).
.equ SCI2_SMR,   0xFFFFA0  ; Serial Mode 2
.equ SCI2_BRR,   0xFFFFA1  ; Bit Rate 2
.equ SCI2_SCR,   0xFFFFA2  ; Serial Control 2
.equ SCI2_TDR,   0xFFFFA3  ; Transmit Data 2
.equ SCI2_SSR,   0xFFFFA4  ; Serial Status 2
.equ SCI2_RDR,   0xFFFFA5  ; Receive Data 2
.equ SCI2_SCMR,  0xFFFFA6  ; Smart Card Mode 2

;--- WDT (Watchdog Timers) ---
; Special 16-bit write protocol: 0xA5xx→TCNT, 0x5Axx→TCSR
; Read: TCSR at base, TCNT at base+1
.equ WDT0_TCSR,  0xFFFFA8  ; WDT0 Control/Status (OVF,WT/IT,TME,_,RSTS,CKS2-0)
.equ WDT0_TCNT,  0xFFFFA9  ; WDT0 Counter (read); write via word at 0xFFFFA8
.equ WDT1_TCSR,  0xFFFFEA  ; WDT1 Control/Status
.equ WDT1_TCNT,  0xFFFFEB  ; WDT1 Counter (read); write via word at 0xFFFFEA

;--- Port A (PA0-PA3, open-drain) ---
.equ PAODR,      0xFFFFAA  ; Port A Output Data (W)
.equ PAPIN,      0xFFFFAB  ; Port A Pin read (R) / PADDR direction (W)

;--- Port I/O: Ports 1-6 ---
; DDR/DR are interleaved in pairs: (P1,P2), (P3,P4), (P5,P6).
; DDR is write-only; reading DDR address returns DR of the adjacent port.
; PCR = Pull-up Control Register (not in h8s2144_registers.md but in manual).
.equ P1PCR,      0xFFFFAC  ; Port 1 Pull-up Control (R/W)
.equ P2PCR,      0xFFFFAD  ; Port 2 Pull-up Control (R/W)
.equ P3PCR,      0xFFFFAE  ; Port 3 Pull-up Control (R/W)
.equ P1DDR,      0xFFFFB0  ; Port 1 Data Direction (W only)
.equ P2DDR,      0xFFFFB1  ; Port 2 Data Direction (W only)
.equ P1DR,       0xFFFFB2  ; Port 1 Data Register (R/W)
.equ P2DR,       0xFFFFB3  ; Port 2 Data Register (R/W)
.equ P3DDR,      0xFFFFB4  ; Port 3 Data Direction (W only)
.equ P4DDR,      0xFFFFB5  ; Port 4 Data Direction (W only)
.equ P3DR,       0xFFFFB6  ; Port 3 Data Register (R/W) — LCD data bus
.equ P4DR,       0xFFFFB7  ; Port 4 Data Register (R/W)
.equ P5DDR,      0xFFFFB8  ; Port 5 Data Direction (W only)
.equ P6DDR,      0xFFFFB9  ; Port 6 Data Direction (W only)
.equ P5DR,       0xFFFFBA  ; Port 5 Data Register (R/W)
.equ P6DR,       0xFFFFBB  ; Port 6 Data Register (R/W)

;--- Port B (PB0-PB7, open-drain) ---
.equ PBODR,      0xFFFFBC  ; Port B Output Data (W)
.equ PBPIN,      0xFFFFBD  ; Port B Pin read (R) — shared addr with P8DDR (W)
                            ;   was also PB_DDR at 0xFFFFBE — WRONG, 0xFFFFBE is P7PIN

;--- Port 7 (P70-P73, input-only — analog AN0-AN3) ---
.equ P7PIN,      0xFFFFBE  ; Port 7 Pin read (R only, analog input readback)

;--- Port 8 (P80-P86, keyboard scan) ---
; P8DDR write shares address 0xFFFFBD with PBPIN read.
.equ P8DDR,      0xFFFFBD  ; Port 8 Data Direction (W only, shared with PBPIN R)
.equ P8DR,       0xFFFFBF  ; Port 8 Data Register (R/W)

;--- Port 9 ---
.equ P9DDR,      0xFFFFC0  ; Port 9 Data Direction (W only)
.equ P9DR,       0xFFFFC1  ; Port 9 Data Register (R/W)

;--- Bus Control ---
.equ BCR,        0xFFFFC6  ; Bus Control (ICIS1,ICIS0,BRLE,WDBE)
.equ WSCR,       0xFFFFC7  ; Wait State Control (RAMS,RAM0,ABW,AST,WMS1-0,WC1-0)

;--- SCI0 (Serial Comm Interface 0) ---
; Base 0xFFFFD8.  Same register layout as SCI1/SCI2.
.equ SCI0_SMR,   0xFFFFD8  ; Serial Mode 0
.equ SCI0_BRR,   0xFFFFD9  ; Bit Rate 0
.equ SCI0_SCR,   0xFFFFDA  ; Serial Control 0
.equ SCI0_TDR,   0xFFFFDB  ; Transmit Data 0
.equ SCI0_SSR,   0xFFFFDC  ; Serial Status 0
.equ SCI0_RDR,   0xFFFFDD  ; Receive Data 0
.equ SCI0_SCMR,  0xFFFFDE  ; Smart Card Mode 0

;--- 8-bit Timer Ch0 / Ch1 ---
; Registers interleaved: even addr = ch0, odd addr = ch1.
.equ TMR0_TCR,   0xFFFFC8  ; Timer 0 Control (CMIEB,CMIEA,OVIE,CCLR1-0,CKS2-0)
.equ TMR1_TCR,   0xFFFFC9  ; Timer 1 Control
.equ TMR0_TCSR,  0xFFFFCA  ; Timer 0 Control/Status (CMFB,CMFA,OVF,ADTE,OS3-0)
.equ TMR1_TCSR,  0xFFFFCB  ; Timer 1 Control/Status
.equ TMR0_TCORA, 0xFFFFCC  ; Timer 0 Compare Match A
.equ TMR1_TCORA, 0xFFFFCD  ; Timer 1 Compare Match A
.equ TMR0_TCORB, 0xFFFFCE  ; Timer 0 Compare Match B
.equ TMR1_TCORB, 0xFFFFCF  ; Timer 1 Compare Match B
.equ TMR0_TCNT,  0xFFFFD0  ; Timer 0 Counter
.equ TMR1_TCNT,  0xFFFFD1  ; Timer 1 Counter

;--- A/D Converter (10-bit, 4-channel) ---
; Result in upper 10 bits of 16-bit register.
.equ ADDRA,      0xFFFFE0  ; A/D Data Register A (16-bit, ch0/AN0)
.equ ADDRB,      0xFFFFE2  ; A/D Data Register B (16-bit, ch1/AN1)
.equ ADDRC,      0xFFFFE4  ; A/D Data Register C (16-bit, ch2/AN2)
.equ ADDRD,      0xFFFFE6  ; A/D Data Register D (16-bit, ch3/AN3)
.equ ADCSR,      0xFFFFE8  ; A/D Control/Status (ADF,ADIE,ADST,SCAN,CKS,CH2-0)
.equ ADCR,       0xFFFFE9  ; A/D Control (TRGS1-0)

;--- 8-bit Timer Y (TMRY) ---
; Shares addresses with KBC registers (module-dependent):
;   TMRY_TCR / KBC_TPMR at 0xFFFFF0
;   TMRY_TCSR / KBC_KMIMR at 0xFFFFF1
;   TMRY_TCORA / KBC_P6PCR at 0xFFFFF2
;   TMRY_TCORB / KBC_KMIMRA at 0xFFFFF3
.equ TMRY_TCR,   0xFFFFF0  ; Timer Y Control
.equ TMRY_TCSR,  0xFFFFF1  ; Timer Y Control/Status (alias: KMIMR for KBC)
.equ TMRY_TCORA, 0xFFFFF2  ; Timer Y Compare Match A (alias: P6PCR for KBC)
.equ TMRY_TCORB, 0xFFFFF3  ; Timer Y Compare Match B (alias: KMIMRA for KBC)
.equ TMRY_TCNT,  0xFFFFF4  ; Timer Y Counter
.equ TMRY_TISR,  0xFFFFF5  ; Timer Y Interrupt Status

;--- D/A Converter (8-bit, 2-channel) ---
.equ DADR0,      0xFFFFF8  ; D/A Data Register 0
.equ DADR1,      0xFFFFF9  ; D/A Data Register 1
.equ DACR,       0xFFFFFA  ; D/A Control (DAOE1,DAOE0,DAE)

;--- Unknown ---
.equ unk_FFFF00, 0xFFFF00  ; unmapped (HIF region? needs investigation)

;--- Interrupt Vector Addresses ---
; Vector# x 4 = address.  H8S/2144 has 104 vectors (0x000-0x19C).
.equ VEC_TRAP0,  0x20       ; TRAP #0 (vector 8)
.equ VEC_TRAP2,  0x28       ; TRAP #2 (vector 10)
.equ VEC_TRAP3,  0x2C       ; TRAP #3 (vector 11)
.equ VEC_IRQ7,   0x5C       ; IRQ7    (vector 23)


#define serial
#define advanced

#ifdef INTEGRATED
    .global _mon_start
    .global _mon_breakp
    .global _mon_trap
    .global _mon_breakp0

    .extern _lcd_SetRow
    .extern _lcd_WriteString
    .extern _mon_protocol_loop

    .global _mon_go_regs
    .global _mon_go_asm
    .global _mon_saved_sp
#endif

.macro prepcom					; prepare communication port
#ifdef	serial
	LOCAL	l7, l8, l9, l10, s0
	mov.b	@mon_proctype,r0l
	cmp.b	#5,r0l				; H8S/21xx? (proctype 5, incl. 2144)
	beq	l7:8
	cmp.b	#6,r0l				; H8S/222x/223x? (proctype 6)
	beq	l9:8
	cmp.b	#7,r0l				; H8S/262x-264x? (proctype 7)
	beq	l8:8
	bra	l10:8				; H8S/265x/23xx/224x (proctypes 2-4)

	; --- H8S/21xx (proctype 5, incl. 2144) ---
	; MSTPCRL bits: b7=SCI0, b6=SCI1, b5=SCI2, b4=IIC0, b3=IIC1, b2=KBC
	; Clear bit (7 - channel) to enable the selected SCI module clock.
l7:	mov.b	@mon_scichannel,r2l
	mov.b	#7,r0l
	sub.b	r2l,r0l				; r0l = 7 - channel = MSTPCRL bit#
	bclr	r0l,@MSTPCRL			; clear module-stop → enable SCI clock
	bra	s0:8

	; --- H8S/262x-264x (proctype 7) — PLL setup then fall through to l9 ---
	; NOTE: addresses below are for H8S/262x, NOT H8S/2144.
l8:	mov.b	@mon_pllfactor,r0l
	mov.b	@0xfffdec,r0h			; LPWRCR (H8S/262x @ 0xFFFDEC)
	or.b	r0l,r0h
	mov.b	r0h,@0xfffdec			; set PLL multiplier in LPWRCR
	mov.l	#0xfffde6,er0			; PLL control reg (H8S/262x @ 0xFFFDE6)
	bset	#3,@er0				; validate new PLL factor
	; fall through to l9 (proctypes 6+7 share SCI enable path)

	; --- H8S/222x-264x (proctypes 6, 7) — SCI enable ---
l9:	mov.b	@mon_scichannel,r2l
	mov.b	#7,r0l
	sub.b	r2l,r0l				; r0l = 7 - channel = bit#
	bclr	r0l,@0xfffde9			; MSTPCRL (H8S/222x/262x @ 0xFFFDE9)
	bra	s0:8

	; --- H8S/265x/23xx/224x (proctypes 2-4) — SCI enable ---
l10:	mov.b	@mon_scichannel,r0l
	add.b	#5,r0l				; bit# = channel + 5
	bclr	r0l,@0xffff3d			; MSTPCR (H8S/265x/23xx/224x @ 0xFFFF3D)

	; --- Common SCI register setup ---
	; scibase = SMR address.  Register layout: +0=SMR +1=BRR +2=SCR
	;   +3=TDR +4=SSR +5=RDR.  Global regs: er4=RDR er5=SSR er6=TDR.
s0:	mov.l	@mon_scibase,er2			; er2 = SCI base (→ SMR)
	mov.l	er2,er1
	add.b	#2,r1l				; er1 → SCR (base+2)
	bclr	#4,@er1				; SCR.RE = 0 (disable receiver)
	bclr	#5,@er1				; SCR.TE = 0 (disable transmitter)
	mov.b	#0x30,r0l   ;was 0 patrick
	mov.b	r0l,@er1			; SCR = 0x30 (TE+RE, clears TIE/RIE/CKE)
	mov.b	#0x0,r0l     ;patrick
	mov.b	r0l,@er2			; SMR = 0 (async 8N1, Pφ/1)
	mov.l	er2,er1
	add.b	#1,r1l				; er1 → BRR (base+1)
	mov.b	@mon_scibrr,r0l			; load baud rate divisor
	mov.b	r0l,@er1			; set BRR
	mov.l	er2,er4
	add.b	#5,r4l				; er4 → RDR (base+5) — global read port
	mov.l	er2,er5
	add.b	#4,r5l				; er5 → SSR (base+4) — global status port
	mov.l	er2,er6
	add.b	#3,r6l				; er6 → TDR (base+3) — global write port
	add.b	#2,r2l				; er2 → SCR (base+2)
	bset	#4,@er2				; SCR.RE = 1 (enable receiver)
#endif

.endm

;-------------------------------

.macro flushread						; flush input port
	LOCAL	skip
#ifdef	serial
	btst	#6,@er5			; RDRF=0 ?
	beq	skip\@:8
	mov.b	@er4,r0l			; read RDR
	bclr	#6,@er5			; clear RDRF
skip\@:	nop
#endif


.endm
	
;-------------------------------

.macro waitread						; wait for input
	LOCAL	loop
#ifdef	esi
loop\@:	btst	#0,@er5
	beq	loop\@:8
#endif
.endm
	
;-------------------------------

.macro readbyte						; input one character from host
	LOCAL	loop
#ifdef	serial
loop\@:	btst	#6,@er5			; RDRF=0 ?
	beq	loop\@:8
	mov.b	@er4,r0l			; read RDR
	bclr	#6,@er5			; clear RDRF
#endif


.endm
	
;-------------------------------

.macro writebyte						; output one character to host
	LOCAL	loop1, loop2
#ifdef	serial
	mov.b	#3,r0h
	sub.b	r0h,r4l
	bset	#5,@er4			; set TE
loop1\@:	btst	#7,@er5			; TDRE=0 ?
	beq	loop1\@:8
	mov.b	r0l,@er6			; write TDR
	bclr	#7,@er5			; clear TDRE
loop2\@:	btst	#2,@er5			; TEND=0 ?
	beq	loop2\@:8
	bclr	#5,@er4			; clear TE
	add.b	r0h,r4l
#endif


.endm
	
;-------------------------------

.macro waitwrite						; prepare to output one character to host
	LOCAL	loop

.endm

;-------------------------------

.macro prepgo						; prepare cpu before start of emulation
	LOCAL	ac2, ac5, p20, p21, p22, p60, p61, p62, e
#ifdef	serial
	mov.b	@mon_proctype,r1l
	cmp.b	#5,r1l				; H8S/21xx? (proctype 5, incl. 2144)
	beq	ac5\@:8
	cmp.b	#6,r1l				; H8S/222x/223x? (proctype 6)
	beq	p60\@:8
	cmp.b	#7,r1l				; H8S/262x-264x? (proctype 7)
	beq	p60\@:8
	bra	ac2\@:8				; H8S/265x/23xx/224x (proctypes 2-4)

	; --- H8S/21xx (proctype 5, incl. 2144) ---
	; ICRC bits: b7=SCI0, b6=SCI1, b5=SCI2, b4=IIC0, b3=IIC1
	; Setting bit = high priority.  bit# = 7 - channel.
ac5\@:	mov.b	@mon_scichannel,r0h
	mov.b	#7,r0l
	sub.b	r0h,r0l				; r0l = 7 - channel = ICRC bit#
	bset	r0l,@ICRC			; set SCI interrupt to high priority
	bra	e\@:16

	; --- H8S/265x/23xx/224x (proctypes 2-4) ---
	; NOTE: addresses below are for H8S/265x, NOT H8S/2144.
	; These chips use IPR registers for 3-level interrupt priority.
ac2\@:	mov.b	@mon_scichannel,r0h
	mov.b	#4,r0l
	sub.b	r0h,r0l
	bset	r0l,@0xfffec2			; IPRA (H8S/265x @ 0xFFFEC2) — SCI high
p20\@:	cmp.b	#0,r0h
	bne	p21\@
	mov.l	#0xfffecd,er1			; IPRG (H8S/265x @ 0xFFFECD)
	bset	#0,@er1
	bset	#1,@er1
	bset	#2,@er1				; set SCI0 interrupt priority to 7
	bra	e\@:16
p21\@:	cmp.b	#1,r0h
	bne	p22\@
	mov.l	#0xfffece,er1			; IPRH (H8S/265x @ 0xFFFECE)
	bset	#4,@er1
	bset	#5,@er1
	bset	#6,@er1				; set SCI1 interrupt priority to 7
	bra	e\@:8
p22\@:	mov.l	#0xfffece,er1			; IPRH (H8S/265x @ 0xFFFECE)
	bset	#0,@er1
	bset	#1,@er1
	bset	#2,@er1				; set SCI2 interrupt priority to 7
	bra	e\@:8

	; --- H8S/222x-264x (proctypes 6, 7) ---
	; NOTE: addresses below are for H8S/262x, NOT H8S/2144.
p60\@:	mov.l	#0xfffec4,er1			; IPRE (H8S/262x @ 0xFFFEC4)
	bset	#4,@er1
	bset	#5,@er1
	bset	#6,@er1				; set PBC interrupt priority to 7
	bclr	#4,@0xfffdea			; PBCR1 (H8S/262x @ 0xFFFDEA) — enable PBC
	mov.b	@mon_scichannel,r0h
	cmp.b	#0,r0h
	bne	p61\@
	mov.l	#0xfffec9,er1			; IPRI (H8S/262x @ 0xFFFEC9)
	bset	#0,@er1
	bset	#1,@er1
	bset	#2,@er1				; set SCI0 interrupt priority to 7
	bra	e\@:8
p61\@:	cmp.b	#1,r0h
	bne	p62\@
	mov.l	#0xfffeca,er1			; IPRJ (H8S/262x @ 0xFFFECA)
	bset	#4,@er1
	bset	#5,@er1
	bset	#6,@er1				; set SCI1 interrupt priority to 7
	bra	e\@:8
p62\@:	mov.l	#0xfffeca,er1			; IPRJ (H8S/262x @ 0xFFFECA)
	bset	#0,@er1
	bset	#1,@er1
	bset	#2,@er1				; set SCI2 interrupt priority to 7
	bra	e\@:8

	; --- Common: enable SCI receive interrupt ---
e\@:	mov.l	@mon_scibase,er0			; er0 = SCI base (→ SMR)
	add.b	#2,r0l				; er0 → SCR (base+2)
	bset	#6,@er0				; SCR.RIE = 1 (enable receive interrupt)
#endif


.endm


;----------------------------------------------------------------------
;
;	Interrupt Vector Table
;
;----------------------------------------------------------------------

#ifndef INTEGRATED
	//.org	0x0



#ifdef	advanced_NOTUSED
	.rept	2
	.long	_mon_start				; Power-on and Manual Reset
	.endr

	.rept	5
	.long	_mon_trap
	.endr

	.rept	5
	.long	_mon_breakp				; Entry to Monitor (NMI, TRAPA)
	.endr

	.rept	15
	.long	_mon_trap
	.endr

	.long	_mon_breakp				; Entry to Monitor (PBC and Address-Breaks of H8S/21xx)

	.rept	52
	.long	_mon_trap
	.endr

	.rept	3
	.long	_mon_trap
	.long	_mon_breakp				; Entry to Monitor (RXI)
	.long	_mon_trap
	.long	_mon_trap
	.endr
#endif
#endif /* !INTEGRATED */

;----------------------------------------------------------------------
;
;	Entry Jump Table
;
;----------------------------------------------------------------------

#ifndef INTEGRATED
	//.org	0x200
#endif

_mon_start:	mov.b	#0x40,r3h		; entry by reset
#ifdef INTEGRATED
	; LCD debug: show "MON:START" on row 1
	; Save r3h (entry reason) — C calls clobber er0-er3
	push.l	er3
	mov.b	#1,r0l
	jsr	@_lcd_SetRow
	mov.l	#mon_lcd_start,er0
	jsr	@_lcd_WriteString
	pop.l	er3
#endif
	bra	mon_startr:16
	

#ifndef INTEGRATED
	//.org	0x280
#endif

_mon_breakp:	push.l	er0
	mov.l	#0x1000,er0
	push.l	er0
	bra	_mon_breakp0:16			; entry by breakpoint
	
	
#ifndef INTEGRATED
	//.org	0x300
#endif

_mon_trap:	push.l	er0
	mov.l	#0x3000,er0
	push.l	er0
	bra	_mon_breakp0:16			; entry by trap
	
	
;----------------------------------------------------------------------
;
;	Monitor Configuration Table
;
;----------------------------------------------------------------------
     .align	4
#ifndef INTEGRATED
	//.org	0x330
#endif

mon_proctype:	.byte	0x5			; processor type (hex)
						; 0x1: H8/300H
						; 0x2: H8S/265x
						; 0x3: H8S/23xx
						; 0x4: H8S/224x
						; 0x5: H8S/21xx
						; 0x6: H8S/222x/223x
						; 0x7: H8S/262x/263x/264x
mon_imask0:	.byte	0x3				; meaning of bits:
						; bit0 = I
						; bit1 = UI
mon_imask1:	.byte	0x7				; meaning of bits:
						; bit2-0: I2-0
	.byte	0x0				; reserved

mon_scichannel:	.byte	0x1			; SCI Channel:
						; 0x0: Channel 0
						; 0x1: Channel 1
						; 0x2: Channel 2
mon_scibrr:	.byte	0x08			; BRR-Register of SCI → 57600 baud
						; Must match InitSci2() BRR=8.
						; Lauterbach defaults (for reference):
						; 0x0a for EVB2655 (57.6kbit/s)
						; 0x09 for EVB2357F (57.6kbit/s)
						; 0x09 for EVB2144F (57.6kbit/s)
						; 0x0a for EVB2623F (57.6kbit/s)
						; 0x05 for EVB2329F (115.2kbit/s)
	.byte	0x0				; reserved
	.byte	0x0				; reserved

mon_scibase:	.long	0xffff88		; base address of the SCI registers
						; for the corresponding channel
;On-chip PLL multiplication factor (0x33C)
;0x0 = PLL x1 (default)
;0x1 = PLL x2
;0x2 = PLL x4
;0x3 = PLL off
mon_pllfactor:	.byte	0x3			; Specifies the multiplication factor
						; of the onchip PLL for the corresponding
						; CPUs. The monitor sets the LPWRCR to
						; this value in the serial-line mode

;----------------------------------------------------------------------
;
;	Monitor Main Program
;
;	Functions:
;
;	1	read memory
;	2	write memory
;	3	read shadow registers
;	4	write shadow registers
;	5	go
;	6	step
;	7	go to RAM for hot patch
;
;----------------------------------------------------------------------
	 .align	4
#ifndef INTEGRATED
	//.org	0x500
#endif

mon_startr:	nop

mon_start0:						; prepare communication port
	prepcom

	; === HYBRID TEST: call C protocol handler instead of assembly loop ===
	; r3h = entry_reason (0x10=breakp, 0x30=trap, 0x40=reset)
	; er7 = stack frame pointer (mon_frame_t) — set by _mon_breakp0
	; Prepcom has set er2=SCR, er4=RDR, er5=SSR, er6=TDR — C uses absolutes
	mov.l	er7,er0				; arg1 = frame pointer
	sub.l	er1,er1
	mov.b	r3h,r1l				; arg2 = entry_reason
	jsr	@_mon_protocol_loop
.Lhybrid_hang:
	bra	.Lhybrid_hang:8

	; ---- Original assembly protocol loop below (now dead code) ----
	cmp.b	#0x10,r3h
	beq	mon_start2:8			; no burst after breakpoint

mon_start1:	mov.b	#0xff,r0l			; send burst to host to flush
						; communication receiver
	writebyte
	dec.b	r1l				; decrement counter
	bne	mon_start1:8
	
mon_start2:						; flush input buffer from host
	flushread
	
mon_mainlp2:


	mov.b	#0x0,r0l
	writebyte

    mov.b	#0x20,r0l			; message prepare to host patrick
    ;mov.b	#0x10,r0l			; message prepare to host
	writebyte
	
	readbyte
	cmp.b	#0x3f,r0l
	bne	mon_mainlp:16			; host ready to receive ?
	
	mov.w	#0x0,e3
	mov.w	#0x0,r0			; send message size (=0)
	add.w	r0,e3
	writebyte
	
	mov.b	r3h,r0l			; send message id and type
	extu.w	r0
	add.w	r0,e3
	writebyte
	
mon_mainlp1:	mov.w	e3,r0
	mov.b	r0h,r0l			; checksum high
	writebyte
	mov.w	e3,r0				; checksum low
	writebyte
	
mon_mainlp:	mov.w	#0x0,e3
	readbyte
	cmp.b	#0x20,r0l ; was 20 patrick
	bne	mon_mainlp:8
	
	mov.b	#0x3f,r0l
	writebyte
	
	readbyte
	extu.w	r0
	add.w	r0,e3				; lenght of message
	
	readbyte
	extu.w	r0
	add.w	r0,e3
	and.b	#0x0f,r0l
	mov.b	r0l,r3h			; id of message
	
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r3l			; command-type to R3L
	
	cmp.b	#0x0,r0l
	beq	goram:16			; NOP
	cmp.b	#0x7,r0l
	beq	goram:16			; run to RAM
	cmp.b	#0x7,r0l
	bgt	mon_mainlp:16			; out of range
	
	readbyte
	extu.w	r0
	add.w	r0,e3				; read memory access class
	
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.w	r0,e1				; memory access width to E1
	
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r2h
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r2l
	mov.w	r2,e2
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r2h
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r2l			; memory/buffer address to ER2
	
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.w	r0,r1				; read lenght of data block
	cmp.w	#0x00ff,r1
	beq	mon_mainlp:16			; ignore invalid packets

	readbyte
	extu.w	r0
	mov.w	e3,e0
	extu.w	e0
	cmp.w	e0,r0				; parameter checksum
	bne	mon_mainlp:16
	add.w	r0,e3
	
	
;----------------------------------------------------------------------
;
;	block oriented memory and register operations
;
;	parameters:
;
;	E1	Access Width
;	R3L	Command (Function)
;	R1	Lenght of Data Block
;	R2	Memory/Buffer Address
;
;----------------------------------------------------------------------

	cmp.b	#0x3,r3l
	beq	readreg:16			; read CPU registers
	cmp.b	#0x4,r3l
	beq	writereg:16			; write CPU registers
	cmp.b	#0x5,r3l
	beq	mon_go:16			; go
	cmp.b	#0x6,r3l
	beq	mon_go:16			; step
	cmp.b	#0x2,r3l
	beq	wr:16				; write memory


;----------------------------------------------------------------------
;
;	Read Memory
;
;----------------------------------------------------------------------

rd:
	readbyte
	mov.b	r0l,r0h
	xor.w	e3,r0
	cmp.b	#0x0,r0h			; checksum 1
	bne	mon_mainlp:16
	
	readbyte
	xor	e3,r0
	cmp.b	#0x0,r0l			; checksum 2
	bne	mon_mainlp:16
	
	mov.b	#0x20,r0l
	writebyte
	
	readbyte
	cmp.b	#0x3f,r0l
	bne	mon_mainlp:16
	
	mov.w	#0x0,e3
	mov.w	r1,r0				; send lenght of data block
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.b	r3h,r0l			; send message id
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.w	e1,r0
	btst	#2,r0l
	bne	readl:16
	btst	#1,r0l
	bne	readw:8
	
readloopb:	mov.b	@er2,r0l			; read byte
	extu.w	r0
	add.w	r0,e3
	writebyte
	inc.l	#1,er2				; increment address
	dec.w	#1,r1				; decrement block lenght
	bne	readloopb:8
	bra	mon_mainlp1:16
	
readw:	btst	#0,r2l
	bne	readloopb
	btst	#0,r1l
	bne	readloopb
readloopw:	mov.w	@er2,r0			; read word
	mov.b	r0h,r0l
	extu.w	r0
	add.w	r0,e3
	writebyte
	mov.w	@er2,r0
	extu.w	r0
	add.w	r0,e3
	writebyte
	inc.l	#2,er2
	dec.w	#2,r1
	bne	readloopw:8
	bra	mon_mainlp1:16
	
readl:	btst	#1,r2l
	bne	readw
	btst	#0,r2l
	bne	readw
	btst	#1,r1l
	bne	readw
	btst	#0,r1l
	bne	readw
readloopl:	mov.l	@er2,er0			; read long
	mov.w	e0,r0
	mov.b	r0h,r0l
	extu.w	r0
	add.w	r0,e3
	writebyte
	mov.l	@er2,er0
	mov.w	e0,r0
	extu.w	r0
	add.w	r0,e3
	writebyte
	mov.l	@er2,er0
	mov.b	r0h,r0l
	extu.w	r0
	add.w	r0,e3
	writebyte
	mov.l	@er2,er0
	extu.w	r0
	add.w	r0,e3
	writebyte
	inc.l	#2,er2
	inc.l	#2,er2
	dec.w	#2,r1
	dec.w	#2,r1
	bne	readloopl:16
	bra	mon_mainlp1:16


;----------------------------------------------------------------------
;
;	Write Memory
;
;----------------------------------------------------------------------

wr:	mov.w	e1,r0
	btst	#2,r0l
	bne	writel:16
	btst	#1,r0l
	bne	writew:8
	
writeloopb:
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,@er2			; write byte
	inc.l	#1,er2
	dec.w	#1,r1
	bne	writeloopb:8
	bra	mon_mainlp3:16
	
writew:	btst	#0,r2l
	bne	writeloopb
	btst	#0,r1l
	bne	writeloopb
writeloopw:
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r0h
	readbyte
	mov.w	r0,e0
	extu.w	r0
	add.w	r0,e3
	mov.w	e0,@er2			; write word
	inc.l	#2,er2
	dec.w	#2,r1
	bne	writeloopw:8
	bra	mon_mainlp3:16
	
writel:	btst	#1,r2l
	bne	writew
	btst	#0,r2l
	bne	writew
	btst	#1,r1l
	bne	writew
	btst	#0,r1l
	bne	writew
writeloopl:
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r0h
	readbyte
	mov.w	r0,e0
	extu.w	r0
	add.w	r0,e3
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r0h
	readbyte
	mov.l	er0,@er2			; write long
	extu.w	r0
	add.w	r0,e3
	inc.l	#2,er2
	inc.l	#2,er2
	dec.w	#2,r1
	dec.w	#2,r1
	bne	writeloopl:8
	
mon_mainlp3:
	readbyte
	mov.b	r0l,r0h
	xor.w	e3,r0
	cmp.b	#0x0,r0h			; checksum 1
	bne	mon_mainlp:16
	
	readbyte
	xor	e3,r0
	cmp.b	#0x0,r0l			; checksum 2
	bne	mon_mainlp:16
	
	bra	mon_mainlp2:16
	
	
checkandreply:
	readbyte
	mov.b	r0l,r0h
	xor.w	e3,r0
	cmp.b	#0x0,r0h			; checksum 1
	bne	mon_mainlp:16
	
	readbyte
	xor	e3,r0
	cmp.b	#0x0,r0l			; checksum 2
	bne	mon_mainlp:16

	mov.b	#0x20,r0l
	writebyte
	
	readbyte
	cmp.b	#0x3f,r0l
	bne	mon_mainlp:16
	
	mov.w	#0x0,e3
	mov.w	r1,r0				; send lenght of data block
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.b	r3h,r0l			; send message id
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	jmp	@er2
	
	
;----------------------------------------------------------------------
;
;	Read Register
;
;----------------------------------------------------------------------

readreg:	mov.l	#readreg1,er2
	bra	checkandreply:16
	
readreg1:	cmp.b	#36,r1l
	beq	readregpc:16

readreg01:	mov.l	er7,er1
	mov.l	#readreg02,er2
	bra	writelong:16			; read stack-pointer (ER7)
	
readreg02:	mov.b	@mon_proctype,r0l
	cmp.b	#2,r0l				; H8S/265x ?
	beq	readreg021:8
	cmp.b	#7,r0l				; H8S/262x/263x/264x ?
	beq	readreg021:8
	bra	readreg05:8
	
readreg021:	stmac	mach,er1
	mov.l	#readreg03,er2
	bra	writelong:16			; read MACH
	
readreg03:	stmac	macl,er1
	mov.l	#readreg04,er2
	bra	writelong:16			; read MACL
	
readreg04:	bra	mon_mainlp1:16
	
readreg05:	mov.l	#0x0,er1
	mov.l	#readreg06,er2
	bra	writelong:16			; read dummy MACH
	
readreg06:	mov.l	#0x0,er1
	mov.l	#readreg07,er2
	bra	writelong:16			; read dummy MACL

readreg07:	bra	mon_mainlp1:16	

writelong:	mov.w	e1,r0				; operand in ER1
	mov.b	r0h,r0l
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.w	e1,r0
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.b	r1h,r0l
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.b	r1l,r0l
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	jmp @er2


readregpc:	mov.l	#readregpc01,er2
	pop.l	er1
	bra	writelong:16			; read ER6
	
readregpc01:	mov.l	#readregpc02,er2
	pop.l	er1
	bra	writelong:16			; read ER5
	
readregpc02:	mov.l	#readregpc03,er2
	pop.l	er1
	bra	writelong:16			; read	ER4
	
readregpc03:	mov.l	#readregpc04,er2
	pop.l	er1
	bra	writelong:16			; read ER3
	
readregpc04:	mov.l	#readregpc05,er2
	pop.l	er1
	bra	writelong:16			; read ER2
	
readregpc05:	mov.l	#readregpc06,er2
	pop.l	er1
	bra	writelong:16			; read ER1
	
readregpc06:	inc.l	#2,er7				; jump over message id
	inc.l	#2,er7
	mov.l	#readregpc07,er2
	pop.l	er1
	bra	writelong:16			; read ER0

readregpc07:	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H ?
	beq	readregpc09:8
	cmp.b	#4,r0l				; H8S/224x ?
	beq	readregpc09:8
	cmp.b	#5,r0l				; H8S/21xx ?
	beq	readregpc09:8
	bra	readregpc10:8
	
readregpc09:	mov.l	#0x0,er1
	mov.l	#readregpc11,er2
	bra	writelong:16			; read dummy EXR
	
readregpc10:	mov.b	@mon_proctype,r0l
	cmp.b	#6,r0l				; H8S/222x/223x ?
	beq	readregpc108:8
	cmp.b	#7,r0l				; H8S/262x/263x/264x ?
	beq	readregpc108:8
readregpc107:	mov.l	@SYSCR0,er1
	bra	readregpc109:8
readregpc108:	mov.l	@SYSCR1,er1
readregpc109:	mov.b	@er1,r0l
	shlr.b	r0l
	shlr.b	r0l
	shlr.b	r0l
	shlr.b	r0l
	and.b	#0x3,r0l			; interrupt control mode ?
	cmp.b	#1,r0l
	bgt	readregpc10a:8		; mode 2 or 3
	bra	readregpc09:8			; read dummy EXR (mode 0 or 1)
	
readregpc10a:	mov.l	#readregpc11,er2
	pop.w	r1
	mov.b	#0x0,r1l
	mov.w	#0x0,e1
	bra	writelong:16			; read EXR

readregpc11:	mov.l	#readregpc12,er2
	pop.l	er1
	bra	writelong:16			; read CCR and PC

readregpc12:	bra	mon_mainlp1:16

	.ALIGN 2
SYSCR0:	.long	0xffff39
SYSCR1:	.long	0xfffde5


;----------------------------------------------------------------------
;
;	Write Register
;
;----------------------------------------------------------------------

; writereg — Write a single CPU register.
;
; Entry:  er2 = register number, r1l = data length (must be 4)
;
; Register numbering (matches readreg output order / Trace32 protocol):
;   0 = ER7/SP   3 = ER6    6 = ER3    9 = ER0
;   1 = MACH     4 = ER5    7 = ER2   10 = EXR
;   2 = MACL     5 = ER4    8 = ER1   11 = CCR+PC
;
; Stack frame at this point (built by _mon_breakp0 + hardware exception):
;   SP+0  = saved ER6        SP+16 = saved ER2
;   SP+4  = saved ER5        SP+20 = saved ER1
;   SP+8  = saved ER4        SP+24 = reason code (4 bytes)
;   SP+12 = saved ER3        SP+28 = saved ER0
;                             SP+32 = saved CCR+PC (H8S/2144, no EXR)

writereg:	cmp.b	#0x0,r1l
	beq	mon_mainlp3:16			; length=0 → no data, just checksum

	cmp.b	#0x4,r1l
	bne	mon_mainlp:16			; illegal packet length → resync

	; Read 4-byte register value into ER0 (high word in E0, low in R0)
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r1h
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r1l
	mov.w	r1,e0
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r1h
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r1h,r0h			; register-value now in ER0

	; Range check AFTER reading value (prevents protocol desync)
	cmp.l	#12,er2
	bge	writeregfinish:16		; out of range → ignore, do checksum

writereg1:	mov.l	er2,er1			; nr. of writereg to ER1
	mov.l	#writeregjumptab,er2
	shll.l	er1
	shll.l	er1				; x4
	add.l	er1,er2
	mov.l	@er2,er1
	mov.l	#writeregjumptab,er2
	add.l	er1,er2
	jmp	@er2

	.ALIGN	2
writeregjumptab:
	.long	writereger7-writeregjumptab	; 0: ER7/SP
	.long	writeregmach-writeregjumptab	; 1: MACH
	.long	writeregmacl-writeregjumptab	; 2: MACL
	.long	writereger6-writeregjumptab	; 3: ER6  → stack SP+0
	.long	writereger5-writeregjumptab	; 4: ER5  → stack SP+4
	.long	writereger4-writeregjumptab	; 5: ER4  → stack SP+8
	.long	writereger3-writeregjumptab	; 6: ER3  → stack SP+12
	.long	writereger2-writeregjumptab	; 7: ER2  → stack SP+16
	.long	writereger1-writeregjumptab	; 8: ER1  → stack SP+20
	.long	writereger0-writeregjumptab	; 9: ER0  → stack SP+28 (skip reason)
	.long	writeregexr-writeregjumptab	; 10: EXR  → no-op on 2144
	.long	writeregpc-writeregjumptab	; 11: CCR+PC → stack SP+32

	.ALIGN	2
writeregfinish:
	bra	mon_mainlp3:16

writereger7:	mov.l	er0,er7
	bra	writeregfinish:8

writeregmach:	mov.b	@mon_proctype,r1l
	cmp.b	#2,r1l				; H8S/265x
	beq	writeregmach1:8
	cmp.b	#7,r1l				; H8S/262x/263x/264x
	beq	writeregmach1:8
	bra	writeregfinish:8		; no MACH on 2144
writeregmach1:
	ldmac	er0,mach
	bra	writeregfinish:8

writeregmacl:	mov.b	@mon_proctype,r1l
	cmp.b	#2,r1l				; H8S/265x
	beq	writeregmacl1:8
	cmp.b	#7,r1l				; H8S/262x/263x/264x
	beq	writeregmacl1:8
	bra	writeregfinish:8		; no MACL on 2144
writeregmacl1:
	ldmac	er0,macl
	bra	writeregfinish:8

; Write ER6-ER1: modify saved values on the stack.
; Offsets are from current SP to the saved register locations.
writereger6:	mov.l	er0,@(0,er7)
	bra	writeregfinish:8
writereger5:	mov.l	er0,@(4,er7)
	bra	writeregfinish:8
writereger4:	mov.l	er0,@(8,er7)
	bra	writeregfinish:8
writereger3:	mov.l	er0,@(12,er7)
	bra	writeregfinish:8
writereger2:	mov.l	er0,@(16,er7)
	bra	writeregfinish:8
writereger1:	mov.l	er0,@(20,er7)
	bra	writeregfinish:8

; Write ER0: saved at SP+28 (skip 4-byte reason code at SP+24)
writereger0:	mov.l	er0,@(28,er7)
	bra	writeregfinish:8

; Write EXR: H8S/2144 has no EXR on stack (interrupt mode 0) — ignore.
; Chips with EXR (265x/262x) would write to stack here.
writeregexr:	bra	writeregfinish:8

; Write CCR+PC: saved at SP+32 by hardware exception frame.
; ER0 contains the combined CCR (high byte) + PC (24-bit address).
writeregpc:	mov.l	er0,@(32,er7)
	bra	writeregfinish:8

readlong:
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r1h
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r1l
	mov.w	r1,e0
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r0l,r1h
	readbyte
	extu.w	r0
	add.w	r0,e3
	mov.b	r1h,r0h			; longword in ER0
	jmp	@er2


;----------------------------------------------------------------------
;
;	Start Target Program
;
;----------------------------------------------------------------------

mon_go:	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H ?
	beq	go1:8
	cmp.b	#5,r0l				; H8S/21xx ?
	beq	go1:8
	cmp.b	#4,r0l				; H8S/224x ?
	beq	go1:8
	cmp.b	#2,r0l				; H8S/2655 ?
	beq	go00:8
	cmp.b	#3,r0l				; H8S/23xx ?
	beq	go00:8
	cmp.b	#6,r0l				; H8S/222x/223x ?
	beq	go01:8
	cmp.b	#7,r0l				; H8S/262x/263x/264x ?
	beq	go01:8
go00:	mov.l	@SYSCR0,er1
	bra	go09:8
go01:	mov.l	@SYSCR1,er1
go09:	mov.b	@er1,r0l
	shlr.b	r0l
	shlr.b	r0l
	shlr.b	r0l
	shlr.b	r0l
	and.b	#0x3,r0l
	extu.w	r0
	mov.w	r0,e1				; store interrupt control mode

go1:	mov.l	#go11,er2
	bra	readlong:16			; CCR and PC
go11:	push.l	er0

	mov.l	#go12,er2
	bra	readlong:16
go12:	push.w r0				; EXR

	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H ?
	beq	go13:8
	cmp.b	#5,r0l				; H8S/21xx ?
	beq	go13:8
	cmp.b	#4,r0l				; H8S/224x ?
	beq	go13:8
	mov.w	e1,r0
	cmp.b	#2,r0l
	blt	go13:8				; int-ctrl mode 0 or 1
	bra	go14:8
	
go13:	add.l	#2,er7				; correct stackpointer if no EXR

go14:	mov.l	#go15,er2
	bra	readlong:16			; ER0
go15:	push.l	er0

	mov.l	#go16,er2
	bra	readlong:16			; ER1
go16:	push.l	er0

	mov.l	#go17,er2
	bra	readlong:16			; ER2
go17:	push.l	er0

	mov.l	#go18,er2
	bra	readlong:16			; ER3
go18:	push.l	er0

	mov.l	#go19,er2
	bra	readlong:16			; ER4
go19:	push.l	er0

	mov.l	#go20,er2
	bra	readlong:16			; ER5
go20:	push.l	er0

	mov.l	#go21,er2
	bra	readlong:16			; ER6
go21:	push.l	er0

	readbyte
	mov.b	r0l,r0h
	xor.w	e3,r0
	cmp.b	#0x0,r0h			; checksum 1
	bne	mon_goerr:16
	
	readbyte
	xor	e3,r0
	cmp.b	#0x0,r0l			; checksum 2
	bne	mon_goerr:16

	mov.b	#0x20,r0l
	writebyte
	
	readbyte
	cmp.b	#0x3f,r0l
	bne	mon_mainlp:16

	mov.w	#0x0,e3
	mov.w	#0x0,r0
	add.w	r0,r3
	writebyte
	
	mov.b	r3h,r0l			; send message id
	extu.w	r0
	add.w	r0,e3
	writebyte
	
	mov.w	e3,r0
	mov.b	r0h,r0l			; checksum high
	writebyte
	mov.w	e3,r0				; checksum low
	writebyte

	prepgo

#ifdef INTEGRATED
	; --- Integrated mode: prepare for return to firmware code ---
	;
	; Problem: prepgo set SCI1 RIE=1 for Trace32 break-in.  But in
	; integrated mode, the firmware also uses SCI1.  If RIE stays on
	; and mon_active=1, the RXI1 wrapper routes all received bytes to
	; the monitor instead of the firmware's serial handler.  The
	; firmware's GetChar() hangs, WDT fires, CPU resets.
	;
	; Fix: clear RIE, mon_active, and mon_magic before returning to
	; user code.  Trace32 break-in via serial is NOT supported in
	; integrated mode — use hardware breakpoints or re-enter via menu.
	;
	; Registers: er0-er6 will be overwritten by the pops below, so we
	; can freely use them here.

	; Clear SCI1 RIE (undo prepgo)
	mov.l	@mon_scibase,er0
	add.b	#2,r0l				; er0 → SCR
	bclr	#6,@er0				; SCR.RIE = 0

	; Clear runtime flag — firmware ISRs handle SCI1 normally
	sub.b	r0l,r0l
	mov.b	r0l,@_mon_active:32

	; Clear magic word — prevent reset-loop if user code crashes
	sub.l	er0,er0
	mov.l	er0,@_mon_magic:32
#endif

	pop.l	er6				; pop registers used by monitor
	pop.l	er5
	pop.l	er4
	pop.l	er3
	pop.l	er2
	pop.l	er1
	pop.l	er0
	rte					; pop CCR and PC
	
	
mon_goerr:	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H ?
	beq	mon_goerr1:8
	cmp.b	#5,r0l				; H8S/21xx ?
	beq	mon_goerr1:8
	cmp.b	#4,r0l				; H8S/224x ?
	beq	mon_goerr1:8
	mov.w	e1,r0
	cmp.b	#2,r0l
	blt	mon_goerr1:8			; int-ctrl mode 0 or 1
	bra	mon_goerr2:8

mon_goerr1:	add.l	#32,er7			; reset SP
	bra	mon_mainlp:16
mon_goerr2:	add.l	#34,er7			; reset SP
	bra	mon_mainlp:16


;----------------------------------------------------------------------
;
;	entry to monitor and save registers
;
;----------------------------------------------------------------------

_mon_breakp0:	push.l	er1				; push registers used by monitor
	push.l	er2
	push.l	er3
	push.l	er4
	push.l	er5
	push.l	er6

breakp1:	stc	ccr,r0l
	mov.b	@mon_imask0,r0h
	btst	#0,r0h
	beq	breakp11:8
	bset	#7,r0l
	bra	breakp12:8
breakp11:	bclr	#7,r0l
breakp12:	btst	#1,r0h
	beq	breakp13:8
	bset	#6,r0l
	bra	breakp14:8
breakp13:	bclr	#6,r0l
breakp14:	ldc	r0l,ccr			; set I- and UI-bit to mon_imask0

	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H ?
	beq	breakp2:8
	cmp.b	#5,r0l				; H8S/21xx ?
	beq	breakp2:8
	cmp.b	#4,r0l				; H8S/224x ?
	beq	breakp2:8

	stc.b	exr,r0l
	mov.b	@mon_imask1,r0h
	and.b	#0x7,r0h
	and.b	#0x0,r0l
	or.b	r0h,r0l
	ldc	r0l,exr			; set I2..0 to mon_imask1 patrick

breakp2:	mov.l	@(24,er7),er3			; get monitor entry reason to R3H

#ifdef INTEGRATED
	; Clear mon_magic — prevent reset-loop if user code later crashes.
	; (mon_magic was set by Start_Monitor; clear it now that we're safely
	; in the monitor.  If the user does "go" and the code crashes, the
	; CPU will do a normal boot instead of re-entering the monitor.)
	sub.l	er0,er0
	mov.l	er0,@_mon_magic:32

	; LCD debug: show entry reason on row 1
	; r3h = 0x10 (breakp) or 0x30 (trap) — all user regs on stack
	; Pick string BEFORE calling C (which clobbers er0-er3)
	cmp.b	#0x10,r3h
	bne	.Lbrk_trap
	mov.l	#mon_lcd_break,er4		; er4 = string (C won't clobber er4-er6)
	bra	.Lbrk_show
.Lbrk_trap:
	mov.l	#mon_lcd_trap,er4
.Lbrk_show:
	push.l	er3
	mov.b	#1,r0l
	jsr	@_lcd_SetRow
	mov.l	er4,er0
	jsr	@_lcd_WriteString
	pop.l	er3
#endif

	bra	mon_start0:16


;----------------------------------------------------------------------
;
;	run to RAM for EPROM hot patch
;
;----------------------------------------------------------------------
#ifdef	esi
ramloops:	btst	#1,@er6			; send checksum low (last byte acknowledge)

ramloop1:	mov.w	#0x4,e0

ramloop2:						; wait for character from host
	waitread
	mov.l	@er4,er6
	cmp.l	er6,er1			; are we seeing the EPROM ?
	bne	ramloop1:8
	mov.l	@(4,er4),er6
	cmp.l	er6,er3			; are we really seeing the EPROM ?
	bne	ramloop1:8
	dec.w	#1,e0
	bne	ramloop2:8			; retry to verify that the EPROM is visible
ramloope:	jmp	@er2				; jump back to EPROM
	
	.ALIGN 2
magic1:	.long	0x12345678
magic2:	.long	0x87654321
ramloopsize:	.long	0x28
ramloopsize1:	.word	0xb
#endif
	
goram:	mov.b	#0x0,r1l
	mov.l	#goram4,er2
	bra	checkandreply:16
	
goram4:	mov.w	e3,r1
	mov.b	r1h,r0l			; checksum high
	writebyte
	
	cmp.b	#0x7,r3l
	beq	goram0:8			; go to RAM
	
	mov.w	e3,r0				; checksum low
	writebyte
	
	bra	mon_mainlp:16
	
goram0:	mov.w	e3,r0
#ifdef	esi
	waitwrite
	
	stc	ccr,r0h			; copy I- and UI-bit of CCR
	and.b	#0xc0,r0h
	orc	#0xc0,ccr			; disable interrupts in CCR
	
	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H?
	beq	goram01:8
	
	stc.b	exr,r0l			; copy I2-0 of EXR
	and.b	#0x7,r0l
	or.b	r0l,r0h
	orc	#0x7,exr			; disable interrupts in EXR
	
goram01:	mov.l	@ramloopsize,er1
	add.l	#0x4,er1
	sub.l	er1,er7			; decrement SP for ramloop
	mov.l	er7,er2
	mov.b	r1l,r0l			; store displacement of SP
	mov.l	#ramloops,er1
	mov.w	@ramloopsize1,e0
	
goram1:	mov.l	@er1+,er3
	mov.l	er3,@er2			; copy ramloop to RAM
	inc.l	#2,er2
	inc.l	#2,er2
	dec.w	#1,e0
	bne	goram1:8
	
	mov.l	@magic1,er1
	mov.l	@magic2,er3
	mov.l	#magic1,er4
	mov.l	#goram2,er2
	jmp	@er7				; jump to RAM
	
goram2:	mov.b	r0l,r1l
	and.l	#0x000000ff,er1		; restore displacement of SP
	add.l	er1,er7			; inc SP to original value

	mov.b	r0h,r0l
	and.b	#0xc0,r0l
	stc	ccr,r1l
	and.b	#0x3f,r1l
	or.b	r0l,r1l
	ldc	r1l,ccr			; enable interrupts in CCR

	mov.b	@mon_proctype,r0l
	cmp.b	#1,r0l				; H8/300H?
	beq	goram3:8
	
	and.b	#0x7,r0h
	stc.b	exr,r0l
	and.b	#0xf8,r0l
	or.b	r0h,r0l
	ldc	r0l,exr			; enable interrupts in EXR

goram3:	nop
	prepcom
	bra	mon_mainlp:16
#endif

;----------------------------------------------------------------------
;
;	LCD debug strings (INTEGRATED only)
;
;----------------------------------------------------------------------

#ifdef INTEGRATED
	.align	2
mon_lcd_start:	.asciz	"MON:START       "
mon_lcd_break:	.asciz	"MON:BREAK       "
mon_lcd_trap:	.asciz	"MON:TRAP        "
mon_lcd_go:	.asciz	"MON:GO          "
#endif

;----------------------------------------------------------------------
;
;	Integrated-mode ISR wrappers and variables
;
;	When INTEGRATED is defined, these wrappers check mon_active
;	to route SCI1 interrupts to either the monitor or the firmware.
;
;----------------------------------------------------------------------

#ifdef INTEGRATED
	.extern _RXI1_SCI1
	.extern _ERI1_SCI1

	.global _mon_rxi1_wrapper
	.global _mon_eri1_wrapper
	.global _mon_active
	.global _mon_magic

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

;----------------------------------------------------------------------
; _mon_go_asm — Go/Step command: restore registers and RTE.
;
; C fills mon_go_regs[9] with Trace32-sent register values, then
; calls this function (noreturn).  We set SP to the user's desired
; value (mon_saved_sp), push the register values onto the stack,
; clear INTEGRATED flags, pop all registers, and RTE to user code.
;
; mon_go_regs[] layout:
;   [0]=CCR+PC, [1]=EXR(unused), [2]=ER0, [3]=ER1, [4]=ER2,
;   [5]=ER3, [6]=ER4, [7]=ER5, [8]=ER6
;
; Stack frame built by pushes (top=low addr, bottom=high addr):
;   SP+0  = ER6    (popped first)
;   SP+4  = ER5
;   SP+8  = ER4
;   SP+12 = ER3
;   SP+16 = ER2
;   SP+20 = ER1
;   SP+24 = ER0    (popped last before RTE)
;   SP+28 = CCR+PC (consumed by RTE)
;
; After 7 pops + RTE, SP returns to mon_saved_sp exactly.
;----------------------------------------------------------------------
	.align	2
_mon_go_asm:
	; Load user's desired SP and register array base
	mov.l	@_mon_saved_sp:32,er1	; er1 = user's desired SP
	mov.l	#_mon_go_regs,er2	; er2 = register array base

	; Set SP to user's desired value
	mov.l	er1,er7

	; Push CCR+PC (RTE will pop this last)
	mov.l	@(0,er2),er0		; [0] = CCR+PC
	push.l	er0

	; Push ER0 (popped just before RTE)
	mov.l	@(8,er2),er0		; [2] = ER0  (skip [1]=EXR, unused on H8S/2144)
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

	; --- INTEGRATED: clear RIE, mon_active, mon_magic ---
	; Clear SCI1 RIE (safety — ensure no monitor RX interrupts)
	mov.l	@mon_scibase,er0
	add.b	#2,r0l			; er0 → SCR
	bclr	#6,@er0			; SCR.RIE = 0

	; Clear runtime flag — firmware ISRs handle SCI1 normally
	sub.b	r0l,r0l
	mov.b	r0l,@_mon_active:32

	; Clear magic word — prevent reset-loop if user code crashes
	sub.l	er0,er0
	mov.l	er0,@_mon_magic:32

	; Pop all registers and return to user code
	pop.l	er6
	pop.l	er5
	pop.l	er4
	pop.l	er3
	pop.l	er2
	pop.l	er1
	pop.l	er0
	rte				; pop CCR+PC → resume user code

;----------------------------------------------------------------------
; mon_active — runtime flag: 0=firmware, 1=monitor active
; Placed in .bss (zeroed on cold boot = firmware mode).
;----------------------------------------------------------------------
	.section .bss
	.align	2
_mon_active:
	.byte	0

;----------------------------------------------------------------------
; mon_go_regs — 9 × 4 bytes for Go command register buffer.
; C fills this, assembly reads it to push registers and RTE.
;----------------------------------------------------------------------
	.align	4
_mon_go_regs:
	.space	36

;----------------------------------------------------------------------
; mon_saved_sp — user's SP for Go command.
; C stores the desired SP here (from frame or Write Register).
; Assembly _mon_go_asm loads this to set ER7 before push/pop/RTE.
;----------------------------------------------------------------------
	.align	4
_mon_saved_sp:
	.space	4

;----------------------------------------------------------------------
; mon_magic — persistent magic word for soft-reset re-entry.
; Placed in .noinit (NOT zeroed by start.S).
; Value 0x4D4F4E21 = "MON!" means monitor was active before reset.
;----------------------------------------------------------------------
	.section .noinit,"aw",@nobits
	.align	4
_mon_magic:
	.long	0

#endif /* INTEGRATED */

