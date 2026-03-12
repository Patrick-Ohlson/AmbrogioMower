;/****************************************************************
;KPIT Cummins Infosystems Ltd, Pune, India. - 4th September 2003.
;
;This program is distributed in the hope that it will be useful,
;but WITHOUT ANY WARRANTY; without even the implied warranty of
;MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE
;
;*****************************************************************/


;*********************************************************************
; File: start.S
;
;
; desc:
; 
;  System initialisation routine - entry point for the application.
;  The stack pointer is initialised, then the hardware initialisation
;  routine called.  The static data areas are then initialised, before
;  the main function is executed.  A simple exit funtion is also
;  supplied
;
;*********************************************************************
;
.macro Trace
    ;jsr doShutoff:24
	;trace Patrick
	;mov.l #0,er2
g:
	
	;push.l er2
		mov.l	#debugTxt,er2
	push.l	er2		; Push data on stack
	sub.w	r0, r0		; Subtract binary
	jsr     tm_DisplayWrite:24 ;Jump to subroutine
	;jsr _printString:24
	;mov.l   #0x3E8, er0
	;jsr sub_4F6E:24
	;adds    #4, sp
	;mov.b   r0l, r0l
	;beq g:8
h:
    jmp h
.endm
;;
#ifdef __H8300H__  

#ifdef __NORMAL_MODE__
	.h8300hn
#else
	.h8300h
#endif

#endif	/*_H8300H_ */

#ifdef __H8300S__

#ifdef __NORMAL_MODE__
	.h8300sn
#else
	.h8300s
#endif

#endif /* __H8300S__ */
	
	.section .text
	.global _start
	.global _printString
	.global _UNDEFINED_ISR
	

#ifdef CPPAPP	
___dso_handle:
	.global ___dso_handle
#endif	

	.extern _hw_initialise
	.extern _main
	.extern _xputs

	.extern _data
	.extern _mdata
	.extern _edata
	.extern _bss
	.extern _ebss
	.extern _stack

	; Trace32 monitor integration
	.extern _mon_magic
	.extern _mon_active
	.extern _mon_start
.equ KBCOMP_BYTE, 0xFFFEE4
.equ INTC_ICRA_BYTE, 0xFFFEE8
.equ INTC_ICRB_BYTE, 0xFFFEE9
.equ INTC_ICRC_BYTE, 0xFFFEEA
.equ INTC_ISR_BYTE, 0xFFFEEB
.equ INTC_ISCR_WORD, 0xFFFEEC
.equ INTC_ISCR_BYTE_H, 0xFFFEEC
.equ INTC_ISCR_BYTE_L, 0xFFFEED
.equ INTC_wk1, 0xFFFEEE
.equ INTC_ABRKCR_BYTE, 0xFFFEF4
.equ INTC_BARA, 0xFFFEF5
.equ INTC_BARB, 0xFFFEF6
.equ INTC_BARC, 0xFFFEF7
.equ INTC_wk2, 0xFFFEF8
.equ INTC_IER_BYTE, 0xFFFFC2
.equ INTC_wk3, 0xFFFFC3
.equ INTC_KMIMR_BYTE, 0xFFFFF1
.equ INTC_wk4, 0xFFFFF2
.equ INTC_KMIMRA_BYTE, 0xFFFFF3
.equ FLASH_FLMCR1_BYTE, 0xFFFF80
.equ FLASH_FLMCR2_BYTE, 0xFFFF81
.equ FLASH_EBR1_BYTE, 0xFFFF82
.equ FLASH_EBR2_BYTE, 0xFFFF83
.equ SBYCR_BYTE, 0xFFFF84
.equ LPWRCR_BYTE, 0xFFFF85
.equ MSTPCR_WORD, 0xFFFF86
.equ MSTPCR_BYTE_H, 0xFFFF86
.equ MSTPCR_BYTE_L, 0xFFFF87
.equ STCR_BYTE, 0xFFFFC3
.equ SYSCR_BYTE, 0xFFFFC4
.equ MDCR_BYTE, 0xFFFFC5
.equ SCI0_SMR_BYTE, 0xFFFFD8
.equ SCI0_BRR, 0xFFFFD9
.equ SCI0_SCR_BYTE, 0xFFFFDA
.equ SCI0_TDR, 0xFFFFDB
.equ SCI0_SSR_BYTE, 0xFFFFDC
.equ SCI0_RDR, 0xFFFFDD
.equ SCI0_SCMR_BYTE, 0xFFFFDE
.equ SCI1_SMR_BYTE, 0xFFFF88
.equ SCI1_BRR, 0xFFFF89
.equ SCI1_SCR_BYTE, 0xFFFF8A
.equ SCI1_TDR, 0xFFFF8B
.equ SCI1_SSR_BYTE, 0xFFFF8C
.equ SCI1_RDR, 0xFFFF8D
.equ SCI1_SCMR_BYTE, 0xFFFF8E
.equ SCI2_SMR_BYTE, 0xFFFFA0
.equ SCI2_BRR, 0xFFFFA1
.equ SCI2_SCR_BYTE, 0xFFFFA2
.equ SCI2_TDR, 0xFFFFA3
.equ SCI2_SSR_BYTE, 0xFFFFA4
.equ SCI2_RDR, 0xFFFFA5
.equ SCI2_SCMR_BYTE, 0xFFFFA6
.equ FRT_TIER_BYTE, 0xFFFF90
.equ FRT_TCSR_BYTE, 0xFFFF91
.equ FRT_FRC, 0xFFFF92
.equ FRT_OCRA, 0xFFFF94
.equ FRT_TCR_BYTE, 0xFFFF96
.equ FRT_TOCR_BYTE, 0xFFFF97
.equ FRT_ICRA, 0xFFFF98
.equ FRT_ICRB, 0xFFFF9A
.equ FRT_ICRC, 0xFFFF9C
.equ FRT_ICRD, 0xFFFF9E
.equ PWMX_REGS1_ST_DACR_BYTE, 0xFFFFA0
.equ PWMX_REGS1_wk, 0xFFFFA1
.equ PWMX_REGS1_ST_DACNT_WORD, 0xFFFFA6
.equ PWMX_REGS0_ST_DADRA_WORD, 0xFFFFA0
.equ PWMX_REGS0_wk, 0xFFFFA2
.equ PWMX_REGS0_ST_DADRB_WORD, 0xFFFFA6
.equ WDT0_READ_TCSR_BYTE, 0xFFFFA8
.equ WDT0_READ_TCNT, 0xFFFFA9
.equ WDT0_WRITE_TCSR, 0xFFFFA8
.equ WDT0_WRITE_TCNT, 0xFFFFA8
.equ WDT1_READ_TCSR_BYTE, 0xFFFFEA
.equ WDT1_READ_TCNT, 0xFFFFEB
.equ WDT1_WRITE_TCSR, 0xFFFFEA
.equ WDT1_WRITE_TCNT, 0xFFFFEA
.equ P1_PCR_BYTE, 0xFFFFAC
.equ P1_wk1, 0xFFFFAD
.equ P1_DDR, 0xFFFFB0
.equ P1_wk2, 0xFFFFB1
.equ P1_DR_BYTE, 0xFFFFB2
.equ P2_PCR_BYTE, 0xFFFFAD
.equ P2_wk1, 0xFFFFAE
.equ P2_DDR, 0xFFFFB1
.equ P2_wk2, 0xFFFFB2
.equ P2_DR_BYTE, 0xFFFFB3
.equ P3_PCR_BYTE, 0xFFFFAE
.equ P3_wk1, 0xFFFFAF
.equ P3_DDR, 0xFFFFB4
.equ P3_wk2, 0xFFFFB5
.equ P3_DR_BYTE, 0xFFFFB6
.equ P4_DDR, 0xFFFFB5
.equ P4_wk, 0xFFFFB6
.equ P4_DR_BYTE, 0xFFFFB7
.equ P5_DDR, 0xFFFFB8
.equ P5_wk, 0xFFFFB9
.equ P5_DR_BYTE, 0xFFFFBA
.equ P6_DDR, 0xFFFFB9
.equ P6_wk1, 0xFFFFBA
.equ P6_DR_BYTE, 0xFFFFBB
.equ P6_wk2, 0xFFFFBC
.equ P6_PCR_BYTE, 0xFFFFF2
.equ P7_PIN_BYTE, 0xFFFFBE
.equ P8_DDR, 0xFFFFBD
.equ P8_wk, 0xFFFFBE
.equ P8_DR_BYTE, 0xFFFFBF
.equ P9_DDR, 0xFFFFC0
.equ P9_DR_BYTE, 0xFFFFC1
.equ PA_ODR_BYTE, 0xFFFFAA
.equ PA_EQU_DDR, 0xFFFFAB
.equ PA_EQU_PIN_BYTE, 0xFFFFAB
.equ PB_ODR_BYTE, 0xFFFFBC
.equ PB_PIN_BYTE, 0xFFFFBD
.equ PB_DDR, 0xFFFFBE
.equ BSC_BCR_BYTE, 0xFFFFC6
.equ BSC_WSCR_BYTE, 0xFFFFC7
.equ TMR_TCR0_BYTE, 0xFFFFC8
.equ TMR_TCR1_BYTE, 0xFFFFC9
.equ TMR_TCSR0_BYTE, 0xFFFFCA
.equ TMR_TCSR1_BYTE, 0xFFFFCB
.equ TMR_TCORA, 0xFFFFCC
.equ TMR_TCORB, 0xFFFFCE
.equ TMR_TCNT, 0xFFFFD0
.equ TMR0_TCR_BYTE, 0xFFFFC8
.equ TMR0_wk1, 0xFFFFC9
.equ TMR0_TCSR_BYTE, 0xFFFFCA
.equ TMR0_wk2, 0xFFFFCB
.equ TMR0_TCORA, 0xFFFFCC
.equ TMR0_wk3, 0xFFFFCD
.equ TMR0_TCORB, 0xFFFFCE
.equ TMR0_wk4, 0xFFFFCF
.equ TMR0_TCNT, 0xFFFFD0
.equ TMR1_TCR_BYTE, 0xFFFFC9
.equ TMR1_wk1, 0xFFFFCA
.equ TMR1_TCSR_BYTE, 0xFFFFCB
.equ TMR1_wk2, 0xFFFFCC
.equ TMR1_TCORA, 0xFFFFCD
.equ TMR1_wk3, 0xFFFFCE
.equ TMR1_TCORB, 0xFFFFCF
.equ TMR1_wk4, 0xFFFFD0
.equ TMR1_TCNT, 0xFFFFD1
.equ TMRY_TCR_BYTE, 0xFFFFF0
.equ TMRY_TCSR_BYTE, 0xFFFFF1
.equ TMRY_TCORA, 0xFFFFF2
.equ TMRY_TCORB, 0xFFFFF3
.equ TMRY_TCNT, 0xFFFFF4
.equ TMRY_TISR_BYTE, 0xFFFFF5
.equ AD_ADDRA, 0xFFFFE0
.equ AD_ADDRB, 0xFFFFE2
.equ AD_ADDRC, 0xFFFFE4
.equ AD_ADDRD, 0xFFFFE6
.equ AD_ADCSR_BYTE, 0xFFFFE8
.equ AD_ADCR_BYTE, 0xFFFFE9
.equ DA_DADR0, 0xFFFFF8
.equ DA_DADR1, 0xFFFFF9
.equ DA_DACR_BYTE, 0xFFFFFA
.equ unk_FFFF00, 0xFFFF00		; unknown register (patrick)
.equ e_INT_TRAP1,0x20
.equ e_INT_TRAP3,0x28
.equ e_INT_TRAP4,0x2c
.equ e_INT_IRQ7,0x5c

_start:
	; initialise the SP for non-vectored code
    mov.l   #_stack,er7

	; --- Trace32 monitor soft-reset re-entry ---
	; If mon_magic == 0x4D4F4E21 ("MON!"), the monitor was active
	; before reset.  Clear magic, set runtime flag, enter monitor.
	; Skip hw_initialise / .data / .bss init — monitor does its own.
	mov.l	#_mon_magic,er0
	mov.l	@er0,er1
	mov.l	#0x4D4F4E21,er2
	cmp.l	er1,er2
	bne	normal_boot
	; Magic matches — clear it and re-enter monitor
	sub.l	er1,er1
	mov.l	er1,@er0		; clear magic (prevent loop)
	mov.b	#1,r1l
	mov.b	r1l,@_mon_active:32	; set runtime flag for ISR wrappers
	jmp	@_mon_start		; enter monitor (it inits SCI itself)

normal_boot:
	; call the hardware initialiser
	jsr     @_hw_initialise
#ifdef ROMSTART	
	; get the boundaries for the .data section initialisation
    mov.l   #_data,er0
    mov.l   #_edata,er1
    mov.l   #_mdata,er2
    cmp.l   er0,er1
	beq     start_1
start_l:
    mov.b   @er2,r3l  ;get from src
    mov.b   r3l,@er0  ;place in dest
    inc.l   #1,er2    ;inc src
    inc.l   #1,er0    ;inc dest
    cmp.l   er0,er1   ;dest == edata?
	bne     start_l
start_1:
#endif		//ROMSTART
	; zero out bss
    mov.l   #_bss,er0
    mov.l   #_ebss,er1
    cmp.l   er0,er1         
	beq     start_3
	sub.b   r2l,r2l
start_2:
    mov.b   r2l,@er0
    inc.l   #1,er0
    cmp.l   er0,er1
	bne     start_2
start_3:

#ifdef CPPAPP	
	;Initialize global constructor	
;	jsr	@___main
#endif
	

	; call the mainline     
	jsr     @_main
	
 exit:
	bra	exit




