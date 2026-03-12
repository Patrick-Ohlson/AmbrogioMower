/****************************************************************
KPIT Cummins Infosystems Ltd, Pune, India. - 10-Mar-2003.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************/
void UNDEFINED_ISR(void);
void ICIA_FRT(void);
void ICIB_FRT(void);
void ICIC_FRT(void);
void FOVI_FRT(void);
void CMIAY_8BitXY(void);
void CMIBY_8BitXY(void);
void ERI0_SCI0(void);
void RXI0_SCI0(void);
void TXI0_SCI0(void);
void ERI1_SCI1(void);
void RXI1_SCI1(void);
void TXI1_SCI1(void);
void loc_BD8(void);
void loc_B92(void);
void loc_BC2(void);

void start(void); /* Startup code (in start.asm)  */
//extern void INT_TGI1B_TPU1(void);
extern void INT_NMI(void);

/* Trace32 ROM monitor entry points (monitor.S, INTEGRATED mode) */
extern void mon_breakp(void);       /* NMI / ABRK entry (reason 0x1000) */
extern void mon_trap(void);         /* TRAP #n entry (reason 0x3000) */
extern void mon_rxi1_wrapper(void); /* RXI1 dispatcher: mon_active → monitor or firmware */
extern void mon_eri1_wrapper(void);  /* ERI1 dispatcher: mon_active → monitor or firmware */

typedef void (*fp) (void);
#define VECT_SECT          __attribute__ ((section (".vects")))
//
extern void INT_CMIAY_8BitXY(void);
//L200 testbed
//#define L200

//Main Testbed
#define TESTMODE
#ifdef L200
const fp HardwareVectors[] VECT_SECT = {
    //(f p)(0),
start,		/*  vector 0 Reset */
UNDEFINED_ISR,	/*  vector 1 Reserved */
UNDEFINED_ISR,	/*  vector 2 Reserved */
UNDEFINED_ISR,	/*  vector 3 Reserved */
UNDEFINED_ISR,	/*  vector 4 Reserved */
UNDEFINED_ISR,	/*  vector 5 Reserved */
UNDEFINED_ISR,	/*  vector 6 Direct Transition	*/
UNDEFINED_ISR,//INT_NMI,	/*  vector 7 NMI */
UNDEFINED_ISR,	/*  vector 8 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 9 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 10 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 11 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 12 Reserved */
UNDEFINED_ISR,	/*  vector 13 Reserved */
UNDEFINED_ISR,	/*  vector 14 Reserved */
UNDEFINED_ISR,	/*  vector 15 Reserved */
UNDEFINED_ISR,	/*  vector 16 External trap IRQ0	*/
UNDEFINED_ISR,	/*  vector 17 External trap IRQ1	*/
UNDEFINED_ISR,	/*  vector 18 External trap IRQ2	*/
UNDEFINED_ISR,	/*  vector 19 Reserved */
UNDEFINED_ISR,	/*  vector 20 Reserved */
UNDEFINED_ISR,	/*  vector 21 Reserved */
UNDEFINED_ISR,	/*  vector 22 Reserved */
UNDEFINED_ISR,	/*  vector 23 Reserved */
UNDEFINED_ISR,	/*  vector 24 SWDTEND DTC	*/
UNDEFINED_ISR,	/*  vector 25 WOVI0 	*/
UNDEFINED_ISR,	/*  vector 26 WOVI1	*/
UNDEFINED_ISR,	/*  vector 27 PC BREAK */
UNDEFINED_ISR,	/*  vector 28 ADI	*/
UNDEFINED_ISR,	/*  vector 29 Reserved */
UNDEFINED_ISR,	/*  vector 30 Reserved */
UNDEFINED_ISR,	/*  vector 31 Reserved */
UNDEFINED_ISR,	/*  vector 32 Reserved */
UNDEFINED_ISR,	/*  vector 33 Reserved */
UNDEFINED_ISR,	/*  vector 34 Reserved */
UNDEFINED_ISR,	/*  vector 35 Reserved */
UNDEFINED_ISR,	/*  vector 36 Reserved */
UNDEFINED_ISR,	/*  vector 37 Reserved */
UNDEFINED_ISR,	/*  vector 38 Reserved */
UNDEFINED_ISR,	/*  vector 39 Reserved */
UNDEFINED_ISR,	/*  vector 40 Reserved */
//INT_TGI1B_TPU1,	/*  vector 41 Reserved */
UNDEFINED_ISR, /*  vector 41 Reserved */
UNDEFINED_ISR,	/*  vector 42 Reserved */
UNDEFINED_ISR,	/*  vector 43 Reserved */
UNDEFINED_ISR,	/*  vector 44 Reserved */
UNDEFINED_ISR,	/*  vector 45 Reserved */
UNDEFINED_ISR,	/*  vector 46 Reserved */
UNDEFINED_ISR,	/*  vector 47 Reserved */
ICIA_FRT,	/*  vector 48 ICIA FRT */
ICIB_FRT,	/*  vector 49 ICIB FRT */
ICIC_FRT,	/*  vector 50 ICIC FRT */
UNDEFINED_ISR,	/*  vector 51 ICID FRT */
UNDEFINED_ISR,	/*  vector 52 OCIA FRT */
UNDEFINED_ISR,	/*  vector 53 OCIB FRT */
FOVI_FRT,	/*  vector 54 FOVI FRT */
UNDEFINED_ISR,	/*  vector 55 Reserved */
UNDEFINED_ISR,	/*  vector 56 Reserved */
UNDEFINED_ISR,	/*  vector 57 Reserved */
UNDEFINED_ISR,	/*  vector 58 Reserved */
UNDEFINED_ISR,	/*  vector 59 Reserved */
UNDEFINED_ISR,	/*  vector 60 Reserved */
UNDEFINED_ISR,	/*  vector 61 Reserved */
UNDEFINED_ISR,	/*  vector 62 Reserved */
UNDEFINED_ISR,	/*  vector 63 Reserved */
UNDEFINED_ISR,	/*  vector 64 CMIA0 8Bit0 */
UNDEFINED_ISR,	/*  vector 65 CMIB0 8Bit0 */
UNDEFINED_ISR,	/*  vector 66 OVI0 8Bit0 */
UNDEFINED_ISR,	/*  vector 67 Reserved */
UNDEFINED_ISR,	/*  vector 68 CMIA1 8Bit1 */
UNDEFINED_ISR,	/*  vector 69 CMIB1 8Bit1 */
UNDEFINED_ISR,	/*  vector 70 OVI1 8Bit1 */
UNDEFINED_ISR,	/*  vector 71 Reserved */
CMIAY_8BitXY,    //INT_CMIAY_8BitXY,	/*  vector 72 CMIAY 8BitXY */
CMIBY_8BitXY,	/*  vector 73 CMIBY 8BitXY */
UNDEFINED_ISR,	/*  vector 74 OVIY 8BitXY */
UNDEFINED_ISR,	/*  vector 75 ICIX 8BitXY */
UNDEFINED_ISR,	/*  vector 76 Reserved */
UNDEFINED_ISR,	/*  vector 77 Reserved */
UNDEFINED_ISR,	/*  vector 78 Reserved */
UNDEFINED_ISR,	/*  vector 79 Reserved */
ERI0_SCI0,	/*  vector 80 ERI0 SCI0 */
RXI0_SCI0,	/*  vector 81 RXI0 SCI0 */
TXI0_SCI0,	/*  vector 82 TXI0 SCI0 */
UNDEFINED_ISR,	/*  vector 83 TEI0 SCI0 */
ERI1_SCI1,	/*  vector 84 ERI1 SCI1 */
RXI1_SCI1,	/*  vector 85 RXI1 SCI1 */
TXI1_SCI1,	/*  vector 86 TXI1 SCI1 */
UNDEFINED_ISR,	/*  vector 87 TEI1 SCI1 */
loc_B92,	/*  vector 88 Reserved */
loc_BC2,	/*  vector 89 Reserved */
loc_BD8,	/*  vector 90 Reserved */
UNDEFINED_ISR,	/*  vector 91 Reserved */
UNDEFINED_ISR,	/*  vector 92 IICIO IIC0 */
UNDEFINED_ISR,	/*  vector 93 DDCSWI IIC0 */
UNDEFINED_ISR,	/*  vector 94 IICI1 IIC1 */
UNDEFINED_ISR,	/*  vector 95 Reserved */
UNDEFINED_ISR,	/*  vector 96 Reserved */
UNDEFINED_ISR,	/*  vector 97 Reserved */
UNDEFINED_ISR,	/*  vector 98 Reserved */
UNDEFINED_ISR,	/*  vector 99 Reserved */
UNDEFINED_ISR,	/*  vector 100 Reserved */
UNDEFINED_ISR,	/*  vector 101 Reserved */
UNDEFINED_ISR,	/*  vector 102 Reserved */
(fp)(0)		/*  vector 103 Reserved */
};

#else
#ifdef TESTMODE
const fp HardwareVectors[] VECT_SECT = {
    //(f p)(0),
start,		/*  vector 0 Reset */
UNDEFINED_ISR,	/*  vector 1 Reserved */
UNDEFINED_ISR,	/*  vector 2 Reserved */
UNDEFINED_ISR,	/*  vector 3 Reserved */
UNDEFINED_ISR,	/*  vector 4 Reserved */
UNDEFINED_ISR,	/*  vector 5 Reserved */
UNDEFINED_ISR,	/*  vector 6 Direct Transition	*/
mon_breakp,	/*  vector 7 NMI → Trace32 monitor entry */
mon_breakp,	/*  vector 8 TRAP #0 → Trace32 monitor entry */
mon_breakp,	/*  vector 9 TRAP #1 → Trace32 monitor entry */
mon_trap,	/*  vector 10 TRAP #2 → Trace32 breakpoint (reason 0x3000) */
mon_trap,	/*  vector 11 TRAP #3 → Trace32 single-step (reason 0x3000) */
UNDEFINED_ISR,	/*  vector 12 Reserved */
UNDEFINED_ISR,	/*  vector 13 Reserved */
UNDEFINED_ISR,	/*  vector 14 Reserved */
UNDEFINED_ISR,	/*  vector 15 Reserved */
UNDEFINED_ISR,	/*  vector 16 External trap IRQ0	*/
UNDEFINED_ISR,	/*  vector 17 External trap IRQ1	*/
UNDEFINED_ISR,	/*  vector 18 External trap IRQ2	*/
UNDEFINED_ISR,	/*  vector 19 Reserved */
UNDEFINED_ISR,	/*  vector 20 Reserved */
UNDEFINED_ISR,	/*  vector 21 Reserved */
UNDEFINED_ISR,	/*  vector 22 Reserved */
UNDEFINED_ISR,	/*  vector 23 Reserved */
UNDEFINED_ISR,	/*  vector 24 SWDTEND DTC	*/
UNDEFINED_ISR,	/*  vector 25 WOVI0 	*/
UNDEFINED_ISR,	/*  vector 26 WOVI1	*/
mon_breakp,	/*  vector 27 PC BREAK → Trace32 address break */
UNDEFINED_ISR,	/*  vector 28 ADI	*/
UNDEFINED_ISR,	/*  vector 29 Reserved */
UNDEFINED_ISR,	/*  vector 30 Reserved */
UNDEFINED_ISR,	/*  vector 31 Reserved */
UNDEFINED_ISR,	/*  vector 32 Reserved */
UNDEFINED_ISR,	/*  vector 33 Reserved */
UNDEFINED_ISR,	/*  vector 34 Reserved */
UNDEFINED_ISR,	/*  vector 35 Reserved */
UNDEFINED_ISR,	/*  vector 36 Reserved */
UNDEFINED_ISR,	/*  vector 37 Reserved */
UNDEFINED_ISR,	/*  vector 38 Reserved */
UNDEFINED_ISR,	/*  vector 39 Reserved */
UNDEFINED_ISR,	/*  vector 40 Reserved */
//INT_TGI1B_TPU1,	/*  vector 41 Reserved */
UNDEFINED_ISR, /*  vector 41 Reserved */
UNDEFINED_ISR,	/*  vector 42 Reserved */
UNDEFINED_ISR,	/*  vector 43 Reserved */
UNDEFINED_ISR,	/*  vector 44 Reserved */
UNDEFINED_ISR,	/*  vector 45 Reserved */
UNDEFINED_ISR,	/*  vector 46 Reserved */
UNDEFINED_ISR,	/*  vector 47 Reserved */
ICIA_FRT,	/*  vector 48 ICIA FRT — encoder capture A (left motor) */
ICIB_FRT,	/*  vector 49 ICIB FRT — encoder capture B */
ICIC_FRT,	/*  vector 50 ICIC FRT — encoder capture C (right motor) */
UNDEFINED_ISR,	/*  vector 51 ICID FRT */
UNDEFINED_ISR,	/*  vector 52 OCIA FRT */
UNDEFINED_ISR,	/*  vector 53 OCIB FRT */
FOVI_FRT,	/*  vector 54 FOVI FRT — overflow handler */
UNDEFINED_ISR,	/*  vector 55 Reserved */
UNDEFINED_ISR,	/*  vector 56 Reserved */
UNDEFINED_ISR,	/*  vector 57 Reserved */
UNDEFINED_ISR,	/*  vector 58 Reserved */
UNDEFINED_ISR,	/*  vector 59 Reserved */
UNDEFINED_ISR,	/*  vector 60 Reserved */
UNDEFINED_ISR,	/*  vector 61 Reserved */
UNDEFINED_ISR,	/*  vector 62 Reserved */
UNDEFINED_ISR,	/*  vector 63 Reserved */
UNDEFINED_ISR,	/*  vector 64 CMIA0 8Bit0 */
UNDEFINED_ISR,	/*  vector 65 CMIB0 8Bit0 */
UNDEFINED_ISR,	/*  vector 66 OVI0 8Bit0 */
UNDEFINED_ISR,	/*  vector 67 Reserved */
UNDEFINED_ISR,	/*  vector 68 CMIA1 8Bit1 */
UNDEFINED_ISR,	/*  vector 69 CMIB1 8Bit1 */
UNDEFINED_ISR,	/*  vector 70 OVI1 8Bit1 */
UNDEFINED_ISR,	/*  vector 71 Reserved */
INT_CMIAY_8BitXY,	/*  vector 72 CMIAY 8BitXY (system tick) */
UNDEFINED_ISR,	/*  vector 73 CMIBY 8BitXY */
UNDEFINED_ISR,	/*  vector 74 OVIY 8BitXY */
UNDEFINED_ISR,	/*  vector 75 ICIX 8BitXY */
UNDEFINED_ISR,	/*  vector 76 Reserved */
UNDEFINED_ISR,	/*  vector 77 Reserved */
UNDEFINED_ISR,	/*  vector 78 Reserved */
UNDEFINED_ISR,	/*  vector 79 Reserved */
UNDEFINED_ISR,	/*  vector 80 ERI0 SCI0 */
UNDEFINED_ISR,	/*  vector 81 RXI0 SCI0 */
UNDEFINED_ISR,	/*  vector 82 TXI0 SCI0 */
UNDEFINED_ISR,	/*  vector 83 TEI0 SCI0 */
mon_eri1_wrapper,	/*  vector 84 ERI1 SCI1 — monitor or firmware dispatcher */
mon_rxi1_wrapper,	/*  vector 85 RXI1 SCI1 — monitor or firmware dispatcher */
TXI1_SCI1,	/*  vector 86 TXI1 SCI1 — transmits byte from TX ring buffer */
UNDEFINED_ISR,	/*  vector 87 TEI1 SCI1 */
UNDEFINED_ISR,	/*  vector 88 Reserved */
UNDEFINED_ISR,	/*  vector 89 Reserved */
UNDEFINED_ISR,	/*  vector 90 Reserved */
UNDEFINED_ISR,	/*  vector 91 Reserved */
UNDEFINED_ISR,	/*  vector 92 IICIO IIC0 */
UNDEFINED_ISR,	/*  vector 93 DDCSWI IIC0 */
UNDEFINED_ISR,	/*  vector 94 IICI1 IIC1 */
UNDEFINED_ISR,	/*  vector 95 Reserved */
UNDEFINED_ISR,	/*  vector 96 Reserved */
UNDEFINED_ISR,	/*  vector 97 Reserved */
UNDEFINED_ISR,	/*  vector 98 Reserved */
UNDEFINED_ISR,	/*  vector 99 Reserved */
UNDEFINED_ISR,	/*  vector 100 Reserved */
UNDEFINED_ISR,	/*  vector 101 Reserved */
UNDEFINED_ISR,	/*  vector 102 Reserved */
(fp)(0)		/*  vector 103 Reserved */
};
#else
const fp HardwareVectors[] VECT_SECT = {
    //(f p)(0),
start,		/*  vector 0 Reset */
UNDEFINED_ISR,	/*  vector 1 Reserved */
UNDEFINED_ISR,	/*  vector 2 Reserved */
UNDEFINED_ISR,	/*  vector 3 Reserved */
UNDEFINED_ISR,	/*  vector 4 Reserved */
UNDEFINED_ISR,	/*  vector 5 Reserved */
UNDEFINED_ISR,	/*  vector 6 Direct Transition	*/
UNDEFINED_ISR,//INT_NMI,	/*  vector 7 NMI */
UNDEFINED_ISR,	/*  vector 8 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 9 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 10 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 11 User breakpoint trap	*/
UNDEFINED_ISR,	/*  vector 12 Reserved */
UNDEFINED_ISR,	/*  vector 13 Reserved */
UNDEFINED_ISR,	/*  vector 14 Reserved */
UNDEFINED_ISR,	/*  vector 15 Reserved */
UNDEFINED_ISR,	/*  vector 16 External trap IRQ0	*/
UNDEFINED_ISR,	/*  vector 17 External trap IRQ1	*/
UNDEFINED_ISR,	/*  vector 18 External trap IRQ2	*/
UNDEFINED_ISR,	/*  vector 19 Reserved */
UNDEFINED_ISR,	/*  vector 20 Reserved */
UNDEFINED_ISR,	/*  vector 21 Reserved */
UNDEFINED_ISR,	/*  vector 22 Reserved */
UNDEFINED_ISR,	/*  vector 23 Reserved */
UNDEFINED_ISR,	/*  vector 24 SWDTEND DTC	*/
UNDEFINED_ISR,	/*  vector 25 WOVI0 	*/
UNDEFINED_ISR,	/*  vector 26 WOVI1	*/
UNDEFINED_ISR,	/*  vector 27 PC BREAK */
UNDEFINED_ISR,	/*  vector 28 ADI	*/
UNDEFINED_ISR,	/*  vector 29 Reserved */
UNDEFINED_ISR,	/*  vector 30 Reserved */
UNDEFINED_ISR,	/*  vector 31 Reserved */
UNDEFINED_ISR,	/*  vector 32 Reserved */
UNDEFINED_ISR,	/*  vector 33 Reserved */
UNDEFINED_ISR,	/*  vector 34 Reserved */
UNDEFINED_ISR,	/*  vector 35 Reserved */
UNDEFINED_ISR,	/*  vector 36 Reserved */
UNDEFINED_ISR,	/*  vector 37 Reserved */
UNDEFINED_ISR,	/*  vector 38 Reserved */
UNDEFINED_ISR,	/*  vector 39 Reserved */
UNDEFINED_ISR,	/*  vector 40 Reserved */
//INT_TGI1B_TPU1,	/*  vector 41 Reserved */
UNDEFINED_ISR, /*  vector 41 Reserved */
UNDEFINED_ISR,	/*  vector 42 Reserved */
UNDEFINED_ISR,	/*  vector 43 Reserved */
UNDEFINED_ISR,	/*  vector 44 Reserved */
UNDEFINED_ISR,	/*  vector 45 Reserved */
UNDEFINED_ISR,	/*  vector 46 Reserved */
UNDEFINED_ISR,	/*  vector 47 Reserved */
UNDEFINED_ISR,/*  vector 48 ICIA FRT */
UNDEFINED_ISR,	/*  vector 49 ICIB FRT */
UNDEFINED_ISR,	/*  vector 50 ICIC FRT */
UNDEFINED_ISR,	/*  vector 51 ICID FRT */
UNDEFINED_ISR,	/*  vector 52 OCIA FRT */
UNDEFINED_ISR,	/*  vector 53 OCIB FRT */
UNDEFINED_ISR,	/*  vector 54 FOVI FRT */
UNDEFINED_ISR,	/*  vector 55 Reserved */
UNDEFINED_ISR,	/*  vector 56 Reserved */
UNDEFINED_ISR,	/*  vector 57 Reserved */
UNDEFINED_ISR,	/*  vector 58 Reserved */
UNDEFINED_ISR,	/*  vector 59 Reserved */
UNDEFINED_ISR,	/*  vector 60 Reserved */
UNDEFINED_ISR,	/*  vector 61 Reserved */
UNDEFINED_ISR,	/*  vector 62 Reserved */
UNDEFINED_ISR,	/*  vector 63 Reserved */
UNDEFINED_ISR,	/*  vector 64 CMIA0 8Bit0 */
UNDEFINED_ISR,	/*  vector 65 CMIB0 8Bit0 */
UNDEFINED_ISR,	/*  vector 66 OVI0 8Bit0 */
UNDEFINED_ISR,	/*  vector 67 Reserved */
UNDEFINED_ISR,	/*  vector 68 CMIA1 8Bit1 */
UNDEFINED_ISR,	/*  vector 69 CMIB1 8Bit1 */
UNDEFINED_ISR,	/*  vector 70 OVI1 8Bit1 */
UNDEFINED_ISR,	/*  vector 71 Reserved */
UNDEFINED_ISR,    //INT_CMIAY_8BitXY,	/*  vector 72 CMIAY 8BitXY */
UNDEFINED_ISR,	/*  vector 73 CMIBY 8BitXY */
UNDEFINED_ISR,	/*  vector 74 OVIY 8BitXY */
UNDEFINED_ISR,	/*  vector 75 ICIX 8BitXY */
UNDEFINED_ISR,	/*  vector 76 Reserved */
UNDEFINED_ISR,	/*  vector 77 Reserved */
UNDEFINED_ISR,	/*  vector 78 Reserved */
UNDEFINED_ISR,	/*  vector 79 Reserved */
UNDEFINED_ISR,	/*  vector 80 ERI0 SCI0 */
UNDEFINED_ISR,	/*  vector 81 RXI0 SCI0 */
UNDEFINED_ISR,	/*  vector 82 TXI0 SCI0 */
UNDEFINED_ISR,	/*  vector 83 TEI0 SCI0 */
UNDEFINED_ISR,	/*  vector 84 ERI1 SCI1 */
UNDEFINED_ISR,	/*  vector 85 RXI1 SCI1 */
UNDEFINED_ISR,	/*  vector 86 TXI1 SCI1 */
UNDEFINED_ISR,	/*  vector 87 TEI1 SCI1 */
UNDEFINED_ISR,	/*  vector 88 Reserved */
UNDEFINED_ISR,	/*  vector 89 Reserved */
UNDEFINED_ISR,	/*  vector 90 Reserved */
UNDEFINED_ISR,	/*  vector 91 Reserved */
UNDEFINED_ISR,	/*  vector 92 IICIO IIC0 */
UNDEFINED_ISR,	/*  vector 93 DDCSWI IIC0 */
UNDEFINED_ISR,	/*  vector 94 IICI1 IIC1 */
UNDEFINED_ISR,	/*  vector 95 Reserved */
UNDEFINED_ISR,	/*  vector 96 Reserved */
UNDEFINED_ISR,	/*  vector 97 Reserved */
UNDEFINED_ISR,	/*  vector 98 Reserved */
UNDEFINED_ISR,	/*  vector 99 Reserved */
UNDEFINED_ISR,	/*  vector 100 Reserved */
UNDEFINED_ISR,	/*  vector 101 Reserved */
UNDEFINED_ISR,	/*  vector 102 Reserved */
(fp)(0)		/*  vector 103 Reserved */
};
#endif
#endif