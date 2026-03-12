
// FD
// OUT
#define	READ_DATA			P4.DR.BIT.B0
#define	INDEX				P4.DR.BIT.B3
#define	TMR1_OUT			P4.DR.BIT.B4
#define	WRITE_PROTECT		P4.DR.BIT.B6
#define	DISK_CHANGE			P6.DR.BIT.B0
#define	READY				P6.DR.BIT.B1
#define	TRACK00				P6.DR.BIT.B3
// IN
#define	MODE_SELECT			PB.PIN.BIT.B0
#define	DRIVE_SELECT0		PB.PIN.BIT.B1
#define	DRIVE_SELECT1		PB.PIN.BIT.B2
#define	MOTOR_ON			PB.PIN.BIT.B3
#define	DIRECTION_SELECT	PB.PIN.BIT.B4
#define	SIDEONE_SELECT		PB.PIN.BIT.B5
#define	STEP				P6.DR.BIT.B7	// IRQ7 立ち上がりエッジ
#define	WRITE_DATA			P6.DR.BIT.B6
#define	WRITE_GATE			P6.DR.BIT.B4	// IRQ6 立ち下がりエッジ
// 注意 DDRレジスタはライトオンリー
//#define fd_io_init()	{P4.DDR|=0x69; P6.DDR|=0x03; PB.DDR&=0xC0; P6.DDR&=0x2F;}
// P4 0101 1001
// P6 0011 1011
// PB 0000 0000
// P8 0000 0001
// P5 0000 0101
#define ioport_init() {P4.DDR=0x59; P6.DDR=0x3B; PB.DDR=0x00; P8.DDR=0x01; P5.DDR=0x05;}

// SD / MMC
// OUT
#define	MMC_CS		P8.DR.BIT.B0
#define	MMC_CLK		P5.DR.BIT.B2
#define	MMC_DI		P5.DR.BIT.B0
// IN
#define	MMC_DO		P5.DR.BIT.B1
//#define mmc_io_init()	{P8.DDR|=0x01; P5.DDR|=0x05; P5.DDR&=0xFD;}

#define LED_BLUE	P6.DR.BIT.B4
#define LED_RED		P6.DR.BIT.B5
//#define led_io_init()	{P6.DDR|=0x30;}