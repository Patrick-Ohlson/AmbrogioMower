/****************************************************************
KPIT Cummins Infosystems Ltd, Pune, India. - 10-Mar-2003.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************/

/* Serial Port Definitions */
#define Serial_Port_Enable_Bit 		    MSTPCR.BIT._SCI1
#define Serial_Port_Control_Register	SCI1.SCR
#define Serial_Port_Status_Register		SCI1.SSR
#define Serial_Port_Mode_Register		SCI1.SMR
#define Serial_Port_Baud_Rate_Register	SCI1.BRR
#define Serial_Port_Receive_Data_Reg	SCI1.RDR
#define Serial_Port_Transmit_Data_Reg	SCI1.TDR
//

/* Bit Rate Register (BRR) settings for 14.7456MHz */
#define B4800     95
#define B9600     47    
#define B19200	  28   
#define B38400	  11 
#define B57600     7
#define B115200    3  

/* Serial control register (SCR) bit values/masks */
#define SCR_TIE  	0x80
#define SCR_RIE  	0x40
#define SCR_TE   	0x20
#define SCR_RE   	0x10
#define SCR_MPIE 	0x8
#define SCR_TEIE 	0x4
#define SCR_CKE1 	0x2
#define SCR_CKE0 	0x1


/* Serial mode register (SMR) bit values */
#define SMR_PAR_ENABLE		0x20
#define SMR_PAR_DISABLE		0x00
#define SMR_PAR_ODD			0x10
#define SMR_PAR_EVEN		0x00
#define SMR_1_STOP			0x00
#define SMR_2_STOP			0x08
#define SMR_7_CHAR			0x40
#define SMR_8_CHAR			0x00


/* Serial status register (SSR) bit values */
#define SSR_TDRE 	0x80
#define SSR_RDRF 	0x40
#define SSR_ORER 	0x20
#define SSR_FER  	0x10
#define SSR_PER  	0x8
#define SSR_TEND 	0x4
#define SSR_MPB  	0x2
#define SSR_MPBT 	0x1


/* Parity values used in port initialisation */
#define P_NONE		0
#define P_EVEN		1
#define P_ODD		2

     
/* Potential error codes from initialisation function */
#define SCI_OK		0x00
#define SCI_ERR		0x01  


#define INTERVAL	10000L
   
    
/* Structure definition used for SCI initialisation */
struct SCI_Init_Params
{        
	unsigned char Baud; 
	unsigned char Parity;
	unsigned char Stops;
	unsigned char Length;
};

   

/* Function Prototypes */
unsigned char 	InitSCI( struct SCI_Init_Params SCI_Config );
void			InitSci2( void );
unsigned char	GetChar( void );
void			PutChar( unsigned char Data );
void			PutStr( unsigned char* String );
void			SCI1_ClearErrors( void );

/* Phase 2: Interrupt-driven SCI1 utilities */
void			sci1_tx_flush( void );
void			sci1_rx_flush( void );
unsigned char	sci1_rx_available( void );


// serial.h

#ifndef _SERIAL_H
#define _SERIAL_H



#define BAUD_115200			4
#define BAUD_38400			15
#define BAUD_9600			59

#define SCI_CH_NUM			1

#define LINEFEED			10
#define CARRIAGE_RETURN		13

// status info
#define OK					0
#define ERROR				1
#define TIMEOUT				2

// IO defines to get around the problem of differing 'iodefine.h' files
// change the defines here and not all through the code
#if SCI_CH_NUM == 0
#define SCI_CH				SCI0
#define SCI_SMR				SCI_CH.SMR
#define SCI_BRR				SCI_CH.BRR
#define SCI_SCR				SCI_CH.SCR
#define SCI_SSR				SCI_CH.SSR
#define SCI_TDR				SCI_CH.TDR
#define SCI_RDR				SCI_CH.RDR
#define SCI_SCMR			SCI_CH.SCMR
#elif SCI_CH_NUM == 1
#define SCI_CH				SCI1
#define SCI_SMR				SCI_CH.SMR
#define SCI_BRR				SCI_CH.BRR
#define SCI_SCR				SCI_CH.SCR
#define SCI_SSR				SCI_CH.SSR
#define SCI_TDR				SCI_CH.TDR
#define SCI_RDR				SCI_CH.RDR
#define SCI_SCMR			SCI_CH.SCMR
#elif SCI_CH_NUM == 2
#define SCI_CH				SCI2
#define SCI_SMR				SCI_CH.SMR
#define SCI_BRR				SCI_CH.BRR
#define SCI_SCR				SCI_CH.SCR
#define SCI_SSR				SCI_CH.SSR
#define SCI_TDR				SCI_CH.TDR
#define SCI_RDR				SCI_CH.RDR
#define SCI_SCMR			SCI_CH.SCMR
#endif


// structure definitions
union union_c2s {
		unsigned char uc[2];
		unsigned short us;
};

// function prototypes
void InitSci (void);
unsigned short GetByte (unsigned long timeout);
void SendByte (unsigned char b);
void SendString (char* str);
unsigned char RxByteWaiting (void);
void PurgeComms (unsigned long timeout);

#endif

