/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        user.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 2.30.01+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       11/19/04    Original.
 * Brian Schmalz		03/15/06	Added user code to impliment
 *									firmware version D v1.0 for UBW
 *									project. See www.greta.dhs.org/UBW
 * Brian Schmalz		05/04/06	Starting version 1.1, which will 
 * 									include several fixes. See website.
 * BPS					06/21/06	Starting v1.2 -
 * - Fixed problem with I packets (from T command) filling up TX buffer
 * 		and not letting any incoming commands be received. (strange)
 * - Adding several commands - Analog inputs being the biggest set.
 * - Also Byte read/Byte write (PEEK/POKE) anywhere in memory
 * - Individual pin I/O and direction
 * BPS					08/16/06	v1.3 - Fixed bug with USB startup
 * BPS					09/09/06	v1.4 - Starting 1.4
 * - Fixed Microchip bug with early silicon - UCONbits.PKTDIS = 0;
 * - Adding BO and BC commands for parallel output to graphics pannels
 * BPS					12/06/06	v1.4 - More work on 1.4
 * - Re-wrote all I/O buffering code for increased speed and functionality
 * - Re-wrote error handling code
 * - Added delays to BC/BO commands to help Corey
 * BPS					01/06/07	v1.4 - Added RC command for servos
 * BPS					03/07/07	v1.4.1 - Changed blink rate for SFE
 * BPS					05/24/07	v1.4.2 - Fixed RC command bug - it
 *									wouldn't shut off.
 * BPS					08/28/07	v1.4.3 - Allowed UBW to run without
 *									usb connected.
 * BPS                  02/12/10    v1.4.4 - Fixed bug in C command 
 *                                  all 10 analogs now enabled
 * BPS                  04/04/10    v1.4.5 - Fixed bug in PI command 
 *                                  now just sends 1 or 0 back
 * BPS/JA               06/11/10	v1.4.6 - Added F (Frequency) command
 * BPS                  02/11/11    v1.4.7 - Added USB buffer checking code
 *                                  so that you can't overflow the TX buf
 *                                  Also switched to USB_INTERRUPT rather
 *                                  than polling.
 * BPS                  02/20/11    v1.4.7 - Rebuilt c018i_BL.o to remove
 *                                  extra GOTO command at 0x0000
 * BPS					02/27/11	v1.4.8 - Fixed BS command so it can take
 *									all binary data
 * BPS                  03/28/11    v1.4.8 - No functionality change, just
 *                                  modified main.c, linker scripts, and
 *                                  project file to allow for easy conversion
 *                                  to compiling without the bootloader support.
 *                                  (i.e. stand-alone). Simply comment out
 *                                  #define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
 *                                  in HardwareProfile_UBW.h and set your
 *                                  proper crystal speed in config bits in main.c
 *                                  Also remove the /uPROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
 *                                  from the Project->Build Options->Project->MPLINK Linker->Use Alternate Settings
 *                                  command line.
 * BPS                  05/07/11    v1.4.9 - Added fourth paramter (optional) to
 *                                  "F" command - duty cycle, from 1 to 99 as a percentage
 *                                  
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <usart.h>
#include <stdio.h>
#include <ctype.h>
#include <delays.h>
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "Usb\usb_device.h"
#include "HardwareProfile.h"
#include "user.h"

/** D E F I N E S ********************************************************/

#define kUSART_TX_BUF_SIZE		32				// In bytes
#define kUSART_RX_BUF_SIZE		32				// In bytes

#define kISR_FIFO_A_DEPTH		3
#define kISR_FIFO_D_DEPTH		3
#define kPR2_RELOAD				250		// For 1ms TMR2 tick
#define kCR						0x0D
#define kLF						0x0A

//#define F_COMMAND_EXTRA_CYCLES		18		// Number of instructions between ISR fire and reload
#define F_COMMAND_EXTRA_CYCLES		20		// Number of instructions between ISR fire and reload
#define F_COMMAND_MIN_CYCLES		40		// Number of instructions of entire ISR
#define F_COMMAND_PRESCALE_CUT_OFF	200		// Hz below which we use the F_COMMAND_PRESCALE prescale
#define F_COMMAND_PRESCALE			128		//

/** V A R I A B L E S ********************************************************/
#pragma udata access fast_vars

// Rate variable - how fast does interrupt fire to capture inputs?
near unsigned int time_between_updates;

near volatile unsigned int ISR_D_RepeatRate;			// How many 1ms ticks between Digital updates
near volatile unsigned char ISR_D_FIFO_in;				// In pointer
near volatile unsigned char ISR_D_FIFO_out;				// Out pointer
near volatile unsigned char ISR_D_FIFO_length;			// Current FIFO depth

near volatile unsigned int ISR_A_RepeatRate;			// How many 1ms ticks between Analog updates
near volatile unsigned char ISR_A_FIFO_in;				// In pointer
near volatile unsigned char ISR_A_FIFO_out;				// Out pointer
near volatile unsigned char ISR_A_FIFO_length;			// Current FIFO depth
near volatile unsigned char AnalogEnable;				// Maximum ADC channel to convert

// This byte has each of its bits used as a seperate error flag
near unsigned char error_byte;

// RC servo variables
// First the main array of data for each servo
near unsigned char g_RC_primed_ptr;
near unsigned char g_RC_next_ptr;
near unsigned char g_RC_timing_ptr;

// Used only in LowISR
near unsigned int D_tick_counter;
near unsigned int A_tick_counter;
near unsigned char A_cur_channel;

// Used in high ISR for F command
near unsigned char FCommandReloadValueOn_high;
near unsigned char FCommandReloadValueOn_low;
near unsigned char FCommandReloadValueOff_high;
near unsigned char FCommandReloadValueOff_low;
near unsigned char FCommandPrescaleOn;
near unsigned char FCommandPrescaleOff;
near unsigned char FCommandPrescaleCount;
near unsigned char FCommandPinHigh;

// ROM strings
const rom char st_OK[] = {"OK\r\n"};
const rom char st_LFCR[] = {"\r\n"};
const rom char st_version[] = {"UBW FW D Bersion 1.4.9\r\n"};

#pragma udata ISR_buf=0x100
volatile unsigned int ISR_A_FIFO[12][kISR_FIFO_A_DEPTH];	// Stores the most recent analog conversions
volatile unsigned char ISR_D_FIFO[3][kISR_FIFO_D_DEPTH];	// FIFO of actual data
volatile tRC_state g_RC_state[kRC_DATA_SIZE];				// Stores states for each pin for RC command
volatile unsigned int g_RC_value[kRC_DATA_SIZE];			// Stores reload values for TMR0

#pragma udata com_tx_buf = 0x200
// USB Transmit buffer for packets (back to PC)
unsigned char g_TX_buf[kTX_BUF_SIZE];

unsigned char g_RX_command_buf[kRX_COMMAND_BUF_SIZE];

#pragma udata com_rx_buf = 0x700
// USB Receiving buffer for commands as they come from PC
unsigned char g_RX_buf[kRX_BUF_SIZE];

// These variables are in normal storage space
#pragma udata

// USART Receiving buffer for data coming from the USART
unsigned char g_USART_RX_buf[kUSART_RX_BUF_SIZE];

// USART Transmit buffer for data going to the USART
unsigned char g_USART_TX_buf[kUSART_TX_BUF_SIZE];

// These are used for the Fast Parallel Output routines
unsigned char g_BO_init;
unsigned char g_BO_strobe_mask;
unsigned char g_BO_wait_mask;
unsigned char g_BO_wait_delay;
unsigned char g_BO_strobe_delay;

// Pointers to USB transmit (back to PC) buffer
unsigned char g_TX_buf_in;
unsigned char g_TX_buf_out;

// Pointers to USB receive (from PC) buffer
unsigned char g_RX_buf_in;
unsigned char g_RX_buf_out;

// In and out pointers to our USART input buffer
unsigned char g_USART_RX_buf_in;
unsigned char g_USART_RX_buf_out;

// In and out pointers to our USART output buffer
unsigned char g_USART_TX_buf_in;
unsigned char g_USART_TX_buf_out;

// Normally set to TRUE. Able to set FALSE to not send "OK" message after packet recepetion
BOOL	g_ack_enable;


volatile unsigned char FCommandPortAMask = 0; 
volatile unsigned char FCommandPortBMask = 0; 
volatile unsigned char FCommandPortCMask = 0; 
#ifdef __18F4550
volatile unsigned char FCommandPortDMask = 0; 
volatile unsigned char FCommandPortEMask = 0; 
#endif

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void BlinkUSBStatus (void);		// Handles blinking the USB status LED
BOOL SwitchIsPressed (void);	// Check to see if the user (PRG) switch is pressed
void parse_packet (void);		// Take a full packet and dispatch it to the right function
signed char extract_digit (signed short long * acc, unsigned char digits); // Pull a character out of the packet
void parse_R_packet (void);		// R for resetting UBW
void parse_C_packet (void);		// C for configuring I/O and analog pins
void parse_CX_packet (void); 	// CX For configuring serial port
void parse_O_packet (void);		// O for output digital to pins
void parse_I_packet (void);		// I for input digital from pins
void parse_V_packet (void);		// V for printing version
void parse_A_packet (void);		// A for requesting analog inputs
void parse_T_packet (void);		// T for setting up timed I/O (digital or analog)
void parse_PI_packet (void);	// PI for reading a single pin
void parse_PO_packet (void);	// PO for setting a single pin state
void parse_PD_packet (void);	// PD for setting a pin's direction
void parse_MR_packet (void);	// MR for Memory Read
void parse_MW_packet (void); 	// MW for Memory Write
void parse_TX_packet (void);	// TX for transmitting serial
void parse_RX_packet (void);	// RX for receiving serial
void parse_RC_packet (void);	// RC is for outputing RC servo pulses 
void parse_BO_packet (void);	// BO sends data to fast parallel output
void parse_BC_packet (void);	// BC configures fast parallel outputs
void parse_BS_packet (void);	// BS sends binary data to fast parallel output
void parse_CU_packet (void);	// CU configures UBW (system wide parameters)
void parse_SS_packet (void);	// SS Send SPI
void parse_RS_packet (void);	// RS Receive SPI
void parse_CS_packet (void);	// CS Configure SPI
void parse_SI_packet (void);	// SI Send I2C
void parse_RI_packet (void);	// RI Receive I2C
void parse_CI_packet (void);	// CI Configure I2C
void parse_F_packet  (void);	// F  Frequency out
void check_and_send_TX_data (void); // See if there is any data to send to PC, and if so, do it
void print_ack (void);			// Print "OK" after packet is parsed
int _user_putc (char c);		// Our UBS based stream character printer

/** D E C L A R A T I O N S **************************************************/
#pragma code

#pragma interruptlow low_ISR
void low_ISR(void)
{	
	#if defined(USB_INTERRUPT)
		USBDeviceTasks();
	#endif

	// Do we have a Timer2 interrupt? (1ms rate)
	if (PIR1bits.TMR2IF)
	{
		// Clear the interrupt 
		PIR1bits.TMR2IF = 0;
		
		// The most time critical part of this interrupt service routine is the 
		// handling of the RC command's servo output pulses.
		// Each time we get this interrupt, we look to see if the next pin on the
		// list has a value greater than zero. If so, we arm set it high and set
		// it's state to PRIMED. Then we advance the pointers to the next pair.
		if (kPRIMED == g_RC_state[g_RC_primed_ptr])
		{
			// This is easy, throw the value into the timer
			TMR0H = g_RC_value[g_RC_primed_ptr] >> 8;
			TMR0L = g_RC_value[g_RC_primed_ptr] & 0xFF;
	
			// Then make sure the timer's interrupt enable is set
			INTCONbits.TMR0IE = 1;
			// And be sure to clear the flag too
			INTCONbits.TMR0IF = 0;
			// Turn on Timer0
			T0CONbits.TMR0ON = 1;
	
			// And set this pin's state to timing
			g_RC_state[g_RC_primed_ptr] = kTIMING;
			
			// Remember which pin is now timing
			g_RC_timing_ptr = g_RC_primed_ptr;
		}

		if (kWAITING == g_RC_state[g_RC_next_ptr])
		{
			// If the value is zero, then shut this pin off
			// otherwise, prime it for sending a pulse
			if (0 == g_RC_value[g_RC_next_ptr])
			{
				g_RC_state[g_RC_next_ptr] = kOFF;
			}
			else
			{
				// Set the bit high
				if (g_RC_next_ptr < 8)
				{
					bitset (LATA, g_RC_next_ptr & 0x7);
				}
				else if (g_RC_next_ptr < 16)
				{
					bitset (LATB, g_RC_next_ptr & 0x7);
				}
				else
				{
					bitset (LATC, g_RC_next_ptr & 0x7);
				}
				// Set the state to primed so we know to do next
				g_RC_state[g_RC_next_ptr] = kPRIMED;
				// And remember which pin is primed
				g_RC_primed_ptr = g_RC_next_ptr;
			}
		}

		// And always advance the main pointer
		// NOTE: we need to skip RA6, RA7, and RC3, RC4, and RC5
		// (Because UBW doesn't bring those pins out to headers)
		g_RC_next_ptr++;
		if (6 == g_RC_next_ptr)
		{
			g_RC_next_ptr = 8;
		}
		else if (19 == g_RC_next_ptr)
		{
			g_RC_next_ptr = 22;
		}
		else if (kRC_DATA_SIZE == g_RC_next_ptr)
		{
			g_RC_next_ptr = 0;
		}
				
		// See if it's time to fire off an I packet
		if (ISR_D_RepeatRate > 0)
		{
			D_tick_counter++;
			if (D_tick_counter >= ISR_D_RepeatRate)
			{
				D_tick_counter = 0;
				// Tell the main code to send an I packet
				if (ISR_D_FIFO_length < kISR_FIFO_D_DEPTH)
				{
					// And copy over our port values
					ISR_D_FIFO[0][ISR_D_FIFO_in] = PORTA;
					ISR_D_FIFO[1][ISR_D_FIFO_in] = PORTB;
					ISR_D_FIFO[2][ISR_D_FIFO_in] = PORTC;
					ISR_D_FIFO_in++;
					if (ISR_D_FIFO_in >= kISR_FIFO_D_DEPTH)
					{
						ISR_D_FIFO_in = 0;	
					}
					ISR_D_FIFO_length++;
				}
				else
				{
					// Stop the madness! Something is wrong, we're
					// not getting our packets out. So kill the 
					// timer.
					ISR_D_RepeatRate = 0;
				}
			}	
		}
		
		// See if it's time to fire off an A packet
		if ((ISR_A_RepeatRate > 0) && (AnalogEnable > 0))
		{
			A_tick_counter++;
			if (A_tick_counter >= ISR_A_RepeatRate)
			{
				A_tick_counter = 0;
				// Tell the main code to send an A packet
				if (ISR_A_FIFO_length < kISR_FIFO_A_DEPTH)
				{
					ISR_A_FIFO_in++;
					if (ISR_A_FIFO_in >= kISR_FIFO_A_DEPTH)
					{
						ISR_A_FIFO_in = 0;	
					}
					ISR_A_FIFO_length++;
				}
				else
				{
					// Stop the madness! Something is wrong, we're
					// not getting our packets out. So kill the A
					// packets.
					ISR_A_RepeatRate = 0;
				}
			}	
		}

		// See if it's time to start analog conversions
		if (AnalogEnable > 0)
		{
			// Set the channel to zero to start off with
			A_cur_channel = 0;
			ADCON0 = (A_cur_channel << 2) + 1;

			// Clear the interrupt
			PIR1bits.ADIF = 0;

			// And make sure to always use low priority.
			IPR1bits.ADIP = 0;

			// Set the interrupt enable
			PIE1bits.ADIE = 1;

			// Make sure it's on!
			ADCON0bits.ADON = 1;

			// And tell the A/D to GO!
			ADCON0bits.GO_DONE = 1;
		}
		
	}

	// Do we have an analog interrupt?
	if (PIR1bits.ADIF)
	{
		// Clear the interrupt
		PIR1bits.ADIF = 0;

		// Read out the value that we just converted, and store it.
		ISR_A_FIFO[A_cur_channel][ISR_A_FIFO_in] = 
			(unsigned int)ADRESL 
			| 
			((unsigned int)ADRESH << 8);
	
		// Incriment the channel and write the new one in 
		A_cur_channel++;
		if (A_cur_channel >= AnalogEnable)
		{
			// We're done, so just sit and wait
			// Turn off our interrupts though.
			PIE1bits.ADIE = 0;
		}
		else
		{
			// Update the channel number
			ADCON0 = (A_cur_channel << 2) + 1;
			// And start the next conversion
			ADCON0bits.GO_DONE = 1;
		}
	}

	// Do we have a TMR0 interrupt? (RC command)
	// TMR0 is in 16 bit mode, and counts up to FFFF and overflows, generating
	// this interrupt.
	if (INTCONbits.TMR0IF)
	{
		// Turn off Timer0
		T0CONbits.TMR0ON = 0;

		// Clear the interrupt
		INTCONbits.TMR0IF = 0;
		
		// And disable it
		INTCONbits.TMR0IE = 0;

		// Only do our stuff if the pin is in the proper state
		if (kTIMING == g_RC_state[g_RC_timing_ptr])
		{
			// All we need to do is clear the pin and change its state to kWAITING
			if (g_RC_timing_ptr < 8)
			{
				bitclr (LATA, g_RC_timing_ptr & 0x7);
			}
			else if (g_RC_timing_ptr < 16)
			{
				bitclr (LATB, g_RC_timing_ptr & 0x7);
			}
			else
			{
				bitclr (LATC, g_RC_timing_ptr & 0x7);
			}
			g_RC_state[g_RC_timing_ptr] = kWAITING;		
		}
	}
}


#pragma interrupt high_ISR
void high_ISR(void)
{
	// Check if the timer 3 interrupt has fired
	if (PIR2bits.TMR3IF)
	{
		// Clear the interrupt flag
		PIR2bits.TMR3IF = 0;

		if (FCommandPinHigh)
		{
			//reload the timer
			TMR3H = FCommandReloadValueOn_high;
			TMR3L = FCommandReloadValueOn_low;
		
			// apply the current prescale
			FCommandPrescaleCount++;
			if (FCommandPrescaleCount >= FCommandPrescaleOn)
			{
				// We're going to go low here so record that fact
				FCommandPinHigh = FALSE;

				// Toggle the output pin state
				LATA = LATA & ~FCommandPortAMask;
				LATB = LATB & ~FCommandPortBMask;
				LATC = LATC & ~FCommandPortCMask;
				#ifdef __18F4550
				LATD = LATD & ~FCommandPortDMask;
				LATE = LATE & ~FCommandPortEMask;
				#endif

				TMR3H = FCommandReloadValueOff_high;
				TMR3L = FCommandReloadValueOff_low;
	
				FCommandPrescaleCount = 0;
			}
		}
		else
		{
			//reload the timer
			TMR3H = FCommandReloadValueOff_high;
			TMR3L = FCommandReloadValueOff_low;
		
			// apply the current prescale
			FCommandPrescaleCount++;
			if (FCommandPrescaleCount >= FCommandPrescaleOff)
			{
				// We're going to go high here so record that fact
				FCommandPinHigh = TRUE;
		
				// Toggle the output pin state
				LATA = LATA | FCommandPortAMask;
				LATB = LATB | FCommandPortBMask;
				LATC = LATC | FCommandPortCMask;
				#ifdef __18F4550
				LATD = LATD | FCommandPortDMask;
				LATE = LATE | FCommandPortEMask;
				#endif

				TMR3H = FCommandReloadValueOn_high;
				TMR3L = FCommandReloadValueOn_low;

				FCommandPrescaleCount = 0;
			}
		}
	}
}

void UserInit(void)
{
	char i, j;

	// Make all of 3 digital inputs
	LATA = 0x00;
	TRISA = 0xFF;
	// Turn all analog inputs into digital inputs
	ADCON1 = 0x0F;
	// Turn off the ADC
	ADCON0bits.ADON = 0;
	// Turn off our own idea of how many analog channels to convert
	AnalogEnable = 0;
	CMCON = 0x07;	// Comparators as digital inputs
	// Make all of PORTB inputs
	LATB = 0x00;
	TRISB = 0xFF;
	// Make all of PORTC inputs
	LATC = 0x00;
	TRISC = 0xFF;
#ifdef __18F4550
	// Make all of PORTD and PORTE inputs too
	LATD = 0x00;
	TRISD = 0xFF;
	LATE = 0x00;
	TRISE = 0xFF;
#endif

	// Initalize LED I/Os to outputs
    mInitAllLEDs();
	// Initalize switch as an input
    mInitAllSwitches();

	// Start off always using "OK" acknoledge.
	g_ack_enable = TRUE;

	// Use our own special output function for STDOUT
	stdout = _H_USER;

	// Initalize all of the ISR FIFOs
    ISR_A_FIFO_out = 0;
    ISR_A_FIFO_in = 0;
    ISR_A_FIFO_length = 0;
    ISR_D_FIFO_out = 0;
    ISR_D_FIFO_in = 0;
    ISR_D_FIFO_length = 0;

	// Make sure that our timer stuff starts out disabled
	ISR_D_RepeatRate = 0;
	ISR_A_RepeatRate = 0;
	D_tick_counter = 0;
	A_tick_counter = 0;
	A_cur_channel = 0;
	
    // Now init our registers
	// The prescaler will be at 16
    T2CONbits.T2CKPS1 = 1;
    T2CONbits.T2CKPS0 = 1;
    // We want the TMR2 post scaler to be a 3
    T2CONbits.T2OUTPS3 = 0;
    T2CONbits.T2OUTPS2 = 0;
    T2CONbits.T2OUTPS1 = 1;
    T2CONbits.T2OUTPS0 = 0;
	// Set our reload value
	PR2 = kPR2_RELOAD;

	// Set up the Analog to Digital converter
	// Clear out the FIFO data
	for (i = 0; i < 12; i++)
	{
		for (j = 0; j < kISR_FIFO_A_DEPTH; j++)
		{
			ISR_A_FIFO[i][j] = 0;
		}
	}	

    // Inialize USB TX and RX buffer management
    g_RX_buf_in = 0;
    g_RX_buf_out = 0;
	g_TX_buf_in = 0;
	g_TX_buf_out = 0;

	// And the USART TX and RX buffer management
	g_USART_RX_buf_in = 0;
	g_USART_RX_buf_out = 0;
	g_USART_TX_buf_in = 0;
	g_USART_TX_buf_out = 0;

	// Clear out the RC servo output pointer values
	g_RC_primed_ptr = 0;
	g_RC_next_ptr = 0;
	g_RC_timing_ptr = 0;

	// Clear the RC data structure
	for (i = 0; i < kRC_DATA_SIZE; i++)
	{
		g_RC_value[i] = 0;
		g_RC_state[i] = kOFF;
	}

	// Enable TMR0 for our RC timing operation
	T0CONbits.PSA = 1;		// Do NOT use the prescaler
	T0CONbits.T0CS = 0;		// Use internal clock
	T0CONbits.T08BIT = 0;	// 16 bit timer
	INTCONbits.TMR0IF = 0;	// Clear the interrupt flag
	INTCONbits.TMR0IE = 0;	// And clear the interrupt enable
	INTCON2bits.TMR0IP = 0;	// Low priority

    // Enable interrupt priorities
    RCONbits.IPEN = 1;
	T2CONbits.TMR2ON = 0;
    
    PIE1bits.TMR2IE = 1;
    IPR1bits.TMR2IP = 0;
    
    INTCONbits.GIEH = 1;	// Turn high priority interrupts on
    INTCONbits.GIEL = 1;	// Turn low priority interrupts on

	// Turn on the Timer2
	T2CONbits.TMR2ON = 1;
	
	T3CONbits.TMR3ON = 0;

}//end UserInit

/******************************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        In this function, we check for a new packet that just
 * 					arrived via USB. We do a few checks on the packet to see
 *					if it is worthy of us trying to interpret it. If it is,
 *					we go and call the proper function based upon the first
 *					character of the packet.
 *					NOTE: We need to see everything in one packet (i.e. we
 *					won't treat the USB data as a stream and try to find our
 *					start and end of packets within the stream. We just look 
 *					at the first character of each packet for a command and
 * 					check that there's a CR as the last character of the
 *					packet.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{   
	static char last_command[64] = {0};
	static BOOL in_esc = FALSE;
	static char esc_sequence[3] = {0};
	static BOOL in_cr = FALSE;
	static BYTE last_fifo_size;
    unsigned char tst_char;
	static unsigned char button_state = 0;
	static unsigned int button_ctr = 0;
	char i;
	BOOL	done = FALSE;
	unsigned char rx_bytes = 0;
	unsigned char byte_cnt = 0;
	static BOOL	gotBS = FALSE;		// True if we have detected BS command
	static BYTE	BSlen = 0;			// Number of total bytes of BS data

	BlinkUSBStatus();

	// Check for any new I packets (from T command) ready to go out
	while (ISR_D_FIFO_length > 0)
	{
		// Spit out an I packet first
		parse_I_packet ();

		// Then upate our I packet fifo stuff
		ISR_D_FIFO_out++;
		if (ISR_D_FIFO_out == kISR_FIFO_D_DEPTH)
		{
			ISR_D_FIFO_out = 0;
		}
		ISR_D_FIFO_length--;
	}			

	// Check for a new A packet (from T command) ready to go out
	while (ISR_A_FIFO_length > 0)
	{
		// Spit out an A packet first
		parse_A_packet ();

		// Then update our A packet fifo stuff
		ISR_A_FIFO_out++;
		if (ISR_A_FIFO_out == kISR_FIFO_A_DEPTH)
		{
			ISR_A_FIFO_out = 0;
		}
		ISR_A_FIFO_length--;
	}			
	// Bail from here if we're not 'plugged in' to a PC or we're suspended
    if(
		(USBDeviceState < CONFIGURED_STATE)
		||
		(USBSuspendControl==1)
	) 
	{
		return;
	}

	// Pull in some new data if there is new data to pull in
	// And we aren't waiting for the current move to finish
	
	rx_bytes = getsUSBUSART((char *)g_RX_command_buf, 64);

	if (rx_bytes > 0)
	{
		for(byte_cnt = 0; byte_cnt < rx_bytes; byte_cnt++)
		{
			tst_char = g_RX_command_buf[byte_cnt];

			// Check for BS command binary dataa
			if (gotBS && BSlen > 0)
			{
				g_RX_buf[g_RX_buf_in] = tst_char;
				g_RX_buf_in++;
				BSlen--;
			}
			// Check to see if we are in a CR/LF situation
			else if (
				!in_cr 
				&& 
				(
					kCR == tst_char
					||
					kLF == tst_char
				)
			)
			{
				in_cr = TRUE;
				g_RX_buf[g_RX_buf_in] = kCR;
				g_RX_buf_in++;
			
				// At this point, we know we have a full packet
				// of information from the PC to parse
	
				// Now, if we've gotten a full command (user send <CR>) then
				// go call the code that deals with that command, and then
				// keep parsing. (This allows multiple small commands per packet)
				// Copy the new command over into the 'up-arrow' buffer
				for (i=0; i<g_RX_buf_in; i++)
				{
					last_command[i] = g_RX_buf[i];
				}
				parse_packet ();
				gotBS = FALSE;
				BSlen = 0;
				g_RX_buf_in = 0;
				g_RX_buf_out = 0;
			}
			else if (tst_char == 27 && in_esc == FALSE)
			{
				in_esc = TRUE;
				esc_sequence[0] = 27;
				esc_sequence[1] = 0;
				esc_sequence[2] = 0;
			}
			else if (
				in_esc == TRUE 
				&& 
				tst_char == 91 
				&& 
				esc_sequence[0] == 27 
				&& 
				esc_sequence[1] == 0
			)
			{
				esc_sequence[1] = 91;
			}
			else if (
				in_esc == TRUE 
				&& 
				tst_char == 65 
				&&
				esc_sequence[0] == 27 
				&& 
				esc_sequence[1] == 91
			)
			{
				esc_sequence[0] = 0;
				esc_sequence[1] = 0;
				esc_sequence[2] = 0;
				in_esc = FALSE;
	
				// We got an up arrow (d27, d91, d65) and now copy over last command
				for (i = 0; g_RX_buf[i] != kCR; i++)
				{
					g_RX_buf[i] = last_command[i];
				}
				g_RX_buf_in = i;
				g_RX_buf[g_RX_buf_in] = 0;
	
				// Also send 'down arrow' to counter act the affect of the up arrow
				printf((far rom char *)"\x1b[B\x1b[1K\x1b[0G");
				printf((far rom char *)"%s", g_RX_buf);
			}
			else if (tst_char == 8 && g_RX_buf_in > 0)
			{
				// Handle the backspace thing
				g_RX_buf_in--;
				g_RX_buf[g_RX_buf_in] = 0x00;
				printf((far rom char *)" \b");
			}
			else if (
				tst_char != kCR
				&&
				tst_char != kLF
				&&
				tst_char >= 32
			)
			{
				esc_sequence[0] = 0;
				esc_sequence[1] = 0;
				esc_sequence[2] = 0;
				in_esc = FALSE;
	
				// Only add a byte if it is not a CR or LF
				g_RX_buf[g_RX_buf_in] = tst_char;
				in_cr = FALSE;
				g_RX_buf_in++;
				
				// Check for BS command
				if (g_RX_buf_in == 3)
				{
					if (
						(g_RX_buf[0] == 'B')
						&&
						(g_RX_buf[1] == 'S')
					)
					{
						gotBS = TRUE;
					}
				}
				// Check for BS length
				if (gotBS && BSlen == 0)
				{
					if (
						(g_RX_buf_in == 5)
						&&
						(g_RX_buf[4] == ',')
					)
					{
						BSlen = g_RX_buf[3] - '0';
					}
					else if (
						(g_RX_buf_in == 6)
						&&
						(g_RX_buf[5] == ',')					
					)
					{
						BSlen = (g_RX_buf[3] - '0') * 10;
						BSlen += (g_RX_buf[4] - '0');
					}		
				}	
						
			}
			// Check for buffer wraparound
			if (kRX_BUF_SIZE == g_RX_buf_in)
			{
				bitset (error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN);
				g_RX_buf_in = 0;
				g_RX_buf_out = 0;
			}
		}
	}

	// Check for any errors logged in error_byte that need to be sent out
	if (error_byte)
	{
		if (bittst (error_byte, 0))
		{
			// Unused as of yet
			printf ((far rom char *)"!0 \r\n");
		}
		if (bittst (error_byte, kERROR_BYTE_TX_BUF_OVERRUN))
		{
			printf ((far rom char *)"!2 Err: TX Buffer overrun\r\n");
		}
		if (bittst (error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN))
		{
			printf ((far rom char *)"!3 Err: RX Buffer overrun\r\n");
		}
		if (bittst (error_byte, kERROR_BYTE_MISSING_PARAMETER))
		{
			printf ((far rom char *)"!4 Err: Missing parameter(s)\r\n");
		}
		if (bittst (error_byte, kERROR_BYTE_PRINTED_ERROR))
		{
			// We don't need to do anything since something has already been printed out
			//printf ((rom char *)"!5\r\n");
		}
		if (bittst (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT))
		{
			printf ((far rom char *)"!6 Err: Invalid paramter value\r\n");
		}
		if (bittst (error_byte, kERROR_BYTE_EXTRA_CHARACTERS))
		{
			printf ((far rom char *)"!7 Err: Extra parmater\r\n");
		}
		error_byte = 0;
	}

	// Go send any data that needs sending to PC
	check_and_send_TX_data ();
}

// This is our replacement for the standard putc routine
// This enables printf() and all related functions to print to
// the UBS output (i.e. to the PC) buffer
int _user_putc (char c)
{
	BYTE OldPtr = g_TX_buf_in;

	// Check to see if adding this byte will cause us to be full
	OldPtr++;
	if (kTX_BUF_SIZE == OldPtr)
	{
		OldPtr = 0;
	}
	// If so, then wait until some bytes go away first and make room
	if (OldPtr == g_TX_buf_out)
	{
		check_and_send_TX_data();
	}
	// Copy the character into the output buffer
	g_TX_buf[g_TX_buf_in] = c;
	g_TX_buf_in++;

	// Check for wrap around
	if (kTX_BUF_SIZE == g_TX_buf_in)
	{
		g_TX_buf_in = 0;
	}
	
	// Also check to see if we bumpted up against our output pointer
	if (g_TX_buf_in == g_TX_buf_out)
	{
		bitset (error_byte, kERROR_BYTE_TX_BUF_OVERRUN);
	}
	return (c);
}

// In this function, we check to see if we have anything to transmit. 
// If so then we schedule the data for sending.
void check_and_send_TX_data (void)
{
	char temp;

	// Oly send if there's something there to send
	if (g_TX_buf_out != g_TX_buf_in)
	{
		// Yes, Microchip says not to do this. We'll be blocking
		// here until there's room in the USB stack to send
		// something new. But without making our buffers huge,
		// I don't know how else to do it.
		while (!USBUSARTIsTxTrfReady())
		{
			CDCTxService();
#if defined(USB_POLLING)
			USBDeviceTasks();				
#endif
		}
		// Now we know that the stack can transmit some new data

		// Now decide if we need to break it up into two parts or not
		if (g_TX_buf_in > g_TX_buf_out)
		{
			// Since IN is beyond OUT, only need one chunk
			temp = g_TX_buf_in - g_TX_buf_out;
			putUSBUSART((char *)&g_TX_buf[g_TX_buf_out], temp);
			// Now that we've scheduled the data for sending,
			// update the pointer
			g_TX_buf_out = g_TX_buf_in;
		}
		else
		{
			// Since IN is before OUT, we need to send from OUT to the end
			// of the buffer, then the next time around we'll catch
			// from the beginning to IN.
			temp = kTX_BUF_SIZE - g_TX_buf_out;
			putUSBUSART((char *)&g_TX_buf[g_TX_buf_out], temp);
			// Now that we've scheduled the data for sending,
			// update the pointer
			g_TX_buf_out = 0;
		}
		CDCTxService();
	}
}


// Look at the new packet, see what command it is, and 
// route it appropriately. We come in knowing that
// our packet is in g_RX_buf[], and that the beginning
// of the packet is at g_RX_buf_out, and the end (CR) is at
// g_RX_buf_in. Note that because of buffer wrapping,
// g_RX_buf_in may be less than g_RX_buf_out.
void parse_packet(void)
{
	unsigned int	command = 0;
	unsigned char	cmd1 = 0;
	unsigned char	cmd2 = 0;

	// Always grab the first character (which is the first byte of the command)
	cmd1 = toupper (g_RX_buf[g_RX_buf_out]);
	advance_RX_buf_out();
	command = cmd1;

	// Only grab second one if it is not a comma
	if (g_RX_buf[g_RX_buf_out] != ',' && g_RX_buf[g_RX_buf_out] != kCR)
	{
		cmd2 = toupper (g_RX_buf[g_RX_buf_out]);
		advance_RX_buf_out();
		command = ((unsigned int)(cmd1) << 8) + cmd2;
	}

	// Now 'command' is equal to one or two bytes of our command
	switch (command)
	{
		case ('R' * 256) + 'X':
		{
			// For receiving serial
			parse_RX_packet ();
			break;
		}
		case 'R':
		{
			// Reset command (resets everything to power-on state)
			parse_R_packet ();
			break;
		}
		case 'C':
		{
			// Configure command (configure ports for Input or Ouptut)
			parse_C_packet ();
			break;
		}		
		case ('C' * 256) + 'X':
		{
			// For configuring serial port
			parse_CX_packet ();
			break;
		}
		case ('C' * 256) + 'U':
		{
			// For configuring UBW
			parse_CU_packet ();
			break;
		}
		case 'O':
		{
			// Output command (tell the ports to output something)
			parse_O_packet ();
			break;
		}
		case 'I':
		{
			// Input command (return the current status of the ports)
			parse_I_packet ();
			break;
		}
		case 'V':
		{
			// Version command
			parse_V_packet ();
			break;
		}
		case 'A':
		{
			// Analog command
			parse_A_packet ();
			break;
		}
		case 'T':
		{
			// For timed I/O
			parse_T_packet ();
			break;
		}	
		case ('T' * 256) + 'X':
		{
			// For transmitting serial
			parse_TX_packet ();
			break;
		}
		case ('P' * 256) + 'I':
		{
			// PI for reading a single pin
			parse_PI_packet ();
			break;
		}
		case ('P' * 256) + 'O':
		{
			// PO for setting a single pin
			parse_PO_packet ();
			break;
		}
		
		case ('P' * 256) + 'D':
		{
			// PD for setting a pin's direction
			parse_PD_packet ();
			break;
		}
		case ('M' * 256) + 'R':
		{
			// MR for Memory Read
			parse_MR_packet ();
			break;
		}
		case ('M' * 256) + 'W':
		{
			// MW for Memory Write
			parse_MW_packet ();
			break;
		}
		case ('B' * 256) + 'O':
		{
			// MR for Fast Parallel Output
			parse_BO_packet ();		
			break;
		}
		case ('R' * 256) + 'C':
		{
			// RC for RC servo output
			parse_RC_packet ();		
			break;
		}
		case ('B' * 256) + 'C':
		{
			// BC for Fast Parallel Configure
			parse_BC_packet ();
			break;
		}
		case ('B' * 256) + 'S':
		{
			// BS for Fast Binary Stream output
			parse_BS_packet ();
			break;
		}
		case ('S' * 256) + 'S':
		{
			// SS for Send SPI
			parse_SS_packet ();
			break;
		}
		case ('R' * 256) + 'S':
		{
			// RS for Receive SPI
			parse_RS_packet ();
			break;
		}
		case ('C' * 256) + 'S':
		{
			// CS for Configure SPI
			parse_CS_packet ();
			break;
		}
		case ('S' * 256) + 'I':
		{
			// SI for Send I2C
			parse_SI_packet ();
			break;
		}
		case ('R' * 256) + 'I':
		{
			// RI for Receive I2C
			parse_RI_packet ();
			break;
		}
		case ('C' * 256) + 'I':
		{
			// CI for Configure I2C
			parse_CI_packet ();
			break;
		}
		case 'F':
		{
			// F for Frequency out.
			parse_F_packet ();
			break;
		}
		
		default:
		{
			if (0 == cmd2)
			{
				// Send back 'unknown command' error
				printf (
					 (far rom char *)"!8 Err: Unknown command '%c:%2X'\r\n"
					,cmd1
					,cmd1
				);
			}
			else
			{
				// Send back 'unknown command' error
				printf (
					 (far rom char *)"!8 Err: Unknown command '%c%c:%2X%2X'\r\n"
					,cmd1
					,cmd2
					,cmd1
					,cmd2
				);
			}
			break;
		}
	}

	// Double check that our output pointer is now at the ending <CR>
	// If it is not, this indicates that there were extra characters that
	// the command parsing routine didn't eat. This would be an error and needs
	// to be reported. (Ignore for Reset command because FIFO pointers get cleared.)
	if (
		(g_RX_buf[g_RX_buf_out] != kCR && 0 == error_byte)
		&&
		('R' != command)
	)
	{
		bitset (error_byte, kERROR_BYTE_EXTRA_CHARACTERS);
	}

	// Clean up by skipping over any bytes we haven't eaten
	// This is safe since we parse each packet as we get a <CR>
	// (i.e. g_RX_buf_in doesn't move while we are in this routine)
	g_RX_buf_out = g_RX_buf_in;
}

// Print out the positive acknoledgement that the packet was received
// if we have acks turned on.
void print_ack(void)
{
	if (g_ack_enable)
	{
		printf ((far rom char *)st_OK);
	}
}

// Return all I/Os to their default power-on values
void parse_R_packet(void)
{
	UserInit ();
	print_ack ();
}

// CU is "Configure UBW" and controls system-wide configruation values
// "CU,<parameter_number>,<paramter_value><CR>"
// <paramter_number>	<parameter_value>
// 1					{1|0} turns on or off the 'ack' ("OK" at end of packets)
void parse_CU_packet(void)
{
	unsigned char parameter_number;
	signed int paramater_value;

	extract_number (kUCHAR, &parameter_number, kREQUIRED);
	extract_number (kINT, &paramater_value, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	if (1 == parameter_number)
	{
		if (0 == paramater_value || 1 == paramater_value)
		{
			g_ack_enable = paramater_value;			
		}
		else
		{
			bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		}
	}
	print_ack();
}

// "T" Packet
// Causes PIC to sample digital or analog inputs at a regular interval and send
// I (or A) packets back at that interval.
// Send T,0,0<CR> to stop I (or A) packets
// FORMAT: T,<TIME_BETWEEN_UPDATES_IN_MS>,<MODE><CR>
// <MODE> is 0 for digital (I packets) and 1 for analog (A packets)
// EXAMPLE: "T,4000,0<CR>" to send an I packet back every 4 seconds.
// EXAMPLE: "T,2000,1<CR>" to send an A packet back every 2 seconds.
void parse_T_packet(void)
{
	unsigned int value;
	unsigned char mode = 0;

	// Extract the <TIME_BETWEEN_UPDATES_IN_MS> value
	extract_number(kUINT, (void *)&time_between_updates, kREQUIRED);
	// Extract the <MODE> value
	extract_number (kUCHAR, &mode, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Now start up the timer at the right rate or shut 
	// it down.
	if (0 == mode)
	{
		if (0 == time_between_updates)
		{
			// Turn off sending of I packets.
			ISR_D_RepeatRate = 0;
		}
		else
		{
			T2CONbits.TMR2ON = 1;    
		
			// Eventually gaurd this section from interrupts
			ISR_D_RepeatRate = time_between_updates;
		}
	}	
	else
	{
		if (0 == time_between_updates)
		{
			// Turn off sending of A packets.
			ISR_A_RepeatRate = 0;
		}
		else
		{
			T2CONbits.TMR2ON = 1;    
		
			// Eventually gaurd this section from interrupts
			ISR_A_RepeatRate = time_between_updates;
		}
	}
	
	print_ack ();
}


// FORMAT: C,<portA_IO>,<portB_IO>,<portC_IO>,<analog_config><CR>
// EXAMPLE: "C,255,0,4,0<CR>"
// <portX_IO> is the byte sent to the Data Direction (DDR) regsiter for
// each port. A 1 in a bit location means input, a 0 means output.
// <analog_config> is a value between 0 and 13. It tells the UBW
// how many analog inputs to enable. If a zero is sent for this 
// parameter, all analog inputs are disabled.
// For the other values, see the following chart to know what pins are 
// used for what:
// 
// Note that in the following chart, PortE is references. This port
// only exists on the 40 and 44 pin versions of the UBW. For the 
// 28 pin versions of the UBW, all PortE based analog pins will return
// zero.
//
// <analog_config>	Analog Inputs Enabled	Pins Used For Analog Inputs
// ---------------	---------------------	-------------------------------
//	0				<none>					<none>
//	1				AN0						A0
//	2				AN0,AN1					A0,A1	
//	3				AN0,AN1,AN2				A0,A1,A2	
//	4				AN0,AN1,AN2,AN3			A0,A1,A2,A3	
//	5				AN0,AN1,AN2,AN3,AN4		A0,A1,A2,A3,A5		
//	6				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0
//						AN5						
//	7				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1
//						AN5,AN6						
//	8				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1,E2
//						AN5,AN6,AN7						
//	9				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1,E2,B2
//						AN5,AN6,AN7,AN8						
//	10				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1,E2,B2,B3
//						AN5,AN6,AN7,AN8,
//						AN9						
//	11				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1,E2,B2,B3,B1
//						AN5,AN6,AN7,AN8,
//						AB9,AN10						
//	12				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1,E2,B2,B3,B1,B4
//						AN5,AN6,AN7,AN8,
//						AN9,AN10,AN11
// 	13				AN0,AN1,AN2,AN3,AN4,	A0,A1,A2,A3,A5,E0,E1,E2,B2,B3,B1,B4,B0
//						AN5,AN6,AN7,AN8,
//						AN9,AN10,AN11,
//						AN12
// NOTE: it is up to the user to tell the proper port direction bits to be
// inputs for the analog channels they wish to use.
void parse_C_packet(void)
{
	unsigned char PA, PB, PC, AA;
#ifdef __18F4550
	unsigned char PD, PE;
#endif

	// Extract each of the four values.
	extract_number (kUCHAR, &PA, kREQUIRED);
	extract_number (kUCHAR, &PB, kREQUIRED);
	extract_number (kUCHAR, &PC, kREQUIRED);
#ifdef __18F4550
	extract_number (kUCHAR, &PD, kREQUIRED);
	extract_number (kUCHAR, &PE, kREQUIRED);
#endif
	extract_number (kUCHAR, &AA, kREQUIRED);


	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Now write those values to the data direction registers.
	TRISA = PA;
	TRISB = PB;
	TRISC = PC;
#ifdef __18F4550
	TRISD = PD;
	TRISE = PE;
#endif
	
	// Handle the analog value.
	// Maximum value of 13.
	if (AA > 13)
	{
		AA = 13;
	}
	
	// If we are turning off Analog inputs
	if (0 == AA)
	{
		// Turn all analog inputs into digital inputs
		ADCON1 = 0x0F;
		// Turn off the ADC
		ADCON0bits.ADON = 0;
		// Turn off our own idea of how many analog channels to convert
		AnalogEnable = 0;
	}
	else
	{
		// Some protection from ISR
		AnalogEnable = 0;
	
		// We're turning some on.
		// Start by selecting channel zero		
		ADCON0 = 0;
	
		// Then enabling the proper number of channels
		ADCON1 = 15 - AA;
	
		// Set up ADCON2 options
		// A/D Result right justified
		// Acq time = 20 Tad (?)
		// Tad = Fosc/64
		ADCON2 = 0b10111110;
	
		// Turn on the ADC
		ADCON0bits.ADON = 1;
	
		// Tell ourselves how many channels to convert, and turn on ISR conversions
		AnalogEnable = AA;
	
		T2CONbits.TMR2ON = 1;
	}
	
	print_ack ();
}

// Outputs values to the ports pins that are set up as outputs.
// Example "O,121,224,002<CR>"
void parse_O_packet(void)
{
	unsigned char Value;
	ExtractReturnType RetVal;

	// Extract each of the values.
	RetVal = extract_number (kUCHAR,  &Value, kREQUIRED);
	if (error_byte) return;
	if (kEXTRACT_OK == RetVal)
	{
		LATA = Value;
	}
	RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
	if (error_byte) return;
	if (kEXTRACT_OK == RetVal)
	{
		LATB = Value;
	}
	RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
	if (error_byte) return;
	if (kEXTRACT_OK == RetVal)
	{
		LATC = Value;
	}
#ifdef __18F4550
	RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
	if (error_byte) return;
	if (kEXTRACT_OK == RetVal)
	{
		LATD = Value;
	}
	RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
	if (error_byte) return;
	if (kEXTRACT_OK == RetVal)
	{
		LATE = Value;
	}
#endif
		
	print_ack ();
}

// Read in the three I/O ports (A,B,C) and create
// a packet to send back with all of values.
// Example: "I,143,221,010<CR>"
// Remember that on UBW 28 pin boards, we only have
// Port A bits 0 through 5
// Port B bits 0 through 7
// Port C bits 0,1,2 and 6,7
// And that Port C bits 0,1,2 are used for
// 		User1 LED, User2 LED and Program switch respectively.
// The rest will be read in as zeros.
void parse_I_packet(void)
{
#ifdef __18F4550
	printf (
		(far rom char*)"I,%03i,%03i,%03i,%03i,%03i\r\n", 
		PORTA,
		PORTB,
		PORTC,
		PORTD,
		PORTE
	);
#else
	printf (
		(far rom char*)"I,%03i,%03i,%03i\r\n", 
		PORTA,
		PORTB,
		PORTC
	);
#endif
}

// All we do here is just print out our version number
void parse_V_packet(void)
{
	printf ((far rom char *)st_version);
}

// A is for read Analog inputs
// Just print out the last analog values for each of the
// enabled channels. The number of value returned in the
// A packet depend upon the number of analog inputs enabled.
// The user can enabled any number of analog inputs between 
// 0 and 12. (none enabled, through all 12 analog inputs enabled).
// Returned packet will look like "A,0,0,0,0,0,0<CR>" if
// six analog inputs are enabled but they are all
// grounded. Note that each one is a 10 bit (or 12 bit if you 
// have a 12 bit ADC)
// value, where 0 means the intput was at ground, and
// 1024 means it was at +5 V. (Or whatever the USB +5 
// pin is at.) 
void parse_A_packet(void)
{
	char channel = 0;

	// Put the beginning of the packet in place
	printf ((far rom char *)"A");
	
	// Now add each analog value
	for (channel = 0; channel < AnalogEnable; channel++)
	{
		printf(
			(far rom char *)",%04u" 
			,ISR_A_FIFO[channel][ISR_A_FIFO_out]
		);
	}
	
	// Add \r\n and terminating zero.
	printf ((far rom char *)st_LFCR);
}

// MW is for Memory Write
// "MW,<location>,<value><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to write to 
// <value> is a decimal value between 0 and 255 that is the value to write
void parse_MW_packet(void)
{
	unsigned int location;
	unsigned char value;

	extract_number (kUINT, &location, kREQUIRED);
	extract_number (kUCHAR, &value, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}
	// Limit check the address and write the byte in
	if (location < 4096)
	{
		*((unsigned char *)location) = value;
	}
	
	print_ack ();
}


// MR is for Memory Read
// "MW,<location><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to read from 
// The UBW will then send a "MR,<value><CR>" packet back to the PC
// where <value> is the byte value read from the address
void parse_MR_packet(void)
{
	unsigned int location;
	unsigned char value;

	extract_number (kUINT, &location, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Limit check the address and write the byte in
	if (location < 4096)
	{
		value = *((unsigned char *)location);
	}
	
	// Now send back the MR packet
	printf (
		(far rom char *)"MR,%03u\r\n" 
		,value
	);
}

// PD is for Pin Direction
// "PD,<port>,<pin>,<direction><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to change direction on
// <direction> is "1" for input, "0" for output
void parse_PD_packet(void)
{
	unsigned char port;
	unsigned char pin;
	unsigned char direction;

	extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
	extract_number (kUCHAR, &pin, kREQUIRED);
	extract_number (kUCHAR, &direction, kREQUIRED);
	
	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Limit check the parameters
	if (direction > 1)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	if (pin > 7)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	if ('A' == port)
	{
		if (0 == direction)
		{
			bitclr (TRISA, pin);  	
		}
		else
		{
			bitset (TRISA, pin);  	
		}
	}
	else if ('B' == port)
	{
		if (0 == direction)
		{
			bitclr (TRISB, pin);  	
		}
		else
		{
			bitset (TRISB, pin);  	
		}		
	}
	else if ('C' == port)
	{
		if (0 == direction)
		{
			bitclr (TRISC, pin);  	
		}
		else
		{
			bitset (TRISC, pin);  	
		}		
	}
#ifdef __18F4550
	else if ('D' == port)
	{
		if (0 == direction)
		{
			bitclr (TRISD, pin);  	
		}
		else
		{
			bitset (TRISD, pin);  	
		}		
	}
	else if ('E' == port)
	{
		if (0 == direction)
		{
			bitclr (TRISE, pin);  	
		}
		else
		{
			bitset (TRISE, pin);  	

		}		
	}
#endif
	else
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;	
	}
	
	print_ack ();
}

// PI is for Pin Input
// "PI,<port>,<pin><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to read
// The command returns a "PI,<value><CR>" packet,
// where <value> is the value (0 or 1 for digital)
// value for that pin.
void parse_PI_packet(void)
{
	unsigned char port;
	unsigned char pin;
	unsigned char value = 0;

	extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
	extract_number (kUCHAR, &pin, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Limit check the parameters
	if (pin > 7)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	
	// Then test the bit in question based upon port
	if ('A' == port)
	{
		value = bittst (PORTA, pin);  	
	}
	else if ('B' == port)
	{
		value = bittst (PORTB, pin);  	
	}
	else if ('C' == port)
	{
		value = bittst (PORTC, pin);  	
	}
#ifdef __18F4550
	else if ('D' == port)
	{
		value = bittst (PORTD, pin);  	
	}
	else if ('E' == port)
	{
		value = bittst (PORTE, pin);  	
	}
#endif
	else
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;	
	}
	
	// Value is now non-zero if the bit tests was true. But it may not be 1! It might be more than that.
	if (value)
	{
		value = 1;
	}

	// Now send back our response
	printf(
		 (far rom char *)"PI,%1u\r\n" 
		,value
	);
}

// PO is for Pin Output
// "PO,<port>,<pin>,<value><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to write out the value to
// <value> is "1" or "0" and indicates the state to change the pin to
void parse_PO_packet(void)
{
	unsigned char port;
	unsigned char pin;
	unsigned char value;

	extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
	extract_number (kUCHAR, &pin, kREQUIRED);
	extract_number (kUCHAR, &value, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Limit check the parameters
	if (value > 1)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	if (pin > 7)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	if ('A' == port)
	{
		if (0 == value)
		{
			bitclr (LATA, pin);  	
		}
		else
		{
			bitset (LATA, pin);  	
		}
	}
	else if ('B' == port)
	{
		if (0 == value)
		{
			bitclr (LATB, pin);  	
		}
		else
		{
			bitset (LATB, pin);  	
		}		
	}
	else if ('C' == port)
	{
		if (0 == value)
		{
			bitclr (LATC, pin);  	
		}
		else
		{
			bitset (LATC, pin);  	
		}		
	}
#ifdef __18F4550
	else if ('D' == port)
	{
		if (0 == value)
		{
			bitclr (LATD, pin);  	
		}
		else
		{
			bitset (LATD, pin);  	
		}		
	}
	else if ('E' == port)
	{
		if (0 == value)
		{
			bitclr (LATE, pin);  	
		}
		else
		{
			bitset (LATE, pin);  	
		}		
	}
#endif
	else
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;	
	}
	
	print_ack ();
}

// TX is for Serial Transmit
// "TX,<data_length>,<variable_length_data><CR>"
// <data_length> is a count of the number of bytes in the <variable_length_data> field.
// It must never be larger than the number of bytes that are currently free in the
// software TX buffer or some data will get lost.
// <variable_length_data> are the bytes that you want the UBW to send. It will store them
// in its software TX buffer until there is time to send them out the TX pin.
// If you send in "0" for a <data_length" (and thus nothing for <variable_length_data>
// then the UBW will send back a "TX,<free_buffer_space><CR>" packet,
// where <free_buffer_space> is the number of bytes currently available in the 
// software TX buffer.
void parse_TX_packet(void)
{
	print_ack ();
}

// RX is for Serial Receive
// "RX,<length_request><CR>"
// <length_request> is the maximum number of characters that you want the UBW to send
// back to you in the RX packet. If you use "0" for <length_request> then the UBW
// will just send you the current number of bytes in it's RX buffer, and if
// there have been any buffer overruns since the last time a <length_request> of 
// "0" was received by the UBW.
// This command will send back a "RX,<length>,<variable_length_data><CR>"
// or "RX,<buffer_fullness>,<status><CR>" packet depending upon if you send
// "0" or something else for <length_request>
// <length> in the returning RX packet is a count of the number of bytes
// in the <variable_length_data> field. It will never be more than the
// <length_request> you sent in.
// <variable_length_data> is the data (in raw form - byte for byte what was received - 
// i.e. not translated in any way, into ASCII values or anything else) that the UBW
// received. This may include <CR>s and NULLs among any other bytes, so make sure
// your PC application treates the RX packet coming back from the UBW in a speical way
// so as not to screw up normal packet processing if any special caracters are received.
// <buffer_fullness> is a valule between 0 and MAX_SERIAL_RX_BUFFER_SIZE that records
// the total number of bytes, at that point in time, that the UBW is holding, waiting
// to pass on to the PC.
// <status> has several bits. 
//	Bit 0 = Software RX Buffer Overrun (1 means software RX buffer (on RX pin)
//		has been overrun and data has been lost) This will happen if you don't
//		read the data out of the UWB often enough and the data is coming in too fast.
//	Bit 1 = Software TX Buffer Overrun (1 means software TX buffer (on TX pin)
//		as been overrun and data hs been lost. This will happen if you send too much
//		data to the UBW and you have the serial port set to a low baud rate.
void parse_RX_packet(void)
{
	print_ack ();
}

// CX is for setting up serial port parameters
// TBD
void parse_CX_packet(void)
{
	print_ack ();
}

// RC is for outputting RC servo pulses on a pin
// "RC,<port>,<pin>,<value><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to output the new value on
// <value> is an unsigned 16 bit number between 0 and 11890.
// If <value> is "0" then the RC output on that pin is disabled.
// Otherwise <value> = 1 means 1ms pulse, <value> = 11890 means 2ms pulse,
// any value inbetween means proportional pulse values between those two
// Note: The pin used for RC output must be set as an output, or not much will happen.
// The RC command will continue to send out pulses at the last set value on 
// each pin that has RC output with a repition rate of 1 pulse about every 19ms.
// If you have RC output enabled on a pin, outputting a digital value to that pin
// will be overwritten the next time the RC pulses. Make sure to turn off the RC
// output if you want to use the pin for something else.
void parse_RC_packet(void)
{
	unsigned char port;
	unsigned char pin;
	unsigned int value;

	extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
	extract_number (kUCHAR, &pin, kREQUIRED);
	extract_number (kUINT, &value, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Max value user can input. (min is zero)
	if (value > 11890)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	
	// Now get Value in the form that TMR0 needs it
	// TMR0 needs to get filled with values from 65490 (1ms) to 53600 (2ms)
	if (value != 0)
	{
		value = (65535 - (value + 45));
	}

	if (pin > 7)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	if ('A' == port)
	{
		port = 0;
	}
	else if ('B' == port)
	{
		port = 8;
	}
	else if ('C' == port)
	{
		port = 16;
	}
	else
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;	
	}

	// Store the new RC time valuebuwloloh
	
	g_RC_value[pin + port] = value;
	// Only set this state if we are off - if we are already running on 
	// this pin, then the new value will be picked up next time around (19ms)
	if (kOFF == g_RC_state[pin + port])
	{
		g_RC_state[pin + port] = kWAITING;
	}

	print_ack ();
}

// BC is for Bulk Configure
// BC,<port A init>,<waitmask>,<wait delay>,<strobemask>,<strobe delay><CR>
// This command sets up the mask and strobe bits on port A for the
// BO (Bulk Output) command below. Also suck in wait delay, strobe delay, etc.
void parse_BC_packet(void)
{
	unsigned char BO_init;
	unsigned char BO_strobe_mask;
	unsigned char BO_wait_mask;
	unsigned char BO_wait_delay;
	unsigned char BO_strobe_delay;

	extract_number (kUCHAR, &BO_init, kREQUIRED);
	extract_number (kUCHAR, &BO_wait_mask, kREQUIRED);
	extract_number (kUCHAR, &BO_wait_delay, kREQUIRED);
	extract_number (kUCHAR, &BO_strobe_mask, kREQUIRED);
	extract_number (kUCHAR, &BO_strobe_delay, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Copy over values to their gloabls
	g_BO_init = BO_init;
	g_BO_wait_mask = BO_wait_mask;
	g_BO_strobe_mask = BO_strobe_mask;
	g_BO_wait_delay = BO_wait_delay;
	g_BO_strobe_delay = BO_strobe_delay;
	// And initalize Port A
	LATA = g_BO_init;
	
	print_ack ();
}

// Bulk Output (BO)
// BO,4AF2C124<CR>
// After the inital comma, pull in hex values and spit them out to port A
// Note that the procedure here is as follows:
//	1) Write new value to PortB
//	2) Assert <strobemask>
//	3) Wait for <strobdelay> (if not zero)
//	4) Deassert <strobemask>
//	5) Wait for <waitmask> to be asserted
//	6) Wait for <waitmask> to be deasserted
//	7) If 5) or 6) takes longer than <waitdelay> then just move on to next byte
//	Repeat for each byte
void parse_BO_packet(void)
{
	unsigned char BO_data_byte;
	unsigned char new_port_A_value;
	unsigned char tmp;
	unsigned char wait_count = 0;
	
	// Check for comma where ptr points
	if (g_RX_buf[g_RX_buf_out] != ',')
	{
		printf ((far rom char *)"!5 Err: Need comma next, found: '%c'\r\n", g_RX_buf[g_RX_buf_out]);
		bitset (error_byte, kERROR_BYTE_PRINTED_ERROR);
		return;
	}

	// Move to the next character
	advance_RX_buf_out ();

	// Make sure Port A is correct
	LATA = g_BO_init;
	new_port_A_value = ((~LATA & g_BO_strobe_mask)) | (LATA & ~g_BO_strobe_mask);
	
	while (g_RX_buf[g_RX_buf_out] != 13)
	{
		// Pull in a nibble from the input buffer
		tmp = toupper (g_RX_buf[g_RX_buf_out]);
		if (tmp >= '0' && tmp <= '9')
		{
			tmp -= '0';	
		}
		else if (tmp >= 'A' && tmp <= 'F')
		{
			tmp -= 55;
		}
		else 
		{
			bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
			return;
		}
		BO_data_byte = tmp << 4;
		advance_RX_buf_out ();

		// Check for CR next
		if (kCR == g_RX_buf[g_RX_buf_out])
		{
			bitset (error_byte, kERROR_BYTE_MISSING_PARAMETER);
			return;
		}

		tmp =  toupper (g_RX_buf[g_RX_buf_out]);
		if (tmp >= '0' && tmp <= '9')
		{
			tmp -= '0';	
		}
		else if (tmp >= 'A' && tmp <= 'F')
		{
			tmp -= 55;
		}
		else
		{
			bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
			return;
		}
		BO_data_byte = BO_data_byte + tmp;
		advance_RX_buf_out ();
	
		// Output the byte on Port B
		LATB = BO_data_byte;
		
		// And strobe the Port A bits that we're supposed to
		LATA = new_port_A_value;
		if (g_BO_strobe_delay)
		{
			Delay10TCYx (g_BO_strobe_delay);
		}
		LATA = g_BO_init;

		if (g_BO_wait_delay)
		{
			// Now we spin on the wait bit specified in WaitMask
			// (Used for Busy Bits) We also have to wait here
			// for a maximum of g_BO_wait_delay, which is in 10 clock units
			// First we wait for the wait mask to become asserted

			// Set the wait counter to the number of delays we want
			wait_count = g_BO_wait_delay;
			while (
				((g_BO_init & g_BO_wait_mask) == (PORTA & g_BO_wait_mask))
				&& 
				(wait_count != 0)
			)
			{
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				wait_count--;
			}

			// Set the wait counter to the number of delays we want
			wait_count = g_BO_wait_delay;
			// Then we wait for the wait mask to become de-asserted
			while ( 
				((g_BO_init & g_BO_wait_mask) != (PORTA & g_BO_wait_mask))
				&&
				(wait_count != 0)
			)
			{
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				wait_count--;
			}
		}
	}
	print_ack ();
}

// Bulk Stream (BS) (he he, couldn't think of a better name)
// BS,<count>,<binary_data><CR>
// This command is extremely similar to the BO command
// except that instead of ASCII HEX values, it actually 
// takes raw binary data.
// So in order for the UBW to know when the end of the stream
// is, we need to have a <count> of bytes.
// <count> represents the number of bytes after the second comma
// that will be the actual binary data to be streamed out port B.
// Then, <binary_data> must be exactly that length.
// <count> must be between 1 and 56 (currently - in the future
// it would be nice to extend the upper limit)
// The UBW will pull in one byte at a time within the <binary_data>
// section and output it to PORTB exactly as the BO command does.
// It will do this for <count> bytes. It will then pull in another
// byte (which must be a carrige return) and be done.
// The whole point of this command is to improve data throughput
// from the PC to the UBW. This form of data is also more efficient
// for the UBW to process.
void parse_BS_packet(void)
{
	unsigned char BO_data_byte;
	unsigned char new_port_A_value;
	unsigned char tmp;
	unsigned char wait_count = 0;
	unsigned char byte_count = 0;	

	// Get byte_count
	extract_number (kUCHAR, &byte_count, kREQUIRED);
	
	// Limit check it
	if (0 == byte_count || byte_count > 56)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}

	// Check for comma where ptr points
	if (g_RX_buf[g_RX_buf_out] != ',')
	{
		printf ((far rom char *)"!5 Err: Need comma next, found: '%c'\r\n", g_RX_buf[g_RX_buf_out]);
		bitset (error_byte, kERROR_BYTE_PRINTED_ERROR);
		return;
	}

	// Move to the next character
	advance_RX_buf_out ();

	// Make sure Port A is correct
	LATA = g_BO_init;
	new_port_A_value = ((~LATA & g_BO_strobe_mask)) | (LATA & ~g_BO_strobe_mask);
	
	while (byte_count != 0)
	{
		// Pull in a single byte from input buffer
		BO_data_byte = g_RX_buf[g_RX_buf_out];
		advance_RX_buf_out ();

		// Count this byte
		byte_count--;
	
		// Output the byte on Port B
		LATB = BO_data_byte;
		
		// And strobe the Port A bits that we're supposed to
		LATA = new_port_A_value;
		if (g_BO_strobe_delay)
		{
			Delay10TCYx (g_BO_strobe_delay);
		}
		LATA = g_BO_init;

		if (g_BO_wait_delay)
		{
			// Now we spin on the wait bit specified in WaitMask
			// (Used for Busy Bits) We also have to wait here
			// for a maximum of g_BO_wait_delay, which is in 10 clock units
			// First we wait for the wait mask to become asserted

			// Set the wait counter to the number of delays we want
			wait_count = g_BO_wait_delay;
			while (
				((g_BO_init & g_BO_wait_mask) == (PORTA & g_BO_wait_mask))
				&& 
				(wait_count != 0)
			)
			{
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				wait_count--;
			}

			// Set the wait counter to the number of delays we want
			wait_count = g_BO_wait_delay;
			// Then we wait for the wait mask to become de-asserted
			while ( 
				((g_BO_init & g_BO_wait_mask) != (PORTA & g_BO_wait_mask))
				&&
				(wait_count != 0)
			)
			{
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				Delay1TCY ();
				wait_count--;
			}
		}
	}
	print_ack ();
}

// SS Send SPI
void parse_SS_packet (void)
{
	print_ack ();

}	

// RS Receive SPI
void parse_RS_packet (void)
{
	print_ack ();

}	

// CS Configure SPI
void parse_CS_packet (void)
{
	print_ack ();

}	

// SI Send I2C
void parse_SI_packet (void)
{
	print_ack ();

}	

// RI Receive I2C
void parse_RI_packet (void)
{
	print_ack ();

}	

// CI Configure I2C
void parse_CI_packet (void)
{
	print_ack ();

}	



// F is for square wave out on specified pin (PWM)
// "F,<freq>,<port>,<pin>,<duty_cycle><CR>"
// <freq> is a value from 0 through 32,000. Using 0 will shut off the 
//   frequency output on the pin. 1 through 32,000 will use that as the frequency
// <port> is "A", "B", "C" and indicates the port ("D" and "E" for 18F4550s)
// <pin> is a number between 0 and 7 and indicates which pin used for output
// <duty_cycle> is optional, and is from 1 through 99 and indicates the percentage of the 
//   <freq> period that the pin should be high (effectively PWM)

void parse_F_packet(void)
{
	unsigned long valueOn, valueOff;
	unsigned int freq;
	unsigned char port;
	unsigned char pin;
	unsigned char dutyCycle = 50;
	static unsigned char OldPin;
	static unsigned char OldPort;
	unsigned char TempPrescaleOn = 1, TempPrescaleOff = 1;

	extract_number (kUINT, &freq, kREQUIRED);

	// If the user is turning off the frequency command, (value = 0)
	// make the pin and port optional.
	if (0 == freq)
	{
		extract_number (kUCASE_ASCII_CHAR, &port, kOPTIONAL);
		extract_number (kUCHAR, &pin, kOPTIONAL);
	}
	else
	{
		extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
		extract_number (kUCHAR, &pin, kREQUIRED);
	}
	extract_number (kUCHAR, &dutyCycle, kOPTIONAL);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// check if this is the 'stop' command.
	if (freq == 0)
	{
		// halt the timer
		T3CONbits.TMR3ON = 0;

		// disable the interrupt
		PIE2bits.TMR3IE = 0;

		// Shut off the pin
		if ('A' == OldPort)
		{
			bitclr (LATA, OldPin);
		}
		else if ('B' == OldPort)
		{
			bitclr (LATB, OldPin);	
		}
		else if ('C' == OldPort)
		{
			bitclr (LATC, OldPin);
		}
#ifdef __18F4550
		else if ('D' == OldPort)
		{
			bitclr (LATD, OldPin);
		}
		else if ('E' == OldPort)
		{
			bitclr (LATE, OldPin);
		}
#endif

		print_ack ();

		return;
	}

	valueOn = (12000000L * dutyCycle) /((long)(freq) * 100);
	valueOff = (12000000L * (100 - dutyCycle))/((long)(freq) * 100);

	// Check our prescale range
	if (valueOn > 0xFFFF)
	{
		TempPrescaleOn = F_COMMAND_PRESCALE;
		valueOn = valueOn/F_COMMAND_PRESCALE;
	}
	else
	{
		TempPrescaleOn = 1;

	}

	// Make sure we don't wrap around
	if (valueOn < F_COMMAND_MIN_CYCLES)
	{
		valueOn = F_COMMAND_MIN_CYCLES;
	}

	if (valueOff > 0xFFFF)
	{
		TempPrescaleOff = F_COMMAND_PRESCALE;
		valueOff = (valueOff/F_COMMAND_PRESCALE);
	}
	else
	{
		TempPrescaleOff = 1;
	}
	// Make sure we don't wrap around
	if (valueOff < F_COMMAND_MIN_CYCLES)
	{
		valueOff = F_COMMAND_MIN_CYCLES;
	}

	valueOff = valueOff - F_COMMAND_EXTRA_CYCLES;
	valueOn = valueOn - F_COMMAND_EXTRA_CYCLES;

	// Limit check the parameters
	if (pin > 7)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}

	if (dutyCycle < 0 || dutyCycle > 99)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}	

	// Users are entering 'freq' as the desired frequency.
	// We need to do the following
	// value = (12,000,000/freq)/2
	// we also need to apply a prescale if we're below a threshold so we can get down to 1 HZ
	// value = (12,000,000/freq)/2/FCommandPrescale 
	
	// Only update the bits if we are NOT already running
	if (T3CONbits.TMR3ON == 0)
	{
		// Setup the port and pin requested 
		FCommandPortAMask = 0;
		FCommandPortBMask = 0;
		FCommandPortCMask = 0;
#ifdef __18F4550
		FCommandPortDMask = 0;
		FCommandPortEMask = 0;
#endif
		if ('A' == port)
		{
			bitclr (TRISA, pin); 
			bitset (FCommandPortAMask, pin);
		}
		else if ('B' == port)
		{
			bitclr (TRISB, pin);   
			bitset (FCommandPortBMask, pin);	
		}
		else if ('C' == port)
		{
			bitclr (TRISC, pin); 
			bitset (FCommandPortCMask, pin);
		}
#ifdef __18F4550
		else if ('D' == port)
		{
			bitclr (TRISD, pin);
			bitset (FCommandPortDMask, pin);
		}
		else if ('E' == port)
		{
			bitclr (TRISE, pin);
			bitset (FCommandPortEMask, pin);
		}
#endif
		else
		{
			bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
			return;	
		}
		
		OldPin = pin;
		OldPort = port;
		
		// We always start by setting our bit
		FCommandPinHigh = TRUE;
		FCommandPrescaleCount = 0;
	}

	// Limit check our on and off times
	if (valueOn > 0xFFFF || valueOff > 0xFFFF)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;	
	}	
	
	//Setup and activate Timer 3
	valueOn = 0xFFFF - valueOn;	
	valueOff = 0xFFFF - valueOff;	
	// Make sure no interrupts happen here:
	INTCONbits.GIEH = 0;          //disable interrupts
	FCommandReloadValueOn_high = valueOn >> 8;
	FCommandReloadValueOn_low = valueOn & 0xFF;
	FCommandReloadValueOff_high = valueOff >> 8;
	FCommandReloadValueOff_low = valueOff & 0xFF;
	FCommandPrescaleOn = TempPrescaleOn;
	FCommandPrescaleOff = TempPrescaleOff;
	INTCONbits.GIEH = 1;          //enable interrupts

	// Only update the bits if we are NOT already running
	if (T3CONbits.TMR3ON == 0)
	{
		T3CONbits.TMR3CS = 0;
		T3CONbits.T3CKPS0 = 0;	
		T3CONbits.T3CKPS1 = 0;
	
		IPR2bits.TMR3IP = 1;
		PIR2bits.TMR3IF = 0;
		PIE2bits.TMR3IE = 1;
	
		T3CONbits.TMR3ON = 1;
	}
	
	// Now send back our response
	print_ack ();
}

// Look at the string pointed to by ptr
// There should be a comma where ptr points to upon entry.
// If not, throw a comma error.
// If so, then look for up to three bytes after the
// comma for numbers, and put them all into one
// byte (0-255). If the number is greater than 255, then
// thow a range error.
// Advance the pointer to the byte after the last number
// and return.
ExtractReturnType extract_number(
	ExtractType Type, 
	void * ReturnValue, 
	unsigned char Required
)
{
	signed short long Accumulator;
	unsigned char Negative = FALSE;

	// Check to see if we're already at the end
	if (kCR == g_RX_buf[g_RX_buf_out])
	{
		if (0 == Required)
		{
			bitset (error_byte, kERROR_BYTE_MISSING_PARAMETER);
		}
		return (kEXTRACT_MISSING_PARAMETER);
	}

	// Check for comma where ptr points
	if (g_RX_buf[g_RX_buf_out] != ',')
	{
		if (0 == Required)
		{
			printf ((rom char far *)"!5 Err: Need comma next, found: '%c'\r\n", g_RX_buf[g_RX_buf_out]);
			bitset (error_byte, kERROR_BYTE_PRINTED_ERROR);
		}
		return (kEXTRACT_COMMA_MISSING);
	}

	// Move to the next character
	advance_RX_buf_out ();

	// Check for end of command
	if (kCR == g_RX_buf[g_RX_buf_out])
	{
		if (0 == Required)
		{
			bitset (error_byte, kERROR_BYTE_MISSING_PARAMETER);
		}
		return (kEXTRACT_MISSING_PARAMETER);
	}
	
	// Now check for a sign character if we're not looking for ASCII chars
	if (
		('-' == g_RX_buf[g_RX_buf_out]) 
		&& 
		(
			(kASCII_CHAR != Type)
			&&
			(kUCASE_ASCII_CHAR != Type)
		)
	)
	{
		// It's an error if we see a negative sign on an unsigned value
		if (
			(kUCHAR == Type)
			||
			(kUINT == Type)
		)
		{
			bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
			return (kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
		}
		else
		{
			Negative = TRUE;
			// Move to the next character
			advance_RX_buf_out ();
		}
	}

	// If we need to get a digit, go do that
	if (
		(kASCII_CHAR != Type)
		&&
		(kUCASE_ASCII_CHAR != Type)
	)
	{
		extract_digit(&Accumulator, 5);
	}
	else
	{
		// Otherwise just copy the byte
		Accumulator = g_RX_buf[g_RX_buf_out];
	
		// Force uppercase if that's what type we have
		if (kUCASE_ASCII_CHAR == Type)
		{
			Accumulator = toupper (Accumulator);
		}
		
		// Move to the next character
		advance_RX_buf_out ();
	}

	// Handle the negative sign
	if (Negative)
	{
		Accumulator = -Accumulator;
	}

	// Range check the new value
	if (
		(
			kCHAR == Type
			&&
			(
				(Accumulator > 127)
				||
				(Accumulator < -128)
			)
		)
		||
		(
			kUCHAR == Type
			&&
			(
				(Accumulator > 255)
				||
				(Accumulator < 0)
			)
		)
		||
		(
			kINT == Type
			&&
			(
				(Accumulator > 32767)
				||
				(Accumulator < -32768)
			)
		)
		||
		(
			kUINT == Type
			&&
			(
				(Accumulator > 65535)
				||
				(Accumulator < 0)
			)
		)
	)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return (kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
	}

	// If all went well, then copy the result
	switch (Type)
	{	
		case kCHAR:
			*(signed char *)ReturnValue = (signed char)Accumulator;
			break;
		case kUCHAR:
		case kASCII_CHAR:
		case kUCASE_ASCII_CHAR:
			*(unsigned char *)ReturnValue = (unsigned char)Accumulator;
			break;
		case kINT:
			*(signed int *)ReturnValue = (signed int)Accumulator;
			break;
		case kUINT:
			*(unsigned int *)ReturnValue = (unsigned int)Accumulator;
			break;
		default:
			return (kEXTRACT_INVALID_TYPE);
	}	
	return(kEXTRACT_OK);	
}

// Loop 'digits' number of times, looking at the
// byte in input_buffer index *ptr, and if it is
// a digit, adding it to acc. Take care of 
// powers of ten as well. If you hit a non-numerical
// char, then return FALSE, otherwise return TRUE.
// Store result as you go in *acc.
signed char extract_digit(signed short long * acc,	unsigned char digits)
{
	unsigned char val;
	unsigned char digit_cnt;
	
	*acc = 0;

	for (digit_cnt = 0; digit_cnt < digits; digit_cnt++)
	{
		val = g_RX_buf[g_RX_buf_out];
		if ((val >= 48) && (val <= 57))
		{
			*acc = (*acc * 10) + (val - 48);
			// Move to the next character
			advance_RX_buf_out ();
		}
		else
		{
			return (FALSE);
		}
	}
	return (TRUE);
}


// For debugging, this command will spit out a bunch of values.
void print_status(void)
{
	printf( 
		(far rom char*)"Status=%i\r\n"
		,ISR_D_FIFO_length
	);
}

/******************************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs corresponding to
 *                  the USB device state.
 *
 * Note:            mLED macros can be found in io_cfg.h
 *                  usb_device_state is declared in usbmmap.c and is modified
 *                  in usbdrv.c, usbctrltrf.c, and usb9.c
 ******************************************************************************/
void BlinkUSBStatus(void)
{
    static WORD LEDCount = 0;
	static unsigned char LEDState = 0;
    
    if (
		USBDeviceState == DETACHED_STATE
       	||
       	1 == USBSuspendControl
    )
    {
		LEDCount--;
		if (0 == LEDState)
		{
			if (0 == LEDCount)
			{
				mLED_1_On();
				LEDCount = 2000U;				
				LEDState = 1;
			}
		}
		else
		{
			if (0 == LEDCount)
			{
				mLED_1_Off();
				LEDCount = 2000U;				
				LEDState = 0;
			}
		}
	}
    else if (
		USBDeviceState == ATTACHED_STATE
		||
		USBDeviceState == POWERED_STATE		
		||
		USBDeviceState == DEFAULT_STATE
		||
		USBDeviceState == ADDRESS_STATE
	)
    {
        mLED_1_On();
    }
    else if (USBDeviceState == CONFIGURED_STATE)
    {
		LEDCount--;
		if (0 == LEDState)
		{
			if (0 == LEDCount)
			{
				mLED_1_On();
				LEDCount = 10000U;				
				LEDState = 1;
			}
		}
		else if (1 == LEDState)
		{
			if (0 == LEDCount)
			{
				mLED_1_Off();
				LEDCount = 10000U;				
				LEDState = 2;
			}
		}
		else if (2 == LEDState)
		{
			if (0 == LEDCount)
			{
				mLED_1_On();
				LEDCount = 100000U;				
				LEDState = 3;
			}
		}
		else if (LEDState >= 3)
		{
			if (0 == LEDCount)
			{
				mLED_1_Off();
				LEDCount = 10000U;				
				LEDState = 0;
			}
		}
    }
}

BOOL SwitchIsPressed(void)
{
	if( 0 == swProgram)                   	// If pressed
	{
	    return (TRUE);                	// Was pressed
	}
	else
	{
		return (FALSE);					// Was not pressed
	}
}

/** EOF user.c ***************************************************************/
