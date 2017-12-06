/***********************************************************************************
  Filename:     uart_isr.c

  Description:  This example sends/receives data on UART0, using interrupt method.

  Comments:     To execute this example, compile one SmartRF05EB unit as transmitter,
                by activating the UART_TST_MODE_TX definition, then compile another
                unit as receiver, by activating the UART_TST_MODE_RX definition.
                Pressing S1 makes the transmitter send its allocated uartTxBuffer[]
                to the receiver. The transferred data will arrive at uartRxBuffer[]
                and can be verified using the debugger.

  Note:         Once the UART receiver has been enabled (U0CSR.RE = 1) it will
                automatically trigger data reception as soon the Start bit is
                detected. The transmitter I/O should therefore be configured
                before the receiver, such that potential pin toggling (during
                I/O configuration of the transmitter) does not trigger data
                reception at the receiver side. Also, remember to use common
                ground for the TX unit and RX unit!

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_types.h>
#include <hal_wait.h>
// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
#include <ioCC254x_bitdef.h>
// Include device specific file
#if (chip==2541)
#include "ioCC2541.h"
#elif (chip==2543)
#include "ioCC2543.h"
#elif (chip==2544)
#include "ioCC2544.h"
#warning "The CC2544Dongle is not supported for this software example."
#warning "The definitions for CC2544 in the code illustrate how to set up an SPI interface."
#elif (chip==2545)
#include "ioCC2545.h"
#else
#error "Chip not supported!"
#endif


/***********************************************************************************
* CONSTANTS
*/

// Size of allocated UART RX/TX buffer (just an example)
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

// UART test characters
#define UART_TEST_DATA "Texas Instruments LPRF!"

// Test definitions
#define UART_TST_MODE_RX
//#define UART_TST_MODE_TX

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 216, U0GCR.BAUD_E = 10), given 32 MHz system clock
#define UART_BAUD_M  216
#define UART_BAUD_E  10


/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer+index for UART RX/TX.
static uint8 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint8 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER] = UART_TEST_DATA;
static uint16 __xdata uartTxIndex;
static uint16 __xdata uartRxIndex;

// Variable for UART packet monitoring.
static uint8 __xdata uartPktReceived = 1;

// Prototype for local functions.
void uart0StartRxForIsr(void);
void uart0StartTxForIsr(void);


/***********************************************************************************
* @fn          main
*
* @brief       Send/receive data on UART0, using interrupt.
*
* @param       void
*
* @return      void
*/
void main (void)
{
    /****************************************************************************
    * Clock setup
    * See basic software example "clk_xosc_cc254x"
    */
  
    // Set system clock source to HS XOSC, with no pre-scaling.
    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
    // Wait until clock source has changed.
    while (CLKCONSTA & CLKCON_OSC);   
  
    // Note the 32 kHz RCOSC starts calibrating, if not disabled.
  
  
    /***************************************************************************
    * Setup I/O ports
    *
    * Port and pins used by USART0 operating in UART-mode, at the Alternative 1
    * location are:
    * RX     : P0_2
    * TX     : P0_3
    * CT/CTS : P0_4
    * RT/RTS : P0_5
    *
    * These pins must be set to function as peripheral I/O to be used by UART0.
    * The TX pin on the transmitter must be connected to the RX pin on the receiver.
    * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
    * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
    * receiver.
    */ 
#if (chip==2541 || chip==2543 || chip==2545)
    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0).
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U0CFG_ALT1;    
#endif
        
#if (chip==2541)
    // Give priority to USART 0 over Timer 1 for port 0 pins.
    P2DIR &= P2DIR_PRIP0_USART0;
#elif (chip==2543 || chip==2545)
    // Give priority to USART 0 over Timer 1 for port 0 pins.
    PPRI &= ~PPRI_PRI0P0;
#endif

#if (chip==2541 || chip==2543 || chip==2545)
    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 as GPIO output.
    P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;
#elif (chip==2544)
    // Set pins 1, 2 and 3 as peripheral I/O and pin 0 as GPIO output.
    P0SEL0 = 0x11;        // Map P0_0 and P0_1 as UASRT0. 
    P0SEL1 = 0x11;        // Map P0_3 and P0_2 as UASRT0. 
#endif
     
#if (chip==2541 || chip==2543 || chip==2545)
    // Initialize P0_1 for SRF05EB S1 button.
    P0SEL &= ~BIT1;           // Function as General Purpose I/O.
    P0DIR &= ~BIT1;           // Input.
#elif (chip==2544)
    // Initialize P0_1 for SRF05EB S1 button.
    P0SEL0 &= ~P0SEL0_SELP0_1;// Function as General Purpose I/O.
    PDIR &= ~PDIR_DIRP0_1;    // Input.
#endif
  
    /***************************************************************************
    * Configure UART
    *
    */

    // Initialise bitrate = 57.6 kbps.
    U0BAUD = UART_BAUD_M;
    U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;

    // Initialise UART protocol (start/stop bit, data bits, parity, etc.):
    // USART mode = UART (U0CSR.MODE = 1).
    U0CSR |= U0CSR_MODE;

    // Start bit level = low => Idle level = high  (U0UCR.START = 0).
    U0UCR &= ~U0UCR_START;

    // Stop bit level = high (U0UCR.STOP = 1).
    U0UCR |= U0UCR_STOP;

    // Number of stop bits = 1 (U0UCR.SPB = 0).
    U0UCR &= ~U0UCR_SPB;

    // Parity = disabled (U0UCR.PARITY = 0).
    U0UCR &= ~U0UCR_PARITY;

    // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0).
    U0UCR &= ~U0UCR_BIT9;

    // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1.
    // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1.
    // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1.
    // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1.
    U0UCR &= ~U0UCR_D9;

    // Flow control = disabled (U0UCR.FLOW = 0).
    U0UCR &= ~U0UCR_FLOW;

    // Bit order = LSB first (U0GCR.ORDER = 0).
    U0GCR &= ~U0GCR_ORDER;


    /***************************************************************************
    * Transfer UART data
    */

    
    while(1)
    {
#ifdef UART_TST_MODE_RX
        // Use debugger to check received UART packet.
        if(uartPktReceived)
        {
            uart0StartRxForIsr();
        }
#else 
        // Start UART TX when S1 pressed (S1 is high when pushed).
        if (P0 & BIT1)
        {
            halMcuWaitMs(200);      // Button debouncing delay
            uart0StartTxForIsr();
        }
#endif
    }
}



/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          uart0StartTxForIsr
*
* @brief       Function which starts/initiates the transmission of byte sequence
*              on UART0.
*
* @param       void
*
* @return      void
*/
void uart0StartTxForIsr(void)
{
    // Initialize the UART TX buffer index.
    uartTxIndex = 0;
  
    // Clear any pending UART TX Interrupt Flag (IRCON2.UTXxIF = 0, UxCSR.TX_BYTE = 0).
    UTX0IF = 0;
    U0CSR &= ~U0CSR_TX_BYTE;
  
    // Enable global interrupt (IEN0.EA = 1) and UART TX Interrupt (IEN2.UTXxIE = 1).
    IEN2 |= IEN2_UTX0IE;
    EA = 1;
  
    // Trigger TX interrupt to start transmitting.
    UTX0IF = 1;
}


/***********************************************************************************
* @fn          uart0StartRxForIsr
*
* @brief       Function which starts/initiates the receive of a byte sequence
*              on UART0.
*
* @param       void
*
* @return      void
*/
void uart0StartRxForIsr(void)
{
    // Ready for new packet.
    uartPktReceived = 0;
    
    // Initialize the UART RX buffer index.
    uartRxIndex = 0;

    // Clear any pending UART RX Interrupt Flag (TCON.URXxIF = 0, UxCSR.RX_BYTE = 0).
    URX0IF = 0;

    // Enable UART RX (UxCSR.RE = 1).
    U0CSR |= U0CSR_RE;

    // Enable global Interrupt (IEN0.EA = 1) and UART RX Interrupt (IEN0.URXxIE = 1).
    URX0IE = 1;
    EA = 1; 
}




/***********************************************************************************
* @fn          UART0_TX_ISR
*
* @brief       Function which completes the UART transmit session, that is it
*              sends the rest of the UART0 packet.
*
* @param       void
*
*/
#pragma vector = UTX0_VECTOR
__interrupt void UART0_TX_ISR(void)
{
    // Clear UART0 TX Interrupt Flag (IRCON2.UTX0IF = 0).
    UTX0IF = 0;

    // Send next UART byte.
    U0DBUF = uartTxBuffer[uartTxIndex++];

    // If no UART byte left to transmit, stop this UART TX session
    if (uartTxIndex >= SIZE_OF_UART_TX_BUFFER)
    {
    // Note:
    // In order to start another UART TX session the application just needs
    // to prepare the source buffer, and trigger the TX isr.
    uartTxIndex = 0;
    IEN2 &= ~IEN2_UTX0IE;
    }
}



/***********************************************************************************
* @fn          UART0_RX_ISR
*
* @brief       Function which completes the UART receive session.
*
* @param       void
*
*/
#pragma vector = URX0_VECTOR
__interrupt void UART0_RX_ISR(void)
{
    // Clear UART0 RX Interrupt Flag (TCON.URX0IF = 0).
    URX0IF = 0;

    // Read UART0 RX buffer.
    uartRxBuffer[uartRxIndex++] = U0DBUF;

    // If all UART data received, stop this UART RX session.
    if (uartRxIndex >= SIZE_OF_UART_RX_BUFFER)
    {
        uartRxIndex = 0;
        uartPktReceived = 1;
    }
}

/***********************************************************************************
  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/