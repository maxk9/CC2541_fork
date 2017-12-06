/***********************************************************************************
  Filename:     uart_dma.c

  Description:  This example sends/receives data on UART0, using DMA method.

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
#include <dma.h>
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

// Size of allocated UART RX/TX buffer (just an example).
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

// UART test characters.
#define UART_TEST_DATA "Texas Instruments! \r\n"

// Test definitions.
//#define UART_TST_MODE_RX
#define UART_TST_MODE_TX

// Baudrate = 115.200 kbps (U0BAUD.BAUD_M = 216, U0GCR.BAUD_E = 11), given 32 MHz system clock.
#define UART_BAUD_M  216
#define UART_BAUD_E  11



/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer+index for UART RX/TX.
static uint8 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint8 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER] = UART_TEST_DATA;


// Variable for UART packet monitoring.
static uint8 __xdata uartPktReceived = 1;


// DMA descriptor for UART RX/TX.
static DMA_DESC __xdata uartDmaRxTxCh[2];


// Prototype for local functions.
void uart0StartRxDmaChan( DMA_DESC *uartDmaRxDescr,
                          uint8 uartDmaRxChan,
                          uint8 *uartRxBuf,
                          uint16 uartRxBufSize);
void uart0StartTxDmaChan( DMA_DESC *uartDmaTxDescr,
                          uint8 uartDmaTxChan,
                          uint8 *uartTxBuf,
                          uint16 uartTxBufSize);



/***********************************************************************************
* @fn          main
*
* @brief       Send/receive data on UART0, using DMA.
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
    while (CLKCONSTA & CLKCON_OSC);   // Wait until clock source has changed.
  
    // Note the 32 kHz RCOSC starts calibrating, if not disabled.
  
  
    /***************************************************************************
    * Setup I/O ports
    *
    * Port and pins used by USART0 operating in UART-mode, at the Alternative 1
    * location for CC2541EM, CC2543EM and CC2545EM are:
    * RX     : P1_7
    * TX     : P1_6
    * 
    * 
    *
    * These pins must be set to function as peripheral I/O to be used by UART1.
    * The TX pin on the transmitter must be connected to the RX pin on the receiver.
    * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
    * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
    * receiver.
    */

#if (chip==2541 || chip==2543 || chip==2545)
    // Configure USART0 for Alternative 1 => Port P1 (PERCFG.U0CFG = 0).
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;    
#endif
        
#if (chip==2541)
    // Give priority to USART 1 over Timer 1 for port 0 pins.
    P2DIR |= P2DIR_PRIP0_USART1;
#endif

#if (chip==2541 || chip==2543 || chip==2545)
    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 as GPIO output.
    P1SEL |= BIT6 | BIT7;
#endif
     
#if (chip==2541 || chip==2543 || chip==2545)
    // Initialize P0_1 for SRF05EB S1 button
    P1SEL &= ~BIT0;           // Function as General Purpose I/O
    P1DIR &= ~BIT0;           // Input
#endif

    P0INP =0; //pull UP input mode
    P2INP =0;
    /***************************************************************************
    * Configure UART
    *
    */

    // Initialise bitrate = 115.200 kbps.
    U1BAUD = UART_BAUD_M;
    U1GCR = (U1GCR&~U0GCR_BAUD_E) | UART_BAUD_E;

    // Initialise UART protocol (start/stop bit, data bits, parity, etc.):
    // USART mode = UART (U0CSR.MODE = 1).
    U1CSR |= U0CSR_MODE;

    // Start bit level = low => Idle level = high  (U0UCR.START = 0).
    U1UCR &= ~U0UCR_START;

    // Stop bit level = high (U0UCR.STOP = 1).
    U1UCR |= U0UCR_STOP;

    // Number of stop bits = 1 (U0UCR.SPB = 0).
    U1UCR &= ~U0UCR_SPB;

    // Parity = disabled (U0UCR.PARITY = 0).
    U1UCR &= ~U0UCR_PARITY;

    // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0).
    U1UCR &= ~U0UCR_BIT9;

    // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1.
    // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1.
    // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1.
    // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1.
    U1UCR &= ~U0UCR_D9;

    // Flow control = disabled (U0UCR.FLOW = 0).
    U1UCR &= ~U0UCR_FLOW;

    // Bit order = LSB first (U0GCR.ORDER = 0).
    U1GCR &= ~U0GCR_ORDER;


  /***************************************************************************
   * Transfer UART data
   */
    while(1)
    {

#ifdef UART_TST_MODE_RX
        // Use debugger to check received UART packet.
        if(uartPktReceived)
        {
            uartPktReceived = 0;
            uart0StartRxDmaChan(&uartDmaRxTxCh[0], 0, uartRxBuffer, SIZE_OF_UART_RX_BUFFER);
        }
#else
        // Start UART TX when S1 pressed.
        if (!(P1 & BIT0))
        {
            halMcuWaitMs(200);      // Button debouncing delay.
            uart0StartTxDmaChan(&uartDmaRxTxCh[1], 1, uartTxBuffer, SIZE_OF_UART_TX_BUFFER);
        }
#endif
    }
}


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          uart0StartTxDmaChan
*
* @brief       Function which sets up a DMA channel for UART1 TX.
*
* @param       DMA_DESC *uartDmaTxDescr - pointer to DMA descriptor for UART TX
*              uint8 uartDmaTxChan - DMA channel number for UART TX
*              uint8* uartTxBuf - pointer to allocated UART TX buffer
*              uint16 uartTxBufSize - size of allocated UART TX buffer
*
* @return      void
*/
void uart0StartTxDmaChan( DMA_DESC *uartDmaTxDescr,
                          uint8 uartDmaTxChan,
                          uint8 *uartTxBuf,
                          uint16 uartTxBufSize)
{

    // Set source/destination pointer (UART TX buffer address) for UART TX DMA channel,
    // and total number of DMA word transfer (according to UART TX buffer size).
    uartDmaTxDescr->SRCADDRH   = (uint16)(uartTxBuf + 1) >> 8;      // +1 since we're sending the first packet manually later on,
    uartDmaTxDescr->SRCADDRL   = (uint16)(uartTxBuf + 1);           // to start the DMA
    uartDmaTxDescr->DESTADDRH  = ((uint16)(&X_U1DBUF) >> 8) & 0x00FF;
    uartDmaTxDescr->DESTADDRL  = (uint16)(&X_U1DBUF) & 0x00FF;
    uartDmaTxDescr->LENH       = ((uartTxBufSize - 1) >> 8) & 0xFF;
    uartDmaTxDescr->LENL       = (uartTxBufSize - 1) & 0xFF;
    uartDmaTxDescr->VLEN       = DMA_VLEN_FIXED;    // Use fixed length DMA transfer count.

    // Perform 1-byte transfers
    uartDmaTxDescr->WORDSIZE   = DMA_WORDSIZE_BYTE;

    // Transfer a single word after each DMA trigger
    uartDmaTxDescr->TMODE      = DMA_TMODE_SINGLE;

    // DMA word trigger = USARTx TX complete
    uartDmaTxDescr->TRIG       = DMA_TRIG_UTX1;

    uartDmaTxDescr->SRCINC     = DMA_SRCINC_1;          // Increment source pointer by 1 word
                                                        // address after each transfer.
    uartDmaTxDescr->DESTINC    = DMA_DESTINC_0;         // Do not increment destination pointer:
                                                        // points to USART UxDBUF register.
    uartDmaTxDescr->IRQMASK    = DMA_IRQMASK_ENABLE;    // Enable DMA interrupt to the CPU
    uartDmaTxDescr->M8         = DMA_M8_USE_8_BITS;     // Use all 8 bits for transfer count
    uartDmaTxDescr->PRIORITY   = DMA_PRI_LOW;           // DMA memory access has low priority

    // Link DMA descriptor with its corresponding DMA configuration register.
    if (uartDmaTxChan < 1)
    {
        DMA0CFGH = (uint8)((uint16)uartDmaTxDescr >> 8);
        DMA0CFGL = (uint8)((uint16)uartDmaTxDescr & 0x00FF);
    } else {
        DMA1CFGH = (uint8)((uint16)uartDmaTxDescr >> 8);
        DMA1CFGL = (uint8)((uint16)uartDmaTxDescr & 0x00FF);
    }

    // Arm the relevant DMA channel for UART TX, and apply 9 NOPs
    // to allow the DMA configuration to load
    DMAARM = (1 << uartDmaTxChan);
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); // 9 Nops

    // Enable the DMA interrupt (IEN1.DMAIE = IEN0.EA = 1),
    // and clear potential pending DMA interrupt requests (IRCON.DMAIF = 0).
    EA = 1; DMAIE = 1; DMAIF = 0;

    // Send the very first UART byte to trigger a UART TX session:
    U1DBUF = uartTxBuf[0];

    /* 
    *   At this point the UART peripheral generates a DMA trigger each time it has
    *   transmitted a byte, leading to a DMA transfer from the allocated source buffer
    *   to the UxDBUF register. Once the DMA controller has completed the defined
    *   range of transfers, the CPU vectors its execution to the DMA ISR. 
    */
}



/***********************************************************************************
* @fn          uart0StartRxForIsr
*
* @brief       Function which sets up a DMA channel for UART0 RX.
*
* @param       DMA_DESC *uartDmaRxDescr - pointer to DMA descriptor for UART RX
*              uint8 uartDmaRxChan - DMA channel number for UART RX
*              uint8* uartRxBuf - pointer to allocated UART RX buffer
*              uint16 uartRxBufSize - size of allocated UART RX buffer
*
* @return      void
*/
void uart0StartRxDmaChan( DMA_DESC *uartDmaRxDescr,
                          uint8 uartDmaRxChan,
                          uint8 *uartRxBuf,
                          uint16 uartRxBufSize)
{

    // Set source/destination pointer (UART RX buffer address) for UART RX DMA channel,
    // and total number of DMA word transfer (according to UART RX buffer size).
    uartDmaRxDescr->DESTADDRH  = (uint16)uartRxBuf >> 8;
    uartDmaRxDescr->DESTADDRL  = (uint16)uartRxBuf;
    uartDmaRxDescr->SRCADDRH   = ((uint16)(&X_U1DBUF) >> 8) & 0x00FF;
    uartDmaRxDescr->SRCADDRL   = (uint16)(&X_U1DBUF) & 0x00FF;
    uartDmaRxDescr->LENH       = (uartRxBufSize >> 8) & 0xFF;
    uartDmaRxDescr->LENL       = uartRxBufSize & 0xFF;

    uartDmaRxDescr->VLEN       = DMA_VLEN_FIXED;  // Use fixed length DMA transfer count.

    // Perform 1-byte transfers.
    uartDmaRxDescr->WORDSIZE   = DMA_WORDSIZE_BYTE;

    // Transfer a single word after each DMA trigger.
    uartDmaRxDescr->TMODE      = DMA_TMODE_SINGLE;

    // DMA word trigger = USART0 RX complete.
    uartDmaRxDescr->TRIG       = DMA_TRIG_URX1;

    uartDmaRxDescr->SRCINC     = DMA_SRCINC_0;        // Do not increment source pointer;
                                                    // points to USART UxDBUF register.
    uartDmaRxDescr->DESTINC    = DMA_DESTINC_1;       // Increment destination pointer by.
                                                    // 1 word address after each transfer.
    uartDmaRxDescr->IRQMASK    = DMA_IRQMASK_ENABLE;  // Enable DMA interrupt to the CPU.
    uartDmaRxDescr->M8         = DMA_M8_USE_8_BITS;   // Use all 8 bits for transfer count.
    uartDmaRxDescr->PRIORITY   = DMA_PRI_LOW;         // DMA memory access has low priority.

    // Link DMA descriptor with its corresponding DMA configuration register.
    if (uartDmaRxChan < 1)
    {
        DMA0CFGH = (uint8)((uint16)uartDmaRxDescr >> 8);
        DMA0CFGL = (uint8)((uint16)uartDmaRxDescr & 0x00FF);
    } else {
        DMA1CFGH = (uint8)((uint16)uartDmaRxDescr >> 8);
        DMA1CFGL = (uint8)((uint16)uartDmaRxDescr & 0x00FF);
    }
    // Arm the relevant DMA channel for RX, and apply 9 NOPs
    // to allow the DMA configuration to load.
    DMAARM = (1 << uartDmaRxChan);
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();    // 9 NOPs

    // Enable the DMA interrupt (IEN1.DMAIE = IEN0.EA = 1),
    // and clear potential pending DMA interrupt requests (IRCON.DMAIF = 0).
    EA = 1; DMAIE = 1; DMAIF = 0;

    // Enable UARTx RX, clear any pending RX interrupt request and permit data flow:
    // Enable UART1 RX (U0CSR.RE = 1)
    U1CSR |= U0CSR_RE;

    /* 
    *   At this point the UART peripheral generates a DMA trigger each time it
    *   has received a byte, leading to a DMA transfer from the UxDBUF register
    *   to the allocated target buffer. Once the DMA controller has completed the
    *   defined range of transfers, the CPU vectors its execution to the DMA ISR. 
    */
}



/***********************************************************************************
* @fn          DMA_ISR
*
* @brief       DMA Interrupt Service Routine, called when DMA has finished
*              transferring one packet/buffer between memory and UxDBUF.
*
* @param       none
*
*/
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{

    // Clear the main DMA interrupt Request Flag (IRCON.DMAIF = 0)
    DMAIF = 0;

    // Start a new UART RX session on DMA channel 1:
    if (DMAIRQ & DMAIRQ_DMAIF0)
    {
        // Indicate UART packet received (monitored by main-loop for data integrity/error check.
        uartPktReceived = 1;

        // Clear DMA Channel 0 Interrupt Request Flag (DMAIRQ.DMAIF0 = 0).
        DMAIRQ = ~DMAIRQ_DMAIF0;

        // Re-arm DMA Channel 0 (DMAARM.DMAARM0 = 1).
        DMAARM |= DMAARM_DMAARM0;
    }

    // Start a new UART TX session on DMA channel 1:
    if (DMAIRQ & DMAIRQ_DMAIF1)
    {
        // Clear DMA Channel 1 Interrupt Request Flag (DMAIRQ.DMAIF1 = 0)
        DMAIRQ = ~DMAIRQ_DMAIF1;

        /* 
        *   In this particular software example a new UART/DMA TX session must
        *   be initialized outside this DMA ISR. However, it would also be possible
        *   to use this DMA ISR to start a new UART/DMA TX session by simply
        *   re-arming the DMA channel and sending the first UART TX byte.
        */
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