/******************************************************************************
* Copyright (c) 2014, Leo Hendrawan
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the MSS PROJECT nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE MSS PROJECT OR ITS
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**************************************************************************//**
*
* @file     msp430g2553_uart_echo.c
*
* @brief    example code to use C preprocessor smartly for calculating
*           MSP430G2553 baud rate register setting
*
* @version  0.1
*
* @author   Leo Hendrawan (lhend.mss (at) gmail.com)
*
* @remark
*  - target device: MSP430G2553
*  - IDE: CCS IDE v5.5 - MSP430 CGT v4.2.1
*
******************************************************************************/

//*****************************************************************************
// Include section
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>
#include "usci_settings.h"

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************

#if (USCI_INPUT_CLK == 1000000UL)
  #define BCSCTL1_VAL    (CALBC1_1MHZ)
  #define DCOCTL_VAL     (CALDCO_1MHZ)
#elif (USCI_INPUT_CLK == 8000000UL)
  #define BCSCTL1_VAL    (CALBC1_8MHZ)
  #define DCOCTL_VAL     (CALDCO_8MHZ)
#elif (USCI_INPUT_CLK == 12000000UL)
  #define BCSCTL1_VAL    (CALBC1_12MHZ)
  #define DCOCTL_VAL     (CALDCO_12MHZ)
#elif (USCI_INPUT_CLK == 16000000UL)
  #define BCSCTL1_VAL    (CALBC1_16MHZ)
  #define DCOCTL_VAL     (CALDCO_16MHZ)
#else
  #error Invalid input clock setting! Choose between 1, 8, 12, or 16 MHz!
#endif


//*****************************************************************************
// Internal function declarations
//*****************************************************************************

static void splash(void);
static void sendByte(uint8_t byte);
static bool rcvByte(uint8_t *byte);

//*****************************************************************************
// External functions
//*****************************************************************************

/**************************************************************************//**
*
* main
*
* @brief      main function
*
* @param      -
*
* @return     -
*
******************************************************************************/
void main(void)
{
  uint8_t data;

  // source ACLK with internal VLO clock
  BCSCTL3 |= LFXT1S_2;

  // Set DCO register value based on the selected input clock frequency
  BCSCTL1 = BCSCTL1_VAL;
  DCOCTL = DCOCTL_VAL;

  // set GPIO as UART pins - P1.1=UCA0RXD, P1.2=UCA0TXD
  P1SEL = BIT1 + BIT2 ;
  P1SEL2 = BIT1 + BIT2;

  // setup USCI UART registers
  UCA0CTL1 |= UCSSEL_2 + UCSWRST;
  UCA0BR0 = USCI_BR0_VAL;
  UCA0BR1 = USCI_BR1_VAL;
  UCA0MCTL = USCI_BRS_VAL;
  UCA0CTL1 &= ~UCSWRST;

  // do somekind of splash screen
  splash();

  while(1)
  {
    if(rcvByte(&data) == true)
    {
      // echo back the received data
      sendByte(data);
    }
  }
}

//*****************************************************************************
// Internal functions
//*****************************************************************************

/**************************************************************************//**
*
* sendByte
*
* @brief      do somekind of splash screen to check UART during init
*
* @param      -
*
* @return     -
*
******************************************************************************/
static void splash(void)
{
  const uint8_t splashText[] = {"Launchpad UART Echo Test\r\nType to generate echo\r\n"};
  uint8_t i;

  for(i=0 ; i<sizeof(splashText) ; i++)
  {
	sendByte(splashText[i]);
  }
}


/**************************************************************************//**
*
* sendByte
*
* @brief      send a byte via USCI UART channel
*
* @param      byte   byte to be sent out via UART channel
*
* @return     -
*
******************************************************************************/
static void sendByte(uint8_t byte)
{
  // wait until last transmit has finished
  while(!(IFG2 & UCA0TXIFG));

  // copy byte to transmit buffer
  UCA0TXBUF = byte;
}

/**************************************************************************//**
*
* rcvByte
*
* @brief      try to receive byte from USCI UART channel by polling mechanism
*
* @param      byte    pointer to received byte if any available
*
* @return     true if there is a byte received
*
******************************************************************************/
static bool rcvByte(uint8_t *byte)
{
  bool rcv = false;

  if(IFG2 & UCA0RXIFG)
  {
	// copy byte from hardware receive buffer to software buffer
	*byte = UCA0RXBUF;

	// set return value on successful receive
	rcv = true;
  }

  return rcv;
}

/**************************************************************************//**
*
* _system_pre_init
*
* @brief      C pre-init function
*
* @param      -
*
* @return     1 to enable C data initialization
*
******************************************************************************/
int _system_pre_init(void)
{
  // stop WDT
  WDTCTL = WDTPW + WDTHOLD;

  // Perform C/C++ global data initialization
  return 1;
}
