/******************************************************************************
* Copyright (c) 2013, Leo Hendrawan
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the copyright holder(s) nor the names of its
*      contributor(s) may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTOR(S) BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**************************************************************************//**
*
* @file     open_bsl_app_ex1.c
*
* @brief    example application code which resides together with
*           OpenBSL bootloader
*
* @version  0.1
*
* @remark   target device: MSP430G2553 on MSP-EXP430G2 Launchpad
*
******************************************************************************/

//*****************************************************************************
// Include section
//*****************************************************************************
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "OpenBSL_Device.h"

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************

#ifdef USE_TIMER_A0
  // use Timer_A0
  #define CCTL_REG             TA0CCTL0
  #define CCR_REG              TA0CCR0
  #define CTL_REG              TA0CTL
  #define LED_BIT              BIT0
#else
  // use Timer_A1
  #define CCTL_REG             TA1CCTL0
  #define CCR_REG              TA1CCR0
  #define CTL_REG              TA1CTL
  #define LED_BIT              BIT6
#endif

//*****************************************************************************
// Internal function declarations
//*****************************************************************************

//*****************************************************************************
// External functions
//*****************************************************************************

/**************************************************************************//**
*
* main
*
* @brief     main function
*
* @param
*
* @return
*
******************************************************************************/
void main(void)
{
  // stop WDT
  WDTCTL = WDTPW + WDTHOLD;

  // select VLO as ACLK source
  BCSCTL3 |= LFXT1S_2;

  // set P1.x as output
  P1DIR |= LED_BIT;

  // setup Timer module
  CCTL_REG = CCIE;
  CCR_REG = 12000; // VLOS is defined aroud 12kHz
  CTL_REG = TASSEL_1 + MC_1 + TACLR;

  // go sleep and never wake up
  __bis_SR_register(LPM3_bits + GIE);

  while(1);
}

//*****************************************************************************
// Internal functions
//*****************************************************************************

/**************************************************************************//**
*
* TimerA_ISR
*
* @brief      TIMERA_ISR
*
* @param      -
*
* @return     -
*
* @remark     don't use pragma vector!
*
******************************************************************************/
#pragma RETAIN(TimerA_ISR)
__interrupt void TimerA_ISR(void)
{
  // toggle P1.x
  P1OUT ^= LED_BIT;
}

