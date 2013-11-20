/******************************************************************************
* Copyright (c) 2012-2013, Leo Hendrawan
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
* @file     ram_func_ccs.c
*
* @brief    example code to run function on RAM with CCSTUDIO compiler
*
* @version  0.1
*
* @author   Leo Hendrawan
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
#include <string.h>
#include <msp430.h>
#include "ram_func_ccs_mem.h"

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************

#define DELAY_CYCLES   (60000)

//*****************************************************************************
// Internal function declarations
//*****************************************************************************

void main_loop(void);
void blink_led1(void);
void blink_led2(void);

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
  // source ACLK with internal VLO clock
  BCSCTL3 |= LFXT1S_2;

  // set all pin as output low
  P1DIR = 0xFF;
  P1OUT = 0x00;
  P2DIR = 0xFF;
  P2OUT = 0x00;

  // setup Timer A0 CCR0 interrupt
  TACCTL0 = CCIE;
  TACCR0 = 10000;
  TACTL = TASSEL_1 + MC_1 + TACLR;

  // goto main loop - shall never return
  main_loop();

  while(1);
}

//*****************************************************************************
// Internal functions
//*****************************************************************************

/**************************************************************************//**
*
* main_loop
*
* @brief      main loop function
*
* @param      -
*
* @return     -
*
******************************************************************************/
#pragma CODE_SECTION(main_loop, ".ram_code")
void main_loop(void)
{
  while(1)
  {
    // enable interrupt and sleep
    __bis_SR_register(LPM3_bits + GIE);

    // blink LED2
    blink_led2();
  }
}

/**************************************************************************//**
*
* TIMER_ISR
*
* @brief      Timer A0 CCR0 ISR
*
* @param      -
*
* @return     -
*
******************************************************************************/
#pragma CODE_SECTION(TIMER_ISR, ".ram_code")
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER_ISR(void)
{
  // blink LED1
  blink_led1();

  // wake up CPU
  __bic_SR_register_on_exit(LPM3_bits);
}

/**************************************************************************//**
*
* blink_led1
*
* @brief      blink LED 1
*
* @param      -
*
* @return     -
*
******************************************************************************/
#pragma CODE_SECTION(blink_led1, ".ram_code")
void blink_led1(void)
{
  // blink P1.0 bit
  P1OUT |= BIT0;
  __delay_cycles(DELAY_CYCLES);
  P1OUT &= ~BIT0;
}

/**************************************************************************//**
*
* blink_led2
*
* @brief      blink LED 2
*
* @param      -
*
* @return     -
*
******************************************************************************/
#pragma CODE_SECTION(blink_led2, ".ram_code")
void blink_led2(void)
{
  // blink P1.6 bit
  P1OUT |= BIT6;
  __delay_cycles(DELAY_CYCLES);
  P1OUT &= ~BIT6;
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

  // copy ram code from flash to ram
  memcpy(((void*)RAM_CODE_START_ADDR),
		 ((void*)FLASH_RAM_CODE_START_ADDR),
		 RAM_CODE_LEN);

  // Perform C/C++ global data initialization
  return 1;
}
