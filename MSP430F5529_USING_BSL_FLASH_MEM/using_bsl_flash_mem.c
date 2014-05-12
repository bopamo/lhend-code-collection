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
* @file     using_bsl_flash_mem.c
*
* @brief    example code to use BSL Flash memory for both code and data on
*           MSP430F5xx/6xx devices
*
* @version  0.1
*
* @author   Leo Hendrawan (lhend.mss (at) gmail.com)
*
* @remark
*  - target device: MSP430F5529 on MSP-TS430PN80USB
*  - IDE: CCS IDE v5.5 - MSP430 CGT v4.2.4
*
******************************************************************************/

//*****************************************************************************
// Include section
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************

// constants to be placed in BSL Flash memory
#pragma SET_DATA_SECTION (".bsl0")
const uint8_t const1 = 123;
const uint16_t const2 = 2500;
const uint32_t const3 = 1323;
#pragma SET_DATA_SECTION ()

// reserved memory location - do not touch
#pragma RETAIN(bsl_reserved)
#pragma DATA_SECTION(bsl_reserved, ".bsl3_res")
const uint8_t bsl_reserved[0x0E] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
									0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//*****************************************************************************
// Internal function declarations
//*****************************************************************************

static uint16_t addition(uint16_t a, uint16_t b);
static uint16_t substraction(uint16_t a, uint16_t b);

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
  volatile uint16_t param1, param2, result1, result2;

  // copy constant data
  param1 = (uint16_t) const1;
  param2 = (uint16_t) const2;

  // call addition function
  result1 = addition(param1, param2);

  // copy constant data
  param1 = (uint16_t) const3;
  param2 = (uint16_t) const1;

  // call substraction function
  result2 = substraction(param1, param2);

  while(1);
}

//*****************************************************************************
// Internal functions
//*****************************************************************************

/**************************************************************************//**
*
* addition
*
* @brief      do addition from both parameters
*
* @param      a  -  first input
* @param      b  -  second input
*
* @return     addition result of both parameters
*
******************************************************************************/
#pragma CODE_SECTION (addition, ".bsl1")
static uint16_t addition(uint16_t a, uint16_t b)
{
  return (a+b);
}


/**************************************************************************//**
*
* substraction
*
* @brief      do substraction from both parameters
*
* @param      a  -  first input
* @param      b  -  second input
*
* @return     substraction result of input a to b
*
******************************************************************************/
#pragma CODE_SECTION (substraction, ".bsl2")
static uint16_t substraction(uint16_t a, uint16_t b)
{
  return (a-b);
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

  // make sure to release protection for the used BSL flash memory
  // and keep the protection for BSL3 containing reserved area used
  // for JTAG lock, etc.
  SYSBSLC = SYSBSLPE | SYSBSLOFF;

  // Perform C/C++ global data initialization
  return 1;
}
