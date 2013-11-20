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
* @file     OpenBSL_Main.c
*
* @brief    Main function source file for OpenBSL implementation
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
#include "devtypes.h"
#include "OpenBSL.h"

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************


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
  // initialize OpenBSL
  OpenBSL_Init();

  // check for necessary BSL entry
  if(OpenBSL_EntryCheck() == false)
  {
    // skip BSL, run application
	OpenBSL_RunApp();
  }

  // run BSL, should never return
  OpenBSL_RunBSL();

  while(1);
}

//*****************************************************************************
// Internal functions
//*****************************************************************************

/**************************************************************************//**
*
* _system_pre_init/__low_level_init
*
* @brief      system pre initialization function
*
* @param      -
*
* @return     -
*
******************************************************************************/
#if defined (__IAR_SYSTEMS_ICC__)
int __low_level_init(void)
{
  // stop WDT
  WDTCTL = WDTPW + WDTHOLD;

  return 1;
}
#else
int _system_pre_init(void)
{
  // stop WDT
  WDTCTL = WDTPW + WDTHOLD;

  return 1;
}
#endif

