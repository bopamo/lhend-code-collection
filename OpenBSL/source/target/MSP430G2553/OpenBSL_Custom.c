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
* @file     OpenBSL_Custom.c
*
* @brief    Custom/device, application specific source file of OpenBSL
*           implementation
*
* @version  0.1
*
* @remark   target device: MSP430G2553 on MSP-EXP430G2 Launchpad
*           compiler: CCSTUIO MSP430 CGT v4.2.1
*
******************************************************************************/

//*****************************************************************************
// Include section
//*****************************************************************************
#include <msp430.h>
#include "devtypes.h"
#include "OpenBSL.h"
#include "OpenBSL_Int.h"

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************


/**************************************************************************//**
* @page MSP430_INT_VECT_REDIRECT     MSP430 Interrupt Vector Redirection
*
* @section DESCRIPTION   Description
* The OpenBSL is implemented in the main code memory flash of MSP430 devices
* specifically in the last flash sections which is adjacent to the interrupt
* vector table. Therefore in order to enable the application to use interrupt,
* the interrupt vector table shall contain the addresses of small functions
* (at the moment it is implemented as assembly file) which jumps to the real
* ISR function.
*
* The real/hardware interrupt vector is implemented as VectorTbl[] constant
* array pointing to certain address in main code memory just before the
* memory section containing the OpenBSL.
*
* The secondary memory addresses is calculated based on the content of
* OpenBSL_Device.h header file. This header file shall be also used
* by the application file to implement the secondary interrupt vector table.
*
******************************************************************************/

#define EMPTY_VECTOR                (0xFFFF)
#define APP_VECTOR(x)               (APP_MEM_INT_VECT_START_ADDR + (DEV_JUMP_INST_LEN*x))

/** VectorTbl
 *  interrupt vector table implemented as constant array pointing to the
 *  secondary interrupt vector table
 */
#pragma DATA_SECTION(VectorTbl, ".vector_tbl")
#pragma RETAIN(VectorTbl)
const uint16_t VectorTbl[(OPEN_BSL_INT_VECT_LEN/sizeof(uint16_t))] =
{
	EMPTY_VECTOR,                                // 0xFFE0 = unused
	EMPTY_VECTOR,                                // 0xFFE2 = unused
    APP_VECTOR(DEV_VECTOR_INT_IDX_P1),           // 0xFFE4 = P1
    APP_VECTOR(DEV_VECTOR_INT_IDX_P2),           // 0xFFE6 = P2
    EMPTY_VECTOR,                                // 0xFFE8 = unused
    APP_VECTOR(DEV_VECTOR_INT_IDX_ADC10),        // 0xFFEA = ADC10
    APP_VECTOR(DEV_VECTOR_INT_IDX_USCI_RX_TX),   // 0xFFEC = USCI I2C TX/RX
    APP_VECTOR(DEV_VECTOR_INT_IDX_USCI_STAT),    // 0xFFEE = USCI I2C STAT
    APP_VECTOR(DEV_VECTOR_INT_IDX_TA0_1),        // 0xFFF0 = TA0_1
    APP_VECTOR(DEV_VECTOR_INT_IDX_TA0_0),        // 0xFFF2 = TA0_0
    APP_VECTOR(DEV_VECTOR_INT_IDX_WDT),          // 0xFFF4 = WDT
    APP_VECTOR(DEV_VECTOR_INT_IDX_COMP_A),       // 0xFFF6 = COMP_A
    APP_VECTOR(DEV_VECTOR_INT_IDX_TA1_1),        // 0xFFF8 = TA1_1
    APP_VECTOR(DEV_VECTOR_INT_IDX_TA1_0),        // 0xFFFA = TA1_0
    APP_VECTOR(DEV_VECTOR_INT_IDX_NMI),          // 0xFFFC = NMI
};

//*****************************************************************************
// Internal function declarations
//*****************************************************************************


//*****************************************************************************
// External functions
//*****************************************************************************

/**************************************************************************//**
*
* OpenBSL_DevInit
*
* @brief     device/HW initialization function for OpenBSL
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_DevInit(void)
{
  // initialize communication module
  OpenBSL_CommInit();

  // initialize memory module
  OpenBSL_MemInit();
}

/**************************************************************************//**
*
* OpenBSL_EntryCheck
*
* @brief     check whether it is necessary to run BSL or application
*
* @param     -
*
* @return    true - run BSL, false - run application
*
******************************************************************************/
bool OpenBSL_EntryCheck(void)
{
  // at the moment, always run BSL first
  // TODO: change if necessary
  return true;
}

/**************************************************************************//**
*
* OpenBSL_RunApp
*
* @brief     run application, leave BSL mode
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_RunApp(void)
{
  memAddr_t appBootAddr;

  // get app boot address
  appBootAddr = *((memAddr_t*)APP_MEM_RESET_VECT_ADDR);

  // jump to application entry point pointed by the secondary reset vector
  OpenBSL_Jump(appBootAddr);
}

/**************************************************************************//**
*
* OpenBSL_Jump
*
* @brief     run application, leave BSL mode
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_Jump(memAddr_t addr)
{
  // make sure we have finished sending a byte out
  while(UCA0STAT & UCBUSY);

  // jump to given address
  ((void(*)(void))addr)();
}

#if (OPEN_BSL_CONFIG_DEBUG == true)
/**************************************************************************//**
*
* OpenBSL_AssertHdl
*
* @brief     assert handle function
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_AssertHdl(void)
{
  // disable interrupt to avoid wake-up
  __disable_interrupt();

  while(1)
  {
    // sleep forever
    __bic_SR_register(LPM4_bits);
  }
}
#endif // (OPEN_BSL_CONFIG_DEBUG == true)


//*****************************************************************************
// Internal functions
//*****************************************************************************
