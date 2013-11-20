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
* @file     OpenBSL_Device.h
*
* @brief    OpenBSL device configuration header file
*
* @version  0.1
*
* @remark  - Target: MSP430G2553
*          - Application code memory address (including secondary interrupt
*            vector table) : 0xC000 - 0xF7FF
*          - OpenBSL code memory address (including hardware interrupt
*            vector table) : 0xF800 - 0xFFFF
*
******************************************************************************/

#ifndef _OPEN_BSL_DEVICE_H_
#define _OPEN_BSL_DEVICE_H_


//*****************************************************************************
// Include section
//*****************************************************************************


//*****************************************************************************
// Global variable declarations
//*****************************************************************************

///////////////////////////////////////////////////////////////////////////////
// device interrupt configuration
///////////////////////////////////////////////////////////////////////////////
#define DEV_NUM_OF_INT                    (12)
#define DEV_JUMP_INST_LEN                 (4)

#define DEV_VECTOR_INT_IDX_P1             (0)
#define DEV_VECTOR_INT_IDX_P2             (1)
#define DEV_VECTOR_INT_IDX_ADC10          (2)
#define DEV_VECTOR_INT_IDX_USCI_RX_TX     (3)
#define DEV_VECTOR_INT_IDX_USCI_STAT      (4)
#define DEV_VECTOR_INT_IDX_TA0_1          (5)
#define DEV_VECTOR_INT_IDX_TA0_0          (6)
#define DEV_VECTOR_INT_IDX_WDT            (7)
#define DEV_VECTOR_INT_IDX_COMP_A         (8)
#define DEV_VECTOR_INT_IDX_TA1_1          (9)
#define DEV_VECTOR_INT_IDX_TA1_0          (10)
#define DEV_VECTOR_INT_IDX_NMI            (11)

///////////////////////////////////////////////////////////////////////////////
// device memory configuration
///////////////////////////////////////////////////////////////////////////////

/**************************************************************************//**
*  INFO memory
******************************************************************************/
#define INFO_MEM_START_ADDR               (0x1000)
#define INFO_MEM_END_ADDR                 (0x10BF)
#define INFO_MEM_LEN                      (INFO_MEM_END_ADDR-INFO_MEM_START_ADDR+1)

/**************************************************************************//**
*  MAIN code memory for application
******************************************************************************/
#define APP_MEM_START_ADDR                (0xC000)
#define APP_MEM_END_ADDR                  (0xF7FF)
#define APP_MEM_LEN                       (APP_MEM_END_ADDR-APP_MEM_START_ADDR+1)

#define APP_MEM_RESET_VECT_ADDR           (0xF7FE)
#define APP_MEM_RESET_VECT_LEN            (2)

#define APP_MEM_INT_VECT_START_ADDR       (APP_MEM_RESET_VECT_ADDR-(DEV_JUMP_INST_LEN*DEV_NUM_OF_INT))
#define APP_MEM_INT_VECT_END_ADDR         (APP_MEM_RESET_VECT_ADDR-1)
#define APP_MEM_INT_VECT_LEN              (APP_MEM_INT_VECT_END_ADDR-APP_MEM_INT_VECT_START_ADDR+1)

#define APP_MEM_CODE_START_ADDR           (APP_MEM_START_ADDR)
#define APP_MEM_CODE_END_ADDR             (APP_MEM_INT_VECT_START_ADDR-1)
#define APP_MEM_CODE_LEN                  (APP_MEM_CODE_END_ADDR-APP_MEM_CODE_START_ADDR+1)


/**************************************************************************//**
* MAIN code memory for OpenBSL bootloader
******************************************************************************/
#define OPEN_BSL_CODE_START_ADDR          (0xF800)  // ~2KB
#define OPEN_BSL_CODE_END_ADDR            (0xFFDD)
#define OPEN_BSL_CODE_LEN                 (OPEN_BSL_CODE_END_ADDR-OPEN_BSL_CODE_START_ADDR+1)
#define OPEN_BSL_INT_VECT_START_ADDR      (0xFFE0)
#define OPEN_BSL_INT_VECT_END_ADDR        (0xFFFD)
#define OPEN_BSL_INT_VECT_LEN             (OPEN_BSL_INT_VECT_END_ADDR-OPEN_BSL_INT_VECT_START_ADDR+1)

//*****************************************************************************
// Macros (defines) and data types
//*****************************************************************************


//*****************************************************************************
// External function declarations
//*****************************************************************************


#endif /* _OPEN_BSL_DEVICE_H_ */
