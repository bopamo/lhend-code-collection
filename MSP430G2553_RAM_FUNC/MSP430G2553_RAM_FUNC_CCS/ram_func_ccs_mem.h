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
* @file     ram_func_ccs_mem.h
*
* @brief    header file containing the memory layout for example code
*           running function in RAM with CCS IDE
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

#ifndef _RAM_FUNC_CCS_MEM_H_
#define _RAM_FUNC_CCS_MEM_H_


//*****************************************************************************
// Include section
//*****************************************************************************


//*****************************************************************************
// Global variable declarations
//*****************************************************************************


//*****************************************************************************
// Macros (defines) and data types
//*****************************************************************************

// RAM memory (0x0200 - 0x03FF)
#define RAM_CODE_START_ADDR             (0x0200) // 128 bytes for code
#define RAM_CODE_END_ADDR               (0x027F)
#define RAM_CODE_LEN                    (RAM_CODE_END_ADDR - RAM_CODE_START_ADDR + 1)

#define RAM_DATA_START_ADDR             (RAM_CODE_END_ADDR + 1)
#define RAM_DATA_END_ADDR               (0x03FF)
#define RAM_DATA_LEN                    (RAM_DATA_END_ADDR - RAM_DATA_START_ADDR + 1)


// Flash memory (0xC000-0xFFBF), Interrupt Vector (0xFFC0-0xFFFF)
#define FLASH_RAM_CODE_START_ADDR       (0xC000)  // shall match the length of RAM for code
#define FLASH_RAM_CODE_END_ADDR         (FLASH_RAM_CODE_START_ADDR + RAM_CODE_LEN - 1)
#define FLASH_RAM_CODE_LEN              (RAM_CODE_LEN)

#define FLASH_START_ADDR                (FLASH_RAM_CODE_END_ADDR + 1)
#define FLASH_END_ADDR                  (0xFFBF)
#define FLASH_LEN                       (FLASH_END_ADDR - FLASH_START_ADDR + 1)



//*****************************************************************************
// External function declarations
//*****************************************************************************

#endif /* _RAM_FUNC_CCS_MEM_H_ */
