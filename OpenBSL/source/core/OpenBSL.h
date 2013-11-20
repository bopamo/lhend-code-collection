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
* @file     OpenBSL.h
*
* @brief    OpenBSL main (external) header file
*
* @version  0.1
*
* @remark
*   v0.1: Hello World! Support for MSP430G2553 and TI CCS compiler
*
******************************************************************************/

#ifndef _OPEN_BSL_H_
#define _OPEN_BSL_H_


/**************************************************************************//**
* @mainpage
*
* @author    Leo Hendrawan
*
* OpenBSL is a simple, light-weight, open source Boot Strap Loader (BSL) for
* small 8/16/32 bits microcontroller device family which is designed and
* implemented as part in the main code memory.
*
* The implementation of OpenBSL is inspired by various application notes from
* Texas Instruments for MSP430 BSL (Boot Strap Loader).
*
* At the moment, OpenBSL supports the following compiler/target device:
*   - MSP430G2553 on MSP-EXP430G2 Launchpad (IDE: Code Composer Studio)
*
******************************************************************************/

/**************************************************************************//**
* @page OPEN_BSL_CMD  OpenBSL Commands
*
* @subpage OPEN_BSL_CMD_GET_MEM_INFO
* @subpage OPEN_BSL_CMD_ERASE_IMAGE
* @subpage OPEN_BSL_CMD_DOWNLOAD_IMAGE
* @subpage OPEN_BSL_CMD_UPLOAD_IMAGE
* @subpage OPEN_BSL_CMD_RUN_APP
* @subpage OPEN_BSL_CMD_CALCULATE_CHECKSUM
* @subpage OPEN_BSL_CMD_ERASE_SEGMENT
* @subpage OPEN_BSL_CMD_DOWNLOAD_SEGMENT
* @subpage OPEN_BSL_CMD_UPLOAD_SEGMENT
* @subpage OPEN_BSL_CMD_PASSWD
* @subpage OPEN_BSL_CMD_JUMP_TO_ADDR
* @subpage OPEN_BSL_CMD_SYNC
*
******************************************************************************/

//*****************************************************************************
// Include section
//*****************************************************************************
#include "OpenBSL_Config.h"

//*****************************************************************************
// Global variable declarations
//*****************************************************************************


//*****************************************************************************
// Macros (defines) and data types
//*****************************************************************************


//*****************************************************************************
// External function declarations
//*****************************************************************************

extern void OpenBSL_Init(void);
extern bool OpenBSL_EntryCheck(void);
extern void OpenBSL_RunApp(void);
extern void OpenBSL_RunBSL(void);


#endif /* _OPEN_BSL_H_ */
