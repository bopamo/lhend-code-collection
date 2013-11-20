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
* @file     OpenBSL_Config.h
*
* @brief    OpenBSL configuration header file
*
* @version  0.1
*
* @remark
*
******************************************************************************/

#ifndef _OPEN_BSL_CONFIG_H_
#define _OPEN_BSL_CONFIG_H_


//*****************************************************************************
// Include section
//*****************************************************************************


//*****************************************************************************
// Global variable declarations
//*****************************************************************************

/** OPEN_BSL_CONFIG_DEBUG
 *  set to true to activate debugging mode - can result in bigger code size
 */
#define OPEN_BSL_CONFIG_DEBUG                                    (false)

/** OPEN_BSL_NUM_OF_MEM_SECTIONS
 *  total number of memory sections of the device's on-chip non-volatile
 *  memory
 */
#define OPEN_BSL_NUM_OF_MEM_SECTIONS                             (2)

/** OPEN_BSL_CONFIG_PASSWORD_PROTECTED
 *  set to true to activate security feature in which all command -
 *  except SYNC - are password protected, i.e. support for
 *  OPEN_BSL_CMD_PASSWD command
 */
#define OPEN_BSL_CONFIG_PASSWORD_PROTECTED                       (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE
 *  set to true to activate OPEN_BSL_CMD_ERASE_IMAGE command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE                  (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE
 *  set to true to activate OPEN_BSL_CMD_DOWNLOAD_IMAGE command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE               (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE
 *  set to true to activate OPEN_BSL_CMD_UPLOAD_IMAGE command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE                 (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM
 *  set to true to activate OPEN_BSL_CMD_CALCULATE_CHECKSUM command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM           (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT
 *  set to true to activate OPEN_BSL_CMD_ERASE_SEGMENT command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT                (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT
 *  set to true to activate OPEN_BSL_CMD_DOWNLOAD_SEGMENT command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT             (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SEGMENT
 *  set to true to activate OPEN_BSL_CMD_UPLOAD_SEGMENT command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SEGMENT               (true)

/** OPEN_BSL_CONFIG_SUPPORT_CMD_JUMP_TO_ADDR
 *  set to true to activate OPEN_BSL_CMD_JUMP_TO_ADDR command
 */
#define OPEN_BSL_CONFIG_SUPPORT_CMD_JUMP_TO_ADDR                 (true)

/** OPEN_BSL_CONFIG_PASSWORD
 *  8 bytes password which is used to open the commands inside
 *  OPEN_BSL_CMD_PASSWD command - only if OPEN_BSL_CONFIG_PASSWORD_PROTECTED
 *  is set as true
 */
#define OPEN_BSL_CONFIG_PASSWORD      {'M', 'Y', 'P', 'A', 'S', 'S', 'W', 'D'}



//*****************************************************************************
// Macros (defines) and data types
//*****************************************************************************


//*****************************************************************************
// External function declarations
//*****************************************************************************


#endif /* _OPEN_BSL_CONFIG_H_ */
