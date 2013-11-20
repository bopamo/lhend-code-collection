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
* @file     OpenBSL_Int.h
*
* @brief    OpenBSL internal header file
*
* @version  0.1
*
* @remark
*
******************************************************************************/

#ifndef _OPEN_BSL_INT_H_
#define _OPEN_BSL_INT_H_


//*****************************************************************************
// Include section
//*****************************************************************************

#include "OpenBSL_Device.h"

//*****************************************************************************
// Global variable declarations
//*****************************************************************************

// checksums
extern uint16_t OpenBSL_InChecksum, OpenBSL_OutChecksum;


//*****************************************************************************
// Macros (defines) and data types
//*****************************************************************************

// function pointer lists based on the configuration header file
#if (OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE == true)
  #define OPEN_BSL_FUNC_PROC_ERASE_IMAGE  OpenBSL_ProcEraseImg
#else
  #define OPEN_BSL_FUNC_PROC_ERASE_IMAGE  NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE == true)
  #define OPEN_BSL_FUNC_PROC_DOWNLOAD_IMAGE   OpenBSL_ProcDnldImg
#else
  #define OPEN_BSL_FUNC_PROC_DOWNLOAD_IMAGE   NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE == true)
  #define OPEN_BSL_FUNC_PROC_UPLOAD_IMAGE   OpenBSL_ProcUpldImg
#else
  #define OPEN_BSL_FUNC_PROC_UPLOAD_IMAGE   NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM == true)
  #define OPEN_BSL_FUNC_PROC_CALCULATE_CHECKSUM   OpenBSL_ProcCalcChksm
#else
  #define OPEN_BSL_FUNC_PROC_CALCULATE_CHECKSUM   NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT == true)
  #define OPEN_BSL_FUNC_PROC_ERASE_SEGMENT   OpenBSL_ProcEraseSegment
#else
  #define OPEN_BSL_FUNC_PROC_ERASE_SEGMENT   NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT == true)
  #define OPEN_BSL_FUNC_PROC_CMD_DOWNLOAD_SEGMENT   OpenBSL_ProcDnldSegment
#else
  #define OPEN_BSL_FUNC_PROC_DOWNLOAD_SEGMENT       NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SEGMENT == true)
  #define OPEN_BSL_FUNC_PROC_UPLOAD_SEGMENT   OpenBSL_ProcUpldSegment
#else
  #define OPEN_BSL_FUNC_PROC_UPLOAD_SEGMENT   NULL
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SECTION == true)

#if (OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)
  #define OPEN_BSL_FUNC_PROC_PASSWORD   OpenBSL_ProcPwdCmd
#else
  #define OPEN_BSL_FUNC_PROC_PASSWORD   NULL
#endif //(OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_JUMP_TO_ADDR == true)
  #define OPEN_BSL_FUNC_PROC_JUMP_TO_ADDR   OpenBSL_ProcJumpToAddr
#else
  #define OPEN_BSL_FUNC_PROC_JUMP_TO_ADDR   NULL
#endif //(OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)

/** OPEN_BSL_CMD_FUNC_TABLE
 *  X Macro for OpenBSL commands - processing functions table
 */
#define OPEN_BSL_CMD_FUNC_TABLE    \
          ENTRY(OPEN_BSL_CMD_GET_MEM_INFO = 0x00, OpenBSL_ProcGetMemInfo) \
          ENTRY(OPEN_BSL_CMD_ERASE_IMAGE = 0x01, OPEN_BSL_FUNC_PROC_ERASE_IMAGE) \
          ENTRY(OPEN_BSL_CMD_DOWNLOAD_IMAGE = 0x02, OPEN_BSL_FUNC_PROC_DOWNLOAD_IMAGE) \
          ENTRY(OPEN_BSL_CMD_UPLOAD_IMAGE = 0x03, OPEN_BSL_FUNC_PROC_UPLOAD_IMAGE) \
          ENTRY(OPEN_BSL_CMD_RUN_APP = 0x04, OpenBSL_ProcRunApp) \
          ENTRY(OPEN_BSL_CMD_CALCULATE_CHECKSUM = 0x05, OPEN_BSL_FUNC_PROC_CALCULATE_CHECKSUM) \
          ENTRY(OPEN_BSL_CMD_ERASE_SEGMENT = 0x06, OPEN_BSL_FUNC_PROC_ERASE_SEGMENT) \
          ENTRY(OPEN_BSL_CMD_DOWNLOAD_SEGMENT = 0x07, OPEN_BSL_FUNC_PROC_CMD_DOWNLOAD_SEGMENT) \
          ENTRY(OPEN_BSL_CMD_UPLOAD_SEGMENT = 0x08, OPEN_BSL_FUNC_PROC_UPLOAD_SEGMENT) \
          ENTRY(OPEN_BSL_CMD_PASSWD = 0x09, OPEN_BSL_FUNC_PROC_PASSWORD) \
          ENTRY(OPEN_BSL_CMD_JUMP_TO_ADDR = 0x0A, OPEN_BSL_FUNC_PROC_JUMP_TO_ADDR)


/** OpenBSL_Cmd_t
 *  OpenBSL supported list of commands
 */
typedef enum {
#define ENTRY(cmd, func)    cmd,
	OPEN_BSL_CMD_FUNC_TABLE
#undef ENTRY

  /* sync command */
  OPEN_BSL_CMD_SYNC = 0x90
} OpenBSL_Cmd_t;



/** OpenBSL_Resp_t
 *  OpenBSL supported list of response
 */
typedef enum {
  OPEN_BSL_RESP_OK = 0x00,
  OPEN_BSL_RESP_ERR_UNKNOWN_CMD = 0xE1,
  OPEN_BSL_RESP_ERR_UNSUPPORTED_CMD = 0xE2,
  OPEN_BSL_RESP_ERR_PASSWORD_PROTECTED = 0xE3,
  OPEN_BSL_RESP_ERR_INVALID_PARAM = 0xE4,
  OPEN_BSL_RESP_ERR_INVALID_FORMAT = 0xE5,
  OPEN_BSL_RESP_ERR_INVALID_CHECKSUM = 0xE6
} OpenBSL_Resp_t;

/** OPEN_BSL_RESP_BIT_MASK
 *  response bit mask for positive response of incoming command
 */
#define OPEN_BSL_RESP_BIT_MASK         (0x80)


/* OPEN_BSL_ASSERT
 * assert function
 */
#if (OPEN_BSL_CONFIG_DEBUG == true)
#define OPEN_BSL_ASSERT(x)   if(!(x)) OpenBSL_AssertHdl();
#else
#define OPEN_BSL_ASSERT(x)
#endif // (OPEN_BSL_CONFIG_DEBUG == true)

/* BV
 * bit value macro function
 */
#ifndef BV
#define BV(x)     (1 << x)
#endif

/** OpenBSL_MemSect_t
 *  OpenBSL memory section data type
 */
typedef struct {
  memAddr_t start_addr;
  memAddr_t end_addr;
} OpenBSL_MemSect_t;


// address data size
#define OPEN_BSL_ADDR_DATA_SIZE          (sizeof(uint32_t))

// memory segment transfer maximum size in bytes
#define OPEN_BSL_MAX_SEGMENT_READ_WRITE_DATA_SIZE   (32)

// OpenBSL password length in bytes
#define OPEN_BSL_PASSWORD_LENGTH          (8)

// packet length
#define OPEN_BSL_CHKSUM_LEN               (2)  // 2 bytes checksum


//*****************************************************************************
// External function declarations
//*****************************************************************************

// core functions
extern void OpenBSL_CalcChksum(uint8_t byte, uint16_t *checksum);
extern void OpenBSL_DevInit(void);

// device/application specific functions
extern bool OpenBSL_EntryCheck(void);
extern void OpenBSL_AssertHdl(void);
extern void OpenBSL_Jump(memAddr_t addr);

// core command processing functions
extern void OpenBSL_ProcGetMemInfo(void);
extern void OpenBSL_ProcEraseImg(void);
extern void OpenBSL_ProcDnldImg(void);
extern void OpenBSL_ProcUpldImg(void);
extern void OpenBSL_ProcCalcChksm(void);
extern void OpenBSL_ProcEraseSegment(void);
extern void OpenBSL_ProcDnldSegment(void);
extern void OpenBSL_ProcUpldSegment(void);
extern void OpenBSL_ProcPwdCmd(void);
extern void OpenBSL_ProcJumpToAddr(void);
extern void OpenBSL_ProcRunApp(void);

// communication module functions
extern void OpenBSL_CommInit(void);
extern uint8_t OpenBSL_CommRcvByte(void);
extern uint8_t OpenBSL_CommRcvByteChksum(void);
extern uint16_t OpenBSL_CommRcvShort(void);
extern uint16_t OpenBSL_CommRcvShortChksum(void);
extern uint32_t OpenBSL_CommRcvLong(void);
extern uint32_t OpenBSL_CommRcvLongChksum(void);
extern void OpenBSL_CommSendByte(uint8_t byte);
extern void OpenBSL_CommSendByteChksum(uint8_t byte);
extern void OpenBSL_CommSendShort(uint16_t val);
extern void OpenBSL_CommSendShortChksum(uint16_t val);
extern void OpenBSL_CommSendLong(uint32_t val);
extern void OpenBSL_CommSendLongChksum(uint32_t val);

// memory module functions
extern void OpenBSL_MemInit(void);
extern uint8_t OpenBSL_MemReadByte(memAddr_t address);
extern void OpenBSL_MemOpenForErase(uint8_t section);
extern void OpenBSL_MemErase(memAddr_t address);
extern void OpenBSL_MemCloseForErase(uint8_t section);
extern void OpenBSL_MemOpenForWrite(uint8_t section);
extern void OpenBSL_MemWrite(uint8_t byte, memAddr_t address);
extern void OpenBSL_MemCloseForWrite(uint8_t section);
extern uint8_t OpenBSL_MemChkSection(memAddr_t address,  memAddr_t end);
extern uint32_t OpenBSL_MemSgmntGetSize(uint8_t section);


#endif /* _OPEN_BSL_INT_H_ */
