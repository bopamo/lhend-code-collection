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
* @file     OpenBSL_Core.c
*
* @brief    Core (HW-independent) source file of OpenBSL implementation
*
* @version  0.1
*
* @remark
*
******************************************************************************/

//*****************************************************************************
// Include section
//*****************************************************************************
#include "devtypes.h"
#include "OpenBSL.h"
#include "OpenBSL_Int.h"

//*****************************************************************************
// Global variables
//*****************************************************************************

// memory section info
extern const OpenBSL_MemSect_t MemInfo[OPEN_BSL_NUM_OF_MEM_SECTIONS];

// checksums
uint16_t OpenBSL_InChecksum = 0, OpenBSL_OutChecksum = 0;


//*****************************************************************************
// Macros (defines), data types, static variables
//*****************************************************************************

#define PASSWORD_RECEIVED_FLAG    (BV(0))

// OpenBSL application flag
static uint8_t OpenBSLFlag = 0;

#if (OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)

// OpenBSL password
static const uint8_t OpenBslPasswd[OPEN_BSL_PASSWORD_LENGTH] = OPEN_BSL_CONFIG_PASSWORD;

#endif //(OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)


// function pointer table
static void(* ProcFunc[])(void) = {
#define ENTRY(cmd, func)    func,
	OPEN_BSL_CMD_FUNC_TABLE
#undef ENTRY

	// NULL terminator
	NULL
};


// address parameters
static memAddr_t StartAddrParam, EndAddrParam;

// memory section index parameters
static uint8_t StartMemSectIdx, EndMemSectIdx;

//*****************************************************************************
// Internal function declarations
//*****************************************************************************

static bool GetAndCheckChksum(uint16_t chksum);
static uint8_t GetStartEndAddrParams(void);
static bool GetStartEndMemSectIdxParam(void);


//*****************************************************************************
// External functions
//*****************************************************************************

/**************************************************************************//**
*
* OpenBSL_Init()
*
* @brief     device/HW initialization function for OpenBSL
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_Init(void)
{
  // device specific initialization
  OpenBSL_DevInit();
}

/**************************************************************************//**
* @page OPEN_BSL_CMD_SYNC  OPEN_BSL_CMD_SYNC command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_SYNC is used to synchronize with the target device
* during startup and after error has occured. It is advised to execute this
* command first after startup and also after target device sends an error
* response to make sure that the host and target device are synchronized with
* each other in terms of command byte order.
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    -------
*    | CMD |
*    -------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_SYNC (0x90)</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_RESP_OK (0x00)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_RunBSL()
*
* @brief     BSL main routine
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_RunBSL(void)
{
  OpenBSL_Cmd_t cmd;

  while(1)
  {
    // wait for first byte byte - command byte
    cmd = (OpenBSL_Cmd_t) OpenBSL_CommRcvByte();

    if(cmd == OPEN_BSL_CMD_SYNC)
	{
      // send response OK
      OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_OK);
	}
    else if(cmd <= OPEN_BSL_CMD_PASSWD)
    {

    #if (OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)
      // check if password has been received
      if((cmd < OPEN_BSL_CMD_PASSWD) && (!(OpenBSLFlag &  PASSWORD_RECEIVED_FLAG)))
      {
        // send response OK
        OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_PASSWORD_PROTECTED);
      }
      else
    #endif // (OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)
      {
        if(ProcFunc[cmd] != NULL)
        {
          // call the processing function
          ProcFunc[cmd]();
        }
        else
        {
          // unsupported command
    	  OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_UNSUPPORTED_CMD);
        }
      }
    }
    else
    {
      // unknown command
      OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_UNKNOWN_CMD);
	}
  }

}

/**************************************************************************//**
*
* OpenBSL_CalcChksum()
*
* @brief      OpenBSL checksum calcuation routine - based on the simple
*             BSD checksum algorithm
*
* @param[in]  byte       input byte
* @param[out] checksum   pointer to the checksum buffer
*
* @return    -
*
******************************************************************************/
void OpenBSL_CalcChksum(uint8_t byte, uint16_t *checksum)
{
  uint16_t val = *checksum;

  // rotate right old value
  *checksum = (val >> 1) | ((val & 0x0001) ? 0x8000 : 0x0000);
  *checksum += byte;
}

/**************************************************************************//**
* @page OPEN_BSL_CMD_GET_MEM_INFO  OPEN_BSL_CMD_GET_MEM_INFO command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_GET_MEM_INFO is used to retrieve the memory layout
* information from the target device which consists of the following
* information:
*  - Number of continuous memory section
*  - Start and end addresses of every memory section
*
* The information is basically derived from the
* @ref const OpenBSL_MemSect_t MemInfo[OPEN_BSL_NUM_OF_MEM_SECTIONS]
* variable defined in @ref OpenBSL_Mem.c
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    -------
*    | CMD |
*    -------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_GET_MEM_INFO (0x00)</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    -------------------------------------------------------------------------
*    | RESP | NUM | START_0 | END_0 | ....... | START_N-1 | END_N-1 | CHKSUM |
*    -------------------------------------------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_GET_MEM_INFO|OPEN_BSL_RESP_BIT_MASK) (0x80)</td>
*     </tr>
*     <tr>
*       <td>NUM</td>
*       <td>1 byte</td>
*       <td>number of memory sections (N sections)</td>
*     </tr>
*     <tr>
*       <td>START_0 </td>
*       <td>4 bytes</td>
*       <td>start address of memory section index 0 (LSB first)</td>
*     </tr>
*     <tr>
*       <td>END_0 </td>
*       <td>4 bytes</td>
*       <td>end address of memory section index 0 (LSB first)</td>
*     </tr>
*     <tr>
*       <td>START_N-1 </td>
*       <td>4 bytes</td>
*       <td>start address of memory section N-1 (LSB first)</td>
*     </tr>
*     <tr>
*       <td>END_N-1 </td>
*       <td>4 bytes</td>
*       <td>end address of memory section N-1 (LSB first)</td>
*     </tr>
*     <tr>
*       <td>CHKSUM</td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from NUM to END_N-1 (LSB first)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcGetMemInfo
*
* @brief     send memory info regarding on-chip non-volatile memory
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcGetMemInfo(void)
{
  uint8_t i;

  // send ok response header
  OpenBSL_CommSendByte((uint8_t)(OPEN_BSL_CMD_GET_MEM_INFO | OPEN_BSL_RESP_BIT_MASK));

  // reset checksum
  OpenBSL_OutChecksum = 0;

  // send number of memory sections
  OpenBSL_CommSendByteChksum((uint8_t) OPEN_BSL_NUM_OF_MEM_SECTIONS);

  // send memory sections info
  for(i=0 ; i<OPEN_BSL_NUM_OF_MEM_SECTIONS ; i++)
  {
    // send start and end address (LSB first)
	OpenBSL_CommSendLongChksum((uint32_t)MemInfo[i].start_addr);
	OpenBSL_CommSendLongChksum((uint32_t)MemInfo[i].end_addr);
  }

  // send checksum
  OpenBSL_CommSendShort(OpenBSL_OutChecksum);
}

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_ERASE_IMAGE  OPEN_BSL_CMD_ERASE_IMAGE command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_ERASE_IMAGE is used to erase all/a specific memory section.
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE
* compile options in @ref OpenBSL_Config.h
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    -------------
*    | CMD | IDX |
*    -------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_ERASE_IMAGE (0x01)</td>
*     </tr>
*     <tr>
*       <td>IDX</td>
*       <td>1 byte</td>
*       <td> memory section index (0 - @ref OPEN_BSL_NUM_OF_MEM_SECTIONS-1),
*           0xFF for all memory section</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_ERASE_IMAGE|OPEN_BSL_RESP_BIT_MASK) (0x81)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcEraseImg
*
* @brief     erase application image/memory area
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcEraseImg(void)
{
  memAddr_t addr;

  if(GetStartEndMemSectIdxParam() == true)
  {
    // send memory sections info
    for( ; StartMemSectIdx<=EndMemSectIdx ; StartMemSectIdx++)
    {
   	  // open memory section
   	  OpenBSL_MemOpenForErase(StartMemSectIdx);

      for(addr = (memAddr_t) MemInfo[StartMemSectIdx].start_addr ;
          addr < (memAddr_t) MemInfo[StartMemSectIdx].end_addr ;
    	  addr += OpenBSL_MemSgmntGetSize(StartMemSectIdx))
      {
        // erase memory
        OpenBSL_MemErase(addr);
      }

	  // close memory
	  OpenBSL_MemCloseForErase(StartMemSectIdx);
    }

    // send ok response header
    OpenBSL_CommSendByte((uint8_t)(OPEN_BSL_CMD_ERASE_IMAGE | OPEN_BSL_RESP_BIT_MASK));
  }
}
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_IMAGE == true)


#if (OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_DOWNLOAD_IMAGE  OPEN_BSL_CMD_DOWNLOAD_IMAGE command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_DOWNLOAD_IMAGE is used to download byte streams to
* a specific memory section.
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE
* compile options in @ref OpenBSL_Config.h
*
* The host shall implement small delay (device specific) between sending the bytes
* of the byte streams since the target device might need additional
* to write every incoming byte into the memory
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    ----------------------------------------------------
*    | CMD | IDX | BYTE_0 | ....... | BYTE_N-1 | CHKSUM |
*    ----------------------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_DOWNLOAD_IMAGE (0x02)</td>
*     </tr>
*     <tr>
*       <td>IDX</td>
*       <td>1 byte</td>
*       <td> memory section index (0 - @ref OPEN_BSL_NUM_OF_MEM_SECTIONS-1)</td>
*     </tr>
*     <tr>
*       <td>BYTE_0</td>
*       <td>1 byte</td>
*       <td>byte stream index 0</td>
*     </tr>
*     <tr>
*       <td>BYTE_N-1</td>
*       <td>1 byte</td>
*       <td>byte stream index N-1, where N is the length of memory section
*          (can be retrieved from @ref OPEN_BSL_CMD_GET_MEM_INFO) </td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from BYTE_0 to BYTE_N-1</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_DOWNLOAD_IMAGE|OPEN_BSL_RESP_BIT_MASK) (0x82)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcDnldImg
*
* @brief     download/write incoming byte streams to application image/memory
*            area, ended with 2 checksum bytes
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcDnldImg(void)
{
  uint8_t idx, byte;
  memAddr_t addr;

  // get memory section index
  idx = OpenBSL_CommRcvByte();
  if(idx < OPEN_BSL_NUM_OF_MEM_SECTIONS)
  {
	// reset checksum
	OpenBSL_InChecksum = 0;

	// open memory section
	OpenBSL_MemOpenForWrite(idx);

	for(addr = (memAddr_t) MemInfo[idx].start_addr ;
		addr <= (memAddr_t) MemInfo[idx].end_addr ;
		addr++)
	{
	  // write byte memory
      byte = OpenBSL_CommRcvByteChksum();
	  OpenBSL_MemWrite(byte, addr);
	}

	// close memory
	OpenBSL_MemCloseForWrite(idx);

	// get and verify checksum
	if(GetAndCheckChksum(OpenBSL_InChecksum) == false)
	{
	  return;
	}

    // send ok response
    OpenBSL_CommSendByte(((uint8_t)OPEN_BSL_CMD_DOWNLOAD_IMAGE) | OPEN_BSL_RESP_BIT_MASK);
  }
  else
  {
	// invalid index - send error response
	OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_INVALID_PARAM);
  }
}
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_IMAGE == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_UPLOAD_IMAGE  OPEN_BSL_CMD_UPLOAD_IMAGE command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_UPLOAD_IMAGE is used to upload byte streams from
* a specific memory section.
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE
* compile options in @ref OpenBSL_Config.h
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    -------------
*    | CMD | IDX |
*    -------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_UPLOAD_IMAGE (0x03)</td>
*     </tr>
*     <tr>
*       <td>IDX</td>
*       <td>1 byte</td>
*       <td> memory section index (0 - @ref OPEN_BSL_NUM_OF_MEM_SECTIONS-1)</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    -------------------------------------------------
*    | RESP | IDX | BYTE_0 | ... | BYTE_N-1 | CHKSUM |
*    -------------------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_UPLOAD_IMAGE|OPEN_BSL_RESP_BIT_MASK) (0x83)</td>
*     </tr>
*     <tr>
*       <td>IDX</td>
*       <td>1 byte</td>
*       <td> memory section index (0 - @ref OPEN_BSL_NUM_OF_MEM_SECTIONS-1)</td>
*     </tr>
*     <tr>
*       <td>BYTE_0</td>
*       <td>1 byte</td>
*       <td>byte stream index 0</td>
*     </tr>
*     <tr>
*       <td>BYTE_N-1</td>
*       <td>1 byte</td>
*       <td>byte stream index N-1, where N is the length of memory section
*          (can be retrieved from @ref OPEN_BSL_CMD_GET_MEM_INFO) </td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from BYTE_0 to BYTE_N-1</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcUpldImg
*
* @brief     upload/read application image/memory area as byte streams,
*            ended with 2 checksum bytes
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcUpldImg(void)
{
  memAddr_t addr, len;

  if(GetStartEndMemSectIdxParam() == true)
  {
    // send memory sections info
    for( ; StartMemSectIdx<=EndMemSectIdx ; StartMemSectIdx++)
    {
	  // reset checksum
	  OpenBSL_OutChecksum = 0;

	  // initialize variables
	  addr = (memAddr_t) MemInfo[StartMemSectIdx].start_addr;
	  len = (memAddr_t) (MemInfo[StartMemSectIdx].end_addr - MemInfo[StartMemSectIdx].start_addr + 1);

	  // send ok response header
	  OpenBSL_CommSendByte((uint8_t)(OPEN_BSL_CMD_UPLOAD_IMAGE | OPEN_BSL_RESP_BIT_MASK));

	  // send byte of the current memory section number
	  OpenBSL_CommSendByte(StartMemSectIdx);

	  while(len--)
	  {
	    // send image bytes
	    OpenBSL_CommSendByteChksum(OpenBSL_MemReadByte(addr++));
	  }

	  // send checksum
	  OpenBSL_CommSendShort(OpenBSL_OutChecksum);
    }
  }
}
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_IMAGE == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_CALCULATE_CHECKSUM  OPEN_BSL_CMD_CALCULATE_CHECKSUM command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_CALCULATE_CHECKSUM is used to calculate the checksum value
* of given memory area
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM
* compile options in @ref OpenBSL_Config.h
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    ----------------------------------------
*    | CMD | START_ADDR | END_ADDR | CHKSUM |
*    ----------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_CALCULATE_CHECKSUM (0x05)</td>
*     </tr>
*     <tr>
*       <td>START_ADDR</td>
*       <td>4 bytes</td>
*       <td>start address (LSB first) of the memory area whose content checksum to be calculated</td>
*     </tr>
*     <tr>
*       <td>END_ADDR</td>
*       <td>4 bytes</td>
*       <td>end address (LSB first) of the memory area whose content checksum to be calculated</td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from START_ADDR to END_ADDR</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    -----------------
*    | RESP | CHKSUM |
*    -----------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_CALCULATE_CHECKSUM|OPEN_BSL_RESP_BIT_MASK) (0x85)</td>
*     </tr>
*     <tr>
*       <td>CHKSUM</td>
*       <td>2 bytes</td>
*       <td>16 bit of calculated checksum (LSB first), calculated from the
*           memory content starting from START_ADDR to END_ADDR</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcCalcChksm
*
* @brief     calculate 2 checksum bytes of given memory area
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcCalcChksm(void)
{
  uint8_t idx;
  uint16_t checksum = 0;

  // get addresses parameter
  idx = GetStartEndAddrParams();
  if(idx != 0xFF)
  {
    // calculate checksum
	for( ; StartAddrParam <= EndAddrParam ; StartAddrParam++)
	{
      OpenBSL_CalcChksum(OpenBSL_MemReadByte(StartAddrParam), &checksum);
	}

	// send response
	OpenBSL_CommSendByte((uint8_t) OPEN_BSL_CMD_CALCULATE_CHECKSUM | OPEN_BSL_RESP_BIT_MASK);
	OpenBSL_CommSendShort(checksum);
  }
}
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_CALCULATE_CHECKSUM == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_ERASE_SEGMENT OPEN_BSL_CMD_ERASE_SEGMENT command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_ERASE_SEGMENT is used to erase the given memory segment
* with segment's start and end addresses as parameters
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT
* compile options in @ref OpenBSL_Config.h
*
* Depeding on the device target hardware, the command might effect other
* memory area (e.g. take into account if target device can only
* erase the memory page-wise).
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    ----------------------------------------
*    | CMD | START_ADDR | END_ADDR | CHKSUM |
*    ----------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_ERASE_SEGMENT (0x06)</td>
*     </tr>
*     <tr>
*       <td>START_ADDR</td>
*       <td>4 bytes</td>
*       <td>start address (LSB first) of the memory area to be erased</td>
*     </tr>
*     <tr>
*       <td>END_ADDR</td>
*       <td>4 bytes</td>
*       <td>end address (LSB first) of the memory area to be erased</td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from START_ADDR to END_ADDR</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_ERASE_SEGMENT|OPEN_BSL_RESP_BIT_MASK) (0x86)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcEraseSegment
*
* @brief     erase certain part of application image/memory area
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcEraseSegment(void)
{
  uint8_t idx;

  // get addresses parameter
  idx = GetStartEndAddrParams();
  if(idx != 0xFF)
  {
	// open memory section
	OpenBSL_MemOpenForErase(idx);

	for(; StartAddrParam <= EndAddrParam ; StartAddrParam += OpenBSL_MemSgmntGetSize(idx))
	{
	  // erase memory
      OpenBSL_MemErase(StartAddrParam);
	}

	// close memory
    OpenBSL_MemCloseForErase(idx);

    // send ok response header
  	OpenBSL_CommSendByte((uint8_t)(OPEN_BSL_CMD_ERASE_SEGMENT | OPEN_BSL_RESP_BIT_MASK));
  }

}
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_ERASE_SEGMENT == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_DOWNLOAD_SEGMENT OPEN_BSL_CMD_DOWNLOAD_SEGMENT command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_DOWNLOAD_SEGMENT is used to download/write the given
* memory segment with given data bytes
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT
* compile options in @ref OpenBSL_Config.h
*
* Depeding on the device target hardware (e.g. target device with flash
* memory), it might necessary to erase the memory section first
* (e.g. @ref with OPEN_BSL_CMD_ERASE_SEGMENT) before writing the bytes.
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    ---------------------------------------------------------------
*    | CMD | LEN | START_ADDR | BYTE_0 | ..... | BYTE_N-1 | CHKSUM |
*    ---------------------------------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_DOWNLOAD_SEGMENT (0x07)</td>
*     </tr>
*     <tr>
*       <td>LEN</td>
*       <td>1 byte</td>
*       <td>packet length from START_ADDR to CHKSUM</td>
*     </tr>
*     <tr>
*       <td>START_ADDR</td>
*       <td>4 bytes</td>
*       <td>start address (LSB first) of the memory area to be written</td>
*     </tr>
*     <tr>
*       <td>BYTE_0</td>
*       <td>1 byte</td>
*       <td>data byte index 0</td>
*     </tr>
*     <tr>
*       <td>BYTE_N-1</td>
*       <td>1 byte</td>
*       <td>data byte index N-1, where N is the number of data to be written</td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from START_ADDR to BYTE_N-1</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_DOWNLOAD_SEGMENT|OPEN_BSL_RESP_BIT_MASK) (0x87)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcDnldSegment
*
* @brief     download/write input bytes to certain part of application
*            image/memory area
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcDnldSegment(void)
{
  uint8_t len, idx, byte;
  memAddr_t addr;

  // get data segment length
  len = OpenBSL_CommRcvByte() - OPEN_BSL_CHKSUM_LEN - OPEN_BSL_ADDR_DATA_SIZE;

  // clear chksum
  OpenBSL_InChecksum = 0;

  // get address
  addr = (memAddr_t) OpenBSL_CommRcvLongChksum();

  // check memory area
  idx = OpenBSL_MemChkSection(addr, (addr+len-1));
  if(idx != 0xFF)
  {
    // open memory section
	OpenBSL_MemOpenForWrite(idx);

	for( ; len-- ; addr++)
	{
	  // write byte memory
	  byte = OpenBSL_CommRcvByteChksum();
	  OpenBSL_MemWrite(byte, addr);
	}

	// close memory
	OpenBSL_MemCloseForWrite(idx);

	// get and verify checksum
	if(GetAndCheckChksum(OpenBSL_InChecksum) != false)
	{
	  // send ok response
	  OpenBSL_CommSendByte(((uint8_t)OPEN_BSL_CMD_DOWNLOAD_SEGMENT) | OPEN_BSL_RESP_BIT_MASK);
	}
  }
  else
  {
    // invalid memory segment - send error response
    OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_INVALID_PARAM);
  }
}
#endif // (OPEN_BSL_CONFIG_SUPPORT_CMD_DOWNLOAD_SEGMENT == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SEGMENT == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_UPLOAD_SEGMENT OPEN_BSL_CMD_UPLOAD_SEGMENT command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_UPLOAD_SEGMENT is used to upload/read the given
* memory segment.
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SEGMENT
* compile options in @ref OpenBSL_Config.h
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    ----------------------------------------
*    | CMD | START_ADDR | END_ADDR | CHKSUM |
*    ----------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_UPLOAD_SEGMENT (0x08)</td>
*     </tr>
*     <tr>
*       <td>START_ADDR</td>
*       <td>4 bytes</td>
*       <td>start address (LSB first) of the memory area to be uploaded</td>
*     </tr>
*     <tr>
*       <td>START_ADDR</td>
*       <td>4 bytes</td>
*       <td>start address (LSB first) of the memory area to be uploaded</td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from START_ADDR to END_ADDR</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    ------------------------------------------------------
*    | RESP | LEN |  BYTE_0 | ....... | BYTE_N-1 | CHKSUM |
*    ------------------------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_UPLOAD_SEGMENT|OPEN_BSL_RESP_BIT_MASK) (0x88)</td>
*     </tr>
*     <tr>
*       <td>LEN</td>
*       <td>1 byte</td>
*       <td>packet length (calculated from BYTE_0 to BYTE_N-1)</td>
*     </tr>
*     <tr>
*       <td>BYTE_0</td>
*       <td>1 byte</td>
*       <td>data byte index 0</td>
*     </tr>
*     <tr>
*       <td>BYTE_N-1</td>
*       <td>1 byte</td>
*       <td>data byte index N-1, where N is the total number of data byte
*           to be uploaded</td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from BYTE_0 to BYTE_N-1</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcUpldSegment
*
* @brief     upload/read bytes from certain part of application
*            image/memory area
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcUpldSegment(void)
{
  uint8_t idx;

  // get addresses parameter
  idx = GetStartEndAddrParams();
  if(idx != 0xFF)
  {
    // send ok response
    OpenBSL_CommSendByte(((uint8_t)OPEN_BSL_CMD_UPLOAD_SEGMENT) | OPEN_BSL_RESP_BIT_MASK);

    // send length
    OpenBSL_CommSendByte((uint8_t)((EndAddrParam - StartAddrParam + 1) + OPEN_BSL_CHKSUM_LEN));

    // reset checksum
    OpenBSL_OutChecksum = 0;

    // send data bytes
    for( ; StartAddrParam<=EndAddrParam ; StartAddrParam++)
    {
      OpenBSL_CommSendByteChksum(OpenBSL_MemReadByte(StartAddrParam));
    }

    // send checksum
    OpenBSL_CommSendShort(OpenBSL_OutChecksum);
  }
}
#endif // (OPEN_BSL_CONFIG_SUPPORT_CMD_UPLOAD_SEGMENT == true)

#if (OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_PASSWD OPEN_BSL_CMD_PASSWD command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_PASSWD is used to unlock other commands.
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_PASSWORD_PROTECTED
* compile options in @ref OpenBSL_Config.h
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    --------------------------------------------
*    | CMD | PWD_0 | ....... | PWD_N-1 | CHKSUM |
*    --------------------------------------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_PASSWD (0x09)</td>
*     </tr>
*     <tr>
*       <td>PWD_0</td>
*       <td>1 byte</td>
*       <td>password byte index 0</td>
*     </tr>
*     <tr>
*       <td>PWD_N-1</td>
*       <td>1 byte</td>
*       <td>password byte index N-1, where N is @ref OPEN_BSL_PASSWORD_LENGTH</td>
*     </tr>
*     <tr>
*       <td>CHKSUM </td>
*       <td>2 bytes</td>
*       <td>16 bit packet checksum (LSB first),
*           calculated starting from PWD_0 to PWD_N-1</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_PASSWD|OPEN_BSL_RESP_BIT_MASK) (0x89)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcPwdCmd
*
* @brief     process incoming password to unlock other commands
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_ProcPwdCmd(void)
{
  uint8_t i, byte;
  bool correct_pwd = true;

  // reset checksum
  OpenBSL_InChecksum = 0;

  // get the password
  for(i=0 ; i<OPEN_BSL_PASSWORD_LENGTH ; i++)
  {
    // get one byte
    byte = OpenBSL_CommRcvByteChksum();
    if(byte != OpenBslPasswd[i])
    {
      // wrong password;
      correct_pwd = false;
    }
  }

  // get and verify checksum
  if(GetAndCheckChksum(OpenBSL_InChecksum) == false)
  {
	return;
  }

  // last check: whether correct password is received
  if(correct_pwd == true)
  {
    // set flag
    OpenBSLFlag |= PASSWORD_RECEIVED_FLAG;

    // send ok response
    OpenBSL_CommSendByte(((uint8_t)OPEN_BSL_CMD_PASSWD) | OPEN_BSL_RESP_BIT_MASK);
  }
  else
  {
    // send error response
    OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_INVALID_PARAM);
  }
}
#endif //(OPEN_BSL_CONFIG_PASSWORD_PROTECTED == true)

#if (OPEN_BSL_CONFIG_SUPPORT_CMD_JUMP_TO_ADDR == true)
/**************************************************************************//**
* @page OPEN_BSL_CMD_JUMP_TO_ADDR OPEN_BSL_CMD_JUMP_TO_ADDR command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_JUMP_TO_ADDR is used to make the target device jump to a
* certain destination address and start executing the instruction from there
*
* This command can be enabled by using the @ref OPEN_BSL_CONFIG_SUPPORT_CMD_JUMP_TO_ADDR
* compile options in @ref OpenBSL_Config.h
*
* Depending on the target/application specific implementation, this function
* most probably causes the device also to leave the BSL mode
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    ---------------
*    | CMD |  ADDR |
*    ---------------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_JUMP_TO_ADDR (0x0A)</td>
*     </tr>
*     <tr>
*       <td>ADDR </td>
*       <td>4 bytes</td>
*       <td>Destination address (LSB first)</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_JUMP_TO_ADDR|OPEN_BSL_RESP_BIT_MASK) (0x8A)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcJumpToAddr
*
* @brief     process command to jump to certain given address
*
* @param     -
*
* @return    -
*
* @remark
*
*  CMD packet format:
*
*
*    CMD (1 byte) = OPEN_BSL_CMD_JUMP_TO_ADDR
*    ADDR (4 bytes) =
*
*
*  positive RESPONSE packet format:
*
*    --------
*    | RESP |
*    --------
*
*    RESP (1 byte) = OPEN_BSL_CMD_JUMP_TO_ADDR | OPEN_BSL_RESP_BIT_MASK
*
*
******************************************************************************/
void OpenBSL_ProcJumpToAddr(void)
{
  memAddr_t addr;

  // receive address
  addr = (memAddr_t) OpenBSL_CommRcvLong();

  // send ok response
  OpenBSL_CommSendByte(((uint8_t)OPEN_BSL_CMD_JUMP_TO_ADDR) | OPEN_BSL_RESP_BIT_MASK);

  // jump
  OpenBSL_Jump(addr);
}
#endif //(OPEN_BSL_CONFIG_SUPPORT_CMD_JUMP_TO_ADDR == true)

/**************************************************************************//**
* @page OPEN_BSL_CMD_RUN_APP OPEN_BSL_CMD_RUN_APP command
*
*
* @section DESCRIPTION   Description
*
* The OPEN_BSL_CMD_RUN_APP is used to make the target device to leave the BSL
* mode and start executing the application.
*
*
* @section PACKET_FORMAT   Packet Format
*
* <b>REQUEST packet format</b>
*
*   <div markdown="0">
*    -------
*    | CMD |
*    -------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>CMD</td>
*       <td>1 byte</td>
*       <td>OPEN_BSL_CMD_RUN_APP (0x04)</td>
*     </tr>
*   </table>
*
*
* <b>Positive RESPONSE packet format:</b>
*
*   <div markdown="0">
*    --------
*    | RESP |
*    --------
*   </div>
*
*   <table>
*     <tr>
*       <th>Field Name</th>
*       <th>Length</th>
*       <th>Description</th>
*     </tr>
*     <tr>
*       <td>RESP</td>
*       <td>1 byte</td>
*       <td>(OPEN_BSL_CMD_RUN_APP|OPEN_BSL_RESP_BIT_MASK) (0x84)</td>
*     </tr>
*   </table>
*
*
******************************************************************************/
/**************************************************************************//**
*
* OpenBSL_ProcRun
*
* @brief     process command to run application (jump to application reset
*            vector at address APP_MEM_RESET_VECT_ADDR
*
* @param     -
*
* @return    -
*
* @remark
*
*  CMD packet format:
*
*    ---------------
*    | CMD |  ADDR |
*    ---------------
*
*    CMD (1 byte) = OPEN_BSL_CMD_RUN_APP
*
*
*  positive RESPONSE packet format:
*
*    --------
*    | RESP |
*    --------
*
*    RESP (1 byte) = OPEN_BSL_CMD_RUN_APP | OPEN_BSL_RESP_BIT_MASK
*
*
******************************************************************************/
void OpenBSL_ProcRunApp(void)
{
  // send ok response
  OpenBSL_CommSendByte(((uint8_t)OPEN_BSL_CMD_RUN_APP) | OPEN_BSL_RESP_BIT_MASK);

  // call device specific function to start application
  OpenBSL_RunApp();
}

//*****************************************************************************
// Internal functions
//*****************************************************************************

/**************************************************************************//**
*
* GetAndCheckChksum
*
* @brief       get 2 bytes of incoming checksum bytes and compare with the
*              given calculated checksumm, sends also error checksum response
*              if necessary
*
* @param[in]   chksum    calculated checksum of previously received data bytes
*
* @return      true if checksum matches, otherwise false
*
******************************************************************************/
static bool GetAndCheckChksum(uint16_t chksum)
{
  bool ret_val = true;

  // receive 2 bytes checksum
  if(OpenBSL_CommRcvShort() != chksum)
  {
    // send error response
    OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_INVALID_CHECKSUM);

    // return false
    ret_val = false;
  }

  return ret_val;
}

/**************************************************************************//**
*
* GetStartEndAddrParams
*
* @brief       get 8 bytes of incoming start and end address parameters,
*              check the checksum and memory section validity. Send error
*              message upon wrong checksum or invalid memory section parameter
*
* @param[in]   start    pointer to the start address parameter
* @param[in]   end      pointer to the end address parameter
*
* @return     0xFF if failed, otherwise index of memory area section of
*             received start and end address
*
******************************************************************************/
static uint8_t GetStartEndAddrParams(void)
{
  uint8_t idx = 0xFF;

  // reset checksum
  OpenBSL_InChecksum = 0;

  // get start address
  StartAddrParam = (memAddr_t) OpenBSL_CommRcvLongChksum();

  // get end address
  EndAddrParam = (memAddr_t) OpenBSL_CommRcvLongChksum();

  // get and verify checksum
  if(GetAndCheckChksum(OpenBSL_InChecksum) == true)
  {
	// check memory area validity
	idx = OpenBSL_MemChkSection(StartAddrParam, EndAddrParam);
	if(idx == 0xFF)
	{
	  // send error response
	  OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_INVALID_PARAM);
	}
  }

  return idx;
}


/**************************************************************************//**
*
* GetStartEndMemSectIdxParam
*
* @brief       get one byte of incoming memory section index parameter and set
*              StartMemSectIdx, EndMemSectIdx accordingly
*
* @param       -
*
* @return      true if the received index is valid, otherwise false
*
******************************************************************************/
static bool GetStartEndMemSectIdxParam(void)
{
  bool ret_val = true;

  // get memory section index
  StartMemSectIdx = OpenBSL_CommRcvByte();
  if(StartMemSectIdx == 0xFF)
  {
	// all memory sections access
	StartMemSectIdx = 0;
	EndMemSectIdx = OPEN_BSL_NUM_OF_MEM_SECTIONS-1;
  }
  else if(StartMemSectIdx < OPEN_BSL_NUM_OF_MEM_SECTIONS)
  {
	// only single memory section access
	EndMemSectIdx = StartMemSectIdx;
  }
  else
  {
	// invalid index - send error response
	OpenBSL_CommSendByte((uint8_t)OPEN_BSL_RESP_ERR_INVALID_PARAM);

	// return false
	ret_val = false;
  }

  return ret_val;
}

