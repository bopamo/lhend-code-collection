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
* @file     OpenBSL_Mem.c
*
* @brief    Custom/device specific source file of OpenBSL implementation for
*           the device specific memory module
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
#include "OpenBSL_Int.h"

//*****************************************************************************
// Global variables
//*****************************************************************************

// memory sections on MSP430G2553
const OpenBSL_MemSect_t MemInfo[OPEN_BSL_NUM_OF_MEM_SECTIONS] = {
  // first section: InfoB-D - start address: 0x1000, end address : 0x10BF
  {
    INFO_MEM_START_ADDR, INFO_MEM_END_ADDR
  },
  // second section: Code memory - start address: 0xC000, end address : 0xFBFF
  {
    0xC000, 0xF7FF
  }
};

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
* OpenBSL_MemInit
*
* @brief     device/HW initialization memory module function for OpenBSL
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_MemInit(void)
{
  // setup Flash controller clock setting
  FCTL2 = FWKEY + FSSEL_2 + ((uint16_t)((8000000UL/400000UL)-1));
}

/**************************************************************************//**
*
* OpenBSL_MemReadByte
*
* @brief      read byte from device memory
*
* @param      address     address/pointer to the memory address to be read
*
* @return     -
*
******************************************************************************/
uint8_t OpenBSL_MemReadByte(memAddr_t address)
{
  return (*((uint8_t*)address));
}

/**************************************************************************//**
*
* OpenBSL_MemOpenForErase
*
* @brief      open memory for erase operation
*
* @param[in]  section    memory section number to be erased
*
* @return     -
*
******************************************************************************/
void OpenBSL_MemOpenForErase(uint8_t section)
{
  // unlock memory for erase operation
  FCTL3 = FWKEY;
}

/**************************************************************************//**
*
* OpenBSL_MemErase
*
* @brief      erase an address/segment from device memory
*
* @param[in]  address   memory address to be erased
*
* @return     -
*
******************************************************************************/
void OpenBSL_MemErase(memAddr_t address)
{
  // set individual segment erase operation
  FCTL1 = FWKEY + ERASE;

  // dummy write to erase memory segment
  *((uint8_t*)address) = 0;
}

/**************************************************************************//**
*
* OpenBSL_MemCloseForErase
*
* @brief      close memory for erase operation
*
* @param[in]  section    memory section number to be erased
*
* @return     -
*
******************************************************************************/
void OpenBSL_MemCloseForErase(uint8_t section)
{
  // lock memory from erase operation
  FCTL3 = FWKEY + LOCK;
}

/**************************************************************************//**
*
* OpenBSL_MemOpenForWrite
*
* @brief      open memory for write operation
*
* @param[in]  section    memory section number to be written
*
* @return     -
*
******************************************************************************/
void OpenBSL_MemOpenForWrite(uint8_t section)
{
  // unlock memory for write operation
  FCTL3 = FWKEY;

  // set byte/word write operation
  FCTL1 = FWKEY + WRT;
}

/**************************************************************************//**
*
* OpenBSL_MemWrite
*
* @brief      write byte to device memory
*
* @param[in]  byte      data byte to be written
* @param[in]  address   memory address to be written
*
* @return     -
*
******************************************************************************/
void OpenBSL_MemWrite(uint8_t byte, memAddr_t address)
{
  // write to given address
  *((uint8_t*)address) = byte;
}

/**************************************************************************//**
*
* OpenBSL_MemCloseForWrite
*
* @brief      close memory for write operation
*
* @param[in]  section    memory section number to be written
*
* @return     -
*
******************************************************************************/
void OpenBSL_MemCloseForWrite(uint8_t section)
{
  // lock memory from erase operation
  FCTL1 = FWKEY;
  FCTL3 = FWKEY + LOCK;
}


/**************************************************************************//**
*
* OpenBSL_MemChkSection
*
* @brief      check for validity of a given memory section parameter
*
* @param[in]  start        start address
* @param[in]  end          end address
*
* @return     0xFF if not a valid memory area, otherwise index of memory
*             area section
*
******************************************************************************/
uint8_t OpenBSL_MemChkSection(memAddr_t start, memAddr_t end)
{
  uint8_t i, sect = 0xFF;

  for(i=0 ; i<OPEN_BSL_NUM_OF_MEM_SECTIONS ; i++)
  {
    // check if start address and end address lie in the memory section
	if((start >= MemInfo[i].start_addr) &&
	   (end <= MemInfo[i].end_addr))
	{
	  // it is a valid memory area
	  sect = i;
	  break;
	}
  }

  return sect;
}

/**************************************************************************//**
*
* OpenBSL_MemSgmntGetSize
*
* @brief       get device memory segment size
*
* @param[in]   section     memory section number
*
* @return      size of the device memory segment size
*
******************************************************************************/
uint32_t OpenBSL_MemSgmntGetSize(uint8_t section)
{
  if(section == 0)
  {
	// INFO memory has 64 bytes segment size
	return ((uint32_t)64);
  }
  else
  {
    // main code flash memory has 512 bytes segment size
	return ((uint32_t)512);
  }
}

//*****************************************************************************
// Internal functions
//*****************************************************************************
