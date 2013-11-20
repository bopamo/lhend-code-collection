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
* @file     OpenBSL_Comm.c
*
* @brief    Custom/device specific source file of OpenBSL implementation for
*           the communication module
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
* OpenBSL_CommInit
*
* @brief     device/HW initialization communication module function for OpenBSL
*
* @param     -
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommInit(void)
{
  // check calibration data
  OPEN_BSL_ASSERT(CALBC1_8MHZ != 0xFF);
  OPEN_BSL_ASSERT(CALDCO_8MHZ != 0xFF);

  // set basic clock module+
  BCSCTL1 = XT2OFF | CALBC1_8MHZ;
  DCOCTL = CALDCO_8MHZ;
  //BCSCTL2 |= DIVS_3;  // 1 MHz SMCLK
  BCSCTL3 = LFXT1S_2; // source VLOCLK as ACLK

  // initialize UART USCI_A0 interface at P1.1 (RXD) and P1.2 (TXD)
  P1SEL = BIT1 + BIT2;
  P1SEL2 = BIT1 + BIT2;
  UCA0CTL1 |= UCSWRST + UCSSEL_2;
  UCA0BR0 = (833 & 0xFF);
  UCA0BR1 = (833 >> 8);
  UCA0MCTL = UCBRS_2;
  UCA0CTL1 &= ~UCSWRST;
}

/**************************************************************************//**
*
* OpenBSL_CommRcvByte
*
* @brief     receive a single byte from communication line - blocking function
*
* @param     -
*
* @return    received single byte from communication line
*
******************************************************************************/
uint8_t OpenBSL_CommRcvByte(void)
{
  // wait until RX bit is set
  while(!(IFG2 & UCA0RXIFG));

  return ((uint8_t)UCA0RXBUF);
}

/**************************************************************************//**
*
* OpenBSL_CommRcvByteChksum
*
* @brief     receive a single byte from communication line and update
*            the OpenBSL_InChecksum checksum variable - blocking function
*
* @param[in] checksum    pointer to the checksum buffer to be updated
*
* @return    received single byte from communication line
*
******************************************************************************/
uint8_t OpenBSL_CommRcvByteChksum(void)
{
  uint8_t byte;

  // get byte
  byte = OpenBSL_CommRcvByte();

  // update the checksum value
  OpenBSL_CalcChksum(byte, &OpenBSL_InChecksum);

  return byte;
}

/**************************************************************************//**
*
* OpenBSL_CommRcvShort
*
* @brief     receive a two bytes data (16 bit) from communication line -
*            blocking function
*
* @param     -
*
* @return    received two bytes data from communication line
*
******************************************************************************/
uint16_t OpenBSL_CommRcvShort(void)
{
  uint16_t val;
  uint8_t *ptr = (uint8_t*) &val;

  // receive starting LSB
#ifdef LITTLE_ENDIAN
  ptr[0] = OpenBSL_CommRcvByte();
  ptr[1] = OpenBSL_CommRcvByte();
#else
  ptr[1] = OpenBSL_CommRcvByte();
  ptr[0] = OpenBSL_CommRcvByte();
#endif

  return val;
}

/**************************************************************************//**
*
* OpenBSL_CommRcvShortChksum
*
* @brief     receive a two bytes data (16 bit) from communication line and
*            calculate/update the OpenBSL_InChecksum checksum variable -
*            blocking function
*
* @param     -
*
* @return    received two bytes data from communication line
*
******************************************************************************/
uint16_t OpenBSL_CommRcvShortChksum(void)
{
  uint16_t val;
  uint8_t *ptr = (uint8_t*) &val;

  // receive starting LSB
#ifdef LITTLE_ENDIAN
  ptr[0] = OpenBSL_CommRcvByteChksum();
  ptr[1] = OpenBSL_CommRcvByteChksum();
#else
  ptr[1] = OpenBSL_CommRcvByteChksum();
  ptr[0] = OpenBSL_CommRcvByteChksum();
#endif

  return val;
}

/**************************************************************************//**
*
* OpenBSL_CommRcvLong
*
* @brief     receive a four bytes data (32 bit) from communication line -
*            blocking function
*
* @param     -
*
* @return    received four bytes data from communication line
*
******************************************************************************/
uint32_t OpenBSL_CommRcvLong(void)
{
  uint32_t val;
  uint8_t *ptr = (uint8_t*) &val;

  // receive starting LSB
#ifdef LITTLE_ENDIAN
  ptr[0] = OpenBSL_CommRcvByte();
  ptr[1] = OpenBSL_CommRcvByte();
  ptr[2] = OpenBSL_CommRcvByte();
  ptr[3] = OpenBSL_CommRcvByte();
#else
  ptr[3] = OpenBSL_CommRcvByte();
  ptr[2] = OpenBSL_CommRcvByte();
  ptr[1] = OpenBSL_CommRcvByte();
  ptr[0] = OpenBSL_CommRcvByte();
#endif

  return val;
}

/**************************************************************************//**
*
* OpenBSL_CommRcvLongChksum
*
* @brief     receive a four bytes data (32 bit) from communication line and
*            calculate/update the OpenBSL_InChecksum checksum variable -
*            blocking function
*
* @param     -
*
* @return    received four bytes from communication line
*
******************************************************************************/
uint32_t OpenBSL_CommRcvLongChksum(void)
{
  uint32_t val;
  uint8_t *ptr = (uint8_t*) &val;

  // receive starting LSB
#ifdef LITTLE_ENDIAN
  ptr[0] = OpenBSL_CommRcvByteChksum();
  ptr[1] = OpenBSL_CommRcvByteChksum();
  ptr[2] = OpenBSL_CommRcvByteChksum();
  ptr[3] = OpenBSL_CommRcvByteChksum();
#else
  ptr[3] = OpenBSL_CommRcvByteChksum();
  ptr[2] = OpenBSL_CommRcvByteChksum();
  ptr[1] = OpenBSL_CommRcvByteChksum();
  ptr[0] = OpenBSL_CommRcvByteChksum();
#endif

  return val;
}


/**************************************************************************//**
*
* OpenBSL_CommSendByte
*
* @brief     send a single byte to communication line - blocking function
*
* @param     single byte to be sent out to communication line
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommSendByte(uint8_t byte)
{
  // wait until TX bit is set
  while(!(IFG2 & UCA0TXIFG));

  // push byte to TX buffer
  UCA0TXBUF = byte;
}

/**************************************************************************//**
*
* OpenBSL_CommSendByteChksum
*
* @brief     send a single byte to communication line and update the
*            OpenBSL_OutChecksum checksum variable - blocking function
*
* @param[in]  byte       input byte to be sent out
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommSendByteChksum(uint8_t byte)
{
  // first update checksum
  OpenBSL_CalcChksum(byte, &OpenBSL_OutChecksum);

  // then send out the byte
  OpenBSL_CommSendByte(byte);
}


/**************************************************************************//**
*
* OpenBSL_CommSendShort
*
* @brief     send a two bytes (16 bit) value to communication line in
*            little endian byte order (LSB first) - blocking function
*
* @param[in] val    16 bit data to be sent out to communication line
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommSendShort(uint16_t val)
{
  uint8_t *ptr = (uint8_t*) &val;

  // send starting LSB
#ifdef LITTLE_ENDIAN
  OpenBSL_CommSendByte(ptr[0]);
  OpenBSL_CommSendByte(ptr[1]);
#else
  OpenBSL_CommSendByte(ptr[1]);
  OpenBSL_CommSendByte(ptr[0]);
#endif
}

/**************************************************************************//**
*
* OpenBSL_CommSendShortChksum
*
* @brief     send a two bytes (16 bit) to communication line and update the
*            OpenBSL_OutChecksum checksum variable - blocking function
*
* @param[in]  val        input word (16 bit) to be sent out
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommSendShortChksum(uint16_t val)
{
  uint8_t *ptr = (uint8_t*) &val;

  // calculate the checksum first
#ifdef LITTLE_ENDIAN
  OpenBSL_CommSendByteChksum(ptr[0]);
  OpenBSL_CommSendByteChksum(ptr[1]);
#else
  OpenBSL_CommSendByteChksum(ptr[1]);
  OpenBSL_CommSendByteChksum(ptr[0]);
#endif
}

/**************************************************************************//**
*
* OpenBSL_CommSendLong
*
* @brief     send a four bytes (32 bit) value to communication line in
*            little endian byte order (LSB first) - blocking function
*
* @param[in] val    32 bit data to be sent out to communication line
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommSendLong(uint32_t val)
{
  uint8_t *ptr = (uint8_t*) &val;

  // send starting LSB
#ifdef LITTLE_ENDIAN
  OpenBSL_CommSendByte(ptr[0]);
  OpenBSL_CommSendByte(ptr[1]);
  OpenBSL_CommSendByte(ptr[2]);
  OpenBSL_CommSendByte(ptr[3]);
#else
  OpenBSL_CommSendByte(ptr[3]);
  OpenBSL_CommSendByte(ptr[2]);
  OpenBSL_CommSendByte(ptr[1]);
  OpenBSL_CommSendByte(ptr[0]);
#endif
}


/**************************************************************************//**
*
* OpenBSL_CommSendLongChksum
*
* @brief     send a four bytes (32 bit) to communication line and update the
*            OpenBSL_OutChecksum checksum variable - blocking function
*
* @param[in]  val        input word (32 bit) to be sent out
*
* @return    -
*
******************************************************************************/
void OpenBSL_CommSendLongChksum(uint32_t val)
{
  uint8_t *ptr = (uint8_t*) &val;

  // calculate the checksum first
#ifdef LITTLE_ENDIAN
  OpenBSL_CommSendByteChksum(ptr[0]);
  OpenBSL_CommSendByteChksum(ptr[1]);
  OpenBSL_CommSendByteChksum(ptr[2]);
  OpenBSL_CommSendByteChksum(ptr[3]);
#else
  OpenBSL_CommSendByteChksum(ptr[3]);
  OpenBSL_CommSendByteChksum(ptr[2]);
  OpenBSL_CommSendByteChksum(ptr[1]);
  OpenBSL_CommSendByteChksum(ptr[0]);
#endif
}

//*****************************************************************************
// Internal functions
//*****************************************************************************
