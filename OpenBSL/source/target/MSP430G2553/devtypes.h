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
* @file     devtypes.h
* 
* @brief    header file various variable type definitions
*
* @version  
* 
* @remark 
* 
******************************************************************************/

#ifndef _DEVTYPES_H_
#define _DEVTYPES_H_

//*****************************************************************************
// Include section
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Global variable declarations 
//*****************************************************************************


//*****************************************************************************
// Macros (defines) and data types 
//*****************************************************************************

/** TRUE
 *  true boolean value */
#ifndef TRUE
#define TRUE                               true
#endif

/** FALSE
 *  false boolean value */
#ifndef FALSE
#define FALSE                              false
#endif

/** NULL
 *  null pointer */
#ifndef NULL
#define NULL                               ((void*)0)
#endif

/* test part */

/** test part for int8_t size */
typedef char my_test_int8[((sizeof(int8_t)==1)? 1 : -1)];

/** test part for uint8_t size */
typedef char my_test_uint8[((sizeof(uint8_t)==1)? 1 : -1)];

/** test part for int16_t size */
typedef char my_test_int16[((sizeof(int16_t)==2)? 2 : -1)];

/** test part for uint16_t size */
typedef char my_test_uint16[((sizeof(uint16_t)==2)? 2 : -1)];

/** test part for int32_t size */
typedef char my_test_int32[((sizeof(int32_t)==4)? 4 : -1)];

/** test part for uint32_t size */
typedef char my_test_uint32[((sizeof(uint32_t)==4)? 4 : -1)];


/** LITTLE_ENDIAN
 *  defining this means that the device supports little endian format
 *  for storing multi bytes data
 */
#define LITTLE_ENDIAN

/** memAddr_t
 *  data type which represents best the memory space width of the device
 */
typedef unsigned int   memAddr_t;

//*****************************************************************************
// External function declarations
//*****************************************************************************

#endif /* _DEVTYPES_H_ */
