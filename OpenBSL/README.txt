/******************************************************************************
* README.txt 
*
* OpenBSL - Open BSL (bootstrap loader)
*
* version : 0.1.0
*
******************************************************************************/

OpenBSL is a simple, light-weight, open source Boot Strap Loader (BSL) for
small 8/16/32 bits microcontroller device family which is designed and
implemented as part in the main code memory.

The implementation of OpenBSL is inspired by various application notes from
Texas Instruments for MSP430 BSL (Boot Strap Loader).

At the moment, OpenBSL supports the following compiler/target device:
  - MSP430G2553 on MSP-EXP430G2 Launchpad (IDE: Code Composer Studio)

/******************************************************************************
* Documentation
******************************************************************************/

Go to <ROOT_DIRECTORY>/documents/html/index.html for the MSS documentation

/******************************************************************************
* Directory Structures 
******************************************************************************/

<ROOT_DIRECTORY>
  |
  |- documents : documentation stuffs
  |
  |- app_examples : application examples - at the moment only one example 
  |                 available for MSP430G2553
  |
  |- projects : project files - at the moment only one project files
  |             available for MSP430G2553
  |    
  |- Sources : OpenBSL codes 
      |
      |- core : core/common source codes of OpenBSL (hardware independent) 
      |
      |- target: hardware dependent OpenBSL source codes (at the moment only
                 supports MSP430G2553)

