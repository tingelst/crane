#ifndef __MLPIGLOBALHELPER_H__
#define __MLPIGLOBALHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiGlobalHelper.h>
// -----------------------------------------------------------------------
// Copyright (c) 2013 Bosch Rexroth. All rights reserved.
// Redistribution and use in source and binary forms of this MLPI software
// (SW) provided to you, with or without modification, are permitted
// without prior approval provided that the following conditions are met:
//
// 1. Redistributions of source code of SW must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form of SW must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. User recognizes and acknowledges that it acquires no right,
// title or interest in or to any of the names or trademarks used in
// connection with the SW ("names") by virtue of this License and waives
// any right to or interest in the names. User recognizes and acknowledges
// that names of companies or names or products of companies displayed
// in the documentation of SW to indicate the interoperability of products
// with the SW are the names of their respective owners. The use of such
// names in the documentation of SW does not imply any sponsorship,
// approval, or endorsement by such companies of this product.
//
// 4. Modified code versions, i.e. any addition to or deletion from
// the substance or structure of the original code of the SW running
// the MLPI must be plainly marked as such and must not be misrepresented
// as being original SW.
//
// 5. The SW may only be used in connection with a Bosch Rexroth product.
//
// THIS INFORMATION IS PROVIDED BY BOSCH REXROTH CORPORATION "AS IS"
// AND WITHOUT WARRANTY OF ANY KIND, EXPRESSED OR IMPLIED, INCLUDING
// (BUT NOTLIMITED TO) ANY IMPLIED WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR ANY PARTICULAR PURPOSE, OR NON-INFRINGEMENT. WHILE THE
// INFORMATION PROVIDED IS BELIEVED TO BE ACCURATE, IT MAY INCLUDE
// ERRORS OR INACCURACIES.
// SUBJECT TO COMPULSORY STATUTORY PROVISIONS OF THE GERMAN LAW AS
// THE APPLICABLE LAW FOR THIS LICENSE BOSCH REXROTH CORPORATION WILL
// NOT BE LIABLE FOR ANY DAMAGES OF ANY KIND ARISING FROM THE USE OF
// THE  SOFTWARE  DISTRIBUTED HEREUNDER, INCLUDING BUT NOT LIMITED TO
// DIRECT, INDIRECT, INCIDENTAL, PUNITIVE, AND CONSEQUENTIAL DAMAGES.
// -----------------------------------------------------------------------
//
//! @file
//!
//! @author     DC-IA/EAM1 (SK, JR)
//!
//! @copyright  Bosch Rexroth Corporation http://www.boschrexroth.com/oce
//!
//! @version    1.29.1
//!
//! @date       2013
//
// -----------------------------------------------------------------------



//! @addtogroup UtilGlobalHelper UtilGlobalHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for common handling.
//!
//! @details
//! Please note that this piece of source code is not directly part
//! of the MLPI. You do not need this file to program against the
//! MLPI. Nevertheless, at least parts of this file have been considered
//! to be somewhat useful when using or learning to use MLPI functionality.
//! It is therefore included without any support, but to act as sample code
//! and source of inspiration.
//! @}



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include <ctype.h>

#include "mlpiGlobal.h"
#include "wchar16.h"

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------

//! @ingroup UtilGlobalHelper
//! Function to print the memory dump of a container onto the screen. See @ref ContainerLib for further information.
//! @param[in]  data    Name of the container.
//! @param[in]  size    Size of the container.
inline void utilHexdump(const void *data, size_t size)
{
  unsigned char *ptr = (unsigned char*) data;
  size_t i, j;
  for (i=0; i<size; i+=16) {
    printf("%06x: ", i);
    for (j=0; j<16; j++) {
      if (i+j < size)
        printf("%02x ", (unsigned char) ptr[i+j]);
      else
        printf("   ");
    }
    printf(" ");
    for (j=0; j<16; j++) {
      if (i+j < size)
        printf("%u", isprint((unsigned char) ptr[i+j]) ? (unsigned char) ptr[i+j] : '.');
    }
    printf("\n");
  }
}

//! @ingroup UtilGlobalHelper
//! Function to determine the size of a MLPI data type.
//! @param[in]  type   MLPI data type whose size is to be determined
//! @return            The size of the assigned MLPI data type.
inline size_t utilSizeOfMlpiType(MlpiType type)
{
  switch (type)
  {
    default:
    case MLPI_TYPE_INVALID:           return 0;
    case MLPI_TYPE_CHAR:              return sizeof(CHAR);
    case MLPI_TYPE_UCHAR:             return sizeof(UCHAR);
    case MLPI_TYPE_SHORT:             return sizeof(SHORT);
    case MLPI_TYPE_USHORT:            return sizeof(USHORT);
    case MLPI_TYPE_LONG:              return sizeof(LONG);
    case MLPI_TYPE_ULONG:             return sizeof(ULONG);
    case MLPI_TYPE_LLONG:             return sizeof(LLONG);
    case MLPI_TYPE_ULLONG:            return sizeof(ULLONG);
    case MLPI_TYPE_FLOAT:             return sizeof(FLOAT);
    case MLPI_TYPE_DOUBLE:            return sizeof(DOUBLE);
    case MLPI_TYPE_BOOL8:             return sizeof(BOOL8);
    case MLPI_TYPE_CHAR_ARRAY:        return sizeof(CHAR);
    case MLPI_TYPE_UCHAR_ARRAY:       return sizeof(UCHAR);
    case MLPI_TYPE_SHORT_ARRAY:       return sizeof(SHORT);
    case MLPI_TYPE_USHORT_ARRAY:      return sizeof(USHORT);
    case MLPI_TYPE_LONG_ARRAY:        return sizeof(LONG);
    case MLPI_TYPE_ULONG_ARRAY:       return sizeof(ULONG);
    case MLPI_TYPE_LLONG_ARRAY:       return sizeof(LLONG);
    case MLPI_TYPE_ULLONG_ARRAY:      return sizeof(ULLONG);
    case MLPI_TYPE_FLOAT_ARRAY:       return sizeof(FLOAT);
    case MLPI_TYPE_DOUBLE_ARRAY:      return sizeof(DOUBLE);
    case MLPI_TYPE_BOOL8_ARRAY:       return sizeof(BOOL8);
    case MLPI_TYPE_CHAR_UTF8:         return sizeof(CHAR);
    case MLPI_TYPE_CHAR_UTF16:        return sizeof(WCHAR16);
  }
}


#endif /* __MLPIGLOBALHELPER_H__ */
