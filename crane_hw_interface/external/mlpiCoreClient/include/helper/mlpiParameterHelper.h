#ifndef __MLPIPARAMETERHELPER_H__
#define __MLPIPARAMETERHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiParameterHelper.h>
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




//! @addtogroup UtilParameterHelper UtilParameterHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for parameter handling.
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
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "mlpiGlobal.h"
#include "wchar16.h"

#include "mlpiParameterLib.h"

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE   (128)

const CHAR      MLPI_VALUE_SEPARATOR_UTF8       = ' ';
const WCHAR16   MLPI_VALUE_SEPARATOR_UTF16      = (WCHAR16) MLPI_VALUE_SEPARATOR_UTF8;
const WCHAR16   MLPI_BOOLEAN_VALUE_TRUE[]       = { 'T', 'R', 'U', 'E', '\0' };
const WCHAR16   MLPI_BOOLEAN_VALUE_FALSE[]      = { 'F', 'A', 'L', 'S', 'E', '\0' };
const ULONG     MLPI_SIZEOF_BOOLEAN_VALUE_TRUE  = sizeof(MLPI_BOOLEAN_VALUE_TRUE);
const ULONG     MLPI_SIZEOF_BOOLEAN_VALUE_FALSE = sizeof(MLPI_BOOLEAN_VALUE_FALSE);


#define PARAM_MAX_NUMBER_PARAMETER                        (4096)
#define PARAM_LIST_ALL_C                                  MLPI_SIDN_C(110)
#define PARAM_LIST_ALL_A                                  MLPI_SIDN_A(10)
#define PARAM_LIST_ALL_K                                  MLPI_SIDN_K(10)
#define PARAM_LIST_ALL_O                                  MLPI_SIDN_O(10)
#define PARAM_LIST_ALL_M                                  MLPI_SIDN_M(10)
#define PARAM_LIST_ALL_N                                  MLPI_SIDN_N(10)
#define PARAM_LIST_ALL_SP                                 MLPI_SIDN_S(17)


// Sercos formated attribute
// +-------------+-----+---+---+---+---+----+----+----+----+----+----+
// |             | List|   |  D|  F|   | SLL| ULL|  SL|  UL|  SS|  US|
// +-------------+-----+---+---+---+---+----+----+----+----+----+----+
// 31           14     13 11  10   9   6    5    4    3    2    1    0


#define PARAM_ATTR_DATA_TYPE                     ( 0x00700000 )  // data type and display format mask (Bit 20-22)
#define PARAM_ATTR_DATA_TYPE_BINARY              ( 0x00000000 )
#define PARAM_ATTR_DATA_TYPE_UINT_DEC            ( 0x00100000 )
#define PARAM_ATTR_DATA_TYPE_INT_DEC             ( 0x00200000 )
#define PARAM_ATTR_DATA_TYPE_UINT_HEX            ( 0x00300000 )
#define PARAM_ATTR_DATA_TYPE_TEXT                ( 0x00400000 )
#define PARAM_ATTR_DATA_TYPE_UINT_IDN            ( 0x00500000 )
#define PARAM_ATTR_DATA_TYPE_FLOAT               ( 0x00600000 )
#define PARAM_ATTR_DATA_TYPE_MLC_IDN             ( 0x00700000 )

#define PARAM_ATTR_DATA_LENGTH                   ( 0x00070000 )  // data length mask (Bit 16-18)
#define PARAM_ATTR_DATA_LENGTH_1BYTE             ( 0x00000000 )
#define PARAM_ATTR_DATA_LENGTH_2BYTE             ( 0x00010000 )
#define PARAM_ATTR_DATA_LENGTH_4BYTE             ( 0x00020000 )
#define PARAM_ATTR_DATA_LENGTH_8BYTE             ( 0x00030000 )
#define PARAM_ATTR_DATA_LENGTH_1BYTE_VAR         ( 0x00040000 )
#define PARAM_ATTR_DATA_LENGTH_2BYTE_VAR         ( 0x00050000 )
#define PARAM_ATTR_DATA_LENGTH_4BYTE_VAR         ( 0x00060000 )
#define PARAM_ATTR_DATA_LENGTH_8BYTE_VAR         ( 0x00070000 )
#define PARAM_ATTR_DATA_LENGTH_LIST              ( 0x00040000 )
#define PARAM_ATTR_DATA_LENGTH_NO_LIST           ( 0x00030000 )

#define PARAM_ATTR_FUNCTION                      ( 0x00080000 )  // function mask (Bit 19)
#define PARAM_ATTR_FUNCTION_PARAMETER            ( 0x00000000 )  // parameter
#define PARAM_ATTR_FUNCTION_COMMAND              ( 0x00080000 )  // command

#define PARAM_ATTR_DECIMAL_POINT                 ( 0x0F000000 )  // decimal point mask (Bit 24-27)
#define PARAM_ATTR_DECIMAL_POINT_00PLACES        ( 0x00000000 )
#define PARAM_ATTR_DECIMAL_POINT_01PLACES        ( 0x01000000 )
#define PARAM_ATTR_DECIMAL_POINT_02PLACES        ( 0x02000000 )
#define PARAM_ATTR_DECIMAL_POINT_03PLACES        ( 0x03000000 )
#define PARAM_ATTR_DECIMAL_POINT_04PLACES        ( 0x04000000 )
#define PARAM_ATTR_DECIMAL_POINT_05PLACES        ( 0x05000000 )
#define PARAM_ATTR_DECIMAL_POINT_06PLACES        ( 0x06000000 )
#define PARAM_ATTR_DECIMAL_POINT_07PLACES        ( 0x07000000 )
#define PARAM_ATTR_DECIMAL_POINT_08PLACES        ( 0x08000000 )
#define PARAM_ATTR_DECIMAL_POINT_09PLACES        ( 0x09000000 )
#define PARAM_ATTR_DECIMAL_POINT_10PLACES        ( 0x0A000000 )
#define PARAM_ATTR_DECIMAL_POINT_11PLACES        ( 0x0B000000 )
#define PARAM_ATTR_DECIMAL_POINT_12PLACES        ( 0x0C000000 )
#define PARAM_ATTR_DECIMAL_POINT_13PLACES        ( 0x0D000000 )
#define PARAM_ATTR_DECIMAL_POINT_14PLACES        ( 0x0E000000 )
#define PARAM_ATTR_DECIMAL_POINT_15PLACES        ( 0x0F000000 )

#define PARAM_ATTR_WRITE_PROTECTION              ( 0x70000000 )  // write protection mask (Bit 28-30)
#define PARAM_ATTR_WRITE_PROTECTION_NONE         ( 0x00000000 )  // write protected none,          writeable P0..P4
#define PARAM_ATTR_WRITE_PROTECTION_P0_P1_P2     ( 0x10000000 )  // write protected in P0..P2,     writeable P3..P4
#define PARAM_ATTR_WRITE_PROTECTION_P3           ( 0x20000000 )  // write protected in P3,         writeable P0..2, P4 (do not use, use NONE instead!)
#define PARAM_ATTR_WRITE_PROTECTION_P0_P1_P2_P3  ( 0x30000000 )  // write protected in P0..P3,     writeable P4        (do not use, use P0_P1_P2 instead!)
#define PARAM_ATTR_WRITE_PROTECTION_P4           ( 0x40000000 )  // write protected in P4,         writeable P0..P3    (do not use, use P3_P4 instead!)
#define PARAM_ATTR_WRITE_PROTECTION_P0_P1_P2_P4  ( 0x50000000 )  // write protected in P0..P2, P4, writeable P3        (do not use, use READ_ONLY instead!)
#define PARAM_ATTR_WRITE_PROTECTION_P3_P4        ( 0x60000000 )  // write protected in P3..P4,     writeable P0..P2
#define PARAM_ATTR_WRITE_PROTECTION_READ_ONLY    ( 0x70000000 )  // write protected in P0..P4,     writeable none

#define PARAM_MAX_IDN_STRING_LENGTH              (         24 )  //!< Maximum length of parameter ident string
#define PARAM_SERCOS_TIME_STRING_LENGTH          (         31 )  //!< Maximum length of sercos time string

typedef enum ParamWriteProtection
{
  PARAM_WRITE_PROTECTION_NONE        = 0,
  PARAM_WRITE_PROTECTION_P0_P1_P2    = 1,
  PARAM_WRITE_PROTECTION_P3          = 2,
  PARAM_WRITE_PROTECTION_P0_P1_P2_P3 = 3,
  PARAM_WRITE_PROTECTION_P4          = 4,
  PARAM_WRITE_PROTECTION_P0_P1_P2_P4 = 5,
  PARAM_WRITE_PROTECTION_P3_P4       = 6,
  PARAM_WRITE_PROTECTION_READ_ONLY   = 7
}ParamWriteProtection;

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------
#define PARAM_ATTR_IS_TYPE_CHAR(a)                                           \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_INT_DEC) ) &&    \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_1BYTE) )

#define PARAM_ATTR_IS_TYPE_UCHAR(a)                                          \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_DEC) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_HEX) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_BINARY) ) &&     \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_1BYTE) )

#define PARAM_ATTR_IS_TYPE_SHORT(a)                                          \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_INT_DEC) ) &&    \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_2BYTE) )

#define PARAM_ATTR_IS_TYPE_USHORT(a)                                         \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_DEC) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_HEX) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_IDN) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_BINARY) ) &&     \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_2BYTE) )

#define PARAM_ATTR_IS_TYPE_LONG(a)                                           \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_INT_DEC) ) &&    \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_4BYTE) )

#define PARAM_ATTR_IS_TYPE_ULONG(a)                                          \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_DEC) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_HEX) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_IDN) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_MLC_IDN)  ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_BINARY) ) &&     \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_4BYTE) )

#define PARAM_ATTR_IS_TYPE_LLONG(a)                                          \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_INT_DEC) ) &&    \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_8BYTE) )

#define PARAM_ATTR_IS_TYPE_ULLONG(a)      \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_DEC) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_UINT_HEX) ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_MLC_IDN)  ||     \
      (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_BINARY) ) &&     \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_8BYTE) )

#define PARAM_ATTR_IS_TYPE_FLOAT(a)                                          \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_FLOAT) ) &&      \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_4BYTE) )

#define PARAM_ATTR_IS_TYPE_DOUBLE(a)                                         \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_FLOAT) ) &&      \
    (((a) & PARAM_ATTR_DATA_LENGTH_NO_LIST) == PARAM_ATTR_DATA_LENGTH_8BYTE) )

#define PARAM_ATTR_IS_TYPE_STRING(a)                                         \
  ( ( (((a) & PARAM_ATTR_DATA_TYPE) == PARAM_ATTR_DATA_TYPE_TEXT) )  &&      \
    (((a) & PARAM_ATTR_DATA_LENGTH) == PARAM_ATTR_DATA_LENGTH_1BYTE_VAR)     )

#define PARAM_ATTR_IS_FIXED_LENGTH(a)                                        \
  ( (((a) & PARAM_ATTR_DATA_LENGTH) == PARAM_ATTR_DATA_LENGTH_1BYTE) ||      \
    (((a) & PARAM_ATTR_DATA_LENGTH) == PARAM_ATTR_DATA_LENGTH_2BYTE) ||      \
    (((a) & PARAM_ATTR_DATA_LENGTH) == PARAM_ATTR_DATA_LENGTH_4BYTE) ||      \
    (((a) & PARAM_ATTR_DATA_LENGTH) == PARAM_ATTR_DATA_LENGTH_8BYTE)         )

#define PARAM_ATTR_IS_VARIABLE_LENGTH(a)    ( !PARAM_ATTR_IS_FIXED_LENGTH(a) )



// -----------------------------------------------------------------------
// GLOBAL FUNCTIONS
// -----------------------------------------------------------------------
inline MLPIRESULT utilParameterCheckAttributeSigned(ULONG attribute, ULONG errorCode)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;
  switch(attribute & PARAM_ATTR_DATA_TYPE)
  {
    case PARAM_ATTR_DATA_TYPE_INT_DEC:  mlpiResult = MLPI_S_OK; break;
    default:                            mlpiResult = errorCode; break;
  }
  return mlpiResult;
}

inline MLPIRESULT utilParameterCheckAttributeUnsigned(ULONG attribute, ULONG errorCode)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;
  switch(attribute & PARAM_ATTR_DATA_TYPE)
  {
    case PARAM_ATTR_DATA_TYPE_BINARY:   mlpiResult = MLPI_S_OK; break;
    case PARAM_ATTR_DATA_TYPE_UINT_DEC: mlpiResult = MLPI_S_OK; break;
    case PARAM_ATTR_DATA_TYPE_UINT_HEX: mlpiResult = MLPI_S_OK; break;
    case PARAM_ATTR_DATA_TYPE_UINT_IDN: mlpiResult = MLPI_S_OK; break;
    case PARAM_ATTR_DATA_TYPE_MLC_IDN:  mlpiResult = MLPI_S_OK; break;
    default:                            mlpiResult = errorCode; break;
  }
  return mlpiResult;
}

inline MLPIRESULT utilParameterCheckAttributeFloat(ULONG attribute, ULONG errorCode)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;
  switch(attribute & PARAM_ATTR_DATA_TYPE)
  {
    case PARAM_ATTR_DATA_TYPE_FLOAT:    mlpiResult = MLPI_S_OK; break;
    default:                            mlpiResult = errorCode; break;
  }
  return mlpiResult;
}

inline MLPIRESULT utilParameterCheckAttributeText(ULONG attribute, ULONG errorCode)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;
  switch(attribute & PARAM_ATTR_DATA_TYPE)
  {
    case PARAM_ATTR_DATA_TYPE_TEXT:     mlpiResult = MLPI_S_OK; break;
    default:                            mlpiResult = errorCode; break;
  }
  return mlpiResult;
}

//! @ingroup UtilParameterHelper
//! This Function extracts the data type which is coded in Sercos attribute and
//! translates it the an MLPI data type of the enum @ref MlpiType.
//! @param[in]    attribute   Sercos attribute
//! @return                   If succeeded, returns the type.
//!                           Otherwise MLPI_TYPE_UNKNOWN is returned.
//!
//! In most cases, it's too difficult to know the correct data type for a
//! parameter request. This function can help.
//! It takes the original attribute of a parameter and converts
//! it to a type you can use for a parameter request.
//!
//! @par Example:
//! @code
//! ULONG sercosAttribute;
//! ULONG readCount;
//!
//! // read original sercos attribute
//! mlpiParameterReadAttribute(connection, 1, MLPI_SIDN_O(4), &sercosAttribute);
//!
//! // extract MLPI data type information
//! mlpiType dataType = utilParameterGetDataTypeFromAttribute(sercosAttribute)
//!
//! // read data using the correct function
//! switch(dataType)
//! {
//!   case MLPI_TYPE_CHAR_UTF8:
//!   case MLPI_TYPE_CHAR_UTF16:
//!     mlpiParameterReadDataString(connection, 1, MLPI_SIDN_O(4), strValue, _countof(strValue), &readCount);
//!     break;
//!   case MLPI_TYPE_UCHAR:
//!     mlpiParameterReadDataUchar(connection, 1, MLPI_SIDN_O(4), &cValue);
//!     break;
//!   case MLPI_TYPE_USHORT:
//!     mlpiParameterReadDataUshort(connection, 1, MLPI_SIDN_O(4), &usValue);
//!     break;
//!   case MLPI_TYPE_SHORT:
//!     mlpiParameterReadDataShort(connection, 1, MLPI_SIDN_O(4), &sValue);
//!     break;
//!   case MLPI_TYPE_ULONG:
//!     mlpiParameterReadDataUlong(connection, 1, MLPI_SIDN_O(4), &ulValue);
//!     break;
//!   case MLPI_TYPE_LONG:
//!     mlpiParameterReadDataLong(connection, 1, MLPI_SIDN_O(4), &lValue);
//!     break;
//!   case MLPI_TYPE_ULLONG:
//!     mlpiParameterReadDataUllong(connection, 1, MLPI_SIDN_O(4), &ullValue);
//!     break;
//!   case MLPI_TYPE_LLONG:
//!     mlpiParameterReadDataLlong(connection, 1, MLPI_SIDN_O(4), &llValue);
//!     break;
//!   case MLPI_TYPE_FLOAT:
//!     mlpiParameterReadDataFloat(connection, 1, MLPI_SIDN_O(4), &fValue);
//!     break;
//!   case MLPI_TYPE_DOUBLE:
//!     mlpiParameterReadDataDouble(connection, 1, MLPI_SIDN_O(4), &dValue);
//!     break;
//!   case MLPI_TYPE_UCHAR_ARRAY:
//!     mlpiParameterReadDataUcharArray(connection, 1, MLPI_SIDN_O(4), acValue, _countof(acValue), &readCount);
//!     break;
//!   case MLPI_TYPE_USHORT_ARRAY:
//!     mlpiParameterReadDataUshortArray(connection, 1, MLPI_SIDN_O(4), ausValue, _countof(ausValue), &readCount);
//!     break;
//!   case MLPI_TYPE_SHORT_ARRAY:
//!     mlpiParameterReadDataShortArray(connection, 1, MLPI_SIDN_O(4), asValue, _countof(asValue), &readCount);
//!     break;
//!   case MLPI_TYPE_ULONG_ARRAY:
//!     mlpiParameterReadDataUlongArray(connection, 1, MLPI_SIDN_O(4), aulValue, _countof(aulValue), &readCount);
//!     break;
//!   case MLPI_TYPE_LONG_ARRAY:
//!     mlpiParameterReadDataLongArray(connection, 1, MLPI_SIDN_O(4), alValue, _countof(alValue), &readCount);
//!     break;
//!   case MLPI_TYPE_ULLONG_ARRAY:
//!     mlpiParameterReadDataUllongArray(connection, 1, MLPI_SIDN_O(4), aullValue, _countof(aullValue), &readCount);
//!     break;
//!   case MLPI_TYPE_LLONG_ARRAY:
//!     mlpiParameterReadDataLlongArray(connection, 1, MLPI_SIDN_O(4), allValue, _countof(allValue), &readCount);
//!     break;
//!   case MLPI_TYPE_FLOAT_ARRAY:
//!     mlpiParameterReadDataFloatArray(connection, 1, MLPI_SIDN_O(4), afValue, _countof(afValue), &readCount);
//!     break;
//!   case MLPI_TYPE_DOUBLE_ARRAY:
//!     mlpiParameterReadDataDoubleArray(connection, 1, MLPI_SIDN_O(4), adValue, _countof(adValue), &readCount);
//!     break;
//!   default:
//!     // unknown or invalid data type...
//!     break;
//! }
//! @endcode
inline MlpiType utilParameterGetDataTypeFromAttribute(const ULONG attribute)
{
  MlpiType type = MLPI_TYPE_INVALID;

  if (PARAM_ATTR_IS_VARIABLE_LENGTH(attribute))
  {
    if (PARAM_ATTR_IS_TYPE_UCHAR(attribute))
      type = MLPI_TYPE_UCHAR_ARRAY;
    if (PARAM_ATTR_IS_TYPE_USHORT(attribute))
      type = MLPI_TYPE_USHORT_ARRAY;
    if (PARAM_ATTR_IS_TYPE_SHORT(attribute))
      type = MLPI_TYPE_SHORT_ARRAY;
    if (PARAM_ATTR_IS_TYPE_ULONG(attribute))
      type = MLPI_TYPE_ULONG_ARRAY;
    if (PARAM_ATTR_IS_TYPE_LONG(attribute))
      type = MLPI_TYPE_LONG_ARRAY;
    if (PARAM_ATTR_IS_TYPE_ULLONG(attribute))
      type = MLPI_TYPE_ULLONG_ARRAY;
    if (PARAM_ATTR_IS_TYPE_LLONG(attribute))
      type = MLPI_TYPE_LLONG_ARRAY;
    if (PARAM_ATTR_IS_TYPE_FLOAT(attribute))
      type = MLPI_TYPE_FLOAT_ARRAY;
    if (PARAM_ATTR_IS_TYPE_DOUBLE(attribute))
      type = MLPI_TYPE_DOUBLE_ARRAY;
    if (PARAM_ATTR_IS_TYPE_STRING(attribute))
      type = MLPI_TYPE_CHAR_UTF16;
  }
  else
  {
    if (PARAM_ATTR_IS_TYPE_UCHAR(attribute))
      type = MLPI_TYPE_UCHAR;
    if (PARAM_ATTR_IS_TYPE_CHAR(attribute))
      type = MLPI_TYPE_CHAR;
    if (PARAM_ATTR_IS_TYPE_USHORT(attribute))
      type = MLPI_TYPE_USHORT;
    if (PARAM_ATTR_IS_TYPE_SHORT(attribute))
      type = MLPI_TYPE_SHORT;
    if (PARAM_ATTR_IS_TYPE_ULONG(attribute))
      type = MLPI_TYPE_ULONG;
    if (PARAM_ATTR_IS_TYPE_LONG(attribute))
      type = MLPI_TYPE_LONG;
    if (PARAM_ATTR_IS_TYPE_ULLONG(attribute))
      type = MLPI_TYPE_ULLONG;
    if (PARAM_ATTR_IS_TYPE_LLONG(attribute))
      type = MLPI_TYPE_LLONG;
    if (PARAM_ATTR_IS_TYPE_FLOAT(attribute))
      type = MLPI_TYPE_FLOAT;
    if (PARAM_ATTR_IS_TYPE_DOUBLE(attribute))
      type = MLPI_TYPE_DOUBLE;
    if (PARAM_ATTR_IS_TYPE_STRING(attribute))
      type = MLPI_TYPE_CHAR_UTF16;
  }
  return type;
}

#define PARAM_ADD_ELEMENT_TO_SIDN(sidn, element) ( (ULLONG)(  \
  ((ULLONG)sidn) |                                            \
  ((ULLONG)(element & (SZ_PSP_ELEMENT-1))  << 40) ))

//! @ingroup UtilParameterHelper
//! This Function extracts the 'procedure command' property which is coded in Sercos attribute.
//! @param[in]    attribute   Sercos attribute.
//! @return                   0: Parameter is not a 'procedure command', 1: Parameter is a 'procedure command'.
inline BOOL8 utilParameterGetProcedureCmdFromAttribute(const ULONG attribute)
{
  BOOL8 cmd = false;
  switch (attribute & PARAM_ATTR_FUNCTION)
  {
    case PARAM_ATTR_FUNCTION_PARAMETER:   cmd = false;  break;
    case PARAM_ATTR_FUNCTION_COMMAND:     cmd = true;   break;
  }
  return cmd;
}

//! @ingroup UtilParameterHelper
//! This Function extracts the places after decimal point which is coded in Sercos attribute.
//! @param[in]    attribute   Sercos attribute.
//! @return                   Places after decimal point.
inline SHORT utilParameterGetDecimalPointFromAttribute(const ULONG attribute)
{
  SHORT decimalPoint = 0;
  switch (attribute & PARAM_ATTR_DECIMAL_POINT)
  {
    case PARAM_ATTR_DECIMAL_POINT_00PLACES: decimalPoint =  0; break;
    case PARAM_ATTR_DECIMAL_POINT_01PLACES: decimalPoint =  1; break;
    case PARAM_ATTR_DECIMAL_POINT_02PLACES: decimalPoint =  2; break;
    case PARAM_ATTR_DECIMAL_POINT_03PLACES: decimalPoint =  3; break;
    case PARAM_ATTR_DECIMAL_POINT_04PLACES: decimalPoint =  4; break;
    case PARAM_ATTR_DECIMAL_POINT_05PLACES: decimalPoint =  5; break;
    case PARAM_ATTR_DECIMAL_POINT_06PLACES: decimalPoint =  6; break;
    case PARAM_ATTR_DECIMAL_POINT_07PLACES: decimalPoint =  7; break;
    case PARAM_ATTR_DECIMAL_POINT_08PLACES: decimalPoint =  8; break;
    case PARAM_ATTR_DECIMAL_POINT_09PLACES: decimalPoint =  9; break;
    case PARAM_ATTR_DECIMAL_POINT_10PLACES: decimalPoint = 10; break;
    case PARAM_ATTR_DECIMAL_POINT_11PLACES: decimalPoint = 11; break;
    case PARAM_ATTR_DECIMAL_POINT_12PLACES: decimalPoint = 12; break;
    case PARAM_ATTR_DECIMAL_POINT_13PLACES: decimalPoint = 13; break;
    case PARAM_ATTR_DECIMAL_POINT_14PLACES: decimalPoint = 14; break;
    case PARAM_ATTR_DECIMAL_POINT_15PLACES: decimalPoint = 15; break;
  }
  return decimalPoint;
}

//! @ingroup UtilParameterHelper
//! This Function extracts the write protection of a parameter in certain states which is coded in Sercos attribute.
//! @param[in]    attribute   Sercos attribute.
//! @return                   Returns the state in which a parameter is write protected.
inline ParamWriteProtection utilParameterGetWriteProtectionFromAttribute(const ULONG attribute)
{
  ParamWriteProtection writeProtection = (ParamWriteProtection)PARAM_ATTR_WRITE_PROTECTION_READ_ONLY;
  switch (attribute & PARAM_ATTR_WRITE_PROTECTION)
  {
  case PARAM_ATTR_WRITE_PROTECTION_NONE:        writeProtection = PARAM_WRITE_PROTECTION_NONE;        break;
  case PARAM_ATTR_WRITE_PROTECTION_P0_P1_P2:    writeProtection = PARAM_WRITE_PROTECTION_P0_P1_P2;    break;
  case PARAM_ATTR_WRITE_PROTECTION_P3:          writeProtection = PARAM_WRITE_PROTECTION_P3;          break;
  case PARAM_ATTR_WRITE_PROTECTION_P0_P1_P2_P3: writeProtection = PARAM_WRITE_PROTECTION_P0_P1_P2_P3; break;
  case PARAM_ATTR_WRITE_PROTECTION_P4:          writeProtection = PARAM_WRITE_PROTECTION_P4;          break;
  case PARAM_ATTR_WRITE_PROTECTION_P0_P1_P2_P4: writeProtection = PARAM_WRITE_PROTECTION_P0_P1_P2_P4; break;
  case PARAM_ATTR_WRITE_PROTECTION_P3_P4:       writeProtection = PARAM_WRITE_PROTECTION_P3_P4;       break;
  case PARAM_ATTR_WRITE_PROTECTION_READ_ONLY:   writeProtection = PARAM_WRITE_PROTECTION_READ_ONLY;   break;
  }
  return writeProtection;
}

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------



//! @ingroup UtilParameterHelper
//! This function parses a Sercos IDN string and returns its properties. These can be used to build a binary
//! expression of the Sercos IDN.
//! @param[in]  idn       String representation of a Sercos parameter IDN. For example "S-0-0051", "S-0-1050.0.1" or "P-100".
//! @param[out] address   Pointer to a variable which receives the address.
//! @param[out] type      Pointer to a variable which receives the type.
//! @param[out] set       Pointer to a variable which receives the set element.
//! @param[out] block     Pointer to a variable which receives the block.
//! @param[out] si        Pointer to a variable which receives the si element.
//! @param[out] se        Pointer to a variable which receives the se element.
//! @param[out] idnNxt    Pointer to the first character after parsed Sercos IDN string.
//! @return               Return value indicating success (>=0) or error (<0).
inline MLPIRESULT utilParameterParseIdn(const WCHAR16 *idn, ULONG *address, ULLONG *type, ULLONG *set, ULLONG *block, ULLONG *si, ULLONG *se, WCHAR16 **idnNxt=NULL)
{
  ULONG _address=0;
  ULLONG _type=0, _set=0, _block=0, _si=0, _se=0;
  ULONG i=0;

  // parse index/address if available
  if (wcschr16(&idn[i], L':')!=NULL)
  {
    while( !(((idn[i]>='0') && (idn[i]<='9')) || (idn[i]==L':')) )
      i++;

    _address = wtoi16(&idn[i]);

    while( !(idn[i]==L':') )
      i++;

    i++;
  }

  // parse type, this information is always necessary
  switch (idn[i])
  {
  case 'A':
  case 'a':
    _type = MLPI_SIDN_TYPE_AXIS;
    break;
  case 'C':
  case 'c':
    _type = MLPI_SIDN_TYPE_CONTROL;
    break;
  case 'K':
  case 'k':
    _type = MLPI_SIDN_TYPE_KINEMATIC;
    break;
  case 'M':
  case 'm':
    _type = MLPI_SIDN_TYPE_PROBE;
    break;
  case 'N':
  case 'n':
    _type = MLPI_SIDN_TYPE_POSLIMSWITCH;
    break;
  case 'O':
  case 'o':
    _type = MLPI_SIDN_TYPE_OSCILLOSCOPE;
    break;
  case 'P':
  case 'p':
    _type = MLPI_SIDN_TYPE_DRIVE_P;
    break;
  case 'S':
  case 's':
    _type = MLPI_SIDN_TYPE_DRIVE_S;
    break;
  default:
    return MLPI_E_INVALIDARG;
  }
  i++;

  // parse block for special case, when only a caracter and number is given
  if ( (wcslen16(&idn[i])>=1) && (idn[i+0]!='-') && (idn[i+0]>='0') && (idn[i+0]<='9') )
  {
    _block = wtoi16(&idn[i+0]);
    i++;
    while( (idn[i]>='0') && (idn[i]<='9') )
      i++;
  }

  // parse set, this info is optional
  if ( (wcslen16(&idn[i])>=3) && (idn[i+0]=='-') && (idn[i+1]>='0') && (idn[i+1]<='7') && (idn[i+2]=='-') )
  {
    _set = idn[i+1] - '0';
    i+=2;
  }

  // parse block, this info is optional
  if ( (wcslen16(&idn[i])>=2) && (idn[i+0]=='-') && (idn[i+1]>='0') && (idn[i+1]<='9') )
  {
    _block = wtoi16(&idn[i+1]);
    i++;
    while( (idn[i]>='0') && (idn[i]<='9') )
      i++;
  }

  // parse si, this info is optional
  if ( (wcslen16(&idn[i])>=2) && (idn[i+0]=='.') && (idn[i+1]>='0') && (idn[i+1]<='9') )
  {
    _si = wtoi16(&idn[i+1]);
    i++;
    while( (idn[i]>='0') && (idn[i]<='9') )
      i++;
  }

  // parse se, this info is optional
  if ( (wcslen16(&idn[i])>=2) && (idn[i+0]=='.') && (idn[i+1]>='0') && (idn[i+1]<='9') )
  {
    _se = wtoi16(&idn[i+1]);
    i++;
    while( (idn[i]>='0') && (idn[i]<='9') )
      i++;
  }

  // return the values
  if (type)
    *type     = _type;
  if (set)
    *set      = _set;
  if (block)
    *block    = _block;
  if (si)
    *si       = _si;
  if (se)
    *se       = _se;
  if (address)
    *address   = _address;

  // Remove separator spaces
  while(idn[i]==MLPI_VALUE_SEPARATOR_UTF16)
    i++;

  if (idnNxt)
    *idnNxt     = const_cast<WCHAR16*>(&idn[i]);

  return MLPI_S_OK;
}


//! @ingroup UtilParameterHelper
//! This function parses a Sercos IDN string and returns its binary expression.
//! @param[in]  idnString   String representation of a Sercos parameter IDN. For example, "S-0-0051", "S-0-1050.0.1" or "P-100".
//! @param[out] idnValue    Pointer to a variable which receives the binary expression of the given string. This can be used
//!                         for subsequent calls to ParameterRead functions.
//! @param[out] address     Pointer to a variable which receives the address.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULLONG ullIdn;
//! utilParameterParseIdn(L"C-0-0012", &ullIdn);
//! mlpiParameterReadData(1, ullIdn, szFirmware, _countof(szFirmware), 0);
//! @endcode
inline MLPIRESULT utilParameterParseIdn(const WCHAR16 *idnString, ULLONG *idnValue, ULONG *address = NULL)
{
  ULLONG _type=0, _set=0, _block=0, _si=0, _se=0;
  ULONG _address=0;

  // parse the string
  MLPIRESULT result = utilParameterParseIdn(idnString, &_address, &_type, &_set, &_block, &_si, &_se, NULL);

  // build the IDN
  if (idnValue)
    *idnValue = MLPI_SIDN(_type, _set, _block, _si, _se);

  // address in not part of the IDN
  if (address)
    *address = _address;

  return result;
}


//! @ingroup UtilParameterHelper
//! Some SIDNs contain the address field in the upper byte of the reserved space. Use this function
//! to cut out the address from the rest of the SIDN. See for example parameters like A-0-0002 or O-0-0003.
//! @param[in]  sidn      Sercos IDN, in format as from O-0-0003.
//! @param[out] idn       Pointer to a variable which receives the binary expression of the given SIDN. This can be used
//!                       for subsequent calls to ParameterRead functions.
//! @param[out] address   Pointer to a variable which receives the address.
//! @return               Return value indicating success (>=0) or error (<0).
inline MLPIRESULT utilParameterSplitSidn(const ULLONG sidn, ULLONG* idn, ULONG* address)
{
  ULLONG _idn = 0;
  ULONG  _address = 0;
  const ULLONG _mask =  MLPI_SIDN_TYPE_MASK                      |
                        ((ULLONG)(MLPI_SIDN_BLOCK_MASK))   <<  0 |
                        ((ULLONG)(MLPI_SIDN_SET_MASK))     << 12 |
                        ((ULLONG)(MLPI_SIDN_SE_MASK))      << 16 |
                        ((ULLONG)(MLPI_SIDN_SI_MASK))      << 24 ;

  _idn = sidn & _mask;
  _address = (ULONG) ((sidn & (~_mask)) >> 56);

  if(address)
    *address = _address;

  if(idn)
    *idn = _idn;

  return MLPI_S_OK;
}


//! @ingroup UtilParameterHelper
//! This function splits a sercos SIDN and returns its properties.
//! Some SIDNs contain the address field in the upper byte of the reserved space (see for example parameters
//! such as A-0-0002 or O-0-0003). Use this function to cut out the address from the rest of the
//! SIDN as well as all other elements within a SIDN.
//! @param[in]  sidn     SIDN which should be split.
//! @param[out] address  Pointer to a variable which receives the address.
//! @param[out] type     Pointer to a variable which receives the type.
//! @param[out] block    Pointer to a variable which receives the block.
//! @param[out] spFlag   Pointer to a variable which receives the S or P flag.
//! @param[out] set      Pointer to a variable which receives the set element.
//! @param[out] si       Pointer to a variable which receives the si element.
//! @param[out] se       Pointer to a variable which receives the se element.
//! @return              Return value indicating success (>=0) or error (<0).
inline MLPIRESULT utilParameterSplitSidn(const ULLONG sidn, ULONG *address, ULONG *type, ULONG *block, ULONG *spFlag, ULONG *set, ULONG *si, ULONG *se)
{
  typedef struct
  {
    ULONG block    : 12;
    ULONG set      :  3;
    ULONG flag     :  1;
    ULONG se       :  8;
    ULONG si       :  8;
    ULONG type     :  8;
    ULONG reserved : 16;
    ULONG address  :  8;
  }R_SIDN;

  R_SIDN *_sidn = (R_SIDN*) &sidn;

  if(address)
    *address = _sidn->address;
  if(type)
    *type = _sidn->type;
  if(block)
    *block = _sidn->block;
  if(spFlag)
    *spFlag = _sidn->flag;
  if(set)
    *set = _sidn->set;
  if(si)
    *si = _sidn->si;
  if(se)
    *se = _sidn->se;

  return MLPI_S_OK;
}


//! @ingroup UtilParameterHelper
//! This function parses a address and returns a sercos IDN string.
//! @param[in]  address       64 Bit value of an EIDN.
//! @param[out] idnString     Pointer to a variable which receives the string. Use define @ref PARAM_MAX_IDN_STRING_LENGTH for creation.
//! @param[in]  numElements   Number of WCHAR16 elements in 'idnString' available to read.
//! @return                   Pointer to 'idnString'.
//!
//! @par Example:
//! @code
//! WCHAR16 idnString[PARAM_MAX_IDN_STRING_LENGTH] = {L'0'};
//! printf("\nParameter %s", utilParameterParseIdn(MLPI_SIDN_C(400), idnString, _countof(idnString)));
//! @endcode
inline WCHAR16* utilParameterParseIdn(const ULLONG address, WCHAR16 *idnString, const ULONG numElements )
{
  WCHAR16 _idn[PARAM_MAX_IDN_STRING_LENGTH] = {L'0'};
  CHAR    _helper[PARAM_MAX_IDN_STRING_LENGTH]  = {'0'};
  ULONG _address=0, _type=0, _block=0, _spFlag=0, _set=0, _si=0, _se=0, i=0;
  utilParameterSplitSidn(address, &_address, &_type, &_block, &_spFlag, &_set, &_si, &_se);

  switch(_type)
  {
    case (ULONG)MLPI_SIDN_TYPE_DRIVE_S:
    {
      if(_spFlag)
        wcscpy16(_idn, L"P");
      else
        wcscpy16(_idn, L"S");
      break;
    }
    case (ULONG)(MLPI_SIDN_TYPE_AXIS>>32):
    {
      wcscpy16(_idn, L"A");
      break;
    }
    case (ULONG)(MLPI_SIDN_TYPE_CONTROL>>32):
    {
      wcscpy16(_idn, L"C");
      break;
    }
    case (ULONG)(MLPI_SIDN_TYPE_KINEMATIC>>32):
    {
      wcscpy16(_idn, L"K");
      break;
    }
    case (ULONG)(MLPI_SIDN_TYPE_PROBE>>32):
    {
      wcscpy16(_idn, L"M");
      break;
    }
    case (ULONG)(MLPI_SIDN_TYPE_POSLIMSWITCH>>32):
    {
      wcscpy16(_idn, L"N");
      break;
    }
    case (ULONG)(MLPI_SIDN_TYPE_OSCILLOSCOPE>>32):
    {
      wcscpy16(_idn, L"O");
      break;
    }
    default:
      wcscpy16(_idn, L"?");
      break;
  }
  i++;
  wcscat16(_idn,L"-");
  i++;
  sprintf(_helper, "%01u", (UCHAR)((_set>0x7)?0:_set));
  i += (ULONG) mbstowcs16(&_idn[i], _helper, strlen(_helper));
  memset(_helper, 0, PARAM_MAX_IDN_STRING_LENGTH);
  wcscat16(_idn,L"-");
  i++;
  sprintf(_helper, "%04u", (USHORT)((_block>0xFFF)?0:_block));
  i += (ULONG) mbstowcs16(&_idn[i], _helper, strlen(_helper));
  memset(_helper, 0, PARAM_MAX_IDN_STRING_LENGTH);
  if(_si)
  {
    wcscat16(_idn,L".");
    i++;
    sprintf(_helper, "%03u", (UCHAR)((_si>0xFF)?0:_si));
    i += (ULONG) mbstowcs16(&_idn[i], _helper, strlen(_helper));
    memset(_helper, 0, PARAM_MAX_IDN_STRING_LENGTH);
    if(_se)
    {
      wcscat16(_idn,L".");
      i++;
      sprintf(_helper, "%03u", (UCHAR)((_se>0xFF)?0:_se));
      i += (ULONG) mbstowcs16(&_idn[i], _helper, strlen(_helper));
      memset(_helper, 0, PARAM_MAX_IDN_STRING_LENGTH);
    }
  }

  if(numElements >= wcslen16(_idn))
  {
    wcscpy16(idnString, _idn);
  }

  return idnString;
}


//! @ingroup UtilParameterHelper
//! This function parse a 64-bit sercos time value and returns a sercos time string (CHAR).
//! Structure of sercos time    Bit 63-32: seconds since 1970-01-01, 00:00:00
//!                             Nit 31-00: nanoseconds up to 999.999.999
//! @param[in]  timeStamp       64-bit sercos time value.
//! @param[out] time            Pointer to a variable which receives the sercos time string (ISO-8601). Use define @ref PARAM_SERCOS_TIME_STRING_LENGTH for creation.
//! @param[in]  numElements     Number of CHAR elements in 'time' available to write.
//! @param[in]  numElementsRet  Number of CHAR elements in complete 'time'.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULLONG timeStamp = 0x50E227003B9AC9FF;
//! CHAR time[PARAM_SERCOS_TIME_STRING_LENGTH] = "";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = utilParameterParseSercosTimeToString(timeStamp, time, _countof(time), &numElementsRet);
//! @endcode
inline MLPIRESULT utilParameterParseSercosTimeToString(const ULLONG timeStamp, CHAR *time, const ULONG numElements, ULONG *numElementsRet=NULL)
{
  MLPIRESULT mlpiResult=MLPI_S_OK;

  ULONG len = 0;

#if defined (TARGET_OS_WINNT) || defined (TARGET_OS_LINUX)
  ULLONG seconds = (ULONG) (timeStamp >> 32);
#else
  ULONG seconds = (ULONG) (timeStamp >> 32);
#endif

  ULONG nanoseconds = (ULONG) (timeStamp);

  if(time==NULL) {
    len = 0;
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(numElements<PARAM_SERCOS_TIME_STRING_LENGTH) {
    len = 1;
    *time = '\0';
    mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  else
  {
    // MAX: 2106-02-07T06:28:15.999999999Z == 0xFFFFFFFF, subtract one second because of return value '-1' of mktime() in case of error
    if (seconds==0xFFFFFFFF)
      seconds = 0xFFFFFFFE;

    if(nanoseconds>999999999) {
      nanoseconds = 999999999;
      mlpiResult = MLPI_E_LIMIT_MAX;
    }
#if defined(TARGET_OS_WINCE32)
#pragma message ("not supported on TARGET_OS_WINCE32")
      mlpiResult = MLPI_E_UNIMPLEMENTED;
#elif defined(TARGET_OS_WINNT)
    tm timeBuffer;
    memset(&timeBuffer, 0, sizeof(timeBuffer));

    gmtime_s(&timeBuffer, (time_t*) &seconds);

    len = (ULONG) strftime(time, numElements, "%Y-%m-%dT%H:%M:%S", &timeBuffer);
    len += sprintf(time+len, ".%09uZ", static_cast<unsigned int>(nanoseconds));
    len++;
#else
    tm timeBuffer;
    memset(&timeBuffer, 0, sizeof(timeBuffer));

    gmtime_r((time_t*) &seconds, &timeBuffer);

    len = (ULONG) strftime(time, numElements, "%Y-%m-%dT%H:%M:%S", &timeBuffer);
    len += sprintf(time+len, ".%09uZ", static_cast<unsigned int>(nanoseconds));
    len++;
#endif
  }

  if(numElementsRet != NULL)
    *numElementsRet = len;

  return mlpiResult;
}


//! @ingroup UtilParameterHelper
//! This function parse a 64-bit sercos time value and returns a sercos time string (CHAR).
//! Structure of sercos time    Bit 63-32: seconds since 1970-01-01, 00:00:00
//!                             Nit 31-00: nanoseconds up to 999.999.999
//! @param[in]  timeStamp       64-bit sercos time value.
//! @param[out] time            Pointer to a variable which receives the sercos time string (ISO-8601). Use define @ref PARAM_SERCOS_TIME_STRING_LENGTH for creation.
//! @param[in]  numElements     Number of WCHAR16 elements in 'time' available to write.
//! @param[in]  numElementsRet  Number of WCHAR16 elements in complete 'time'.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULLONG timeStamp = 0x50E227003B9AC9FF;
//! CHAR time[PARAM_SERCOS_TIME_STRING_LENGTH] = "";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = utilParameterParseSercosTimeToString(timeStamp, time, _countof(time), &numElementsRet);
//! @endcode
inline MLPIRESULT utilParameterParseSercosTimeToString(const ULLONG timeStamp, WCHAR16 *time, const ULONG numElements, ULONG *numElementsRet=NULL )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  ULONG len = 0;
  CHAR localTime[PARAM_SERCOS_TIME_STRING_LENGTH] = "";

  mlpiResult = utilParameterParseSercosTimeToString(timeStamp, localTime, PARAM_SERCOS_TIME_STRING_LENGTH, &len);

  len = (ULONG) mbstowcs16(time, localTime, len);

  if(numElementsRet != NULL)
    *numElementsRet = len;

  return mlpiResult;
}


//! @ingroup UtilParameterHelper
//! This function parse a sercos time string (CHAR) and returns a 64-bit sercos time value.
//! Structure of sercos time    Bit 63-32: seconds since 1970-01-01, 00:00:00
//!                             Nit 31-00: nanoseconds up to 999.999.999
//! @param[in]   time           Pointer to a variable which contains at least one sercos time string (ISO-8601).
//! @param[in]   separator      Separator between two or more sercos time strings. Use space character at default.
//! @param[out]  timeStamp      64-bit sercos time value.
//! @param[out]  timeNext       Pointer next sercos time string.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! CHAR time[] = L"2000-01-01T23:55:00.123456789Z 2100-12-24T20:00:00.999999999Z 2007-08-14T12:00:00Z";
//! CHAR separator = ' ';
//! ULLONG timeStamp = 0;
//! CHAR *timeNext = NULL;
//! MLPIRESULT result = utilParameterParseStringToSercosTime(time, separator, &timeStamp, &timeNext );
//! @endcode
inline MLPIRESULT utilParameterParseStringToSercosTime(const CHAR *time, const CHAR separator, ULLONG *timeStamp, CHAR **timeNext=NULL)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  ULONG year = 0;
  ULONG month = 0;
  ULONG day = 0;
  ULONG hour = 0;
  ULONG minute = 0;
  ULONG seconds = 0;
  ULONG nanoseconds = 0;

  if(time==NULL)
    mlpiResult = MLPI_E_INVALIDARG;
  else if (timeStamp==NULL)
    mlpiResult = MLPI_E_INVALIDARG;
  else
  {
    CHAR localDataUtf8[PARAM_SERCOS_TIME_STRING_LENGTH];
    CHAR *localDataUtf8Tmp = localDataUtf8;
    ULONG numElements = 0;

    // convert single sercos time string UTF16 to UTF8
    while( (*time!='\0') && (*time!=separator) ) {
      if( numElements<(PARAM_SERCOS_TIME_STRING_LENGTH-1) )
        localDataUtf8Tmp[numElements++] = (CHAR) *time++;
      else {
        mlpiResult = MLPI_E_INVALIDARG;
        break;
      }
    }

    if(MLPI_SUCCEEDED(mlpiResult))
    {
      if(timeNext!=NULL)
      {
        // Remove separator spaces
        while(*time==MLPI_VALUE_SEPARATOR_UTF8)
          time++;

        *timeNext = const_cast<CHAR*>(time);
      }

      // clear remaining buffer
      while(numElements<PARAM_SERCOS_TIME_STRING_LENGTH)
        localDataUtf8Tmp[numElements++] = '\0';
      localDataUtf8Tmp = localDataUtf8;

      //
      // check characters, valid strings are...
      //
      // YYYY-mm-ddTHH:MM:SS.uuuuuuuuuZ
      // 1970-01-01T00:00:00.000000000Z
      // ...
      // 1970-01-01T00:00:00.00000Z
      // ...
      // 1970-01-01T00:00:00.0Z
      // 1970-01-01T00:00:00Z
      //
      // check string parts: 4 digits for years
      for(ULONG i=0; i<4; i++, localDataUtf8Tmp++)
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 1 digit for year - month separator
      if(*localDataUtf8Tmp++!='-')
        mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 2 digits for months
      for(ULONG i=0; i<2; i++, localDataUtf8Tmp++)
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 1 digit for month - day separator
      if(*localDataUtf8Tmp++!='-')
        mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 2 digits for days
      for(ULONG i=0; i<2; i++, localDataUtf8Tmp++)
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 1 digit for day - hour separator
      if(*localDataUtf8Tmp++!='T')
        mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 2 digits for hours
      for(ULONG i=0; i<2; i++, localDataUtf8Tmp++)
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 1 digit for hour - minute separator
      if(*localDataUtf8Tmp++!=':')
        mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 2 digits for minutes
      for(ULONG i=0; i<2; i++, localDataUtf8Tmp++)
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 1 digit for minute - second separator
      if(*localDataUtf8Tmp++!=':')
        mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: 2 digits for seconds
      for(ULONG i=0; i<2; i++, localDataUtf8Tmp++)
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;

      // check string parts: max. 9 digits for nanoseconds
      if(*localDataUtf8Tmp++=='.') {
        if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') )
          mlpiResult = MLPI_E_INVALIDARG;
        else {
          localDataUtf8Tmp++;
          for(ULONG i=0; i<9; i++, localDataUtf8Tmp++) {
            if( (*localDataUtf8Tmp<'0') || (*localDataUtf8Tmp>'9') ) {
              if ( (*localDataUtf8Tmp++!='Z') || (*localDataUtf8Tmp!='\0') )
              {
                mlpiResult = MLPI_E_INVALIDARG;
              }
              break;
            }
          }
        }
      }
      else if ( (*localDataUtf8Tmp++!='Z') || (*localDataUtf8Tmp!='\0') )
        mlpiResult = MLPI_E_INVALIDARG;
      else
        mlpiResult = MLPI_E_INVALIDARG;
    }

    // convert string to values and check values
    if(MLPI_SUCCEEDED(mlpiResult))
    {
      // MIN: 1970-01-01T00:00:00.000000000Z
      // MAX: 2106-02-07T06:28:15.999999999Z, subtract one second because of return value '-1' of mktime() in case of error

      year = strtoul(&localDataUtf8[0], NULL, 10);
      month = strtoul(&localDataUtf8[5], NULL, 10);
      day = strtoul(&localDataUtf8[8], NULL, 10);
      hour = strtoul(&localDataUtf8[11], NULL, 10);
      minute = strtoul(&localDataUtf8[14], NULL, 10);
      seconds = strtoul(&localDataUtf8[17], NULL, 10);
      nanoseconds = strtoul(&localDataUtf8[20], NULL, 10);

      if( (year<1970) || (year>2106) )                                                            mlpiResult = MLPI_E_INVALIDARG;
      if( (month<1) || (month>12) )                                                               mlpiResult = MLPI_E_INVALIDARG;
      if( (day<1) || (day>31) )                                                                   mlpiResult = MLPI_E_INVALIDARG;
      if( (hour>23) )                                                                             mlpiResult = MLPI_E_INVALIDARG;
      if( (minute>59) )                                                                           mlpiResult = MLPI_E_INVALIDARG;
      if( (seconds>59) )                                                                          mlpiResult = MLPI_E_INVALIDARG;
      if( (nanoseconds>999999999) )                                                               mlpiResult = MLPI_E_INVALIDARG;
      if( (year==2106) && (month>2) )                                                             mlpiResult = MLPI_E_INVALIDARG;
      if( (year==2106) && (month==2) && (day>7) )                                                 mlpiResult = MLPI_E_INVALIDARG;
      if( (year==2106) && (month==2) && (day==7) && (hour>6) )                                    mlpiResult = MLPI_E_INVALIDARG;
      if( (year==2106) && (month==2) && (day==7) && (hour==6) && (minute>28) )                    mlpiResult = MLPI_E_INVALIDARG;
      if( (year==2106) && (month==2) && (day==7) && (hour==6) && (minute==28) && (seconds>15) )   mlpiResult = MLPI_E_INVALIDARG;
      if( (year==2106) && (month==2) && (day==7) && (hour==6) && (minute==28) && (seconds==15) )  seconds = 14;
    }

    if(MLPI_SUCCEEDED(mlpiResult))
    {
      time_t timeSeconds=0;
      tm timeBuffer;
      memset(&timeBuffer, 0, sizeof(timeBuffer));

      timeBuffer.tm_year = year - 1970 + 70;
      timeBuffer.tm_mon = month - 1;
      timeBuffer.tm_mday = day;
      timeBuffer.tm_hour = hour;
      timeBuffer.tm_min = minute;
      timeBuffer.tm_sec = seconds;


#if defined(TARGET_OS_WINCE32)
#pragma message ("not supported on TARGET_OS_WINCE32")
      mlpiResult = MLPI_E_UNIMPLEMENTED;
#else
      // time correction for time zone differences
      time_t rawtime;
      struct tm * timeinfo;
      LLONG CorrectSeconds = 0;

      rawtime = 23 * 60 * 60; // 23 hours in seconds, hours from 0 to 23
      timeinfo = localtime (&rawtime);

      if(timeinfo->tm_mday == 1)
      {
        // negative time zone difference or 0 from UTC
        CorrectSeconds =(timeinfo->tm_hour - 23) * 60 * 60;
      }
      else
      {
        //positive time zone difference from UTC
        CorrectSeconds = timeinfo->tm_hour * 60 * 60 + 60 * 60;
      }

      if( (timeSeconds = mktime( &timeBuffer )) != ((time_t)-1) )
      {
        *timeStamp = ((((ULLONG) timeSeconds) + CorrectSeconds)<<32 ) | ((ULLONG) nanoseconds);
      }
      else
        *timeStamp = 0;
#endif
    }
  }

  return mlpiResult;
}


//! @ingroup UtilParameterHelper
//! This function parse a sercos time string (WCHAR16) and returns a 64-bit sercos time value.
//! Structure of sercos time    Bit 63-32: seconds since 1970-01-01, 00:00:00
//!                             Nit 31-00: nanoseconds up to 999.999.999
//! @param[in]   time           Pointer to a variable which contains at least one sercos time string (ISO-8601).
//! @param[in]   separator      Separator between two or more sercos time strings. Use space character at default.
//! @param[out]  timeStamp      64-bit sercos time value.
//! @param[out]  timeNext       Pointer next sercos time string.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 time[] = L"2000-01-01T23:55:00.123456789Z 2100-12-24T20:00:00.999999999Z 2007-08-14T12:00:00Z";
//! WCHAR16 separator = " ";
//! ULLONG timeStamp = 0;
//! WCHAR16 *timeNext = NULL;
//! MLPIRESULT result = utilParameterParseStringToSercosTime(time, separator, &timeStamp, &timeNext );
//! @endcode
inline MLPIRESULT utilParameterParseStringToSercosTime(const WCHAR16 *time, const WCHAR16 separator, ULLONG *timeStamp, WCHAR16 **timeNext=NULL)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(time==NULL)
    mlpiResult = MLPI_E_INVALIDARG;
  else if (timeStamp==NULL)
    mlpiResult = MLPI_E_INVALIDARG;
  else
  {
    CHAR localDataUtf8[PARAM_SERCOS_TIME_STRING_LENGTH];
    CHAR *localDataUtf8Tmp = localDataUtf8;
    ULONG numElements = 0;

    // convert single sercos time string UTF16 to UTF8
    while( (*time!='\0') && (*time!=separator) ) {
      if( numElements<(PARAM_SERCOS_TIME_STRING_LENGTH-1) )
        localDataUtf8Tmp[numElements++] = (CHAR) *time++;
      else
      {
        mlpiResult = MLPI_E_INVALIDARG;
        break;
      }
    }

    if(MLPI_SUCCEEDED(mlpiResult))
    {
      if(timeNext!=NULL)
        *timeNext = const_cast<WCHAR16*>(time);

      // clear remaining buffer
      while(numElements<PARAM_SERCOS_TIME_STRING_LENGTH)
        localDataUtf8Tmp[numElements++] = '\0';
      localDataUtf8Tmp = localDataUtf8;

      mlpiResult = utilParameterParseStringToSercosTime(localDataUtf8,(CHAR)separator, timeStamp, (CHAR**)timeNext);

    }
  }

  return mlpiResult;
}

//! @ingroup UtilParameterHelper
//! This class helps to access the data read via the 'mlpiParameterReadEverything' function.
//! @par Example:
//! See @ref mlpiParameterReadEverything
class MlpiReadEverythingDataAccess
{

#define DATA_STATUS_OP_DATA_INVALID         (0x0100)
#define DATA_IS_INVALID(index)              ((index > _numElements) || _argInvalid)
#define DATA_IS_VALID(index)                ((index <= _numElements) && !_argInvalid)
#define VALID_ELEMENT_ACCESS(name, bit)     bool name(const ULONG index) const {return (DATA_IS_VALID(index) && (_readEverything[index].validElements & (1<<bit))) ? true : false;}

public:
  MlpiReadEverythingDataAccess(const MlpiReadEverything* const readEverything, const ULONG numElements, const UCHAR* const data, const ULONG dataSize)
    : _readEverything(readEverything)
    , _data(data)
    , _numElements(numElements)
    , _dataSize(dataSize)
    , _argInvalid(( (NULL == readEverything) || (NULL == data) || (0 == numElements) || (0 == dataSize) ) ? true : false)
  {
  };

  ~MlpiReadEverythingDataAccess(){};

  VALID_ELEMENT_ACCESS(isValid_DataStatus  , 0);
  VALID_ELEMENT_ACCESS(isValid_Name        , 1);
  VALID_ELEMENT_ACCESS(isValid_Attribute   , 2);
  VALID_ELEMENT_ACCESS(isValid_Unit        , 3);
  VALID_ELEMENT_ACCESS(isValid_MinimumValue, 4);
  VALID_ELEMENT_ACCESS(isValid_MaximumValue, 5);
  VALID_ELEMENT_ACCESS(isValid_Data        , 6);

  const WCHAR16* getName(const ULONG index) const {
    return isValid_Name(index) ? reinterpret_cast<const WCHAR16*>(_data + _readEverything[index].nameOffset) : L"";
  }

  const WCHAR16* getUnit(const ULONG index) const {
    return isValid_Unit(index) ? reinterpret_cast<const WCHAR16*>(_data + _readEverything[index].unitOffset) : L"";
  }

  const UCHAR* getData(const ULONG index) const {
    return isValid_Data(index) ? (_data + _readEverything[index].dataOffset) : 0;
  }

  template<class T>
  const T* getData(const ULONG index) const {
    return isValid_Data(index) ? reinterpret_cast<const T*>(_data + _readEverything[index].dataOffset) : 0;
  }

  ULONG getNumDataElements(const ULONG index) const {
    return isValid_Data(index) ?  (_readEverything[index].dataLength ? (_readEverything[index].dataSize / _readEverything[index].dataLength) : 0) : 0;
  }

  const UCHAR* getMinimumValue(const ULONG index) const {
    return isValid_MinimumValue(index) ? (_data + _readEverything[index].minOffset) : 0;
  }

  template<class T>
  const T* getMinimumValue(const ULONG index) const {
    return isValid_MinimumValue(index) ? reinterpret_cast<const T*>(_data + _readEverything[index].minOffset) : 0;
  }

  const UCHAR* getMaximumValue(const ULONG index) const {
    return isValid_MaximumValue(index) ?  (_data + _readEverything[index].maxOffset) : 0;
  }

  template<class T>
  const T* getMaximumValue(const ULONG index) const {
    return isValid_MaximumValue(index) ?  reinterpret_cast<const T*>(_data + _readEverything[index].maxOffset) : 0;
  }

  ULONG getAttribute(const ULONG index) const {
    return isValid_Attribute(index) ? (_readEverything[index].attribute) : 0;
  }

  ULONG getValidElements(const ULONG index) const {
    return (_readEverything[index].validElements);
  }

  ULONG getDataStatus(const ULONG index) const {
    return isValid_DataStatus(index) ? _readEverything[index].dataStatus : DATA_STATUS_OP_DATA_INVALID;
  }

  MlpiParameterCommandStatus getCommandStatus(const ULONG index) const {
    ULONG status = getDataStatus(index);
    return ((status & DATA_STATUS_OP_DATA_INVALID) ? MLPI_PARAMETER_COMMAND_STATUS_INVALID : ((MlpiParameterCommandStatus) status));
  }

  BOOL8 isDataStatusValid(const ULONG index) const {
    return ((getDataStatus(index) & DATA_STATUS_OP_DATA_INVALID) ? FALSE : TRUE);
  }

  MlpiType getDataType(const ULONG index) const {
    if( DATA_IS_INVALID(index) )
      return MLPI_TYPE_INVALID;
    return utilParameterGetDataTypeFromAttribute(_readEverything[index].attribute);
  }

  MLPIRESULT getReturnValue(const ULONG index) const {
    if( DATA_IS_INVALID(index) )
      return 0;
    return (_readEverything[index].result);
  }

private:
  const MlpiReadEverything* const _readEverything;
  const UCHAR*  const _data;
  const ULONG   _numElements;
  const ULONG   _dataSize;
  const bool    _argInvalid;
};


template <typename T>
inline MLPIRESULT mlpiConvertBooleanDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    ULONG numElementsLocal = sizeof(WCHAR16)*numElements;
    ULONG numElementsRetLocal = 0;

    // Convert data to utf16
    if (data!=0) {
      numElementsRetLocal = MLPI_SIZEOF_BOOLEAN_VALUE_TRUE;
      memcpy(dataUtf16, MLPI_BOOLEAN_VALUE_TRUE, numElementsLocal<numElementsRetLocal?numElementsLocal:numElementsRetLocal);
    }
    else {
      numElementsRetLocal = MLPI_SIZEOF_BOOLEAN_VALUE_FALSE;
      memcpy(dataUtf16, MLPI_BOOLEAN_VALUE_FALSE, numElementsLocal<numElementsRetLocal?numElementsLocal:numElementsRetLocal);
    }

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = numElementsRetLocal/sizeof(WCHAR16);

    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if(numElementsLocal < numElementsRetLocal)
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}


template <typename T>
inline MLPIRESULT mlpiConvertBinaryDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen];
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    dataBuffCnt = sprintf(&dataUtf8[dataBuffCnt], "0b");
    for(LONG j=(sizeof(data)*8)-1; j>=0 ; j--) {
      dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], "%01u", (USHORT) ((data>>j) & 1));
      if (((j%4)==0) && j!=0)
        dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], ".");
    }

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertBinaryDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertBinaryDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet ) { return MLPI_E_INVALIDSIGNATURE; }


// Helper for mlpiConvertUintDecDataToUtf16 to prevent warnings
template <typename T>
inline ULONG mlpiConvertUintDecDataToUtf16ConvertLine( char* dst, T data )                  { return sprintf(dst, "%llu", data);  }
template <> inline ULONG mlpiConvertUintDecDataToUtf16ConvertLine( char* dst, ULLONG data ) { return sprintf(dst, "%llu", data);  }
template <> inline ULONG mlpiConvertUintDecDataToUtf16ConvertLine( char* dst, ULONG data )  { return sprintf(dst, "%u", static_cast<unsigned int>(data));   }
template <> inline ULONG mlpiConvertUintDecDataToUtf16ConvertLine( char* dst, USHORT data ) { return sprintf(dst, "%hu", data);   }
template <> inline ULONG mlpiConvertUintDecDataToUtf16ConvertLine( char* dst, UCHAR data )  { return sprintf(dst, "%hhu", data);  }
// Implementation mlpiConvertUintDecDataToUtf16
template <typename T>
inline MLPIRESULT mlpiConvertUintDecDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    dataBuffCnt = mlpiConvertUintDecDataToUtf16ConvertLine(dataUtf8, data);

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUintDecDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )   { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUintDecDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }


// Helper for mlpiConvertIntDecDataToUtf16 to prevent warnings
template <typename T>
inline ULONG mlpiConvertIntDecDataToUtf16ConvertLine( char* dst, T data )                 { return sprintf(dst, "%lld", data);  }
template <> inline ULONG mlpiConvertIntDecDataToUtf16ConvertLine( char* dst, LLONG data ) { return sprintf(dst, "%lld", data);  }
template <> inline ULONG mlpiConvertIntDecDataToUtf16ConvertLine( char* dst, LONG data )  { return sprintf(dst, "%d", static_cast<int>(data));   }
template <> inline ULONG mlpiConvertIntDecDataToUtf16ConvertLine( char* dst, SHORT data ) { return sprintf(dst, "%hd", data);   }
template <> inline ULONG mlpiConvertIntDecDataToUtf16ConvertLine( char* dst, CHAR data )  { return sprintf(dst, "%hhd", data);  }
// Implementation mlpiConvertIntDecDataToUtf16
template <typename T>
inline MLPIRESULT mlpiConvertIntDecDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    dataBuffCnt = mlpiConvertIntDecDataToUtf16ConvertLine(dataUtf8, data);

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertIntDecDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertIntDecDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet ) { return MLPI_E_INVALIDSIGNATURE; }


// Helper for mlpiConvertUintHexDataToUtf16 to prevent warnings
template <typename T>
inline ULONG mlpiConvertUintHexDataToUtf16ConvertLine( char* dst, T data )                  { return sprintf(dst, "0x%08X%08X",   ((ULONG)(((ULLONG)data)>>32)), ((ULONG)(data)));  }
template <> inline ULONG mlpiConvertUintHexDataToUtf16ConvertLine( char* dst, ULLONG data ) { return sprintf(dst, "0x%08X%08X", (static_cast<unsigned int>(((ULLONG)data)>>32)), (static_cast<unsigned int>(data)));  }
template <> inline ULONG mlpiConvertUintHexDataToUtf16ConvertLine( char* dst, ULONG data )  { return sprintf(dst, "0x%08X", static_cast<unsigned int>(data));                                                }
template <> inline ULONG mlpiConvertUintHexDataToUtf16ConvertLine( char* dst, USHORT data ) { return sprintf(dst, "0x%04X",  data);                                                }
template <> inline ULONG mlpiConvertUintHexDataToUtf16ConvertLine( char* dst, UCHAR data )  { return sprintf(dst, "0x%02X",  data);                                                }
// Implementation mlpiConvertUintHexDataToUtf16
template <typename T>
inline MLPIRESULT mlpiConvertUintHexDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    dataBuffCnt = mlpiConvertUintHexDataToUtf16ConvertLine(dataUtf8, data);

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUintHexDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )   { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUintHexDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }


template <typename T>
inline MLPIRESULT mlpiConvertUintIdnDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    switch(sizeof(T))
    {
      default:
      case sizeof(ULLONG):  dataBuffCnt = sprintf(dataUtf8, "0x%08X%08X", (static_cast<unsigned int>(((ULLONG)data)>>32)), (static_cast<unsigned int>(data))); break; /* No rules for this case so use default */
      case sizeof(UCHAR):   dataBuffCnt = sprintf(dataUtf8, "0x%02X", data);                                               break; /* No rules for this case so use default */
      case sizeof(USHORT):
      {
        if (data & 0x8000)
          dataBuffCnt = sprintf(dataUtf8, "P-%01u-%04u", MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data));
        else
          dataBuffCnt = sprintf(dataUtf8, "S-%01u-%04u", MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data));
        break;
      }
      case sizeof(ULONG):
      {
        if (data & 0x00008000)
          dataBuffCnt = sprintf(dataUtf8, "P-%01u-%04u.%01u.%01u", MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
        else
          dataBuffCnt = sprintf(dataUtf8, "S-%01u-%04u.%01u.%01u", MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
        break;
      }
    }

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUintIdnDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )   { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUintIdnDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }


// Helper for mlpiConvertMlcIdnDataToUtf16 to prevent warnings
template <typename T>
inline ULONG mlpiConvertMlcIdnDataToUtf16ConvertLine( char* dst, T data )                  { return sprintf(dst, "0x%08X%08X", ((ULONG)(((ULLONG)data)>>32)), ((ULONG)(data)));  }
template <> inline ULONG mlpiConvertMlcIdnDataToUtf16ConvertLine( char* dst, UCHAR data )  { return sprintf(dst, "0x%02X",  data);                                                }
template <> inline ULONG mlpiConvertMlcIdnDataToUtf16ConvertLine( char* dst, USHORT data ) { return sprintf(dst, "0x%04X",  data);                                                }
template <> inline ULONG mlpiConvertMlcIdnDataToUtf16ConvertLine( char* dst, ULONG data )  { return sprintf(dst, "0x%08X", static_cast<unsigned int>(data));                                                }
template <> inline ULONG mlpiConvertMlcIdnDataToUtf16ConvertLine( char* dst, ULLONG data )
{
  switch(data & MLPI_SIDN_TYPE_MASK)
  {
    default:                          return sprintf(dst, "0x%08X%08X", (static_cast<unsigned int>(((ULLONG)data)>>32)), (static_cast<unsigned int>(data)));
    case MLPI_SIDN_TYPE_DRIVE_S:      return sprintf(dst, "A%03u:S-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_DRIVE_P:      return sprintf(dst, "A%03u:P-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_AXIS:         return sprintf(dst, "A%03u:A-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_CONTROL:      return sprintf(dst, "C%03u:C-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_KINEMATIC:    return sprintf(dst, "K%03u:K-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_PROBE:        return sprintf(dst, "M%03u:M-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_POSLIMSWITCH: return sprintf(dst, "N%03u:N-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
    case MLPI_SIDN_TYPE_OSCILLOSCOPE: return sprintf(dst, "O%03u:O-%01u-%04u.%01u.%01u", MLPI_SIDN_INST(data), MLPI_SIDN_SET(data), MLPI_SIDN_BLOCK(data), MLPI_SIDN_SI(data), MLPI_SIDN_SE(data));
  }
}
// Implementation mlpiConvertMlcIdnDataToUtf16
template <typename T>
inline MLPIRESULT mlpiConvertMlcIdnDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    dataBuffCnt = mlpiConvertMlcIdnDataToUtf16ConvertLine(dataUtf8, data);

    // Set length
    if(numElementsRet != NULL)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertMlcIdnDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertMlcIdnDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet ) { return MLPI_E_INVALIDSIGNATURE; }


template <typename T>
inline MLPIRESULT mlpiConvertFloatDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    switch(sizeof(T))
    {
      default:             dataBuffCnt = sprintf(dataUtf8, "0x%08X%08X", (static_cast<unsigned int>(((ULLONG)data)>>32)), (static_cast<unsigned int>(data)));    /* No rules for this case so use default */
      case sizeof(DOUBLE): dataBuffCnt = sprintf(dataUtf8, "%lf", data);  break;
      case sizeof(FLOAT):  dataBuffCnt = sprintf(dataUtf8, "%f", data);   break;
    }

    // Set length
    if(numElementsRet)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}


// Helper for mlpiConvertSercosTimeDataToUtf16 to prevent warnings
template <typename T>
inline ULONG mlpiConvertSercosTimeDataToUtf16ConvertLine( char* dst, T data )                  { return sprintf(dst, "0x%08X%08X", ((ULONG)(((ULLONG)data)>>32)), ((ULONG)(data)));  }
template <> inline ULONG mlpiConvertSercosTimeDataToUtf16ConvertLine( char* dst, UCHAR data )  { return sprintf(dst, "0x%02X",  data);                                                }
template <> inline ULONG mlpiConvertSercosTimeDataToUtf16ConvertLine( char* dst, USHORT data ) { return sprintf(dst, "0x%04X",  data);                                                }
template <> inline ULONG mlpiConvertSercosTimeDataToUtf16ConvertLine( char* dst, ULONG data )  { return sprintf(dst, "0x%08X", static_cast<unsigned int>(data));                                                }
template <> inline ULONG mlpiConvertSercosTimeDataToUtf16ConvertLine( char* dst, ULLONG data )
{
  ULONG dataBuffCnt = 0;
  utilParameterParseSercosTimeToString(data, dst, MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE, &dataBuffCnt);
  if(dataBuffCnt!=0) dataBuffCnt--;
  return dataBuffCnt;
}
// Implementation mlpiConvertSercosTimeDataToUtf16
template <typename T>
inline MLPIRESULT mlpiConvertSercosTimeDataToUtf16( T data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    dataBuffCnt = mlpiConvertSercosTimeDataToUtf16ConvertLine(dataUtf8, data);

    // Set length
    if(numElementsRet)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertSercosTimeDataToUtf16( FLOAT data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertSercosTimeDataToUtf16( DOUBLE data, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet ) { return MLPI_E_INVALIDSIGNATURE; }

inline MLPIRESULT mlpiConvertTimeDataToUtf16(ULONG dataValue, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    ULONG timeMs = dataValue % 1000; dataValue /= 1000;
    ULONG timeS  = dataValue % 60;   dataValue /= 60;
    ULONG timeM  = dataValue % 60;   dataValue /= 60;
    ULONG timeH  = dataValue % 24;   dataValue /= 24;
    ULONG timeD  = dataValue;

    if(timeD)  dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], "%ud", static_cast<unsigned int>(timeD));
    if(timeH)  dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], "%uh", static_cast<unsigned int>(timeH));
    if(timeM)  dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], "%um", static_cast<unsigned int>(timeM));
    if(timeS)  dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], "%us", static_cast<unsigned int>(timeS));
    if(timeMs) dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], "%ums", static_cast<unsigned int>(timeMs));

    // Set length
    if(numElementsRet)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}

inline MLPIRESULT mlpiConvertDateDataToUtf16(ULONG dataValue, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

#if defined(TARGET_OS_WINCE32)
#pragma message ("not supported on TARGET_OS_WINCE32")
    mlpiResult = MLPI_E_UNIMPLEMENTED;
#elif defined(TARGET_OS_WINNT)
    tm timeBuffer;
    memset(&timeBuffer, 0, sizeof(timeBuffer));

    gmtime_s(&timeBuffer, (time_t*) &dataValue);

    dataBuffCnt = (ULONG) strftime(dataUtf8, numElements, "%Y-%m-%d", &timeBuffer);
#else
    tm timeBuffer;
    memset(&timeBuffer, 0, sizeof(timeBuffer));

    gmtime_r((time_t*) &dataValue, &timeBuffer);

    dataBuffCnt = (ULONG) strftime(dataUtf8, numElements, "%Y-%m-%d", &timeBuffer);
#endif

    // Set length
    if(numElementsRet)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}

inline MLPIRESULT mlpiConvertDateAndTimeDataToUtf16(ULONG dataValue, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

#if defined(TARGET_OS_WINCE32)
#pragma message ("not supported on TARGET_OS_WINCE32")
    mlpiResult = MLPI_E_UNIMPLEMENTED;
#elif defined(TARGET_OS_WINNT)
    tm timeBuffer;
    memset(&timeBuffer, 0, sizeof(timeBuffer));

    gmtime_s(&timeBuffer, (time_t*) &dataValue);

    dataBuffCnt = (ULONG) strftime(dataUtf8, numElements, "%Y-%m-%d-%H:%M:%S", &timeBuffer);
#else
    tm timeBuffer;
    memset(&timeBuffer, 0, sizeof(timeBuffer));

    gmtime_r((time_t*) &dataValue, &timeBuffer);

    dataBuffCnt = (ULONG) strftime(dataUtf8, numElements, "%Y-%m-%d-%H:%M:%S", &timeBuffer);
#endif

    // Set length
    if(numElementsRet)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}

inline MLPIRESULT mlpiConvertTimeOfDayDataToUtf16(ULONG dataValue, WCHAR16* dataUtf16, const ULONG numElements, ULONG *numElementsRet)
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    ULONG dataBuffCnt = 0;

    // Convert data to utf8
    ULONG timeMs = dataValue % 1000; dataValue /= 1000;
    ULONG timeS  = dataValue % 60;   dataValue /= 60;
    ULONG timeM  = dataValue % 60;   dataValue /= 60;
    ULONG timeH  = dataValue % 24;   dataValue /= 24;

    dataBuffCnt = sprintf(dataUtf8, "%02u:%02u:%02u", static_cast<unsigned int>(timeH), static_cast<unsigned int>(timeM), static_cast<unsigned int>(timeS));
    if(timeMs) dataBuffCnt += sprintf(&dataUtf8[dataBuffCnt], ".%u", static_cast<unsigned int>(timeMs));

    // Set length
    if(numElementsRet)
      *numElementsRet = dataBuffCnt+1;

    // convert utf8 to utf16
    mbstowcs16(dataUtf16, dataUtf8, numElements);
    if(numElements != 0)
      dataUtf16[numElements-1] = 0;
    if( dataBuffCnt >= numElements )
      mlpiResult = MLPI_E_BUFFERTOOSHORT;
  }
  return mlpiResult;
}


template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToBooleanData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLenTemp1 = MLPI_SIZEOF_BOOLEAN_VALUE_TRUE>MLPI_SIZEOF_BOOLEAN_VALUE_FALSE?MLPI_SIZEOF_BOOLEAN_VALUE_TRUE:MLPI_SIZEOF_BOOLEAN_VALUE_FALSE;
    const ULONG dataBuffLen = (MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE<dataBuffLenTemp1?MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE:dataBuffLenTemp1) / sizeof(WCHAR16);
    WCHAR16 dataUtf16Local[dataBuffLen];

    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    memcpy(dataUtf16Local, dataUtf16, dataBuffLen*sizeof(WCHAR16));
    dataUtf16Local[dataBuffLen-1] = 0;

    for(ULONG idx=0; idx<dataBuffLen; idx++, dataUtf16++) {
      if(!dataUtf16Local[idx]) break;
      if(dataUtf16Local[idx]==MLPI_VALUE_SEPARATOR_UTF16) {
        dataUtf16Local[idx--] = 0;
      }
    }

    if (wcscmp16_(MLPI_BOOLEAN_VALUE_TRUE, dataUtf16Local)==0)
      *data = 1;
    else if (wcscmp16_(MLPI_BOOLEAN_VALUE_FALSE, dataUtf16Local)==0)
      *data = 0;
    else
      mlpiResult = MLPI_E_NOTSUPPORTED;

    // Remove separator spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    if(dataUtf16Next!=NULL)
      *dataUtf16Next = const_cast<WCHAR16*>(dataUtf16);
  }
  return mlpiResult;
}

template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToUdecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    T data_ = 0;

    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    // Remove leading zeros
    while( (*dataUtf16=='0') || (*dataUtf16=='+') )
      dataUtf16++;

    if(*dataUtf16!=0)
    {
      switch(*dataUtf16)
      {
        case 'b':
        case 'B':
        {
          // hide 'b', accept only hex characters, but ignore separators '.'
          for(dataUtf16++; (*dataUtf16!=0) && (*dataUtf16!=MLPI_VALUE_SEPARATOR_UTF16); dataUtf16++)
          {
            if(*dataUtf16=='.')
              continue;
            if( (*dataUtf16=='0') || (*dataUtf16=='1') )
              data_ = (data_ << 1) | (*dataUtf16=='1');
            else {
              mlpiResult = MLPI_E_INVALIDARG;
              break;
            }
          }
          break;
        }
        case 'x':
        case 'X':
        {
          // hide 'x', accept only hex characters, but ignore separators '.'
          for(dataUtf16++; (*dataUtf16!=0) && (*dataUtf16!=MLPI_VALUE_SEPARATOR_UTF16); dataUtf16++)
          {
            if(*dataUtf16=='.')
              continue;
            if( (*dataUtf16>='0') && (*dataUtf16<='9') )
              data_ = data_*16 + (*dataUtf16-'0');
            else if ( (*dataUtf16>='a') && (*dataUtf16<='f') )
              data_ = data_*16 + (*dataUtf16-'a'+10);
            else if ( (*dataUtf16>='A') && (*dataUtf16<='F') )
              data_ = data_*16 + (*dataUtf16-'A'+10);
            else {
              mlpiResult = MLPI_E_INVALIDARG;
              break;
            }
          }
          break;
        }
        case 'o':
        case 'O':
        {
          // hide 'o', accept only octal characters, but ignore separators '.'
          for(dataUtf16++; (*dataUtf16!=0) && (*dataUtf16!=MLPI_VALUE_SEPARATOR_UTF16); dataUtf16++)
          {
            if(*dataUtf16=='.')
              continue;
            if( (*dataUtf16>='0') && (*dataUtf16<='7') )
              data_ = data_*8 + (*dataUtf16-'0');
            else {
              mlpiResult = MLPI_E_INVALIDARG;
              break;
            }
          }
          break;
        }
        default:
        {
          // accept only decimal characters, but set special error code if decimal place separator '.' occurs
          for( ; (*dataUtf16!=0) && (*dataUtf16!=MLPI_VALUE_SEPARATOR_UTF16); dataUtf16++)
          {
            if(*dataUtf16=='.') {
              mlpiResult = MLPI_E_NOTSUPPORTED;
              break;
            }
            if( (*dataUtf16>='0') && (*dataUtf16<='9') )
              data_ = data_*10 + (*dataUtf16-'0');
            else {
              mlpiResult = MLPI_E_INVALIDARG;
              break;
            }
          }
          break;
        }
      }
    }

    if(MLPI_SUCCEEDED(mlpiResult))
      *data = data_;

    // Remove separator spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    if(dataUtf16Next!=NULL)
      *dataUtf16Next = const_cast<WCHAR16*>(dataUtf16);
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUtf16ToUdecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, FLOAT *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToUdecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, DOUBLE *data ) { return MLPI_E_INVALIDSIGNATURE; }


template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    BOOL8 signFlag = FALSE;
    T data_ = 0;

    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    // Remove leading zeros
    while( (*dataUtf16=='0') || (*dataUtf16=='+') )
      dataUtf16++;

    // recognize sign
    if(*dataUtf16=='-') {
      signFlag = TRUE;
      dataUtf16++;
    }

    // accept only decimal characters, but set special error code if decimal place separator '.' occurs
    for( ; (*dataUtf16!=0) && (*dataUtf16!=MLPI_VALUE_SEPARATOR_UTF16); dataUtf16++)
    {
      if(*dataUtf16=='.') {
        mlpiResult = MLPI_E_NOTSUPPORTED;
        break;
      }
      if( (*dataUtf16>='0') && (*dataUtf16<='9') )
        data_ = data_*10 + (*dataUtf16-'0');
      else {
        mlpiResult = MLPI_E_INVALIDARG;
        break;
      }
    }

    if(MLPI_SUCCEEDED(mlpiResult))
    {
      if(signFlag==TRUE)
        *data = -data_;
      else
        *data = data_;
    }

    // Remove separator spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    if(dataUtf16Next!=NULL)
      *dataUtf16Next = const_cast<WCHAR16*>(dataUtf16);
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, FLOAT *data )   { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, DOUBLE *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, ULLONG *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, ULONG *data )   { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, USHORT *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToDecData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, UCHAR *data )   { return MLPI_E_INVALIDSIGNATURE; }


template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToFloatData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    const ULONG dataBuffLen = MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE;
    CHAR dataUtf8[dataBuffLen] = "";
    CHAR* dataUtf8End = NULL;

    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    wcstombs16(dataUtf8, dataUtf16, MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE);
    dataUtf8[MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE-1] = 0;

    DOUBLE data_ = strtod(dataUtf8, &dataUtf8End);

    if(dataUtf8End==&dataUtf8[MLPI_CONVERT_VALUE_TO_WCHAR16_BUFFER_STACK_SIZE-1]) {
      // Insufficient stack buffer size
      mlpiResult = MLPI_E_SYSTEMERROR;
    }
    else
    {
      if( (*dataUtf8End==0) || (*dataUtf8End==MLPI_VALUE_SEPARATOR_UTF8) )
      {
        switch(sizeof(T))
        {
          default:              mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(UCHAR):   mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(USHORT):  mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(FLOAT):   *data = (T) data_; break;
          case sizeof(DOUBLE):  *data = (T) data_; break;
        }

        if(MLPI_SUCCEEDED(mlpiResult))
        {
          // Remove separator spaces
          while(*dataUtf8End==MLPI_VALUE_SEPARATOR_UTF8)
            dataUtf8End++;
          if(dataUtf16Next!=NULL)
            *dataUtf16Next = const_cast<WCHAR16*>(&dataUtf16[dataUtf8End-dataUtf8]);
        }
      }
      else {
        mlpiResult = MLPI_E_INVALIDARG;
      }
    }
  }
  return mlpiResult;
}


template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToUintIdnData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    ULONG address = 0;
    ULLONG type = 0, set = 0, block = 0, si = 0, se = 0;

    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    mlpiResult = utilParameterParseIdn(dataUtf16, &address, &type, &set, &block, &si, &se, dataUtf16Next);
    if(MLPI_SUCCEEDED(mlpiResult))
    {
      if( (address!=0) || ((type!=MLPI_SIDN_TYPE_DRIVE_S)&&(type!=MLPI_SIDN_TYPE_DRIVE_P)) )
        mlpiResult = MLPI_E_INVALIDARG;
      else
      {
        switch(sizeof(T))
        {
          default:              mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(UCHAR):   mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(USHORT):  ((si!=0)||(se!=0)) ?  mlpiResult = MLPI_E_INVALIDARG : *data = (T) MLPI_SIDN(type, set, block, si, se); break;
          case sizeof(ULONG):   *data = (T) MLPI_SIDN(type, set, block, si, se); break;
          case sizeof(ULLONG):  mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
        }
      }
    }
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUtf16ToUintIdnData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, FLOAT *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToUintIdnData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, DOUBLE *data ) { return MLPI_E_INVALIDSIGNATURE; }


template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToMlcIdnData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    ULONG address = 0;
    ULLONG type = 0, set = 0, block = 0, si = 0, se = 0;

    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    mlpiResult = utilParameterParseIdn(dataUtf16, &address, &type, &set, &block, &si, &se, dataUtf16Next);
    if(MLPI_SUCCEEDED(mlpiResult))
    {
      if(address>0xFF) {
        mlpiResult = MLPI_E_INVALIDARG;
      }
      else
      {
        switch(sizeof(T))
        {
          default:              mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(UCHAR):   mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(USHORT):  mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(ULONG):   mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
          case sizeof(ULLONG):  *data = (T) (((T) MLPI_SIDN(type, set, block, si, se)) | (((ULLONG) address) << 56)); break;
        }
      }
    }
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUtf16ToMlcIdnData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, FLOAT *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToMlcIdnData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, DOUBLE *data ) { return MLPI_E_INVALIDSIGNATURE; }


template <typename T>
inline MLPIRESULT mlpiConvertUtf16ToSercosTimeData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, T *data )
{
  MLPIRESULT mlpiResult = MLPI_S_OK;

  if(dataUtf16==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else if(data==NULL) {
    mlpiResult = MLPI_E_INVALIDARG;
  }
  else
  {
    // Remove leading spaces
    while(*dataUtf16==MLPI_VALUE_SEPARATOR_UTF16)
      dataUtf16++;

    switch(sizeof(T))
    {
      default:              mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
      case sizeof(UCHAR):   mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
      case sizeof(USHORT):  mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
      case sizeof(ULONG):   mlpiResult = MLPI_E_INVALIDSIGNATURE; break;
      case sizeof(ULLONG):
      {
        ULLONG data_=0;
        mlpiResult = utilParameterParseStringToSercosTime(dataUtf16, MLPI_VALUE_SEPARATOR_UTF16, &data_, dataUtf16Next);
        if(MLPI_SUCCEEDED(mlpiResult))
          *data = (T) (data_);
        else
          mlpiResult = MLPI_E_INVALIDARG;
        break;
      }
    }
  }
  return mlpiResult;
}
template <> inline MLPIRESULT mlpiConvertUtf16ToSercosTimeData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, FLOAT *data )  { return MLPI_E_INVALIDSIGNATURE; }
template <> inline MLPIRESULT mlpiConvertUtf16ToSercosTimeData( const WCHAR16* dataUtf16, WCHAR16** dataUtf16Next, DOUBLE *data ) { return MLPI_E_INVALIDSIGNATURE; }

#endif /* __MLPIPARAMETERHELPER_H__ */
