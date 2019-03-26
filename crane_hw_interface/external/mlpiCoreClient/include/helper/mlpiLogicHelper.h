#ifndef __MLPILOGICHELPER_H__
#define __MLPILOGICHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiLogicHelper.h>
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
//! @version    1.22.0
//!
//! @date       2013
//
// -----------------------------------------------------------------------



//! @addtogroup UtilLogicHelper UtilLogicHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for logic handling.
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
#include <sstream>
#include <vector>

#include "mlpiGlobal.h"
#include "wchar16.h"

#include "mlpiLogicLib.h"

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! Defines a container which contains a list of strings
typedef std::vector<std::wstring> SymbolList;

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
const WCHAR16 MLPI_READ_VAR_DISPLAY_TYPE_DEC[]      = {'D','E','C',':', '\0'};
const WCHAR16 MLPI_READ_VAR_DISPLAY_TYPE_HEX[]      = {'H','E','X',':', '\0'};
const WCHAR16 MLPI_READ_VAR_DISPLAY_TYPE_BIN[]      = {'B','I','N',':', '\0'};
const WCHAR16 MLPI_READ_VAR_DISPLAY_TYPE_BOOLEAN[]  = {'B','L','N',':', '\0'};

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------


//! @ingroup UtilLogicHelper
//! This functions reads the list of variables of a given application and returns it as
//! as a SymbolList container.
//! @param[in]    connection    Handle for multiple connections.
//! @param[in]    application   Name of application.
//! @param[out]   symbolList    Container of strings which receives all symbols. Note that the container doesn't
//!                             get cleared. New symbols only get added.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! SymbolList symbolsAll;
//!
//! MLPIRESULT result = utilLogicGetSymbolList(connection, L"Application", symbolsAll);
//! if (MLPI_FAILED(result))
//! return result;
//!
//! printf("\n\nList of symbol configuration:");
//! for (SymbolList::iterator iter=symbolsAll.begin(); iter!=symbolsAll.end(); iter++) {
//!   printf("\n- %s", W2A16((*iter).c_str()));
//! }
//! @endcode
inline MLPIRESULT utilLogicGetSymbolList(MLPIHANDLE connection, const std::wstring &application, SymbolList &symbolList)
{
  // Read all symbols of an application.
  // we need to loop as long as there are more symbols available, which is signaled by the value
  // of node. See also mlpi documentation.
  ULONG node = 0;
  WCHAR16 symbols[1024] = L"";

  do {
    // read list of symbols to string
    MLPIRESULT result = mlpiLogicGetSymbolsOfApplication(connection, application.c_str(), &node, symbols, _countof(symbols));

    if (MLPI_FAILED(result))
      return result;

    // tokenized list of strings to vector
    WCHAR16* context = NULL;
    WCHAR16 *token = wcstok_r16(symbols, L";", &context);
    while(token != NULL) {
      symbolList.push_back(token);
      token = wcstok_r16(NULL, L";", &context);
    }
  }
  while (node != 0xFFFFFFFF);

  return MLPI_S_OK;
}

#if !defined (TARGET_OS_VXWORKS)  // no wstringstream support on vxw :-(

//! @ingroup UtilLogicHelper
//! Recursive function which checks if a symbol is an array or user defined structure and
//! adds all array elements or struct elements also to the list of symbols.
//! @param[in]      connection    Handle for multiple connections.
//! @param[in]      symbol        Name of application.
//! @param[out]     symbolList    Container of strings which receives all symbols. Note that the container doesn't
//!                               get cleared. New symbols only get added.
//! @param[in,out]  symbolInfo    Used as optimization to minimize MLPI function calls and to reuse information from previous recursions.
//! @return                       Return value indicating success (>=0) or error (<0).
inline MLPIRESULT utilLogicInflateSymbol(MLPIHANDLE connection, const std::wstring &symbol, SymbolList &symbolList, MlpiLogicSymbolInformation &symbolInfo)
{
  // optimization: we use the symbolInfo variable as kind of a cached information placeholder to minimize MLPI function calls
  // in case of arrays. If the type is not marked with MLPI_LOGIC_TYPE_UNSUPPORTED, the information about the symbol is still
  // known from the last recursion and we can reuse it. Otherwise, the last recursion was not of the same symbol and therefore we
  // need to call the MLPI function to get the symbol information
  if (symbolInfo.type == MLPI_LOGIC_TYPE_UNSUPPORTED) {
    MLPIRESULT result = mlpiLogicGetInformationOfSymbol(connection, symbol.c_str(), &symbolInfo);
    if (MLPI_FAILED(result))
      return result;
  }

  if(symbolInfo.type == MLPI_LOGIC_TYPE_USERDEF) {
    //
    // symbol is user defined (struct) --> recurse again for each element in struct
    //
    std::vector<MlpiLogicUserTypeInformation> userTypeInfos(symbolInfo.numElements);
    MLPIRESULT result = mlpiLogicGetInformationOfUserType(connection, symbol.c_str(), &userTypeInfos[0], symbolInfo.numElements, NULL);
    if (MLPI_FAILED(result))
      return result;

    for (ULONG i=0; i<symbolInfo.numElements; i++) {
      MlpiLogicSymbolInformation symbolInfoCache;
      symbolInfoCache.type = MLPI_LOGIC_TYPE_UNSUPPORTED;

      std::wstringstream structSymbol;
      structSymbol << symbol << L"." << userTypeInfos.at(i).name;
      MLPIRESULT resultInflate = utilLogicInflateSymbol(connection, structSymbol.str(), symbolList, symbolInfoCache);
      if (MLPI_FAILED(resultInflate))
        return resultInflate;
    }

  } else if (symbolInfo.type == MLPI_LOGIC_TYPE_ARRAY) {
    //
    // symbol is array --> recurse again for each element of array
    //
    MlpiLogicSymbolInformation symbolInfoCache;
    symbolInfoCache.type = MLPI_LOGIC_TYPE_UNSUPPORTED;

    for (ULONG a=symbolInfo.range[0].minimum; a<=symbolInfo.range[0].maximum; a++) {
      for (ULONG b=symbolInfo.range[1].minimum; b<=symbolInfo.range[1].maximum; b++) {
        for (ULONG c=symbolInfo.range[2].minimum; c<=symbolInfo.range[2].maximum; c++) {
          std::wstringstream arraySymbol;
          arraySymbol << symbol << L"[" << a << L"]";
          if (symbolInfo.dimension >= 2) arraySymbol << L"[" << b << L"]";
          if (symbolInfo.dimension >= 3) arraySymbol << L"[" << c << L"]";

          if (symbolInfo.subType == MLPI_LOGIC_TYPE_USERDEF) {
            MLPIRESULT result = utilLogicInflateSymbol(connection, arraySymbol.str(), symbolList, symbolInfoCache);
            if (MLPI_FAILED(result))
              return result;
          } else {
            symbolList.push_back(arraySymbol.str());
          }
        }
      }
    }
  } else {
    //
    // symbol is primitive --> add to list of symbols, end of recursion
    //
    symbolList.push_back(symbol);
  }

  return MLPI_S_OK;
}


//! @ingroup UtilLogicHelper
//! This functions reads the extended list of variables of a given application and returns it as
//! as a SymbolList container. The extended list includes each array element as well as all
//! elements of user defines structs. Please note, that this list might get very long in case of
//! massive use of arrays!
//! @param[in]    connection    Handle for multiple connections.
//! @param[in]    application   Name of application.
//! @param[out]   symbolList    Container of strings which receives all symbols. Note, that the container doesn't
//!                             get cleared. New symbols only get added.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! SymbolList symbolsAllExtended;
//!
//! MLPIRESULT result = utilLogicGetSymbolListExtended(connection, L"Application", symbolsAllExtended);
//! if (MLPI_FAILED(result))
//! return result;
//!
//! printf("\n\nExtended List of symbol configuration:");
//! for (SymbolList::iterator iter=symbolsAllExtended.begin(); iter!=symbolsAllExtended.end(); iter++) {
//!   printf("\n- %s", W2A16((*iter).c_str()));
//! }
//! @endcode
inline MLPIRESULT utilLogicGetSymbolListExtended(MLPIHANDLE connection, const std::wstring &application, SymbolList &symbolList)
{
  // start by reading the simple symbol list, which gets inflated later on.
  SymbolList symbolListSimple;
  MLPIRESULT result = utilLogicGetSymbolList(connection, application, symbolListSimple);
  if (MLPI_FAILED(result))
    return result;

  // based on the simple symbol list which shows all first level symbols, we loop over each variable and
  // check for arrays or user defined structures. All symbols get pushed to a new list which is then
  // our extended symbol list.
  for (SymbolList::iterator iter=symbolListSimple.begin(); iter!=symbolListSimple.end(); iter++) {
    MlpiLogicSymbolInformation symbolInfoCache;
    symbolInfoCache.type = MLPI_LOGIC_TYPE_UNSUPPORTED;

    result = utilLogicInflateSymbol(connection, (*iter).c_str(), symbolList, symbolInfoCache);
    if (MLPI_FAILED(result))
      return result;
  }

  return MLPI_S_OK;
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

//
// "Read variable by symbol" wrapper of PLC data types within the IEC61131 environment 'IndraLogic'
//

inline MLPIRESULT mlpiLogicReadVariableBySymbolBool(const MLPIHANDLE connection, const WCHAR16 *symbol, BOOL8 *data)
  {return mlpiLogicReadVariableBySymbolBool8(connection, symbol, data);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolSint(const MLPIHANDLE connection, const WCHAR16 *symbol, CHAR *data)
  {return mlpiLogicReadVariableBySymbolChar(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolInt(const MLPIHANDLE connection, const WCHAR16 *symbol, SHORT *data)
  {return mlpiLogicReadVariableBySymbolShort(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolDint(const MLPIHANDLE connection, const WCHAR16 *symbol, LONG *data)
  {return mlpiLogicReadVariableBySymbolLong(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolLint(const MLPIHANDLE connection, const WCHAR16 *symbol, LLONG *data)
  {return mlpiLogicReadVariableBySymbolLlong(connection, symbol, data);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolUsint(const MLPIHANDLE connection, const WCHAR16 *symbol, UCHAR *data)
  {return mlpiLogicReadVariableBySymbolUchar(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolUint(const MLPIHANDLE connection, const WCHAR16 *symbol, USHORT *data)
  {return mlpiLogicReadVariableBySymbolUshort(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolUdint(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *data)
  {return mlpiLogicReadVariableBySymbolUlong(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolUlint(const MLPIHANDLE connection, const WCHAR16 *symbol, ULLONG *data)
  {return mlpiLogicReadVariableBySymbolUllong(connection, symbol, data);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolByte(const MLPIHANDLE connection, const WCHAR16 *symbol, UCHAR *data)
  {return mlpiLogicReadVariableBySymbolUchar(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolWord(const MLPIHANDLE connection, const WCHAR16 *symbol, USHORT *data)
  {return mlpiLogicReadVariableBySymbolUshort(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolDword(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *data)
  {return mlpiLogicReadVariableBySymbolUlong(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolLword(const MLPIHANDLE connection, const WCHAR16 *symbol, ULLONG *data)
  {return mlpiLogicReadVariableBySymbolUllong(connection, symbol, data);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolReal(const MLPIHANDLE connection, const WCHAR16 *symbol, FLOAT *data)
  {return mlpiLogicReadVariableBySymbolFloat(connection, symbol, data);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolLreal(const MLPIHANDLE connection, const WCHAR16 *symbol, DOUBLE *data)
  {return mlpiLogicReadVariableBySymbolDouble(connection, symbol, data);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolWstring(const MLPIHANDLE connection, const WCHAR16 *symbol, WCHAR16 *data, const ULONG numElements)
  {return mlpiLogicReadVariableBySymbolString(connection, symbol, data, numElements);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayBool(const MLPIHANDLE connection, const WCHAR16 *symbol, BOOL8 *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayBool8(connection, symbol, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolArraySint(const MLPIHANDLE connection, const WCHAR16 *symbol, CHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayChar(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayInt(const MLPIHANDLE connection, const WCHAR16 *symbol, SHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayShort(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayDint(const MLPIHANDLE connection, const WCHAR16 *symbol, LONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayLong(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayLint(const MLPIHANDLE connection, const WCHAR16 *symbol, LLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayLlong(connection, symbol, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayUsint(const MLPIHANDLE connection, const WCHAR16 *symbol, UCHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUchar(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayUint(const MLPIHANDLE connection, const WCHAR16 *symbol, USHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUshort(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayUdint(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUlong(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayUlint(const MLPIHANDLE connection, const WCHAR16 *symbol, ULLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUllong(connection, symbol, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayByte(const MLPIHANDLE connection, const WCHAR16 *symbol, UCHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUchar(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayWord(const MLPIHANDLE connection, const WCHAR16 *symbol, USHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUshort(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayDword(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUlong(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayLword(const MLPIHANDLE connection, const WCHAR16 *symbol, ULLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayUllong(connection, symbol, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayReal(const MLPIHANDLE connection, const WCHAR16 *symbol, FLOAT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayFloat(connection, symbol, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadVariableBySymbolArrayLreal(const MLPIHANDLE connection, const WCHAR16 *symbol, DOUBLE *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadVariableBySymbolArrayDouble(connection, symbol, data, numElements, numElementsRet);}

//
// "Write variable by symbol" wrapper of PLC data types within the IEC61131 environment 'IndraLogic'
//

inline MLPIRESULT mlpiLogicWriteVariableBySymbolBool(const MLPIHANDLE connection, const WCHAR16 *symbol, const BOOL8 data)
  {return mlpiLogicWriteVariableBySymbolBool8(connection, symbol, data);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolSint(const MLPIHANDLE connection, const WCHAR16 *symbol, const CHAR data)
  {return mlpiLogicWriteVariableBySymbolChar(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolInt(const MLPIHANDLE connection, const WCHAR16 *symbol, const SHORT data)
  {return mlpiLogicWriteVariableBySymbolShort(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolDint(const MLPIHANDLE connection, const WCHAR16 *symbol, const LONG data)
  {return mlpiLogicWriteVariableBySymbolLong(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolLint(const MLPIHANDLE connection, const WCHAR16 *symbol, const LLONG data)
  {return mlpiLogicWriteVariableBySymbolLlong(connection, symbol, data);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolUsint(const MLPIHANDLE connection, const WCHAR16 *symbol, const UCHAR data)
  {return mlpiLogicWriteVariableBySymbolUchar(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolUint(const MLPIHANDLE connection, const WCHAR16 *symbol, const USHORT data)
  {return mlpiLogicWriteVariableBySymbolUshort(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolUdint(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG data)
  {return mlpiLogicWriteVariableBySymbolUlong(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolUlint(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULLONG data)
  {return mlpiLogicWriteVariableBySymbolUllong(connection, symbol, data);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolByte(const MLPIHANDLE connection, const WCHAR16 *symbol, const UCHAR data)
  {return mlpiLogicWriteVariableBySymbolUchar(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolWord(const MLPIHANDLE connection, const WCHAR16 *symbol, const USHORT data)
  {return mlpiLogicWriteVariableBySymbolUshort(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolDword(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG data)
  {return mlpiLogicWriteVariableBySymbolUlong(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolLword(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULLONG data)
  {return mlpiLogicWriteVariableBySymbolUllong(connection, symbol, data);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolReal(const MLPIHANDLE connection, const WCHAR16 *symbol, const FLOAT data)
  {return mlpiLogicWriteVariableBySymbolFloat(connection, symbol, data);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolLreal(const MLPIHANDLE connection, const WCHAR16 *symbol, const DOUBLE data)
  {return mlpiLogicWriteVariableBySymbolDouble(connection, symbol, data);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolWstring(const MLPIHANDLE connection, const WCHAR16 *symbol, const WCHAR16 *data)
  {return mlpiLogicWriteVariableBySymbolString(connection, symbol, data);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayBool(const MLPIHANDLE connection, const WCHAR16 *symbol, const BOOL8 *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayBool8(connection, symbol, data, numElements);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolArraySint(const MLPIHANDLE connection, const WCHAR16 *symbol, const CHAR *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayChar(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayInt(const MLPIHANDLE connection, const WCHAR16 *symbol, const SHORT *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayShort(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayDint(const MLPIHANDLE connection, const WCHAR16 *symbol, const LONG *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayLong(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayLint(const MLPIHANDLE connection, const WCHAR16 *symbol, const LLONG *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayLlong(connection, symbol, data, numElements);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUsint(const MLPIHANDLE connection, const WCHAR16 *symbol, const UCHAR *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUchar(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUint(const MLPIHANDLE connection, const WCHAR16 *symbol, const USHORT *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUshort(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUdint(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUlong(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUlint(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULLONG *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUllong(connection, symbol, data, numElements);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayByte(const MLPIHANDLE connection, const WCHAR16 *symbol, const UCHAR *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUchar(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayWord(const MLPIHANDLE connection, const WCHAR16 *symbol, const USHORT *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUshort(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayDword(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUlong(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayLword(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULLONG *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayUllong(connection, symbol, data, numElements);}

inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayReal(const MLPIHANDLE connection, const WCHAR16 *symbol, const FLOAT *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayFloat(connection, symbol, data, numElements);}
inline MLPIRESULT mlpiLogicWriteVariableBySymbolArrayLreal(const MLPIHANDLE connection, const WCHAR16 *symbol, const DOUBLE *data, const ULONG numElements)
  {return mlpiLogicWriteVariableBySymbolArrayDouble(connection, symbol, data, numElements);}

//
// "Read memory area" wrapper of PLC data types within the IEC61131 environment 'IndraLogic'
//

inline MLPIRESULT mlpiLogicReadMemoryAreaBool(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG bitOffset, BOOL8 *data)
  {return mlpiLogicReadMemoryAreaBool8(connection, application, area, bitOffset, data);}

inline MLPIRESULT mlpiLogicReadMemoryAreaSint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, CHAR *data)
  {return mlpiLogicReadMemoryAreaChar(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaInt(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, SHORT *data)
  {return mlpiLogicReadMemoryAreaShort(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaDint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LONG *data)
  {return mlpiLogicReadMemoryAreaLong(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaLint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LLONG *data)
  {return mlpiLogicReadMemoryAreaLlong(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicReadMemoryAreaUsint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, UCHAR *data)
  {return mlpiLogicReadMemoryAreaUchar(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaUint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, USHORT *data)
  {return mlpiLogicReadMemoryAreaUshort(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaUdint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULONG *data)
  {return mlpiLogicReadMemoryAreaUlong(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaUlint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULLONG *data)
  {return mlpiLogicReadMemoryAreaUllong(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicReadMemoryAreaByte(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, UCHAR *data)
  {return mlpiLogicReadMemoryAreaUchar(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaWord(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, USHORT *data)
  {return mlpiLogicReadMemoryAreaUshort(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaDword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULONG *data)
  {return mlpiLogicReadMemoryAreaUlong(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaLword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULLONG *data)
  {return mlpiLogicReadMemoryAreaUllong(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicReadMemoryAreaReal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, FLOAT *data)
  {return mlpiLogicReadMemoryAreaFloat(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicReadMemoryAreaLreal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, DOUBLE *data)
  {return mlpiLogicReadMemoryAreaDouble(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicReadMemoryAreaArraySint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, CHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayChar(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayInt(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, SHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayShort(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayDint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayLong(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayLint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayLlong(connection, application, area, byteOffset, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadMemoryAreaArrayUsint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, UCHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUchar(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayUint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, USHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUshort(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayUdint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUlong(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayUlint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUllong(connection, application, area, byteOffset, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadMemoryAreaArrayByte(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, UCHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUchar(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayWord(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, USHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUshort(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayDword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUlong(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayLword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayUllong(connection, application, area, byteOffset, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiLogicReadMemoryAreaArrayReal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, FLOAT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayFloat(connection, application, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiLogicReadMemoryAreaArrayLreal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, DOUBLE *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiLogicReadMemoryAreaArrayDouble(connection, application, area, byteOffset, data, numElements, numElementsRet);}

//
// "Write memory area" wrapper of PLC data types within the IEC61131 environment 'IndraLogic'
//

inline MLPIRESULT mlpiLogicWriteMemoryAreaBool(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG bitOffset, const BOOL8 data)
  {return mlpiLogicWriteMemoryAreaBool8(connection, application, area, bitOffset, data);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaSint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const CHAR data)
  {return mlpiLogicWriteMemoryAreaChar(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaInt(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const SHORT data)
  {return mlpiLogicWriteMemoryAreaShort(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaDint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LONG data)
  {return mlpiLogicWriteMemoryAreaLong(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaLint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LLONG data)
  {return mlpiLogicWriteMemoryAreaLlong(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaUsint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const UCHAR data)
  {return mlpiLogicWriteMemoryAreaUchar(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaUint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const USHORT data)
  {return mlpiLogicWriteMemoryAreaUshort(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaUdint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULONG data)
  {return mlpiLogicWriteMemoryAreaUlong(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaUlint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULLONG data)
  {return mlpiLogicWriteMemoryAreaUllong(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaByte(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const UCHAR data)
  {return mlpiLogicWriteMemoryAreaUchar(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaWord(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const USHORT data)
  {return mlpiLogicWriteMemoryAreaUshort(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaDword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULONG data)
  {return mlpiLogicWriteMemoryAreaUlong(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaLword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULLONG data)
  {return mlpiLogicWriteMemoryAreaUllong(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaReal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const FLOAT data)
  {return mlpiLogicWriteMemoryAreaFloat(connection, application, area, byteOffset, data);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaLreal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const DOUBLE data)
  {return mlpiLogicWriteMemoryAreaDouble(connection, application, area, byteOffset, data);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaArraySint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const CHAR *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayChar(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayInt(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const SHORT *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayShort(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayDint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LONG *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayLong(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayLint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LLONG *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayLlong(connection, application, area, byteOffset, data, numElements);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayUsint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const UCHAR *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUchar(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayUint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const USHORT *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUshort(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayUdint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULONG *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUlong(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayUlint(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULLONG *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUllong(connection, application, area, byteOffset, data, numElements);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayByte(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const UCHAR *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUchar(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayWord(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const USHORT *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUshort(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayDword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULONG *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUlong(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayLword(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULLONG *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayUllong(connection, application, area, byteOffset, data, numElements);}

inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayReal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const FLOAT *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayFloat(connection, application, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiLogicWriteMemoryAreaArrayLreal(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const DOUBLE *data, const ULONG numElements)
  {return mlpiLogicWriteMemoryAreaArrayDouble(connection, application, area, byteOffset, data, numElements);}



#ifdef __cplusplus
}
#endif


/*
==============================================================================
History
------------------------------------------------------------------------------
01-Jan-2012
  - first version
==============================================================================
*/

#endif /* __MLPILOGICHELPER_H__ */
