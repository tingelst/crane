#ifndef __MLPIAPIHELPER_H__
#define __MLPIAPIHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiApiHelper.h>
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



//! @addtogroup UtilApiHelper UtilApiHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for api handling.
//!
//! @details
//! Please note that this piece of source code is not directly part
//! of the MLPI. You do not need this file to program against the
//! MLPI. Nevertheless, at least parts of this file have been considered
//! to be somewhat useful when using or learning to use MLPI functionality.
//! It is therefore included without any support but to act as sample code
//! and source of inspiration.
//! @}



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include <set>

#include "mlpiGlobal.h"
#include "wchar16.h"

#include "mlpiApiLib.h"

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

using std::set;
using std::string;

// -----------------------------------------------------------------------
// CLASS
// -----------------------------------------------------------------------
//! @ingroup UtilApiHelper
//! This class helps to find out whether a permission is available.
//! @par Example:
//! @code
//! ULONG numElementsRet = 0;
//! WCHAR16 permissions[4096] = L"";
//!
//! MLPIRESULT result = mlpiApiGetOwnPermissions(connection, permissions, _countof(permissions), &numElementsRet);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! else
//! {
//!   WCHAR16 permission[] = L"MLPI_APILIB_PERMISSION_CONNECTION_CLOSE";
//!
//!   MlpiApiPermissionEvaluation evalPermission(permissions);
//!
//!   if (evalPermission.hasPermission(permission))
//!     log(L"\nThe logged in user has the permission '%s'.", permission);
//!   else
//!     warning(L"\nThe logged in user hasn't the permission '%s'.", permission);
//! }
//! @endcode
class MlpiApiPermissionEvaluation
{
private:
  string *W2S16(WCHAR16 *permission) {
    return(new string(W2A16(permission)));
  }

  set<string> c_setPermission;

public:
  MlpiApiPermissionEvaluation(const WCHAR16 *permissions) {
    if (permissions==NULL)
      return;
    WCHAR16 *loc = wcsdup16(permissions), *curr = loc, *nxt = loc;
    if (loc!=NULL)
    {
      while (true)
      {
        // find separator of two permissions or end of string
        while (*nxt!=0 && *nxt!=';')
          nxt++;
        if (curr==nxt)
          break;
        if (*nxt!=0)
          *nxt++=0;
        // store permission string
        c_setPermission.insert(*MlpiApiPermissionEvaluation::W2S16(curr));
        // go to next permission string
        curr = nxt;
      }
      delete[] loc;
    }
  }

  virtual ~MlpiApiPermissionEvaluation(void) {
    c_setPermission.clear();
  }

  BOOL8 hasPermission(WCHAR16 *permission) {
    string tmp(W2A16(permission));
    if(c_setPermission.find(tmp)!=c_setPermission.end())
      return true;
    else
      return false;
  }
};

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif


#endif /* __MLPIAPIHELPER_H__ */
