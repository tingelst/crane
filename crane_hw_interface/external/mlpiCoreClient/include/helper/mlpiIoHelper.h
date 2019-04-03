#ifndef __MLPIIOHELPER_H__
#define __MLPIIOHELPER_H__
// -----------------------------------------------------------------------
// MLPI - <mlpiIoHelper.h>
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


//! @addtogroup UtilIoHelper UtilIoHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for I/O handling.
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
#include "mlpiGlobal.h"
#include "wchar16.h"

#include "mlpiIoLib.h"

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------
#define MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(name, bit) bool name() const {return (_flags & (1<<bit)) ? true : false;}

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------

//! @ingroup UtilIoHelper
//! @typedef MlpiIoDiagnosisFlagDecoder
//! The following struct helps to decode the diagnosis flags of MlpiIoDiagnosis as given by
//! mlpiIoReadFieldbusMasterInfo, mlpiIoReadFieldbusSlaveInfo or mlpiIoReadFieldbusSlaveInfos
//!
//! @par Example:
//! @code
//! MLPIRESULT result =  MLPI_S_OK;
//! MlpiIoFieldbusMasterInfo infoMaster;
//!
//! printf("\nWaiting for boot up of fieldbus, time out after 60 seconds...");
//! for (ULONG t=0; t<60; t++)
//! {
//!   memset(&infoMaster, 0, sizeof(infoMaster));
//!
//!   result = mlpiIoReadFieldbusMasterInfo(connection, "Profibus_DP_Master", &infoMaster);
//!   if(MLPI_FAILED(result))
//!   {
//!     printf("\nCall of MLPI function failed with 0x%08x!", (unsigned) result);
//!     break;
//!   }
//!   else
//!   {
//!     MlpiIoDiagnosisFlagDecoder master = infoMaster.diagnosis.flags;
//!
//!     if(master._isOkay()) {
//!       printf("\nReady!");
//!       break;
//!     }
//!     else {
//!       printf(".");
//!       sleep(1000);
//!     }
//!   }
//! }
//! @endcode
typedef struct MlpiIoDiagnosisFlagDecoder
{
public:
  MlpiIoDiagnosisFlagDecoder(const ULONG flags)
    : _flags(flags) {
  }

  operator ULONG() const {
    return _flags;
  }

  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(enable, 0);                                    //!< Fieldbus diagnosis flag device enabled
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(driverAvailable, 4);                           //!< Fieldbus diagnosis flag driver available
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(detected, 5);                                  //!< Fieldbus diagnosis flag device detected
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(configured, 6);                                //!< Fieldbus diagnosis flag device configured
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(active, 7);                                    //!< Fieldbus diagnosis flag device active, bus active
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(busError, 8);                                  //!< Fieldbus diagnosis flag bus error
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(error, 9);                                     //!< Fieldbus diagnosis flag general device error
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(diagnosis, 10);                                //!< Fieldbus diagnosis flag diagnostic information available
  MLPI_IO_DIAGNOSIS_FLAGS_ACCESS(passive, 11);                                  //!< Fieldbus diagnosis flag passive mode of the second master in redundancy systems

  bool _isOkay()    const { return MLPI_IO_FIELDBUS_DIAGNOSIS_OKAY(_flags);  }  //!< Fieldbus diagnosis flag combination 'okay'
  bool _isNotOkay() const { return MLPI_IO_FIELDBUS_DIAGNOSIS_ERROR(_flags); }  //!< Fieldbus diagnosis flag combination 'error or diagnosis'
  bool _hasDiag()   const { return MLPI_IO_FIELDBUS_DIAGNOSIS_DIAG(_flags); }   //!< Fieldbus diagnosis flag combination 'diagnosis'

private:
  ULONG _flags;
}MlpiIoDiagnosisFlagDecoder;

#ifdef __cplusplus
extern "C" {
#endif

//
// "Read fieldbus I/O" wrapper of PLC data types within the IEC61131 environment 'IndraLogic'
//

inline MLPIRESULT mlpiIoReadFieldbusIoBool(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG bitOffset, BOOL8 *data)
  {return mlpiIoReadFieldbusIoBool8(connection, masterName, slaveAddress, area, bitOffset, data);}

inline MLPIRESULT mlpiIoReadFieldbusIoSint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, CHAR *data)
  {return mlpiIoReadFieldbusIoChar(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoInt(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, SHORT *data)
  {return mlpiIoReadFieldbusIoShort(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoDint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LONG *data)
  {return mlpiIoReadFieldbusIoLong(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoLint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LLONG *data)
  {return mlpiIoReadFieldbusIoLlong(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoReadFieldbusIoUsint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, UCHAR *data)
  {return mlpiIoReadFieldbusIoUchar(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoUint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, USHORT *data)
  {return mlpiIoReadFieldbusIoUshort(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoUdint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULONG *data)
  {return mlpiIoReadFieldbusIoUlong(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoUlint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULLONG *data)
  {return mlpiIoReadFieldbusIoUllong(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoReadFieldbusIoByte(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, UCHAR *data)
  {return mlpiIoReadFieldbusIoUchar(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoWord(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, USHORT *data)
  {return mlpiIoReadFieldbusIoUshort(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoDword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULONG *data)
  {return mlpiIoReadFieldbusIoUlong(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoLword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULLONG *data)
  {return mlpiIoReadFieldbusIoUllong(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoReadFieldbusIoReal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, FLOAT *data)
  {return mlpiIoReadFieldbusIoFloat(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoReadFieldbusIoLreal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, DOUBLE *data)
  {return mlpiIoReadFieldbusIoDouble(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoReadFieldbusIoArraySint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, CHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayChar(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayInt(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, SHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayShort(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayDint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayLong(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayLint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayLlong(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiIoReadFieldbusIoArrayUsint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, UCHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUchar(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayUint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, USHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUshort(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayUdint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUlong(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayUlint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUllong(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiIoReadFieldbusIoArrayByte(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, UCHAR *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUchar(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayWord(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, USHORT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUshort(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayDword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUlong(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayLword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULLONG *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayUllong(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}

inline MLPIRESULT mlpiIoReadFieldbusIoArrayReal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, FLOAT *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayFloat(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}
inline MLPIRESULT mlpiIoReadFieldbusIoArrayLreal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, DOUBLE *data, const ULONG numElements, ULONG *numElementsRet)
  {return mlpiIoReadFieldbusIoArrayDouble(connection, masterName, slaveAddress, area, byteOffset, data, numElements, numElementsRet);}

//
// "Write fieldbus I/O" wrapper of PLC data types within the IEC61131 environment 'IndraLogic'
//

inline MLPIRESULT mlpiIoWriteFieldbusIoBool(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG bitOffset, const BOOL8 data)
  {return mlpiIoWriteFieldbusIoBool8(connection, masterName, slaveAddress, area, bitOffset, data);}

inline MLPIRESULT mlpiIoWriteFieldbusIoSint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const CHAR data)
  {return mlpiIoWriteFieldbusIoChar(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoInt(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const SHORT data)
  {return mlpiIoWriteFieldbusIoShort(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoDint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LONG data)
  {return mlpiIoWriteFieldbusIoLong(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoLint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LLONG data)
  {return mlpiIoWriteFieldbusIoLlong(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoWriteFieldbusIoUsint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const UCHAR data)
  {return mlpiIoWriteFieldbusIoUchar(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoUint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const USHORT data)
  {return mlpiIoWriteFieldbusIoUshort(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoUdint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULONG data)
  {return mlpiIoWriteFieldbusIoUlong(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoUlint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULLONG data)
  {return mlpiIoWriteFieldbusIoUllong(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoWriteFieldbusIoByte(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const UCHAR data)
  {return mlpiIoWriteFieldbusIoUchar(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoWord(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const USHORT data)
  {return mlpiIoWriteFieldbusIoUshort(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoDword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULONG data)
  {return mlpiIoWriteFieldbusIoUlong(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoLword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULLONG data)
  {return mlpiIoWriteFieldbusIoUllong(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoWriteFieldbusIoReal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const FLOAT data)
  {return mlpiIoWriteFieldbusIoFloat(connection, masterName, slaveAddress, area, byteOffset, data);}
inline MLPIRESULT mlpiIoWriteFieldbusIoLreal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const DOUBLE data)
  {return mlpiIoWriteFieldbusIoDouble(connection, masterName, slaveAddress, area, byteOffset, data);}

inline MLPIRESULT mlpiIoWriteFieldbusIoArraySint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const CHAR *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayChar(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayInt(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const SHORT *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayShort(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayDint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LONG *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayLong(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayLint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LLONG *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayLlong(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}

inline MLPIRESULT mlpiIoWriteFieldbusIoArrayUsint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const UCHAR *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUchar(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayUint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const USHORT *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUshort(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayUdint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULONG *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUlong(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayUlint(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULLONG *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUllong(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}

inline MLPIRESULT mlpiIoWriteFieldbusIoArrayByte(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const UCHAR *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUchar(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayWord(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const USHORT *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUshort(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayDword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULONG *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUlong(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayLword(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULLONG *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayUllong(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}

inline MLPIRESULT mlpiIoWriteFieldbusIoArrayReal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const FLOAT *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayFloat(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}
inline MLPIRESULT mlpiIoWriteFieldbusIoArrayLreal(const MLPIHANDLE connection, const WCHAR16 *masterName, const WCHAR16 *slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const DOUBLE *data, const ULONG numElements)
  {return mlpiIoWriteFieldbusIoArrayDouble(connection, masterName, slaveAddress, area, byteOffset, data, numElements);}



#ifdef __cplusplus
}
#endif


#endif /* __MLPIIOHELPER_H__ */
