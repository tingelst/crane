#ifndef __MLPIGLOBAL_H__
#define __MLPIGLOBAL_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiGlobal.h>
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


#ifndef __cplusplus
  #error MLPI requires C++ compilation (use a .cpp suffix)
#endif

#include "targetinfo.h"

// Needed only when RTP is used.
// Because when RTP is used. The ACI includes the libBase type definition headers and they cause conflic with the
// VxWorks header semLib.h added in marshal.h
// The conflict hierarchy occurs as follows:
// mlpiApiLibClient.cpp -> mlpiApiLibId.h -> mlpiGlobalId.h -> marshal.h -> semLib.h -> vxWorks.h -> vxWorksCommon.h -> vxTypesOld.h (VOID defined here)
// mlpiApiLibClient.cpp -> mlpiApiLib.h -> mlpiGlobal.h -> globtype.h (VOID defined here)
#if defined(__RTP__)
#undef VOID
#endif

// In case ACI is used, then the Atomic Double is used to avoid redefine conflicts
#if defined(__ACI_MLPI__) && defined(GLOBTYPE_USE_ATOMIC_DOUBLE)
  #include "AtomicDouble.h"
#endif

#if defined (TARGET_OS_LINUX) && (_SYS_SYSMACROS_H)
  // There is a capital mistake within sysmacros.h by define 'major',
  // which cause the error 'class 'MlpiVersion' does not have any field named 'gnu_dev_major''
  // therefore we switched it off.
  // Note: You have to add the 'major' define of sysmacros.h (below) again if needed.
  //       #define major(dev) gnu_dev_major(dev)
  #undef major
#endif

#if defined (TARGET_OS_LINUX) && (_SYS_SYSMACROS_H)
  // There is a capital mistake within sysmacros.h by define 'minor',
  // which cause the error 'class 'MlpiVersion' does not have any field named 'gnu_dev_minor''
  // therefore we switched it off.
  // Note: You have to add the 'minor' define of sysmacros.h (below) again if needed.
  //       #define minor(dev) gnu_dev_minor(dev)
  #undef minor
#endif


//
// supported platforms
//
#if defined (TARGET_OS_VXWORKS_KERNEL)
  // compiling under VxWorks environment
#elif defined (TARGET_OS_VXWORKS_RTP)
  // compiling under WIN64 environment
#elif defined (TARGET_OS_WINNT64)
  // compiling under WIN64 environment
#elif defined (TARGET_OS_WINNT32)
  // compiling under WIN32 environment
#elif defined (TARGET_OS_WINCE32)
  // compiling under WINCE environment
#elif defined (TARGET_OS_MAC)
  // compiling under Mac OS environment
#elif defined (TARGET_OS_IOS)
  // compiling under iOS environment
#elif defined (TARGET_OS_ANDROID)
  // compiling under Android environment
#elif defined (TARGET_OS_LINUX)
  // compiling under Linux environment
#else
  #pragma warning "unknown operating system"
#endif

//
// supported compilers
//
#if defined(TARGET_COMPILER_GCC)
  // compiling with GNU GCC/G++ Compiler
#elif defined(TARGET_COMPILER_MSVC)
  // compiling with Microsoft Visual Studio Compiler
#elif defined(TARGET_COMPILER_INTEL)
  // compiling with Intel ICC/ICPC Compiler
#elif defined(TARGET_COMPILER_LLVM)
  // compiling with Clang/LLVM Compiler
#else
  // unknown compiler
  #pragma warning "unknown compiler"
#endif

//
// additional defines
//
// - MLPI_SERVER_LIB: to be used only in the server lib. do not set in the client application!
// - MLPI_CLIENT_LIB: to be used only in the client lib. do not set in the server application!



// -----------------------------------------------------------------------
// GLOBAL TYPES
// -----------------------------------------------------------------------

//
// MLPI basic data types
//
typedef signed char                   BOOL8;      //!< 1 byte boolean
typedef char                          CHAR;       //!< 1 byte signed integer
typedef unsigned char                 UCHAR;      //!< 1 byte unsigned integer
typedef short                         SHORT;      //!< 2 byte signed integer
typedef unsigned short                USHORT;     //!< 2 byte unsigned integer
#if !defined (TARGET_COMPILER_MSVC) && !defined (TARGET_OS_VXWORKS)
typedef int                           LONG;       //!< 4 byte signed integer
typedef unsigned int                  ULONG;      //!< 4 byte unsigned integer
#else
typedef long                          LONG;       //!< 4 byte signed integer
typedef unsigned long                 ULONG;      //!< 4 byte unsigned integer
#endif
typedef long long                     LLONG;      //!< 8 byte signed integer
typedef unsigned long long            ULLONG;     //!< 8 byte unsigned integer
typedef float                         FLOAT;      //!< 4 byte floating point
#if !defined(MLPI_SERVER_LIB)
  #if defined(__ACI_MLPI__) && defined(GLOBTYPE_USE_ATOMIC_DOUBLE)
  typedef AtomicDouble                DOUBLE;  
  #else
  typedef double                      DOUBLE;     //!< 8 byte floating point
  #endif
#endif
#if defined(MLPI_SHORT_WCHAR16)
// This is the setting for all unix platforms, as they have buildin 4 byte wchar_t (__APPLE__, __ANDROID__, __linux__).
typedef unsigned short                WCHAR16;    //!< UTF16 string, because wchar_t is 4 byte on mac/linux platforms
#else
  #if (__cplusplus > 201103L)
  // This is the alternative setting for all unix platforms with C++11 support (__APPLE__, __ANDROID__, __linux__).
  typedef char16_t                    WCHAR16;    //!< UTF16 string
  #else
  // This is the alternative settings on unix platforms without C++11 support (__APPLE__, __ANDROID__, __linux__) and all
  // other platforms (_WIN32, __VXWORKS__).
  //
  // Note: Unix platforms (__APPLE__, __ANDROID__, __linux__) has to be used with compiler option '-fshort-wchar' to make
  //       wchar 2 byte wide.
  typedef wchar_t                     WCHAR16;    //!< UTF16 string
  #endif
#endif

#if !defined (TARGET_COMPILER_MSVC) && !defined (TARGET_OS_VXWORKS)
typedef int                           MLPIRESULT; //!< common MLPI-API return value
#else
typedef long                          MLPIRESULT; //!< common MLPI-API return value
#endif

#if defined (TARGET_BITNESS_64BIT)
typedef unsigned long long            MLPIHANDLE; //!< common MLPI-API handle value
#else
typedef unsigned long                 MLPIHANDLE; //!< common MLPI-API handle value
#endif

typedef unsigned long long MLPITASKHANDLE;        //!< MLPI-API handle value used for tasks.

typedef unsigned long      PROCESSHANDLE;         //!< MLPI handle value used for a process.



//! @enum MlpiType
//! This enumeration defines the basic types of as used by the MLPI.
typedef enum MlpiType
{
  MLPI_TYPE_INVALID           =    0,   //!< invalid or not supported type

  MLPI_TYPE_CHAR              =    1,   //!< 1 byte signed integer
  MLPI_TYPE_UCHAR             =    2,   //!< 1 byte unsigned integer
  MLPI_TYPE_SHORT             =    3,   //!< 2 byte signed integer
  MLPI_TYPE_USHORT            =    4,   //!< 2 byte unsigned integer
  MLPI_TYPE_LONG              =    5,   //!< 4 byte signed integer
  MLPI_TYPE_ULONG             =    6,   //!< 4 byte unsigned integer
  MLPI_TYPE_LLONG             =    7,   //!< 8 byte signed integer
  MLPI_TYPE_ULLONG            =    8,   //!< 8 byte unsigned integer
  MLPI_TYPE_FLOAT             =    9,   //!< 4 byte floating point
  MLPI_TYPE_DOUBLE            =   10,   //!< 8 byte floating point

  MLPI_TYPE_CHAR_ARRAY        =   11,   //!< 1 byte signed integer array
  MLPI_TYPE_UCHAR_ARRAY       =   12,   //!< 1 byte unsigned integer array
  MLPI_TYPE_SHORT_ARRAY       =   13,   //!< 2 byte signed integer array
  MLPI_TYPE_USHORT_ARRAY      =   14,   //!< 2 byte unsigned integer array
  MLPI_TYPE_LONG_ARRAY        =   15,   //!< 4 byte signed integer array
  MLPI_TYPE_ULONG_ARRAY       =   16,   //!< 4 byte unsigned integer array
  MLPI_TYPE_LLONG_ARRAY       =   17,   //!< 8 byte signed integer array
  MLPI_TYPE_ULLONG_ARRAY      =   18,   //!< 8 byte unsigned integer array
  MLPI_TYPE_FLOAT_ARRAY       =   19,   //!< 4 byte floating point array
  MLPI_TYPE_DOUBLE_ARRAY      =   20,   //!< 8 byte floating point array

  MLPI_TYPE_BOOL8             =   21,   //!< 1 byte boolean
  MLPI_TYPE_BOOL8_ARRAY       =   22,   //!< 1 byte boolean array

  MLPI_TYPE_CHAR_UTF8         =   23,   //!< string with 1 byte per character
  MLPI_TYPE_CHAR_UTF16        =   24,   //!< string with 2 bytes per character
  MLPI_TYPE_CHAR_UTF8_ARRAY   =   25,   //!< string array with 1 byte per character
  MLPI_TYPE_CHAR_UTF16_ARRAY  =   26    //!< string array with 2 bytes per character
}MlpiType;

//! @enum MlpiProcessState
//! This enumeration defines the state of a process.
typedef enum MlpiProcessState
{
  MLPI_PROCESS_STATUS_NOT_STARTED          =   0, //!< Process not started
  MLPI_PROCESS_STATUS_INITIALIZATION       =  32, //!< Initialization of process
  MLPI_PROCESS_STATUS_INWORK               =  64, //!< Process is in work
  MLPI_PROCESS_STATUS_FINISHED             = 128, //!< Process has finished
  MLPI_PROCESS_STATUS_ERROR                = 256, //!< Process stopped due to internal errors
}MlpiProcessState;

// -----------------------------------------------------------------------
// Alignment
// -----------------------------------------------------------------------

#if defined(TARGET_COMPILER_GCC)
  // compiling with GNU GCC/G++ Compiler
  #define MLPI_DATA_ALIGN(x) __attribute__ ((aligned(x)))
#elif defined(TARGET_COMPILER_MSVC)
  // compiling with Microsoft Visual Studio Compiler
  #define MLPI_DATA_ALIGN(x) __declspec(align(x))
#elif defined(TARGET_COMPILER_INTEL)
  // compiling with Intel ICC/ICPC Compiler
  #define MLPI_DATA_ALIGN(x) __declspec(align(x))
#elif defined(TARGET_COMPILER_LLVM)
  // compiling with Clang/LLVM Compiler
  #define MLPI_DATA_ALIGN(x) __attribute__ ((aligned(x)))
#else
  // unknown compiler
  #pragma warning "unknown compiler"
#endif


// NOTE: Macros for alignment and packing of data structures within MLPI structs.

#define MLPI_STRUCT_ALIGN_BOOL8     MLPI_DATA_ALIGN(1)      //!< 1 byte boolean, aligned within structs to MLPI data type BOOL8

#define MLPI_STRUCT_ALIGN_CHAR      MLPI_DATA_ALIGN(1)      //!< 1 byte signed integer, aligned within structs to MLPI data type CHAR
#define MLPI_STRUCT_ALIGN_SHORT     MLPI_DATA_ALIGN(2)      //!< 2 byte signed integer, aligned within structs to MLPI data type SHORT
#define MLPI_STRUCT_ALIGN_LONG      MLPI_DATA_ALIGN(4)      //!< 4 byte signed integer, aligned within structs to MLPI data type LONG
#define MLPI_STRUCT_ALIGN_LLONG     MLPI_DATA_ALIGN(8)      //!< 8 byte signed integer, aligned within structs to MLPI data type LLONG

#define MLPI_STRUCT_ALIGN_UCHAR     MLPI_DATA_ALIGN(1)      //!< 1 byte unsigned integer, aligned within structs to MLPI data type UCHAR
#define MLPI_STRUCT_ALIGN_USHORT    MLPI_DATA_ALIGN(2)      //!< 2 byte unsigned integer, aligned within structs to MLPI data type USHORT
#define MLPI_STRUCT_ALIGN_ULONG     MLPI_DATA_ALIGN(4)      //!< 4 byte unsigned integer, aligned within structs to MLPI data type ULONG
#define MLPI_STRUCT_ALIGN_ULLONG    MLPI_DATA_ALIGN(8)      //!< 8 byte unsigned integer, aligned within structs to MLPI data type ULLONG

#define MLPI_STRUCT_ALIGN_FLOAT     MLPI_DATA_ALIGN(4)      //!< 4 byte floating point, aligned within structs to MLPI data type FLOAT
#define MLPI_STRUCT_ALIGN_DOUBLE    MLPI_DATA_ALIGN(8)      //!< 8 byte floating point, aligned within structs to MLPI data type DOUBLE

#define MLPI_STRUCT_ALIGN_ENUM      MLPI_DATA_ALIGN(4)      //!< enumeration, aligned within structs to 32 bit

#define MLPI_STRUCT_ALIGN_WCHAR16   MLPI_DATA_ALIGN(2)      //!< 2 byte char, aligned within structs to MLPI data type WCHAR16

#define MLPI_STRUCT_ALIGN_STRUCT                            //!< structure, dummy


// NOTE: Macros for alignment and packing of data structures regarding a PLC interface.
//       It's recommended to use these MLPI IEC types for data types within structures
//       which represent a interface of a external implemented IEC POU (see also
//       mlpiLogicPouExtensionRegister etc.).

typedef BOOL8     MLPI_DATA_ALIGN(1)    MLPI_IEC_BOOL;      //!< 1 byte boolean, aligned to IEC data type BOOL

typedef CHAR      MLPI_DATA_ALIGN(1)    MLPI_IEC_SINT;      //!< 1 byte signed integer, aligned to IEC data type SINT
typedef SHORT     MLPI_DATA_ALIGN(2)    MLPI_IEC_INT;       //!< 2 byte signed integer, aligned to IEC data type INT
typedef LONG      MLPI_DATA_ALIGN(4)    MLPI_IEC_DINT;      //!< 4 byte signed integer, aligned to IEC data type DINT
typedef LLONG     MLPI_DATA_ALIGN(8)    MLPI_IEC_LINT;      //!< 8 byte signed integer, aligned to IEC data type LINT

typedef UCHAR     MLPI_DATA_ALIGN(1)    MLPI_IEC_USINT;     //!< 1 byte unsigned integer, aligned to IEC data type USINT
typedef USHORT    MLPI_DATA_ALIGN(2)    MLPI_IEC_UINT;      //!< 2 byte unsigned integer, aligned to IEC data type UINT
typedef ULONG     MLPI_DATA_ALIGN(4)    MLPI_IEC_UDINT;     //!< 4 byte unsigned integer, aligned to IEC data type UDINT
typedef ULLONG    MLPI_DATA_ALIGN(8)    MLPI_IEC_ULINT;     //!< 8 byte unsigned integer, aligned to IEC data type ULINT

typedef UCHAR     MLPI_DATA_ALIGN(1)    MLPI_IEC_BYTE;      //!< 1 byte unsigned integer, aligned to IEC data type BYTE
typedef USHORT    MLPI_DATA_ALIGN(2)    MLPI_IEC_WORD;      //!< 2 byte unsigned integer, aligned to IEC data type WORD
typedef ULONG     MLPI_DATA_ALIGN(4)    MLPI_IEC_DWORD;     //!< 4 byte unsigned integer, aligned to IEC data type DWORD
typedef ULLONG    MLPI_DATA_ALIGN(8)    MLPI_IEC_LWORD;     //!< 8 byte unsigned integer, aligned to IEC data type LWORD

typedef FLOAT     MLPI_DATA_ALIGN(4)    MLPI_IEC_REAL;      //!< 4 byte floating point, aligned to IEC data type REAL
typedef DOUBLE    MLPI_DATA_ALIGN(8)    MLPI_IEC_LREAL;     //!< 8 byte floating point, aligned to IEC data type LREAL

typedef ULONG     MLPI_DATA_ALIGN(4)    MLPI_IEC_TIME;      //!< 4 byte unsigned integer, aligned to IEC data type TIME
typedef ULONG     MLPI_DATA_ALIGN(4)    MLPI_IEC_DATE;      //!< 4 byte unsigned integer, aligned to IEC data type DATE
typedef ULONG     MLPI_DATA_ALIGN(4)    MLPI_IEC_DAT;       //!< 4 byte unsigned integer, aligned to IEC data type DATE_AND_TIME
typedef ULONG     MLPI_DATA_ALIGN(4)    MLPI_IEC_TOD;       //!< 4 byte unsigned integer, aligned to IEC data type TIME_OF_DTAE
typedef SHORT     MLPI_DATA_ALIGN(4)    MLPI_IEC_ENUM;      //!< 2 byte signed integer, aligned to IEC data type ENUM

typedef CHAR      MLPI_DATA_ALIGN(1)    MLPI_IEC_STRING;    //!< 1 byte signed integer, aligned to IEC data type STRING
typedef SHORT     MLPI_DATA_ALIGN(2)    MLPI_IEC_WSTRING;   //!< 2 byte signed integer, aligned to IEC data type WSTRING



// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

#ifndef TRUE
#define TRUE                        ( 1 )    /* Boolean true  */
#endif

#ifndef FALSE
#define FALSE                       ( 0 )    /* Boolean false */
#endif

//
// MLPI error codes
//
// NOTE: here are the most common basic return codes given by the API.
//       It is not possible to list all codes here. See user documentation of the device for
//       specific error codes.
#define MLPI_S_OK                           ((MLPIRESULT) 0x00360000L)   //!< Return code "everything okay".
#define MLPI_E_FAIL                         ((MLPIRESULT) 0xF0360001L)   //!< General error during function call.
#define MLPI_E_NOTSUPPORTED                 ((MLPIRESULT) 0xF0360002L)   //!< The given function is not supported yet.
#define MLPI_E_INVALIDARG                   ((MLPIRESULT) 0xF0360003L)   //!< Invalid argument given to method.
#define MLPI_E_OUTOFMEMORY                  ((MLPIRESULT) 0xF0360004L)   //!< Out of memory or resources (RAM, sockets, handles, disk space ...).
#define MLPI_E_TIMEOUT                      ((MLPIRESULT) 0xF0360005L)   //!< Timeout during function call.
#define MLPI_E_SERVEREXCEPTION              ((MLPIRESULT) 0xF0360006L)   //!< System exception on control side.
#define MLPI_E_CONNECTFAILED                ((MLPIRESULT) 0xF0360007L)   //!< Connection failed.
#define MLPI_E_CREATEERROR                  ((MLPIRESULT) 0xF0360008L)   //!< Error creating the resource.
#define MLPI_E_SYSTEMERROR                  ((MLPIRESULT) 0xF0360009L)   //!< System error during execution.
#define MLPI_E_BUFFERTOOSHORT               ((MLPIRESULT) 0xF036000AL)   //!< Given buffer is too short.
#define MLPI_E_INVALIDSIGNATURE             ((MLPIRESULT) 0xF036000BL)   //!< Invalid signature.
#define MLPI_E_STARTERROR                   ((MLPIRESULT) 0xF036000CL)   //!< Error during start of functionality.
#define MLPI_E_WATCHDOGWARNING              ((MLPIRESULT) 0xF036000DL)   //!< Watchdog warning occurred.
#define MLPI_E_WATCHDOGERROR                ((MLPIRESULT) 0xF536000EL)   //!< Watchdog error occurred.
#define MLPI_E_UNIMPLEMENTED                ((MLPIRESULT) 0xF036000FL)   //!< The given function is not implemented on this specific device.
#define MLPI_E_LIMIT_MIN                    ((MLPIRESULT) 0xF0360010L)   //!< The minimum of a limitation is exceeded.
#define MLPI_E_LIMIT_MAX                    ((MLPIRESULT) 0xF0360011L)   //!< The maximum of a limitation is exceeded.
#define MLPI_E_VERSION                      ((MLPIRESULT) 0xF0360012L)   //!< Version conflict.
#define MLPI_E_DEPRECATED                   ((MLPIRESULT) 0xF0360013L)   //!< Deprecated. The lib or function is no longer supported.
#define MLPI_E_PERMISSION                   ((MLPIRESULT) 0xF0360014L)   //!< Request declined due to missing permission rights.
#define MLPI_E_TYPE_MISSMATCH               ((MLPIRESULT) 0xF0360015L)   //!< Type mismatch, present type doesn't match requested type.
#define MLPI_E_SIZE_MISSMATCH               ((MLPIRESULT) 0xF0360016L)   //!< Size mismatch, present size doesn't match requested size.
#define MLPI_E_INVALID_HANDLE               ((MLPIRESULT) 0xF0360017L)   //!< Invalid handle argument or NULL pointer argument.
#define MLPI_E_NOCONNECTION                 ((MLPIRESULT) 0xF0360018L)   //!< The connection is not established, no longer established or has been quit by peer.
#define MLPI_E_RD_WR_PROTECTION             ((MLPIRESULT) 0xF0360019L)   //!< Request declined due to read or write protection.
#define MLPI_E_INVALID_FLOATINGPOINT        ((MLPIRESULT) 0xF036001AL)   //!< Invalid floating point number.
#define MLPI_E_NOTINITIALIZED               ((MLPIRESULT) 0xF036001BL)   //!< Object not initialized yet.
#define MLPI_E_STREAM_OUT_OF_SYNC           ((MLPIRESULT) 0xF036001CL)   //!< Send / Receive stream out of synchronization.
#define MLPI_E_INVALID_PRIVATEKEY_FILE      ((MLPIRESULT) 0xF036001DL)   //!< The given private key file is broken or not existing
#define MLPI_E_INVALID_CERTIFICATE_FILE     ((MLPIRESULT) 0xF036001EL)   //!< The given certificate key file is broken or not existing
#define MLPI_E_KEYPAIR_MISSMATCH            ((MLPIRESULT) 0xF036001FL)   //!< The given certificate does not match the given private key
#define MLPI_E_TLS_HANDSHAKE_FAILED         ((MLPIRESULT) 0xF0360020L)   //!< The TLS handshake was not successful
#define MLPI_E_GENERAL_AC_ERROR             ((MLPIRESULT) 0xF0360021L)   //!< General Access Control Error.
#define MLPI_E_POLICY_VIOLATION_AC          ((MLPIRESULT) 0xF0360022L)   //!< A violation in the user policies occurred.
#define MLPI_E_USER_ALREADY_EXISTS          ((MLPIRESULT) 0xF0360023L)   //!< The given username is already in use.
#define MLPI_E_GROUP_ALREADY_EXISTS         ((MLPIRESULT) 0xF0360024L)   //!< The given groupname is already in use.
#define MLPI_E_MEMBERSHIP_ALREADY_EXISTS    ((MLPIRESULT) 0xF0360025L)   //!< The given user is already member of the given group.
#define MLPI_E_USER_NOT_EXISTING            ((MLPIRESULT) 0xF0360026L)   //!< The given user does not exist on the control.
#define MLPI_E_GROUP_NOT_EXISTING           ((MLPIRESULT) 0xF0360027L)   //!< The given group does not exist on the control.
#define MLPI_E_MEMBERSHIP_NOT_EXISTING      ((MLPIRESULT) 0xF0360028L)   //!< The given user is not member of the given group.

#define MLPI_SUCCEEDED(hr)                ((MLPIRESULT)(hr) >= 0)      //!< Returns true if given error code was successful.
#define MLPI_FAILED(hr)                   ((MLPIRESULT)(hr) < 0)       //!< Returns true if given error code was not successful.


//
// Common constants
//
#if defined (TARGET_BITNESS_64BIT)
#define MLPI_INVALIDHANDLE           ((MLPIHANDLE) 0xFFFFFFFFFFFFFFFF)  //!< Invalid handle value.
#else
#define MLPI_INVALIDHANDLE           ((MLPIHANDLE) 0xFFFFFFFF)          //!< Invalid handle value.
#endif



// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
  #pragma pack(push,8)
#endif

//! @typedef MlpiDateAndTime
//! @brief This structure defines the broken date and time information.
//! @details Elements of struct MlpiDateAndTime
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>           Element     </TH><TH> Description                    </TH><TH> Example                                     </TH></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> year        </TD><TD> Year                           </TD><TD> 2000..2099                                  </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> month       </TD><TD> Month                          </TD><TD> 1..12: 1==January, ..., 12==December        </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> day         </TD><TD> Day of month                   </TD><TD> 1..31                                       </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> hour        </TD><TD> Hours after midnight           </TD><TD> 0..23                                       </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> minute      </TD><TD> Minutes after hour             </TD><TD> 0..59                                       </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> second      </TD><TD> Seconds after minute           </TD><TD> 0..59                                       </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> milliSecond </TD><TD> Milliseconds after second      </TD><TD> 0..999                                      </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> microSecond </TD><TD> Microseconds after millisecond </TD><TD> 0..999                                      </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> dayOfWeek   </TD><TD> Day of week                    </TD><TD> 1..7: 1==Monday, ..., 7==Sunday             </TD></TR>
//! <TR><TD id="st_t"> SHORT </TD><TD id="st_e"> dayOfYear   </TD><TD> Day of year                    </TD><TD> 1..366: 1==January 1, 365/366== December 31 </TD></TR>
//! </TABLE>
typedef struct MlpiDateAndTime
{
  SHORT  year;           //! Year, 2000..2099
  SHORT  month;          //! Month, 1..12: 1==January, ..., 12==December
  SHORT  day;            //! Day of month, 1..31
  SHORT  hour;           //! Hour after midnight, 0..23
  SHORT  minute;         //! Minute after hour, 0..59
  SHORT  second;         //! Seconds after minute, 0..59
  SHORT  milliSecond;    //! Milliseconds after second, 0..999
  SHORT  microSecond;    //! Microseconds after millisecond, 0..999
  SHORT  dayOfWeek;      //! Day of Week, 1..7: 1==Monday, ..., 7==Sunday
  SHORT  dayOfYear;      //! Day of Year, 1..366: 1==January 1, 365/366== December 31
}MlpiDateAndTime;


//! @typedef MlpiVersion
//! Describes the API version information.
//! The build number counts continuously within a major release, all other elements count at the beginning,
//! if a higher element is to be incremented.
//! @details Elements of struct MlpiVersion
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>           Element </TH><TH> Description                                                                                </TH></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> major   </TD><TD> Major release, changes of interfaces, behavior or significant changes of code may occur.   </TD></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> minor   </TD><TD> Minor release, new features or a significant set of bug fixes may occur.                   </TD></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> bugfix  </TD><TD> Bugfixes of existing features.                                                             </TD></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> patch   </TD><TD> Patch of an existing release.                                                              </TD></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> build   </TD><TD> Build number.                                                                              </TD></TR>
//! </TABLE>
//!
//! @par Example:
//! @code
//! // initialize a MlpiVersion structure with default values, e.g. to read current version
//! MlpiVersion version;
//! // initialize a MlpiVersion structure with '1.2.3.0', e.g. to compare two version
//! const MlpiVersion version(1,2,3,0);
//! // initialize an aray of MlpiVersion structures (for example with 2 elements), e.g. to set up a black list
//! const MlpiVersion version[] =
//! {
//!   MlpiVersion(1,2,3,4),
//!   MlpiVersion(1,3,5,7)
//! };
//! @endcode
typedef struct MlpiVersion
{
public:
  MlpiVersion()
    :major(0)
    ,minor(0)
    ,bugfix(0)
    ,patch(0)
    ,build(0){}

  MlpiVersion(ULONG major, ULONG minor, ULONG bugfix, ULONG patch, ULONG build)
    :major(major)
    ,minor(minor)
    ,bugfix(bugfix)
    ,patch(patch)
    ,build(build){}

  MlpiVersion(ULONG major, ULONG minor, ULONG bugfix, ULONG patch)
    :major(major)
    ,minor(minor)
    ,bugfix(bugfix)
    ,patch(patch)
    ,build(0){}

  ULONG major;         //!< Major release, changes of interfaces, behavior or significant changes of code may occur.
  ULONG minor;         //!< Minor release, new features or a significant set of bug fixes may occur.
  ULONG bugfix;        //!< Bug fixes of existing features.
  ULONG patch;         //!< Patch of an existing release.
  ULONG build;         //!< Build number.
} MlpiVersion;


#if !defined(TARGET_OS_VXWORKS)
  #pragma pack(pop)
#endif



// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------

// unfortunately missing on some platforms
#ifndef _countof
  #define _countof(x) (sizeof(x)/sizeof(x[0]))
#endif

// convert a given literal to WCHAR16 format
#define MLPI_ADAPT_STRING_WCHAR16_(x) L ## x
#define MLPI_ADAPT_STRING_WCHAR16(x) MLPI_ADAPT_STRING_WCHAR16_(x)  // convert normal string always to WCHAR16 literal
#ifndef MLPI_SERVER_LIB
  #define MLPI_ADAPT_STRING(x) MLPI_ADAPT_STRING_WCHAR16_(x)        // convert normal string to WCHAR16 literal on client side
#else
  #define MLPI_ADAPT_STRING(x) x                                    // convert normal string NOT to WCHAR16 literal on server side
#endif



// -----------------------------------------------------------------------
// GLOBAL FUNCTIONS
// -----------------------------------------------------------------------

// Use to check whether FLOAT or DOUBLE 'data' contains a valid floating point number
class MlpiTestFpu
{
public:
  MlpiTestFpu(const FLOAT *in) {
    test(in);
  }
  MlpiTestFpu(const DOUBLE *in) {
    test(in);
  }
  template <typename T> MlpiTestFpu(const FLOAT *in, T numElements)
    : out(TRUE)
  {
    for(T i=0; (i<numElements) && (out==TRUE); i++) {
      test(in+i);
    }
  }
  template <typename T> MlpiTestFpu(const DOUBLE *in, T numElements)
    : out(TRUE)
  {
    for(T i=0; (i<numElements) && (out==TRUE); i++) {
      test(in+i);
    }
  }
  inline BOOL8 isValid(void) {return out;}

private:
  inline void test(const FLOAT *in)
  {
    ULONG ul = *reinterpret_cast<const ULONG*>(in);
    out = ((ul&0x7fffffff)==0) ? TRUE : ( ((ul&0x7f800000)==0) ? FALSE : ( ((ul&0x7f800000)==0x7f800000) ? FALSE : TRUE ) );
  }
  inline void test(const DOUBLE *in)
  {
    ULLONG ull = *reinterpret_cast<const ULLONG*>(in);
    out = ((ull&0x7fffffffffffffffLL)==0) ? TRUE : ( ((ull&0x7ff0000000000000LL)==0) ? FALSE : ( ((ull&0x7ff0000000000000LL)==0x7ff0000000000000LL) ? FALSE : TRUE ) );
  }
  BOOL8 out;
};

// Use to check whether FLOAT 'data' contains a valid floating point number
inline BOOL8 MLPI_TEST_IS_FLOAT(const FLOAT data) { return(MlpiTestFpu(&data).isValid()); }

// Use to check whether DOUBLE 'data' contains a valid floating point number
inline BOOL8 MLPI_TEST_IS_DOUBLE(const DOUBLE data) { return(MlpiTestFpu(&data).isValid()); }

// Use to check whether FLOAT array 'data' contains a valid array of floating point numbers with 'numElements' number of elements
template <typename T> inline BOOL8 MLPI_TEST_IS_FLOAT_ARRAY(const FLOAT *data, T numElements) { return(MlpiTestFpu(data, numElements).isValid()); }

// Use to check whether DOUBLE array 'data' contains a valid array of floating point numbers with 'numElements' number of elements
template <typename T> inline BOOL8 MLPI_TEST_IS_DOUBLE_ARRAY(const DOUBLE *data, T numElements)  { return(MlpiTestFpu(data, numElements).isValid()); }


#endif // endof: #ifndef __MLPIGLOBAL_H__


