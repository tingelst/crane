#ifndef __MLPITRACELIB_H__
#define __MLPITRACELIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiTraceLib.h>
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



//! @addtogroup TraceLib TraceLib
//! @{
//! @brief Provide access to the common internal tracing module of the device.
//!
//! The tracing system provides a mechanism for collecting and viewing device debug output. Logs from various components
//! and the firmware system are collected in a circular buffer, which then can be read and viewed.
//! There are 3 types of logs in the trace system: message, warning and error. For better filtering and to minimize performance
//! impact on the running system, the logs can be grouped to modules and each module can be activated or deactivated.
//! A log to an activated module is written to one of the given circular buffers of the trace system. Logs to deactivated modules
//! are discarded. It is possible to read the current content of the buffers.
//!
//! The following diagram shows the information flow of a trace message in the system.
//! @image html overview_tracelib.png
//!
//! @note The TraceLib functions trace their debug information mainly into module the MLPI_TRACE_LIB
//!       and in addition into the module MLPI_BASE_MODULES. For further information, see also the
//!       detailed description of the library @ref TraceLib and the notes about @ref sec_TraceViewer.
//!
//! @}

//! @addtogroup TraceLibControlModules Trace modules
//! @ingroup TraceLib
//! @{
//! @brief Use the following functions to activate, deactivate and read information about the trace modules.
//! @}

//! @addtogroup TraceLibControlBuffers Trace buffers
//! @ingroup TraceLib
//! @{
//! @brief Use the following functions to read information on or the content of the trace buffers.
//! @}

//! @addtogroup TraceLibAdd Trace entries
//! @ingroup TraceLib
//! @{
//! @brief Use the following functions to add new entries to the trace system.
//! @}

//! @addtogroup TraceLibVersionPermission Version and Permission
//! @ingroup TraceLib
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_TRACELIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_TRACELIB_PERMISSION_ALWAYS cannot be blocked.
//!
//! @par List of permissions of mlpiTraceLib using in accounts.xml
//! - MLPI_TRACELIB_PERMISSION_ALL
//! - MLPI_TRACELIB_PERMISSION_SETUP
//! - MLPI_TRACELIB_PERMISSION_INFO
//! - MLPI_TRACELIB_PERMISSION_USE
//!
//! <TABLE>
//! <TR><TH>           Function                             </TH><TH> Server version </TH><TH> Permission                         </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceActivateModule         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceDeactivateModule       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceActivateAllModules     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceDeactivateAllModules   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceMessage                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceWarning                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceError                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceEvent                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceGetNumberOfModules     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceGetModuleList          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceGetNumberOfBuffers     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceGetBufferList          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceReadBuffer             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceGetNewestMessageIndex  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceGetOldestMessageIndex  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTraceClearAllBuffers        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TRACELIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! </TABLE>
//!
//! <TABLE>
//! <TR><TH> shortcut </TH><TH> user account            </TH></TR>
//! <TR><TD> a(1)     </TD><TD> administrator           </TD></TR>
//! <TR><TD> i(1)     </TD><TD> indraworks              </TD></TR>
//! <TR><TD> i(2)     </TD><TD> indraworksonline        </TD></TR>
//! <TR><TD> i(3)     </TD><TD> indraworksadministrator </TD></TR>
//! <TR><TD> m(1)     </TD><TD> MlcTrending             </TD></TR>
//! </TABLE>
//!
//! @see
//! @ref sec_Permission
//! @}

//! @addtogroup TraceLibStructTypes Structs, types, ...
//! @ingroup TraceLib
//! @{
//! @brief List of used types, enumerations, structures and more...




// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"


// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_TRACE_MODULE_NAME_SIZE    (20)
#define MLPI_TRACE_BUFFER_NAME_SIZE    (20)
#define MLPI_TRACE_FUNCTION_NAME_SIZE  (30)
#define MLPI_TRACE_MESSAGE_SIZE        (130)

#define MLPI_TRACE_MAX_MODULES         (2000)
#define MLPI_TRACE_MAX_BUFFERS         (16)

#define MLPI_TRACE_MAIN_BUFFER_NAME     L"MAIN"                 //!< Name of the main trace buffer which is always available.
#define MLPI_TRACE_KIS_CMD_BUFFER_NAME  L"KIS_CMD_LOGGER"       //!< Name of the kinematics trace buffer. This Buffer is used for tracing all kinematic user commands.

#define MLPI_TRACE_MODULE_USER          L"MLPI_TRACE_USER"      //!< Name of the module filter name of user.



// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! @enum MlpiTraceType
//! This enumeration defines the different types of entries that can be found in a trace buffer.
typedef enum MlpiTraceType
{
  MLPI_TRACE_TYPE_OTHER     = 0,               //!< Unspecified message type.
  MLPI_TRACE_TYPE_MESSAGE   = 1,               //!< Entry is log message.
  MLPI_TRACE_TYPE_WARNING   = 2,               //!< Entry is warning.
  MLPI_TRACE_TYPE_ERROR     = 3                //!< Entry is error.
}MlpiTraceType;

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiTraceModuleInformation
//! @brief This structure defines the information about a trace module as used by @ref mlpiTraceGetModuleList.
//! @details Elements of struct MlpiTraceModuleInformation
//! <TABLE>
//! <TR><TH>           Type       </TH><TH>           Element           </TH><TH> Description                                                         </TH></TR>
//! <TR><TD id="st_t"> BOOL8      </TD><TD id="st_e"> isActive          </TD><TD> Is this module active? If not, all messages to it are ignored.      </TD></TR>
//! <TR><TD id="st_t"> BOOL8      </TD><TD id="st_e"> toStdOut          </TD><TD> Is output to stdout enabled?                                        </TD></TR>
//! <TR><TD id="st_t"> BOOL8      </TD><TD id="st_e"> toMainBuffer      </TD><TD> Is output to main buffer enabled?                                   </TD></TR>
//! <TR><TD id="st_t"> BOOL8      </TD><TD id="st_e"> toSeparateBuffer  </TD><TD> Is output to separate trace buffer enabled?                         </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]  </TD><TD id="st_e"> moduleName        </TD><TD> Name of the module.                                                 </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]  </TD><TD id="st_e"> bufferName        </TD><TD> Name of optional separate buffer.                                   </TD></TR>
//! </TABLE>
typedef struct MlpiTraceModuleInformation
{
  BOOL8   isActive;                                 //!< Is this module active? If not, all messages to it are ignored.
  BOOL8   toStdOut;                                 //!< Is output to stdout enabled?
  BOOL8   toMainBuffer;                             //!< Is output to main buffer enabled?
  BOOL8   toSeparateBuffer;                         //!< Is output to separate trace buffer enabled?
  WCHAR16 moduleName[MLPI_TRACE_MODULE_NAME_SIZE];  //!< Name of the module.
  WCHAR16 bufferName[MLPI_TRACE_BUFFER_NAME_SIZE];  //!< Name of optional separate buffer.
}MlpiTraceModuleInformation;

//! @typedef MlpiTraceBufferInformation
//! @brief This structure defines the information about a trace buffer as used by @ref mlpiTraceGetBufferList.
//! @details Elements of struct MlpiTraceBufferInformation
//! <TABLE>
//! <TR><TH>           Type       </TH><TH>           Element           </TH><TH> Description                                                         </TH></TR>
//! <TR><TD id="st_t"> WCHAR16[]  </TD><TD id="st_e"> bufferName        </TD><TD> Name of the buffer.                                                 </TD></TR>
//! <TR><TD id="st_t"> LONG       </TD><TD id="st_e"> maximumBufferSize </TD><TD> Maximum number of messages to be stored in the buffer.              </TD></TR>
//! <TR><TD id="st_t"> LONG       </TD><TD id="st_e"> actualBufferSize  </TD><TD> Actual number of messages in the buffer.                            </TD></TR>
//! <TR><TD id="st_t"> BOOL8      </TD><TD id="st_e"> isLocked          </TD><TD> Is this module locked? If true, all messages to it are ignored.     </TD></TR>
//! </TABLE>
typedef struct MlpiTraceBufferInformation
{
  WCHAR16 bufferName[MLPI_TRACE_BUFFER_NAME_SIZE]; //!< Name of the buffer.
  LONG    maximumBufferSize;                       //!< Maximum number of messages to be stored in the buffer.
  LONG    actualBufferSize;                        //!< Actual number of messages in the buffer.
  BOOL8   isLocked;                                //!< Is this module locked? If true, all messages to it are ignored.
}MlpiTraceBufferInformation;

//! @typedef MlpiTraceMessage
//! @brief This structure defines the information about a trace buffer as used by @ref mlpiTraceReadBuffer.
//! @details Elements of struct MlpiTraceMessage
//! <TABLE>
//! <TR><TH>           Type           </TH><TH>           Element       </TH><TH> Description                                         </TH></TR>
//! <TR><TD id="st_t"> ULLONG         </TD><TD id="st_e"> index         </TD><TD> Unique index of the message.                        </TD></TR>
//! <TR><TD id="st_t"> MlpiTraceType  </TD><TD id="st_e"> type          </TD><TD> Type of message (message, error, warning).          </TD></TR>
//! <TR><TD id="st_t"> ULONG          </TD><TD id="st_e"> milliseconds  </TD><TD> Time in milliseconds when the message was sent.     </TD></TR>
//! <TR><TD id="st_t"> ULONG          </TD><TD id="st_e"> lineNumber    </TD><TD> Line in the code in which trace message was sent.   </TD></TR>
//! <TR><TD id="st_t"> ULONG          </TD><TD id="st_e"> threadId      </TD><TD> ID of thread in which trace message was sent.       </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]      </TD><TD id="st_e"> text          </TD><TD> User message text given with trace message.         </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]      </TD><TD id="st_e"> moduleName    </TD><TD> Module name of which the trace message was sent.    </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]      </TD><TD id="st_e"> functionName  </TD><TD> Function name of which the trace message was sent.  </TD></TR>
//! </TABLE>
typedef struct MlpiTraceMessage
{
  ULLONG        index;                                          //!< Unique index of the message.
  MlpiTraceType type;                                           //!< Type of message (message, error, warning).
  ULONG         milliseconds;                                   //!< Time in milliseconds when the message was sent.
  ULONG         lineNumber;                                     //!< Line in the code in which trace message was sent.
  ULONG         threadId;                                       //!< ID of thread in which trace message was sent.
  WCHAR16       text[MLPI_TRACE_MESSAGE_SIZE];                  //!< User message text given with trace message.
  WCHAR16       moduleName[MLPI_TRACE_MODULE_NAME_SIZE];        //!< Module name of which the trace message was sent.
  WCHAR16       functionName[MLPI_TRACE_FUNCTION_NAME_SIZE];    //!< Function name of which the trace message was sent.
}MlpiTraceMessage;

#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup TraceLibStructTypes




// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------
#ifdef MLPI_API
  #undef MLPI_API
#endif

#if defined(TARGET_OS_WINNT)
  #if defined(MLPI_EXPORTS)
    #define MLPI_API __declspec(dllexport)
  #elif defined(MLPI_IMPORTS)
    #define MLPI_API __declspec(dllimport)
  #else
    #define MLPI_API
  #endif
#else
  #if defined(MLPI_EXPORTS)
    #define MLPI_API __attribute__ ((visibility("default")))
  #elif defined(MLPI_IMPORTS)
    #define MLPI_API
  #else
    #define MLPI_API
  #endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

//! @ingroup TraceLibControlModules
//! This function activates a trace module. Trace messages to modules which are not activated will not be written
//! to the trace buffer. This means, that you have to activate a module to trace its messages. Available Module names can be
//! retrieved with the function @ref mlpiTraceGetModuleList. It is highly recommended to use the macro MLPI_TRACE_MODULE_USER
//! for your code.
//!
//! @note Enabling trace modules might cost some CPU performance.
//!
//! @param[in]    connection       Handle for multiple connections.
//! @param[out]   moduleName       Pointer to module name string.
//! @return                        Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Activate only the module 'MLPI_TRACE_MODULE_USER'
//! MLPIRESULT result = mlpiTraceActivateModule(connection, MLPI_TRACE_MODULE_USER);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceActivateModule(const MLPIHANDLE connection, const WCHAR16* moduleName);


//! @ingroup TraceLibControlModules
//! This function deactivates a trace module. Messages to a deactivated trace module are ignored by the tracing system
//! and thus will not appear in the trace buffer. Available Module names can be retrieved with the function
//! @ref mlpiTraceGetModuleList. 
//! @param[in]    connection          Handle for multiple connections.
//! @param[out]   moduleName          Pointer to module name string.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//!
//! @par Example:
//! @code
//! // Deactivate only the module 'MLPI_TRACE_MODULE_USER'
//! MLPIRESULT result = mlpiTraceDeactivateModule(connection, MLPI_TRACE_MODULE_USER);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceDeactivateModule(const MLPIHANDLE connection, const WCHAR16* moduleName);


//! @ingroup TraceLibControlModules
//! This function activates all available trace modules.
//!
//! @note Enabling all trace modules might result in a performance drop, as tracing costs CPU time.
//!       It is not recommended to enable all trace modules on a productive machine. Try activating only
//!       the modules of interest, or one module at a time.
//!
//! @param[in]    connection          Handle for multiple connections.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // activate all available trace modules (not recommended!)
//! MLPIRESULT result = mlpiTraceActivateAllModules(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceActivateAllModules(const MLPIHANDLE connection);


//! @ingroup TraceLibControlModules
//! This function deactivates all available trace modules.
//! @param[in]    connection          Handle for multiple connections.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // activate all available trace modules
//! MLPIRESULT result = mlpiTraceDeactivateAllModules(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceDeactivateAllModules(const MLPIHANDLE connection);


//! @ingroup TraceLibControlModules
//! This function returns the number of registered modules.
//! @param[in]    connection            Handle for multiple connections.
//! @param[out]   numberOfModules       Number of registered modules.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numModules=0;
//! MLPIRESULT result = mlpiTraceGetNumberOfModules(connection, &numModules);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! printf("\nNumber of available trace modules: %d", numModules);
//! @endcode
MLPI_API MLPIRESULT mlpiTraceGetNumberOfModules(const MLPIHANDLE connection, ULONG* numberOfModules);


//! @ingroup TraceLibControlModules
//! This function returns a list of all modules currently available in the tracing system. The module information also contains the name
//! of each module. This name can be used with other calls to the tracing system.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   moduleInfo        Pointer to an array which will receive the module information. One element for each module.
//! @param[in]    numElements       Array size of buffer given to function in number of elements.
//! @param[out]   numElementsRet    Returns the actual number of elements returned.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numModules=0;
//! MlpiTraceModuleInformation modules[MLPI_TRACE_MAX_MODULES];
//!
//! // read array of available modules
//! MLPIRESULT result = mlpiTraceGetModuleList(connection, modules, _countof(modules), &numModules);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! // print all modules found
//! printf("\nFound %d Modules\n", numModules);
//! printf("\n ModuleName          | Active | StdOut | Main   | Separat| Buffer     ");
//! printf("\n---------------------+--------+--------+--------+--------+--------    ");
//! for (ULONG i=0; i<numModules; i++)
//! {
//!   printf("\n%20S", modules[i].moduleName);
//!   printf(" |%7s", (modules[i].isActive) ? "TRUE" : "FALSE");
//!   printf(" |%7s", (modules[i].toStdOut) ? "TRUE" : "FALSE");
//!   printf(" |%7s", (modules[i].toMainBuffer) ? "TRUE" : "FALSE");
//!   printf(" |%7s", (modules[i].toSeparateBuffer) ? "TRUE" : "FALSE");
//!   printf(" | %s", W2A16(modules[i].bufferName));
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceGetModuleList(const MLPIHANDLE connection, MlpiTraceModuleInformation *moduleInfo, const ULONG numElements, ULONG *numElementsRet = 0);


//! @ingroup TraceLibControlBuffers
//! This function returns the number of registered buffers.
//! @param[in]    connection            Handle for multiple connections.
//! @param[out]   numberOfBuffers       Number of registered buffers.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numBuffers=0;
//!
//! MLPIRESULT result = mlpiTraceGetNumberOfBuffers(connection, &numBuffers);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! printf("\nNumber of available trace buffers: %d", numBuffers);
//! @endcode
MLPI_API MLPIRESULT mlpiTraceGetNumberOfBuffers(const MLPIHANDLE connection, ULONG* numberOfBuffers);


//! @ingroup TraceLibControlBuffers
//! This function returns a list of all buffers currently available in the tracing system. The buffer information also contains the name
//! of each buffer. This name can be used with other calls to the tracing system.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   bufferInfo        Pointer to an array which will receive the buffer information. One element for each buffer.
//! @param[in]    numElements       Array size of buffer given to function in number of elements.
//! @param[out]   numElementsRet    Returns the actual number of elements returned.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numBuffers=0;
//! MlpiTraceBufferInformation buffers[MLPI_TRACE_MAX_BUFFERS];
//!
//! // read array of available buffers
//! MLPIRESULT result = mlpiTraceGetBufferList(connection, buffers, _countof(buffers), &numBuffers);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! // print all buffers found
//! printf("\nFound %d Buffers\n", numBuffers);
//! printf("\n BufferName          | Locked | MaxSize  | ActSize        ");
//! printf("\n---------------------+--------+----------+---------       ");
//! for (ULONG i=0; i<numBuffers; i++)
//! {
//!   printf("\n%20s", W2A16(buffers[i].bufferName));
//!   printf(" |%7s", (buffers[i].isLocked) ? "TRUE" : "FALSE");
//!   printf(" | % 8d", buffers[i].maximumBufferSize);
//!   printf(" | % 8d  ", buffers[i].actualBufferSize);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceGetBufferList(const MLPIHANDLE connection, MlpiTraceBufferInformation *bufferInfo, const ULONG numElements, ULONG *numElementsRet = 0);


//! @ingroup TraceLibControlBuffers
//! This functions returns the messages of a given buffer.
//! @param[in]    connection      Handle for multiple connections.
//! @param[in]    bufferName      Name of the buffer to access.
//! @param[in]    startIndex      Index of the first message to start reading. Should be between the indices of @ref mlpiTraceGetNewestMessageIndex
//!                               and @ref mlpiTraceGetOldestMessageIndex.
//! @param[out]   messages        Pointer to buffer which will receive the messages.
//! @param[in]    numElements     Array size of buffer given to function in number of elements.
//! @param[out]   numElementsRet  Returns the actual number of elements(messages) returned.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! WCHAR16 bufferName[] = L"MAIN";   // let's read the buffer called 'MAIN'. Should be available on most targets.
//! MlpiTraceMessage messages[150];   // let's read 150 messages
//! ULLONG oldestIndex = 0;
//! ULLONG newestIndex = 0;
//!
//! // read message index of the newest and oldest available message in the buffer.
//! // We use this information to calculate how many messages are available and to
//! // read the 150 latest messages later on.
//! MLPIRESULT result = mlpiTraceGetOldestMessageIndex(connection, bufferName, &oldestIndex);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! result = mlpiTraceGetNewestMessageIndex(connection, bufferName, &newestIndex);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! ULLONG numMessages = newestIndex-oldestIndex;
//! printf("\nFound %d messages in buffer %s\n", (ULONG)numMessages, W2A16(bufferName));
//! if (numMessages == 0) {
//!   printf("\n->currently no messages to read :-( (tip: enable some modules!)");
//! }
//! else {
//!   // limit to array size
//!   if (numMessages > _countof(messages)) {
//!     numMessages = _countof(messages);
//!   }
//! }
//!
//!
//! // now read the buffer to our array
//! ULONG numMessagesReturned = 0;
//! result = mlpiTraceReadBuffer(connection, bufferName, newestIndex, messages, numMessages, &numMessagesReturned);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! // print all messages to console.
//! // messages in the array are sorted from newest to oldest.
//! // Therefore, inverse the array when printing to console to get newest message
//! // printed last and to read output from top to bottom (oldest to newest).
//! printf("\n Filename                    | Line | Module            | Type | Message      ");
//! printf("\n-----------------------------+------+-------------------+------+------------- ");
//!
//! for (LONG i=numMessagesReturned-1; i>=0; i--) {
//!   printf("\n%-*s %-*d %-*s %-*d %s",
//!     MLPI_TRACE_FUNCTION_NAME_SIZE, W2A16(messages[i].functionName),
//!     5, messages[i].lineNumber,
//!     MLPI_TRACE_MODULE_NAME_SIZE, W2A16(messages[i].moduleName),
//!     5, messages[i].type,
//!     W2A16(messages[i].text)
//!     );
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceReadBuffer(const MLPIHANDLE connection, const WCHAR16 *bufferName, const ULLONG startIndex, MlpiTraceMessage *messages, const ULONG numElements, ULONG *numElementsRet = 0);


//! @ingroup TraceLibControlBuffers
//! This function returns the message index of the newest message available in the given trace buffer. You can
//! use this information for subsequent calls to @c mlpiTraceReadBuffer to read the newest messages in the buffer.
//!
//! @note Each new message that gets added to the trace buffer gets a new unique index. Indices are given in ascending order.
//! @param[in]    connection      Handle for multiple connections.
//! @param[in]    bufferName      Name of the buffer to access.
//! @param[out]   newestIndex     Index of the newest available trace message.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @see mlpiTraceReadBuffer
MLPI_API MLPIRESULT mlpiTraceGetNewestMessageIndex(const MLPIHANDLE connection, const WCHAR16 *bufferName, ULLONG *newestIndex);


//! @ingroup TraceLibControlBuffers
//! This function returns the message index of the oldest message available in the given trace buffer. You can
//! use this information for subsequent calls to @c mlpiTraceReadBuffer to read the oldest messages in the buffer.
//!
//! @note Each new message that gets added to the trace buffer gets a new unique index. Indices are given in ascending order.
//! @param[in]    connection      Handle for multiple connections.
//! @param[in]    bufferName      Name of the buffer to access.
//! @param[out]   oldestIndex     Index of the oldest available trace message. (newestIndex > oldestIndex).
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @see mlpiTraceReadBuffer
MLPI_API MLPIRESULT mlpiTraceGetOldestMessageIndex(const MLPIHANDLE connection, const WCHAR16 *bufferName, ULLONG *oldestIndex);


//! @ingroup TraceLibControlBuffers
//! This function clears all buffers. This means that all trace messages currently available and stored in the system
//! will be deleted.
//! @param[in]    connection      Handle for multiple connections.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Clear all buffers
//! MLPIRESULT result = mlpiTraceClearAllBuffers(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTraceClearAllBuffers(const MLPIHANDLE connection);


//! @ingroup TraceLibAdd
//! This function traces a message.
//! @param[in]  connection    Handle for multiple connections.
//! @param[in]  text          Message text.
//! @param[in]  moduleName    Module filter name.
//! @param[in]  functionName  Name of source function.
//! @param[in]  lineNumber    Line of source file.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @note The message gets discarded if the corresponding module is not activated. Use @ref mlpiTraceActivateModule to activate
//!       the module.
//!
//! @par Example:
//! @code
//! // let's define some macros for automatic function and line insertion
//! #define TRACE_MESSAGE(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceMessage(c, t, m, f, __LINE__); }
//! #define TRACE_WARNING(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceWarning(c, t, m, f, __LINE__); }
//! #define TRACE_ERROR(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceError(c, t, m, f, __LINE__); }
//! #define TRACE_EVENT(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceEvent(c, t, m, f, __LINE__); }
//!
//! // Use macro to set a trace message
//! TRACE_MESSAGE(connection, L"MLPI_TRACE_USER", L"This is a test message for the trace.");
//! @endcode
MLPI_API MLPIRESULT mlpiTraceMessage(const MLPIHANDLE connection, const WCHAR16* text, const WCHAR16 *moduleName=0, const WCHAR16 *functionName=0, const LONG lineNumber=0);


//! @ingroup TraceLibAdd
//! This function traces a warning.
//! @param[in]  connection    Handle for multiple connections.
//! @param[in]  text          Message text.
//! @param[in]  moduleName    Module filter name.
//! @param[in]  functionName  Name of source function.
//! @param[in]  lineNumber    Line of source file.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @note The message gets discarded if the corresponding module is not activated. Use @ref mlpiTraceActivateModule to activate
//!       the module.
//!
//! @par Example:
//! @code
//! // let's define some macros for automatic function and line insertion
//! #define TRACE_MESSAGE(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceMessage(c, t, m, f, __LINE__); }
//! #define TRACE_WARNING(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceWarning(c, t, m, f, __LINE__); }
//! #define TRACE_ERROR(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceError(c, t, m, f, __LINE__); }
//! #define TRACE_EVENT(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceEvent(c, t, m, f, __LINE__); }
//!
//! // Use macro to set a trace warning
//! TRACE_WARNING(connection, L"MLPI_TRACE_USER", L"This is a test warning for the trace.");
//! @endcode
MLPI_API MLPIRESULT mlpiTraceWarning(const MLPIHANDLE connection, const WCHAR16* text, const WCHAR16 *moduleName=0, const WCHAR16 *functionName=0, const LONG lineNumber=0);


//! @ingroup TraceLibAdd
//! This function traces an error.
//! @param[in]  connection    Handle for multiple connections.
//! @param[in]  text          Message text.
//! @param[in]  moduleName    Module filter name.
//! @param[in]  functionName  Name of source function.
//! @param[in]  lineNumber    Line of source file.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @note The error gets discarded if the corresponding module is not activated. Use @ref mlpiTraceActivateModule to activate
//!       the module.
//!
//! @par Example:
//! @code
//! // let's define some macros for automatic function and line insertion
//! #define TRACE_MESSAGE(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceMessage(c, t, m, f, __LINE__); }
//! #define TRACE_WARNING(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceWarning(c, t, m, f, __LINE__); }
//! #define TRACE_ERROR(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceError(c, t, m, f, __LINE__); }
//! #define TRACE_EVENT(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceEvent(c, t, m, f, __LINE__); }
//!
//! // Use macro to set a trace error
//! TRACE_ERROR(connection, L"MLPI_TRACE_USER", L"This is a test error for the trace.");
//! @endcode
MLPI_API MLPIRESULT mlpiTraceError(const MLPIHANDLE connection, const WCHAR16* text, const WCHAR16 *moduleName=0, const WCHAR16 *functionName=0, const LONG lineNumber=0);


//! @ingroup TraceLibAdd
//! This function traces an event.
//! @param[in]  connection    Handle for multiple connections.
//! @param[in]  text          Message text.
//! @param[in]  moduleName    Module filter name.
//! @param[in]  functionName  Name of source function.
//! @param[in]  lineNumber    Line of source file.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @note The event gets discarded if the corresponding module is not activated. Use @ref mlpiTraceActivateModule to activate
//!       the module.
//!
//! @par Example:
//! @code
//! // let's define some macros for automatic function and line insertion
//! #define TRACE_MESSAGE(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceMessage(c, t, m, f, __LINE__); }
//! #define TRACE_WARNING(c, m, t) { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceWarning(c, t, m, f, __LINE__); }
//! #define TRACE_ERROR(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceError(c, t, m, f, __LINE__); }
//! #define TRACE_EVENT(c, m, t)   { WCHAR16* f = A2W16(__FUNCTION__); mlpiTraceEvent(c, t, m, f, __LINE__); }
//!
//! // Use macro to set a trace event
//! TRACE_EVENT(connection, L"MLPI_TRACE_USER", L"This is a test event for the trace.");
//! @endcode
MLPI_API MLPIRESULT mlpiTraceEvent(const MLPIHANDLE connection, const WCHAR16* text, const WCHAR16 *moduleName=0, const WCHAR16 *functionName=0, const LONG lineNumber=0);



#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPITRACELIB_H__
