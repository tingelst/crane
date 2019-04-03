#ifndef __MLPIWATCHDOGLIB_H__
#define __MLPIWATCHDOGLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiWatchdogLib.h>
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



//! @addtogroup WatchdogLib WatchdogLib
//! @{
//! @brief Using the Watchdog library, you can establish a mechanism between your application
//! and the firmware in order to react to exceptions or problems within your client program execution.
//!
//! The following steps are necessary:
//! @li Setup the watchdog and specify the timeout in milliseconds as well as the
//!     action to be taken when the timeout is reached.
//! @li Start the watchdog.
//! @li Reset the watchdog on a cyclic time basis that is lower than the specified timeout.
//!     As soon as you do not reset the watchdog in time, it will execute the action as specified
//!     during setup, e.g. stop the PLC or restart the device.
//!
//!
//! Here is some sample code:
//! @code
//! // initialization of the watchdog (2 seconds).
//! // configure that the watchdog will fire a warning if it doesn't get triggered in time.
//! mlpiWatchdogSetup(connection, 2000, MLPI_WATCHDOG_WARNING);
//!
//! // starting the Watchdog
//! mlpiWatchdogStart(connection);
//!
//! // enter main loop
//! printf("\nWatchdog active! Press any key to block watchdog reset or 'q' to quit.");
//! bool done = false;
//! while(!done)
//! {
//!   // you may want to do some work here...
//!   Sleep(1000);
//!
//!   // reset watchdog to prevent watchdog from triggering, do this every loop
//!   mlpiWatchdogReset(connection);
//!
//!   // print current state of watchdog
//!   MlpiWatchdogState state;
//!   mlpiWatchdogGetState(connection, &state);
//!   switch(state)
//!   {
//!   case MLPI_WATCHDOG_DISABLED:
//!     printf("\nWATCHDOG_DISABLED");
//!     break;
//!   case MLPI_WATCHDOG_ENABLED:
//!     printf("\nWATCHDOG_ENABLED");
//!     break;
//!   case MLPI_WATCHDOG_TIMEOUT:
//!     printf("\nWATCHDOG_TIMEOUT");
//!     break;
//!   }
//!
//!   // on key press by user, delay and block reset of the watchdog --> simulate watchdog timeout
//!   if (_kbhit())
//!   {
//!     char key = _getch();
//!     if (key=='q'){
//!       done = true;
//!     }else{
//!       printf("\nBlocking reset of Watchdog.");
//!       Sleep(5000);
//!     }
//!   }
//! }
//! mlpiWatchdogStop(connection);
//! @endcode
//!
//! @note The WatchdogLib functions trace their debug information mainly into module the MLPI_WATCHDOG_LIB
//!       and in addition into the module MLPI_BASE_MODULES. For further information, see also the
//!       detailed description of the library TraceLib.
//!
//! @}

//! @addtogroup WatchdogLibControl Watchdog handling
//! @ingroup WatchdogLib
//! @{
//! @brief The following functions are used for initializing and using the watchdogs from the client application.
//! @}

//! @addtogroup WatchdogLibVersionPermission Version and Permission
//! @ingroup WatchdogLib
//! @{
//! @addtogroup WatchdogLibVersionPermission_new Server version since 1.26.0.0 (MLC-FW: 14V22)
//! @ingroup WatchdogLibVersionPermission
//! @{
//!
//! @note Since firmware version 14V22 (MLPI-Server-Version: 1.26.0.0) a centralized permission management has been implemented in target 
//! controls XM2, L75 and VPx. Some permissions have been summarized in order to improve their usability. 
//! Additional information regarding the usage of older manifest files (i.e. accounts.xml) with newer server versions can be found in @ref newest_manifest.\n
//! @note <b><span style="color:red">Users of other CML controls (i.e. L25, L45, L65) have to use the old permissions as defined in @ref WatchdogLibVersionPermission_old</span></b>
//!
//!
//! @par List of valid permissions for mlpiWatchdogLib. These permissions shall be assigned to the groups (i.e. in the group manifest file groups.xml) rather than the users.
//! <TABLE>
//! <TR><TH> Permission-Ident       </TH><TH> Description                                                               </TH></TR>                  
//! <TR><TD id="st_e"> WATCHDOG_USE </TD><TD> Use watchdog library - Use watchdog library for MLPI client applications. </TD></TR>  
//! </TABLE>
//! 
//!  @par List of available functions in mlpiWatchdogLib and the permissions required for their use. 
//! <TABLE>
//! <TR><TH>           Function                   </TH><TH> Server version </TH><TH> Permission-Ident </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogSetup     </TD><TD> 1.0.0.0        </TD><TD> "WATCHDOG_USE"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogStart     </TD><TD> 1.0.0.0        </TD><TD> "WATCHDOG_USE"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogStop      </TD><TD> 1.0.0.0        </TD><TD> "WATCHDOG_USE"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogReset     </TD><TD> 1.0.0.0        </TD><TD> "WATCHDOG_USE"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogGetState  </TD><TD> 1.0.0.0        </TD><TD> "WATCHDOG_USE"   </TD></TR>
//! </TABLE>
//! 
//! @par List of the old permissions of mlpiWatchdogLib and their corresponding new permission.
//! <TABLE>
//! <TR><TH> Old permission                            </TH><TH> new Permission    </TH></TR>                  
//! <TR><TD id="st_e"> MLPI_WATCHDOGLIB_PERMISSION_USE </TD><TD> WATCHDOG_USE      </TD></TR>  
//! </TABLE>
//!
//! @}
//! @addtogroup WatchdogLibVersionPermission_old Server versions before 1.26.0.0 
//! @ingroup WatchdogLibVersionPermission
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_WATCHDOGLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_WATCHDOGLIB_PERMISSION_ALWAYS cannot be blocked.
//!
//! @par List of permissions of mlpiWatchdogLib using in accounts.xml
//! - MLPI_APILIB_PERMISSION_ALL
//! - MLPI_WATCHDOGLIB_PERMISSION_USE
//!
//! <TABLE>
//! <TR><TH>           Function                   </TH><TH> Server version </TH><TH> Permission                         </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogSetup     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_WATCHDOGLIB_PERMISSION_USE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogStart     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_WATCHDOGLIB_PERMISSION_USE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogStop      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_WATCHDOGLIB_PERMISSION_USE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogReset     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_WATCHDOGLIB_PERMISSION_USE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiWatchdogGetState  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_WATCHDOGLIB_PERMISSION_USE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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
//! @}

//! @addtogroup WatchdogLibStructTypes Structs, Types, ...
//! @ingroup WatchdogLib
//! @{
//! @brief List of used types, enumerations, structures and more...



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"


// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

//! @enum MlpiWatchdogAction
//! This enumeration must be used to define the action when a watchdog has expired.
typedef enum MlpiWatchdogAction
{
  MLPI_WATCHDOG_REBOOT   = 0x00,  //!< device will be rebooted.
  MLPI_WATCHDOG_PLCSTOPP = 0x01,  //!< PLC will be stopped.
  MLPI_WATCHDOG_ERROR    = 0x02,  //!< Error with class F5 severity (all axes go to error stop) will be generated.
  MLPI_WATCHDOG_WARNING  = 0x03   //!< Error with class F0 severity (warning, no action) will be generated.
} MlpiWatchdogAction;


//! @enum MlpiWatchdogState
//! This enumeration must be used to define the action when a watchdog has expired.
typedef enum MlpiWatchdogState
{
  MLPI_WATCHDOG_DISABLED = 0x00,  //!< Watchdog has not started.
  MLPI_WATCHDOG_ENABLED  = 0x01,  //!< Watchdog has started.
  MLPI_WATCHDOG_TIMEOUT  = 0x02   //!< Watchdog has expired and was disabled.
} MlpiWatchdogState;

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8-byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif


#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup TaskLibStructTypes



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

//! @ingroup WatchdogLibControl
//! This function initializes or changes the watchdog functionality.
//! @param[in]    connection    Handle for multiple connections.
//! @param[in]    timeout       This is the timeout of the watchdog in ms. The timeout is checked every millisecond.
//! @param[in]    action        This is the action that will be taken in the case that the watchdog expires.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // initialization of the watchdog (2 seconds).
//! // configured so that the watchdog will fire a warning if it doesn't get triggered in time.
//! MLPIRESULT result = mlpiWatchdogSetup(connection, 2000, MLPI_WATCHDOG_WARNING);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return;
//! }
//!
//! // starting the Watchdog
//! result = mlpiWatchdogStart(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiWatchdogSetup(const MLPIHANDLE connection, ULONG timeout, MlpiWatchdogAction action);


//! @ingroup WatchdogLibControl
//! This function starts the watchdog supervision.
//! @param[in]    connection    Handle for multiple connections.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See example for @ref mlpiWatchdogSetup.
MLPI_API MLPIRESULT mlpiWatchdogStart(const MLPIHANDLE connection);


//! @ingroup WatchdogLibControl
//! This function stops the watchdog supervision.
//! @param[in]    connection    Handle for multiple connections.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // stop the Watchdog, calls to mlpiWatchdogReset() no longer necessary
//! MLPIRESULT result = mlpiWatchdogStop(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiWatchdogStop(const MLPIHANDLE connection);


//! @ingroup WatchdogLibControl
//! This function resets the watchdog. Has to be called in an interval which is shorter than the timeout
//! that has been specified in @c Setup method.
//! @param[in]    connection    Handle for multiple connections.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! while(1)
//! {
//!   // you may want to do some work here...
//!   Sleep(1000);
//!
//!   // reset watchdog to prevent watchdog from triggering, do this every loop
//!   MLPIRESULT result = mlpiWatchdogReset(connection);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiWatchdogReset(const MLPIHANDLE connection);


//! @ingroup WatchdogLibControl
//! This function will return the current state of the watchdog. Once the watchdog fired,
//! the function will return that state until it is reset using @ref mlpiWatchdogReset. 
//! @param[in]    connection    Handle for multiple connections.
//! @param[out]   state         Pointer to where the state should be stored.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // print current state of watchdog
//! MlpiWatchdogState state;
//! MLPIRESULT result = mlpiWatchdogGetState(connection, &state);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! } else {
//!   switch(state)
//!   {
//!   case MLPI_WATCHDOG_DISABLED:
//!     printf("\nWATCHDOG_DISABLED");
//!     break;
//!   case MLPI_WATCHDOG_ENABLED:
//!     printf("\nWATCHDOG_ENABLED");
//!     break;
//!   case MLPI_WATCHDOG_TIMEOUT:
//!     printf("\nWATCHDOG_TIMEOUT");
//!     break;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiWatchdogGetState(const MLPIHANDLE connection, MlpiWatchdogState *state);



#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIWATCHDOGLIB_H__
