#ifndef __MLPIAPILIB_H__
#define __MLPIAPILIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiApiLib.h>
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


//! @addtogroup ApiLib ApiLib
//! @{
//! @brief The ApiLib contains elementary functions used to establish a connection between your client application
//! and a target system. This has to be done before any other MLPI function can be called.
//!
//! The MLPI is session oriented. This means, that you need to connect to a target before you can use any other
//! MLPI function call (with exception of the function @ref mlpiApiGetClientCoreVersion). After successfully connecting
//! to the target (using @ref mlpiApiConnect) you will get a handle which identifies your connection and which
//! can be used for any further MLPI function call. Please have a look at the documentation and examples of the
//! function @ref mlpiApiConnect for more information.
//!
//! You will find some additional functions to test if the session is still established (@ref mlpiApiIsConnected),
//! to test the connection speed (@ref mlpiApiTestConnection) or to get version information
//! (@ref mlpiApiGetClientCoreVersion, @ref mlpiApiGetServerCoreVersion).
//!
//! @note The ApiLib functions trace their debug information mainly into the module MLPI_API_LIB and in additional
//!       into the module MLPI_BASE_MODULES. For further information, see also the detailed description of the
//!       library @ref TraceLib and the notes about @ref sec_TraceViewer.
//!
//! @}

//! @addtogroup ApiLibConnect Connection control
//! @ingroup ApiLib
//! @{
//! @brief The following functions are used for initializing and configuring the API.
//! This has to be done at least once and before the user application wants to use the MLPI-API.
//! @}
//!

//! @addtogroup ApiLibUtility Utility functions
//! @ingroup ApiLib
//! @{
//! @brief Contains various functions which are handy for MLPI connection testing and information.
//! @}

//! @addtogroup ApiVersionPermission Version and Permission
//! @ingroup ApiLib
//! @{
//! @brief Version and permission information.
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the user
//! permission needed to execute the desired function. Furthermore, the table shows the current user and
//! permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. On using the
//! permission @b "MLPI_APILIB_PERMISSION_ALL" with the value "true" you will enable all functions of this
//! library for a user account.
//!
//! @note Function with permission MLPI_APILIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiApiLib using in accounts.xml
//! - MLPI_APILIB_PERMISSION_ALL
//! - MLPI_APILIB_PERMISSION_CONNECTION_INFO
//! - MLPI_APILIB_PERMISSION_MLPI_INFO
//! - MLPI_APILIB_PERMISSION_CONNECTION_OWN_DESCRIPTION
//! - MLPI_APILIB_PERMISSION_CONNECTION_ALL_DESCRIPTION
//! - MLPI_APILIB_PERMISSION_CONNECTION_CLOSE
//! - MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_RELOAD
//! - MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_LOAD
//! - MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_ACCESS
//!
//! <TABLE>
//! <TR><TH>           Function                                     </TH><TH> Server version </TH><TH> Permission                                           </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiApiConnect                          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiDisconnect                       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiSetDefaultTimeout                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetDefaultTimeout                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiIsConnected                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiTestConnection                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_INFO"             </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiDelay                            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_INFO"             </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetClientCoreVersion             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetServerCoreVersion             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_MLPI_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiSetNameOfConnection              </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_OWN_DESCRIPTION"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiSetLabelOfConnection             </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_OWN_DESCRIPTION"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetOwnConnectionDescription      </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_OWN_DESCRIPTION"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetAllConnectionDescription      </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_ALL_DESCRIPTION"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiCloseConnectionByUid             </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_CLOSE"            </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiCloseConnectionsByUser           </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_CLOSE"            </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiCloseConnectionsByUri            </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_CONNECTION_CLOSE"            </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiUserAccountControlReload         </TD><TD> 1.1.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_RELOAD" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiUserAccountControlLoadAccounts   </TD><TD> 1.4.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_LOAD"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiUserAccountControlUnloadAccounts </TD><TD> 1.4.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_LOAD"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetOwnPermissions                </TD><TD> 1.4.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetAccounts                      </TD><TD> 1.4.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_ACCESS" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetAccountPermissions            </TD><TD> 1.4.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_USER_ACCOUNT_CONTROL_ACCESS" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiNotifyAlive                      </TD><TD> 1.4.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiApiGetLibrarySupport                </TD><TD> 1.8.0.0        </TD><TD> "MLPI_APILIB_PERMISSION_ALWAYS"                      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
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

//! @addtogroup ApiLibStructTypes Structs, Types, ...
//! @ingroup ApiLib
//! @{
//! @brief List of used types, enumerations, structures and more...


// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"


// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_CONNECT_IDENT_LENGTH               (1024)          //!< Length of connect ident string @ref mlpiApiConnect
#define MLPI_DEFAULTPORT                        (5300)          //!< Default IP port when connecting over tcp back end.
#define MLPI_DEFAULTSECUREPORT                  (5335)          //!< Default IP port when connecting over tls back end.
#define MLPI_INFINITE                           (0xFFFFFFFF)    //!< Infinite timeout value.
#define MLPI_TIMEOUT_OS                         (0)             //!< Use timeout values as chosen by the operating system (default).
#define MLPI_MAX_PAYLOAD                        (500000)        //!< Maximum total payload in bytes of MLPI function with all of its arguments.

#define MLPI_API_CONNECTION_NAME_LEN            (128)           //!< Length of user-defined name of connection.
#define MLPI_API_CONNECTION_URI_LEN             (128)           //!< Length of URI of connection.
#define MLPI_API_CONNECTION_LABEL_LEN           (254)           //!< Length of user-defined label of connection.
#define MLPI_API_CONNECTION_USER_LEN            (128)           //!< Length of login user name of connection.
#define MLPI_API_CONNECTION_PWD_LEN             (128)           //!< Length of login user password of connection.

#define MLPI_API_LIBRARY_NAME_LEN               (64)            //!< Length of library name.

#define MLPI_API_MAX_NUMBER_OF_CONNECTIONS      (50)            //!< Maximum number of concurrent connections.

static const WCHAR16 MLPI_LOCALHOST[]  = {'L','O','C','A','L','H','O','S','T', '\0'};                                                     //!< String for local host connection when client application and target are on the same device.
static const WCHAR16 MLPI_ACCOUNT_EMPTY_NAME[]  = {'_','A','C','C','O','U','N','T','_','E','M','P','T','Y','_','N','A','M','E','_','\0'}; //!< Surrogate string of account with empty name defined in account manifest 'accounts.xml'.


// -----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
// -----------------------------------------------------------------------
//! @enum MlpiApiProtection
//! This enumeration defines the protection levels of a connection.
typedef enum MlpiApiProtection
{
  MLPI_API_PROTECTION_NON      = 0,   //!< The connection is not protected and can be closed by an other connection.
  MLPI_API_PROTECTION_WATCHDOG = 1,   //!< The connection is protected against closing by an other connection if the connection watchdog is active (enabled and not fired).
  MLPI_API_PROTECTION_COMPLETE = 2    //!< The connection is protected against closing by an other connection completely.
}MlpiApiProtection;


// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiConnectionInfo
//! This structure is used by the function @ref mlpiApiTestConnection to return the results of the timing measurements.
//! @details Elements of struct MlpiConnectionInfo
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element            </TH><TH> Description                                                </TH></TR>
//! <TR><TD id="st_t"> DOUBLE </TD><TD id="st_e"> minimum            </TD><TD> Minimum time duration of all measurements in microseconds. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE </TD><TD id="st_e"> maximum            </TD><TD> Maximum time duration of all measurements in microseconds. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE </TD><TD id="st_e"> average            </TD><TD> Average time duration of all measurements in microseconds. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE </TD><TD id="st_e"> variance           </TD><TD> Variance of all measurements in square microseconds.       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE </TD><TD id="st_e"> standardDeviation  </TD><TD> Standard deviation of all measurements in microseconds.    </TD></TR>
//! </TABLE>
typedef struct MlpiConnectionInfo
{
  DOUBLE minimum;             //!< Minimum time duration of all measurements.
  DOUBLE maximum;             //!< Maximum time duration of all measurements.
  DOUBLE average;             //!< Average time duration of all measurements.
  DOUBLE variance;            //!< Variance of all measurements.
  DOUBLE standardDeviation;   //!< Standard deviation of all measurements.
} MlpiConnectionInfo;

//! @typedef MlpiConnectionDescription
//! This structure is used by the functions @ref mlpiApiGetOwnConnectionDescription and @ref mlpiApiGetAllConnectionDescription to return
//! information about the connections.
//! @details Elements of struct MlpiConnectionDescription
//! <TABLE>
//! <TR><TH>           Type               </TH><TH>           Element         </TH><TH> Description                                                                         </TH></TR>
//! <TR><TD id="st_t"> ULLONG             </TD><TD id="st_e"> uid             </TD><TD> Unique identifier of connection.                                                    </TD></TR>
//! <TR><TD id="st_t"> WCHAR16            </TD><TD id="st_e"> user            </TD><TD> Login user name of connection.                                                      </TD></TR>
//! <TR><TD id="st_t"> WCHAR16            </TD><TD id="st_e"> uri             </TD><TD> URI of client of connection (e.g. "mlpi.tcp://'IP-address':'port'").                </TD></TR>
//! <TR><TD id="st_t"> WCHAR16            </TD><TD id="st_e"> name            </TD><TD> User-defined name of connection.                                                    </TD></TR>
//! <TR><TD id="st_t"> WCHAR16            </TD><TD id="st_e"> label           </TD><TD> User-defined label of connection.                                                   </TD></TR>
//! <TR><TD id="st_t"> MlpiDateAndTime    </TD><TD id="st_e"> dateTime        </TD><TD> Buildup system date and time (broken down time, UTC) of connection.                 </TD></TR>
//! <TR><TD id="st_t"> ULONG              </TD><TD id="st_e"> requestCounter  </TD><TD> Absolute number of requests.                                                        </TD></TR>
//! <TR><TD id="st_t"> ULONG              </TD><TD id="st_e"> lastRequest     </TD><TD> Elapsed time since last access in milliseconds.                                     </TD></TR>
//! <TR><TD id="st_t"> MlpiApiProtection  </TD><TD id="st_e"> protection      </TD><TD> Protection status of connection.                                                    </TD></TR>
//! <TR><TD id="st_t"> ULONG              </TD><TD id="st_e"> watchdog        </TD><TD> Watchdog status of connection (==1 if watchdog is active (enabled and not fired)).  </TD></TR>
//! </TABLE>
typedef struct MlpiConnectionDescription
{
  ULLONG              uid;                                    //!< Unique identifier of connection.
  WCHAR16             user[MLPI_API_CONNECTION_USER_LEN];     //!< Login user name of connection.
  WCHAR16             uri[MLPI_API_CONNECTION_URI_LEN];       //!< URI of client of connection (e.g. "mlpi.tcp://'IP-address':'port'").
  WCHAR16             name[MLPI_API_CONNECTION_NAME_LEN];     //!< User-defined name of connection.
  WCHAR16             label[MLPI_API_CONNECTION_LABEL_LEN];   //!< User-defined label of connection.
  MlpiDateAndTime     dateTime;                               //!< Buildup system date and time (broken down time, UTC) of connection.
  ULONG               requestCounter;                         //!< Absolute number of requests.
  ULONG               lastRequest;                            //!< Elapsed time since last access in milliseconds.
  MlpiApiProtection   protection;                             //!< Protection status of connection.
  ULONG               watchdog;                               //!< Watchdog status of connection (==1 if watchdog is active (enabled and not fired)).
} MlpiConnectionDescription;

//! @typedef MlpiLibrary
//! This structure is used by the function @ref mlpiApiGetLibrarySupport to return information about available libraries.
//! @details Elements of struct MlpiLibrary
//! <TABLE>
//! <TR><TH>           Type     </TH><TH>           Element         </TH><TH> Description                           </TH></TR>
//! <TR><TD id="st_t"> WCHAR16  </TD><TD id="st_e"> name            </TD><TD> Name of MLPI library.                 </TD></TR>
//! <TR><TD id="st_t"> ULONG    </TD><TD id="st_e"> id              </TD><TD> ID of MLPI library.                   </TD></TR>
//! </TABLE>
typedef struct MlpiLibrary
{
  WCHAR16 name[MLPI_API_LIBRARY_NAME_LEN];      //!< Name of MLPI library.
  ULONG   id;                                   //!< ID of MLPI library.
} MlpiLibrary;


#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup ApiLibStructTypes


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


//! @ingroup ApiLibConnect
//! This function connects the user application with a specified MLC/MLP/XLC.
//! The first argument of the connect ident string is used to specify the target device to which you want to connect.
//! This can either be the same physical device as your application or a device that is connected
//! via a network. In this case you have to give the IP or host name to the target. As an alternative,
//! this address argument can be set as an option on each place within the connect ident string.
//!
//! Additionally, the communication to the target can be insecured (MLPI) or secured (MLPIS). @ref InsecCommunication 
//! is achieved with the use of the TCP/IP protocol that allows to establish a connection with the target device and 
//! the information exchanged within this connection is unencrypted. @ref SecCommunication is achieved with the use
//! of the TLS/SSL protocol that allows to encrypt the information exchanged. See @ref sec_Communication for more 
//! information about these two communication protocols.
//!
//! To establish an MLPIS connection the arguments <b>'tls'</b> needs to be specified (as seen in the table below),
//! if it is not given, then non secure MLPI connection is used by default. The client is not required to provide credentials (i.e. 
//! private key and certificate), however, the server must provide them. These credentials are automatically generated
//! in targets with Firmware versions starting with version 14V18. They can also be replaced by other credentials as specified in @ref SecCommunication.
//!
//! On a successful connect, the function returns a connection handle on the second argument. Use this connection
//! handle for subsequent calls to mlpi functions.
//!
//! @par Example using TCP/IP communication (MLPI):
//! @code
//! MLPIHANDLE connection = 0;
//! MLPIRESULT result = mlpiApiConnect(L"192.168.0.42", &connection);
//! if (MLPI_FAILED(result)){
//!   printf("\nfailed to connect to MLPI with 0x%08x", (unsigned) result);
//!   return;
//! }else {
//!   printf("\nsuccessfully connected!");
//! }
//! @endcode
//!
//! It is also possible to set additional options in the connect string as an argument list.
//! <TABLE>
//! <TR><TH> Option                                   </TH><TH> Description                                                                                                                 </TH></TR>
//! <TR><TD> @c @b user=value                         </TD><TD> Login name of user. See @ref sec_Permission for more information about user and permission system.                          </TD></TR>
//! <TR><TD> @c @b password=value                     </TD><TD> Password of user. See @ref sec_Permission for more information about user and permission system.                            </TD></TR>
//! <TR><TD> @c @b address=value                      </TD><TD> IP or URL path of connection.                                                                                               </TD></TR>
//! <TR><TD> @c @b timeout_connect=value              </TD><TD> Timeout used for connecting in milliseconds (infinite: MLPI_INFINITE).                                                      </TD></TR>
//! <TR><TD> @c @b timeout_send=value                 </TD><TD> Timeout of MLPI client used for sending data to server (target) in milliseconds (default: 0, infinite: MLPI_INFINITE).      </TD></TR>
//! <TR><TD> @c @b timeout_receive=value              </TD><TD> Timeout of MLPI client used for receiving data from server (target) in milliseconds (default: 0, infinite: MLPI_INFINITE).  </TD></TR>
//! <TR><TD> @c @b auto_reconnect=value               </TD><TD> If set to 1 or 'true', then the MLPI will try to reconnect with each new call
//!                                                             of any MLPI function after connection was lost. For connecting, the same timeout
//!                                                             settings are used as given for the first connect (default: false).                                                          </TD></TR>
//! <TR><TD> @c @b keepalive_mode_server=value        </TD><TD> If set to 1 (default, isn't set), then the MLPI server will send a MLPI keepalive
//!                                                             telegram after keepalive timeout, if the client does not send any request before.
//!                                                             If set to 0, then MLPI server will not send any MLPI keepalive telegrams.                                                   </TD></TR>
//! <TR><TD> @c @b keepalive_timeout_server=value     </TD><TD> Timeout used for sending MLPI keepalive telegram in milliseconds (default: 30000).                                          </TD></TR>
//! <TR><TD> @c @b keepalive_probes_server=value      </TD><TD> Number of probes for sending MLPI keepalive telegram (default: 10).                                                         </TD></TR>
//! <TR><TD> @c @b timeout_send_server=value          </TD><TD> Timeout of MLPI server used for sending data to client in milliseconds (default: 60000, infinite: MLPI_INFINITE).           </TD></TR>
//! <TR><TD> @c @b require_hash=value                 </TD><TD> If true, then only logins to servers which support hashed password logins are allowed. Server version
//!                                                             has to be greater than 1.1.1.0 (default: false).
//!                                                             @note It's highly recommended to use this option if your control firmware corresponds to or it newer than 13V06.            </TD></TR>
//! <TR><TD> @c @b protection=value                   </TD><TD> Select your protection level using values of @ref MlpiApiProtection to protect your connection
//!                                                             against closing by an other connection with the assistance of call of @ref mlpiApiCloseConnectionByUid,
//!                                                             @ref mlpiApiCloseConnectionsByUser or @ref mlpiApiCloseConnectionsByUri.
//!                                                             @note This option can be also used as an attribute of an account of the user management.
//!                                                             See @ref sec_Permission for more information about user and permission system.
//!                                                             \n@arg @c @b value==0 The connection is not protected and can be closed by an other connection (default).
//!                                                             \n@arg @c @b value==1 The connection is protected against closing by an other connection if the
//!                                                                                   connection watchdog is active (enabled and not fired).
//!                                                             \n@arg @c @b value==2 The connection is protected against closing by an other connection completely.                        </TD></TR>
//! <TR><TD> @c @b tls=value                          </TD><TD>If set to 'active', then MLPI is tunneled through TLS rather than TCP, thus providing data confidentiality and integrity for
//!                                                            the connection (default: false). It is highly recommended to change this setting to true if possible.                        </TD></TR>
//! </TABLE>
//!
//!
//! @par Example using TCP/IP communication (MLPI):
//! @code
//! MLPIHANDLE connection = 0;
//! MLPIRESULT result = mlpiApiConnect(L"192.168.0.42 -timeout_connect=2000 -timeout_send=5000 -timeout_receive=5000 -user=guest -password=guest -auto_reconnect=true", &connection);
//! if (MLPI_FAILED(result)){
//!   printf("\nfailed to connect to MLPI with 0x%08x", (unsigned)result);
//!   return;
//! }else{
//!   printf("\nsuccessfully connected!");
//! }
//! @endcode
//!
//! @par Example using TLS/SSL communication (MLPIS):
//! @code
//! MLPIHANDLE connection = 0;
//! MLPIRESULT result = mlpiApiConnect(L"192.168.0.42 -timeout_connect=2000 -timeout_send=5000 -timeout_receive=5000 -user=guest -password=guest -auto_reconnect=true 
//! -tls=active", &connection);
//! if (MLPI_FAILED(result)){
//!   printf("\nfailed to connect to MLPI with 0x%08x", (unsigned)result);
//!   return;
//! }else{
//!   printf("\nsuccessfully connected!");
//! }
//! @endcode
//!
//! A timeout value of 0 means that the timeout value is to be chosen automatically by the operating system. If no
//! timeout is specified here, the global default value as set by the function @ref mlpiApiSetDefaultTimeout is used.
//! This means that any option given here will override the value as set by @ref mlpiApiSetDefaultTimeout.
//! In most cases it is not recommended to override the timeout values. Especially the send and receive timeouts should
//! stay at their default values of 0.
//!
//! The second argument is handy if you want to establish more than one connection using the MLPI. For example, if
//! you want to connect to two or more devices at the same time. Or if you want to have more than a single connection
//! to the same device. As already noted, you have to pass a pointer to a @c MLPIHANDLE variable as second argument to
//! this function. If the function succeeds, it will return a handle which identifies your connection. Use this handle
//! as first argument to all existing MLPI functions. This way, the MLPI function  knows on which connection and
//! therefore on which target to execute the MLPI function.
//!
//! If no request telegram of the MLPI client will be received by the MLPI server, by default, the MLPI server will send
//! on any <b>'keepalive_timeout_server'</b> milliseconds timeout an empty MLPI keepalive telegram. This is done to check
//! if the client machine is still reachable or if the connection can be closed and freed again. The server will repeat
//! this keepalive telegram for <b>'keepalive_probes_server'</b> times. After this time
//! (<b>'keepalive_timeout_server' * 'keepalive_probes_server'</b>), the MLPI server waits <b>'timeout_send_server'</b>
//! milliseconds until the connection is closed finally.
//!
//! @par Overall timeout until close of connection:
//! @code
//! t(close) = ( t(keepalive_timeout_server) * n(keepalive_probes_server) ) + t(timeout_send_server)
//! @endcode
//!
//! @note  The server cannot detect whether the client application is still running. It can only be checked if the telegrams
//!        reached the client machine and the socket is still available. To setup a durable connection, it can also be helpful
//!        to implement the function @ref mlpiApiNotifyAlive.
//!
//! @par Example using TCP/IP (MLPI) and TLS/SSL communications (MLPIS):
//! @code
//! MLPIHANDLE handle1=0;                                     // this is the handle of the first connection
//! MLPIHANDLE handle2=0;                                     // this is the handle of the second connection
//! mlpiApiConnect(L"192.168.0.17", &handle1);
//! mlpiApiConnect(L"192.168.0.42 -tls=active", &handle2);
//!
//! WCHAR szText1[128];
//! WCHAR szText2[128];
//! mlpiSystemGetName(handle1, szText1, _countof(szText1));   // now using the handle to identify the first target
//! mlpiSystemGetName(handle2, szText2, _countof(szText2));   // now using the handle to identify the second target
//! printf("\nName1: %s", W2A16(szText1));
//! printf("\nName2: %s", W2A16(szText2));
//!
//! mlpiApiDisconnect(&handle1);
//! mlpiApiDisconnect(&handle2);
//! @endcode
//!
//!
//! @par About the return value.
//! A few words on the return values of the @ref mlpiApiConnect() function.
//! <TABLE>
//! <TR><TH>           Use case                                                                                                                                 </TH><TH>           Return value                      </TH></TR>
//! <TR><TD id="st_t"> Successful connect to a MLPI device.                                                                                                     </TD><TD id="st_e"> Positive value.                   </TD></TR>
//! <TR><TD id="st_t"> The given connection ident string doesn't contain an address.                                                                            </TD><TD id="st_e"> MLPI_E_INVALIDARG                 </TD></TR>
//! <TR><TD id="st_t"> No MLPI device found at the given address, e.g. invalid IP address.                                                                      </TD><TD id="st_e"> MLPI_E_CONNECTFAILED              </TD></TR>
//! <TR><TD id="st_t"> A connection could not be established in the given timeout time as provided by @ref mlpiApiSetDefaultTimeout or "-timeout_connect=xxxx". </TD><TD id="st_e"> MLPI_E_TIMEOUT                    </TD></TR>
//! <TR><TD id="st_t"> MLPI device found, but connection refused because of incompatible version of client library and mlpi device.                             </TD><TD id="st_e"> MLPI_E_VERSION                    </TD></TR>
//! <TR><TD id="st_t"> MLPI device found, but connection refused because of missing privileges or invalid user/password settings.                               </TD><TD id="st_e"> MLPI_E_PERMISSION                 </TD></TR>
//! <TR><TD id="st_t"> MLPI device found, but connection refused because there are already too much clients or user connected.                                  </TD><TD id="st_e"> MLPI_E_LIMIT_MAX                  </TD></TR>
//! <TR><TD id="st_t"> A connection could not be established because there was an error with the provided private key.                                          </TD><TD id="st_e"> MLPI_E_INVALID_PRIVATEKEY_FILE    </TD></TR>
//! <TR><TD id="st_t"> A connection could not be established because there was an error with the provided certificate.                                          </TD><TD id="st_e"> MLPI_E_INVALID_CERTIFICATE_FILE   </TD></TR>
//! <TR><TD id="st_t"> A connection could not be established because the private key and certificate do not match one another.                                  </TD><TD id="st_e"> MLPI_E_KEYPAIR_MISSMATCH          </TD></TR>
//! <TR><TD id="st_t"> A connection could not be established because there was an error with the TLS handshake.                                                 </TD><TD id="st_e"> MLPI_E_TLS_HANDSHAKE_FAILED       </TD></TR>
//! </TABLE>
//!
//! A call to any other MLPI function prior to calling @ref mlpiApiConnect will return @c MLPI_E_NOCONNECTION. If a send
//! or receive timeout is set and a MLPI function call times out due to these settings, then this MLPI function call will
//! return @c MLPI_E_TIMEOUT. The connection is then inconsistent and will therefore be closed automatically. Any following
//! MLPI function call will then return with @c MLPI_E_NOCONNECTION. This means that after a timeout, a new connection
//! has to be established using @ref mlpiApiConnect unless you made the connect with optional argument
//! @c auto_reconnect=true (see above). In this case, the MLPI tries to reconnect in the context of each new MLPI function
//! call.
//!
//! @param[in]  connectionIdentifier  The connect ident string identifying the device. Use MLPI_LOCALHOST to connect to
//!                                   the system running on the same host device. Use an IP address to connect
//!                                   over Ethernet to another device. This string may also contain an additional argument
//!                                   list (options).
//! @param[out] connection            Returns handle of connection, if successful. Has to be canceled by calling @ref mlpiApiDisconnect.
//!                                   Calling @ref mlpiApiConnect without corresponding @ref mlpiApiDisconnect on application termination
//!                                   will result in a memory leak!
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiApiConnect(const WCHAR16* connectionIdentifier, MLPIHANDLE* connection);


//! @ingroup ApiLibConnect
//! This function disconnects the user application from the target.
//!
//! The connection identified by the handle is closed and destroyed. It is then not allowed to use the handle in any subsequent MLPI call.
//!
//! @param[in]  connection            Handle for multiple connections.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example using TCP/IP communication (MLPI):
//! @code
//! MLPIHANDLE connection = 0;
//! MLPIRESULT result = mlpiApiConnect(L"192.168.0.10:5300 -timeout_connect=2000 -timeout_send=5000 -timeout_receive=5000 -user=guest -pass=guest -auto_reconnect=true", &connection);
//! if (MLPI_FAILED(result)){
//!   printf("\nfailed to connect to MLPI with 0x%08x", (unsigned)result);
//!   return;
//! }else{
//!   printf("\nsuccessfully connected!");
//! }
//!
//! // insert some mlpi calls here...
//!
//! result = mlpiApiDisconnect(&connection);
//! if (MLPI_FAILED(result)){
//!   printf("\nerror on disconnect from MLPI with 0x%08x", (unsigned)result);
//!   return;
//! }else{
//!   printf("\nsuccessfully disconnected!");
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiDisconnect(MLPIHANDLE *connection);


//! @ingroup ApiLibConnect
//! This function sets the default timeout for remote procedure calls done by the API. The timeout is used
//! for connecting, sending and receiving data. During the debugging of your application, you might want to set this
//! value to @c MLPI_INFINITE.
//! @note
//! @li This value has to be set BEFORE connecting to a target.
//! @li This value can be overridden by options which are given as an argument list to mlpiApiConnect.
//! @param[in]  timeout               This is the timeout in milliseconds when doing remote procedure calls via the MLPI-API.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiApiSetDefaultTimeout(const ULONG timeout);


//! @ingroup ApiLibConnect
//! This function reads the default timeout for remote procedure calls done by the API. This might not be the value
//! that is set for a currently active connection.
//! @param[out]  timeout              This is the timeout in milliseconds when doing remote procedure calls via the MLPI-API.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiApiGetDefaultTimeout(ULONG *timeout);


//! @ingroup ApiLibConnect
//! This function returns the current state of the MLPI connection.
//! If @c FALSE is returned, then the connection is either not yet established or closed because of an error in
//! communication or because @ref mlpiApiDisconnect has been called.
//! In all cases a reconnect using @ref mlpiApiConnect has to be made to make new MLPI function calls.
//! If connection is lost for unknown reason, try increasing the connection timeout using @ref mlpiApiSetDefaultTimeout.
//! @param[in]  connection            Handle for multiple connections.
//! @param[out] isConnected           Returns TRUE if connection is established and FALSE if disconnected.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! BOOL8 isConnected = FALSE;
//! MLPIRESULT result = mlpiApiIsConnected(connection, &isConnected);
//!
//! if (MLPI_SUCCEEDED(result) && isConnected) {
//!   printf("\nMLPI connection is established!");
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiIsConnected(const MLPIHANDLE connection, BOOL8 *isConnected);


//! @ingroup ApiLibConnect
//! This function performs a benchmark on the MLPI communication mechanism.
//! Use it to measure the duration of a MLPI function call with the given payload in bytes.
//! The resulting timing values are the durations which are necessary to marshal the payload, send the payload to
//! the MLPI server, unmarshal it on the MLPI server, perform the remote procedure call, marshal the results,
//! send them back to the MLPI client unmarshal them again and return them to the client thread program.
//! These are the basic costs needed for nearly every MLPI call.
//! @param[in]  connection            Handle for multiple connections.
//! @param[in]  payload               Payload in number of bytes that is used for the MLPI communication to be measured.
//! @param[in]  numMeasurements       Number of measurements to do for calculating the average resulting timing values.
//! @param[out] info                  Pointer to structure which receives the calculated timing values.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG payload = 100;              // 100 byte transfer data
//! ULONG numMeasurements = 1000;     // 1000 iterations for statistics
//! MlpiConnectionInfo connectionInfo = {0};
//!
//! // do measurements
//! MLPIRESULT result = mlpiApiTestConnection(connection, payload, numMeasurements, &connectionInfo);
//! if (MLPI_FAILED(result)){
//!   printf("mlpiApiTestConnection failed with 0x%08x", (unsigned)result);
//!   return;
//! }
//! // print results
//! printf(L"\nTimings in milliseconds:");
//! printf(L"\nPayload: \t\t%7d", payload);
//! printf(L"\nAverage: \t\t%15.6lf", connectionInfo.average*0.001);
//! printf(L"\nMinimum: \t\t%15.6lf", connectionInfo.minimum*0.001);
//! printf(L"\nMaximum: \t\t%15.6lf", connectionInfo.maximum*0.001);
//! printf(L"\nVariance: \t\t%15.6lf", connectionInfo.variance*0.001*0.001);
//! printf(L"\nStandard Deviation: \t\t%15.6lf", connectionInfo.standardDeviation*0.001);
//! @endcode
MLPI_API MLPIRESULT mlpiApiTestConnection(const MLPIHANDLE connection, const ULONG payload, const ULONG numMeasurements, MlpiConnectionInfo *info);


//! @ingroup ApiLibConnect
//! This function notifies the MLPI server that the client is still alive. You may need this function if you use the
//! keepalive mechanism of the MLPI server (@ref mlpiApiConnect).
//!
//! @note By using of keepalive options, the server will send keepalive telegrams to the client until the server recognizes
//!       a zombie connection. This is because the keepalive telegrams of the MLPI server also fills the receive buffer
//!       of the client until the client sends any new request or until the client buffer is full and causes an error. To
//!       prevent this error and empty the client buffer from keepalive telegrams, use this or any other function.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // notify MLPI server that the client is still alive
//! MLPIRESULT result = mlpiApiNotifyAlive(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiNotifyAlive(const MLPIHANDLE connection);


//! @ingroup ApiLibUtility
//! This function does a simple delay on the server side.
//! All this function performs is a blocking delay on the server side in the given amount of milliseconds. Use it during
//! development or debugging of your application to simulate high traffic on your network. This way, you can test the
//! behavior of your client application when MLPI communication slows down.
//! The delay has no impact on the performance of the device.
//! @param[in]  connection            Handle for multiple connections.
//! @param[in]  delayMilliseconds     Delay in milliseconds to block.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiApiDelay(const MLPIHANDLE connection, const ULONG delayMilliseconds);


//! @ingroup ApiLibUtility
//! This function returns the version info of the MLPI client library.
//! @param[out] versionInfo           Pointer to struct receiving the version info.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiVersion versionInfo;
//! memset(&versionInfo, 0, sizeof(versionInfo));
//! MLPIRESULT result = mlpiApiGetClientCoreVersion(&versionInfo);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! } else {
//!   printf("\nVersion (Major, Minor, Bugfix, Patch): %d.%d.%d.%d Build: %d"
//!     , versionInfo.major
//!     , versionInfo.minor
//!     , versionInfo.bugfix
//!     , versionInfo.patch
//!     , versionInfo.build);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetClientCoreVersion(MlpiVersion *versionInfo);


//! @ingroup ApiLibUtility
//! This function returns the version info of the MLPI server library.
//! @param[in]  connection            Handle for multiple connections.
//! @param[out] versionInfo           Pointer to struct receiving the version info.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiVersion versionInfo;
//! memset(&versionInfo, 0, sizeof(versionInfo));
//! MLPIRESULT result = mlpiApiGetServerCoreVersion(connection, &versionInfo);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! } else {
//!   printf("\nVersion (Major, Minor, Bugfix, Patch): %d.%d.%d.%d Build: %d"
//!     , versionInfo.major
//!     , versionInfo.minor
//!     , versionInfo.bugfix
//!     , versionInfo.patch
//!     , versionInfo.build);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetServerCoreVersion(const MLPIHANDLE connection, MlpiVersion *versionInfo);


//! @ingroup ApiLibUtility
//! Using this function, you can assign a short descriptive name to your connection. It is not necessary to
//! give your connection a name, but the name might be useful for debugging and maintaining your connections.
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  name              User-defined name of connection.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // let's call our connection 'The Hitchhiker's Guide to the Galaxy.'
//! MLPIRESULT result = mlpiApiSetNameOfConnection(connection, L"The Hitchhiker's Guide to the Galaxy.");
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiSetNameOfConnection(const MLPIHANDLE connection, const WCHAR16 *name);


//! @ingroup ApiLibUtility
//! Using this function, you can assign a long descriptive label to your connection. It is not necessary to
//! set the connection label, but the label might be useful for debugging and maintaining your connections.
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  label             User-defined label of connection.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // let's label our connection with 'The answer to the Ultimate Question of Life, the Universe, and Everything is... 42.'
//! MLPIRESULT result = mlpiApiSetLabelOfConnection(connection, L"The answer to the Ultimate Question of Life, the Universe, and Everything is... 42.");
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiSetLabelOfConnection(const MLPIHANDLE connection, const WCHAR16 *label);


//! @ingroup ApiLibUtility
//! This function will return information about the own connection.
//! @param[in]  connection        Handle for multiple connections.
//! @param[out] description       Pointer to struct receiving the connection descriptions.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiConnectionDescription description;
//! memset(&description, 0, sizeof(description));
//!
//! // get information about all connections
//! MLPIRESULT result = mlpiApiGetOwnConnectionDescription(connection, description);
//! if(MLPI_SUCCEEDED(result)) {
//!   printf("\n\n* %d", i);
//!   printf("\n\tuid: 0x%08X.%08X", (ULONG) (description.uid>>32), (ULONG) description.uid);
//!   printf("\n\tuser: %s", W2A16(description.user));
//!   printf("\n\turi: %s", description.uri);
//!   printf("\n\tname: %s", W2A16(description.name));
//!   printf("\n\tlabel: %s", W2A16(description.label));
//!   printf("\n\tdate an time: %04u-%02u-%02u %02u-%02u-%02u (UTC)",
//!     description.dateTime.year, description.dateTime.month, description.dateTime.day,
//!     description.dateTime.hour, description.dateTime.minute, description.dateTime.second);
//!   printf("\n\trequests: %d", description.requestCounter);
//!   printf("\n\tlast request: %d", description.lastRequest);
//!   printf("\n\tprotection: %d", description.protection);
//!   printf("\n\twatchdog: %s",  (description.watchdog==1) ? "TRUE" : "FALSE");
//! }
//! else {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetOwnConnectionDescription(const MLPIHANDLE connection, MlpiConnectionDescription *description);


//! @ingroup ApiLibUtility
//! This function will return information about all established connections of the device.
//! @param[in]  connection        Handle for multiple connections.
//! @param[out] description       Pointer to struct receiving the connection descriptions.
//! @param[in]  numElements       Number of MlpiConnectionDescription elements available in 'description' to read.
//! @param[out] numElementsRet    Number of elements used.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numElementsRet = 0;
//! MlpiConnectionDescription description[MLPI_API_MAX_NUMBER_OF_CONNECTIONS];
//! memset(description, 0, sizeof(description));
//!
//! // get information about all connections
//! MLPIRESULT result = mlpiApiGetAllConnectionDescription(connection, description, MLPI_API_MAX_NUMBER_OF_CONNECTIONS, &numElementsRet);
//! if(MLPI_SUCCEEDED(result)) {
//!   for(ULONG i=0; i<numElementsRet; i++) {
//!     printf("\n\n* %d", i);
//!     printf("\n\tuid: 0x%08X.%08X", (ULONG) (description[i].uid>>32), (ULONG) description[i].uid);
//!     printf("\n\tuser: %s", W2A16(description[i].user));
//!     printf("\n\turi: %s", description[i].uri);
//!     printf("\n\tname: %s", W2A16(description[i].name));
//!     printf("\n\tlabel: %s", W2A16(description[i].label));
//!     printf("\n\tdate an time: %04u-%02u-%02u %02u-%02u-%02u (UTC)",
//!       description[i].dateTime.year, description[i].dateTime.month, description[i].dateTime.day,
//!       description[i].dateTime.hour, description[i].dateTime.minute, description[i].dateTime.second);
//!     printf("\n\trequests: %d", description[i].requestCounter);
//!     printf("\n\tlast request: %d", description[i].lastRequest);
//!     printf("\n\tprotection: %d", description[i].protection);
//!     printf("\n\twatchdog: %s",  (description[i].watchdog==1) ? "TRUE" : "FALSE");
//!   }
//! }
//! else {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetAllConnectionDescription(const MLPIHANDLE connection, MlpiConnectionDescription *description, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ApiLibUtility
//! This function closes a connection selected by the unique identifier (uid) of a connection. You
//! can determine the uid by using the function @ref mlpiApiGetOwnConnectionDescription.
//!
//! @note A connection cannot be closed if it runs in a protected mode (see @ref MlpiApiProtection).
//!       In this case, the function will return the error @ref MLPI_E_RD_WR_PROTECTION.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  uid               Unique identifier of connection.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // close the connection with the uid 0x0000002A.12345678
//! MLPIRESULT result = mlpiApiCloseConnectionByUid(connection, 0x0000002A.12345678);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiCloseConnectionByUid(const MLPIHANDLE connection, const ULLONG uid);


//! @ingroup ApiLibUtility
//! This function closes one or multiple connections selected by the user.
//!
//! @note A connection cannot be closed if it runs in a protected mode (see @ref MlpiApiProtection).
//!       In this case, the function will return the error @ref MLPI_E_RD_WR_PROTECTION.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  user              Login user name of connection.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // close all connections of the user 'Batman'
//! MLPIRESULT result = mlpiApiCloseConnectionsByUser(connection, L"Batman");
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiCloseConnectionsByUser(const MLPIHANDLE connection, const WCHAR16 *user);


//! @ingroup ApiLibUtility
//! This function closes one or multiple connections selected by a combination of the uniform
//! resource identifier (uri) and the placeholder '*'.
//!
//! @note A connection cannot be closed if it runs in a protected mode (see @ref MlpiApiProtection).
//!       In this case, the function will return the error @ref MLPI_E_RD_WR_PROTECTION.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  uri               URI of connection (e.g. "mlpi.tcp://'IP-address':'port'").
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // close all connections of the device with the uri 'mlpi.tcp://192.168.0.42*'
//! MLPIRESULT result = mlpiApiCloseConnectionsByUri(connection, L"mlpi.tcp://192.168.0.42*");
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiCloseConnectionsByUri(const MLPIHANDLE connection, const WCHAR16 *uri);


//! @ingroup ApiLibUtility
//! This function provides the reloading of all currently loaded account manifests
//! know as 'accounts.xml'.
//!
//! @note The permissions of an established connection will not be influenced from new
//!       accounts settings.
//!
//! @attention A fail of reloading of the default account manifests maybe requires a reboot
//!            of the control because another new connection cannot be established furthermore.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // reload accounts of default account manifests and additional account manifests
//! MLPIRESULT result = mlpiApiUserAccountControlReload(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiUserAccountControlReload(const MLPIHANDLE connection);


//! @ingroup ApiLibUtility
//!
//! @note <b><span style="color:red">This function is deprecated since server version 1.19.0. Do not use this function in combination with the
//!       @ref AccessControlLib!</span></b>
//!
//! This function provides the loading of the accounts of an additional account manifest.
//!
//! @note It's recommended, but not required to name the additional account manifest
//!       'accounts.xml'.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  path              Path to the additional account manifest.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // load accounts of an additional account manifest
//! WCHAR16 path[256] = L"";
//! WCHAR16 file[] = L"bundles/com.boschrexroth.mlpi.test42/accounts.xml";
//! if ( MLPI_SUCCEEDED(mlpiSystemGetSpecialPath(MLPI_PATH_OEM, path, _countof(path))) )
//! {
//!   wcscat(path, file);
//!   MLPIRESULT result = mlpiApiUserAccountControlLoadAccounts(connection, path);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!     return result;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiUserAccountControlLoadAccounts(const MLPIHANDLE connection, const WCHAR16 *path);


//! @ingroup ApiLibUtility

//! @note <b><span style="color:red">This function is deprecated since server version 1.19.0. Do not use this function in combination with the
//!       @ref AccessControlLib!</span></b>
//!
//!
//! This function provides the unloading of the accounts of an additional account manifest.
//!
//!
//! @note The permissions of an established connection will not be influenced from new
//!       accounts settings.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  path              Path to the additional account manifest.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // unload accounts of an additional account manifest
//! WCHAR16 path[256] = L"";
//! WCHAR16 file[] = L"bundles/com.boschrexroth.mlpi.test42/accounts.xml";
//! if ( MLPI_SUCCEEDED(mlpiSystemGetSpecialPath(MLPI_PATH_OEM, path, _countof(path))) )
//! {
//!   wcscat16(path, file);
//!   MLPIRESULT result = mlpiApiUserAccountControlUnloadAccounts(connection, path);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!     return result;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiUserAccountControlUnloadAccounts(const MLPIHANDLE connection, const WCHAR16 *path);


//! @ingroup ApiLibUtility
//! This function provides the reading of own permissions based on account manifest 'accounts.xml'.
//!
//! @note In case the buffer is too short, the function will still return the required number of
//!       WCHAR16 elements by 'numElementsRet'.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[out] permissions       String where the permissions will be stored.
//! @param[in]  numElements       Number of WCHAR16 elements available in 'permissions' to read.
//! @param[out] numElementsRet    Number of WCHAR16 elements in complete 'permissions'.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read own permissions
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
//!   ULONG idx = 0;
//!   CHAR permission[128] = "";
//!   WCHAR16 *curr = permissions, *nxt = permissions;
//!   while (true)
//!   {
//!     // find separator between two permissions or end of string
//!     while ( (*nxt!=NULL) && (*nxt!=';') )
//!       nxt++;
//!     if(curr==nxt)
//!       break;
//!     // separate current permission string from next permission string
//!     if(*nxt!=0)
//!       *nxt++=0;
//!     // convert and print permission string
//!     wcstombs16(permission, curr, wcslen16(curr)+1);
//!     printf("\n%d:\t%s", idx++, permission);
//!     // go to next permission string
//!     curr = nxt;
//!   }
//! }
//! @endcode
//!
//! @par Example:
//! @code
//! #include "mlpiApiHelper.h"
//!
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
//!   WCHAR16 requiredPermission[] = L"MLPI_APILIB_PERMISSION_CONNECTION_CLOSE";
//!
//!   MlpiApiPermissionEvaluation evalPermission(permissions);
//!
//!   if (evalPermission.hasPermission(requiredPermission))
//!     log(L"\nThe logged in user has the permission '%s'.", requiredPermission);
//!   else
//!     warning(L"\nThe logged in user hasn't the permission '%s'.", requiredPermission);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetOwnPermissions(const MLPIHANDLE connection, WCHAR16 *permissions, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ApiLibUtility
//!
//! @note <b><span style="color:red">This function is deprecated since server version 1.19.0. Use the function @ref mlpiAccessControlGetAllUserInfos from the 
//!       @ref AccessControlLib instead!</span></b>
//!
//! This function provides the reading of all available accounts based on account manifest 'accounts.xml'.
//!
//! @note An account with empty name will be replaced by the surrogate string @ref MLPI_ACCOUNT_EMPTY_NAME but
//!       this surrogate string cannot used as login name!
//!
//! @par Note: In case the buffer is too short, the function will still return the required number of WCHAR16
//!            elements by 'numElementsRet'.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[out] accounts          String where the accounts will be stored.
//! @param[in]  numElements       Number of WCHAR16 elements available in 'accounts' to read.
//! @param[out] numElementsRet    Number of WCHAR16 elements in complete 'accounts'.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read all available accounts
//! ULONG numElementsRet = 0;
//! WCHAR16 accounts[1024] = L"";
//!
//! MLPIRESULT result = mlpiApiGetAccounts(connection, accounts, _countof(accounts), &numElementsRet);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! else
//! {
//!   ULONG idx = 0;
//!   CHAR account[128] = "";
//!   WCHAR16 *curr = accounts, *nxt = accounts;
//!   while (true)
//!   {
//!     // find separator between two accounts or end of string
//!     while ( (*nxt!=NULL) && (*nxt!=';') )
//!       nxt++;
//!     if(curr==nxt)
//!       break;
//!     // separate current account string from next account string
//!     if (*nxt!=0)
//!       *nxt++=0;
//!     // remove surrogate string of account with empty name
//!     if(wcscmp16(curr, MLPI_ACCOUNT_EMPTY_NAME)==0)
//!       *curr = 0;
//!     // convert account string
//!     wcstombs16(account, curr, wcslen16(curr)+1);
//!     printf("\n%d:\t%s", idx++, account);
//!     // go to next account string
//!     curr = nxt;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetAccounts(const MLPIHANDLE connection, WCHAR16 *accounts, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ApiLibUtility
//!
//! This function provides the reading of the permissions of an account based on account manifest 'accounts.xml'.
//!
//! @note In case the buffer is too short, the function will still return the required number of
//!       WCHAR16 elements by 'numElementsRet'.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  account           String that identifies the account for which permissions were read. Set to NULL to get your own permissions.
//! @param[out] permissions       String where the permissions will be stored.
//! @param[in]  numElements       Number of WCHAR16 elements available in 'permissions' to read.
//! @param[out] numElementsRet    Number of WCHAR16 elements in complete 'permissions'.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read permissions of account 'administrator'
//! ULONG numElementsRet = 0;
//! WCHAR16 permissions[4096] = L"";
//!
//! MLPIRESULT result = mlpiApiGetAccountPermissions(connection, L"administrator", permissions, _countof(permissions), &numElementsRet);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! else
//! {
//!   ULONG idx = 0;
//!   CHAR permission[128] = "";
//!   WCHAR16 *curr = permissions, *nxt = permissions;
//!   while (true)
//!   {
//!     // find separator between two permissions or end of string
//!     while ( (*nxt!=NULL) && (*nxt!=';') )
//!       nxt++;
//!     if(curr==nxt)
//!       break;
//!     // separate current permission string from next permission string
//!     if (*nxt!=0)
//!       *nxt++=0;
//!     // convert and print permission string
//!     wcstombs16(permission, curr, wcslen16(curr)+1);
//!     printf("\n%d:\t%s", idx++, permission);
//!     // go to next permission string
//!     curr = nxt;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetAccountPermissions(const MLPIHANDLE connection, const WCHAR16 *account, WCHAR16 *permissions, const ULONG numElements, ULONG *numElementsRet);

//! @ingroup ApiLibUtility
//! This function provides names and IDs of supported libraries.
//!
//! @param[in]  connection        Handle for multiple connections.
//! @param[out] library           Struct where names and IDs of supported libraries will be stored.
//! @param[in]  numElements       Number of MlpiLibrary elements available in 'library' to read.
//! @param[out] numElementsRet    Number of MlpiLibrary elements in complete 'library'.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numElementsRet = 0;
//! MlpiLibrary library[32];
//! memset(library, 0, sizeof(library));
//!
//! // Get names and IDs of supported libraries.
//! MLPIRESULT result = mlpiApiGetLibrarySupport(connection, library, _countof(library), &numElementsRet);
//! if(MLPI_SUCCEEDED(result)) {
//!   for(ULONG i=0; i<numElementsRet; i++) {
//!     printf("\n\n* %d", i);
//!     printf("\n\tname: %s", W2A16(library[i].name));
//!     printf("\n\tid: %08X", library[i].id);
//!   }
//! }
//! else {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiApiGetLibrarySupport(const MLPIHANDLE connection, MlpiLibrary *library, const ULONG numElements, ULONG *numElementsRet);

#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIAPILIB_H__
