#ifndef __MLPISECURITYLIB_H__
#define __MLPISECURITYLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiSecurityLib.h>
// -----------------------------------------------------------------------
// Copyright (c) 2016 Bosch Rexroth. All rights reserved.
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
//! @author     DC-IA/EAO7 (ABR, SK, JR)
//!
//! @copyright  Bosch Rexroth Corporation http://www.boschrexroth.com/oce
//!
//! @version    1.22.0
//!
//! @date       2016
//
// -----------------------------------------------------------------------

//! @addtogroup SecurityLib SecurityLib
//! @{
//! @brief This security library provides functionality in order to manage security settings. It also allows to get information 
//! information about them.
//! @attention Please consider: The SecurityLib is only useable with the TLS based mlpi-connection (mlpiS)!
//!
//! All the following functions allow to modify and obtain information regarding the IT-Security configuration of the target control. 
//! Network security is one part of IT-Security. For example, it is possible to activate and deactivate protocols like SSH.
//! 
//! @attention Please consider: Some old, insecure protocols might be activated by default! This is necessary in order to provide downward compatibility. 
//!            If they are not needed, it is strongly recommended to turn them off. 
//! @}

//! @addtogroup SecurityLibVersionPermission Version and Permission
//! @ingroup SecurityLib
//! @{
//! @brief Version and permission information
//!
//! The following table shows the requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permissions needed to execute the desired functions. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_SECURITYLIB_PERMISSION_ALL" with the value "true", all functions
//! of this library are enabled for the given user account.
//!
//! @note Functions with permission MLPI_SECURITYLIB_PERMISSION_ALWAYS cannot be blocked.
//!
//! @par List of permissions for mlpiSecurityLib that can be configured in 'accounts.xml'
//! - MLPI_SECURITYLIB_PERMISSION_ALWAYS  
//! - MLPI_SECURITYLIB_PERMISSION_CONFIG  
//! - MLPI_SECURITYLIB_PERMISSION_INFO    
//! - MLPI_SECURITYLIB_PERMISSION_ALL     
//! 
//! <TABLE>
//! <TR><TH>           Function                                         </TH><TH> Server version  </TH><TH> Permission                              </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiSecurityGetNetworkServiceInformation    </TD><TD> 1.15.0.0        </TD><TD> "MLPI_SECURITYLIB_PERMISSION_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSecurityGetNetworkServiceActivation     </TD><TD> 1.15.0.0        </TD><TD> "MLPI_SECURITYLIB_PERMISSION_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSecuritySetNetworkServiceActivation     </TD><TD> 1.15.0.0        </TD><TD> "MLPI_SECURITYLIB_PERMISSION_SETUP"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSecurityGetNetworkServiceConfiguration  </TD><TD> 1.19.0.0        </TD><TD> "MLPI_SECURITYLIB_PERMISSION_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSecuritySetNetworkServiceConfiguration  </TD><TD> 1.19.0.0        </TD><TD> "MLPI_SECURITYLIB_PERMISSION_SETUP"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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

//! @}
//! @addtogroup SecurityLibVersionStructs Structs, Types, ...
//! @ingroup SecurityLib
//! @{
//! @brief List of used types, enumerations, structures and more...

// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

#define MLPI_SECURIY_NETWORKSERVICE_MAX_NAME_LEN (16)
#define MLPI_SECURIY_MAX_NETWORKSERVICES (16)
#define MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_NAME_LEN (32)
#define MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_VALUE_LEN (128)

//! This constants defines the network services that may be available on the target.
#define MLPI_SECURITY_NETWORKSERVICE_FTP   L"FTP"      //!< File Transfer Protocol (unsafe!). The use of SFTP instead is strongly recommended!
#define MLPI_SECURITY_NETWORKSERVICE_SSH   L"SSH"      //!< Secure Shell that includes SSH File Transfer Protocol (SFTP).
#define MLPI_SECURITY_NETWORKSERVICE_MLPI  L"MLPI"     //!< Motion Logic Programming Interface (not encrypted).
#define MLPI_SECURITY_NETWORKSERVICE_MLPIS L"MLPIS"    //!< Motion Logic Programming Interface Secured (MLPI with TLS encryption). See @ref sec_Communication.
#define MLPI_SECURITY_NETWORKSERVICE_OPCUA L"OPCUA"    //!< OPC Unified Architecture.

//-----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
//-----------------------------------------------------------------------

//! @enum MlpiSecurityServiceState
//! This enumeration describes the possible states of the network services.
typedef enum MlpiSecurityServiceState
{
  MLPI_SECURITY_SERVICE_DISABLED = 0, //!< Protocol is not active.
  MLPI_SECURITY_SERVICE_ENABLED  = 1  //!< Protocol is active.
}MlpiSecurityServiceState;

//! @enum MlpiSecurityServiceControl
//! This enumeration describes the control possibilities of an specific network service.
typedef enum MlpiSecurityServiceControl
{
  MLPI_SECURITY_SERVICE_NOTCHANGEABLE = 0x0000, //!< Protocol state can not be changed.
  MLPI_SECURITY_SERVICE_ACTIVATABLE   = 0x0001, //!< Protocol can be switched on.
  MLPI_SECURITY_SERVICE_DEACTIVATABLE = 0x0002, //!< Protocol can be switched off.
  MLPI_SECURITY_SERVICE_CHANGEABLE    = 0x0003  //!< Protocol can be switched on an off.
}MlpiSecurityServiceControl;

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiNetworkServiceInfo
//! @brief This structure provides all information about a specific network-service 
//! @details Elements of struct MlpiNetworkServiceInfo
//! <TABLE>
//! <TR><TH>           Type                      </TH><TH>           Element                   </TH><TH> Description                                 </TH></TR>
//! <TR><TD id="st_t"> WCHAR16                   </TD><TD id="st_e"> networkServiceName        </TD><TD> Name of the protocol (e.g. "SSH" or "FTP"). </TD></TR>
//! <TR><TD id="st_t"> MlpiSecurityServiceState  </TD><TD id="st_e"> networkServiceState       </TD><TD> State of the protocol.                      </TD></TR>
//! <TR><TD id="st_t"> networkServiceControlInfo </TD><TD id="st_e"> networkServiceControlInfo </TD><TD> Configuration possiblities of the protocol. </TD></TR>
//! </TABLE>
typedef struct MlpiNetworkServiceInfo
{
  WCHAR16 networkServiceName[MLPI_SECURIY_NETWORKSERVICE_MAX_NAME_LEN];
  MlpiSecurityServiceState networkServiceState;
  MlpiSecurityServiceControl networkServiceControlInfo;
} MlpiNetworkServiceInfo;

//! @typedef MlpiNetworkConfigurationValue
//! @brief This structure provides one configuration value about a specific network-service 
//! @details Elements of struct MlpiNetworkConfigurationValue
//! <TABLE>
//! <TR><TH>           Type     </TH><TH>           Element </TH><TH> Description                      </TH></TR>
//! <TR><TD id="st_t"> WCHAR16  </TD><TD id="st_e"> name    </TD><TD> Name of the configuration value. </TD></TR>
//! <TR><TD id="st_t"> WCHAR16  </TD><TD id="st_e"> value   </TD><TD> The value itself.                </TD></TR>
//! </TABLE>
//! Value can hold only strings. There is no possibility to hold other values. Application is responsible 
//! to parse and insert values correctly.
typedef struct MlpiNetworkConfigurationValue
{
  WCHAR16 MLPI_STRUCT_ALIGN_WCHAR16 name[MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_NAME_LEN];
  WCHAR16 MLPI_STRUCT_ALIGN_WCHAR16 value[MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_VALUE_LEN];
} MlpiNetworkConfigurationValue;

#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @}
//! @addtogroup NetworkSecurity Network Security 
//! @ingroup SecurityLib
//! @{
//! @brief Contains various functions which are handy for controlling the network services on the control
//! @}

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

//! @ingroup NetworkSecurity
//! This function returns information regarding all the network services available on the target device.
//! @param[in]   connection          Handle for multiple connections.
//! @param[out]  serviceInformation  Array of all network services available (name and actual activation state).
//! @param[in]   numElements         Size of the given serviceInformation array.
//! @param[out]  numElementsRet      Return count representing the total amount of network services available on the target device.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiNetworkServiceInfo serviceInformations[10];
//! ULONG numElementRet = 0; 
//! MLPIRESULT result = mlpiSecurityGetNetworkServiceInformation(connection, serviceInformations, 10, &numElementRet);
//! @endcode
MLPI_API MLPIRESULT mlpiSecurityGetNetworkServiceInformation(const MLPIHANDLE connection, MlpiNetworkServiceInfo *serviceInformation, const ULONG numElements, ULONG *numElementsRet);

//! @ingroup NetworkSecurity
//! This function obtains the current activation state for a specific network service.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   service             Identifier of the network service (i.e. MLPI, MLPIS, FTP or SSH).
//! @param[out]  state               Current state of the specific network service (i.e. MLPI_SECURITY_SERVICE_NOTCHANGEABLE,
//!                                  MLPI_SECURITY_SERVICE_ACTIVATABLE, MLPI_SECURITY_SERVICE_DEACTIVATABLE, MLPI_SECURITY_SERVICE_CHANGEABLE)
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiSecurityServiceState state = MLPI_SECURITY_SERVICE_DISABLED;
//! MLPIRESULT result = mlpiSecurityGetNetworkServiceActivation(connection, MLPI_SECURITY_NETWORKSERVICE_FTP, &state);
//! @endcode
MLPI_API MLPIRESULT mlpiSecurityGetNetworkServiceActivation(const MLPIHANDLE connection, const WCHAR16 *service, MlpiSecurityServiceState *state);

//! @ingroup NetworkSecurity
//! This function activates or deactivates a specific network service.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   service             Identifier of the network service (i.e. MLPI, MLPIS, FTP or SSH).
//! @param[in]   state               Future state of the network service (i.e. MLPI_SECURITY_SERVICE_NOTCHANGEABLE,
//!                                  MLPI_SECURITY_SERVICE_ACTIVATABLE, MLPI_SECURITY_SERVICE_DEACTIVATABLE, MLPI_SECURITY_SERVICE_CHANGEABLE).
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result = mlpiSecuritySetNetworkServiceActivation(connection, MLPI_SECURITY_NETWORKSERVICE_SSH, MLPI_SECURITY_SERVICE_ENABLED);
//! @endcode
MLPI_API MLPIRESULT mlpiSecuritySetNetworkServiceActivation(const MLPIHANDLE connection, const WCHAR16 *service, const MlpiSecurityServiceState state);

//! @ingroup NetworkSecurity
//! This function returns current configuration for all network services.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   service             Identifier of the network service (i.e. MLPI, MLPIS, FTP or SSH).
//! @param[out]  config              Array of all configuration options and its values
//! @param[in]   numElements         Size of the given config array.
//! @param[out]  numElementsRet      Return count representing the total amount of configuration values available for the services.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiNetworkConfigurationValue config[2];
//! ULONG numElementRet = 0; 
//! MLPIRESULT result = mlpiSecurityGetNetworkServiceConfiguration(connection, MLPI_SECURITY_NETWORKSERVICE_SSH, config, 2, &numElementRet);
//! @endcode
MLPI_API MLPIRESULT mlpiSecurityGetNetworkServiceConfiguration(const MLPIHANDLE connection, const WCHAR16 *service, MlpiNetworkConfigurationValue *const config, const ULONG numElements, ULONG *const numElementsRet);

//! @ingroup NetworkSecurity
//! This function sets the configurations for selected network services.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   service             Identifier of the network service (i.e. MLPI, MLPIS, FTP or SSH).
//! @param[in]   config              Array of all configuration options and its values
//! @param[in]   numElements         Size of the given config array.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiNetworkConfigurationValue config[2];
//!  wcsncpy( config[0].name,  L"config1", MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_NAME_LEN );
//!  wcsncpy( config[1].name,  L"config2", MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_NAME_LEN );
//!  wcsncpy( networkConfig[0].value, L"on",  MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_VALUE_LEN );
//!  wcsncpy( networkConfig[1].value, L"on",  MLPI_SECURITY_NETWORKCONFIGVALUE_MAX_VALUE_LEN );
//! MLPIRESULT result = mlpiSecuritySetNetworkServiceConfiguration(connection, MLPI_SECURITY_NETWORKSERVICE_SSH, config, 2);
//! @endcode
MLPI_API MLPIRESULT mlpiSecuritySetNetworkServiceConfiguration(const MLPIHANDLE connection, const WCHAR16 *service, MlpiNetworkConfigurationValue *const config, const ULONG numElements);

#ifdef __cplusplus
}
#endif
#endif /*__MLPISECURITYLIB_H__*/
