#ifndef __MLPIACCESSCONTROLLIB_H__
#define __MLPIACCESSCONTROLLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiAccessControlLib.h>
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
//! @author     DC-IA/EAO7 (CV, ABR, SK, JR)
//!
//! @copyright  Bosch Rexroth Corporation http://www.boschrexroth.com/oce
//!
//! @version    1.22.0
//!
//! @date       2016
//
// -----------------------------------------------------------------------

//! @addtogroup AccessControlLib AccessControlLib
//! @{
//! @brief This library provides the functionality that allows to manage users and their information. This information comprises 
//! user credentials used for authentication or groups to which the user belongs. 
//!
//! @note When you use this library to modify your userdatabase it is highly recommended to not use the following options any more:
//!       @ref mlpiApiUserAccountControlLoadAccounts @ref mlpiApiUserAccountControlUnloadAccounts 
//!
//!
//! All the following functions allow to modify and obtain information regarding the user and group configuration of the target control. 
//! The users are a critical part of MLPI, as their authentication is necessary in order to allow/disallow the usage of libraries.
//!
//! @note Currently, only one group is supported and hence, only one group can be assigned to a user. The name of this group
//! is "engineer" and it is used by the Indraworks software in order to perform engineering functions. To assign MLPI permissions of specific MLPI libraries to 
//! newly aggregated users, it is necessary to manually aggregate the permissions after the user account is create. This requires to modify the accounts.xml file located in
//! the /OEM/ProjectData partition of the target device and add the MLPI permissions manually. Afterwards, a device reboot or an update of the accounts.xml is necessary employing
//! the mlpi method @ref mlpiApiUserAccountControlReload.
//! 
//! @}

//! @addtogroup AccessControlLibVersionPermission Version and Permission
//! @ingroup AccessControlLib
//! @{
//! @brief Version and permission information
//!
//! The following table shows the requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permissions needed to execute the desired functions. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_ACCESSCONTROLLIB_PERMISSION_ALL" with the value "true", all functions
//! of this library are enabled for the given user account, except those that modify specific credentials of a user such as
//! mlpiAccessControlChangePassword().
//!
//! @note Functions with permission MLPI_ACCESSCONTROLLIB_PERMISSION_ALWAYS cannot be blocked.
//!
//! @par List of permissions for mlpiAccessControlLib that can be configured in 'accounts.xml'
//! - MLPI_ACCESSCONTROLLIB_PERMISSION_ALWAYS  
//! - MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG  
//! - MLPI_ACCESSCONTROLLIB_PERMISSION_INFO    
//! - MLPI_ACCESSCONTROLLIB_PERMISSION_ALL
//!
//! <TABLE>
//! <TR><TH>           Function                                         </TH><TH> Server version  </TH><TH> Permission                                    </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetUserPolicies            </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetPasswordPolicies        </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetAllUserInfos            </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlAddUser                    </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetUser                    </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlSetUser                    </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlDeleteUser                 </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlChangePassword             </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_ALWAYS"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetAllGroupInfos           </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetAllGroupsOfUser         </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlGetAllUsersOfGroup         </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlAddUserToGroup             </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlRemoveUserFromGroup        </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlSetUsersOfGroup            </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiAccessControlSetGroupsOfUser            </TD><TD> 1.19.0.0        </TD><TD> "MLPI_ACCESSCONTROLLIB_PERMISSION_CONFIG"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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
// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

//! @}
//! @addtogroup AccessControlLibStructs Structs, Types, ...
//! @ingroup AccessControlLib
//! @{
//! @brief List of used types, enumerations, structures and more.

// , ; " "

#define MLPI_MAX_USERNAME_LEN            (32)
#define MLPI_MAX_GROUPNAME_LEN           (32)
#define MLPI_MAX_INVALIDCHARS_LEN        (32)
#define MLPI_MAX_USER_DESCRIPTION_LEN    (128)

//-----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
//-----------------------------------------------------------------------

//! @enum MlpiUserPersistenceType
//! This enumeration describes the possible persistence type of a user, whether it a temp or remanent user.
typedef enum MlpiUserPersistenceType
{
  MLPI_USER_PERSISTENCE_TYPE_TEMPORARY  = 0, //!< The user is only available temporarily.
  MLPI_USER_PERSISTENCE_TYPE_PERMANENT  = 1  //!< The user is available permanently until specified otherwise.
}MlpiUserPersistenceType;

//! @enum MlpiAccessControlExpirationTimeUnit
//! This enumeration describes the possible units that the expiration time can have (i.e. days or minutes).
typedef enum MlpiAccessControlExpirationTimeUnit
{
  MLPI_ACCESSCONTROL_EXPIRATION_UNIT_MINUTES   = 0, //!< Unit of expiration time is in minutes.
  MLPI_ACCESSCONTROL_EXPIRATION_UNIT_DAYS      = 1, //!< Unit of expiration time is in days.
}MlpiAccessControlExpirationTimeUnit;

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiUserInfo
//! @brief This structure provides all basic information about a specific user 
//! @details Elements of struct MlpiUserInfo
//! <TABLE>
//! <TR><TH>           Type                           </TH><TH>           Element                   </TH><TH> Description                                                         </TH></TR>
//! <TR><TD id="st_t"> ULONG                          </TD><TD id="st_e"> userId                    </TD><TD> User's unique identification number.                                </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                        </TD><TD id="st_e"> username                  </TD><TD> Name of the user employed for Login (i.e. login name).              </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                        </TD><TD id="st_e"> description               </TD><TD> Description of the user account.                                    </TD></TR>
//! <TR><TD id="st_t"> MlpiUserPersistenceType        </TD><TD id="st_e"> persistenceType           </TD><TD> Type of lifetime: temporary (only valid for a session) or permanent.</TD></TR>
//! <TR><TD id="st_t"> BOOL8                          </TD><TD id="st_e"> disabled                  </TD><TD> If true, the user authentication fails.                             </TD></TR>
//! </TABLE>
typedef struct MlpiUserInfo
{
  ULONG                      MLPI_STRUCT_ALIGN_ULONG    userId;
  WCHAR16                    MLPI_STRUCT_ALIGN_WCHAR16  username[MLPI_MAX_USERNAME_LEN];
  WCHAR16                    MLPI_STRUCT_ALIGN_WCHAR16  description[MLPI_MAX_USER_DESCRIPTION_LEN];
  MlpiUserPersistenceType    MLPI_STRUCT_ALIGN_ENUM     persistenceType;
  BOOL8                      MLPI_STRUCT_ALIGN_BOOL8    disabled;
} MlpiUserInfo;


//! @typedef MlpiAccessControlExpirationDetails
//! @brief This structure provides information about the expiration settings applied to sessions and passwords 
//! @details Elements of struct MlpiAccessControlExpirationDetails
//! <TABLE>
//! <TR><TH>           Type                                 </TH><TH>           Element               </TH><TH> Description                                                                       </TH></TR>
//! <TR><TD id="st_t"> BOOL8                                </TD><TD id="st_e"> enabled               </TD><TD> Indicates if the object expiration is enabled.                                    </TD></TR>
//! <TR><TD id="st_t"> ULONG                                </TD><TD id="st_e"> expirationTime        </TD><TD> Time after the creation of the object, at which the object will expire.           </TD></TR>
//! <TR><TD id="st_t"> MlpiAccessControlExpirationTimeUnit  </TD><TD id="st_e"> expirationTimeUnit    </TD><TD> Unit of expiration time (i.e. days or minutes).                                   </TD></TR>
//! </TABLE>

typedef struct MlpiAccessControlExpirationDetails
{
  BOOL8                                MLPI_STRUCT_ALIGN_BOOL8  enabled;
  ULONG                                MLPI_STRUCT_ALIGN_ULONG  expirationTime;
  MlpiAccessControlExpirationTimeUnit  MLPI_STRUCT_ALIGN_ENUM   expirationTimeUnit;
} MlpiAccessControlExpirationDetails;

//! @typedef MlpiUserDetails
//! @brief This structure provides all detailed information about a specific user 
//! @details Elements of struct MlpiUserDetails
//! <TABLE>
//! <TR><TH>           Type                                </TH><TH>           Element                      </TH><TH> Description                                                                       </TH></TR>
//! <TR><TD id="st_t"> ULONG                               </TD><TD id="st_e"> userId                       </TD><TD> User's unique identification number.                                              </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                             </TD><TD id="st_e"> username                     </TD><TD> Name of the user employed for Login (i.e. Username).                              </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                             </TD><TD id="st_e"> description                  </TD><TD> Description of the user account.                                                  </TD></TR>
//! <TR><TD id="st_t"> BOOL8                               </TD><TD id="st_e"> disabled                     </TD><TD> If true, the user is disabled and the authentication will fail.                   </TD></TR>
//! <TR><TD id="st_t"> BOOL8                               </TD><TD id="st_e"> userExpirationEnabled        </TD><TD> Indicates if the user is to be disabled automatically at a specific date.         </TD></TR>
//! <TR><TD id="st_t"> MlpiDateAndTime                     </TD><TD id="st_e"> userExpirationTime           </TD><TD> Date in which the user will be disabled if autoDisableEnabled is true.            </TD></TR>
//! <TR><TD id="st_t"> MlpiAccessControlExpirationDetails  </TD><TD id="st_e"> sessionExpiration            </TD><TD> Information regarding a user's session expiration.                                </TD></TR>
//! <TR><TD id="st_t"> MlpiUserPersistenceType             </TD><TD id="st_e"> persistenceType              </TD><TD> Type of lifetime: temporary (only valid for a session) or permanent.              </TD></TR>
//! </TABLE>
typedef struct MlpiUserDetails
{
  ULONG                                 MLPI_STRUCT_ALIGN_ULONG    userId;
  WCHAR16                               MLPI_STRUCT_ALIGN_WCHAR16  username[MLPI_MAX_USERNAME_LEN];
  WCHAR16                               MLPI_STRUCT_ALIGN_WCHAR16  description[MLPI_MAX_USER_DESCRIPTION_LEN];
  BOOL8                                 MLPI_STRUCT_ALIGN_BOOL8    disabled;
  BOOL8                                 MLPI_STRUCT_ALIGN_BOOL8    userExpirationEnabled;
  MlpiDateAndTime                       MLPI_STRUCT_ALIGN_STRUCT   userExpirationTime;
  MlpiAccessControlExpirationDetails    MLPI_STRUCT_ALIGN_STRUCT   sessionExpiration;
  MlpiUserPersistenceType               MLPI_STRUCT_ALIGN_ENUM     persistenceType;
} MlpiUserDetails;

//! @typedef MlpiPasswordPolicies
//! @brief This structure contains the policies to be followed to define the user passwords, such as password length boundaries and minimum amount of specific-type characters.
//! @details Elements of struct MlpiPasswordPolicies
//! <TABLE>
//! <TR><TH>           Type                         </TH><TH>           Element                   </TH><TH> Description                                                                           </TH></TR>
//! <TR><TD id="st_t"> ULONG                        </TD><TD id="st_e"> minLength                 </TD><TD> Minimum length that a password shall contain.                                         </TD></TR>
//! <TR><TD id="st_t"> ULONG                        </TD><TD id="st_e"> maxLength                 </TD><TD> Maximum length that a password shall contain.                                         </TD></TR>
//! <TR><TD id="st_t"> ULONG                        </TD><TD id="st_e"> minUpperCase              </TD><TD> Minimum amount of upper case characters that a password shall contain.                </TD></TR>
//! <TR><TD id="st_t"> ULONG                        </TD><TD id="st_e"> minLower                  </TD><TD> Minimum amount of lower case characters that a password shall contain.                </TD></TR>
//! <TR><TD id="st_t"> ULONG                        </TD><TD id="st_e"> minNumerical              </TD><TD> Minimum amount of numerical characters that a password shall contain.                 </TD></TR>
//! <TR><TD id="st_t"> ULONG                        </TD><TD id="st_e"> minSpecial                </TD><TD> Minimum amount of special characters (e.g. #, $, etc.) that a password shall contain. </TD></TR>
//! </TABLE>
typedef struct MlpiPasswordPolicies
{
  ULONG  MLPI_STRUCT_ALIGN_ULONG  minLength;
  ULONG  MLPI_STRUCT_ALIGN_ULONG  maxLength;
  ULONG  MLPI_STRUCT_ALIGN_ULONG  minUpperCase;
  ULONG  MLPI_STRUCT_ALIGN_ULONG  minLowerCase;
  ULONG  MLPI_STRUCT_ALIGN_ULONG  minNumerical;
  ULONG  MLPI_STRUCT_ALIGN_ULONG  minSpecial;
} MlpiPasswordPolicies;

//! @typedef MlpiUserPolicies
//! @brief This structure contains the policies to be followed by a user.
//! @details Elements of struct MlpiUserPolicies
//! <TABLE>
//! <TR><TH>           Type                                </TH><TH>           Element                                  </TH><TH> Description                                                                                   </TH></TR>
//! <TR><TD id="st_t"> ULONG                               </TD><TD id="st_e"> usernameMinLength                        </TD><TD> Minimum length that a Username shall contain.                                                 </TD></TR>
//! <TR><TD id="st_t"> ULONG                               </TD><TD id="st_e"> usernameMaxLength                        </TD><TD> Maximum length that a Username shall contain.                                                 </TD></TR>
//! <TR><TD id="st_t"> ULONG                               </TD><TD id="st_e"> usernameInvalidChars                     </TD><TD> Contains the characters that are not allowed to be used on the username.                      </TD></TR>
//! <TR><TD id="st_t"> ULONG                               </TD><TD id="st_e"> descriptionMinLength                     </TD><TD> Minimum length that a user description shall contain.                                         </TD></TR>
//! <TR><TD id="st_t"> ULONG                               </TD><TD id="st_e"> descriptionMaxLength                     </TD><TD> Maximum length that a user description shall contain.                                         </TD></TR>
//! <TR><TD id="st_t"> MlpiAccessControlExpirationDetails  </TD><TD id="st_e"> sessionExpirationSettings                </TD><TD> Global session expiration settings                                                            </TD></TR>
//! </TABLE>
typedef struct MlpiUserPolicies
{
  ULONG                                MLPI_STRUCT_ALIGN_ULONG    usernameMinLength;
  ULONG                                MLPI_STRUCT_ALIGN_ULONG    usernameMaxLength;
  WCHAR16                              MLPI_STRUCT_ALIGN_WCHAR16  usernameInvalidChars[MLPI_MAX_INVALIDCHARS_LEN];  
  ULONG                                MLPI_STRUCT_ALIGN_ULONG    descriptionMinLength;
  ULONG                                MLPI_STRUCT_ALIGN_ULONG    descriptionMaxLength;
  MlpiAccessControlExpirationDetails   MLPI_STRUCT_ALIGN_STRUCT   sessionExpirationSettings;
} MlpiUserPolicies;

//! @typedef MlpiGroupInfo
//! @brief This structure provides all basic information about a specific group 
//! @details Elements of struct MlpiGroupInfo
//! <TABLE>
//! <TR><TH>           Type                           </TH><TH>           Element                   </TH><TH> Description                                                                               </TH></TR>
//! <TR><TD id="st_t"> ULONG                          </TD><TD id="st_e"> groupId                   </TD><TD> Group's unique identification number.                                                     </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                        </TD><TD id="st_e"> groupName                 </TD><TD> Name of the group.                                                                        </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                        </TD><TD id="st_e"> description               </TD><TD> Description of the group.                                                                 </TD></TR>
//! <TR><TD id="st_t"> BOOL8                          </TD><TD id="st_e"> disabled                  </TD><TD> If true, the permissions assigned to the group are no longer valid for the group members. </TD></TR>
//! </TABLE>
typedef struct MlpiGroupInfo
{
  ULONG    MLPI_STRUCT_ALIGN_ULONG    groupId;
  WCHAR16  MLPI_STRUCT_ALIGN_WCHAR16  groupName[MLPI_MAX_GROUPNAME_LEN];
  WCHAR16  MLPI_STRUCT_ALIGN_WCHAR16  description[MLPI_MAX_USER_DESCRIPTION_LEN];
  BOOL8    MLPI_STRUCT_ALIGN_BOOL8    disabled;
} MlpiGroupInfo;

#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

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


//! @addtogroup UserControlLib User Management functions
//! @ingroup AccessControlLib
//! @{
//! @brief Contains various functions which are handy for the management of users configured on the target control
//! @}

//! @ingroup UserControlLib
//! This function returns information regarding the policies that apply to all users.
//! @param[in]   connection          Handle for multiple connections.
//! @param[out]  userPolicies        Pointer to structure where the user policies will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiUserPolicies userPolicies;
//! MLPIRESULT result = mlpiAccessControlGetUserPolicies(connection, &userPolicies);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetUserPolicies(const MLPIHANDLE connection, MlpiUserPolicies* userPolicies);

//! @ingroup UserControlLib
//! This function returns information regarding the policies that apply to all the user passwords.
//! @param[in]   connection          Handle for multiple connections.
//! @param[out]  passwordPolicies    Pointer to structure where the password policies will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiPasswordPolicies passwordPolicies;
//! MLPIRESULT result = mlpiAccessControlGetPasswordPolicies(connection, &passwordPolicies);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetPasswordPolicies(const MLPIHANDLE connection, MlpiPasswordPolicies* passwordPolicies);

//! @ingroup UserControlLib
//! This function returns information regarding all the users currently found within the user manager located on the target device.
//! @param[in]   connection          Handle for multiple connections.
//! @param[out]  userInfos           Array that will store all the available user information..
//! @param[in]   numElements         Size of the given userInfos array.
//! @param[out]  numElementsRet      Return count representing the total amount of user information available on the target device.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiUserInfo *userInfos = new MlpiUserInfo[10];
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiAccessControlGetAllUserInfos(connection, userInfos, 10, &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetAllUserInfos(const MLPIHANDLE connection, MlpiUserInfo* userInfos, const ULONG numElements, ULONG* numElementsRet);

//! @ingroup UserControlLib
//! This function adds a new user in the target that can be used for authentication. Once the user is added the password shall be changed immediately by logging in using the temporary password.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   userDetails         Structure to be written containing the information of the new user (the userId is obsolete, as it is defined automatically by the user manager on the target control).
//! @param[in]   tmpPassword         Temporary password that is suggested to be modified during first login.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *tmpPassword = L"newTestPassword";
//! MlpiUserDetails userDetails;
//! wcscpy16(userDetails.username, L"testUsername");
//! wcscpy16(userDetails.description, L"testDescription");
//! userDetails.lifetimeType = MLPI_ACCESSCONTROL_USER_PERMANENT;
//! userDetails.disabled = false;
//!
//! MLPIRESULT result = mlpiAccessControlAddUser(connection, userDetails, tmpPassword);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlAddUser(const MLPIHANDLE connection, MlpiUserDetails userDetails, const WCHAR16* tmpPassword);

//! @ingroup UserControlLib
//! This function obtains the user details of a specific user.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   username            Username that identifies the user, whose information will be retrieved.
//! @param[in]   userDetails         Pointer to structure to be written containing the information of the indicated user.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiUserDetails userDetails;
//! WCHAR16 *username = L"testUsername";
//!
//! MLPIRESULT result = mlpiAccessControlGetUser(connection, username, &userDetails);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetUser(const MLPIHANDLE connection,  const WCHAR16* username, MlpiUserDetails* userDetails);

//! @ingroup UserControlLib
//! This function allows to modify the information of a user. Within the userDetails argument, the correct Id of the user (if known) or username must be given.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   userDetails         Allows to modify the information of a specific user.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiUserDetails userDetails;
//! userDetails.userId = 0;
//! wcscpy16(userDetails.username, L"testUsername");
//! wcscpy16(userDetails.description, L"TestDescriptionModified");
//! userDetails.lifetimeType = MLPI_ACCESSCONTROL_USER_TEMPORARY;
//! userDetails.disabled = true;
//!
//! MLPIRESULT result = mlpiAccessControlSetUser(connection, userDetails);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlSetUser(const MLPIHANDLE connection, MlpiUserDetails userDetails);

//! @ingroup UserControlLib
//! This function deletes a user specified by a given username.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   username            Username that identifies the user that is to be deleted.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *username = L"testUsername";
//!
//! MLPIRESULT result = mlpiAccessControlDeleteUser(connection, username);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlDeleteUser(const MLPIHANDLE connection, const WCHAR16* username);

//! @ingroup UserControlLib
//! This function changes a user's password.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   username            Username that identifies the user, whose password will be changed.
//! @param[in]   oldPassword         String that identifies the current password of the specified user.
//! @param[in]   newPassword         String that identifies the new password to be set for the specified user.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *username = L"testUsername";
//! WCHAR16 *oldPassword = L"newTestPassword";
//! WCHAR16 *newPassword = L"testPassword";
//!
//! MLPIRESULT result = mlpiAccessControlChangePassword(connection, username, oldPassword, newPassword);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlChangePassword(const MLPIHANDLE connection, const WCHAR16* username, const WCHAR16* oldPassword, const WCHAR16* newPassword);

//! @ingroup UserControlLib
//! This function provides information regarding all the groups of users currently supported by the user manager located on the target device.
//! @param[in]   connection          Handle for multiple connections.
//! @param[out]  groupInfos          Array that will store all the available group information.
//! @param[in]   numElements         Size of the given groupInfos array.
//! @param[out]  numElementsRet      Return count representing the total amount of group information available on the target device.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiGroupInfo *groupInfos = new MlpiGroupInfo[10];
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiAccessControlGetAllGroupInfos(connection, groupInfos, 10, &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetAllGroupInfos(const MLPIHANDLE connection, MlpiGroupInfo* groupInfos, const ULONG numElements, ULONG* numElementsRet);

//! @ingroup UserControlLib
//! This function provides the general group information of all the groups assigned to the specified user.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   username            Name that identifies the user, whose group information will be retrieved.
//! @param[out]  groupNames          String where the group names (separated by semicolons) will be stored.
//! @param[in]   numElements         Number of WCHAR16 elements available in 'groupNames'.
//! @param[out]  numElementsRet      Number of WCHAR16 elements in complete 'groupNames'.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *username = L"testUsername";
//! WCHAR16 groupNames[128] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiAccessControlGetAllGroupsOfUser(connection, username, groupNames,  _countof(groupNames), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetAllGroupsOfUser(const MLPIHANDLE connection, const WCHAR16* username, WCHAR16* groupNames, const ULONG numElements, ULONG* numElementsRet);

//! @ingroup UserControlLib
//! This function obtains the user details of all the users assigned to the specified group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   groupName           Name that identifies the group, whose users information will be retrieved.
//! @param[out]  usernames           String where the user names (separated by semicolons) will be stored.
//! @param[in]   numElements         Number of WCHAR16 elements available in 'usernames'.
//! @param[out]  numElementsRet      Number of WCHAR16 elements in complete 'usernames'.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *groupName = L"testGroupName";
//! WCHAR16 usernames[128] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiAccessControlGetAllUsersOfGroup(connection, groupName, usernames,  _countof(usernames), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlGetAllUsersOfGroup(const MLPIHANDLE connection, const WCHAR16* groupName, WCHAR16* usernames, const ULONG numElements, ULONG* numElementsRet);

//! @ingroup UserControlLib
//! This function add a user to a specified group. In other words, the user will be identified as a member of the assigned group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   groupName           Name that identifies the group to which the user will be assigned.
//! @param[out]  username            Name that identifies the user that is to be assigned to the specified group.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *groupName = L"testGroup";
//! WCHAR16 *username = L"testUsername";
//! MLPIRESULT result = mlpiAccessControlAddUserToGroup(connection, groupName, username);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlAddUserToGroup(const MLPIHANDLE connection, const WCHAR16* groupName, const WCHAR16* username);

//! @ingroup UserControlLib
//! This function removes a user from specified group. In other words, the user will not be identified as a member of the assigned group and hence, it won't have the group permissions anymore.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   groupName           Name that identifies the group from which the user will be removed.
//! @param[out]  username            Name that identifies the user that is to be removed from the specified group.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *groupName = L"testGroup";
//! WCHAR16 *username = L"testUsername";
//! MLPIRESULT result = mlpiAccessControlRemoveUserFromGroup(connection, groupName, username);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlRemoveUserFromGroup(const MLPIHANDLE connection, const WCHAR16* groupName, const WCHAR16* username);

//! @ingroup UserControlLib
//! This function sets the Users of a Group. It is capable of adding multiple new users or deleting pre-existing ones in the group. It is recommended to first use the function @ref mlpiAccessControlGetAllUsersOfGroup
//! in order to verify that no unwanted user is added/deleted.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   groupName           Name that identifies the group to which the users will be assigned.
//! @param[out]  usernames           String that contains the user names separated by semicolons.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *groupName = L"testGroup";
//! WCHAR16 *usernames = L"testUsername1;testUsername2";
//! MLPIRESULT result = mlpiAccessControlSetUsersOfGroup(connection, groupName, usernames);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlSetUsersOfGroup(const MLPIHANDLE connection, const WCHAR16* groupName, const WCHAR16* usernames);

//! @ingroup UserControlLib
//! This function sets the Groups of a User. It is capable of adding multiple new groups to a user or deleting pre-existing ones in the user. It is recommended to first use the function @ref mlpiAccessControlGetAllGroupsOfUser
//! in order to verify that no unwanted group is added/deleted.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   username            Name that identifies the user to which the groups will be assigned.
//! @param[out]  groupNames          String that contains the group names separated by semicolons.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 *username = L"testUser";
//! WCHAR16 *groupNames = L"testGroup1;testGroup2";
//! MLPIRESULT result = mlpiAccessControlSetGroupsOfUser(connection, username, groupNames);
//! @endcode
MLPI_API MLPIRESULT mlpiAccessControlSetGroupsOfUser(const MLPIHANDLE connection, const WCHAR16* username, const WCHAR16* groupNames);

#ifdef __cplusplus
}
#endif
#endif /*__MLPIACCESSCONTROLLIB_H__*/
