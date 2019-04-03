#ifndef __MLPISYSTEMLIB_H__
#define __MLPISYSTEMLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiSystemLib.h>
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


//! @addtogroup SystemLib SystemLib
//! @{
//! @brief This library contains functions used to provide access to common device
//! settings of the target device system.
//!
//! The diagnosis logbook of the target device can also be accessed using this library.
//! All relevant functions can be found in @ref SystemLibDiagnosis.
//!
//! @note The SystemLib functions trace their debug information mainly into the module MLPI_SYSTEM_LIB and
//!       in addition into the modules MLPI_PARAMETER_LIB and MLPI_BASE_MODULES. For further information,
//!       see also the detailed description of the library @ref TraceLib and the notes about @ref sec_TraceViewer.
//!
//! @}

//! @addtogroup SystemLibCommon Common system functions
//! @ingroup SystemLib
//! @{
//! @brief Contains functions used to read different information of the target device.
//! @}

//! @addtogroup SystemLibDiagnosis Diagnosis system
//! @ingroup SystemLib
//! @{
//! @brief The following functions provide access to the internal diagnosis logbook
//! on the target device. For a detailed description of all error codes and diagnosis numbers, please
//! have a look to the diagnosis help of the device you are connected too.
//! @}

//! @addtogroup SystemLibTransmission Transmission
//! @ingroup SystemLib
//! @{
//! @brief The following functions provide access to the transmission functionality
//! on the target device.
//! @}

//! @addtogroup SystemLibVersionPermission Version and Permission
//! @ingroup SystemLib
//! @{
//! @addtogroup SystemLibVersionPermission_new Server version since 1.26.0.0 (MLC-FW: 14V22)
//! @ingroup SystemLibVersionPermission
//! @{
//!
//! @note Since firmware version 14V22 (MLPI-Server-Version: 1.26.0.0) a centralized permission management has been implemented in target 
//! controls XM2, L75 and VPx. Some permissions have been summarized in order to improve their usability. 
//! Additional information regarding the usage of older manifest files (i.e. accounts.xml) with newer server versions can be found in @ref newest_manifest.\n
//! @note <b><span style="color:red">Users of other CML controls (i.e. L25, L45, L65) have to use the old permissions as defined in @ref SystemLibVersionPermission_old</span></b>
//!
//!
//! @par List of valid permissions for mlpiSystemLib. These permissions shall be assigned to the groups (i.e. in the group manifest file groups.xml) rather than the users.
//! <TABLE>
//! <TR><TH> Permission-Ident                     </TH><TH> Description                                                                                                                                   </TH></TR>                  
//! <TR><TD id="st_e"> SYSTEM_CLEANUP             </TD><TD> Factory reset - Allows to perform a facotry reset of the device.                                                                              </TD></TR>  
//! <TR><TD id="st_e"> SYSTEM_SETUP               </TD><TD> Configure system settings - Allows to configure system settings e.g. IP address, date and time etc. and reboot the device.                    </TD></TR>  
//! <TR><TD id="st_e"> SYSTEM_STATUS              </TD><TD> View system settings and status - Allows to view system settings and status e.g. IP address, date and time, system temperature, CPU load etc. </TD></TR>  
//! <TR><TD id="st_e"> SYSTEM_TRANSMISSIONCLIENT  </TD><TD> Use the device as TFTP client - Allows to use the device as TFTP client, e.g. for IndraDrive firmware update.                                 </TD></TR>  
//! </TABLE>
//!
//!  @par List of available functions in mlpiSystemLib and the permissions required for their use. 
//! <TABLE>
//! <TR><TH>           Function                                         </TH><TH> Server version </TH><TH> Permission-Ident             </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetVersionInfo                    </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLanguage                       </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetLanguage                       </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetName                           </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetName                           </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetIpAddress                      </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetIpAddress                      </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSubnetMask                     </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetSubnetMask                     </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetGateway                        </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetGateway                        </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetCurrentMode                    </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetTargetMode                     </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTemperature                    </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTemperatureMax                 </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetCpuLoad                        </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetCpuLoadMax                     </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetOperationHours                 </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetDateAndTimeUtc                 </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetDateAndTimeUtc                 </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSerialNumber                   </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetHardwareDetails                </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLocalBusConfiguration          </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetFunctionModulBusConfiguration  </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetFunctionModuleHardwareDetails  </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetMacAddress                     </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetMemoryInfo                     </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSpecialPath                    </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSpecialPathInfo                </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemClearError                        </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemReboot                            </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemCleanup                           </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_CLEANUP"             </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetDisplayedDiagnosis             </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetDiagnosis                      </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetNewestDiagnosisIndex           </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetOldestDiagnosisIndex           </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetDiagnosisLog                   </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemConvertDiagnosisNumberToText      </TD><TD> 1.0.0.0        </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetDateAndTimeUtcByNtp            </TD><TD> 1.11.0.0       </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetRootPaths                      </TD><TD> 1.11.0.0       </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemStartTransmission                 </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTransmissionStatus             </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTransmissionInfo               </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpSetTimeout                    </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpGetTimeout                    </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpSetRetransmitCount            </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpGetRetransmitCount            </TD><TD> 1.14.0.0       </TD><TD> "SYSTEM_TRANSMISSIONCLIENT"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetBootLed                        </TD><TD> 1.18.0.0       </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemClearBootLed                      </TD><TD> 1.18.0.0       </TD><TD> "SYSTEM_SETUP"               </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemResetTemperature                  </TD><TD> 1.18.0.0       </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemResetCpuLoad                      </TD><TD> 1.18.0.0       </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLicenseInformation             </TD><TD> 1.18.0.0       </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLimit                          </TD><TD> 1.24.0.0       </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTargetId                       </TD><TD> 1.24.0.0       </TD><TD> "SYSTEM_STATUS"              </TD></TR>
//! </TABLE>
//!
//!
//! @par List of the old permissions of mlpiSystemLib and their corresponding new permission.
//! <TABLE>
//! <TR><TH> Old permission                                         </TH><TH> new Permission            </TH></TR>                  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_ALWAYS             </TD><TD> IMPLICIT                  </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_ENGINEERING_PORT   </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_TARGET_MODE        </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_LANGUAGE           </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_NAME               </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_DATE_AND_TIME      </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_CLEAR_ERROR        </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_REBOOT             </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_CLEANUP            </TD><TD> SYSTEM_CLEANUP            </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO        </TD><TD> SYSTEM_STATUS             </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO     </TD><TD> SYSTEM_STATUS             </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS          </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION       </TD><TD> SYSTEM_TRANSMISSIONCLIENT </TD></TR>  
//! <TR><TD id="st_e"> MLPI_SYSTEMLIB_PERMISSION_LED_BT             </TD><TD> SYSTEM_SETUP              </TD></TR>  
//! </TABLE>
//!
//!
//! @}
//! @addtogroup SystemLibVersionPermission_old Server versions before 1.26.0.0 
//! @ingroup SystemLibVersionPermission
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_SYSTEMLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_SYSTEMLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiSystemLib using in accounts.xml
//! - MLPI_SYSTEMLIB_PERMISSION_ALL
//! - MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO
//! - MLPI_SYSTEMLIB_PERMISSION_LANGUAGE
//! - MLPI_SYSTEMLIB_PERMISSION_NAME
//! - MLPI_SYSTEMLIB_PERMISSION_ENGINEERING_PORT
//! - MLPI_SYSTEMLIB_PERMISSION_TARGET_MODE
//! - MLPI_SYSTEMLIB_PERMISSION_DATE_AND_TIME
//! - MLPI_SYSTEMLIB_PERMISSION_CLEAR_ERROR
//! - MLPI_SYSTEMLIB_PERMISSION_REBOOT
//! - MLPI_SYSTEMLIB_PERMISSION_CLEANUP
//! - MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO
//! - MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS
//! - MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION
//! - MLPI_SYSTEMLIB_PERMISSION_LED_BT
//!
//! <TABLE>
//! <TR><TH>           Function                                         </TH><TH> Server version </TH><TH> Permission                                   </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetVersionInfo                    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLanguage                       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetLanguage                       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_LANGUAGE"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetName                           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetName                           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_NAME"             </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetIpAddress                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetIpAddress                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_ENGINEERING_PORT" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSubnetMask                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetSubnetMask                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_ENGINEERING_PORT" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetGateway                        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetGateway                        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_ENGINEERING_PORT" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetCurrentMode                    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetTargetMode                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TARGET_MODE"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTemperature                    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTemperatureMax                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetCpuLoad                        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetCpuLoadMax                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetOperationHours                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetDateAndTimeUtc                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetDateAndTimeUtc                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DATE_AND_TIME"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSerialNumber                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetHardwareDetails                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLocalBusConfiguration          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetFunctionModulBusConfiguration  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetFunctionModuleHardwareDetails  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetMacAddress                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetMemoryInfo                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSpecialPath                    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetSpecialPathInfo                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemClearError                        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_CLEAR_ERROR"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemReboot                            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_REBOOT"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemCleanup                           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_CLEANUP"          </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetDisplayedDiagnosis             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetDiagnosis                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS"        </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetNewestDiagnosisIndex           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetOldestDiagnosisIndex           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetDiagnosisLog                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemConvertDiagnosisNumberToText      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DIAGNOSIS_INFO"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetDateAndTimeUtcByNtp            </TD><TD> 1.11.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DATE_AND_TIME"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetRootPaths                      </TD><TD> 1.11.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemStartTransmission                 </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTransmissionStatus             </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTransmissionInfo               </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpSetTimeout                    </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpGetTimeout                    </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpSetRetransmitCount            </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemTftpGetRetransmitCount            </TD><TD> 1.14.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_TRANSMISSION"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemSetBootLed                        </TD><TD> 1.18.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_LED_BT"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemClearBootLed                      </TD><TD> 1.18.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_LED_BT"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemResetTemperature                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemResetCpuLoad                      </TD><TD> 1.18.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLicenseInformation             </TD><TD> 1.18.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetLimit                          </TD><TD> 1.24.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiSystemGetTargetId                       </TD><TD> 1.24.0.0       </TD><TD> "MLPI_SYSTEMLIB_PERMISSION_DEVICE_INFO"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
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

//! @addtogroup SystemLibStructTypes Structs, types, ...
//! @ingroup SystemLib
//! @{
//! @brief List of used types, enumerations, structures and more...




// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"



// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

//! The variable defines the maximum text length to be returned in a diagnosis from the device.
static const LONG MLPI_DIAGNOSIS_STRING_LEN           = 124;

//! This variable defines the value to read the firmware version string.
static const WCHAR16 MLPI_VERSION_FIRMWARE[]          = {'V','E','R','S','I','O','N','_','F','I','R','M','W','A','R','E','\0'};

//! This variable defines the value to read the hardware version string.
static const WCHAR16 MLPI_VERSION_HARDWARE[]          = {'V','E','R','S','I','O','N','_','H','A','R','D','W','A','R','E','\0'};

//! This variable defines the value to read the logic version string.
static const WCHAR16 MLPI_VERSION_LOGIC[]             = {'V','E','R','S','I','O','N','_','L','O','G','I','C','\0'};

//! This variable defines the value to read the board support version string.
static const WCHAR16 MLPI_VERSION_BSP[]               = {'V','E','R','S','I','O','N','_','B','S','P','\0'};

//! This variable defines the value to read the MLPI server core version string.
static const WCHAR16 MLPI_VERSION_MLPI_SERVER_CORE[]  = {'V','E','R','S','I','O','N','_','M','L','P','I','_','S','E','R','V','E','R','_','C','O','R','E','\0'};

//! The variable defines the maximum length of a URL.
static const LONG MLPI_TRANSMISSION_URL_LEN           = 256;

//! The variable defines the maximum length of an error message.
static const LONG MLPI_TRANSMISSION_ERROR_LEN         = 128;


// -----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
// -----------------------------------------------------------------------

//! @enum MlpiLanguage
//! This enumeration must be used when accessing language settings in the device.
typedef enum MlpiLanguage
{
  MLPI_LANGUAGE_GERMAN                  = 00,     //!< German language setting.
  MLPI_LANGUAGE_ENGLISH                 = 01,     //!< English language setting.
  MLPI_LANGUAGE_USER_DEFINED            = 99      //!< Load language data from user defined xml file.
} MlpiLanguage;

//! @enum MlpiSystemMode
//! This enumeration must be used when accessing the mode settings in the device.
//! The mode can only be set to initialization mode (MLPI_SYSTEMMODE_P0),
//! parameter mode (MLPI_SYSTEMMODE_P2) or operation mode (MLPI_SYSTEMMODE_BB).
//! The read only modes can also be displayed in case of an error during a mode transition.
typedef enum MlpiSystemMode
{
  MLPI_SYSTEMMODE_P0                    = 0x00,   //!< initialization mode.
  MLPI_SYSTEMMODE_P1                    = 0x01,   //!< transition state from initialization mode to parameter mode.
  MLPI_SYSTEMMODE_P2                    = 0x02,   //!< parameter mode.
  MLPI_SYSTEMMODE_P3                    = 0x03,   //!< transition state from parameter mode to operation mode.
  MLPI_SYSTEMMODE_BB                    = 0x04,   //!< operation mode.
  MLPI_SYSTEMMODE_SWITCHING             = 0xFF    //!< mode switching active.
} MlpiSystemMode;

//! @enum MlpiDiagnosisState
//! This enumeration must be used when accessing the diagnostic details in the device.
typedef enum MlpiDiagnosisState
{
  MLPI_DIAGNOSIS_PASSIVE                = 0x00,   //!< diagnosis entry is passive.
  MLPI_DIAGNOSIS_ACTIVE                 = 0x01,   //!< diagnosis entry is active.
  MLPI_DIAGNOSIS_RESET                  = 0x02,   //!< diagnosis entry is reset.
  MLPI_DIAGNOSIS_UNKNOWN                = 0x03    //!< diagnosis entry state is unknown.
} MlpiDiagnosisState;

//! @enum MlpiDiagnosisDespatcher
//! This enumeration must be used when accessing the diagnostic details in the device.
typedef enum MlpiDiagnosisDespatcher
{
  MLPI_DESPATCHER_CONTROL               = 0x00,   //!< diagnosis entry from control.
  MLPI_DESPATCHER_DRIVE                 = 0x01,   //!< diagnosis entry from drive.
  MLPI_DESPATCHER_LOGIC                 = 0x02,   //!< diagnosis entry from PLC function block.
  MLPI_DESPATCHER_AXIS                  = 0x03,   //!< diagnosis entry from axis.
  MLPI_DESPATCHER_KINEMATIC             = 0x04,   //!< diagnosis entry from kinematic.
  MLPI_DESPATCHER_PCI                   = 0x05    //!< diagnosis entry from PCI device.
} MlpiDiagnosisDespatcher;

//! @enum MlpiSpecialPath
//! This enumeration contains different logical paths on the device.
//!
//! @note Usually, your target device has at least 3 partitions, a SYSTEM, an OEM and an USER partition. The absolute, native paths
//!       of these partitions depends on the target itself, e.g. on an IndraMotion XLC CML45 the absolute path of the OEM partition
//!       is called "/ata0a/". Different FTP clients shows the absolute paths itself (Filezilla, e.g. "/ata0a/") or the logical
//!       paths (IndraWorks, e.g. "OEM").
//!       <TABLE>
//!       <TR><TD> MLPI_PATH_SYSTEM                           </TD><TD> "/ata0/"                                    </TD><TD> "/SYSTEM/"                                </TD></TR>
//!       <TR><TD> MLPI_PATH_OEM                              </TD><TD> "/ata0a/"                                   </TD><TD> "/OEM/"                                   </TD></TR>
//!       <TR><TD> MLPI_PATH_USER                             </TD><TD> "/ata0b/"                                   </TD><TD> "/USER/"                                  </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA                      </TD><TD> "/ata0a/ProjectData/"                       </TD><TD> "/OEM/ProjectData/"                       </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATAPROTECTED             </TD><TD> "/ata0a/ProjectDataProtected/"              </TD><TD> "/OEM/ProjectDataProtected/"              </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_BUNDLES              </TD><TD> "/ata0a/ProjectData/bundles/"               </TD><TD> "/OEM/ProjectData/bundles/"               </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATAPROTECTED_BUNDLES     </TD><TD> "/ata0a/ProjectDataProtected/bundles"       </TD><TD> "/OEM/ProjectDataProtected/bundles"       </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_PLC                  </TD><TD> "/ata0a/ProjectData/PLC/"                   </TD><TD> "/OEM/ProjectData/PLC/"                   </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_TMP                  </TD><TD> "/ata0a/ProjectData/TMP/"                   </TD><TD> "/OEM/ProjectData/TMP/"                   </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_JVM                  </TD><TD> "/ata0a/ProjectData/jvm/"                   </TD><TD> "/OEM/ProjectData/jvm/"                   </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_JVM_BUNDLES          </TD><TD> "/ata0a/ProjectData/jvm/bundles/"           </TD><TD> "/OEM/ProjectData/jvm/bundles/"           </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_JVM_LIC              </TD><TD> "/ata0a/ProjectData/jvm/lic/"               </TD><TD> "/OEM/ProjectData/jvm/lic/"               </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATA_JVM_USER             </TD><TD> "/ata0a/ProjectData/jvm/user/"              </TD><TD> "/OEM/ProjectData/jvm/user/"              </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATAPROTECTED_JVM         </TD><TD> "/ata0a/ProjectDataProtected/jvm/"          </TD><TD> "/OEM/ProjectDataProtected/jvm/"          </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATAPROTECTED_JVM_BUNDLES </TD><TD> "/ata0a/ProjectDataProtected/jvm/bundles/"  </TD><TD> "/OEM/ProjectDataProtected/jvm/bundles/"  </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATAPROTECTED_JVM_LIC     </TD><TD> "/ata0a/ProjectDataProtected/jvm/lic/"      </TD><TD> "/OEM/ProjectDataProtected/jvm/lic/"      </TD></TR>
//!       <TR><TD> MLPI_PATH_PROJECTDATAPROTECTED_JVM_USER    </TD><TD> "/ata0a/ProjectDataProtected/jvm/user/"     </TD><TD> "/OEM/ProjectDataProtected/jvm/user/"     </TD></TR>
//!       <TR><TD> MLPI_PATH_OTHER                            </TD><TD> Any other path.                             </TD><TD> Any other path.                           </TD></TR>
//!       </TABLE>
//!
//! Use @ref mlpiSystemGetSpecialPath to retrieve absolute path information on a specific device.
typedef enum MlpiSpecialPath
{
  MLPI_PATH_SYSTEM                            = 0x00,   //!< SYSTEM path, root.
  MLPI_PATH_OEM                               = 0x01,   //!< OEM path, root.
  MLPI_PATH_USER                              = 0x02,   //!< User path, root.
  MLPI_PATH_PROJECTDATAPROTECTED              = 0x03,   //!< Protected project data path.
  MLPI_PATH_PROJECTDATA                       = 0x04,   //!< Project data path.
  MLPI_PATH_PROJECTDATA_BUNDLES               = 0x05,   //!< C/C++ user bundles path.
  MLPI_PATH_PROJECTDATA_PLC                   = 0x06,   //!< PLC project data path.
  MLPI_PATH_PROJECTDATA_TMP                   = 0x07,   //!< Temporary project data path.
  MLPI_PATH_PROJECTDATA_JVM                   = 0x08,   //!< Java VM root path.
  MLPI_PATH_PROJECTDATA_JVM_BUNDLES           = 0x09,   //!< Java VM bundles path.
  MLPI_PATH_PROJECTDATA_JVM_LIC               = 0x0A,   //!< Java VM licenses path.
  MLPI_PATH_PROJECTDATA_JVM_USER              = 0x0B,   //!< Java VM user data path.
  MLPI_PATH_PROJECTDATAPROTECTED_BUNDLES      = 0x0C,   //!< Protected C/C++ user bundles path.
  MLPI_PATH_PROJECTDATAPROTECTED_JVM          = 0x0D,   //!< Protected Java VM root path.
  MLPI_PATH_PROJECTDATAPROTECTED_JVM_BUNDLES  = 0x0E,   //!< Protected Java VM bundles path.
  MLPI_PATH_PROJECTDATAPROTECTED_JVM_LIC      = 0x0F,   //!< Protected Java VM licenses path.
  MLPI_PATH_PROJECTDATAPROTECTED_JVM_USER     = 0x10,   //!< Protected Java VM user data path.
  MLPI_PATH_OTHER                             = 0xFF    //!< Path(s) without special assignments.
} MlpiSpecialPath;

//! @enum MlpiDiagnosisCategory
//! This enumeration contains different diagnosis categories on the device.
typedef enum MlpiDiagnosisCategory
{
  MLPI_DIAGNOSIS_ERROR_FATAL            = 0xF8,   //!< Fatal error, error reaction in motion system.
  MLPI_DIAGNOSIS_ERROR_NONFATAL_CONTROL = 0xF5,   //!< Non-fatal error, all axes are disengaged the best possible.
  MLPI_DIAGNOSIS_ERROR_NONFATAL         = 0xF0,   //!< Non-fatal error, no error reaction.
  MLPI_DIAGNOSIS_WARNING                = 0xE0,   //!< Warning.
  MLPI_DIAGNOSIS_MESSAGE                = 0xA0    //!< Message.
} MlpiDiagnosisCategory;

//! @enum MlpiLedPattern
//! This enumeration provides different LED status resp. diagnosis pattern.
typedef enum MlpiLedPattern
{
  MLPI_LED_PATTERN_GREEN                 =  0,  //!< Switch on LED permanently green.
  MLPI_LED_PATTERN_GREEN_250             =  1,  //!< Toggle LED green/OFF with 4Hz ( 250ms).
  MLPI_LED_PATTERN_GREEN_500             =  2,  //!< Toggle LED green/OFF with 2Hz ( 500ms).
  MLPI_LED_PATTERN_GREEN_1000            =  3,  //!< Toggle LED green/OFF with 1Hz (1000ms).
  MLPI_LED_PATTERN_GREEN_ORANGE_250      =  4,  //!< Toggle LED green/orange with 4Hz ( 250ms).
  MLPI_LED_PATTERN_GREEN_ORANGE_500      =  5,  //!< Toggle LED green/orange with 2Hz ( 500ms).
  MLPI_LED_PATTERN_GREEN_ORANGE_1000     =  6,  //!< Toggle LED green/orange with 1Hz (1000ms).
  MLPI_LED_PATTERN_GREEN_RED_250         =  7,  //!< Toggle LED green/red with 4Hz ( 250ms).
  MLPI_LED_PATTERN_GREEN_RED_500         =  8,  //!< Toggle LED green/red with 2Hz ( 500ms).
  MLPI_LED_PATTERN_GREEN_RED_1000        =  9,  //!< Toggle LED green/red with 1Hz (1000ms).
  MLPI_LED_PATTERN_INVALID               = -1   //!< Invalid LED pattern.
} MlpiLedPattern;


//! @enum MlpiSystemLimit
//! This enumeration provides different limit defines.
typedef enum MlpiSystemLimit
{
  MLPI_SYSTEM_LIMIT_MAX_APPLICATIONS    =  0,   //!< Maximum number of application.
  MLPI_SYSTEM_LIMIT_MAX_CONTROLS        =  1,   //!< Maximum number of controls.
  MLPI_SYSTEM_LIMIT_MAX_AXIS            =  2,   //!< Maximum number of axis.
  MLPI_SYSTEM_LIMIT_MAX_PLS             =  3,   //!< Maximum number of PLS.
  MLPI_SYSTEM_LIMIT_MAX_KINEMATICS      =  4,   //!< Maximum number of kinematics.
  MLPI_SYSTEM_LIMIT_MAX_TOUCHPROBES     =  5,   //!< Maximum number of touch probes.
  MLPI_SYSTEM_LIMIT_MAX_OSCI_CHANNELS   =  6,   //!< Maximum number of oscilloscope channels.
  MLPI_SYSTEM_LIMIT_INVALID             = -1    //!< Invalid limit.
}MlpiSystemLimit;


// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiDiagnosis
//! @brief This structure defines the information returned when accessing the diagnosis. A diagnosis
//! is either an error (F...), a warning (E...) or a message (A...). An error must be cleared by the
//! user, warnings remain active until the reason for the warning is removed.
//! Messages only serve an informational purpose. When an error is cleared, two entries exist in the
//! diagnostic log. One where state is set MLPI_DIAGNOSIS_ACTIVE and another one where it is set to
//! MLPI_DIAGNOSIS_RESET.
//! @details Elements of struct MlpiDiagnosis
//! <TABLE>
//! <TR><TH>                Type                    </TH><TH>           Element         </TH><TH> Description                                                                                            </TH></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> number          </TD><TD> Ident number that is described in the help manual and identifies the diagnosis.                        </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiDiagnosisState      </TD><TD id="st_e"> state           </TD><TD> State of a diagnostic message.                                                                         </TD></TR>
//! <TR><TD id="st_t">      MlpiDateAndTime         </TD><TD id="st_e"> dateTime        </TD><TD> Date and time when the diagnosis took place.                                                           </TD></TR>
//! <TR><TD id="st_t">      LONG                    </TD><TD id="st_e"> logicalAddress  </TD><TD> Logical address of the device(e.g. axis, touchprobe,...) that dispatched the diagnosis.                </TD></TR>
//! <TR><TD id="st_t">      LONG                    </TD><TD id="st_e"> physicalAddress </TD><TD> Physical address of a physical device possibly connected to the logical object. E.g. Drive of an axis. </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiDiagnosisDespatcher </TD><TD id="st_e"> despatcher      </TD><TD> Identifying source of diagnosis.                                                                       </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> index           </TD><TD> Index of the diagnosis inside the diagnosis ring buffer.                                               </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> text            </TD><TD> Diagnosis text with diagnosis description.                                                             </TD></TR>
//! </TABLE>
typedef struct MlpiDiagnosis
{
  ULONG                   number;                          //!< Ident number that is described in the help manual and identifiers the diagnosis.
  MlpiDiagnosisState      state;                           //!< State of a diagnostic message.
  MlpiDateAndTime         dateTime;                        //!< Date and time when the diagnosis took place.
  LONG                    logicalAddress;                  //!< Logical address of the device (e.g. axis, touchprobe,...) that dispatched the diagnosis.
  LONG                    physicalAddress;                 //!< Physical address of a physical device possibly connected to the logical object. E.g. Drive of an axis.
  MlpiDiagnosisDespatcher despatcher;                      //!< Identifying source of diagnosis.
  ULONG                   index;                           //!< Index of the diagnosis inside the diagnosis ring buffer.
  WCHAR16                 text[MLPI_DIAGNOSIS_STRING_LEN]; //!< Diagnosis text with diagnosis description.
}MlpiDiagnosis;

//! @typedef MlpiMemoryInfo
//! This structure defines the memory information.
//! @details Elements of struct MlpiMemoryInfo
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element               </TH><TH> Description                                                                                               </TH></TR>
//! <TR><TD id="st_t"> ULLONG </TD><TD id="st_e"> totalVolatileBytes    </TD><TD> Total RAM of the device in bytes.                                                                         </TD></TR>
//! <TR><TD id="st_t"> ULLONG </TD><TD id="st_e"> totalNonvolatileBytes </TD><TD> Total non-volatile RAM of the device in bytes. This memory is not directly accessible
//!                                                                               for the user, but used for storing system parameters and settings. For example, axis configuration etc... </TD></TR>
//! <TR><TD id="st_t"> ULLONG </TD><TD id="st_e"> freeVolatileBytes     </TD><TD> Free RAM of the device in bytes.                                                                          </TD></TR>
//! <TR><TD id="st_t"> ULLONG </TD><TD id="st_e"> freeNonvolatileBytes  </TD><TD> Free non-volatile RAM of the device in bytes. This value should never reach 0.                            </TD></TR>
//! </TABLE>
typedef struct MlpiMemoryInfo
{
  ULLONG totalVolatileBytes;     //!< Total RAM of the device in bytes.
  ULLONG totalNonvolatileBytes;  //!< Total non-volatile RAM of the device in bytes.
  ULLONG freeVolatileBytes;      //!< Free RAM of the device in bytes.
  ULLONG freeNonvolatileBytes;   //!< Free non-volatile RAM of the device in bytes. This value should never reach 0.
} MlpiMemoryInfo;

//! @typedef MlpiSpecialPathInfo
//! This structure defines the special path information.
//! @details Elements of struct MlpiSpecialPathInfo
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element     </TH><TH> Description                                 </TH></TR>
//! <TR><TD id="st_t"> ULLONG </TD><TD id="st_e"> totalBytes  </TD><TD> Total size of the partition in bytes.       </TD></TR>
//! <TR><TD id="st_t"> ULLONG </TD><TD id="st_e"> freeBytes   </TD><TD> Free disk-space of the partition in bytes.  </TD></TR>
//! </TABLE>
typedef struct MlpiSpecialPathInfo
{
  ULLONG totalBytes;      //!< Total size of the partition in bytes.
  ULLONG freeBytes;       //!< Free disk-space of the partition in bytes.
} MlpiSpecialPathInfo;

//! @typedef MlpiTransmissionStatus
//! @brief This structure defines the information returned when accessing status of a transmission.
//! @details Elements of struct MlpiTransmissionStatus
//! <TABLE>
//! <TR><TH>                Type                    </TH><TH>           Element         </TH><TH> Description                                                                                            </TH></TR>
//! <TR><TD id="st_t">      MLPIHANDLE              </TD><TD id="st_e"> handle          </TD><TD> Handle of transmission.                                                                                </TD></TR>
//! <TR><TD id="st_t">      MLPIRESULT              </TD><TD id="st_e"> result          </TD><TD> MLPI status code of this transmission.                                                                 </TD></TR>
//! <TR><TD id="st_t">      MlpiProcessState        </TD><TD id="st_e"> state           </TD><TD> State of the transmission.                                                                             </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> totalBytes      </TD><TD> Total bytes to transfer. Zero if size of file is unknown.                                              </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> transmittedBytes</TD><TD> Current bytes transferred.                                                                             </TD></TR>
//! </TABLE>
typedef struct MlpiTransmissionStatus
{
  MLPITASKHANDLE          handle;                   //!< Handle of transmission.
  MLPIRESULT              result;                   //!< MLPI status code of this transmission.
  MlpiProcessState        state;                    //!< State of the transmission.
  ULONG                   totalBytes;               //!< Total bytes to transfer.
  ULONG                   transmittedBytes;         //!< Current bytes transferred.
}MlpiTransmissionStatus;

//! @typedef MlpiTransmissionInfo
//! @brief This structure defines the information returned when accessing information of a transmission.
//! @details Elements of struct MlpiTransmissionInfo
//! <TABLE>
//! <TR><TH>                Type                    </TH><TH>           Element         </TH><TH> Description                                                                                            </TH></TR>
//! <TR><TD id="st_t">      MLPIRESULT              </TD><TD id="st_e"> result          </TD><TD> MLPI status code of this transmission.                                                                 </TD></TR>
//! <TR><TD id="st_t">      MlpiProcessState        </TD><TD id="st_e"> state           </TD><TD> State of the transmission.                                                                             </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> host            </TD><TD> Remote host.                                                                                           </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> localFilename   </TD><TD> Local file name.                                                                                       </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> remoteFilename  </TD><TD> Remote file name.                                                                                      </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> totalBytes      </TD><TD> Total bytes to transfer. Zero if size of file is unknown.                                              </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> transmittedBytes</TD><TD> Current bytes transferred.                                                                             </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> errorMessage    </TD><TD> error message of transmission.                                                                         </TD></TR>
//! <TR><TD id="st_t">      BOOL                    </TD><TD id="st_e"> direction       </TD><TD> Direction of transmission. TRUE = send, FALSE = receive                                                </TD></TR>
//! </TABLE>
typedef struct MlpiTransmissionInfo
{
  MlpiTransmissionStatus  status;                                         //!< status of transmission
  WCHAR16                 source[MLPI_TRANSMISSION_URL_LEN];              //!< source of transmission
  WCHAR16                 destination[MLPI_TRANSMISSION_URL_LEN];         //!< destination of transmission
  WCHAR16                 errorMessage[MLPI_TRANSMISSION_ERROR_LEN];      //!< error message of transmission
}MlpiTransmissionInfo;



#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup SystemLibStructTypes



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

//! @ingroup SystemLibCommon
//! This function returns the version information of different components of the device. The component name has to be given
//! as string. The following constants are already defined by the MLPI:
//! @arg @ref MLPI_VERSION_FIRMWARE
//! @arg @ref MLPI_VERSION_HARDWARE
//! @arg @ref MLPI_VERSION_LOGIC
//! @arg @ref MLPI_VERSION_BSP
//! @arg @ref MLPI_VERSION_MLPI_SERVER_CORE
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    component         String that identifies the component for which version string was read (see global
//!                                 constants e.g. @ref MLPI_VERSION_FIRMWARE).
//! @param[out]   version           String where the version information will be stored.
//!                                 Note that the format of the version information string may differ depending on
//!                                 the selected component.
//! @param[in]    numElements       Number of WCHAR16 elements in 'version' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the firmware version of the device.
//! WCHAR16 version[128] = L"";
//! MLPIRESULT result = mlpiSystemGetVersionInfo(connection, MLPI_VERSION_FIRMWARE, version, _countof(version));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetVersionInfo(const MLPIHANDLE connection, const WCHAR16 *component, WCHAR16 *version, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function reads the current system language of the device (@ref MlpiLanguage).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   language          Pointer to variable where the currently selected system language will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the current system language of the device.
//! MlpiLanguage language = MLPI_LANGUAGE_GERMAN;
//! MLPIRESULT result = mlpiSystemGetLanguage(connection, &language);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetLanguage(const MLPIHANDLE connection, MlpiLanguage *language);


//! @ingroup SystemLibCommon
//! This function writes the system language of the device (@ref MlpiLanguage).
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    language          System language of the device to which it has to be switched.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the system language of the device.
//! MlpiLanguage language = MLPI_LANGUAGE_GERMAN;
//! MLPIRESULT result = mlpiSystemSetLanguage(connection, language);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetLanguage(const MLPIHANDLE connection, const MlpiLanguage language);


//! @ingroup SystemLibCommon
//! This function reads the device name.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   name              String where the device name will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'name' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the device name.
//! WCHAR16 name[128] = L"";
//! MLPIRESULT result = mlpiSystemGetName(connection, name, _countof(name));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetName(const MLPIHANDLE connection, WCHAR16 *name, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function writes the device name.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    name              String which to be written to the device name.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the device name.
//! WCHAR16 name[] = L"Control_42";
//! MLPIRESULT result = mlpiSystemSetName(connection, name);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetName(const MLPIHANDLE connection, const WCHAR16 *name);


//! @ingroup SystemLibCommon
//! This function reads the IP address of the device. Value is a string in quad-dotted notation of four decimal integers, e.g. "192.168.001.001" (IPv4).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   ipAddress         String where the IP address will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'ipAddress' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the IP address of the device.
//! WCHAR16 ipAddress[128] = L"";
//! MLPIRESULT result = mlpiSystemGetIpAddress(connection, ipAddress, _countof(ipAddress));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetIpAddress(const MLPIHANDLE connection, WCHAR16 *ipAddress, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function writes the IP address of the device. Value is a string in quad-dotted notation of four decimal integers, e.g. "192.168.001.001" (IPv4).
//! A reboot is necessary for the new IP address to become active.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    ipAddress         String to be written to the IP address.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the IP address of the device.
//! WCHAR16 ipAddress[] = L"192.168.1.42";
//! MLPIRESULT result = mlpiSystemSetIpAddress(connection, ipAddress);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetIpAddress(const MLPIHANDLE connection, const WCHAR16 *ipAddress);


//! @ingroup SystemLibCommon
//! This function reads the subnet mask of the device. Value is a string in quad-dotted notation of four decimal integers, e.g. "255.255.255.000" (IPv4).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   subnet            String where the subnet mask will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'subnet' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the subnet mask of the device.
//! WCHAR16 subnet[128] = L"";
//! MLPIRESULT result = mlpiSystemGetSubnetMask(connection, subnet, _countof(subnet));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetSubnetMask(const MLPIHANDLE connection, WCHAR16 *subnet, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function writes the subnet mask of the device. Value is a string in quad-dotted notation of four decimal integers, e.g. "255.255.255.000" (IPv4).
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    subnet            String to be written to the subnet mask.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the subnet mask of the device.
//! WCHAR16 subnet[] = L"255.255.255.0";
//! MLPIRESULT result = mlpiSystemSetSubnetMask(connection, subnet);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetSubnetMask(const MLPIHANDLE connection, const WCHAR16 *subnet);


//! @ingroup SystemLibCommon
//! This function reads the gateway address of the device. Value is a string in quad-dotted notation of four decimal integers, e.g. "192.168.001.001" (IPv4).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   gateway           String where the gateway address will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'gateway' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the gateway address of the device.
//! WCHAR16 gateway[128] = L"";
//! MLPIRESULT result = mlpiSystemGetGateway(connection, gateway, _countof(gateway));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetGateway(const MLPIHANDLE connection, WCHAR16 *gateway, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function writes the gateway address of the device. Value is a string in quad-dotted notation of four decimal integers, e.g. "192.168.001.001" (IPv4).
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    gateway           String to be written to the gateway address.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the gateway address of the device.
//! WCHAR16 gateway[] = L"192.168.1.1";
//! MLPIRESULT result = mlpiSystemSetGateway(connection, gateway);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetGateway(const MLPIHANDLE connection, const WCHAR16 *gateway);


//! @ingroup SystemLibCommon
//! This function reads the current device mode (@ref MlpiSystemMode).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   mode              Pointer to variable where the current mode of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the current mode of device.
//! MlpiSystemMode mode = MLPI_SYSTEMMODE_P0;
//! MLPIRESULT result = mlpiSystemGetCurrentMode(connection, &mode);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetCurrentMode(const MLPIHANDLE connection, MlpiSystemMode *mode);


//! @ingroup SystemLibCommon
//! This function writes the device mode (@ref MlpiSystemMode). After writing the mode, the device will NOT
//! be in this mode. It will be in the process of switching to this mode. In order to switch to a certain mode
//! and perform an action while in this mode, you have to set the mode then read it back until the desired mode
//! is reached.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    mode              Mode of the device to which it has to be switched.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the mode to which the device should be switched.
//! MlpiSystemMode mode = MLPI_SYSTEMMODE_P2;
//! MLPIRESULT result = mlpiSystemSetTargetMode(connection, mode);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetTargetMode(const MLPIHANDLE connection, const MlpiSystemMode mode);


//! @ingroup SystemLibCommon
//! This function reads the current temperature of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   temperature       Pointer to variable where the current temperature of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the current temperature of the device.
//! FLOAT temperature = 0.0;
//! MLPIRESULT result = mlpiSystemGetTemperature(connection, &temperature);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetTemperature(const MLPIHANDLE connection, FLOAT *temperature);


//! @ingroup SystemLibCommon
//! This function reads the maximum temperature of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   temperature       Pointer to variable where the maximum temperature of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum temperature of the device.
//! FLOAT temperature = 0.0;
//! MLPIRESULT result = mlpiSystemGetTemperatureMax(connection, &temperature);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetTemperatureMax(const MLPIHANDLE connection, FLOAT *temperature);


//! @ingroup SystemLibCommon
//! This function reset the temperature values of the device to the current temperature.
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Reset the temperature values of the device.
//! MLPIRESULT result = mlpiSystemResetTemperature(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemResetTemperature(const MLPIHANDLE connection);


//! @ingroup SystemLibCommon
//! This function reads the target ID of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   targetId          Pointer to variable where the target ID of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the target ID
//! LLONG targetId = 0;
//! MLPIRESULT result = mlpiSystemGetTargetId(connection, &targetId);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetTargetId(const MLPIHANDLE connection, LLONG* targetId);


//! @ingroup SystemLibCommon
//! This function reads the current CPU load device in percent.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   load              Pointer to variable where the current cpu load of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the current CPU load of the device in percent.
//! FLOAT load = 0.0;
//! MLPIRESULT result = mlpiSystemGetCpuLoad(connection, &load);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetCpuLoad(const MLPIHANDLE connection, FLOAT *load);


//! @ingroup SystemLibCommon
//! This function reads the maximum CPU load of the device in percent.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   load              Pointer to variable where the maximum cpu load of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum CPU load of the device in percent.
//! FLOAT load = 0.0;
//! MLPIRESULT result = mlpiSystemGetCpuLoadMax(connection, &load);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetCpuLoadMax(const MLPIHANDLE connection, FLOAT *load);


//! @ingroup SystemLibCommon
//! This function reset the CPU load values of the device to the current CPU load.
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Reset the CPU load values of the device.
//! MLPIRESULT result = mlpiSystemResetCpuLoad(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemResetCpuLoad(const MLPIHANDLE connection);


//! @ingroup SystemLibCommon
//! This function reads the operating time in industrial minutes of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   operatingTime     Pointer to variable where the operating time of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the operating time of the device.
//! ULONG operatingTime = 0;
//! MLPIRESULT result = mlpiSystemGetOperationHours(connection, &operatingTime);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetOperationHours(const MLPIHANDLE connection, ULONG *operatingTime);


//! @ingroup SystemLibCommon
//! This function reads the system date and time (broken down time, UTC) of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   dateTime          Pointer to structure where the date and time of the device will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the date and time of the device.
//! MlpiDateAndTime dateTime;
//! memset(&dateTime, 0, sizeof(dateTime));
//! MLPIRESULT result = mlpiSystemGetDateAndTimeUtc(connection, &dateTime);
//! printf("\nSystem time: %04u-%02u-%02u %02u-%02u-%02u (UTC)",
//!        dateTime.year, dateTime.month, dateTime.day,
//!        dateTime.hour, dateTime.minute, dateTime.second);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetDateAndTimeUtc(const MLPIHANDLE connection, MlpiDateAndTime *dateTime);


//! @ingroup SystemLibCommon
//! This function writes the system date and time (broken down time, UTC) of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    dateTime          Structure to be written to the date and time of the device.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the date and time of the device to 2012-12-21 11:55:42.
//! MlpiDateAndTime dateTime;
//! memset(&dateTime, 0, sizeof(dateTime));
//! dateTime.year = 2012;
//! dateTime.month = 12;
//! dateTime.day = 21;
//! dateTime.hour = 11;
//! dateTime.minute = 55;
//! dateTime.second = 42;
//! MLPIRESULT result = mlpiSystemSetDateAndTimeUtc(connection, dateTime);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetDateAndTimeUtc(const MLPIHANDLE connection, MlpiDateAndTime dateTime);


//! @ingroup SystemLibCommon
//! This function writes the system date and time of the device by a NTP server.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    timeout           Timeout until NTP server response.
//! @param[in]    server            NTP server URI.
//! @param[in]    port              Port of NTP server (optional).
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the date and time of the device by a NTP server.
//! WCHAR16 server[] = L"192.168.1.1";
//! ULONG timeout = 5000;
//! MLPIRESULT result = mlpiSystemSetDateAndTimeUtcByNtp(connection, timeout, server);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetDateAndTimeUtcByNtp(const MLPIHANDLE connection, const ULONG timeout, const WCHAR16* server, const USHORT port=0);


//! @ingroup SystemLibCommon
//! This function reads the serial number of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   serialNumber      String where the serial number will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'serialNumber' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the serial number of the device.
//! WCHAR16 serialNumber[128] = L"";
//! MLPIRESULT result = mlpiSystemGetSerialNumber(connection, serialNumber, _countof(serialNumber));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetSerialNumber(const MLPIHANDLE connection, WCHAR16 *serialNumber, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function reads the hardware details of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   hardwareDetails   String where the hardware details will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'hardwareDetails' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the hardware details of the device.
//! WCHAR16 hardwareDetails[512] = L"";
//! MLPIRESULT result = mlpiSystemGetHardwareDetails(connection, hardwareDetails, _countof(hardwareDetails));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetHardwareDetails(const MLPIHANDLE connection, WCHAR16 *hardwareDetails, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function reads the local bus configuration of the device. Value is a string in semicolon separated format
//! like "A005: netX (0x0100);A006: Sercos III PCI-BusMaster (0x000E);A007: n/a (0x0000);A254: SRAM (0x0013)".
//! Each separated segment contains the address, the name and an identification code (hex format).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   configuration     String where the local bus configuration will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'configuration' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the local bus configuration of the device.
//! WCHAR16 configuration[512] = L"";
//! MLPIRESULT result = mlpiSystemGetLocalBusConfiguration(connection, configuration, _countof(configuration));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetLocalBusConfiguration(const MLPIHANDLE connection, WCHAR16 *configuration, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function reads the function module bus configuration of the device. Value is a string in semicolon separated
//! format like "A001: SERCOS III (0x002B);A002: n/a (0x0000);A003: n/a (0x0000);A004: n/a (0x0000)". Each separated
//! segment contains the address, the name and the identification code (hex format).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   configuration     String where the function module bus configuration will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'configuration' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the function module bus configuration of the device.
//! WCHAR16 configuration[512] = L"";
//! MLPIRESULT result = mlpiSystemGetFunctionModulBusConfiguration(connection, configuration, _countof(configuration));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetFunctionModulBusConfiguration(const MLPIHANDLE connection, WCHAR16 *configuration, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function reads the hardware details of the function modules on the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    functionModuleNum Value indicating the number of the function module. Valid values are 1 to 4.
//! @param[out]   hardwareDetails   String where the hardware details will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'hardwareDetails' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the hardware details of the function module no. 1.
//! WCHAR16 hardwareDetails[512] = L"";
//! ULONG functionModuleNum = 1;
//! MLPIRESULT result = mlpiSystemGetFunctionModuleHardwareDetails(connection, functionModuleNum, hardwareDetails, _countof(hardwareDetails));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetFunctionModuleHardwareDetails(const MLPIHANDLE connection, const ULONG functionModuleNum, WCHAR16 *hardwareDetails, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function reads the MAC address of the device. Value is a string of format "00:00:00:00:00:00".
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   macAddress        String where the MAC address will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'macAddress' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the MAC address of the device.
//! WCHAR16 macAddress[128] = L"";
//! MLPIRESULT result = mlpiSystemGetMacAddress(connection, macAddress, _countof(macAddress));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetMacAddress(const MLPIHANDLE connection, WCHAR16 *macAddress, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function retrieves some information about the current free RAM on the device. The figures are given
//! in Bytes. The function can be used to trace the total system performance of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   memoryInfo        Pointer to structure where the memory information will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read information about the current free RAM of the device.
//! MlpiMemoryInfo memoryInfo;
//! memset(&memoryInfo, 0, sizeof(MlpiMemoryInfo));
//! MLPIRESULT result = mlpiSystemGetMemoryInfo(connection, &memoryInfo);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//! } else {
//!   printf("\nRAM Total %lf MB", (DOUBLE)memoryInfo.totalVolatileBytes/1024.0/1024.0);
//!   printf("\nRAM Free  %lf MB", (DOUBLE)memoryInfo.freeVolatileBytes/1024.0/1024.0);
//!   printf("\nNVRAM Total %lf MB", (DOUBLE)memoryInfo.totalNonvolatileBytes/1024.0/1024.0);
//!   printf("\nNVRAM Free  %lf MB", (DOUBLE)memoryInfo.freeNonvolatileBytes/1024.0/1024.0);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetMemoryInfo(const MLPIHANDLE connection, MlpiMemoryInfo *memoryInfo);


//! @ingroup SystemLibCommon
//! This function returns the platform specific absolute path of given symbolic path (@ref MlpiSpecialPath).
//! For example, it is possible to retrieve the absolute path of the system path on the connected device. This
//! information can then be used to access files via ftp on the given path.
//! This function is necessary because it is not guaranteed that the system path always has the same absolute path.
//! Those special paths can vary in different operating systems or on storage devices.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    pathIdentifier    Enumeration that identifies the path to query for.
//! @param[out]   path              String where the special path will be stored. Starts and ends with path delimiter ("/").
//! @param[in]    numElements       Number of WCHAR16 elements in 'path' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the system path of the device.
//! WCHAR16 path[512] = L"";
//! MlpiSpecialPath pathIdentifier = MLPI_PATH_SYSTEM;
//! MLPIRESULT result = mlpiSystemGetSpecialPath(connection, pathIdentifier, path, _countof(path));
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetSpecialPath(const MLPIHANDLE connection, const MlpiSpecialPath pathIdentifier, WCHAR16 *path, const ULONG numElements);


//! @ingroup SystemLibCommon
//! This function retrieves some information about the current spaces of special paths on the device.
//! The figures are given in Bytes.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    pathIdentifier    Variable that identifies the path to query for.
//! @param[out]   pathInfo          Pointer to structure where the path information will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read information (total size, free disk space) about the path.
//! MLPIRESULT result = MLPI_S_OK;
//! MlpiSpecialPath pathIdentifier = MLPI_PATH_USER;
//! MlpiSpecialPathInfo pathInfo;
//! memset(&pathInfo, 0, sizeof(MlpiSpecialPathInfo));
//! MLPIRESULT result = mlpiSystemGetSpecialPathInfo(connection, pathIdentifier, &pathInfo);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//! } else {
//!   printf("\nUSR-Disk Total %lf MB", (DOUBLE)pathInfo.totalBytes/1024.0/1024.0);
//!   printf("\nUSR-Disk Free  %lf MB", (DOUBLE)pathInfo.freeBytes/1024.0/1024.0);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetSpecialPathInfo(const MLPIHANDLE connection, const MlpiSpecialPath pathIdentifier, MlpiSpecialPathInfo *pathInfo);


//! @ingroup SystemLibCommon
//! This function returns all available platform specific root paths (e.g. "/ata0a/;/ata0b;...").
//! The root paths are separated by a semicolon (@c ;). The information about the root paths can be used to access
//! files via ftp. This function is necessary because it is not guaranteed that the root path always has the same
//! absolute path, meaning the root paths can varies in different operating systems or on storage devices.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   paths             String where all available root paths will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'paths' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read all available root paths of the control.
//! MLPIRESULT result = MLPI_S_OK;
//! WCHAR16 paths[512] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiSystemGetRootPaths(connection, paths, _countof(paths), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetRootPaths(const MLPIHANDLE connection, WCHAR16* paths, const ULONG numElements, ULONG* numElementsRet);


//! @ingroup SystemLibCommon
//! This function clears pending errors of the device.
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Clear pending errors.
//! MLPIRESULT result = mlpiSystemClearError(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemClearError(const MLPIHANDLE connection);


//! @ingroup SystemLibCommon
//! This function performs a reboot of the complete device. This is only allowed if the device
//! is switched to initialization mode (MLPI_SYSTEMMODE_P0 using @ref mlpiSystemSetTargetMode) before.
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Reboot device.
//! MLPIRESULT result = mlpiSystemReboot(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemReboot(const MLPIHANDLE connection);


//! @ingroup SystemLibCommon
//! This function performs a reboot and total clean up of the complete device. This is only allowed if the device
//! is switched to initialization mode (MLPI_SYSTEMMODE_P0  using @ref mlpiSystemSetTargetMode) before.
//! Please note that the device reboots immediately
//! and that all project data is deleted during the next boot up.
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Reboot and clean up device.
//! MLPIRESULT result = mlpiSystemCleanup(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemCleanup(const MLPIHANDLE connection);


//! @ingroup SystemLibCommon
//! This function reads the license information of the device (raw format).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   licenses          String where the licence information will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'licenses' available to read.
//! @param[in]    numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the license information of the device.
//! MLPIRESULT result = MLPI_S_OK;
//! WCHAR16 licenses[512] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiSystemGetLicenseInformation(connection, license, _countof(licenses), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetLicenseInformation(const MLPIHANDLE connection, WCHAR16 *licenses, const ULONG numElements, ULONG* numElementsRet);


//! @ingroup SystemLibCommon
//! This function reads a limit (e.g. maximum number of applications) of the device (@ref MlpiSystemLimit).
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   limit             Limit selector.
//! @param[out]   value             Limit value.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum number of application of device.
//! LLONG max = 0;
//! MLPIRESULT result = mlpiSystemGetLimit(connection, MLPI_SYSTEM_LIMIT_MAX_APPLICATIONS, &max);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetLimit(const MLPIHANDLE connection, MlpiSystemLimit limit, LLONG *value);


//! @ingroup SystemLibDiagnosis
//! This function reads the displayed diagnostic message of the device.
//! The displayed diagnosis is the diagnosis with the highest severity of all currently active diagnoses.
//! This means, that the displayed diagnostic message can differ from the latest or last inserted diagnostic message.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   diagnosis         Pointer to structure where the diagnosis will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the displayed diagnostic message of the device.
//! MlpiDiagnosis diagnosis;
//! memset(&diagnosis, 0, sizeof(diagnosis));
//! MLPIRESULT result = mlpiSystemGetDisplayedDiagnosis(connection, &diagnosis);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//! } else {
//!   printf("\n0x%08X %s", diagnosis.number, W2A16(diagnosis.text));
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetDisplayedDiagnosis(const MLPIHANDLE connection, MlpiDiagnosis *diagnosis);


//! @ingroup SystemLibDiagnosis
//! This function writes a diagnosis with message to the diagnosis system of the device. The diagnostic group is set
//! to 0x3F automatically.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    category          Diagnosis category (@ref MlpiDiagnosisCategory).
//! @param[in]    number            User specific diagnosis number.
//! @param[in]    diagnosisText     Diagnosis text. @note The maximum length of a diagnosis text is 60 characters.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write a diagnosis with message to the diagnosis system of the device.
//! USHORT number = 1;
//! WCHAR16 diagnosisText[] = L"This is a new diagnosis";
//! MLPIRESULT result = mlpiSystemSetDiagnosis(connection, MLPI_DIAGNOSIS_ERROR_NONFATAL, number, diagnosisText);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetDiagnosis(const MLPIHANDLE connection, const MlpiDiagnosisCategory category, const USHORT number, const WCHAR16 *diagnosisText=0);


//! @ingroup SystemLibDiagnosis
//! Each new diagnosis in the diagnosis logbook gets a unique incremented diagnosis index. This function returns
//! the index of the latest and therefore most recently inserted diagnosis. This index can be used as a parameter
//! using the function @ref mlpiSystemGetDiagnosisLog to read the complete logbook of the newest diagnostic messages.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   index             Pointer to variable where the index of the newest diagnosis will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the index of the latest diagnosis in the diagnosis logbook of the device.
//! ULONG index = 0;
//! MLPIRESULT result = mlpiSystemGetNewestDiagnosisIndex(connection, &index);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetNewestDiagnosisIndex(const MLPIHANDLE connection, ULONG *index);


//! @ingroup SystemLibDiagnosis
//! As the device is limited in memory resources, it is not possible to hold all diagnostic messages in the logbook
//! from the start of power up. Because of that, older diagnostic messages are first swapped to disk and then discarded.
//! Use this function to get the index of the oldest diagnostic message that is still in memory and can be returned by
//! the function @ref mlpiSystemGetDiagnosisLog.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   index             Pointer to variable where the index of oldest diagnosis will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the index of oldest diagnosis in the diagnosis logbook of the device.
//! ULONG index = 0;
//! MLPIRESULT result = mlpiSystemGetOldestDiagnosisIndex(connection, &index);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetOldestDiagnosisIndex(const MLPIHANDLE connection, ULONG *index);


//! @ingroup SystemLibDiagnosis
//! This function reads the device diagnosis log. The function returns an array of diagnostic messages
//! starting from newer to older diagnostic messages.
//! Each diagnosis in the diagnosis logbook has a unique incrementing diagnosis index. You need to specify from which
//! index you want to start reading the diagnosis logbook. To get the index of the latest and thus most recently inserted
//! diagnosis, you can use the function @ref mlpiSystemGetNewestDiagnosisIndex.
//! As the device is limited in memory resources, it is not possible to hold all diagnostic messages in the logbook
//! from the start of power up. Because of that, older diagnostic messages are first swapped to disk and then discarded.
//! Use the function @ref mlpiSystemGetOldestDiagnosisIndex to get the oldest available diagnosis to read.
//! Diagnostic messages which are not located between the oldest and newest diagnosis index can no longer be read by
//! this function. This function will therefore return the number of diagnoses that have actually been read.
//! This function will not return an error if the number of diagnoses you want to read is larger than the number of diagnoses
//! that are available.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    index             Index of the first diagnosis index to start reading.
//! @param[out]   diagnosisLog      Pointer to an array of structure where the diagnosis will be stored.
//! @param[in]    numElements       Number of MlpiDiagnosis elements available in 'diagnosisLog' for reading.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the complete diagnosis log of the device.
//! MlpiDiagnosis diagnosisLog[1000];
//! ULONG numElementsRet = 0;
//! ULONG indexNewest = 0;
//! ULONG indexOldest = 0;
//! memset(diagnosisLog, 0, sizeof(diagnosisLog));
//!
//! mlpiSystemGetOldestDiagnosisIndex(connection, &indexOldest);
//! mlpiSystemGetNewestDiagnosisIndex(connection, &indexNewest);
//! mlpiSystemGetDiagnosisLog(connection, indexNewest, diagnosisLog, min(indexNewest-indexOldest,_countof(diagnosisLog)), &numElementsRet);
//!
//! for (ULONG i=0; i<numElementsRet; i++){
//!   printf("\n%i 0x%X %s", diagnosisLog[i].index, diagnosisLog[i].number, W2A16(diagnosisLog[i].text));
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetDiagnosisLog(const MLPIHANDLE connection, const ULONG index, MlpiDiagnosis *diagnosisLog, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup SystemLibDiagnosis
//! This function tries to retrieve a textual description for a given diagnosis number. Please note that this is not
//! possible for every diagnosis number that is returned from the system. This includes user-specific diagnosis numbers
//! or diagnosis numbers which represent a group of error conditions.
//! In case no match was found, the string "No entry found in data base" is returned.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    diagnosisNumber   Diagnosis number.
//! @param[out]   diagnosisText     String where the diagnosis text will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements available in 'diagnosisText' for reading.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the textual description for a diagnosis number.
//! ULONG diagnosisNumber = 0xF02D1001;
//! WCHAR16 diagnosisText[256] = L"";
//! MLPIRESULT result = mlpiSystemConvertDiagnosisNumberToText(connection, diagnosisNumber, diagnosisText, _countof(diagnosisText));
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//! } else {
//!   printf("\n%08X means: %s", diagnosisNumber, W2A16(diagnosisText));
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemConvertDiagnosisNumberToText(const MLPIHANDLE connection, const ULONG diagnosisNumber, WCHAR16 *diagnosisText, const ULONG numElements);


//! @ingroup SystemLibDiagnosis
//! This function switch on the boot LED 'BT' (if available) regarding the selected @ref MlpiLedPattern pattern.
//!
//! @note Because the system use the boot LED 'BT' for signal boot status, the function will return
//!       an error if system is still booting.
//!       - MLPI_E_NOTINITIALIZED:  Boot end not achieved yet.
//!       - MLPI_E_STARTERROR:      Boot finished with error(s), reboot may be required.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    pattern           LED pattern.
//! @param[in]    count             Count of repeat pattern. Use "-1" for repeat infinite.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Switch on the boot LED 'BT' permanently.
//! MLPIRESULT result = mlpiSystemSetBootLed(connection, MLPI_LED_PATTERN_GREEN);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemSetBootLed(const MLPIHANDLE connection, const MlpiLedPattern pattern, const LONG count = -1);


//! @ingroup SystemLibDiagnosis
//! This function switch off the boot LED 'BT' (if available).
//!
//! @note Because the system use the boot LED 'BT' for signal boot status, the function will return
//!       an error if system is still booting.
//!       - MLPI_E_NOTINITIALIZED:  Boot end not achieved yet.
//!       - MLPI_E_STARTERROR:      Boot finished with error(s), reboot may be required.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Switch off the boot LED 'BT'.
//! MLPIRESULT result = mlpiSystemClearBootLed(connection);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiSystemClearBootLed(const MLPIHANDLE connection);


//! @ingroup SystemLibTransmission
//! This function starts a transmission between control and a remote device. A source and destination must be provided in URI
//! format (see RFC 3986). At the moment only "file" and tftp" is supported as scheme. URI format for Scheme "tftp" is defined
//! in RFC 3617.
//! \n
//! Examples for valid URI
//! \li file:///ata0b/localfile.txt
//! \li tftp://172.31.254.1/remotefile.txt
//! \li tftp://172.31.254.1:69/remotefile.txt
//! \n
//! Handle is returned to track the transmission.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    source            String that contains the source URI.
//! @param[in]    destination       String that contains the destination URI.
//! @param[out]   handle            Handle to track transmission.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // start TFTP put transfer
//! MLPITASKHANDLE tftpHandle;
//! MLPIRESULT result = mlpiSystemStartTransmission(connection, "file:///ata0b/local.txt", "tftp://172.168.31.1:69/remote.txt", &tftpHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemStartTransmission(const MLPIHANDLE connection, const WCHAR16* source, const WCHAR16* destination, MLPITASKHANDLE* handle);


//! @ingroup SystemLibTransmission
//! This function retrieves the status of one or more transmissions.
//! @param[in]     connection        Handle for multiple connections.
//! @param[in,out] status            Pointer to one or more MlpiTransmissionStatus structures containing the status of the transmissions. At least the structure element "handle" must be filled when passing the structure to the function.
//! @param[in]     numElements       Count of status.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // start TFTP get transfer
//! MlpiTransmissionStatus status[2];
//! mlpiSystemStartTransmission(connection, "file:///ata0b/local.txt", "tftp://172.168.31.1/remote.txt", &status[0].handle);
//! mlpiSystemStartTransmission(connection, "file:///ata0b/local.txt", "tftp://172.168.31.2/remote.txt", &status[1].handle);
//! MLPIRESULT result = mlpiSystemGetTransmissionStatus(connection, status, 2);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetTransmissionStatus(const MLPIHANDLE connection, MlpiTransmissionStatus* status, ULONG numElements);


//! @ingroup SystemLibTransmission
//! This function retrieves detailed information of a transmission.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    handle            Handle of a transmission.
//! @param[out]   info              Detailed information of a TFTP transmission.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPITASKHANDLE handle;
//! MlpiTransmissionInfo info;
//! mlpiSystemStartTransmission(connection, "file:///ata0b/local.txt", "tftp://172.168.31.1/remote.txt", &handle);
//! MLPIRESULT result = mlpiSystemGetTransmissionInfo(connection, tftpHandle, &info);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemGetTransmissionInfo(const MLPIHANDLE connection, const MLPITASKHANDLE handle, MlpiTransmissionInfo* info);


//! @ingroup SystemLibTransmission
//! This function sets the timeout of future TFTP transmissions.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    seconds           New timeout of future TFTP transmissions.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // set TFTP timeout to 10 seconds
//! MLPIRESULT result = mlpiSystemTftpSetTimeout(connection, 10);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemTftpSetTimeout(const MLPIHANDLE connection, const USHORT seconds);


//! @ingroup SystemLibTransmission
//! This function gets the timeout of future TFTP transmissions.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   seconds           Timeout of future TFTP transmissions.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // get TFTP timeout
//! USHORT timeout;
//! MLPIRESULT result = mlpiSystemTftpGetTimeout(connection, &timeout);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemTftpGetTimeout(const MLPIHANDLE connection, USHORT* seconds);


//! @ingroup SystemLibTransmission
//! This function sets the maximum retransmit count of future TFTP transmissions.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    count             New maximum retransmit count of future TFTP transmissions.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // set TFTP maximum retransmit count to 5
//! MLPIRESULT result = mlpiSystemTftpSetRetransmitCount(connection, 5);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemTftpSetRetransmitCount(const MLPIHANDLE connection, const USHORT count);


//! @ingroup SystemLibTransmission
//! This function gets the maximum retransmit count of future TFTP transmissions.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   count             Maximum retransmit count of future TFTP transmissions.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // get TFTP maximum retransmit count
//! USHORT maxRetransmitCount;
//! MLPIRESULT result = mlpiSystemTftpGetRetransmitCount(connection, &maxRetransmitCount);
//! @endcode
MLPI_API MLPIRESULT mlpiSystemTftpGetRetransmitCount(const MLPIHANDLE connection, USHORT* count);


#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPISYSTEMLIB_H__

