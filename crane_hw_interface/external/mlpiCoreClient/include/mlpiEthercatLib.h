#ifndef __MLPIETHERCATLIB_H__
#define __MLPIETHERCATLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiEthercatLib.h>
// -----------------------------------------------------------------------
// Copyright (c) 2017 Bosch Rexroth. All rights reserved.
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
//! @author     DC-IA/EAO5 (ME)
//!
//! @copyright  Bosch Rexroth Corporation http://www.boschrexroth.com/oce
//!
//! @version    1.22.0
//!
//! @date       2017
//
// -----------------------------------------------------------------------


//! @addtogroup EthercatLib EthercatLib
//! @{
//! @brief This library contains functions for configuration of the EtherCAT master.
//!
//! @note EtherCAT&reg; is a registered trademark and patented technology, licensed by Beckhoff Automation GmbH, Germany.
//!
//! @section sec_EcmAcronyms Acronyms regarding EtherCAT
//!
//! <TABLE>
//! <TR><TH> Acronym       </TH><TH> Description                                                                </TH></TR>
//! <TR><TD> ADS           </TD><TD> Automation Device Specification                                            </TD></TR>
//! <TR><TD> AL            </TD><TD> Application Layer                                                          </TD></TR>
//! <TR><TD> AoE           </TD><TD> ADS over EtherCAT                                                          </TD></TR>
//! <TR><TD> API           </TD><TD> Application Programming Interface                                          </TD></TR>
//! <TR><TD> CAN           </TD><TD> Controller Area Network                                                    </TD></TR>
//! <TR><TD> CoE           </TD><TD> CANopen over EtherCAT                                                      </TD></TR>
//! <TR><TD> CRC           </TD><TD> Cyclic Redundancy Check                                                    </TD></TR>
//! <TR><TD> DC            </TD><TD> Distributed Clocks                                                         </TD></TR>
//! <TR><TD> DL            </TD><TD> Data Link                                                                  </TD></TR>
//! <TR><TD> EAP           </TD><TD> EtherCAT Automation Protocol                                               </TD></TR>
//! <TR><TD> EC / ECAT     </TD><TD> EtherCAT                                                                   </TD></TR>
//! <TR><TD> EEPROM        </TD><TD> Electrical Erasable Programmable Read Only Memory                          </TD></TR>
//! <TR><TD> ENI           </TD><TD> EtherCAT Network Information (used for master configuration)               </TD></TR>
//! <TR><TD> EoE           </TD><TD> Ethernet over EtherCAT                                                     </TD></TR>
//! <TR><TD> EPU           </TD><TD> EtherCAT Processing Unit                                                   </TD></TR>
//! <TR><TD> ESC           </TD><TD> EtherCAT Slave Controller                                                  </TD></TR>
//! <TR><TD> ESI           </TD><TD> EtherCAT Slave Information (slave device description)                      </TD></TR>
//! <TR><TD> ESM           </TD><TD> EtherCAT State Machine                                                     </TD></TR>
//! <TR><TD> ETG           </TD><TD> EtherCAT Technology Group                                                  </TD></TR>
//! <TR><TD> EtherCAT      </TD><TD> Ethernet for Control and Automation Technology                             </TD></TR>
//! <TR><TD> FoE           </TD><TD> File Access of EtherCAT                                                    </TD></TR>
//! <TR><TD> FMMU          </TD><TD> Fieldbus Memory Management Unit                                            </TD></TR>
//! <TR><TD> FSoE          </TD><TD> Fail Safe over EtherCAT                                                    </TD></TR>
//! <TR><TD> HC            </TD><TD> Hot Connect                                                                </TD></TR>
//! <TR><TD> IDN           </TD><TD> Identification Number                                                      </TD></TR>
//! <TR><TD> Mbx           </TD><TD> Mailbox (acyclic communication)                                            </TD></TR>
//! <TR><TD> OD            </TD><TD> Object Dictionary                                                          </TD></TR>
//! <TR><TD> PD            </TD><TD> Process Data                                                               </TD></TR>
//! <TR><TD> PDI           </TD><TD> Process Data Interface                                                     </TD></TR>
//! <TR><TD> PDO           </TD><TD> Process Data Object                                                        </TD></TR>
//! <TR><TD> RO            </TD><TD> Read Only                                                                  </TD></TR>
//! <TR><TD> RAS           </TD><TD> Remote Access Service                                                      </TD></TR>
//! <TR><TD> RW            </TD><TD> Read Write                                                                 </TD></TR>
//! <TR><TD> Rx            </TD><TD> Receive                                                                    </TD></TR>
//! <TR><TD> SDO           </TD><TD> Service Data Object                                                        </TD></TR>
//! <TR><TD> SII           </TD><TD> Slave Information Interface                                                </TD></TR>
//! <TR><TD> SM            </TD><TD> Sync Manager                                                               </TD></TR>
//! <TR><TD> SoE           </TD><TD> Servo drive over EtherCAT                                                  </TD></TR>
//! <TR><TD> SyncM         </TD><TD> Sync Manager                                                               </TD></TR>
//! <TR><TD> TFTP          </TD><TD> Trivial File Transfer Protocol                                             </TD></TR>
//! <TR><TD> Tx            </TD><TD> Transmit                                                                   </TD></TR>
//! <TR><TD> VoE           </TD><TD> Vendor specific over EtherCAT                                              </TD></TR>
//! <TR><TD> WO            </TD><TD> Write Only                                                                 </TD></TR>
//! <TR><TD> WKC           </TD><TD> Working Counter                                                            </TD></TR>
//! <TR><TD> XML           </TD><TD> Extensible Markup Language                                                 </TD></TR>
//! </TABLE>
//!
//!
//! @}


//! @addtogroup EthercatLibConfig Configuration functions
//! @ingroup EthercatLib
//! @{
//! @brief Contains functions to configure the EtherCAT master and its slaves.
//! @}

//! @addtogroup EthercatLibInfo EtherCAT information
//! @ingroup EthercatLib
//! @{
//! @brief Contains functions to read information of the EtherCAT master and its slaves.
//! @}

//! @addtogroup EthercatLibDll Data link layer
//! @ingroup EthercatLib
//! @{
//! @brief Contains functions to read/write information on the data link layer.
//! @}

//! @addtogroup EthercatLibCoE CANopen over EtherCAT
//! @ingroup EthercatLib
//! @{
//! @brief Contains functions to communicate with slaves over CANopen application protocol over EtherCAT.
//! @}

//! @addtogroup EthercatLibSoE Servo drive over EtherCAT
//! @ingroup EthercatLib
//! @{
//! @brief Contains functions to communicate with slaves over Servo drive over EtherCAT.
//! @}

//! @addtogroup EthercatLibVersionPermission Version and Permission
//! @ingroup EthercatLib
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_ETHERCATLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_ETHERCATLIB_PERMISSION_ALWAYS cannot be blocked.
//!
//! @par List of permissions of mlpiEthercatLib used in accounts.xml
//! - MLPI_ETHERCATLIB_PERMISSION_ALL
//! - MLPI_ETHERCATLIB_PERMISSION_CONFIG
//! - MLPI_ETHERCATLIB_PERMISSION_INFO
//! - MLPI_ETHERCATLIB_PERMISSION_OPERATION
//! - MLPI_ETHERCATLIB_PERMISSION_DATA_READ
//! - MLPI_ETHERCATLIB_PERMISSION_DATA_WRITE
//!
//! <TABLE>
//! <TR><TH>           Function                                         </TH><TH> Server version </TH><TH> Permission                                   </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetTopologyStatus               </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetConfigFiles                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSetConfigFiles                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatControl                         </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetMasterState                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSetMasterState                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_OPERATION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetNumConfiguredSlaves          </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetNumConnectedSlaves           </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetSlaveState                   </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSetSlaveState                   </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_OPERATION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetSlaveOnlineInfo              </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetSlaveConfigInfo              </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetSlavePortState               </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatReadSlaveRegister               </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_OPERATION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatWriteSlaveRegister              </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatResetSlaveController            </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatReadSlaveEeprom                 </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatWriteSlaveEeprom                </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatReloadSlaveEeprom               </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatAssignSlaveEeprom               </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatCheckSlaveEeprom                </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatReadSlaveIdentification         </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetSlaveStatistics              </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatResetSlaveStatistics            </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_OPERATION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSetSlaveStatisticsPeriod        </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatCoeGetOdList                    </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatCoeGetObjectDescription         </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatCoeGetEntryDescription          </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatCoeSdoUpload                    </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_DATA_READ"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatCoeSdoDownload                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_DATA_WRITE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSoeRead                         </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_DATA_WRITE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSoeWrite                        </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_DATA_READ"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetVendorInfo                   </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGenerateEsi                     </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_CONFIG"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatStartBusScan                    </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_OPERATION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetBusScanStatus                </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetBusScanSlaveInfo             </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetProcessData                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatSetProcessData                  </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_DATA_WRITE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiEthercatGetProcessDataSize              </TD><TD> 1.18.0.0       </TD><TD> "MLPI_ETHERCATLIB_PERMISSION_INFO"           </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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

//! @addtogroup EthercatLibStructTypes Structs, types, ...
//! @ingroup EthercatLib
//! @{
//! @brief List of used types, enumerations, structures and more...




// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"



// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
//! Maximum number of slave ports.
#define MLPI_ETHERCAT_MAX_ESC_PORTS               ( 4 )

//! Maximum number of slave ports.
#define MLPI_ETHERCAT_MAX_SLAVE_NAME_LEN          ( 128 )

//! Maximum number of slave ports.
#define MLPI_ETHERCAT_MAX_VENDOR_DATA_STRING_LEN  ( 128 )

//! Maximum number of supported process data sections.
#define MLPI_ETHERCAT_MAX_PROCESS_DATA_SECTIONS   ( 4 )


// -----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
// -----------------------------------------------------------------------

//! @enum MlpiEthercatAddressType
//! This enumeration specifies the mode of addressing.
typedef enum MlpiEthercatAddressType
{
  MLPI_ETHERCAT_AUTO_INCREMENT  = 0,     //!< Incremental addressing.
  MLPI_ETHERCAT_FIXED_PHYSICAL  = 1      //!< Physical addressing.
} MlpiEthercatAddressType;

//! @enum MlpiEthercatTopologyStatus
//! This enumeration indicates the status of topology detection.
typedef enum MlpiEthercatTopologyStatus
{
  MLPI_ETHERCAT_TOPOLOGY_STATUS_UNKNOWN             = 0,    //!< Topology unknown.
  MLPI_ETHERCAT_TOPOLOGY_STATUS_VALID               = 1,    //!< Topology verified and valid (e.g. elements of MlpiEthercatSlaveOnlineInfo can be used for drawing bus topology).
  MLPI_ETHERCAT_TOPOLOGY_STATUS_PENDING             = 2     //!< Topology detection still pending / in progress.
} MlpiEthercatTopologyStatus;

//! @enum MlpiEthercatState
//! This enumeration represents the specific states of the EtherCAT State Machine.
typedef enum MlpiEthercatState
{
  MLPI_ETHERCAT_STATE_UNKNOWN   = 0,     //!< Unknown.
  MLPI_ETHERCAT_STATE_INIT      = 1,     //!< Init.
  MLPI_ETHERCAT_STATE_PREOP     = 2,     //!< Pre-operational.
  MLPI_ETHERCAT_STATE_BOOTSTRAP = 3,     //!< BootStrap.
  MLPI_ETHERCAT_STATE_SAFEOP    = 4,     //!< Safe operational.
  MLPI_ETHERCAT_STATE_OP        = 8      //!< Operational.
} MlpiEthercatState;

//! @enum MlpiEthercatOdListType
//! This enumeration specifies the type of object dictionary list.
typedef enum MlpiEthercatOdListType
{
  MLPI_ETHERCAT_OD_LIST_TYPE_LENGTHS                = 0,     //!< Lengths of each list type.
  MLPI_ETHERCAT_OD_LIST_TYPE_ALL                    = 1,     //!< All objects.
  MLPI_ETHERCAT_OD_LIST_TYPE_RXPDOMAP               = 2,     //!< Only pdo mappable objects.
  MLPI_ETHERCAT_OD_LIST_TYPE_TXPDOMAP               = 3,     //!< Only pdo mappable objects that can be changed.
  MLPI_ETHERCAT_OD_LIST_TYPE_STORED_FOR_REPLACEMENT = 4,     //!< Only stored for a device replacement objects.
  MLPI_ETHERCAT_OD_LIST_TYPE_STARTUP_PARAM          = 8      //!< Only startup parameter objects.
} MlpiEthercatOdListType;

//! @enum MlpiEthercatProcessDataType
//! This enumeration specifies the type of process data.
typedef enum MlpiEthercatProcessDataType
{
  MLPI_ETHERCAT_PD_TYPE_INPUTS                = 0,    //!< Read input data.
  MLPI_ETHERCAT_PD_TYPE_OUTPUTS               = 1     //!< Write output data.
} MlpiEthercatProcessDataType;



// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiEthercatIdentityInfo
//! @brief This structure containing slave information.
//! @details Elements of struct MlpiEthercatIdentityInfo
//! <TABLE>
//! <TR><TH>                Type              </TH><TH>           Element                   </TH><TH> Description               </TH></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> vendorId                  </TD><TD> Vendor identification.    </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> productCode               </TD><TD> Product code.             </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> revisionNumber            </TD><TD> Revision number.          </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> serialNumber              </TD><TD> Serial number.            </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatIdentityInfo
{
  ULONG     vendorId;                       //!< Vendor identification.
  ULONG     productCode;                    //!< Product code.
  ULONG     revisionNumber;                 //!< Revision number.
  ULONG     serialNumber;                   //!< Serial number.
}MlpiEthercatIdentityInfo;

//! @typedef MlpiEthercatMailboxInfo
//! @brief This structure containing mailbox information.
//! @details Elements of struct MlpiEthercatMailboxInfo
//! <TABLE>
//! <TR><TH>                Type              </TH><TH>           Element                   </TH><TH> Description                                                     </TH></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> sizeIn                    </TD><TD> Mailbox input size in unit of bytes (e.g. ENI: Mailbox/Recv).   </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> sizeOut                   </TD><TD> Mailbox output size in unit of bytes (e.g. ENI: Mailbox/Send).  </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatMailboxInfo
{
  ULONG     sizeIn;                         //!< Mailbox input size in unit of bytes (e.g. ENI: Mailbox/Recv).
  ULONG     sizeOut;                        //!< Mailbox output size in unit of bytes (e.g. ENI: Mailbox/Send).
}MlpiEthercatMailboxInfo;

//! @typedef MlpiEthercatMemoryInfo
//! @brief This structure describing a specific memory area.
//! @details Elements of struct MlpiEthercatMemoryInfo
//! <TABLE>
//! <TR><TH>                Type              </TH><TH>           Element                   </TH><TH> Description     </TH></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> offset                    </TD><TD> Memory offset.  </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> size                      </TD><TD> Memory size.    </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatMemoryInfo
{
  ULONG     offset;                         //!< Memory offset.
  ULONG     size;                           //!< Memory size.
}MlpiEthercatMemoryInfo;

//! @typedef MlpiEthercatSlaveOnlineInfo
//! @brief This structure defines the slave online info and is used in @ref mlpiEthercatGetSlaveOnlineInfo.
//! @details Elements of struct MlpiEthercatSlaveOnlineInfo
//! <TABLE>
//! <TR><TH>                Type                      </TH><TH>           Element               </TH><TH> Description                                                                                                                           </TH></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> autoIncAddr           </TD><TD> Auto increment address.                                                                                                               </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> ethercatAddr          </TD><TD> The slave station address / EtherCAT address (Value of slave ESC register 0x0010).                                                    </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> stationAlias          </TD><TD> The slave alias address (Value of slave ESC register 0x0012).                                                                         </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> identifyValue         </TD><TD> Last read identification value.                                                                                                       </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> slaveHandle           </TD><TD> Internal management handle (unique identification).                                                                                   </TD></TR>
//! <TR><TD id="st_t">      ULONG[]                   </TD><TD id="st_e"> portSlaveHandles      </TD><TD> Link handles to connected slaves.                                                                                                     </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatIdentityInfo  </TD><TD id="st_e"> slaveIdentity         </TD><TD> Slave information stored in EEPROM (beginning at offset 0x0008).                                                                      </TD></TR>
//! <TR><TD id="st_t">      UCHAR                     </TD><TD id="st_e"> escType               </TD><TD> Type of ESC (Value of slave ESC register 0x0000).                                                                                     </TD></TR>
//! <TR><TD id="st_t">      UCHAR                     </TD><TD id="st_e"> escRevision           </TD><TD> Revision number of ESC (Value of slave ESC register 0x0001).                                                                          </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> escBuild              </TD><TD> Build number of ESC (Value of slave ESC register 0x0002).                                                                             </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> escFeatures           </TD><TD> Features supported (Value of slave ESC register 0x0008).                                                                              </TD></TR>
//! <TR><TD id="st_t">      UCHAR                     </TD><TD id="st_e"> portDescriptor        </TD><TD> Port descriptor (Value of slave ESC register 0x0007).                                                                                 </TD></TR>
//! <TR><TD id="st_t">      UCHAR                     </TD><TD id="st_e"> reserved01            </TD><TD> Reserved.                                                                                                                             </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> alStatus              </TD><TD> AL status (Value of slave ESC register 0x0130).                                                                                       </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> alStatusCode          </TD><TD> AL status code. (Value of slave ESC register 0x0134 during last error acknowledge). This value is reset after a slave state change.   </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> mbxProtocols          </TD><TD> Supported Mailbox Protocols stored in the EEPROM at offset 0x001C.                                                                    </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> dlStatus              </TD><TD> DL status (Value of slave ESC register 0x0110).                                                                                       </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> portState             </TD><TD> Port link state. Detailed description at @ref mlpiEthercatGetSlavePortState.                                                          </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> previousPort          </TD><TD> Connected port of the previous slave.                                                                                                 </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> systemTimeDifference  </TD><TD> System time difference. (Value of slave ESC register 0x092C).                                                                         </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> slaveDelay            </TD><TD> Delay behind slave in ns. This value is only valid if a DC configuration is used.                                                     </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> propagationDelay      </TD><TD> Propagation delay in ns. This value is only valid if a DC configuration is used.                                                      </TD></TR>
//! <TR><TD id="st_t">      ULONG[]                   </TD><TD id="st_e"> reserved02            </TD><TD> Reserved.                                                                                                                             </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> dcSupport             </TD><TD> Slave supports DC.                                                                                                                    </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> dc64Support           </TD><TD> Slave supports 64 bit DC.                                                                                                             </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> isRefClock            </TD><TD> Slave is reference clock.                                                                                                             </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> lineCrossed           </TD><TD> Line crossed was detected at this slave.                                                                                              </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatSlaveOnlineInfo
{
  USHORT                    autoIncAddr;                                      //!< Auto increment address.
  USHORT                    ethercatAddr;                                     //!< The slave station address / EtherCAT address (Value of slave ESC register 0x0010).
  USHORT                    stationAlias;                                     //!< The slave alias address (Value of slave ESC register 0x0012).
  USHORT                    identifyValue;                                    //!< Last read identification value.
  ULONG                     slaveHandle;                                      //!< Internal management handle (unique identification).
  ULONG                     portSlaveHandles[MLPI_ETHERCAT_MAX_ESC_PORTS];    //!< Link handles to connected slaves.
  MlpiEthercatIdentityInfo  slaveIdentity;                                    //!< Slave information stored in EEPROM (beginning at offset 0x0008).
  UCHAR                     escType;                                          //!< Type of ESC (Value of slave ESC register 0x0000).
  UCHAR                     escRevision;                                      //!< Revision number of ESC (Value of slave ESC register 0x0001).
  USHORT                    escBuild;                                         //!< Build number of ESC (Value of slave ESC register 0x0002).
  USHORT                    escFeatures;                                      //!< Features supported (Value of slave ESC register 0x0008).
  UCHAR                     portDescriptor;                                   //!< Port descriptor (Value of slave ESC register 0x0007).
  UCHAR                     reserved01;                                       //!< Reserved.
  USHORT                    alStatus;                                         //!< AL status (Value of slave ESC register 0x0130).
  USHORT                    alStatusCode;                                     //!< AL status code. (Value of slave ESC register 0x0134 during last error acknowledge). This value is reset after a slave state change.
  USHORT                    mbxProtocols;                                     //!< Supported Mailbox Protocols stored in the EEPROM at offset 0x001C.
  USHORT                    dlStatus;                                         //!< DL status (Value of slave ESC register 0x0110).
  USHORT                    portState;                                        //!< Port link state. Detailed description at @ref mlpiEthercatGetSlavePortState.
  USHORT                    previousPort;                                     //!< Connected port of the previous slave.
  ULONG                     systemTimeDifference;                             //!< System time difference. (Value of slave ESC register 0x092C).
  ULONG                     slaveDelay;                                       //!< Delay behind slave in ns. This value is only valid if a DC configuration is used.
  ULONG                     propagationDelay;                                 //!< Propagation delay in ns. This value is only valid if a DC configuration is used.
  ULONG                     reserved02[4];                                    //!< Reserved.
  BOOL8                     dcSupport;                                        //!< Slave supports DC.
  BOOL8                     dc64Support;                                      //!< Slave supports 64 bit DC.
  BOOL8                     isRefClock;                                       //!< Slave is reference clock.
  BOOL8                     lineCrossed;                                      //!< Line crossed was detected at this slave.
}MlpiEthercatSlaveOnlineInfo;

//! @typedef MlpiEthercatSlaveConfigInfo
//! @brief This structure defines the the slave config info and is used in @ref mlpiEthercatGetSlaveConfigInfo.
//! @details Elements of struct MlpiEthercatSlaveConfigInfo
//! <TABLE>
//! <TR><TH>                Type                      </TH><TH>           Element                   </TH><TH> Description.                                                                    </TH></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> autoIncAddr               </TD><TD> Auto increment address                                                          </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> ethercatAddr              </TD><TD> The slave's station address / EtherCAT address (given by configuration tool).   </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> identifyAdo               </TD><TD> ADO used for identification command.                                            </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> identifyValue             </TD><TD> Identification value to be validated.                                           </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> slaveHandle               </TD><TD> Internal management handle (unique identification).                             </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> hcGroupIdx                </TD><TD> Index of the hot connect group, 0 for mandatory.                                </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> previousEthercatAddr      </TD><TD> Station address / EtherCAT address of the previous slave.                       </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> previousPort              </TD><TD> Connected port of the previous slave.                                           </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatIdentityInfo  </TD><TD id="st_e"> slaveIdentity             </TD><TD> Slave information.                                                              </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]                 </TD><TD id="st_e"> slaveName                 </TD><TD> Slave name.                                                                     </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> mbxProtocols              </TD><TD> Supported mailbox protocols.                                                    </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatMailboxInfo   </TD><TD id="st_e"> mbxStandard               </TD><TD> Standard mailbox information (ENI: Mailbox).                                    </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatMailboxInfo   </TD><TD id="st_e"> mbxBootstrap              </TD><TD> Bootstrap mailbox information (ENI: Mailbox).                                   </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatMemoryInfo[]  </TD><TD id="st_e"> processDataIn             </TD><TD> Process data input information from ENI file in unit of bits.                   </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatMemoryInfo[]  </TD><TD id="st_e"> processDataOut            </TD><TD> Process data output information from ENI file in unit of bits.                  </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> numProcessVarsIn          </TD><TD> Number of output process data variables.                                        </TD></TR>
//! <TR><TD id="st_t">      USHORT                    </TD><TD id="st_e"> numProcessVarsOut         </TD><TD> Number of input process data variables.                                         </TD></TR>
//! <TR><TD id="st_t">      UCHAR                     </TD><TD id="st_e"> portDescriptor            </TD><TD> Port descriptor (ESC register  0x0007).                                         </TD></TR>
//! <TR><TD id="st_t">      UCHAR[]                   </TD><TD id="st_e"> reserved01                </TD><TD> Reserved.                                                                       </TD></TR>
//! <TR><TD id="st_t">      USHORT[]                  </TD><TD id="st_e"> wkcStateDiagOffsIn        </TD><TD> Internal.                                                                       </TD></TR>
//! <TR><TD id="st_t">      USHORT[]                  </TD><TD id="st_e"> wkcStateDiagOffsOut       </TD><TD> Internal.                                                                       </TD></TR>
//! <TR><TD id="st_t">      ULONG[]                   </TD><TD id="st_e"> reserved02                </TD><TD> Reserved.                                                                       </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> isPresent                 </TD><TD> Slave is currently present on bus.                                              </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> isHcGroupPresent          </TD><TD> Slave the hot connect group of the slave is present.                            </TD></TR>
//! <TR><TD id="st_t">      BOOL8                     </TD><TD id="st_e"> dcSupport                 </TD><TD> Slave supports DC.                                                              </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatSlaveConfigInfo
{
  USHORT                    autoIncAddr;                                                    //!< Auto increment address.
  USHORT                    ethercatAddr;                                                   //!< The slave's station address / EtherCAT address (given by configuration tool).
  USHORT                    identifyAdo;                                                    //!< ADO used for identification command.
  USHORT                    identifyValue;                                                  //!< Identification value to be validated.
  ULONG                     slaveHandle;                                                    //!< Internal management handle (unique identification).
  ULONG                     hcGroupIdx;                                                     //!< Index of the hot connect group, 0 for mandatory.
  USHORT                    previousEthercatAddr;                                           //!< Station address / EtherCAT address of the previous slave.
  USHORT                    previousPort;                                                   //!< Connected port of the previous slave.
  MlpiEthercatIdentityInfo  slaveIdentity;                                                  //!< Slave information.
  WCHAR16                   slaveName[MLPI_ETHERCAT_MAX_SLAVE_NAME_LEN];                    //!< Slave name.
  ULONG                     mbxProtocols;                                                   //!< Supported mailbox protocols.
  MlpiEthercatMailboxInfo   mbxStandard;                                                    //!< Standard mailbox information (ENI: Mailbox).
  MlpiEthercatMailboxInfo   mbxBootstrap;                                                   //!< Bootstrap mailbox information (ENI: Mailbox).
  MlpiEthercatMemoryInfo    processDataIn[MLPI_ETHERCAT_MAX_PROCESS_DATA_SECTIONS];         //!< Process data input information from ENI file in unit of bits.
  MlpiEthercatMemoryInfo    processDataOut[MLPI_ETHERCAT_MAX_PROCESS_DATA_SECTIONS];        //!< Process data output information from ENI file in unit of bits.
  USHORT                    numProcessVarsIn;                                               //!< Number of output process data variables.
  USHORT                    numProcessVarsOut;                                              //!< Number of input process data variables.
  UCHAR                     portDescriptor;                                                 //!< Port descriptor (ESC register  0x0007).
  UCHAR                     reserved01[3];                                                  //!< Reserved.
  USHORT                    wkcStateDiagOffsIn[MLPI_ETHERCAT_MAX_PROCESS_DATA_SECTIONS];    //!< Internal.
  USHORT                    wkcStateDiagOffsOut[MLPI_ETHERCAT_MAX_PROCESS_DATA_SECTIONS];   //!< Internal.
  ULONG                     reserved02[3];                                                  //!< Reserved.
  BOOL8                     isPresent;                                                      //!< Slave is currently present on bus.
  BOOL8                     isHcGroupPresent;                                               //!< Slave the hot connect group of the slave is present.
  BOOL8                     dcSupport;                                                      //!< Slave supports DC.
}MlpiEthercatSlaveConfigInfo;

//! @typedef MlpiEthercatSoeElementFlags
//! @brief With this structure each element of an IDN can be addressed. The ElementFlags indicating which elements of
//! an IDN are read or written. The ElementFlags indicating which data will be transmitted in the SoE data buffer.
//! @details Elements of struct MlpiEthercatSoeElementFlags
//! <TABLE>
//! <TR><TH>                Type                    </TH><TH>           Element           </TH><TH> Description                           </TH></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> dataState         </TD><TD> DataState requested / available.      </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> name              </TD><TD> Name requested / available.           </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> attribute         </TD><TD> Attribute requested / available.      </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> unit              </TD><TD> Unit requested / available.           </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> minValue          </TD><TD> Minimal value requested / available.  </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> maxValue          </TD><TD> Maximal value requested / available.  </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> value             </TD><TD> Value requested / available.          </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> defaultValue      </TD><TD> Default value requested / available.  </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatSoeElementFlags
{
  BOOL8       dataState;              //!< DataState requested / available.
  BOOL8       name;                   //!< Name requested / available.
  BOOL8       attribute;              //!< Attribute requested / available.
  BOOL8       unit;                   //!< Unit requested / available.
  BOOL8       minValue;               //!< Minimal value requested / available.
  BOOL8       maxValue;               //!< Maximal value requested / available.
  BOOL8       value;                  //!< Value requested / available.
  BOOL8       defaultValue;           //!< Default value requested / available.
}MlpiEthercatSoeElementFlags;

//! @typedef MlpiEthercatVendorData
//! @brief This structure defines the information of the vendor of the EtherCAT master.
//! @details Elements of struct MlpiEthercatVendorData
//! <TABLE>
//! <TR><TH>                Type                    </TH><TH>           Element           </TH><TH> Description               </TH></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> name              </TD><TD> Company name.             </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> department        </TD><TD> Department name.          </TD></TR>
//! <TR><TD id="st_t">      WCHAR16[]               </TD><TD id="st_e"> url               </TD><TD> Url.                      </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatVendorData
{
  WCHAR16     name[MLPI_ETHERCAT_MAX_VENDOR_DATA_STRING_LEN];            //!< Company name.
  WCHAR16     department[MLPI_ETHERCAT_MAX_VENDOR_DATA_STRING_LEN];      //!< Department name.
  WCHAR16     url[MLPI_ETHERCAT_MAX_VENDOR_DATA_STRING_LEN];             //!< Url.
}MlpiEthercatVendorData;

//! @typedef MlpiEthercatVendorInfo
//! @brief This structure defines the member and predecessor of the vendor.
//! @details Elements of struct MlpiEthercatVendorInfo
//! <TABLE>
//! <TR><TH>                Type                    </TH><TH>           Element           </TH><TH> Description                             </TH></TR>
//! <TR><TD id="st_t">      MlpiEthercatVendorData  </TD><TD id="st_e"> member            </TD><TD> Member information.                     </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatVendorData  </TD><TD id="st_e"> predecessor       </TD><TD> Predecessor information.                </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatVendorInfo
{
  MlpiEthercatVendorData  member;           //!< Member information.
  MlpiEthercatVendorData  predecessor;      //!< Predecessor information.
}MlpiEthercatVendorInfo;

//! @typedef MlpiEthercatSlaveStatistics
//! @brief This structure defines the statistic information of an EtherCAT slave.
//! @details Elements of struct MlpiEthercatSlaveStatistics
//! <TABLE>
//! <TR><TH>                Type              </TH><TH>           Element                   </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t">      USHORT[]          </TD><TD id="st_e"> rxErrorCounter            </TD><TD> Rx error counters per slave port.             </TD></TR>
//! <TR><TD id="st_t">      UCHAR[]           </TD><TD id="st_e"> fwdRxErrorCounter         </TD><TD> Forwarded Rx error counters per slave port.   </TD></TR>
//! <TR><TD id="st_t">      UCHAR             </TD><TD id="st_e"> ecatProcUnitErrorCounter  </TD><TD> EtherCAT processing unit error counter.       </TD></TR>
//! <TR><TD id="st_t">      UCHAR             </TD><TD id="st_e"> pdiErrorCounter           </TD><TD> PDI error counter.                            </TD></TR>
//! <TR><TD id="st_t">      USHORT            </TD><TD id="st_e"> alStatusCode              </TD><TD> AL status code.                               </TD></TR>
//! <TR><TD id="st_t">      UCHAR[]           </TD><TD id="st_e"> lostLinkCounter           </TD><TD> Lost link counter per slave port.             </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatSlaveStatistics
{
  USHORT    rxErrorCounter[MLPI_ETHERCAT_MAX_ESC_PORTS];        //!< Rx error counters per slave port.
  UCHAR     fwdRxErrorCounter[MLPI_ETHERCAT_MAX_ESC_PORTS];     //!< Forwarded Rx error counters per slave port.
  UCHAR     ecatProcUnitErrorCounter;                           //!< EtherCAT processing unit error counter.
  UCHAR     pdiErrorCounter;                                    //!< PDI error counter.
  USHORT    alStatusCode;                                       //!< AL status code.
  UCHAR     lostLinkCounter[MLPI_ETHERCAT_MAX_ESC_PORTS];       //!< Lost link counter per slave port.
}MlpiEthercatSlaveStatistics;

//! @typedef MlpiEthercatSlaveBusScanInfo
//! @brief This structure containing information (retrieved while bus scan) about a slave connected to the EtherCAT bus.
//! @details Elements of struct MlpiEthercatSlaveBusScanInfo
//! <TABLE>
//! <TR><TH>                Type                      </TH><TH>           Element         </TH><TH> Description                                                                       </TH></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> busScanStatus   </TD><TD> Status of bus scan (indicating if slave information could be read successfully).  </TD></TR>
//! <TR><TD id="st_t">      MlpiEthercatIdentityInfo  </TD><TD id="st_e"> slaveIdentity   </TD><TD> Slave information stored in EEPROM (beginning at offset 0x0008).                  </TD></TR>
//! </TABLE>
typedef struct MlpiEthercatSlaveBusScanInfo
{
  ULONG                     busScanStatus;                //!< Status of bus scan (indicating if slave information could be read successfully).
  MlpiEthercatIdentityInfo  slaveIdentity;                //!< Slave information stored in EEPROM (beginning at offset 0x0008).
}MlpiEthercatSlaveBusScanInfo;


#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup EthercatLibStructTypes



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

  //! @ingroup EthercatLibInfo
  //! This function returns the status of topology detection.
  //!
  //! @param[in]    connection        Handle for multiple connections.
  //! @param[in]    interfaceNumber   Master interface number.
  //! @param[out]   topologyStatus    Status of topology detection.
  //! @param[out]   numChanges        Number of topology changes (incremented each time a topology change event occurs).
  //! @return                         Return value indicating success (>=0) or error (<0).
  //!
  //! @par Example:
  //! @code
  //! // Return the status of topology detection.
  //! ULONG interfaceNumber = 0;
  //! MlpiEthercatTopologyStatus topologyStatus = MLPI_ETHERCAT_TOPOLOGY_STATUS_UNKNOWN;
  //! ULONG numChanges = 0;
  //! MLPIRESULT result = mlpiEthercatGetTopologyStatus(connection, interfaceNumber, &topologyStatus, &numChanges);
  //! @endcode
  MLPI_API MLPIRESULT mlpiEthercatGetTopologyStatus(const MLPIHANDLE connection, const ULONG interfaceNumber, MlpiEthercatTopologyStatus *topologyStatus, ULONG *numChanges);

//! @ingroup EthercatLibInfo
//! This function returns the master configuration (files will be stored on local drive of control).
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    iniPathFileName   Path and filename of master configuration file (Bosch Rexroth specific).
//! @param[in]    eniPathFileName   Path and filename of EtherCAT network information file.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Return the master configuration (files will be stored on local drive of control).
//! ULONG interfaceNumber = 0;
//! WCHAR16 iniPathFileName[] = L"/USER/EtherCAT.ini";
//! WCHAR16 eniPathFileName[] = L"/USER/EtherCAT.xml";
//! MLPIRESULT result = mlpiEthercatGetConfigFiles(connection, interfaceNumber, iniPathFileName, eniPathFileName);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetConfigFiles(const MLPIHANDLE connection, const ULONG interfaceNumber, const WCHAR16 *iniPathFileName, const WCHAR16 *eniPathFileName);

//! @ingroup EthercatLibConfig
//! This function sets the master configuration (files will be loaded from local drive of control).
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    iniPathFileName   Path and filename of master configuration file (Bosch Rexroth specific).
//! @param[in]    eniPathFileName   Path and filename of EtherCAT network information file.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Set the master configuration (files will be loaded from local drive of control).
//! ULONG interfaceNumber = 0;
//! WCHAR16 iniPathFileName[] = L"/USER/EtherCAT.ini";
//! WCHAR16 eniPathFileName[] = L"/USER/EtherCAT.xml";
//! MLPIRESULT result = mlpiEthercatSetConfigFiles(connection, interfaceNumber, iniPathFileName, eniPathFileName);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSetConfigFiles(const MLPIHANDLE connection, const ULONG interfaceNumber, const WCHAR16 *iniPathFileName, const WCHAR16 *eniPathFileName);

//! @ingroup EthercatLibConfig
//! This function enables raw data exchange with the EtherCAT master stack.
//! @attention
//! It is mainly intended to be used by engineering systems like Bosch Rexroth IndraWorks.
//! In standard use cases please use type specified functions instead.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    command           Internal command number.
//! @param[in]    dataIn            Pointer to input bytes.
//! @param[in]    dataSizeIn        Number of input bytes.
//! @param[out]   dataOut           Pointer to output bytes.
//! @param[in]    dataSizeOut       Maximum number of output bytes (size provided by dataSizeOutRet).
//! @param[out]   dataSizeOutRet    Number of output bytes returned.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // This function should only be used internally.
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatControl(const MLPIHANDLE connection, const ULONG interfaceNumber, const ULONG command, UCHAR *dataIn, const ULONG dataSizeIn, UCHAR *dataOut, const ULONG dataSizeOut, ULONG *dataSizeOutRet);

//! @ingroup EthercatLibInfo
//! This function returns the current master state.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[out]   state             Current master state.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the current master state.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatState state = MLPI_ETHERCAT_STATE_UNKNOWN;
//! MLPIRESULT result = mlpiEthercatGetMasterState(connection, interfaceNumber, &state);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetMasterState(const MLPIHANDLE connection, const ULONG interfaceNumber, MlpiEthercatState *state);

//! @ingroup EthercatLibConfig
//! This function sets the a new master state.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    state             New master state.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Set a new master state.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatState state = MLPI_ETHERCAT_STATE_INIT;
//! MLPIRESULT result = mlpiEthercatSetMasterState(connection, interfaceNumber, state);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSetMasterState(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatState state);

//! @ingroup EthercatLibInfo
//! This function returns the number of configured slaves.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[out]   numSlaves         Number of configured slaves.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the number of configured slaves.
//! ULONG interfaceNumber = 0;
//! ULONG numSlaves = 0;
//! MLPIRESULT result = mlpiEthercatGetNumConfiguredSlaves(connection, interfaceNumber, &numSlaves);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetNumConfiguredSlaves(const MLPIHANDLE connection, const ULONG interfaceNumber, ULONG *numSlaves);

//! @ingroup EthercatLibInfo
//! This function returns the number of connected slaves.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[out]   numSlaves         Number of connected slaves.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the number of connected slaves.
//! ULONG interfaceNumber = 0;
//! ULONG numSlaves = 0;
//! MLPIRESULT result = mlpiEthercatGetNumConnectedSlaves(connection, interfaceNumber, &numSlaves);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetNumConnectedSlaves(const MLPIHANDLE connection, const ULONG interfaceNumber, ULONG *numSlaves);

//! @ingroup EthercatLibInfo
//! This function returns the current and requested slave state.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[out]   currentState      Current slave state.
//! @param[out]   requestedState    Requested slave state.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the current and requested slave state.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT currentState = 0;
//! USHORT requestedState = 0;
//! MLPIRESULT result = mlpiEthercatGetSlaveState(connection, interfaceNumber, addressType, address, &currentState, &requestedState);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetSlaveState(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, USHORT *currentState, USHORT *requestedState);

//! @ingroup EthercatLibConfig
//! This function sets a new slave state.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    slaveState        New slave state.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Set a new slave state.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MlpiEthercatState slaveState = MLPI_ETHERCAT_STATE_PREOP;
//! MLPIRESULT result = mlpiEthercatSetSlaveState(connection, interfaceNumber, addressType, address, slaveState);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSetSlaveState(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const MlpiEthercatState slaveState);

//! @ingroup EthercatLibInfo
//! This function returns information about a slave connected to the EtherCAT bus.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[out]   slaveOnlineInfo   Slave online information.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get information about a slave connected to the EtherCAT bus.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MlpiEthercatSlaveOnlineInfo slaveOnlineInfo;
//! memset (&slaveOnlineInfo, 0, sizeof(MlpiEthercatSlaveOnlineInfo));
//! MLPIRESULT result = mlpiEthercatGetSlaveOnlineInfo(connection, interfaceNumber, addressType, address, &slaveOnlineInfo);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetSlaveOnlineInfo(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, MlpiEthercatSlaveOnlineInfo *slaveOnlineInfo);

//! @ingroup EthercatLibInfo
//! This function returns information about a slave configured within the ENI file.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[out]   slaveConfigInfo   Slave configuration information.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get information about a slave configured within the ENI file.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MlpiEthercatSlaveConfigInfo slaveConfigInfo;
//! memset (&slaveConfigInfo, 0, sizeof(MlpiEthercatSlaveConfigInfo));
//! MLPIRESULT result = mlpiEthercatGetSlaveConfigInfo(connection, interfaceNumber, addressType, address, &slaveConfigInfo);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetSlaveConfigInfo(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, MlpiEthercatSlaveConfigInfo *slaveConfigInfo);

//! @ingroup EthercatLibDll
//! This function returns the state of the slave ports.
//!
//! Slave port state is encoded as follows:
//! USHORT consists out of four nibble. Each nibble signals one information for all four ports.
//!
//! Port order within each nibble: 
//! 3210
//!
//! Nibble:
//! wwww.xxxx.yyyy.zzzz
//! wwww: Signal detected 1=TRUE  0=FALSE
//! xxxx: Loop closed 1=TRUE  0=FALSE
//! yyyy: Link established 1=TRUE  0=FALSE
//! zzzz: Slave connected 1=TRUE  0=FALSE (logical result of w,x,y)
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[out]   portState         Slave port state.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the state of the slave ports.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT portState = 0;
//! MLPIRESULT result = mlpiEthercatGetSlavePortState(connection, interfaceNumber, addressType, address, &portState);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetSlavePortState(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, USHORT *portState);

//! @ingroup EthercatLibDll
//! This function reads data from the EtherCAT Slave Controller memory of a specific device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    registerOffset    Register offset, e.g. 0x0120 for AL Control register.
//! @param[out]   data              Data buffer for uploaded data.
//! @param[in]    dataSize          Number of bytes in 'data' available to read.
//! @param[out]   dataSizeRet       Number of bytes used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read data from the EtherCAT Slave Controller memory of a specific device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT registerOffset = 5;
//! UCHAR data[128] = {0};
//! ULONG dataSize = _countof(data);
//! ULONG dataSizeRet = 0;
//! MLPIRESULT result = mlpiEthercatReadSlaveRegister(connection, interfaceNumber, addressType, address, registerOffset, data, dataSize, &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatReadSlaveRegister(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT registerOffset, UCHAR *data, const ULONG dataSize, ULONG *dataSizeRet);

//! @ingroup EthercatLibDll
//! This function writes data into the EtherCAT Slave Controller memory of a specific device.
//! @attention
//! Changing contents of ESC registers may lead to unpredictable behavior of the slaves and/or the master.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    registerOffset    Register offset, e.g. 0x0120 for AL Control register.
//! @param[in]    data              Data to be transferred.
//! @param[in]    dataSize          Number of bytes in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write data into the EtherCAT Slave Controller memory of a specific device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT registerOffset = 5;
//! UCHAR data[100] = {0};
//! ULONG dataSize = _countof(data);
//! MLPIRESULT result = mlpiEthercatWriteSlaveRegister(connection, interfaceNumber, addressType, address, registerOffset, data, dataSize);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatWriteSlaveRegister(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT registerOffset, UCHAR *data, const ULONG dataSize);

//! @ingroup EthercatLibDll
//! This function resets a ESC (e.g., ET1100, ET1200, and IP Core) if it is capable of issuing a hardware reset.
//! A special sequence of <b>three</b> independent and consecutive frames/commands has to be sent do the slave (Reset register ECAT 0x0040 or PDI 0x0041). 
//! Afterwards, the slave is reset.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Reset a ESC if it is capable of issuing a hardware reset.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MLPIRESULT result = mlpiEthercatResetSlaveController(connection, interfaceNumber, addressType, address);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatResetSlaveController(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address);

//! @ingroup EthercatLibDll
//! This function reads data from the EEPROM of a specific device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    eepromOffset      EEPROM offset (word address to start reading from).
//! @param[out]   data              Data buffer for EEPROM content.
//! @param[in]    numElements       Number of words in 'data' available to read.
//! @param[out]   numElementsRet    Number of words used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read data from the EEPROM of a specific device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT eepromOffset = 5;
//! USHORT data[128] = {0};
//! ULONG numElements = _countof(data);
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiEthercatReadSlaveEeprom(connection, interfaceNumber, addressType, address, eepromOffset, data, numElements, &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatReadSlaveEeprom(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT eepromOffset, USHORT *data, const ULONG numElements, ULONG *numElementsRet);

//! @ingroup EthercatLibDll
//! This function writes data into the EEPROM of a specific device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    eepromOffset      EEPROM offset (word address to start reading from).
//! @param[in]    data              Data buffer for EEPROM content.
//! @param[in]    numElements       Number of words in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write data into the EEPROM of a specific device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT eepromOffset = 5;
//! USHORT data[128] = {0};
//! ULONG numElements = _countof(data);
//! MLPIRESULT result = mlpiEthercatWriteSlaveEeprom(connection, interfaceNumber, addressType, address, eepromOffset, data, numElements);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatWriteSlaveEeprom(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT eepromOffset, USHORT *data, const ULONG numElements);

//! @ingroup EthercatLibDll
//! This function causes a specific slave to reload its EEPROM values to ESC registers.
//! @note
//! Alias address 0x12 is not reloaded through this command. This is prevented by the slave hardware. To reload the alias address use @ref mlpiEthercatResetSlaveController.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Cause a specific slave to reload its EEPROM values to ESC registers.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MLPIRESULT result = mlpiEthercatReloadSlaveEeprom(connection, interfaceNumber, addressType, address);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatReloadSlaveEeprom(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address);

//! @ingroup EthercatLibDll
//! This function assigns the slave EEPROM either to Slave PDI or EtherCAT Master.
//!
//! @param[in]    connection            Handle for multiple connections.
//! @param[in]    interfaceNumber       Master interface number.
//! @param[in]    addressType           Address type.
//! @param[in]    address               Slave address (depending on addressType).
//! @param[in]    slavePdiAccessEnable  Assigns EEPROM to Slave PDI (TRUE) or EtherCAT master (FALSE).
//! @param[in]    forceAssign           Force Assignment of EEPROM (only in case of assignment to EtherCAT master).
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Assign the slave EEPROM either to Slave PDI or EtherCAT Master.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! BOOL8 slavePdiAccessEnable = TRUE;
//! BOOL8 forceAssign = FALSE;
//! MLPIRESULT result = mlpiEthercatAssignSlaveEeprom(connection, interfaceNumber, addressType, address, slavePdiAccessEnable, forceAssign);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatAssignSlaveEeprom(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const BOOL8 slavePdiAccessEnable, const BOOL8 forceAssign);

//! @ingroup EthercatLibDll
//! This function checks if the slave EEPROM is marked for slave PDI access or not.
//!
//! @param[in]    connection            Handle for multiple connections.
//! @param[in]    interfaceNumber       Master interface number.
//! @param[in]    addressType           Address type.
//! @param[in]    address               Slave address (depending on addressType).
//! @param[out]   slavePdiAccessActive  Assigns EEPROM to Slave PDI (TRUE) or EtherCAT master (FALSE).
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Check if the slave EEPROM is marked for slave PDI access or not.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! BOOL8 slavePdiAccessActive = TRUE;
//! MLPIRESULT result = mlpiEthercatCheckSlaveEeprom(connection, interfaceNumber, addressType, address, &slavePdiAccessActive);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatCheckSlaveEeprom(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, BOOL8 *slavePdiAccessActive);

//! @ingroup EthercatLibDll
//! This function reads the identification value from a slave.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    ado               ADO used for identification.
//! @param[out]   value             Identification value.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the identification value from a slave.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT ado = 4;
//! USHORT value = 0;
//! MLPIRESULT result = mlpiEthercatReadSlaveIdentification(connection, interfaceNumber, addressType, address, ado, &value);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatReadSlaveIdentification(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT ado, USHORT *value);

//! @ingroup EthercatLibDll
//! This function returns the slave's statistics counter. Counter can be used to detect errors on Ethernet Layer.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[out]   slaveStatistics   Statistics of a slave.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the slave's statistics counter. Counter can be used to detect errors on Ethernet Layer.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MlpiEthercatSlaveStatistics slaveStatistics;
//! memset (&slaveStatistics, 0, sizeof(MlpiEthercatSlaveStatistics));
//! MLPIRESULT result = mlpiEthercatGetSlaveStatistics(connection, interfaceNumber, addressType, address, &slaveStatistics);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetSlaveStatistics(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, MlpiEthercatSlaveStatistics *slaveStatistics);

//! @ingroup EthercatLibDll
//! This function clears all error registers (statistics counter) in all slaves.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Clear all error registers (statistics counter) in all slaves.
//! ULONG interfaceNumber = 0;
//! MLPIRESULT result = mlpiEthercatResetSlaveStatistics(connection, interfaceNumber);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatResetSlaveStatistics(const MLPIHANDLE connection, const ULONG interfaceNumber);

//! @ingroup EthercatLibDll
//! This function sets the update period of internal slave statistics collection.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    periodMs          Period in ms.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Set the update period of internal slave statistics collection.
//! ULONG interfaceNumber = 0;
//! ULONG periodMs = 1000;
//! MLPIRESULT result = mlpiEthercatSetSlaveStatisticsPeriod(connection, interfaceNumber, periodMs);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSetSlaveStatisticsPeriod(const MLPIHANDLE connection, const ULONG interfaceNumber, const ULONG periodMs);

//! @ingroup EthercatLibCoE
//! This function gets a list of object indices which are available in a EtherCAT CoE device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    listType          List type.
//! @param[out]   indicies          Data buffer for object list.
//! @param[in]    numElements       Number of elements in 'indicies' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get a list of object indices which are available in a EtherCAT CoE device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! MlpiEthercatOdListType listType = MLPI_ETHERCAT_OD_LIST_TYPE_ALL;
//! USHORT indicies[30] = {0};
//! ULONG numElements = _countof(indicies);
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiEthercatCoeGetOdList(connection, interfaceNumber, addressType, address, listType, indicies, numElements, &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatCoeGetOdList(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const MlpiEthercatOdListType listType, USHORT *indicies, const ULONG numElements, ULONG *numElementsRet);

//! @ingroup EthercatLibCoE
//! This function gets the object description for a specific object (SDO) in EtherCAT CoE device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    objectIndex       Object index.
//! @param[out]   data              Data buffer for object description (see ETG 1000 specification).
//! @param[in]    dataSize          Size of buffer provided by data in bytes.
//! @param[out]   dataSizeRet       Size of data written to data in bytes.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the object description for a specific object (SDO) in EtherCAT CoE device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT objectIndex = 0;
//! UCHAR data[1024] = {0};
//! ULONG dataSize = _countof(data);
//! ULONG dataSizeRet = 0;
//! MLPIRESULT result = mlpiEthercatCoeGetObjectDescription(connection, interfaceNumber, addressType, address, objectIndex, data, dataSize, &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatCoeGetObjectDescription(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT objectIndex, UCHAR *data, const ULONG dataSize, ULONG *dataSizeRet);

//! @ingroup EthercatLibCoE
//! This function gets the object entry description for a specific object (SDO) in EtherCAT CoE device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    objectIndex       Object index.
//! @param[in]    subIndex          Sub index.
//! @param[in]    valueInfo         Bit mask to define which information to determine (see ETG 1000 specification).
//! @param[out]   data              Data buffer for entry description (see ETG 1000 specification).
//! @param[in]    dataSize          Size of buffer provided by data in bytes.
//! @param[out]   dataSizeRet       Size of data written to data in bytes.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the object entry description for a specific object (SDO) in EtherCAT CoE device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT objectIndex = 0;
//! UCHAR subIndex = 0;
//! UCHAR valueInfo = 20;
//! UCHAR data[1024] = {0};
//! ULONG dataSize = _countof(data);
//! ULONG dataSizeRet = 0;
//! MLPIRESULT result = mlpiEthercatCoeGetObjectDescription(connection, interfaceNumber, addressType, address, objectIndex, subIndex, valueInfo, data, dataSize, &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatCoeGetEntryDescription(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT objectIndex, const UCHAR subIndex, const UCHAR valueInfo, UCHAR *data, const ULONG dataSize, ULONG *dataSizeRet);

//! @ingroup EthercatLibCoE
//! This function performs a CoE SDO upload from an EtherCAT slave device to the master.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    objectIndex       Object index.
//! @param[in]    subIndex          Sub index.
//! @param[in]    flags             Mailbox flags specifying the upload behavior. Bit 0: Complete Access, Bit 1-31: Reserved (see ETG 1000.6 specification).
//! @param[out]   data              Data buffer for uploaded data.
//! @param[in]    dataSize          Number of bytes in 'data' available to upload.
//! @param[out]   dataSizeRet       Number of bytes used for upload.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Perform a CoE SDO upload from an EtherCAT slave device to the master.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT objectIndex = 4;
//! UCHAR subIndex = 0;
//! ULONG flags = 0;
//! UCHAR data[1024] = {0};
//! ULONG dataSize = _countof(data);
//! ULONG dataSizeRet = 0;
//! for (subIndex = 1; subIndex <= 4; subIndex++)
//! {
//!   result = mlpiEthercatCoeSdoUpload(connection, interfaceNumber, addressType, address, objectIndex, subIndex, flags, data, dataSize, &dataSizeRet);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatCoeSdoUpload(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT objectIndex, const UCHAR subIndex, const ULONG flags, UCHAR *data, const ULONG dataSize, ULONG *dataSizeRet);

//! @ingroup EthercatLibCoE
//! This function performs a CoE SDO download from the master to an EtherCAT slave device.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    addressType       Address type.
//! @param[in]    address           Slave address (depending on addressType).
//! @param[in]    objectIndex       Object index.
//! @param[in]    subIndex          Sub index.
//! @param[in]    flags             Mailbox flags specifying the upload behavior. Bit 0: Complete Access, Bit 1-31: Reserved (see ETG 1000.6 specification).
//! @param[in]    data              Data to be transferred.
//! @param[in]    dataSize          Number of bytes in 'data' available to download.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Perform a CoE SDO download from the master to an EtherCAT slave device.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! USHORT objectIndex = 0x4000;
//! UCHAR subIndex = 5;
//! ULONG flags = 0;
//! UCHAR data[1024] = {0};
//! ULONG dataSize = _countof(data);
//! MLPIRESULT result = mlpiEthercatCoeSdoDownload(connection, interfaceNumber, addressType, address, objectIndex, subIndex, flags, data, dataSize);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatCoeSdoDownload(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const USHORT objectIndex, const UCHAR subIndex, const ULONG flags, UCHAR *data, const ULONG dataSize);

//! @ingroup EthercatLibSoE
//! This function reads Sercos parameters.
//!
//! @param[in]      connection        Handle for multiple connections.
//! @param[in]      interfaceNumber   Master interface number.
//! @param[in]      addressType       Address type.
//! @param[in]      address           Slave address (depending on addressType).
//! @param[in]      driveNumber       Number of the servo drive to read from.
//! @param[in,out]  elementFlags      Elements requested and afterwards returned.
//! @param[in]      idn               Identifier for the element to read.
//! @param[out]     data              RAW data.
//! @param[in]      dataSize          Number of bytes in 'data' available to read.
//! @param[out]     dataSizeRet       Number of bytes used for reading.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the Sercos parameter.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! UCHAR driveNumber = 5;
//! MlpiEthercatSoeElementFlags elementFlags;
//! memset (&elementFlags, 0, sizeof(MlpiEthercatSoeElementFlags));
//! elementFlags.dataState = TRUE;
//! elementFlags.name = FALSE;
//! elementFlags.attribute = TRUE;
//! elementFlags.unit = FALSE;
//! elementFlags.minValue = TRUE;
//! elementFlags.maxValue = FALSE;
//! elementFlags.value = TRUE;
//! elementFlags.defaultValue = TRUE;
//! USHORT idn = 100;
//! UCHAR data[512] = {0};
//! ULONG dataSize = _countof(data);
//! ULONG dataSizeRet = 0;
//! MLPIRESULT result = mlpiEthercatSoeRead(connection, interfaceNumber, addressType, address, driveNumber, &elementFlags, idn, data, dataSize, &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSoeRead(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const UCHAR driveNumber, MlpiEthercatSoeElementFlags *elementFlags, const USHORT idn, UCHAR *data, const ULONG dataSize, ULONG *dataSizeRet);

//! @ingroup EthercatLibSoE
//! This function writes Sercos parameters.
//!
//! @param[in]      connection        Handle for multiple connections.
//! @param[in]      interfaceNumber   Master interface number.
//! @param[in]      addressType       Address type.
//! @param[in]      address           Slave address (depending on addressType).
//! @param[in]      driveNumber       Number of the servo drive to write on.
//! @param[in,out]  elementFlags      Elements requested and afterwards returned.
//! @param[in]      idn               Identifier for the element to write.
//! @param[in]      data              RAW data.
//! @param[in]      dataSize          Number of bytes in 'data' available to write.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the Sercos parameter.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatAddressType addressType = MLPI_ETHERCAT_AUTO_INCREMENT;
//! USHORT address = 0;
//! UCHAR driveNumber = 5;
//! MlpiEthercatSoeElementFlags elementFlags;
//! memset (&elementFlags, 0, sizeof(MlpiEthercatSoeElementFlags));
//! elementFlags.dataState = TRUE;
//! elementFlags.name = FALSE;
//! elementFlags.attribute = TRUE;
//! elementFlags.unit = FALSE;
//! elementFlags.minValue = TRUE;
//! elementFlags.maxValue = FALSE;
//! elementFlags.value = TRUE;
//! elementFlags.defaultValue = TRUE;
//! USHORT idn = 20;
//! UCHAR data[128] = {0};
//! ULONG dataSize = _countof(data);
//! MLPIRESULT result = mlpiEthercatSoeWrite(connection, interfaceNumber, addressType, address, driveNumber, &elementFlags, idn, data, dataSize);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSoeWrite(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatAddressType addressType, const USHORT address, const UCHAR driveNumber, MlpiEthercatSoeElementFlags *elementFlags, const USHORT idn, UCHAR *data, const ULONG dataSize);

//! @ingroup EthercatLibInfo
//! This function returns the vendor information which is connected to a specific vendor id.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    vendorId          EtherCAT vendor id.
//! @param[out]   vendorInfo        Vendor information.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the vendor information which is connected to a specific vendor id.
//! ULONG vendorId = 1;
//! MlpiEthercatVendorInfo vendorInfo;
//! MLPIRESULT result = mlpiEthercatGetVendorInfo(connection, vendorId, &vendorInfo);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetVendorInfo(const MLPIHANDLE connection, const ULONG vendorId, MlpiEthercatVendorInfo *vendorInfo);

//! @ingroup EthercatLibConfig
//! This function generates an EtherCAT Slave Information (Device Description) from an EEPROM image.
//!
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    eepromPathFileName  Path and filename of SII image.
//! @param[in]    esiPathFileName     Path and filename where ESI will be generated.
//! @param[in]    flags               Behavior of Esi generator. Currently set to zero.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Generate an EtherCAT Slave Information (Device Description) from an EEPROM image.
//! WCHAR16 eepromPathFileName[] = L"/USER/Eeprom.bin";
//! WCHAR16 esiPathFileName[] = L"/USER/Box1.xml";
//! ULONG flags = 0x00;
//! MLPIRESULT result = mlpiEthercatGenerateEsi(connection, eepromPathFileName, esiPathFileName, flags);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGenerateEsi(const MLPIHANDLE connection, const WCHAR16 *eepromPathFileName, const WCHAR16 *esiPathFileName, const ULONG flags);

//! @ingroup EthercatLibConfig
//! This function starts the bus scan procedure of the master.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    flags             Flags specifying the behavior of bus scan (e.g. if serial number should be checked).
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Start the bus scan procedure of the master.
//! ULONG interfaceNumber = 0;
//! ULONG flags = 0;
//! MLPIRESULT result = mlpiEthercatStartBusScan(connection, interfaceNumber, flags);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatStartBusScan(const MLPIHANDLE connection, const ULONG interfaceNumber, const ULONG flags);

//! @ingroup EthercatLibInfo
//! This function returns the status of the bus scan.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[out]   status            Status indicating if bus scan has been executed or not.
//! @param[out]   slaveCount        Number of connected slaves (in case of executed bus scan).
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Return the status of the bus scan.
//! ULONG interfaceNumber = 0;
//! MLPIRESULT status = 0;
//! ULONG slaveCount = 0;
//! MLPIRESULT result = mlpiEthercatGetBusScanStatus(connection, interfaceNumber, &status, &slaveCount);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetBusScanStatus(const MLPIHANDLE connection, const ULONG interfaceNumber, MLPIRESULT *status, ULONG *slaveCount);

//! @ingroup EthercatLibInfo
//! This function returns the slave information collected during bus scan.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    autoIncAddr       Auto increment address from which on slave infos should be returned.
//! @param[out]   slaveBusScanInfo  Pointer to slave infos.
//! @param[in]    numElements       Number of elements provided by slaveBusScanInfo.
//! @param[out]   numElementsRet    Number of elements written to slaveBusScanInfo.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Return the slave information collected during bus scan.
//! ULONG interfaceNumber = 0;
//! USHORT autoIncAddr = 5;
//! ULONG numElements = 128;
//! MlpiEthercatSlaveBusScanInfo slaveBusScanInfo[numElements];
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiEthercatGetBusScanSlaveInfo(connection, interfaceNumber, autoIncAddr, slaveBusScanInfo, numElements, &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetBusScanSlaveInfo(const MLPIHANDLE connection, const ULONG interfaceNumber, const USHORT autoIncAddr, MlpiEthercatSlaveBusScanInfo *slaveBusScanInfo, const ULONG numElements, ULONG *numElementsRet);

//! @ingroup EthercatLibInfo
//! This function reads process data (consistent) and returns them as byte stream.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    pdType            Process data type.
//! @param[in]    offset            Byte offset inside process data (where reading starts).
//! @param[out]   data              Pointer to data image where data will be copied to. pbyData has to provide at least ulNumBytes of space.
//! @param[in]    dataSize          Number of bytes to be read.
//! @param[out]   dataSizeRet       Number of bytes used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read process data (consistent) and returns them as byte stream.
//! ULONG interfaceNumber = 0;
//! MlpiEthercatProcessDataType pdType = MLPI_ETHERCAT_PD_TYPE_INPUTS;
//! ULONG offset = 2;
//! UCHAR data[100] = {0};
//! ULONG dataSize = _countof(data);
//! ULONG dataSizeRet = 0;
//! MLPIRESULT result = mlpiEthercatGetProcessData(connection, interfaceNumber, pdType, offset, data, dataSize, &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetProcessData(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatProcessDataType pdType, const ULONG offset, UCHAR *data, const ULONG dataSize, ULONG *dataSizeRet);

//! @ingroup EthercatLibConfig
//! This function writes byte stream to process data (consistent).
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[in]    pdType            Process data type.
//! @param[in]    offset            Byte offset inside process data (where writing starts).
//! @param[in]    data              Pointer to data image where data will be copied from. pbyData has to provide at least ulNumBytes of content.
//! @param[in]    dataSize          Number of bytes to be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write byte stream to process data (consistent).
//! ULONG interfaceNumber = 0;
//! MlpiEthercatProcessDataType pdType = MLPI_ETHERCAT_PD_TYPE_INPUTS;
//! ULONG offset = 5;
//! UCHAR data[100] = {0};
//! ULONG dataSize = _countof(data);
//! MLPIRESULT result = mlpiEthercatSetProcessData(connection, interfaceNumber, pdType, offset, data, dataSize);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatSetProcessData(const MLPIHANDLE connection, const ULONG interfaceNumber, const MlpiEthercatProcessDataType pdType, const ULONG offset, UCHAR *data, const ULONG dataSize);

//! @ingroup EthercatLibInfo
//! This function returns the size of the current process data image, e.g. use for diagnosis purpose.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    interfaceNumber   Master interface number.
//! @param[out]   numOutputBytes    Size of output image in bytes.
//! @param[out]   numInputBytes     Size of image image in bytes.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the size of the current process data image, e.g. use for diagnosis purpose.
//! ULONG interfaceNumber = 0;
//! ULONG numOutputBytes = 0;
//! ULONG numInputBytes = 0;
//! MLPIRESULT result = mlpiEthercatGetProcessDataSize(connection, interfaceNumber, &numOutputBytes, &numInputBytes);
//! @endcode
MLPI_API MLPIRESULT mlpiEthercatGetProcessDataSize(const MLPIHANDLE connection, const ULONG interfaceNumber, ULONG *numOutputBytes, ULONG *numInputBytes);


#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIETHERCATLIB_H__
