#ifndef __MLPIIOLIB_H__
#define __MLPIIOLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiIoLib.h>
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



//! @addtogroup IoLib IoLib
//! @{
//! @brief This library contains functions to control and manage, read and
//! write fieldbus I/Os.
//!
//! @note The IoLib functions trace their debug information mainly into module the MLPI_IO_LIB
//!       and in addition, into the modules CMP_OPENCTRL and MLPI_BASE_MODULES. For further
//!       information, see also the detailed description of the library @ref TraceLib and the
//!       notes about @ref sec_TraceViewer.
//!
//! The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On reading or writing fieldbus I/Os, on default,
//! you use the functions below named after the MLPI data type like @ref mlpiIoWriteFieldbusIoUchar. If you include
//! the header <b>mlpiIoHelper.h</b>, you can also use the equivalent functions named after the IEC61131
//! data types like <b>mlpiIoWriteFieldbusIoByte</b> or like <b>mlpiIoWriteFieldbusIoUsint</b>.
//!
//! <TABLE>
//! <TR><TH>           Number of Bits </TH><TH>           Data types of PLC </TH><TH>           Data types of MLPI  </TH></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> BOOL              </TD><TD id="st_e"> BOOL8               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> SINT              </TD><TD id="st_e"> CHAR                </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> INT               </TD><TD id="st_e"> SHORT               </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> DINT              </TD><TD id="st_e"> LONG                </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LINT              </TD><TD id="st_e"> LLONG               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> USINT             </TD><TD id="st_e"> UCHAR               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> UINT              </TD><TD id="st_e"> USHORT              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> UDINT             </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> ULINT             </TD><TD id="st_e"> ULLONG              </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> BYTE              </TD><TD id="st_e"> UCHAR               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> WORD              </TD><TD id="st_e"> USHORT              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> DWORD             </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LWORD             </TD><TD id="st_e"> ULLONG              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> REAL              </TD><TD id="st_e"> FLOAT               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LREAL             </TD><TD id="st_e"> DOUBLE              </TD></TR>
//! <TR><TD id="st_t"> 8n             </TD><TD id="st_e"> STRING            </TD><TD id="st_e"> WCHAR16             </TD></TR>
//! <TR><TD id="st_t"> 16n            </TD><TD id="st_e"> WSTRING           </TD><TD id="st_e"> WCHAR16             </TD></TR>
//! </TABLE>
//! @}

//! @addtogroup IoLibFieldbusControl Fieldbus control
//! @ingroup IoLib
//! @{
//! @brief This library contains functions to control and manage fieldbus I/Os.
//!
//! @details
//! @}

//! @addtogroup IoLibFieldbusIoRd Read fieldbus I/O
//! @ingroup IoLib
//! @{
//! @brief These functions read from fieldbus I/Os.
//!
//! @details The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On reading fieldbus I/Os, on default, you use the
//! functions below named after the MLPI data type like @ref mlpiIoReadFieldbusIoUlong. If you include the header
//! <b>mlpiIoHelper.h</b>, you can furthermore use the equivalent functions named after the IEC61131 data types
//! like <b>mlpiIoReadFieldbusIoDword</b> or like <b>mlpiIoReadFieldbusIoUdint</b>.
//!
//! <TABLE>
//! <TR><TH>           Number of Bits </TH><TH>           Data types of PLC </TH><TH>           Data types of MLPI  </TH></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> BOOL              </TD><TD id="st_e"> BOOL8               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> SINT              </TD><TD id="st_e"> CHAR                </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> INT               </TD><TD id="st_e"> SHORT               </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> DINT              </TD><TD id="st_e"> LONG                </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LINT              </TD><TD id="st_e"> LLONG               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> USINT             </TD><TD id="st_e"> UCHAR               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> UINT              </TD><TD id="st_e"> USHORT              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> UDINT             </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> ULINT             </TD><TD id="st_e"> ULLONG              </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> BYTE              </TD><TD id="st_e"> UCHAR               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> WORD              </TD><TD id="st_e"> USHORT              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> DWORD             </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LWORD             </TD><TD id="st_e"> ULLONG              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> REAL              </TD><TD id="st_e"> FLOAT               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LREAL             </TD><TD id="st_e"> DOUBLE              </TD></TR>
//! <TR><TD id="st_t"> 8n             </TD><TD id="st_e"> STRING            </TD><TD id="st_e"> WCHAR16             </TD></TR>
//! <TR><TD id="st_t"> 16n            </TD><TD id="st_e"> WSTRING           </TD><TD id="st_e"> WCHAR16             </TD></TR>
//! </TABLE>
//! @}

//! @addtogroup IoLibFieldbusIoWr Write fieldbus I/O
//! @ingroup IoLib
//! @{
//! @brief These functions write to fieldbus I/Os.
//!
//! @details The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On writing fieldbus I/Os, on default, you use the
//! functions below named after the MLPI data type like @ref mlpiIoWriteFieldbusIoUchar. If you include the header
//! <b>mlpiIoHelper.h</b>, you can furthermore use the equivalent functions named after the IEC61131 data types
//! like <b>mlpiIoWriteFieldbusIoByte</b> or like <b>mlpiIoWriteFieldbusIoUsint</b>.
//!
//! <TABLE>
//! <TR><TH>           Number of Bits </TH><TH>           Data types of PLC </TH><TH>           Data types of MLPI  </TH></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> BOOL              </TD><TD id="st_e"> BOOL8               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> SINT              </TD><TD id="st_e"> CHAR                </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> INT               </TD><TD id="st_e"> SHORT               </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> DINT              </TD><TD id="st_e"> LONG                </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LINT              </TD><TD id="st_e"> LLONG               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> USINT             </TD><TD id="st_e"> UCHAR               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> UINT              </TD><TD id="st_e"> USHORT              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> UDINT             </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> ULINT             </TD><TD id="st_e"> ULLONG              </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> BYTE              </TD><TD id="st_e"> UCHAR               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> WORD              </TD><TD id="st_e"> USHORT              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> DWORD             </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LWORD             </TD><TD id="st_e"> ULLONG              </TD></TR>
//! <TR><TD id="st_t"> 32             </TD><TD id="st_e"> REAL              </TD><TD id="st_e"> FLOAT               </TD></TR>
//! <TR><TD id="st_t"> 64             </TD><TD id="st_e"> LREAL             </TD><TD id="st_e"> DOUBLE              </TD></TR>
//! <TR><TD id="st_t"> 8n             </TD><TD id="st_e"> STRING            </TD><TD id="st_e"> WCHAR16             </TD></TR>
//! <TR><TD id="st_t"> 16n            </TD><TD id="st_e"> WSTRING           </TD><TD id="st_e"> WCHAR16             </TD></TR>
//! </TABLE>
//! @}

//! @addtogroup IoLibVersionPermission Version and Permission
//! @ingroup IoLib
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. On using
//! the permission @b "MLPI_IOLIB_PERMISSION_ALL" with the value "true", you will enable all functions of
//! this library for a user account.
//!
//! @note Function with permission MLPI_IOLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiIoLib using in accounts.xml
//! - MLPI_IOLIB_PERMISSION_ALL
//! - MLPI_IOLIB_PERMISSION_INFO
//! - MLPI_IOLIB_PERMISSION_IO_READ
//! - MLPI_IOLIB_PERMISSION_IO_WRITE
//! - MLPI_IOLIB_PERMISSION_UPDATE
//!
//! <TABLE>
//! <TR><TH>           Function                                   </TH><TH> Server version </TH><TH> Permission                       </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiIoReadFieldbusMasterList          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiIoReadFieldbusMasterInfo          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiIoReadFieldbusSlaveList           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiIoReadFieldbusSlaveInfo           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiIoReadFieldbusSlaveInfos          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiIoUpdateFieldbusIo                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_UPDATE"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e">      mlpiIoReadFieldbusIo...       (21x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_IO_READ"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e">      mlpiIoWriteFieldbusIo...      (21x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_IOLIB_PERMISSION_IO_WRITE" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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

//! @addtogroup IoLibStructTypes Structs, Types, ...
//! @ingroup IoLib
//! @{
//! @brief List of used types, enumerations, structures and more...


// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_IO_FIELDBUS_SLAVE_ADDRESS_LEN                          ( 64 )    //!< Maximum length of slave address
#define MLPI_IO_FIELDBUS_DEVICE_NAME_LEN                            ( 81 )    //!< Maximum length of name of master or slave

#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_ENABLE                      (0x0001)  //!< Fieldbus diagnosis flag device enabled
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DRIVER_AVAILABLE            (0x0010)  //!< Fieldbus diagnosis flag driver available
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_FOUND                (0x0020)  //!< Fieldbus diagnosis flag device detected
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_CONFIGURED           (0x0040)  //!< Fieldbus diagnosis flag device configured
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_ACTIVE               (0x0080)  //!< Fieldbus diagnosis flag device active, bus active
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_BUS_ERROR            (0x0100)  //!< Fieldbus diagnosis flag bus error
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_ERROR                (0x0200)  //!< Fieldbus diagnosis flag general device error
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_DIAGNOSTIC_AVAILABLE (0x0400)  //!< Fieldbus diagnosis flag diagnostic information available
#define MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_PASSIVE              (0x0800)  //!< Fieldbus diagnosis flag passive mode of the second master in redundancy systems

//!< Fieldbus diagnosis flag combination 'okay'
#define MLPI_IO_FIELDBUS_DIAGNOSIS_OKAY(flags)   ( flags == (                                                                     \
                                                                MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_ENABLE                            \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DRIVER_AVAILABLE                  \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_FOUND                      \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_CONFIGURED                 \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_ACTIVE                     \
                                                             )                                                                    \
                                                  )

//!< Fieldbus diagnosis flag combination 'diagnosis'
#define MLPI_IO_FIELDBUS_DIAGNOSIS_DIAG(flags)   ( flags == (                                                                     \
                                                                MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_ENABLE                            \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DRIVER_AVAILABLE                  \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_FOUND                      \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_CONFIGURED                 \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_ACTIVE                     \
                                                              | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_DIAGNOSTIC_AVAILABLE       \
                                                             )                                                                    \
                                                  )

//!< Fieldbus diagnosis flag combination 'error or diagnosis'
#define MLPI_IO_FIELDBUS_DIAGNOSIS_ERROR(flags)  (    ( !MLPI_IO_FIELDBUS_DIAGNOSIS_OKAY(flags) )                                 \
                                                   || ( 0 != (   (   MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_BUS_ERROR             \
                                                                   | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_ERROR                 \
                                                                   | MLPI_IO_FIELDBUS_DIAGNOSIS_FLAG_DEVICE_DIAGNOSTIC_AVAILABLE  \
                                                                  )                                                               \
                                                               & flags                                                            \
                                                              )                                                                   \
                                                       )                                                                          \
                                                  )

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! @enum MlpiIoFieldbusMasterType
//! This enumeration defines types of fieldbus master.
typedef enum MlpiIoFieldbusMasterType
{
  MLPI_FIELDBUS_MASTER_GENERIC         = 0,        //!< Generic, not all features are supported
  MLPI_FIELDBUS_MASTER_ONBOARD_IO      = 1,        //!< Onboard I/O
  MLPI_FIELDBUS_MASTER_INLINE_IO       = 2,        //!< Inline I/O
  MLPI_FIELDBUS_MASTER_PROFIBUS_DP     = 3,        //!< Profibus DP
  MLPI_FIELDBUS_MASTER_SERCOS_III      = 4,        //!< Sercos
  MLPI_FIELDBUS_MASTER_FAST_IO         = 5,        //!< FastIO
  MLPI_FIELDBUS_MASTER_PROFINET        = 6,        //!< ProfiNet
  MLPI_FIELDBUS_MASTER_DEVICENET       = 7,        //!< DeviceNet
  MLPI_FIELDBUS_MASTER_ETHERNET_IP     = 8,        //!< EthernetIP
  MLPI_FIELDBUS_MASTER_ETHERNET_NETX   = 9         //!< Ethernet
}MlpiIoFieldbusMasterType;

//! @enum MlpiIoFieldbusArea
//! This enumeration defines the I/O areas 'Input' and 'Output' of a fieldbus device.
typedef enum MlpiIoFieldbusArea
{
  MLPI_IO_AREA_INPUT  = 0,           //!< Input area of a fieldbus device.
  MLPI_IO_AREA_OUTPUT = 1            //!< Output area of a fieldbus device.
}MlpiIoFieldbusArea;

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiIoDiagnosis
//! @brief This structure defines the diagnosis flag information of a fieldbus device.
//! @details Elements of struct MlpiIoDiagnosis
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>           Element  </TH><TH> Description                                    </TH></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> flags    </TD><TD> Flag combination of fieldbus device diagnosis. </TD></TR>
//! </TABLE>
typedef struct MlpiIoDiagnosis
{
  ULONG flags;        //!< Flag combination of fieldbus device diagnosis.
} MlpiIoDiagnosis;

//! @typedef MlpiIoHandle
//! @brief This structure defines the handle to a fieldbus device.
//! @details Elements of struct MlpiIoHandle
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>           Element  </TH><TH> Description                </TH></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> ident    </TD><TD> Ident of fieldbus device.  </TD></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> hash     </TD><TD> Hash of fieldbus device.   </TD></TR>
//! </TABLE>
typedef struct MlpiIoHandle
{
  ULONG ident;      //!< Ident of fieldbus device.
  ULONG hash;       //!< Hash of fieldbus device.
} MlpiIoHandle;

//! @typedef MlpiIoFieldbusSlaveInfo
//! @brief This structure defines the information of a fieldbus slave.
//! @details Elements of struct MlpiIoFieldbusSlaveInfo
//! <TABLE>
//! <TR><TH>           Type            </TH><TH>           Element         </TH><TH> Description                  </TH></TR>
//! <TR><TD id="st_t"> MlpiIoHandle    </TD><TD id="st_e"> handle          </TD><TD> Handle of fieldbus slave.    </TD></TR>
//! <TR><TD id="st_t"> WCHAR16         </TD><TD id="st_e"> name            </TD><TD> Name of fieldbus slave.      </TD></TR>
//! <TR><TD id="st_t"> MlpiIoDiagnosis </TD><TD id="st_e"> diagnosis       </TD><TD> Diagnosis of fieldbus slave. </TD></TR>
//! <TR><TD id="st_t"> WCHAR16         </TD><TD id="st_e"> address         </TD><TD> Address of fieldbus slave.   </TD></TR>
//! <TR><TD id="st_t"> ULONG           </TD><TD id="st_e"> numberOfInputs  </TD><TD> Number of inputs.            </TD></TR>
//! <TR><TD id="st_t"> ULONG           </TD><TD id="st_e"> numberOfOutputs </TD><TD> Number of outputs.           </TD></TR>
//! </TABLE>
typedef struct MlpiIoFieldbusSlaveInfo
{
  MlpiIoHandle      handle;                                         //!< Handle of fieldbus slave.
  WCHAR16           name[MLPI_IO_FIELDBUS_DEVICE_NAME_LEN];         //!< Name of fieldbus slave.
  MlpiIoDiagnosis   diagnosis;                                      //!< Diagnosis of fieldbus slave.
  WCHAR16           address[MLPI_IO_FIELDBUS_SLAVE_ADDRESS_LEN];    //!< Address of fieldbus slave.
  ULONG             numberOfInputs;                                 //!< Number of inputs.
  ULONG             numberOfOutputs;                                //!< Number of outputs.
}MlpiIoFieldbusSlaveInfo;

//! @typedef MlpiIoFieldbusMasterInfo
//! @brief This structure defines the information about a fieldbus master.
//! @details Elements of struct MlpiIoFieldbusMasterInfo
//! <TABLE>
//! <TR><TH>           Type                           </TH><TH>           Element         </TH><TH> Description                    </TH></TR>
//! <TR><TD id="st_t">      MlpiIoHandle              </TD><TD id="st_e"> handle          </TD><TD> Handle of fieldbus master.     </TD></TR>
//! <TR><TD id="st_t">      WCHAR16                   </TD><TD id="st_e"> name            </TD><TD> Name of fieldbus master.       </TD></TR>
//! <TR><TD id="st_t">      MlpiIoDiagnosis           </TD><TD id="st_e"> diagnosis       </TD><TD> Diagnosis of fieldbus master.  </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiIoFieldbusMasterType  </TD><TD id="st_e"> type            </TD><TD> Type of fieldbus master.       </TD></TR>
//! <TR><TD id="st_t">      ULONG                     </TD><TD id="st_e"> numberOfSlaves  </TD><TD> Number of slaves.              </TD></TR>
//! </TABLE>
typedef struct MlpiIoFieldbusMasterInfo
{
  MlpiIoHandle              handle;                                 //!< Handle of fieldbus master.
  WCHAR16                   name[MLPI_IO_FIELDBUS_DEVICE_NAME_LEN]; //!< Name of fieldbus master.
  MlpiIoDiagnosis           diagnosis;                              //!< Diagnosis of fieldbus master.
  MlpiIoFieldbusMasterType  type;                                   //!< Type of fieldbus master.
  ULONG                     numberOfSlaves;                         //!< Number of slaves.
}MlpiIoFieldbusMasterInfo;


#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup IoLibStructTypes




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

//! @ingroup IoLibFieldbusControl
//! This function reads the name list of the available fieldbus masters on the target.
//! @param[in]   connection         Handle for multiple connections.
//! @param[out]  masterList         Name list of fieldbus master. The names of the masters are separated by a semicolon(;).
//! @param[in]   numElements        Number of elements in masterList available to read.
//! @param[out]  countOfMaster      Returns number of returned fieldbus master names in parameter @c masterList.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the name list of fieldbus master.
//! WCHAR16 masterList[1024] = L"";
//! ULONG countOfMaster = 0;
//! MLPIRESULT result = mlpiIoReadFieldbusMasterList(connection, masterList, _countof(masterList), &countOfMaster);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusMasterList(const MLPIHANDLE connection, WCHAR16* masterList, const ULONG numElements, ULONG* countOfMaster);


//! @ingroup IoLibFieldbusControl
//! This function reads the information about a fieldbus master.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[out]  masterInfo         Information about the fieldbus master.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the information about the fieldbus master.
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! MlpiIoFieldbusMasterInfo masterInfo;
//! memset(&masterInfo, 0, sizeof(masterInfo));
//! MLPIRESULT result = mlpiIoReadFieldbusMasterInfo(connection, masterName, &masterInfo);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusMasterInfo(const MLPIHANDLE connection, const WCHAR16* masterName, MlpiIoFieldbusMasterInfo* masterInfo);


//! @ingroup IoLibFieldbusControl
//! This function reads the name list of fieldbus slaves.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[out]  slaveList          Name list of fieldbus slaves. The names of the slaves are separated by a semicolon(;).
//! @param[in]   numElements        Number of elements in slaveList available to read.
//! @param[out]  countOfSlave       Count of fieldbus slaves.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the name list of fieldbus slaves.
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveList[4096] = L"";
//! ULONG countOfSlave = 0;
//! MLPIRESULT result = mlpiIoReadFieldbusSlaveList(connection, masterName, slaveList, _countof(slaveList), &countOfSlave);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusSlaveList(const MLPIHANDLE connection, const WCHAR16* masterName, WCHAR16* slaveList, const ULONG numElements, ULONG* countOfSlave);


//! @ingroup IoLibFieldbusControl
//! This function reads the information about a fieldbus slave of a fieldbus master.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[out]  slaveInfo          Information about the fieldbus slave.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the information about the fieldbus master.
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusSlaveInfo slaveInfo;
//! memset(&slaveInfo, 0, sizeof(slaveInfo));
//! MLPIRESULT result = mlpiIoReadFieldbusSlaveInfo(connection, masterName, slaveAddress, &slaveInfo);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusSlaveInfo(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, MlpiIoFieldbusSlaveInfo* slaveInfo);


//! @ingroup IoLibFieldbusControl
//! This function reads the information about the fieldbus slaves of a fieldbus master.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[out]  slaveInfo          Information about the fieldbus slaves.
//! @param[in]   numElements        Number of elements in numElements available to read.
//! @param[out]  numElementsRet     Pointer to data where the number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the information about the fieldbus master.
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! MlpiIoFieldbusSlaveInfo slaveInfo[16];
//! ULONG numElementsRet = 0;
//! memset(&slaveInfo, 0, sizeof(slaveInfo));
//! MLPIRESULT result = mlpiIoReadFieldbusSlaveInfos(connection, masterName, slaveInfo, _countof(slaveInfo), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusSlaveInfos(const MLPIHANDLE connection, const WCHAR16* masterName, MlpiIoFieldbusSlaveInfo* slaveInfo, const ULONG numElements, ULONG* numElementsRet);


//! @ingroup IoLibFieldbusControl
//! This function performs a bus cycle to update the fieldbus I/Os of all attached slaves of a master.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Perform a bus cycle of the Profibus DP.
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! MLPIRESULT result = mlpiIoUpdateFieldbusIo(connection, masterName);
//! @endcode
MLPI_API MLPIRESULT mlpiIoUpdateFieldbusIo(const MLPIHANDLE connection, const WCHAR16* masterName);


//! @ingroup IoLibFieldbusIoRd
//! This function reads a bit from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   bitOffset          Bit offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! BOOL8 data = FALSE;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[]= L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG bitOffset = 3;
//! MLPIRESULT result = mlpiIoReadFieldbusIoBool8(connection, masterName, slaveAddress, area, bitOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoBool8(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG bitOffset, BOOL8* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 8-bit signed data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! CHAR data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[]= L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoChar(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoChar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, CHAR* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 8-bit unsigned data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! UCHAR data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoUchar(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoUchar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, UCHAR* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 16-bit signed data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! SHORT data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoShort(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoShort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, SHORT* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 16-bit unsigned data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data, where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! USHORT data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoUshort(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoUshort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, USHORT* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 32-bit signed data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data, where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! LONG data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoLong(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoLong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LONG* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 32-bit unsigned data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! ULONG data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoUlong(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoUlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULONG* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 64-bit signed data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! LLONG data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoLlong(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoLlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LLONG* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 64-bit unsigned data value from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! ULLONG data = 0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoUllong(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoUllong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULLONG* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 32-bit floating point data value (single precision) from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! FLOAT data = 0.0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoFloat(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoFloat(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, FLOAT* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads the 64-bit floating point data value (double precision) from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.

//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data value from the input of the Profibus DP slave.
//! DOUBLE data = 0.0;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! MLPIRESULT result = mlpiIoReadFieldbusIoDouble(connection, masterName, slaveAddress, area, byteOffset, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoDouble(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, DOUBLE* data);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 8-bit signed data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! CHAR data[2];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayChar(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayChar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, CHAR* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 8-bit unsigned data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the inputs of the Profibus DP slave.
//! UCHAR data[4];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayUchar(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayUchar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, UCHAR* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 16-bit signed data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! SHORT data[6];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayShort(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayShort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, SHORT* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 16-bit unsigned data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! USHORT data[4];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayUshort(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayUshort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, USHORT* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 32-bit signed data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! LONG data[4];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayLong(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayLong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 32-bit unsigned data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the inputs of the Profibus DP slave.
//! ULONG data[8];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayUlong(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayUlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 64-bit signed data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! LLONG data[2];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayLlong(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayLlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, LLONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 64-bit unsigned data values from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! ULLONG data[4];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayUllong(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayUllong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, ULLONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 32-bit floating point data values (single precision) from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! FLOAT data[6];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayFloat(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayFloat(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, FLOAT* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoRd
//! This function reads an array of 64-bit floating point data values (double precision) from a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   numElements        Number of elements in data available to read.
//! @param[out]  numElementsRet     Pointer to data where number of elements in complete data will be stored.
//! @param[out]  data               Pointer to data where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Read the data values from the input of the Profibus DP slave.
//! DOUBLE data[4];
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_INPUT;
//! ULONG byteOffset = 2;
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiIoReadFieldbusIoArrayDouble(connection, masterName, slaveAddress, area, byteOffset, &data, _countof(data), numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiIoReadFieldbusIoArrayDouble(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, DOUBLE* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup IoLibFieldbusIoWr
//! This function writes a bit to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   bitOffset          Bit offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! BOOL8 data = TRUE;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG bitOffset = 3;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoBool8(connection, masterName, slaveAddress, area, bitOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoBool8(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG bitOffset, const BOOL8 data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 8-bit signed data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! CHAR data = -1;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoChar(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoChar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const CHAR data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 8-bit unsigned data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! UCHAR data = 2;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoUchar(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoUchar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const UCHAR data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 16-bit signed data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! SHORT data = -1;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoShort(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoShort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const SHORT data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 16-bit unsigned data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! USHORT data = 2;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoUshort(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoUshort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const USHORT data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 32-bit signed data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! LONG data = -1;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoLong(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoLong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LONG data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 32-bit unsigned data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! ULONG data = 2;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoUlong(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoUlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULONG data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 64-bit signed data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the outputs of the Profibus DP slave.
//! LLONG data = -1;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoLlong(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoLlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LLONG data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 64-bit unsigned data value to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the outputs of the Profibus DP slave.
//! ULLONG data = 2;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoUllong(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoUllong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULLONG data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 32-bit floating point data value (single precision) to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! FLOAT data = 1.234;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoFloat(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoFloat(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const FLOAT data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes the 64-bit floating point data value (double precision) to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data value to be written to the fieldbus I/O.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data value to the output of the Profibus DP slave.
//! DOUBLE data = 1.23456789;
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoDouble(connection, masterName, slaveAddress, area, byteOffset, data);
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoDouble(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const DOUBLE data);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 8-bit signed data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! CHAR data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayChar(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayChar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const CHAR* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 8-bit unsigned data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! UCHAR data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayUchar(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayUchar(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const UCHAR* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 16-bit signed data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! SHORT data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayShort(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayShort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const SHORT* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 16-bit unsigned data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! USHORT data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayUshort(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayUshort(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const USHORT* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 32-bit signed data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! LONG data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayLong(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayLong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LONG* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 32-bit unsigned data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! ULONG data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayUlong(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayUlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULONG* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 64-bit signed data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! LLONG data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayLlong(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayLlong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const LLONG* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 64-bit unsigned data values to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! ULLONG data[] = {1, 85};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayUllong(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayUllong(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const ULLONG* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 32-bit floating point data values (single precision) to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! FLOAT data[] = {1.234, 9.876};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusmlpiIoWriteFieldbusIoArrayFloatIo(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayFloat(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const FLOAT* data, const ULONG numElements);


//! @ingroup IoLibFieldbusIoWr
//! This function writes an array of 64-bit floating point data values (double precision) to a fieldbus I/O.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   masterName         Name of fieldbus master. The name of the fieldbus master is the name of the regarding
//!                                 master node in your IndraWorks project. You can also retrieve the list of configured
//!                                 master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @param[in]   slaveAddress       Address of fieldbus slave.
//! @param[in]   area               I/O area of access (0==MLPI_IO_AREA_INPUT, 1==MLPI_IO_AREA_OUTPUT).
//! @param[in]   byteOffset         Byte offset within the I/O range of the fieldbus slave.
//! @param[in]   data               Data values to be written to the fieldbus I/O.
//! @param[in]   numElements        Number of elements in data available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Write the data values to the output of the Profibus DP slave.
//! DOUBLE data[] = {1.23456789, 9.87654321};
//! WCHAR16 masterName[] = L"Profibus_DP_Master";
//! WCHAR16 slaveAddress[] = L"3";
//! MlpiIoFieldbusArea area = MLPI_IO_AREA_OUTPUT;
//! ULONG byteOffset = 0;
//! MLPIRESULT result = mlpiIoWriteFieldbusIoArrayDouble(connection, masterName, slaveAddress, area, byteOffset, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiIoWriteFieldbusIoArrayDouble(const MLPIHANDLE connection, const WCHAR16* masterName, const WCHAR16* slaveAddress, const MlpiIoFieldbusArea area, const ULONG byteOffset, const DOUBLE* data, const ULONG numElements);



#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIIOLIB_H__
