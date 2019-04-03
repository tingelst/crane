#ifndef __MLPILOGICLIB_H__
#define __MLPILOGICLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiLogicLib.h>
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


//! @addtogroup LogicLib LogicLib
//! @{
//! @brief This library contains functions to control and manage the logic runtime
//! system on the target device. Furthermore, this library provides the symbolic
//! read and write access to logic variables and the possibility to read and write
//! memory areas such as the input area (like "%IB0" or "%QB0").
//!
//! @attention The addresses of the variables of the symbolic access might change on every download,
//!            reset origin or online change of the PLC application, so you have to stop each variable
//!            access before you load or change the PLC application!
//!
//! @note The LogicLib functions trace their debug information mainly into module the MLPI_LOGIC_LIB
//!       and in addition into the modules CMP_OPENCTRL, CMP_MLPI and MLPI_BASE_MODULES. For further
//!       information, see also the detailed description of the library @ref TraceLib and the notes
//!       about @ref sec_TraceViewer.
//!
//! The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On reading or writing variables by symbols or memory
//! areas, on default, you use the functions below named after the MLPI data type like @ref mlpiLogicWriteVariableBySymbolUshort.
//! If you include the header <b>mlpiLogicHelper.h</b>, you can also use the equivalent functions named after the
//! IEC61131 data types like <b>mlpiLogicWriteVariableBySymbolWord</b> or like <b>mlpiLogicWriteVariableBySymbolUint</b>.
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
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> TIME              </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> DATE              </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> DATE_AND_TIME     </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 16             </TD><TD id="st_e"> TIME_OF_DAY       </TD><TD id="st_e"> ULONG               </TD></TR>
//! <TR><TD id="st_t"> 8              </TD><TD id="st_e"> Enumerator        </TD><TD id="st_e"> SHORT               </TD></TR>
//! </TABLE>





//! @note The LogicLib functions trace their debug information mainly into module the MLPI_LOGIC_LIB
//!       and in addition into the modules CMP_OPENCTRL, CMP_MLPI and MLPI_BASE_MODULES. For further
//!       information, see also the detailed description of the library TraceLib.
//!
//! @}

//! @addtogroup LogicLibApplication Application, symbols
//! @ingroup LogicLib
//! @{
//! @brief The following functions manage the application lifecycle of the logic applications
//! on the target device.
//!
//! @attention The addresses of the variables of the symbolic access might change on every download,
//!            reset origin or online change of the PLC application, so you have to stop each variable
//!            access before you load or change the PLC application!
//!
//! @details With these functions you can start, stop, reset and even load new logic applications
//! onto the target. The following functions are not necessary if you prefer to download your
//! applications from within IndraWorks.
//! @}

//! @addtogroup LogicLibPouExtension Extension of PLC functions by C/C++ implementation
//! @ingroup LogicLib
//! @{
//! @brief The following functions provide the possibility of using customized C/C++ extension within
//! the IEC61131-3 environment IndraWorks / IndraLogic.
//!
//! @note This functions can only be used within a real-time application on the target and there only within the
//!       kernel space! This functions is not available in other toolboxes!
//!
//! @details With these functions, you can integrate your customized C/C++ functionality into the IEC61131-3
//! environment IndraWorks / IndraLogic as a IEC 61131-3 function, function block, method or property. The
//! C/C++ functions must be implemented in a user-defined real-time extension. For more information about
//! customized, a user-defined real-time extension and the <b> Bosch Rexroth Workbench OEM </b>; please refer
//! to the respective documents (@ref page_ProjectWorkbench).
//!
//! @}

//! @addtogroup LogicLibReadSymbol Read variable using symbols
//! @ingroup LogicLib
//! @{
//! @brief The following functions provide symbolic read access to logic variables.
//!
//! @attention The addresses of the variables of the symbolic access might change on every download,
//!            reset origin or online change of the PLC application, so you have to stop each variable
//!            access before you load or change the PLC application!
//!
//! @details The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On reading variables by symbols, on default, you
//! use the functions below named after the MLPI data type like @ref mlpiLogicReadVariableBySymbolUlong. If you include
//! the header <b>mlpiLogicHelper.h</b>, you can also use the equivalent functions named after the IEC61131
//! data types like <b>mlpiLogicReadVariableBySymbolDword</b> or like <b>mlpiLogicReadVariableBySymbolUdint</b>.
//!
//! <TABLE>
//! <TR><TH>           Number of Bits </TH><TH>           Data types of PLC  </TH><TH>          Data types of MLPI  </TH></TR>
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

//! @addtogroup LogicLibWriteSymbol Write variable using symbols
//! @ingroup LogicLib
//! @{
//! @brief The following functions provide symbolic write access to logic variables.
//!
//! @attention The addresses of the variables of the symbolic access might change on every download,
//!            reset origin or online change of the PLC application, so you have to stop each variable
//!            access before you load or change the PLC application!
//!
//! @details The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On writing variables by symbols, on default, you
//! use the functions below named after the MLPI data type like @ref mlpiLogicWriteVariableBySymbolUchar. If you include
//! the header <b>mlpiLogicHelper.h</b>, you can also use the equivalent functions named after the IEC61131
//! data types like <b>mlpiLogicWriteVariableBySymbolByte</b> or like <b>mlpiLogicWriteVariableBySymbolUsint</b>.
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

//! @addtogroup LogicLibAreaRd Read memory area
//! @ingroup LogicLib
//! @{
//! @brief These functions read the following memory areas: "%Ix" (Input), "%Qx" (Output) and "%Mx" (Marker)
//!
//! @details The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On reading from memory areas, on default, you
//! use the functions below named after the MLPI data type like @ref mlpiLogicReadMemoryAreaShort. If you include
//! the header <b>mlpiLogicHelper.h</b>, you can also use the equivalent functions named after the IEC61131
//! data types like <b>mlpiLogicReadMemoryAreaWord</b> or like <b>mlpiLogicReadMemoryAreaInt</b>.
//!
//! @note Reading on outputs and writing on inputs is not supported yet.
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

//! @addtogroup LogicLibAreaWr Write memory area
//! @ingroup LogicLib
//! @{
//! @brief These functions write the following memory areas: "%Ix" (Input), "%Qx" (Output) and "%Mx" (Marker)
//!
//! @details The table shows the PLC data types within the IEC61131 environment 'IndraLogic' and the corresponding data types
//! within the C/C++ environment (e.g. 'Workbench OEM') of the MLPI. On writing to memory areas, on default, you
//! use the functions below named after the MLPI data type like @ref mlpiLogicWriteMemoryAreaUllong. If you include
//! the header <b>mlpiLogicHelper.h</b>, you can also use the equivalent functions named after the IEC61131
//! data types like <b>mlpiLogicWriteMemoryAreaLword</b> or like <b>mlpiLogicWriteMemoryAreaUlint</b>.
//!
//! @note Reading on outputs and writing on inputs is not supported yet.
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


//! @addtogroup LogicLibVersionPermission Version and Permission
//! @ingroup LogicLib
//! @{
//! @addtogroup LogicLibVersionPermission_new Server version since 1.26.0.0 (MLC-FW: 14V22)
//! @ingroup LogicLibVersionPermission
//! @{
//!
//! @note Since firmware version 14V22 (MLPI-Server-Version: 1.26.0.0) a centralized permission management has been implemented in target 
//! controls XM2, L75 and VPx. Some permissions have been summarized in order to improve their usability. 
//! Additional information regarding the usage of older manifest files (i.e. accounts.xml) with newer server versions can be found in @ref newest_manifest.\n
//! @note <b><span style="color:red">Users of other CML controls (i.e. L25, L45, L65) have to use the old permissions as defined in @ref LogicLibVersionPermission_old</span></b>
//!
//!
//! @par List of valid permissions for mlpiLogicLib. These permissions shall be assigned to the groups (i.e. in the group manifest file groups.xml) rather than the users.
//! <TABLE>
//! <TR><TH> Permission-Ident          </TH><TH> Description                                                                                                                    </TH></TR>                  
//! <TR><TD id="st_e"> LOGIC_CONFIG    </TD><TD> Load PLC application - Allows to load PLC applications.                                                                        </TD></TR>  
//! <TR><TD id="st_e"> LOGIC_CONTROL   </TD><TD> Start, stop, reset and debug PLC application - Allows to start, stop, reset and debug PLC application.                         </TD></TR>  
//! <TR><TD id="st_e"> LOGIC_INFO      </TD><TD> View PLC application status - Allows to view PLC application status.                                                           </TD></TR>
//! <TR><TD id="st_e"> LOGIC_READ      </TD><TD> Read variables and memory areas an get address of symbols - Allows read variables and memory areas an get address of symbols.  </TD></TR>
//! <TR><TD id="st_e"> LOGIC_WRITE     </TD><TD> Write variables and memory areas - Allows to write variables and memory areas.                                                 </TD></TR>
//! </TABLE>
//!
//!  @par List of available functions in mlpiLogicLib and the permissions required for their use. 
//! <TABLE>
//! <TR><TH>           Function                                           </TH><TH> Server version </TH><TH> Permission-Ident </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetNumberOfApplications              </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetNameOfApplication                 </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicLoadBootApplication                  </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicStopApplication                      </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONTROL"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicStartApplication                     </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONTROL"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicRunSingleCycleApplication            </TD><TD> 1.4.0.0        </TD><TD> "LOGIC_CONTROL"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicResetApplication                     </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONTROL"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetStateOfApplication                </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetOperationStateOfApplication       </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetOperationStateOfApplicationUlong  </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetTaskInfoOfApplication             </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetInfoOfApplication                 </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicSaveRetainOfApplication              </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicRestoreRetainOfApplication           </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicWaitForEventOfApplication            </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetSymbolsOfApplication              </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetTypeOfSymbol                      </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetSizeOfSymbol                      </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetNumElementsOfSymbol               </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetDimensionOfSymbol                 </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetArrayRangeOfSymbol                </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetAccessRightsOfSymbol              </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetInformationOfSymbol               </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetInformationOfUserType             </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_INFO"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicPouExtensionRegister                 </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicPouExtensionUnregister               </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicPouExtensionUnregisterAll            </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicSetCapabilityOfOperation             </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetCapabilityOfOperation             </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_CONFIG"   </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicReadVariableBySymbol...  (24x)       </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_READ"     </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicWriteVariableBySymbol... (24x)       </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_WRITE"    </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicReadMemoryArea...        (21x)       </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_READ"     </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicWriteMemoryArea...       (21x)       </TD><TD> 1.0.0.0        </TD><TD> "LOGIC_WRITE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetAddressOfSymbol                   </TD><TD> 1.0.13.0       </TD><TD> "LOGIC_READ"     </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetStopAxesConfiguration             </TD><TD> 1.24.0.0       </TD><TD> "LOGIC_INFO"     </TD></TR>
//! </TABLE>
//!
//! @par List of the old permissions of mlpiLogicLib and their corresponding new permission.
//! <TABLE>
//! <TR><TH> Old permission                                       </TH><TH> new Permission </TH></TR>                  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_ALWAYS            </TD><TD> IMPLICIT       </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_APPLICATION_LOAD  </TD><TD> LOGIC_CONFIG   </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_APPLICATION_STOP  </TD><TD> LOGIC_CONTROL  </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_APPLICATION_START </TD><TD> LOGIC_CONTROL  </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_APPLICATION_RESET </TD><TD> LOGIC_CONTROL  </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO  </TD><TD> LOGIC_INFO     </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_RETAIN_DATA       </TD><TD> LOGIC_CONFIG   </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_EVENT_SINK        </TD><TD> LOGIC_CONFIG   </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO       </TD><TD> LOGIC_INFO     </TD></TR> 
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_POU_EXTENSION     </TD><TD> LOGIC_CONFIG   </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_OP_CAPABILITY     </TD><TD> LOGIC_CONFIG   </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_VARIABLE_READ     </TD><TD> LOGIC_READ     </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_VARIABLE_WRITE    </TD><TD> LOGIC_WRITE    </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_MEMORY_READ       </TD><TD> LOGIC_READ     </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_MEMORY_WRITE      </TD><TD> LOGIC_WRITE    </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_SYMBOL_ADDRESS    </TD><TD> LOGIC_READ     </TD></TR>  
//! <TR><TD id="st_e"> MLPI_LOGICLIB_PERMISSION_APPLICATION_CYCLE </TD><TD> LOGIC_CONTROL  </TD></TR>  
//! </TABLE>
//!
//! @}
//! @addtogroup LogicLibVersionPermission_old Server versions before 1.26.0.0 
//! @ingroup LogicLibVersionPermission
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. On using
//! the permission @b "MLPI_LOGICLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_LOGICLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiLogicLib using in accounts.xml
//! - MLPI_LOGICLIB_PERMISSION_ALL
//! - MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO
//! - MLPI_LOGICLIB_PERMISSION_APPLICATION_LOAD
//! - MLPI_LOGICLIB_PERMISSION_APPLICATION_STOP
//! - MLPI_LOGICLIB_PERMISSION_APPLICATION_START
//! - MLPI_LOGICLIB_PERMISSION_APPLICATION_RUN_SINGLE_CYCLE
//! - MLPI_LOGICLIB_PERMISSION_APPLICATION_RESET
//! - MLPI_LOGICLIB_PERMISSION_RETAIN_DATA
//! - MLPI_LOGICLIB_PERMISSION_EVENT_SINK
//! - MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO
//! - MLPI_LOGICLIB_PERMISSION_POU_EXTENSION
//! - MLPI_LOGICLIB_PERMISSION_OP_CAPABILITY
//! - MLPI_LOGICLIB_PERMISSION_VARIABLE_READ
//! - MLPI_LOGICLIB_PERMISSION_VARIABLE_WRITE
//! - MLPI_LOGICLIB_PERMISSION_MEMORY_READ
//! - MLPI_LOGICLIB_PERMISSION_MEMORY_WRITE
//! - MLPI_LOGICLIB_PERMISSION_SYMBOL_ADDRESS
//!
//! <TABLE>
//! <TR><TH>           Function                                           </TH><TH> Server version </TH><TH> Permission                                               </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetNumberOfApplications              </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetNameOfApplication                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicLoadBootApplication                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_LOAD"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicStopApplication                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_STOP"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicStartApplication                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_START"             </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicRunSingleCycleApplication            </TD><TD> 1.4.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_RUN_SINGLE_CYCLE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicResetApplication                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_RESET"             </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetStateOfApplication                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetOperationStateOfApplication       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetOperationStateOfApplicationUlong  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetTaskInfoOfApplication             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetInfoOfApplication                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicSaveRetainOfApplication              </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_RETAIN_DATA"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicRestoreRetainOfApplication           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_RETAIN_DATA"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicWaitForEventOfApplication            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_EVENT_SINK"                    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetSymbolsOfApplication              </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetTypeOfSymbol                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetSizeOfSymbol                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetNumElementsOfSymbol               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetDimensionOfSymbol                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetArrayRangeOfSymbol                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetAccessRightsOfSymbol              </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetInformationOfSymbol               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetInformationOfUserType             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_INFO"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicPouExtensionRegister                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_POU_EXTENSION"                 </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicPouExtensionUnregister               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_POU_EXTENSION"                 </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicPouExtensionUnregisterAll            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_POU_EXTENSION"                 </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicSetCapabilityOfOperation             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_OP_CAPABILITY"                 </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetCapabilityOfOperation             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_OP_CAPABILITY"                 </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicReadVariableBySymbol...  (24x)       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_VARIABLE_READ"                 </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicWriteVariableBySymbol... (24x)       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_VARIABLE_WRITE"                </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicReadMemoryArea...        (21x)       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_MEMORY_READ"                   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e">      mlpiLogicWriteMemoryArea...       (21x)       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_LOGICLIB_PERMISSION_MEMORY_WRITE"                  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetAddressOfSymbol                   </TD><TD> 1.0.13.0       </TD><TD> "MLPI_LOGICLIB_PERMISSION_SYMBOL_ADDRESS"                </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiLogicGetStopAxesConfiguration             </TD><TD> 1.24.0.0       </TD><TD> "MLPI_LOGICLIB_PERMISSION_APPLICATION_INFO"              </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
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


//! @addtogroup LogicLibStructTypes Structs, Types, ...
//! @ingroup LogicLib
//! @{
//! @brief List of used types, enumerations, structures and more...


// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"


// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_LOGIC_MAX_DIMENSION_OF_ARRAY                (  3)         //!< Maximum dimension of an array

#define MLPI_APPLICATION_MAX_LENGTH_OF_POU_NAME          ( 81)         //!< Maximum length of name of a POU
#define MLPI_APPLICATION_MAX_LENGTH_OF_TASK_NAME         (260)         //!< Maximum length of name of a task
#define MLPI_APPLICATION_MAX_LENGTH_OF_INFO              (260)         //!< Maximum length of info element of application

#define MLPI_APPLICATION_OP_STATE_NONE                   (0x00000000)  //!< Operation states of an application: Unspecified state (init state)
#define MLPI_APPLICATION_OP_STATE_PROGRAM_LOADED         (0x00000001)  //!< Operation states of an application: Application is completely loaded
#define MLPI_APPLICATION_OP_STATE_DOWNLOAD               (0x00000002)  //!< Operation states of an application: Application download in progress
#define MLPI_APPLICATION_OP_STATE_ONLINE_CHANGE          (0x00000004)  //!< Operation states of an application: Application online-change in progress
#define MLPI_APPLICATION_OP_STATE_STORE_BOOTPROJECT      (0x00000008)  //!< Operation states of an application: Store bootproject in progress
#define MLPI_APPLICATION_OP_STATE_FORCE_ACTIVE           (0x00000010)  //!< Operation states of an application: Force values is active on the application
#define MLPI_APPLICATION_OP_STATE_EXCEPTION              (0x00000020)  //!< Operation states of an application: Application is in exception state (an exception occurred in this application)
#define MLPI_APPLICATION_OP_STATE_RUN_AFTER_DOWNLOAD     (0x00000040)  //!< Operation states of an application: Download code at the end of download is in progress (initialization of the application)
#define MLPI_APPLICATION_OP_STATE_STORE_BOOTPROJECT_ONLY (0x00000080)  //!< Operation states of an application: Only the boot project is stored at download
#define MLPI_APPLICATION_OP_STATE_EXIT                   (0x00000100)  //!< Operation states of an application: Application exit is still executed (application is no longer active)
#define MLPI_APPLICATION_OP_STATE_DELETE                 (0x00000200)  //!< Operation states of an application: Application is deleted (object is available, but the content is deleted)
#define MLPI_APPLICATION_OP_STATE_RESET                  (0x00000400)  //!< Operation states of an application: Application reset is in progress
#define MLPI_APPLICATION_OP_STATE_RETAIN_MISMATCH        (0x00000800)  //!< Operation states of an application: Retain mismatch occurred during loading the boot project (retain data does not match the application)
#define MLPI_APPLICATION_OP_STATE_BOOTPROJECT_VALID      (0x00001000)  //!< Operation states of an application: Boot project available (boot project matched running application in RAM)
#define MLPI_APPLICATION_OP_STATE_LOAD_BOOTPROJECT       (0x00002000)  //!< Operation states of an application: Loading of the boot project in progress
#define MLPI_APPLICATION_OP_STATE_FLOW_ACTIVE            (0x00004000)  //!< Operation states of an application: Flow control active
#define MLPI_APPLICATION_OP_STATE_RUN_IN_FLASH           (0x00008000)  //!< Operation states of an application: Application is running in flash

// -----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
// -----------------------------------------------------------------------
//! @enum MlpiApplicationState
//! This enumeration defines the state of an application using @ref mlpiLogicGetStateOfApplication.
typedef enum MlpiApplicationState
{
  MLPI_STATE_NONE                           =   0,  //!< Invalid state of application.
  MLPI_STATE_RUN                            =   1,  //!< The application is in state RUN.
  MLPI_STATE_STOP                           =   2,  //!< The application is in state STOP.
  MLPI_STATE_BP                             =   3,  //!< The application is halted on breakpoint.
  MLPI_STATE_SINGLE_CYCLE                   =   4   //!< The application runs for one single cycle.
}MlpiApplicationState;

//! @enum MlpiApplicationResetMode
//! This enumeration defines the different reset possibilities of an application using @ref mlpiLogicResetApplication.
typedef enum MlpiApplicationResetMode
{
  MLPI_RESET_WARM                           =   0,  //!< Reset warm of application, all global data except retain data is reset to default.
  MLPI_RESET_COLD                           =   1,  //!< Reset cold of application, all global data and (!) retain data is reset to default.
  MLPI_RESET_ORIGIN                         =   2   //!< Reset application back to origin, delete application, delete all application files (boot project, etc.), reset all global and retain data.
}MlpiApplicationResetMode;

//! @enum MlpiApplicationEvent
//! This enumeration defines the event of an application using by @ref mlpiLogicWaitForEventOfApplication.
typedef enum MlpiApplicationEvent
{
  MLPI_APPLICATIONEVENT_START_INIT          =   0,  //!< This event raises on begin of 'start application' sequence.
  MLPI_APPLICATIONEVENT_START_DONE          =   1,  //!< This event raises on end of 'start application' sequence.
  MLPI_APPLICATIONEVENT_STOP_INIT           =   2,  //!< This event raises on begin of 'stop application' sequence.
  MLPI_APPLICATIONEVENT_STOP_DONE           =   3,  //!< This event raises on end of 'stop application' sequence.
  MLPI_APPLICATIONEVENT_RESET_INIT          =   4,  //!< This event raises on begin of 'reset application' sequence.
  MLPI_APPLICATIONEVENT_RESET_DONE          =   5,  //!< This event raises on end of 'reset application' sequence.
  MLPI_APPLICATIONEVENT_ONLINE_CHANGE_INIT  =   6,  //!< This event raises on begin of 'online change application' sequence.
  MLPI_APPLICATIONEVENT_ONLINE_CHANGE_DONE  =   7,  //!< This event raises on end of 'online change application' sequence.
  MLPI_APPLICATIONEVENT_DOWNLOAD_INIT       =   8,  //!< This event raises on begin of 'download application' sequence.
  MLPI_APPLICATIONEVENT_DOWNLOAD_DONE       =   9,  //!< This event raises on end of 'download application' sequence.
  MLPI_APPLICATIONEVENT_DELETE_INIT         =  10,  //!< This event raises on begin of 'delete application' sequence.
  MLPI_APPLICATIONEVENT_DELETE_DONE         =  11,  //!< This event raises on end of 'delete application' sequence.
  MLPI_APPLICATIONEVENT_EXIT_INIT           =  12,  //!< This event raises on begin of 'exit application' sequence.
  MLPI_APPLICATIONEVENT_EXIT_DONE           =  13,  //!< This event raises on end of 'exit application' sequence.
  MLPI_APPLICATIONEVENT_EXCEPTION           =  14,  //!< This event raises on an exception of application.
  MLPI_APPLICATIONEVENT_OP_STATE_CHANGE     =  15   //!< This event raises on change of operation state of application ( ref@ MlpiApplicationOpState ).
}MlpiApplicationEvent;

//! @enum MlpiApplicationMemoryArea
//! This enumeration defines the memory areas '%Ix' (Input), '%Qx' (Output) and '%Mx' (Marker) of an application.
typedef enum MlpiApplicationMemoryArea
{
  MLPI_MEMORY_AREA_INPUT                    =   0,  //!< 'Ix' input memory area (Input) of an application.
  MLPI_MEMORY_AREA_OUTPUT                   =   1,  //!< 'Qx' output memory area (Output) of an application.
  MLPI_MEMORY_AREA_MARKER                   =   2   //!< 'Mx' marker memory area (Marker) of an application.
}MlpiApplicationMemoryArea;

//! @enum MlpiLogicCapabilityOperation
//! This enumeration defines an operation which can be enabled or disabled by @ref mlpiLogicSetCapabilityOfOperation.
//! The current capability can be read by @ref mlpiLogicGetCapabilityOfOperation . The capability will be set globally
//! for all applications.
typedef enum MlpiLogicCapabilityOperation
{
  MLPI_LOGIC_CAP_OPERATION_STOP             =   0,  //!< Capability to execute the operation 'stop application'.
  MLPI_LOGIC_CAP_OPERATION_RESET            =   1,  //!< Capability to execute the operation 'reset application'.
  MLPI_LOGIC_CAP_OPERATION_DOWNLOAD         =   2,  //!< Capability to execute the operation 'download application'.
  MLPI_LOGIC_CAP_OPERATION_ONLINE_CHANGE    =   3,  //!< Capability to execute the operation 'online change application'.
  MLPI_LOGIC_CAP_OPERATION_SET_BREAKPOINT   =   4,  //!< Capability to execute the operation 'set breakpoint into an application'.
  MLPI_LOGIC_CAP_OPERATION_WRITE_VARIABLE   =   5,  //!< Capability to execute the operation 'write variable of application'.
  MLPI_LOGIC_CAP_OPERATION_FORCE_VARIABLE   =   6   //!< Capability to execute the operation 'force variable of application'.
}MlpiLogicCapabilityOperation;

//! @enum MlpiLogicCapabilityOperationValue
//! This enumeration defines the possible capability to execute an operation.
typedef enum MlpiLogicCapabilityOperationValue
{
  MLPI_LOGIC_CAP_OPERATION_ENABLE           =   0,  //!< Enable capability to execute an operation (default).
  MLPI_LOGIC_CAP_OPERATION_DISABLE          =   1   //!< Disable capability to execute an operation.
}MlpiLogicCapabilityOperationValue;

//! @enum MlpiLogicSymbolAccessRights
//! This enumeration defines the access rights to a variable using @ref mlpiLogicGetAccessRightsOfSymbol.
typedef enum MlpiLogicSymbolAccessRights
{
  MLPI_ACCESS_RIGHTS_NONE                   =   0,  //!< No access rights to the variable.
  MLPI_ACCESS_RIGHTS_READ                   =   1,  //!< Read access rights to the variable.
  MLPI_ACCESS_RIGHTS_WRITE                  =   2,  //!< Write access rights to the variable.
  MLPI_ACCESS_RIGHTS_READWRITE              =   3   //!< Read and write access rights to the variable.
}MlpiLogicSymbolAccessRights;

//! @enum MlpiLogicType
//! This enumeration defines the different symbol types of the logic using @ref mlpiLogicGetTypeOfSymbol.
typedef enum MlpiLogicType
{
  MLPI_LOGIC_TYPE_BOOL                      =   0,  //!< 1 Byte (BOOL8)
  MLPI_LOGIC_TYPE_BIT                       =   1,  //!< Symbolic access unsupported (1 Bit)
  MLPI_LOGIC_TYPE_BYTE                      =   2,  //!< 8 Bit binary (UCHAR)
  MLPI_LOGIC_TYPE_WORD                      =   3,  //!< 16 Bit binary (USHORT)
  MLPI_LOGIC_TYPE_DWORD                     =   4,  //!< 32 Bit binary (ULONG)
  MLPI_LOGIC_TYPE_LWORD                     =   5,  //!< 64 Bit binary (ULLONG)
  MLPI_LOGIC_TYPE_SINT                      =   6,  //!< 1 Byte signed short integer (CHAR)
  MLPI_LOGIC_TYPE_INT                       =   7,  //!< 2 Byte signed integer (SHORT)
  MLPI_LOGIC_TYPE_DINT                      =   8,  //!< 4 Byte signed double integer (LONG)
  MLPI_LOGIC_TYPE_LINT                      =   9,  //!< 8 Byte signed long integer (LLONG)
  MLPI_LOGIC_TYPE_USINT                     =  10,  //!< 1 Byte unsigned signed short integer (USHORT)
  MLPI_LOGIC_TYPE_UINT                      =  11,  //!< 2 Byte unsigned signed integer (USHORT)
  MLPI_LOGIC_TYPE_UDINT                     =  12,  //!< 4 Byte unsigned double integer (ULONG)
  MLPI_LOGIC_TYPE_ULINT                     =  13,  //!< 8 Byte unsigned long integer (ULLONG)
  MLPI_LOGIC_TYPE_REAL                      =  14,  //!< 4 Byte floating point IEC 559 - 4 Byte (FLOAT)
  MLPI_LOGIC_TYPE_LREAL                     =  15,  //!< 8 Byte floating point IEC 559 - 8 Byte (DOUBLE)
  MLPI_LOGIC_TYPE_STRING                    =  16,  //!< 1 Byte character strings (use WCHAR16 string)
  MLPI_LOGIC_TYPE_WSTRING                   =  17,  //!< 2 Byte character strings (WCHAR16)
  MLPI_LOGIC_TYPE_TIME                      =  18,  //!< 4 Byte, time is given in milliseconds (ULONG)
  MLPI_LOGIC_TYPE_DATE                      =  19,  //!< 4 Byte, time is given in seconds beginning with January 1, 1970 at 12:00 a.m. (00:00 a.m.) (ULONG)
  MLPI_LOGIC_TYPE_DATEANDTIME               =  20,  //!< 4 Byte, time is given in seconds beginning with January 1, 1970 at 12:00 a.m. (00:00 a.m.) (ULONG)
  MLPI_LOGIC_TYPE_TIMEOFDAY                 =  21,  //!< 4 Byte, time is given in milliseconds, time begins at 12:00 a.m. (00:00 a.m.) (ULONG)
  MLPI_LOGIC_TYPE_POINTER                   =  22,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_REFERENCE                 =  23,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_SUBRANGE                  =  24,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ENUM                      =  25,  //!< 2 byte user type (SHORT)
  MLPI_LOGIC_TYPE_ARRAY                     =  26,  //!< Array (use element type, base types or VOID in case of complex type)
  MLPI_LOGIC_TYPE_PARAMS                    =  27,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_USERDEF                   =  28,  //!< RAW data (VOID)
  MLPI_LOGIC_TYPE_NONE                      =  29,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ANY                       =  30,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ANYBIT                    =  31,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ANYDATE                   =  32,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ANYINT                    =  33,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ANYNUM                    =  34,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_ANYREAL                   =  35,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_LAZY                      =  36,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_LTIME                     =  37,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_BITCONST                  =  38,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_MAX_TYPE                  =  39,  //!< Symbolic access unsupported
  MLPI_LOGIC_TYPE_UNSUPPORTED               = 255   //!< Symbolic access unsupported
} MlpiLogicType;


// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

typedef void (*MLPIPOUFNCPTR)(void*);               //!< MLPI POU function pointer 'void function(void*);'


//! @typedef MlpiApplicationTaskInfo
//! @brief This structure defines the information about an IEC task using @ref mlpiLogicGetTaskInfoOfApplication.
//! @details Elements of struct MlpiApplicationTaskInfo
//! <TABLE>
//! <TR><TH>           Type      </TH><TH>           Element          </TH><TH> Description.                       </TH></TR>
//! <TR><TD id="st_t"> WCHAR16[] </TD><TD id="st_e"> name             </TD><TD> name of the IEC task.              </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> priority         </TD><TD> priority of the IEC task.          </TD></TR>
//! <TR><TD id="st_t"> BOOL8     </TD><TD id="st_e"> watchdog         </TD><TD> state of watchdog enabling.        </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> watchdogTime     </TD><TD> watchdog time.                     </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> cycleTime        </TD><TD> cycletime of the IEC task.         </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> averageCycleTime </TD><TD> average cycletime of the IEC task. </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> maxCycleTime     </TD><TD> maximum cycletime of the IEC task. </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> minCycleTime     </TD><TD> minimum cycletime of the IEC task. </TD></TR>
//! <TR><TD id="st_t"> ULONG     </TD><TD id="st_e"> cycleCount       </TD><TD> actual cycleCount of the IEC task. </TD></TR>
//! </TABLE>
typedef struct MlpiApplicationTaskInfo
{
  WCHAR16 name[MLPI_APPLICATION_MAX_LENGTH_OF_TASK_NAME];
  ULONG   priority;
  BOOL8   watchdog;
  ULONG   watchdogTime;
  ULONG   cycleTime;
  ULONG   averageCycleTime;
  ULONG   maxCycleTime;
  ULONG   minCycleTime;
  ULONG   cycleCount;
}MlpiApplicationTaskInfo;

//! @typedef MlpiApplicationInfo
//! @brief This structure defines the information about the application using @ref mlpiLogicGetInfoOfApplication.
//! @details Elements of struct MlpiApplicationInfo
//! <TABLE>
//! <TR><TH>           Type            </TH><TH>           Element     </TH><TH> Description                                                            </TH></TR>
//! <TR><TD id="st_t"> WCHAR16[]       </TD><TD id="st_e"> name        </TD><TD> name of project.                                                       </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]       </TD><TD id="st_e"> author      </TD><TD> author of application.                                                 </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]       </TD><TD id="st_e"> version     </TD><TD> version of application (e.g. "1.0.0.0").                               </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]       </TD><TD id="st_e"> description </TD><TD> description of application.                                            </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[]       </TD><TD id="st_e"> profile     </TD><TD> profile of application.                                                </TD></TR>
//! <TR><TD id="st_t"> MlpiDateAndTime </TD><TD id="st_e"> dateTime    </TD><TD> date and time of last modification (e.g. "2010-12-31 12:00:00") (UTC). </TD></TR>
//! </TABLE>
typedef struct MlpiApplicationInfo
{
  WCHAR16          name[MLPI_APPLICATION_MAX_LENGTH_OF_INFO];
  WCHAR16          author[MLPI_APPLICATION_MAX_LENGTH_OF_INFO];
  WCHAR16          version[MLPI_APPLICATION_MAX_LENGTH_OF_INFO];
  WCHAR16          description[MLPI_APPLICATION_MAX_LENGTH_OF_INFO];
  WCHAR16          profile[MLPI_APPLICATION_MAX_LENGTH_OF_INFO];
  MlpiDateAndTime  dateTime;
} MlpiApplicationInfo;

//! @typedef MlpiApplicationOpState
//! @anchor MlpiApplicationOpState
//! @brief This structure defines the operation states of an application using @ref mlpiLogicGetOperationStateOfApplication.
//! @details Elements of struct MlpiApplicationOpState
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>           Element                    </TH><TH> Description                                                                                                </TH></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> none                       </TD><TD> Unspecified state (init state).                                                                            </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> loaded                     </TD><TD> Application is completely loaded.                                                                          </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> downloadActive             </TD><TD> Application download in progress.                                                                          </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> onlineChangeActive         </TD><TD> Application online-change in progress.                                                                     </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> storeBootprojectActive     </TD><TD> Storing of bootproject in progress.                                                                        </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> forceVariablesActive       </TD><TD> Force values is active on the application.                                                                 </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> exception                  </TD><TD> Application is in exception state (an exception occurred in this application).                             </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> initializeActive           </TD><TD> Download code at the end of download is in progress (initialization of the application).                   </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> storeBootprojectOnlyActive </TD><TD> Only the bootproject is stored at download.                                                                </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> exitActive                 </TD><TD> Application exit is still executed (application is no longer active).                                      </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> deleted                    </TD><TD> Application is deleted (object is available, but the content is deleted).                                  </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> resetActive                </TD><TD> Application reset is in progress.                                                                          </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> retainMismatch             </TD><TD> Retain mismatch occurred during loading of the boot project (retain data does not match the application).  </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> bootprojectValid           </TD><TD> Boot project available (boot project matched running application in RAM).                                  </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> loadBootprojectActive      </TD><TD> Loading of boot project in progress.                                                                       </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> flowControlActive          </TD><TD> Flow control active.                                                                                       </TD></TR>
//! <TR><TD id="st_t"> BOOL8 </TD><TD id="st_e"> runInFlash                 </TD><TD> Application is running in flash.                                                                           </TD></TR>
//! </TABLE>
typedef struct MlpiApplicationOpState
{
  BOOL8 none;

  BOOL8 loaded;
  BOOL8 downloadActive;
  BOOL8 onlineChangeActive;
  BOOL8 storeBootprojectActive;

  BOOL8 forceVariablesActive;
  BOOL8 exception;
  BOOL8 initializeActive;
  BOOL8 storeBootprojectOnlyActive;

  BOOL8 exitActive;
  BOOL8 deleted;
  BOOL8 resetActive;
  BOOL8 retainMismatch;

  BOOL8 bootprojectValid;
  BOOL8 loadBootprojectActive;
  BOOL8 flowControlActive;
  BOOL8 runInFlash;
}MlpiApplicationOpState;

//! @typedef MlpiLogicArrayRange
//! @brief This structure defines the range of an array dimension.
//! @details Elements of struct MlpiLogicArrayRange
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element </TH><TH> Description                   </TH></TR>
//! <TR><TD id="st_t"> ULONG  </TD><TD id="st_e"> minimum </TD><TD> Minimum range value of array. </TD></TR>
//! <TR><TD id="st_t"> ULONG  </TD><TD id="st_e"> maximum </TD><TD> Maximum range value of array. </TD></TR>
//! </TABLE>
typedef struct MlpiLogicArrayRange
{
  ULONG minimum;
  ULONG maximum;
} MlpiLogicArrayRange;

//! @typedef MlpiLogicSymbolInformation
//! @brief This structure defines the information about a symbol using @ref mlpiLogicGetInformationOfSymbol.
//! @details Elements of struct MlpiLogicSymbolInformation
//! <TABLE>
//! <TR><TH>           Type                             </TH><TH>           Element      </TH><TH> Description                                                          </TH></TR>
//! <TR><TD id="st_t"> @ref MlpiLogicType               </TD><TD id="st_e"> type         </TD><TD> Type of symbol.                                                      </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiLogicType               </TD><TD id="st_e"> subType      </TD><TD> Type of symbol if 'type' equal MLPI_LOGIC_TYPE_ARRAY.                </TD></TR>
//! <TR><TD id="st_t">      ULONG                       </TD><TD id="st_e"> dataSize     </TD><TD> Size of symbol (bytes).                                              </TD></TR>
//! <TR><TD id="st_t">      ULONG                       </TD><TD id="st_e"> numElements  </TD><TD> Number of elements.                                                  </TD></TR>
//! <TR><TD id="st_t">      ULONG                       </TD><TD id="st_e"> dimension    </TD><TD> Dimension of array if 'type' equal MLPI_LOGIC_TYPE_ARRAY.            </TD></TR>
//! <TR><TD id="st_t">      MlpiLogicArrayRange         </TD><TD id="st_e"> range        </TD><TD> Range of a dimension of array if 'type' equal MLPI_LOGIC_TYPE_ARRAY. </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiLogicSymbolAccessRights </TD><TD id="st_e"> accessRights </TD><TD> Access rights to symbol.                                             </TD></TR>
//! </TABLE>
typedef struct MlpiLogicSymbolInformation
{
  MlpiLogicType               type;
  MlpiLogicType               subType;
  ULONG                       dataSize;
  ULONG                       numElements;
  ULONG                       dimension;
  MlpiLogicArrayRange         range[MLPI_LOGIC_MAX_DIMENSION_OF_ARRAY];
  MlpiLogicSymbolAccessRights accessRights;
} MlpiLogicSymbolInformation;

//! @typedef MlpiLogicUserTypeInformation
//! @brief This structure defines the information about a type of a symbol using @ref mlpiLogicGetInformationOfUserType.
//! @details Elements of struct MlpiLogicSymbolInformation
//! <TABLE>
//! <TR><TH>           Type                        </TH><TH>           Element      </TH><TH> Description                             </TH></TR>
//! <TR><TD id="st_t"> WCHAR16                     </TD><TD id="st_e"> name         </TD><TD> Name of user type variable.             </TD></TR>
//! <TR><TD id="st_t"> MlpiLogicSymbolInformation  </TD><TD id="st_e"> info         </TD><TD> Information about user type variable.   </TD></TR>
//! </TABLE>
typedef struct MlpiLogicUserTypeInformation
{
  WCHAR16                     name[MLPI_APPLICATION_MAX_LENGTH_OF_POU_NAME];
  MlpiLogicSymbolInformation  info;
} MlpiLogicUserTypeInformation;

#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup LogicLibStructTypes

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


//! @ingroup LogicLibApplication
//! This function will return the number of applications.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   number            Pointer to variable where the number of applications will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the number of applications.
//! ULONG number = 0;
//! MLPIRESULT result = mlpiLogicGetNumberOfApplications(connection, &number);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetNumberOfApplications(const MLPIHANDLE connection, ULONG* number);


//! @ingroup LogicLibApplication
//! This function will return the name of an application selected by index 0 until (number-1).
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    index             Index of application.
//! @param[out]   application       Pointer to variable where the name of application will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'application' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the name of an application.
//! WCHAR16 application[256] = L"";
//! ULONG index = 0;
//! MLPIRESULT result = mlpiLogicGetNameOfApplication(connection, index, application, _countof(application));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetNameOfApplication(const MLPIHANDLE connection, const ULONG index, WCHAR16* application, const ULONG numElements);


//! @ingroup LogicLibApplication
//! This function will load an application from the device file system. The application files
//! can be created from within IndraWorks by creating a boot project when not logged in to the device.
//! This file *.app together with a corresponding *.crc checksum file can be copied to the flash card of the
//! device (e.g. using FTP transfer). Use this function to load it and to replace the current
//! application.
//!
//! @attention The addresses of the variables of the symbolic access might change on every download,
//!            reset origin or online change of the PLC application, so you have to stop each variable
//!            access before you load or change the PLC application!
//!
//! @note
//! This method will not unload a previous loaded application. This method may therefore fail if there are already
//! too many applications loaded on the target and there is no additional application supported.
//! To make room for a new application, you can unload a single application or all applications
//! by calling @ref mlpiLogicResetApplication with argument @ref MLPI_RESET_ORIGIN.
//!
//! @par
//! After loading the application, it is in the state @ref MLPI_STATE_STOP. You have to start the application by
//! using the function ::mlpiLogicStartApplication.
//!
//! @note
//! After loading the application, some fieldbusses (e.g. Profibus DP) need a while for initialization so you can't
//! get a valid access for this time. As a suggestion, you pause the access to the fieldbus I/Os of your application by
//! following code sequence.
//!
//! @code
//! MlpiIoFieldbusMasterInfo infoMaster;
//! memset(&infoMaster, 0, sizeof(infoMaster));
//! WCHAR16 master[] = L"Profibus_DP_Master";
//!
//! for (ULONG maxTimeOutSec=0; maxTimeOutSec<30; maxTimeOutSec++)
//! {
//!   // get fieldbus information incl. diagnosis flags
//!   MLPIRESULT result = mlpiIoReadFieldbusMasterInfo(connection, master, &infoMaster);
//!   if ( MLPI_SUCCEEDED(result) )
//!   {
//!     if (MLPI_IO_FIELDBUS_DIAGNOSIS_ERROR(infoMaster.diagnosis.flags)
//!       sleep(1);
//!     else
//!       break;
//!   }
//!   else
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned) result);
//! }
//! @endcode
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    file              Filename of the new application file without file extension, e.g. "Application".
//! @param[in]    path              Path to the file of the new application. The default root paths can be determined
//!                                 by using the function @ref mlpiSystemGetSpecialPath.
//! @param[out]   application       Pointer to variable where the name of the new application will be stored. The
//!                                 name of the application is stored inside the application project file (.app) and
//!                                 will be returned here. Use the application name as a handle for other function
//!                                 calls which need the application name to identify which application you want to
//!                                 access on the control.
//! @param[in]    numElements       Number of WCHAR16 elements in 'application' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!                                 Will return @c 0xF0360011 (E_LIMIT_MAX) if there are already too many applications
//!                                 loaded on the target. Remove an application by calling @ref mlpiLogicResetApplication
//!                                 with @ref MLPI_RESET_ORIGIN and try again.
//!
//! @par Example:
//! @code
//! // Load the boot application 'Application' from the subfolder 'FolderOfBootApplications' of the user partition.
//! WCHAR16 file[] = L"Application";
//! WCHAR16 path[256] = L"";
//! WCHAR16 subfolder[] = L"FolderOfBootApplications";
//! WCHAR16 application[256] = L"";
//! if ( MLPI_SUCCEEDED(mlpiSystemGetSpecialPath(connection, MLPI_PATH_USER, path, _countof(path))) )
//! {
//!   wcscat16(path, subfolder);
//!   MLPIRESULT result = mlpiLogicLoadBootApplication(connection, file, path, application, _countof(application));
//!   if ( MLPI_SUCCEEDED(result) )
//!   {
//!     printf("\nsuccessfully loaded new bootproject!");
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiLogicLoadBootApplication(const MLPIHANDLE connection, const WCHAR16 *file, const WCHAR16 *path, WCHAR16 *application, const ULONG numElements);


//! @ingroup LogicLibApplication
//! This function will stop an application. All motion that is assigned to the application will stop!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application. Use 0 to stop all applications.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Stop all applications.
//! MLPIRESULT result = mlpiLogicStopApplication(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicStopApplication(const MLPIHANDLE connection, const WCHAR16 *application = 0);


//! @ingroup LogicLibApplication
//! This function will start an application.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application. Use 0 to start all applications.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Start all applications.
//! MLPIRESULT result = mlpiLogicStartApplication(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicStartApplication(const MLPIHANDLE connection, const WCHAR16 *application = 0);


//! @ingroup LogicLibApplication
//! This function will run an application for one single cycle.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application. Use 0 to run all applications for one single cycle.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Run all applications for one single cycle.
//! MLPIRESULT result = mlpiLogicRunSingleCycleApplication(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicRunSingleCycleApplication(const MLPIHANDLE connection, const WCHAR16 *application = 0);


//! @ingroup LogicLibApplication
//! This function will reset an application. All motion that is assigned to the application will stop!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    resetMode         Reset mode
//! @param[in]    application       Name of application. Use 0 to reset all applications.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @attention The addresses of the variables of the symbolic access might change on every download,
//!            reset origin or online change of the PLC application, so you have to stop each variable
//!            access before you load or change the PLC application!
//!
//! @note
//! MLPI_RESET_WARM:   All global data except retain data is reset to default.
//! @par
//! MLPI_RESET_COLD:   All global data and (!) retain data is reset to default.
//! @par
//! MLPI_RESET_ORIGIN: Delete the application, delete all application files (boot project, etc.),
//!                    reset all global and retain data.
//!
//! @par Example:
//! @code
//! // Reset all applications.
//! MlpiApplicationResetMode resetMode = MLPI_RESET_COLD;
//! MLPIRESULT result = mlpiLogicResetApplication(connection, resetMode);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicResetApplication(const MLPIHANDLE connection, const MlpiApplicationResetMode resetMode, const WCHAR16 *application = 0);


//! @ingroup LogicLibApplication
//! This function will return the state of an application.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[out]   state             Pointer to variable where the state will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the state of an application.
//! WCHAR16 application = L"Application";
//! MlpiApplicationState state = MLPI_STATE_NONE;
//! MLPIRESULT result = mlpiLogicGetStateOfApplication(connection, application, &state);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetStateOfApplication(const MLPIHANDLE connection, const WCHAR16 *application, MlpiApplicationState* state);


//! @ingroup LogicLibApplication
//! This function will return the extended operation state of an application by using an unsigned 4 byte variable.
//! Evaluate the state by using respective definitions (e.g. @ref MLPI_APPLICATION_OP_STATE_PROGRAM_LOADED).
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[out]   state             Pointer to variable where the extended operation state will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the extended operation state of an application by using an unsigned length.
//! // Evaluate state by using respective definitions.
//! WCHAR16 application = L"Application";
//! ULONG state = 0;
//! MLPIRESULT result = mlpiLogicGetOperationStateOfApplicationUlong(connection, application, &state);
//! if ((state & MLPI_APPLICATION_OP_STATE_EXCEPTION) == MLPI_APPLICATION_OP_STATE_EXCEPTION)
//!   printf("Exception occurred!");
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetOperationStateOfApplicationUlong(const MLPIHANDLE connection, const WCHAR16 *application, ULONG* state);


//! @ingroup LogicLibApplication
//! This function will return the extended operation state of an application by using struct @ref MlpiApplicationOpState.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[out]   state             Pointer to variable where the extended operation state will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the extended operation state of an application by using struct MlpiApplicationOpState .
//! WCHAR16 application = L"Application";
//! MlpiApplicationOpState state;
//! memset(&state, 0, sizeof(state));
//! MLPIRESULT result = mlpiLogicGetOperationStateOfApplication(connection, application, &state);
//! if (state.exception == TRUE)
//!   printf("Exception occurred!");
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetOperationStateOfApplication(const MLPIHANDLE connection, const WCHAR16 *application, MlpiApplicationOpState* state);


//! @ingroup LogicLibApplication
//! This function will return information about all running IEC tasks of an application.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[out]   taskInfo          Pointer to an array of structure where the information will be stored.
//! @param[in]    numElements       Number of MlpiApplicationTaskInfo elements available in 'taskInfo' to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the task information of all running IEC tasks of an application.
//! WCHAR16 application[] = L"Application";
//! MlpiApplicationTaskInfo taskInfo[32];
//! ULONG numElementsRet = 0;
//! memset(taskInfo, 0, sizeof(taskInfo));
//! MLPIRESULT result = mlpiLogicGetTaskInfoOfApplication(connection, application, taskInfo, _countof(taskInfo), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetTaskInfoOfApplication(const MLPIHANDLE connection, const WCHAR16* application, MlpiApplicationTaskInfo *taskInfo, const ULONG numElements, ULONG *numElementsRet = 0);


//! @ingroup LogicLibApplication
//! This function will return information about the given application.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[out]   info              Pointer to struct where the information will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read information about the application.
//! WCHAR16 application[] = L"Application";
//! MlpiApplicationInfo info;
//! memset(&info, 0, sizeof(info));
//! MLPIRESULT result = mlpiLogicGetInfoOfApplication(connection, application, info);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetInfoOfApplication(const MLPIHANDLE connection, const WCHAR16* application, MlpiApplicationInfo *info);


//! @ingroup LogicLibApplication
//! This function will save the retain data of an application to default storage or to a user-defined file.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[in]    path              Path and name of retain data file. Use 0 to select default storage to
//!                                 load it on next reboot.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Store the retain data of the application 'Application' to the file 'retain.dat' in the subfolder 'Data'
//! // of the user partition.
//! WCHAR16 application[] = L"Application";
//! WCHAR16 path[256] = L"";
//! WCHAR16 subpath[] = L"Data/retain.dat";
//! if ( MLPI_SUCCEEDED(mlpiSystemGetSpecialPath(MLPI_PATH_USER, path, _countof(path))) )
//! {
//!   wcscat(path, subpath);
//!   MLPIRESULT result = mlpiLogicSaveRetainOfApplication(connection, application, path);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiLogicSaveRetainOfApplication(const MLPIHANDLE connection, const WCHAR16* application, const WCHAR16* path = 0);


//! @ingroup LogicLibApplication
//! This function will restore the retain data of an application from a user-defined file.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[in]    path              Path and name of retain data file.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Restore the retain data of the application 'Application' from the file 'retain.dat' in the subfolder 'Data'
//! // of the user partition.
//! WCHAR16 application[] = L"Application";
//! WCHAR16 path[256] = L"";
//! WCHAR16 subpath[] = L"Data/retain.dat";
//! if ( MLPI_SUCCEEDED(mlpiSystemGetSpecialPath(MLPI_PATH_USER, path, _countof(path))) )
//! {
//!   wcscat(path, subpath);
//!   MLPIRESULT result = mlpiLogicRestoreRetainOfApplication(connection, application, path);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiLogicRestoreRetainOfApplication(const MLPIHANDLE connection, const WCHAR16* application, const WCHAR16* path);


//! @ingroup LogicLibApplication
//! This function pends the calling task until the given event occurs.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application (unused).
//! @param[in]    event             Event to wait for.
//! @param[in]    timeout           The timeout after which the function should return an error if the
//!                                 event did not raise. Use MLPI_INFINITE to wait forever.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note: The events occurs regardless the given application name, they don't care about the application name.
//!
//! @par Example:
//! @code
//! // Wait until event of 'Online change'.
//! WCHAR16 application[] = L"Application";
//! MLPIRESULT result = mlpiLogicWaitForEventOfApplication(connection, application, MLPI_APPLICATIONEVENT_ONLINE_CHANGE_INIT, MLPI_INFINITE);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWaitForEventOfApplication(const MLPIHANDLE connection, const WCHAR16* application, const MlpiApplicationEvent event, const ULONG timeout);


//! @ingroup LogicLibApplication
//! This function will return the active "StopAxes" and "StopAxesCommon" configuration of all applications.
//! @param[in]    connection        Handle for multiple connections.
//! @param[out]   configuration     Pointer to variable where the configuration of applications will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'configuration' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the active "StopAxes" and "StopAxesCommon" configuration of all applications.
//! WCHAR16 configuration[256] = L"";
//! ULONG index = 0;
//! MLPIRESULT result = mlpiLogicGetStopAxesConfiguration(connection, configuration, _countof(configuration));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetStopAxesConfiguration(const MLPIHANDLE connection, WCHAR16* configuration, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibApplication
//! This function will read all symbols of an application. The symbols themselves are separated from
//! each other by a semicolon. The argument 'node' will be used as a token for further requests, if the buffer size
//! is insufficient for all symbols.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    application       Name of application.
//! @param[out]   node              Token to control the requests. Use on first request the value 0 to sign it
//!                                 as first, use on all following requests the return value you get from last call
//!                                 until you get ((PROCESSHANDLE) -1). This value signals the last response.
//! @param[out]   symbols           Pointer to variable where the symbols will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'symbols' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note The symbols need to be exported to the symbol configuration of the indralogic project. You may want to use the
//!       attribute @c {attribute 'symbol' := 'readwrite'} above your variable declaration to add it explicit to the
//!       symbol configuration.
//!
//! @par Example:
//! @code
//! // Read all symbols of an application.
//! WCHAR16 application[] = L"Application";
//! PROCESSHANDLE node = NULL;
//! WCHAR16 symbols[4096] = L"";
//! MLPIRESULT result = mlpiLogicGetSymbolsOfApplication(connection, application, &node, symbols, _countof(symbols));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetSymbolsOfApplication(const MLPIHANDLE connection, const WCHAR16 *application, PROCESSHANDLE* node, WCHAR16 *symbols, const ULONG numElements);


//! @ingroup LogicLibApplication
//! This function reads the type and sybtype of a symbol variable of PLC application.
//! This and further information can also be read by @ref mlpiLogicGetInformationOfSymbol.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   type              Pointer to variable where the type will be stored.
//! @param[out]   subtype           Pointer to variable where in case of array the sybtype will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the type of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! MlpiLogicType type = MLPI_LOGIC_TYPE_NONE;
//! MlpiLogicType subtype = MLPI_LOGIC_TYPE_NONE;
//! MLPIRESULT result = mlpiLogicGetTypeOfSymbol(connection, L"Application.PlcProg.x1", &type, &subtype);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetTypeOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, MlpiLogicType *type, MlpiLogicType *subtype=0);


//! @ingroup LogicLibApplication
//! This function reads the size in bytes of a symbol variable of PLC application.
//! This and further information can also be read by @ref mlpiLogicGetInformationOfSymbol.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   dataSize          Pointer to variable where the size in bytes will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the size in bytes of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG dataSize = 0;
//! MLPIRESULT result = mlpiLogicGetSizeOfSymbol(connection, L"Application.PlcProg.x1", &dataSize);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetSizeOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *dataSize);


//! @ingroup LogicLibApplication
//! This function reads the number of elements of an array or user type of a symbol variable of PLC application.
//! This and further information can also be read by @ref mlpiLogicGetInformationOfSymbol.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   numElements       Pointer to variable where the number of elements will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the number of elements of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG numElements = 0;
//! MLPIRESULT result = mlpiLogicGetNumElementsOfSymbol(connection, L"Application.PlcProg.x1", &numElements);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetNumElementsOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *numElements);


//! @ingroup LogicLibApplication
//! This function reads the array dimension of a symbol variable of PLC application.
//! This and further information can also be read by @ref mlpiLogicGetInformationOfSymbol.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   dimension         Pointer to variable where the dimension will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the dimension of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG dimension = 0;
//! MLPIRESULT result = mlpiLogicGetDimensionOfSymbol(connection, L"Application.PlcProg.x1", &dimension);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetDimensionOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *dimension);


//! @ingroup LogicLibApplication
//! This function reads the minimum index and maximum index resp. the range of an array dimension of a symbol
//! variable of PLC application. This and further information can also be read by @ref mlpiLogicGetInformationOfSymbol.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    index             Index of dimension of array.
//! @param[out]   range             Pointer to variable where the range of array will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the range of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG index = 0;
//! MlpiLogicArrayRange range;
//! memset(&range, 0, sizeof(range));
//! MLPIRESULT result = mlpiLogicGetArrayRangeOfSymbol(connection, L"Application.PlcProg.x1", index, &range);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetArrayRangeOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG index, MlpiLogicArrayRange *range);


//! @ingroup LogicLibApplication
//! This function reads the access rights of a symbol variable of PLC application.
//! This and further information can also be read by @ref mlpiLogicGetInformationOfSymbol.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   accessrights      Pointer to variable where the access rights of array will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the access rights of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! MlpiLogicSymbolAccessRights accessrights = MLPI_ACCESS_RIGHTS_NONE;
//! MLPIRESULT result = mlpiLogicGetAccessRightsOfSymbol(connection, L"Application.PlcProg.x1", &accessrights);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetAccessRightsOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, MlpiLogicSymbolAccessRights *accessrights);


//! @ingroup LogicLibApplication
//! This function reads the physical kernel space address of a symbol variable of a PLC application.
//!
//! @note This function can only be used within a real-time application on the target and there only within the
//!       kernel space! This function is not available in other toolboxes!
//!
//! @attention This address might change on every download, reset origin or online change of PLC application.
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   address           Pointer to variable where the physical address of symbol will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the physical address of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG address = NULL;
//! MLPIRESULT result = mlpiLogicGetAddressOfSymbol(connection, L"Application.PlcProg.x1", &address);
//! if (MLPI_FAILED(result)){
//!   printf("\nCould not read address of symbol, function returned with error 0x%08x", result);
//!   return result;
//! }
//!
//! printf("\nThe physical kernel space address of symbol is: 0x%08X", address);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetAddressOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *address);


//! @ingroup LogicLibApplication
//! This function reads the types, size, number of elements, array information and access rights of a symbol
//! variable of PLC application. It therefore combines the functionality of @ref mlpiLogicGetTypeOfSymbol,
//! @ref mlpiLogicGetSizeOfSymbol, @ref mlpiLogicGetNumElementsOfSymbol, @ref mlpiLogicGetDimensionOfSymbol,
//! @ref mlpiLogicGetArrayRangeOfSymbol, @ref mlpiLogicGetAccessRightsOfSymbol in a single call.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   info              Pointer to struct array where the information will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the information of an application symbol.
//! MlpiLogicSymbolInformation info;
//! memset(&info, 0, sizeof(info));
//! MLPIRESULT result = mlpiLogicGetInformationOfSymbol(connection, L"Application.PlcProg.x1", &info);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetInformationOfSymbol(const MLPIHANDLE connection, const WCHAR16 *symbol, MlpiLogicSymbolInformation *info);


//! @ingroup LogicLibApplication
//! This function reads names and information of variables (MlpiLogicSymbolInformation) of a symbol variable
//! of PLC application if type of symbol equal MLPI_LOGIC_TYPE_USERDEF.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   info              Pointer to struct array where the names and information of the variables will
//!                                 be stored.
//! @param[in]    numElements       Number of MlpiLogicUserTypeInformation elements available in 'info' to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the information of a symbol variable of PLC application.
//! WCHAR16 symbol[] = L"Application.PlcProg.x1";
//! MlpiLogicSymbolInformation symbolInfo;
//! memset(&symbolInfo, 0, sizeof(symbolInfo));
//! MLPIRESULT result = mlpiLogicGetInformationOfSymbol(connection, symbol, &symbolInfo);
//!
//! // Read names and information of variables of a symbol if the type is equal MLPI_LOGIC_TYPE_USERDEF.
//! if(symbolInfo.type==MLPI_LOGIC_TYPE_USERDEF)
//! {
//!   ULONG numElementsRet = 0;
//!   MlpiLogicUserTypeInformation *info = new MlpiLogicUserTypeInformation[symbolInfo.numElements];
//!   memset(info, 0, symbolInfo.numElements * sizeof(MlpiLogicUserTypeInformation));
//!
//!   MLPIRESULT result = mlpiLogicGetInformationOfUserType(connection, symbol, info, symbolInfo.numElements, &numElementsRet);
//!
//!   for( ULONG i=0; i<numElementsRet; i++ )
//!   {
//!     printf("\n\n\t+--> Variable No. %d", i);
//!     printf("\n\t\t Name: %s", info[i].name);
//!     printf("\n\t\t Type: %d", info[i].info.type);
//!     printf("\n\t\t Subtype: %d", info[i].info.subType);
//!     printf("\n\t\t Size: %d", info[i].info.dataSize);
//!     printf("\n\t\t Number of elements: %d", info[i].info.numElements);
//!     printf("\n\t\t Dimension: %d", info[i].info.dimension);
//!     for( ULONG j=0; j<info[i].info.dimension; j++ )
//!     {
//!       printf("\n\t\t Minimum array range of dimension %d: %d", j+1, info[i].info.range[0].minimum);
//!       printf("\n\t\t Maximum array range of dimension %d: %d", j+1, info[i].info.range[0].maximum);
//!     }
//!     printf("\n\t\t Access rights: %d", info[i].info.accessRights);
//!   }
//!   delete info;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetInformationOfUserType(const MLPIHANDLE connection, const WCHAR16* symbol, MlpiLogicUserTypeInformation *info, const ULONG numElements, ULONG *numElementsRet = 0);


//! @ingroup LogicLibApplication
//! This function enables or disables the execution capability of operations @ref MlpiLogicCapabilityOperation.
//! The capability will be set to global for all applications. After reboot, the capability will be set to default.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    operation         Operation which can be enabled or disabled.
//! @param[in]    value             Target capability of execution of operation.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Disable capability to execute online changes.
//! MLPIRESULT result = mlpiLogicSetCapabilityOfOperation(MLPI_LOGIC_CAP_OPERATION_ONLINE_CHANGE, MLPI_LOGIC_CAP_OPERATION_DISABLE);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicSetCapabilityOfOperation(const MLPIHANDLE connection, const MlpiLogicCapabilityOperation operation, const MlpiLogicCapabilityOperationValue value);


//! @ingroup LogicLibApplication
//! This function reads the current execution capability of operations @ref MlpiLogicCapabilityOperation.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    operation         Operation which can be enabled or disabled.
//! @param[out]   value             Pointer to variable where the current capability of execution of operation
//!                                 will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the current capability to execute online changes.
//! MlpiLogicCapabilityOperationValue value = MLPI_LOGIC_CAP_OPERATION_ENABLE;
//! MLPIRESULT result = mlpiLogicGetCapabilityOfOperation(MLPI_LOGIC_CAP_OPERATION_ONLINE_CHANGE, &value);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicGetCapabilityOfOperation(const MLPIHANDLE connection, const MlpiLogicCapabilityOperation operation, MlpiLogicCapabilityOperationValue *value);


#if defined (TARGET_OS_VXWORKS_KERNEL) || (TARGET_OS_WINNT)

//! @ingroup LogicLibPouExtension
//! This function registers a C/C++ implementation of a POU (e.g. function block) from the MLPI real-time environment.
//! The registration enables the possibility to call the C/C++ implementation by a related POU declared in an
//! IEC 61131-3 environment IndraWorks. This means, you can call POUs from PLC which are coded in C/C++.
//!
//! @note This function can only be used within a real-time application on the target and there only within the
//!       kernel space! This function is not available in other toolboxes!
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    name              Name of POU in the IEC 61131 environment.
//! @param[in]    function          Function pointer to C/C++ implementation.
//! @param[in]    signature         Signature of POU interface (reserved).
//! @param[in]    version           Version of POU library if implemented within a library.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note The name of POUs declared in the IEC 61131-3 environment IndraWorks share a single namespace so use
//!       your own prefix (e.g. your project name) to avoid collisions!
//!
//! @note The signature of POU is a checksum of their interface. Disable this check by setting it to zero (default).
//!
//! @note The versions of POUs are taken from the library where they are implemented. The value results from the four
//!       elements of the library version ("aa.bb.cc.dd" -> 0xaabbccdd). When not used within a library this value is
//!       set to zero (default).
//!
//! @attention The memory layout of all C/C++ structs which are mapped to IEC POUs, as described below, needs to follow
//!            'natural alignment'! This is because the IEC compiler aligns the memory of elements and variables given
//!            by the POUs to addresses which are multiples of its own size. You may want to use padding bytes in your
//!            structs or compiler dependent pragmas to make your C/C++ structs to match with the IEC memory layout or
//!            use the provided MLPI IEC data types (like MLPI_IEC_LREAL, MLPI_IEC_USINT, ... see mlpiGlobal.h)
//!
//! @note The declaration of an IEC variable of type STRING without any length notation cause an implicit allocation
//!       of 80+1 characters so in this case you have to declare an array of 81 elements of MLPI_IEC_STRING within your
//!       C/C++ application. The declaration of an IEC variable of type STRING with length n cause an allocation of n+1
//!       characters so in this case you have to declare an array of n+1 elements of MLPI_IEC_STRING within your C/C++
//!       application. The same applies to the type WSTRING and the corresponding MLPI_IEC_WSTRING.
//!
//! @par Example "IEC function":
//! \n
//! To use a C/C++ implementation within the IEC 61131 environment IndraWorks as a @b function you need to do
//! the following:
//! \n\n
//! - Declare an external POU as a function with a proper interface within IndraWorks.
//!   - Add a POU @b "<function_name>" (e.g. 'SimpleCalc') as a function.
//!     @image html ExtIecFncSimpleCalc_Add.png "Adding new POU 'SimpleCalc' as function." \n
//!   - Declare the interface of inputs and outputs of the function. You do not need to implement any code.
//!     @code
//!       FUNCTION SimpleCalc : LREAL
//!       VAR_INPUT
//!         in1: LREAL;
//!         in2: LREAL;
//!       END_VAR
//!       VAR
//!       END_VAR
//!     @endcode
//!     @image html ExtIecFncSimpleCalc_Decl.png "Declaration of interface of IEC function." \n
//!   - Enable the build property "External implementation" (late link in the runtime) of POU. You will find this
//!     build property on right click of POU within the project tree ("Property...->Build").
//!     @image html ExtIecFncSimpleCalc_ExtImpl.png "Build property: External Implementation." \n
//! - Declare a corresponding interface within a C/C++ file and implement the runtime code needed.
//!   - Declare a structure @b "<function_name>_struct" with the elements of the IEC function in following order:
//!     - All input variables (VAR_INPUT) and all in-out variables (VAR_IN_OUT) in order defined in IEC.
//!     - All output variables (VAR_OUTPUT) in order defined in IEC.
//!     - The implicit function output comes last.
//!     .
//!     @code
//!       #include "mlpiLogicLib.h"
//!
//!       typedef struct SimpleCalc_struct
//!       {
//!         MLPI_IEC_LREAL  in1;                          // Declaration of input
//!         MLPI_IEC_LREAL  in2;                          // Declaration of input
//!         MLPI_IEC_LREAL  out;                          // Declaration of implicit function output
//!       }SimpleCalc_struct;
//!     @endcode
//!   - Declare a C/C++ function @b "<function_name>" with the following interface and implement the desired functionality.
//!     - The return type of the function is @b void.
//!     - The function has exactly one parameter of type pointer to @b "<function_name>_struct".
//!     .
//!     @code
//!       void SimpleCalc(SimpleCalc_struct *instance)
//!       {
//!         // Implementation of functionality
//!         instance->out = instance->in1 + instance->in2;
//!         return;
//!       }
//!     @endcode
//! \n
//! - Finally register the C/C++ implementation within the runtime to enable call of function @b "<function_name>"
//!   within a PLC application. The registered name of the C/C++ function must match the name of the IEC function. The
//!   name of the C/C++ function freely selectable and can be different.
//!   @code
//!     WCHAR16 name[MLPI_APPLICATION_MAX_LENGTH_OF_POU_NAME] = L"SimpleCalc";
//!     MLPIPOUFNCPTR function = (MLPIPOUFNCPTR) SimpleCalc;
//!     MLPIRESULT result = mlpiLogicPouExtensionRegister(connection, name, function);
//!   @endcode
//! \n
//! - After the external function has been registered, you can download your PLC program and use the function.
//!
//! @par Example "IEC function block":
//! \n
//! A function block corresponds to at least one implicit method called "__Main". A function block can be extended by a
//! composition of several methods and properties. Additional @b predefined methods are @b "FB_Init", @b "FB_Exit" and
//! @b "FB_Reinit". These initialization and exit methods have a fixed interface and fixed variable names and will be called
//! automatically in predefined situations. The inputs of the predefined methods will be set by the runtime to mark
//! the situation exactly.
//! Furthermore, a customer can add any further method or property. Each method or property can be implemented independently
//! in the IEC 61131 environment IndraWorks respectively in the C/C++ environment.
//! \n\n
//! The predefined methods @b "FB_Init", @b "FB_Exit" and @b "FB_Reinit" are called on following situations:
//! <TABLE>
//! <TR><TH rowspan="2">Action                </TH><TH> FB_Exit     </TH><TH colspan="2"> FB_Init </TH>                        <TH rowspan="2">FB_Reinit </TH><TH rowspan="2">Shown in diagram... </TH></TR>
//! <TR>                                           <TH> bInCopyCode </TH><TH> bInCopyCode         </TH><TH> bInitRetains  </TH>                                                                        </TR>
//! <TR><TD id="st_e"> Start, Stop            </TD><TD> --          </TD><TD> --                  </TD><TD> --            </TD><TD> --                   </TD><TD> 'base actions'                 </TD></TR>
//! <TR><TD id="st_e"> Single cycle           </TD><TD> --          </TD><TD> --                  </TD><TD> --            </TD><TD> --                   </TD><TD> 'other actions'                </TD></TR>
//! <TR><TD id="st_e"> Download, Boot up      </TD><TD> --          </TD><TD> FALSE               </TD><TD> TRUE          </TD><TD> --                   </TD><TD> 'base actions'                 </TD></TR>
//! <TR><TD id="st_e"> Download new           </TD><TD> FALSE       </TD><TD> FALSE               </TD><TD> TRUE          </TD><TD> --                   </TD><TD> 'other actions'                </TD></TR>
//! <TR><TD id="st_e"> Reset warm             </TD><TD> FALSE       </TD><TD> FALSE               </TD><TD> FALSE         </TD><TD> --                   </TD><TD> 'reset actions ...'            </TD></TR>
//! <TR><TD id="st_e"> Reset cold             </TD><TD> FALSE       </TD><TD> FALSE               </TD><TD> TRUE          </TD><TD> --                   </TD><TD> 'reset actions ...'            </TD></TR>
//! <TR><TD id="st_e"> Reset origin           </TD><TD> FALSE       </TD><TD> --                  </TD><TD> --            </TD><TD> --                   </TD><TD> 'reset actions ...'            </TD></TR>
//! <TR><TD id="st_e"> Online change, w/o     </TD><TD> --          </TD><TD> --                  </TD><TD> --            </TD><TD> --                   </TD><TD> 'online change actions ...'    </TD></TR>
//! <TR><TD id="st_e"> Online change, new     </TD><TD> --          </TD><TD> FALSE               </TD><TD> FALSE         </TD><TD> --                   </TD><TD> 'online change actions ...'    </TD></TR>
//! <TR><TD id="st_e"> Online change, delete  </TD><TD> FALSE       </TD><TD> --                  </TD><TD> --            </TD><TD> --                   </TD><TD> 'online change actions ...'    </TD></TR>
//! <TR><TD id="st_e"> Online change, move    </TD><TD> <b>TRUE</b> </TD><TD> <b>TRUE</b>         </TD><TD> FALSE         </TD><TD> called               </TD><TD> 'online change actions ...'    </TD></TR>
//! </TABLE>
//!
//! @remark If you allocate resp. new objects within FB_Init you have to free resp. delete it on FB_Exit except on action
//!         "Online Change, move', which will be recognized through 'bInCopyCode == TRUE'.
//!
//! @note Used enumeration values on transition notes are short forms of @ref MlpiApplicationEvent meaning e.g. 'STOP_INIT'
//!       is equal to 'MLPI_APPLICATIONEVENT_STOP_INIT'.
//!       The event 'OP_STATE_CHANGE' resp. 'MLPI_APPLICATIONEVENT_OP_STATE_CHANGE' could be called in series (multiple)
//!       without further notice.
//!       The method '__main' resp. "__Main" could be called in series (multiple) without further notice.
//!
//! @image html PouStateDiagram_base_action.png "Call of predefined methods in context of 'base actions'" \n
//! @image html PouStateDiagram_reset_actions_RUN.png "Call of predefined methods in context of 'reset action on RUN'" \n
//! @image html PouStateDiagram_reset_actions_STOP.png "Call of predefined methods in context of 'reset actions on STOP'" \n
//! @image html PouStateDiagram_online_change_actions_RUN.png "Call of predefined methods in context of 'online change actions on RUN'" \n
//! @image html PouStateDiagram_online_change_actions_STOP.png "Call of predefined methods in context of 'online change actions on STOP'" \n
//! @image html PouStateDiagram_other_action.png "Call of predefined methods in context of 'other actions'" \n
//! \n
//! To use a C/C++ implementation within the IEC 61131 environment IndraWorks as a @b function @b block you need to do the
//! following:
//! \n\n
//! - Declare an external POU (function block) with a proper interface within IndraWorks.
//!   - Add a POU @b "<functionblock_name>" (e.g. 'SimpleCalc') as a function block .
//!     @image html ExtIecFbSimpleCalc_Add.png "Adding new POU 'SimpleCalc' as a function block." \n
//!   - Declare the interface of inputs, outputs and local variables of the function block. You need not implement any
//!     IEC code.
//!     @code
//!       FUNCTION_BLOCK SimpleCalc
//!       VAR_INPUT
//!         in1: LREAL;
//!         in2: LREAL;
//!       END_VAR
//!       VAR_OUTPUT
//!         out: LREAL;
//!       END_VAR
//!       VAR
//!       END_VAR
//!     @endcode
//!     @image html ExtIecFbSimpleCalc_Decl.png "Declaration of interface of IEC function block." \n
//!   - Enable the build property "External implementation" (late link in the runtime) of POU. You will find the
//!     build property on right click on POU within the project tree ("Property...->Build").
//!     @image html ExtIecFbSimpleCalc_ExtImpl.png "Build property: External Implementation." \n
//!   - According to your requirements, add the predefined methods @b FB_Init, @b FB_Exit and / or @b FB_Reinit and your further
//!     methods and properties and declare the interface of each POU. You also need not implement any code.
//!     @code
//!       METHOD FB_Init : BOOL
//!       VAR_INPUT
//!         bInitRetains: BOOL;
//!         bInCopyCode: BOOL;
//!       END_VAR
//!     @endcode
//!     @image html ExtIecFbSimpleCalc_DeclInit.png "Declaration of interface of INIT method of IEC function block." \n
//!     @code
//!       METHOD FB_Reinit : BOOL
//!       VAR_INPUT
//!       END_VAR
//!     @endcode
//!     @image html ExtIecFbSimpleCalc_DeclReinit.png "Declaration of interface of REINIT method of IEC function block." \n
//!     @code
//!       METHOD FB_Exit : BOOL
//!       VAR_INPUT
//!         bInCopyCode: BOOL;
//!       END_VAR
//!     @endcode
//!     @image html ExtIecFbSimpleCalc_DeclExit.png "Declaration of interface of EXIT method of IEC function block." \n
//!   - Enable the build property "External implementation" (late link in the runtime) of all desired POUs.
//! \n\n
//! - Declare the corresponding interfaces within a C/C++ file.
//!   - The instance of a function block contains all function block variables in the declared order plus a pointer
//!     to the virtual function table at first. Declare an instance structure @b "<functionblock_name>_struct" with
//!     the elements of the IEC function block. Please  note that the structure of the @b instance is solely determined
//!     by the order of the declaration, rather than through affiliation with an input, output or local. If the outputs
//!     and inputs were interchanged in the IEC declaration, they would also have to be interchanged in your C/C++
//!     environment. This example uses the MLPI IEC data types like MLPI_IEC_LREAL (see mlpiGlobal.h) for a valid alignment.
//!     The declaration of your instance must match the following criteria:
//!     - At first a virtual function pointer.
//!     - All variables in the order defined in IEC function block.
//!     - All implicit local property variables in case of using of properties, named like the resp. properties.
//!     .
//!     @code
//!       typedef struct SimpleCalc_struct
//!       {
//!         void*   __VFTABLEPOINTER;             // Declaration of virtual function pointer
//!         MLPI_IEC_LREAL  in1;                  // Declaration of function block input
//!         MLPI_IEC_LREAL  in2;                  // Declaration of function block input
//!         MLPI_IEC_LREAL  out;                  // Declaration of function block output
//!         MLPI_IEC_LREAL  userPropVar;          // Implicit local property variable
//!       }SimpleCalc_struct;
//!     @endcode
//!   - Alternatively the structure can be declared with proper alignment dummies.
//!     @code
//!       typedef struct SimpleCalc_struct
//!       {
//!         void*   __VFTABLEPOINTER;             // Declaration of virtual function pointer
//!         ULONG   alignment;                    // Alignment dummy
//!         DOUBLE  in1;                          // Declaration of function block input
//!         DOUBLE  in2;                          // Declaration of function block input
//!         DOUBLE  out;                          // Declaration of function block output
//!         DOUBLE  userPropVar;                  // Implicit local property variable
//!       }SimpleCalc_struct;
//!     @endcode
//!   - Declare a structure @b "<functionblock_name>_Main_struct" with a pointer to the instance structure
//!     @b "<functionblock_name>_struct".
//!     @code
//!       typedef struct SimpleCalc_Main_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!       }SimpleCalc_Main_struct;
//!     @endcode
//!   - In addition, declare all needed method structures like in the following examples @b "<functionblock_name>_<method_name>_struct".
//!     All methods first have to implement a pointer to the instance structure. These examples use the MLPI IEC data types like MLPI_IEC_BOOL
//!     (see mlpiGlobal.h) for valid alignments, but in this case, there are no matters if using BOOL8.
//!     @code
//!       typedef struct SimpleCalc_FB_Init_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         MLPI_IEC_BOOL       bInitRetains;     // Declaration of predefined method input (no matter if using BOOL8)
//!         MLPI_IEC_BOOL       bInCopyCode;      // Declaration of predefined method input (no matter if using BOOL8)
//!         MLPI_IEC_BOOL       FB_Init;          // Declaration of implicit method output (no matter if using BOOL8)
//!       }SimpleCalc_FB_Init_struct;
//!     @endcode
//!     @code
//!       typedef struct SimpleCalc_FB_Reinit_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         MLPI_IEC_BOOL       FB_Reinit;        // Declaration of implicit method output (no matter if using BOOL8)
//!       }SimpleCalc_FB_Reinit_struct;
//!     @endcode
//!     @code
//!       typedef struct SimpleCalc_FB_Exit_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         MLPI_IEC_BOOL       bInCopyCode;      // Declaration of predefined method input (no matter if using BOOL8)
//!         MLPI_IEC_BOOL       FB_Exit;          // Declaration of implicit method output (no matter if using BOOL8)
//!       }SimpleCalc_FB_Exit_struct;
//!     @endcode
//!   - If desired, you have to declare properties like in the following examples @b "<functionblock_name>_set<property_name>_struct"
//!     resp.  @b "<functionblock_name>_get<property_name>_struct". A property also has to implement a pointer
//!     to the instance structure. A property implements an implicit local variable within the instance of the POU. This example
//!     uses the MLPI IEC data types like MLPI_IEC_LREAL (see mlpiGlobal.h) for a valid alignment.
//!     @code
//!       typedef struct SimpleCalc_setUserPropVar_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         MLPI_IEC_LREAL      setUserPropVar;   // Declaration of setter input
//!         MLPI_IEC_BOOL       __setUserPropVar; // Declaration implicit property output
//!       }SimpleCalc_setUserPropVar_struct;
//!     @endcode
//!     @code
//!       typedef struct SimpleCalc_getUserPropVar_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         MLPI_IEC_LREAL      __getUserPropVar; // Declaration of implicit getter output
//!       }SimpleCalc_getUserPropVar_struct;
//!     @endcode
//!   - Alternatively the setter and getter can be declared with proper alignment dummies.
//!     @code
//!       typedef struct SimpleCalc_setUserPropVar_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         ULONG               alignment;        // Alignment dummy
//!         DOUBLE              setUserPropVar;   // Declaration of setter input
//!         BOOL8               __setUserPropVar; // Declaration implicit property output (status)
//!       }SimpleCalc_setUserPropVar_struct;
//!     @endcode
//!     @code
//!       typedef struct SimpleCalc_getUserPropVar_struct
//!       {
//!         SimpleCalc_struct  *instance;         // Declaration of instance pointer
//!         ULONG               alignment;        // Alignment dummy
//!         DOUBLE              __getUserPropVar; // Declaration of implicit getter output
//!       }SimpleCalc_getUserPropVar_struct;
//!     @endcode
//!   - Implement the required functionalities of @b "<function_name>__Main", @b "<function_name>__FB_Init",
//!     @b "<function_name>__FB_Reinit", @b "<function_name>__FB_Exit", ..., @b "<function_name>__<method_name>",
//!     @b "<function_name>__set<property_name>" and @b "<function_name>__get<property_name>".
//!     - The return type of each function is @b void.
//!     - Each function has exactly one parameter of type pointer to the respective structure.
//!     .
//!     @code
//!       #include "mlpiLogicLib.h"
//!
//!       void SimpleCalc__Main(SimpleCalc_Main_struct *p)
//!       {
//!         SimpleCalc_struct *instance = (SimpleCalc_struct*) p->instance;
//!
//!         // Implementation of functionality
//!         instance->out = instance->in1 + instance->in2;
//!         return;
//!       }
//!     @endcode
//!     @code
//!       void SimpleCalc__FB_Init(SimpleCalc_FB_Init_struct *p)
//!       {
//!         SimpleCalc_struct *instance = (SimpleCalc_struct*) p->instance;
//!
//!         // Implementation of functionality
//!         if( (p->bInitRetains==TRUE) && (p->bInCopyCode==FALSE) ) {
//!           // Download active, do something...
//!         }
//!
//!         instance->out = 0;
//!         p->FB_Init = TRUE;
//!         return;
//!       }
//!     @endcode
//!     @code
//!       void SimpleCalc__getUserPropVar(SimpleCalc_getUserPropVar_struct *p)
//!       {
//!         p->__getUserPropVar = p->instance->userPropVar * 42.0;
//!       }
//!       void SimpleCalc__setUserPropVar(SimpleCalc_setUserPropVar_struct *p)
//!       {
//!         p->instance->userPropVar = p->setUserPropVar / 42.0;
//!         p->__setUserPropVar = TRUE;
//!       }
//!     @endcode
//! \n
//! - Finally, register all C/C++ implementations within the runtime to enable call of the function block
//!   @b "<functionblock_name>" and the desired method and properties within a PLC application. The registered
//!   names of the C/C++ functions must match the name of IEC function block and the following scheme.
//!   The function block name [functionblock_name] must be separated by two underscores from the extension part.
//!   - [functionblock_name]__[method_name]
//!   - [functionblock_name]__set[property_name]
//!   - [functionblock_name]__get[property_name]
//!   .
//!   @code
//!     WCHAR16 name[MLPI_APPLICATION_MAX_LENGTH_OF_POU_NAME] = L"SimpleCalc__Main";
//!     MLPIPOUFNCPTR function = (MLPIPOUFNCPTR) SimpleCalc__Main;
//!     MLPIRESULT result = mlpiLogicPouExtensionRegister(connection, name, function);
//!     if(MLPI_SUCCEEDED(result)) {
//!       wcscpy16(name, L"SimpleCalc__FB_Init");
//!       function = (MLPIPOUFNCPTR) SimpleCalc__FB_Init;
//!       result = mlpiLogicPouExtensionRegister(connection, name, function);
//!     }
//!     if(MLPI_SUCCEEDED(result)) {
//!       wcscpy16(name, L"SimpleCalc__getUserPropVar");
//!           function = (MLPIPOUFNCPTR) SimpleCalc__getUserPropVar;
//!           result = mlpiLogicPouExtensionRegister(connection, name, function);
//!     }
//!     if(MLPI_SUCCEEDED(result)) {
//!     wcscpy16(name, L"SimpleCalc__setUserPropVar");
//!           function = (MLPIPOUFNCPTR) SimpleCalc__setUserPropVar;
//!           result = mlpiLogicPouExtensionRegister(connection, name, function);
//!     }
//!   @endcode
//! \n
//! - After the external function is registered, you can download your PLC program and use the function block.
MLPI_API MLPIRESULT mlpiLogicPouExtensionRegister(const MLPIHANDLE connection, const WCHAR16* name, const MLPIPOUFNCPTR function, const ULONG signature = 0, const ULONG version = 0);


//! @ingroup LogicLibPouExtension
//! This function unregisters a C/C++ implementation of a POU (e.g. function block) at MLPI real-time environment.
//!
//! @note This function can only be used within a real-time application on the target and there only within the
//!       kernel space! This function is not available in other toolboxes!
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    name              Name of POU in the IEC 61131 environment.
//! @param[in]    function          Function pointer to C/C++ implementation.
//! @param[in]    signature         Signature of POU interface.
//! @param[in]    version           Version of POU library.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Unregister a C/C++ implementation to disable call of function "SimpleCalc" in a PLC application.
//! WCHAR16 name[MLPI_APPLICATION_MAX_LENGTH_OF_POU_NAME] = L"SimpleCalc";
//! MLPIPOUFNCPTR function = (MLPIPOUFNCPTR) SimpleCalc;
//! MLPIRESULT result = mlpiLogicPouExtensionUnregister(connection, name, function);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicPouExtensionUnregister(const MLPIHANDLE connection, const WCHAR16* name, const MLPIPOUFNCPTR function, const ULONG signature = 0, const ULONG version = 0);


//! @ingroup LogicLibPouExtension
//! This function unregisters all C/C++ implementations of POUs (e.g. function blocks) at MLPI real-time environment.
//!
//! @note This function can only be used within a real-time application on the target and there only within the
//!       kernel space! This function is not available in other toolboxes!
//!
//! @param[in]    connection        Handle for multiple connections.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Unregister all C/C++ implementations to disable all calls in a PLC application.
//! MLPIRESULT result = mlpiLogicPouExtensionUnregisterAll(connection);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicPouExtensionUnregisterAll(const MLPIHANDLE connection);

#endif


//! @ingroup LogicLibReadSymbol
//! This function reads a boolean value (BOOL, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! BOOL8 data = FALSE;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolBool8(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolBool8(const MLPIHANDLE connection, const WCHAR16 *symbol, BOOL8 *data);


//! @ingroup LogicLibReadSymbol
//! This function reads an 8 bit signed value (SINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! CHAR data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolChar(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolChar(const MLPIHANDLE connection, const WCHAR16 *symbol, CHAR *data);


//! @ingroup LogicLibReadSymbol
//! This function reads an 8-bit unsigned value (BYTE, USINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! UCHAR data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolUchar(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolUchar(const MLPIHANDLE connection, const WCHAR16 *symbol, UCHAR *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 16-bit signed value (INT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! SHORT data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolShort(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolShort(const MLPIHANDLE connection, const WCHAR16 *symbol, SHORT *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 16-bit unsigned value (WORD, UINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! USHORT data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolUshort(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolUshort(const MLPIHANDLE connection, const WCHAR16 *symbol, USHORT *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 32-bit signed value (DINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LONG data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolLong(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolLong(const MLPIHANDLE connection, const WCHAR16 *symbol, LONG *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 32-bit unsigned value (DWORD, UDINT, TIME, DATE, DATE_AND_TIME, ..., @ref MlpiLogicType)
//! from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolUlong(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolUlong(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 64-bit signed value (LINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LLONG data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolLlong(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolLlong(const MLPIHANDLE connection, const WCHAR16 *symbol, LLONG *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 64-bit unsigned value (LWORD, ULINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULLONG data = 0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolUllong(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolUllong(const MLPIHANDLE connection, const WCHAR16 *symbol, ULLONG *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 32-bit floating point value (single precision, REAL, @ref MlpiLogicType) from a variable
//! by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! FLOAT data = 0.0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolFloat(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolFloat(const MLPIHANDLE connection, const WCHAR16 *symbol, FLOAT *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a 64-bit floating point value (double precision, LREAL, @ref MlpiLogicType) from a variable
//! by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! DOUBLE data = 0.0;
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolDouble(connection, L"Application.PlcProg.x1", &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolDouble(const MLPIHANDLE connection, const WCHAR16 *symbol, DOUBLE *data);


//! @ingroup LogicLibReadSymbol
//! This function reads a character string value (STRING, @ref MlpiLogicType) from a variable by symbolic access.
//! You can use this function not only for string variables, but also for other simple types and arrays of simple
//! types. In this case, the function returns a string representation of the given value. The values of a string
//! represented array will be separated by spaces. The reading of values as a string can be useful if you only
//! want to display the variable as a string and you don't need to know the type of the variable.
//!
//! @par Hint:
//!            From version 1.4.0.0 of the MLPI server this function supports arrays of simple types and furthermore
//!            different display types to show the string represented values in hexadecimal, decimal, binary or
//!            boolean format. To use the display extension you have to add a prefix to your symbol. The prefixes
//!            are defined in the helper header "mlpiLogicHelper.h".
//! Hint:
//!            From Version 1.11.8.0 of the MLPI server this function supports data types TIME, DATE, DATE_AND_TIME,
//!            and TIME_OF_DAY
//!
//! <TABLE>
//! <TR><TH>             Data types of PLC          </TH><TH>           Valid prefixes for argument 'symbol'  </TH><TH> Example of argument 'symbol'           </TH><TH> Example of result 'data' </TH></TR>
//! <TR><TD rowspan="5"> BOOL                       </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarBool"      </TD><TD> L"TRUE"                  </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_BOOLEAN    </TD><TD> L"BLN:Application.PlcProg.locVarBool"  </TD><TD> L"TRUE"                  </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_DEC        </TD><TD> L"DEC:Application.PlcProg.locVarBool"  </TD><TD> L"1"                     </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_HEX        </TD><TD> L"HEX:Application.PlcProg.locVarBool"  </TD><TD> L"0x01"                  </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_BIN        </TD><TD> L"BIN:Application.PlcProg.locVarBool"  </TD><TD> L"0b0000.0001"           </TD></TR>
//! <TR><TD rowspan="4"> BYTE, WORD, DWORD, LWORD   </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarByte"      </TD><TD> L"222"                   </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_DEC        </TD><TD> L"DEC:Application.PlcProg.locVarByte"  </TD><TD> L"222"                   </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_HEX        </TD><TD> L"HEX:Application.PlcProg.locVarByte"  </TD><TD> L"0xDE"                  </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_BIN        </TD><TD> L"BIN:Application.PlcProg.locVarByte"  </TD><TD> L"0b1101.1110"           </TD></TR>
//! <TR><TD rowspan="4"> USINT, UINT, UDINT, ULINT  </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarUint"      </TD><TD> L"42"                    </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_DEC        </TD><TD> L"DEC:Application.PlcProg.locVarUint"  </TD><TD> L"42"                    </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_HEX        </TD><TD> L"HEX:Application.PlcProg.locVarUint"  </TD><TD> L"0x2A"                  </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_BIN        </TD><TD> L"BIN:Application.PlcProg.locVarUint"  </TD><TD> L"0b0010.1010"           </TD></TR>
//! <TR><TD rowspan="4"> SINT, INT, DINT, LINT      </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarInt"       </TD><TD> L"-16982"                </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_DEC        </TD><TD> L"DEC:Application.PlcProg.locVarInt"   </TD><TD> L"-16982"                </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_HEX        </TD><TD> L"HEX:Application.PlcProg.locVarInt"   </TD><TD> L"0xBDAA"                </TD></TR>
//! <TR>                                                 <TD id="st_e"> MLPI_READ_VAR_DISPLAY_TYPE_BIN        </TD><TD> L"BIN:Application.PlcProg.locVarInt"   </TD><TD> L"0b1011.1101.1010.1010" </TD></TR>
//! <TR><TD rowspan="1"> REAL, LREAL                </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarReal"      </TD><TD> L"3.141592"              </TD></TR>
//! <TR><TD rowspan="1"> STRING, WSTRING            </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarString"    </TD><TD> L"Don't panic!"          </TD></TR>
//! <TR><TD rowspan="1"> TIME                       </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarTime  "    </TD><TD> L"5d12h34m15s12ms"       </TD></TR>
//! <TR><TD rowspan="1"> DATE                       </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarTime  "    </TD><TD> L"2010-03-29"            </TD></TR>
//! <TR><TD rowspan="1"> DATE_AND_TIME              </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarTime  "    </TD><TD> L"2010-03-29-15:36:30"   </TD></TR>
//! <TR><TD rowspan="1"> TIME_OF_DAY                </TD><TD id="st_e"> none                                  </TD><TD> L"Application.PlcProg.locVarTime  "    </TD><TD> L"15:36:30.123"          </TD></TR>
//! </TABLE>
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the value(s) will be stored.
//! @param[in]    numElements       Number of WCHAR16 elements in 'data' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the value of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! WCHAR16 data[4096] = L"";
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolString(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolString(const MLPIHANDLE connection, const WCHAR16 *symbol, WCHAR16 *data, const ULONG numElements);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of Boolean values (BOOL, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of BOOL8 elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! BOOL8 data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayBool8(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayBool8(const MLPIHANDLE connection, const WCHAR16 *symbol, BOOL8 *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 8-bit signed values (SINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of CHAR elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! CHAR data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayChar(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayChar(const MLPIHANDLE connection, const WCHAR16 *symbol, CHAR *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 8-bit unsigned values (BYTE, USINT, @ref MlpiLogicType) from a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of UCHAR elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! UCHAR data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayUchar(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayUchar(const MLPIHANDLE connection, const WCHAR16 *symbol, UCHAR *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 16-bit signed values (INT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of SHORT elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! SHORT data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayShort(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayShort(const MLPIHANDLE connection, const WCHAR16 *symbol, SHORT *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 16-bit unsigned values (WORD, UINT, @ref MlpiLogicType) from a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of USHORT elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! USHORT data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayUshort(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayUshort(const MLPIHANDLE connection, const WCHAR16 *symbol, USHORT *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 32-bit signed values (DINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of LONG elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LONG data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayLong(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayLong(const MLPIHANDLE connection, const WCHAR16 *symbol, LONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 32-bit unsigned values (DWORD, UDINT, TIME, DATE, DATE_AND_TIME, ...,
//! @ref MlpiLogicType) from a variable.
//! by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of ULONG elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayUlong(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayUlong(const MLPIHANDLE connection, const WCHAR16 *symbol, ULONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 64-bit signed values (LINT, @ref MlpiLogicType) from a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of LLONG elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LLONG data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayLlong(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayLlong(const MLPIHANDLE connection, const WCHAR16 *symbol, LLONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 64-bit unsigned values (LWORD, ULINT, @ref MlpiLogicType) from a variable
//! by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of ULLONG elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULLONG data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayUllong(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayUllong(const MLPIHANDLE connection, const WCHAR16 *symbol, ULLONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 32-bit floating point values (single precision, REAL, @ref MlpiLogicType) from a
//! variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of FLOAT elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! FLOAT data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayFloat(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayFloat(const MLPIHANDLE connection, const WCHAR16 *symbol, FLOAT *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads an array of 64-bit floating point values (double precision, LREAL, @ref MlpiLogicType) from
//! a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    numElements       Number of DOUBLE elements in 'data' available to read.
//! @param[out]   numElementsRet    Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! DOUBLE data[10];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayDouble(connection, L"Application.PlcProg.x1", data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayDouble(const MLPIHANDLE connection, const WCHAR16 *symbol, DOUBLE *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibReadSymbol
//! This function reads user-defined data type values from a variable by symbolic access (raw reading).
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[out]   data              Pointer to variable where the values will be stored.
//! @param[in]    dataSize          Number of bytes in 'data' available to read.
//! @param[out]   dataSizeRet       Number of bytes used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note It's highly recommended to use MLPI IEC data types (like MLPI_IEC_LREAL, MLPI_IEC_USINT, ... see mlpiGlobal.h)
//!       for an user defined structure.
//!
//! @par Example:
//! @code
//! // Read the values of the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! DUT data;
//! ULONG dataSizeRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayVoid(connection, L"Application.PlcProg.x1", &data, sizeof(data), &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadVariableBySymbolArrayVoid(const MLPIHANDLE connection, const WCHAR16 *symbol, void *data, const ULONG dataSize, ULONG *dataSizeRet);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given Boolean value (BOOL, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value 'TRUE' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! BOOL8 data = TRUE;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolBool8(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolBool8(const MLPIHANDLE connection, const WCHAR16 *symbol, const BOOL8 data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 8-bit signed value (SINT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '42' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! CHAR data = 42;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolChar(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolChar(const MLPIHANDLE connection, const WCHAR16 *symbol, const CHAR data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 8-bit unsigned value (BYTE, USINT, @ref MlpiLogicType) to a variable by
//!symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '0x2A' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! UCHAR data = 0x2A;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolUchar(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolUchar(const MLPIHANDLE connection, const WCHAR16 *symbol, const UCHAR data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 16-bit signed value (INT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '42' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! SHORT data = 42;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolShort(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolShort(const MLPIHANDLE connection, const WCHAR16 *symbol, const SHORT data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 16-bit unsigned value (WORD, UINT, @ref MlpiLogicType) to a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '0x2A' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! USHORT data = 0x2A;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolUshort(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolUshort(const MLPIHANDLE connection, const WCHAR16 *symbol, const USHORT data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 32-bit signed value (DINT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '42' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LONG data = 42;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolLong(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolLong(const MLPIHANDLE connection, const WCHAR16 *symbol, const LONG data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 32-bit unsigned value (DWORD, UDINT, TIME, DATE, DATE_AND_TIME, ...,
//! @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '0x2A' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG data = 0x2A;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolUlong(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolUlong(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 64-bit signed value (LINT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '42' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LLONG data = 42;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolLlong(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolLlong(const MLPIHANDLE connection, const WCHAR16 *symbol, const LLONG data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 64-bit unsigned value (LWORD, ULINT, @ref MlpiLogicType) to a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '0x2A' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULLONG data = 0x2A;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolUllong(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolUllong(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULLONG data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 32-bit floating point value (single precision, REAL, @ref MlpiLogicType) to a
//! variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '1.23' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! FLOAT data = 1.23;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolFloat(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolFloat(const MLPIHANDLE connection, const WCHAR16 *symbol, const FLOAT data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given 64-bit floating point value (double precision, LREAL, @ref MlpiLogicType) to a
//! variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value '1.23456' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! DOUBLE data = 1.23456;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolDouble(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolDouble(const MLPIHANDLE connection, const WCHAR16 *symbol, const DOUBLE data);


//! @ingroup LogicLibWriteSymbol
//! This function writes a given character string value (STRING, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the value 'This is a string' to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! WCHAR16 data[]=L"This is a string";
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolString(connection, L"Application.PlcProg.x1", data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolString(const MLPIHANDLE connection, const WCHAR16 *symbol, const WCHAR16 *data);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of Boolean values (BOOL, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of BOOL8 elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! BOOL8 data[] = {TRUE, FALSE};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayBool8(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayBool8(const MLPIHANDLE connection, const WCHAR16 *symbol, const BOOL8 *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 8-bit signed values (SINT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of CHAR elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! CHAR data[] = {-1, 2};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayChar(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayChar(const MLPIHANDLE connection, const WCHAR16 *symbol, const CHAR *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 8-bit unsigned values (BYTE, USINT, @ref MlpiLogicType) to a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of UCHAR elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! UCHAR data[] = {1, 85};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayUchar(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUchar(const MLPIHANDLE connection, const WCHAR16 *symbol, const UCHAR *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 16-bit signed values (INT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of SHORT elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! SHORT data[] = {-18, 65};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayShort(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayShort(const MLPIHANDLE connection, const WCHAR16 *symbol, const SHORT *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 16-bit unsigned values (WORD, UINT, @ref MlpiLogicType) to a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of USHORT elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! USHORT data[] = {18, 33};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayUshort(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUshort(const MLPIHANDLE connection, const WCHAR16 *symbol, const USHORT *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 32-bit signed values (DINT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of LONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LONG data[] = {-42, 99};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayLong(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayLong(const MLPIHANDLE connection, const WCHAR16 *symbol, const LONG *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 32-bit unsigned values (DWORD, UDINT, TIME, DATE, DATE_AND_TIME, ...,
//! @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of ULONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULONG data[] = {42, 55};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayUlong(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUlong(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULONG *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 64-bit signed values (LINT, @ref MlpiLogicType) to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of LLONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! LLONG data[] = {-13, 44};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayLlong(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayLlong(const MLPIHANDLE connection, const WCHAR16 *symbol, const LLONG *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 64-bit unsigned values (LWORD, ULINT, @ref MlpiLogicType) to a variable by
//! symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values should be written.
//! @param[in]    numElements       Number of ULLONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! ULLONG data[] = {13, 77};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayUllong(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayUllong(const MLPIHANDLE connection, const WCHAR16 *symbol, const ULLONG *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 32-bit floating point values (single precision, REAL, @ref MlpiLogicType) to a
//! variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of FLOAT elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! FLOAT data[] = {1.234, 9.907};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayFloat(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayFloat(const MLPIHANDLE connection, const WCHAR16 *symbol, const FLOAT *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes an array of 64-bit floating point values (double precision, LREAL, @ref MlpiLogicType) to a
//! variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    numElements       Number of DOUBLE elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writing the values to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! DOUBLE data[] = {1.23456789, 9.87654321};
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayDouble(connection, L"Application.PlcProg.x1", data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayDouble(const MLPIHANDLE connection, const WCHAR16 *symbol, const DOUBLE *data, const ULONG numElements);


//! @ingroup LogicLibWriteSymbol
//! This function writes user-defined data type values to a variable by symbolic access.
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    symbol            Symbol variable of PLC application.
//! @param[in]    data              Pointer to variable which contains the values that should be written.
//! @param[in]    dataSize          Number of bytes in data available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note It's highly recommended to use MLPI IEC data types (like MLPI_IEC_LREAL, MLPI_IEC_USINT, ... see mlpiGlobal.h)
//!       for an user defined structure.
//!
//! @par Example:
//! @code
//! // Writing the value to the variable 'x1' in the program 'PlcProg' in an application called 'Application'.
//! DUT data;
//! MLPIRESULT result = mlpiLogicWriteVariableBySymbolArrayVoid(connection, L"Application.PlcProg.x1", &data, sizeof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteVariableBySymbolArrayVoid(const MLPIHANDLE connection, const WCHAR16 *symbol, const void *data, const ULONG dataSize);


//! @ingroup LogicLibAreaRd
//! This function reads a bit as a BOOL8 (BOOL8, @ref MlpiLogicType) from the memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   bitOffset          Bit offset based on zero (e.g. %IX0.0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IX1.5.
//! BOOL8 data = FALSE;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaBool8(connection, application, MLPI_MEMORY_AREA_INPUT, 13, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaBool8(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG bitOffset, BOOL8* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 8-bit signed data value (SINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IB42.
//! CHAR data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaChar(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaChar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, CHAR* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 8-bit unsigned data value (BYTE, USINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IB42.
//! UCHAR data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaUchar(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaUchar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, UCHAR* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 16-bit signed data value (INT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IW42.
//! SHORT data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaShort(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaShort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, SHORT* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 16-bit unsigned data value (WORD, UINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IW42.
//! USHORT data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaUshort(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaUshort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, USHORT* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 32-bit signed data value (DINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %ID42.
//! LONG data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaLong(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaLong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LONG* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 32-bit unsigned data value (DWORD, UDINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %ID42.
//! ULONG data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaUlong(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaUlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULONG* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 64-bit signed data value (LINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IL42.
//! LLONG data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaLlong(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaLlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LLONG* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 64-bit unsigned data value (LWORD, ULINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IL42.
//! ULLONG data = 0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaUllong(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaUllong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULLONG* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 32-bit floating point data value (single precision, REAL, @ref MlpiLogicType) from the
//! memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %ID42.
//! FLOAT data = 0.0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaFloat(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaFloat(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, FLOAT* data);


//! @ingroup LogicLibAreaRd
//! This function reads the 64-bit floating point data value (double precision, LREAL, @ref MlpiLogicType)  from the
//! memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IL42.
//! DOUBLE data = 0.0;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicReadMemoryAreaDouble(connection, application, MLPI_MEMORY_AREA_INPUT, 42, &data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaDouble(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, DOUBLE* data);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 8-bit signed data values (SINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of CHAR elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data values from the memory area %IB0 to %IB1.
//! CHAR data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayChar(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayChar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, CHAR* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 8-bit unsigned data values (BYTE, USINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of UCHAR elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data values from the memory area %IB0 to %IB1.
//! UCHAR data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayUchar(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayUchar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, UCHAR* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 16-bit signed data values (INT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of SHORT elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IW0 to %IW2.
//! SHORT data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayShort(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayShort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, SHORT* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 16-bit unsigned data values (WORD, UINT, @ref MlpiLogicType) from the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of USHORT elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IW0 to %IW2.
//! USHORT data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayUshort(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayUshort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, USHORT* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 32-bit signed data values (DINT, @ref MlpiLogicType) from the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of LONG elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %ID0 to %ID4.
//! LONG data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayLong(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayLong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 32-bit unsigned data values (DWORD, UDINT, @ref MlpiLogicType) from the
//! memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of ULONG elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %ID0 to %ID4.
//! ULONG data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayUlong(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayUlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 64-bit signed data values (LINT, @ref MlpiLogicType) from the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of LLONG elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IL0 to %IL8.
//! LLONG data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayLlong(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayLlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, LLONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 64-bit unsigned data values (LWORD, ULINT, @ref MlpiLogicType) from the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of ULLONG elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IL0 to %IL8.
//! ULLONG data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayUllong(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayUllong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, ULLONG* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 32-bit floating point data values (single precision, REAL, @ref MlpiLogicType)
//! from the memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of FLOAT elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %ID0 to %ID4.
//! FLOAT data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayFloat(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayFloat(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, FLOAT* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaRd
//! This function reads an array of 64-bit floating point data values (double precision, LREAL, @ref MlpiLogicType)
//! from the memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %IB0).
//! @param[out]  data               Pointer to variable where the values will be stored.
//! @param[in]   numElements        Number of DOUBLE elements in data available to read.
//! @param[out]  numElementsRet     Number of elements used.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value from the memory area %IL0 to %IL8.
//! DOUBLE data[2];
//! ULONG numElementsRet = 0;
//! WCHAR16 application = L"Application";
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiLogicReadMemoryAreaArrayDouble(connection, application, MLPI_MEMORY_AREA_INPUT, 0, data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicReadMemoryAreaArrayDouble(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, DOUBLE* data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup LogicLibAreaWr
//! This function writes a Boolean value e.g. a bit (BOOL8, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   bitOffset          Bit offset based on zero (e.g. %QX0.0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QX1.5.
//! BOOL8 data = TRUE;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaBool8(connection, application, MLPI_MEMORY_AREA_OUTPUT, 13, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaBool8(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG bitOffset, const BOOL8 data);


//! @ingroup LogicLibAreaWr
//! This function writes the 8-bit signed data value (SINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QB42.
//! CHAR data = -1;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaChar(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaChar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const CHAR data);


//! @ingroup LogicLibAreaWr
//! This function writes the 8-bit unsigned data value (BYTE, USINT, @ref MlpiLogicType) to the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QB42.
//! UCHAR data = 1;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaUchar(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaUchar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const UCHAR data);


//! @ingroup LogicLibAreaWr
//! This function writes the 16-bit signed data value (INT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QW42.
//! SHORT data = -18;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaShort(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaShort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const SHORT data);


//! @ingroup LogicLibAreaWr
//! This function writes the 16-bit unsigned data value (WORD, UINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QW42.
//! USHORT data = 18;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaUshort(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaUshort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const USHORT data);


//! @ingroup LogicLibAreaWr
//! This function writes the 32-bit signed data value (DINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QD42.
//! LONG data = -42;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaLong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaLong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LONG data);


//! @ingroup LogicLibAreaWr
//! This function writes the 32-bit unsigned data value (DWORD, UDINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QD42.
//! ULONG data = 42;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaUlong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaUlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULONG data);


//! @ingroup LogicLibAreaWr
//! This function writes the 64-bit signed data value (LINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QL42.
//! LLONG data = -13;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaLlong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaLlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LLONG data);


//! @ingroup LogicLibAreaWr
//! This function writes the 64-bit unsigned data value (LWORD, ULINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QL42.
//! ULLONG data = 13;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaUllong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaUllong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULLONG data);


//! @ingroup LogicLibAreaWr
//! This function writes the 32-bit floating point data value (single precision, REAL, @ref MlpiLogicType) to the
//! memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QD42.
//! FLOAT data = 1.234;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaFloat(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaFloat(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const FLOAT data);


//! @ingroup LogicLibAreaWr
//! This function writes the 64-bit floating point data value (double precision, LREAL, @ref MlpiLogicType) to
//! the memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Variable which contains the value that should be written.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data value to the memory area %QL42.
//! DOUBLE data = 1.23456789;
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaDouble(connection, application, MLPI_MEMORY_AREA_OUTPUT, 42, data);
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaDouble(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const DOUBLE data);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 8-bit signed data values (SINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of CHAR elements in 'data*' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IB0 to %IB1.
//! CHAR data[] = {-1, 2};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayChar(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayChar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const CHAR* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 8-bit unsigned data values (BYTE, USINT, @ref MlpiLogicType) to the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of UCHAR elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IB0 to %IB1.
//! UCHAR data[] = {1, 85};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayUchar(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayUchar(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const UCHAR* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 16-bit signed data values (INT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of SHORT elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IW0 to %IW2.
//! SHORT data[] = {-18, 65};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayShort(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayShort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const SHORT* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 16-bit unsigned data values (WORD, UINT, @ref MlpiLogicType) to the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of USHORT elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IW0 to %IW2.
//! USHORT data[] = {18, 33};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayUshort(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayUshort(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const USHORT* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 32-bit signed data values (DINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of LONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %ID0 to %ID4.
//! LONG data[] = {-42, 99};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayLong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayLong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LONG* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 32-bit unsigned data values (DWORD, UDINT, @ref MlpiLogicType) to the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of ULONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %ID0 to %ID4.
//! ULONG data[] = {42, 55};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayUlong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayUlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULONG* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 64-bit signed data values (LINT, @ref MlpiLogicType) to the memory area
//! (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of LLONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IL0 to %IL8.
//! LLONG data[] = {-13, 44};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayLlong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayLlong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const LLONG* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 64-bit unsigned data values (LWORD, ULINT, @ref MlpiLogicType) to the memory
//! area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of ULLONG elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IL0 to %IL8.
//! ULLONG data[] = {13, 77};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayUllong(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayUllong(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const ULLONG* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 32-bit floating point data values (single precision, REAL, @ref MlpiLogicType)
//! to the memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of FLOAT elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %ID0 to %ID4.
//! FLOAT data[] = {1.234, 9.907};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayFloat(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayFloat(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const FLOAT* data, const ULONG numElements);


//! @ingroup LogicLibAreaWr
//! This function writes an array of 64-bit floating point data values (double precision, LREAL, @ref MlpiLogicType)
//! to the memory area (@ref MlpiApplicationMemoryArea).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   application        Name of application.
//! @param[in]   area               Memory area of access (@ref MlpiApplicationMemoryArea).
//! @param[in]   byteOffset         Byte offset based on zero (e.g. %QB0).
//! @param[in]   data               Pointer to variable which contains the values that should be written.
//! @param[in]   numElements        Number of DOUBLE elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Writes the data values to the memory area %IL0 to %IL8.
//! DOUBLE data[] = {1.23456789, 9.87654321};
//! WCHAR16 application = L"Application";
//! MLPIRESULT result = mlpiLogicWriteMemoryAreaArrayDouble(connection, application, MLPI_MEMORY_AREA_OUTPUT, 0, data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiLogicWriteMemoryAreaArrayDouble(const MLPIHANDLE connection, const WCHAR16 *application, const MlpiApplicationMemoryArea area, const ULONG byteOffset, const DOUBLE* data, const ULONG numElements);



#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPILOGICLIB_H__
