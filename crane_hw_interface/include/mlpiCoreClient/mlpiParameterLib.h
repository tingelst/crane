#ifndef __MLPIPARAMETERLIB_H__
#define __MLPIPARAMETERLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiParameterLib.h>
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


//! @addtogroup ParameterLib ParameterLib
//! @{
//! @brief This library contains functions which can be used to read and write parameters of
//! control or sercos objects.
//!
//! @anchor descriptionParam
//! A parameter is a quantity that serves to relate functions and variables.
//!
//! @note During the execution of a command for loading basic parameters (e.g. via parameter C-0-1000), all
//!       regarding parameter values will be set to their initial values. Considering that IndraWorks use
//!       some parameters for global identification etc., you have to save and restore some parameter values
//!       therewith you are able to reconnect with Indraworks without another project download.
//! <TABLE>
//! <TR><TH> Control parameter (C)      </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="3"> C-0-1000       </TD><TD> C-0-0030, Control name                      </TD></TR>
//! <TR>                                     <TD> C-0-0033, Project identification number     </TD></TR>
//! <TR>                                     <TD> C-0-0203, Engineering, user name            </TD></TR>
//! <TR><TH> Axis parameter (A)         </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="4"> A-0-1000       </TD><TD> A-0-0002, Axis name                         </TD></TR>
//! <TR>                                     <TD> A-0-0016, Project identification number     </TD></TR>
//! <TR>                                     <TD> A-0-0008, Link axis, master axis selection  </TD></TR>
//! <TR>                                     <TD> A-0-0630, Controller axis, controller name  </TD></TR>
//! <TR><TH> Kinematic parameter (K)    </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="5"> K-0-1000       </TD><TD> K-0-0002, Kinematic name                    </TD></TR>
//! <TR>                                     <TD> K-0-0008, Axis configuration list           </TD></TR>
//! <TR>                                     <TD> K-0-0016, Project identification number     </TD></TR>
//! <TR>                                     <TD> K-0-0031, Transformation scheme             </TD></TR>
//! <TR>                                     <TD> K-0-0121 to K-0-0136, Coordinate names      </TD></TR>
//! <TR><TH> Touch probe parameter (M)  </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="1"> M-0-1000       </TD><TD> non                                         </TD></TR>
//! <TR><TH> PLS parameter (N)          </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="1"> N-0-1000       </TD><TD> non                                         </TD></TR>
//! <TR><TH> Oscilloscope parameter (O) </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="1"> O-0-1000       </TD><TD> non                                         </TD></TR>
//! </TABLE>
//!
//! Every parameter consists of up to seven elements. In a parameter, elements 1, 3
//! and 7 are mandatory and shall always be present. Elements 2, 4, 5, and 6 are
//! optional and may be supported depending on configuration. Elements 5 and 6
//! are mandatory for cycle time parameters only.
//!
//! All parameters are assigned to IDNs.
//!
//! @note The defines and functions within 'util/mlpiParameterHelper.h' supports you on parrameter access.
//!
//! @anchor paramElements
//! <BR><B>Parameter structure</B>
//! <TABLE>
//! <TR><TH> Element No. </TH><TH> Description                               </TH><TH> Requirement                                                                               </TH></TR>
//! <TR><TD> 1           </TD><TD> IDN                                       </TD><TD> global unique identifier, mandatory                                                       </TD></TR>
//! <TR><TD> 2           </TD><TD> Name                                      </TD><TD> name of parameter, optional                                                               </TD></TR>
//! <TR><TD> 3           </TD><TD> Attribute                                 </TD><TD> properties of parameter (@ref attribute), changeable, mandatory                           </TD></TR>
//! <TR><TD> 4           </TD><TD> Unit                                      </TD><TD> unit of parameter, changeable, optional                                                   </TD></TR>
//! <TR><TD> 5           </TD><TD> Minimum input value                       </TD><TD> minimum value of operation data, changeable, optional                                     </TD></TR>
//! <TR><TD> 6           </TD><TD> Maximum input value                       </TD><TD> maximum value of operation data, changeable, optional                                     </TD></TR>
//! <TR><TD> 7           </TD><TD> Operation data                            </TD><TD> operation data, changeable, mandatory                                                     </TD></TR>
//! </TABLE>
//!
//! <BR><B>Parameter element 'IDN'</B>
//! <TABLE>
//! <TR><TH> Bit No.     </TH><TH> Short description                         </TH><TH> Description                                                                               </TH></TR>
//! <TR><TD> 40-63       </TD><TD> Reserved                                  </TD><TD> --                                                                                        </TD></TR>
//! <TR><TD> 32-39       </TD><TD> EIDN types                                </TD><TD> Extend standard or product specific IDN (e.g. C-x-xxxx, A-x-xxxx, ...)                    </TD></TR>
//! <TR><TD> 31-24       </TD><TD> Structure instance (SI)                   </TD><TD> Number of structure instance (SI)                                                         </TD></TR>
//! <TR><TD> 23-16       </TD><TD> Structure element (SE)                    </TD><TD> 0-127: Standard SE <BR> 128-255: Product-specific SE                                      </TD></TR>
//! <TR><TD>    15       </TD><TD> Standard or Product specific IDN (S or P) </TD><TD> 0: Standard IDN (S-x-xxxx) <BR> 1: Product-specific IDN (P-x-xxxx)                        </TD></TR>
//! <TR><TD> 14-12       </TD><TD> Parameter sets                            </TD><TD> sercos specifies IDNs with parameter set 0 only.                                          </TD></TR>
//! <TR><TD>  0-11       </TD><TD> Data block or Function group              </TD><TD> Data block number (if SI = SE = 0) <BR> Function group (if SI or SE is not 0)             </TD></TR>
//! </TABLE>
//!
//! @anchor attribute
//! <BR><B>Parameter element 'Attribute'</B>
//! <TABLE>
//! <TR><TH> Bit No.     </TH><TH> Short description                          </TH><TH> Description                                                                              </TH></TR>
//! <TR><TD>    31       </TD><TD> --                                         </TD><TD> Reserved                                                                                 </TD></TR>
//! <TR><TD>    30       </TD><TD> Write protected in CP4                     </TD><TD> 0: Operation data is writable <BR> 1: Operation data is write protected                  </TD></TR>
//! <TR><TD>    29       </TD><TD> Write protected in CP3                     </TD><TD> 0: Operation data is writable <BR> 1: Operation data is write protected                  </TD></TR>
//! <TR><TD>    28       </TD><TD> Write protected in CP2                     </TD><TD> 0: Operation data is writable <BR> 1: Operation data is write protected                  </TD></TR>
//! <TR><TD> 24-27       </TD><TD> Decimal point                              </TD><TD> 0000..1111: No place to 15 places after decimal point (maximum)                          </TD></TR>
//! <TR><TD>    23       </TD><TD> --                                         </TD><TD> Reserved                                                                                 </TD></TR>
//! <TR><TD> 20-22       </TD><TD> Data type and display format               </TD><TD> 000: Data type: Binary number;          Display format: Binary                           <BR>
//!                                                                                     001: Data type: Unsigned integer;       Display format: Unsigned decimal                 <BR>
//!                                                                                     010: Data type: Integer;                Display format: Signed decimal                   <BR>
//!                                                                                     011: Data type: Unsigned integer;       Display format: Hexadecimal                      <BR>
//!                                                                                     100: Data type: Extended character set; Display format: UTF8                             <BR>
//!                                                                                     101: Data type: Unsigned integer;       Display format: IDN                              <BR>
//!                                                                                     110: Data type: Floating-point number; Display format: Signed decimal with exponent      <BR>
//!                                                                                     111: Data type: sercos time; Display format: 4 octets seconds and 4 octets nano seconds,
//!                                                                                                     starts with 1.1.1970 computed in UTC                                     </TD></TR>
//! <TR><TD>    19       </TD><TD> Command                                    </TD><TD> 0: Parameter is not a procedure command <BR> 1: Parameter is a procedure command         </TD></TR>
//! <TR><TD> 16-18       </TD><TD> Data length                                </TD><TD> 000: Reserved                                                                            <BR>
//!                                                                                     001: Operation data is two octets long                                                   <BR>
//!                                                                                     010: Operation data is four octets long                                                  <BR>
//!                                                                                     011: Operation data is eight octets long                                                 <BR>
//!                                                                                     100: Variable length with one-octet data strings                                         <BR>
//!                                                                                     101: Variable length with two-octet data strings                                         <BR>
//!                                                                                     110: Variable length with four-octet data strings                                        <BR>
//!                                                                                     111: Variable length with eight-octet data strings                                       </TD></TR>
//! <TR><TD>  0-15       </TD><TD> Conversion factor                          </TD><TD> The conversion factor is an unsigned integer used to convert numeric data to display
//!                                                                                     format. The conversion factor shall be set to a value of 1 when it is not needed for
//!                                                                                     data display.                                                                            </TD></TR>
//! </TABLE>
//!
//! @note The ParameterLib functions trace their debug information mainly into the module MLPI_PARAMETER_LIB
//!       and in addition into the modules APS_EXP_PARSING, APS_EXP_PROCESSING, APS_PARAM_HANDLER and
//!       MLPI_BASE_MODULES. For further information, see also the detailed description of the library
//!       @ref TraceLib and the notes about @ref sec_TraceViewer.
//!
//! @anchor sample
//! <BR><B>Sample</B>
//! <BR>This sample provides a step by step instruction on how to proceed if there is only the parameter number (e.g. "A-0-0100.0.0") given and the data of it is required.
//! <BR>First of all the data type of the parameter needs to be identified. Therefor you can look into the documentation of the parametersystem, e.g. the IndraWorks help.
//! For the parameter A-0-0100.0.0 it is FLOAT.
//! If you have no access to this kind of information, you can use @ref utilParameterGetDataTypeFromAttribute from the @ref UtilParameterHelper to determine the datatype.
//! In this case you can continue like it is done in the example of the function. For further knowledgement you should check the sample 'Using the Parameterlib'.
//! Another possibility it to use the universal function @ref mlpiParameterReadDataString, which does not require a data type, but therefore gives the data in a WCHAR16 string.
//! <BR>After that the proper function for the data type needs to be picked. In this case it is @ref mlpiParameterReadDataFloat.
//! <BR>The function needs the MLPIHANDLE as first argument.
//! As second argument you can use the @ref MLPI_ADDRESS_x macro to create the 64-bit address identifier. The macro is used like MLPI_ADDRESS(MLPI_ADDRESS_MODE_LOGICAL, masterAddress, slaveAddress) in this case.
//! For the third argument one of the macros from @ref MLPI_SIDN_x can be used to determine the SIDN of the parameter. In this case it is @ref MLPI_SIDN_A(100).
//! <BR>At the end the function call should look like this:
//! @code
//! FLOAT data = 0.0;
//! ULLONG masterAddress = 0;
//! ULLONG slaveAddress = 1;
//! MLPIRESULT result = mlpiParameterReadDataFloat(connection, MLPI_ADDRESS(MLPI_ADDRESS_MODE_LOGICAL, masterAddress, slaveAddress), MLPI_SIDN_A(100), &data);
//! @endcode
//! @}

//! @addtogroup ParamLibNameAttribUnit Name, Attribute, Unit
//! @ingroup ParameterLib
//! @{
//! @brief These functions read the name, the attribute and the unit of a parameter.
//!
//! @details The Name, the attribute and the unit are parts of a parameter
//! (@ref paramElements "Parameter structure, sercos element 2, 3 and 4").
//!
//! @note The defines and functions within 'util/mlpiParameterHelper.h' supports you to evaluate the sercos attribute.
//! @}

//! @addtogroup ParamLibMinMax Minimum, Maximum
//! @ingroup ParameterLib
//! @{
//! @brief These functions read the minimum and maximum values of a parameter.
//!
//! @details The minimum and maximum values are parts of a parameter
//! (@ref paramElements "Parameter structure, sercos element 5 and 6").
//! @}

//! @addtogroup ParamLibMin Minimum
//! @ingroup ParamLibMinMax
//! @{
//! @brief This function reads the minimum value of a parameter.
//!
//! @details The minimum is part of a parameter (@ref paramElements "Parameter structure, sercos element 5").
//! @}

//! @addtogroup ParamLibMax Maximum
//! @ingroup ParamLibMinMax
//! @{
//! @brief This function reads the maximum value of a parameter.
//!
//! @details The maximum is part of a parameter (@ref paramElements "Parameter structure, sercos element 6").
//! @}

//! @addtogroup ParamLibData Data, Default
//! @ingroup ParameterLib
//! @{
//! @brief These functions read and write to the operation data and read the default value of a parameter.
//!
//! @details The operation data is part of a parameter (@ref paramElements "Parameter structure, sercos element 7").
//! The default value is the initial value of the operation data.
//! <BR>For more information you should check the @ref descriptionParam "Detailed Description".
//! @}

//! @addtogroup ParamLibDataRd Read data value
//! @ingroup ParamLibData
//! @{
//! @brief This function reads the operation data of a parameter.
//!
//! @details The operation data is part of a parameter (@ref paramElements "Parameter structure, sercos element 7").
//! @}

//! @addtogroup ParamLibDataWr Write data value
//! @ingroup ParamLibData
//! @{
//! @brief This function writes the operation data of a parameter.
//!
//! @details The operation data is part of a parameter (@ref paramElements "Parameter structure, sercos element 7").
//! @}

//! @addtogroup ParamLibDefault Read default value
//! @ingroup ParamLibData
//! @{
//! @brief This function reads the default value of a parameter.
//!
//! @details The default value is the initial value of the operation data.
//! @}

//! @addtogroup ParamLibCmd Command, Status
//! @ingroup ParameterLib
//! @{
//! @brief These functions read the data status and handle the execution command.
//!
//! @details
//! @}

//! @addtogroup ParamLibAux Auxiliary function
//! @ingroup ParameterLib
//! @{
//! @brief These functions support import, export and further activities in relation to parameters.
//!
//! @details
//! @}

//! @addtogroup ParameterLibVersionPermission Version and Permission
//! @ingroup ParameterLib
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_PARAMETERLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_PARAMETERLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiParameterLib using in accounts.xml
//! - MLPI_PARAMETERLIB_PERMISSION_ALL
//! - MLPI_PARAMETERLIB_PERMISSION_DATA_INFO
//! - MLPI_PARAMETERLIB_PERMISSION_COMMAND
//! - MLPI_PARAMETERLIB_PERMISSION_PARAMETER_INFO
//! - MLPI_PARAMETERLIB_PERMISSION_DATA_READ
//! - MLPI_PARAMETERLIB_PERMISSION_DATA_WRITE
//! - MLPI_PARAMETERLIB_PERMISSION_DEFAULT_READ
//!
//! <TABLE>
//! <TR><TH>           Function                                           </TH><TH> Server version </TH><TH> Permission                                     </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadName                         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_PARAMETER_INFO"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadAttribute                    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_PARAMETER_INFO"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadUnit                         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_PARAMETER_INFO"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadDataStatus                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterCommand                          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_COMMAND"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadCommandStatus                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_COMMAND"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadListLength                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_PARAMETER_INFO"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadMinimumString                </TD><TD> 1.1.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiParameterReadMinimum...           (10x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadMaximumString                </TD><TD> 1.1.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiParameterReadMaximum...           (10x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiParameterReadData...              (20x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiParameterWriteData...             (20x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_WRITE"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadDefaultString                </TD><TD> 1.1.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DEFAULT_READ"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e">      mlpiParameterReadDefault...           ( 8x)   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DEFAULT_READ"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterImportFile                       </TD><TD> 1.0.13.0       </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_WRITE"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterExportFile                       </TD><TD> 1.0.13.0       </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterReadEverything                   </TD><TD> 1.3.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterExportFileStartProcess           </TD><TD> 1.6.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterImportFileStartProcess           </TD><TD> 1.6.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_WRITE"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterImportExportStatus               </TD><TD> 1.6.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterImportExportGetInfo              </TD><TD> 1.6.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_INFO"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterImportExportAbort                </TD><TD> 1.6.0.0        </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_WRITE"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterWriteAccessSetup                 </TD><TD> 1.14.0.0       </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterWriteAccessStatus                </TD><TD> 1.14.0.0       </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiParameterWriteAccessAbort                 </TD><TD> 1.14.0.0       </TD><TD> "MLPI_PARAMETERLIB_PERMISSION_DATA_READ"       </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
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

//! @addtogroup ParameterLibStructTypes Structs, Types, ...
//! @ingroup ParameterLib
//! @{
//! @brief List of used types, enumerations, structures and more...



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------

#include "mlpiGlobal.h"



// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

// A MLPI Parameter Identification Number (SIDN resp. EIDN) is build like this.
//
// +----------+----------+------+------+------+-----+-----+---------+
// | Instance | reserved | Type |  SI  |  SE  | S/P | Set |  Block  |
// +----------+----------+------+------+------+-----+-----+---------+
// 63         55         39     31     23     15    14    11        0
//
// The instance (address) will be used to assign a instance to parameter
// within a parameter configuration (-list).
//
//

//! This definition represents the valid type of an S parameter ident.
#define MLPI_SIDN_TYPE_DRIVE_S                (0x0000000000ULL)

//! This definition represents the valid type of a P parameter ident.
#define MLPI_SIDN_TYPE_DRIVE_P                (0x0000008000ULL)

//! This definition represents the valid type of an A parameter ident.
#define MLPI_SIDN_TYPE_AXIS                   (0x0100000000ULL)

//! This definition represents the valid type of a C parameter ident.
#define MLPI_SIDN_TYPE_CONTROL                (0x0200000000ULL)

//! This definition represents the valid type of a K parameter ident.
#define MLPI_SIDN_TYPE_KINEMATIC              (0x2B00000000ULL)

//! This definition represents the valid type of an M parameter ident.
#define MLPI_SIDN_TYPE_PROBE                  (0x2D00000000ULL)

//! This definition represents the valid type of an N parameter ident.
#define MLPI_SIDN_TYPE_POSLIMSWITCH           (0x2E00000000ULL)

//! This definition represents the valid type of an O parameter ident.
#define MLPI_SIDN_TYPE_OSCILLOSCOPE           (0x2F00000000ULL)

//! This definition masks the instance address of a parameter ident.
#define MLPI_SIDN_INSTANCE_MASK               (0xFF00000000000000ULL)

//! This definition masks the element type of a parameter ident.
#define MLPI_SIDN_TYPE_MASK                   (0xFF00008000ULL)

//! This definition masks the element set of a parameter ident.
#define MLPI_SIDN_SET_MASK                    (0x00007000)

//! This definition masks the element block of a parameter ident.
#define MLPI_SIDN_BLOCK_MASK                  (0x00000FFF)

//! This definition masks the element SE of a parameter ident.
#define MLPI_SIDN_SE_MASK                     (0x00FF0000)

//! This definition masks the element SI of a parameter ident.
#define MLPI_SIDN_SI_MASK                     (0xFF000000)



// A MLPI address ident is build like this.
//
// +--------------+------+-------------+--------------+
// | reserved     | mode | master adr. |  slave adr.  |
// +--------------+------+-------------+--------------+
// 63             39     31           15              0
//

//! @note In case of using the logical addressing (@ref MLPI_ADDRESS_MODE_LOGICAL) on S and P parameter access all values
//!       will be <b>automatically</b> converted to DOUBLE if decimal places unequal nil. In this case the regarding
//!       attribute will be also adapted to "data type / display format" "floating point number".\n
//!       To switched of this feature please use the natvive logical addressing (@ref MLPI_ADDRESS_MODE_LOGICAL_NATIVE)
//!       on S and P parameter access.

//! Address the object by its logical address given in the lower 4 bytes.
#define MLPI_ADDRESS_MODE_LOGICAL             (0x0000000000000000ULL)

//! Address the object by its physical address(sercos device address) given in the lower 4 bytes.
#define MLPI_ADDRESS_MODE_PHYSICAL            (0x0000000000000001ULL)

//! Address the object by its topological address(position in the ring) given in the lower 4 bytes.
#define MLPI_ADDRESS_MODE_TOPOLOGICAL         (0x0000000000000002ULL)

//! Address the object by its logical address given in the lower 4 bytes, but don't convert
//! value and attribute. Using this address mode within any other parameter access than S or P parameter will be
//! handled like using the logical addressing mode @ref MLPI_ADDRESS_MODE_LOGICAL.
#define MLPI_ADDRESS_MODE_LOGICAL_NATIVE      (0x0000000000000003ULL)


//! This definition masks the slave address of an address ident.
#define MLPI_ADDRESS_SLAVE_MASK               (0x000000000000FFFFULL)

//! This definition masks the master address of an address ident.
#define MLPI_ADDRESS_MASTER_MASK              (0x00000000FFFF0000ULL)

//! This definition masks the address mode of an address ident.
#define MLPI_ADDRESS_MODE_MASK                (0x000000FF00000000ULL)




// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------

//! @anchor MLPI_SIDN_x

//! This macro creates the 64-bit value of an EIDN like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! using all elements of parameter ident number.
//! @param   type    Type of parameter such as A, C, ... .
//!                  Use definitions MLPI_SIDN_TYPE_AXIS, MLPI_SIDN_TYPE_CONTROL, ... to set it.
//! @param   set     Set of parameter (value: 0..7).
//! @param   block   Block of parameter (value: 0..4095).
//! @param   si      SI
//! @param   se      SE
//! @return          The 64-bit SIDN.
#define MLPI_SIDN(type, set, block, si, se) ( (ULLONG)  ( \
  (  ( ((ULLONG) type )      ) & MLPI_SIDN_TYPE_MASK  ) | \
  (  ( ((ULLONG) set  ) << 12) & MLPI_SIDN_SET_MASK   ) | \
  (  ( ((ULLONG) block)      ) & MLPI_SIDN_BLOCK_MASK ) | \
  (  ( ((ULLONG) si   ) << 24) & MLPI_SIDN_SI_MASK    ) | \
  (  ( ((ULLONG) se   ) << 16) & MLPI_SIDN_SE_MASK    ) ) )

//! This easy-to-use macro creates the 64-bit value of an A parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_A(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_AXIS, 0, block, 0, 0))

//! This easy-to-use macro creates the 64-bit value of an C parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_C(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_CONTROL, 0, block,0, 0))

//! This easy-to-use macro creates the 64-bit value of a K parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_K(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_KINEMATIC, 0, block, 0, 0))

//! This easy-to-use macro creates the 64-bit value of an M parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_M(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_PROBE, 0, block, 0, 0))

//! This easy-to-use macro creates the 64-bit value of an N parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_N(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_POSLIMSWITCH, 0, block, 0, 0))

//! This easy-to-use macro creates the 64-bit value of an O parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_O(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_OSCILLOSCOPE, 0, block, 0, 0))

//! This easy-to-use macro creates the 64-bit value of an S parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_S(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_DRIVE_S, 0, block, 0, 0))

//! This easy-to-use macro creates the 64-bit value of a P parameter EIDN
//! like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@> using the block value.
//! @param   block   Block of parameter (value: 0..4095).
//! @return          The 64-bit SIDN.
#define MLPI_SIDN_P(block)  (MLPI_SIDN(MLPI_SIDN_TYPE_DRIVE_P, 0, block, 0, 0))

//! This macro splits an EIDN address like @<INSTANCE>:@<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! @param   sidn    64-bit parameter ident number
//! @return          The 8-bit instance address.
#define MLPI_SIDN_INST(sidn)    ( (UCHAR)  ((sidn & MLPI_SIDN_INSTANCE_MASK ) >> 56 ) )

//! This macro splits an EIDN type like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! @param   sidn    64-bit parameter ident number
//! @return          The 64-bit type.
#define MLPI_SIDN_TYPE(sidn)    ( (ULLONG) ((sidn & MLPI_SIDN_TYPE_MASK )       ) )

//! This macro splits an EIDN set like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! @param   sidn    64-bit parameter ident number
//! @return          The 8-bit set value.
#define MLPI_SIDN_SET(sidn)     ( (UCHAR)  ((sidn & MLPI_SIDN_SET_MASK  ) >> 12 ) )

//! This macro splits an EIDN block like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! @param   sidn    64-bit parameter ident number
//! @return          The 16-bit block value.
#define MLPI_SIDN_BLOCK(sidn)   ( (USHORT) ((sidn & MLPI_SIDN_BLOCK_MASK)       ) )

//! This macro splits an EIDN SI like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! @param   sidn    64-bit parameter ident number
//! @return          The 8-bit SI value.
#define MLPI_SIDN_SI(sidn)      ( (UCHAR)  ((sidn & MLPI_SIDN_SI_MASK   ) >> 24 ) )

//! This macro splits an EIDN SE like @<TYPE-SET-BLOCK@>.@<SI@>.@<SE@>
//! @param   sidn    64-bit parameter ident number
//! @return          The 8-bit SE value.
#define MLPI_SIDN_SE(sidn)      ( (UCHAR)  ((sidn & MLPI_SIDN_SE_MASK   ) >> 16 ) )



//! @anchor MLPI_ADDRESS_x

//! This macro creates the 64-bit address identifier
//! @param   mode    Type of address (logical, physical, topological). The slave element of this address identifier
//!                  is interpreted based on this selection.
//!                  Use the definitions @ref MLPI_ADDRESS_MODE_LOGICAL, @ref MLPI_ADDRESS_MODE_PHYSICAL,
//!                  @ref MLPI_ADDRESS_MODE_TOPOLOGICAL or @ref MLPI_ADDRESS_MODE_LOGICAL_NATIVE to set it.
//! @param   master  Master address.
//! @param   slave   Slave address.
//! @return          The 64-bit address identifier.
#define MLPI_ADDRESS(mode, master, slave) ( (ULLONG)            ( \
  (  ( ((ULLONG) mode   ) << 32 ) & MLPI_ADDRESS_MODE_MASK    ) | \
  (  ( ((ULLONG) master ) << 16 ) & MLPI_ADDRESS_MASTER_MASK  ) | \
  (  ( ((ULLONG) slave  )       ) & MLPI_ADDRESS_SLAVE_MASK   ) ) )


#define MLPI_PARAMETER_MASTER_ADDRESS(x)      ( (USHORT)((x>>16) & 0xFFFF) )
#define MLPI_PARAMETER_SLAVE_ADDRESS(x)       ( (USHORT) (x & 0xFFFF) )
#define MLPI_PARAMETER_ADDRESS_MODE(x)        ( (UCHAR) (((ULLONG)x>>32) & 0x0F) )



//! @anchor Defines of sercos specification

//! This definition represents the maximum number of characters of the sercos parameter element 'name'.
#define MLPI_PARAMETER_EIDN_MAX_LENGTH_OF_NAME  (   60)

//! This definition represents the maximum number of characters of the sercos parameter element 'unit'.
#define MLPI_PARAMETER_EIDN_MAX_LENGTH_OF_UNIT  (   12)

//! This definition represents the maximum number of bytes of the sercos parameter element 'data'.
#define MLPI_PARAMETER_EIDN_MAX_LENGTH_OF_DATA  (65532)



// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! @enum MlpiParameterProcessType
//! This enumeration defines the type of the parameter process
typedef enum MlpiParameterProcessType
{
  MLPI_PARAMETER_PROCESSTYPE_IMPORT        =  0,  //!< Import
  MLPI_PARAMETER_PROCESSTYPE_EXPORT        =  1,  //!< Export
}MlpiParamImExportStatus;

//! @enum MlpiParameterCommandStatus
//! This enumeration defines the status of a command execution using @ref mlpiParameterReadCommandStatus.
typedef enum MlpiParameterCommandStatus
{
  MLPI_PARAMETER_COMMAND_INACTIVE           =   0,  //!< Procedure command has been canceled and is inactive.
  MLPI_PARAMETER_COMMAND_SET                =   1,  //!< Procedure command is set.
  MLPI_PARAMETER_COMMAND_EXECUTED_CORRECTLY =   3,  //!< Procedure command has been executed correctly.
  MLPI_PARAMETER_COMMAND_INTERRUPTED        =   5,  //!< Procedure command execution is interrupted.
  MLPI_PARAMETER_COMMAND_IN_PROGRESS        =   7,  //!< Procedure command is activated but not yet executed.
  MLPI_PARAMETER_COMMAND_ERROR              =  15,  //!< Error, procedure command execution impossible.
  MLPI_PARAMETER_COMMAND_STATUS_INVALID     = 256   //!< Error, command status invalid.
}MlpiParameterCommandStatus;

// message packing follows 8-byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiSidnError
//! @brief This structure defines the information returned in case an error occurred during parameter import or export.
//! @details Elements of struct MlpiSidnError
//! <TABLE>
//! <TR><TH>                Type      </TH><TH>           Element         </TH><TH> Description                                                   </TH></TR>
//! <TR><TD id="st_t">      ULLONG    </TD><TD id="st_e"> parameterNumber </TD><TD> SIDN of parameter which has an error during import or export. </TD></TR>
//! <TR><TD id="st_t">      ULONG     </TD><TD id="st_e"> errorCode       </TD><TD> Error code of occurred error.                                 </TD></TR>
//! </TABLE>
typedef struct MlpiSidnError
{
  ULLONG  parameterNumber;      //!< SIDN of parameter which has an error during import or export.
  ULONG   errorCode;            //!< Error code of occurred error.
  ULONG   reserved;
}MlpiSidnError;

//! @typedef MlpiReadEverything
//! @brief This structure defines the information returned using @ref mlpiParameterReadEverything.
//! @details Elements of struct MlpiReadEverything
//! <TABLE>
//! <TR><TH>           Type       </TH><TH> Direction </TH><TH>           Element       </TH><TH> Description                                                                                                                                                               </TH></TR>
//! <TR><TD id="st_t"> ULLONG     </TD><TD> [in]      </TD><TD id="st_e"> address       </TD><TD> Address identifying the object to access. Use macro @ref MLPI_ADDRESS_x to generate an address field.                                                                     </TD></TR>
//! <TR><TD id="st_t"> ULLONG     </TD><TD> [in]      </TD><TD id="st_e"> sidn          </TD><TD> ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.                                                                                         </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> validElements </TD><TD> Bitmask of elements, which are valid in this response invalid length elements are set to nil.                                                                             </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> dataStatus    </TD><TD> Data status of the parameter (@ref paramElements "Parameter structure, sercos element 1").                                                                                </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> nameOffset    </TD><TD> Offset in the buffer where the name of the parameter stands. The name is an null-terminated WCHAR16 string (@ref paramElements "Parameter structure, sercos element 2").  </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> attribute     </TD><TD> Attribute of the parameter (@ref paramElements "Parameter structure, sercos element 3").                                                                                  </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> unitOffset    </TD><TD> Offset in the buffer where the unit of the parameter stands. The unit is an null-terminated WCHAR16 string  (@ref paramElements "Parameter structure, sercos element 4"). </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> minOffset     </TD><TD> Offset in the buffer where the minimum value of the parameter stands. (@ref paramElements "Parameter structure, sercos element 5").                                       </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> maxOffset     </TD><TD> Offset in the buffer where the maximum value of the parameter stands. (@ref paramElements "Parameter structure, sercos element 6").                                       </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> dataLength    </TD><TD> Length of minimum, maximum and data in octets. Length of one list element if parameter is a list.                                                                         </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> dataOffset    </TD><TD> Offset in the buffer where the data value of the parameter stands (@ref paramElements "Parameter structure, sercos element 7")                                            </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> dataSize      </TD><TD> Length of data in octets.                                                                                                                                                 </TD></TR>
//! <TR><TD id="st_t"> ULONG      </TD><TD> [out]     </TD><TD id="st_e"> maxDataSize   </TD><TD> Maximum length of data in octets. Valid if parameter is a list.                                                                                                           </TD></TR>
//! <TR><TD id="st_t"> MLPIRESULT </TD><TD> [out]     </TD><TD id="st_e"> result        </TD><TD> Return value indicating success (>=0) or error (<0).                                                                                                                      </TD></TR>
//! </TABLE>
typedef struct MlpiReadEverything
{
  // Input Parameters
  ULLONG  address;              //!<  Address identifying the object to access. Use macro @ref MLPI_ADDRESS_x to generate an address field.
  ULLONG  sidn;                 //!<  ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.

  // Output Parameters
  ULONG  validElements;         //!<  bitmask of the following elements, which are valid in this response invalid length elements are set to 0
                                //!<  bitmask | description
                                //!-----------------------------------------------------------------------------------------------------------
  ULONG  dataStatus;            //!<  0x01    | Data status of the parameter
  ULONG  nameOffset;            //!<  0x02    | Offset in the buffer where the name of the parameter stands. The name is a null-terminated WCHAR16 string
  ULONG  attribute;             //!<  0x04    | Attribute of the parameter
  ULONG  unitOffset;            //!<  0x08    | Offset in the buffer where the unit of the parameter stands. The unit is a null-terminated WCHAR16 string
  ULONG  minOffset;             //!<  0x10    | Offset in the buffer where the minimum value of the parameter stands
  ULONG  maxOffset;             //!<  0x20    | Offset in the buffer where the maximum value of the parameter stands
  ULONG  dataLength;            //!<  ----    | Length of minimum, maximum and data in octets. Length of one list element if parameter is a list
  ULONG  dataOffset;            //!<  0x40    | Offset in the buffer where the data value of the parameter stands
  ULONG  dataSize;              //!<  0x40    | Length of data in octets
  ULONG  maxDataSize;           //!<  ----    | Maximum length of data in octets. Valid if parameter is a list

  MLPIRESULT result;            //!< Return value indicating success (>=0) or error (<0).
}MlpiReadEverything;

//! @typedef MlpiParamProcessStatus
//! @brief This structure defines the information about a parameter process status
//! @details Elements of struct MlpiParamProcessStatus
//! <TABLE>
//! <TR><TH>                Type                </TH><TH>           Element         </TH><TH> Description                                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiParameterProcessType </TD><TD id="st_e"> processType     </TD><TD> Type of parameter process (import or export).                 </TD></TR>
//! <TR><TD id="st_t"> MlpiProcessState         </TD><TD id="st_e"> processStatus   </TD><TD> Status of process.                                            </TD></TR>
//! <TR><TD id="st_t"> MLPIRESULT               </TD><TD id="st_e"> processResult   </TD><TD> Return value of asynchron function.                           </TD></TR>
//! </TABLE>
typedef struct MlpiParamProcessStatus
{
  MlpiParameterProcessType   processType;   //!< Type of parameter process (import or export).
  MlpiProcessState           processState;  //!< State of process.
  MLPIRESULT                 processResult; //!< Return value of asynchron function.
}MlpiParamProcessStatus;

//! @typedef MlpiParamWriteAccess
//! @brief This structure defines the information of an parameter used by function @ref mlpiParameterWriteAccessSetup and @ref mlpiParameterWriteAccessStatus
//! @details Elements of struct MlpiParamWriteAccess
//! <TABLE>
//! <TR><TH>            Type   </TH><TH>           Element  </TH><TH> Description                                                                                           </TH></TR>
//! <TR><TD id="st_t"> ULLONG  </TD><TD id="st_e"> address  </TD><TD> Address identifying the object to access. Use macro @ref MLPI_ADDRESS_x to generate an address field. </TD></TR>
//! <TR><TD id="st_t"> ULLONG  </TD><TD id="st_e"> sidn     </TD><TD> ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.                     </TD></TR>
//! </TABLE>
typedef struct MlpiParamWriteAccess
{
  ULLONG address;  //!< Address identifying the object to access. Use macro @ref MLPI_ADDRESS_x to generate an address field.
  ULLONG sidn;     //!< ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
}MlpiParamWriteAccess;


#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup ParameterLibStructTypes


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

//! @ingroup ParamLibNameAttribUnit
//! This function reads the name of a parameter (@ref paramElements "Parameter structure, sercos element 2").
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field. This is the
//!                                 sercos address.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  name               String where the name will be stored.
//! @param[in]   numElements        Number of WCHAR16 elements in 'name' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the name of parameter 'C-0-0012'.
//! WCHAR16 name[512] = L"";
//! MLPIRESULT result = mlpiParameterReadName(connection, 0, MLPI_SIDN_C(12), name, _countof(name));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadName(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, WCHAR16 *name, const ULONG numElements);


//! @ingroup ParamLibNameAttribUnit
//! This function reads the @ref attribute of a parameter (@ref paramElements "Parameter structure, sercos element 3").
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to access. Use macro @ref MLPI_ADDRESS_x to generate an address field. This is the
//!                                 sercos address.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  attribute          Pointer to variable where the attribute will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the attribute of parameter 'C-0-0023'.
//! ULONG attribute = 0;
//! MLPIRESULT result = mlpiParameterReadAttribute(connection, 0, MLPI_SIDN_C(23), &attribute);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadAttribute(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *attribute);


//! @ingroup ParamLibNameAttribUnit
//! This function reads the unit of a parameter (@ref paramElements "d structure, sercos element 4").
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field. This is the
//!                                 sercos address.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  unit               String where the unit will be stored.
//! @param[in]   numElements        Number of WCHAR16 elements in 'unit' available to read.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the unit of parameter 'C-0-0051'.
//! WCHAR16 unit[16] = L"";
//! MLPIRESULT result = mlpiParameterReadUnit(connection, 0, MLPI_SIDN_C(51), unit, _countof(unit));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadUnit(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, WCHAR16 *unit, const ULONG numElements);


//! @ingroup ParamLibMin
//! This function reads the 8-bit signed minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'C-0-0081'.
//! CHAR data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumChar(connection, 1, MLPI_SIDN_C(81), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumChar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, CHAR *data);


//! @ingroup ParamLibMin
//! This function reads the 8-bit unsigned minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'C-0-0081'.
//! UCHAR data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumUchar(connection, 1, MLPI_SIDN_C(81), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumUchar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, UCHAR *data);


//! @ingroup ParamLibMin
//! This function reads the 16-bit signed minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'A-0-0058' of axis '1'.
//! SHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumShort(connection, 1, MLPI_SIDN_A(58), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, SHORT *data);


//! @ingroup ParamLibMin
//! This function reads the 16-bit unsigned minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'C-0-0450'.
//! USHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumUshort(connection, 0, MLPI_SIDN_C(450), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, USHORT *data);


//! @ingroup ParamLibMin
//! This function reads the 32-bit signed minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'M-0-0201' of touch probe '1'.
//! LONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumLong(connection, 1, MLPI_SIDN_M(201), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LONG *data);


//! @ingroup ParamLibMin
//! This function reads the 32-bit unsigned minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'C-0-0400'.
//! ULONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumUlong(connection, 0, MLPI_SIDN_C(400), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *data);


//! @ingroup ParamLibMin
//! This function reads the 64-bit signed minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'M-0-0141' of touch probe '1'.
//! // Note: The result should be an error because this parameter has no minimum value.
//! LLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumLlong(connection, 1, MLPI_SIDN_M(141), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LLONG *data);


//! @ingroup ParamLibMin
//! This function reads the 64-bit unsigned minimum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'M-0-0141' of touch probe '1'.
//! // Note: The result should be an error because this parameter has no minimum value.
//! ULLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumUllong(connection, 1, MLPI_SIDN_M(141), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULLONG *data);


//! @ingroup ParamLibMin
//! This function reads the 32-bit floating point minimum value (single precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to access. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'C-0-0418'.
//! FLOAT data = 0.0;
//! MLPIRESULT result = mlpiParameterReadMinimumFloat(connection, 0, MLPI_SIDN_C(418), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, FLOAT *data);


//! @ingroup ParamLibMin
//! This function reads the 64-bit floating point minimum value (double precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'A-0-0045' of axis '1'.
//! DOUBLE data = 0.0;
//! MLPIRESULT result = mlpiParameterReadMinimumDouble(connection, 1, MLPI_SIDN_A(45), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, DOUBLE *data);


//! @ingroup ParamLibMin
//! This function reads the minimum value of a parameter and returns it as String. A conversion is done automatically
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the minimum value will be stored in UTF16 format (string).
//! @param[in]   numElements        Number of WCHAR16 elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of WCHAR16 elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the minimum value of parameter 'A-0-0045' of axis '1'.
//! WCHAR16 data[512] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiParameterReadMinimumString(connection, 1, MLPI_SIDN_A(45), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMinimumString(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, WCHAR16 *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibMax
//! This function reads the 8-bit signed maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'C-0-0081'.
//! CHAR data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumChar(connection, 1, MLPI_SIDN_A(58), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumChar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, CHAR *data);


//! @ingroup ParamLibMax
//! This function reads the 8-bit unsigned maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'C-0-0081'.
//! UCHAR data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumUchar(connection, 1, MLPI_SIDN_A(58), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumUchar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, UCHAR *data);


//! @ingroup ParamLibMax
//! This function reads the 16-bit signed maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'A-0-0058' of axis '1'.
//! SHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumShort(connection, 1, MLPI_SIDN_A(58), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, SHORT *data);


//! @ingroup ParamLibMax
//! This function reads the 16-bit unsigned maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'C-0-0450'.
//! USHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumUshort(connection, 0, MLPI_SIDN_C(450), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, USHORT *data);


//! @ingroup ParamLibMax
//! This function reads the 32-bit signed maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'M-0-0201' of touch probe '1'.
//! LONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumLong(connection, 1, MLPI_SIDN_M(201), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LONG *data);


//! @ingroup ParamLibMax
//! This function reads the 32-bit unsigned maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'C-0-0400'.
//! ULONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumUlong(connection, 0, MLPI_SIDN_C(400), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *data);


//! @ingroup ParamLibMax
//! This function reads the 64-bit signed maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'M-0-0141' of touch probe '1'.
//! // Note: The result should be an error because this parameter has no maximum value.
//! LLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumLlong(connection, 1, MLPI_SIDN_M(141), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LLONG *data);


//! @ingroup ParamLibMax
//! This function reads the 64-bit unsigned maximum value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'M-0-0141' of touch probe '1'.
//! // Note: The result should be an error because this parameter has no maximum value.
//! ULLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumUllong(connection, 1, MLPI_SIDN_M(141), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULLONG *data);


//! @ingroup ParamLibMax
//! This function reads the 32-bit floating point maximum value (single precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'C-0-0418'.
//! FLOAT data = 0.0;
//! MLPIRESULT result = mlpiParameterReadMaximumFloat(connection, 0, MLPI_SIDN_C(418), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, FLOAT *data);


//! @ingroup ParamLibMax
//! This function reads the 64-bit floating point maximum value (double precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'A-0-0045' of axis '1'.
//! DOUBLE data = 0.0;
//! MLPIRESULT result = mlpiParameterReadMaximumDouble(connection, 1, MLPI_SIDN_A(45), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, DOUBLE *data);


//! @ingroup ParamLibMax
//! This function reads the maximum value of a parameter and returns it as UTF16 String. A conversion is done automatically.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the maximum value will be stored in UTF16 format (string).
//! @param[in]   numElements        Number of WCHAR16 elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of WCHAR16 elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the maximum value of parameter 'A-0-0045' of axis '1'.
//! WCHAR16 data[512] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiParameterReadMaximumString(connection, 1, MLPI_SIDN_A(45), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadMaximumString(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, WCHAR16 *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads the 16-bit signed data value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'A-0-0058' of axis '1'.
//! SHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadDataShort(connection, 1, MLPI_SIDN_A(58), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, SHORT *data);


//! @ingroup ParamLibDataRd
//! This function reads the 16-bit unsigned data value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'C-0-0450'.
//! USHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadDataUshort(connection, 0, MLPI_SIDN_C(450), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, USHORT *data);


//! @ingroup ParamLibDataRd
//! This function reads the 32-bit signed data value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'M-0-0200' of touch probe '1'.
//! LONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDataLong(connection, 1, MLPI_SIDN_M(200), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LONG *data);


//! @ingroup ParamLibDataRd
//! This function reads the 32-bit unsigned data value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'C-0-0400'.
//! ULONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDataUlong(connection, 0, MLPI_SIDN_C(400), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *data);


//! @ingroup ParamLibDataRd
//! This function reads the 64-bit signed data value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'M-0-0140' of touch probe '1'.
//! // Note: The return value could be misinterpreted because this parameter has an unsigned data value.
//! LLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDataLlong(connection, 1, MLPI_SIDN_M(140), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LLONG *data);


//! @ingroup ParamLibDataRd
//! This function reads the 64-bit unsigned data value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'M-0-0140' of touch probe '1'.
//! ULLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDataUllong(connection, 1, MLPI_SIDN_M(140), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULLONG *data);


//! @ingroup ParamLibDataRd
//! This function reads the 32-bit floating point data value (single precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'C-0-0070'.
//! FLOAT data = 0.0;
//! MLPIRESULT result = mlpiParameterReadDataFloat(connection, 0, MLPI_SIDN_C(70), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, FLOAT *data);


//! @ingroup ParamLibDataRd
//! This function reads the 64-bit floating point data value (double precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'A-0-0045' of axis '1'.
//! DOUBLE data = 0.0;
//! MLPIRESULT result = mlpiParameterReadDataDouble(connection, 1, MLPI_SIDN_A(45), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, DOUBLE *data);


//! @ingroup ParamLibDataRd
//! This function reads the value of a parameter and returns it as string. A conversion from other types is done automatically.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored in UTF16 format (string).
//! @param[in]   numElements        Number of WCHAR16 elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of WCHAR16 elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'C-0-0012'.
//! WCHAR16 data[512] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiParameterReadDataString(connection, 0, MLPI_SIDN_C(12), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataString(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, WCHAR16 *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads the data value of a parameter.
//! @note
//! This function is not type safe and should only be used when no type is necessary. For example, when reading
//! all parameters in order to make a safe set and restoring them in exactly the same way. It will expect string
//! parameters in UTF-8 format. The size for strings must be given without a trailing \\0.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   dataSize           Size in bytes available in 'data' for reading.
//! @param[out]  dataSizeRet        Size in bytes in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the data value of parameter 'A-0-0045' of axis '1'.
//! DOUBLE data = 0.0;
//! ULONG dataSizeRet = 0;
//! MLPIRESULT result = mlpiParameterReadDataArrayVoid(connection, 1, MLPI_SIDN_A(45), &data, sizeof(data), &dataSizeRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayVoid(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, void *data, const ULONG dataSize, ULONG *dataSizeRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 8-bit signed data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0081'.
//! CHAR data[512];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayChar(connection, 0, MLPI_SIDN_C(81), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayChar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, CHAR *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 8-bit unsigned data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0081'.
//! UCHAR data[512];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayUchar(connection, 0, MLPI_SIDN_C(81), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayUchar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, UCHAR *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 16-bit signed data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0625'.
//! SHORT data[4096];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayShort(connection, 0, MLPI_SIDN_C(625), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, SHORT *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 16-bit unsigned data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0484'.
//! USHORT data[128];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayUshort(connection, 0, MLPI_SIDN_C(484), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, USHORT *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 32-bit signed data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-2001'.
//! LONG data[1024];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayLong(connection, 0, MLPI_SIDN_C(2001), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 32-bit unsigned data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0050'.
//! ULONG data[2];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayUlong(connection, 0, MLPI_SIDN_C(50), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 64-bit signed data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0110'.
//! // Note: The return values could be misinterpreted because this parameter has unsigned data values.
//! LLONG data[2048];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayLlong(connection, 0, MLPI_SIDN_C(110), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LLONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 64-bit unsigned data values of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0110'.
//! ULLONG data[2048];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayUllong(connection, 0, MLPI_SIDN_C(110), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULLONG *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 32-bit floating point data values (single precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'C-0-0702'.
//! FLOAT data[2];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayFloat(connection, 0, MLPI_SIDN_C(702), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, FLOAT *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataRd
//! This function reads an array of 64-bit floating point data values (double precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the value will be stored.
//! @param[in]   numElements        Number of elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read an array of data values of parameter 'A-0-3027' of axis '1'.
//! DOUBLE data[2048];
//! ULONG numElementsRet = 0;
//! memset(data, 0, sizeof(data));
//! MLPIRESULT result = mlpiParameterReadDataArrayDouble(connection, 0, MLPI_SIDN_C(702), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataArrayDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, DOUBLE *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibDataWr
//! This function writes a given 16-bit signed data value to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '-10' to the parameter 'A-0-0058' of axis '1'.
//! SHORT data = -10;
//! MLPIRESULT result = mlpiParameterWriteDataShort(connection, 1, MLPI_SIDN_A(58), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const SHORT data);


//! @ingroup ParamLibDataWr
//! This function writes a given 16-bit unsigned data value to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to access. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '2' to the parameter 'C-0-0450'.
//! USHORT data = 2;
//! MLPIRESULT result = mlpiParameterWriteDataUshort(connection, 0, MLPI_SIDN_C(450), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const USHORT data);


//! @ingroup ParamLibDataWr
//! This function writes a given 32-bit signed data value to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '100' to the parameter 'M-0-0200' of touch probe '1'.
//! LONG data = 100;
//! MLPIRESULT result = mlpiParameterWriteDataLong(connection, 1, MLPI_SIDN_M(200), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const LONG data);


//! @ingroup ParamLibDataWr
//! This function writes a given 32-bit unsigned data value to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '4000' to the parameter 'C-0-0400'.
//! ULONG data = 4000;
//! MLPIRESULT result = mlpiParameterWriteDataUlong(connection, 0, MLPI_SIDN_C(400), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const ULONG data);


//! @ingroup ParamLibDataWr
//! This function writes a given 64-bit signed data value to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '42' to the parameter 'M-0-0140' of touch probe '1'.
//! // Note: The function should return an error because this parameter is write protected.
//! LLONG data = 42;
//! MLPIRESULT result = mlpiParameterWriteDataLlong(connection, 1, MLPI_SIDN_M(140), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const LLONG data);


//! @ingroup ParamLibDataWr
//! This function writes a given 64-bit unsigned data value to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '0x2A' to the parameter 'M-0-0140' of touch probe '1'.
//! // Note: The function should return an error because this parameter is write protected.
//! LLONG data = 42;
//! MLPIRESULT result = mlpiParameterWriteDataUllong(connection, 1, MLPI_SIDN_M(140), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const ULLONG data);


//! @ingroup ParamLibDataWr
//! This function writes a given 32-bit floating point data value (single precision) to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '1.23' to the parameter 'A-0-0036' of axis '1'.
//! FLOAT data = 1.23;
//! MLPIRESULT result = mlpiParameterWriteDataFloat(connection, 1, MLPI_SIDN_A(36), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const FLOAT data);


//! @ingroup ParamLibDataWr
//! This function writes a given 64-bit floating point data value (double precision) to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Data value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value '1.23456789' to the parameter 'A-0-0046' of axis '1'.
//! DOUBLE data = 1.23456789;
//! MLPIRESULT result = mlpiParameterWriteDataDouble(connection, 1, MLPI_SIDN_A(46), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const DOUBLE data);


//! @ingroup ParamLibDataWr
//! This function writes a given UTF16 string to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to variable value to be written to the parameter.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the value 'This is a string' to the parameter 'C-0-0081'.
//! WCHAR16 data[] = L"This is a string";
//! MLPIRESULT result = mlpiParameterWriteDataString(connection, 0, MLPI_SIDN_C(81), data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataString(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const WCHAR16 *data);


//! @ingroup ParamLibDataWr
//! This function writes a given data value to a parameter.
//! @note
//! This function is not type safe and should only be used when no type is necessary. For example, when reading
//! all parameters in order to make a safe set and restoring them exactly in the same way. It will expect string
//! parameters in UTF-8 format. The size for strings must be given without a trailing \\0.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to variable to be written to the parameter.
//! @param[in]  dataSize            Size in bytes of 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the data value '1.23456789' to the parameter 'A-0-0045' of axis '1'.
//! DOUBLE data = 180.0;
//! MLPIRESULT result = mlpiParameterWriteDataArrayVoid(connection, 1, MLPI_SIDN_A(45), &data, sizeof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayVoid(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const void *data,  const ULONG dataSize);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 8-bit signed data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to variable values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'C-0-0081'.
//! CHAR data[] = {-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5};
//! MLPIRESULT result = mlpiParameterWriteDataArrayChar(connection, 0, MLPI_SIDN_C(81), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayChar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const CHAR *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 8-bit unsigned data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to variable values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values ('This is a string') to the parameter 'C-0-0081'.
//! UCHAR data[] = {0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, 0x61, 0x20, 0x73, 0x74, 0x72, 0x69, 0x6e, 0x67};
//! MLPIRESULT result = mlpiParameterWriteDataArrayUchar(connection, 0, MLPI_SIDN_C(81), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayUchar(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const UCHAR *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 16-bit signed data values to a parameter.
//! @param[in]  connection          Handle for multiple connections..
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'C-0-0625'.
//! // Note: The function should return an error because this parameter is write protected.
//! SHORT data[] = {0, 0, 0, 0, 0, 0, 0, 0};
//! MLPIRESULT result = mlpiParameterWriteDataArrayShort(connection, 0, MLPI_SIDN_C(625), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const SHORT *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes a array of given 16-bit unsigned data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'C-0-2484'.
//! USHORT data[] = {1, 1, 1, 1, 1, 1, 1, 1};
//! MLPIRESULT result = mlpiParameterWriteDataArrayUshort(connection, 0, MLPI_SIDN_C(2484), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const USHORT *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 32-bit signed data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'N-0-0311' of PLS '1'.
//! LONG data[] = {125, 125, 125, 125, 125, 125, 125, 125};
//! MLPIRESULT result = mlpiParameterWriteDataArrayLong(connection, 1, MLPI_SIDN_N(311), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const LONG *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 32-bit unsigned data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'C-0-0050'.
//! ULONG data[] = {0x1C984660, 0x01BF5458};
//! MLPIRESULT result = mlpiParameterWriteDataArrayUlong(connection, 0, MLPI_SIDN_C(50), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const ULONG *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 64-bit signed data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'C-0-0050'.
//! // Note: The result should be an error because the size of the element (LLONG) isn't valid.
//! LLONG data[] = {0x2A, 0x2A};
//! MLPIRESULT result = mlpiParameterWriteDataArrayLlong(connection, 0, MLPI_SIDN_C(50), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const LLONG *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 64-bit unsigned data values to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                Id of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired Id.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'C-0-0110'.
//! // Note: The result should be an error because this parameter is write protected.
//! ULLONG data[] = {0x2A, 0x2A};
//! MLPIRESULT result = mlpiParameterWriteDataArrayUllong(connection, 0, MLPI_SIDN_C(110), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const ULLONG *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 32-bit floating point data values (single precision) to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'N-0-0315' of PLS '1'.
//! FLOAT data[] = {180.0, 210.0};
//! MLPIRESULT result = mlpiParameterWriteDataArrayFloat(connection, 1, MLPI_SIDN_C(110), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const FLOAT *data, const ULONG numElements);


//! @ingroup ParamLibDataWr
//! This function writes an array of given 64-bit floating point data values (double precision) to a parameter.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[in]  data                Pointer to values to be written to the parameter.
//! @param[in]  numElements         Number of elements in 'data' available to write.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Write the listed values to the parameter 'A-0-3027' of axis '1'.
//! DOUBLE data[] = {100.0, 500.0, 750.0, 900.0};
//! MLPIRESULT result = mlpiParameterWriteDataArrayDouble(connection, 1, MLPI_SIDN_A(3027), data, _countof(data));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteDataArrayDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, const DOUBLE *data, const ULONG numElements);


//! @ingroup ParamLibDefault
//! This function reads the 16-bit signed default value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'A-0-0058' of axis '1'.
//! SHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultShort(connection, 1, MLPI_SIDN_A(58), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultShort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, SHORT *data);


//! @ingroup ParamLibDefault
//! This function reads the 16-bit unsigned default value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'C-0-0450'.
//! USHORT data = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultUshort(connection, 0, MLPI_SIDN_C(450), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultUshort(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, USHORT *data);


//! @ingroup ParamLibDefault
//! This function reads the 32-bit signed default value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'M-0-0200' of touch probe '1'.
//! LONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultLong(connection, 1, MLPI_SIDN_M(200), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultLong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LONG *data);


//! @ingroup ParamLibDefault
//! This function reads the 32-bit unsigned default value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'C-0-0400'.
//! ULONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultUlong(connection, 0, MLPI_SIDN_C(400), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultUlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *data);


//! @ingroup ParamLibDefault
//! This function reads the 64-bit signed default value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'M-0-0140' of touch probe '1'.
//! // Note: The return value could be misinterpreted because this parameter has an unsigned data value.
//! LLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultLlong(connection, 1, MLPI_SIDN_M(140), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultLlong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, LLONG *data);


//! @ingroup ParamLibDefault
//! This function reads the 64-bit unsigned default value of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'M-0-0140' of touch probe '1'.
//! ULLONG data = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultUllong(connection, 1, MLPI_SIDN_M(140), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultUllong(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULLONG *data);


//! @ingroup ParamLibDefault
//! This function reads the 32-bit floating point default value (single precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'A-0-2761' of axis '1'.
//! FLOAT data = 0.0;
//! MLPIRESULT result = mlpiParameterReadDefaultFloat(connection, 1, MLPI_SIDN_C(2761), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultFloat(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, FLOAT *data);


//! @ingroup ParamLibDefault
//! This function reads the 64-bit floating point default value (double precision) of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'A-0-0045' of axis '1'.
//! DOUBLE data = 0.0;
//! MLPIRESULT result = mlpiParameterReadDefaultDouble(connection, 1, MLPI_SIDN_A(45), &data);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultDouble(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, DOUBLE *data);


//! @ingroup ParamLibDefault
//! This function reads the default value of a parameter and returns it as string. A conversion is done automatically.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  data               Pointer to variable where the default value will be stored in UTF16 format (string).
//! @param[in]   numElements        Number of WCHAR16 elements in 'data' available to read.
//! @param[out]  numElementsRet     Number of WCHAR16 elements in complete 'data'.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the default value of parameter 'A-0-0045' of axis '1'.
//! WCHAR16 data[512] = L"";
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiParameterReadDefaultString(connection, 1, MLPI_SIDN_A(45), data, _countof(data), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDefaultString(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, WCHAR16 *data, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ParamLibCmd
//! This function reads the data status of a parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  status             Pointer to variable where the status will be stored. TRUE if data status is valid.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note You may want to look at the sercos specifications for a detailed description of the meaning of the data status.
//!
//! @par Example:
//! @code
//! // Read the data status of parameter 'C-0-0850'.
//! BOOL8 status = FALSE;
//! MLPIRESULT result = mlpiParameterReadDataStatus(connection, 0, MLPI_SIDN_C(850), &status);
//! if (status)
//!   printf("data status valid!");
//! else
//!   printf("data status not valid!");
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadDataStatus(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, BOOL8 *status);


//! @ingroup ParamLibCmd
//! This function sets a command through a parameter. The command will be executed and the function will return
//! when the command was either executed successfully or with an error (function is blocking while executing the command).
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  address             Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]  sidn                ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Execute command of parameter 'C-0-1010'.
//! MLPIRESULT result = mlpiParameterCommand(connection, 0, MLPI_SIDN_C(1010));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterCommand(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn);


//! @ingroup ParamLibCmd
//! This function reads the parameter command status (comparing to @ref MlpiParameterCommandStatus). Use this function
//! to get the command status of parallel command executions triggered by several calls of @ref mlpiParameterWriteDataUlong
//! or the like.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  status             Pointer to variable where the status will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note You may want to look at the sercos specifications for a detailed description of the meaning of the command status.
//!
//! @par Example:
//! @code
//! // Read the command status of parameter 'C-0-1010'.
//! USHORT status = 0;
//! MLPIRESULT result = mlpiParameterReadCommandStatus(connection, 0, MLPI_SIDN_C(1010), &status);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadCommandStatus(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, USHORT *status);


//! @ingroup ParamLibAux
//! This function reads the current and maximum length of a list parameter.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   address            Address identifying the object to be accessed. Use macro @ref MLPI_ADDRESS_x to generate an address field.
//! @param[in]   sidn               ID of parameter to be accessed. Use macro @ref MLPI_SIDN_x to get the desired ID.
//! @param[out]  numElements        Pointer to variable where the number of current elements in the list will be stored.
//! @param[out]  numMaxElements     Pointer to variable where the maximum number of elements that can be in the list
//!                                 will be stored.
//! @param[out]  elementSize        Pointer to variable where the size in bytes of single list element will be stored.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @note Many parameters are volatile and may change. This means that also the length of a parameter list may change
//! between a call to @ref mlpiParameterReadListLength and mlpiParameterReadData.
//!
//! @par Example:
//! @code
//! // Read the length of parameter 'C-0-1010'.
//! ULONG numElements = 0;
//! ULONG numMaxElements = 0;
//! ULONG elementSize = 0;
//! MLPIRESULT result = mlpiParameterReadListLength(connection, 0, MLPI_SIDN_C(1010), &numElements, &numMaxElements, &elementSize);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadListLength(const MLPIHANDLE connection, const ULLONG address, const ULLONG sidn, ULONG *numElements, ULONG *numMaxElements, ULONG *elementSize);


//! @ingroup ParamLibAux
//! This function imports a parameter file from CF-card. The string 'mapping' can give information on which instance should be
//! imported and to which number of instance it should be written.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   path               Absolute path of file which should be imported.
//! @param[out]  errorList          Pointer to an array of structure where the wrong parameter and the error codes will be stored
//! @param[in]   numElements        Number of MlpiSidnError elements available in 'errorList' for writing.
//! @param[out]  numElementsRet     Number of elements needed to collect all information of import errors.
//! @param[in]   mapping            String which tells you which parameter instance should be imported and to which instance number.\n
//!                                 The general structure is a tag list of jobs: <b>{JobPart;}</b>\n
//!                                 Each job is separated by a semicolon. When no job is given, all parameters will
//!                                 be imported to the instance written in the header of the sercos ASCII file.\n
//!                                 A job has the following structure: <b>[InstanceMapping]:Parametertype</b> (in squared brackets).\n
//!                                 <b>Parametertype</b> is represented by A, C, K, M, N, O, S and P.\n
//!                                 In part <b>InstanceMapping</b>, you can select which parameter is imported to which instance.
//!                                 If given a single number only, this instance will be imported. If there is a parameter file in
//!                                 which data are associated with another instance number, one instance can be mapped to the other using '>'.
//!                                 It's also possible to map one instance to a range of instances.\n
//!                                 Examples:\n
//!                                 <b>[1]:A</b> will import axis parameter for axis 1.\n
//!                                 <b>[1>3]:A</b> will map axis parameter from instance 1 to 3.\n
//!                                 <b>[1>2-5]:A</b> will map axis parameter from instance 1 to axis 2 till 5.
//! @return                         Return value indicating success (>=0) or error (<0).\n
//! @par Example:
//! @code
//! // Import axis parameter from given file with included parameters for instance 1 to axis 6
//! WCHAR path[] = L"/ata0b/importFile.par";
//! MlpiSidnError errorList[10] = {0}; // array to store errors which occur during import
//! ULONG numElementsRet = 0;
//! WCHAR mapping[] = L"[1>6]:A";
//! MLPIRESULT result = mlpiParameterImportFile(connection, path, errorList, _countof(errorList), &numElementsRet, mapping);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterImportFile(const MLPIHANDLE connection, const WCHAR16* path, MlpiSidnError* errorList, const ULONG numElements, ULONG* numElementsRet, const WCHAR16* mapping);


//! @ingroup ParamLibAux
//! This function exports parameters to a file on CF-card. In String 'exportPattern' the instances can be selected.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   path               Absolute path of file where exported parameter will be stored.
//! @param[in]   exportPattern      String where you can select which instances and which types of parameter should be exported.\n
//!                                 The general structure is a tag list of jobs: <b>(JobPart;)*</b>\n
//!                                 All jobs are separated by a semicolon. When no job is selected, no parameter will be exported.\n
//!                                 The structure of a job is the following: <b>[InstanceRanges]:ParameterType[ParameterRanges]</b>
//!                                 (with all squared brackets).\n
//!                                 The elements <b>"[InstanceRanges]:"</b> and <b>"[ParameterRanges]"</b> are optional. If these elements
//!                                 are not part of a string, the whole available range is used. The element <b>ParameterType</b> is mandatory. \n
//!                                 <b>"[InstanceRanges]:"</b> and <b>"[ParameterRanges]"</b> can be lists of numbers, ranges or words "TO_SAVE"
//!                                 (all parameters which are needed to restore) or "MODIFIED" (all modified parameters). The stated words cannot be
//!                                 combined with the list of numbers or ranges. Furthermore, "MODIFIED" is only available for sercos parameters. When
//!                                 related parameter not exist on sercos device none parameter will export.\n
//!                                 Examples:\n
//!                                 <b>A;C</b> will export all axis and control parameters for each instance.\n
//!                                 <b>[2]:A</b> will export all axis parameter for axis 2.\n
//!                                 <b>A[1-5]</b> will export axis parameter A-0-0001 till A-0-0005 for each axis.\n
//!                                 <b>[1,3-4,5]:A[8-9,11-13]</b> Will export axis parameters (A-0-0008, A-0-0009, A-0-0011, A-0-0012, A-0-0013) for axis 1, 3, 4 and 5.\n
//!                                 <b>A[TO_SAVE];[1-3]:S[MODIFIED]</b> Will export parameters of all axes which are needed to resore and all modified parameters of sercos device 1 till 3.
//! @param[out]  errorList          Pointer to an array of structures where the wrong parameter and the error codes will be stored
//! @param[in]   numElements        Number of MlpiSidnError elements available in 'errorList' for reading.
//! @param[out]  numElementsRet     Number of elements used or needed. When this is greater than numElements, all elements in the array are
//!                                 filled out but some information about errors is lost.
//! @return                         Return value indicating success (>=0) or error (<0).\n
//! @par Example:
//! @code
//! // Export Parameter A-0-0001 till A-0003 for axis 2 and all control parameters
//! WCHAR path[] = L"/ata0b/exportFile.par";
//! WCHAR exportPattern[] = L"[2]:A[1-3];C";
//! MlpiSidnError errorList[10] = {0}; // array to store errors that occur during export
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiParameterExportFile(connection, path, exportPattern, errorList, _countof(errorList), &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterExportFile(const MLPIHANDLE connection, const WCHAR16* path, const WCHAR16* exportPattern, MlpiSidnError* errorList, const ULONG numElements, ULONG* numElementsRet);


//! @ingroup ParamLibAux
//! This function reads every element of a list of given parameters (@ref paramElements "Parameter structure"). This means that you can read multiple parameters at once and you can read
//! each element (value, default value, min, max, attribute, ...) of each parameter at once. All with a single function call instead of multiple calls to different functions.
//! On the server side, the firmware tries to read the data in parallel where possible.
//! @param[in]   connection         Handle for multiple connections.
//! @param[out]  readEverything     Structure where the information of all elements of the parameter are stored. \n
//!                                 The address and the sidn are input parameters. They define the address identifying the object and the ID of the parameter to be accessed. \n
//!                                 The elements attribute and IDN of an parameter have an fixed length. The values are stored in attribute and dataStatus. \n
//!                                 All other elements have an variable length. Their values are written into 'data'.
//!                                 To access an element value, use the offsets. The length of the elements minimum, maximum and data is stored in dataLength.
//!                                 If the the parameter is a list its length of the data is stored in dataSize. In this case the maximum list length is stored in maxDataSize. \n
//!                                 The parameter result indicates if this single request succeeded or failed. An error is returned if an general error occurred or if reading of
//!                                 one mandatory element failed (@ref paramElements "Parameter structure").
//!                                 Which elements are valid can be read via the validElements parameter.
//! @param[in]   numElements        Number of MlpiReadEverything elements available in 'readEverything' for reading.
//! @param[out]  data               Buffer to store the data of all elements with variable length.
//! @param[in]   dataSize           Number of UCHAR elements available in 'data' for reading.
//! @return                         Return value indicating success (>=0) or error (<0). If an error is returned the data is invalid.\n
//!
//! @note If the parameter has a @ref paramElements "data length" of one octet with a variable length and a @ref paramElements "data type"
//!       is an extended character set, its data is converted to an an null-terminated WCHAR16 string. The attribute isn't converted in this case.
//!
//! @par Example:
//! @code
//! //reading every element of the parameters C-0-0400 and S-0-1002 of the device 1
//! MLPIRESULT  result = MLPI_S_OK;
//! const ULONG numElements = 2;
//! UCHAR       buffer[256] = {};
//! WCHAR16     idn[PARAM_MAX_IDN_STRING_LENGTH] = {L'0'};
//! MlpiReadEverything readEverything[numElements];
//!
//! readEverything[0].address = 0;
//! readEverything[0].sidn    = MLPI_SIDN_C(400);
//! readEverything[1].address = MLPI_ADDRESS(MLPI_ADDRESS_MODE_PHYSICAL, 0, 1);
//! readEverything[1].sidn    = MLPI_SIDN_S(1002);
//!
//! result = mlpiParameterReadEverything(connection, readEverything, numElements, buffer, _countof(buffer));
//!
//! // we use the helper class from #include <util/mlpiParameterHelper.h> to access the data
//! MlpiReadEverythingDataAccess dataAccess(readEverything, numElements, buffer, _countof(buffer));
//! for(ULONG i=0; numElements>i; i++)
//! {
//!   memset(idn, 0, _countof(idn));
//!   printf("\n\nData of Parameter %s", utilParameterParseIdn(readEverything[i].sidn, idn, _countof(idn)));
//!   printf("\nParameter element 1 (DataStatus)    = %lu",   dataAccess.getDataStatus(i));
//!   printf("\nParameter element 2 (Name)          = %s",    dataAccess.getName(i));
//!   printf("\nParameter element 3 (Attribute)     = 0x%x",  dataAccess.getAttribute(i));
//!   printf("\nParameter element 4 (Unit)          = %s",    dataAccess.getUnit(i));
//!   // read data using the correct function
//!   switch(dataAccess.getDataType(i))
//!   {
//!     case MLPI_TYPE_UCHAR:
//!     {
//!       const UCHAR *pMinValue = dataAccess.getMinimumValue<UCHAR>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %c", *pMinValue);
//!       const UCHAR *pMaxValue = dataAccess.getMaximumValue<UCHAR>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %c", *pMaxValue);
//!       const UCHAR *pData = dataAccess.getData<UCHAR>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %c", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_USHORT:
//!     {
//!       const USHORT *pMinValue = dataAccess.getMinimumValue<USHORT>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %u", *pMinValue);
//!       const USHORT *pMaxValue = dataAccess.getMaximumValue<USHORT>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %u", *pMaxValue);
//!       const USHORT *pData = dataAccess.getData<USHORT>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %u", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_SHORT:
//!     {
//!       const SHORT *pMinValue = dataAccess.getMinimumValue<SHORT>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %d", *pMinValue);
//!       const SHORT *pMaxValue = dataAccess.getMaximumValue<SHORT>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %d", *pMaxValue);
//!       const SHORT *pData = dataAccess.getData<SHORT>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %d", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_ULONG:
//!     {
//!       const ULONG *pMinValue = dataAccess.getMinimumValue<ULONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %lu", *pMinValue);
//!       const ULONG *pMaxValue = dataAccess.getMaximumValue<ULONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %lu", *pMaxValue);
//!       const ULONG *pData = dataAccess.getData<ULONG>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %lu", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_LONG:
//!     {
//!       const LONG *pMinValue = dataAccess.getMinimumValue<LONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %ld", *pMinValue);
//!       const LONG *pMaxValue = dataAccess.getMaximumValue<LONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %ld", *pMaxValue);
//!       const LONG *pData = dataAccess.getData<LONG>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %ld", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_ULLONG:
//!     {
//!       const ULLONG *pMinValue = dataAccess.getMinimumValue<ULLONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %llu", *pMinValue);
//!       const ULLONG *pMaxValue = dataAccess.getMaximumValue<ULLONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %llu", *pMaxValue);
//!       const ULLONG *pData = dataAccess.getData<ULLONG>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %llu", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_LLONG:
//!     {
//!       const LLONG *pMinValue = dataAccess.getMinimumValue<LLONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %lld", *pMinValue);
//!       const LLONG *pMaxValue = dataAccess.getMaximumValue<LLONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %lld", *pMaxValue);
//!       const LLONG *pData = dataAccess.getData<LLONG>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %lld", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_FLOAT:
//!     {
//!       const FLOAT *pMinValue = dataAccess.getMinimumValue<FLOAT>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %f", *pMinValue);
//!       const FLOAT *pMaxValue = dataAccess.getMaximumValue<FLOAT>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %f", *pMaxValue);
//!       const FLOAT *pData = dataAccess.getData<FLOAT>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %f", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_DOUBLE:
//!     {
//!       const DOUBLE *pMinValue = dataAccess.getMinimumValue<DOUBLE>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %f", *pMinValue);
//!       const DOUBLE *pMaxValue = dataAccess.getMaximumValue<DOUBLE>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %f", *pMaxValue);
//!       const DOUBLE *pData = dataAccess.getData<DOUBLE>(i);
//!       if (pData)      printf("\nParameter element 7 (Data)          = %f", *pData);
//!       break;
//!     }
//!     case MLPI_TYPE_UCHAR_ARRAY:
//!     {
//!       const UCHAR *pMinValue = dataAccess.getMinimumValue<UCHAR>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %c", *pMinValue);
//!       const UCHAR *pMaxValue = dataAccess.getMaximumValue<UCHAR>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %c", *pMaxValue);
//!       const UCHAR *pData = dataAccess.getData<UCHAR>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %c", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_USHORT_ARRAY:
//!     {
//!       const USHORT *pMinValue = dataAccess.getMinimumValue<USHORT>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %u", *pMinValue);
//!       const USHORT *pMaxValue = dataAccess.getMaximumValue<USHORT>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %u", *pMaxValue);
//!       const USHORT *pData = dataAccess.getData<USHORT>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %u", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_SHORT_ARRAY:
//!     {
//!       const SHORT *pMinValue = dataAccess.getMinimumValue<SHORT>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %d", *pMinValue);
//!       const SHORT *pMaxValue = dataAccess.getMaximumValue<SHORT>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %d", *pMaxValue);
//!       const SHORT *pData = dataAccess.getData<SHORT>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %d", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_ULONG_ARRAY:
//!     {
//!       const ULONG *pMinValue = dataAccess.getMinimumValue<ULONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %lu", *pMinValue);
//!       const ULONG *pMaxValue = dataAccess.getMaximumValue<ULONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %lu", *pMaxValue);
//!       const ULONG *pData = dataAccess.getData<ULONG>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %lu", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_LONG_ARRAY:
//!     {
//!       const LONG *pMinValue = dataAccess.getMinimumValue<LONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %ld", *pMinValue);
//!       const LONG *pMaxValue = dataAccess.getMaximumValue<LONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %ld", *pMaxValue);
//!       const LONG *pData = dataAccess.getData<LONG>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %ld", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_ULLONG_ARRAY:
//!     {
//!       const ULLONG *pMinValue = dataAccess.getMinimumValue<ULLONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %llu", *pMinValue);
//!       const ULLONG *pMaxValue = dataAccess.getMaximumValue<ULLONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %llu", *pMaxValue);
//!       const ULLONG *pData = dataAccess.getData<ULLONG>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %llu", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_LLONG_ARRAY:
//!     {
//!       const LLONG *pMinValue = dataAccess.getMinimumValue<LLONG>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %lld", *pMinValue);
//!       const LLONG *pMaxValue = dataAccess.getMaximumValue<LLONG>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %lld", *pMaxValue);
//!       const LLONG *pData = dataAccess.getData<LLONG>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)      printf("\nParameter element 7 (Data); List element %lu = %lld", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_FLOAT_ARRAY:
//!     {
//!       const FLOAT *pMinValue = dataAccess.getMinimumValue<FLOAT>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %f", *pMinValue);
//!       const FLOAT *pMaxValue = dataAccess.getMaximumValue<FLOAT>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %f", *pMaxValue);
//!       const FLOAT *pData = dataAccess.getData<FLOAT>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %f", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_DOUBLE_ARRAY:
//!     {
//!       const DOUBLE *pMinValue = dataAccess.getMinimumValue<DOUBLE>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %f", *pMinValue);
//!       const DOUBLE *pMaxValue = dataAccess.getMaximumValue<DOUBLE>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %f", *pMaxValue);
//!       const DOUBLE *pData = dataAccess.getData<DOUBLE>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %f", j, pData[j]);
//!       }
//!       break;
//!     }
//!     case MLPI_TYPE_CHAR_UTF8_ARRAY :
//!     case MLPI_TYPE_CHAR_UTF16_ARRAY:
//!     {
//!       const WCHAR16 *pMinValue = dataAccess.getMinimumValue<WCHAR16>(i);
//!       if (pMinValue)  printf("\nParameter element 5 (Minimum value) = %s", *pMinValue);
//!       const WCHAR16 *pMaxValue = dataAccess.getMaximumValue<WCHAR16>(i);
//!       if (pMaxValue)  printf("\nParameter element 6 (Maximum value) = %s", *pMaxValue);
//!       const WCHAR16 *pData = dataAccess.getData<WCHAR16>(i);
//!       for (ULONG j=0; dataAccess.getNumDataElements(i)>j; j++)
//!       {
//!         if (pData)    printf("\nParameter element 7 (Data); List element %lu = %s", j, pData[j]);
//!       }
//!       break;
//!     }
//!     default:
//!       // unknown or invalid data type...
//!       break;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiParameterReadEverything(const MLPIHANDLE connection, MlpiReadEverything* readEverything, const ULONG numElements, UCHAR* data, const ULONG dataSize);


//! @ingroup ParamLibAux
//! This function exports parameters to a file on CF card in a asynchronous way. In string 'exportPattern', the instances can be selected.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   path               Absolute path of file where exported parameter will be stored.
//! @param[in]   exportPattern      String where you can select which instances and which types of parameter should be exported.\n
//!                                 The general structure is a tag list of jobs: <b>(JobPart;)*</b>\n
//!                                 All jobs are separated by a semicolon. When no job is selected, no parameter will be exported.\n
//!                                 The structure of a job is the following: <b>[InstanceRanges]:ParameterType[ParameterRanges]</b>
//!                                 (with all squared brackets).\n
//!                                 The elements <b>"[InstanceRanges]:"</b> and <b>"[ParameterRanges]"</b> are optional. If these elements
//!                                 are not part of string, the whole available range is used. The element <b>ParameterType</b> is mandatory. \n
//!                                 <b>"[InstanceRanges]:"</b> and <b>"[ParameterRanges]"</b> can be lists of numbers, ranges or words "TO_SAVE"
//!                                 (all parameters which are needed to restore) or "MODIFIED" (all modified parameters). The stated words cannot
//!                                 combined with list of numbers or ranges. Furthermore, "MODIFIED" is only available for sercos parameters. When
//!                                 related parameter not exist on sercos device none parameter will export.\n
//!                                 Examples:\n
//!                                 <b>A;C</b> will export all axis and control parameters for each instance.\n
//!                                 <b>[2]:A</b> will export all axis parameter for axis 2.\n
//!                                 <b>A[1-5]</b> will export axis parameter A-0-0001 till A-0-0005 for each axis.\n
//!                                 <b>[1,3-4,5]:A[8-9,11-13]</b> Will export axis parameters (A-0-0008, A-0-0009, A-0-0011, A-0-0012, A-0-0013) for axis 1, 3, 4 and 5.\n
//!                                 <b>A[TO_SAVE];[1-3]:S[MODIFIED]</b> Will export parameters of all axes which are needed to resore and all modified parameters of sercos device 1 till 3.
//! @param[out]  process            Handle of created process
//! @return                         Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPIHANDLE process;
//! result = mlpiParameterExportFileStartProcess(connection, L"/ata0b/export.par", L"C;A;M;N;K;O;S[TO_SAVE]", &process)
//! if (MLPI_FAILED(result))
//!   printf("unable to start export with error code: %08X", result);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterExportFileStartProcess(const MLPIHANDLE connection, const WCHAR16* path, const WCHAR16* exportPattern, PROCESSHANDLE* process);


//! @ingroup ParamLibAux
//! This function imports a parameter file from CF card in a asynchronous way. The string 'mapping' can give information on which instance should be
//! imported and to which number of instance it should be written.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   path               Absolute path of file which should be imported.
//! @param[in]   mapping            String which tells you which parameter instance should be imported and to which instance number.\n
//!                                 The general structure is a tag list of jobs: <b>{JobPart;}</b>\n
//!                                 Each job is separated by a semicolon. When no job is given, all parameters will
//!                                 be imported to the instance written in the header of the sercos ASCII file.\n
//!                                 A job has the following structure: <b>[InstanceMapping]:Parametertype</b> (in squared brackets).\n
//!                                 <b>Parametertype</b> is represented by A, C, K, M, N, O, S and P.\n
//!                                 In part <b>InstanceMapping</b>, you can select which parameter is imported to which instance.
//!                                 If given a single number only, this instance will be imported. If there is a parameter file in
//!                                 which data are associated with another instance number, one instance can be mapped to the other using '>'.
//!                                 It's also possible to map one instance to a range of instances.\n
//!                                 Examples:\n
//!                                 <b>[1]:A</b> will import axis parameter for axis 1.\n
//!                                 <b>[1>3]:A</b> will map axis parameter from instance 1 to 3.\n
//!                                 <b>[1>2-5]:A</b> will map axis parameter from instance 1 to axis 2 till 5.
//! @param[out]  process            Handle of created process
//! @return                         Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPIHANDLE process;
//! // import everything of file /ata0b/import.par
//! result = mlpiParameterImportFileStartProcess(connection, L"/ata0b/import.par", L"", &process)
//! if (MLPI_FAILED(result))
//!   printf("unable to start import with error code: %08X", result);
//! @endcode
MLPI_API MLPIRESULT mlpiParameterImportFileStartProcess(const MLPIHANDLE connection, const WCHAR16* path, const WCHAR16* mapping, PROCESSHANDLE* process);


//! @ingroup ParamLibAux
//! This function returns the status of a parameter process (import or export).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   process            Handle of process
//! @param[out]  status             status of process
//! @param[out]  currentCount       count of current parameters to export or import
//! @param[out]  expectedCount      count of expected parameters to export or import
//! @param[out]  errorsCount        Number of errors while import or export. When this is greater than numElements, all elements in the array are
//!                                 filled out but some information about errors is lost.
//! @param[out]  errorList          Pointer to an array of structures where the wrong parameter and the error codes will be stored
//! @param[in]   numElements        Number of MlpiSidnError elements available in 'errorList' for reading.
//! @return                         Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPIHANDLE process;
//! result = mlpiParameterImportFileStartProcess(connection, L"/ata0b/import.par", L"", &process)
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start import with error code: %08X", result);
//!   return;
//! }
//! 
//! ULONG countCurrent = 0;
//! ULONG countExpected = 0;
//! ULONG countError = 0;
//! MlpiSidnError errorList[1024];
//! MlpiParamProcessStatus processStatus;
//! do 
//! {
//!   APICALL_AND_CHECK(mlpiParameterImportExportStatus(connection,importHandle,&processStatus,&countCurrent, &countExpected, &countError, errorList, 1024));
//!   printf("\nImported %4u of %4u (errors %4u, state %u)",countCurrent, countExpected, countError, processStatus.processState);
//! } while ((processStatus.processState != MLPI_PROCESS_STATUS_ERROR) && (processStatus.processState != MLPI_PROCESS_STATUS_FINISHED));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterImportExportStatus(const MLPIHANDLE connection, const PROCESSHANDLE process, MlpiParamProcessStatus* status, ULONG* currentCount, ULONG* expectedCount, ULONG* errorsCount, MlpiSidnError* errorList, ULONG numElements);


//! @ingroup ParamLibAux
//! This function returns information about a parameter process (import or export).
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   process            Handle of process
//! @param[out]  pattern            Given pattern of process
//! @param[in]   numElementsPattern Size of pattern
//! @param[out]  file               Given file of process
//! @param[in]   numElementsFile    Size of file
//! @return                         Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPIHANDLE process;
//! result = mlpiParameterImportFileStartProcess(connection, L"/ata0b/import.par", L"", &process)
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start import with error code: %08X", result);
//!   return;
//! }
//!
//! WCHAR16 strPattern[256];
//! WCHAR16 strFile[256];
//! result = mlpiParameterImportExportGetInfo(connection, process, strPattern, 256, strFile, 256);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to abort import with error code: %08X", result);
//!   return;
//! }
//! printf("import file: \"%s\", pattern: \"%s\", W2A16(strFile), W2A16(strPattern));
//! @endcode
MLPI_API MLPIRESULT mlpiParameterImportExportGetInfo(const MLPIHANDLE connection, const PROCESSHANDLE process, WCHAR16* pattern, ULONG numElementsPattern, WCHAR16* file, ULONG numElementsFile);


//! @ingroup ParamLibAux
//! This function stops a given parameter process
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   process            Handle of process
//! @return                         Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPIHANDLE process;
//! result = mlpiParameterImportFileStartProcess(connection, L"/ata0b/import.par", L"", &process)
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start import with error code: %08X", result);
//!   return;
//! }
//! 
//! result = mlpiParameterImportExportAbort(connection, process);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to abort import with error code: %08X", result);
//!   return;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiParameterImportExportAbort(const MLPIHANDLE connection, const PROCESSHANDLE process);


//! @ingroup ParamLibAux
//! This function initializes a monitoring of parameters on a write access of the operation data. Not the modification of the operation data
//! is monitored, but the write access. A process is started in the control. The status of this process can be read via the function 
//! @ref mlpiParameterWriteAccessStatus. The process can be stopped via the function @ref mlpiParameterWriteAccessAbort.
//! @note
//! A process is related to the connection that initialized the process. If the connection is terminated also the process
//! will be stopped and the handle is not longer valid. \n
//! If the option 'auto_reconnect' is used for the connection and the connection is terminated, the process will also be stopped.
//! A new monitoring will not automatically be initialized. 
//! @param[in]   connection           Handle for multiple connections.
//! @param[in]   writeAccess          Structure which conntains the information of the parameters to be monitored.
//! @param[in]   numElements          Number of MlpiParamWriteAccess elements available in 'writeAccess'.
//! @param[out]  process              Handle of created process
//! @return                           Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! // initialize write access monitoring of parameters C-0-0400 and A-0-0024 of axis 5
//! MLPIRESULT result = S_OK;
//! PROCESSHANDLE process = 0;
//! MlpiParamWriteAccess writeAccess[] = {{0, MLPI_SIDN_C(400)}, {MLPI_ADDRESS(MLPI_ADDRESS_MODE_LOGICAL,0,5),  MLPI_SIDN_A(24)}};
//! result = mlpiParameterWriteAccessSetup(connection, writeAccess, _countof(writeAccess), &process);
//! if(MLPI_FAILED(result))
//! {
//!   printf("error during initialization of write access monitoring. result:0x%08X", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteAccessSetup(const MLPIHANDLE connection, MlpiParamWriteAccess* writeAccess, const ULONG numElements, PROCESSHANDLE* process);


//! @ingroup ParamLibAux
//! This function reads the status of a write access monitoring process. The process has be initialized via the function @ref mlpiParameterWriteAccessSetup.
//! The written parameters will only be reseted if this function succeded.
//!
//! @param[in]   connection           Handle for multiple connections.
//! @param[in]   process              Handle of process
//! @param[in]   waitForWriteAccess   Set this value 'true' to call this function synchronously. The function returns in this case if one monitored parameter has been written.
//! @param[in]   timeout              Timeout in milliseconds if the value of 'waitForWriteAccess' is 'true'. Use MLPI_INFINITE for infinite wait.
//! @param[in]   writeAccess          Structure to store the information of (only) the written parameters.
//! @param[out]  numElements          Number of MlpiParamWriteAccess elements available in 'writeAccess'.
//! @param[out]  numElementsRet       Number of elements used.
//! @return                           Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! // initialize write access monitoring of parameters C-0-0400 and A-0-0024 of axis 5
//! MLPIRESULT result = S_OK;
//! PROCESSHANDLE process = 0;
//! MlpiParamWriteAccess writeAccess[] = {{0, MLPI_SIDN_C(400)}, {MLPI_ADDRESS(MLPI_ADDRESS_MODE_LOGICAL,0,5),  MLPI_SIDN_A(24)}};
//! result = mlpiParameterWriteAccessSetup(connection, writeAccess, _countof(writeAccess), &process);
//! if(MLPI_FAILED(result))
//! {
//!   printf("error during initialization of write access monitoring. result:0x%08X", result);
//!   return result;
//! }
//!
//! // modify parameter C-0-0400
//! result = mlpiParameterWriteDataUlong(connection, 0, MLPI_SIDN_C(400), (ULONG) 4000);
//! if(MLPI_FAILED(result))
//! {
//!   printf("error writing parameter C-0-0400. result:0x%08X", result);
//!   return result;
//! }
//!
//! // get status of process
//! ULONG   numElementsRet=0;
//! WCHAR16 idn[PARAM_MAX_IDN_STRING_LENGTH] = {L'0'};
//! memset(idn, 0, _countof(idn));
//! result = mlpiParameterWriteAccessStatus(connection, process, FALSE, 0, writeAccess, _countof(writeAccess), &numElementsRet);
//! if(MLPI_FAILED(result))
//! {
//!   printf("error getting status of write access monitoring. result:0x%08X", result);
//!   return result;
//! }
//! for (ULONG i=0; numElementsRet>i; ++i)
//! {
//!   printf("\nParameter %s has been written", utilParameterParseIdn(writeAccess[i].sidn, idn, _countof(idn)));
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteAccessStatus(const MLPIHANDLE connection, const PROCESSHANDLE process, const BOOL8 waitForWriteAccess, const ULONG timeout, MlpiParamWriteAccess* writeAccess, const ULONG numElements, ULONG* numElementsRet);


//! @ingroup ParamLibAux
//! This function aborts a write access monitoring process.
//! @param[in]   connection         Handle for multiple connections.
//! @param[in]   process            Handle of process
//! @return                         Return value indicating success (>=0) or error (<0).\n
//!
//! @par Example:
//! @code
//! MLPIRESULT result = S_OK;
//! PROCESSHANDLE process = 0;
//! MlpiParamWriteAccess writeAccess[] = {{0, MLPI_SIDN_C(400)}, {MLPI_ADDRESS(MLPI_ADDRESS_MODE_LOGICAL,0,5),  MLPI_SIDN_A(24)}};
//! result = mlpiParameterWriteAccessSetup(connection, writeAccess, _countof(writeAccess), &process);
//! if(MLPI_FAILED(result))
//! {
//!   printf("error during initialization of write access monitoring. result:0x%08X", result);
//!   return result;
//! }
//!
//! // cancel process
//! result = mlpiParameterWriteAccessAbort(connection, process);
//! if(MLPI_FAILED(result))
//! {
//!   printf("error canceling of write access monitoring. result:0x%08X", result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiParameterWriteAccessAbort(const MLPIHANDLE connection, const PROCESSHANDLE process);


#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIPARAMETERLIB_H__

