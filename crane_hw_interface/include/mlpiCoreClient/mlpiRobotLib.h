#ifndef __MLPIROBOTLIB_H__
#define __MLPIROBOTLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiRobotLib.h>
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
//! @author     DC-IA/EAM1 (GB, SK, JR)
//!
//! @copyright  Bosch Rexroth Corporation http://www.boschrexroth.com/oce
//!
//! @version    1.22.0
//!
//! @date       2013
//
// -----------------------------------------------------------------------


//! @addtogroup RobotLib RobotLib
//! @{
//! @brief This robot library provides functionality for defining a robot, commanding
//! motion and getting robot information.
//!
//! @note You find a more detailed description of the concept of robot in the IndraWorks help system
//! of the XLC/MLC product.
//!
//! The following functions all deal with the motion of a robot. In general, "robot" is used to address a robot.
//! Each motion command usually uses a structure to pass on the information about
//! a particular move. The command will be executed in the control and returns a handle. This handle must
//! be used in future requests for getting status information. The handle and the robot reference a particular
//! motion command. One robot can have multiple active commands (for example, MoveLinearAbs is one and a MoveLinearRel is
//! commanded). A previous commanded move will not be interrupted by a currently executed move.
//! All moves will be executed consecutively.
//!
//! @note The RobotLib functions trace their debug information mainly into the module MLPI_ROBOT_LIB
//!       and in addition into the modules RCF*, KIC, KIN_CMD_LOGGER and MLPI_BASE_MODULES. For
//!       further information, see also the detailed description of the library @ref TraceLib and the
//!       notes about @ref sec_TraceViewer.
//!
//! @}

//! @addtogroup RobotLibVersionPermission Version and Permission
//! @ingroup RobotLib
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_ROBOTLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_ROBOTLIB_PERMISSION_ALWAYS cannot be blocked.
//!
//! @par List of permissions of mlpiRobotLib using in accounts.xml
//! - MLPI_ROBOTLIB_PERMISSION_ALL
//! - MLPI_ROBOTLIB_PERMISSION_INFO
//! - MLPI_ROBOTLIB_PERMISSION_CLEAR
//! - MLPI_ROBOTLIB_PERMISSION_CONFIG
//! - MLPI_ROBOTLIB_PERMISSION_MOVE
//! - MLPI_ROBOTLIB_PERMISSION_TEACHIN
//!
//! <TABLE>
//! <TR><TH>           Function                                  </TH><TH> Server version </TH><TH> Permission                         </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotReset                       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CLEAR"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetBeltConfiguration        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetRotaryTableConfiguration </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetCartesianTransform       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetCylindricTransform       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetBeltErrorReaction        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetBeltErrorReaction        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveLinearAbs               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveCircularAbs             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveCircularRel             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveLinearRel               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveDirectAbs               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveDirectRel               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMoveJumpAbs                 </TD><TD> 1.14.0.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSyncOnWithLimits            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSyncOffWithLimits           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotOpenCyclicChannel           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotWriteCyclicChannel          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotStop                        </TD><TD> 1.0.17.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotInterrupt                   </TD><TD> 1.0.17.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotContinue                    </TD><TD> 1.0.17.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMotionGetStatus             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotReadCyclicChannel           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotReadPos                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotMovePoint                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetPerformanceEnable        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetPerfomanceResult         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetMechanicData             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetTransform                </TD><TD> 1.0.18.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetBeltConfiguration        </TD><TD> 1.0.18.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotAddAllAxisToGroup           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotRemAllAxisFromGroup         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetKinematicsValues         </TD><TD> 1.1.1.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetDiagnosisText            </TD><TD> 1.1.1.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetKinematicsUnits          </TD><TD> 1.1.1.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTransformPoint              </TD><TD> 1.1.1.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotWait                        </TD><TD> 1.2.1.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetSafeZone                 </TD><TD> 1.3.1.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotJogStep                     </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotJogCont                     </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotStopCmd                     </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_MOVE"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetBeltDesyncRelConfig      </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetBeltDesyncRelConfig      </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInWritePoint           </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInReadPoint            </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInReadNextPoint        </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInDeletePoints         </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInSavePointFile        </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInLoadPointFile        </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotTeachInDeletePointFile      </TD><TD> 1.4.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_TEACHIN" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetCoordinateSystemInfo     </TD><TD> 1.6.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetStatusSummary            </TD><TD> 1.6.0.0        </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetSafeZones                </TD><TD> 1.11.0.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetMoveId                   </TD><TD> 1.14.0.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotSetMoveId                   </TD><TD> 1.14.0.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_CONFIG"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiRobotGetCmdInfo                  </TD><TD> 1.16.0.0       </TD><TD> "MLPI_ROBOTLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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
//! @}

//! @defgroup RobotLibMovement Movement functions
//! @ingroup RobotLib
//! @{
//! @brief The following functions deal with movement of groups. In general, "groups" is used to address a group.
//! @}

//! @defgroup RobotLibConfig Configuration functions
//! @ingroup RobotLib
//! @{
//! @brief The following functions all deal with administration of groups.
//! @}

//! @defgroup RobotLibInfo Robot information
//! @ingroup RobotLib
//! @{
//! @brief This group of functions allows the user to get group related information
//! @}

//! @defgroup RobotLibTeachIn Robot TeachIn
//! @ingroup RobotLib
//! @{
//! @brief This group of functions allows the user to use the TeachIn functions
//! @}

//! @addtogroup RobotLibStructTypes Structs, types, ...
//! @ingroup RobotLib
//! @{
//! @brief List of used types, enumerations, structures and more...



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"

#include "mlpiMotionLib.h"   // robot lib needs the motion functionality


// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_ROBOT_COORDINATES_IN_POINT           (16) //!< number of coordinates in a point
#define MLPI_ROBOT_MAX_POS_AXIS                   (3)  //!< number of positioning axes
#define MLPI_ROBOT_AXIS_COUNT                     (16) //!< number of axis of a robot
#define MLPI_ROBOT_MAX_UNIT_LEN                   (12) //!< maximum length of unit name
#define MLPI_ROBOT_MAX_CSNAME_LEN                 (30) //!< maximum length of coordinate system name
#define MLPI_ROBOT_MAX_COORDINATE_SYSTEMS         (19) //!< maximum number of coordinate systems
#define MLPI_ROBOT_MAX_UNITS_LEN                  (MLPI_ROBOT_MAX_UNIT_LEN * MLPI_ROBOT_AXIS_COUNT)
#define MLPI_ROBOT_MAX_SAFEZONES                  (32) //!< maximum number of safezones
#define MLPI_ROBOT_MAX_TEACH_NAME_LEN             (21) //!< maximum name length of teach points

//-----------------------------------------------------------------------
//GLOBAL CONSTANTS
//-----------------------------------------------------------------------

//! This enumeration defines the types of axes used in a robot. This is parameter K-0-0008.
typedef enum MlpiRobotAxisTypes
{
  MLPI_ROBOT_AXISTYPE_NONE         = 0,  //!< No axis configured
  MLPI_ROBOT_AXISTYPE_POSITIONING  = 8,  //!< axis is used as a positioning axis
  MLPI_ROBOT_AXISTYPE_ORIENTATION  = 16, //!< axis is used as an orientation axis
  MLPI_ROBOT_AXISTYPE_BELT         = 32, //!< axis is used as a belt axis
}MlpiRobotAxisTypes;

//! This enumeration defines the set mode of movement commands
typedef enum MlpiRobotSetMode
{
  MLPI_ROBOT_SETMODE_TO      = 0,  //!< no blending
  MLPI_ROBOT_SETMODE_VIA     = 1,  //!< blending
}MlpiRobotSetMode;

//! This enumeration defines coordinate systems of a robot
typedef enum MlpiRobotCoordinateSystem
{
  MLPI_ROBOT_CS_MCS       = 0x0000,  //!< machine coordinate system
  MLPI_ROBOT_CS_ACS       = 0x0001,  //!< axis coordinate system
  MLPI_ROBOT_CS_BCS       = 0x0002,  //!< base coordinate system
  MLPI_ROBOT_CS_PCS1      = 0x0003,  //!< product coordinate system 1
  MLPI_ROBOT_CS_PCS2      = 0x0103,  //!< product coordinate system 2
  MLPI_ROBOT_CS_PCS3      = 0x0203,  //!< product coordinate system 3
  MLPI_ROBOT_CS_PCS4      = 0x0303,  //!< product coordinate system 4
  MLPI_ROBOT_CS_PCS5      = 0x0403,  //!< product coordinate system 5
  MLPI_ROBOT_CS_PCS6      = 0x0503,  //!< product coordinate system 6
  MLPI_ROBOT_CS_PCS7      = 0x0603,  //!< product coordinate system 7
  MLPI_ROBOT_CS_PCS8      = 0x0703,  //!< product coordinate system 8
  MLPI_ROBOT_CS_PCS9      = 0x0803,  //!< product coordinate system 9
  MLPI_ROBOT_CS_PCS10     = 0x0903,  //!< product coordinate system 10
  MLPI_ROBOT_CS_PCS11     = 0x0A03,  //!< product coordinate system 11
  MLPI_ROBOT_CS_PCS12     = 0x0B03,  //!< product coordinate system 12
  MLPI_ROBOT_CS_PCS13     = 0x0C03,  //!< product coordinate system 13
  MLPI_ROBOT_CS_PCS14     = 0x0D03,  //!< product coordinate system 14
  MLPI_ROBOT_CS_PCS15     = 0x0E03,  //!< product coordinate system 15
  MLPI_ROBOT_CS_PCS16     = 0x0F03,  //!< product coordinate system 16
}MlpiRobotCoordinateSystem;

//! This enumeration defines the type of the coordinate system
typedef enum MlpiRobotCSType
{
  MLPI_ROBOT_CARTESIAN = 0,  //!< cartesian coordinate system
  MLPI_ROBOT_CYLINDRIC = 1,  //!< cylindric coordinate system
}MlpiRobotCSType;



//! This enumeration defines the point type
typedef enum MlpiRobotPointType
{
  MLPI_ROBOT_POINT_TYPE_PREP  = 0,   //!< position of preparation
  MLPI_ROBOT_POINT_TYPE_EXEC  = 1,   //!< position of execution
}MlpiRobotPointType;


//! This enumeration defines the slope types of movement commands
typedef enum MlpiRobotSlopeType
{
  MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE = 0,  //!< blending by starting next command early
  MLPI_ROBOT_SLOPETYPE_PROGR_SLOPE = 1,  //!< blending using a circle (deprecated)
  MLPI_ROBOT_SLOPETYPE_CONT_SLOPE  = 2,  //!< blending using a spline
}MlpiRobotSlopeType;

//! This enumeration defines the error reaction mode of a belt
typedef enum MlpiRobotSyncErrorReactionMode
{
  MLPI_ROBOT_BELT_ERROR_NO_SPECIFIC_REACTION = 0, //!< If an error occurs, no specific belt synchronous error reaction will happen. The kinematics stays synchronous to the belt.
  MLPI_ROBOT_BELT_ERROR_DESYNC_OVER_TIME     = 1, //!< If an error occurs, the kinematics desynchronizes from the belt over time.
  MLPI_ROBOT_BELT_ERROR_DESYNC_WITH_LIMITS   = 2, //!< If an error occurs, the kinematics desynchronizes from the belt over limits.
}MlpiRobotSyncErrorReactionMode;

//! This enum defines the type of a cyclic position command
typedef enum MlpiRobotCyclicMode
{
  MLPI_ROBOT_CYCLIC_MODE_POS_ABS       = 0, //!< Absolute position
  MLPI_ROBOT_CYCLIC_MODE_POS_REL       = 1, //!< Relative position delta to last position
  MLPI_ROBOT_CYCLIC_MODE_VEL           = 2, //!< Velocity in robot units
}MlpiRobotCyclicMode;

//! This enum defines the command for a mechanical object
typedef enum MlpiRobotMechCmd
{
  MLPI_ROBOT_MECH_CREATE_ROD       = 0,  //!< create a rod
  MLPI_ROBOT_MECH_CREATE_SPHERE    = 1,  //!< create a sphere
  MLPI_ROBOT_MECH_POSITION         = 2,  //!< change position of object to (x,y,z)
  MLPI_ROBOT_MECH_DIRECTION        = 3,  //!< change direction of object to (x,y,z)
  MLPI_ROBOT_MECH_SIZE             = 4,  //!< change size of object to (x,y,z)
  MLPI_ROBOT_MECH_CREATE_CUBOID    = 5,  //!< create a cuboid
  MLPI_ROBOT_MECH_SET_MCS          = 6,  //!< set reference coordinate of object system to MCS
  MLPI_ROBOT_MECH_ROTATE           = 7,  //!< change rotation of object to (A,B,C)
}MlpiRobotMechCmd;

//! This enum defines the type of a save zone
typedef enum MlpiRobotSafeZoneType
{
  MLPI_ROBOT_SAFE_ZONE_TYPE_DEACTIVATED = 0,  //!< zone deactivated
  MLPI_ROBOT_SAFE_ZONE_TYPE_INSIDE_OK   = 1,  //!< points within this zone are allowed
  MLPI_ROBOT_SAFE_ZONE_TYPE_OUTSIDE_OK  = 2,  //!< points outside of this zone are allowed
}MlpiRobotSafeZoneType;

//! This enum defines the possible options for writing a point while the TeachIn functions
typedef enum MlpiRobotTeachInWriteOption
{
  MLPI_ROBOT_TEACH_IN_WRITE_POINT_SIMPLE       = 0x000,
  MLPI_ROBOT_TEACH_IN_WRITE_POINT_OVERWRITE    = 0x001,
  MLPI_ROBOT_TEACH_IN_WRITE_POINT_BELT_VALUES  = 0x002,
}MlpiRobotTeachInWriteOption;

//! This enum defines the options, how a file can be loaded or saved
typedef enum MlpiRobotTeachInPointFileOption
{
  MLPI_ROBOT_TEACH_IN_NEW = 0,
  MLPI_ROBOT_TEACH_IN_APPEND,
  MLPI_ROBOT_TEACH_IN_APPEND_OVERWRITE,
}MlpiRobotTeachInPointFileOption;

//! This enum defines the type of information which should be read
typedef enum MlpiRobotCmdInfoType
{
  MLPI_ROBOT_CMD_INFO_TYPE_ALL            = 0,  //!< read all types of information available in this enumeration
  MLPI_ROBOT_CMD_DATA_TYPE_PROGRESS       = 1,  //!< read the progress of move-commands in the block buffer
  MLPI_ROBOT_CMD_DATA_TYPE_TOTAL_DISTANCE = 2,  //!< read the total distance of move-commands in the block buffer
  MLPI_ROBOT_CMD_DATA_TYPE_UID            = 3,  //!< read the unique identifier of a command
  MLPI_ROBOT_CMD_DATA_TYPE_STATE          = 4,  //!< read the state of a command
}MlpiRobotCmdInfoType;


// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif


//! @typedef MlpiRobotMotionStatus
//! @brief This structure defines the status of a robot movement command
//! @details Elements of struct MlpiRobotMotionStatus
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element         </TH><TH> Description                                               </TH></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> done            </TD><TD> @c TRUE when command is completed                         </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> active          </TD><TD> @c TRUE as long as command is active                      </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> inbuffer        </TD><TD> @c TRUE when command is in buffer                         </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> commandaborted  </TD><TD> @c TRUE when command has been aborted by another command  </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> error           </TD><TD> @c TRUE when motion command issued an error               </TD></TR>
//! <TR><TD id="st_t"> USHORT </TD><TD id="st_e"> errorID         </TD><TD> Identifier of error                                       </TD></TR>
//! <TR><TD id="st_t"> USHORT </TD><TD id="st_e"> table           </TD><TD> Table identifier of error                                 </TD></TR>
//! <TR><TD id="st_t"> ULONG  </TD><TD id="st_e"> additional1     </TD><TD> Additional diagnosis number1                              </TD></TR>
//! <TR><TD id="st_t"> ULONG  </TD><TD id="st_e"> additional2     </TD><TD> Additional diagnosis number2                              </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMotionStatus
{
  BOOL8  MLPI_STRUCT_ALIGN_BOOL8            done;             //! @c TRUE when command is completed
  BOOL8  MLPI_STRUCT_ALIGN_BOOL8            active;           //! @c TRUE as long as command is active
  BOOL8  MLPI_STRUCT_ALIGN_BOOL8            inbuffer;         //! @c TRUE when command is in buffer
  BOOL8  MLPI_STRUCT_ALIGN_BOOL8            commandaborted;   //! @c TRUE when command has been aborted by another command
  BOOL8  MLPI_STRUCT_ALIGN_BOOL8            error;            //! @c TRUE when motion command issued an error
  USHORT MLPI_STRUCT_ALIGN_USHORT           errorID;          //! Identifier of error
  USHORT MLPI_STRUCT_ALIGN_USHORT           table;            //! Table identifier of error
  ULONG  MLPI_STRUCT_ALIGN_ULONG            additional1;      //! Additional diagnosis number1
  ULONG  MLPI_STRUCT_ALIGN_ULONG            additional2;      //! Additional diagnosis number2
}MlpiRobotMotionStatus;


//! @typedef MlpiRobotPoint
//! @brief This structure defines a point.
//! @details Elements of struct MlpiRobotPoint
//! <TABLE>
//! <TR><TH>           Type            </TH><TH>           Element          </TH><TH> Description             </TH></TR>
//! <TR><TD id="st_t"> array of DOUBLE </TD><TD id="st_e"> coordinate       </TD><TD> Array of coordinates of a point with MLPI_ROBOT_COORDINATES_IN_POINT elements. </TD></TR>
//! </TABLE>
typedef struct MlpiRobotPoint
{
  DOUBLE MLPI_STRUCT_ALIGN_DOUBLE coordinate[MLPI_ROBOT_COORDINATES_IN_POINT]; // Array of coordinates of a point with MLPI_ROBOT_COORDINATES_IN_POINT elements
}MlpiRobotPoint;

//! @typedef MlpiRobotMoveLinearAbsolute
//! @brief This structure defines the parameters of a linear absolute movement
//! @details Elements of struct MlpiRobotMoveLinearAbsolute
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> point          </TD><TD> End Point of movement                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                             </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveLinearAbsolute
{
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT point;                    //!< End Point of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE velocity;                 //!< Maximum velocity of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE acceleration;             //!< Maximum acceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE deceleration;             //!< Maximum deceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE jerk;                     //!< Maximum jerk of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;           //!< Commanded radius of blending movement
  MlpiRobotSetMode          MLPI_STRUCT_ALIGN_ENUM   setMode;                  //!< set mode
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM   coordSystem;              //!< coordinate system
  MlpiRobotSlopeType        MLPI_STRUCT_ALIGN_ENUM   slopeType;                //!< slope type
} MlpiRobotMoveLinearAbsolute;

//! @typedef MlpiRobotMoveLinearRelative
//! @brief This structure defines the parameters of a linear relative movement.
//! @details Elements of struct MlpiRobotMoveLinearRelative
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> distance       </TD><TD> Distance of movement                   </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                             </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveLinearRelative
{
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT distance;         //!< Distance of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE velocity;         //!< Maximum velocity of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE acceleration;     //!< Maximum acceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;   //!< Commanded radius of blending movement
  MlpiRobotSetMode          MLPI_STRUCT_ALIGN_ENUM   setMode;          //!< set mode
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM   coordSystem;      //!< coordinate system
  MlpiRobotSlopeType        MLPI_STRUCT_ALIGN_ENUM   slopeType;        //!< slope type
} MlpiRobotMoveLinearRelative;

//! @typedef MlpiRobotMoveDirectAbsolute
//! @brief This structure defines the parameters of an absolute direct movement.
//! @details Elements of struct MlpiRobotMoveDirectAbsolute
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> point          </TD><TD> End Point of movement                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                             </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveDirectAbsolute
{
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT point;            //!< End Point of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE velocity;         //!< Maximum velocity of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE acceleration;     //!< Maximum acceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;   //!< Commanded radius of blending movement
  MlpiRobotSetMode          MLPI_STRUCT_ALIGN_ENUM   setMode;          //!< set mode
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM   coordSystem;      //!< coordinate system
  MlpiRobotSlopeType        MLPI_STRUCT_ALIGN_ENUM   slopeType;        //!< slope type
} MlpiRobotMoveDirectAbsolute;

//! @typedef MlpiRobotMoveDirectRelative
//! @brief This structure defines the parameters of a direct relative movement.
//! @details Elements of struct MlpiRobotMoveDirectRelative
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> distance       </TD><TD> Distance  of movement                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                             </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveDirectRelative
{
  MlpiRobotPoint             MLPI_STRUCT_ALIGN_STRUCT distance;         //!< Distance of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE velocity;         //!< Maximum velocity of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE acceleration;     //!< Maximum acceleration of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;   //!< Commanded radius of blending movement
  MlpiRobotSetMode           MLPI_STRUCT_ALIGN_ENUM   setMode;          //!< set mode
  MlpiRobotCoordinateSystem  MLPI_STRUCT_ALIGN_ENUM   coordSystem;      //!< coordinate system
  MlpiRobotSlopeType         MLPI_STRUCT_ALIGN_ENUM   slopeType;        //!< slope type
} MlpiRobotMoveDirectRelative;

//! @typedef MlpiRobotMoveCircularAbsolute
//! @brief This structure defines the parameters of an absolute circular movement.
//! @details Elements of struct MlpiRobotMoveCircularAbsolute
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> auxPoint       </TD><TD> Aux Point of movement                  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> endPoint       </TD><TD> End Point of movement                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                             </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveCircularAbsolute
{
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT auxPoint;          //!< Aux Point of movement
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT endPoint;          //!< End Point of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE velocity;          //!< Maximum velocity of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE acceleration;      //!< Maximum acceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE deceleration;      //!< Maximum deceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE jerk;              //!< Maximum jerk of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;    //!< Commanded radius of blending movement
  MlpiRobotSetMode          MLPI_STRUCT_ALIGN_ENUM   setMode;           //!< set mode
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM   coordSystem;       //!< coordinate system
  MlpiRobotSlopeType        MLPI_STRUCT_ALIGN_ENUM   slopeType;         //!< slope type
} MlpiRobotMoveCircularAbsolute;

//! @typedef MlpiRobotMoveCircularRelative
//! @brief This structure defines the parameters of a circular relative movement.
//! @details Elements of struct MlpiRobotMoveCircularRelative
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> auxPoint       </TD><TD> Aux Point of movement                  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> endPoint       </TD><TD> End Point of movement                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                             </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveCircularRelative
{
  MlpiRobotPoint             MLPI_STRUCT_ALIGN_STRUCT auxPoint;          //!< Aux Point of movement
  MlpiRobotPoint             MLPI_STRUCT_ALIGN_STRUCT endPoint;          //!< End Point of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE velocity;          //!< Maximum velocity of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE acceleration;      //!< Maximum acceleration of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE deceleration;      //!< Maximum deceleration of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE jerk;              //!< Maximum jerk of movement
  DOUBLE                     MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;    //!< Commanded radius of blending movement
  MlpiRobotSetMode           MLPI_STRUCT_ALIGN_ENUM   setMode;           //!< set mode
  MlpiRobotCoordinateSystem  MLPI_STRUCT_ALIGN_ENUM   coordSystem;       //!< coordinate system
  MlpiRobotSlopeType         MLPI_STRUCT_ALIGN_ENUM   slopeType;         //!< slope type
} MlpiRobotMoveCircularRelative;

//! @typedef MlpiRobotMoveJumpAbsolute
//! @brief This structure defines the parameters of a jump absolute movement
//! @details Elements of struct MlpiRobotMoveJumpAbsolute
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                                         </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> point          </TD><TD> End Point of movement                               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement                        </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement                    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement                    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement                            </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode           </TD><TD id="st_e"> setMode        </TD><TD> set mode                                            </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                                   </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSlopeType         </TD><TD id="st_e"> slopeType      </TD><TD> slope type                                          </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> startHeight    </TD><TD> Minimal start height before the horizontal movement </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> maxHeight      </TD><TD> Maximum height of movement                          </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> endHeight      </TD><TD> Minimal end height after the horizontal movement    </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMoveJumpAbsolute
{
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT point;                    //!< End Point of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE velocity;                 //!< Maximum velocity of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE acceleration;             //!< Maximum acceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE deceleration;             //!< Maximum deceleration of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE jerk;                     //!< Maximum jerk of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;           //!< Commanded radius of blending movement
  MlpiRobotSetMode          MLPI_STRUCT_ALIGN_ENUM   setMode;                  //!< set mode
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM   coordSystem;              //!< coordinate system
  MlpiRobotSlopeType        MLPI_STRUCT_ALIGN_ENUM   slopeType;                //!< slope type
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE startHeight;              //!< Minimal start height before the horizontal movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE maxHeight;                //!< Maximum height of movement
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE endHeight;                //!< Minimal end height after the horizontal movement
} MlpiRobotMoveJumpAbsolute;

//! @typedef MlpiRobotSetBeltConfiguration
//! @brief This structure defines the parameters for the configuration of a belt.
//! @details Elements of struct MlpiRobotSetBeltConfiguration
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef         </TD><TD id="st_e"> belt           </TD><TD> Reference to the belt                         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> posX           </TD><TD> X position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> posY           </TD><TD> Y position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> posZ           </TD><TD> Z position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> rotA           </TD><TD> rotation of belt around x axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> rotB           </TD><TD> rotation of belt around y axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> rotC           </TD><TD> rotation of belt around z axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> begin          </TD><TD> begin length of belt synchronous working area </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> total          </TD><TD> total length of belt synchronous working area </TD></TR>
//! </TABLE>
typedef struct MlpiRobotSetBeltConfiguration
{
  MlpiAxisRef          MLPI_STRUCT_ALIGN_STRUCT belt;          //!< Reference to the belt
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE posX;          //!< X position of the belt in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE posY;          //!< Y position of the belt in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE posZ;          //!< Z position of the belt in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE rotA;          //!< rotation of belt around x axis in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE rotB;          //!< rotation of belt around y axis in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE rotC;          //!< rotation of belt around z axis in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE begin;         //!< begin length of belt synchronous working area
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE total;         //!< total length of belt synchronous working area
}MlpiRobotSetBeltConfiguration;

//! @typedef MlpiRobotGetBeltConfiguration
//! @brief This structure defines the belt configuration
//! @details Elements of struct MlpiRobotGetBeltConfiguration
//! <TABLE>
//! <TR><TH>           Type                </TH><TH> Direction </TH><TH>           Element        </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef         </TD><TD> [in]      </TD><TD id="st_e"> belt           </TD><TD> Reference to the belt                         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> posX           </TD><TD> X position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> posY           </TD><TD> Y position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> posZ           </TD><TD> Z position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> rotA           </TD><TD> rotation of belt around x axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> rotB           </TD><TD> rotation of belt around y axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> rotC           </TD><TD> rotation of belt around z axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> begin          </TD><TD> begin length of belt synchronous working area </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD> [out]     </TD><TD id="st_e"> total          </TD><TD> total length of belt synchronous working area </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCSType     </TD><TD> [out]     </TD><TD id="st_e"> csType         </TD><TD> Type of coordinate system                     </TD></TR>
//! </TABLE>
typedef struct MlpiRobotGetBeltConfiguration
{
  MlpiAxisRef          MLPI_STRUCT_ALIGN_STRUCT belt;          //!< Reference to the belt
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE posX;          //!< X position of the belt in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE posY;          //!< Y position of the belt in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE posZ;          //!< Z position of the belt in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE rotA;          //!< rotation of belt around x axis in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE rotB;          //!< rotation of belt around y axis in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE rotC;          //!< rotation of belt around z axis in MCS
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE begin;         //!< begin length of belt synchronous working area
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE total;         //!< total length of belt synchronous working area
  MlpiRobotCSType      MLPI_STRUCT_ALIGN_ENUM   csType;        //!< Type of coordinate system
}MlpiRobotGetBeltConfiguration;

//! @typedef MlpiRobotSetBeltErrorReaction
//! @brief This structure defines the parameters for the configuration of the error reaction of a belt.
//! @details Elements of struct MlpiRobotSetBeltErrorReaction
//! <TABLE>
//! <TR><TH>           Type                             </TH><TH>           Element        </TH><TH> Description                                                                                                               </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef                      </TD><TD id="st_e"> belt           </TD><TD> Reference to the belt                                                                                                     </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                           </TD><TD id="st_e"> time           </TD><TD> Defines the time in seconds to stop the belt synchronous movement when error reaction mode of belt is time                </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                           </TD><TD id="st_e"> decelaration   </TD><TD> Defines the deceleration to stop the belt synchronous movement when error reaction mode is desynchronization with limits  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                           </TD><TD id="st_e"> jerk           </TD><TD> Defines the jerk to stop the belt synchronous movement when error reaction mode is desynchronization with limits          </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSyncErrorReactionMode   </TD><TD id="st_e"> errorType      </TD><TD> Type of error reaction                                                                                                    </TD></TR>
//! </TABLE>
typedef struct MlpiRobotSetBeltErrorReaction
{
  MlpiAxisRef                    MLPI_STRUCT_ALIGN_STRUCT belt;          //!< Reference to the axis
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE time;          //!< Defines the time in seconds to stop the belt synchronous movement when error reaction mode of belt is time
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE decelaration;  //!< Defines the deceleration to stop the belt synchronous movement when error reaction mode is desynchronization with limits
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE jerk;          //!< Defines the jerk to stop the belt synchronous movement when error reaction mode is desynchronization with limits
  MlpiRobotSyncErrorReactionMode MLPI_STRUCT_ALIGN_ENUM   errorType;     //!< Type of error reaction
}MlpiRobotSetBeltErrorReaction;

//! @typedef MlpiRobotGetBeltErrorReaction
//! @brief This structure defines the parameters of the configuration of the error reaction of a belt.
//! @details Elements of struct MlpiRobotSetBeltErrorReaction
//! <TABLE>
//! <TR><TH>           Type                             </TH><TH> Direction </TH><TH>           Element        </TH><TH> Description                                                                                                               </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef                      </TD><TD> [in]      </TD><TD id="st_e"> belt           </TD><TD> Reference to the belt                                                                                                     </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                           </TD><TD> [out]     </TD><TD id="st_e"> time           </TD><TD> Defines the time in seconds to stop the belt synchronous movement when error reaction mode of belt is time                </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                           </TD><TD> [out]     </TD><TD id="st_e"> deceleration   </TD><TD> Defines the deceleration to stop the belt synchronous movement when error reaction mode is desynchronization with limits  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                           </TD><TD> [out]     </TD><TD id="st_e"> jerk           </TD><TD> Defines the jerk to stop the belt synchronous movement when error reaction mode is desynchronization with limits          </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSyncErrorReactionMode   </TD><TD> [out]     </TD><TD id="st_e"> errorType      </TD><TD> Type of error reaction                                                                                                    </TD></TR>
//! </TABLE>
typedef struct MlpiRobotGetBeltErrorReaction
{
  MlpiAxisRef                    MLPI_STRUCT_ALIGN_STRUCT belt;          //!< Reference to the axis
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE time;          //!< Defines the time to stop the belt synchronous movement when error reaction mode of belt is time
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE deceleration;  //!< Defines the deceleration to stop the belt synchronous movement when error reaction mode is desynchronization with limits
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE jerk;          //!< Defines the jerk to stop the belt synchronous movement when error reaction mode is desynchronization with limits
  MlpiRobotSyncErrorReactionMode MLPI_STRUCT_ALIGN_ENUM   errorType;     //!< Type of error reaction
}MlpiRobotGetBeltErrorReaction;

//! @typedef MlpiRobotSyncOffWithLimits
//! This structure defines the parameters for configuring a desynchronizing movement of a belt with limits
//! @details Elements of struct MlpiRobotSyncOffWithLimits
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef         </TD><TD id="st_e"> belt           </TD><TD> Reference to the belt                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> blendingRadius </TD><TD> Maximum radius of blending movement    </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode    </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! </TABLE>
typedef struct MlpiRobotSyncOffWithLimits
{
  MlpiAxisRef                    MLPI_STRUCT_ALIGN_STRUCT belt;          //!< Reference to the axis
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE deceleration;  //!< Maximum deceleration of movement
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE jerk;          //!< Maximum jerk of movement
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;//!< Maximum radius of blending movement
  MlpiRobotSetMode               MLPI_STRUCT_ALIGN_ENUM   setMode;       //!< set mode
}MlpiRobotSyncOffWithLimits;

//! @typedef MlpiRobotSyncOnWithLimits
//! This structure defines the parameters for configuring a synchronizing movement of a belt with limits
//! @details Elements of struct MlpiRobotSyncOnWithLimits
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef         </TD><TD id="st_e"> belt           </TD><TD> Reference to the belt                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! </TABLE>
typedef struct MlpiRobotSyncOnWithLimits
{
  MlpiAxisRef                   MLPI_STRUCT_ALIGN_STRUCT belt;        //!< Reference to the axis
  DOUBLE                        MLPI_STRUCT_ALIGN_DOUBLE acceleration;//!< Maximum acceleration of movement
  DOUBLE                        MLPI_STRUCT_ALIGN_DOUBLE jerk;        //!< Maximum jerk of movement
}MlpiRobotSyncOnWithLimits;

//! @typedef MlpiRobotSetTrafoParameter
//! @brief This structure defines the parameters for configuring a product coordinate system
//! @details Elements of struct MlpiRobotSetTrafoParameter
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> Reference to the coordinate system            </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> posX           </TD><TD> X position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> posY           </TD><TD> Y position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> posZ           </TD><TD> Z position of the belt in MCS                 </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> rotA           </TD><TD> rotation of belt around x axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> rotB           </TD><TD> rotation of belt around y axis in MCS         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> rotC           </TD><TD> rotation of belt around z axis in MCS         </TD></TR>
//! </TABLE>
typedef struct MlpiRobotSetTrafoParameter
{
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM     coordSystem;   //!< Reference to the coordinate system
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE   posX;          //!< X position of the PCS in MCS
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE   posY;          //!< Y position of the PCS in MCS
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE   posZ;          //!< Z position of the PCS in MCS
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE   rotA;          //!< rotation of PCS around x axis in MCS
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE   rotB;          //!< rotation of PCS around y axis in MCS
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE   rotC;          //!< rotation of PCS around z axis in MCS
}MlpiRobotSetTrafoParameter;

//! @typedef MlpiRobotGetTrafoParameter
//! @brief This structure defines the parameter of a transformation.
//! @details Elements of struct MlpiRobotGetTrafoParameter
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH> Direction </TH><TH>           Element        </TH><TH> Description                                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD> [in]      </TD><TD id="st_e"> coordSystem    </TD><TD> Reference to the coordinate system                     </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD> [out]     </TD><TD id="st_e"> posX           </TD><TD> X position of the coordinate system in MCS             </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD> [out]     </TD><TD id="st_e"> posY           </TD><TD> Y position of the coordinate system in MCS             </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD> [out]     </TD><TD id="st_e"> posZ           </TD><TD> Z position of the coordinate system in MCS             </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD> [out]     </TD><TD id="st_e"> rotA           </TD><TD> rotation of the coordinate system around x axis in MCS </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD> [out]     </TD><TD id="st_e"> rotB           </TD><TD> rotation of the coordinate system around y axis in MCS </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD> [out]     </TD><TD id="st_e"> rotC           </TD><TD> rotation of the coordinate system around z axis in MCS </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCSType            </TD><TD> [out]     </TD><TD id="st_e"> csType         </TD><TD> Type of coordinate system                              </TD></TR>
//! </TABLE>

typedef struct MlpiRobotGetTrafoParameter
{
  MlpiRobotCoordinateSystem   MLPI_STRUCT_ALIGN_ENUM      coordSystem;   //!< Reference to the coordinate system
  DOUBLE                      MLPI_STRUCT_ALIGN_DOUBLE    posX;          //!< X position of the coordinate system in MCS
  DOUBLE                      MLPI_STRUCT_ALIGN_DOUBLE    posY;          //!< Y position of the coordinate system in MCS
  DOUBLE                      MLPI_STRUCT_ALIGN_DOUBLE    posZ;          //!< Z position of the coordinate system in MCS
  DOUBLE                      MLPI_STRUCT_ALIGN_DOUBLE    rotA;          //!< rotation of coordinate system around x axis in MCS
  DOUBLE                      MLPI_STRUCT_ALIGN_DOUBLE    rotB;          //!< rotation of coordinate system around y axis in MCS
  DOUBLE                      MLPI_STRUCT_ALIGN_DOUBLE    rotC;          //!< rotation of coordinate system around z axis in MCS
  MlpiRobotCSType             MLPI_STRUCT_ALIGN_ENUM      csType;        //!< Type of coordinate system
}MlpiRobotGetTrafoParameter;

//! @typedef MlpiRobotOpenCyclicChannel
//! @brief This structure defines the parameters for opening a cyclic position channel
//! @details Elements of struct MlpiRobotOpenCyclicChannel
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> blendingRadius </TD><TD> Commanded radius of blending movement  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSetMode    </TD><TD id="st_e"> setMode        </TD><TD> set mode                               </TD></TR>
//! </TABLE>
typedef struct MlpiRobotOpenCyclicChannel
{
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  DOUBLE               MLPI_STRUCT_ALIGN_DOUBLE blendingRadius;   //!< Commanded radius of blending movement
  MlpiRobotSetMode     MLPI_STRUCT_ALIGN_ENUM   setMode;          //!< set mode
}MlpiRobotOpenCyclicChannel;

//! @typedef MlpiRobotCyclicChannel
//! @brief This structure defines the parameters for a cyclic position
//! @details Elements of struct MlpiRobotCyclicChannel
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> point          </TD><TD> Point of movement                      </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCyclicMode        </TD><TD id="st_e"> cyclicMode     </TD><TD> Mode of movement                       </TD></TR>
//! </TABLE>
typedef struct MlpiRobotCyclicChannel
{
  MlpiRobotCoordinateSystem MLPI_STRUCT_ALIGN_ENUM      coordSystem; //!< coordinate system
  MlpiRobotPoint            MLPI_STRUCT_ALIGN_STRUCT    point;       //!< Point of movement
  MlpiRobotCyclicMode       MLPI_STRUCT_ALIGN_ENUM      cyclicMode;  //!< Mode of movement
}MlpiRobotCyclicChannel;

//! @typedef MlpiRobotPerformanceResult
//! @brief This structure defines the information for a performance measurement.
//! @details Elements of struct MlpiRobotPerformanceResult
//! <TABLE>
//! <TR><TH>           Type          </TH><TH>           Element       </TH><TH> Description </TH></TR>
//! <TR><TD id="st_t">      WCHAR16  </TD><TD id="st_e"> name[16]      </TD><TD> Name of the performance measurement.    </TD></TR>
//! <TR><TD id="st_t">      LONG     </TD><TD id="st_e"> count         </TD><TD> Count over recorded measurements.       </TD></TR>
//! <TR><TD id="st_t">      DOUBLE   </TD><TD id="st_e"> min           </TD><TD> Minimum over all recorded measurements. </TD></TR>
//! <TR><TD id="st_t">      DOUBLE   </TD><TD id="st_e"> max           </TD><TD> Maximum over all recorded measurements. </TD></TR>
//! <TR><TD id="st_t">      DOUBLE   </TD><TD id="st_e"> avg           </TD><TD> Average over all recorded measurements. </TD></TR>
//! </TABLE>
typedef struct MlpiRobotPerformanceResult
{
  WCHAR16 MLPI_STRUCT_ALIGN_WCHAR16 name[16];    //!< Name of the performance measurement
  ULONG   MLPI_STRUCT_ALIGN_ULONG   count;       //!< count of measurements
  DOUBLE  MLPI_STRUCT_ALIGN_DOUBLE  min;         //!< minimum value of measurement
  DOUBLE  MLPI_STRUCT_ALIGN_DOUBLE  max;         //!< maximum value of measurement
  DOUBLE  MLPI_STRUCT_ALIGN_DOUBLE  avg;         //!< average value of measurement
}MlpiRobotPerformanceResult;

//! @typedef MlpiRobotMechanicData
//! @brief This structure defines the information about the mechanical structure of a robot.
//! @details Elements of struct MlpiRobotMechanicData
//! <TABLE>
//! <TR><TH>           Type            </TH><TH>           Element       </TH><TH> Description              </TH></TR>
//! <TR><TD id="st_t">MlpiRobotMechCmd </TD><TD id="st_e"> cmd           </TD><TD> Command for the object.  </TD></TR>
//! <TR><TD id="st_t">WCHAR16          </TD><TD id="st_e"> object[8]     </TD><TD> Object id.               </TD></TR>
//! <TR><TD id="st_t">DOUBLE           </TD><TD id="st_e"> x             </TD><TD> x data for the command.  </TD></TR>
//! <TR><TD id="st_t">DOUBLE           </TD><TD id="st_e"> y             </TD><TD> y data for the command.  </TD></TR>
//! <TR><TD id="st_t">DOUBLE           </TD><TD id="st_e"> z             </TD><TD> z data for the command.  </TD></TR>
//! </TABLE>
typedef struct MlpiRobotMechanicData
{
  MlpiRobotMechCmd  MLPI_STRUCT_ALIGN_ENUM          cmd;                  //!< command for the object
  WCHAR16           MLPI_STRUCT_ALIGN_WCHAR16       object[8];            //!< object id
  DOUBLE            MLPI_STRUCT_ALIGN_DOUBLE        x;                    //!< x data for the command
  DOUBLE            MLPI_STRUCT_ALIGN_DOUBLE        y;                    //!< y data for the command
  DOUBLE            MLPI_STRUCT_ALIGN_DOUBLE        z;                    //!< z data for the command
}MlpiRobotMechanicData;

//! @typedef MlpiKinematicsValues
//! Structure containing operation information about a kinematics. These values do change as soon as the kinematics is in operation.
//! You may want to use this structure to read several sets of kinematics information using one single function call during operation of the kinematics.
//! This provides increased performance in comparison to reading the values bit by bit. Especially when reading the values for
//! multiple kinematics.
//! @details Elements of struct MlpiKinematicsValues
//! <TABLE>
//! <TR><TH>           Type         </TH><TH>           Element                                              </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t"> ULONG        </TD><TD id="st_e"> state                                                </TD><TD> Kinematics state of the kinematics.           </TD></TR>
//! <TR><TD id="st_t"> ULONG        </TD><TD id="st_e"> stateExtended                                        </TD><TD> Extended kinematics state of the kinematics.  </TD></TR>
//! <TR><TD id="st_t"> ULONG        </TD><TD id="st_e"> diagnosisNumber                                      </TD><TD> DiagnosisNumber of the kinematics.            </TD></TR>
//! <TR><TD id="st_t"> FLOAT        </TD><TD id="st_e"> actualPosition[MLPI_ROBOT_COORDINATES_IN_POINT]      </TD><TD> Actual position of the kinematics.            </TD></TR>
//! <TR><TD id="st_t"> FLOAT        </TD><TD id="st_e"> actualVelocity                                       </TD><TD> Actual velocity of the kinematics.            </TD></TR>
//! <TR><TD id="st_t"> FLOAT        </TD><TD id="st_e"> actualAcceleration                                   </TD><TD> Actual acceleration of the kinematics.        </TD></TR>
//! </TABLE>
typedef struct MlpiKinematicsValues
{
  // Output Parameters:
  ULONG     MLPI_STRUCT_ALIGN_ULONG state;                                              //!< Kinematics state of the kinematics.
  ULONG     MLPI_STRUCT_ALIGN_ULONG stateExtended;                                      //!< Extended kinematics state of the kinematics.
  ULONG     MLPI_STRUCT_ALIGN_ULONG diagnosisNumber;                                    //!< DiagnosisNumber of the kinematics.
  FLOAT     MLPI_STRUCT_ALIGN_FLOAT actualPosition[MLPI_ROBOT_COORDINATES_IN_POINT];    //!< Actual position of the kinematics.
  FLOAT     MLPI_STRUCT_ALIGN_FLOAT actualVelocity;                                     //!< Actual velocity of the kinematics.
  FLOAT     MLPI_STRUCT_ALIGN_FLOAT actualAcceleration;                                 //!< Actual acceleration of the kinematics.
}MlpiKinematicsValues;

//! @typedef MlpiKinematicsUnits
//! @brief Structure containing units of the kinematics as strings.
//! You may want to use this structure to read all units of a kinematics information set using one single function call.
//! @details Elements of struct MlpiKinematicsUnits
//! <TABLE>
//! <TR><TH>           Type         </TH><TH>           Element      </TH><TH> Description                                         </TH></TR>
//! <TR><TD id="st_t"> WCHAR16      </TD><TD id="st_e"> position[MLPI_ROBOT_MAX_UNITS_LEN]     </TD><TD> Position unit of the kinematics. e.g. 'mm'.         </TD></TR>
//! <TR><TD id="st_t"> WCHAR16      </TD><TD id="st_e"> velocity[MLPI_ROBOT_MAX_UNITS_LEN]     </TD><TD> Velocity unit of the kinematics. e.g. 'mm/s'.       </TD></TR>
//! <TR><TD id="st_t"> WCHAR16      </TD><TD id="st_e"> acceleration[MLPI_ROBOT_MAX_UNITS_LEN] </TD><TD> Acceleration unit of the kinematics. e.g. 'mm/s2'.  </TD></TR>
//! </TABLE>
typedef struct MlpiKinematicsUnits
{
  WCHAR16 MLPI_STRUCT_ALIGN_WCHAR16 position[MLPI_ROBOT_MAX_UNITS_LEN];        //!< Position unit of the kinematics. e.g. 'mm'.
  WCHAR16 MLPI_STRUCT_ALIGN_WCHAR16 velocity[MLPI_ROBOT_MAX_UNITS_LEN];        //!< Velocity unit of the kinematics. e.g. 'mm/s'.
  WCHAR16 MLPI_STRUCT_ALIGN_WCHAR16 acceleration[MLPI_ROBOT_MAX_UNITS_LEN];    //!< Acceleration unit of the kinematics. e.g. 'mm/s2'.
}MlpiKinematicsUnits;

//! @typedef MlpiRobotStop
//! @brief This structure defines the parameters for stopping the group
//! @details Elements of struct MlpiRobotStop
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                               </TH></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement          </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement                  </TD></TR>
//! <TR><TD id="st_t"> BOOL8               </TD><TD id="st_e"> stop           </TD><TD> TRUE: stopping, FALSE: exit stopping mode </TD></TR>
//! </TABLE>
typedef struct MlpiRobotStop
{
  DOUBLE        MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE        MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  BOOL8         MLPI_STRUCT_ALIGN_BOOL8  stop;             //!< TRUE: stopping, FALSE: exit stopping mode
                                  //!< Important Note: This command is special in the way that it has to be
                                  //!< called one time with 'stop' = TRUE. This will bring the group to a
                                  //!< standstill. And when the group is standing, the command has to be called again
                                  //!< with 'stop' = FALSE in order to enter the PLCopen 'Standstill' mode.
}MlpiRobotStop;

//! @typedef MlpiRobotInterrupt
//! @brief This structure defines the parameters for interrupting the group
//! @details Elements of struct MlpiRobotInterrupt
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                                                     </TH></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement                                </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement                                        </TD></TR>
//! </TABLE>
typedef struct MlpiRobotInterrupt
{
  DOUBLE        MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE        MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
}MlpiRobotInterrupt;

//! @typedef MlpiRobotSafeZoneData
//! @brief This structure defines the parameters for a save zone
//! @details Elements of struct MlpiRobotSafeZoneData
//! <TABLE>
//! <TR><TH>           Type                  </TH><TH>           Element                                 </TH><TH> Description                                                     </TH></TR>
//! <TR><TD id="st_t"> USHORT                </TD><TD id="st_e"> zoneNumber                              </TD><TD> zone number                                                     </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotSafeZoneType </TD><TD id="st_e"> zoneType                                </TD><TD> zone type                                                       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                </TD><TD id="st_e"> pointPos[MLPI_ROBOT_MAX_POS_AXIS]       </TD><TD> minimum position of zone                                        </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                </TD><TD id="st_e"> pointNeg[MLPI_ROBOT_MAX_POS_AXIS]       </TD><TD> maximum position of zone                                        </TD></TR>
//! </TABLE>
typedef struct MlpiRobotSafeZoneData
{
  USHORT                MLPI_STRUCT_ALIGN_USHORT zoneNumber;                             //!< zone number
  MlpiRobotSafeZoneType MLPI_STRUCT_ALIGN_ENUM   zoneType;                               //!< zone type
  DOUBLE                MLPI_STRUCT_ALIGN_DOUBLE pointPos[MLPI_ROBOT_MAX_POS_AXIS];      //!< minimum position of zone
  DOUBLE                MLPI_STRUCT_ALIGN_DOUBLE pointNeg[MLPI_ROBOT_MAX_POS_AXIS];      //!< maximum position of zone
}MlpiRobotSafeZoneData;

//! @typedef MlpiRobotJogStepData
//! @brief This structure defines the parameters of a jog step movement.
//! @details Elements of struct MlpiRobotJogStepData
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> distance       </TD><TD> Distance of movement                   </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! </TABLE>
typedef struct MlpiRobotJogStepData
{
  MlpiRobotPoint               MLPI_STRUCT_ALIGN_STRUCT distance;         //!< Distance of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE velocity;         //!< Maximum velocity of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE acceleration;     //!< Maximum acceleration of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  MlpiRobotCoordinateSystem    MLPI_STRUCT_ALIGN_ENUM   coordSystem;      //!< coordinate system
} MlpiRobotJogStepData;

//! @typedef MlpiRobotJogContData
//! @brief This structure defines the parameters of a continuous jog movement.
//! @details Elements of struct MlpiRobotJogContData
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element        </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> direction      </TD><TD> Direction of movement                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> velocity       </TD><TD> Maximum velocity of movement           </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> acceleration   </TD><TD> Maximum acceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> deceleration   </TD><TD> Maximum deceleration of movement       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE                     </TD><TD id="st_e"> jerk           </TD><TD> Maximum jerk of movement               </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSystem    </TD><TD> coordinate system                      </TD></TR>
//! </TABLE>
typedef struct MlpiRobotJogContData
{
  MlpiRobotPoint               MLPI_STRUCT_ALIGN_STRUCT direction;        //!< Direction of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE velocity;         //!< Maximum velocity of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE acceleration;     //!< Maximum acceleration of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE deceleration;     //!< Maximum deceleration of movement
  DOUBLE                       MLPI_STRUCT_ALIGN_DOUBLE jerk;             //!< Maximum jerk of movement
  MlpiRobotCoordinateSystem    MLPI_STRUCT_ALIGN_ENUM   coordSystem;      //!< coordinate system
} MlpiRobotJogContData;

//! @typedef MlpiRobotStopCmdData
//! @brief This structure defines the parameters of a stop command.
//! @details Elements of struct MlpiRobotStopCmdData
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                             </TH></TR>
//! <TR><TD id="st_t"> MLPIMOTIONHANDLE    </TD><TD id="st_e"> cmdID          </TD><TD> ID of the command that will be stopped  </TD></TR>
//! </TABLE>
typedef struct MlpiRobotStopCmdData
{
  MLPIMOTIONHANDLE               MLPI_STRUCT_ALIGN_ULLONG cmdID;         //!< ID of the command that will be stopped
} MlpiRobotStopCmdData;

//! @typedef MlpiRobotBeltDesyncRelData
//! @brief This structure defines the parameters of a relative desynchronization configuration.
//! @details Elements of struct MlpiRobotBeltDesyncRelData
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element        </TH><TH> Description                                    </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef         </TD><TD id="st_e"> belt           </TD><TD> Reference to the axis                          </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> startPercent   </TD><TD> percentage of start relative desynchronization </TD></TR>
//! <TR><TD id="st_t"> DOUBLE              </TD><TD id="st_e"> endPercent     </TD><TD> percentage of end relative desynchronization   </TD></TR>
//! </TABLE>
typedef struct MlpiRobotBeltDesyncRelData
{
  MlpiAxisRef                    MLPI_STRUCT_ALIGN_STRUCT belt;        //!< Reference to the axis
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE startPercent;//!< percentage of start relative desynchronization
  DOUBLE                         MLPI_STRUCT_ALIGN_DOUBLE endPercent;  //!< percentage of end relative desynchronization
}MlpiRobotBeltDesyncRelData;


//! @typedef MlpiRobotTeachInDataWrite
//! @brief This structure defines the parameters of the TeachInWrite command.
//! @details Elements of struct MlpiRobotTeachInDataWrite
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>          Element                                    </TH><TH> Description                          </TH></TR>
//! <TR><TD id="st_t"> WCHAR16                    </TD><TD id="st_e"> pointName[MLPI_ROBOT_MAX_TEACH_NAME_LEN]  </TD><TD> Name of the point that will be saved </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD id="st_e"> teachPoint                                </TD><TD> Point that will be saved             </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD id="st_e"> coordSys                                  </TD><TD> Coordinate system of the point       </TD></TR>
//! <TR><TD id="st_t"> USHORT                     </TD><TD id="st_e"> options                                   </TD><TD> Option, what will be written and how </TD></TR>
//! </TABLE>
typedef struct MlpiRobotTeachInDataWrite
{
  WCHAR16                       MLPI_STRUCT_ALIGN_WCHAR16 pointName[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
  MlpiRobotPoint                MLPI_STRUCT_ALIGN_STRUCT  teachPoint;
  MlpiRobotCoordinateSystem     MLPI_STRUCT_ALIGN_ENUM    coordSys;
  USHORT                        MLPI_STRUCT_ALIGN_USHORT  options;
} MlpiRobotTeachInDataWrite;

//! @typedef MlpiRobotTeachInDataRead
//! @brief This structure defines the parameters of the TeachInRead command.
//! @details Elements of struct MlpiRobotTeachInDataRead
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH> Direction </TH><TH>           Element                                     </TH><TH> Description                           </TH></TR>
//! <TR><TD id="st_t"> WCHAR16                    </TD><TD> [in]      </TD><TD id="st_e"> pointName[MLPI_ROBOT_MAX_TEACH_NAME_LEN]    </TD><TD> Name of the point that will be loaded </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD> [out]     </TD><TD id="st_e"> teachPoint                                  </TD><TD> Point that will be returned           </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD> [out]     </TD><TD id="st_e"> coordSys                                    </TD><TD> Coordinate system of the point        </TD></TR>
//! </TABLE>
typedef struct MlpiRobotTeachInDataRead
{
  //in
  WCHAR16                       MLPI_STRUCT_ALIGN_WCHAR16 pointName[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
  //out
  MlpiRobotPoint                MLPI_STRUCT_ALIGN_STRUCT  teachPoint;
  MlpiRobotCoordinateSystem     MLPI_STRUCT_ALIGN_ENUM    coordSys;

} MlpiRobotTeachInDataRead;

//! @typedef MlpiRobotTeachInDataReadNext
//! @brief This structure defines the parameters of the TeachInDataReadNext command.
//! @details Elements of struct MlpiRobotTeachInDataReadNext
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH> Direction </TH><TH>           Element                                         </TH><TH> Description                          </TH></TR>
//! <TR><TD id="st_t"> WCHAR16                    </TD><TD> [in]      </TD><TD id="st_e"> pointNameInput[MLPI_ROBOT_MAX_TEACH_NAME_LEN]   </TD><TD> Name of the current point            </TD></TR>
//! <TR><TD id="st_t"> WCHAR16                    </TD><TD> [out]     </TD><TD id="st_e"> pointNameOutput[MLPI_ROBOT_MAX_TEACH_NAME_LEN]  </TD><TD> Name of the next point after current </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint             </TD><TD> [out]     </TD><TD id="st_e"> teachPoint                                      </TD><TD> Point that will be saved             </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotCoordinateSystem  </TD><TD> [out]     </TD><TD id="st_e"> coordSys                                        </TD><TD> Coordinate system of the point       </TD></TR>
//! </TABLE>
typedef struct MlpiRobotTeachInDataReadNext
{
  // in
  WCHAR16                       MLPI_STRUCT_ALIGN_WCHAR16 pointNameInput[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
  // out
  WCHAR16                       MLPI_STRUCT_ALIGN_WCHAR16 pointNameOutput[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
  MlpiRobotPoint                MLPI_STRUCT_ALIGN_STRUCT  teachPoint;
  MlpiRobotCoordinateSystem     MLPI_STRUCT_ALIGN_ENUM    coordSys;
} MlpiRobotTeachInDataReadNext;

//! @typedef MlpiRobotTeachInDataDeletePoints
//! @brief This structure defines the parameters of the TeachInDataDeletePoints command.
//! @details Elements of struct MlpiRobotTeachInDataDeletePoints
//! <TABLE>
//! <TR><TH>           Type     </TH><TH>           Element                                     </TH><TH> Description                              </TH></TR>
//! <TR><TD id="st_t"> WCHAR16  </TD><TD id="st_e"> pointName[MLPI_ROBOT_MAX_TEACH_NAME_LEN]    </TD><TD> Name of the point that will be deleted   </TD></TR>
//! </TABLE>
typedef struct MlpiRobotTeachInDataDeletePoints
{
  WCHAR16                           MLPI_STRUCT_ALIGN_WCHAR16 pointName[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
} MlpiRobotTeachInDataDeletePoints;

//! @typedef MlpiRobotTeachInDataFileData
//! @brief This structure defines the parameters of the TeachInDataFileData command.
//! @details Elements of struct MlpiRobotTeachInDataFileData
//! <TABLE>
//! <TR><TH>           Type                            </TH><TH>           Element                                 </TH><TH> Description                                    </TH></TR>
//! <TR><TD id="st_t"> WCHAR16                         </TD><TD id="st_e"> fileName[MLPI_ROBOT_MAX_TEACH_NAME_LEN] </TD><TD> Name of the file that will be saved or loaded  </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotTeachInPointFileOption </TD><TD id="st_e"> fileOptions                             </TD><TD> Option, how the loading or writing will happen </TD></TR>
//! </TABLE>
typedef struct MlpiRobotTeachInDataFileData
{
  WCHAR16                           MLPI_STRUCT_ALIGN_WCHAR16 fileName[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
  MlpiRobotTeachInPointFileOption   MLPI_STRUCT_ALIGN_ENUM    fileOptions;
} MlpiRobotTeachInDataFileData;

//! @typedef MlpiRobotTeachInDataDeleteFileData
//! @brief This structure defines the parameters of the TeachInDataFileData command.
//! @details Elements of struct MlpiRobotTeachInDataDeleteFileData
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>              Element                                   </TH><TH> Description                            </TH></TR>
//! <TR><TD id="st_t"> WCHAR16  </TD><TD id="st_e"> fileName[MLPI_ROBOT_MAX_TEACH_NAME_LEN]   </TD><TD> Name of the file that will be deleted  </TD></TR>
//! </TABLE>
typedef struct MlpiRobotTeachInDataDeleteFileData
{
  WCHAR16                           MLPI_STRUCT_ALIGN_WCHAR16 fileName[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
} MlpiRobotTeachInDataDeleteFileData;

//! @typedef MlpiRobotGetStatusSummary
//! @brief This structure defines the parameters of the GetStatusSummary command.
//! @details Elements of struct MlpiRobotGetStatusSummary
//! <TABLE>
//! <TR><TH>           Type                                   </TH><TH> Direction </TH><TH>           Element             </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiGroupRef                           </TD><TD> [in]      </TD><TD id="st_e"> group               </TD><TD> Reference to the group.                       </TD></TR>
//! <TR><TD id="st_t"> ULONG                                  </TD><TD> [in]      </TD><TD id="st_e"> coordSys            </TD><TD> Coordinate System.                            </TD></TR>
//! <TR><TD id="st_t"> ULONG                                  </TD><TD> [out]     </TD><TD id="st_e"> state               </TD><TD> Kinematics state                              </TD></TR>
//! <TR><TD id="st_t"> ULONG                                  </TD><TD> [out]     </TD><TD id="st_e"> stateExtended       </TD><TD> Extended kinematics state                     </TD></TR>
//! <TR><TD id="st_t"> ULONG                                  </TD><TD> [out]     </TD><TD id="st_e"> diagnosisNumber     </TD><TD> Diagnosis number of the kinematics.           </TD></TR>
//! <TR><TD id="st_t"> USHORT                                 </TD><TD> [out]     </TD><TD id="st_e"> dimension           </TD><TD> Dimension of a point of the kinematics        </TD></TR>
//! <TR><TD id="st_t"> MlpiRobotPoint                         </TD><TD> [out]     </TD><TD id="st_e"> actualPosition      </TD><TD> Actual position of the kinematics.            </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[MLPI_ROBOT_MAX_UNITS_LEN]      </TD><TD> [out]     </TD><TD id="st_e"> actualPosUnit       </TD><TD> Unit of actual position.                      </TD></TR>
//! <TR><TD id="st_t"> FLOAT                                  </TD><TD> [out]     </TD><TD id="st_e"> actualVelocity      </TD><TD> Actual velocity of the kinematics.            </TD></TR>
//! <TR><TD id="st_t"> FLOAT                                  </TD><TD> [out]     </TD><TD id="st_e"> actualAcceleration  </TD><TD> Actual acceleration of the kinematics.        </TD></TR>
//! <TR><TD id="st_t"> FLOAT                                  </TD><TD> [out]     </TD><TD id="st_e"> actualJerk          </TD><TD> Actual jerk of the kinematics.                </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[MLPI_ROBOT_MAX_UNIT_LEN]       </TD><TD> [out]     </TD><TD id="st_e"> pathUnits           </TD><TD> Unit of Vel, Acc and Jrk.                     </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[MLPI_ROBOT_MAX_UNIT_LEN]       </TD><TD> [out]     </TD><TD id="st_e"> timeBase            </TD><TD> Unit of time.                                 </TD></TR>
//! </TABLE>
typedef struct MlpiRobotGetStatusSummary
{
   // in
   MlpiGroupRef          MLPI_STRUCT_ALIGN_STRUCT  group;                                            //!< Reference to the group.
   ULONG                 MLPI_STRUCT_ALIGN_ULONG   coordSys;                                         //!< Coordinate system.
   //outs
   ULONG                 MLPI_STRUCT_ALIGN_ULONG   state;                                            //!< Kinematics state.
   ULONG                 MLPI_STRUCT_ALIGN_ULONG   stateExtended;                                    //!< Extended kinematics state.
   ULONG                 MLPI_STRUCT_ALIGN_ULONG   diagnosisNumber;                                  //!< Diagnosis number of the kinematics.
   USHORT                MLPI_STRUCT_ALIGN_USHORT  dimension;                                        //!< Dimension of a point of the kinematics.
   MlpiRobotPoint        MLPI_STRUCT_ALIGN_STRUCT  actualPosition;                                   //!< Actual position of the kinematics.
   WCHAR16               MLPI_STRUCT_ALIGN_WCHAR16 actualPosUnit[MLPI_ROBOT_MAX_UNITS_LEN];          //!< Unit of actual position.
   FLOAT                 MLPI_STRUCT_ALIGN_FLOAT   actualVelocity;                                   //!< Actual velocity of the kinematics.
   FLOAT                 MLPI_STRUCT_ALIGN_FLOAT   actualAcceleration;                               //!< Actual acceleration of the kinematics.
   FLOAT                 MLPI_STRUCT_ALIGN_FLOAT   actualJerk;                                       //!< Actual jerk of the kinematics.
   WCHAR16               MLPI_STRUCT_ALIGN_WCHAR16 pathUnit[MLPI_ROBOT_MAX_UNIT_LEN];                //!< Unit of Vel, Acc and Jrk.
   WCHAR16               MLPI_STRUCT_ALIGN_WCHAR16 timeBase[MLPI_ROBOT_MAX_UNIT_LEN];                //!< Unit of time.
} MlpiRobotGetStatusSummary;

//! @typedef MlpiRobotCoordinateSystemInfo
//! @brief This structure defines the parameters of the GetCSInfo command.
//! @details Elements of struct MlpiRobotCoordinateSystemInfo
//! <TABLE>
//! <TR><TH>           Type                               </TH><TH>           Element       </TH><TH> Description                             </TH></TR>
//! <TR><TD id="st_t"> ULONG                              </TD><TD id="st_e"> cs            </TD><TD> Reference to the coordinate system.     </TD></TR>
//! <TR><TD id="st_t"> USHORT                             </TD><TD id="st_e"> dimension     </TD><TD> Dimension of a point of the kinematics. </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[MLPI_ROBOT_MAX_CSNAME_LEN] </TD><TD id="st_e"> name          </TD><TD> Name of the kinematics.                 </TD></TR>
//! <TR><TD id="st_t"> WCHAR16[MLPI_ROBOT_MAX_UNITS_LEN]  </TD><TD id="st_e"> posunits      </TD><TD> Unit of Vel, Acc and Jrk.               </TD></TR>
//! </TABLE>
typedef struct MlpiRobotCoordinateSystemInfo
{
  //out
  ULONG                           MLPI_STRUCT_ALIGN_ULONG   cs;                                     //!< Reference to the coordinate system.
  USHORT                          MLPI_STRUCT_ALIGN_USHORT  dimension;                              //!< Dimension of a point of the kinematics.
  WCHAR16                         MLPI_STRUCT_ALIGN_WCHAR16 name[MLPI_ROBOT_MAX_CSNAME_LEN];        //!< Name of the kinematics.   
  WCHAR16                         MLPI_STRUCT_ALIGN_WCHAR16 posunits[MLPI_ROBOT_MAX_UNITS_LEN];     //!< Unit of Vel, Acc and Jrk.
} MlpiRobotCoordinateSystemInfo;

//! @typedef MlpiRobotCmdInfo
//! @brief This structure defines the information about the mechanical structure of a robot.
//! @details Elements of struct MlpiRobotCmdInfo
//! <TABLE> 
//! <TR><TH>          Type                 </TH><TH>           Element       </TH><TH> Description                  </TH></TR>
//! <TR><TD id="st_t">ULONG                </TD><TD id="st_e"> commandID     </TD><TD> ID of the command.           </TD></TR>
//! <TR><TD id="st_t">MlpiRobotCmdInfoType </TD><TD id="st_e"> type          </TD><TD> Type of given information.   </TD></TR>
//! <TR><TD id="st_t">DOUBLE               </TD><TD id="st_e"> value         </TD><TD> Value of the information.    </TD></TR>
//! </TABLE>
typedef struct MlpiRobotCmdInfo
{
  ULONG                 MLPI_STRUCT_ALIGN_ULONG  commandID;    //!< ID of the command.
  MlpiRobotCmdInfoType  MLPI_STRUCT_ALIGN_ENUM   infoType;     //!< Type of given information.
  DOUBLE                MLPI_STRUCT_ALIGN_DOUBLE value;        //!< Value of the information.
}MlpiRobotCmdInfo;

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

//! @ingroup RobotLibMovement
//! This function moves a kinematics linear absolute.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move linear absolute with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveLinearAbsolute paramSet;
//! paramSet.point.coordinate[0] = 0.0;
//! paramSet.point.coordinate[1] = 0.0;
//! paramSet.point.coordinate[2] = 0.0;
//! paramSet.velocity = 1000.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! MLPIRESULT result = mlpiRobotMoveLinearAbs(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveLinearAbs(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveLinearAbsolute* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function moves a kinematics linear relative.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move linear relative with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveLinearRelative paramSet;
//! paramSet.distance.coordinate[0] = 100.0;
//! paramSet.distance.coordinate[1] = 0.0;
//! paramSet.distance.coordinate[2] = 0.0;
//! paramSet.velocity = 1000.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! MLPIRESULT result = mlpiRobotMoveLinearRel(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveLinearRel(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveLinearRelative* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function moves a kinematics direct absolute
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move direct absolute with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveDirectAbsolute paramSet;
//! paramSet.point.coordinate[0] = 0.0;
//! paramSet.point.coordinate[1] = 0.0;
//! paramSet.point.coordinate[2] = 0.0;
//! paramSet.velocity = 1000.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! MLPIRESULT result = mlpiRobotMoveDirectAbs(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveDirectAbs(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveDirectAbsolute* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function moves a kinematics direct relative
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move direct relative with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveDirectRelative paramSet;
//! paramSet.distance.coordinate[0] = 100.0;
//! paramSet.distance.coordinate[1] = 0.0;
//! paramSet.distance.coordinate[2] = 0.0;
//! paramSet.velocity = 1000.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! MLPIRESULT result = mlpiRobotMoveDirectRel(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveDirectRel(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveDirectRelative* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function moves a kinematics circular absolute.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move circular absolute with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveCircularAbsolute paramSet;
//! paramSet.auxPoint.coordinate[0] = -100.0;
//! paramSet.auxPoint.coordinate[1] = 100.0;
//! paramSet.auxPoint.coordinate[2] = 0.0;
//! paramSet.endPoint.coordinate[0] = 0.0;
//! paramSet.endPoint.coordinate[1] = 0.0;
//! paramSet.endPoint.coordinate[2] = 0.0;
//! paramSet.velocity = 100.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! MLPIRESULT result = mlpiRobotMoveCircularAbs(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveCircularAbs(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveCircularAbsolute* paramSet, MLPIMOTIONHANDLE* motionHandle);

//! @ingroup RobotLibMovement
//! This function moves a kinematics circular relative
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move circular relative with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveCircularRelative paramSet;
//! paramSet.auxPoint.coordinate[0] = -100.0;
//! paramSet.auxPoint.coordinate[1] = 100.0;
//! paramSet.auxPoint.coordinate[2] = 0.0;
//! paramSet.endPoint.coordinate[0] = 0.0;
//! paramSet.endPoint.coordinate[1] = 0.0;
//! paramSet.endPoint.coordinate[2] = 0.0;
//! paramSet.velocity = 100.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! MLPIRESULT result = mlpiRobotMoveCircularRel(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveCircularRel(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveCircularRelative* paramSet, MLPIMOTIONHANDLE* motionHandle);

//! @ingroup RobotLibMovement
//! This function moves a kinematics jump absolute.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Move linear absolute with values of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMoveLinearAbsolute paramSet;
//! paramSet.point.coordinate[0] = 50.0;
//! paramSet.point.coordinate[1] = 0.0;
//! paramSet.point.coordinate[2] = 0.0;
//! paramSet.velocity = 1000.0;
//! paramSet.acceleration = 1000.0;
//! paramSet.deceleration = 1000.0;
//! paramSet.jerk = 0.0;
//! paramSet.blendingRadius = 0.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.slopeType = MLPI_ROBOT_SLOPETYPE_BLOCK_SLOPE;
//! paramSet.startHeight = 10.0;
//! paramSet.maxHeight = 20.0;
//! paramSet.endHeight = 10.0;
//! MLPIRESULT result = mlpiRobotMoveJumpAbs(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMoveJumpAbs(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotMoveJumpAbsolute* paramSet, MLPIMOTIONHANDLE* motionHandle);

//! @ingroup RobotLibConfig
//! This function configures a belt
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Configures a belt with the settings of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSetBeltConfiguration paramSet;
//! paramSet.belt.axisNo = MLPI_AXIS_4;
//! paramSet.belt.controlNo = MLPI_LOCAL_CONTROL;
//! paramSet.posX = 0.0;
//! paramSet.posY = 0.0;
//! paramSet.posZ = 0.0;
//! paramSet.rotA = 0.0;
//! paramSet.rotB = 0.0;
//! paramSet.rotC = 0.0;
//! paramSet.begin = 10.0;
//! paramSet.total = 110.0;
//! MLPIRESULT result = mlpiRobotSetBeltConfiguration(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetBeltConfiguration(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSetBeltConfiguration* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibConfig
//! This function configures a rotary table
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Configures a rotary table with the settings of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSetBeltConfiguration paramSet;
//! paramSet.belt.axisNo = MLPI_AXIS_4;
//! paramSet.belt.controlNo = MLPI_LOCAL_CONTROL;
//! paramSet.posX = 0.0;
//! paramSet.posY = 0.0;
//! paramSet.posZ = 0.0;
//! paramSet.rotA = 0.0;
//! paramSet.rotB = 0.0;
//! paramSet.rotC = 0.0;
//! paramSet.begin = 10.0;
//! paramSet.total = 110.0;
//! MLPIRESULT result = mlpiRobotSetRotaryTableConfiguration(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetRotaryTableConfiguration(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSetBeltConfiguration* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function synchronizes the kinematics to a belt considering limits for jerk and acceleration
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Synchronizes the kinematics to a belt with settings of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSyncOnWithLimits paramSet;
//! paramSet.acceleration = 200.0;
//! paramSet.belt.axisNo = MLPI_AXIS_4;
//! paramSet.belt.controlNo = MLPI_LOCAL_CONTROL;
//! paramSet.jerk = 20000.0;
//! MLPIRESULT result = mlpiRobotSyncOnWithLimits(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSyncOnWithLimits(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSyncOnWithLimits* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function desynchronizes the kinematics from a belt considering limits for jerk and deceleration
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Desynchronizes the kinematics from a belt with settings of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSyncOffWithLimits paramSet;
//! paramSet.belt.axisNo = MLPI_AXIS_4;
//! paramSet.belt.controlNo = MLPI_LOCAL_CONTROL;
//! paramSet.blendingRadius = 10.0;
//! paramSet.deceleration = 200.0;
//! paramSet.jerk = 20000.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! MLPIRESULT result = mlpiRobotSyncOffWithLimits(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSyncOffWithLimits(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSyncOffWithLimits* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function opens a channel for cyclic position commands.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // opens a cyclic position command channel
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotOpenCyclicChannel paramSet;
//! paramSet.blendingRadius = 10.0;
//! paramSet.deceleration = 200.0;
//! paramSet.jerk = 20000.0;
//! paramSet.setMode = MLPI_ROBOT_SETMODE_TO;
//! MLPIRESULT result = mlpiRobotOpenCyclicChannel(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotOpenCyclicChannel(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotOpenCyclicChannel* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function writes a new cyclic position to a cyclic position channel.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // writes a new cyclic position to a cyclic position channel.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotCyclicChannel paramSet;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.cyclicMode = MLPI_ROBOT_CYCLIC_MODE_POS_ABS;
//! paramSet.point.coordinate[0] = -0.01;
//! paramSet.point.coordinate[1] = 0.1, 0.0;
//! paramSet.point.coordinate[2] = 0.0;
//! MLPIRESULT result = mlpiRobotWriteCyclicChannel(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotWriteCyclicChannel(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotCyclicChannel* paramSet);


//! @ingroup RobotLibMovement
//! This function stops the group with settings of paramSet.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Stop the group.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotStop paramSet;
//! paramSet.deceleration = 100.0;
//! paramSet.jerk = 20000;
//! paramSet.stop = TRUE;
//! MLPIRESULT result = mlpiRobotStop(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotStop(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotStop* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function interrupts the group
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Interrupt the group with settings of paramSet.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotInterrupt paramSet;
//! paramSet.deceleration = 100.0;
//! paramSet.jerk = 20000;
//! MLPIRESULT result = mlpiRobotInterrupt(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotInterrupt(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotInterrupt* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function continues the previously interrupted movement of a group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Continue movement of the group.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MLPIRESULT result = mlpiRobotContinue(connection, group, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotContinue(const MLPIHANDLE connection, const MlpiGroupRef group, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibInfo
//! This function returns last values written by mlpiRobotWriteCyclicChannel.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[out]  paramSet            Structure containing all information necessary for the command. Returned cyclic pos.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // returns last values written by mlpiRobotWriteCyclicChannel
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotCyclicChannel paramSet;
//! MLPIRESULT result = mlpiRobotReadCyclicChannel(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotReadCyclicChannel(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotCyclicChannel* paramSet);


//! @ingroup RobotLibInfo
//! This function returns the current position of the robot
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   coordSystem         Coordinate system to read position from.
//! @param[in]   pointType           Point type to read position from.
//! @param[out]  position            Current position.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the current position of the robot.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPoint position;
//! MLPIRESULT result = mlpiRobotReadPos(connection, group, MLPI_ROBOT_CS_MCS, MLPI_ROBOT_POINT_TYPE_PREP, &position);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotReadPos(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotCoordinateSystem coordSystem, const MlpiRobotPointType pointType, MlpiRobotPoint *position);


//! @ingroup RobotLibInfo
//! This function moves a point at the current position in space. Usually this function should be called for points on a belt. A point
//! with one more belt values will be moved to current belt values. After calling this function the point on a belt has the current
//! position in space with the current belt value. Points without belt values will not be modified.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in]      group               Reference to group.
//! @param[in]      coordSystem         Coordinate system of the point.
//! @param[in,out]  position            Current position.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // moves a point on a belt to current position.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPoint position;
//! MLPIRESULT result = mlpiRobotMovePoint(connection, group, MLPI_ROBOT_CS_MCS, &position);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMovePoint(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotCoordinateSystem coordSystem, MlpiRobotPoint *position);


//! @ingroup RobotLibInfo
//! Transform a point from coordSystemIn to coordSystemOut. The location of the point stays the same.
//! Only the point of view (coordinate system) changes.
//! @param[in]      connection       Handle for multiple connections.
//! @param[in]      group            Reference to group.
//! @param[in]      coordSystemIn    Coordinate system to read position from.
//! @param[in]      coordSystemOut   Coordinate system to which the position gets transformed.
//! @param[in,out]  position         Current position.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Transform point from MCS to BCS.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPoint position;
//! MLPIRESULT result = mlpiRobotTransformPoint(connection, group, MLPI_ROBOT_CS_MCS, MLPI_ROBOT_CS_BCS, &position);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTransformPoint(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotCoordinateSystem coordSystemIn, const MlpiRobotCoordinateSystem coordSystemOut, MlpiRobotPoint *position);


//! @ingroup RobotLibConfig
//! This function configures the error reaction of a belt.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Configure error reaction of a belt to desync over 100ms.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiAxisRef belt;
//! belt.axisNo = MLPI_AXIS_4;
//! belt.controlNo = MLPI_LOCAL_CONTROL;
//! MlpiRobotSetBeltErrorReaction paramSet;
//! paramSet.belt = belt;
//! paramSet.decelaration = 0.0;
//! paramSet.errorType = MLPI_ROBOT_BELT_ERROR_DESYNC_OVER_TIME;
//! paramSet.jerk = 0.0;
//! paramSet.time = 100.0;
//! MLPIRESULT result = mlpiRobotSetBeltErrorReaction(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetBeltErrorReaction(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSetBeltErrorReaction* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibConfig
//! This function gets the configured error reaction of a belt.
//! @param[in]       connection        Handle for multiple connections.
//! @param[in]       group             Reference to group.
//! @param[in,out]   paramSet          Structure containing all information necessary for the command.
//! @return                            Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get Configured error reaction of a belt.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiAxisRef belt;
//! belt.axisNo = MLPI_AXIS_4;
//! belt.controlNo = MLPI_LOCAL_CONTROL;
//! MlpiRobotGetBeltErrorReaction paramSet;
//! paramSet.belt = belt;
//! MLPIRESULT result = mlpiRobotGetBeltErrorReaction(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetBeltErrorReaction(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotGetBeltErrorReaction* paramSet);


//! @ingroup RobotLibConfig
//! This function configures a Cartesian transformation between PCS and MCS. Note that this function will return an error 
//! if a belt is configured for the PCS.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Transforms the origin of MLPI_ROBOT_CS_PCS1 to (100.0,50.0,0.0,0.0,0.0,0.0) in MCS.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSetTrafoParameter paramSet;
//! paramSet.coordSystem = MLPI_ROBOT_CS_PCS1;
//! paramSet.posX = 100;
//! paramSet.posY = 50;
//! paramSet.posZ = 0;
//! paramSet.rotA = 0.0;
//! paramSet.rotB = 0.0;
//! paramSet.rotB = 0.0;
//! MLPIRESULT result = mlpiRobotSetCartesianTransform(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetCartesianTransform(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSetTrafoParameter* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibConfig
//! This function configures a cylindric transformation between PCS and MCS
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Transforms the origin of MLPI_ROBOT_CS_PCS1 to (100.0,50.0,0.0,0.0,0.0,0.0) in MCS.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSetTrafoParameter paramSet;
//! paramSet.coordSystem = MLPI_ROBOT_CS_PCS1;
//! paramSet.posX = 100;
//! paramSet.posY = 50;
//! paramSet.posZ = 0;
//! paramSet.rotA = 0.0;
//! paramSet.rotB = 0.0;
//! paramSet.rotB = 0.0;
//! MLPIRESULT result = mlpiRobotSetCylindricTransform(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetCylindricTransform(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotSetTrafoParameter* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibConfig
//! This function gets configuration values of transformation. This function can be called for MLPI_ROBOT_CS_BCS and MLPI_ROBOT_CS_PCSx.
//! For the future: ACS_BCS is not supported yet
//! @param[in]      connection        Handle for multiple connections.
//! @param[in]      group             Reference to group.
//! @param[in,out]  trafoValues       Structure containing all information
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the origin of MLPI_ROBOT_CS_BCS in MCS.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotGetTrafoParameter trafoValues;
//! trafoValues.coordSystem = MLPI_ROBOT_CS_BCS;
//! trafoValues.csType = MLPI_ROBOT_CARTESIAN;
//! MLPIRESULT result = mlpiRobotGetTransform(connection, group, &trafoValues);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetTransform(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotGetTrafoParameter* trafoValues);


//! @ingroup RobotLibMovement
//! This function lets the robot wait at last commanded position for a specified time in seconds.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   time                Time to wait in seconds 
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // lets the robot wait one second at last commanded position
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MLPIRESULT result = mlpiRobotWait(connection, group, 1, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotWait(const MLPIHANDLE connection, const MlpiGroupRef group, const DOUBLE time, MLPIMOTIONHANDLE* motionHandle);

//! @ingroup RobotLibConfig
//! This function reads a belt configuration.
//! @param[in]       connection       Handle for multiple connections.
//! @param[in]       group            Reference to group.
//! @param[in,out]   trafoValues      Structure containing all information
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get the belt configuration.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiAxisRef belt;
//! belt.axisNo = MLPI_AXIS_4;
//! belt.controlNo = MLPI_LOCAL_CONTROL;
//! MlpiRobotGetBeltConfiguration trafoValues;
//! trafoValues.belt = belt;
//! MLPIRESULT result = mlpiRobotGetBeltConfiguration(connection, group, &trafoValues);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetBeltConfiguration(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotGetBeltConfiguration* trafoValues);


//! @ingroup RobotLibConfig
//! This function clears all errors of the group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group to reset all errors from
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Clear all errors of the group.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MLPIRESULT result = mlpiRobotReset(connection, group, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotReset(const MLPIHANDLE connection, const MlpiGroupRef group, MLPIMOTIONHANDLE* motionHandle);

//! @ingroup RobotLibInfo
//! This function reads the status of a movement command
//! @param[in]   connection    Handle for multiple connections.
//! @param[in]   group         Reference to the group.
//! @param[in]   motionHandle  Handle of the motion command where the status should be requested.
//! @param[out]  status        Pointer to data where status will be stored.
//! @return                    Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read status of a movement command.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMotionStatus status;
//! MLPIRESULT result = mlpiRobotMotionGetStatus(connection, group, motionHandle, &status);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotMotionGetStatus(const MLPIHANDLE connection, const MlpiGroupRef group, const MLPIMOTIONHANDLE motionHandle, MlpiRobotMotionStatus* status);


//! @ingroup RobotLibInfo
//! This function enables/disables the performance measurement of the core of the defined group
//! @param[in]   connection    Handle for multiple connections.
//! @param[in]   group         Reference to the group.
//! @param[in]   enable        true to start measurement, false to stop measurement
//! @return                    Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Enable the performance measurement of the core of the defined group.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MLPIRESULT result = mlpiRobotSetPerformanceEnable(connection, group, TRUE);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetPerformanceEnable(const MLPIHANDLE connection, const MlpiGroupRef group, const BOOL8 enable);


//! @ingroup RobotLibInfo
//! This function reads the performance measurement result from the defined group.
//! @param[in]     connection         Handle for multiple connections.
//! @param[in]     group              Reference to the group.
//! @param[out]    performanceResult  Performance measurement result.
//! @param[in]     numElements        Count of performanceResult array size.
//! @param[out]    numElementsRet     Return count of performanceResult.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read performance measurement result from defined group.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPerformanceResult arPerfResults[100];
//! ULONG ulPerfCount = 0;
//! MLPIRESULT result = mlpiRobotGetPerfomanceResult(connection, group, arPerfResults, 100, &ulPerfCount);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetPerfomanceResult(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotPerformanceResult* performanceResult, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup RobotLibInfo
//! This function reads the mechanical data from the defined group.
//! @param[in]     connection         Handle for multiple connections.
//! @param[in]     group              Reference to the group.
//! @param[out]    mechanicData       Mechanical data.
//! @param[in]     numElements        Count of mechanical data array size.
//! @param[out]    numElementsRet     Return count of mechanical data read.
//! @param[in]     updateOnly         true to update only, false to refresh everything
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the mechanical data from the defined group
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotMechanicData mechanicalData[100];
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiRobotGetMechanicData(connection, group, &mechanicalData, 100, &numElementsRet, TRUE);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetMechanicData(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotMechanicData* mechanicData, const ULONG numElements, ULONG *numElementsRet, BOOL8 updateOnly);


//! @ingroup RobotLibConfig
//! This function adds all configured axis to a group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Add all configured axis to a group.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MLPIRESULT result = mlpiRobotAddAllAxisToGroup(connection, group);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotAddAllAxisToGroup(const MLPIHANDLE connection, const MlpiGroupRef group);


//! @ingroup RobotLibConfig
//! This function removes all configured axis from a group
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Remove all configured axis from a group.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MLPIRESULT result = mlpiRobotRemAllAxisFromGroup(connection, group);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotRemAllAxisFromGroup(const MLPIHANDLE connection, const MlpiGroupRef group);


//! @ingroup RobotLibInfo
//! This function returns an array of MlpiKinematicsValues structures. Use the @c group element of the structure to specify the
//! kinematics for which information should be read.
//! You may want to use this structure to read several sets of kinematics information for several kinematics using one single
//! function call during operation of the kinematics. This provides increased performance in comparison to reading the values
//! bit by bit and kinematics by kinematics.
//! @param[in]  connection        Handle for multiple connections.
//! @param[in]  groups            Reference to groups.
//! @param[out] kinematicsValues  Returns a struct with the current operation information about kinematics.
//! @param[in]  numElements       Number of kinematics for which values should be read. This is the array length of the parameter @c kinematicsValues.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @note Elements of the struct (beside controlNo and groupNo) that can not be read will be set to 0.
//!
//! @par Example:
//! @code
//! // Get KinematicsValues of defined groups.
//! const ULONG numElements = 2;
//! MlpiGroupRef groups[2];
//! groups[0].controlNo = MLPI_LOCAL_CONTROL;
//! groups[0].groupNo = (MlpiGroupNumber)(1);
//! groups[1].controlNo = MLPI_LOCAL_CONTROL;
//! groups[1].groupNo = (MlpiGroupNumber)(2);
//! MlpiKinematicsValues kinematicsValues[2];
//! MLPIRESULT result = mlpiRobotGetKinematicsValues(connection, groups, kinematicsValues, numElements);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetKinematicsValues(const MLPIHANDLE connection, MlpiGroupRef* groups, MlpiKinematicsValues* kinematicsValues, const ULONG numElements);


//! @ingroup RobotLibInfo
//! This function reads the diagnostic message of the kinematics.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    group               Reference to the group.
//! @param[out]   buffer              Pointer to where text should be stored.
//! @param[in]    numElements         Number of available WCHAR16 characters in buffer.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get DiagnosisText of defined group.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! WCHAR16 buffer[BUFFERLENGTH];
//! MLPIRESULT result = mlpiRobotGetDiagnosisText(connection, group, buffer, BUFFERLENGTH );
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetDiagnosisText(const MLPIHANDLE connection, const MlpiGroupRef group, WCHAR16 *buffer, const ULONG numElements);


//! @ingroup RobotLibInfo
//! This function returns the current unit settings of an array of kinematics in string representation. Use it to read the unit settings for
//! display in an HMI.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    groups              Reference to groups.
//! @param[out]   kinematicsUnits     Returns a struct with the current unit settings of the kinematics.
//! @param[in]    numElements         Number of kinematics for which values should be read. This is the array length of the parameter @c kinematicsUnits.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Get KinematicsUnits of defined groups.
//! const ULONG numElements = 2;
//! MlpiGroupRef groups[2];
//! groups[0].controlNo = MLPI_LOCAL_CONTROL;
//! groups[0].groupNo = (MlpiGroupNumber)(1);
//! groups[1].controlNo = MLPI_LOCAL_CONTROL;
//! groups[1].groupNo = (MlpiGroupNumber)(2);
//! MlpiKinematicsUnits kinematicsUnits[numElements];
//! MLPIRESULT result = mlpiRobotGetKinematicsUnits(connection, groups, kinematicsUnits, numElements);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetKinematicsUnits(const MLPIHANDLE connection, MlpiGroupRef* groups, MlpiKinematicsUnits* kinematicsUnits, const ULONG numElements);


//! @ingroup RobotLibConfig
//! This function configures a save zone of a robot.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    group               Reference to group.
//! @param[in]    safeZoneData        Data of save zone.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Set safe zone data of the defined group.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSafeZoneData paramSet;
//! paramSet.pointNeg[0] = 50;
//! paramSet.pointPos[0] = 100;
//! paramSet.zoneNumber = 1;
//! paramSet.zoneType = MLPI_ROBOT_SAFE_ZONE_TYPE_INSIDE_OK;
//! MLPIRESULT result = mlpiRobotSetSafeZone(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetSafeZone(const MLPIHANDLE connection, MlpiGroupRef group, MlpiRobotSafeZoneData* safeZoneData, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibInfo
//! This function gets all safezones of a kinematics.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    group               Reference to group.
//! @param[out]   safeZoneData        Data of save zone.
//! @param[in]    numElements         Number of safezones that should be read. This is the array length of the parameter @c safeZoneData.
//! @param[out]   numElementsRet      Return count of safezones.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read all safe zones of a kinematics.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotSafeZoneData paramSet[MLPI_ROBOT_MAX_SAFEZONES];
//! ULONG numRetElements = 0;
//! MLPIRESULT result = mlpiRobotGetSafeZones(connection, group, &paramSet, MLPI_ROBOT_MAX_SAFEZONES, &numRetElements);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetSafeZones(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotSafeZoneData* safeZoneData, const ULONG numElements,  ULONG* numElementsRet);


//! @ingroup RobotLibMovement
//! This function jogs a kinematics in steps
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Jog a distance in x-direction.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPoint P1 = {0};
//! P1.coordinate[0] = 100.0;
//! P1.coordinate[1] = 0.0;
//! P1.coordinate[2] = 0.0;
//! MlpiRobotJogStepData paramSet;
//! paramSet.acceleration = 1200;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.deceleration = 1200;
//! paramSet.distance = P1;
//! paramSet.jerk = 20000;
//! paramSet.velocity = 2000;
//! MLPIRESULT result = mlpiRobotJogStep(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotJogStep(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotJogStepData* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function jogs a kinematics continuously
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Jog in x-direction.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPoint P1 = {0};
//! P1.coordinate[0] = 100.0;
//! P1.coordinate[1] = 0.0;
//! P1.coordinate[2] = 0.0;
//! MlpiRobotJogContData paramSet;
//! paramSet.acceleration = 1200;
//! paramSet.coordSystem = MLPI_ROBOT_CS_MCS;
//! paramSet.deceleration = 1200;
//! paramSet.direction = P1;
//! paramSet.jerk = 20000;
//! paramSet.velocity = 2000;
//! MLPIRESULT result = mlpiRobotJogCont(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotJogCont(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotJogContData* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibMovement
//! This function stops a command
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Stop the command.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotStopCmdData paramSet;
//! paramSet.cmdID = motionHandle;
//! MLPIRESULT result = mlpiRobotStopCmd(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotStopCmd(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotStopCmdData* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibConfig
//! This function configures relative desynchronization from a belt.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the function.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Configure relative desynchronization from the defined belt.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotBeltDesyncRelData paramSet;
//! paramSet.belt.axisNo = MLPI_AXIS_4;
//! paramSet.belt.controlNo = MLPI_LOCAL_CONTROL;
//! paramSet.endPercent = 50.0;
//! paramSet.startPercent = 25.0;
//! MLPIRESULT result = mlpiRobotSetBeltDesyncRelConfig(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetBeltDesyncRelConfig(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotBeltDesyncRelData* paramSet);


//! @ingroup RobotLibConfig
//! This function reads relative desynchronization from a belt configuration.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the function.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read relative desynchronization from the defined belt.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotBeltDesyncRelData paramSet;
//! paramSet.belt.axisNo = MLPI_AXIS_4;
//! paramSet.belt.controlNo = MLPI_LOCAL_CONTROL;
//! MLPIRESULT result = mlpiRobotGetBeltDesyncRelConfig(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetBeltDesyncRelConfig(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotBeltDesyncRelData* paramSet);


//! @ingroup RobotLibTeachIn
//! This function saves or modifies a point for the TeachIn.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Save the defined point for the TeachIn.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotPoint P1 = {0};
//! P1.coordinate[0] = 100.0;
//! P1.coordinate[1] = 0.0;
//! P1.coordinate[2] = 0.0;
//! MlpiRobotTeachInDataWrite paramSet;
//! paramSet.teachPoint = P1;
//! paramSet.usOptions = MLPI_ROBOT_TEACH_IN_WRITE_POINT_SIMPLE;
//! paramSet.wCoordSys = MLPI_ROBOT_CS_MCS;
//! memcpy(paramSet.pointName, L"Point1", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! MLPIRESULT result = mlpiRobotTeachInWritePoint(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInWritePoint(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotTeachInDataWrite* paramSet);

//! @ingroup RobotLibTeachIn
//! This function returns a point of the TeachIn map.
//! @param[in]     connection          Handle for multiple connections.
//! @param[in]     group               Reference to group.
//! @param[in,out] paramSet            Structure containing all information necessary for the command.
//! @return                            Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read a point of the TeachIn map.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotTeachInDataRead paramSet;
//! memcpy(paramSet.pointName, L"Point1", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! MLPIRESULT result = mlpiRobotTeachInReadPoint(connection, group, &paramSet);
//! MlpiRobotPoint P1 = paramSet.teachPoint;
//! MlpiRobotCoordinateSystem CoordSys =  paramSet.wCoordSys;
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInReadPoint(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotTeachInDataRead *paramSet);


//! @ingroup RobotLibTeachIn
//! This function returns the next point of the TeachIn map.
//! @param[in]     connection          Handle for multiple connections.
//! @param[in]     group               Reference to group.
//! @param[in,out] paramSet            Structure containing all information necessary for the command.
//! @return                            Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! //Read the next point of the TeachIn map.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotTeachInDataReadNext paramSet;
//! memcpy(paramSet.pointNameInput, L"Point1", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! MLPIRESULT result = mlpiRobotTeachInReadNextPoint(connection, group, &paramSet);
//! WCHAR16 nextPoint[MLPI_ROBOT_MAX_TEACH_NAME_LEN];
//! memcpy(nextPoint, paramSet.pointNameInput, MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! MlpiRobotPoint P1 = paramSet.teachPoint;
//! MlpiRobotCoordinateSystem CoordSys =  paramSet.wCoordSys;
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInReadNextPoint(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotTeachInDataReadNext *paramSet);


//! @ingroup RobotLibTeachIn
//! This function deletes points of the TeachIn map. For delete of multiple points use "*" at the end of the name.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Delete the defined point of the TeachIn map.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotTeachInDataDeletePoints paramSet;
//! memcpy(paramSet.pointName, L"Point1", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! MLPIRESULT result = mlpiRobotTeachInDeletePoints(connection, group, &paramSet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInDeletePoints(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotTeachInDataDeletePoints *paramSet);


//! @ingroup RobotLibTeachIn
//! This function will save the actual map to a file
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Save the actual map to the file.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotTeachInDataFileData paramSet;
//! memcpy(paramSet.fileName, L"TestFile", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! paramSet.fileOptions = MLPI_ROBOT_TEACH_IN_NEW;
//! MLPIRESULT result = mlpiRobotTeachInSavePointFile(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInSavePointFile(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotTeachInDataFileData* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibTeachIn
//! This function will load the map from a file
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Load the map from a file.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotTeachInDataFileData paramSet;
//! memcpy(paramSet.fileName, L"TestFile", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! paramSet.fileOptions = MLPI_ROBOT_TEACH_IN_NEW;
//! MLPIRESULT result = mlpiRobotTeachInLoadPointFile(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInLoadPointFile(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotTeachInDataFileData* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibTeachIn
//! This function will  delete a point file
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   group               Reference to group.
//! @param[in]   paramSet            Structure containing all information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Delete a point file.
//! MLPIMOTIONHANDLE motionHandle;
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotTeachInDataDeleteFileData paramSet;
//! memcpy(paramSet.fileName, L"TestFile", MLPI_ROBOT_MAX_TEACH_NAME_LEN*sizeof(WCHAR16));
//! MLPIRESULT result = mlpiRobotTeachInDeletePointFile(connection, group, &paramSet, &motionHandle);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotTeachInDeletePointFile(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotTeachInDataDeleteFileData* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup RobotLibInfo
//! This function will return information about the supported coordinate systems from the kinematics.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in]      group               Reference to group.
//! @param[out]     csInfo              Structure containing all information necessary for the command.
//! @param[in]      numElements         Number of elements.
//! @param[out]     numElementsRet      Number of elements returned.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read information about the supported coordinate systems from the kinematics.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotCoordinateSystemInfo coordinateSystems[19];
//! ULONG numElemsRet;
//! MLPIRESULT result = mlpiRobotGetCoordinateSystemInfo(connection, group, coordinateSystems, 19, &numElemsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetCoordinateSystemInfo(const MLPIHANDLE connection, const MlpiGroupRef group, MlpiRobotCoordinateSystemInfo *csInfo, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup RobotLibInfo
//! This function will return actual status information about kinematics in a specified coordinate systems.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in,out]  statusSummary       Structure containing all information necessary for the command.
//! @param[in]      numElements         Number of elements.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read actual status information.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotGetStatusSummary statusSummary[2];
//! statusSummary[0].group.controlNo = MLPI_LOCAL_CONTROL;
//! statusSummary[0].group.groupNo = MLPI_GROUP_1;
//! statusSummary[0].coordSys = MLPI_ROBOT_CS_MCS;
//! statusSummary[1].group.controlNo = MLPI_LOCAL_CONTROL;
//! statusSummary[1].group.groupNo = MLPI_GROUP_2;
//! statusSummary[1].coordSys = MLPI_ROBOT_CS_ACS;
//! MLPIRESULT result = mlpiRobotGetStatusSummary(connection, statusSummary, 2);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetStatusSummary(const MLPIHANDLE connection, MlpiRobotGetStatusSummary *statusSummary, const ULONG numElements);

//! @ingroup RobotLibInfo
//! This function will read the block id and the progress of the currently executed move command.
//! The integer value in front of the dot represents the block id. The decimal part after the dot shows the progress of a movement.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in]      group               Reference to the group.
//! @param[out]     moveId              Id and progress of the currently executed command.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read block id
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! DOUBLE moveId = 0.0;
//! MLPIRESULT result = mlpiRobotGetMoveID(connection, group, &moveId);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetMoveId(const MLPIHANDLE connection, const MlpiGroupRef group, DOUBLE* moveId);

//! @ingroup RobotLibConfig
//! This function will set an id in the block buffer for a command. This function can be called at any point. The block id will be incremented after the call of a 
//! movement command. The block id can be reseted with the id zero.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in]      group               Reference to the group.
//! @param[in]      moveId              To be set id.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Set block id.
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! ULONG moveId = 150;
//! MLPIRESULT result = mlpiRobotSetMoveID(connection, group, moveId);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotSetMoveId(const MLPIHANDLE connection, const MlpiGroupRef group, const ULONG moveId);

//! @ingroup RobotLibInfo
//! This function reads the command info from the defined group.
//! @param[in]     connection         Handle for multiple connections.
//! @param[in]     group              Reference to the group.
//! @param[in]     infoType           Type of information.
//! @param[out]    cmdInfo            Command info.
//! @param[in]     numElements        Count of command info array size.
//! @param[out]    numElementsRet     Return count of command info read
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // Read the command info from the defined group
//! MlpiGroupRef group;
//! group.controlNo = MLPI_LOCAL_CONTROL;
//! group.groupNo = MLPI_GROUP_1;
//! MlpiRobotCmdInfoType infoType = MLPI_ROBOT_CMD_DATA_TYPE_UID;
//! MlpiRobotCmdInfo cmdInfo[100];
//! ULONG numElementsRet = 0;
//! MLPIRESULT result = mlpiRobotGetCmdInfo(connection, group, infoType, cmdInfo, 100, &numElementsRet);
//! @endcode
MLPI_API MLPIRESULT mlpiRobotGetCmdInfo(const MLPIHANDLE connection, const MlpiGroupRef group, const MlpiRobotCmdInfoType infoType, MlpiRobotCmdInfo* cmdInfo, const ULONG numElements, ULONG *numElementsRet);


#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIROBOTLIB_H__

