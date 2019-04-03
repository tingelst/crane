#ifndef __MLPIMOTIONLIB_H__
#define __MLPIMOTIONLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiMotionLib.h>
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




//! @addtogroup MotionLib MotionLib
//! @{
//! @brief This motion library provides functionality for defining an axis, commanding
//! motion and getting axis information.
//!
//! @note There are numerous getter and setter for accessing axis properties like position, velocity ... .
//!       All these functions using parameter accesses to read and write values. This can take some time
//!       as this parameter accesses are concurrent with other parameter accesses of parts of the system
//!       (IndraWorks, HMI ...). In this case a short access resp. real time access cannot be guaranteed.
//!       For a fast real time access you have to use a container with parts of the global PLC variable
//!       <c>AxisData</c> of the regarding axes. This variable is part of the GVL <c>"GVL_Base"</c> of
//!       the library <c>"BASELIB"</c>.\n
//!       For more information about the contents and format of the global PLC variable <c>AxisData</c>,
//!       type <c>"ML_AXIS_DATA_SM"</c> please consult the IndraWorks help.
//!
//! The conceptual difference between a drive or device and an axis is
//! that a drive or device is the physical device that is connected to the
//! sercos bus and can be configured via S and P parameters.
//! An axis is assigned to a drive or device and is accessible through
//! A parameters. Multiple axes can be assigned to a drive or device (for
//! example, one axis is a real axis that is moving the drive, a second
//! axis is connected to the same drive, but implements an encoder axis).
//!
//! @note You find a more detailed description of the concept of axis in the IndraWorks help system
//! of the XLC/MLC product.
//!
//! The following functions all deal with the motion of an axis. In general, "axis" is used to address an axis.
//! The motion commands can be divided roughly in single axis commands, where a single axis is moved,
//! synchronous commands, where axes are moved in relation to master and utility functions like power
//! or homing of an axis. Each motion command usually uses a structure to pass on the information about
//! a particular move. The command will be executed in the control and returns a handle. This handle must
//! be used in future requests for getting status information. The handle and the axis reference a particular
//! motion command. One axis can have multiple active commands (for example, power is one and a move is
//! commanded). On the contrary, a currently executed move will be interrupted as soon as a new move is
//! commanded.
//!
//! @note There are different axis types and an axis can be linked to different drive types. Thus, not every
//!       command is supported by each different axis. If a command returns an error, then please check if
//!       you are operating with the expected axis type. Check the drive or control documentation for additional
//!       information about the motion system.
//!
//! @note The MotionLib functions trace their debug information mainly into the module MLPI_MOTION_LIB
//!       and in addition into the modules MCI_ADMIN* and MLPI_BASE_MODULES. For further information,
//!       see also the detailed description of the library @ref TraceLib and the notes about
//!       @ref sec_TraceViewer.
//!
//! @}

//! @addtogroup MotionLibConfiguration Motion Configuration
//! @ingroup MotionLib
//! @{
//! @brief The following functions can be used for the configuration of motion settings.
//! @}

//! @addtogroup MotionLibMovement Movement Functions
//! @ingroup MotionLib
//! @{
//! @brief The following functions can be used to command and move an axis.
//! The following axis shows how to do a single axis movement. For this example to work, you
//! need to add an axis to your axis configuration and switch the control to BB. It is recommended
//! to do this using IndraWorks.
//!
//! @note If you get a permission error, when trying to move an axis, then the user profile you are using
//!       does not have enough permissions. See @ref sec_Permission which explains the permission
//!       handling of the MLPI. See @ref mlpiApiConnect on how to login as a different user.
//!
//! We begin the example by checking if the axis is available and what type of axis
//! we want to move. Only virtual and real axis can be commanded.
//!
//! @code
//! MlpiAxisRef axis(1);    // change to select an available axis number
//!
//! // let's read the axis type to check if this example is working with the given
//! // axis address
//! USHORT axisType;
//! MLPIRESULT result = mlpiMotionGetAxisType(connection, axis, &axisType);
//! if (MLPI_FAILED(result)){
//!   printf("\nCould not read axis type of axis %d with error 0x%08x. Is the axis address correct and the axis configured?", axis.axisNo, result);
//!   return -1;
//! }
//!
//! if (MlpiAxisTypeDecoder(axisType).Type() != MLPI_AXISTYPE_REAL && MlpiAxisTypeDecoder(axisType).Type() != MLPI_AXISTYPE_VIRTUAL) {
//!   printf("\nThis example is only working with real or virtual axis!");
//!   return -1;
//! }
//! @endcode
//!
//! The following lines perform a check to make sure that we can move the axis and that there
//! are no configuration errors.
//!
//! @code
//! // check if axis is referenced.
//! ULONG axisState = 0;
//! result = mlpiMotionGetState(connection, axis, &axisState);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // we use the helper struct from #include <util/mlpiMotionHelper.h> to decode the axisState
//! MlpiAxisStateDecoder state = axisState;
//!
//! if (state.Error()) {
//!   // axis is in error state. Clear error!
//!   result = mlpiMotionClearError(connection, axis);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//! }
//!
//! if (!state.InAb()) {
//!   // axis needs to be in AB. Otherwise, we cannot switch to power or home.
//!   printf("\nAxis is not in AB. Cannot continue with example. Switch axis to AB and start example again!");
//!   return -1;
//! }
//!
//! if (!state.Homed()) {
//!   // axis is not homed, let's set absolute measurement reference at current position
//!   result = mlpiMotionSetAbsoluteMeasurement(connection, axis);
//!   if (MLPI_FAILED(result)) {
//!     printf("\nCould not set absolute reference. This example needs an axis with absolute encoder! Error: 0x%08x", (unsigned)result);
//!     return -1;
//!   }
//! }
//! @endcode
//!
//! We are now ready to start the axis movement. Please note that all axis command functions have asynchronous behavior.
//! This means that when the MLPI function returns, the command has not been finished yet. It might even not have been started yet.
//! The function tells the axis to abort the current command and start with the new movement as soon as possible
//! (e.g. next motion cycle). After setting the new command, the MLPI function returns immediately. It returns an error code
//! if the new command has not been accepted by the axis.
//! If the motion command has been accepted, the axis will start to execute your command. For example, moving to a given absolute
//! position if you have commanded it using @ref mlpiMotionMoveAbsolute.
//! Other than the return code, the movement function also returns a so called 'Motion-Handle' (@ref MLPIMOTIONHANDLE) as a
//! output argument. You can use this output handle at any time with the function @ref mlpiMotionGetStatus to
//! check the current state of the axis movement. This way, you can check if the axis has already finished the
//! movement (e.g. reached the target position) or an error has occurred.
//!
//! In the example below, the helper function utilMotionWait is used to poll if the axis has finished the given motion command.
//! It blocks the current thread until the command has been finished by the axis. It is also possible to give a timeout to
//! the function.
//! Have a look at the source code of utilMotionWait to see an example of how @ref mlpiMotionGetStatus can be used.
//!
//! @code
//! // we need a motion handle for our motion commands
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! //
//! // switch power ON
//! //
//! if (MlpiAxisTypeDecoder(axisType).Type() == MLPI_AXISTYPE_REAL) {
//!   MlpiMotionPower cmdPower;
//!   cmdPower.axis = axis;
//!   cmdPower.power = TRUE;
//!
//!   result = mlpiMotionPower(connection, &cmdPower, &motionHandle);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//!
//!   // wait until command power on is finished and axis has power
//!   result = utilMotionWait(connection, axis, motionHandle, 1000);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//! }
//!
//! //
//! // let's do some absolute positioning (MoveAbsolute)
//! //
//! const MlpiMotionMoveAbsolute cmdAbsolute[] = {
//!   //      Pos Vel Acc Dec Jrk
//!   {axis,   0, 100, 10, 10,  0},
//!   {axis,  90, 100, 40, 10,  0},
//!   {axis, 180,  50, 10, 10,  0},
//!   {axis, 270, 100, 40,  5,  0},
//!   {axis,  90, 100, 40,  5,  0},
//!   {axis,   0,  10, 10, 10,  0}
//! };
//!
//! for (ULONG i=0; i<_countof(cmdAbsolute); i++) {
//!   printf("\nNew MoveAbsolute to position %lf", cmdAbsolute[i].position);
//!
//!   // command MoveAbsolute to axis
//!   result = mlpiMotionMoveAbsolute(connection, &cmdAbsolute[i], &motionHandle);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//!
//!   // wait until command power on is finished and has reached target position
//!   result = utilMotionWait(connection, axis, motionHandle, 10000);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//! }
//!
//! //
//! // try some velocity movement (MoveVelocity)
//! //
//! const MlpiMotionMoveVelocity cmdVelocity[] = {
//!   //     Vel Acc Dec Jrk
//!   {axis, 100, 10, 10,  0},
//!   {axis, 500, 40, 10,  0},
//!   {axis,  50, 10, 10,  0},
//!   {axis, 500, 40,  5,  0},
//!   {axis, 100, 40,  5,  0},
//!   {axis, 500, 10, 10,  0}
//! };
//!
//! for (ULONG i=0; i<_countof(cmdAbsolute); i++) {
//!   printf("\nNew MoveVelocity to velocity %lf", cmdVelocity[i].velocity);
//!
//!   // command MoveVelocity to axis
//!   result = mlpiMotionMoveVelocity(connection, &cmdVelocity[i], &motionHandle);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//!
//!   // wait until command power on is finished and has reached target velocity
//!   result = utilMotionWait(connection, axis, motionHandle, 10000);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//! }
//!
//! //
//! // STOP axis
//! //
//! MlpiMotionStop cmdStop;
//! cmdStop.axis = axis;
//! cmdStop.stop = TRUE;
//! cmdStop.deceleration = 100;
//! cmdStop.jerk = 0;
//!
//! // command Stop to axis
//! result = mlpiMotionStop(connection, &cmdStop, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // wait until axis has stopped
//! result = utilMotionWait(connection, axis, motionHandle, 10000);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // after the axis stops, we have to call the stop command again
//! // with "FALSE" to release axis from "STOPPED" state.
//! // As long as the axis is in state "STOPPED", no other motion commands
//! // will be allowed!
//! cmdStop.stop = FALSE;
//! result = mlpiMotionStop(connection, &cmdStop, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! //
//! // switch power OFF
//! //
//! if (MlpiAxisTypeDecoder(axisType).Type() == MLPI_AXISTYPE_REAL) {
//!   MlpiMotionPower cmdPower;
//!   cmdPower.axis = axis;
//!   cmdPower.power = FALSE;
//!
//!   result = mlpiMotionPower(connection, &cmdPower, &motionHandle);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//!
//!   // wait until command power on is finished and axis has power off
//!   result = utilMotionWait(connection, axis, motionHandle, 1000);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return -1;
//!   }
//! }
//! @endcode
//! @}

//! @addtogroup MotionLibVersionPermission Version and Permission
//! @ingroup MotionLib
//! @{
//! @addtogroup MotionLibVersionPermission_new Server version since 1.26.0.0 (MLC-FW: 14V22)
//! @ingroup MotionLibVersionPermission
//!
//! @note Since firmware version 14V22 (MLPI-Server-Version: 1.26.0.0) a centralized permission management has been implemented in target 
//! controls XM2, L75 and VPx. Some permissions have been summarized in order to improve their usability. 
//! Additional information regarding the usage of older manifest files (i.e. accounts.xml) with newer server versions can be found in @ref newest_manifest.\n
//! @note <b><span style="color:red">Users of other CML controls (i.e. L25, L45, L65) have to use the old permissions as defined in @ref MotionLibVersionPermission_old</span></b>
//!
//!
//! @par List of valid permissions for mlpiMotionLib. These permissions shall be assigned to the groups (i.e. in the group manifest file groups.xml) rather than the users.
//! <TABLE>
//! <TR><TH> Permission-Ident        </TH><TH> Description                                                                          </TH></TR>                  
//! <TR><TD id="st_e"> MOTION_INFO   </TD><TD> Monitor axes - Allows to monitor axes, e.g. current position, current velocity etc.  </TD></TR>  
//! <TR><TD id="st_e"> MOTION_MOVE   </TD><TD> Move axes - Allows to move axes                                                      </TD></TR>  
//! <TR><TD id="st_e"> MOTION_SETUP  </TD><TD> Create, delete and configure axes - Allows to create, delete and configure axes.     </TD></TR>
//! </TABLE>
//!
//!  @par List of available functions in mlpiMotionLib and the permissions required for their use. 
//! <TABLE>
//! <TR><TH>           Function                                   </TH><TH> Server version </TH><TH> Permission-Ident </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetAbsoluteMeasurement      </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionChangeFlexProfileSet        </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionCreateAxis                  </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionDestroyAxis                 </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetConfiguredAxes           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualPosition           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualVelocity           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualAcceleration       </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualTorque             </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetInterpolatedPosition     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetInterpolatedVelocity     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetInterpolatedTorque       </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetPositionLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetPositionLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetPositionLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetPositionLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAccelerationLimitBip     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetAccelerationLimitBip     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetJerkLimitBip             </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetJerkLimitBip             </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetTorqueLimitBip           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetTorqueLimitBip           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetPositionScaling          </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetPositionScaling          </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityScaling          </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityScaling          </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAccelerationScaling      </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetAccelerationScaling      </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetModulo                   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetModulo                   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetSlaveDriveFeedTravel     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetSlaveDriveFeedTravel     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetState                    </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetStateExtended            </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetDiagnosisNumber          </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetDiagnosisText            </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetName                     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetName                     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisType                 </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetCondition                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetCondition                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionLoadDefaultParameters       </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionClearError                  </TD><TD> 1.0.0.0        </TD><TD> "MOTION_SETUP"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisValues               </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisUnits                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetStatus                   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionPower                       </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionStop                        </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionHome                        </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveVelocity                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveAbsolute                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveAdditive                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveRelative                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionTorqueControl               </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicPositionChannel   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicVelocityChannel   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicAnalogChannel     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicTorqueChannel     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicPosition         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicVelocity         </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicAnalog           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicTorque           </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionControlOn                   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionControlOff                  </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGearIn                      </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGearInPos                   </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionCamIn                       </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMotionProfile               </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionFlexProfile                 </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSynchronOut                 </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionPhasing                     </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionPhasingSlave                </TD><TD> 1.0.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionAddAxisToGroup              </TD><TD> 1.6.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionRemAxisFromGroup            </TD><TD> 1.6.0.0        </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityLimitBip         </TD><TD> 1.10.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityLimitBip         </TD><TD> 1.10.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetTorqueLimitPos           </TD><TD> 1.10.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetTorqueLimitPos           </TD><TD> 1.10.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetTorqueLimitNeg           </TD><TD> 1.10.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetTorqueLimitNeg           </TD><TD> 1.10.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveContinuousAbsolute      </TD><TD> 1.12.0.0       </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveContinuousRelative      </TD><TD> 1.12.0.0       </TD><TD> "MOTION_MOVE"    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisStatus               </TD><TD> 1.23.0.0       </TD><TD> "MOTION_INFO"    </TD></TR>
//! </TABLE>                                                                                          
//!
//! @par List of the old permissions of mlpiMotionLib and their corresponding new permission.
//! <TABLE>
//! <TR><TH> Old permission                             </TH><TH> new Permission  </TH></TR>                  
//! <TR><TD id="st_e"> MLPI_MOTIONLIB_PERMISSION_ALWAYS </TD><TD> IMPLICIT        </TD></TR>  
//! <TR><TD id="st_e"> MLPI_MOTIONLIB_PERMISSION_SETUP  </TD><TD> MOTION_SETUP    </TD></TR>  
//! <TR><TD id="st_e"> MLPI_MOTIONLIB_PERMISSION_INFO   </TD><TD> MOTION_INFO     </TD></TR>  
//! <TR><TD id="st_e"> MLPI_MOTIONLIB_PERMISSION_MOVE   </TD><TD> MOTION_MOVE     </TD></TR>  
//! </TABLE>
//!
//! @addtogroup MotionLibVersionPermission_old Server versions before 1.26.0.0 
//! @ingroup MotionLibVersionPermission
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_MOTIONLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_MOTIONLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiMotionLib using in accounts.xml
//! - MLPI_MOTIONLIB_PERMISSION_ALL
//! - MLPI_MOTIONLIB_PERMISSION_INFO
//! - MLPI_MOTIONLIB_PERMISSION_SETUP
//! - MLPI_MOTIONLIB_PERMISSION_MOVE
//!
//! <TABLE>
//! <TR><TH>           Function                                   </TH><TH> Server version </TH><TH> Permission                           </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetAbsoluteMeasurement      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionChangeFlexProfileSet        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionCreateAxis                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionDestroyAxis                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetConfiguredAxes           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualPosition           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualVelocity           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualAcceleration       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetActualTorque             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetInterpolatedPosition     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetInterpolatedVelocity     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetInterpolatedTorque       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetPositionLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetPositionLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetPositionLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetPositionLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityLimitPos         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityLimitNeg         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAccelerationLimitBip     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetAccelerationLimitBip     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetJerkLimitBip             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetJerkLimitBip             </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetTorqueLimitBip           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetTorqueLimitBip           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetPositionScaling          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetPositionScaling          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityScaling          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityScaling          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAccelerationScaling      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetAccelerationScaling      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetModulo                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetModulo                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetSlaveDriveFeedTravel     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetSlaveDriveFeedTravel     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetState                    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetStateExtended            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetDiagnosisNumber          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetDiagnosisText            </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetName                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetName                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisType                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetCondition                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetCondition                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionLoadDefaultParameters       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionClearError                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_SETUP"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisValues               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisUnits                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetStatus                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionPower                       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionStop                        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionHome                        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveVelocity                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveAbsolute                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveAdditive                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveRelative                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionTorqueControl               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicPositionChannel   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicVelocityChannel   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicAnalogChannel     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionOpenCyclicTorqueChannel     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicPosition         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicVelocity         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicAnalog           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionWriteCyclicTorque           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionControlOn                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionControlOff                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGearIn                      </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGearInPos                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionCamIn                       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMotionProfile               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionFlexProfile                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSynchronOut                 </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionPhasing                     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionPhasingSlave                </TD><TD> 1.0.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionAddAxisToGroup              </TD><TD> 1.6.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionRemAxisFromGroup            </TD><TD> 1.6.0.0        </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetVelocityLimitBip         </TD><TD> 1.10.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetVelocityLimitBip         </TD><TD> 1.10.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetTorqueLimitPos           </TD><TD> 1.10.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetTorqueLimitPos           </TD><TD> 1.10.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetTorqueLimitNeg           </TD><TD> 1.10.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionSetTorqueLimitNeg           </TD><TD> 1.10.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveContinuousAbsolute      </TD><TD> 1.12.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionMoveContinuousRelative      </TD><TD> 1.12.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_MOVE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiMotionGetAxisStatus               </TD><TD> 1.23.0.0       </TD><TD> "MLPI_MOTIONLIB_PERMISSION_INFO"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD></TR>
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

//! @addtogroup MotionLibStructTypes Structs, Types, ...
//! @ingroup MotionLib
//! @{
//! @brief List of used types, enumerations, structures and more...



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"


// -----------------------------------------------------------------------
// GLOBAL TYPES
// -----------------------------------------------------------------------
typedef unsigned long long MLPIMOTIONHANDLE;            //!< MLPI-API handle value used for motion functions.

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_MOTION_MAX_AXIS_NAME_LEN        (80)
#define MLPI_MOTION_MAX_UNITS_LEN            (16)

//! @enum MlpiAxisType
//! This enumeration defines the type of axis.
typedef enum MlpiAxisType
{
  MLPI_AXISTYPE_VIRTUAL     = 0,                        //!< Virtual axis, no physical drive attached.
  MLPI_AXISTYPE_REAL        = 1,                        //!< Real axis, this is the common axis when doing motion.
  MLPI_AXISTYPE_ENCODER     = 2,                        //!< An encoder that is attached to a real drive, no motion possible.
  MLPI_AXISTYPE_LINK        = 3,                        //!< A link ring axis.
  MLPI_AXISTYPE_CONTROLLER  = 4                         //!< An axis that can be used when generating your own controller to operate the drive.
}MlpiAxisType;

//! @enum MlpiProfileSetSelection
//! The motion profile or flex profile works based on a set of parameters.
//! In order to change a set consistently, the set that is not in use should
//! be changed. Once the change is done, the set can be verified and then used.
typedef enum MlpiProfileSetSelection
{
  MLPI_PROFILE_SET_0 = 0,                               //!< Set 0 on axis.
  MLPI_PROFILE_SET_1 = 1,                               //!< Set 1 on axis.
  MLPI_PROFILE_SET_2 = 2,                               //!< Set 2 on axis.
  MLPI_PROFILE_SET_3 = 3                                //!< Set 3 on axis.
}MlpiProfileSetSelection;

//! @enum MlpiSyncDirection
//! This enumeration defines the direction in which synchronization takes place.
typedef enum MlpiSyncDirection
{
  MLPI_SYNC_SHORTESTWAY  = 0,                           //!< Axis will synchronize in the direction in which it has to move the shortest distance.
  MLPI_SYNC_CATCHUP      = 1,                           //!< Axis will only move in positive direction.
  MLPI_SYNC_SLOWDOWN     = 2                            //!< Axis will only move in negative direction.
}MlpiSyncDirection;

//! @enum MlpiSyncType
//! This enumeration defines the direction and type in which synchronization takes place. Only for FlexProfile!
typedef enum MlpiSyncType
{
  MLPI_SYNC_RAMPIN_SHORTESTWAY  = 0,                    //!< Axis will synchronize in the direction in which it has to move the shortest distance.
  MLPI_SYNC_RAMPIN_CATCHUP      = 1,                    //!< Axis will only move in positive direction.
  MLPI_SYNC_RAMPIN_SLOWDOWN     = 2,                    //!< Axis will only move in negative direction.
  MLPI_SYNC_DIRECT              = 3                     //!< No sync at all.
}MlpiSyncType;

//! @enum MlpiStartMode
//! This enumeration defines how the synchronization is done.
typedef enum MlpiStartMode
{
  MLPI_STARTMODE_ABSOLUTE        = 0,                   //!< Slave axis position is absolutely synchronous with the master position.
  MLPI_STARTMODE_RELATIVE        = 1,                   //!< Slave axis position is relatively synchronous with the master position (velocity synchronous).
  MLPI_STARTMODE_ABSOLUTE_RAMPIN = 2,                   //!< Slave axis position ramps to an absolutely synchronous position with the master position.
  MLPI_STARTMODE_RELATIVE_RAMPIN = 3                    //!< Slave axis position ramps to a relatively synchronous position with the master position.
}MlpiStartMode;

//! @enum MlpiProfileStartPoint
//! This enumeration defines the method of how a flex profile interprets the start point.
typedef enum MlpiProfileStartPoint
{
  MLPI_SLAVE_ORIGIN_MASTER_ORIGIN    = 0,               //!< Synchronize absolute slave and master position.
  MLPI_SLAVE_CURRENT_MASTER_CURRENT  = 1,               //!< Synchronize absolute to master position and relative to current slave position.
  MLPI_SLAVE_ORIGIN_MASTER_CURRENT   = 2,               //!< Synchronize absolute to slave position and relative to current master position.
  MLPI_SLAVE_CURRENT_MASTER_ORIGIN   = 3,               //!< Synchronize start point relative to current axis positions.
  MLPI_SLAVE_RELATIVE_MASTER_ORIGIN  = 4                //!< Synchronize current point of profile relative to current axis positions to create no slave axis jump.
}MlpiProfileStartPoint;

//! @enum MlpiProfileStepType
//! This enumeration defines the relation between master and slave for each flex profile step.
typedef enum MlpiProfileStepType
{
  MLPI_STEP_FIX             = 0,                        //!< Default step type with fixed relative hub and range. Start and end velocity also fixed.
  MLPI_STEP_FLEX_REL_REL    = 1,                        //!< Flex step type with fixed relative hub and range. Start and end velocity are taken dynamically from preceding and following step.
  MLPI_STEP_FLEX_REL_ABS    = 2,                        //!< Flex step type with fixed relative hub absolute master axis position. Start and end velocity are taken dynamically from preceding and following step.
  MLPI_STEP_FLEX_ABS_REL    = 3,                        //!< Flex step type with fixed relative range and absolute master axis position. Start and end velocity are taken dynamically from preceding and following step.
  MLPI_STEP_FLEX_ABS_ABS    = 4                         //!< Flex step type with absolute master and slave axis positions. Start and end velocity are taken dynamically from preceding and following step.
}MlpiProfileStepType;

//! @enum MlpiProfileExecutionMode
//! This enumeration defines how the flex profile is executed.
typedef enum MlpiProfileExecutionMode
{
  MLPI_EXECUTE_CYCLIC = 0,                              //!< Cyclic execution.
  MLPI_EXECUTE_SINGLE = 1                               //!< Single execution. When the profile is finished, the axis will continue to run with end velocity of the profile.
}MlpiProfileExecutionMode;

//! @enum MlpiProfileMasterType
//! Enumeration for Flex profile master definition.
typedef enum MlpiProfileMasterType
{
  MLPI_MASTER_TIME  = 0,                                //!< Master is time-based.
  MLPI_MASTER_AXIS0 = 1                                 //!< Master is position-based.
}MlpiProfileMasterType;

//! @enum MlpiCamTableId
//! This enumeration defines the available cam numbers. The cams actually available depend on the hardware used.
typedef enum MlpiCamTableId
{
  MLPI_CAM_TABLE_1    =   1,
  MLPI_CAM_TABLE_2    =   2,
  MLPI_CAM_TABLE_3    =   3,
  MLPI_CAM_TABLE_4    =   4,
  MLPI_CAM_TABLE_5    =   5,
  MLPI_CAM_TABLE_6    =   6,
  MLPI_CAM_TABLE_7    =   7,
  MLPI_CAM_TABLE_8    =   8,
  MLPI_CAM_TABLE_9    =   9,
  MLPI_CAM_TABLE_10   =  10,
  MLPI_CAM_TABLE_11   =  11,
  MLPI_CAM_TABLE_12   =  12,
  MLPI_CAM_TABLE_13   =  13,
  MLPI_CAM_TABLE_14   =  14,
  MLPI_CAM_TABLE_15   =  15,
  MLPI_CAM_TABLE_16   =  16,
  MLPI_CAM_TABLE_17   =  17,
  MLPI_CAM_TABLE_18   =  18,
  MLPI_CAM_TABLE_19   =  19,
  MLPI_CAM_TABLE_20   =  20,
  MLPI_CAM_TABLE_21   =  21,
  MLPI_CAM_TABLE_22   =  22,
  MLPI_CAM_TABLE_23   =  23,
  MLPI_CAM_TABLE_24   =  24,
  MLPI_CAM_TABLE_25   =  25,
  MLPI_CAM_TABLE_26   =  26,
  MLPI_CAM_TABLE_27   =  27,
  MLPI_CAM_TABLE_28   =  28,
  MLPI_CAM_TABLE_29   =  29,
  MLPI_CAM_TABLE_30   =  30,
  MLPI_CAM_TABLE_31   =  31,
  MLPI_CAM_TABLE_32   =  32,
  MLPI_CAM_TABLE_33   =  33,
  MLPI_CAM_TABLE_34   =  34,
  MLPI_CAM_TABLE_35   =  35,
  MLPI_CAM_TABLE_36   =  36,
  MLPI_CAM_TABLE_37   =  37,
  MLPI_CAM_TABLE_38   =  38,
  MLPI_CAM_TABLE_39   =  39,
  MLPI_CAM_TABLE_40   =  40,
  MLPI_CAM_TABLE_41   =  41,
  MLPI_CAM_TABLE_42   =  42,
  MLPI_CAM_TABLE_43   =  43,
  MLPI_CAM_TABLE_44   =  44,
  MLPI_CAM_TABLE_45   =  45,
  MLPI_CAM_TABLE_46   =  46,
  MLPI_CAM_TABLE_47   =  47,
  MLPI_CAM_TABLE_48   =  48,
  MLPI_CAM_TABLE_49   =  49,
  MLPI_CAM_TABLE_50   =  50,
  MLPI_CAM_TABLE_51   =  51,
  MLPI_CAM_TABLE_52   =  52,
  MLPI_CAM_TABLE_53   =  53,
  MLPI_CAM_TABLE_54   =  54,
  MLPI_CAM_TABLE_55   =  55,
  MLPI_CAM_TABLE_56   =  56,
  MLPI_CAM_TABLE_57   =  57,
  MLPI_CAM_TABLE_58   =  58,
  MLPI_CAM_TABLE_59   =  59,
  MLPI_CAM_TABLE_60   =  60,
  MLPI_CAM_TABLE_61   =  61,
  MLPI_CAM_TABLE_62   =  62,
  MLPI_CAM_TABLE_63   =  63,
  MLPI_CAM_TABLE_64   =  64,
  MLPI_CAM_TABLE_65   =  65,
  MLPI_CAM_TABLE_66   =  66,
  MLPI_CAM_TABLE_67   =  67,
  MLPI_CAM_TABLE_68   =  68,
  MLPI_CAM_TABLE_69   =  69,
  MLPI_CAM_TABLE_70   =  70,
  MLPI_CAM_TABLE_71   =  71,
  MLPI_CAM_TABLE_72   =  72,
  MLPI_CAM_TABLE_73   =  73,
  MLPI_CAM_TABLE_74   =  74,
  MLPI_CAM_TABLE_75   =  75,
  MLPI_CAM_TABLE_76   =  76,
  MLPI_CAM_TABLE_77   =  77,
  MLPI_CAM_TABLE_78   =  78,
  MLPI_CAM_TABLE_79   =  79,
  MLPI_CAM_TABLE_80   =  80,
  MLPI_CAM_TABLE_81   =  81,
  MLPI_CAM_TABLE_82   =  82,
  MLPI_CAM_TABLE_83   =  83,
  MLPI_CAM_TABLE_84   =  84,
  MLPI_CAM_TABLE_85   =  85,
  MLPI_CAM_TABLE_86   =  86,
  MLPI_CAM_TABLE_87   =  87,
  MLPI_CAM_TABLE_88   =  88,
  MLPI_CAM_TABLE_89   =  89,
  MLPI_CAM_TABLE_90   =  90,
  MLPI_CAM_TABLE_91   =  91,
  MLPI_CAM_TABLE_92   =  92,
  MLPI_CAM_TABLE_93   =  93,
  MLPI_CAM_TABLE_94   =  94,
  MLPI_CAM_TABLE_95   =  95,
  MLPI_CAM_TABLE_96   =  96,
  MLPI_CAM_TABLE_97   =  97,
  MLPI_CAM_TABLE_98   =  98,
  MLPI_CAM_TABLE_99   =  99,
  MLPI_CAM_TABLE_100  = 100
}MlpiCamTableId;

//! @enum MlpiLawType
//! Enumeration for Flex profile motion laws for segments.
typedef enum MlpiLawType
{
  MLPI_REST_IN_REST_INCLINEDSINE              = 0x0000,
  MLPI_REST_IN_REST_POLY5                     = 0x0100,
  MLPI_REST_IN_VELOCITY_POLY5                 = 0x0200,
  MLPI_REST_IN_VELOCITY_POLY7                 = 0x0300,
  MLPI_VELOCITY_IN_REST_POLY5                 = 0x0400,
  MLPI_VELOCITY_IN_REST_POLY7                 = 0x0500,
  MLPI_CONSTANT_VELOCITY                      = 0x0600,
  MLPI_VELOCITY_IN_VELOCITY_POLY5             = 0x0700,
  MLPI_REST_IN_REST_LINEAR                    = 0x0800,
  MLPI_REST_IN_REST_POLY7                     = 0x0900,
  MLPI_REST_IN_REST_SINE                      = 0x0A00,
  MLPI_REST_IN_REST_GUTMANSINE                = 0x0B00,
  MLPI_REST_IN_REST_SINEACC                   = 0x0C00,
  MLPI_REST_IN_REST_SINETORQUE                = 0x0D00,
  MLPI_REST_IN_REST_MOD_TRAPEZE               = 0x0E00,
  MLPI_REST_IN_REST_MOD_SINE                  = 0x0F00,
  MLPI_VELOCITY_IN_VELOCITY_POLY7             = 0x1000,
  MLPI_VELOCITY_IN_VELOCITY_MOD_SINE          = 0x1100,
  MLPI_REST_IN_REST_POLY5_VLIM                = 0x1200,
  MLPI_REST_IN_REST_PARABOLA                  = 0x1300,
  MLPI_REST_IN_REST_POLY8                     = 0x1400,
  MLPI_MOTION_IN_MOTION_POLY5                 = 0x2000,
  MLPI_MOTION_IN_MOTION_POLY7                 = 0x2100,

  // common polynomials
  MLPI_COMMON_POLY5                           = 0x4000,
  MLPI_COMMON_POLY7                           = 0x4100,
  MLPI_COMMON_POLY2                           = 0x4200,
  MLPI_COMMON_POLY3                           = 0x4300,
  MLPI_COMMON_POLY4                           = 0x4400,
  MLPI_COMMON_POLY8                           = 0x4500,

  // extended laws
  MLPI_X_VELOCITY_IN_VELOCITY_TRAPEZE_ALIM    = 0x5000,
  MLPI_X_MOTION_IN_MOTION_POLY5_VLIM          = 0x6000,
  MLPI_X_MOTION_IN_MOTION_POLY5_SLIM          = 0x6100,
  MLPI_X_FIT_VEL_TRAPEZE_ALIM                 = 0x7000,
  MLPI_X_FIT_SINE_TRAPEZE_ALIM                = 0x7100,
  MLPI_X_FIT_ACC_TRAPEZE_ALIM                 = 0x7200,
  MLPI_X_MOTION_IN_MOTION_ACAM                = 0x8000,
  MLPI_X_MOTION_IN_MOTION_VCAM                = 0x8100,
  MLPI_X_MOTION_IN_MOTION_VCAM2_A             = 0x8200,
  MLPI_X_MOTION_IN_MOTION_VCAM2_B             = 0x8300,

  // cam tables
  MLPI_CAMTABLE_1                             =      1,
  MLPI_CAMTABLE_2                             =      2,
  MLPI_CAMTABLE_3                             =      3,
  MLPI_CAMTABLE_4                             =      4,
  MLPI_CAMTABLE_5                             =      5,
  MLPI_CAMTABLE_6                             =      6,
  MLPI_CAMTABLE_7                             =      7,
  MLPI_CAMTABLE_8                             =      8,
  MLPI_CAMTABLE_9                             =      9,
  MLPI_CAMTABLE_10                            =     10,
  MLPI_CAMTABLE_11                            =     11,
  MLPI_CAMTABLE_12                            =     12,
  MLPI_CAMTABLE_13                            =     13,
  MLPI_CAMTABLE_14                            =     14,
  MLPI_CAMTABLE_15                            =     15,
  MLPI_CAMTABLE_16                            =     16,
  MLPI_CAMTABLE_17                            =     17,
  MLPI_CAMTABLE_18                            =     18,
  MLPI_CAMTABLE_19                            =     19,
  MLPI_CAMTABLE_20                            =     20,
  MLPI_CAMTABLE_21                            =     21,
  MLPI_CAMTABLE_22                            =     22,
  MLPI_CAMTABLE_23                            =     23,
  MLPI_CAMTABLE_24                            =     24,
  MLPI_CAMTABLE_25                            =     25,
  MLPI_CAMTABLE_26                            =     26,
  MLPI_CAMTABLE_27                            =     27,
  MLPI_CAMTABLE_28                            =     28,
  MLPI_CAMTABLE_29                            =     29,
  MLPI_CAMTABLE_30                            =     30,
  MLPI_CAMTABLE_31                            =     31,
  MLPI_CAMTABLE_32                            =     32,
  MLPI_CAMTABLE_33                            =     33,
  MLPI_CAMTABLE_34                            =     34,
  MLPI_CAMTABLE_35                            =     35,
  MLPI_CAMTABLE_36                            =     36,
  MLPI_CAMTABLE_37                            =     37,
  MLPI_CAMTABLE_38                            =     38,
  MLPI_CAMTABLE_39                            =     39,
  MLPI_CAMTABLE_40                            =     40,
  MLPI_CAMTABLE_41                            =     41,
  MLPI_CAMTABLE_42                            =     42,
  MLPI_CAMTABLE_43                            =     43,
  MLPI_CAMTABLE_44                            =     44,
  MLPI_CAMTABLE_45                            =     45,
  MLPI_CAMTABLE_46                            =     46,
  MLPI_CAMTABLE_47                            =     47,
  MLPI_CAMTABLE_48                            =     48,
  MLPI_CAMTABLE_49                            =     49,
  MLPI_CAMTABLE_50                            =     50,
  MLPI_CAMTABLE_51                            =     51,
  MLPI_CAMTABLE_52                            =     52,
  MLPI_CAMTABLE_53                            =     53,
  MLPI_CAMTABLE_54                            =     54,
  MLPI_CAMTABLE_55                            =     55,
  MLPI_CAMTABLE_56                            =     56,
  MLPI_CAMTABLE_57                            =     57,
  MLPI_CAMTABLE_58                            =     58,
  MLPI_CAMTABLE_59                            =     59,
  MLPI_CAMTABLE_60                            =     60,
  MLPI_CAMTABLE_61                            =     61,
  MLPI_CAMTABLE_62                            =     62,
  MLPI_CAMTABLE_63                            =     63,
  MLPI_CAMTABLE_64                            =     64,
  MLPI_CAMTABLE_65                            =     65,
  MLPI_CAMTABLE_66                            =     66,
  MLPI_CAMTABLE_67                            =     67,
  MLPI_CAMTABLE_68                            =     68,
  MLPI_CAMTABLE_69                            =     69,
  MLPI_CAMTABLE_70                            =     70,
  MLPI_CAMTABLE_71                            =     71,
  MLPI_CAMTABLE_72                            =     72,
  MLPI_CAMTABLE_73                            =     73,
  MLPI_CAMTABLE_74                            =     74,
  MLPI_CAMTABLE_75                            =     75,
  MLPI_CAMTABLE_76                            =     76,
  MLPI_CAMTABLE_77                            =     77,
  MLPI_CAMTABLE_78                            =     78,
  MLPI_CAMTABLE_79                            =     79,
  MLPI_CAMTABLE_80                            =     80,
  MLPI_CAMTABLE_81                            =     81,
  MLPI_CAMTABLE_82                            =     82,
  MLPI_CAMTABLE_83                            =     83,
  MLPI_CAMTABLE_84                            =     84,
  MLPI_CAMTABLE_85                            =     85,
  MLPI_CAMTABLE_86                            =     86,
  MLPI_CAMTABLE_87                            =     87,
  MLPI_CAMTABLE_88                            =     88,
  MLPI_CAMTABLE_89                            =     89,
  MLPI_CAMTABLE_90                            =     90,
  MLPI_CAMTABLE_91                            =     91,
  MLPI_CAMTABLE_92                            =     92,
  MLPI_CAMTABLE_93                            =     93,
  MLPI_CAMTABLE_94                            =     94,
  MLPI_CAMTABLE_95                            =     95,
  MLPI_CAMTABLE_96                            =     96,
  MLPI_CAMTABLE_97                            =     97,
  MLPI_CAMTABLE_98                            =     98,
  MLPI_CAMTABLE_99                            =     99,
  MLPI_CAMTABLE_100                           =    100
}MlpiLawType;

//! @enum MlpiControl
//! This enumeration defines the control that should be addressed
typedef enum MlpiControl
{
  MLPI_LOCAL_CONTROL = 0,
  MLPI_CONTROL_1  =  1,
  MLPI_CONTROL_2  =  2,
  MLPI_CONTROL_3  =  3,
  MLPI_CONTROL_4  =  4,
  MLPI_CONTROL_5  =  5,
  MLPI_CONTROL_6  =  6,
  MLPI_CONTROL_7  =  7,
  MLPI_CONTROL_8  =  8,
  MLPI_CONTROL_9  =  9,
  MLPI_CONTROL_10 = 10,
  MLPI_CONTROL_11 = 11,
  MLPI_CONTROL_12 = 12,
  MLPI_CONTROL_13 = 13,
  MLPI_CONTROL_14 = 14,
  MLPI_CONTROL_15 = 15,
  MLPI_CONTROL_16 = 16,
  MLPI_CONTROL_17 = 17,
  MLPI_CONTROL_18 = 18,
  MLPI_CONTROL_19 = 19,
  MLPI_CONTROL_20 = 20,
  MLPI_CONTROL_21 = 21,
  MLPI_CONTROL_22 = 22,
  MLPI_CONTROL_23 = 23,
  MLPI_CONTROL_24 = 24,
  MLPI_CONTROL_25 = 25,
  MLPI_CONTROL_26 = 26,
  MLPI_CONTROL_27 = 27,
  MLPI_CONTROL_28 = 28,
  MLPI_CONTROL_29 = 29,
  MLPI_CONTROL_30 = 30,
  MLPI_CONTROL_31 = 31,
  MLPI_CONTROL_32 = 32,
  MLPI_CONTROL_33 = 33,
  MLPI_CONTROL_34 = 34,
  MLPI_CONTROL_35 = 35,
  MLPI_CONTROL_36 = 36,
  MLPI_CONTROL_37 = 37,
  MLPI_CONTROL_38 = 38,
  MLPI_CONTROL_39 = 39,
  MLPI_CONTROL_40 = 40,
  MLPI_CONTROL_41 = 41,
  MLPI_CONTROL_42 = 42,
  MLPI_CONTROL_43 = 43,
  MLPI_CONTROL_44 = 44,
  MLPI_CONTROL_45 = 45,
  MLPI_CONTROL_46 = 46,
  MLPI_CONTROL_47 = 47,
  MLPI_CONTROL_48 = 48,
  MLPI_CONTROL_49 = 49,
  MLPI_CONTROL_50 = 50,
  MLPI_CONTROL_51 = 51,
  MLPI_CONTROL_52 = 52,
  MLPI_CONTROL_53 = 53,
  MLPI_CONTROL_54 = 54,
  MLPI_CONTROL_55 = 55,
  MLPI_CONTROL_56 = 56,
  MLPI_CONTROL_57 = 57,
  MLPI_CONTROL_58 = 58,
  MLPI_CONTROL_59 = 59,
  MLPI_CONTROL_60 = 60,
  MLPI_CONTROL_61 = 61,
  MLPI_CONTROL_62 = 62,
  MLPI_CONTROL_63 = 63,
  MLPI_CONTROL_64 = 64,
  MLPI_CONTROL_65 = 65,
  MLPI_CONTROL_66 = 66,
  MLPI_CONTROL_67 = 67,
  MLPI_CONTROL_68 = 68,
  MLPI_CONTROL_69 = 69,
  MLPI_CONTROL_70 = 70,
  MLPI_CONTROL_71 = 71,
  MLPI_CONTROL_72 = 72,
  MLPI_CONTROL_73 = 73,
  MLPI_CONTROL_74 = 74,
  MLPI_CONTROL_75 = 75,
  MLPI_CONTROL_76 = 76,
  MLPI_CONTROL_77 = 77,
  MLPI_CONTROL_78 = 78,
  MLPI_CONTROL_79 = 79,
  MLPI_CONTROL_80 = 80,
  MLPI_CONTROL_81 = 81,
  MLPI_CONTROL_82 = 82,
  MLPI_CONTROL_83 = 83,
  MLPI_CONTROL_84 = 84,
  MLPI_CONTROL_85 = 85,
  MLPI_CONTROL_86 = 86,
  MLPI_CONTROL_87 = 87,
  MLPI_CONTROL_88 = 88,
  MLPI_CONTROL_89 = 89,
  MLPI_CONTROL_90 = 90,
  MLPI_CONTROL_91 = 91,
  MLPI_CONTROL_92 = 92,
  MLPI_CONTROL_93 = 93,
  MLPI_CONTROL_94 = 94,
  MLPI_CONTROL_95 = 95,
  MLPI_CONTROL_96 = 96,
  MLPI_CONTROL_97 = 97,
  MLPI_CONTROL_98 = 98,
  MLPI_CONTROL_99 = 99
}MlpiControl;

//! @enum MlpiAxisNumber
//! This enumeration defines the available axis numbers
typedef enum MlpiAxisNumber
{
  MLPI_NO_OBJECT =  0,
  MLPI_AXIS_1    =  1,
  MLPI_AXIS_2    =  2,
  MLPI_AXIS_3    =  3,
  MLPI_AXIS_4    =  4,
  MLPI_AXIS_5    =  5,
  MLPI_AXIS_6    =  6,
  MLPI_AXIS_7    =  7,
  MLPI_AXIS_8    =  8,
  MLPI_AXIS_9    =  9,
  MLPI_AXIS_10   = 10,
  MLPI_AXIS_11   = 11,
  MLPI_AXIS_12   = 12,
  MLPI_AXIS_13   = 13,
  MLPI_AXIS_14   = 14,
  MLPI_AXIS_15   = 15,
  MLPI_AXIS_16   = 16,
  MLPI_AXIS_17   = 17,
  MLPI_AXIS_18   = 18,
  MLPI_AXIS_19   = 19,
  MLPI_AXIS_20   = 20,
  MLPI_AXIS_21   = 21,
  MLPI_AXIS_22   = 22,
  MLPI_AXIS_23   = 23,
  MLPI_AXIS_24   = 24,
  MLPI_AXIS_25   = 25,
  MLPI_AXIS_26   = 26,
  MLPI_AXIS_27   = 27,
  MLPI_AXIS_28   = 28,
  MLPI_AXIS_29   = 29,
  MLPI_AXIS_30   = 30,
  MLPI_AXIS_31   = 31,
  MLPI_AXIS_32   = 32,
  MLPI_AXIS_33   = 33,
  MLPI_AXIS_34   = 34,
  MLPI_AXIS_35   = 35,
  MLPI_AXIS_36   = 36,
  MLPI_AXIS_37   = 37,
  MLPI_AXIS_38   = 38,
  MLPI_AXIS_39   = 39,
  MLPI_AXIS_40   = 40,
  MLPI_AXIS_41   = 41,
  MLPI_AXIS_42   = 42,
  MLPI_AXIS_43   = 43,
  MLPI_AXIS_44   = 44,
  MLPI_AXIS_45   = 45,
  MLPI_AXIS_46   = 46,
  MLPI_AXIS_47   = 47,
  MLPI_AXIS_48   = 48,
  MLPI_AXIS_49   = 49,
  MLPI_AXIS_50   = 50,
  MLPI_AXIS_51   = 51,
  MLPI_AXIS_52   = 52,
  MLPI_AXIS_53   = 53,
  MLPI_AXIS_54   = 54,
  MLPI_AXIS_55   = 55,
  MLPI_AXIS_56   = 56,
  MLPI_AXIS_57   = 57,
  MLPI_AXIS_58   = 58,
  MLPI_AXIS_59   = 59,
  MLPI_AXIS_60   = 60,
  MLPI_AXIS_61   = 61,
  MLPI_AXIS_62   = 62,
  MLPI_AXIS_63   = 63,
  MLPI_AXIS_64   = 64,
  MLPI_AXIS_65   = 65,
  MLPI_AXIS_66   = 66,
  MLPI_AXIS_67   = 67,
  MLPI_AXIS_68   = 68,
  MLPI_AXIS_69   = 69,
  MLPI_AXIS_70   = 70,
  MLPI_AXIS_71   = 71,
  MLPI_AXIS_72   = 72,
  MLPI_AXIS_73   = 73,
  MLPI_AXIS_74   = 74,
  MLPI_AXIS_75   = 75,
  MLPI_AXIS_76   = 76,
  MLPI_AXIS_77   = 77,
  MLPI_AXIS_78   = 78,
  MLPI_AXIS_79   = 79,
  MLPI_AXIS_80   = 80,
  MLPI_AXIS_81   = 81,
  MLPI_AXIS_82   = 82,
  MLPI_AXIS_83   = 83,
  MLPI_AXIS_84   = 84,
  MLPI_AXIS_85   = 85,
  MLPI_AXIS_86   = 86,
  MLPI_AXIS_87   = 87,
  MLPI_AXIS_88   = 88,
  MLPI_AXIS_89   = 89,
  MLPI_AXIS_90   = 90,
  MLPI_AXIS_91   = 91,
  MLPI_AXIS_92   = 92,
  MLPI_AXIS_93   = 93,
  MLPI_AXIS_94   = 94,
  MLPI_AXIS_95   = 95,
  MLPI_AXIS_96   = 96,
  MLPI_AXIS_97   = 97,
  MLPI_AXIS_98   = 98,
  MLPI_AXIS_99   = 99
}MlpiAxisNumber;

//! @enum MlpiGroupNumber
//! This enumeration defines the available kinematics group numbers
typedef enum MlpiGroupNumber
{
  MLPI_NO_GROUP =  0,
  MLPI_GROUP_1  =  1,
  MLPI_GROUP_2  =  2,
  MLPI_GROUP_3  =  3,
  MLPI_GROUP_4  =  4,
  MLPI_GROUP_5  =  5,
  MLPI_GROUP_6  =  6,
  MLPI_GROUP_7  =  7,
  MLPI_GROUP_8  =  8,
  MLPI_GROUP_9  =  9,
  MLPI_GROUP_10 = 10,
  MLPI_GROUP_11 = 11,
  MLPI_GROUP_12 = 12,
  MLPI_GROUP_13 = 13,
  MLPI_GROUP_14 = 14,
  MLPI_GROUP_15 = 15,
  MLPI_GROUP_16 = 16
}MlpiGroupNumber;


// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------


// message packing follows 8-byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
  #pragma pack(push,8)
#endif


//! @typedef MlpiAxisRef
//! @brief This structure defines the axis through the definition of control and axis number
//! @details Elements of struct MlpiAxisRef
//! <TABLE>
//! <TR><TH>           Type                </TH><TH>           Element   </TH><TH> Description        </TH></TR>
//! <TR><TD id="st_t"> @ref MlpiControl    </TD><TD id="st_e"> controlNo </TD><TD> Addressed control. </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiAxisNumber </TD><TD id="st_e"> axisNo    </TD><TD> Addressed axis.    </TD></TR>
//! </TABLE>
typedef struct MlpiAxisRef
{
  MlpiAxisRef()
    : controlNo(MLPI_LOCAL_CONTROL)
    , axisNo(MLPI_NO_OBJECT)
  {}
  MlpiAxisRef(MlpiAxisNumber axisNo)
    : controlNo(MLPI_LOCAL_CONTROL)
    , axisNo(axisNo)
  {}
  MlpiAxisRef(ULONG axisNo)
    : controlNo(MLPI_LOCAL_CONTROL)
    , axisNo(static_cast<MlpiAxisNumber>(axisNo))
  {}
  MlpiAxisRef(MlpiControl controlNo, MlpiAxisNumber axisNo)
    : controlNo(controlNo)
    , axisNo(axisNo)
  {}

  MlpiControl       controlNo;                          //!< addressed control.
  MlpiAxisNumber    axisNo;                             //!< addressed axis.
}MlpiAxisRef;

//! @typedef MlpiGroupRef
//! @brief This structure defines the group through the definition of control and group number
//! @details Elements of struct MlpiGroupRef
//! <TABLE>
//! <TR><TH>           Type                 </TH><TH>           Element   </TH><TH> Description        </TH></TR>
//! <TR><TD id="st_t"> @ref MlpiControl     </TD><TD id="st_e"> controlNo </TD><TD> Addressed control. </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiGroupNumber </TD><TD id="st_e"> groupNo   </TD><TD> Addressed group.   </TD></TR>
//! </TABLE>
typedef struct MlpiGroupRef
{
  MlpiGroupRef()
    : controlNo(MLPI_LOCAL_CONTROL)
    , groupNo(MLPI_NO_GROUP)
  {}
  MlpiGroupRef(MlpiGroupNumber groupNo)
    : controlNo(MLPI_LOCAL_CONTROL)
    , groupNo(groupNo)
  {}
  MlpiGroupRef(ULONG groupNo)
    : controlNo(MLPI_LOCAL_CONTROL)
    , groupNo(static_cast<MlpiGroupNumber>(groupNo))
  {}
  MlpiGroupRef(MlpiControl controlNo, MlpiGroupNumber groupNo)
    : controlNo(controlNo)
    , groupNo(groupNo)
  {}

  MlpiControl       controlNo;                          //!< addressed control.
  MlpiGroupNumber   groupNo;                            //!< addressed group.
}MlpiGroupRef;


//! @typedef MlpiMotionStatus
//! @brief This structure defines the status of a motion command
//! @details Elements of struct MlpiMotionStatus
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element         </TH><TH> Description                                                </TH></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> done            </TD><TD> @c TRUE when command is completed.                         </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> active          </TD><TD> @c TRUE as long as command is active.                      </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> aborted         </TD><TD> @c TRUE when command has been aborted by another command.  </TD></TR>
//! <TR><TD id="st_t"> BOOL8  </TD><TD id="st_e"> error           </TD><TD> @c TRUE when motion command issued an error.               </TD></TR>
//! <TR><TD id="st_t"> USHORT </TD><TD id="st_e"> errorID         </TD><TD> Short description of error.                                </TD></TR>
//! <TR><TD id="st_t"> USHORT </TD><TD id="st_e"> table           </TD><TD> Additional description of error.                           </TD></TR>
//! <TR><TD id="st_t"> ULONG  </TD><TD id="st_e"> additional1     </TD><TD> Additional diagnosis number1.                              </TD></TR>
//! <TR><TD id="st_t"> ULONG  </TD><TD id="st_e"> additional2     </TD><TD> Additional diagnosis number2.                              </TD></TR>
//! </TABLE>
typedef struct MlpiMotionStatus
{
  BOOL8             done;                               //! @c TRUE when command is completed.
  BOOL8             active;                             //! @c TRUE as long as command is active.
  BOOL8             aborted;                            //! @c TRUE when command has been aborted by another command.
  BOOL8             error;                              //! @c TRUE when motion command issued an error.
  USHORT            errorID;                            //! Short description of error.
  USHORT            table;                              //! Additional description of error.
  ULONG             additional1;                        //! Additional diagnosis number1.
  ULONG             additional2;                        //! Additional diagnosis number2.
}MlpiMotionStatus;


//! @typedef MlpiAxisInformation
//! @brief Structure containing parametric information about an axis. These values no longer change, once the axis is in operation mode.
//! @details Elements of struct MlpiAxisInformation
//! <TABLE>
//! <TR><TH>           Type         </TH><TH>           Element       </TH><TH> Description                           </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef  </TD><TD id="st_e"> axis          </TD><TD> Logical axis address.                 </TD></TR>
//! <TR><TD id="st_t"> ULONG        </TD><TD id="st_e"> deviceAddress </TD><TD> (SERCOS) device address.              </TD></TR>
//! <TR><TD id="st_t"> MlpiAxisType </TD><TD id="st_e"> axisType      </TD><TD> Type of axis (virtual, real, etc...). </TD></TR>
//! <TR><TD id="st_t"> WCHAR16      </TD><TD id="st_e"> name          </TD><TD> The axis name.                        </TD></TR>
//! </TABLE>
typedef struct MlpiAxisInformation
{
  MlpiAxisRef     axis;                                 //!< Logical axis address.
  ULONG           deviceAddress;                        //!< (SERCOS) device address.
  MlpiAxisType    axisType;                             //!< Type of axis (virtual, real, etc...).
  WCHAR16         name[MLPI_MOTION_MAX_AXIS_NAME_LEN];  //!< The axis name.
}MlpiAxisInformation;

//! @typedef MlpiAxisValues
//! Structure containing operation information about an axis. These values do change as soon as the axis is in operation.
//! You may want to use this structure to read several sets of axis information using one single function call during operation of the axis.
//! This gives increased performance in comparison to reading the values bit by bit. Especially when reading the values for
//! multiple axes.
//! @details Elements of struct MlpiAxisValues
//! <TABLE>
//! <TR><TH>           Type        </TH><TH> Direction </TH><TH>           Element            </TH><TH> Description                                                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD> [in]      </TD><TD id="st_e"> axis               </TD><TD> Logical axis address.                                                         </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD> [out]     </TD><TD id="st_e"> actualPosition     </TD><TD> Actual position of the axis. See @ref mlpiMotionGetActualPosition.            </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD> [out]     </TD><TD id="st_e"> actualVelocity     </TD><TD> Actual velocity of the axis. See @ref mlpiMotionGetActualVelocity.            </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD> [out]     </TD><TD id="st_e"> actualAcceleration </TD><TD> Actual acceleration of the axis. See @ref mlpiMotionGetActualAcceleration.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD> [out]     </TD><TD id="st_e"> actualTorque       </TD><TD> Actual torque of the axis. See @ref mlpiMotionGetActualTorque.                </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> state              </TD><TD> Axis state of the axis. See @ref mlpiMotionGetState.                          </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> stateExtended      </TD><TD> Extended axis state of the axis. See @ref mlpiMotionGetStateExtended.         </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> diagnosisNumber    </TD><TD> Current diagnosis number of the axis. See @ref mlpiMotionGetDiagnosisNumber.  </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> condition          </TD><TD> Condition of the axis. See @ref mlpiMotionGetCondition.                       </TD></TR>
//! </TABLE>
typedef struct MlpiAxisValues
{
  // Input Parameters
  MlpiAxisRef axis;                                     //!< Logical axis address.

  // Output Parameters:
  DOUBLE      actualPosition;                           //!< Actual position of the axis. See @ref mlpiMotionGetActualPosition.
  DOUBLE      actualVelocity;                           //!< Actual velocity of the axis. See @ref mlpiMotionGetActualVelocity.
  DOUBLE      actualAcceleration;                       //!< Actual acceleration of the axis. See @ref mlpiMotionGetActualAcceleration.
  DOUBLE      actualTorque;                             //!< Actual torque of the axis. See @ref mlpiMotionGetActualTorque.
  ULONG       state;                                    //!< Axis state of the axis. See @ref mlpiMotionGetState.
  ULONG       stateExtended;                            //!< Extended axis state of the axis. See @ref mlpiMotionGetStateExtended.
  ULONG       diagnosisNumber;                          //!< Current diagnosis number of the axis. See @ref mlpiMotionGetDiagnosisNumber.
  ULONG       condition;                                //!< Condition of the axis. See @ref mlpiMotionGetCondition.
}MlpiAxisValues;

//! @typedef MlpiAxisUnits
//! @brief Structure containing units of the axis as strings.
//! You may want to use this structure to read all units of an axis information set using one single function call.
//! @details Elements of struct MlpiAxisUnits
//! <TABLE>
//! <TR><TH>           Type        </TH><TH> Direction </TH><TH>           Element      </TH><TH> Description                                     </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD> [in]      </TD><TD id="st_e"> axis         </TD><TD> Logical axis address.                           </TD></TR>
//! <TR><TD id="st_t"> WCHAR16     </TD><TD> [out]     </TD><TD id="st_e"> position     </TD><TD> Position unit of the axis. e.g. 'Degree'.       </TD></TR>
//! <TR><TD id="st_t"> WCHAR16     </TD><TD> [out]     </TD><TD id="st_e"> velocity     </TD><TD> Velocity unit of the axis. e.g. 'Rpm'.          </TD></TR>
//! <TR><TD id="st_t"> WCHAR16     </TD><TD> [out]     </TD><TD id="st_e"> acceleration </TD><TD> Acceleration unit of the axis. e.g. 'rad/sec'.  </TD></TR>
//! <TR><TD id="st_t"> WCHAR16     </TD><TD> [out]     </TD><TD id="st_e"> jerk         </TD><TD> Jerk unit of the axis. e.g. 'rad/sec^2'.        </TD></TR>
//! <TR><TD id="st_t"> WCHAR16     </TD><TD> [out]     </TD><TD id="st_e"> torque       </TD><TD> Torque unit of the axis. e.g. 'Nm'.             </TD></TR>
//! </TABLE>
typedef struct MlpiAxisUnits
{
  // Input Parameters
  MlpiAxisRef axis;                                     //!< Logical axis address.

  // Output Parameters:
  WCHAR16 position[MLPI_MOTION_MAX_UNITS_LEN];          //!< Position unit of the axis. e.g. 'Degree'.
  WCHAR16 velocity[MLPI_MOTION_MAX_UNITS_LEN];          //!< Velocity unit of the axis. e.g. 'Rpm'.
  WCHAR16 acceleration[MLPI_MOTION_MAX_UNITS_LEN];      //!< Acceleration unit of the axis. e.g. 'rad/sec'.
  WCHAR16 jerk[MLPI_MOTION_MAX_UNITS_LEN];              //!< Jerk unit of the axis. e.g. 'rad/sec^2'.
  WCHAR16 torque[MLPI_MOTION_MAX_UNITS_LEN];            //!< Torque unit of the axis. e.g. 'Nm'.
}MlpiAxisUnits;

//! @typedef MlpiAxisStatus
//! Structure containing status information about an axis. These values do change as soon as the axis is in operation.
//! You may want to use this structure to read several sets of axis status information using one single function call during operation of the axis.
//! @details Elements of struct MlpiAxisStatus
//! <TABLE>
//! <TR><TH>           Type        </TH><TH> Direction </TH><TH>           Element            </TH><TH> Description                                                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD> [in]      </TD><TD id="st_e"> axis               </TD><TD> Logical axis address.                                                         </TD></TR>
//! <TR><TD id="st_t"> ULLONG      </TD><TD> [out]     </TD><TD id="st_e"> state              </TD><TD> Axis state of the axis. See @ref mlpiMotionGetState.                          </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> stateExtended      </TD><TD> Extended axis state of the axis. See @ref mlpiMotionGetStateExtended.         </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> diagnosisNumber    </TD><TD> Current diagnosis number of the axis. See @ref mlpiMotionGetDiagnosisNumber.  </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD> [out]     </TD><TD id="st_e"> condition          </TD><TD> Condition of the axis. See @ref mlpiMotionGetCondition.                       </TD></TR>
//! </TABLE>
typedef struct MlpiAxisStatus
{
  // Input Parameters
  MlpiAxisRef       MLPI_STRUCT_ALIGN_STRUCT      axis;                                     //!< Logical axis address.

  // Output Parameters:
  ULLONG            MLPI_STRUCT_ALIGN_ULLONG      state;                                    //!< Axis state of the axis. See @ref mlpiMotionGetState.
  ULONG             MLPI_STRUCT_ALIGN_ULONG       stateExtended;                            //!< Extended axis state of the axis. See @ref mlpiMotionGetStateExtended.
  ULONG             MLPI_STRUCT_ALIGN_ULONG       diagnosisNumber;                          //!< Current diagnosis number of the axis. See @ref mlpiMotionGetDiagnosisNumber.
  ULONG             MLPI_STRUCT_ALIGN_ULONG       condition;                                //!< Condition of the axis. See @ref mlpiMotionGetCondition.
}MlpiAxisStatus;

//! @typedef MlpiMotionStop
//! @brief Structure to command stop to an axis.
//! @details Elements of struct MlpiMotionStop
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                                                                           </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the axis.                                                                </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit.                                                          </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.                                                                  </TD></TR>
//! <TR><TD id="st_t"> BOOL8       </TD><TD id="st_e"> stop         </TD><TD> 1: stopping, 0: exit Stopping mode. Important Note: This command is special in
//!                                                                              the way that it has to be called once with 'stop' = TRUE. This will bring the axis
//!                                                                              to a standstill. Once the axis is standing still, the command has to be called again
//!                                                                              with 'stop' = FALSE in order to enter the PLCopen 'Stand Still' mode.              </TD></TR>
//! </TABLE>
typedef struct MlpiMotionStop
{
  MlpiAxisRef axis;                                     //!< Reference to the axis.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
  BOOL8       stop;                                     //!< 1: stopping, 0: exit Stopping mode. Important Note: This command is special in
                                                        //!< the way that it has to be called one time with 'stop' = TRUE. This will bring the axis
                                                        //!< to a standstill. And when the axis is standing still, the command has to be called again
                                                        //!< with 'stop' = FALSE in order to enter the PLCopen 'Stand Still' mode.
}MlpiMotionStop;

//! @typedef MlpiMotionPower
//! @brief Structure to command power to an axis.
//! @details Elements of struct MlpiMotionPower
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element </TH><TH> Description                </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis    </TD><TD> Reference to the axis.     </TD></TR>
//! <TR><TD id="st_t"> BOOL8       </TD><TD id="st_e"> power   </TD><TD> 1: Power on, 0: Power off. </TD></TR>
//! </TABLE>
typedef struct MlpiMotionPower
{
  MlpiAxisRef axis;                                     //!< Reference to the axis.
  BOOL8       power;                                    //!< 1: Power on, 0: Power off.
}MlpiMotionPower;

//! @typedef MlpiMotionGearIn
//! @brief This structure defines a velocity synchronization of the slave to the master using
//! a gear and fine adjust.
//! @details Elements of struct MlpiMotionGearIn
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element     </TH><TH> Description                   </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis        </TD><TD> Reference to the slave axis.  </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD id="st_e"> numerator   </TD><TD> Gear ratio numerator.         </TD></TR>
//! <TR><TD id="st_t"> ULONG       </TD><TD id="st_e"> denominator </TD><TD> Gear ratio denominator.       </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> fineadjust  </TD><TD> Fine adjust for gear in %.    </TD></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> master      </TD><TD> Reference to the master axis. </TD></TR>
//! </TABLE>
typedef struct MlpiMotionGearIn
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  ULONG       numerator;                                //!< Gear ratio numerator.
  ULONG       denominator;                              //!< Gear ratio denominator.
  DOUBLE      fineadjust;                               //!< Fine adjust for gear in %.
  MlpiAxisRef master;                                   //!< Reference to the master axis.
}MlpiMotionGearIn;

//! @typedef MlpiMotionGearInPos
//! @brief This structure defines a position synchronization of the slave to the master using
//! a gear and fine adjust.
//! @details Elements of struct MlpiMotionGearInPos
//! <TABLE>
//! <TR><TH>           Type                   </TH><TH>           Element     </TH><TH> Description                   </TH></TR>
//! <TR><TD id="st_t">      MlpiAxisRef       </TD><TD id="st_e"> axis        </TD><TD> Reference to the slave axis.  </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> numerator   </TD><TD> Gear ratio numerator.         </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> denominator </TD><TD> Gear ratio denominator.       </TD></TR>
//! <TR><TD id="st_t">      DOUBLE            </TD><TD id="st_e"> fineadjust  </TD><TD> Fine adjust for gear in %.    </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiStartMode     </TD><TD id="st_e"> startMode   </TD><TD> Synchronization mode.         </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiSyncDirection </TD><TD id="st_e"> syncMode    </TD><TD> Synchronization direction.    </TD></TR>
//! <TR><TD id="st_t">      MlpiAxisRef       </TD><TD id="st_e"> master      </TD><TD> Reference to the master axis. </TD></TR>
//! </TABLE>
typedef struct MlpiMotionGearInPos
{
  MlpiAxisRef         axis;                             //!< Reference to the slave axis.
  ULONG               numerator;                        //!< Gear ratio numerator.
  ULONG               denominator;                      //!< Gear ratio denominator.
  DOUBLE              fineadjust;                       //!< Fine adjust for gear in %.
  MlpiStartMode       startMode;                        //!< Synchronization mode.
  MlpiSyncDirection   syncMode;                         //!< Synchronization direction.
  MlpiAxisRef         master;                           //!< Reference to the master axis.
}MlpiMotionGearInPos;

//! @typedef MlpiMotionCamIn
//! @brief This structure defines a position synchronization of the slave to the master using
//! gear, fine adjust and a cam table.
//! @details Elements of struct MlpiMotionCamIn
//! <TABLE>
//! <TR><TH>           Type                   </TH><TH>           Element           </TH><TH> Description                        </TH></TR>
//! <TR><TD id="st_t">      MlpiAxisRef       </TD><TD id="st_e"> axis              </TD><TD> Reference to the slave axis.       </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> numerator         </TD><TD> Gear ratio numerator.              </TD></TR>
//! <TR><TD id="st_t">      ULONG             </TD><TD id="st_e"> denominator       </TD><TD> Gear ratio denominator.            </TD></TR>
//! <TR><TD id="st_t">      DOUBLE            </TD><TD id="st_e"> fineadjust        </TD><TD> Fine adjust for gear in %.         </TD></TR>
//! <TR><TD id="st_t">      DOUBLE            </TD><TD id="st_e"> camShaftDistance  </TD><TD> Cam shaft distance in slave units. </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiStartMode     </TD><TD id="st_e"> startMode         </TD><TD> Synchronization mode.              </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiSyncDirection </TD><TD id="st_e"> syncMode          </TD><TD> Synchronization direction.         </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiCamTableId    </TD><TD id="st_e"> camTable          </TD><TD> ID of the Cam table to be used.    </TD></TR>
//! <TR><TD id="st_t">      MlpiAxisRef       </TD><TD id="st_e"> master            </TD><TD> Reference to the master axis.      </TD></TR>
//! </TABLE>
typedef struct MlpiMotionCamIn
{
  MlpiAxisRef         axis;                             //!< Reference to the slave axis.
  ULONG               numerator;                        //!< Gear ratio numerator.
  ULONG               denominator;                      //!< Gear ratio denominator.
  DOUBLE              fineadjust;                       //!< Fine adjust for gear in %.
  DOUBLE              camShaftDistance;                 //!< Cam shaft distance in slave units.
  MlpiStartMode       startMode;                        //!< Synchronization mode.
  MlpiSyncDirection   syncMode;                         //!< Synchronization direction.
  MlpiCamTableId      camTable;                         //!< ID of the Cam table to be used.
  MlpiAxisRef         master;                           //!< Reference to the master axis.
}MlpiMotionCamIn;

//! @typedef MlpiMotionMotionProfile
//! @brief This structure defines a position synchronization of the slave to the master using gear, fine adjust and a motion profile.
//! This command can only be used on IndraDrives with interpolation in the drive.
//! @details Elements of struct MlpiMotionMotionProfile
//! <TABLE>
//! <TR><TH>           Type                         </TH><TH>           Element             </TH><TH> Description                                                            </TH></TR>
//! <TR><TD id="st_t">      MlpiAxisRef             </TD><TD id="st_e"> axis                </TD><TD> Reference to the slave axis.                                           </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> numerator           </TD><TD> Gear ratio numerator.                                                  </TD></TR>
//! <TR><TD id="st_t">      ULONG                   </TD><TD id="st_e"> denominator         </TD><TD> Gear ratio denominator.                                                </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                  </TD><TD id="st_e"> fineadjust          </TD><TD> Fine adjust for gear in %.                                             </TD></TR>
//! <TR><TD id="st_t">      BOOL8                   </TD><TD id="st_e"> relativePositioning </TD><TD> Determines if offset angle of the parameter A-0-2901 should be active. </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiProfileSetSelection </TD><TD id="st_e"> setSelection        </TD><TD> Selects the active set.                                                </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiStartMode           </TD><TD id="st_e"> startMode           </TD><TD> Synchronization mode.                                                  </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiSyncDirection       </TD><TD id="st_e"> syncMode            </TD><TD> Synchronization direction.                                             </TD></TR>
//! <TR><TD id="st_t">      MlpiAxisRef             </TD><TD id="st_e"> master              </TD><TD> Reference to the master axis.                                          </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMotionProfile
{
  MlpiAxisRef                 axis;                 //!< Reference to the slave axis.
  ULONG                       numerator;            //!< Gear ratio numerator.
  ULONG                       denominator;          //!< Gear ratio denominator.
  DOUBLE                      fineadjust;           //!< Fine adjust for gear in %.
  BOOL8                       relativePositioning;  //!< Determines if offset angle of the parameter A-0-2901 should be active.
  MlpiProfileSetSelection     setSelection;         //!< Selects the active set.
  MlpiStartMode               startMode;            //!< Synchronization mode.
  MlpiSyncDirection           syncMode;             //!< Synchronization direction.
  MlpiAxisRef                 master;               //!< Reference to the master axis.
}MlpiMotionMotionProfile;

//! @typedef MlpiMotionFlexProfileStep
//! @brief This struct defines a single step of a FlexProfile. Use multiple steps to create a profile.
//! @details Elements of struct MlpiMotionFlexProfileStep
//! <TABLE>
//! <TR><TH>           Type                       </TH><TH>           Element                  </TH><TH> Description                                                                </TH></TR>
//! <TR><TD id="st_t"> @ref MlpiProfileMasterType </TD><TD id="st_e"> master                   </TD><TD> Definition of master type.                                                 </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> camShaftDistance         </TD><TD> Cam shaft distance (Hub).                                                  </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> range                    </TD><TD> Definition of master range.                                                </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiLawType           </TD><TD id="st_e"> motionLaw                </TD><TD> Definition of motion law.                                                  </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiProfileStepType   </TD><TD id="st_e"> stepType                 </TD><TD> Segment type: relation between master and slave.                           </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> startVel                 </TD><TD> Slave axis starting velocity.                                              </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> startAcc                 </TD><TD> Slave axis starting acceleration.                                          </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> startJrk                 </TD><TD> Slave axis starting jerk.                                                  </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> endVel                   </TD><TD> Slave axis ending velocity.                                                </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> endAcc                   </TD><TD> Slave axis ending acceleration.                                            </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> endJrk                   </TD><TD> Slave axis ending jerk.                                                    </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> travelVel                </TD><TD> Slave axis velocity during move (not implemented for all motion laws).     </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> limitAcc                 </TD><TD> Slave axis acceleration during move (not implemented for all motion laws). </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> limitJrk                 </TD><TD> Slave axis jerk during move (not implemented for all motion laws).         </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                </TD><TD id="st_e"> turningPointDisplacement </TD><TD> Displacement of point of inflection.                                       </TD></TR>
//! </TABLE>
typedef struct MlpiMotionFlexProfileStep
{
  MlpiProfileMasterType  MLPI_STRUCT_ALIGN_ENUM     master;                     //!< Definition of master type.
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   camShaftDistance;           //!< Cam shaft distance (Hub).
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   range;                      //!< Definition of master range.

  MlpiLawType            MLPI_STRUCT_ALIGN_ENUM     motionLaw;                  //!< Definition of motion law.
  MlpiProfileStepType    MLPI_STRUCT_ALIGN_ENUM     stepType;                   //!< Segment type: relation between master and slave.

  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   startVel;                   //!< Slave axis starting velocity.
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   startAcc;                   //!< Slave axis starting acceleration.
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   startJrk;                   //!< Slave axis starting jerk.

  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   endVel;                     //!< Slave axis ending velocity.
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   endAcc;                     //!< Slave axis ending acceleration.
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   endJrk;                     //!< Slave axis ending jerk.

  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   travelVel;                  //!< Slave axis velocity during move (not implemented for all motion laws).
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   limitAcc;                   //!< Slave axis acceleration during move (not implemented for all motion laws).
  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   limitJrk;                   //!< Slave axis jerk during move (not implemented for all motion laws).

  DOUBLE                 MLPI_STRUCT_ALIGN_DOUBLE   turningPointDisplacement;   //!< Displacement of point of inflection.
} MlpiMotionFlexProfileStep;

//! @typedef MlpiMotionFlexProfile
//! @brief This structure defines a position synchronization of the slave to the master using
//! gear, fine adjust and a flex profile. This command can only be used on IndraDrives
//! with interpolation in the control.
//! @details Elements of struct MlpiMotionFlexProfile
//! <TABLE>
//! <TR><TH>           Type                          </TH><TH>           Element                 </TH><TH> Description                                         </TH></TR>
//! <TR><TD id="st_t">      MlpiAxisRef              </TD><TD id="st_e"> axis                    </TD><TD> Reference to the slave axis.                        </TD></TR>
//! <TR><TD id="st_t">      ULONG                    </TD><TD id="st_e"> numerator               </TD><TD> Gear ratio numerator.                               </TD></TR>
//! <TR><TD id="st_t">      ULONG                    </TD><TD id="st_e"> denominator             </TD><TD> Gear ratio denominator.                             </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                   </TD><TD id="st_e"> fineadjust              </TD><TD> Fine adjust for gear in %.                          </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiProfileSetSelection  </TD><TD id="st_e"> setSelection            </TD><TD> Selects the active set                              </TD></TR>
//! <TR><TD id="st_t">      BOOL8                    </TD><TD id="st_e"> useSwitchingPositioning </TD><TD> Determines whether switching position will be used. </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                   </TD><TD id="st_e"> switchingPosition       </TD><TD> Switching position.                                 </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiProfileStartPoint    </TD><TD id="st_e"> profileEntry            </TD><TD> Switching conditions.                               </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                   </TD><TD id="st_e"> masterOffset            </TD><TD> Master offset.                                      </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                   </TD><TD id="st_e"> slaveOffset             </TD><TD> Slave offset.                                       </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiSyncType             </TD><TD id="st_e"> syncType                </TD><TD> Synchronization type.                               </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                   </TD><TD id="st_e"> syncVelocity            </TD><TD> Synchronization velocity.                           </TD></TR>
//! <TR><TD id="st_t">      DOUBLE                   </TD><TD id="st_e"> syncAcceleration        </TD><TD> Synchronization acceleration.                       </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiProfileExecutionMode </TD><TD id="st_e"> executionMode           </TD><TD> Execution mode.                                     </TD></TR>
//! <TR><TD id="st_t">      MlpiAxisRef              </TD><TD id="st_e"> master                  </TD><TD> Reference to the master axis.                       </TD></TR>
//! </TABLE>
typedef struct MlpiMotionFlexProfile
{
  MlpiAxisRef               MLPI_STRUCT_ALIGN_STRUCT    axis;                       //!< Reference to the slave axis.
  ULONG                     MLPI_STRUCT_ALIGN_ULONG     numerator;                  //!< Gear ratio numerator.
  ULONG                     MLPI_STRUCT_ALIGN_ULONG     denominator;                //!< Gear ratio denominator.
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE    fineadjust;                 //!< Fine adjust for gear in %.
  MlpiProfileSetSelection   MLPI_STRUCT_ALIGN_ENUM      setSelection;               //!< Selects the active set
  BOOL8                     MLPI_STRUCT_ALIGN_BOOL8     useSwitchingPositioning;    //!< Determines whether switching position will be used.
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE    switchingPosition;          //!< Switching position.
  MlpiProfileStartPoint     MLPI_STRUCT_ALIGN_DOUBLE    profileEntry;               //!< Switching conditions.
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE    masterOffset;               //!< Master offset.
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE    slaveOffset;                //!< Slave offset.
  MlpiSyncType              MLPI_STRUCT_ALIGN_ENUM      syncType;                   //!< Synchronization type.
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE    syncVelocity;               //!< Synchronization velocity.
  DOUBLE                    MLPI_STRUCT_ALIGN_DOUBLE    syncAcceleration;           //!< Synchronization acceleration.
  MlpiProfileExecutionMode  MLPI_STRUCT_ALIGN_ENUM      executionMode;              //!< Execution mode.
  MlpiAxisRef               MLPI_STRUCT_ALIGN_STRUCT    master;                     //!< Reference to the master axis.
}MlpiMotionFlexProfile;

//! @typedef MlpiMotionMoveVelocity
//! @brief This structure defines a velocity move of a single axis.
//! @details Elements of struct MlpiMotionMoveVelocity
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                  </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.     </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.         </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMoveVelocity
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
}MlpiMotionMoveVelocity;

//! @typedef MlpiMotionPhasing
//! @brief This structure defines a phase shift of a slave axis that can either be applied to
//! the master position of that slave or to the gear/cam/flex profile/motion profile.
//! @details Elements of struct MlpiMotionPhasing
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis.                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> phaseShift   </TD><TD> Phase shift in drives unit (master or slave). </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.                      </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit.                  </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit.                  </TD></TR>
//! </TABLE>
typedef struct MlpiMotionPhasing
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      phaseShift;                               //!< Phase shift in drives unit (master or slave).
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
}MlpiMotionPhasing;

//! @typedef MlpiMotionMoveAbsolute
//! @brief This structure defines an absolute position move of a single axis.
//! @details Elements of struct MlpiMotionMoveAbsolute
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                     </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> position     </TD><TD> Target position in drives unit. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.        </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.            </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMoveAbsolute
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      position;                                 //!< Target position in drives unit.
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
}MlpiMotionMoveAbsolute;

//! @typedef MlpiMotionMoveContinuousAbsolute
//! @brief This structure defines a continuous absolute move of a single axis.
//! @details Elements of struct MlpiMotionMoveContinuousAbsolute
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                     </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> position     </TD><TD> Target position in drives unit. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> endVelocity  </TD><TD> End Velocity in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.        </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.            </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMoveContinuousAbsolute
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      position;                                 //!< Target position in drives unit.
  DOUBLE      endVelocity;                              //!< End Velocity in drives unit.
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
}MlpiMotionMoveContinuousAbsolute;

//! @typedef MlpiMotionMoveContinuousRelative
//! @brief This structure defines a continuous relative move of a single axis.
//! @details Elements of struct MlpiMotionMoveContinuousRelative
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                     </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> distance     </TD><TD> Distance in drives unit.        </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> endVelocity  </TD><TD> End Velocity in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.        </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit.    </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.            </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMoveContinuousRelative
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      distance;                                 //!< Target distance in drives unit.
  DOUBLE      endVelocity;                              //!< End Velocity in drives unit.
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
}MlpiMotionMoveContinuousRelative;

//! @typedef MlpiMotionMoveRelative
//! @brief This structure defines a relative position move of a single axis.
//! @details Elements of struct MlpiMotionMoveRelative
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                  </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> distance     </TD><TD> Distance in drives unit.     </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.     </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit. </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.         </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMoveRelative
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      distance;                                 //!< Distance in drives unit.
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
}MlpiMotionMoveRelative;

//! @typedef MlpiMotionMoveAdditive
//! @brief This structure defines an additive position move of a single axis.
//! @details Elements of struct MlpiMotionMoveAdditive
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element      </TH><TH> Description                                                </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis         </TD><TD> Reference to the slave axis.                               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> distance     </TD><TD> Distance in drives unit (is added to the target position). </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> velocity     </TD><TD> Velocity in drives unit.                                   </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> acceleration </TD><TD> Acceleration in drives unit.                               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> deceleration </TD><TD> Deceleration in drives unit.                               </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> jerk         </TD><TD> Jerk in drives unit.                                       </TD></TR>
//! </TABLE>
typedef struct MlpiMotionMoveAdditive
{
  MlpiAxisRef axis;                                     //!< Reference to the slave axis.
  DOUBLE      distance;                                 //!< Distance in drives unit (is added to the target position).
  DOUBLE      velocity;                                 //!< Velocity in drives unit.
  DOUBLE      acceleration;                             //!< Acceleration in drives unit.
  DOUBLE      deceleration;                             //!< Deceleration in drives unit.
  DOUBLE      jerk;                                     //!< Jerk in drives unit.
}MlpiMotionMoveAdditive;

//! @typedef MlpiMotionAdminAxisGroup
//! @brief This structure defines information for adding and removing an axis to or from a group.
//! @details Elements of struct MlpiMotionAdminAxisGroup
//! <TABLE>
//! <TR><TH>           Type         </TH><TH>           Element   </TH><TH> Description                 </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef  </TD><TD id="st_e"> axis      </TD><TD> Axis to add to the group.   </TD></TR>
//! <TR><TD id="st_t"> MlpiGroupRef </TD><TD id="st_e"> group     </TD><TD> Group to add the axis.      </TD></TR>
//! </TABLE>
typedef struct MlpiMotionAdminAxisGroup
{
  MlpiAxisRef             axis;                         //!< axis to add to the group
  MlpiGroupRef            group;                        //!< group to add the axis
} MlpiMotionAdminAxisGroup;

//! @typedef MlpiMotionTorqueControl
//! @brief Structure to command torque to an axis.
//! @details Elements of struct MlpiMotionTorqueControl
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element    </TH><TH> Description                 </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis       </TD><TD> Reference to the axis.      </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> torque     </TD><TD> Torque in drives unit.      </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> torqueRamp </TD><TD> TorqueRamp in drives unit.  </TD></TR>
//! </TABLE>
typedef struct MlpiMotionTorqueControl
{
  MlpiAxisRef axis;                                     //!< Reference to the axis.
  DOUBLE      torque;                                   //!< Torque in drives unit.
  DOUBLE      torqueRamp;                               //!< TorqueRamp in drives unit.
}MlpiMotionTorqueControl;

//! @typedef MlpiMotionCyclic
//! @brief This structure is used when operating the axis in cyclic position command mode.
//! A cyclic value (position or velocity) must be updated every sercos cycle.
//! @details Elements of struct MlpiMotionCyclic
//! <TABLE>
//! <TR><TH>           Type        </TH><TH>           Element     </TH><TH> Description                              </TH></TR>
//! <TR><TD id="st_t"> MlpiAxisRef </TD><TD id="st_e"> axis        </TD><TD> Reference to the axis.                   </TD></TR>
//! <TR><TD id="st_t"> DOUBLE      </TD><TD id="st_e"> cyclicValue </TD><TD> Cyclic value (position, velocity, ...).  </TD></TR>
//! </TABLE>
typedef struct MlpiMotionCyclic
{
  MlpiAxisRef axis;                                     //!< Reference to the axis.
  DOUBLE      cyclicValue;                              //!< Cyclic value (position, velocity, ...).
}MlpiMotionCyclic;

#if !defined(TARGET_OS_VXWORKS)
  #pragma pack(pop)
#endif

//! @} // endof: @ingroup MotionLibStructTypes


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


//! @ingroup MotionLibMovement
//! This function reads the status of a motion command.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[in]    motionHandle        Handle of the motion command where the status should be requested.
//! @param[out]   status              Pointer to data where status will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionGetStatus(const MLPIHANDLE connection, const MlpiAxisRef axis, const MLPIMOTIONHANDLE motionHandle, MlpiMotionStatus* status);


//! @ingroup MotionLibMovement
//! This function commands power to the drive. This is only necessary for a real axis. Virtual axis can be moved without
//! giving power to the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref MotionLibMovement
MLPI_API MLPIRESULT mlpiMotionPower(const MLPIHANDLE connection, const MlpiMotionPower* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands a stop of motion to the drive.
//! @note After calling this function with @c TRUE you have to call it again with @c FALSE to get axis from state 'Stopping'
//! to state 'Standstill' command and to make the axis accept new commands.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Reference to the axis that should be stopped.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref MotionLibMovement
MLPI_API MLPIRESULT mlpiMotionStop(const MLPIHANDLE connection, const MlpiMotionStop* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands to home an axis. Only axes with incremental encoders can be homed.
//! If the axis has an absolute encoder, the command "set absolute measurement" must be used.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//! MLPIRESULT result = mlpiMotionHome(connection, 1, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // wait till homing is finished
//! result = utilMotionWait(connection, axis, motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionHome(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands to "set absolute measurement" of an axis. Only axis with absolute
//! encoders can be used with this function.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref MotionLibMovement
MLPI_API MLPIRESULT mlpiMotionSetAbsoluteMeasurement(const MLPIHANDLE connection, const MlpiAxisRef axis);


//! @ingroup MotionLibMovement
//! This function writes all parameters necessary to configure a flex profile.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[in]    paramSet            Pointer to array of structures containing the parameters for the flex profile.
//! @param[in]    numElements         Number of structures passed.
//! @param[in]    masterVel           Master velocity for given flex profiles.
//! @param[in]    set                 Set where flex profile should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiMotionFlexProfileStep profile[2];
//! memset(profile, 0, sizeof(profile));
//!
//! // first step in profile is rest in rest poly5
//! profile[0].camShaftDistance          = 180.0;  // on 180 degrees of master axis travel..
//! profile[0].range                     = 180.0;  // ...move the slave axis by 180 degrees
//! profile[0].motionLaw                 = MLPI_REST_IN_REST_POLY5;
//! profile[0].master                    = MLPI_MASTER_AXIS0;
//! profile[0].startVel                  = 0.0;
//! profile[0].startAcc                  = 0.0;
//! profile[0].endVel                    = 0.0;
//! profile[0].endAcc                    = 0.0;
//! profile[0].turningPointDisplacement  = 0.5;
//!
//! // second step in profile is a rest in rest sinusoidal
//! profile[1].camShaftDistance          = 180.0;   // on 180 degrees of master axis travel..
//! profile[1].range                     = -180.0;  // ...reverse the slave axis by 180 degrees
//! profile[1].motionLaw                 = MLPI_REST_IN_REST_MOD_SINE;
//! profile[1].master                    = MLPI_MASTER_AXIS0;
//! profile[1].startVel                  = 0.0;
//! profile[1].startAcc                  = 0.0;
//! profile[1].endVel                    = 0.0;
//! profile[1].endAcc                    = 0.0;
//! profile[1].turningPointDisplacement  = 0.5;
//!
//! // write the profile to slave axis 1 onto set 0.
//! MLPIRESULT result = mlpiMotionChangeFlexProfileSet(connection, 1, profile, 2, 10, MLPI_PROFILE_SET_0);
//! if (MLPI_FAILED(result)){
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // next step would be to start the profile on the slave axis using mlpiMotionFlexProfile()...
//! @endcode
MLPI_API MLPIRESULT mlpiMotionChangeFlexProfileSet(const MLPIHANDLE connection, const MlpiAxisRef axis, const MlpiMotionFlexProfileStep* paramSet, const ULONG numElements, const DOUBLE masterVel, const MlpiProfileSetSelection set);


//! @ingroup MotionLibMovement
//! This function commands a velocity movement. Once the commanded velocity had been reached,
//! the axis will continue to run with this velocity until another command is issued.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref MotionLibMovement
MLPI_API MLPIRESULT mlpiMotionMoveVelocity(const MLPIHANDLE connection, const MlpiMotionMoveVelocity* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands a movement to a position where a commanded velocity is reached. Once this position is reached,
//! the axis will continue to run with the commanded velocity until another command is issued.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // command continuous absolute movement with end velocity 100 at position 50
//! MlpiMotionMoveContinuousAbsolute cmdMoveConAbs;
//! cmdMoveConAbs.axis         = 1;
//! cmdMoveConAbs.position     = 50;
//! cmdMoveConAbs.endVelocity  = 100;
//! cmdMoveConAbs.velocity     = 100;
//! cmdMoveConAbs.acceleration = 10;
//! cmdMoveConAbs.deceleration = 10;
//! cmdMoveConAbs.jerk         = 0;
//!
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionMoveContinuousAbsolute(connection, &cmdMoveConAbs, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionMoveContinuousAbsolute(const MLPIHANDLE connection, const MlpiMotionMoveContinuousAbsolute* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands a movement of a specified relative distance ending with the specified velocity.
//! Once this distance is reached, the axis will continue to run with the commanded velocity until another command is issued.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // command continuous relative movement with end velocity 100 and a distance of 200
//! MlpiMotionMoveContinuousRelative cmdMoveConRel;
//! cmdMoveConRel.axis         = 1;
//! cmdMoveConRel.distance     = 200;
//! cmdMoveConRel.endVelocity  = 100;
//! cmdMoveConRel.velocity     = 100;
//! cmdMoveConRel.acceleration = 10;
//! cmdMoveConRel.deceleration = 10;
//! cmdMoveConRel.jerk         = 0;
//!
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionMoveContinuousRelative(connection, &cmdMoveConRel, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionMoveContinuousRelative(const MLPIHANDLE connection, const MlpiMotionMoveContinuousRelative* paramSet, MLPIMOTIONHANDLE* motionHandle);

//! @ingroup MotionLibMovement
//! This function commands an absolute movement. This means that the axis will move to a given target
//! position.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref MotionLibMovement
MLPI_API MLPIRESULT mlpiMotionMoveAbsolute(const MLPIHANDLE connection, const MlpiMotionMoveAbsolute* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an additive movement. The additive position is added to the target
//! position and is used as new target. It can be used to add a relative position to the
//! already commanded movement.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // command a move relative by 90 degrees/millimeter on axis 1
//! MlpiMotionMoveAdditive cmdMoveAdditive;
//! cmdMoveAdditive.axis         = 1;
//! cmdMoveAdditive.distance     = 90;
//! cmdMoveAdditive.velocity     = 100;
//! cmdMoveAdditive.acceleration = 10;
//! cmdMoveAdditive.deceleration = 10;
//! cmdMoveAdditive.jerk         = 0;
//!
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionMoveAdditive(connection, &cmdMoveAdditive, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionMoveAdditive(const MLPIHANDLE connection, const MlpiMotionMoveAdditive* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands a relative movement. The relative position is added to the current
//! position and is used as new target. The difference between this and an additive movement is that
//! the position offset is added to the current position, not the target position. So
//! a movement command which is currently running will be interrupted and only the position offset is
//! added to the current position.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // command a move relative by 90 degrees/millimeter on axis 1
//! MlpiMotionMoveRelative cmdMoveRelative;
//! cmdMoveRelative.axis         = 1;
//! cmdMoveRelative.distance     = 90;
//! cmdMoveRelative.velocity     = 100;
//! cmdMoveRelative.acceleration = 10;
//! cmdMoveRelative.deceleration = 10;
//! cmdMoveRelative.jerk         = 0;
//!
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionMoveRelative(connection, &cmdMoveRelative, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionMoveRelative(const MLPIHANDLE connection, const MlpiMotionMoveRelative* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands a move torque.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionTorqueControl(const MLPIHANDLE connection, const MlpiMotionTorqueControl* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! The following functions all deal with motion that is commanded by sending cyclic data.
//! This is used to be able to generate your own profiles. This is contrary to all other
//! motion commands where the profiles are generated by the control. In order to be able to
//! write cyclic data, a "channel" must be opened first. Once opened, cyclic data should be written
//! every motion cycle. No explicit error will occur if no cyclic data is written. In this case, the last known
//! cyclic data will be send again. But keep in mind, that the movement on the drive might be jittered or even result
//! in errors in the drive.
//! To correctly synchronize your cyclic write access to the drive you might want to have a look at the
//! function @c mlpiTaskWaitForEvent.
//! The cyclic command mode can be canceled using any other single or synchronous motion command.
//! Prior to opening a cyclic channel, you should initialize the first channel value by using the corresponding
//! write command (e.g. mlpiMotionWriteCyclicPosition).

//! @ingroup MotionLibMovement
//! This function opens a cyclic channel for cyclic position commands.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionOpenCyclicPositionChannel(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function opens a cyclic channel for cyclic velocity commands.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionOpenCyclicVelocityChannel(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function opens a cyclic channel for cyclic analogue commands.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionOpenCyclicAnalogChannel(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function opens a cyclic channel for cyclic torque commands.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionOpenCyclicTorqueChannel(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands a cyclic position. It is possible to give an array to paramSet
//! and to set the position of several axes using one call only.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[in]    numElements         Number of given positions in paramSet.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionWriteCyclicPosition(const MLPIHANDLE connection, const MlpiMotionCyclic* paramSet, const ULONG numElements=1);


//! @ingroup MotionLibMovement
//! This function commands a cyclic velocity. It is possible to give an array to paramSet
//! and to set the velocity of several axes with only one call in this way.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[in]    numElements         Number of given velocities in paramSet.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionWriteCyclicVelocity(const MLPIHANDLE connection, const MlpiMotionCyclic* paramSet, const ULONG numElements=1);


//! @ingroup MotionLibMovement
//! This function commands a cyclic analog value. It is possible to give an array to paramSet
//! and to set the velocity of several axes using only one call in this way.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[in]    numElements         Number of given analog values in paramSet.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionWriteCyclicAnalog(const MLPIHANDLE connection, const MlpiMotionCyclic* paramSet, const ULONG numElements=1);


//! @ingroup MotionLibMovement
//! This function commands a cyclic torque value. It is possible to give an array to paramSet
//! and to set the torque of several axes with only one call in this way.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[in]    numElements         Number of given torque values in paramSet.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionWriteCyclicTorque(const MLPIHANDLE connection, const MlpiMotionCyclic* paramSet, const ULONG numElements=1);


//! @ingroup MotionLibMovement
//! This function switches the axis to controlled motion.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionControlOn(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function switches the axis away from controlled motion.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionControlOff(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function adds an axis to a group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   paramSet            Structure containing the information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionAddAxisToGroup(const MLPIHANDLE connection, const MlpiMotionAdminAxisGroup* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function removes an axis from a group.
//! @param[in]   connection          Handle for multiple connections.
//! @param[in]   paramSet            Structure containing the information necessary for the command.
//! @param[out]  motionHandle        Pointer to value where motion handle will be stored.
//! @return                          Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionRemAxisFromGroup(const MLPIHANDLE connection, const MlpiMotionAdminAxisGroup* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands velocity synchronization to the master.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // sync axis 2 to axis 1
//! MlpiMotionGearIn cmdGearIn;
//! cmdGearIn.master      = 1; // master axis
//! cmdGearIn.axis        = 2; // slave axis
//! cmdGearIn.numerator   = 1;
//! cmdGearIn.denominator = 1;
//! cmdGearIn.fineadjust  = 0;
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionGearIn(connection, &cmdGearIn, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGearIn(const MLPIHANDLE connection, const MlpiMotionGearIn* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands position synchronization to the master.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // sync axis 2 to axis 1
//! MlpiMotionGearInPos cmdGearInPos;
//! cmdGearInPos.master      = 1; // master axis
//! cmdGearInPos.axis        = 2; // slave axis
//! cmdGearInPos.numerator   = 1;
//! cmdGearInPos.denominator = 1;
//! cmdGearInPos.fineadjust  = 0;
//! cmdGearInPos.startMode   = MLPI_STARTMODE_ABSOLUTE;
//! cmdGearInPos.syncMode    = MLPI_SYNC_SHORTESTWAY;
//!
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionGearInPos(connection, &cmdGearInPos, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGearInPos(const MLPIHANDLE connection, const MlpiMotionGearInPos* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an axis to synchronize to the master by using a cam table.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionCamIn(const MLPIHANDLE connection, const MlpiMotionCamIn* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an axis to synchronize to the master by using a motion profile.
//! The command can only be used on IndraDrives with interpolation in the drive.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionMotionProfile(const MLPIHANDLE connection, const MlpiMotionMotionProfile* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an axis to synchronize to the master by using a flex profile.
//! The command can only be used on IndraDrives with interpolation in the control or with
//! Sercos drives.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // start FlexProfile 0 on slave axis 2.
//! // note that you need to write a profile to axis 2 prior to
//! // starting the FlexProfile. Do this by using the function
//! // mlpiMotionChangeFlexProfile().
//! MlpiMotionFlexProfile cmdFlex;
//! cmdFlex.master                   = 1; // master axis
//! cmdFlex.axis                     = 2; // slave axis
//! cmdFlex.numerator                = 1;
//! cmdFlex.denominator              = 1;
//! cmdFlex.fineadjust               = 0;
//! cmdFlex.profileEntry             = MLPI_SLAVE_ORIGIN_MASTER_ORIGIN;
//! cmdFlex.executionMode            = MLPI_EXECUTE_CYCLIC;
//! cmdFlex.masterOffset             = 0;
//! cmdFlex.slaveOffset              = 0;
//! cmdFlex.setSelection             = MLPI_PROFILE_SET_0;
//! cmdFlex.switchingPosition        = 0.0;
//! cmdFlex.useSwitchingPositioning  = false;
//! cmdFlex.syncVelocity             = 100;
//! cmdFlex.syncAcceleration         = 100;
//! cmdFlex.syncType                 = MLPI_SYNC_RAMPIN_SHORTESTWAY;
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionFlexProfile(connection, &cmdFlex, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionFlexProfile(const MLPIHANDLE connection, const MlpiMotionFlexProfile* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an axis to become unsynchronized. The axis will continue to run
//! with the velocity at the time the command was issued. Can be called after GearIn, GearInPos
//! MotionProfile, FlexProfile and CamIn.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Reference to the axis.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//! MLPIRESULT result = mlpiMotionSynchronOut(connection, 1, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // wait until sync out is finished
//! result = utilMotionWait(connection, axis, motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSynchronOut(const MLPIHANDLE connection, const MlpiAxisRef axis, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an axis to perform a master phase offset when the drive is
//! synchronized to the master. The command can also be executed when unsynchronized, but it
//! will only take effect when the axis is synchronized.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // set the commanding values
//! MlpiMotionPhasing cmdPhasing;
//! cmdPhasing.axis         = 1;
//! cmdPhasing.phaseShift   = 90;
//! cmdPhasing.velocity     = 100;
//! cmdPhasing.acceleration = 10;
//! cmdPhasing.deceleration = 10;
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionPhasing(connection, &cmdPhasing, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionPhasing(const MLPIHANDLE connection, const MlpiMotionPhasing* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibMovement
//! This function commands an axis to perform a slave phase offset when the drive is
//! synchronized to the master. The command can also be executed when unsynchronized, but it
//! will only take effect when synchronizing the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    paramSet            Structure containing all information necessary for the command.
//! @param[out]   motionHandle        Pointer to value where motion handle will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIMOTIONHANDLE motionHandle = MLPI_INVALIDHANDLE;
//!
//! // set the commanding values
//! MlpiMotionPhasing cmdPhasing;
//! cmdPhasing.axis         = 1;
//! cmdPhasing.phaseShift   = 90;
//! cmdPhasing.velocity     = 100;
//! cmdPhasing.acceleration = 10;
//! cmdPhasing.deceleration = 10;
//!
//! // do the command
//! MLPIRESULT result = mlpiMotionPhasingSlave(connection, &cmdPhasing, &motionHandle);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionPhasingSlave(const MLPIHANDLE connection, const MlpiMotionPhasing* paramSet, MLPIMOTIONHANDLE* motionHandle);


//! @ingroup MotionLibConfiguration
//! This function returns a list of configured axes.
//! @param[in]    connection          Handle for multiple connections.
//! @param[out]   configAxes          Pointer to an array of structures where the configured axes are stored.
//! @param[in]    numElements         Number of  structures available in the array. A number of 0 is valid and will not copy any data.
//! @param[out]   numElementsRet      Pointer to a variable where the number of configured axes will be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! ULONG numAxes;
//! MlpiAxisInformation configuredAxes[100];
//!
//! // read current axes configured from motion system
//! memset(configuredAxes, 0, sizeof(configuredAxes));
//! result = mlpiMotionGetConfiguredAxes(connection, configuredAxes, _countof(configuredAxes), &numAxes);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! // print information about all configured axes
//! printf("\nFound %d configured axes:", numAxes);
//! for (ULONG i=0; i<numAxes; i++) {
//!   printf("\n - (%d) %s, ", configuredAxes[i].axis.axisNo, W2A16(configuredAxes[i].name));
//!
//!   switch (configuredAxes[i].axisType) {
//!         case MLPI_AXISTYPE_VIRTUAL:     printf("(VIRTUAL)");     break;
//!         case MLPI_AXISTYPE_REAL:        printf("(REAL)");        break;
//!         case MLPI_AXISTYPE_ENCODER:     printf("(ENCODER)");     break;
//!         case MLPI_AXISTYPE_LINK:        printf("(LINK)");        break;
//!         case MLPI_AXISTYPE_CONTROLLER:  printf("(CONTROLLER)");  break;
//!   }
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetConfiguredAxes(const MLPIHANDLE connection, MlpiAxisInformation* configAxes, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup MotionLibConfiguration
//! This function fills an array of MlpiAxisValues structures. Use the @c axis element of the structure to specify the
//! axis for which information should be read.
//! You may want to use this structure to read several sets of axis information for several axes using one single
//! function call during operation of the axes. This gives increased performance in comparison to reading the values
//! bit by bit and axis by axis.
//! @param[in]      connection        Handle for multiple connections.
//! @param[in, out] axisValues        Fills a struct with the current operation information about axes. AxisRef has to be set by the caller.
//! @param[in]      numElements       Number of axes for which values should be read. This is the array length of the parameter @c axisValues.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @note Elements of the struct that can not be read will be set to 0.
//!
//! @par Example:
//! @code
//! // This example reads a list of all current configured axes.
//! // It uses this information to read the axis values (position, velocity, state...)
//! // as well as the corresponding units and shows a summary of the state of all
//! // axes.
//! //
//! // Please note that 3 different MLPI functions are used. This is due to performance
//! // reasons. You may want to call mlpiMotionGetConfiguredAxes and mlpiMotionGetAxisUnits
//! // only when the axis configuration is changed. In contrast, the function
//! // mlpiMotionGetAxisValues can be polled more often and contains values which are
//! // more dynamic (position, velocity, state, diagnosis...). You can also use it for
//! // a subset of axes only.
//!
//! MLPIRESULT result;
//! ULONG numAxes;
//! MlpiAxisInformation configuredAxes[100];
//!
//! // read current axes configured from motion system to know which axes are available
//! memset(configuredAxes, 0, sizeof(configuredAxes));
//! result = mlpiMotionGetConfiguredAxes(connection, configuredAxes, _countof(configuredAxes), &numAxes);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! printf("\nFound %d configured axes:", numAxes);
//!
//! // build list of axis to read the values for
//! MlpiAxisValues axisValues[100];
//! MlpiAxisUnits axisUnits[100];
//! for (ULONG i=0; i<numAxes; i++) {
//!   axisValues[i].axis = configuredAxes[i].axis;
//!   axisUnits[i].axis = configuredAxes[i].axis;
//! }
//!
//! // read all axis values for the given axes list
//! result = mlpiMotionGetAxisValues(connection, axisValues, numAxes);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! result = mlpiMotionGetAxisUnits(connection, axisUnits, numAxes);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! // let's print some information of all axes
//! for (ULONG i=0; i<numAxes; i++) {
//!   printf("\n--- (%d) %s ---", axisValues[i].axis.axisNo, W2A16(configuredAxes[i].name));
//!   printf("\n Position:     %lf %s", axisValues[i].actualPosition, W2A16(axisUnits[i].position));
//!   printf("\n Velocity:     %lf %s", axisValues[i].actualVelocity, W2A16(axisUnits[i].velocity));
//!   printf("\n Acceleration: %lf %s", axisValues[i].actualAcceleration, W2A16(axisUnits[i].acceleration));
//!   printf("\n Torque:       %lf %s", axisValues[i].actualTorque, W2A16(axisUnits[i].torque));
//!   printf("\n Diagnosis:    0x%08X", axisValues[i].diagnosisNumber);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetAxisValues(const MLPIHANDLE connection, MlpiAxisValues* axisValues, const ULONG numElements);


//! @ingroup MotionLibConfiguration
//! This function returns the current unit settings of an array of axes in string representation. Use it to read the unit settings for
//! display in an HMI. If you want detailed information about the unit and scaling settings of the axis, please look
//! at the functions @ref mlpiMotionGetPositionScaling, @ref mlpiMotionGetVelocityScaling, etc...
//! @param[in]        connection          Handle for multiple connections.
//! @param[in, out]   axisUnits           Fills a struct with the current unit settings of the axis. AxisRef in the struct has to be set by the caller.
//! @param[in]        numElements         Number of axes for which values should be read. This is the array length of the parameter @c axisUnits.
//! @return                               Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref mlpiMotionGetAxisValues
MLPI_API MLPIRESULT mlpiMotionGetAxisUnits(const MLPIHANDLE connection, MlpiAxisUnits* axisUnits, const ULONG numElements);

//! @ingroup MotionLibConfiguration
//! This function fills an array of MlpiAxisStatus structures. Use the @c axis element of the structure to specify the
//! axis for which information should be read.
//! You may want to use this function to read several sets of axis status information for several axes using one single
//! function call during operation of the axes. This gives increased performance in comparison to reading the values
//! with the function @ref mlpiMotionGetAxisValues, since the actual values of axes are not needed (e.g. actual position A-0-0101).
//! @param[in]      connection        Handle for multiple connections.
//! @param[in, out] axisStatus        Fills a struct with the current operation information about axes. AxisRef has to be set by the caller.
//! @param[in]      numElements       Number of axes for which status values should be read. This is the array length of the parameter @c axisStatus.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @note Elements of the struct that can not be read will be set to 0.
//!
//! @par Example:
//! @code
//! // This example reads a list of all current configured axes.
//! // It uses this information to read the axis status values (state, stateExtended, diagnosis...)
//! // of all axes.
//! //
//! // Please note that 2 different MLPI functions are used. This is due to performance
//! // reasons. You may want to call mlpiMotionGetConfiguredAxes
//! // only when the axis configuration is changed. In contrast, the function
//! // mlpiMotionGetAxisStatus can be polled more often and contains status values which are
//! // more dynamic (state, diagnosis...). You can also use it for
//! // a subset of axes only.
//!
//! MLPIRESULT result;
//! ULONG numAxes;
//! MlpiAxisInformation configuredAxes[100];
//!
//! // read current axes configured from motion system to know which axes are available
//! memset(configuredAxes, 0, sizeof(configuredAxes));
//! result = mlpiMotionGetConfiguredAxes(connection, configuredAxes, _countof(configuredAxes), &numAxes);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//! printf("\nFound %d configured axes:", numAxes);
//!
//! // build list of axis to read the status values for
//! MlpiAxisStatus axisStatus[100];
//! for (ULONG i=0; i<numAxes; i++) {
//!   axisStatus[i].axis = configuredAxes[i].axis;
//! }
//!
//! // read all axis status values for the given axes list
//! result = mlpiMotionGetAxisStatus(connection, axisStatus, numAxes);
//!
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", result);
//!   return result;
//! }
//!
//! // let's print some information of all axes
//! for (ULONG i=0; i<numAxes; i++) {
//!   printf("\n--- (%d) %s ---", axisStatus[i].axis.axisNo, W2A16(configuredAxes[i].name));
//!   printf("\n Diagnosis:    0x%08X", axisStatus[i].diagnosisNumber);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetAxisStatus(const MLPIHANDLE connection, MlpiAxisStatus* axisStatus, const ULONG numElements);

//! @ingroup MotionLibConfiguration
//! This function creates an axis. If an axis already exists, it will be overwritten.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axisType            Defines the axis type.
//! @param[in]    name                Name of the axis. It will not be used other than for display purposes.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    deviceAddress       Sercos address of the axis. In case of a virtual axis, this parameter is not used.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionCreateAxis(const MLPIHANDLE connection, const MlpiAxisType axisType, const WCHAR16* name, const MlpiAxisRef axis, const ULONG deviceAddress);


//! @ingroup MotionLibConfiguration
//! This function destroys an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis to be destroyed.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionDestroyAxis(const MLPIHANDLE connection, const MlpiAxisRef axis);


//! @ingroup MotionLibConfiguration
//! This function reads the actual position of a axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the position
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetActualPosition(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent position of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetActualPosition(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the actual velocity of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the velocity
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetActualVelocity(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent velocity of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetActualVelocity(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the actual acceleration of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the acceleration
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetActualAcceleration(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent acceleration of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetActualAcceleration(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the actual torque of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the torque
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetActualTorque(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent torque of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetActualTorque(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the actual interpolated position of an axis. The interpolated position is the position as
//! calculated by the interpolator of the axis. For axes with interpolation in the control, this is the value that
//! gets sent to the drive as a commanded value when running an operation mode with position control.
//! If there is no operation mode with position interpolation active, or if the interpolation is not done in the
//! control, then this function might return an error.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the interpolated position
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetInterpolatedPosition(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent interpolated position of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetInterpolatedPosition(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the actual interpolated velocity of an axis. The interpolated velocity is the velocity as
//! calculated by the interpolator of the axis. For axes with interpolation in the control, this is the value that
//! gets sent to the drive as a commanded value when running
//! an operation mode with velocity control.
//! If there is no operation mode with velocity interpolation active, or if the interpolation is not done in the
//! control, then this function might return an error.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the interpolated velocity
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetInterpolatedVelocity(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent interpolated velocity of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetInterpolatedVelocity(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the interpolated torque of an axis.
//! If there is no operation mode with torque interpolation active, or if the interpolation is not done in the
//! control, then this function might return an error.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the interpolated torque
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetInterpolatedTorque(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nCurrent interpolated torque of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetInterpolatedTorque(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function reads the positive position limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the positive position limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetPositionLimitPos(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nPositive position limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetPositionLimitPos(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the positive position limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the positive position limit
//! DOUBLE value = 36000.0;
//! MLPIRESULT result = mlpiMotionSetPositionLimitPos(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nPositive position limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetPositionLimitPos(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the negative position limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the negative position limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetPositionLimitNeg(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nNegative position limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetPositionLimitNeg(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the negative position limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the negative position limit
//! DOUBLE value = -36000.0;
//! MLPIRESULT result = mlpiMotionSetPositionLimitNeg(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nNegative position limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetPositionLimitNeg(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the positive velocity limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the positive velocity limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetVelocityLimitPos(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nPositive velocity limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetVelocityLimitPos(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the positive velocity limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the positive velocity limit
//! DOUBLE value = 1000.0;
//! MLPIRESULT result = mlpiMotionSetVelocityLimitPos(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nPositive velocity limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetVelocityLimitPos(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the negative velocity limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the negative velocity limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetVelocityLimitNeg(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nNegative velocity limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetVelocityLimitNeg(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the negative velocity limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the negative velocity limit
//! DOUBLE value = -1000.0;
//! MLPIRESULT result = mlpiMotionSetVelocityLimitNeg(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nNegative velocity limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetVelocityLimitNeg(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the bipolar velocity limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the bipolar velocity limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetVelocityLimitBip(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar velocity limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetVelocityLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the bipolar velocity limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the bipolar velocity limit
//! DOUBLE value = 1000.0;
//! MLPIRESULT result = mlpiMotionSetVelocityLimitBip(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar velocity limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetVelocityLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the bipolar acceleration limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the bipolar acceleration limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetAccelerationLimitBip(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar acceleration limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetAccelerationLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the bipolar acceleration limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the bipolar acceleration limit
//! DOUBLE value = 1000.0;
//! MLPIRESULT result = mlpiMotionSetAccelerationLimitBip(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar acceleration limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetAccelerationLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the bipolar jerk limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the bipolar jerk limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetJerkLimitBip(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar jerk limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetJerkLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the bipolar jerk limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the bipolar jerk limit
//! DOUBLE value = 10.0;
//! MLPIRESULT result = mlpiMotionSetJerkLimitBip(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar jerk limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetJerkLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the bipolar torque force limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the bipolar torque force limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetTorqueLimitBip(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar torque force limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetTorqueLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the bipolar torque force limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the bipolar torque force limit
//! DOUBLE value = 100.0;
//! MLPIRESULT result = mlpiMotionSetTorqueLimitBip(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nBipolar torque force limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetTorqueLimitBip(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the positive torque force limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the positive torque force limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetTorqueLimitPos(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nPositive torque force limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetTorqueLimitPos(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the positve torque force limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the positive torque force limit
//! DOUBLE value = 100.0;
//! MLPIRESULT result = mlpiMotionSetTorqueLimitPos(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nPositive torque force limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetTorqueLimitPos(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the negative torque force limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the negative torque force limit
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetTorqueLimitNeg(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nNegative torque force limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetTorqueLimitNeg(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the negative torque force limit of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the negative torque force limit
//! DOUBLE value = -100.0;
//! MLPIRESULT result = mlpiMotionSetTorqueLimitNeg(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nNegative torque force limit of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetTorqueLimitNeg(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the position scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionGetPositionScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, USHORT* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the position scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // set position scaling of axis 1 to rotatory modulo
//! MLPIRESULT result = mlpiMotionSetPositionScaling(connection, 1, MlpiAxisScalingPosition(MLPI_SCALING_TYPE_ROTATORY, MLPI_SCALING_FORMAT_MODULO));
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetPositionScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, const USHORT value);


//! @ingroup MotionLibConfiguration
//! This function reads the velocity scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionGetVelocityScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, USHORT* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the velocity scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // set velocity scaling of axis 1 to rotatory and seconds (deg/s)
//! MLPIRESULT result = mlpiMotionSetVelocityScaling(connection, 1, MlpiAxisScalingVelocity(MLPI_SCALING_TYPE_ROTATORY, MLPI_SCALING_TIME_SECOND));
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetVelocityScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, const USHORT value);


//! @ingroup MotionLibConfiguration
//! This function reads the acceleration scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionGetAccelerationScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, USHORT* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the velocity scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // set acceleration scaling of axis 1 to rotatory and seconds (deg/s)
//! MLPIRESULT result = mlpiMotionSetAccelerationScaling(connection, 1, MlpiAxisScalingAcceleration(MLPI_SCALING_TYPE_ROTATORY));
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetAccelerationScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, const USHORT value);


//! @ingroup MotionLibConfiguration
//! This function reads the acceleration scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
MLPI_API MLPIRESULT mlpiMotionGetTorqueScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, USHORT* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the torque scaling of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // set torque scaling of axis 1 to rotatory and newton meter
//! MLPIRESULT result = mlpiMotionSetTorqueScaling(connection, 1, MlpiAxisScalingTorque(MLPI_SCALING_TYPE_ROTATORY));
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetTorqueScaling(const MLPIHANDLE connection, const MlpiAxisRef axis, const USHORT value);


//! @ingroup MotionLibConfiguration
//! This function reads the modulo value of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the modulo value
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetModulo(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nModulo value of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetModulo(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the modulo value for an axis of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the modulo value
//! DOUBLE value = 360.0;
//! MLPIRESULT result = mlpiMotionSetModulo(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nModulo value of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetModulo(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the feed distance of a following axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the feed distance
//! DOUBLE value = 0.0;
//! MLPIRESULT result = mlpiMotionGetSlaveDriveFeedTravel(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nFeed distance of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetSlaveDriveFeedTravel(const MLPIHANDLE connection, const MlpiAxisRef axis, DOUBLE* value);


//! @ingroup MotionLibConfiguration
//! This function writes the feed distance of a following axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    value               Data value to be written to the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to write the feed distance
//! DOUBLE value = 10.0;
//! MLPIRESULT result = mlpiMotionSetSlaveDriveFeedTravel(connection, 1, value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nFeed distance of axisNo 1: %lf", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetSlaveDriveFeedTravel(const MLPIHANDLE connection, const MlpiAxisRef axis, const DOUBLE value);


//! @ingroup MotionLibConfiguration
//! This function reads the state of an axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiAxisRef axis(1);
//!
//! // read the state
//! ULONG axisState = 0;
//! result = mlpiMotionGetState(connection, axis, &axisState);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // we use the helper struct from #include <util/mlpiMotionHelper.h> to decode the axisState
//! // this makes it easier to read the code:
//! // if(state.Error()) {
//! //   mlpiMotionClearError(axis);
//! // }
//! // ... and so on...
//! MlpiAxisStateDecoder state = axisState;
//!
//! printf("\nAxis is in velocity..................................%s", state.InVelocity() ? "true" : "false");
//! printf("\nAxis is standstill...................................%s", state.Standstill() ? "true" : "false");
//! printf("\nAxis is in position..................................%s", state.InPosition() ? "true" : "false");
//! printf("\nAxis is synchronous to another axis..................%s", state.InSynchron() ? "true" : "false");
//! printf("\nAxis has warning.....................................%s", state.Warning() ? "true" : "false");
//! printf("\nAxis has an error....................................%s", state.Error() ? "true" : "false");
//! printf("\nAxis is homed........................................%s", state.Homed() ? "true" : "false");
//! printf("\nAxis is in torque....................................%s", state.InTorque() ? "true" : "false");
//! printf("\nAxis is in P4........................................%s", state.OperationMode() ? "true" : "false");
//! printf("\nAxis is ready for operation..........................%s", state.InAb() ? "true" : "false");
//! printf("\nAxis is ready for power..............................%s", state.Inbb() ? "true" : "false");
//! printf("\nAxis has power.......................................%s", state.Power() ? "true" : "false");
//! printf("\nAxis has an active sercos command....................%s", state.CmdActive() ? "true" : "false");
//! printf("\nError reaction of axis is in progress................%s", state.ErrInProgress() ? "true" : "false");
//! printf("\nAxis has reached the master phase offset.............%s", state.InMasterPhaseOffset() ? "true" : "false");
//! printf("\nNRT channel for axis is connected....................%s", state.NRTActive() ? "true" : "false");
//! printf("\nAxis did not follow control..........................%s", state.Interrupted() ? "true" : "false");
//! printf("\nAxis is in modulo format.............................%s", state.Modulo() ? "true" : "false");
//! printf("\nAxis is rotatory.....................................%s", state.Rotatory() ? "true" : "false");
//! printf("\nActual data of axis is valid.........................%s", state.Valid() ? "true" : "false");
//! printf("\nAxis is decoupled from commanded position of control.%s", state.Decoupled() ? "true" : "false");
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetState(const MLPIHANDLE connection, const MlpiAxisRef axis, ULONG* value);


//! @ingroup MotionLibConfiguration
//! This function reads the extended state of the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MlpiAxisRef axis(1);
//!
//! // read the state
//! ULONG axisStateExtended = 0;
//! result = mlpiMotionGetStateExtended(connection, axis, &axisStateExtended);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // we use the helper struct from #include <util/mlpiMotionHelper.h> to decode the axisState
//! // this makes it easier to read the code:
//! // if(state.Standstill()) {
//! //   // do something...
//! // }
//! // ... and so on...
//! MlpiAxisStateExtendedDecoder state = axisStateExtended;
//!
//! printf("\nAxis is in state ErrorStop...........%s", state.ErrorStop() ? "true" : "false");
//! printf("\nAxis is in state Stopping............%s", state.Stopping() ? "true" : "false");
//! printf("\nAxis is in state Homing..............%s", state.Homing() ? "true" : "false");
//! printf("\nAxis has power.......................%s", state.PowerOn() ? "true" : "false");
//! printf("\nAxis is in state DiscreteMotion......%s", state.DiscreteMotion() ? "true" : "false");
//! printf("\nAxis is in state ContinuousMotion....%s", state.ContinuousMotion() ? "true" : "false");
//! printf("\nAxis is in state SynchronizedMotion..%s", state.SynchronizedMotion() ? "true" : "false");
//! printf("\nAxis is in state Standstill..........%s", state.Standstill() ? "true" : "false");
//! printf("\nAxis is in state CoordinatedMotion...%s", state.CoordinatedMotion() ? "true" : "false");
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetStateExtended(const MLPIHANDLE connection, const MlpiAxisRef axis, ULONG* value);


//! @ingroup MotionLibConfiguration
//! This function reads the diagnostic number of the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   number              Pointer to a variable which is receiving the error number.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the diagnostic number
//! ULONG value = 0;
//! MLPIRESULT result = mlpiMotionGetDiagnosisNumber(connection, 1, &value);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nDiagnostic number of axisNo 1: 0x%08X", value);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetDiagnosisNumber(const MLPIHANDLE connection, const MlpiAxisRef axis, ULONG* number);


//! @ingroup MotionLibConfiguration
//! This function reads the diagnostic message of the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   buffer              Pointer to where text should be stored.
//! @param[in]    numElements         Number of available WCHAR16 characters in buffer.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the diagnostic message
//! WCHAR16 buffer[128] = L"";
//! MLPIRESULT result = mlpiMotionGetDiagnosisText(connection, 1, buffer, _countof(buffer));
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nDiagnostic message of axisNo 1: %s", W2A16(buffer));
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetDiagnosisText(const MLPIHANDLE connection, const MlpiAxisRef axis, WCHAR16* buffer, const ULONG numElements);


//! @ingroup MotionLibConfiguration
//! This function reads the name of the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   name                Pointer to where text should be stored.
//! @param[in]    numElements         Number of available WCHAR16 characters in buffer.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the axis name
//! WCHAR name[128] = L"";
//! MLPIRESULT result = mlpiMotionGetName(connection, 1, buffer, _countof(name));
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! else
//!   printf("\nName of axisNo 1: %s", W2A16(name));
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetName(const MLPIHANDLE connection, const MlpiAxisRef axis, WCHAR16* name, const ULONG numElements);


//! @ingroup MotionLibConfiguration
//! This function writes the name of the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]    name                Name of the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to change the axis name
//! MLPIRESULT result = mlpiMotionSetName(connection, 1, L"myAxisName");
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetName(const MLPIHANDLE connection, const MlpiAxisRef axis, const WCHAR16* name);


//! @ingroup MotionLibConfiguration
//! This function reads the type of the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   type                Pointer to where value should be stored. The value contains a bit sequence
//!                                   that can be interpreted with @c MlpiAxisTypeDecoder. Refer to mlpiMotionHelper.h
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the type of axis
//! USHORT type;
//! MLPIRESULT result = mlpiMotionGetAxisType(connection, 1, &type);
//! if (MLPI_FAILED(result)){
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // decode the type using AxisTypeDecoder see #include <util/mlpiMotionHelper.h>
//! MlpiAxisTypeDecoder axisType(type);
//! switch(axisType.Type())
//! {
//! case MLPI_AXISTYPE_VIRTUAL:     printf("\nType of axisNo 1 is VIRTUAL");
//!   printf(" - Virtual axis, no physical drive attached.");                                          break;
//! case MLPI_AXISTYPE_REAL:        printf("\nType of axisNo 1 is REAL");
//!   printf(" - Real axis, this is the common axis when doing motion.");                              break;
//! case MLPI_AXISTYPE_ENCODER:     printf("\nType of axisNo 1 is ENCODER");
//!   printf(" - An encoder that is attached to a real drive, no motion possible.");                   break;
//! case MLPI_AXISTYPE_LINK:        printf("\nType of axisNo 1 is LINK");
//!   printf(" - A link ring axis.");                                                                  break;
//! case MLPI_AXISTYPE_CONTROLLER:  printf("\nType of axisNo 1 is CONTROLLER ");
//!   printf(" - An axis that can be used when generating your own controller to operate the drive."); break;
//! default:                        printf("\nType of axisNo 1 is invalid.");                          break;
//! }
//!
//! printf("\nAxis is in interpolation in control......................%s", axisType.InterpolationInControl() ? "true" : "false");
//! printf("\nAttached drive is configured in sercos PackProfile mode..%s", axisType.IsPackProfile() ? "true" : "false");
//! printf("\nAttached drive is a hydraulic drive......................%s", axisType.IsHydraulicDrive() ? "true" : "false");
//! printf("\nAxis supports Parametrization mode.......................%s", axisType.SupportsParametrization() ? "true" : "false");
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetAxisType(const MLPIHANDLE connection, const MlpiAxisRef axis, USHORT* type);


//! @ingroup MotionLibConfiguration
//! This function reads the axis condition.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[out]   value               Pointer to where value should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to read the condition of an axis
//! ULONG condition;
//! MLPIRESULT result = mlpiMotionGetCondition(connection, 1, &condition);
//! if (MLPI_FAILED(result)){
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//!
//! // decode the type using MlpiAxisConditionDecoder see #include \<util/mlpiMotionHelper.h\>
//! MlpiAxisConditionDecoder axisCondition(condition);
//! printf("\nAxis condition: %d", axisCondition.Condition());
//! printf("\nAxis is active................%s", axisCondition.IsActive() ? "true" : "false");
//! printf("\nAxis is deactivated...........%s", axisCondition.IsDeactivated() ? "true" : "false");
//! printf("\nAxis is in parking............%s", axisCondition.IsParking() ? "true" : "false");
//! printf("\nAxis is in parametrization....%s", axisCondition.IsParametrization() ? "true" : "false");
//! printf("\nAxis is decoupled.............%s", axisCondition.IsDecoupled() ? "true" : "false");
//! @endcode
MLPI_API MLPIRESULT mlpiMotionGetCondition(const MLPIHANDLE connection, const MlpiAxisRef axis, ULONG* value);


//! @ingroup MotionLibConfiguration
//! This function writes to the axis condition.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @param[in]   value               Condition that should be stored.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // use the mlpi function to set the condition of axis to active
//! MLPIRESULT result = mlpiMotionSetCondition(connection, 1, MLPI_AXIS_CONDITION_ACTIVE);
//! if (MLPI_FAILED(result)){
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return -1;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiMotionSetCondition(const MLPIHANDLE connection, const MlpiAxisRef axis, const ULONG value);


//! @ingroup MotionLibConfiguration
//! Load default parameters for the axis.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @note During the execution of the command all parameter values of the axis will be set to their initial values.
//!       Considering that IndraWorks use some parameters for global identification etc., you have to save and restore
//!       some parameter values therewith you are able to reconnect with Indraworks without another project download.
//! <TABLE>
//! <TR><TH> Axis parameter (A)         </TH><TH> Save and restore parameter...               </TH></TR>
//! <TR><TD rowspan="4"> A-0-1000       </TD><TD> A-0-0002, Axis name                         </TD></TR>
//! <TR>                                     <TD> A-0-0016, Project identification number     </TD></TR>
//! <TR>                                     <TD> A-0-0008, Link axis, master axis selection  </TD></TR>
//! <TR>                                     <TD> A-0-0630, Controller axis, controller name  </TD></TR>
//! </TABLE>
//!
//! @par Example:
//! @code
//! // use the mlpi function to load default parameter
//! MLPIRESULT result = mlpiMotionLoadDefaultParameters(connection, 1);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionLoadDefaultParameters(const MLPIHANDLE connection, const MlpiAxisRef axis);


//! @ingroup MotionLibConfiguration
//! Clears a pending error for the axis. An axis needs to be free of errors before commanding
//! power and start of movement.
//! @param[in]    connection          Handle for multiple connections.
//! @param[in]    axis                Logical address of the axis.
//! @return                           Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // clear all pending errors
//! MLPIRESULT result = mlpiMotionClearError(connection, 1);
//!
//! // evaluate for success or error
//! if (MLPI_FAILED(result))
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//! @endcode
MLPI_API MLPIRESULT mlpiMotionClearError(const MLPIHANDLE connection, const MlpiAxisRef axis);



#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPIMOTIONLIB_H__
