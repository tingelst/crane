#ifndef __MLPITASKLIB_H__
#define __MLPITASKLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiTaskLib.h>
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



//! @addtogroup TaskLib TaskLib
//! @{
//! @brief Control and change the tasking and scheduling of the MLC/XLC
//!        system and the user application.
//!
//! Using the task lib, it is possible to hook your task into the real-time task scheduler of the target system.
//! Of course, this only makes sense if your program runs on the same target as the device you are connected to. Using
//! the TaskLib when connecting via MLPI over Ethernet is possible, but the timing of the methods is non-deterministic.
//! But when running with your app on the target, you can make sure that your task is one of the first tasks to get scheduled, when for example new
//! data has arrived on the sercos bus. To do this, you have to increase the task priority of your task using
//! @ref mlpiTaskSetCurrentPriority and then you may want to wait for a given system event using @ref mlpiTaskWaitForEvent.
//! @attention Your task is scheduled with the same rights as every other (system) task in a fully native real-time environment! So special
//!            care is needed, when running your task at higher priority levels. Your task will block any task with a lower priority level as long
//!            as it is running. Programming an infinite loop will most likely look up your system (a system watchdog will stop your system).
//!            If you have never programmed a real-time system, then please have a look at the manual of your operating system and read about
//!            the mechanics of a real-time scheduler!
//!
//!
//! Here is some example code:
//! @code
//! extern "C" int taskFunc(MLPIHANDLE connection)
//! {
//!   // We use this example to make sure our task gets scheduled every time new data from the sercos has arrived.
//!   // This way, we can have a code or an algorithm executed every sercos cycle. Much like having
//!   // a cyclic PLC event task.
//!
//!   // Set task priority to high to make it one of the first tasks to run
//!   // as soon as the event occurs.
//!   MLPIRESULT result = mlpiTaskSetCurrentPriority(connection, MLPI_PRIORITY_HIGH_MAX);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return result;
//!   }
//!
//!   unsigned long cycleCounter = 0;
//!   bool bRunning = true;
//!   do {
//!     // wait for the next sercos cycle. Task will be set to state "pending"
//!     // and not consume any cpu time.
//!     MLPIRESULT result = mlpiTaskWaitForEvent(connection, MLPI_TASKEVENT_SERCOS_CYCLE, MLPI_INFINITE);
//!     if (MLPI_FAILED(result)) {
//!       printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!       break;
//!     }
//!
//!     // you may want to do some cyclic calculation in here which
//!     // depends on the sercos data, e.g. read input values, write outputs for
//!     // next cycle, calculate new commanded axis positions, etc...
//!     // note: do not block forever in here!
//!
//!
//!     // Set bRunning to false if you want to exit the loop. In this example, we
//!     // exit after 1000 loops.
//!     cycleCounter++;
//!     if (cycleCounter>1000)
//!       bRunning = false;
//!
//!   }while (bRunning);
//!
//!   // set task priority back to background to not block any other tasks.
//!   result = mlpiTaskSetCurrentPriority(connection, MLPI_PRIORITY_BACKGROUND);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return result;
//!   }
//!   return MLPI_S_OK;
//! }
//! @endcode
//!
//! Here is another example which shows how to start our function on a VxWorks target. For more information about
//! the usage of task on VxWorks, please have a look at the VxWorks Reference help regarding the taskLib functionality.
//! @code
//! taskSpawn("ExampleTask", 100, VX_FP_TASK, 0x200000, (FUNCPTR)&taskFunc, connection, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//! @endcode
//!
//! @note The TaskLib functions trace their debug information mainly into the module MLPI_TASK_LIB
//!       and in addition into the module MLPI_BASE_MODULES. For further information, see also
//!       the detailed description of the library @ref TraceLib and the notes about @ref sec_TraceViewer.
//!
//! @}

//! @addtogroup TaskLibTaskControl Control Tasking
//! @ingroup TaskLib
//! @{
//! @brief The following functions are used for controlling task and schedule behavior.
//! @}

//! @addtogroup TaskLibTaskViewer Control Task Viewer
//! @ingroup TaskLib
//! @{
//! @brief The task viewer resp. "Task Execution Viewer" is part of the development environment IndraWorks and
//!        shows a restricted chronological sequence of task processing on the control. The functions of these
//!        library part can be used to control and interact with the task viewer functionality in the control.
//!
//! @image html TaskViewerTaskOverview.png "Task viewer - overview." \n
//!
//! @note  All recorded tasks must be present at time of call of function @ref mlpiTaskViewerStop otherwise
//!        the task viewer functionality is not able to determine the name and the priority of a missing task.
//!
//! @note  The task viewer functionality is not able to recognize any changes of priority of a task. The task
//!        viewer functionality determine the priority of a task once at call of function @ref mlpiTaskViewerStop.
//!        For deeply debug inspection please use the "System Viewer" of your WindRiver Workbench OEM.
//! @}

//! @addtogroup TaskExecution Control Task Execution
//! @ingroup TaskLib
//! @{
//! @brief The following functions are used for controlling execution of user tasks.
//! @}

//! @addtogroup TaskLibVersionPermission Version and Permission
//! @ingroup TaskLib
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the
//! user permission needed to execute the desired function. Furthermore, the table shows the current user
//! and permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. When using
//! the permission @b "MLPI_TASKLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_TASKLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiTaskLib using in accounts.xml
//! - MLPI_TASKLIB_PERMISSION_ALL
//! - MLPI_TASKLIB_PERMISSION_EVENT_SINK
//! - MLPI_TASKLIB_PERMISSION_TASK_SETUP
//! - MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE
//! - MLPI_TASKLIB_PERMISSION_TASKTRIGGER
//!
//! <TABLE>
//! <TR><TH>           Function                         </TH><TH> Server version </TH><TH> Permission                                   </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskWaitForEvent        </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_EVENT_SINK"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskSetSystemPriority   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASK_SETUP"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskGetSystemPriority   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASK_SETUP"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskSetCurrentPriority  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASK_SETUP"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskGetCurrentPriority  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASK_SETUP"         </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskSetTriggerSetup     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKTRIGGER"        </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskGetTriggerSetup     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKTRIGGER"        </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskSetTrigger          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKTRIGGER"        </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskGetTrigger          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKTRIGGER"        </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskViewerStart         </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskViewerStop          </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskViewerAddItem       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskViewerTaskStart     </TD><TD> 1.5.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskViewerTaskStop      </TD><TD> 1.5.0.0        </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskViewerGetState      </TD><TD> 1.14.0.0       </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKVIEWER_USE"     </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskExecuteFile         </TD><TD> 1.12.0.0       </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKEXECUTION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskExecuteGetStatus    </TD><TD> 1.12.0.0       </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKEXECUTION_INFO" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskExecuteGetActive    </TD><TD> 1.12.0.0       </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKEXECUTION_INFO" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskExecuteKill         </TD><TD> 1.12.0.0       </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKEXECUTION"      </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiTaskExecuteGetName      </TD><TD> 1.12.0.0       </TD><TD> "MLPI_TASKLIB_PERMISSION_TASKEXECUTION_INFO" </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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

//! @addtogroup TaskLibStructTypes Structs, Types, ...
//! @ingroup TaskLib
//! @{
//! @brief List of used types, enumerations, structures and more...




// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
static const ULONG MLPI_PRIORITY_HIGH_MAX       =   1;    //!< The highest available level for cyclic system/user tasks. Use with mlpiTaskSetCurrentPriority and NOT with taskSpawn!
static const ULONG MLPI_PRIORITY_HIGH_MIN       =  20;    //!< The lowest available level for cyclic system/user tasks. Use with mlpiTaskSetCurrentPriority and NOT with taskSpawn!
static const ULONG MLPI_PRIORITY_BACKGROUND     = 254;    //!< Default priority for user task. Use for tasks which are doing GUI, HMI, service, etc...

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! There are different system task events in the system.
typedef enum MlpiTaskEvent
{
  MLPI_TASKEVENT_SERCOS_CYCLE                   =  0,     //!< This event raises every new sercos cycle after the actual values arrive.
  MLPI_TASKEVENT_MOTION_CYCLE                   =  1,     //!< This event raises every new motion cycle after the actual values of all axes have been updated.
  MLPI_TASKEVENT_WATCHDOG                       =  2,     //!< This event raises every time a watchdog expires.
  MLPI_TASKEVENT_PLC_TASK_AFTER_READ_INPUTS     =  3,     //!< This event raises after any plc task will reads its inputs (helpful for I/O simulation, e.g. write to inputs of task images).
  MLPI_TASKEVENT_PLC_TASK_BEFORE_WRITE_OUTPUTS  =  4      //!< This event raises before any plc task will writes its outputs (helpful for I/O simulation, e.g. read from outputs of task images).
}MlpiTaskEvent;

//! There are different system tasks.
typedef enum MlpiSystemTask
{
  MLPI_TASK_MOTIONKERNEL                        = 0       //!< This is the task which is doing all the motion calculation. For example all PLCOpen motion.
}MlpiSystemTask;

//! There are different trigger events.
typedef enum MlpiTaskTriggerEvent
{
  MLPI_TASK_TRIG_EVT_MOTION                     =  0,     //!< This event executes all motion cyclic tasks (e.g. MotionKernel for Interpolation, motion cyclic PLC-Tasks, tasks delaying on event 'MLPI_TASKEVENT_MOTION_CYCLE').
  MLPI_TASK_TRIG_EVT_SERCOS                     =  1      //!< This event executes all sercos cyclic tasks (e.g. sercos cyclic PLC-Tasks, tasks delaying on event 'MLPI_TASKEVENT_SERCOS_CYCLE').
}MlpiTaskTriggerEvent;

//! There are different trigger options.
typedef enum MlpiTaskTriggerOption
{
  MLPI_TASK_TRIG_OPT_NO_CYCLIC_MOTION           =  0,     //!< This option says if the motion is decoupled from cyclic execution.
  MLPI_TASK_TRIG_OPT_NO_CYCLIC_SERCOS           =  1,     //!< This option says if the sercos is decoupled from cyclic execution.
  MLPI_TASK_TRIG_OPT_IMMEDIATE_EXECUTION        =  2,     //!< This option says if the execution of the external trigger should be executed immediately. The events are triggered immediately one after the other, not within their cycletimes.
  MLPI_TASK_TRIG_OPT_NO_ERROR_REACTION          =  3,     //!< This option says if decoupling of events should stay active in case of an error.
  MLPI_TASK_TRIG_OPT_NO_WARNING                 =  4,     //!< This option says if an warning should be set as long as an event is decoupled from cyclic execution.
  MLPI_TASK_TRIG_OPT_SYNCHRONOUS                =  5      //!< This option says if the call of the function 'mlpiTaskSetTrigger' should be synchronous.
}MlpiTaskTriggerOption;

//! There are different task viewer states.
typedef enum MlpiTaskViewerState
{
  MLPI_TASK_VIEWER_STATE_STOPPED           =  0,     //!< This state says that the task viewer is currently stopped.
  MLPI_TASK_VIEWER_STATE_RUNNING           =  1      //!< This state says that the task viewer is currently running.
}MlpiTaskViewerState;

// message packing follows 8-byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiTaskTriggerSetup
//! @brief This structure defines the information needed to get or set the trigger setup.
//! @details Elements of structure MlpiTaskTriggerSetup
//! <TABLE>
//! <TR><TH>           Type                   </TH><TH>           Element         </TH><TH> Description                         </TH></TR>
//! <TR><TD id="st_t"> MlpiTaskTriggerOption  </TD><TD id="st_e"> triggerOption   </TD><TD> Options for task trigger            </TD></TR>
//! <TR><TD id="st_t"> BOOL8                  </TD><TD id="st_e"> active          </TD><TD> Option active or inactive           </TD></TR>
//! </TABLE>
typedef struct MlpiTaskTriggerSetup
{
  MlpiTaskTriggerOption triggerOption;                    //!< Options available for task trigger
  BOOL8 active;                                           //!< Option active or inactive
}MlpiTaskTriggerSetup;

//! @typedef MlpiTaskTrigger
//! @brief This structure defines the information to trigger events.
//! @details Elements of structure MlpiTaskTrigger
//! <TABLE>
//! <TR><TH>           Type                   </TH><TH>           Element         </TH><TH> Description </TH></TR>
//! <TR><TD id="st_t"> MlpiTaskTriggerEvent   </TD><TD id="st_e"> triggerEvent    </TD><TD> Event which should be triggered     </TD></TR>
//! <TR><TD id="st_t"> BOOL8                  </TD><TD id="st_e"> active          </TD><TD> Event active or inactive            </TD></TR>
//! <TR><TD id="st_t"> USHORT                 </TD><TD id="st_e"> numTriggers     </TD><TD> Number of triggers for an event     </TD></TR>
//! </TABLE>
typedef struct MlpiTaskTrigger
{
  MlpiTaskTriggerEvent triggerEvent;                      //!< Event which should be triggered
  BOOL8 active;                                           //!< Event active or inactive
  USHORT numTriggers;                                     //!< Number of triggers for an event
}MlpiTaskTrigger;

//! @} // endof: @ingroup TaskLibStructTypes

#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

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

//! @ingroup TaskLibTaskControl
//! This function pends the calling task until the given event occurs. You can use it to synchronize a task to a
//! system event. This way, you can get for example a task which is activated every time data arrives from the sercos
//! bus. Of course, your task needs to have a priority which is high enough to be activated immediately.
//! Otherwise the time between the event and this function to return is not deterministic.
//! @param[in]  connection  Handle for multiple connections.
//! @param[in]  taskEvent   The event to wait for.
//! @param[in]  timeout     The timeout after which the function should return an error if the event did not raise.
//!                         Use MLPI_INFINITE to wait forever.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref TaskLib
MLPI_API MLPIRESULT mlpiTaskWaitForEvent(const MLPIHANDLE connection, const MlpiTaskEvent taskEvent, const ULONG timeout);


//! @ingroup TaskLibTaskControl
//! This function sets the priority of tasks inside the system. For example, you can set the priority
//! of the MotionKernel(MOK) here.
//! @param[in]  connection  Handle for multiple connections.
//! @param[in]  task        Enum identifying the internal task.
//! @param[in]  priority    The desired priority of the task. This has to be between MLPI_PRIORITY_HIGH_MAX
//!                         and MLPI_PRIORITY_HIGH_MIN.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result = mlpiTaskSetSystemPriority(connection, MLPI_TASK_MOTIONKERNEL, MLPI_PRIORITY_HIGH_MAX);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskSetSystemPriority(const MLPIHANDLE connection, const MlpiSystemTask task, const ULONG priority);


//! @ingroup TaskLibTaskControl
//! This function reads back the priority of a task inside the system. For example, you can get the priority
//! of the MotionKernel(MOK) here.
//! @param[in]  connection  Handle for multiple connections.
//! @param[in]  task        Enum identifying the internal task.
//! @param[out] priority    Pointer to a variable which will receive the priority of the task.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG priority = 0;
//! MLPIRESULT result = mlpiTaskGetSystemPriority(connection, MLPI_TASK_MOTIONKERNEL, &priority);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! printf("\nPriority of MotionTask is %d", priority);
//! @endcode
MLPI_API MLPIRESULT mlpiTaskGetSystemPriority(const MLPIHANDLE connection, const MlpiSystemTask task, ULONG *priority);


//! @ingroup TaskLibTaskControl
//! This function sets the priority of the calling task. Use this to set the priority of your
//! own user C/C++ task to a higher level.
//! @param[in]  connection  Handle for multiple connections.
//! @param[in]  priority    The desired priority of the task. This has to be between MLPI_PRIORITY_HIGH_MAX
//!                         and MLPI_PRIORITY_HIGH_MIN for real-time tasks and MLPI_PRIORITY_BACKGROUND for background tasks.
//! @return                 Return value indicating success (>=0) or error (<0).
//! @par Example:
//! See @ref TaskLib
MLPI_API MLPIRESULT mlpiTaskSetCurrentPriority(const MLPIHANDLE connection, const ULONG priority);


//! @ingroup TaskLibTaskControl
//! Read back the priority of the calling task.
//! @param[in]  connection  Handle for multiple connections.
//! @param[out] priority    Pointer to a variable receiving the current task priority of the calling task.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG priority = 0;
//! MLPIRESULT result = mlpiTaskGetCurrentPriority(connection, &priority);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! printf("\nPriority of current task is %d", priority);
//! @endcode
MLPI_API MLPIRESULT mlpiTaskGetCurrentPriority(const MLPIHANDLE connection, ULONG *priority);


//! @ingroup TaskLibTaskControl
//! This function sets the configuration of the external trigger. Use this function to deactivate cyclic events and
//! configure the behaviour of the external trigger function. \n
//! Use the options 'MLPI_TASK_TRIG_OPT_NO_CYCLIC_MOTION' and 'MLPI_TASK_TRIG_OPT_NO_CYCLIC_SERCOS' to activate or deactivate cyclic execution of the events.
//! It is only possible to decouple events if all axis are in standstill.
//! Events are triggered synchronous to theirs specific cycles. To change this behaviour set the option 'MLPI_TASK_TRIG_OPT_IMMEDIATE_EXECUTION'.
//! In this case all events are executed immediately one after the other and not within their cycles. \n
//! A warning is set as long as an event is decoupled from cyclic execution. To suppress this warning set the option 'MLPI_TASK_TRIG_OPT_NO_WARNING'.
//! If an fatal error occurs all decoupled events will be reset and coupled again, so that an error reaction can be performed. To deactivate the
//! reset of events set the option 'MLPI_TASK_TRIG_OPT_NO_ERROR_REACTION'. The call of the function 'mlpiTaskSetTrigger' is asynchronous and will return
//! immediately. Set the option 'MLPI_TASK_TRIG_OPT_SYNCHRONOUS' to call the function synchronous. In this case the function will return after all triggers are done.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  triggerSetup        Pointer to array of structures to activate and deactivate options.
//! @param[in]  numElements         Number of MlpiTaskTriggerSetup elements available in 'triggerSetup' for writing.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! //decouple motion from cyclic execution and set option synchronous
//! MlpiTaskTriggerSetup triggerSetup[2];
//! triggerSetup[0].triggerOption = MLPI_TASK_TRIG_OPT_NO_CYCLIC_MOTION;
//! triggerSetup[0].active = TRUE;
//! triggerSetup[1].triggerOption = MLPI_TASK_TRIG_OPT_SYNCHRONOUS;
//! triggerSetup[1].active = TRUE;
//! MLPIRESULT result = mlpiTaskSetTriggerSetup(connection, triggerSetup, _countof(triggerSetup));
//! if (MLPI_FAILED(result))
//! {
//!   printf("\ncall of MLPI function failed with 0x%08x", (unsigned)result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskSetTriggerSetup(const MLPIHANDLE connection, const MlpiTaskTriggerSetup* triggerSetup, const ULONG numElements);


//! @ingroup TaskLibTaskControl
//! This function gets the configuration of the external trigger.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in, out] triggerSetup        Pointer to array of structures to store the actual configured options. \n
//!                                     The triggerOption is an input parameter. It defines which option is requested. \n
//!                                     Active is an output parameter. It says if the option is activated or not.
//! @param[in]      numElements         Number of MlpiTaskTriggerSetup elements available in 'triggerSetup' for reading.
//! @param[out]     numElementsRet      Number of elements used.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // get configuration of option 'MLPI_TASK_TRIG_OPT_NO_CYCLIC_MOTION'
//! ULONG numElementsRet = 0;
//! MlpiTaskTriggerSetup triggerSetup;
//! triggerSetup.triggerOption = MLPI_TASK_TRIG_OPT_NO_CYCLIC_MOTION;
//! MLPIRESULT result = mlpiTaskGetTriggerSetup(connection, &triggerSetup, 1, &numElementsRet);
//! if (MLPI_FAILED(result))
//! {
//!   printf("\ncall of MLPI function failed with 0x%08x", (unsigned)result);
//!   return result;
//! }
//!
//! printf("\nOption 'MLPI_TASK_TRIG_OPT_NO_CYCLIC_MOTION' is %s", (triggerSetup.active?"active":"not active"));
//! @endcode
MLPI_API MLPIRESULT mlpiTaskGetTriggerSetup(const MLPIHANDLE connection, MlpiTaskTriggerSetup* triggerSetup, const ULONG numElements, ULONG* numElementsRet);


//! @ingroup TaskLibTaskControl
//! This function executes the external trigger. To trigger an event it has to be decoupled from the cyclic execution.
//! Use the 'mlpiTaskSetTriggerSetup' function to decouple the events and to configure the behavior of execution.
//! It is not possible to execute a new trigger if an execution is still active. To check if an execution is active use the
//! 'mlpiTaskGetTrigger' function.
//! @param[in]  connection          Handle for multiple connections.
//! @param[in]  taskTrigger         Pointer to array of structures to activate and deactivate events. \n
//!                                 Use the parameter numTriggers to define how often an activated events should be triggered. \n
//! @param[in]  numElements         Number of MlpiTaskTrigger elements available in 'taskTrigger' for writing.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @attention According to the operation mode of the real drive connected to the axis, an movement may continue once triggered.
//!            If the drive is e.g. in the velocity control mode and the event motion is triggered, the drive keeps this velocity
//!            until interrupted by a new command. Error on real drives can not be recognized if motion isn't triggered. So no
//!            error reaction can occur.\n
//!            This function should be used for simulation and not on real machines.
//! @par Example:
//! @code
//! // execute motion event 20 times
//! MlpiTaskTrigger taskTrigger;
//! taskTrigger.triggerEvent = MLPI_TASK_TRIG_EVT_MOTION;
//! taskTrigger.active = TRUE;
//! taskTrigger.numTriggers = 20;
//! MLPIRESULT result = mlpiTaskSetTrigger(connection, &taskTrigger, 1);
//! if (MLPI_FAILED(result))
//! {
//!   printf("\ncall of MLPI function failed with 0x%08x", (unsigned)result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskSetTrigger(const MLPIHANDLE connection, const MlpiTaskTrigger* taskTrigger, const ULONG numElements);


//! @ingroup TaskLibTaskControl
//! This function gets the configuration of the events of external trigger functionality.
//! @param[in]      connection          Handle for multiple connections.
//! @param[in, out] taskTrigger         Pointer to array of structures to store configuration of the events. \n
//!                                     The triggerEvent is an input parameter. It defines which event is requested. \n
//!                                     The parameter active defines if an event is actual active
//!                                     or not and numTriggers defines how many triggers are left for this event.
//! @param[in]      numElements         Number of MlpiTaskTrigger elements available in 'taskTrigger' for reading.
//! @param[out]     cmdActive           True if an execution of an external trigger is active.
//! @param[out]     numElementsRet      Number of elements used.
//! @return                             Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // get the configuration of the event 'MLPI_TASK_TRIG_EVT_MOTION' without checking of active execution
//! ULONG numElementsRet = 0;
//! MlpiTaskTrigger taskTrigger;
//! taskTrigger.triggerEvent = MLPI_TASK_TRIG_EVT_MOTION;
//! MLPIRESULT result = mlpiTaskGetTrigger(connection, &taskTrigger, 1, NULL, &numElementsRet);
//! if (MLPI_FAILED(result))
//! {
//!   printf("\ncall of MLPI function failed with 0x%08x", (unsigned)result);
//!   return result;
//! }
//! printf("\nThe event 'MLPI_TASK_TRIG_EVT_MOTION' is %s; number of triggers: %u", (taskTrigger.active?"active":"not active"), taskTrigger.numTriggers);
//!
//! // just check if exection of external tigger is active
//! BOOL8 cmdActive = FALSE;
//! result = mlpiTaskGetTrigger(connection, NULL, 0, &cmdActive, NULL);
//! if (MLPI_FAILED(result))
//! {
//!   printf("\ncall of MLPI function failed with 0x%08x", (unsigned)result);
//!   return result;
//! }
//! printf("\nThe execution of external trigger is %s", (cmdActive?"active":"not active"));
//! @endcode
MLPI_API MLPIRESULT mlpiTaskGetTrigger(const MLPIHANDLE connection, MlpiTaskTrigger* taskTrigger, const ULONG numElements, BOOL8* cmdActive, ULONG* numElementsRet);


//! @ingroup TaskLibTaskViewer
//! This function starts a task viewer session, all task information is stored in a ring buffer as long as
//! the recording is running. You have to use IndraWorks to upload and view the session in a graphical window.
//! Please have a look within the IndraWorks help system to find more information about the task viewer.
//!
//! @param[in]  connection  Handle for multiple connections.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // start task viewer session
//! MLPIRESULT result = mlpiTaskViewerStart(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! // run task viewer for some time
//! Sleep(1000);
//!
//! // stop task viewer session
//! result = mlpiTaskViewerStop(connection);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! // now use IndraWorks for visualization of session data...
//! @endcode
MLPI_API MLPIRESULT mlpiTaskViewerStart(const MLPIHANDLE connection);


//! @ingroup TaskLibTaskViewer
//! This function stops a task viewer session.
//! @param[in]  connection  Handle for multiple connections.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref mlpiTaskViewerStart
MLPI_API MLPIRESULT mlpiTaskViewerStop(const MLPIHANDLE connection);


//! @ingroup TaskLibTaskViewer
//! This function adds a user event into the task viewer session. Use this function to mark special events
//! in the task viewer. This function is very useful for debugging timing problems.
//!
//! @param[in]  connection  Handle for multiple connections.
//! @param[in]  data        Some user data that will be shown in the task viewer together with this event.
//! @param[in]  numElements Number of bytes given by data.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example
//! @code
//! // Add item to task viewer and attach some user data. Task viewer has to be started for this
//! // example to work. After stopping, you should find the item visualized in the task viewer.
//! UCHAR data[] = {0xDE, 0xAD, 0xBE, 0xAF};
//! MLPIRESULT result = mlpiTaskViewerAddItem(connection, data, sizeof(data));
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskViewerAddItem(const MLPIHANDLE connection, const UCHAR *data, const ULONG numElements);


//! @ingroup TaskLibTaskViewer
//! This function sets a START event into a task viewer session.
//!
//! @note For a valid task viewer session, the START event and the STOP event must be set in an alternating manner.
//!
//! @param[in]  connection  Handle for multiple connections.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! Create a background task and run it endlessly. In the background task, check every 20 milliseconds the displayed diagnosis.
//! Visualize the needed runtime of the background task in the task viewer.
//! @code
//! {
//!   ..
//!   taskSpawn("BackgroundDiagnosis", 200, VX_FP_TASK, 0x200000, (FUNCPTR)&taskBackgroundDiagnosis, connection, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//! }
//! @endcode
//! @code
//! extern "C" int taskBackgroundDiagnosis(MLPIHANDLE connection)
//! {
//!   MLPIRESULT result = MLPI_S_OK;
//!   MlpiDiagnosis diagnosis;
//!   memset(&diagnosis, 0, sizeof(diagnosis));
//!
//!   do {
//!     // set STOP event into task viewer session
//!     mlpiTaskViewerTaskStop(connection);
//!
//!     // sleep
//!     taskDelay(20);
//!
//!     // set START event into task viewer session
//!     mlpiTaskViewerTaskStart(connection);
//!
//!     // Get displayed diagnosis
//!     result = mlpiSystemGetDisplayedDiagnosis(connection, &diagnosis);
//!     if(MLPI_SUCCEEDED(result)) {
//!       // do something with diagnosis
//!       ;
//!     }
//!     else {
//!       // error handling
//!       ;
//!       break;
//!     }
//!   }while(TRUE);
//!
//!   return result;
//! }
//! @endcode
//! @image html TaskViewerTaskStartStopEvent.png "Task viewer session." \n
MLPI_API MLPIRESULT mlpiTaskViewerTaskStart(const MLPIHANDLE connection);


//! @ingroup TaskLibTaskViewer
//! This function sets a STOP event into a task viewer session.
//!
//! @note For a valid task viewer session the START event and the STOP event must be set alternating.
//!
//! @param[in]  connection  Handle for multiple connections.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref mlpiTaskViewerTaskStart
MLPI_API MLPIRESULT mlpiTaskViewerTaskStop(const MLPIHANDLE connection);

//! @ingroup TaskLibTaskViewer
//! This function gets the current state if the task viewer.
//!
//! @param[in]  connection  Handle for multiple connections.
//! @param[in]  state       State of the task viewer.
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! MlpiTaskViewerState state;
//! MLPIRESULT result = mlpiTaskViewerGetState(_hControl, &state);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
MLPI_API MLPIRESULT mlpiTaskViewerGetState(const MLPIHANDLE connection, MlpiTaskViewerState* state);

//! @ingroup TaskExecution
//! This function executes a file
//! @param[in]  connection       Handle for multiple connections.
//! @param[in]  path             Path of file to be executed.
//! @param[in]  envArguments     Arguments for execution environment.
//! @param[in]  arguments        Arguments for execution.
//! @param[out] handle           Pointer to value where execution handle will be stored.
//! @return                      Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPITASKHANDLE handle;
//! // start execution of file /ata0b/test.lua without argument for Lua-VM and without arguments for file
//! result = mlpiTaskExecuteFile(connection, L"/ata0b/test.lua", L"", L"", &handle);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start execution of file with error code: %08X", result);
//!   return;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskExecuteFile(const MLPIHANDLE connection, const WCHAR16* path, const WCHAR16* envArguments, const WCHAR16* arguments, MLPITASKHANDLE* handle);

//! @ingroup TaskExecution
//! This function gets the status of an executed file
//! @param[in]  connection      Handle for multiple connections.
//! @param[in]  handle          Handle of the execution
//! @param[out] state           Status of the executed file
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPITASKHANDLE handle;
//! // start execution of file /ata0b/test.lua without argument for Lua-VM and without arguments for file
//! result = mlpiTaskExecuteFile(connection, L"/ata0b/test.lua", L"", L"", &handle);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start execution of file with error code: %08X", result);
//!   return;
//! }
//!
//! MlpiProcessState state;
//! result = mlpiTaskExecuteGetStatus(connection, handle, &state);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to get state of file execution with error code: %08X", result);
//!   return;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskExecuteGetStatus(const MLPIHANDLE connection, const MLPITASKHANDLE handle, MlpiProcessState* state);

//! @ingroup TaskExecution
//! This function gets handles for all executed files
//! @param[in]  connection      Handle for multiple connections.
//! @param[in]  handles         Pointer to an array of handles where handles of executions will be stored
//! @param[in]  numElements     Number of handle elements available in 'handles' for reading.
//! @param[out] numElementsRet  Number of elements used.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPITASKHANDLE handles[100];
//! ULONG numElementsRet = 0;
//! result = mlpiTaskExecuteGetActive(connection, handles, 100, &numElementsRet);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to get all active file executions with error code: %08X", result);
//!   return;
//! }
//! for(ULONG index = 0; index < numElementsRet; index++)
//! {
//!   printf("\nhandle of file %u is %016LX", index, handles[index]);
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskExecuteGetActive(const MLPIHANDLE connection, MLPITASKHANDLE* handles, const ULONG numElements, ULONG* numElementsRet);

//! @ingroup TaskExecution
//! This function stops execution of file
//! @param[in]  connection      Handle for multiple connections.
//! @param[in]  handle          Handle of the execution
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPITASKHANDLE handle;
//! // start execution of file /ata0b/test.lua without argument for Lua-VM and without arguments for file
//! result = mlpiTaskExecuteFile(connection, L"/ata0b/test.lua", L"", L"", &handle);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start execution of file with error code: %08X", result);
//!   return;
//! }
//!
//! result = mlpiTaskExecuteKill(connection, handle);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to kill file execution with error code: %08X", result);
//!   return;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskExecuteKill(const MLPIHANDLE connection, const MLPITASKHANDLE handle);

//! @ingroup TaskExecution
//! This function gets the name of an executed file
//! @param[in]  connection      Handle for multiple connections.
//! @param[in]  handle          Handle of the execution
//! @param[out] name            Name of the executed file
//! @param[in]  numElements     Number of handle elements available in 'name' for reading.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! MLPIRESULT result;
//! MLPITASKHANDLE handle;
//! // start execution of file /ata0b/test.lua without argument for Lua-VM and without arguments for file
//! result = mlpiTaskExecuteFile(connection, L"/ata0b/test.lua", L"", L"", &handle);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to start execution of file with error code: %08X", result);
//!   return;
//! }
//!
//! WCHAR16 name[256];
//! result = mlpiTaskExecuteGetName(connection, handle, name, 256);
//! if (MLPI_FAILED(result))
//! {
//!   printf("unable to get name of file execution with error code: %08X", result);
//!   return;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiTaskExecuteGetName(const MLPIHANDLE connection, const MLPITASKHANDLE handle, WCHAR16* name, const ULONG numElements);

#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPITASKLIB_H__
