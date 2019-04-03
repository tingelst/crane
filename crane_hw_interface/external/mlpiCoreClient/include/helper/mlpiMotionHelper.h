#ifndef __MLPIMOTIONHELPER_H__
#define __MLPIMOTIONHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiMotionHelper.h>
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



//! @addtogroup UtilMotionHelper UtilMotionHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for parameter handling.
//!
//! @details
//! Please note that this piece of source code is not directly part
//! of the MLPI. You do not need this file to program against the
//! MLPI. Nevertheless, at least parts of this file have been considered
//! to be somewhat useful when using or learning to use MLPI functionality.
//! It is therefore included without any support, but to act as sample code
//! and source of inspiration.
//! @}



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "targetinfo.h"

#if defined (TARGET_OS_VXWORKS)
#include <taskLib.h>
#elif defined (TARGET_OS_WINNT)
#include <windows.h>
#elif defined (TARGET_OS_WINCE32)
// compiling under WindowsCE 32bit environment
#elif defined (TARGET_OS_APPLE)
#include <unistd.h>
#elif defined (TARGET_OS_ANDROID)
#include <unistd.h>
#elif defined (TARGET_OS_LINUX)
#include <unistd.h>
#else
#pragma warning "unknown operating system"
#endif

#include "mlpiGlobal.h"
#include "wchar16.h"

#include "mlpiApiLib.h"
#include "mlpiMotionLib.h"

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------
#define MLPI_MOTION_BIT_ACCESS(name, bit) bool name() const {return (_value & (1<<bit)) ? true : false;}

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

//! @enum MlpiAxisCondition
//! This enumeration defines the state of an axis.
//! * When disabled, the device connected to the axis cannot physically be in the sercos ring.
//! A-parameter can not be accessed.
//! * When an axis is parked, the connected device must be in the ring, S and P parameters can
//! be read, but the axis cannot be moved.
//! * When an axis is in parametrization, then it can be parameterized even if control mode
//! and communication are in mode 'BB'.
typedef enum MlpiAxisCondition
{
  MLPI_AXIS_CONDITION_ACTIVE                                 =  0,  //!< Drive in Ring; Setpoint calculation active.
  MLPI_AXIS_CONDITION_PARKING                                =  1,  //!< Drive in Ring; Setpoint calculation not active.
  MLPI_AXIS_CONDITION_DEACTIVATED                            =  2,  //!< Drive not in Ring; Setpoint calculation not active.

  MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED                       =  4,  //!< Drive in Ring; Setpoint calculation active; No setpoint evaluation in drive.
  MLPI_AXIS_CONDITION_PARKING_DECOUPLED                      =  5,  //!< Drive in Ring; Setpoint calculation not active; No setpoint evaluation in drive.
  MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED                  =  6,  //!< Drive not in Ring; Setpoint calculation not active; No setpoint evaluation in drive.

  MLPI_AXIS_CONDITION_ACTIVE_PARAMETERIZATION                =  8,  //!< Drive in Ring; Setpoint calculation active; axis in parameterization.
  MLPI_AXIS_CONDITION_PARKING_PARAMETERIZATION               =  9,  //!< Drive in Ring; Setpoint calculation not active; axis in parameterization.

  MLPI_AXIS_CONDITION_DEACTIVATED_PARAMETERIZATION           = 10,  //!< Drive not in Ring; Setpoint calculation not active; axis in parameterization.

  MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED_PARAMETERIZATION      = 12,  //!< Drive in Ring; Setpoint calculation active; No setpoint evaluation in drive; axis in parameterization.
  MLPI_AXIS_CONDITION_PARKING_DECOUPLED_PARAMETERIZATION     = 13,  //!< Drive in Ring; Setpoint calculation not active; No setpoint evaluation in drive; axis in parameterization.
  MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED_PARAMETERIZATION = 14   //!< Drive not in Ring; Setpoint calculation not active; No setpoint evaluation in drive; axis in parameterization.
}MlpiAxisCondition;

typedef enum MlpiDirections
{
  MLPI_DIRECTION_SHORTEST_WAY  = 0,
  MLPI_DIRECTION_POSITIVE      = 1,
  MLPI_DIRECTION_NEGATIVE      = 2
}MlpiDirections;

typedef enum MlpiScalingType
{
  MLPI_SCALING_TYPE_TRANSLATORY_METER     = 0,
  MLPI_SCALING_TYPE_TRANSLATORY_INCH      = 1,
  MLPI_SCALING_TYPE_ROTATORY              = 2,
  MLPI_SCALING_TYPE_PERCENTAGE          = 3,
  MLPI_SCALING_TYPE_INVALID               = 255
}MlpiScalingType;

typedef enum MlpiScalingDataReference
{
  MLPI_SCALING_REFERENCE_MOTOR_SHAFT  = 0,
  MLPI_SCALING_REFERENCE_LOAD         = 1,
  MLPI_SCALING_REFERENCE_INVALID      = 255
}MlpiScalingDataReference;

typedef enum MlpiScalingFormat
{
  MLPI_SCALING_FORMAT_ABSOLUTE      = 0,
  MLPI_SCALING_FORMAT_MODULO        = 1,
  MLPI_SCALING_FORMAT_INVALID       = 255
}MlpiScalingFormat;

typedef enum MlpiScalingMode
{
  MLPI_SCALING_MODE_PREFERENCE      = 0,
  MLPI_SCALING_MODE_PARAMETER       = 1,
  MLPI_SCALING_MODE_INVALID         = 255
}MlpiScalingMode;

typedef enum MlpiTimeUnit
{
  MLPI_SCALING_TIME_MINUTE  = 0,
  MLPI_SCALING_TIME_SECOND  = 1,
  MLPI_SCALING_TIME_INVALID = 255
}MlpiTimeUnit;

typedef enum MlpiTorqueUnit
{
  MLPI_SCALING_TORQUE_NEWTON  = 0,
  MLPI_SCALING_TORQUE_POUND_FORCE  = 1,
  MLPI_SCALING_TORQUE_NEWTONMETER  = 2,
  MLPI_SCALING_TORQUE_INCH_POUND_FORCE  = 3,
  MLPI_SCALING_TORQUE_INVALID = 255
}MlpiTorqueUnit;



// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! The following struct helps to decode the axis state as given by mlpiMotionGetAxisState
//! or mlpiMotionGetAxisValues.
//!
//! @par Example:
//! See @ref mlpiMotionGetState
typedef struct MlpiAxisStateDecoder
{
public:
  MlpiAxisStateDecoder()
    : _value(0) {
  }

  MlpiAxisStateDecoder(const ULONG state)
    : _value(state) {
  }

  operator ULONG() const {
    return _value;
  }

  MLPI_MOTION_BIT_ACCESS(CamTab_0, 0);                //!< Bit0 of active CAM table of CamIn command.
  MLPI_MOTION_BIT_ACCESS(CamTab_1, 1);                //!< Bit1 of active CAM table of CamIn command.
  MLPI_MOTION_BIT_ACCESS(CamTab_2, 2);                //!< Bit2 of active CAM table of CamIn command.
  ULONG CamTab() const {return _value&0x7;};          //!< Returns the active CAM table of CamIn command.
  MLPI_MOTION_BIT_ACCESS(CamSwitching, 3);            //!< Cam is waiting for switching position.
  MLPI_MOTION_BIT_ACCESS(InVelocity, 4);              //!< Axis is in velocity.
  MLPI_MOTION_BIT_ACCESS(Standstill, 5);              //!< Axis is in standstill.
  MLPI_MOTION_BIT_ACCESS(InPosition, 6);              //!< Axis is in position.
  MLPI_MOTION_BIT_ACCESS(InSynchron, 7);              //!< Axis is synchronous to another axis.
  MLPI_MOTION_BIT_ACCESS(Warning, 8);                 //!< Axis has warning.
  MLPI_MOTION_BIT_ACCESS(Error, 9);                   //!< Axis has an error.
  MLPI_MOTION_BIT_ACCESS(Homed, 10);                  //!< Axis is homed.
  MLPI_MOTION_BIT_ACCESS(InTorque, 11);               //!< Axis is in torque.
  MLPI_MOTION_BIT_ACCESS(OperationMode, 12);          //!< Axis is in P4.
  MLPI_MOTION_BIT_ACCESS(Inbb, 13);                   //!< Axis is ready for power.
  MLPI_MOTION_BIT_ACCESS(InAb, 14);                   //!< Axis is ready for operation.
  MLPI_MOTION_BIT_ACCESS(Power, 15);                  //!< Axis has power.
  MLPI_MOTION_BIT_ACCESS(CmdActive, 16);              //!< Axis has a active sercos command.
  MLPI_MOTION_BIT_ACCESS(ErrInProgress, 17);          //!< Error reaction of axis is in progress.
  MLPI_MOTION_BIT_ACCESS(CyclicPosChannelActive, 18); //!< Cyclic position channel is active.
  MLPI_MOTION_BIT_ACCESS(InMasterPhaseOffset, 19);    //!< Axis has reached the master phase offset.
  MLPI_MOTION_BIT_ACCESS(CyclicTrqChannelActive, 20); //!< Cyclic torque channel is active.
  MLPI_MOTION_BIT_ACCESS(CyclicVelChannelActive, 21); //!< Cyclic velocity channel is active.
  MLPI_MOTION_BIT_ACCESS(CyclicAnaChannelActive, 22); //!< Cyclic analog channel is active.
  MLPI_MOTION_BIT_ACCESS(NRTActive, 23);              //!< NRT channel for axis is connected.
  MLPI_MOTION_BIT_ACCESS(Interrupted, 24);            //!< Axis did not follow control.
  MLPI_MOTION_BIT_ACCESS(Modulo, 28);                 //!< Axis is in Modulo format.
  MLPI_MOTION_BIT_ACCESS(Rotatory, 29);               //!< Axis is rotatory.
  MLPI_MOTION_BIT_ACCESS(Valid, 30);                  //!< Actual data of axis is valid.
  MLPI_MOTION_BIT_ACCESS(Decoupled, 31);              //!< Axis is decoupled from commanded position of control.

private:
  ULONG _value;
}MlpiAxisStateDecoder;


//! The following struct helps to decode the extended axis state as given by
//! mlpiMotionGetAxisStateExtended or mlpiMotionGetAxisValues.
//!
//! @par Example:
//! See @ref mlpiMotionGetStateExtended
typedef struct MlpiAxisStateExtendedDecoder
{
public:
  MlpiAxisStateExtendedDecoder()
    : _value(0) {
  }

  MlpiAxisStateExtendedDecoder(const ULONG state)
    : _value(state) {
  }

  operator ULONG() const {
    return _value;
  }

  MLPI_MOTION_BIT_ACCESS(ErrorStop, 0);            //!< Axis is in state ErrorStop.
  MLPI_MOTION_BIT_ACCESS(Stopping, 1);             //!< Axis is in state Stopping.
  MLPI_MOTION_BIT_ACCESS(Homing, 2);               //!< Axis is in state Homing.
  MLPI_MOTION_BIT_ACCESS(PowerOn, 3);              //!< Axis has power.
  MLPI_MOTION_BIT_ACCESS(DiscreteMotion, 4);       //!< Axis is in state DiscreteMotion.
  MLPI_MOTION_BIT_ACCESS(ContinuousMotion, 5);     //!< Axis is in state ContinousMotion.
  MLPI_MOTION_BIT_ACCESS(SynchronizedMotion, 6);   //!< Axis is in state SynchronizedMotion.
  MLPI_MOTION_BIT_ACCESS(Standstill, 7);           //!< Axis is in state Standstill.
  MLPI_MOTION_BIT_ACCESS(CoordinatedMotion, 8);    //!< Axis is in state CoordinatedMotion.

private:
  const ULONG _value;
}MlpiAxisStateExtendedDecoder;



//! The following struct helps to decode the axis type as given by
//! mlpiMotionGetAxisType
//!
//! @par Example:
//! See @ref mlpiMotionGetAxisType
typedef struct MlpiAxisTypeDecoder
{
public:
  MlpiAxisTypeDecoder()
    : _value(0) {
  }

  MlpiAxisTypeDecoder(const USHORT type)
    : _value(type) {
  }

  operator USHORT() const {
    return _value;
  }

  MLPI_MOTION_BIT_ACCESS(InterpolationInControl, 0);    //!< Axis is interpolated in control.
  MLPI_MOTION_BIT_ACCESS(IsPackProfile, 1);             //!< Attached drive is configured in sercos PackProfile mode.
  MLPI_MOTION_BIT_ACCESS(IsHydraulicDrive, 2);          //!< Attached drive is a hydraulic drive.
  MLPI_MOTION_BIT_ACCESS(SupportsParametrization, 15);  //!< Axis supports Parametrization mode.
  MlpiAxisType Type() const {
    switch (_value & 0xF8) {
      case (1<<3): return MLPI_AXISTYPE_REAL;
      case (1<<4): return MLPI_AXISTYPE_VIRTUAL;
      case (1<<5): return MLPI_AXISTYPE_ENCODER;
      case (1<<6): return MLPI_AXISTYPE_LINK;
      case (1<<7): return MLPI_AXISTYPE_CONTROLLER;
    }
    return MLPI_AXISTYPE_VIRTUAL;
  }
private:
  USHORT _value;
}MlpiAxisTypeDecoder;

//! @typedef MlpiAxisConditionDecoder
//! The following struct helps to decode the axis condition as given by @ref mlpiMotionGetCondition
//! @par Example:
//! See @ref mlpiMotionGetCondition
typedef struct MlpiAxisConditionDecoder
{
public:
  MlpiAxisConditionDecoder()
    : _value(0) {
  }

  MlpiAxisConditionDecoder(const ULONG condition)
    : _value(condition) {
  }

  operator ULONG() const {
    return _value;
  }

  MlpiAxisCondition Condition() const {
    return static_cast<MlpiAxisCondition>(_value);
  }

  bool IsActive() const {
    switch (_value) {
    case MLPI_AXIS_CONDITION_ACTIVE:
    case MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED:
    case MLPI_AXIS_CONDITION_ACTIVE_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED_PARAMETERIZATION:
      return true;
    }
    return false;
  }

  bool IsParking() const {
    switch (_value) {
    case MLPI_AXIS_CONDITION_PARKING:
    case MLPI_AXIS_CONDITION_PARKING_DECOUPLED:
    case MLPI_AXIS_CONDITION_PARKING_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_PARKING_DECOUPLED_PARAMETERIZATION:
      return true;
    }
    return false;
  }

  bool IsDeactivated() const {
    switch (_value) {
    case MLPI_AXIS_CONDITION_DEACTIVATED:
    case MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED:
    case MLPI_AXIS_CONDITION_DEACTIVATED_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED_PARAMETERIZATION:
      return true;
    }
    return false;
  }

  bool IsDecoupled() const {
    switch (_value) {
    case MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED:
    case MLPI_AXIS_CONDITION_PARKING_DECOUPLED:
    case MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED:
    case MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_PARKING_DECOUPLED_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED_PARAMETERIZATION:
      return true;
    }
    return false;
  }

  bool IsParametrization() const {
    switch (_value) {
    case MLPI_AXIS_CONDITION_ACTIVE_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_PARKING_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_DEACTIVATED_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_ACTIVE_DECOUPLED_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_PARKING_DECOUPLED_PARAMETERIZATION:
    case MLPI_AXIS_CONDITION_DEACTIVATED_DECOUPLED_PARAMETERIZATION:
      return true;
    }
    return false;
  }

private:
  ULONG _value;
}MlpiAxisConditionDecoder;


//! The following struct helps to encode or decode the position scaling
//! settings of the axis given or set by mlpiMotionGetPositionScaling or
//! mlpiMotionSetPositionScaling
//!
//! @par Example:
//! See @ref mlpiMotionGetPositionScaling @ref mlpiMotionSetPositionScaling
typedef struct MlpiAxisScalingPosition
{
public:
  MlpiAxisScalingPosition()
    : _value(0) {
  }

  MlpiAxisScalingPosition(const USHORT scaling)
    : _value(scaling) {
  }

  MlpiAxisScalingPosition(const MlpiScalingType type, const MlpiScalingFormat format,
    const MlpiScalingDataReference dataReference = MLPI_SCALING_REFERENCE_LOAD,
    const MlpiScalingMode mode = MLPI_SCALING_MODE_PREFERENCE) {
      _value = 0;
      switch (type)
      {
      case MLPI_SCALING_TYPE_TRANSLATORY_METER:
        _value |= 0x01;
        break;
      case MLPI_SCALING_TYPE_TRANSLATORY_INCH:
        _value |= 0x11;
        break;
      case MLPI_SCALING_TYPE_ROTATORY:
        _value |= 0x02;
        break;
      default:
        break;
      }
      switch (format)
      {
      case MLPI_SCALING_FORMAT_ABSOLUTE:
        _value |= 0x00;
        break;
      case MLPI_SCALING_FORMAT_MODULO:
        _value |= 0x80;
        break;
      default:
        break;
      }

      switch (dataReference)
      {
      case MLPI_SCALING_REFERENCE_MOTOR_SHAFT:
        break;
      case MLPI_SCALING_REFERENCE_LOAD:
        _value |= 0x40;
        break;
      default:
        break;
      }

      switch (mode)
      {
      case MLPI_SCALING_MODE_PREFERENCE:
        break;
      case MLPI_SCALING_MODE_PARAMETER:
        _value |= 0x08;
        break;
      default:
        break;
      }
  }

  MlpiScalingType getType() const
  {
    switch (_value & 0x13)
    {
    case 0x01:
      return MLPI_SCALING_TYPE_TRANSLATORY_METER;
    case 0x11:
      return MLPI_SCALING_TYPE_TRANSLATORY_INCH;
    case 0x02:
      return MLPI_SCALING_TYPE_ROTATORY;
    default:
      return MLPI_SCALING_TYPE_INVALID;
    }
  }

  const WCHAR16* getTypeString() const
  {
    switch (_value & 0x13)
    {
    case 0x01 :
      return L"translatory in mm";
    case 0x11:
      return L"translatory in inch";
    case 0x02:
      return L"rotatory";
    default:
      return L"";
    }
  }

  MlpiScalingFormat getFormat() const
  {
    switch (_value & 0x80)
    {
    case 0x00 :
      return MLPI_SCALING_FORMAT_ABSOLUTE;
    case 0x80:
      return MLPI_SCALING_FORMAT_MODULO;
    default:
      return MLPI_SCALING_FORMAT_INVALID;
    }
  }

  const WCHAR16* getFormatString() const
  {
    switch (_value & 0x80)
    {
    case 0x00 :
      return L"absolute";
    case 0x80:
      return L"modulo";
    default:
      return L"";
    }
  }

  MlpiScalingDataReference getReference() const
  {
    switch (_value & 0x40)
    {
    case 0x00 :
      return MLPI_SCALING_REFERENCE_MOTOR_SHAFT;
    case 0x40:
      return MLPI_SCALING_REFERENCE_LOAD;
    default:
      return MLPI_SCALING_REFERENCE_INVALID;
    }
  }

  const WCHAR16* getReferenceString() const
  {
    switch (_value & 0x40)
    {
    case 0x00 :
      return L"with motor shaft reference";
    case 0x40:
      return L"with load reference";
    default:
      return L"";
    }
  }

  MlpiScalingMode getMode() const
  {
    switch (_value & 0x08)
    {
    case 0x00 :
      return MLPI_SCALING_MODE_PREFERENCE;
    case 0x08:
      return MLPI_SCALING_MODE_PARAMETER;
    default:
      return MLPI_SCALING_MODE_INVALID;
    }
  }

  const WCHAR16* getModeString() const
  {
    switch (_value & 0x08)
    {
    case 0x00 :
      return L"in preference mode";
    case 0x08:
      return L"parameter mode";
    default:
      return L"";
    }
  }

  operator USHORT() const {
    return _value;
  }

  operator USHORT*() {
    return &_value;
  }

private:
  USHORT _value;
}MlpiAxisScalingPosition;



//! The following struct helps to encode or decode the velocity scaling
//! settings of the axis given or set by mlpiMotionGetVelocityScaling or
//! mlpiMotionSetVelocityScaling
//!
//! @par Example:
//! See @ref mlpiMotionGetVelocityScaling @ref mlpiMotionSetVelocityScaling
typedef struct MlpiAxisScalingVelocity
{
public:
  MlpiAxisScalingVelocity()
    : _value(0) {
  }

  MlpiAxisScalingVelocity(const USHORT scaling)
    : _value(scaling) {
  }

  MlpiAxisScalingVelocity(const MlpiScalingType type, const MlpiTimeUnit time,
    const MlpiScalingDataReference dataReference = MLPI_SCALING_REFERENCE_LOAD, const MlpiScalingMode mode = MLPI_SCALING_MODE_PREFERENCE) {
      _value = 0;
      switch (type)
      {
      case MLPI_SCALING_TYPE_TRANSLATORY_METER:
        _value |= 0x01;
        break;
      case MLPI_SCALING_TYPE_TRANSLATORY_INCH:
        _value |= 0x11;
        break;
      case MLPI_SCALING_TYPE_ROTATORY:
        _value |= 0x02;
        break;
      default:
        break;
      }
      switch (time)
      {
      case MLPI_SCALING_TIME_MINUTE:
        _value |= 0x00;
        break;
      case MLPI_SCALING_TIME_SECOND:
        _value |= 0x20;
        break;
      default:
        break;
      }

      switch (dataReference)
      {
      case MLPI_SCALING_REFERENCE_MOTOR_SHAFT:
        break;
      case MLPI_SCALING_REFERENCE_LOAD:
        _value |= 0x40;
        break;
      default:
        break;
      }

      switch (mode)
      {
      case MLPI_SCALING_MODE_PREFERENCE:
        break;
      case MLPI_SCALING_MODE_PARAMETER:
        _value |= 0x08;
        break;
      default:
        break;
      }
  }

  MlpiScalingType getType() const
  {
    switch (_value & 0x13)
    {
    case 0x01 :
      return MLPI_SCALING_TYPE_TRANSLATORY_METER;
    case 0x11:
      return MLPI_SCALING_TYPE_TRANSLATORY_INCH;
    case 0x02:
      return MLPI_SCALING_TYPE_ROTATORY;
    default:
      return MLPI_SCALING_TYPE_INVALID;
    }
  }

  const WCHAR16* getTypeString() const
  {
    switch (_value & 0x13)
    {
    case 0x01 :
      return L"translatory in mm";
    case 0x11:
      return L"translatory in inch";
    case 0x02:
      return L"rotatory";
    default:
      return L"";
    }
  }

  MlpiTimeUnit getTime() const
  {
    switch (_value & 0x20)
    {
    case 0x00 :
      return MLPI_SCALING_TIME_MINUTE;
    case 0x20:
      return MLPI_SCALING_TIME_SECOND;
    default:
      return MLPI_SCALING_TIME_INVALID;
    }
  }

  const WCHAR16* getTimeString() const
  {
    switch (_value & 0x20)
    {
    case 0x00 :
      return L"in minute";
    case 0x20:
      return L"in second";
    default:
      return L"";
    }
  }

  MlpiScalingDataReference getReference() const
  {
    switch (_value & 0x40)
    {
    case 0x00 :
      return MLPI_SCALING_REFERENCE_MOTOR_SHAFT;
    case 0x40:
      return MLPI_SCALING_REFERENCE_LOAD;
    default:
      return MLPI_SCALING_REFERENCE_INVALID;
    }
  }

  const WCHAR16* getReferenceString() const
  {
    switch (_value & 0x40)
    {
    case 0x00 :
      return L"with motor shaft reference";
    case 0x40:
      return L"with load reference";
    default:
      return L"";
    }
  }

  MlpiScalingMode getMode() const
  {
    switch (_value & 0x08)
    {
    case 0x00 :
      return MLPI_SCALING_MODE_PREFERENCE;
    case 0x08:
      return MLPI_SCALING_MODE_PARAMETER;
    default:
      return MLPI_SCALING_MODE_INVALID;
    }
  }

  const WCHAR16* getModeString() const
  {
    switch (_value & 0x08)
    {
    case 0x00 :
      return L"in preference mode";
    case 0x08:
      return L"parameter mode";
    default:
      return L"";
    }
  }

  operator USHORT() const {
    return _value;
  }

  operator USHORT*() {
    return &_value;
  }

private:
  USHORT _value;
}MlpiAxisScalingVelocity;



//! The following struct helps to encode or decode the velocity scaling
//! settings of the axis given or set by mlpiMotionGetAccelerationScaling or
//! mlpiMotionSetAccelerationScaling
//!
//! @par Example:
//! See @ref mlpiMotionGetAccelerationScaling @ref mlpiMotionSetAccelerationScaling
typedef struct MlpiAxisScalingAcceleration
{
public:
  MlpiAxisScalingAcceleration()
    : _value(0) {
  }

  MlpiAxisScalingAcceleration(const USHORT scaling)
    : _value(scaling) {
  }

  MlpiAxisScalingAcceleration(const MlpiScalingType type,
    const MlpiScalingDataReference dataReference = MLPI_SCALING_REFERENCE_LOAD, const MlpiScalingMode mode = MLPI_SCALING_MODE_PREFERENCE) {
      _value = 0;
      switch (type)
      {
      case MLPI_SCALING_TYPE_TRANSLATORY_METER:
        _value |= 0x01;
        break;
      case MLPI_SCALING_TYPE_TRANSLATORY_INCH:
        _value |= 0x11;
        break;
      case MLPI_SCALING_TYPE_ROTATORY:
        _value |= 0x02;
        break;
      default:
        break;
      }

      switch (dataReference)
      {
      case MLPI_SCALING_REFERENCE_MOTOR_SHAFT:
        break;
      case MLPI_SCALING_REFERENCE_LOAD:
        _value |= 0x40;
        break;
      default:
        break;
      }

      switch (mode)
      {
      case MLPI_SCALING_MODE_PREFERENCE:
        break;
      case MLPI_SCALING_MODE_PARAMETER:
        _value |= 0x08;
        break;
      default:
        break;
      }
  }

  MlpiScalingType getType() const
  {
    switch (_value & 0x13)
    {
    case 0x01:
      return MLPI_SCALING_TYPE_TRANSLATORY_METER;
    case 0x11:
      return MLPI_SCALING_TYPE_TRANSLATORY_INCH;
    case 0x02:
      return MLPI_SCALING_TYPE_ROTATORY;
    default:
      return MLPI_SCALING_TYPE_INVALID;
    }
  }

  const WCHAR16* getTypeString() const
  {
    switch (_value & 0x13)
    {
    case 0x01:
      return L"translatory axis in mm";
    case 0x11:
      return L"translatory in inch";
    case 0x02:
      return L"rotatory";
    default:
      return L"";
    }
  }

  MlpiScalingDataReference getReference() const
  {
    switch (_value & 0x40)
    {
    case 0x00:
      return MLPI_SCALING_REFERENCE_MOTOR_SHAFT;
    case 0x40:
      return MLPI_SCALING_REFERENCE_LOAD;
    default:
      return MLPI_SCALING_REFERENCE_INVALID;
    }
  }

  const WCHAR16* getReferenceString() const
  {
    switch (_value & 0x40)
    {
    case 0x00:
      return L"with motor shaft reference";
    case 0x40:
      return L"with load reference";
    default:
      return L"";
    }
  }

  MlpiScalingMode getMode() const
  {
    switch (_value & 0x08)
    {
    case 0x00:
      return MLPI_SCALING_MODE_PREFERENCE;
    case 0x08:
      return MLPI_SCALING_MODE_PARAMETER;
    default:
      return MLPI_SCALING_MODE_INVALID;
    }
  }

  const WCHAR16* getModeString() const
  {
    switch (_value & 0x08)
    {
    case 0x00 :
      return L"in preference mode";
    case 0x08:
      return L"parameter mode";
    default:
      return L"";
    }
  }

  operator USHORT() const {
    return _value;
  }

  operator USHORT*() {
    return &_value;
  }

private:
  USHORT _value;
}MlpiAxisScalingAcceleration;



//! The following struct helps to encode or decode the torque scaling
//! settings of the axis given or set by mlpiMotionGetTorqueScaling or
//! mlpiMotionSetTorqueScaling
//!
//! @par Example:
//! See @ref mlpiMotionGetTorqueScaling @ref mlpiMotionSetTorqueScaling
typedef struct MlpiAxisScalingTorque
{
public:
  MlpiAxisScalingTorque()
    : _value(0) {
  }

  MlpiAxisScalingTorque(const USHORT scaling)
    : _value(scaling) {
  }

  MlpiAxisScalingTorque(const MlpiScalingType type,
    const MlpiScalingDataReference dataReference = MLPI_SCALING_REFERENCE_LOAD, const MlpiScalingMode mode = MLPI_SCALING_MODE_PREFERENCE) {
      _value = 0;
      switch (type)
      {
      case MLPI_SCALING_TYPE_PERCENTAGE:
        _value |= 0x00;
        break;
      case MLPI_SCALING_TYPE_TRANSLATORY_METER:
        _value |= 0x01;
        break;
      case MLPI_SCALING_TYPE_TRANSLATORY_INCH:
        _value |= 0x11;
        break;
      case MLPI_SCALING_TYPE_ROTATORY:
        _value |= 0x02;
        break;
      default:
        break;
      }

      switch (dataReference)
      {
      case MLPI_SCALING_REFERENCE_MOTOR_SHAFT:
        break;
      case MLPI_SCALING_REFERENCE_LOAD:
        _value |= 0x40;
        break;
      default:
        break;
      }

      switch (mode)
      {
      case MLPI_SCALING_MODE_PREFERENCE:
        break;
      case MLPI_SCALING_MODE_PARAMETER:
        _value |= 0x08;
        break;
      default:
        break;
      }
  }

  MlpiScalingType getType() const
  {
    switch (_value & 0x13)
    {
    case 0x00:
      return MLPI_SCALING_TYPE_PERCENTAGE;
    case 0x01:
      return MLPI_SCALING_TYPE_TRANSLATORY_METER;
    case 0x11:
      return MLPI_SCALING_TYPE_TRANSLATORY_INCH;
    case 0x02:
      return MLPI_SCALING_TYPE_ROTATORY;
    default:
      return MLPI_SCALING_TYPE_INVALID;
    }
  }

  const WCHAR16* getTypeString() const
  {
    switch (_value & 0x13)
    {
    case 0x00:
      return L"percentage";
    case 0x01:
      return L"newton";
    case 0x11:
      return L"pound-force";
    case 0x02:
      return L"newton meter";
    case 0x12:
      return L"inch pound-force";
    default:
      return L"";
    }
  }

  MlpiScalingDataReference getReference() const
  {
    switch (_value & 0x40)
    {
    case 0x00:
      return MLPI_SCALING_REFERENCE_MOTOR_SHAFT;
    case 0x40:
      return MLPI_SCALING_REFERENCE_LOAD;
    default:
      return MLPI_SCALING_REFERENCE_INVALID;
    }
  }

  const WCHAR16* getReferenceString() const
  {
    switch (_value & 0x40)
    {
    case 0x00:
      return L"with motor shaft reference";
    case 0x40:
      return L"with load reference";
    default:
      return L"";
    }
  }

  MlpiScalingMode getMode() const
  {
    switch (_value & 0x08)
    {
    case 0x00:
      return MLPI_SCALING_MODE_PREFERENCE;
    case 0x08:
      return MLPI_SCALING_MODE_PARAMETER;
    default:
      return MLPI_SCALING_MODE_INVALID;
    }
  }

  const WCHAR16* getModeString() const
  {
    switch (_value & 0x08)
    {
    case 0x00 :
      return L"in preference mode";
    case 0x08:
      return L"parameter mode";
    default:
      return L"";
    }
  }

  operator USHORT() const {
    return _value;
  }

  operator USHORT*() {
    return &_value;
  }

private:
  USHORT _value;
}MlpiAxisScalingTorque;


// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------


//! @ingroup UtilMotionHelper
//! This function waits until a given motion command is done or an error occurs. An optional timeout can be given to
//! the function.
//! @param[in] connection   Handle for multiple connections.
//! @param[in] axis         The commanded axis.
//! @param[in] motionHandle The handle identifying the motion command to wait for. This handle is returned by the
//!                         motion command.
//! @param[in] timeout      Timeout in milliseconds for motion command to finish (optional).
//! @return                 Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref MotionLibMovement
inline MLPIRESULT utilMotionWait(MLPIHANDLE connection, MlpiAxisRef axis, MLPIMOTIONHANDLE motionHandle, ULONG timeout=MLPI_INFINITE)
{
  const ULONG REFRESH_TIME = 100;
  MlpiMotionStatus status;
  memset(&status, 0, sizeof(MlpiMotionStatus));

  ULONG timeGone = 0;

  MLPIRESULT result = MLPI_S_OK;

  if(0 == motionHandle)
    return result;

  // wait for complete
  result = mlpiMotionGetStatus(connection, axis, motionHandle, &status);
  while( !(status.done || status.error) && (timeout==MLPI_INFINITE || timeGone<timeout) )
  {
    #if defined (TARGET_OS_VXWORKS)
    taskDelay(REFRESH_TIME);
    #elif defined (TARGET_OS_WINNT)
    Sleep(REFRESH_TIME);
    #elif defined (TARGET_OS_WINCE32)
    Sleep(REFRESH_TIME);
    #elif defined (TARGET_OS_APPLE)
    usleep(REFRESH_TIME*1000);
    #elif defined (TARGET_OS_ANDROID)
    usleep(REFRESH_TIME*1000);
    #elif defined (TARGET_OS_LINUX)
    usleep(REFRESH_TIME*1000);
    #else
    #pragma warning "unknown operating system"
    #endif

    result = mlpiMotionGetStatus(connection, axis, motionHandle, &status);
    timeGone += REFRESH_TIME;
  }

  if(timeGone >= timeout)
    return MLPI_E_TIMEOUT;

  if (status.error)
    return status.additional1;

  return result;
}


//! @ingroup UtilMotionHelper
//! This function waits that the axis is no longer interrupted.
//! @param[in] connection   Handle for multiple connections.
//! @param[in] axis         The commanded axis.
//! @param[in] timeout      Timeout in milliseconds for motion command interrupted (optional)
//! @return                 Return value indicating success (>=0) or error (<0).
inline MLPIRESULT utilMotionWaitAxisInterrupted(MLPIHANDLE connection, MlpiAxisRef axis, const ULONG timeout=MLPI_INFINITE)
{
  const ULONG REFRESH_TIME = 100;
  MLPIRESULT result = MLPI_S_OK;
  ULONG timeGone = 0;

  // wait for AxisInterrupted to be gone
  ULONG axisState = 0;
  result = mlpiMotionGetState(connection, axis, &axisState);
  MlpiAxisStateDecoder state = axisState;
  while(MLPI_SUCCEEDED(result) && state.Interrupted() && (timeout==MLPI_INFINITE || timeGone<timeout))
  {
    #if defined (TARGET_OS_VXWORKS)
    taskDelay(REFRESH_TIME);
    #elif defined (TARGET_OS_WINNT)
    Sleep(REFRESH_TIME);
    #elif defined (TARGET_OS_WINCE32)
    Sleep(REFRESH_TIME);
    #elif defined (TARGET_OS_APPLE)
    usleep(REFRESH_TIME*1000);
    #elif defined (TARGET_OS_ANDROID)
    usleep(REFRESH_TIME*1000);
    #elif defined (TARGET_OS_LINUX)
    usleep(REFRESH_TIME*1000);
    #else
    #pragma warning "unknown operating system"
    #endif

    result = mlpiMotionGetState(connection, axis, &axisState);
    state = MlpiAxisStateDecoder(axisState);
    timeGone += REFRESH_TIME;
  }

  if(timeGone >= timeout)
    return MLPI_E_TIMEOUT;

  return result;
}


#endif /* __MLPIMOTIONHELPER_H__ */
