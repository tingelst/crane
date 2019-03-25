#ifndef __TIMER_H__
#define __TIMER_H__

// -----------------------------------------------------------------------
// MLPI - <timer.h>
// -----------------------------------------------------------------------
// Copyright (c) 2012 Bosch Rexroth. All rights reserved.
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
//! @date       2012
//
// -----------------------------------------------------------------------


//! @addtogroup UtilTimer UtilTimer
//! @{
//! @brief This module contains a Helper class for performing timing measurements.
//!
//! @details
//! Please note, that this piece of source code is not directly part
//! of the MLPI. You do not need this file to program against the
//! MLPI. Nevertheless, at least parts of this file have been considered
//! to be somewhat useful when using or learning to use MLPI functionality.
//! It is therefore included without any support, but to act as sample code
//! and source of inspiration.
//!
//! @par Example:
//! @code
//! CPerformanceTimer timer;
//! timer.Start();
//! //insert your code to measure here
//! timer.Stop();
//! printf(L"elapsed time %lf, elapsed time in ms %lf, elapsed time in us %lf",
//!   timer.Elapsed(), timer.ElapsedMilli(), timer.ElapsedMicro());
//! @endcode
//! @}


#ifndef __cplusplus
  #error MLPI requires C++ compilation (use a .cpp suffix)
#endif

// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#if defined(TARGET_OS_VXWORKS_KERNEL)
  #if defined(TARGET_CPU_X86)
    #include "vxWorksCommon.h"
    #include "version.h"
    #include "arch/i86/pentiumLib.h"                                  // tick
    #if((_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR >= 9))
      #include "hwif/cpu/arch/i86/vxCpuIdLib.h"                       // frequence
    #else
      #include "sysLib.h"                                             // frequence
    #endif
  #else
    #include "sysLib.h"                                               // tick + frequence
    #include "string.h"
    #include "time.h"                                                 // clock
  #endif
#elif defined(TARGET_OS_VXWORKS_RTP)
  #include "string.h"
  #include "time.h"
#elif defined(TARGET_OS_WINNT)
  #include <windows.h>
#elif defined(TARGET_OS_WINCE32)
  #include <windows.h>
#elif defined(TARGET_OS_LINUX)
  #include "time.h"
#elif defined(TARGET_OS_ANDROID)
  #include "time.h"
#elif defined(TARGET_OS_APPLE)
  #include "time.h"
  #include <mach/clock.h>
  #include <mach/mach.h>
#else
  #pragma warning "unknown operating system"
#endif

#include "mlpiGlobal.h"

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------

//! @ingroup UtilTimer
//! This class defines a time measurement facility
class CPerformanceTimer
{
public:
  //
  // Constructor/Destructor
  //
  ~CPerformanceTimer() {;}

  CPerformanceTimer(BOOL8 start = FALSE, const LLONG frequency = 0LL)
    : _running(FALSE) {
    Init(start, frequency);
  }

  CPerformanceTimer(const LLONG frequency)
    : _running(FALSE) {
    Init(FALSE, frequency);
  }

  inline CPerformanceTimer(const CPerformanceTimer& src) {
    if (this==&src)
      return;

    // make a deep copy
    _nanoseconds = src._nanoseconds;
    _startTime = src._startTime;
    _startTicks = src._startTicks;
    _adjustTime = src._adjustTime;
    _frequency = src._frequency;
    _running = src._running;
  }

  //
  // Methods
  //

  //! This function starts a measurement
  // @param[in] bReset Should start a new measurement or start with an accumulate time (default TRUE)
  inline void Start(const BOOL8 reset = TRUE) {
    // set status
    _running = TRUE;

    // ignore old results?
    if(reset == TRUE)
      _nanoseconds = 0.0;

    // get current tick stamp
    #if defined(TARGET_OS_WINNT)
      // high resolution (ns)
      QueryPerformanceCounter((LARGE_INTEGER *) &_startTicks);
    #elif defined(TARGET_OS_VXWORKS_KERNEL)
      #if defined(TARGET_CPU_X86)
        // high resolution (ns)
        pentiumTscGet64(&_startTicks);
      #else
        // low resolution support support (ms)
        timespec t;
        memset(&t, 0, sizeof(t));
        clock_gettime(CLOCK_REALTIME, &t);
        _startTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);

        // high resolution (ns), but only 32bit
        _startTicks = (LLONG) sysTimestamp();
      #endif
    #elif defined(TARGET_OS_VXWORKS_RTP)
      timespec t;
      memset(&t, 0, sizeof(t));
      clock_gettime(CLOCK_MONOTONIC, &t);
      _startTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);
      _startTicks = 0LL;
    #elif defined(TARGET_OS_LINUX)
      timespec t;
      memset(&t, 0, sizeof(t));
      clock_gettime(CLOCK_MONOTONIC, &t);
      _startTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);
      _startTicks = 0LL;
    #elif defined(TARGET_OS_ANDROID)
      timespec t;
      memset(&t, 0, sizeof(t));
      clock_gettime(CLOCK_MONOTONIC, &t);
      _startTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);
      _startTicks = 0LL;
    #elif defined(TARGET_OS_APPLE)
      clock_serv_t cc;
      mach_timespec_t t;
      memset(&cc, 0, sizeof(cc));
      memset(&t, 0, sizeof(t));
      host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cc);
      clock_get_time(cc, &t);
      mach_port_deallocate(mach_task_self(), cc);
      _startTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);
      _startTicks = 0LL;
    #else
      _startTime = 0LL;
      _startTicks = 0LL;
    #endif
  }

  //! This function stops a measurement
  inline void Stop() {
    DOUBLE runTime = 0.0;

    // already stopped and/or not running?
    if (_running == FALSE) {
      return;
    }

    // set status
    _running = FALSE;

    #if defined(TARGET_OS_WINNT)
      // high resolution (ns)
      LLONG stopTicks = 0LL;
      QueryPerformanceCounter((LARGE_INTEGER *) &stopTicks);
      runTime = (static_cast<DOUBLE>(stopTicks - _startTicks)) * (1000000000.0 / _frequency);
    #elif defined(TARGET_OS_VXWORKS_KERNEL)
      #if defined(TARGET_CPU_X86)
        // high resolution (ns)
        LLONG stopTicks = 0;
        pentiumTscGet64(&stopTicks);
        runTime = (static_cast<DOUBLE>(stopTicks - _startTicks)) * (1000000000.0 / _frequency);
      #else
        // high resolution (ns), but only 32bit
        LONG stopTicks = sysTimestamp();

        // low resolution support (ms) with 64bit
        timespec t;
        memset(&t, 0, sizeof(t));
        clock_gettime(CLOCK_REALTIME, &t);
        LLONG stopTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);

        // decision between high resolution vs. long term measurement
        if( ((static_cast<DOUBLE>(stopTime - _startTime)) / (1000000000.0)) < (2147483647.0) ) {
          // high resolution
          LONG runTicks = stopTicks - ((LONG) _startTicks);
          runTime = (static_cast<DOUBLE>(runTicks)) * (1000000000.0);
        }
        else {
          // long term measurement
          runTime = static_cast<DOUBLE>(stopTime - _startTime);
        }
      #endif
    #elif defined(TARGET_OS_VXWORKS_RTP)
      timespec t;
      memset(&t, 0, sizeof(t));
      clock_gettime(CLOCK_MONOTONIC, &t);
      LLONG stopTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);

      runTime = static_cast<DOUBLE>(stopTime - _startTime);
    #elif defined(TARGET_OS_LINUX)
      timespec t;
      memset(&t, 0, sizeof(t));
      clock_gettime(CLOCK_MONOTONIC, &t);
      LLONG stopTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);

      runTime = static_cast<DOUBLE>(stopTime - _startTime);
    #elif defined(TARGET_OS_ANDROID)
      timespec t;
      memset(&t, 0, sizeof(t));
      clock_gettime(CLOCK_MONOTONIC, &t);
      LLONG stopTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);

      runTime = static_cast<DOUBLE>(stopTime - _startTime);
    #elif defined(TARGET_OS_APPLE)
      clock_serv_t cc;
      mach_timespec_t t;
      memset(&cc, 0, sizeof(cc));
      memset(&t, 0, sizeof(t));
      host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cc);
      clock_get_time(cc, &t);
      mach_port_deallocate(mach_task_self(), cc);
      LLONG stopTime = (((LLONG) t.tv_sec) * 1000000000LL) + ((LLONG) t.tv_nsec);

      runTime = static_cast<DOUBLE>(stopTime - _startTime);
    #else
      runTime = 0.0;
    #endif

    _nanoseconds += ((runTime > _adjustTime) ? (runTime - _adjustTime) : 0.0);
  }

  //! This function returns whether a measurement is running or not
  //! @return True if a measurement is running
  inline const BOOL8 IsRunning() const {
    return _running;
  }

  //! This function returns the elapsed time in seconds
  //! @return elapsed time in seconds
  inline DOUBLE Elapsed() const {
    CPerformanceTimer temp(*this);
    temp.Stop();
    return temp._nanoseconds / 1000000000.0;
  }

  //! This function returns the elapsed time in milliseconds
  //! @return elapsed time in milliseconds
  inline DOUBLE ElapsedMilli() const {
    CPerformanceTimer temp(*this);
    temp.Stop();
    return temp._nanoseconds / 1000000.0;
  }

  //! This function returns the elapsed time in microseconds
  //! @return elapsed time in microseconds
  inline DOUBLE ElapsedMicro() const {
    CPerformanceTimer temp(*this);
    temp.Stop();
    return temp._nanoseconds / 1000.0;
  }

  //! This function returns the elapsed time in microseconds
  //! @return elapsed time in nanoseconds
  inline DOUBLE ElapsedNano() const {
    CPerformanceTimer temp(*this);
    temp.Stop();
    return temp._nanoseconds;
  }

  inline void SetFrequency(LLONG frequency) {
    if (frequency==0LL) {
      #if defined(TARGET_OS_WINNT)
        QueryPerformanceFrequency((LARGE_INTEGER *) &frequency);
      #elif defined(TARGET_OS_VXWORKS_KERNEL)
        #if(defined(TARGET_CPU_X86)) && ((_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR >= 9))
          frequency = (LLONG) vxCpuIdGetFreq();
        #else
          frequency = (LLONG) sysTimestampFreq();
        #endif
      #elif defined(TARGET_OS_VXWORKS_RTP)
        frequency = 1LL;
      #elif defined(TARGET_OS_LINUX)
        frequency = 1LL;
      #elif defined(TARGET_OS_ANDROID)
        frequency = 1LL;
      #elif defined(TARGET_OS_APPLE)
        frequency = 1LL;
      #else
        frequency = 1LL;
      #endif
    }

    _frequency = static_cast<DOUBLE>(frequency);
  }

private:
  inline void Init(const BOOL8 start = FALSE, LLONG frequency = 0LL) {
    // init all variables

    if (frequency==0LL) {
      #if defined(TARGET_OS_WINNT)
        QueryPerformanceFrequency((LARGE_INTEGER *) &frequency);
      #elif defined (TARGET_OS_VXWORKS_KERNEL)
        #if(defined(TARGET_CPU_X86)) && ((_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR >= 9))
          frequency = (LLONG) vxCpuIdGetFreq();
        #else
          frequency = (LLONG) sysTimestampFreq();
        #endif
      #elif defined (TARGET_OS_VXWORKS_RTP)
        frequency = 1LL;
      #elif defined(TARGET_OS_LINUX)
        frequency = 1LL;
      #elif defined(TARGET_OS_ANDROID)
        frequency = 1LL;
      #elif defined(TARGET_OS_APPLE)
        frequency = 1LL;
      #else
        frequency = 1LL;
      #endif
    }

    _frequency = static_cast<DOUBLE>(frequency);

    // calculate adjustment
    _adjustTime = 0.0;
    Start();
    Stop();
    _adjustTime = _nanoseconds;
    _nanoseconds = 0.0;

    // start if start
    if (start)
      Start();
  }

private:
  DOUBLE _nanoseconds;
  DOUBLE _adjustTime;
  DOUBLE _frequency;
  LLONG _startTime;
  LLONG _startTicks;
  BOOL8 _running;
};


/*
==============================================================================
History
------------------------------------------------------------------------------

==============================================================================
*/

#endif /* __TIMER_H__ */
