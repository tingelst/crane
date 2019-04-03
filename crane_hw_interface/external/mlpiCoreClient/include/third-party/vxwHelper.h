#ifndef __VXWHELPER_H__
#define __VXWHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <vxwHelper.h>
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
//! @version    1.29.1
//!
//! @date       2014
//
// -----------------------------------------------------------------------

//! @addtogroup UtilVxw UtilVxWorks
//! @{
//! @brief This module contains some useful functions and macros for handling within OS vxworks.
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
#include <targetinfo.h>

#if defined(TARGET_OS_VXWORKS)

#include <string.h>
#include <taskLib.h>
#include <ioLib.h>
#include <fiolib.h>
#include <fppLib.h>
#include <shellLib.h>

#if defined(TARGET_CPU_X86)
  #if (_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR >= 9)
    #include <hwif\cpu\arch\i86\vxCpuIdLib.h>
  #endif
  IMPORT CPUID  sysCpuId;
#elif defined(TARGET_CPU_SH4)

#else
  #error "MLPI: Unsupported hardware."
#endif

#if (_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR <= 3)
  #include <coprocLib.h>
#else
  #include <private\coprocLibP.h>
#endif

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------



// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------
static inline void enableTelnetPrintf(void)
{
  // print stdout to telnet connection, use fist available telnet shell
  SHELL_ID shellId = shellFirst();
  while(shellId != ((SHELL_ID) 0)) {
    char* name = taskName(shellTaskGet(shellId));
    if( (name != NULL) && (strcmp(name, ((char*) "tShell0"))!=0) ) {
      ioTaskStdSet(0, 1, ioTaskStdGet(shellTaskGet(shellId), 1));
      break;
    }
    shellId = shellNext(shellId);
  };
}


static inline void enableTelnetPrintfAlt1(void)
{
  // first alternative: print stdout to telnet connection, use last available telnet shell
  SHELL_ID shellIdTmp = shellFirst();
  SHELL_ID shellId = ((SHELL_ID) 0);
  while(shellIdTmp != ((SHELL_ID) 0)) {
    shellId = shellIdTmp;
    shellIdTmp = shellNext(shellId);
  };
  if(shellId != ((SHELL_ID) 0)) {
    char* name = taskName(shellTaskGet(shellId));
    if( (name != NULL) && (strcmp(name, ((char*) "tShell0"))!=0) )
      ioTaskStdSet(0, 1, ioTaskStdGet(shellTaskGet(shellId), 1));
  }
}


static inline void enableFpuSupport(void)
{
#if ((_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR < 9))
  FP_CONTEXT rFpuContext;
#else
  FPREG_SET rFpuContext;
#endif

  // Get current FPU register status
  if (coprocTaskRegsGet(taskIdSelf(), VX_FP_TASK, (VOID*)&rFpuContext) != OK)
    return;

  // Change FPU context of the new thread
#if ((_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR < 9))
  #if defined(TARGET_CPU_X86)
    if (sysCpuId.featuresEdx & CPUID_FXSR)
    {
      // Set precision to DOUBLE (53 Bits mantissa)
      rFpuContext.u.x.fpcr &= FPCR_PC_MASK;
      rFpuContext.u.x.fpcr |= FPCR_PC_DOUBLE;

      // Rounding control
      rFpuContext.u.x.fpcr &= FPCR_RC_MASK;
      rFpuContext.u.x.fpcr |= FPCR_RC_NEAREST;

      // Clear FPU status
      rFpuContext.u.x.fpsr = 0x0000;

      // Clear FPU registers
      memset (&rFpuContext.u.x.fpx, 0, sizeof (DOUBLEX_SSE) * FP_NUM_REGS);
      memset (&rFpuContext.u.x.xmm, 0, sizeof (DOUBLEX_SSE) * XMM_NUM_REGS);
      memset (&rFpuContext.u.x.res2, 0, sizeof (DOUBLEX_SSE) * FP_NUM_RESERVED);
    }
    else
    {
      //Old FPU type used e.g. on IndraMotion CML45

      // Set precision to DOUBLE (53 Bits mantissa)
      rFpuContext.u.o.fpcr &= FPCR_PC_MASK;
      rFpuContext.u.o.fpcr |= FPCR_PC_DOUBLE;

      // Rounding control
      rFpuContext.u.o.fpcr &= FPCR_RC_MASK;
      rFpuContext.u.o.fpcr |= FPCR_RC_NEAREST;

      // Clear FPU status
      rFpuContext.u.o.fpsr = 0x0000;

      // Clear FPU registers
      memset (&rFpuContext.u.o.fpx, 0, sizeof (DOUBLEX) * FP_NUM_REGS);
    }
  #elif defined(TARGET_CPU_SH4)
    // Set precision to DOUBLE (53 Bits mantissa)
    // FPSCR_DOUBLE_PRECISION is set automatically by the compiler each time,
    // he has to switch between float and double and can't be set for each task.

    // Rounding control
    // It has to be done global in the BSP and can't be set for each task.
  #else
    #error "MLPI: Unsupported hardware."
  #endif
#else
  #if defined(TARGET_CPU_X86)
    if((sysCpuId.featuresEcx & VX_CPUID_XSAVE) && (sysCpuId.featuresEcx & VX_CPUID_AVX))
    {
      //FPX_EXT_CONTEXT
      printf("\nUnsupported FPU feature (FPX_EXT_CONTEXT).");
    }
    else if (sysCpuId.featuresEdx & CPUID_FXSR)
    {
      // Set precision to DOUBLE (53 Bits mantissa)
      rFpuContext.n.x.fpcr &= FPCR_PC_MASK;
      rFpuContext.n.x.fpcr |= FPCR_PC_DOUBLE;

      // Rounding control
      rFpuContext.n.x.fpcr &= FPCR_RC_MASK;
      rFpuContext.n.x.fpcr |= FPCR_RC_NEAREST;

      // Clear FPU status
      rFpuContext.n.x.fpsr = 0x0000;

      // Clear FPU registers
      memset (&rFpuContext.n.x.fpx, 0, sizeof (DOUBLEX_SSE) * FP_NUM_REGS);
      memset (&rFpuContext.n.x.xmm, 0, sizeof (DOUBLEX_SSE) * XMM_NUM_REGS);
    }
    else
    {
      //Old FPO_CONTEXT
      printf("\nUnsupported FPU feature (Old FPO_CONTEXT).");
    }
  #elif defined(TARGET_CPU_SH4)
    #error "MLPI: Unsupported hardware (TARGET_CPU_SH4)."
  #else
    #error "MLPI: Unsupported hardware."
  #endif
#endif

  // Store back the modified FPU register
  if (coprocTaskRegsSet(taskIdSelf(), VX_FP_TASK,(VOID*)&rFpuContext) !=0)
    return;
}

#endif


#endif /* __VXWHELPER_H__ */
