#ifndef __TARGETINFO_H__
#define __TARGETINFO_H__
/**
==============================================================================
@file
@copyright  Bosch Rexroth Corporation http://www.boschrexroth.com
@date       2012
------------------------------------------------------------------------------
This header tries to detect the used compiler, operating system and cpu
of the target platform.
The introduced defines for CPU, OS and COMPILER are mutual exclusive.

At the moment, following defines are supported and will be set:
  - TARGET_COMPILER_INTEL
  - TARGET_COMPILER_GCC
  - TARGET_COMPILER_MSVC
  - TARGET_COMPILER_LLVM
  - TARGET_COMPILER_BORLAND

  - TARGET_OS_VXWORKS
    - TARGET_OS_VXWORKS_KERNEL
    - TARGET_OS_VXWORKS_RTP
  - TARGET_OS_WIN
    - TARGET_OS_WINCE32
    - TARGET_OS_WINNT
      - TARGET_OS_WINNT64
      - TARGET_OS_WINNT32
  - TARGET_OS_APPLE
    - TARGET_OS_MAC
    - TARGET_OS_IOS
      - TARGET_OS_IOS_DEVICE
      - TARGET_OS_IOS_SIM
  - TARGET_OS_ANDROID
  - TARGET_OS_LINUX
  - TARGET_OS_QNX
  - TARGET_OS_SYMBIAN
  - TARGET_OS_UNIX
  - TARGET_OS_POSIX
  - TARGET_OS_MINGW

  - TARGET_CPU_PPC64
  - TARGET_CPU_PPC32
  - TARGET_CPU_SH3
  - TARGET_CPU_SH4
  - TARGET_CPU_ARM64
  - TARGET_CPU_ARM
  - TARGET_CPU_MIPS
  - TARGET_CPU_IA64
  - TARGET_CPU_X64
  - TARGET_CPU_X86

  - TARGET_BITNESS_64BIT
  - TARGET_BITNESS_32BIT
==============================================================================
*/


/*
------------------------------------------------------------------------------
 Detect compiler
------------------------------------------------------------------------------
*/
#if defined(__ECC) || defined(__ICC) || defined(__INTEL_COMPILER)
  #define TARGET_COMPILER_STRING "Intel C/C++"
  #define TARGET_COMPILER_INTEL 1

#elif defined(__GNUC__)
  #define TARGET_COMPILER_STRING "Gnu GCC"
  #define TARGET_COMPILER_GCC 1

#elif defined(_MSC_VER)
  #define TARGET_COMPILER_STRING "Microsoft Visual C++"
  #define TARGET_COMPILER_MSVC 1

#elif defined(__clang__) || defined(__llvm__)
  #define TARGET_COMPILER_STRING "The LLVM Compiler Infrastructure"
  #define TARGET_COMPILER_LLVM 1

#elif defined(__BORLANDC__)
  #define TARGET_COMPILER_STRING "Borland C/C++"
  #define TARGET_COMPILER_BORLAND 1

#else
  #define TARGET_COMPILER_STRING "Unknown compiler"
  #error failed to detect compiler!
#endif




/*
------------------------------------------------------------------------------
 Detect operating system
------------------------------------------------------------------------------
*/
#if defined(__VXWORKS__) && (_WRS_KERNEL)
#define TARGET_OS_STRING "VxWorks Kernel " RUNTIME_VERSION
  #define TARGET_OS_VXWORKS 1
  #define TARGET_OS_VXWORKS_KERNEL 1
  // Hint: If you need more specific VxWorks informations, you may include the following 
  //       two files in your application.
  //
  // #if defined(TARGET_OS_VXWORKS)
  //   #include "vxWorksCommon.h"
  //   #include "version.h"
  // #endif
  //
  // After that, you can also check for VxWorks major (_WRS_VXWORKS_MAJOR) and minor (_WRS_VXWORKS_MINOR) version. e.g.:
  //
  // #if defined(TARGET_OS_VXWORKS) && ((_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR >= 9))
  //   <insert code which is only available on VxWorks 6.9 or newer>
  // #endif

#elif defined(__VXWORKS__) && (__RTP__)
  #define TARGET_OS_STRING "VxWorks RTP"
  #define TARGET_OS_VXWORKS 1
  #define TARGET_OS_VXWORKS_RTP 1

#elif defined(_WIN32_WCE)
  #define TARGET_OS_STRING "WindowsCE 32bit"
  #define TARGET_OS_WIN 1
  #define TARGET_OS_WINCE32 1

#elif defined(_WIN64)
  #define TARGET_OS_STRING "WindowsNT 64bit"
  #define TARGET_OS_WIN 1
  #define TARGET_OS_WINNT 1
  #define TARGET_OS_WINNT64 1

#elif defined(_WIN32_WINNT) || defined(_WINDLL) || defined (_WIN32)
  #define TARGET_OS_STRING "WindowsNT 32bit"
  #define TARGET_OS_WIN 1
  #define TARGET_OS_WINNT 1
  #define TARGET_OS_WINNT32 1

#elif defined(__APPLE__)
  #include "TargetConditionals.h"
  #if TARGET_OS_IPHONE
    #define TARGET_OS_STRING "Apple iOS Device"
    #define TARGET_OS_APPLE 1
    #define TARGET_OS_IOS 1
    #define TARGET_OS_IOS_DEVICE 1

  #elif TARGET_IPHONE_SIMULATOR
    #define TARGET_OS_STRING "Apple iOS Simulator"
    #define TARGET_OS_APPLE 1
    #define TARGET_OS_IOS 1
    #define TARGET_OS_IOS_SIM 1

  #elif TARGET_OS_MAC
    #define TARGET_OS_STRING "Apple Macintosh"
    #define TARGET_OS_APPLE 1
    #define TARGET_OS_MAC 1

  #else
    #define TARGET_OS_STRING "Apple Unknown"
  #endif

#elif defined(__ANDROID__)
  #define TARGET_OS_STRING "Android"
  #define TARGET_OS_ANDROID 1

#elif defined(HWDS_OS_PXR)
  // has to be before linux because build environment is used to identify target platform
  #define TARGET_OS_STRING "PXROS"
  #define TARGET_OS_PXROS 1

#elif defined(__linux__) ||  defined(__linux) || defined(linux)
  #define TARGET_OS_STRING "Linux"
  #define TARGET_OS_LINUX 1

#elif defined(__QNX__) || defined(__QNXNTO__)
  #define TARGET_OS_STRING "QNX"
  #define TARGET_OS_QNX 1

#elif defined(__SYMBIAN32__)
  #define TARGET_OS_STRING "Symbian"
  #define TARGET_OS_SYMBIAN 1

#elif defined(__unix)
  #define TARGET_OS_STRING "Unix"
  #define TARGET_OS_UNIX 1

#elif defined(__posix)
  #define TARGET_OS_STRING "Posix"
  #define TARGET_OS_POSIX 1

#elif defined(__MINGW32__)
  #define TARGET_OS_STRING "MinGW"
  #define TARGET_OS_MINGW 1

#else
  #define TARGET_OS_STRING "Unknown operating system"
  #error failed to detect operating system!
#endif




/*
------------------------------------------------------------------------------
 Detect CPU
------------------------------------------------------------------------------
*/
#if defined(__PPC__) || defined(__POWERPC__) || defined(_POWER) || defined(__ppc__) || defined(__powerpc__)
  #if defined(__powerpc64__)
    #define TARGET_CPU_STRING "PowerPC64"
    #define TARGET_CPU_PPC64 1

  #else
    #define TARGET_CPU_STRING "PowerPC"
    #define TARGET_CPU_PPC32 1

  #endif
#elif defined(_SH3)
  #define TARGET_CPU_STRING "Hitachi SH3"
  #define TARGET_CPU_SH3 1

#elif defined(__sh4__) || defined(__SH4__)
  #define TARGET_CPU_STRING "Hitachi SH4"
  #define TARGET_CPU_SH4 1

#elif  defined(__arm64__) || defined(__arm64) || defined(_ARM64)
  #define TARGET_CPU_STRING "ARM64"
  #define TARGET_CPU_ARM64 1

#elif  defined(__arm__) || defined(_ARM)
  #define TARGET_CPU_STRING "ARM"
  #define TARGET_CPU_ARM 1

#elif defined(__mips__) || defined(__MIPS__) || defined(_MIPS)
  #define TARGET_CPU_STRING "MIPS"
  #define TARGET_CPU_MIPS 1

#elif defined(__ia64) || defined(_M_IA64) || defined(__ia64__)
  #define TARGET_CPU_STRING "IA64"
  #define TARGET_CPU_IA64 1

#elif defined(__X86__) || defined(__i386__) || defined(i386) || defined(_M_IX86) || defined(__386__) || defined(__x86_64__) || defined(_M_X64)
  #if defined(__x86_64__) || defined(_M_X64)
    #define TARGET_CPU_STRING "AMD x86-64"
    #define TARGET_CPU_X64 1
  #else
    #define TARGET_CPU_STRING "Intel x86"
    #define TARGET_CPU_X86 1
  #endif

#else
  #define TARGET_CPU_STRING "Unknown CPU"
  #error failed to detect CPU!
#endif


/*
------------------------------------------------------------------------------
 Detect bitness
------------------------------------------------------------------------------
*/
#if defined (_WIN64) || defined (__LP64__) || defined (_LP64)
  #define TARGET_BITNESS_STRING "64 Bit"
  #define TARGET_BITNESS_64BIT 1
#else
  #define TARGET_BITNESS_STRING "32 Bit"
  #define TARGET_BITNESS_32BIT 1
#endif

/*
==============================================================================
History
------------------------------------------------------------------------------
10-Sep-2012 SK
  - first version
11-Sep-2012 SK Subject: MSC: cleanup of globaltype
  - corrected and reformated defines
09-Jun-2015 JR Subject: Add various defines
  - add defines for bitness, VxWorks, Apple, Windows, iOS, Apple, ...
==============================================================================
*/

#endif /* __TARGETINFO_H__ */



