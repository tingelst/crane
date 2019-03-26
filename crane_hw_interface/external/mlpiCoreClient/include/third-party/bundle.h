#ifndef __BUNDLE_H__
#define __BUNDLE_H__

// -----------------------------------------------------------------------
// BUNDLE - <bundle.h>
// -----------------------------------------------------------------------
// Copyright (c) 2013 Bosch Rexroth. All rights reserved.
// Redistribution and use in source and binary forms of this SPACE software
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
// the SPACE must be plainly marked as such and must not be misrepresented
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



//! @addtogroup Bundle Bundle
//! @{
//! @brief This header allows you to write native bundle libraries which can get executed by the server
//!        device.
//!
//! To write a bundle, you have to provide several entry points which get executed in different states
//! during boot of the control. Here is a short sample application.
//!
//!
//! @par Example:
//! @code
//! BUNDLE_INFO_BEGIN(com_boschrexroth_bundleDemo)
//!   BUNDLE_INFO_NAME          (L"Demo Bundle")
//!   BUNDLE_INFO_VENDOR        (L"Bosch Rexroth AG")
//!   BUNDLE_INFO_DESCRIPTION   (L"This is an example application, which shows how to create a bundle.")
//!   BUNDLE_INFO_VERSION       (1,0,0,0,L"Test Version")
//! BUNDLE_INFO_END(com_boschrexroth_bundleDemo)
//!
//!
//!
//! BUNDLE_EXPORT int com_boschrexroth_bundleDemo_create(int param1, int param2, int param3)
//! {
//!   printf("\n###################################################################");
//!   printf("\n## onCreate #######################################################");
//!   printf("\n###################################################################");
//!
//!   return 0;
//! }
//!
//! BUNDLE_EXPORT int com_boschrexroth_bundleDemo_start(int param1, int param2, int param3)
//! {
//!   printf("\n###################################################################");
//!   printf("\n## onStart ########################################################");
//!   printf("\n###################################################################");
//!
//!   return 0;
//! }
//!
//! BUNDLE_EXPORT int com_boschrexroth_bundleDemo_stop(int param1, int param2, int param3)
//! {
//!   printf("\n###################################################################");
//!   printf("\n## onStop #########################################################");
//!   printf("\n###################################################################");
//!
//!   return 0;
//! }
//! BUNDLE_EXPORT int com_boschrexroth_bundleDemo_destroy(int param1, int param2, int param3)
//! {
//!   printf("\n###################################################################");
//!   printf("\n## onDestroy ######################################################");
//!   printf("\n###################################################################");
//!
//!   return 0;
//! }
//! @endcode
//!
//! The resulting executable has to be placed in a subdirectory of the 'bundles' directory on the
//! OEM partition of the device. The name of the executable has to match the name of the directory.
//! Furthermore, this name has to match with the symbolic name of the bundle which is prefixed to
//! the functions, but contains fullstops instead of underscores.
//! In the example above, this is 'com_boschrexroth_bundleDemo'. Which would result in a directory and
//! executable file name of <c>com.boschrexroth.bundleDemo</c>.
//!
//! @}

//! @addtogroup BundleStructTypes Structs, Types, ...
//! @ingroup Bundle
//! @{
//! @brief List of used types, enumerations, structures and more...


// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"
#include "wchar16.h"

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

#define BUNDLE_VERSION_MAJOR                     (1)    //!< Major version of the bundle mechanism. Indicates compatibility breaks.
#define BUNDLE_VERSION_MINOR                     (1)    //!< Minor version of the bundle mechanism. Indicates compatible enhancements.


#ifdef __cplusplus

  #if defined(TARGET_OS_WINNT)
    #define BUNDLE_EXPORT extern "C" __declspec(dllexport)
  #else
    #define BUNDLE_EXPORT extern "C"
  #endif

#else

  #if defined (_MSC_VER)
    #define BUNDLE_EXPORT __declspec(dllexport)
  #else
    #define BUNDLE_EXPORT
  #endif

#endif

// -----------------------------------------------------------------------
// GLOBAL ENUMERATIONS
// -----------------------------------------------------------------------


// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------


// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef BundleInfo1
//! Bundle information structure for major version 1 of the bundle mechanism. This structure is used to hold various meta informations about the bundle.
//! This function has to be filled by using the provided MACROS in this header file.
//! <TABLE>
//! <TR><TH>           Type   </TH><TH>           Element              </TH><TH> Description                                </TH></TR>
//! <TR><TD id="st_t"> DOUBLE </TD><TD id="st_e"> minimum              </TD><TD> Minimum time duration of all measurements. </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> infoVersionMajor    </TD><TD> Describes the interface version between the bundle mechanism itself and the bundle loader. Major version changes might break compatibility.  </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> infoVersionMinor    </TD><TD> Describes the interface version between the bundle mechanism itself and the bundle loader. Minor version changes do not break compatibility. </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> versionMajor        </TD><TD> The version information of the bundle content.                                                                                               </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> versionMinor        </TD><TD> The version information of the bundle content.                                                                                               </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> versionPatch        </TD><TD> The version information of the bundle content.                                                                                               </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> versionBugfix       </TD><TD> The version information of the bundle content.                                                                                               </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> configDebug         </TD><TD> Contains information if the bundle is build with debug settings.                                                                             </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> configRtp           </TD><TD> Contains information if the bundle should be loaded as RTP.                                                                                  </TD></TR>
//! <TR><TD id="st_t"> ULONG   </TD><TD id="st_t"> configReserved      </TD><TD> Reserved for future use.                                                                                                                     </TD></TR>
//! <TR><TD id="st_t"> WCHAR16 </TD><TD id="st_t"> name[128]           </TD><TD> A descriptive name if the bundle. For example: "The Deathstar Bundle"                                                                        </TD></TR>
//! <TR><TD id="st_t"> WCHAR16 </TD><TD id="st_t"> symbolicNameC[128]  </TD><TD> The symbolic name of the bundle in C notation (underscores instead of fullstop). For example: "com_empire_deathstar"                         </TD></TR>
//! <TR><TD id="st_t"> WCHAR16 </TD><TD id="st_t"> vendor[128]         </TD><TD> A descriptive name of the vendor. For example: "The Galactic Empire"                                                                         </TD></TR>
//! <TR><TD id="st_t"> WCHAR16 </TD><TD id="st_t"> versionString[128]  </TD><TD> A descriptive version string of the bundle. For example: "Beta version"                                                                      </TD></TR>
//! <TR><TD id="st_t"> WCHAR16 </TD><TD id="st_t"> description[256]    </TD><TD> A description of the bundle. For example: "The deathstar is a moon-sized imperial battlestation armed with a superlaser!"                    </TD></TR>
//! </TABLE>
typedef struct BundleInfo1
{
  ULONG       infoVersionMajor;       //!< Describes the interface version between the bundle mechanism itself and the bundle loader. Major version changes might break compatibility.
  ULONG       infoVersionMinor;       //!< Describes the interface version between the bundle mechanism itself and the bundle loader. Minor version changes do not break compatibility.
  ULONG       versionMajor;           //!< The version information of the bundle content.
  ULONG       versionMinor;           //!< The version information of the bundle content.
  ULONG       versionPatch;           //!< The version information of the bundle content.
  ULONG       versionBugfix;          //!< The version information of the bundle content.
  ULONG       configDebug;            //!< Contains information if the bundle is build with debug settings.
  ULONG       configRtp;              //!< Contains information if the bundle should be loaded as RTP.
  ULONG       configReserved;         //!< Reserved for future use.
  WCHAR16     name[128];              //!< A descriptive name if the bundle. For example: "The Deathstar Bundle"
  WCHAR16     symbolicNameC[128];     //!< The symbolic name of the bundle in C notation (underscores instead of fullstop). For example: "com_empire_deathstar"
  WCHAR16     vendor[128];            //!< A descriptive name of the vendor. For example: "The Galactic Empire"
  WCHAR16     versionString[128];     //!< A descriptive version string of the bundle. For example: "Beta version"
  WCHAR16     description[256];       //!< A description of the bundle. For example: "The deathstar is a moon-sized imperial battlestation armed with a super laser!"
} BundleInfo1;

#define BUNDLE_ADAPT_STRING_WCHAR16(x) L##x

#ifdef NDEBUG
  #define BUNDLE_INFO_BEGIN(_symbolicName_) \
    static BundleInfo1 *getBundleInfo1(void) \
    { \
      static BundleInfo1 info; \
      memset(&info, 0, sizeof(info)); \
      info.infoVersionMajor = BUNDLE_VERSION_MAJOR; \
      info.infoVersionMinor = BUNDLE_VERSION_MINOR; \
      info.configDebug = 0; \
      info.configRtp = 0; \
      info.configReserved = 0; \
      wcscpy16(info.symbolicNameC, BUNDLE_ADAPT_STRING_WCHAR16(#_symbolicName_));
#else
  #define BUNDLE_INFO_BEGIN(_symbolicName_) \
    static BundleInfo1 *getBundleInfo1(void) \
    { \
      static BundleInfo1 info; \
      memset(&info, 0, sizeof(info)); \
      info.infoVersionMajor = BUNDLE_VERSION_MAJOR; \
      info.infoVersionMinor = BUNDLE_VERSION_MINOR; \
      info.configDebug = 1; \
      info.configRtp = 0; \
      info.configReserved = 0; \
      wcscpy16(info.symbolicNameC, BUNDLE_ADAPT_STRING_WCHAR16(#_symbolicName_));
#endif

#define BUNDLE_INFO_END(_symbolicName_) \
    return &info; \
  } \
  BUNDLE_EXPORT int _symbolicName_##_getBundleInfo(BundleInfo1 *info) \
  { \
    *info = *getBundleInfo1(); \
    return 0; \
  }

//! The descriptive name of the bundle.
#define BUNDLE_INFO_NAME(value) \
  wcscpy16(info.name, value);

//! The bundle vendor.
#define BUNDLE_INFO_VENDOR(value) \
  wcscpy16(info.vendor, value);

//! A short description of the bundle. What does it do?
#define BUNDLE_INFO_DESCRIPTION(value) \
  wcscpy16(info.description, value);

//! The bundle version information and an additional user configurable version string.
#define BUNDLE_INFO_VERSION(_versionMajor, _versionMinor, _versionBugfix, _versionPatch, _versionString) \
  info.versionMajor  = _versionMajor; \
  info.versionMinor  = _versionMinor; \
  info.versionBugfix = _versionBugfix; \
  info.versionPatch  = _versionPatch; \
  wcscpy16(info.versionString, _versionString);



#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup BundleLibStructTypes


/*
==============================================================================
History
------------------------------------------------------------------------------
01-Jan-2012
  - first version
==============================================================================
*/

#endif // endof: #ifndef __BUNDLE_H__
