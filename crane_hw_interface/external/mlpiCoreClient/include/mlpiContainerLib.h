#ifndef __MLPICONTAINERLIB_H__
#define __MLPICONTAINERLIB_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiContainerLib.h>
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



//! @addtogroup ContainerLib ContainerLib
//! @{
//! @brief Use the ContainerLib when you need to access a larger set of data repetitively and with maximum
//! update speed. For example, input data you want to read every machine cycle. Using the function @ref mlpiContainerCreate,
//! you first have to create a container by naming all the elements you want to read with the container. This also defines the
//! memory layout of the container. After that, you can read/write your data using the @ref mlpiContainerUpdate function.
//! When the container is no longer needed, delete it using @ref mlpiContainerDestroy.
//!
//! @attention The addresses of the variables of the container content might change on every download, reset origin
//!            or online change of the PLC application, so you have to stop and destroy each regarding container
//!            before you load or change the PLC application!
//!
//! @attention Please ensure you don't destroy a container during an access on it (update, read information e.g.)!
//!
//! @details A container is a list of items which are described as
//! so called 'tags'. Each 'tag' identifies a data element which can be read or written. For example, a symbolic
//! variable from the PLC or data from the input area of the I/O mapping.
//!
//! The first argument of a tag always defines the type of data to read or write. The following data sources
//! are available:
//! @arg
//! @b LOGICLIB_MEMORY_AREA:         Accessing Input, Output and Marker area of PLC (%%I, %%Q, %%M).
//! @arg
//! @b LOGICLIB_SYMBOL:              Accessing variables and arrays of PLC by symbolic name using symbolic variables.
//!                                  Access requires a symbol configuration of desired variables.
//! @arg
//! @b IOLIB_FIELDBUS_IO:            Accessing fieldbus IO data directly from fieldbus driver. Fixed spelling error (notice the 'd'). Needs Server-Version newer than 1.5.0.0.
//! @arg
//! @b IOLIB_FIELBUS_IO:             Accessing fieldbus IO data directly from fieldbus driver. Compatible tag with spelling error for Server-Verion 1.0.0.0 and newer. Still working in newer versions. Use if you want to access also controls older than 13V12.
//! @arg
//! @b ALIGNMENT_DUMMY:              Dummy elements to align the subsequent tag.
//!
//!
//! @par LOGICLIB_MEMORY_AREA
//! @arg argument 1: application name. (e.g. "Application")
//! @arg argument 2: area. (Either "INPUT", "OUTPUT" or "MARKER")
//! @arg argument 3: bit offset. (e.g. "0" to read from start offset. On bit access any offset value is valid, on byte access the offset must be a multiple of 8.)
//! @arg argument 4: bit length. (Note: Single bit access (1) or byte access (8*n) supported. e.g. "1", "8", "16", "24", ... bit.)
//!
//! @par LOGICLIB_SYMBOL
//! @arg argument 1: symbol name. (e.g. "Application.PlcProg.boDummy")
//!
//! @par IOLIB_FIELDBUS_IO
//! @arg argument 1: master name. (e.g. "Onboard_I_O"). The name of the fieldbus master is the name of the regarding master node in your IndraWorks project. You can also retrieve the list of configured master names by using the function @ref mlpiIoReadFieldbusMasterList.
//! @arg argument 2: slave address. (e.g. "1");
//! @arg argument 3: area. (Either "INPUT" or "OUTPUT")
//! @arg argument 4: bit offset. (e.g. "0" to read from start offset. On bit access any offset value is valid, on byte access the offset must be a multiple of 8.)
//! @arg argument 5: bit length. (Note: Single bit access (1) or byte access (8*n) supported. e.g. "1", "8", "16", "24", ... bit.)
//!
//! @par ALIGNMENT_DUMMY
//! @arg argument 1: byte length (Note: Only "1", "2", "4" or "8" byte supported.)
//!
//! @par Example:
//! @code
//! const WCHAR16 tagList[] =
//! {
//!   L"LOGICLIB_MEMORY_AREA,Application,MARKER,0,32;"      // first 4 bytes of the marker address space (%MX0.0 to %MX3.7)
//!   L"LOGICLIB_MEMORY_AREA,Application,MARKER,32,128;"    // 16 bytes of the marker address space starting at offset 4 (%MX4.0 to %MX19.7)
//!   L"LOGICLIB_MEMORY_AREA,Application,INPUT,32,32;"      // 4 bytes of the input address space starting at offset 4 (%IX4.0 to %IX7.7)
//!   L"IOLIB_FIELBUS_IO,Onboard_I_O,0,INPUT,0,8;"          // first 1 byte of the input area of the first slave on the OnboardIO.
//!   L"ALIGNMENT_DUMMY,1;"                                 // 1 byte alignment dummy value.
//!   L"ALIGNMENT_DUMMY,2;"                                 // 2 byte alignment dummy value.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varUint;" // symbolic variable 'Application.USERVARGLOBAL.varUint'
//!   L"ALIGNMENT_DUMMY,2;"                                 // 2 byte alignment dummy value.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varReal;" // symbolic variable 'Application.USERVARGLOBAL.varReal'
//! };
//! @endcode
//!
//!
//! Let's say we have the following data in the 'UserVarGlobal' variable list of our application project.
//! @code
//! {attribute 'symbol' := 'readwrite'}
//! {attribute 'linkalways'}
//! VAR_GLOBAL
//!   varBoolean    : BOOL := FALSE;
//!   varString     : STRING[31] := 'MLPI Test String';
//!   varUdint      : UDINT := 23;
//!   varUdintArray : ARRAY[0..5] OF UDINT := [4, 8, 15, 16, 23, 42];
//!   varReal       : REAL := 42.0;
//!   varRealArray  : ARRAY[0..5] OF REAL := [4, 8, 15, 16, 23, 42];
//! END_VAR
//! @endcode
//!
//! Now let's imagine, we want to read all of the data and also the first 4 bytes of the input area (%%IX0.0 to %%IX3.7). The
//! corresponding tagList for your C/C++ application would look like this:
//! @code
//! const WCHAR16 tagList[] =
//! {
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varString;"     // 0x00: the 31 byte string + 1 byte null termination.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varUdint;"      // 0x20: 4 byte unsigned integer.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varUdintArray;" // 0x24: 6*4 byte array of unsigned integer.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varBoolean;"    // 0x3C: 1 byte boolean value.
//!   L"ALIGNMENT_DUMMY,1;"                                       // 0x3D: we add 1 bytes of dummy value for better alignment.
//!   L"ALIGNMENT_DUMMY,2;"                                       // 0x3E: we add 2 bytes of dummy value for better alignment. Next value is again aligned to 4 byte.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varReal;"       // 0x40: 4 byte float value.
//!   L"LOGICLIB_SYMBOL,Application.USERVARGLOBAL.varRealArray;"  // 0x44: 6*4 byte array of floats.
//!   L"LOGICLIB_MEMORY_AREA,Application,INPUT,0,32;"             // 0x9C: first 4 bytes of the input address space (%IX0.0 to %IX3.7)
//! };
//! @endcode
//!
//! @note The layout of the different elements and your container is free. You don't need to have the same variable order as in the PLC.
//!       Use the alignment elements to align the elements to have better and faster access from within your C/C++ application.
//!
//! Having defined our tagList, the next step is to create a container for reading by using the function @ref mlpiContainerCreate.
//! A container can either be used for reading or for writing. Not for both! You need to create two containers if you want to
//! read and write the same set of data. In this example, we only create a read container using the following piece of code.
//!
//! @code
//! // this is our handle which is a reference to the container
//! MlpiContainerHandle hContainer;
//! memset(&hContainer, 0, sizeof(hContainer));
//! ULONG containerSize = 0;
//!
//! MLPIRESULT result = mlpiContainerCreate(connection, tagList, MLPI_CONTAINER_ACCESS_READ, &hContainer, &containerSize);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//! printf("\nContainer created. Size in bytes: %d", containerSize);
//! @endcode
//!
//! After we have created the container, we can use the function @ref mlpiContainerUpdate to read the current data of the
//! container elements. This should be really fast now, as the MLPI has cached some information to provide a really fast mechanism here.
//! Call the update function whenever you need to refresh the data.
//!
//! @code
//! // create some memory first
//! UCHAR *containerData = new UCHAR[containerSize];
//! memset(containerData, 0, containerSize);
//!
//! // update the container!
//! result = mlpiContainerUpdate(connection, hContainer, containerData, containerSize);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   delete [] containerData;
//!   return result;
//! }
//! @endcode
//!
//! Now let's dump the content of the container data we've just read:
//!
//! @code
//! // dump the data
//! printf("\ndumping container data:\n");
//! utilHexdump(containerData, containerSize);  // #include "util/mlpiGlobalHelper.h"
//! @endcode
//!
//! This results in the following output:
//! @image html containerdump.png
//! We can see the requested data elements as described in the tagList.
//!
//! For easier access in your application, you can build a struct with memory maps to your requested data.
//! In the example above, a struct would look like this. You may want to place this in a header file.
//!
//! @code
//! // activate structure packing to match container alignment
//! #if defined(_MSC_VER)
//!   #pragma pack(push,1)
//!   #define PACKED
//! #elif defined(__GNUC__)
//!   #define PACKED __attribute__ ((__packed__))
//! #endif
//!
//! // define structure, which maps exactly to our requested container layout
//! // given by the taglist.
//! struct ContainerMap
//! {
//!   CHAR   varString[32];
//!   ULONG  varUdint;
//!   ULONG  varUdintArray[6];
//!   BOOL8  varBoolean;
//!   UCHAR  varDummy1;
//!   USHORT varDummy2;
//!   FLOAT  varReal;
//!   FLOAT  varRealArray[6];
//!   UCHAR  memoryAreaInput[4];
//! } PACKED;
//!
//! // reset structure packing back to default
//! #if defined(_MSC_VER)
//!   #pragma pack(pop)
//!   #undef PACKED
//! #elif defined(__GNUC__)
//!   #undef PACKED
//! #endif
//! @endcode
//!
//! Now you can cast your buffer with the container data to the structure and have easy access to the data:
//!
//! @code
//! // check if structure really matches our container size
//! if (sizeof(ContainerMap) != containerSize) {
//!   printf("\ncontainerSize doesn't match structure! Incorrect structure of container layout?!");
//!   return -1;
//! }
//!
//! // cast memory to structure pointer
//! struct ContainerMap *containerMap = (struct ContainerMap*) containerData;
//!
//! // print the data
//! printf("\nvarString: %s", containerMap->varString);
//! printf("\nvarUdint: %u", containerMap->varUdint);
//! printf("\nvarUdintArray: %u %u %u %u %u %u"
//!   , containerMap->varUdintArray[0], containerMap->varUdintArray[1], containerMap->varUdintArray[2]
//!   , containerMap->varUdintArray[3], containerMap->varUdintArray[4], containerMap->varUdintArray[5]);
//! printf("\nvarBoolean: %s", containerMap->varBoolean ? "TRUE" : "FALSE");
//! printf("\nvarReal: %f", containerMap->varReal);
//! printf("\nvarRealArray: %f %f %f %f %f %f"
//!   , containerMap->varRealArray[0], containerMap->varRealArray[1], containerMap->varRealArray[2]
//!   , containerMap->varRealArray[3], containerMap->varRealArray[4], containerMap->varRealArray[5]);
//! @endcode
//!
//! This also enables debugger support in your development environment.
//! @image html containerwatch.png
//!
//! @note The ContainerLib functions trace their debug information mainly into the module MLPI_CONTAINER_LIB and
//!       in additional into the modules MLPI_ACCESSOR and MLPI_BASE_MODULES. For further information see also
//!       the detailed description of the library @ref TraceLib and the notes about @ref sec_TraceViewer.
//! @}

//! @addtogroup ContainerLibCommon Common function
//! @ingroup ContainerLib
//! @{
//! @brief Contains functions to create, update and destroy containers.
//!
//! @attention The addresses of the variables of the container content might change on every download, reset origin
//!            or online change of the PLC application, so you have to stop and destroy each regarding container
//!            before you load or change the PLC application!
//!
//! @details Before you can use any other container function, you need to create a container using the
//! function @ref mlpiContainerCreate provided here. Update the container by using @ref mlpiContainerUpdate
//! and don't forget to release the container by calling @ref mlpiContainerDestroy when you are finished.
//!
//! @}

//! @addtogroup ContainerLibAux Auxiliary function
//! @ingroup ContainerLib
//! @{
//! @brief Contains additional functions for getting more detailed information about the properties and structure
//! of a container.
//!
//! @attention The addresses of the variables of the container content might change on every download, reset origin
//!            or online change of the PLC application, so you have to stop and destroy each regarding container
//!            before you load or change the PLC application!
//!
//! @details These functions are not needed for simple reading and writing using containers. Use them only if you
//! want to dive deeper into the container system of the MLPI.
//!
//! @}

//! @addtogroup ContainerLibVersionPermission Version and Permission
//! @ingroup ContainerLib
//! @{
//! @addtogroup ContainerLibVersionPermission_new Server version since 1.26.0.0 (MLC-FW: 14V22)
//! @ingroup ContainerLibVersionPermission
//! @{
//!
//! @note Since firmware version 14V22 (MLPI-Server-Version: 1.26.0.0) a centralized permission management has been implemented in target 
//! controls XM2, L75 and VPx. Some permissions have been summarized in order to improve their usability. 
//! Additional information regarding the usage of older manifest files (i.e. accounts.xml) with newer server versions can be found in @ref newest_manifest.\n
//! @note <b><span style="color:red">Users of other CML controls (i.e. L25, L45, L65) have to use the old permissions as defined in @ref ContainerLibVersionPermission_old</span></b>
//!
//!
//! @par List of valid permissions for mlpiContainerLib. These permissions shall be assigned to the groups (i.e. in the group manifest file groups.xml) rather than the users.
//! <TABLE>
//! <TR><TH> Permission-Ident                </TH><TH> Description                                                                            </TH></TR>                  
//! <TR><TD id="st_e"> CONTAINER_BROWSE      </TD><TD> Browse - Allows to browse through existing containers                                  </TD></TR>  
//! <TR><TD id="st_e"> CONTAINER_INFO        </TD><TD> Info - Allows to get information about containers, e.g. name, items etc.               </TD></TR>  
//! <TR><TD id="st_e"> CONTAINER_SETUP       </TD><TD> Setup - Allows to create and delete containers and set container name.                 </TD></TR>
//! <TR><TD id="st_e"> CONTAINER_UPDATE      </TD><TD> Update - Allows to read/write data from/to containers, depending on container type.    </TD></TR>
//! </TABLE>
//!
//!  @par List of available functions in mlpiContainerLib and the permissions required for their use. 
//! <TABLE>
//! <TR><TH>           Function                                   </TH><TH> Server version </TH><TH> Permission-Ident   </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerCreate                   </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_SETUP"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerUpdate                   </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_UPDATE" </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerDestroy                  </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_SETUP"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetName                  </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_INFO"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerSetName                  </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_SETUP"  </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetInformation           </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_INFO"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetTagList               </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_INFO"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetItemInformation       </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_INFO"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetSingleItemInformation </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_INFO"   </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetNumberOfContainer     </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_BROWSE" </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetHandlesOfContainer    </TD><TD> 1.0.0.0        </TD><TD> "CONTAINER_BROWSE" </TD></TR>
//! </TABLE>
//!
//! @par List of the old permissions of mlpiContainerLib and their corresponding new permission.
//! <TABLE>
//! <TR><TH> Old permission                                 </TH><TH> new Permission    </TH></TR>   
//! <TR><TD id="st_e"> MLPI_CONTAINERLIB_PERMISSION_ALWAYS  </TD><TD> IMPLICIT          </TD></TR>  
//! <TR><TD id="st_e"> MLPI_CONTAINERLIB_PERMISSION_SETUP   </TD><TD> CONTAINER_SETUP   </TD></TR>  
//! <TR><TD id="st_e"> MLPI_CONTAINERLIB_PERMISSION_UPDATE  </TD><TD> CONTAINER_UPDATE  </TD></TR>  
//! <TR><TD id="st_e"> MLPI_CONTAINERLIB_PERMISSION_INFO    </TD><TD> CONTAINER_INFO    </TD></TR>  
//! <TR><TD id="st_e"> MLPI_CONTAINERLIB_PERMISSION_BROWSE  </TD><TD> CONTAINER_BROWSE  </TD></TR>   
//! </TABLE>
//!
//! @}
//! @addtogroup ContainerLibVersionPermission_old Server versions before 1.26.0.0 
//! @ingroup ContainerLibVersionPermission
//! @{
//! @brief Version and permission information
//!
//! The table shows requirements regarding the minimum server version (@ref sec_ServerVersion) and the user
//! permission needed to execute the desired function. Furthermore, the table shows the current user and
//! permissions setup of the 'accounts.xml' placed on the SYSTEM partition of the control. On using the
//! permission @b "MLPI_CONTAINERLIB_PERMISSION_ALL" with the value "true", you will enable all functions
//! of this library for a user account.
//!
//! @note Function with permission MLPI_CONTAINERLIB_PERMISSION_ALWAYS cannot blocked.
//!
//! @par List of permissions of mlpiContainerLib using in accounts.xml
//! - MLPI_CONTAINERLIB_PERMISSION_ALL
//! - MLPI_CONTAINERLIB_PERMISSION_INFO
//! - MLPI_CONTAINERLIB_PERMISSION_SETUP
//! - MLPI_CONTAINERLIB_PERMISSION_BROWSE
//! - MLPI_CONTAINERLIB_PERMISSION_UPDATE
//!
//! <TABLE>
//! <TR><TH>           Function                                   </TH><TH> Server version </TH><TH> Permission-Ident                             </TH><TH> a(1) </TH><TH> i(1) </TH><TH> i(2) </TH><TH> i(3) </TH><TH> m(1) </TH></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerCreate                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerUpdate                   </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_UPDATE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerDestroy                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetName                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerSetName                  </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_SETUP"   </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetInformation           </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetTagList               </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetItemInformation       </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetSingleItemInformation </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_INFO"    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetNumberOfContainer     </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_BROWSE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
//! <TR><TD id="st_e"> @ref mlpiContainerGetHandlesOfContainer    </TD><TD> 1.0.0.0        </TD><TD> "MLPI_CONTAINERLIB_PERMISSION_BROWSE"  </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD> x    </TD><TD>      </TD></TR>
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

//! @addtogroup ContainerLibStructTypes Structs, Types, ...
//! @ingroup ContainerLib
//! @{
//! @brief List of used types, enumerations, structures and more...



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include "mlpiGlobal.h"


// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------
#define MLPI_CONTAINER_TAG_SEPARATOR                      MLPI_ADAPT_STRING(';')
#define MLPI_CONTAINER_ARG_SEPARATOR                      MLPI_ADAPT_STRING(',')

#define MLPI_CONTAINER_TAG_LOGICLIB_MEMORY_AREA           MLPI_ADAPT_STRING("LOGICLIB_MEMORY_AREA")
#define MLPI_CONTAINER_TAG_LOGICLIB_SYMBOL                MLPI_ADAPT_STRING("LOGICLIB_SYMBOL")
#define MLPI_CONTAINER_TAG_IOLIB_FIELBUS_IO               MLPI_ADAPT_STRING("IOLIB_FIELBUS_IO")     //<! Deprecated.
#define MLPI_CONTAINER_TAG_IOLIB_FIELDBUS_IO              MLPI_ADAPT_STRING("IOLIB_FIELBUS_IO")     //<! Spelling of "Fieldbus" was wrong till MLPI version 1.5.0.0. Due to compatibility reasons, we can not change the adapted string value.
#define MLPI_CONTAINER_TAG_ALIGNMENT_DUMMY                MLPI_ADAPT_STRING("ALIGNMENT_DUMMY")

#define MLPI_CONTAINER_ARG_IOLIB_IO_AREA_INPUT            MLPI_ADAPT_STRING("INPUT")
#define MLPI_CONTAINER_ARG_IOLIB_IO_AREA_OUTPUT           MLPI_ADAPT_STRING("OUTPUT")

#define MLPI_CONTAINER_ARG_LOGICLIB_MEMORY_AREA_INPUT     MLPI_ADAPT_STRING("INPUT")
#define MLPI_CONTAINER_ARG_LOGICLIB_MEMORY_AREA_OUTPUT    MLPI_ADAPT_STRING("OUTPUT")
#define MLPI_CONTAINER_ARG_LOGICLIB_MEMORY_AREA_MARKER    MLPI_ADAPT_STRING("MARKER")

#define MLPI_CONTAINER_NAME_MAX_LENGTH                    (64)                                        //!< Maximum length of a container name.

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

//! @enum MlpiContainerAccess
//! This enumeration defines whether we have a READ or WRITE container.
typedef enum MlpiContainerAccess
{
  MLPI_CONTAINER_ACCESS_READ   = 0,   //!< Enumeration for read container.
  MLPI_CONTAINER_ACCESS_WRITE  = 1    //!< Enumeration for write container.
}MlpiContainerAccess;

// message packing follows 8 byte natural alignment
#if !defined(TARGET_OS_VXWORKS)
#pragma pack(push,8)
#endif

//! @typedef MlpiContainerHandle
//! @brief This structure defines the handle to a container.
//! @details Elements of struct MlpiContainerHandle
//! <TABLE>
//! <TR><TH>           Type  </TH><TH>           Element      </TH><TH> Description             </TH></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> connectionID </TD><TD> Identity of connection. </TD></TR>
//! <TR><TD id="st_t"> ULONG </TD><TD id="st_e"> containerID  </TD><TD> Identity of container.  </TD></TR>
//! </TABLE>
//! You don't need to change or set the elements of this struct, as they are set by the MLPI.
typedef struct MlpiContainerHandle
{
  ULONG connectionID;      //!< Identity of connection.
  ULONG containerID;       //!< Identity of container.
} MlpiContainerHandle;

//! @typedef MlpiContainerInformation
//! @brief This structure defines the information content of a container.
//! @details Elements of struct MlpiContainerInformation
//! <TABLE>
//! <TR><TH>           Type                     </TH><TH>           Element                </TH><TH> Description                              </TH></TR>
//! <TR><TD id="st_t">      WCHAR16             </TD><TD id="st_e"> name                   </TD><TD> Name of container.                       </TD></TR>
//! <TR><TD id="st_t">      MlpiDateAndTime     </TD><TD id="st_e"> dateTime               </TD><TD> Date and time container was created.     </TD></TR>
//! <TR><TD id="st_t">      ULONG               </TD><TD id="st_e"> numItems               </TD><TD> Number of entries of container.          </TD></TR>
//! <TR><TD id="st_t">      ULONG               </TD><TD id="st_e"> numElementsTagList     </TD><TD> Number of elements describing container. </TD></TR>
//! <TR><TD id="st_t">      ULONG               </TD><TD id="st_e"> dataSize               </TD><TD> Total data size (byte) of container.     </TD></TR>
//! <TR><TD id="st_t"> @ref MlpiContainerAccess </TD><TD id="st_e"> accessFlag             </TD><TD> READ or WRITE access to the container.   </TD></TR>
//! </TABLE>
typedef struct MlpiContainerInformation
{
  WCHAR16             name[MLPI_CONTAINER_NAME_MAX_LENGTH]; //!< Name of container.
  MlpiDateAndTime     dateTime;                             //!< Date and time container was created.
  ULONG               numItems;                             //!< Number of entries of container.
  ULONG               numElementsTagList;                   //!< Number of elements describing container.
  ULONG               dataSize;                             //!< Total data size (byte) of container.
  MlpiContainerAccess accessFlag;                           //!< READ or WRITE access to the container.
} MlpiContainerInformation;

//! @typedef MlpiContainerItemInformation
//! @brief This structure defines the information of a container item.
//! @details Elements of struct MlpiContainerItemInformation
//! <TABLE>
//! <TR><TH>           Type           </TH><TH>           Element   </TH><TH> Description                                   </TH></TR>
//! <TR><TD id="st_t">      MlpiType  </TD><TD id="st_e"> type      </TD><TD> Mlpi data type of item.                       </TD></TR>
//! <TR><TD id="st_t">      ULONG     </TD><TD id="st_e"> offset    </TD><TD> Offset of item within the container in bytes. </TD></TR>
//! <TR><TD id="st_t">      ULONG     </TD><TD id="st_e"> dataSize  </TD><TD> Size of item in bytes.                        </TD></TR>
//! </TABLE>
typedef struct MlpiContainerItemInformation
{
  MlpiType  type;                 //!< Mlpi data type of item.
  ULONG     offset;               //!< Offset of item within the container in bytes.
  ULONG     dataSize;             //!< Size of item in bytes.
} MlpiContainerItemInformation;


#if !defined(TARGET_OS_VXWORKS)
#pragma pack(pop)
#endif

//! @} // endof: @ingroup ContainerLibStructTypes




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


//! @ingroup ContainerLibCommon
//! This function creates a new container. It has to be specified if the container is a read or
//! write container. It is not possible to read and write using the same container.
//! To specify which data to read or write, you have to give a tag list to the function. The tag list
//! is a formatted string of one or multiple tags which are separated by a semicolon (@c ;). Each tag is
//! built of multiple arguments which are again separated by comma (@c ,).
//!
//! Please have a look at @ref ContainerLib for an extensive introduction to the ContainerLib mechanism.
//!
//! @attention The addresses of the variables of the container content might change on every download, reset origin
//!            or online change of the PLC application, so you have to stop and destroy each regarding container
//!            before you load or change the PLC application!
//!
//! @param[in]    connection        Handle for multiple connections.
//! @param[in]    tagList           Configuration description of container. This is a string with multiple tags as described above.
//!                                 Tags are delimited by semicolons.
//! @param[in]    accessFlag        Configuration of container regarding read or write access.
//! @param[out]   handle            Handle for use of container.
//! @param[out]   dataSize          Data size of container in bytes.
//! @return                         Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // This example creates a read buffer which reads 4 bytes of input IO and four variables.
//! MlpiContainerHandle hContainer;
//! memset(&hContainer, 0, sizeof(hContainer));
//! ULONG containerSize = 0;
//!
//! // build the tag list which describes which containerData to pack into the container
//! const WCHAR16 tagList[] =
//! {
//!   L"LOGICLIB_MEMORY_AREA,Application,INPUT,0,32;"
//!   L"LOGICLIB_SYMBOL,Application.PlcProg.varReal;"
//!   L"LOGICLIB_SYMBOL,Application.PlcProg.varUint;"
//!   L"LOGICLIB_SYMBOL,Application.PlcProg.varString;"
//!   L"LOGICLIB_SYMBOL,Application.PlcProg.varUintArray;"
//! };
//!
//! // create the container
//! MLPIRESULT result = mlpiContainerCreate(connection, tagList, MLPI_CONTAINER_ACCESS_READ, &hContainer, &containerSize);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//! printf("\nContainer created. Size in bytes: %d", containerSize);
//!
//! // read the container
//! UCHAR *containerData = new UCHAR[containerSize];
//! memset(containerData, 0, containerSize);
//! result = mlpiContainerUpdate(connection, hContainer, containerData, containerSize);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   delete [] containerData;
//!   return result;
//! }
//!
//! // dump the data
//! printf("\ndumping container data:\n");
//! utilHexdump(containerData, containerSize);  // #include "util/mlpiGlobalHelper.h"
//!
//! // delete container and free resources
//! mlpiContainerDestroy(connection, &hContainer);
//! delete [] containerData;
//! @endcode
MLPI_API MLPIRESULT mlpiContainerCreate(const MLPIHANDLE connection, const WCHAR16 *tagList, const MlpiContainerAccess accessFlag, MlpiContainerHandle *handle, ULONG *dataSize);


//! @ingroup ContainerLibCommon
//! This function updates a container. If you pass a handle to a write container, then you also need to pass the data and
//! correct data size to the data argument of the function. If you pass a handle to a read container, then you will receive
//! the data that gets read by the function in the buffer given to the data argument.
//!
//! @param[in]      connection  Handle for multiple connections.
//! @param[in]      handle      Handle that specifies the container to update. Use @ref mlpiContainerCreate to create a container.
//! @param[in,out]  data        Data to be written in case of write-container. Data buffer to read data to in case of read-container.
//! @param[in]      dataSize    Size in bytes of @c data argument.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref mlpiContainerCreate
MLPI_API MLPIRESULT mlpiContainerUpdate(const MLPIHANDLE connection, const MlpiContainerHandle handle, void *data, const ULONG dataSize);


//! @ingroup ContainerLibCommon
//! This function destroys a container which has been created using @ref mlpiContainerCreate. Containers consume memory resources of the
//! server. You should therefore destroy any containers your application no longer needs!
//!
//!
//! @attention Please ensure you don't destroy a container when accessing it (update, read information e.g.)!
//!
//! @param[in]      connection  Handle for multiple connections.
//! @param[in]      handle      Handle of the container to destroy. Use @ref mlpiContainerCreate to create a container.
//! @return                     Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! See @ref mlpiContainerCreate
MLPI_API MLPIRESULT mlpiContainerDestroy(const MLPIHANDLE connection, MlpiContainerHandle *handle);


//! @ingroup ContainerLibAux
//! Using this function, you can assign a descriptive name to your container. The name can be any string. Duplicates are allowed.
//! It is not necessary to give your container a name, but the name is used on diagnosis messages and might be useful for
//! debugging and maintaining your application.
//!
//! @param[in]    connection  Handle for multiple connections.
//! @param[in]    handle      Handle of the container. Use @ref mlpiContainerCreate to create a container.
//! @param[in]    name        String containing a name for the container. Maximum length is defined by @ref MLPI_CONTAINER_NAME_MAX_LENGTH.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // let's call our container 'Batman'
//! MLPIRESULT result = mlpiContainerSetName(connection, hContainer, L"Batman");
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiContainerSetName(const MLPIHANDLE connection, const MlpiContainerHandle handle, const WCHAR16 *name);


//! @ingroup ContainerLibAux
//! Using this function, you can read out the name of your container. The name can be any string with a maximum size of
//! @c MLPI_CONTAINER_NAME_MAX_LENGTH. Use the function @ref mlpiContainerSetName to set the name.
//!
//! @param[in]    connection  Handle for multiple connections.
//! @param[in]    handle      Handle of the container. Use @ref mlpiContainerCreate to create a container.
//! @param[out]   name        String where the container name will be stored.
//! @param[in]    numElements Number of WCHAR16 elements in 'name' available to write.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! WCHAR16 name[MLPI_CONTAINER_NAME_MAX_LENGTH+1] = {'\0'};
//! MLPIRESULT result = mlpiContainerGetName(connection, hContainer, name, _countof(name));
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! printf("\nName of container: %s", W2A16(name));
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetName(const MLPIHANDLE connection, const MlpiContainerHandle handle, WCHAR16 *name, const ULONG numElements);


//! @ingroup ContainerLibAux
//! Use this function to read various pieces of information about your container. This includes time of creation, size, number of items,
//! and type (read/write). You have to pass the container handle.
//!
//! @param[in]    connection  Handle for multiple connections.
//! @param[in]    handle      Handle of the container. Use @ref mlpiContainerCreate to create a container.
//! @param[out]   info        Pointer to struct which will receive the container information.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read container information
//! MlpiContainerInformation info;
//! memset(&info, 0, sizeof(info));
//!
//! MLPIRESULT result = mlpiContainerGetInformation(connection, hContainer, &info);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! // print container information
//! printf("\nname: %s", W2A16(info.name));
//! printf("\nnumItems: %d", info.numItems);
//! printf("\ndataSize: %d", info.dataSize);
//! printf("\nnumElementsTagList: %d", info.numElementsTagList);
//! printf("\naccessFlag: %d", info.accessFlag);
//! printf("\ndateTime: %04d/%02d/%02d %02d:%02d:%02d.%04d",
//!   info.dateTime.year,
//!   info.dateTime.month,
//!   info.dateTime.day,
//!   info.dateTime.hour,
//!   info.dateTime.minute,
//!   info.dateTime.second,
//!   info.dateTime.milliSecond);
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetInformation(const MLPIHANDLE connection, const MlpiContainerHandle handle, MlpiContainerInformation *info);


//! @ingroup ContainerLibAux
//! This function returns the tag list of a container. This is the same tagList that was used to create the container with the function
//! @ref mlpiContainerCreate.
//!
//! @param[in]    connection  Handle for multiple connections.
//! @param[in]    handle      Handle of the container. Use @ref mlpiContainerCreate to create a container.
//! @param[out]   tagList     String where the tag list will be stored.
//! @param[in]    numElements Number of WCHAR16 elements in 'name' available to write.
//! @return                   Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read container information
//! MlpiContainerInformation info;
//! memset(&info, 0, sizeof(info));
//!
//! MLPIRESULT result = mlpiContainerGetInformation(connection, hContainer, &info);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! // allocate enough memory and read item information
//! WCHAR16 *tagList = new WCHAR16[info.numElementsTagList+1];
//! mlpiContainerGetTagList(connection, hContainer, tagList, info.numElementsTagList+1);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   delete [] tagList;
//!   return result;
//! }
//!
//! // print the tagList
//! printf("\n%s", W2A16(tagList));
//!
//! delete [] tagList;
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetTagList(const MLPIHANDLE connection, const MlpiContainerHandle handle, WCHAR16 *tagList, const ULONG numElements);


//! @ingroup ContainerLibAux
//! This function returns the item information of a container as an array of struct. Each element in the array
//! contains information about a single item within the container. This includes information about type, size
//! and offset of the item within the container data stream. You can use this function to read out information
//! about the memory layout of the container data stream that gets returned by @ref mlpiContainerUpdate.
//!
//! @param[in]    connection      Handle for multiple connections.
//! @param[in]    handle          Handle of the container. Use @ref mlpiContainerCreate to create a container.
//! @param[out]   info            Pointer to an array where the requested information about the items will be stored.
//! @param[in]    numElements     Number of elements in 'info' available to write.
//! @param[out]   numElementsRet  Returns the number of elements actually read. This is the number of items in the container.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read container information
//! MlpiContainerInformation info;
//! memset(&info, 0, sizeof(info));
//!
//! MLPIRESULT result = mlpiContainerGetInformation(connection, hContainer, &info);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! // allocate enough memory and read item information
//! ULONG numElementsRet = 0;
//! MlpiContainerItemInformation *itemInfos = new MlpiContainerItemInformation[info.numItems];
//! mlpiContainerGetItemInformation(connection, hContainer, itemInfos, info.numItems, &numElementsRet);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   delete [] itemInfos;
//!   return result;
//! }
//!
//! // loop over all items and print information of each item in the container
//! printf("\nOffset | Size | Type ");
//! printf("\n-------+------+------");
//! for (ULONG i=0; i<numElementsRet; i++) {
//!   printf("\n %6d  %5d  %4d", itemInfos[i].offset, itemInfos[i].dataSize, itemInfos[i].type);
//! }
//!
//! delete [] itemInfos;
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetItemInformation(const MLPIHANDLE connection, const MlpiContainerHandle handle, MlpiContainerItemInformation *info, const ULONG numElements, ULONG *numElementsRet);


//! @ingroup ContainerLibAux
//! This function returns the item information of a single item in a container. It returns information
//! about type, size and offset of the item within the container data stream. You can use this function to read out information
//! about the memory layout of the container data stream that gets returned by @ref mlpiContainerUpdate.
//! This is similar to the function @ref mlpiContainerGetItemInformation. But only a single item is returned including its tag.
//!
//! @param[in]    connection      Handle for multiple connections.
//! @param[in]    handle          Handle of the container. Use @ref mlpiContainerCreate to create a container.
//! @param[in]    index           Index of the item to return information for. Use @ref mlpiContainerGetInformation to read the
//!                               number of available items in a container.
//! @param[out]   tag             String where the tag of the item will be stored.
//! @param[in]    numElements     Number of WCHAR16 elements in 'tag' available to write.
//! @param[out]   info            Pointer to a struct where the requested information about the item will be stored.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! // read container information
//! MlpiContainerInformation info;
//! memset(&info, 0, sizeof(info));
//!
//! MLPIRESULT result = mlpiContainerGetInformation(connection, hContainer, &info);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!   return result;
//! }
//!
//! // loop over all items and print information of each item in the container
//! printf("\nOffset | Size | Type | Tag         ");
//! printf("\n-------+------+------+-------------");
//!
//! for (ULONG i=0; i<info.numItems; i++) {
//!   MlpiContainerItemInformation itemInfo;
//!   memset(&itemInfo, 0, sizeof(itemInfo));
//!   WCHAR16 tag[512] = {'\0'};
//!
//!   // read item info
//!   MLPIRESULT result = mlpiContainerGetSingleItemInformation(connection, hContainer, i, tag, _countof(tag), &itemInfo);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     return result;
//!   }
//!
//!   // print it
//!   printf("\n %6d  %5d  %4d  %s", itemInfo.offset, itemInfo.dataSize, itemInfo.type, W2A16(tag));
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetSingleItemInformation(const MLPIHANDLE connection, const MlpiContainerHandle handle, const ULONG index, WCHAR16 *tag, const ULONG numElements, MlpiContainerItemInformation *info);


//! @ingroup ContainerLibAux
//! This function returns the total number of containers created on the device. Also including containers of other applications.
//!
//! @param[in]    connection      Handle for multiple connections.
//! @param[out]   number          Returns the total number of allocated containers on the device.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numContainers = 0;
//!
//! // read how many containers are available
//! MLPIRESULT result = mlpiContainerGetNumberOfContainer(connection, &numContainers);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!    return result;
//! }
//! printf("\nFound %d container(s) on the device", numContainers);
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetNumberOfContainer(const MLPIHANDLE connection, ULONG *number);


//! @ingroup ContainerLibAux
//! Returns the handles of all containers that were previously created on the device.
//!
//! @param[in]    connection      Handle for multiple connections.
//! @param[out]   handles         Pointer to an array which will receive all container handles of the device.
//! @param[in]    numElements     Number of elements in 'handles' available to write.
//! @param[out]   numElementsRet  Returns the number of elements actually read. This is the number of handles returned.
//! @return                       Return value indicating success (>=0) or error (<0).
//!
//! @par Example:
//! @code
//! ULONG numContainers = 0;
//!
//! // read how many containers are available
//! MLPIRESULT result = mlpiContainerGetNumberOfContainer(connection, &numContainers);
//! if (MLPI_FAILED(result)) {
//!   printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!    return result;
//! }
//! printf("\nFound %d container(s) on the device", numContainers);
//!
//! // get the handles of all containers and print the names of the containers
//! if (numContainers) {
//!   MlpiContainerHandle *containers = new MlpiContainerHandle[numContainers];
//!
//!   ULONG numElementsRet = 0;
//!   MLPIRESULT result = mlpiContainerGetHandlesOfContainer(connection, containers, numContainers, &numElementsRet);
//!   if (MLPI_FAILED(result)) {
//!     printf("\ncall of MLPI function failed with 0x%08x!", (unsigned)result);
//!     delete [] containers;
//!     return result;
//!   }
//!
//!   // print the name of each container
//!   for (ULONG i=0; i<numContainers; i++) {
//!     WCHAR16 name[K_CONTAINER_NAME_MAX_LENGTH+1];
//!     MLPIRESULT result = mlpiContainerGetName(connection, containers[i], name, _countof(name));
//!     if (MLPI_SUCCEEDED(result)) {
//!       printf("\n%d: %s", i, W2A16(name));
//!     } else {
//!       printf("\n%d: could not retrieve name of container", i);
//!     }
//!   }
//!
//!   delete [] containers;
//! }
//! @endcode
MLPI_API MLPIRESULT mlpiContainerGetHandlesOfContainer(const MLPIHANDLE connection, MlpiContainerHandle *handles, const ULONG numElements, ULONG *numElementsRet);



#ifdef __cplusplus
}
#endif



#endif // endof: #ifndef __MLPICONTAINERLIB_H__
