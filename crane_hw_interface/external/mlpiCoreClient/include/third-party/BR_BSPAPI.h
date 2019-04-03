/**************************************************************************/ /**
*
*             Bosch Rexroth. The Drive & Control Company
*
*             Electric Drives and Controls
*              
* \file       BR_BSPAPI.h
*            
* \brief      constants, type definitions and function prototypes for
*             BRC API to the board support paccage
*       
* \version    02V01  
*              
* \date       2012-06-22
*              
* \author     DCC/EAH2
* 
*******************************************************************************/
#ifndef BR_BSPAPI_H
#define BR_BSPAPI_H


#if __cplusplus
extern "C"
{
#endif


#define BR_BSPAPI_HEADER_VERSION   13L  

#if defined(__VXWORKS__)
#ifndef _WRS_KERNEL
#define BRBspApi BRBspApiUser
#endif /*not defined _WRS_KERNEL*/
#elif defined (__gnu_linux__)
#endif /*defined(__VXWORKS__)*/ 

/*******************************************************************************
***************************************************************************/ /**
* \defgroup   misc  miscelaneous
*
* @{
*
* \brief      everything, that does not fit in the other modules
********************************************************************************
*******************************************************************************/

/**@}*/ /*end group misc********************************************************
*******************************************************************************/
  

/*******************************************************************************
***************************************************************************/ /**
* \defgroup   hardware_types  hardware types
*
* @{
*
* \brief      Some constants that define the hardware target: see HWTYPE_...
********************************************************************************
*******************************************************************************/
  
#define HWTYPE_UNKNOWN    0   /**<unknown hardware type                       */
#define HWTYPE_L40        1   /**<CML40.2                                     */
#define HWTYPE_L45        2   /**<CML45                                       */
#define HWTYPE_L65        3   /**<CML65                                       */
#define HWTYPE_L25        4   /**<CML25                                       */
#define HWTYPE_L85        5   /**<CML85                                       */
#define HWTYPE_VEP_4      6   /**<VEPxx.4                                     */
#define HWTYPE_VXWIN      7   /**<VxWin system                                */
#define HWTYPE_HCQ02_1    8   /**<HCQ02.1 und HCT02.1                         */
#define HWTYPE_L65_2      9   /**<CML65.2                                     */
#define HWTYPE_XM21      10   /**<XM21                                        */
#define HWTYPE_PR41      11   /**<PR41                                        */
#define HWTYPE_XM12      12   /**<XM12                                        */
#define HWTYPE_XM22      13   /**<XM22                                        */
#define HWTYPE_L75       14   /**<CML75                                       */
#define HWTYPE_XM42      15   /**<XM42                                        */
#define HWTYPE_XM41      16   /**<XM41                                        */

/**@}*/ /*end group hardware_types**********************************************
*******************************************************************************/


/*******************************************************************************
***************************************************************************/ /**
* \defgroup   extmod_api  extension module api
* @{
*
* See also \ref bsp_api_commands_extmod
*
* Use BRBspApi command BRBSPAPI_EXTMOD_API to execute commands on the br
* extension modules. 2 data structures of type BRExtModApi_t have
* to be delivered to the BRBspApi. 1 for the write operation and 1 for the read
* operation.
*
* Purpose of structure elements:
*
* - ulBRdeviceHandle: Handle to the device. \n
*                     Set this parameter in the pvInData structure.
*                     Don't care in the pvOutData structure.
* - ulCommand:        Command to be performed (see constants BREXTMOD_API_COMMAND_...). \n
*                     Set this parameter in the pvInData structure.
*                     Don't care in the pvOutData structure.
* - ulError:          Error value returned by ident function (see constants BREXTMOD_API_ERROR_...).
*                     Error is returned in this parameter in the pvOutData structure.
*                     Don't care in the pvInData structure.
* - ulDataLen:        Length of data buffer for operation. \n
* - pvData:           Pointer to data buffer for different use, depending on the command
*                     If ulDataLen == 0 -> pcData may also be NULL.
*
* The routines for extension module api modules must be provided by the module driver.
* The board support package calls a function pointer that has to be initialized by the module driver.
* This can be done by using the BRBspApi command BRBSPAPI_EXTMOD_API_REGISTER. This
* command uses a data structure called BRExtModApiRegister_t.
* For more information see BRBspApi command description.
* 
* \code
* 
* How to use spezial commands:
* 
* BREXTMOD_API_COMMAND_EEPROM_PROG:
* =================================
* This command is used to program logicware eeprom(s) of the addressed device
* (extension module or main unit).
* 
* Usage:
* 
* pvInData
* --------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_EEPROM_PROG
* ulError:            not used
* ulDataLen:          not used
* pvData:             pointer to null terminated string (filename of update file)
* 
* pvOutData
* ---------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_EEPROM_PROG
* ulError:            error code (BREXTMOD_API_...)
* ulDataLen:          not used
* pvData:             not used
* 
* BREXTMOD_API_COMMAND_EEPROM_VERS:
* =================================
* This command is used to retrieve the version info of a logicware file.
* The version information is delivered as null terminated string. If the logicware file
* contains data for multiple parts, the information of all parts is delivered.
* The user has to supply the memory for the string. The size of memory is defined by
* BREXTMOD_API_COMMAND_EEPROM_VERS_LENGHT.
* 
* Usage:
* 
* pvInData
* --------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_EEPROM_VERS
* ulError:            not used
* ulDataLen:          size of buffer for version information (BREXTMOD_API_COMMAND_EEPROM_VERS_LENGHT)
* pvData:             pointer to null terminated string (filename of update file)
* 
* pvOutData
* ---------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_EEPROM_VERS
* ulError:            error code (BREXTMOD_API_...)
* ulDataLen:          not used
* pvData:             pointer to data buffer to hold the version information
* 
* 
* The version information is delivered as string, because future applications
* cannot be predicted. It is not clear, if the version information format is the same
* in all future applications.
* Syntax:
* - Name of the function with a leading hash sign
* - optional whitespaces (or none)
* - colon
* - optional whitespaces (or none)
* - version string
* - \n
* 
* example of version information (main unit XM21):
* #S20V : 01.00.00.00\n
* #S3Co : 04.09.00.00\n
* #S3Dm : 00.02.02.00\n
* ...
* 
* 
* BREXTMOD_API_COMMAND_FALLBACK_CHK:
* ==================================
* This command is used to check, if the device is in fallback mode (golden image) or not
* 
* Usage:
* 
* pvInData
* --------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_FALLBACK_CHK
* ulError:            not used
* ulDataLen:          not used
* pvData:             not used
* 
* pvOutData
* ---------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_FALLBACK_CHK
* ulError:            error code (BREXTMOD_API_...)
* ulDataLen:          not used
* pvData:             pointer to unsigned int: return value (see defines BREXTMOD_API_FALLBACK_xxx)
* 
* 
* BREXTMOD_API_COMMAND_RESET:
* ===========================
* This command is used to reset the device
* 
* Usage:
* 
* pvInData
* --------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_RESET
* ulError:            not used
* ulDataLen:          not used
* pvData:             not used
* 
* pvOutData
* ---------
* ulBRdeviceHandle:   device handler = identifies the device
* ulCommand:          BREXTMOD_API_COMMAND_RESET
* ulError:            not used
* ulDataLen:          not used
* pvData:             not used
* 
* 
* \endcode
********************************************************************************
*******************************************************************************/
typedef struct
{
  unsigned long ulBRdeviceHandle;        /**<handle of the device*/
  unsigned long ulCommand;               /**<command see BREXTMOD_API_COMMAND_...*/
  unsigned long ulError;                 /**<error value of extension module api (see constants)*/
  unsigned long ulDataLen;               /**<length of data, if 0 pvData is not used*/
  void *        pvData;                  /**<pointer to data*/
}
BRExtModApi_t;

typedef struct
{
  unsigned long ulBRdeviceHandle;        /**<Handle to the of br device*/
  /**function pointer to the extension module api function*/
  unsigned long (*BRExtModApi)(BRExtModApi_t * InData, BRExtModApi_t* OutData);
}
BRExtModApiRegister_t;

/*extension module api commands*/               /*format of data*/
#define BREXTMOD_API_COMMAND_UNDEF         0    /**<command undefined*/
#define BREXTMOD_API_COMMAND_EEPROM_PROG   1    /**<program logicware eeprom(s)*/
#define BREXTMOD_API_COMMAND_EEPROM_VERS   2    /**<read version information of logicware file*/
#define BREXTMOD_API_COMMAND_FALLBACK_CHK  3    /**<check if device is in fallback mode*/
#define BREXTMOD_API_COMMAND_RESET         4    /**<reset the device*/
#define BREXTMOD_API_COMMAND_LAST          5    /**<last command (dummy)*/

/*return values of extension module api*/
#define BREXTMOD_API_OK                    0x00 /**<no error*/
#define BREXTMOD_API_ERROR_COMMAND_UNKNOWN 0x01 /**<extension module api command unknown*/
#define BREXTMOD_API_ERROR_HANDLE_NOMATCH  0x02 /**<selected handle does not exist*/
#define BREXTMOD_API_ERROR_READ            0x03 /**<read error*/
#define BREXTMOD_API_ERROR_WRITE           0x04 /**<write error*/
#define BREXTMOD_API_ERROR_VALUE_NOTSET    0x05 /**<value not set*/
#define BREXTMOD_API_ERROR_VALUE_NOTVALID  0x06 /**<value not valid*/
#define BREXTMOD_API_ERROR_NOFUNCPTR       0x07 /**<extension module api function pointer not initialized*/
#define BREXTMOD_API_ERROR_PARAMETER       0x08 /**<extension module api parameter error*/
#define BREXTMOD_API_ERROR_MOD_NOTPRESENT  0xff /**<no module present at this module location*/

#define BREXTMOD_API_FALLBACK_NOTACTIVE    0    /**<return value for command BREXTMOD_API_COMMAND_FALLBACK_CHK*/
#define BREXTMOD_API_FALLBACK_ACTIVE       1    /**<return value for command BREXTMOD_API_COMMAND_FALLBACK_CHK*/

/*version information of fw-file*/
#define BREXTMOD_API_COMMAND_EEPROM_VERS_LENGHT 128    /**<max lenght of version information*/
#define VER_KEYWORD_UDEF          "#UDEF"       /**<undefined keyword used as end of list*/
#define VER_KEYWORD_S20           "#S20V"       /**<keyword for s20 version*/
#define VER_KEYWORD_SE3_MA        "#S3MA"       /**<keyword for sercos 3 master version*/
#define VER_KEYWORD_SE3_SL        "#S3SL"       /**<keyword for sercos 3 slave version*/
#define VER_KEYWORD_SE3_DMA       "#S3DM"       /**<keyword for sercos 3 dma version*/
#define VER_KEYWORD_CAN           "#CANV"       /**<keyword for can version*/


/**@}*/ /*end group extmod_api**************************************************
*******************************************************************************/


/*******************************************************************************
***************************************************************************/ /**
* \defgroup   ident_information  ident information
* @{
*
* \brief      ident information support
*
* See also \ref bsp_api_commands_ident
*
* Ident informations are stored in some bosch rexroth pci devices and in the
* main unit.
*
* Use BRBspApi command BRBSPAPI_IDENT_INFO to read and/or write ident
* information of a BR pci device. 2 data structures of type BRIdentInfo_t have
* to be delivered to the BRBspApi. 1 for the write operation and 1 for the read
* operation.
*
* Purpose of structure elements:
*
* - ulBRdeviceHandle: Handle to the device. \n
*                     Has to be the same in both data structures.
* - ulCommand:        Command to be performed (see constants IDENTINFO_...). \n
*                     Has to be the same in both data structures.
* - ulError:          Error value returned by ident function (see constants
*                     IDENTFUNC_...).
* - ulDataLen:        Length of data buffer for read/write operation. \n
*                     If ulDataLen == 0 -> no data is written/read.
* - pcData:           Pointer to data buffer for read/write operation
*                     (normally a null terminated string). \n
*                     !!!Caution: buffer must be of size ulDataLen or bigger!!!
*                     If ulDataLen == 0 -> pcData may also be NULL.
*                     Minimum size is dependent on data stored in pci-device.
*
* The board support package itself only provides the ident routines for the main
* unit. The ident routines for extension modules and onboard modules must be
* provided by the module driver or module base driver. The board support package
* calls a function pointer that has to be initialized by the module driver. This
* can be done by using the BRBspApi command BRBSPAPI_IDENTFUNC_REGISTER. This
* command uses a data structure called BRIdentInfoFunc_t.
* For more information see BRBspApi command description.
********************************************************************************
*******************************************************************************/

#define TYPE_CODE_SIZE              40   /**<number of type code chars*/
#define IDENT_INFO_STRING_MAX_LEN  100   /**<maximum length of ident info strings*/
#define IDENT_INFO_SYSDATA_SIZE     24   /**<number of system data bytes*/

typedef struct
{
  unsigned long ulBRdeviceHandle;        /**<handle of the device*/
  unsigned long ulCommand;               /**<command see IDENTINFO_COMMANDS*/
  unsigned long ulError;                 /**<error value of ident function (see constants)*/
  unsigned long ulDataLen;               /**<length of data, if 0 no data is transferred*/
  char*         pcData;                  /**<pointer to data*/
}
BRIdentInfo_t;

typedef struct
{
  unsigned long ulBRdeviceHandle;        /**<Handle to the of br device*/
  /**function pointer to the ident function*/
  unsigned long (*BRIdentInfoXChange)(BRIdentInfo_t * InData, BRIdentInfo_t* OutData);
}
BRIdentInfoFunc_t;

/*identinfo commands*/                   /*format of data*/
#define IDENTINFO_UNDEF            0     /**<command undefined*/
#define IDENTINFO_SERIAL_NR        1     /**<serial number: null terminated string e.g. "123456"*/
#define IDENTINFO_MATERIAL_NR      2     /**<material number: null terminated string e.g. "R911123456"*/
#define IDENTINFO_MATERIAL_IND_NR  3     /**<material index: null terminated string e.g. "AA1"*/
#define IDENTINFO_TYPE_CODE        4     /**<type code: null terminated string e.g. "CML65-Typecode"*/
#define IDENTINFO_WORK_TIME        5     /**<working time: null terminated string (working time in seconds), read only*/
#define IDENTINFO_ETH_MAC          6     /**<ethernet mac address: null terminated string */
#define IDENTINFO_BLP_NR           7     /**<board number: null terminated string */
#define IDENTINFO_BLP_IND_NR       8     /**<board index: null terminated string */
#define IDENTINFO_SYSTEM_DATA      9     /**<system specific data, upper 16bits of length parameter specify offset into data area*/
#define IDENTINFO_ETH_MAC_1        10     /**<ethernet mac address: null terminated string */
#define IDENTINFO_ETH_MAC_2        11     /**<ethernet mac address: null terminated string */
#define IDENTINFO_OPTICAL          12     /**<optical identification of device: char, see  IDENTINFO_OPTICAL_xxx */

#define IDENTINFO_LAST             13     /**<last command (dummy)*/

/*return values for ident functions*/
#define IDENTFUNC_OK                    0x00 /**<no error*/
#define IDENTFUNC_ERROR_HANDLE_NOMATCH  0x01 /**<selected handle does not exist*/
#define IDENTFUNC_ERROR_READ            0x02 /**<ident info read error*/
#define IDENTFUNC_ERROR_WRITE           0x03 /**<ident info write error*/
#define IDENTFUNC_ERROR_VALUE_NOTSET    0x04 /**<ident info not programmed*/
#define IDENTFUNC_ERROR_VALUE_NOTVALID  0x05 /**<ident info not valid*/
#define IDENTFUNC_ERROR_NOFUNCPTR       0x06 /**<ident info function pointer not initialized*/
#define IDENTFUNC_ERROR_NOTAVAILABLE    0x07 /**<ident info is not available*/
#define IDENTFUNC_ERROR_NO_STACK        0x08 /**<stack not loaded*/
#define IDENTFUNC_ERROR_STRINGSIZE      0x09 /**<strings size too small*/
#define IDENTFUNC_ERROR_RW              0x0A /**<ident info read and write error*/
#define IDENTFUNC_ERROR_PARAMETER       0x0B /**<ident info parameter error*/
#define IDENTFUNC_ERROR_MOD_NOTPRESENT  0xff /**<no module present at this module location*/


/*identinfo optical commands*/
#define IDENTINFO_OPTICAL_OFF           0 /**<ident info optical: switch optical sequence off*/
#define IDENTINFO_OPTICAL_ON            1 /**<ident info optical: switch optical sequence on*/
#define IDENTINFO_OPTICAL_TRIG          2 /**<ident info optical: trigger optical sequence*/
#define IDENTINFO_OPTICAL_NOP           3 /**<ident info optical: no operation/no command*/


/**@}*/ /*end group ident_information*******************************************
*******************************************************************************/

  
/*******************************************************************************
***************************************************************************/ /**
* \defgroup   br_devices  BR devices
*
* @{
*
* \brief      support for bosch rexroth devices
*
* All bosch rexroth devices are scanned during startup. Their data is stored in
* a table. All routines, that handle the br devices need a handler to address the
* device. This handler is stored in the table.
* 
* How to work with the br devices:
* 
* 
* First of all, find out how many devices of a certain class are present. Use
* BRBspApi command BRBSPAPI_DEVICESOFCLASS_NR_GET. 
* Then iterate over the found number of devices an retrieve the data. Use
* BRBspApi command BRBSPAPI_DEVICEBYCLASS_INFO_GET.
* Now you have all information needed:
* - handler: ulBRdeviceHandle \n
*   The handler is needed for all routines that address the br devices.
* - device location: ulBRdeviceLocation \n
*   The device location tells you, where the device is located physically
* - device type: ulBRdeviceType \n
*   Usually this is the interface to the device. E.g. pci
* - device class: ulBRdeviceClass \n
*   Usually this describes the functionality of the device. E.g. main unit, sercos, netx etc.
* - pointer to extended information: pvBRdeviceInfoExt \n
*   This pointer point to a data structure containing extended information of the device.
*   The type of this structure varies an is dependent on the device type. For example, if
*   the device type is BR_DEVICETYPE_PCI, this pointer points to data structure BRdeviceInfoPci_t.
* - pointer to the ident info exchange routine: IdentInfoXChangeRtn \n
*   This pointer is only used bsp internal. To exchange ident information with the device, the
*   BRBspApi command BRBSPAPI_IDENT_INFO must be used.
* 
* Example code:
* 
* \code
* 
* int i;
* unsigned long ulNumDevices;
* unsigned long ulData[2];
* BRdeviceInfo_t * pBRdeviceInfo;
* 
* //get the number of all br devices
* ulData[0] = BR_DEVICECLASS_UNKNOWN;  //second parameter of BRBspApi
* if(BRBspApi(BRBSPAPI_DEVICESOFCLASS_NR_GET,(void*)ulData,(void*)&ulNumDevices) != BRBSPAPI_OK)
*   ERROR-Handler(API-error)
* 
* //iterate over all found devices
* for(i=0; i<ulNumDevices; i++)
* {
*   ulData[1] = i;                     //instance of device
*   if(BRBspApi(BRBSPAPI_DEVICEBYCLASS_INFO_GET,(void*)ulData,(void*)&pBRdeviceInfo) != BRBSPAPI_OK)
*     ERROR-Handler(API-error)
*     
*   //now all information can be found in pBRDeviceInfo
* }
* 
* 
* //get number of dedicated br devices e.g. sercos device
* //get the number of all br devices
* ulData[0] = BR_DEVICECLASS_SERCOS3_CTRL;  //second parameter of BRBspApi
* if(BRBspApi(BRBSPAPI_DEVICESOFCLASS_NR_GET,(void*)ulData,(void*)&ulNumDevices) != BRBSPAPI_OK)
*   ERROR-Handler(API-error)
* 
* //iterate over all found devices
* for(i=0; i<ulNumDevices; i++)
* {
*   ulData[1] = i;                     //instance of device
*   if(BRBspApi(BRBSPAPI_DEVICEBYCLASS_INFO_GET,(void*)ulData,(void*)&pBRdeviceInfo) != BRBSPAPI_OK)
*     ERROR-Handler(API-error)
*     
*   //now all information can be found in pBRDeviceInfo
* }
* 
* \endcode
* 
* 
* 
* Interrupt handling of br devices (old version):
* ===============================================
* 
* 
* Old command BRBSPAPI_DEVICES_MSI_CONFIG (only MSI)
* --------------------------------------------------
* 
* For br devices only message signalled interrupts are supported. To configure
* the interrupts, the BRBspApi command BRBSPAPI_DEVICES_MSI_CONFIG has to be used.
* This command is still supported for compability purpose. For future please use 
* command BRBSPAPI_DEVICES_MSI_CONF. The new command additionally supports MSI-X.
* 
* On the intel platform message signalled interrupts are handled by interrupt
* vectors. A device, that sends an msi sends a vector to the local apic, which is
* located in the cpu. An interrupt vector is an 8 bit value. The upper 4 bit define
* the interrupt priority group. The lower 4 bit define one of 16 vectors inside this
* interrupt priority group.
* - Vectors 0x00..0x1F are reserved for system exceptions like pgae fault etc.
* - Vectors 0x20..0x3F are configured by our bsp to be used by the ioapic. The ioapic
*                      handles the legacy interrupts.
* - Vectors 0x40..0x5F are configured by our bsp to be used by the windriver vxbus
*                      dynamic vectors. Usually devices like ethernet controller,
*                      ata controller, sdcard controller etc. use interrupts out of
*                      this range.
* - Vectors 0x60..0xDF are configured by our bsp to be used by the br devices.
* - Vectors 0xE0..0xEF are configured by our bsp to be used for high prior system interrupts
*                      like inter processor interrupts or local apic interrupts
* - Vectors 0xF0..0xFF are reserved for very high system interrupts like power fail.
* 
* The priority increases from 0x20 to 0xFF (0x20 lowest priority, 0xFF highest priority).
* The priority of priority group 0x00..0x20 depends on the individual exception and can not
* be changed (see intel specification). Interrupts inside one priority group
* (e.g. between 0x80 and 0x8F) can not interrupt each other (no nested interrupts).
* 
* The interrupts for br devices are choosen to be the most prior interrupt that we could get.
* We have 8 interrupt priority groups. In our BRBspApi we use abstract interrupt priorities:
* BR_DEVICE_INTPRIO_1 to BR_DEVICE_INTPRIO_8, with 1 is the highes priority and 8 the lowest.
* 
* The restrictions are:
* A br device only can use up to 16 interrupts inside one priority group.
* Each br device has to use its own interrupt priority group exclusively.
* The priority of the msi's of one device has to be configured first. The isr(s) can be
* connected later. Once the priority is configred, it can not be changed any more until
* the next reboot.
* 
* When the command BRBSPAPI_DEVICES_MSI_CONFIG is used the first time for a device, the 
* msi priority must be configured (BR_DEVICE_INTPRIO_1 to BR_DEVICE_INTPRIO_8) and the first
* interrupt service routine may or may not be connected. If the index of the isr is set to
* BR_DEVICE_INTINDEX_UNDEF, no isr will be connected, only the msi will be configured and enabled.
* !!! It is imporant, that the device does not generate an msi before the aproprate isr is
* connected.
* Later the command BRBSPAPI_DEVICES_MSI_CONFIG can be called again, to connect or exchange
* the remaining isr's. In that case the priority must be configured to BR_DEVICE_INTPRIO_NOCHANGE
* or the same priority must be used as the first time. Otherwise the api will return an error.
* 
* The msi index in the data structure defines the number, the interrupt has at the device.
* 
* 
* How to use the BRBspApi:
* ------------------------
* 
* It is assumed, that we already have retrieved the device information (BRdeviceInfo_t),
* as described above. The reference to the device always is the handle (ulBRdeviceHandle).
* The device driver has to deliver a structure of the type BRdeviceMsiConfig_t. Its elements
* have to be initialized like this:
* 
* ulBRdeviceHandle: handler to the br device
* ulBRdeviceMsiCpu: index of dedicated cpu (only on multicore systems)
* ulBRdeviceMsiPrio: desired interrupt priority group for the device (BR_DEVICE_INTPRIO_NOCHANGE if only connect isr)
* ulBRdeviceMsiIndex: index of the msi to connect the isr to (BR_DEVICE_INTINDEX_UNDEF, if no isr should be connected)
* fpIsrHandler: pointer to isr
* parg: pointer to arguments for isr
* 
* Example code for a device using 3 interrupts:
* ---------------------------------------------
* 
* \code
*  
* void isr1(void)
* { return; }
* 
* void isr2(void)
* { return; }
*
* void isr3(void)
* { return; }
* 
* void myDeviceMsiConfig(void)
* {
*   int i;
*   unsigned long ulErrorOfSubroutine;
*   BRdeviceMsiConfig_t BRdeviceMsiConfig;
*   
*   //configure interrupt priority
*   //////////////////////////////
*   BRdeviceMsiConfig.ulBRdeviceHandle = ulBrDeviceHandle;           //device handler retrieved before
*   BRdeviceMsiConfig.ulBRdeviceMsiPrio = BR_DEVICE_INTPRIO_3;       //3rd priority group
*   BRdeviceMsiConfig.ulBRdeviceMsiIndex = BR_DEVICE_INTINDEX_UNDEF; //no isr connect
*   BRdeviceMsiConfig.ulBRdeviceMsiCpu = 0;                          //send MSI to cpu 0
*   BRdeviceMsiConfig.fpIsrHandler = 0;                              //no isr
*   BRdeviceMsiConfig.parg = 0;                                      //no arguments
*   if(BRBspApi(BRBSPAPI_DEVICES_MSI_CONFIG,(void*)&BRdeviceMsiConfig,&ulErrorOfSubroutine) != BRBSPAPI_OK)
*   {
*     ERROR-Handler(API-error)
*     printf("\nErrorcode of subroutine: %d",ulErrorOfSubroutine);
*   }
*   
*   //connect isr's
*   ///////////////
*   BRdeviceMsiConfig.ulBRdeviceMsiPrio = BR_DEVICE_INTPRIO_NOCHANGE; //only connect isr's
*   BRdeviceMsiConfig.ulBRdeviceMsiIndex = 0;                         //1st msi to connect
*   BRdeviceMsiConfig.fpIsrHandler = (void*)isr1;                     //1st isr
*   BRdeviceMsiConfig.parg = 0;                                       //isr has no arguments
*   if(BRBspApi(BRBSPAPI_DEVICES_MSI_CONFIG,(void*)BRdeviceMsiConfig,&ulErrorOfSubroutine) != BRBSPAPI_OK)
*   {
*     ERROR-Handler(API-error)
*     printf("\nErrorcode of subroutine: %d",ulErrorOfSubroutine);
*   }
*   
*   BRdeviceMsiConfig.ulBRdeviceMsiPrio = BR_DEVICE_INTPRIO_NOCHANGE; //only connect isr's
*   BRdeviceMsiConfig.ulBRdeviceMsiIndex = 1;                         //2nd msi to connect
*   BRdeviceMsiConfig.fpIsrHandler = (void*)isr2;                     //2nd isr
*   BRdeviceMsiConfig.parg = 0;                                       //isr has no arguments
*   if(BRBspApi(BRBSPAPI_DEVICES_MSI_CONFIG,(void*)BRdeviceMsiConfig,&ulErrorOfSubroutine) != BRBSPAPI_OK)
*   {
*     ERROR-Handler(API-error)
*     printf("\nErrorcode of subroutine: %d",ulErrorOfSubroutine);
*   }
*   
*   BRdeviceMsiConfig.ulBRdeviceMsiPrio = BR_DEVICE_INTPRIO_NOCHANGE; //only connect isr's
*   BRdeviceMsiConfig.ulBRdeviceMsiIndex = 2;                         //3rd msi to connect
*   BRdeviceMsiConfig.fpIsrHandler = (void*)isr3;                     //3rd isr
*   BRdeviceMsiConfig.parg = 0;                                       //isr has no arguments
*   if(BRBspApi(BRBSPAPI_DEVICES_MSI_CONFIG,(void*)BRdeviceMsiConfig,&ulErrorOfSubroutine) != BRBSPAPI_OK)
*   {
*     ERROR-Handler(API-error)
*     printf("\nErrorcode of subroutine: %d",ulErrorOfSubroutine);
*   }
* }
* 
* \endcode
* 
* Now 3 isr are connected to the specified vectors and the device is configured to use
* those vectors. The interrupts are enabled in the system.
* 
* 
* 
* New command BRBSPAPI_DEVICES_MSI_CONF (MSI and MSI-X)
* -----------------------------------------------------
* 
* The new command additionally allows to configure MSI-X. MSI-X means, that each
* interrupt of a device may use a unique interrupt vector. The interrupts may be
* of different priority groups. To support this, the new command
* BRBSPAPI_DEVICES_MSI_CONF uses new data structures for pcInData and pvOutData
* (BRdeviceMsiConf_t and BRdeviceMsiInfo_t). Compared to the old data structur,
* additional variables are needed:
* 
* ulMode: specifies, if MSI or MSI-X is to be configured, or if the api is only
*         used to read, which mode is available (none, MSI, MSI-X). see constants
*         BR_DEVICE_MSIMODE_xxx.
*  
* ulBRdeviceMsiSubPrio: ulMode MSI:
*                       specifies a sub priority inside a priority group (1..16)
*                       1 is the highest subpriority, 16 the lowest.
*                       ulMode MSI-X:
*                       don't care (set to 0)
*                       
* For MSI-X ulBRdeviceMsiPrio and ulBRdeviceMsiSubPrio together will specify the
* interrupt vector. For example if BR_DEVICE_INTPRIO_1 spezifies vector 0xD0 on
* a dedicated intel hardware, the combination of
* ulBRdeviceMsiPrio = BR_DEVICE_INTPRIO_1
* ulBRdeviceMsiSubPrio = 2
* configures the interrupt vector to 0xDE (2nd highest priority, highest would
* be 0XDF).
* 
* The command BRBSPAPI_DEVICES_MSI_CONF returns an information in the data
* structure BRdeviceMsiInfo_t:
* 
* ulError: returns an error value or success (see error codes BR_DEVICE_xxx)
* 
* ulModeAvailable: returns which modes are available (see BR_DEVICE_MSIMODE_xxx)
*                  only if ulMode == BR_DEVICE_MSIMODE_INFO, else don't care
* 
* ulModeConfigured: returns which mode is configured (see BR_DEVICE_MSIMODE_xxx)
*                   only if ulMode == BR_DEVICE_MSIMODE_INFO, else don't care
* 
* 
* MSI and MSI-x cannot be mixed on one device. If this device is configured for
* one mode, the command will return an error if one tries to configure it to
* another mode.
* 
********************************************************************************
*******************************************************************************/
  
/*constants*/
#define BR_DEVICE_HANDLE_MAGIC 0x16F20000 /**<magic number, that must be found in the upper 16 bit of a br device handle*/
#define BR_DEVICE_HANDLE_MASK  0xFFFF0000 /**<mask all bits excluding the magic*/

#define BR_DEVICES_MAXNUM           32    /**<maximum number of BR devices*/

/*br device types*/
#define BR_DEVICETYPE_UNKNOWN        0    /**<unknown device type    -> pvBRdeviceInfoExt is 0*/
#define BR_DEVICETYPE_MAINUNIT       1    /**<main unit              -> pvBRdeviceInfoExt is 0*/
#define BR_DEVICETYPE_PCI            2    /**<pci/pci express device -> pvBRdeviceInfoExt points to BRdeviceInfoPci_t*/
#define BR_DEVICETYPE_SPI            3    /**<spi device             -> pvBRdeviceInfoExt points to tbd*/
#define BR_DEVICETYPE_I2C            4    /**<i2c device             -> pvBRdeviceInfoExt points to tbd*/
#define BR_DEVICETYPE_LPC            5    /**<lpc device             -> pvBRdeviceInfoExt points to tbd*/
#define BR_DEVICETYPE_AMBA           6    /**<amba device            -> pvBRdeviceInfoExt points to BRdeviceInfoPci_t*/


/*br device class*/
#define BR_DEVICECLASS_UNKNOWN       0    /**<device class unknown*/
#define BR_DEVICECLASS_MAINUNIT      1    /**<device class main unit*/

#define BR_DEVICECLASS_SPIBUS_CTRL   11   /**<device class spi bus controller*/
#define BR_DEVICECLASS_I2CBUS_CTRL   12   /**<device class i2c bus controller*/
#define BR_DEVICECLASS_SMBUS_CTRL    13   /**<device class sm bus controller*/
#define BR_DEVICECLASS_GPIO_CTRL     14   /**<device class gpio controller*/
#define BR_DEVICECLASS_LPCBUS_CTRL   15   /**<device class lpc bus controller*/

#define BR_DEVICECLASS_SERCOS3_CTRL  101  /**<device class sercos3*/
#define BR_DEVICECLASS_S20_CTRL      102  /**<device class s20*/
#define BR_DEVICECLASS_SAFETY_CTRL   103  /**<device class safety*/
#define BR_DEVICECLASS_NETX_CTRL     104  /**<device class netx*/
#define BR_DEVICECLASS_S20_REG_CTRL  105  /**<device class s20 with reg. itf */
#define BR_DEVICECLASS_NVRAM_CTRL    106  /**<device class non volatile random access memory (NVRAM) */
#define BR_DEVICECLASS_CAN_CTRL      107  /**<device class can*/
#define BR_DECICECLASS_SERCANS2G     108  /**<device class sercans2g*/

/*device locations, all device locations increment in 1*/
#define BR_DEVICE_LOCATION_UNDEF       0  /**<undefined device location*/
#define BR_DEVICE_LOCATION_MU          1  /**<device location main unit*/
                                          /**<device location 2 .. 10 is reserved*/
#define BR_DEVICE_LOCATION_ONMU_BASE  11  /**<device location on main unit starts at 1*/
#define BR_DEVICE_LOCATION_ONMU_TOP   99  /**<device location on main unit ends at 99*/
                                          /**<device location 100 is reserved*/
#define BR_DEVICE_LOCATION_EXT1_BASE 101  /**<device location extension 1 starts at 101*/
#define BR_DEVICE_LOCATION_EXT1_TOP  199  /**<device location extension 1 ends at 199*/
                                          /**<device location 200 is reserved*/
#define BR_DEVICE_LOCATION_EXT2_BASE 201  /**<device location extension 2 starts at 201*/
#define BR_DEVICE_LOCATION_EXT2_TOP  299  /**<device location extension 2 ends at 299*/
                                          /**<device location 300 is reserved*/
#define BR_DEVICE_LOCATION_EXT3_BASE 301  /**<device location extension 3 starts at 301*/
#define BR_DEVICE_LOCATION_EXT3_TOP  399  /**<device location extension 3 ends at 399*/

/*interrupt priorities*/
#define BR_DEVICE_INTPRIO_NOCHANGE   0xFFFFFFFF  /**<do not change the interrupt priority*/
#define BR_DEVICE_INTPRIO_1          1    /**<interrupt priority 1 = highest interrupt priority*/
#define BR_DEVICE_INTPRIO_2          2    /**<interrupt priority 2                             */
#define BR_DEVICE_INTPRIO_3          3    /**<interrupt priority 3                             */
#define BR_DEVICE_INTPRIO_4          4    /**<interrupt priority 4                             */
#define BR_DEVICE_INTPRIO_5          5    /**<interrupt priority 5                             */
#define BR_DEVICE_INTPRIO_6          6    /**<interrupt priority 6                             */
#define BR_DEVICE_INTPRIO_7          7    /**<interrupt priority 7                             */
#define BR_DEVICE_INTPRIO_8          8    /**<interrupt priority 8 = lowest interrupt priority */

/*interrupt subpriorities*/
/*for msi-x subpriorities have to be configured between 1 and 16. 1 is highest subpriority 16 lowest*/
#define BR_DEVICE_SUBPRIO_LIKEMSI    0xffffffff  /*automatically set the subprio like with msi*/

/*interrupt instances*/
#define BR_DEVICE_INTINDEX_UNDEF     0xFFFFFFFF  /**<only configure int priority, do not connect isr*/
#define BR_DEVICE_INTINSTANCEMAX    16    /**<maximum number if instances per interrupt priority*/

/*error codes BR_DEVICE_xxx*/
#define BR_DEVICE_OK                 0    /**<ok*/
#define BR_DEVICE_HANDLER_ERROR      1    /**<device handler wrong value*/
#define BR_DEVICE_MISSING_FEATURE    2    /**<requested feature not available*/
#define BR_DEVICE_PARAMETER_ERROR    3    /**<parameter error*/

#define BR_DEVICE_PCI_ADDR_REGION_NUM 6    /**<number of address regions in BRdeviceInfoPci_t */

/*data structure to keep information of BR devices*/
typedef struct
{
  unsigned long ulBRdeviceHandle;         /**<handle to device*/
  unsigned long ulBRdeviceLocation;       /**<device location information if available*/
  unsigned long ulBRdeviceType;           /**<type of the found device (e.g. pci, pcie, i2c ...)*/
  unsigned long ulBRdeviceClass;          /**<class of the found device (e.g. sercos, netx ...)*/
  void * pvBRdeviceInfoExt;               /**<pointer to extended device information*/
  unsigned long (*IdentInfoXChangeRtn)(BRIdentInfo_t * InData, BRIdentInfo_t* OutData); /*ident info function pointer*/
  unsigned long (*ExtModApiRtn)(BRExtModApi_t * InData, BRExtModApi_t* OutData);        /*extension module api function pointer*/
}
BRdeviceInfo_t;

/*data structure to keep information of pci devices. Is used, when ulBRdeviceType == BR_DEVICETYPE_PCI*/
typedef struct
{
  unsigned int     uiBusNo;
  unsigned int     uiDeviceNo;
  unsigned int     uiFunctionNo;
  unsigned short   usDeviceId;
  unsigned short   usVendorId;
  unsigned short   usSubSystemId;
  unsigned short   usSubVendorId;
  unsigned long    ulBaseAddr[BR_DEVICE_PCI_ADDR_REGION_NUM];
  unsigned long    ulAddrRange[BR_DEVICE_PCI_ADDR_REGION_NUM];
  unsigned char    ucIntVec;
  unsigned char    ucIntNumOf;
}
BRdeviceInfoPci_t;

/*data structure used in the BRBspApi command BRBSPAPI_DEVICES_MSI_CONFIG*/
typedef struct
{
  unsigned long ulBRdeviceHandle;                  /**<handle to the br device*/
  unsigned long ulBRdeviceMsiPrio;                 /**<interrupt priority for the device*/
  unsigned long ulBRdeviceMsiCpu;                  /**<index of dedicated cpu (only on multicore systems)*/
  unsigned long ulBRdeviceMsiIndex;                /**<interrupt number of device, if 0xffffffff -> no isr shall be connected*/
  void (*fpIsrHandler)(void *);                    /**<function pointer to the interrupt handler*/
  void * parg;                                     /**<pointer to arguments*/
}
BRdeviceMsiConfig_t;

/*data structure used in the BRBspApi command BRBSPAPI_DEVICES_MSI_CONF*/
typedef struct
{
  unsigned long ulBRdeviceHandle;                  /**<handle to the br device*/
  unsigned long ulMode;                            /**<see bits BR_DEVICE_MSIMODE_xxx*/
  unsigned long ulBRdeviceMsiPrio;                 /**<interrupt priority for the device*/
  unsigned long ulBRdeviceMsiSubPrio;              /**<interrupt sub priority for the device. Only MSIX don't care if used for MSI*/
  unsigned long ulBRdeviceMsiCpu;                  /**<index of dedicated cpu (only on multicore systems)*/
  unsigned long ulBRdeviceMsiIndex;                /**<interrupt number of device, if 0xffffffff -> no isr shall be connected*/
  void (*fpIsrHandler)(void *);                    /**<function pointer to the interrupt handler*/
  void * parg;                                     /**<pointer to arguments*/
}
BRdeviceMsiConf_t;

typedef struct
{
  unsigned long ulError;                           /**<see error codes BR_DEVICE_xxx*/
  unsigned long ulModeAvailable;                   /**<available mode: see bits BR_DEVICE_MSIMODE_xxx*/
  unsigned long ulModeConfigured;                  /**<configured mode: see bits BR_DEVICE_MSIMODE_xxx*/
}
BRdeviceMsiInfo_t;

/*msi modes used in data structures BRdeviceMsiConf_t and BRdeviceMsiInfo_t*/
#define BR_DEVICE_MSIMODE_INFO                0    /**<only read msi info*/
#define BR_DEVICE_MSIMODE_NONE                0    /**<interrupt not configured (read back info)*/
#define BR_DEVICE_MSIMODE_MSI                 1    /**<configure/d as msi*/
#define BR_DEVICE_MSIMODE_MSIX                2    /**<configure/d as msi-x*/


/**@}*/ /*end group br_devices**************************************************
*******************************************************************************/


/*******************************************************************************
***************************************************************************/ /**
* \defgroup   core_dump  core dump
*
* @{
*
* \brief      core dump support
*
* See also \ref bsp_api_commands_coredump
*
* The VxWorks core dump feature allows the device to generate core dumps either
* automatically or on demand. You can then analyze the core dump files using the
* Workbench debugger to determine the reason for a failure or to analyze the
* state of a system at a particular moment in its execution.
*
* A VxWorks kernel core dump is an image of kernel memory space, either virtual
* memory space if the MMU is on, or physical memory space if the MMU is off.
* The core dump file is the file where the image is written to memory.
* By default, the core dump file contains only the contents of physical memory
* in the range from the memory base address to sysPhysMemTop( ).
* You can include additional memory regions in the core dump using the BRBspApi.
* Additionally a callback routine, that is called during a coredump can be
* initialized using the BRBspApi.
* The core dump file will be stored in the persistent memory area, which will
* not be erased after a spezial reboot. This core dump file can then be copied
* to another medium using the BRBspApi.
*
* The following BRBspApi-commands exist in the context of core dump and should
* be used in the following sequence:
*
* - BRBSPAPI_COREDUMP_CHECK           check if there is a valid core dump:
* This command checks, if a valid core dump is available. Then you can decide,
* to copy the core dump file to another medium (bsp dependent e.g. CF or USB).
* Call this routine first, because BRBSPAPI_COREDUMP_INIT will format the
* core dump memory and therefore erase every core dump file in memory.
*
* - BRBSPAPI_COREDUMP_COPY            copy the core dump file to another medium:
* This command will copy the core dump file from memory to another medium.
* The medium used is bsp dependent. On the CML..-bsp for example the file
* will be copied to the compact flash user partition
*
* - BRBSPAPI_COREDUMP_INIT            initialize the coredump:
* This command formats the core-dump memory, generates a table which can
* take up to 1024 memory region entries to be saved additionally in the
* core dump.
*
* - BRBSPAPI_COREDUMP_CALLBACK_INIT   initialize the core dump call back routine:
* This command defines the additional routine to be called during core dump
*
* - BRBSPAPI_COREDUMP_MEMREGION_ADD   add or delete a memory region entry:
* This command adds or deletes a memory region entry from the table
* 
* - BRBSPAPI_COREDUMP_GENERATE        generate a core dump:
********************************************************************************
*******************************************************************************/

/**@}*/ /*end group core_dump***************************************************
*******************************************************************************/



/*******************************************************************************
***************************************************************************/ /**
* \defgroup   bsp_api  bsp api
*
* @{
*
* \brief      application interface to the board support pakage
*
*
* This api was implemented as an easy to use interface to bsp specific
* functions. The parameter ulCommand specifies the command to be executed. Some
* commands are only supported on specific hardwares. When a command is not
* supported the api returns BRBSPAPI_COMMAND_UNKNOWN. The parameter pvInData is
* used as a pointer to input data (data delivered to the api). The used data
* type is specified in the command list (defines). The parameter pvOutData is
* used as a pointer to output data (data returned by the api). The used data
* type is specified in the command list (defines). The api returns
* BRBSPAPI_ERROR_PARAM if parameters are missing or wrong. The api returns
* BRBSPAPI_ERROR_IN_SUBRTN if there are errors in called subroutines. 
* Return values of the api see defines below.
********************************************************************************
*******************************************************************************/

/**************************************************************************/ /**
* \brief      universal bsp application interface
*
* \param      [in]  ulCommand  command to be executed by the bsp (see defines)
* \param      [in]  pvInData   pointer to data, to be used by the BSP (see defines)
* \param      [out] pvOutData  pointer to data, returned by the BSP (see defines)
*
* \return     error value (see defines)
*******************************************************************************/
unsigned long BRBspApi(unsigned long ulCommand, void * pvInData, void * pvOutData);


/**************************************************************************/ /**
* \defgroup   bsp_api_errors  bsp api errors
* @{
*******************************************************************************/
#define BRBSPAPI_OK                0  /**<api did run without error*/
#define BRBSPAPI_ERROR_PARAM       1  /**<wrong api parameters*/
#define BRBSPAPI_COMMAND_UNKNOWN   2  /**<api command unknown*/
#define BRBSPAPI_ERROR_IN_SUBRTN   3  /**<error in subroutine called by the api*/
#define BRBSPAPI_ERROR_USERMODE    4  /**<error when called in user mode*/
#define BRBSPAPI_ERROR_LAST        5  /**<dummy define for last error value*/
/**@}*/ /*end group bsp_api_errors*********************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_user  bsp api commands for user mode
* @{
* 
* The BRBspApi() also can be used in user mode (RTP). This header takes care of
* it. The Windriver Workbench defines the constant _WRS_KERNEL only, if the
* project is a kernel mode projekt (Downloadable Kernel Module = DKM). If it is
* a user mode project (Real Time Process = RTP), then this define is not set.
* 
* This header defines the BRBspApi to BRBspApiUser, if the define _WRS_KERNEL
* is not set and therefore the call of BRBspApi() results in a call to
* BRBspApiUser(). BRBspApiUser() is the equivalent routine for user mode and
* is located in the shared library LibBR_BSPAPI.so. In user mode only a subset
* of commands is supported. If supported, this is mentioned in the documentation
* of the command.
* 
* Supported commands
* 
* - \ref BRBSPAPI_TIMESTAMP_FREQ
* - \ref BRBSPAPI_TIMESTAMP_CNT
* - \ref BRBSPAPI_CPUBOUND_TIMESTAMP_FREQ
* - \ref BRBSPAPI_CPUBOUND_TIMESTAMP_CNT
* - \ref BRBSPAPI_TIME_OS
* - \ref BRBSPAPI_BSPVERSION_GET
* - \ref BRBSPAPI_BSPAPIVERSION_GET
* - \ref BRBSPAPI_DEV_LIST_GET
*******************************************************************************/
/**@}*/ /*end group bsp_commands_user******************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands  bsp api commands
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \defgroup   bsp_api_commands_misc  bsp api commands miscelaneous
*******************************************************************************/

/**************************************************************************/ /**
* \brief      no operation
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_NOP 0

/**************************************************************************/ /**
* \brief      get bsp version string
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to version string (char **)
*******************************************************************************/
#define BRBSPAPI_BSPVERSION_STRING_GET 1

/**************************************************************************/ /**
* \defgroup   bsp_api_commands_identinfo_old  bsp api commands for identinfo
*                                             (old - do not use anymore)
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      get serial number of function module or main unit
*
* - pvInData:  pointer to module location, error code of ident function (unsigned int *)
* - pvOutData: pvOutData: pointer to char (empty serial number string) (char *)
*******************************************************************************/
#define BRBSPAPI_SERIALNR_GET 2

/**************************************************************************/ /**
* \brief      get material number of function module or main unit
*
* - pvInData:  pointer to module location, error code of ident function (unsigned int *)
* - pvOutData: pointer to material number variable (unsigned long *)
*******************************************************************************/
#define BRBSPAPI_MATNR_GET 3

/**************************************************************************/ /**
* \brief      get material index number of function module or main unit
*
* - pvInData:  pointer to module location, error code of ident function (unsigned int *)
* - pvOutData: pointer to material index number variable (unsigned long *)
*******************************************************************************/
#define BRBSPAPI_MATINDEXNR_GET 4

/**************************************************************************/ /**
* \brief      get type code of function module or main unit
*
* - pvInData:  pointer to module location, error code of ident function (unsigned int *)
* - pvOutData: pointer to char (empty type code string) (char *)
*******************************************************************************/
#define BRBSPAPI_TYPECODE_GET 5

/**************************************************************************/ /**
* \brief      get working time of function module or main unit
*
* - pvInData:  pointer to module location, error code of ident function (unsigned int *)
* - pvOutData: pointer to working time variable (unsigned long *)
*******************************************************************************/
#define BRBSPAPI_WORKTIME_GET 6
/**@}*/ /*end group bsp_api_commands_identinfo_old*****************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_pic  bsp api commands for the pic
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      enable interrupt in PIC
*
* - pvInData:  pointer to interrupt type (see defines)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_PIC_INT_ENABLE 7

/**************************************************************************/ /**
* \brief      disable interrupt in PIC
*
* - pvInData:  pointer to interrupt type (see defines)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_PIC_INT_DISABLE 8

/**\defgroup bsp_api_commands_pic_constants constants
***@{*/
#define BRBSPAPI_PIC_INT_ETH       0  /**<interrupt ethernet controller*/
#define BRBSPAPI_PIC_INT_ATA       1  /**<interrupt ata controller*/
#define BRBSPAPI_PIC_INT_LAST      2  /**<dummy define for last value*/
/**@}*/ /*end group bsp_api_commands_pic_constants*****************************/
/**@}*/ /*end group bsp_api_commands_pic***************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_eth_control  bsp api commands for eth dma control
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      enable ethernet controller pci bus access
*
* - pvInData:  not used (NULL)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_ETH_PCI_ENABLE 9

/**************************************************************************/ /**
* \brief      disable ethernet controller pci bus access
*
* - pvInData:  not used (NULL)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_ETH_PCI_DISABLE 10
/**@}*/ /*end group bsp_api_commands_eth_control*******************************/


/**************************************************************************/ /**
* \brief      get status of power supply buffer capacitors
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned char. 0: capacitors are full, 1: capacitors not full
*******************************************************************************/
#define BRBSPAPI_CAP_FULL 11

/**************************************************************************/ /**
* \brief      set priority of task tSymSync
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to integer: priority level
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_SET_SYMSYNC_PRIO 12


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_nvram  bsp api commands for nvram control
* @{
*
* different types of nvram:
* - nvram on memory module (no special treatment)
* - unpartitioned area on compact flash (special treatment necessary):
*    - at startup the cf nvram-area is transferred to a special allocated
*      memory.
*    - the checksum and a magic number is checked and can be accessed via
*      BRBSPAPI_NVRAM_CHECK.
*    - the rest of the initialization is done together with
*      BRBSPAPI_PWFNMI_INIT.
*    - at power fail, the power fail nmi routine calculates the checksum and
*      stores the nvram-data, magic number, checksum and statistic info to the
*      cf nvram area.
*******************************************************************************/

/**************************************************************************/ /**
* \brief      initialize power fail nmi routine
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned char (status value)
* this command deliveres a status value: 0 on success, !=0 on error.
*
* 0: success (initialisation done now, or already done)
* 1: capacitors are not full or ups not ready (initialisation not done yet), try again later
* 2: initialisation already done (only CML targets)
* 3: initialization error on other types of nvram then NVRAM_TYPE_CF
* 4: initialization error (e.g. sd-driver failed)
*
* Older versions of BSP (before IC_BASISFW_V14.20.2):
* ---------------------------------------------------
* BSP does not initialize/avtivate the nvram by itself. This command has to be
* called/polled by the system software till the status value is 0.
*
* New versions of BSP (since IC_BASISFW_V14.20.2):
* ------------------------------------------------
* BSPAPI may already initialize/activate the nvram save mechanism. To configure
* this, in KernelConfig.ini in the section "MemoryConfiguration" the parameter
* "NvramInitMode" is used. The values are defined like this:
* 
* parameter not existing: old behaviour - BSPAPI does not activate the nvram save mechanism 
* 0: BSPAPI does not activate the nvram save mechanism
* 1: BSPAPI activates the nvram save mechanism, when caps are full (ups ok on PRxx)
* 2: BSPAPI activates the nvram save mechanism, even without ups (only PRxx, ignored by XMxx)
* 
* If there are errors during initialisation/activation command BRBSPAPI_NVRAM_CHECK
* will deliver those error values
* 
* This command should not be called by the system software any more. Use instead
* a combination of BRBSPAPI_BOOT_STATUS_GET and BRBSPAPI_NVRAM_CHECK.
*******************************************************************************/
#define BRBSPAPI_PWFNMI_INIT 13

/**************************************************************************/ /**
* \brief      check nvram status
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long (nvram status: values see constants
*                                        (values are combined via logic or))
*******************************************************************************/
#define BRBSPAPI_NVRAM_CHECK 14

/**\defgroup bsp_api_commands_nvram_constants constants
***@{*/
#define CF_NVRAM_DATA_OK             (0x00000000)  /**<cf nvram no error*/
#define CF_NVRAM_DATA_NOT_VALID      (0x00000001)  /**<cf nvram data not saved completely, or not saved at all on the last power cycle*/ 
#define CF_NVRAM_CHECKSUM_ERROR      (0x00000002)  /**<cf nvram checksum error. e.g. data not saved completely or error during save cycle*/
#define CF_NVRAM_INIT_ERROR          (0x00000004)  /**<cf nvram initialisation error*/
#define CF_NVRAM_ACTIVATE_SAVE_ERROR (0x00000008)  /**<cf nvram activation error*/
/*******************************************************************************
* some more explanations of the previous error defines:
*
* CF_NVRAM_DATA_NOT_VALID:
* This error applies only to the actual restored nvram
* Reason: magic of actual restored nvram is missing
* When NVRAM is activated, a magic number on the CF card is deleted.
* When NVRAM is saved, the last thing written to CF is the status sector, containing
* the magic and the checksum. A missing magic means, that the nvram save mechanism did
* not complete or did not start at all. The checksum can tell more details. 
*
* CF_NVRAM_CHECKSUM_ERROR:
* This error applies only to the actual restored nvram
* Reason: checksum of actual restored nvram does not fit to nvram data
* When NVRAM is saved, the last thing written to CF is the status sector, containing
* the magic and the checksum. A bad checksum means, that the nvram save mechanism did
* not complete or data was corrupted. The magic can tell more details.
*
* The following combinations might occur:
* CF_NVRAM_DATA_NOT_VALID && CF_NVRAM_CHECKSUM_ERROR: nvram save did not complete
* CF_NVRAM_DATA_NOT_VALID only: nvram save did not start at all. Data fits to checksum
* CF_NVRAM_CHECKSUM_ERROR only: nvram save did complete but data does not fit to checksum (data corruption)
*
* CF_NVRAM_INIT_ERROR: initialisation failed
* This error applies to the actual restored nvram and also to the preparation for the next nvram save.
* Reason: This may be because of driver errors (ATA/SD/...). If driver resources
* are not available, we cannot restore the actual nvram contents and we will not be able
* to save the nvram in the next power down.

* CF_NVRAM_ACTIVATE_SAVE_ERROR:
* This error applies only to the preparation for the next nvram save.
* Reason: activation of the nvram save mechanism for the next power down failed.
* This may be due to problems with power supply buffer capacitors.
*/

#define NVRAM_TYPE_UNDEF          (0)           /**<nvram type undefined*/
#define NVRAM_TYPE_MODULE         (1)           /**<nvram on special nvram module*/
#define NVRAM_TYPE_SRAM           (2)           /**<sram emulates nvram*/
#define NVRAM_TYPE_CF             (3)           /**<cf card emulates nvram*/
#define NVRAM_TYPE_FILE           (4)           /**<nvram data stored in a file*/
#define NVRAM_TYPE_SD             (5)           /**<sd card emulates nvram*/
#define NVRAM_TYPE_SHM            (6)           /**<shared mem emulates nvram*/
#define NVRAM_TYPE_MRAM           (7)           /**<mram emulates nvram*/
/**@}*/ /*end group bsp_api_commands_nvram_constants***************************/
/**@}*/ /*end group bsp_api_commands_nvram*************************************/


/**************************************************************************/ /**
* \brief      get information of onboard sram module
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to array of two unsigned longs i.g. long ulInfo[2];
*              - ulInfo[0] = sram base address, 0 if not available
*              - ulInfo[1] = sram size, 0 if not  available
*******************************************************************************/
#define BRBSPAPI_SRAM_INFO 15


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_ipms4  bsp api commands for ipms4
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      get ipms4 base address
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long (base address without ipms4 offsets)
*******************************************************************************/
#define BRBSPAPI_GET_IPMS4_BASE 16

/**************************************************************************/ /**
* \brief      get ipms4 register base address
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long (register base address)
*******************************************************************************/
#define BRBSPAPI_GET_IPMS4_REG_BASE 17

/**************************************************************************/ /**
* \brief      get ipms4 dual port memory base address
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long (dual poart memory base address)
*******************************************************************************/
#define BRBSPAPI_GET_IPMS4_DPM_BASE 18

/**@}*/ /*end group bsp_api_commands_ipms4*************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_sercos_clk  bsp api commands for sercos clocks
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      set the master for sercos cycle clock
*
* - pvInData:  pointer to unsigned char: selected mode:
*              - 0: only read back actual status of multiplexer
*              - 1: cyc clk not driven
*              - 2: PC104P-module drives cyc clk
*              - 3: timer a drives cyc clk
*              - 4: left onboard module drives cyc clk (module 1)
*              - 5: right onboard module drives cyc clk (module 2)
* - pvOutData: pointer to unsigned char: readback of actual mode
*              - 0xff: error
*              - 1: cyc clk not driven
*              - 2: PC104P-module drives cyc clk
*              - 3: timer a drives cyc clk
*              - 4: left onboard module drives cyc clk (module 1)
*              - 5: right onboard module drives cyc clk (module 2)
* this command replaces the older functions:
* vIcSetSercosTimerMaster(); vIcSetSercosTimerSlave(); ucIcSercosTimerCheckMasterMode();
*******************************************************************************/
#define BRBSPAPI_SET_CYCCLK_MASTER 19

/**************************************************************************/ /**
* \brief      set the master for sercos div clock
*
* - pvInData:  pointer to unsigned char: selected mode:
*              - 0: only read back actual status of multiplexer
*              - 1: right onboard module drives cyc clk (module 2)
*              - 2: PC104P-module drives cyc clk
*              - 3: left onboard module drives cyc clk (module 1)
* - pvOutData: pointer to unsigned char: readback of actual mode
*              - 0xff: error
*              - 1: right onboard module drives cyc clk (module 2)
*              - 2: PC104P-module drives cyc clk
*              - 3: left onboard module drives cyc clk (module 1)
*******************************************************************************/
#define BRBSPAPI_SET_DIVCLK_MASTER 20

/**@}*/ /*end group bsp_api_commands_sercos_clk********************************/
			 
			 
/**************************************************************************/ /**
* \brief      call test funktions (L40Test)
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to unsigned short: test number for L40Test()
* - pvOutData: pointer to unsigned short: return value of L40Test()
*******************************************************************************/
#define BRBSPAPI_L40TEST 21

/**************************************************************************/ /**
* \brief      display message on lcd display
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to string, max 8 characters + EOS
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_LCDMSG 22

/**************************************************************************/ /**
* \brief      get information of nvram
* \ingroup    bsp_api_commands_nvram
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to array of 3 unsigned longs e.g. unsigned long ulInfo[3]
*              - ulInfo[0] = nvram base address, 0 if not available
*              - ulInfo[1] = nvram size, 0 if not  available
*              - ulInfo[2] = nvram type, see constants
*******************************************************************************/
#define BRBSPAPI_NVRAM_INFO 23

/**************************************************************************/ /**
* \defgroup   bsp_api_commands_fm_api  bsp api commands for function module api
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      register a function for BrFmApi
*
* - pvInData:  pointer to an array of 2 unsigned longs
*              e.g. unsigned long ulInfo[2];
*              - ulInfo[0] = module location of module for which function
*                            pointer shall be initialized
*              - ulInfo[1] = function pointer as unsigned long
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_FMAPI_REGISTER 24

/**************************************************************************/ /**
* \brief      call function module api
*
* - pvInData:  pointer to module location (unsigned int *)
* - pvOutData: pointer to data fot function module api (void *)
*******************************************************************************/
#define BRBSPAPI_FMAPI_CALL 25

/**@}*/ /*end group bsp_api_commands_fm_api************************************/


/**************************************************************************/ /**
* \brief      get hardware type
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to hardware type (int *)
*******************************************************************************/
#define BRBSPAPI_GETHWTYPE 26


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_watchdog  bsp api commands for watchdog control
* @{
* 
* \brief      watchdog api
* 
* There are two watchdogs on most BR targets. An analog watchdog and a digital
* watchdog.
* 
* The analog watchdog consists of a hardware device with a fixed
* drop time. If not triggered within the specified time interval, the analog
* watchdog drops and generates an error. Additionally it may generate an
* interrupt.
* The digital watchdog consists of a programmable timer (watchdog period).
* If not triggered within the programmed time interval, the digital
* watchdogs drops and generates an error. Additionally it may generate an
* interrupt.
* 
* The BR_BSPAPI command BRBSPAPI_WDDIGTIME is used to set the time of the
* digital watchdog or read the actual time setting of the digital watchdog.
* If the digital watchdog once is activated and running, it is not recommended
* to change the value of the watchdog time, because  the reaction of the
* watchdog hardware to this action is not specified and it may drop.
* 
* Parameters:
* - pvInData:  pointer to an unsigned long that specifies the digital watchdog
*              time in milliseconds. NULL if you only want to read the actual
*              digital watchdog time.
* - pvOutData: pointer to an unsigned long that can hold the actual digital
*              watchdog time in milliseconds. NULL if you do not want to read
*              the actual digital watchdog time.
* 
* 
* The BR_BSPAPI command BRBSPAPI_WDCONTROL is used to control both watchdogs
* and read the status of both watchdogs.
* 
* Parameters:
* - pvInData:  pointer to an unsigned long that specifies the action to be done.
*              Use the constants (WDCONTROL_xxx) ored together, to specifie the
*              action. The magic (WDCONTROL_MAGIC) always must be included in
*              the control data, otherwise the command will be rejected by the
*              BR_BSPAPI.
* - pvOutData: pointer to an unsigned long that can hold the status information
*              of both watchdogs. For Meaning of the status bits see constants
*              WDSTATUS_xxx.
* 
* It is possible to already trigger the watchdogs, but keep the watchdogs still
* disabled. This may be necessary, if the watchdog trigger task can not
* guarantee, that the watchdog is triggered within the specified time interval
* (e.g. during system startup). After starting the watchdogs using the control
* bits WDCONTROL_WDxxxSTART, the watchdog will drop, if not triggered in time.
* If the watchdog is stopped using using the control bits WDCONTROL_WDxxxSTOP,
* the watchdog will drop when the specified watchdog time has passed after the
* last trigger.
* 
* 
* <B>Interrupts</B>
* 
* If the hardware offers a watchdog interrupt feature, the BR_BSPAPI supports
* connecting own callback routines to the watchdog interrupt service routine.
* The BR_BSPAPI commands BRBSPAPI_WDxxx_ISRINIT are used to initialize the
* watchdog isr's and if wanted, to register own callback routines.
* 
* Parameters:
* - pvInData:  pointer to void, that points to a callback routine
*              (void callback(void)). NULL if no callback shall be connected.
* - pvOutData: pointer to integer that holds the error value of the isr init
*              subroutine (see constants BRBSPAPI_WD_ISRINIT_xxx).
* 
* <B>Target specific issues</B>
* 
* <EM>XM2x</EM>
* 
* On the XM2x targets, the watchdog functionality is realized in the S20 fpga.
* As this is a pcie device, all interrupts of that device share the same
* interrupt priority class, although each interrupt source has its own
* dedicated interrupt vector. The interrupt priority of BR pcie devices can be
* configured with the BR_BSPAPI command BRBSPAPI_DEVICES_MSI_CONFIG. Because
* the watchdog interrupts have the same interrupt priority as the rest of the
* S20 interrupts, the watchdog isr's cannot be connected before the priority
* for the S20 interrupts is configured (the BR_BSPAPI must know the priority
* in order to configure the correct interrupt vector).
* For this reason the system software first has to configure the interrupt
* priority of the S20 device using the BR_BSPAPI command
* BRBSPAPI_DEVICES_MSI_CONFIG and only after that the system software can
* initialize the watchdog isr's including the connect of callback routines.
* 
* 
* <EM>Other targets ...</EM>
*******************************************************************************/

/**************************************************************************/ /**
* \brief      watchdog control
*
* - pvInData:  pointer to unsigned long (watchdog control),
*              bitmask see constants above.
*              important: set watchdog control magic using control magic mask
* - pvOutData: pointer to unsigned long (watchdog status) , bitmask see
*              constants above
*******************************************************************************/
#define BRBSPAPI_WDCONTROL 27

/**************************************************************************/ /**
* \brief      digital watchdog time
*
* - pvInData:  set: pointer to unsigned long (watchdog time in milli seconds), NULL if only get
* - pvOutData: get: pointer to unsigned long (watchdog time in milli seconds), NULL if only set
*******************************************************************************/
#define BRBSPAPI_WDDIGTIME 28

/**\defgroup bsp_api_commands_watchdog_control_constants constants for watchdog control
***@{*/
#define WDCONTROL_MAGIC_MASK      (0x0000FFFF)  /**<mask for magic number bits*/
#define WDCONTROL_MAGIC           (0x0000AFFE)  /**<magic number needed for watchdog accesses*/
#define WDCONTROL_WDANA_TRIGGER   (0x80000000)  /**<control bit for trigger of analog watchdog*/
#define WDCONTROL_WDANASTART      (0x40000000)  /**<control bit for start of analog watchdog*/
#define WDCONTROL_WDANASTOP       (0x20000000)  /**<control bit for stop of analog watchdog*/
#define WDCONTROL_WDDIG_TRIGGER   (0x08000000)  /**<control bit for trigger of digital watchdog*/
#define WDCONTROL_WDDIGSTART      (0x04000000)  /**<control bit for start of digital watchdog*/
#define WDCONTROL_WDDIGSTOP       (0x02000000)  /**<control bit for stop of digital watchdog*/
/**@}*/ /*end group bsp_api_commands_watchdog_control_constants************************/

/**\defgroup bsp_api_commands_watchdog_status_constants constants for watchdog status
***@{*/
#define WDSTATUS_WDANA_RUNS       (0x80000000)  /**<status bit of analog watchdog : 1 = analog watchdog is running*/
#define WDSTATUS_WDANA_ERR        (0x40000000)  /**<status bit of analog watchdog : 1 = analog watchdog error (dropped)*/
#define WDSTATUS_WDDIG_RUNS       (0x08000000)  /**<status bit of digital watchdog : 1 = digital watchdog is running*/
#define WDSTATUS_WDDIG_ERR        (0x04000000)  /**<status bit of digital watchdog : 1 = digital watchdog error (dropped)*/
/**@}*/ /*end group bsp_api_commands_watchdog_status_constants************************/
/**@}*/ /*end group bsp_api_commands_watchdog**********************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_ident  bsp api commands for ident information
* @{
*
* See also \ref ident_information
*******************************************************************************/

/**************************************************************************/ /**
* \brief      exchange ident information
*
* for more info see BRIdentInfo_t declaration
*
* - pvInData:  pointer to BRIdentInfo_t structure (data to write)
* - pvOutData: pointer to BRIdentInfo_t structure (data to read)
*******************************************************************************/
#define BRBSPAPI_IDENT_INFO 29

/**************************************************************************/ /**
* \brief      connect identfunc-exchange routine
*
* for more info see BRIdentInfo_t declaration
*
* - pvInData:  pointer to BRIdentInfoFunc_t structure
* - pvOutData: pointer to error value of subroutine (unsigned long)
*              (see return values of ident functions IDENTFUNC_OK or IDENTFUNC_ERROR...)
*******************************************************************************/
#define BRBSPAPI_IDENTFUNC_REGISTER 30

/**@}*/ /*end group bsp_api_commands_ident*************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_usb  bsp api commands for usb control
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      USB control
*
* - pvInData:  pointer to unsigned long (USB control), bitmask see constants above
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_USB_CONTROL 31

/**\defgroup bsp_api_commands_usb_constants constants
***@{*/
#define USBCONTROL_FRONT_ENABLE   (0x80000000)
#define USBCONTROL_FRONT_DISABLE  (0x08000000)  
/**@}*/ /*end group bsp_api_commands_usb_constants*****************************/
/**@}*/ /*end group bsp_api_commands_usb***************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_fpga_int_ctrl  bsp api commands for fpga int control
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      fpga interrupt controller configuration
*
* - pvInData:  pointer to array of unsigned long
*              - ulArray[0]: number of interrrupt
*              - ulArray[1]: interrupt polarity
*                            - 0 = do not change
*                            - 1 = high active / rising edge
*                            - 2 = low  active / falling edge
*              - ulArray[2]: interrupt trigger mode
*                            - 0 = do not change
*                            - 1 = edge triggered
*                            - 2 = level triggered
*              - ulArray[3]: interrupt enable/disable
*                            - 0 = do not change
*                            - 1 = interrupt enable
*                            - 2 = interrupt disable
* - pvOutData: pointer to unsigned char (return value of subroutine)
*              - 0 : OK, 0xff : interrupt number out of range
*******************************************************************************/
#define BRBSPAPI_FPGA_INT_CONFIG 32

/**************************************************************************/ /**
* \brief      read FPGA interrupt number for a given module location
*
* - pvInData:  pointer to unsigned long (module location see constants FUNCMOD_PCI_LOC_...)
* - pvOutData: pointer to unsigned long (interrupt number) 0xffffffff->error
*******************************************************************************/
#define BRBSPAPI_FPGA_INTNUM_BYMODLOC 33

/**************************************************************************/ /**
* \brief      read FPGA interrupt number for a given feature
*
* - pvInData:  pointer to unsigned long (feature see constants FPGA_FEATURE_...)
* - pvOutData: pointer to unsigned long (interrupt number) 0xffffffff->error
*******************************************************************************/
#define BRBSPAPI_FPGA_INTNUM_BYFEATURE 34

/**************************************************************************/ /**
* \brief      connect a cpu specific nmi routine
*
* - pvInData:  pointer to array of unsigned long
*              - ulArray[0]: cpu id (0 = boot cpu)
*              - ulArray[1]: address of routine
*              - ulArray[2]: pointer to arguments of routine
* - pvOutData: pointer to unsigned long (error value of subroutine)
*******************************************************************************/
#define BRBSPAPI_NMI_CPU_CONNECT 35

/**************************************************************************/ /**
* \brief      unlock a cpu specific nmi routine
*
* - pvInData:  pointer to unsigned long (cpu id (0 = boot cpu))
* - pvOutData: pointer to unsigned long (error value of subroutine)
*******************************************************************************/
#define BRBSPAPI_NMI_CPU_UNLOCK 36

/**@}*/ /*end group bsp_api_commands_fpga_int_ctrl*****************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_vxwin  bsp api commands for vxwin 
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      initialize callback routines
*
* - pvInData:  pointer to BrCallbacks_t structure
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_CALLBACKS_INIT 37

/**************************************************************************/ /**
* \brief      get info, if system fpga is included
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned char
*              - 0: system fpga not included
*              - 1: system fpga included
*******************************************************************************/
#define BRBSPAPI_SYSTEMFPGA_INCLUDED 38

/**@}*/ /*end group bsp_api_commands_vxwin*************************************/

/**************************************************************************/ /**
* \brief      initialize bosch rexroth hardware
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_INIT_HARDWARE 39

/**************************************************************************/ /**
* \defgroup   bsp_api_commands_coredump  bsp api commands for core dump 
* @{
*
* See also \ref core_dump
*******************************************************************************/

/**************************************************************************/ /**
* \brief      initialize the coredump
* \remarks    not thread-safe. Caller should avoid simultaneous coredump
*             initialization.
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to int (error value of subroutine)
*              - 0: OK
*              - else: ERROR
*******************************************************************************/
#define BRBSPAPI_COREDUMP_INIT 40

/**************************************************************************/ /**
* \brief      initialize the coredump callback routine
*
* - pvInData:  pointer to a coredump callback routine (void (* funcptr)(void))
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_COREDUMP_CALLBACK_INIT 41

/**************************************************************************/ /**
* \brief      add a coredump memory region or auxiliary file
* \remark     Mutex-Semaphore is used internal to ensure exclusive access.
*
* - pvInData:  pointer to an array of unsigned longs
*              - ulArray[0]: address of memory region to be added/deleted
*              - ulArray[1]: size of memory region (don't care on delete)
*              - ulArray[2]: - if equals -1, ulArray[3] is valid;
*                            - otherwise, address of memory region in coredump,
*                            usually the same as the address in the target.
*                            ulArray[3] is invalid and can be omitted.
*                            (don't care on delete)
*              - ulArray[3]: point to the filename of the aux file to be added
*                            (don't care on delete)
* - pvOutData: pointer to int (error value of subroutine)
*              - 0: OK
*              - 1: no empty memory region entry found
*              - 2: core dump not initialized
*              - 3: vmStateSet-Error
*******************************************************************************/
#define BRBSPAPI_COREDUMP_MEMREGION_ADD 42

/**************************************************************************/ /**
* \brief      delete a coredump memory region or auxiliary file
* \remark     Mutex-Semaphore is used internal to ensure exclusive access.
*
* - pvInData:  pointer to unsigned longs: address of memory region to be added/deleted
* - pvOutData: pointer to int (error value of subroutine)
*              - 0: OK
*              - 1: specifierd memory region entry not found
*              - 2: core dump not initialized
*              - 3: vmStateSet-Error
*******************************************************************************/
#define BRBSPAPI_COREDUMP_MEMREGION_DEL 43

/**************************************************************************/ /**
* \brief      check for a core dump, get the core dump file size
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to long long (core dump file size).
*              If the core dump file size is 0: coredump not valid or not available
*******************************************************************************/
#define BRBSPAPI_COREDUMP_CHECK 44

/**************************************************************************/ /**
* \brief      copy the coredump-file to storage volume
*
* - pvInData:  pointer to char: path to copy core dump file to, if 0 default path will be used
* - pvOutData: pointer to int:
*              - 0: ok
*              - 1: coredump not available
*              - 2: coredump get error
*              - 3: coredump not valid
*              - 4: coredump copy error
*******************************************************************************/
#define BRBSPAPI_COREDUMP_COPY 45

/**************************************************************************/ /**
* \brief      generate a coredump
*
* - pvInData:  not used (NULL) 
* - pvOutData: not used (NULL)
* 
* if pvInData != 0, then it must point to a BRcoreDumpAddData_t struct.
* coreDumpGenerate will be used instead of coreDumpUsrGenerate
*******************************************************************************/
#define BRBSPAPI_COREDUMP_GENERATE 46

/*additional data for core dump (for use in coreDumpGenerate())*/
typedef struct
{
  int       vector;       /* exception vector number              */
  char *    pEsf;         /* exception frame pointer              */
  void *    pRegs;        /* exception register set (REG_SET *)   */
  void *    pExcInfo;     /* exception information (EXC_INFO *)   */
  int       coreDumpType; /* coreDump type (CORE_DUMP_TYPE)       */
  char *    pInfoString;  /* ptr to coredump information string   */
  int       isException;  /* are we handling an exception? (BOOL) */
}
BRcoreDumpAddData_t;

/**@}*/ /*end group bsp_api_commands_coredump**********************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_timer_c  bsp api commands for timer c 
* @{
*
* <b>
* !!! attention !!! timer c funcionality has changed. The commands are not
* supported as described here. DO NOT USE THOSE COMMANDS!!!!!!!!
* The implementation of those commands will updated in the future.
* </b>
*******************************************************************************/

/**************************************************************************/ /**
* \brief      set/read timer c reload value in 10us steps
*
* \remarks    timer c period = (reload value + 1) * 10us
*
* - pvInData:  pointer to unsigned short: releoad value 0x0000...0x0fff
* - pvOutData: pointer to unsigned short: actual reload value (before actualization)
*******************************************************************************/
#define BRBSPAPI_TIMERC_RELOAD 47

/**************************************************************************/ /**
* \brief      start or stop timer c
*
* - pvInData:  pointer to unsigned char
*              - 0: stop timer c
*              - 1: start timer c
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_TIMERC_START 48

/**************************************************************************/ /**
* \brief      timer c interrupt enable
*
* - pvInData:  pointer to unsigned char
*              - 0: do not change interrupt trigger mode (level/edge)
*              - 1: interrupt level triggered
*              - 2: interrupt edge triggered
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_TIMERC_INTENABLE 49

/**************************************************************************/ /**
* \brief      timer c interrupt disable
*
* - pvInData:  pointer to unsigned char
*              - 0: do not change interrupt trigger mode (level/edge)
*              - 1: interrupt level triggered
*              - 2: interrupt edge triggered
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_TIMERC_INTDISABLE 50

/**************************************************************************/ /**
* \brief      timer c run check
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned char
*              - 0: timer c not running
*              - 1: timer c running
*******************************************************************************/
#define BRBSPAPI_TIMERC_RUNCHECK 51

/**************************************************************************/ /**
* \brief      timer c connect int routine
*
* - pvInData:  pointer to tIPIsrHndlEntry-structure pointer to int routine and
*              arguments. If pointer to int routine == 0 -> disconnect timer c
*              int routine
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_TIMERC_INTCONNECT 52

/**************************************************************************/ /**
* \brief      timer c interrupt status / end of interrupt
*
* - pvInData:  pointer to unsigned char
*              - 0: only read interrupt status
*              - 1: send end of interrupt message
* - pvOutData: pointer to unsigned char
*              - 0: timer c interrupt not active
*              - 1: timer c interrupt active
*******************************************************************************/
#define BRBSPAPI_TIMERC_INTSTATUS 53

/**@}*/ /*end group bsp_api_commands_timer_c***********************************/


/**************************************************************************/ /**
* \brief      get pci bus frequency. If hardware uses the standard value of
*             33 mhz, this command need not be supported
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long: pci frequency in mhz
*******************************************************************************/
#define BRBSPAPI_PCI_FREQ_GET 54

/**************************************************************************/ /**
* \brief      get cpu usage
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to cpu usage variable (unsigned int *)
*******************************************************************************/
#define BRBSPAPI_CPUUSAGE_GET 55

/**************************************************************************/ /**
* \brief      get fpga interrupt controller info
* \ingroup    bsp_api_commands_fpga_int_ctrl
*
* - pvInData:  not used (NULL)
* - pvOutData: (char *) (pointer to string, buffer with min. 100 chars)
*              The interrupt controller info string is copied to pvOutData.
*******************************************************************************/
#define BRBSPAPI_FPGAINTCTRLINFO_GET 56

/**************************************************************************/ /**
* \brief      get fpga interrupt pending bit of dedicated interrupt number
* \ingroup    bsp_api_commands_fpga_int_ctrl
*
* - pvInData:  pointer to unsigned char containing the fpga interrupt number
*              use BRBSPAPI_FPGA_INTNUM_BYMODLOC or BRBSPAPI_FPGA_INTNUM_BYFEATURE
*              to retrieve interrupt number
* - pvOutData: pointer to unsigned char:
*              - 0: Interrupt is not pending
*              - 1: Interrupt is pending
*******************************************************************************/
#define BRBSPAPI_FPGAINTPENDING_BYINTNUM_GET 57

/**************************************************************************/ /**
* \brief      reset fpga interrupt pending bit of dedicated interrupt number
* \ingroup    bsp_api_commands_fpga_int_ctrl
*
* - pvInData:  pointer to unsigned char containing the fpga interrupt number
*              use BRBSPAPI_FPGA_INTNUM_BYMODLOC or BRBSPAPI_FPGA_INTNUM_BYFEATURE
*              to retrieve interrupt number
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_FPGAINTPENDING_BYINTNUM_RESET 58


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_tftp  bsp api commands for tftp
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      file transfer via tftp protocol
* \ingroup    bsp_api_commands_tftp
*
* - pvInData:  
* - pvOutData:
*******************************************************************************/
#define BRBSPAPI_TFTP_COPY	59

#define BRBSPAPI_TFTP_COPY_MAX_OPTIONS 4

typedef struct
{
  char * pcLocalFilePathAndName;	/* file path and name of the local file */
  char * pcRemoteHostNameOrIP;
  unsigned short usPort;
  char * pcRemoteFilePathAndName;
  char * pcDirection;				/* put or get */
  char * pcMode;					/* transfer mode: netascii or octect */

  /* Options:                RFC 2347  TFTP Option Extention
    blksize (default 512)    RFC 2348  TFTP Blocksize Option
    timeout in sec [1..255]  RFC 2349  TFTP Timeout Interval and Transfer Size Options
    tsize                    RFC 2349  TFTP Timeout Interval and Transfer Size Options
  */
  char * pcOptionName [BRBSPAPI_TFTP_COPY_MAX_OPTIONS];
  char * pcOptionValue[BRBSPAPI_TFTP_COPY_MAX_OPTIONS];
}
BRtftpCopyIn_t;

/* BRBSPAPI_TFTP error codes */
#define BRBSPAPI_TFTP_OK

/* BRBSPAPI_TFTP_COPY input parameter check */
#define BRBSPAPI_TFTP_ERR_LOCAL_FILE 
#define BRBSPAPI_TFTP_ERR_REMOTE_HOST
#define BRBSPAPI_TFTP_ERR_PORT
#define BRBSPAPI_TFTP_ERR_REMOTE_FILE
#define BRBSPAPI_TFTP_ERR_DIRECTION
#define BRBSPAPI_TFTP_ERR_MODE
#define BRBSPAPI_TFTP_ERR_OPTION_0_NAME
#define BRBSPAPI_TFTP_ERR_OPTION_0_VALUE
#define BRBSPAPI_TFTP_ERR_OPTION_1_NAME
#define BRBSPAPI_TFTP_ERR_OPTION_1_VALUE
#define BRBSPAPI_TFTP_ERR_OPTION_2_NAME
#define BRBSPAPI_TFTP_ERR_OPTION_2_VALUE
#define BRBSPAPI_TFTP_ERR_OPTION_3_NAME
#define BRBSPAPI_TFTP_ERR_OPTION_3_VALUE

/* TFTP layer specific errors */
#define BRBSPAPI_TFTP_BASE      (2<<16)
#define	BRBSPAPI_TFTP_EUNDEF    (BRBSPAPI_TFTP_BASE+0)  /* not defined          */
#define	BRBSPAPI_TFTP_ENOTFOUND (BRBSPAPI_TFTP_BASE+1)  /* file not found       */
#define	BRBSPAPI_TFTP_EACCESS   (BRBSPAPI_TFTP_BASE+2)  /* access violation     */
#define	BRBSPAPI_TFTP_ENOSPACE  (BRBSPAPI_TFTP_BASE+3)  /* disk full            */
#define	BRBSPAPI_TFTP_EBADOP    (BRBSPAPI_TFTP_BASE+4)  /* illegal operation    */
#define	BRBSPAPI_TFTP_EBADID    (BRBSPAPI_TFTP_BASE+5)  /* unknown transfer ID  */
#define	BRBSPAPI_TFTP_EEXISTS   (BRBSPAPI_TFTP_BASE+6)  /* file already exists  */
#define	BRBSPAPI_TFTP_ENOUSER   (BRBSPAPI_TFTP_BASE+7)  /* no such user         */

/* UDP layer specific errors */

/* IP layer specific errros */

typedef struct
{
  unsigned long ulError;
}
BRtftpCopyOut_t;

/**@}*/ /*end group bsp_api_commands_tftp**************************************/


/**************************************************************************/ /**
* \brief      get the vxWorks boot status
* \ingroup    bsp_api_commands_misc
* \remarks    The startup script (IndraC.ini) is used to start up applications.
*             Because the vxWorks operation system has not booted completely,
*             when the startup script is executed, each application must wait
*             until vxWorks is up completely. The best practice is to start a
*             task in the startup script, that loops without action until
*             vxWorks is up.
* - pvInData:  not used (NULL)
* - pvOutData: pointer to int that returns the bsp boot status:
*              0 if VxWorks is not up
*              1 if VxWorks is up
*******************************************************************************/
#define BRBSPAPI_BOOT_STATUS_GET 60

/**************************************************************************/ /**
* \brief      set and get the end device ip-address for the wdb agent
* \ingroup    bsp_api_commands_misc
*
* \remarks    Set the ip-address of the wdb agent (windriver debug agent)
*             to the desired value and read back the actual ip-address of the
*             wdb agennt. 
*             This does not affect the ip-address of the network stack.
*
*             This routine is necessary, because the wdb-agent, if configured
*             with end-connection instead of network-connection does not use
*             the ip-address configured by ifConfig(), but only uses the
*             ip-address configured in the boot-line.
*
* - pvInData:  if NULL: only read back actual ip address
*              else:    ip address string e.g. "192.168.100.101"
* - pvOutData: if NULL: only set new ip address
*              else:    pointer to a string of 16 chars (incl. EOS)
*******************************************************************************/
#define BRBSPAPI_WDB_IPADDR 61

/**************************************************************************/ /**
* \brief      get the name of the standard ethernet interface(s) (no sercos or netx etc.)
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to int (index of standard interface)
* - pvOutData: pointer to BRifName_t struct
*              empty strings on error, digit = 0 on error
*******************************************************************************/
#define BRBSPAPI_IFNAMEGET 62

#define BRBSPAPI_IFNAME_MAXLEN                16

#define BRBSPAPI_IFNAMEGET_OK                  0
#define BRBSPAPI_IFNAMEGET_ERR_NOTAVAILABLE    1
#define BRBSPAPI_IFNAMEGET_ERR_BUFFERLEN       2

typedef struct
{
  char ifNameComplete[BRBSPAPI_IFNAME_MAXLEN];   /**<ifname including digit e.g. "gei0"*/
  char ifNameBase[BRBSPAPI_IFNAME_MAXLEN];       /**<ifname without digit e.g. "gei"*/
  int  ifIndex;                                  /**<index of ifname e.g. 0*/
  int  error;                                    /**<error value*/
}
BRifName_t;

/**************************************************************************/ /**
* \brief      get the name of a given mount point
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to integer: drive index (see defines in header)
* - pvOutData: pointer to BRmountPointName_t struct
*              empty strings on error
*******************************************************************************/
#define BRBSPAPI_MOUNTPOINT_NAME_GET 63

#define BR_MOUNTPOINT_SYSTEM             0
#define BR_MOUNTPOINT_OEM                1
#define BR_MOUNTPOINT_USER               2
#define BR_MOUNTPOINT_REMOVABLE_SD_A     3
#define BR_MOUNTPOINT_REMOVABLE_USB_A    4

#define BRBSPAPI_MOUNTPOINT_NAME_MAXLEN       64

#define BRBSPAPI_MOUNTPOINT_NAME_OK                  0
#define BRBSPAPI_MOUNTPOINT_NAME_ERR_NOTAVAILABLE    1
#define BRBSPAPI_MOUNTPOINT_NAME_ERR_BUFFERLEN       2

typedef struct
{
  char mountPointName[BRBSPAPI_MOUNTPOINT_NAME_MAXLEN];   /**<mount point name*/
  int  error;                                    /**<error value*/
}
BRmountPointName_t;


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_brdevices  bsp api commands for BR devices
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      get number of BR devices of defined class
* \ingroup    bsp_api_commands_brdevices
*
* - pvInData:  pointer to unsigned long: device class to look for (BR_DEVICECLASS_...)
*                                        BR_DEVICECLASS_UNKNOWN -> get number of all BR devices
* - pvOutData: pointer to unsigned long: number of devices found
*******************************************************************************/
#define BRBSPAPI_DEVICESOFCLASS_NR_GET 64

/**************************************************************************/ /**
* \brief      get device info of x'th device with given device class
* \ingroup    bsp_api_commands_brdevices
*
* - pvInData:  pointer to array of unsigned longs: 
*              - ulArray[0]: device class to look for (BR_DEVICECLASS_...)
*                            BR_DEVICECLASS_UNKNOWN -> get info of all BR devices
*              - ulArray[1]: desired instance of device, starting with 0
* - pvOutData: pointer to pointer to device info struct (BRdeviceInfo_t **)
*              if no device found -> returned pointer = 0
*******************************************************************************/
#define BRBSPAPI_DEVICEBYCLASS_INFO_GET 65

/**************************************************************************/ /**
* \brief      configure message signalled interrupts of br devices
* 
* \remarks    there is a newer version of this command available: BR_BSPAPI_DEVICES_MSI_CONF
*             the new command additionally supports msi-x
*             the old command stays available for compability purpose
*             on new designs please use the new command
* 
* \ingroup    bsp_api_commands_brdevices
*
* - pvInData:  pointer to BRdeviceMsiConfig_t structure
* - pvOutData: pointer to unsigned long: error code of subroutine
*******************************************************************************/
#define BRBSPAPI_DEVICES_MSI_CONFIG 66

/**@}*/ /*end group bsp_api_commands_brdevices*********************************/

/**************************************************************************/ /**
* \brief      get the version of the BR_BSPAPI
* \ingroup    bsp_api_commands_misc
* 
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long: version number of the BR_BSPAPI

* This command should be used, to make shure, that the application, that is loaded
* and the VxWorks-image used the same BR_BSPAPI.h header file. This is very importand,
* because type dfinitions, structures etc. may change over time.
* The VxWorks-Image is build using the BR_BSPAPI header and will return the version
* of this header, when this command is used. The application must compare it's own
* define with the value returned by this api.
* 
* Example code:
* 
* unsigned long ulBspApiVersion;
* 
* if(BRBspApi(BRBSPAPI_APIVERSION,0,&ulBspApiVersion) != BRBSPAPI_OK)
*   ERROR-Handler: Api-error
*   
* if(ulBspApiVersion != BR_BSPAPI_HEADER_VERSION)
*   ERROR-Handler: Different versions of BR_BSPAPI.h between application and VxWorks
*******************************************************************************/
#define BRBSPAPI_APIVERSION 67

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             get the frequency of the timestamp counter
*             
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long: frequency value
*******************************************************************************/
#define BRBSPAPI_TIMESTAMP_FREQ 68

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             get the counter value of the timestamp counter
* \ingroup    bsp_api_commands_misc
* 
* This timestamp counter works on multi processor platforms and on single processor
* platforms. The used timestand counter is not located in the cpu (like the
* pentium tsc) and is therefore smp safe.
* As a disadvantage must be mentioned, that this timer may only be accessed slowly.
* E.g. on an intel platform usually the hpet timer is used. This timer is located
* in the chipset an accessed via pci bus. Each read access usually lasts about 1us.
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long: counter value
*******************************************************************************/
#define BRBSPAPI_TIMESTAMP_CNT 69

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             get the frequency of the cpu bound timestamp counter
* \ingroup    bsp_api_commands_misc
*
* For restrictions, see command BRBSPAPI_CPUBOUND_STIMESTAMP_CNT
* 
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long: frequency value
*******************************************************************************/
#define BRBSPAPI_CPUBOUND_TIMESTAMP_FREQ 70

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             get the counter value of the cpu bound timestamp counter
* \ingroup    bsp_api_commands_misc
* 
* This timestamp counter does not work correctly on multi processor platforms, if
* called from different cpu cores. This timer is a fast accessable timestamp timer
* that is located inside the cpu. Therefore it can be accessed very much faster,
* than the timer used in the command BRBSPAPI_TIMESTAMP_CNT. The user has to take
* care, that this command is used on the same cpu core, if start and stop times
* are read for time measurements.
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned long: counter value
*******************************************************************************/
#define BRBSPAPI_CPUBOUND_TIMESTAMP_CNT 71

/**************************************************************************/ /**
* \defgroup   bsp_api_commands_time  bsp api commands time
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      simple network time protocol
* \ingroup    bsp_api_commands_time
*
* - pvInData:  pointer to BRsntpcIn_t data struct
* - pvOutData: pointer to BRsntpcOut_t data struct
*
* sample code:
* 
* void myTime(void)
* {
*   char ServerAddress[] = {“192.168.123.456”};
*   BRsntpcIn_t myBRsntpcIn;
*   BRsntpcOut_t myBRsntpcOut;
*  
*   myBRsntpcIn.pServerAddr = ServerAddress;   //set address of sntp server
*   myBRsntpcIn.port = 123;                    //portnumber of sntp server is 123
*   myBRsntpcIn.timeout = 100;                 //timeout 1000 ticks
*
*   if(BRBspApi (BRBSPAPI_SNTP_CLIENT, &myBRsntpcIn, &myBRsntpcOut) != BRBSPAPI_OK)
*   {
*     //BSP-API error handling
*   }
*  
*   //now the current time can be found in myBRsntpcOut.BRtimespec.timeSec and myBRsntpcOut.BRtimespec.timeNsec
*   //the status (OK or ERROR) can be found in  myBRsntpcOut.status
* }
*******************************************************************************/
#define BRBSPAPI_SNTP_CLIENT 72

typedef struct
{
  unsigned long timeSec;        /**< time in seconds*/                
  unsigned long timeNsec;       /**< time in nanoseconds*/
}
BRtimespec_t;

typedef struct
{
  char * pServerAddr;           /**< server IP address or hostname*/
  unsigned int timeout;         /**< timeout interval in ticks*/
  unsigned short port;          /**< port, if -1 the last stored port is used, or if never stored before, the default port 123*/
}
BRsntpcIn_t;

typedef struct
{
  BRtimespec_t BRtimespec;      /**< storage for retrieved time value*/
  int status;                   /**< return value of subroutine: OK or ERROR*/
}
BRsntpcOut_t;

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             set and get operating system time
* \ingroup    bsp_api_commands_time
* 
* \remarks    The time is to be based on the unix epoch 00:00:00 UTC on 1 January 1970
* 
* - pvInData:  pointer to struct BRtimespec_t for setting the operating system time or NULL (only get)
* - pvOutData: pointer to struct BRtimespec_t for getting the operating system time or NULL (only set)
*******************************************************************************/
#define BRBSPAPI_TIME_OS 73

/**************************************************************************/ /**
* \brief      set and get real time clock
* \ingroup    bsp_api_commands_time
* 
* \remarks    The time is to be based on the unix epoch 00:00:00 UTC on 1 January 1970
* 
* - pvInData:  pointer to struct BRtimespec_t for setting the real time clock or NULL (only get)
* - pvOutData: pointer to struct BRtimespec_t for getting the real time clock or NULL (only set)
*******************************************************************************/
#define BRBSPAPI_TIME_RTC 74

/**@}*/ /*end group bsp_api_commands_time**************************************/

/**************************************************************************/ /**
* \brief      disable the interrupts (msi's) of real time critical devices like
*             ethernet or sdhc
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  if NULL, all rt critical devices are affected, else tbd
* - pvOutData: not used
*******************************************************************************/
#define BRBSPAPI_RT_CRIT_DEVS_INT_DIS 75

/**************************************************************************/ /**
* \brief      enable the interrupts (msi's) of real time critical devices like
*             ethernet or sdhc
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  if NULL, all rt critical devices are affected, else tbd
* - pvOutData: not used
*******************************************************************************/
#define BRBSPAPI_RT_CRIT_DEVS_INT_EN 76

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             get the bsp version information
*
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to an array of unsigned longs.
*              - ulArray[0]: Version number
*              - ulArray[1]: Release number
*              - ulArray[2]: Patch number
*              - ulArray[3]: Build number
*******************************************************************************/
#define BRBSPAPI_BSPVERSION_GET 77


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_temp  bsp api commands temperature
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      get the temperature status
* \ingroup    bsp_api_commands_temp
*
* \remarks    This command must be called cyclic. Every 10 seconds is a good value
*             to start with (temperature does not change so rapidly).
*
*             The main information is the temperature status. The temperature status
*             may depend on more than one temperature sensor.
*
*             The range and the resolution of the one given temperature value can 
*             differ on different hardware type.
*
*             On the same hardware type the value can differ by exactness of the
*             temperature sensor or by component allowances.
*
*             Ignoring critical temperature status without sanctions can lead to
*             hardware precautions until the power supply switches off the device.
*
* - pvInData:  not used (NULL), if != NULL -> printf outputs
* - pvOutData: pointer to BRtempStatus_t data struct
*******************************************************************************/
#define BRBSPAPI_TEMP_STATUS 78

typedef struct
{
  unsigned long ulTempStatus;    /**<temperature status*/
  float fTempVal;                /**<temperature value*/
}
BRtempStatus_t;

#define BR_TEMPSTATUS_OK     0   /**<temperature ok       */
#define BR_TEMPSTATUS_WARN   1   /**<temperature warning  */
#define BR_TEMPSTATUS_CRIT   2   /**<temperature critical */

#define BR_TEMPSTATUS_ERR    3   /**<temperature error smb read*/

/**@}*/ /*end group bsp_api_commands_temp**************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_taskctrl  bsp api commands task control
* @{
*******************************************************************************/
#define BR_TASK_NONE             0
#define BR_TASK_WORKINGTIME      1
#define BR_TASK_NOTIFIER         2


typedef unsigned int BRcpuset_t;    /**<cpuset variable: bit0: cpu0, bit1 cpu1 etc.*/

typedef struct
{
  int iTask;             /**<define for task to be controlled (see defines above)*/
  int iResolutionInMs;   /**<task invoking interval in milliseconds (0 = don't care)*/
  int iPriority;         /**<task priority (0 = highest prio, 0xff = lowest prio, -1 = don't care)*/
  BRcpuset_t CpuSet;     /**<cpu affinity (0 = don't care)*/
}
BRtaskCtrl_t;

/**************************************************************************/ /**
* \brief      set or get the task priority of existing os or bsp tasks
* \ingroup    bsp_api_commands_taskctrl
*
* - pvInData:  pointer to data struct BRtaskCtrl_t to set params, or NULL if only get
* - pvOutData: pointer to data struct BRtaskCtrl_t to get params, or NULL if only set
*******************************************************************************/
#define BRBSPAPI_TASK_CTRL 79

/**@}*/ /*end group bsp_api_commands_taskctrl**********************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_extmod  bsp api commands extension module api
* @{
*
* See also \ref extmod_api
*******************************************************************************/

/**************************************************************************/ /**
* \brief      extension module api
*
* for more info see BRExtModApi_t declaration
*
* - pvInData:  pointer to BRExtModApi_t structure (data to write)
* - pvOutData: pointer to BRExtModApi_t structure (data to read)
*******************************************************************************/
#define BRBSPAPI_EXTMOD_API 80

/**************************************************************************/ /**
* \brief      connect extension module api routine
*
* for more info see BRIdentInfo_t declaration
*
* - pvInData:  pointer to BRExtModApiRegister_t structure
* - pvOutData: pointer to error value of subroutine (unsigned long)
*              (see return values of ident functions BREXTMOD_API_OK or BREXTMOD_API_ERROR_...)
*******************************************************************************/
#define BRBSPAPI_EXTMOD_API_REGISTER 81

/**@}*/ /*end group bsp_api_commands_ident*************************************/


/**************************************************************************/ /**
* \brief      get bsp compile date and time
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to string (char **)
*******************************************************************************/
#define BRBSPAPI_BSPCOMPILE_DATE_GET 82


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_notifier  bsp api commands notifier
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \ingroup    bsp_api_commands_notifier
* \addtogroup predefined_notifier_commands Pre-defined LED Commands
* @{
*******************************************************************************/
enum BRnotifierRequestNumber
{
  BR_NOTIFIER_NOP = 0x00,
  BR_LED_LOCK,
  BR_LED_RELEASE,
  BR_LED_PATTERN_ACTIVATE,
  BR_LED_STOP,
  BR_LED_GET_STATUS,
  BR_MESSAGE_SEND,
};

/**@}*/ /*end group predefined_notifier_commands*******************************/

/**************************************************************************/ /**
* \brief      notifier command message
* \ingroup    bsp_api_commands_notifier
*******************************************************************************/
typedef struct
{
  enum BRnotifierRequestNumber request; /**< command request number */
  void *argp;                           /**< pointer to arguments */
}
BRnotifierCmd_t;


/**************************************************************************/ /**
* \brief      notifier commands
* \ingroup    bsp_api_commands_notifier
*
* - pvInData:  pointer to typedef ::BRnotifierCmd_t with argp pointing to a
*   	         typdef ::BRledCmdArgs_t
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_NOTIFIER 83

/**************************************************************************/ /**
* \ingroup    bsp_api_commands_notifier
* \addtogroup predefined_led_colors Pre-defined LED Colors
* @{
*******************************************************************************/
enum BRledColors
{
  BR_LED_ALL_ON = 0,
  BR_LED_ALL_OFF,
  BR_LED_RED,
  BR_LED_GREEN,
  BR_LED_YELLOW,
  BR_LED_BLUE,
  BR_LED_CYAN,
  BR_LED_MAGENTA,
  BR_LED_ORANGE = BR_LED_YELLOW,
  BR_LED_WHITE = BR_LED_ALL_ON,
};

/**@}*/ /*end group predefined_led_colors**************************************/

/**************************************************************************/ /**
* \ingroup    bsp_api_commands_notifier
* \addtogroup predefined_led_indexs Pre-defined LED indexes
* @{
*******************************************************************************/
#define BR_LED_STA              1
#define BR_LED_ERR              2
#define BR_LED_BT               3
#define BR_LED_D                4
#define BR_LED_E                5
#define BR_LED_S                6
#define BR_LED_INF              7
#define BR_LED_DGN              8
#define BR_LED_PREDEFINED_END   9

/**@}*/ /*end group predefined_led_indexes*************************************/

/**************************************************************************/ /**
* \brief      describes LED status and its duration.
* \ingroup    bsp_api_commands_notifier
* 
* Patterns should be arranged as array, which has {0,0} as its last element,
* e.g.:
* \code
* BRledPattern_t foo_bar = {
*   {BR_LED_RED, 500},
*   {BR_LED_GREEN, 500},
*   {0,0},
* };
* \endcode
*******************************************************************************/
typedef struct
{
  /** color specified, can be one of \ref predefined_led_colors */
  unsigned short value;
  /** pattern will be valid until #timeout_ms milliseconds, 0: never time out*/
  unsigned short timeout_ms;
}
BRledPattern_t;

/**************************************************************************/ /**
* \brief      message of led command arguments 
* \ingroup    bsp_api_commands_notifier
*
* usage:
* \code
* BRledCmdArgs_t args = {
*   .dev_handle = 0,
*   .led = BR_LED_BOOT,
*   .pattern = &foo_bar,
*   .cycle = 3,
* };
* BRnotifierCmd_t cmd = {
*   .request = BR_LED_PATTERN_ACTIVATE,
*   .argp = &args,
* };
* 
* BRBspApi(BRAPI_NOTIFIER, &cmd, NULL);
* \endcode
*******************************************************************************/
typedef struct
{
  /** handle of the device where LED locates, 0 for mainboard built-in LED's */
  unsigned long dev_handle;
  /** Name of the specified LED, can be one of \ref predefined_led_indexs */
  int led;
  /** pointer to the pattern array, \c NULL means set all colors permanent on */
  BRledPattern_t *pattern;
  /** \c >0 count of cycles the pattern should repeat, \c -1 for infinite loop*/
  int cycle;
  /** pointer to the current pattern status */
  const BRledPattern_t *status;
}
BRledCmdArgs_t;

/**@}*/ /*end group bsp_api_commands_notifier**********************************/

/**************************************************************************/ /**
* \brief      get the status of operating mode switch
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to unsigned char; see also ::BRopModeSwitchState
*******************************************************************************/
#define BRBSPAPI_OPMODE_SWITCH 84

enum BRopModeSwitchState
{
  BR_OPMODE_SWITCH_RUN = 1,
  BR_OPMODE_SWITCH_STOP = 2, 
  BR_OPMODE_SWITCH_RESET = 3,
};


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_memmedia  bsp api commands memory media info
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      memory media info
* \ingroup    bsp_api_commands_memmedia
*
* - pvInData:  pointer to BR_mm_info_in_t (ctrl and device )
* - pvOutData: pointer to struct BR_mm_info_out_t for getting the memory media info (only get)
*******************************************************************************/
#define BRBSPAPI_MM_INFO 85

typedef struct MM_INFO
{
  long dev_type;                 /**< ATA_TYPE_ATA; ATA_TYPE_ATAPI; */
  unsigned long  sector_size;    /**< sector size */
  unsigned long long sector_cnt; /**< number of sectors */
  char product_name[40+24];      /**< name of product (string) */
  char product_revision[8+56];   /**< revision code (string) */
  char serial[20+44];            /**< serial number (string) */
  void *health_state;            /**< health status information */
}
MM_INFO_t;

typedef struct
{
  int ctrl;                     /**< controller instance */
  int drive;                    /**< drive number */
}
BR_mm_info_in_t;

typedef struct
{
  MM_INFO_t * mm_info_ptr;      /**< pointer to struct containing memory media info */
  int status;                   /**< status information (OK or ERROR)*/
}
BR_mm_info_out_t;

/**@}*/ /*end group bsp_api_commands_memmedia**********************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_commands_kernelconfigfile  bsp api commands kernel configuration file
* @{
*******************************************************************************/

/**************************************************************************/ /**
* \brief      kernel configuration file
*             get a parameter value of a defined section in the kernel config file 
* \ingroup    bsp_api_commands_kernelconfiginfo
*
* - pvInData:  pointer to a struct of type BRkernelConfigInfo_t
* - pvOutData: pointer to a struct of type BRkernelConfigParamValue_t
*******************************************************************************/
#define BRBSPAPI_KERNELCONFIG_INFO 86

#define BRBSPAPI_KERNELCONFIG_PARAMTYPE_UNDEF   0   /**<undefined parameter type*/
#define BRBSPAPI_KERNELCONFIG_PARAMTYPE_INT     1   /**<get parameter as integer*/
#define BRBSPAPI_KERNELCONFIG_PARAMTYPE_STRING  2   /**<get parameter as string (80 chars max)*/

#define BRBSPAPI_KERNELCONFIG_OK                0   /**<no error - ok*/
#define BRBSPAPI_KERNELCONFIG_ERROR_FUNCPARAM   1   /**<function parameter error*/
#define BRBSPAPI_KERNELCONFIG_ERROR_SECTION     2   /**<section not found*/
#define BRBSPAPI_KERNELCONFIG_ERROR_PARAMETER   3   /**<parameter not found*/
#define BRBSPAPI_KERNELCONFIG_ERROR_MODE        4   /**<mode unknown*/
#define BRBSPAPI_KERNELCONFIG_ERROR_WRITE       5   /**<write error (index or record full) */

#define BRBSPAPI_KERNELCONFIG_PARAM_STRINGLENG  80  /**<use this a buffer size, if parameter is a string*/

typedef struct
{
  char * sectionName;             /*name of the desired section (without brackets)*/
  char * paramName;               /*name of the desired parameter*/
  int    paramType;               /*parameter type (see constants)*/
  union {						              /*union of string and int value for write operations*/
    const char * stringValue;
    int intValue;
  } v;
}
BRkernelConfigInfo_t;

typedef union
{
  unsigned int uiErrorVal;        /*if the api returns with error, this is the error code*/
  int iParamValue;                /*if the mode is integer, this is the integer value*/
  char * pcParamValue;            /*if the mode is string, this is the pointer to the buffer,
                                    where the string will be copied to*/
}
BRkernelConfigParamValue_t;

/**@}*/ /*end group bsp_api_commands_kernelconfiginfo**************************/

/**************************************************************************/ /**
* \brief      <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n
*             get the bspapi version information
*
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to an array of unsigned longs.
*              - ulArray[0]: Version number
*              - ulArray[1]: Release number
*              - ulArray[2]: Patch number
*              - ulArray[3]: Build number
*******************************************************************************/
#define BRBSPAPI_BSPAPIVERSION_GET 87

/**************************************************************************/ /**
* \brief      get bspapi version string
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to version string (char **)
*******************************************************************************/
#define BRBSPAPI_BSPAPIVERSION_STRING_GET 88

/**************************************************************************/ /**
* \brief      get bspapi compile date and time
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to string (char **)
*******************************************************************************/
#define BRBSPAPI_BSPAPICOMPILE_DATE_GET 89

/**************************************************************************/ /**
* \brief      reboot api
* \ingroup    bsp_api_commands_misc
*
* \remarks    !!! attention !!!
*             BRBSPAPI_REBOOT_MODE_SHUTDOWN, BRBSPAPI_REBOOT_MODE_POWERCYCLE,
*             BRBSPAPI_REBOOT_MODE_HWRESET generate a signal to the power supply
*             circuit. The power supply circuit generates an interrupt first.
*             For this reason those commands will not work in an interrupt lock.
*             !!! Do not use those commands inside an interrupt lock !!!
*
* - pvInData:  pointer to int (int *: reboot mode, see constants)
* - pvOutData: pointer to int (int *: error code of subroutine, see constants)
*******************************************************************************/
#define BRBSPAPI_REBOOT 90

#define BRBSPAPI_REBOOT_MODE_SHUTDOWN     0  /*shutdown hardware*/
#define BRBSPAPI_REBOOT_MODE_POWERCYCLE   1  /*shutdown and restart hardware*/
#define BRBSPAPI_REBOOT_MODE_HWRESET      2  /*generate a hardware reset*/
#define BRBSPAPI_REBOOT_MODE_SWRESET      3  /*soft start the device (jump to bootloader)*/
#define BRBSPAPI_REBOOT_MODE_STARTINITFW  4  /*soft start the device and start init fw*/

#define BRBSPAPI_REBOOT_MODE_ACCEPTED     0
#define BRBSPAPI_REBOOT_MODE_UNKNOWN      1

/**************************************************************************/ /**
* \brief      get or set operating system system tick data
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to struct of type BRosSysTickInfo_t to set data, or NULL
* - pvOutData: pointer to struct of type BRosSysTickInfo_t to get data, or NULL
*              first set than get is done, if both actions are chosen
*              
* \remarks
* 
* This command should be used, to connect the system tick routine to another
* interrupt. This is done by retreiving the pointer to the system tick routine.
* This routine must be attached to the own interrupt. Then the original system
* tick interrupt must be disabled (or the tick stopped) and the own interrupt
* must be enabled (or the timer started).
* Although the original system tick source is not used after switching over,
* it is necesarry to set the osSysTickRate to the value, the own interrupt source
* has. This is because other routines often use the osSysTickRate to calculate
* delays (e.g. taskDelay(sysClkRateGet());)
*******************************************************************************/
#define BRBSPAPI_OSSYSTICK_INFO  91

typedef struct
{
  void (*osSysTickRtn)(void); /*systick routine (only get)*/
  int osSysTickRate;          /*systick rate (ticks per sec.), -1: do not change*/
  int osSysTickEnable;        /*0: disable, 1:enable, else: do not change (only set)*/
}
BRosSysTickInfo_t;

/**************************************************************************/ /**
* \brief      constants for commands init watchdog isr
* \ingroup    bsp_api_commands_watchdog
*******************************************************************************/
/**\defgroup bsp_api_commands_watchdog_isr_constants constants for watchdog isr
***@{*/
#define BRBSPAPI_WD_ISRINIT_OK                       0  /**<isr init successful*/
#define BRBSPAPI_WD_ISRINIT_UNKNWON_HW              -1  /**<unkown watchdog hardware*/
#define BRBSPAPI_WD_ISRINIT_BSPAPIERROR              1  /**<API error during isr init*/
#define BRBSPAPI_WD_ISRINIT_DEVICETYPE_UNKNOWN       2  /**<unkown device type*/
#define BRBSPAPI_WD_ISRINIT_PARENT_NOTINITIALIZED    3  /**<parent device not initialized*/
#define BRBSPAPI_WD_ISRINIT_ERROR_INTCONNECT         4  /**<error during int connect*/
/**@}*/ /*end group bsp_api_commands_watchdog_isr_constants********************/

/**************************************************************************/ /**
* \brief      initialize analog watchdog isr and connect callback
* \ingroup    bsp_api_commands_watchdog
*
* - pvInData:  pointer to void    : pointer to callback routine (void callback(void))
*                                   NULL if no callback shall be connected
* - pvOutData: pointer to integer : error value of subroutine (see defines above)
*******************************************************************************/
#define BRBSPAPI_WDANA_ISRINIT 92

/**************************************************************************/ /**
* \brief      initialize digital watchdog isr and connect callback
* \ingroup    bsp_api_commands_watchdog
*
* - pvInData:  pointer to void    : pointer to callback routine (void callback(void))
*                                   NULL if no callback shall be connected
* - pvOutData: pointer to integer : error value of subroutine (see defines above)
*******************************************************************************/
#define BRBSPAPI_WDDIG_ISRINIT 93

/**************************************************************************/ /**
* \brief      get or set test mode
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to int (int *: set function, see constants)
* - pvOutData: pointer to int (int *: get test mode, see constants)
*
* \remarks
* 
* This command can be used to implement a special test functionality; this
* test functionality (e.g. testbuddy) can be switched on (e.g. in indrac.ini)
*******************************************************************************/
#define BRBSPAPI_TESTMODE 94

#define BRBSPAPI_SET_TEST_MODE_OFF  0  /*function: set testmode inactive*/
#define BRBSPAPI_SET_TEST_MODE_ON   1  /*function: set testmode active*/
#define BRBSPAPI_GET_TEST_MODE      2  /*function: get testmode*/

#define BRBSPAPI_TEST_MODE_OFF      0x70656e74
#define BRBSPAPI_TEST_MODE_ON       0x2d506369
#define BRBSPAPI_TEST_MODE_UNKNOWN  0x0000AFFE

/**************************************************************************/ /**
* \brief      get validity of windows life counter
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: pointer to int (int *: get validity of windows life counter, see constants)
*
* \remarks
* 
* This command should be used, to get validity of windows life counter (system PRxx). 
* This should be done after a constant time interval (e.g. 10sec).
* If the counter is not valid, the system has to decide what to do:
*  - shutting down ?
*  - waiting (may be windows is rebooting) ?
*  - ...
* In case life counter is not valid, no data can be saved.
*******************************************************************************/
#define BRBSPAPI_LIFECOUNT_STATE 95

#define BRBSPAPI_LIFECOUNT_VALID            0x0000
#define BRBSPAPI_LIFECOUNT_INVALID_WIN_DOWN 0x0001
#define BRBSPAPI_LIFECOUNT_INVALID_SYSTRAY  0x0002
#define BRBSPAPI_LIFECOUNT_INVALID_UPSSVC   0x0004
#define BRBSPAPI_LIFECOUNT_INVALID_IPCSVC   0x0008

/**************************************************************************/ /**
* \brief      kernel configuration file write
*             set a parameter value of a defined section in the kernel config file 
* \ingroup    bsp_api_commands_kernelconfiginfo
*
* - pvInData:  pointer to a struct of type BRkernelConfigInfo_t
* - pvOutData: pointer to a struct of type BRkernelConfigParamValue_t
*******************************************************************************/
#define BRBSPAPI_KERNELCONFIG_WRITE 96

/**************************************************************************/ /**
* \brief      register callback function for RTH-Events
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  pointer to void: pointer to callback routine (void callback(void))
* - pvOutData: pointer to unsigned int: error value of subroutine 
*******************************************************************************/
#define BRBSPAPI_REG_RTHEVENT 97

/**************************************************************************/ /**
* \brief      RTH-Events
* \ingroup    bsp_api_commands_misc
*******************************************************************************/
#define BR_RTHEVENT_SHUTDOWN_VXW  1
#define BR_RTHEVENT_SHUTDOWN_ALL  2
#define BR_RTHEVENT_BSOD          3
#define BR_RTHEVENT_UPS_POWERFAIL 4    

/*data structure used in the BRBspApi command BRBSPAPI_REG_RTHEVENT*/
typedef struct
{
  unsigned long (*BRrthEventCallbackRtn)(unsigned char InData); /*callback routine*/
}
BRrthEventCallback_t;

/*return values for callback functions*/
#define RTHEVENTCB_OK                    0x00 /**<no error*/
#define RTHEVENTCB_ERROR_HANDLE_NOMATCH  0x01 /**<selected handle does not exist*/
#define RTHEVENTCB_ERROR_NOFUNCPTR       0x02 /**<ident info function pointer not initialized*/



#define BRBSPAPI_DEBUG_MODE_ACTIVE     1
#define BRBSPAPI_DEBUG_MODE_INACTIVE   0

/**************************************************************************/ /**
* \brief       enable the debug port
* \ingroup     bsp_api_commands_misc
*
* \remarks     VxWorks6.9 targets: Enable the WDB, set debug mode to active
*              VxWorks6.3 targets: WDB cannot be enabled/disabled dynamically
*                                  Only set debug mode to active
*
* - pvInData:  not used, set to 0
* - pvOutData: not used, set to 0
*******************************************************************************/
#define BRBSPAPI_DEBUG_ENABLE 98

/**************************************************************************/ /**
* \brief       disable the debug port
* \ingroup     bsp_api_commands_misc
*
* \remarks     VxWorks6.9 targets: Disable the WDB, set debug mode to inactive
*              VxWorks6.3 targets: WDB cannot be enabled/disabled dynamically
*                                  Only set debug mode to inactive
*
* - pvInData:  not used, set to 0
* - pvOutData: not used, set to 0
*******************************************************************************/
#define BRBSPAPI_DEBUG_DISABLE 99

/**************************************************************************/ /**
* \brief       get the actual status of the debug mode
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  not used, set to 0
* - pvOutData: pointer to int (for return value of debug mode)
*******************************************************************************/
#define BRBSPAPI_DEBUG_MODE_GET 100

/**************************************************************************/ /**
* \brief       save nvram data
* 
* \remarks     only for use in interrupt or exception routine
*              reboot after execution
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  not used, set to 0
* - pvOutData: not used, set to 0
*******************************************************************************/
#define BRBSPAPI_SAVE_NVRAM 101

/**************************************************************************/ /**
* \brief       register a hook routine for duplicate ip conflicts
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  pointer to hook routine
* - pvOutData: not used, set to 0
*******************************************************************************/
#define BRBSPAPI_DUPLICATE_IP_HOOK_SET 102

#define BR_DUPIP_UNDEF        0   /*undefined error*/
#define BR_DUPIP_WARNING      1   /*warning: other device tried to use our ip*/
#define BR_DUPIP_ERROR_GARP   2   /*error: other device used desired ip, recognized during garp*/
#define BR_DUPIP_ERROR_PROBE  3   /*error: other device used desired ip, recognized during arp probe*/

typedef struct
{
  int flags;                /*kind of conflict: see constants above*/
  int lenIfName;            /*lenght of interface name without EOS*/
  unsigned char * pIfName;  /*interface name without EOS*/
  int lenIp;                /*lenght of ip address*/
  unsigned char * pIp;      /*conflict ip address*/
  int lenMacOwn;            /*lenght of own mac address*/
  unsigned char * pMacOwn;  /*own mac address*/
  int lenMacOth;            /*lenght of other mac address*/
  unsigned char * pMacOth;  /*other mac address*/
  int lenInfoAdd;           /*lenght of additional info*/
  unsigned char * pInfoAdd; /*additional info*/
}
BRdupIpInfo_t;

/*function prototype for hook routine*/
typedef void (* BRduplicateIpHook_t)(BRdupIpInfo_t dupInfo);


/**************************************************************************/ /**
* \brief       start a daemon task that calls routec until success or timeout
* 
* \remarks     use the routec daemon instead of routec, because the address
*              conflict detection mechanism lets ifconfig need several seconds
*              to apply the settings. As long as the ifconfig settings are not
*              done, routec will fail. The routec daemon will do the work for
*              you. The routec daemon does a retry every second up to 20 retries.
*              The priority of the daemon task can be set by an entry in the
*              KernelConfig.ini
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  char * (string to deliver to routec) 
* - pvOutData: not used, set to 0
*******************************************************************************/
#define BRBSPAPI_ROUTEC_DAEMON 103


/**************************************************************************/ /**
* \brief       connect a hook routine for fpu exceptions on XM1x
* 
* \remarks     
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  pointer to hook routine
* - pvOutData: not used, set to 0
*******************************************************************************/
#define BRBSPAPI_FPU_EXC_HOOK_ADD 104

/*
* the hook routine should look like this:
* void BRfpuExcHook(TASK_ID tid, int vecNum, void * pEsf);
*
* vecNum is a combination of FPU_EXC_ID_MAGIC and an exception id of the 
* exception occured (id as defined in vfpArmLib.h)
* e.g. division by zero: FPU_EXC_ID_MAGIC | FPSCR_DZC_BIT = 0xca1c0002
*/

/*constants for fpu exception info (bit definitions see vfpArmLib.h)*/
#define FPU_EXC_ID_MAGIC 0xca1c0000   /*magic number that identifies a fpu exception*/
#define FPU_EXC_ID_MASK  0x000000FF   /*mask for fpu exc id bits*/
#define FPU_EXC_EN_MASK  0x0000FF00   /*mask for fpu exc enable bits*/


/**************************************************************************/ /**
* \brief       <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n       
*              get a list of device names actually available similar to iosDevShow()
* 
* \remarks     removable devices may be gone short after calling this command
*              the user has to deliver the memory for the list
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  pointer to an array of unsigned long.
*              - ulArray[0]: number of entries in list (delivered by user)
*              - ulArray[1]: number of characters per entry (delivered by user)
*              for number of charcters per entry the user should use the constant
*              MAX_DEVNAME defined in the VSB_CONFIG_FILE
*              
*              When command is finished, ulArray[] contains the following information:
*              - ulArray[0]: number of entries needed
*                            (if greater than delivered -> list too small, list shortened)
*              - ulArray[1]: number of characters needed for biggest entry
*                            (if greater than delivered -> entry too small, string shortened)
*              
* - pvOutData: pointer to the buffer where the list will be generated
* 
* \return:     BRBSPAPI_OK if number of entries and number of chars big enough
*              BRBSPAPI_ERROR_IN_SUBRTN if number of entries or number of chars too small
*******************************************************************************/
#define BRBSPAPI_DEV_LIST_GET 105


/**************************************************************************/ /**
* \brief       <B>USER MODE</B> supported (see also \ref bsp_api_commands_user)\n       
*              get the license file information (EOS terminated ascii data)
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  pointer to an unsigned long, containing:
*              - size of buffer pointed to by pvOutData (delivered by user)
*              
*              When command is finished, this unsigned long contains the following information:
*              - size of license file including EOS
*              
* - pvOutData: pointer to the buffer where the data will be stored
*              if NULL, only the size of license file will be delivered in pvInData but no data
*
* \remarks     if delivered buffer is too small for the license file, only the fitting
*              license information will be copied to the buffer and it will be terminated with EOS     
* 
* \return:     BRBSPAPI_OK if buffer was big enough
*              BRBSPAPI_ERROR_IN_SUBRTN if buffer was too small
*******************************************************************************/
#define BRBSPAPI_LIC_FILE_GET 106


/**************************************************************************/ /**
* \brief       ask for availability of Cache Allocation Technologie
* 
* \remarks     CAT is only supported at some i-core processors > Gen.4
*              and makes only sense on multicore processors. Not supported
*              on SKL-U but on SKL-H with special settings  
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  not used
* - pvOutData: int pointer (get availability of CAT, see constants)
* 
* \return:     BRBSPAPI_OK if cmd is available        
*              BRBSPAPI_ERROR_PARAM if pvOutData == NULL
*              BRBSPAPI_COMMAND_UNKNOWN if cmd is not available
*******************************************************************************/
#define BRBSPAPI_CAT_AVAILABLE 107

#define BR_CAT_NOTAVAILABLE      0
#define BR_CAT_AVAILABLE         1

/**************************************************************************/ /**
* \brief       set amount of fixed Cache via Cache Allocation Technologie
* 
* \remarks     use percentage of total LLC (last level cache) for this core, allowed values: <= 100
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  unsigned long pointer (set percentage what amount (%) of LLC should be used, see constants)
* - pvOutData: not used, set to 0
* 
* \return:     BRBSPAPI_OK if cmd is available        
*              BRBSPAPI_ERROR_PARAM if pvInData == NULL or if pvInData != allowed values
*              BRBSPAPI_COMMAND_UNKNOWN if cmd is not available
*              BRBSPAPI_CAT_NOTAVAILABLE if CAT could not be set
*******************************************************************************/
#define BRBSPAPI_CAT_SET 108

#define BRBSPAPI_CAT_NOTAVAILABLE  -1

/* constants for proposed values */
#define BR_CAT_PERC_0      0
#define BR_CAT_PERC_30    30
#define BR_CAT_PERC_60    60
#define BR_CAT_PERC_90    90

/**************************************************************************/ /**
* \brief       get amount of assigned cache via Cache Allocation Technologie
* 
* \remarks     
* 
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  not used, set to 0 
* - pvOutData: unsigned long pointer (get percentage what amount (%) of LLC is assigned)
* 
* \return:     BRBSPAPI_OK if cmd is available        
*              BRBSPAPI_ERROR_PARAM if pvOutData == NULL
*              BRBSPAPI_COMMAND_UNKNOWN if cmd is not available
*******************************************************************************/
#define BRBSPAPI_CAT_GET 109


/**************************************************************************/ /**
* \brief       get amount of cache misses in LLC for the calling core
* 
* \remarks     how many cache misses at LLC happened (since last call of function)
*              
*
* \ingroup     bsp_api_commands_misc
*
* - pvInData:  not used, set to 0 
* - pvOutData: unsigned long pointer (get amount of cache misses in LLC of calling (physical) core)
* 
* \return:     BRBSPAPI_OK if cmd is available        
*              BRBSPAPI_ERROR_PARAM if pvOutData == NULL
*              BRBSPAPI_COMMAND_UNKNOWN if cmd is not available
*******************************************************************************/
#define BRBSPAPI_GET_LLC_CACHE_MISSES 110


/**************************************************************************/ /**
* \brief      configure message signalled interrupts of br devices
*             available starting with IC_BR_BSPAPI_V14.20.14
* 
* \remarks    this command configures msi and msi-x for BR devices
*             this command replaces the command BRBSPAPI_DEVICES_MSI_CONFIG
*             BRBSPAPI_DEVICES_MSI_CONFIG is still available for compability
* 
* \ingroup    bsp_api_commands_brdevices
*
* - pvInData:  pointer to BRdeviceMsiConf_t structure
* - pvOutData: pointer to BRdeviceMsiInfo_t structure
*******************************************************************************/
#define BRBSPAPI_DEVICES_MSI_CONF 111


/**************************************************************************/ /**
* \brief      last command = dummy
* \ingroup    bsp_api_commands_misc
*
* - pvInData:  not used (NULL)
* - pvOutData: not used (NULL)
*******************************************************************************/
#define BRBSPAPI_LAST 112


/**@}*/ /*end group bsp_api_commands********************************************
*******************************************************************************/


/**************************************************************************/ /**
* \defgroup   bsp_api_syscall  bsp api system call
* @{
*******************************************************************************/

#define BR_BSPAPI_SCG     2
#define BR_BSPAPI_SCIX    0

/*******************************************************************************
* syscall version of BRBspApi()
*
* syscall number: SYSCALL_NUMBER(BR_BSPAPI_SCG, BR_BSPAPI_SCIX)
* 
* syscall arguments: arg1: ulCommand for BRBspApi
*                    arg2: pvInData for BRBspApi 
*                    arg3: pvOutData for BRBspApi 
*                    arg4: return value of BRBspApi 
* 
*******************************************************************************/

/*******************************************************************************
* Example for a BR_BSPAPI system call:
* 
* 
* void printOsTime(void)
* {
*   BRtimespec_t BRtimespec;
*   unsigned long ulApiError;
*   int syscallError;
* 
*   syscallError = syscall((_Vx_usr_arg_t)BRBSPAPI_TIME_OS,
*                          (_Vx_usr_arg_t)NULL,
*                          (_Vx_usr_arg_t)&BRtimespec,
*                          (_Vx_usr_arg_t)&ulApiError,
*                          0, 0, 0, 0,
*                          SYSCALL_NUMBER(BR_BSPAPI_SCG, BR_BSPAPI_SCIX);
* 
*   if(syscallError != OK)
*   {
*     printf("syscall returned ERROR\n");
*   }
*   
*   if(ulApiError != BRBSPAPI_OK)
*   {
*     printf("BSPAPI returned ERROR\n");
*   }
*   
*   printf("OS time: %d sec, %d nsec\n", BRtimespec.timeSec, BRtimesoec.timeNsec);
* }
* 
*******************************************************************************/

/**@}*/ /*end group bsp_api_syscall*********************************************
*******************************************************************************/


/**@}*/ /*end group bsp_api*****************************************************
*******************************************************************************/


#if __cplusplus
} /* extern "C" */
#endif

#endif /*defined BR_BSPAPI_H*/
