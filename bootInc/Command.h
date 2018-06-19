/******************************************************************************/
/**
* @file Command.h
* @brief Definition of commands
*
*******************************************************************************/
#ifndef COMMAND_H
#define COMMAND_H

/* ***************** Header / include files ( #include ) **********************/
/* *************** Constant / macro definitions ( #define ) *******************/
/* ********************* Type definitions ( typedef ) *************************/
/**
* @enum eCOMMAND_ID
* @brief Send commands.
*/
typedef enum
{
    eCMD_EraseFlash     = 0xFE01, /**< Erase current firmware in flash                                       */
    eCMD_WriteMemory    = 0xFD02, /**< Switch to bootloader mode to expect data packets for writing to flash */
    eCMD_BootloadMode   = 0xFC03, /**< Needs to come within 1s after start up to stay in bootloader mode     */
    eCMD_WriteCRC       = 0xFB04, /**< Finish writting application, write 2 bytes CRC and 2 bytes length     */
    eCMD_Finish         = 0xFA05, /**< End of bootloader mode; jump to application code                      */
    eCMD_NotValid       = 0x0000  /**< */
}eCOMMAND_ID;

/**
* @enum eRESPONSE_ID
* @brief Reply response.
*/
typedef enum
{
    eRES_Ready          = 0x0EF1, /**< Successfully switched to bootloader mode         */
    eRES_Abort          = 0x0dF2, /**< Not recoverable error happened, only can abort   */    
    eRES_OK             = 0x0CF3, /**< Last command was processed correctly             */
    eRES_Error          = 0x0BF4, /**< Last command had an error                        */
    eRES_AppCrcErr      = 0x0AF5  /**< Application CRC error                            */
}eRESPONSE_ID;

/* ***************** Global data declarations ( extern ) **********************/
/* ***** External parameter / constant declarations ( extern const ) **********/
/* ********************** Global func/proc prototypes *************************/

#endif

/* end of Command.h */
