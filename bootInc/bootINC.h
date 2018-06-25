
#ifndef BOOTINC_H_
#define BOOTINC_H_

#include <stdint.h>
#include <string.h>         /** For memset/memcpy */
#include "spi.h"            /** For SPI Driver */
#include "FreeRTOS.h"       /** For semaphore  */
#include "os_semphr.h"      /** For semaphore  */
#include "os_task.h"
#include "Command.h"        /** For protocol commands */
#include "gio.h"            /** For LED blinking */
#include "het.h"            /** For LED blinking */
#include "External_WD.h"    /** For firmware array */

/**
* @struct tDATA_PACKET
* @brief Packet includes 64-byte blocks of data plus two-byte sequence count and two-byte CRC
*/
#define BLOCK_SIZE          128U
#define PROTO_DATA_SIZE     64U
typedef struct
{
    uint8_t  u8Data[PROTO_DATA_SIZE];   /**< Data block     */
    uint16_t u16SeqCnt;                 /**< Sequence count */
    uint16_t u16CRC;                    /**< Two-byte CRC   */
}tDATA_PACKET;

/**
* @struct tFIRMWARE_PARAM
* @brief Two-byte CRC over the whole firmware and the length of firmware in bytes
*/
typedef struct
{
    uint16_t u16FWCRC;    /**< Two-byte CRC over firmware */
    uint16_t u16FWLen;    /**< Length of the firmware     */
}tFIRMWARE_PARAM;

/**
* @enum ePACKET_STATUS
* @brief Status of the data packet.
*/
typedef enum
{
    ePACKET_Ok           = 0,  /**< No error              */
    ePACKET_CRCError     = 1,  /**< CRC mismatch          */
    ePACKET_SNError      = 2   /**< Sequence number error */
}ePACKET_STATUS;


typedef union myCmd{
    uint8_t         bufferCMD[4];
    eRESPONSE_ID    returnValue;
    eCOMMAND_ID     receivedvalue;
}tCmdUnion;

typedef union myPayload{
    tDATA_PACKET    packet;
    uint8_t         bufferPLD[68];
}tPldUnion;

typedef union myAppData{
    tFIRMWARE_PARAM Firmware;
    uint8_t         bufferData[4];
}tAppDataUnion;

typedef enum {
    eDefaultState = 0,
    eFlashEraseCMD,
    eWriteMemory,
    ePayloadReceive,
    ePayloadCheck,
    ePayloadCRC,
    ePayloadCRCCheck,
    eWriteAppCRC,
    eFinishUpdate,
    eFlashVerifyApplication,
    eStartAppCMD
}tProtoState;

void InitStateMachine(void);
uint8_t RunStateMachine(void);
#endif // #define BOOTINC_H_
