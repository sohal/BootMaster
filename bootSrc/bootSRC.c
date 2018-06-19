/*
 * bootDrcSPI_LL.c
 *
 *  Created on: 18 Jun 2018
 *      Author: tosoh
 */

#include <bootINC.h>
static void SPI_RxTx(uint8_t *pbuffer, uint16_t size);
static void SPI_LLInit(void);
static uint16_t Proto_CRC(const uint8_t *data, uint16_t size, uint16_t startVal);
static void Proto_PreparePacket(uint16_t pktCounter);
static const uint16_t cu16CRC16 = 0xAA55U;
static SemaphoreHandle_t xSemaphore;
static uint16_t spiBuffer[BLOCK_SIZE] = {0};
static spiDAT1_t spi3Data;
static uint32_t spiInitDone = 0;
static tPldUnion mtPayload;
static uint8_t *mpu8FileBuffer = NULL;
static uint16_t mu16FullPackets = 0U;
static uint16_t mu16PartialPackets = 0U;
/**
 * void SPI_LLInit(void)
 * @brief The low level initialization of SPI Master in TMS570 SPI3
 * The Chip select is held low until tx/rx
 * The format for data is 0 (8 bit, CPOL, CPHA)
 * There is no delay configured
 * The semaphore is assigned here.
 * The SPI local buffer is cleared
 */
void SPI_LLInit(void)
{
    vSemaphoreCreateBinary( xSemaphore );
    spi3Data.CS_HOLD = TRUE;
    spi3Data.DFSEL = SPI_FMT_0;
    spi3Data.WDEL = FALSE;
    spiInitDone = TRUE;
    memset(spiBuffer, 0, sizeof(spiBuffer));
    mpu8FileBuffer = (uint8_t*)External_WD;
}

void SPI_RxTx(uint8_t *pbuffer, uint16_t size)
{
    uint16_t i = 0;

    if(spiInitDone)
    {
        memset(spiBuffer, 0, sizeof(spiBuffer));

        for(i = 0; i < size; i++)
        {
            spiBuffer[i] = (uint16_t)pbuffer[i];
        }

        if(xSemaphore != NULL)
        {
            xSemaphoreTake(xSemaphore, 0);

            spiTransmitAndReceiveData(spiREG3,
                                      &spi3Data,
                                      size,
                                      spiBuffer,
                                      spiBuffer);

            xSemaphoreGive(xSemaphore);
        }
    }
}

/**
 * @brief Sends a block to the SPI device
 * @param block Block to send
 */
void Boot_SendData(uint8_t *block, uint8_t length)
{
    SPI_RxTx(block, length);
}

/**
 * @brief Sends specified command to the SPI device
 * @param command
 */
void Boot_SendCommand(uint16_t command)
{
    uint8_t cmd[2];
    cmd[0] = (uint8_t)((command) & 0xFF);
    cmd[1] = (uint8_t)((command >> 8) & 0xFF);

    SPI_RxTx(cmd, sizeof(uint16_t));
}

/**
 * @brief Reads the last response from SPI device
 * @return response Response
 */
uint16_t Boot_GetResponse(void)
{
    uint16_t retVal;
    uint8_t ui8Data[2] = {0,0};
    SPI_RxTx(ui8Data, 2);
    retVal = (spiBuffer[0] & 0xFFU) | ((uint16_t)(spiBuffer[1] & 0x00FFU) << 8U);
    return (retVal);
}

/**
 * @brief Reads the last response from SPI device
 * @return response Response
 */
void InitStateMachine(void)
{
    SPI_LLInit();
}
/**
 * @brief Reads the last response from SPI device
 * @return response Response
 */
void RunStateMachine(void)
{
    eRESPONSE_ID response;

    while(1)
    {
        /*! Restore all dynamic counters for firmware transfer*/
        mtPayload.packet.u16SeqCnt = 0;
        /*! Start Bootloader mode for STM */
        Boot_SendCommand(eCMD_BootloadMode);
        vTaskDelay(1U);
        /*! Get response from STM */
        response = (eRESPONSE_ID)Boot_GetResponse();
        /*! Are we ready ? */
        if(eRES_Ready == response)
        {
            /** Proceed with the next command */
            //mtStateNext = eDefaultState;
            vTaskDelay(5);
        }
        vTaskDelay(5);
    }
//
//    switch (mtStateNow)
//    {
//    case eDefaultState:
//        /*! Restore all dynamic counters for firmware transfer*/
//        mtPayload.packet.u16SeqCnt = 0;
//        /*! Start Bootloader mode for STM */
////        Boot_SendCommand(eCMD_BootloadMode);
////        OSTimeDly(1U);
//        /*! Get response from STM */
//        response = static_cast<eRESPONSE_ID>(Boot_GetResponse());
//        /*! Are we ready ? */
//        if(eRES_Ready == response)
//        {
//            /** Proceed with the next command */
//            mtStateNext = eDefaultState;
//        }
//        break;
//
//    case eFlashEraseCMD:
//        /*! Send STM command to erase its program flash */
//        Boot_SendCommand(eCMD_EraseFlash);
//        OSTimeDly(2500U);
//        /*! Get response from STM */
//        response = static_cast<eRESPONSE_ID>(Boot_GetResponse());
//        /*! Are we ready ? */
//        if(eRES_OK == response)
//        {
//            /** Proceed with the next command */
//            mtStateNext = eWriteMemory;
//        }
//        break;
//
//    case eWriteMemory:
//        /*! Send STM command to enter writing program memory */
//        OSTimeDly(100U);
//        Boot_SendCommand(eCMD_WriteMemory);
//        OSTimeDly(1U);
//        /*! Get response from STM */
//        response = static_cast<eRESPONSE_ID>(Boot_GetResponse());
//        /*! Are we ready ? */
//        if(eRES_OK == response)
//        {
//            /** Proceed with the next command */
//            mtStateNext = ePayLoadSend;
//            OSTimeDly(1000U);
//        }
//        break;
//
//    case ePayLoadSend:
//        /*! Prepare Payload to be sent */
//        Proto_PreparePacket(mtPayload.packet.u16SeqCnt);
//        /*! Send Payload to the STM */
//        Boot_SendData(mtPayload.bufferPLD, sizeof(tPldUnion));
//        OSTimeDly(1000U);
//        mtStateNext = ePayloadAckRecv;
//
//    case ePayloadAckRecv:
//        /*! Get response from STM */
//        response = static_cast<eRESPONSE_ID>(Boot_GetResponse());
//        /*! Are we ready ? */
//        if(eRES_OK == response)
//        {
//
//            /*! Check if was the last packet to be sent */
//            if(mtPayload.packet.u16SeqCnt > mu16FullPackets)
//            {
//                /*! No more packet to send thus */
//                mtStateNext = eWriteAppCRC;
//            }else
//            {
//                /*! Only then move to the next packet to be sent */
//                mtPayload.packet.u16SeqCnt++;
//                mtStateNext = eWriteMemory;
//            }
//        }else{
//            if(eRES_Error == response)
//            {
//                mtStateNext = eWriteMemory;
//            }
//        }
//
//        break;
//
////    case ePayLoad0:
////        /*! Prepare Payload to be sent */
////        Boot_PreparePacket(1);
////        /*! Send Payload to the STM */
////        Boot_SendData(mtPayload.bufferPLD, sizeof(tPldUnion));
////        OSTimeDly(1000U);
////        /*! Get response from STM */
////        response = static_cast<eRESPONSE_ID>(Boot_GetResponse());
////        /*! Are we ready ? */
////        if(eRES_OK == response)
////        {
////            /*! Only then move to the next packet to be sent */
////            mtPayload.packet.u16SeqCnt++;
////            /*! Check if was the last packet to be sent */
////            if(mtPayload.packet.u16SeqCnt > mu16FullPackets)
////            {
////                /*! No more packet to send thus */
////                mtStateNext = eWriteAppCRC;
////            }else
////            {
////                mtStateNext = ePayloadReceive;
////                OSTimeDly(200U);
////            }
////        }else if(eRES_Abort == response)
////        {
////            mtStateNext = eDefaultState;
////        }
////        break;
//
//    default:
//        mtStateNext = eDefaultState;
//        break;
//    }
//
//    if(mtStateNow == mtStateNext)
//    {
//        // sohal mu16StateCounter++;
//    }else
//    {
//        mu16StateCounter = 0U;
//    }
//    if(mu16StateCounter < 10U)
//    {
//        retVal = eKBErrorOK;
//    }
//    mtStateNow = mtStateNext;
//    OSTimeDly(1U);
//
//    return (retVal);
}

/**
 * void Boot_PreparePacket(uint16_t pktCounter)
 * @brief
 */
void Proto_PreparePacket(uint16_t pktCounter)
{
    uint16_t i = 0;
    uint8_t *pstartPosition = (pktCounter * BLOCK_SIZE) + mpu8FileBuffer;
    uint16_t u16TxLen = BLOCK_SIZE;

    memset(mtPayload.packet.u8Data, 0U, BLOCK_SIZE);
    /**< If this is the last packet, we should calculate the u16TxLen from the remainder bytes */
    if(pktCounter == mu16FullPackets)
    {
        u16TxLen = mu16PartialPackets;
    }
    /**< Copy all the relevant bytes from source to destination */
    memcpy(mtPayload.packet.u8Data, pstartPosition, u16TxLen);

    /*! save CRC temporarily here in i */
    i = Proto_CRC(mtPayload.packet.u8Data, BLOCK_SIZE, 0U);
    mtPayload.packet.u16CRC = ((i & 0xFF00) >> 8U) | ((uint16_t)(i & 0x00FF) << 8U);
}


/******************************************************************************/
/**
* uint16_t CRCCalc16(const uint8_t *data, uint16_t size, uint16_t startVal)
* @brief Generate 16 bits CRC for an input array.
*
* @param[in] data pointer to data array
* @param[in] size byte number of the array
* @param[in] startVal start value of the CRC calculation
* @returns   calculated 16 bits CRC.
*******************************************************************************/
uint16_t Proto_CRC(const uint8_t *data, uint16_t size, uint16_t startVal)
{
    uint16_t out = startVal;
    uint8_t bits_read = 0, bit_flag;
    /* Sanity check */
    if(data == NULL)
    {
        return 0;
    }
    while(size > 0)
    {
        bit_flag = out >> 15;
        /* Get next bit */
        out <<= 1;
        out |= (*data >> (7 - bits_read)) & 1;
        /* Increment bit counter */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }
        /* Cycle check */
        if(bit_flag)
        {
            out ^= cu16CRC16;
        }
    }
    return out;
}
