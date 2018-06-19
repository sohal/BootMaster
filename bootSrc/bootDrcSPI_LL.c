/*
 * bootDrcSPI_LL.c
 *
 *  Created on: 18 Jun 2018
 *      Author: tosoh
 */

#include "bootDrvSPI_LL.h"
/** Include string.h for memset */
#include <string.h>

static SemaphoreHandle_t xSemaphore;
static uint16_t spiBuffer[128] = {0};
static spiDAT1_t spi3Data;
static uint32_t spiInitDone = 0;
void DrvSPI_LLInit(void)
{
    vSemaphoreCreateBinary( xSemaphore );
    spi3Data.CS_HOLD = TRUE;
    spi3Data.DFSEL = SPI_FMT_0;
    spi3Data.WDEL = FALSE;
    spiInitDone = TRUE;
}

void DrvSPI_LLTxRx(uint8_t *pbuffer, uint16_t size)
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
//            spiTransmitData(spiREG3,
//                            &spi3Data,
//                            size,
//                            spiBuffer);

            xSemaphoreGive(xSemaphore);
        }
    }
}
