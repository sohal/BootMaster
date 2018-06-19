
#ifndef BOOTDRVSPI_LL
#define BOOTDRVSPI_LL

#include <stdint.h>
#include "spi.h"        /** For SPI Driver */
#include "FreeRTOS.h"   /** For semaphore  */
#include "os_semphr.h"  /** For semaphore  */
void DrvSPI_LLTxRx(uint8_t *pbuffer, uint16_t size);
void DrvSPI_LLInit(void);
#endif // #define BOOTDRVSPI_LL
