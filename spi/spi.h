#ifndef SPI_H
#define SPI_H

#include "stdint.h"

#include "stm32h7xx_ll_spi.h"

/* Blocking SPI master transport on LL with bounded waits - symmetric to the
   i2c module. Drop-in replacement for the project MCU/spi.c primitives:
   SPI_SendData / SPI_ReceiveData keep the same signatures, so libraries
   built on the constants-macro layer (max31856, eeprom_mc25lc128) work
   with it unchanged.

   The peripheral itself is configured elsewhere (mode, baud rate, soft NSS).
   On STM32H7 call SPI_MasterStart() once after LL_SPI_Init: the H7 master
   does not clock anything until CSTART is set.

   A stuck flag latches a sticky error instead of hanging the firmware:
   reads then return 0, so write-readback checks (e.g. MAX31856_probe)
   fail loudly instead of looping forever. Poll SPI_GetError() after a
   transaction burst; SPI_ClearError() before retrying. */

typedef enum {
    SPI_OK          = 0,
    SPI_ERR_TIMEOUT = 1
} SPI_Status;

/* Enable the peripheral and start the master transfer (H7 CSTART). */
void SPI_MasterStart(SPI_TypeDef* spi);

/* Byte primitives with bounded waits (same signatures as MCU/spi.c). */
void SPI_SendData(SPI_TypeDef* spi, uint8_t data);
uint8_t SPI_ReceiveData(SPI_TypeDef* spi);

/* Full-duplex byte exchange: send + receive. */
uint8_t SPI_TransferData(SPI_TypeDef* spi, uint8_t data);

/* Sticky timeout flag: set by any timed-out primitive, never cleared
   implicitly. */
SPI_Status SPI_GetError(void);
void SPI_ClearError(void);

#endif
