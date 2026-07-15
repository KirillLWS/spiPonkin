/**
 * @file    spi.h
 * @brief   Blocking SPI master transport on LL with bounded waits.
 * @details Symmetric to the i2c module. Drop-in replacement for the project
 *          MCU/spi.c primitives: SPI_SendData / SPI_ReceiveData keep the same
 *          signatures, so libraries built on the constants-macro layer
 *          (max31856, eeprom_mc25lc128) work with it unchanged.
 *
 *          The peripheral itself is configured elsewhere (mode, baud rate,
 *          soft NSS). On STM32H7 call SPI_MasterStart() once after
 *          LL_SPI_Init: the H7 master does not clock anything until CSTART
 *          is set.
 *
 *          A stuck flag latches a sticky error instead of hanging the
 *          firmware: reads then return 0, so write-readback checks
 *          (e.g. MAX31856_probe) fail loudly instead of looping forever.
 *          Poll SPI_GetError() after a transaction burst; SPI_ClearError()
 *          before retrying.
 *
 *          Not reentrant: when several RTOS tasks share one bus, guard each
 *          call with a mutex on the application side.
 */

#ifndef SPI_H
#define SPI_H

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "stdint.h"

#include "stm32h7xx_ll_spi.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/* SPI_TIMEOUT (spin bound) is defined in spi.c and can be overridden there. */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/**
 * @brief Transport status codes.
 */
typedef enum {
    SPI_OK          = 0,  /**< No timeout has occurred since the last clear   */
    SPI_ERR_TIMEOUT = 1   /**< A flag wait timed out (bus or peripheral dead) */
} SPI_Status;

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* The sticky error flag is private to spi.c; access it via SPI_GetError(). */

/*==============================================================================
 *                           FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief   Enable the peripheral and start the master transfer.
 * @details On STM32H7 the master generates no clocks until CSTART is set;
 *          call once after LL_SPI_Init / LL_SPI_Enable.
 * @param[in,out] spi SPI peripheral instance (SPI1..SPIx).
 * @return  None.
 */
void SPI_MasterStart(SPI_TypeDef* spi);

/**
 * @brief   Transmit one byte with a bounded wait for TX space.
 * @details On timeout the byte is dropped and the sticky error is latched.
 * @param[in,out] spi  SPI peripheral instance.
 * @param[in]     data Byte to transmit.
 * @return  None.
 */
void SPI_SendData(SPI_TypeDef* spi, uint8_t data);

/**
 * @brief   Receive one byte with a bounded wait for RX data.
 * @details On timeout returns 0 and latches the sticky error.
 * @param[in,out] spi SPI peripheral instance.
 * @return  Received byte, or 0 on timeout.
 */
uint8_t SPI_ReceiveData(SPI_TypeDef* spi);

/**
 * @brief   Full-duplex byte exchange (send + receive).
 * @param[in,out] spi  SPI peripheral instance.
 * @param[in]     data Byte to transmit.
 * @return  Received byte, or 0 on timeout.
 */
uint8_t SPI_TransferData(SPI_TypeDef* spi, uint8_t data);

/**
 * @brief   Read the sticky timeout flag.
 * @details Set by any timed-out primitive, never cleared implicitly.
 * @return  SPI_OK or SPI_ERR_TIMEOUT.
 */
SPI_Status SPI_GetError(void);

/**
 * @brief   Clear the sticky timeout flag before retrying.
 * @return  None.
 */
void SPI_ClearError(void);

#endif /* SPI_H */
