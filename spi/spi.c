#include "spi.h"

/* Bounded spin count so a dead peripheral cannot hang the firmware.
   Iteration-based, not time-based: revisit when the core clock changes. */
#ifndef SPI_TIMEOUT
#define SPI_TIMEOUT  100000U
#endif

static volatile SPI_Status spi_error = SPI_OK;

void SPI_MasterStart(SPI_TypeDef* spi) {
    if (LL_SPI_IsEnabled(spi) == 0U) {
        LL_SPI_Enable(spi);
    }

    /* On H7 the master generates no clocks until CSTART is set. */
    LL_SPI_StartMasterTransfer(spi);
}

void SPI_SendData(SPI_TypeDef* spi, uint8_t data) {
    uint32_t t = SPI_TIMEOUT;

    while ((LL_SPI_IsActiveFlag_TXP(spi) == 0U) && (t != 0U)) {
        t--;
    }

    if (LL_SPI_IsActiveFlag_TXP(spi) != 0U) {
        LL_SPI_TransmitData8(spi, data);
    } else {
        spi_error = SPI_ERR_TIMEOUT;
    }
}

uint8_t SPI_ReceiveData(SPI_TypeDef* spi) {
    uint8_t data = 0U;
    uint32_t t = SPI_TIMEOUT;

    while ((LL_SPI_IsActiveFlag_RXP(spi) == 0U) && (t != 0U)) {
        t--;
    }

    if (LL_SPI_IsActiveFlag_RXP(spi) != 0U) {
        data = LL_SPI_ReceiveData8(spi);
    } else {
        spi_error = SPI_ERR_TIMEOUT;
    }

    return data;
}

uint8_t SPI_TransferData(SPI_TypeDef* spi, uint8_t data) {
    SPI_SendData(spi, data);
    return SPI_ReceiveData(spi);
}

SPI_Status SPI_GetError(void) {
    return spi_error;
}

void SPI_ClearError(void) {
    spi_error = SPI_OK;
}
