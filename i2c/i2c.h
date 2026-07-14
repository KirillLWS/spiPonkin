#ifndef I2C_H
#define I2C_H

#include "stdint.h"

#include "stm32h7xx_ll_i2c.h"

/* Blocking I2C master transport on LL, analogous to the project SPI layer.
   The peripheral (I2Cx) must already be configured and enabled elsewhere
   (clock source, TIMINGR, pins); these helpers only drive transactions.
   Addresses are 7-bit (unshifted). */
typedef enum {
    I2C_OK      = 0,
    I2C_ERR_TIMEOUT = 1,
    I2C_ERR_NACK    = 2
} I2C_Status;

/* Write len bytes to the device. */
I2C_Status I2C_Write(I2C_TypeDef* i2c, uint8_t addr, const uint8_t* data, uint32_t len);

/* Read len bytes from the device. */
I2C_Status I2C_Read(I2C_TypeDef* i2c, uint8_t addr, uint8_t* data, uint32_t len);

/* Write wlen bytes, then repeated-start read rlen bytes (register read). */
I2C_Status I2C_WriteRead(I2C_TypeDef* i2c, uint8_t addr,
                         const uint8_t* wdata, uint32_t wlen,
                         uint8_t* rdata, uint32_t rlen);

#endif
