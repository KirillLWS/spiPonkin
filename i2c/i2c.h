#ifndef I2C_H
#define I2C_H

#include "stdint.h"

#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_i2c.h"

/* Blocking I2C master transport on LL, analogous to the project SPI layer.
   The peripheral (I2Cx) must already be configured and enabled elsewhere
   (clock source, TIMINGR, pins); these helpers only drive transactions.
   Addresses are 7-bit (unshifted).

   Not reentrant: when several RTOS tasks share one bus, guard each call
   with a mutex on the application side. */
typedef enum {
    I2C_OK          = 0,
    I2C_ERR_TIMEOUT = 1,  // bus timeout, or device measurement not ready yet
    I2C_ERR_NACK    = 2,  // device did not acknowledge (missing/busy)
    I2C_ERR_DATA    = 3   // frame received but content invalid (CRC, skipped)
} I2C_Status;

/* Write len bytes to the device. */
I2C_Status I2C_Write(I2C_TypeDef* i2c, uint8_t addr, const uint8_t* data, uint32_t len);

/* Read len bytes from the device. */
I2C_Status I2C_Read(I2C_TypeDef* i2c, uint8_t addr, uint8_t* data, uint32_t len);

/* Write wlen bytes, then repeated-start read rlen bytes (register read). */
I2C_Status I2C_WriteRead(I2C_TypeDef* i2c, uint8_t addr,
                         const uint8_t* wdata, uint32_t wlen,
                         uint8_t* rdata, uint32_t rlen);

/* Unstick a bus where a slave holds SDA low (e.g. after an MCU reset in the
   middle of a read): drives SCL as a GPIO for up to 9 clock pulses until the
   slave releases SDA, then generates a STOP condition. The pins are left in
   open-drain GPIO mode - reconfigure them back to their I2C alternate
   function (GPIO_ConfigAlternate + open-drain) and re-enable the peripheral
   afterwards. Returns I2C_OK when SDA is released. */
I2C_Status I2C_BusRecover(GPIO_TypeDef* scl_port, uint32_t scl_pin,
                          GPIO_TypeDef* sda_port, uint32_t sda_pin);

#endif
