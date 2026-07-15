#ifndef AHT20_BMP280_H
#define AHT20_BMP280_H

#include "stdint.h"

#include "i2c.h"
#include "aht20_bmp280_constants.h"

/* --- AHT20 (temperature + relative humidity) --- */
#define AHT20_CMD_STATUS    0x71U  // read status byte
#define AHT20_CMD_INIT      0xBEU  // initialize / calibrate
#define AHT20_CMD_MEASURE   0xACU  // trigger measurement
#define AHT20_STATUS_BUSY   0x80U  // status: measurement in progress
#define AHT20_STATUS_CAL    0x08U  // status: sensor calibrated

/* --- BMP280 (temperature + pressure) --- */
#define BMP280_REG_ID       0xD0U  // chip id (0x58)
#define BMP280_REG_RESET    0xE0U  // soft-reset register
#define BMP280_REG_CALIB    0x88U  // calibration data start (24 bytes)
#define BMP280_REG_CTRL     0xF4U  // ctrl_meas
#define BMP280_REG_CONFIG   0xF5U  // config
#define BMP280_REG_DATA     0xF7U  // burst data start (press + temp, 6 bytes)
#define BMP280_CHIP_ID      0x58U  // expected id
#define BMP280_RESET_CMD    0xB6U  // soft-reset magic value

/* Two usage styles:

   1. Non-blocking (interrupt / state-machine, no delays inside):
      AHT20_trigger() -> poll AHT20_busy() -> AHT20_fetch().
      BMP280_read() has no waits at all (normal mode samples continuously).

   2. Blocking (e.g. a FreeRTOS task): AHT20_read() triggers, waits via
      AHT_BMP_DELAY_MS (remap it to vTaskDelay under an RTOS) and fetches.

   The I2C transport is not reentrant: guard bus access with a mutex when
   several tasks share it. */

/* AHT20: power-up, check calibration, calibrate if needed. */
I2C_Status AHT20_init(void);

/* AHT20 non-blocking: start a measurement (takes ~80 ms in the chip). */
I2C_Status AHT20_trigger(void);

/* AHT20 non-blocking: read the status byte; *busy = 1 while converting. */
I2C_Status AHT20_busy(uint8_t* busy);

/* AHT20 non-blocking: read out a finished measurement.
   Returns I2C_ERR_TIMEOUT if the conversion is still running,
   I2C_ERR_DATA on a CRC mismatch. Either pointer may be NULL. */
I2C_Status AHT20_fetch(float* temperature, float* humidity);

/* AHT20 blocking: trigger + wait + fetch (temperature degC, humidity %). */
I2C_Status AHT20_read(float* temperature, float* humidity);

/* BMP280: check id, soft-reset, read calibration, start normal-mode sampling. */
I2C_Status BMP280_init(void);

/* BMP280: read compensated temperature (degC) and pressure (Pa). No waits.
   Returns I2C_ERR_DATA if the chip reports a skipped measurement (0x80000).
   Either pointer may be NULL. */
I2C_Status BMP280_read(float* temperature, float* pressure);

#endif
