/**
 * @file    aht20_bmp280.h
 * @brief   Driver for the combined AHT20 + BMP280 sensor module on I2C.
 * @details AHT20 measures temperature and relative humidity, BMP280 measures
 *          temperature and pressure; both chips share one I2C bus with
 *          different addresses (see aht20_bmp280_constants.h).
 *
 *          Two usage styles:
 *          1. Non-blocking (interrupt / state machine, no delays inside):
 *             AHT20_trigger() -> poll AHT20_busy() -> AHT20_fetch().
 *             BMP280_read() has no waits at all (normal mode samples
 *             continuously).
 *          2. Blocking (e.g. a FreeRTOS task): AHT20_read() triggers, waits
 *             via AHT_BMP_DELAY_MS (remap it to vTaskDelay under an RTOS)
 *             and fetches.
 *
 *          The I2C transport is not reentrant: guard bus access with a mutex
 *          when several tasks share it.
 */

#ifndef AHT20_BMP280_H
#define AHT20_BMP280_H

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "stdint.h"

#include "i2c.h"
#include "aht20_bmp280_constants.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/* --- AHT20 commands and status bits --- */
#define AHT20_CMD_STATUS    0x71U  /**< Read the status byte                  */
#define AHT20_CMD_INIT      0xBEU  /**< Initialize / calibrate                */
#define AHT20_CMD_MEASURE   0xACU  /**< Trigger a measurement                 */
#define AHT20_CMD_RESET     0xBAU  /**< Soft reset                            */
#define AHT20_STATUS_BUSY   0x80U  /**< Status bit: measurement in progress   */
#define AHT20_STATUS_CAL    0x08U  /**< Status bit: sensor calibrated         */

/* --- BMP280 registers and magic values --- */
#define BMP280_REG_ID       0xD0U  /**< Chip id register                      */
#define BMP280_REG_RESET    0xE0U  /**< Soft-reset register                   */
#define BMP280_REG_CALIB    0x88U  /**< Calibration data start (24 bytes)     */
#define BMP280_REG_CTRL     0xF4U  /**< ctrl_meas register                    */
#define BMP280_REG_CONFIG   0xF5U  /**< config register                       */
#define BMP280_REG_DATA     0xF7U  /**< Burst data start (press + temp, 6 bytes) */
#define BMP280_CHIP_ID      0x58U  /**< Expected chip id value                */
#define BMP280_RESET_CMD    0xB6U  /**< Soft-reset magic value                */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/* Status codes are shared with the transport: see I2C_Status in i2c.h.
   I2C_ERR_TIMEOUT from AHT20_fetch means "conversion still running",
   I2C_ERR_DATA means a CRC mismatch (AHT20) or a skipped measurement
   (BMP280), I2C_ERR_NACK means the sensor does not answer on the bus. */

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* BMP280 calibration data is stored privately in aht20_bmp280.c. */

/*==============================================================================
 *                           FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief   Initialize the AHT20.
 * @details Waits the 40 ms power-up time (blocking, AHT_BMP_DELAY_MS), reads
 *          the status byte and sends the calibration command if the sensor
 *          reports itself uncalibrated.
 * @return  I2C_OK, or a transport error code.
 */
I2C_Status AHT20_init(void);

/**
 * @brief   Soft-reset the AHT20 without power cycling.
 * @details Sends command 0xBA and waits the 20 ms restart time (blocking).
 *          Recovers a hung sensor; call AHT20_init() again afterwards.
 * @return  I2C_OK, or a transport error code.
 */
I2C_Status AHT20_soft_reset(void);

/**
 * @brief   Start an AHT20 measurement (non-blocking).
 * @details The conversion takes ~80 ms inside the chip; poll AHT20_busy()
 *          or call AHT20_fetch() later.
 * @return  I2C_OK, or a transport error code.
 */
I2C_Status AHT20_trigger(void);

/**
 * @brief   Poll whether the AHT20 is still converting (non-blocking).
 * @param[out] busy Set to 1 while a conversion is running, 0 when done.
 *                  May be NULL (then only the bus status is returned).
 * @return  I2C_OK, or a transport error code.
 */
I2C_Status AHT20_busy(uint8_t* busy);

/**
 * @brief   Read out a finished AHT20 measurement (non-blocking).
 * @details Reads the 7-byte frame, verifies the busy bit and the CRC-8
 *          (polynomial 0x31, init 0xFF) and converts the raw 20-bit values.
 * @param[out] temperature Temperature, degC. May be NULL.
 * @param[out] humidity    Relative humidity, %. May be NULL.
 * @return  I2C_OK on success; I2C_ERR_TIMEOUT while the conversion is still
 *          running; I2C_ERR_DATA on a CRC mismatch; transport errors as is.
 */
I2C_Status AHT20_fetch(float* temperature, float* humidity);

/**
 * @brief   Blocking AHT20 measurement: trigger + wait + fetch.
 * @details Waits via AHT_BMP_DELAY_MS (80 ms plus up to three 10 ms
 *          retries). Intended for RTOS tasks; remap the delay macro to
 *          vTaskDelay so the task yields instead of spinning.
 * @param[out] temperature Temperature, degC. May be NULL.
 * @param[out] humidity    Relative humidity, %. May be NULL.
 * @return  I2C_OK on success, otherwise the last error code.
 */
I2C_Status AHT20_read(float* temperature, float* humidity);

/**
 * @brief   Initialize the BMP280.
 * @details Verifies the chip id (0x58), issues a soft reset, reads the 24
 *          calibration bytes and starts continuous sampling (normal mode,
 *          osrs_t x2, osrs_p x16, IIR filter x16). Blocking for the 5 ms
 *          reset time.
 * @return  I2C_OK; I2C_ERR_NACK when the id does not match; transport
 *          errors as is.
 */
I2C_Status BMP280_init(void);

/**
 * @brief   Read compensated BMP280 temperature and pressure (non-blocking).
 * @details Reads the 6-byte burst and applies the datasheet integer
 *          compensation (temperature first: it produces t_fine used by the
 *          pressure formula). No waits inside.
 * @param[out] temperature Temperature, degC. May be NULL.
 * @param[out] pressure    Pressure, Pa. May be NULL.
 * @return  I2C_OK on success; I2C_ERR_DATA when the chip reports a skipped
 *          measurement (raw code 0x80000); transport errors as is.
 */
I2C_Status BMP280_read(float* temperature, float* pressure);

#endif /* AHT20_BMP280_H */
