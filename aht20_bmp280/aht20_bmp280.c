/**
 * @file    aht20_bmp280.c
 * @brief   AHT20 + BMP280 combined sensor driver implementation.
 * @details See aht20_bmp280.h for the public API description.
 */

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "aht20_bmp280.h"

#include "stddef.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

#define AHT20_RAW_FULL_SCALE  1048576.0f  /**< 2^20, full scale of the raw values */
#define BMP280_ADC_SKIPPED    0x80000     /**< Raw code of a skipped measurement  */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/**
 * @brief BMP280 factory calibration coefficients and shared state.
 */
typedef struct {
    uint16_t t1;      /**< dig_T1 temperature coefficient                     */
    int16_t  t2;      /**< dig_T2 temperature coefficient                     */
    int16_t  t3;      /**< dig_T3 temperature coefficient                     */
    uint16_t p1;      /**< dig_P1 pressure coefficient                        */
    int16_t  p2;      /**< dig_P2 pressure coefficient                        */
    int16_t  p3;      /**< dig_P3 pressure coefficient                        */
    int16_t  p4;      /**< dig_P4 pressure coefficient                        */
    int16_t  p5;      /**< dig_P5 pressure coefficient                        */
    int16_t  p6;      /**< dig_P6 pressure coefficient                        */
    int16_t  p7;      /**< dig_P7 pressure coefficient                        */
    int16_t  p8;      /**< dig_P8 pressure coefficient                        */
    int16_t  p9;      /**< dig_P9 pressure coefficient                        */
    int32_t  t_fine;  /**< Fine temperature, produced by the temperature
                           compensation and consumed by the pressure one      */
} bmp280_calib_t;

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/** BMP280 calibration data, filled by BMP280_init(). */
static bmp280_calib_t bmp = {0U, 0, 0, 0U, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

/**
 * @brief   Assemble a little-endian uint16 from two bytes.
 * @param[in] p Pointer to the low byte.
 * @return  Unsigned 16-bit value.
 */
static uint16_t u16_le(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief   Assemble a little-endian int16 from two bytes.
 * @param[in] p Pointer to the low byte.
 * @return  Signed 16-bit value.
 */
static int16_t s16_le(const uint8_t* p) {
    return (int16_t)u16_le(p);
}

/**
 * @brief   CRC-8 of an AHT20 frame (polynomial 0x31, init 0xFF).
 * @param[in] data Frame bytes.
 * @param[in] len  Number of bytes covered.
 * @return  CRC value.
 */
static uint8_t aht20_crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0xFFU;
    uint32_t i = 0U;
    uint8_t bit = 0U;

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x31U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

/**
 * @brief   Read consecutive BMP280 registers.
 * @param[in]  reg Start register address.
 * @param[out] buf Destination buffer, at least len bytes.
 * @param[in]  len Number of registers to read.
 * @return  Transport status.
 */
static I2C_Status bmp280_read_reg(uint8_t reg, uint8_t* buf, uint32_t len) {
    uint8_t r = reg;

    return I2C_WriteRead(AHT_BMP_I2C, BMP280_ADDR, &r, 1U, buf, len);
}

/**
 * @brief   Write one BMP280 register.
 * @param[in] reg   Register address.
 * @param[in] value Byte to write.
 * @return  Transport status.
 */
static I2C_Status bmp280_write_reg(uint8_t reg, uint8_t value) {
    uint8_t cmd[2] = {0U, 0U};

    cmd[0] = reg;
    cmd[1] = value;

    return I2C_Write(AHT_BMP_I2C, BMP280_ADDR, cmd, 2U);
}

/**
 * @brief   Datasheet temperature compensation.
 * @details Integer formula from the BMP280 datasheet; also updates
 *          bmp.t_fine, which the pressure compensation depends on.
 * @param[in] adc_t Raw 20-bit temperature code.
 * @return  Temperature in 0.01 degC units.
 */
static int32_t bmp280_compensate_t(int32_t adc_t) {
    int32_t var1 = 0;
    int32_t var2 = 0;

    var1 = ((((adc_t >> 3) - ((int32_t)bmp.t1 << 1))) * ((int32_t)bmp.t2)) >> 11;
    var2 = (((((adc_t >> 4) - ((int32_t)bmp.t1)) *
              ((adc_t >> 4) - ((int32_t)bmp.t1))) >> 12) * ((int32_t)bmp.t3)) >> 14;
    bmp.t_fine = var1 + var2;

    return (bmp.t_fine * 5 + 128) >> 8;
}

/**
 * @brief   Datasheet pressure compensation.
 * @details 64-bit integer formula from the BMP280 datasheet. Requires
 *          bmp.t_fine from a preceding bmp280_compensate_t() call.
 * @param[in] adc_p Raw 20-bit pressure code.
 * @return  Pressure in Q24.8 fixed point (value / 256 = Pa); 0 when the
 *          formula would divide by zero.
 */
static uint32_t bmp280_compensate_p(int32_t adc_p) {
    int64_t var1 = 0;
    int64_t var2 = 0;
    int64_t p = 0;
    uint32_t result = 0U;

    var1 = ((int64_t)bmp.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp.p6;
    var2 = var2 + ((var1 * (int64_t)bmp.p5) << 17);
    var2 = var2 + (((int64_t)bmp.p4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp.p3) >> 8) +
           ((var1 * (int64_t)bmp.p2) << 12);
    var1 = ((((int64_t)1 << 47) + var1)) * ((int64_t)bmp.p1) >> 33;

    if (var1 != 0) {
        p = 1048576 - adc_p;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)bmp.p9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)bmp.p8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)bmp.p7) << 4);
        result = (uint32_t)p;
    }

    return result;
}

I2C_Status AHT20_init(void) {
    I2C_Status st = I2C_OK;
    uint8_t status = 0U;
    uint8_t cmd[3] = {0U, 0U, 0U};

    AHT_BMP_DELAY_MS(40U);  /* power-up time */

    st = I2C_Read(AHT_BMP_I2C, AHT20_ADDR, &status, 1U);

    if ((st == I2C_OK) && ((status & AHT20_STATUS_CAL) == 0U)) {
        cmd[0] = AHT20_CMD_INIT;
        cmd[1] = 0x08U;
        cmd[2] = 0x00U;
        st = I2C_Write(AHT_BMP_I2C, AHT20_ADDR, cmd, 3U);
        AHT_BMP_DELAY_MS(10U);
    }

    return st;
}

I2C_Status AHT20_soft_reset(void) {
    uint8_t cmd = AHT20_CMD_RESET;
    I2C_Status st = I2C_OK;

    st = I2C_Write(AHT_BMP_I2C, AHT20_ADDR, &cmd, 1U);
    AHT_BMP_DELAY_MS(20U);  /* datasheet: reset completes within 20 ms */

    return st;
}

I2C_Status AHT20_trigger(void) {
    uint8_t cmd[3] = {0U, 0U, 0U};

    cmd[0] = AHT20_CMD_MEASURE;
    cmd[1] = 0x33U;
    cmd[2] = 0x00U;

    return I2C_Write(AHT_BMP_I2C, AHT20_ADDR, cmd, 3U);
}

I2C_Status AHT20_busy(uint8_t* busy) {
    I2C_Status st = I2C_OK;
    uint8_t status = 0U;

    st = I2C_Read(AHT_BMP_I2C, AHT20_ADDR, &status, 1U);

    if ((st == I2C_OK) && (busy != NULL)) {
        *busy = ((status & AHT20_STATUS_BUSY) != 0U) ? 1U : 0U;
    }

    return st;
}

I2C_Status AHT20_fetch(float* temperature, float* humidity) {
    I2C_Status st = I2C_OK;
    uint8_t data[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint32_t raw = 0U;

    st = I2C_Read(AHT_BMP_I2C, AHT20_ADDR, data, 7U);

    if (st == I2C_OK) {
        if ((data[0] & AHT20_STATUS_BUSY) != 0U) {
            st = I2C_ERR_TIMEOUT;  /* conversion still running, retry later */
        } else if (aht20_crc8(data, 6U) != data[6]) {
            st = I2C_ERR_DATA;     /* corrupted frame */
        } else {
            if (humidity != NULL) {
                raw = ((uint32_t)data[1] << 12) |
                      ((uint32_t)data[2] << 4) |
                      ((uint32_t)data[3] >> 4);
                *humidity = ((float)raw * 100.0f) / AHT20_RAW_FULL_SCALE;
            }
            if (temperature != NULL) {
                raw = (((uint32_t)data[3] & 0x0FU) << 16) |
                      ((uint32_t)data[4] << 8) |
                      (uint32_t)data[5];
                *temperature = (((float)raw * 200.0f) / AHT20_RAW_FULL_SCALE) - 50.0f;
            }
        }
    }

    return st;
}

I2C_Status AHT20_read(float* temperature, float* humidity) {
    I2C_Status st = I2C_OK;
    uint32_t retry = 0U;

    st = AHT20_trigger();

    if (st == I2C_OK) {
        AHT_BMP_DELAY_MS(80U);  /* typical measurement time */
        st = AHT20_fetch(temperature, humidity);

        /* If the chip is still converting, give it a little more time. */
        for (retry = 0U; (retry < 3U) && (st == I2C_ERR_TIMEOUT); retry++) {
            AHT_BMP_DELAY_MS(10U);
            st = AHT20_fetch(temperature, humidity);
        }
    }

    return st;
}

I2C_Status BMP280_init(void) {
    I2C_Status st = I2C_OK;
    uint8_t id = 0U;
    uint8_t calib[24] = {0U};

    st = bmp280_read_reg(BMP280_REG_ID, &id, 1U);

    if ((st == I2C_OK) && (id != BMP280_CHIP_ID)) {
        st = I2C_ERR_NACK;  /* wrong or missing device */
    }

    if (st == I2C_OK) {
        st = bmp280_write_reg(BMP280_REG_RESET, BMP280_RESET_CMD);
        AHT_BMP_DELAY_MS(5U);
    }

    if (st == I2C_OK) {
        st = bmp280_read_reg(BMP280_REG_CALIB, calib, 24U);
    }

    if (st == I2C_OK) {
        bmp.t1 = u16_le(&calib[0]);
        bmp.t2 = s16_le(&calib[2]);
        bmp.t3 = s16_le(&calib[4]);
        bmp.p1 = u16_le(&calib[6]);
        bmp.p2 = s16_le(&calib[8]);
        bmp.p3 = s16_le(&calib[10]);
        bmp.p4 = s16_le(&calib[12]);
        bmp.p5 = s16_le(&calib[14]);
        bmp.p6 = s16_le(&calib[16]);
        bmp.p7 = s16_le(&calib[18]);
        bmp.p8 = s16_le(&calib[20]);
        bmp.p9 = s16_le(&calib[22]);

        /* config: t_standby 0.5 ms, IIR filter x16 */
        st = bmp280_write_reg(BMP280_REG_CONFIG, 0x10U);
    }

    if (st == I2C_OK) {
        /* ctrl_meas: osrs_t x2, osrs_p x16, normal mode */
        st = bmp280_write_reg(BMP280_REG_CTRL, 0x57U);
    }

    return st;
}

I2C_Status BMP280_read(float* temperature, float* pressure) {
    I2C_Status st = I2C_OK;
    uint8_t data[6] = {0U, 0U, 0U, 0U, 0U, 0U};
    int32_t adc_t = 0;
    int32_t adc_p = 0;
    int32_t t = 0;
    uint32_t p = 0U;

    st = bmp280_read_reg(BMP280_REG_DATA, data, 6U);

    if (st == I2C_OK) {
        adc_p = (int32_t)(((uint32_t)data[0] << 12) |
                          ((uint32_t)data[1] << 4) |
                          ((uint32_t)data[2] >> 4));
        adc_t = (int32_t)(((uint32_t)data[3] << 12) |
                          ((uint32_t)data[4] << 4) |
                          ((uint32_t)data[5] >> 4));

        /* 0x80000 marks a skipped measurement (channel off / not started) -
           without this check it compensates into plausible-looking garbage. */
        if ((adc_t == BMP280_ADC_SKIPPED) || (adc_p == BMP280_ADC_SKIPPED)) {
            st = I2C_ERR_DATA;
        }
    }

    if (st == I2C_OK) {
        /* temperature must be compensated first: it sets t_fine for pressure */
        t = bmp280_compensate_t(adc_t);
        p = bmp280_compensate_p(adc_p);

        if (temperature != NULL) {
            *temperature = (float)t / 100.0f;
        }
        if (pressure != NULL) {
            *pressure = (float)p / 256.0f;
        }
    }

    return st;
}
