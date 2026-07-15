/**
 * @file    max31856.c
 * @brief   MAX31856 thermocouple-to-digital converter driver implementation.
 * @details See max31856.h for the public API description.
 */

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "max31856.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/** Full-duplex byte exchange: transmit and discard the received byte. */
#define SPI_RXTX(data) do { SPI_TX(data); (void)SPI_RX; } while (0)

/** Scratch value written to MASK during the probe readback test. */
#define MAX31856_PROBE_PATTERN  0x15U

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/* No private data types. */

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* No file-scope variables: the driver keeps no state. */

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

/**
 * @brief   Clamp a signed value into a range.
 * @param[in] value Input value.
 * @param[in] low   Lower bound.
 * @param[in] high  Upper bound.
 * @return  value limited to [low, high].
 */
static int32_t clamp_i32(int32_t value, int32_t low, int32_t high) {
    int32_t result = value;

    if (value < low) {
        result = low;
    } else if (value > high) {
        result = high;
    } else {
        /* in range */
    }

    return result;
}

void MAX31856_write_reg(uint8_t addr, uint8_t value) {
    MAX31856_CS_LOW;

    SPI_RXTX(addr | MAX31856_WRITE_ADDR);
    SPI_RXTX(value);

    MAX31856_CS_HIGH;
}

uint8_t MAX31856_read_reg(uint8_t addr) {
    uint8_t value = 0U;

    MAX31856_CS_LOW;

    SPI_RXTX(addr & MAX31856_ADDR_MASK);
    SPI_TX(0);
    value = SPI_RX;

    MAX31856_CS_HIGH;

    return value;
}

void MAX31856_read_buf(uint8_t addr, uint8_t* buf, uint32_t len) {
    MAX31856_CS_LOW;

    SPI_RXTX(addr & MAX31856_ADDR_MASK);
    for (uint32_t i = 0U; i < len; i++) {
        SPI_TX(0);
        buf[i] = SPI_RX;
    }

    MAX31856_CS_HIGH;
}

MAX31856_Status MAX31856_probe(void) {
    MAX31856_Status st = MAX31856_ERR;
    uint8_t saved = MAX31856_read_reg(MAX31856_REG_MASK);

    MAX31856_write_reg(MAX31856_REG_MASK, MAX31856_PROBE_PATTERN);
    if (MAX31856_read_reg(MAX31856_REG_MASK) == MAX31856_PROBE_PATTERN) {
        st = MAX31856_OK;
    }
    MAX31856_write_reg(MAX31856_REG_MASK, saved);

    return st;
}

void MAX31856_init(uint8_t tc_type, uint8_t avg) {
    uint8_t cr1 = 0U;

    /* CR0: automatic conversion, open-circuit detection on, 60 Hz rejection */
    MAX31856_write_reg(MAX31856_REG_CR0,
                       (uint8_t)(MAX31856_CR0_CMODE | MAX31856_CR0_OCFAULT0));

    /* CR1: averaging mode (bits 6:4) and thermocouple type (bits 3:0) */
    cr1 = (uint8_t)((avg & MAX31856_CR1_AVG_MASK) |
                    (tc_type & MAX31856_CR1_TYPE_MASK));
    MAX31856_write_reg(MAX31856_REG_CR1, cr1);

    /* MASK: unmask all faults (0 = fault asserts on the FAULT pin) */
    MAX31856_write_reg(MAX31856_REG_MASK, 0x00U);
}

void MAX31856_set_filter(uint8_t use_50hz) {
    uint8_t cr0 = MAX31856_read_reg(MAX31856_REG_CR0);
    uint8_t cmode = (uint8_t)(cr0 & MAX31856_CR0_CMODE);

    /* The datasheet requires conversions to be stopped while the 50/60 Hz
       filter is changed, so automatic mode is suspended and restored. */
    cr0 &= (uint8_t)(~MAX31856_CR0_CMODE);
    if (use_50hz != 0U) {
        cr0 |= MAX31856_CR0_50HZ;
    } else {
        cr0 &= (uint8_t)(~MAX31856_CR0_50HZ);
    }
    MAX31856_write_reg(MAX31856_REG_CR0, cr0);

    if (cmode != 0U) {
        MAX31856_write_reg(MAX31856_REG_CR0, (uint8_t)(cr0 | MAX31856_CR0_CMODE));
    }
}

void MAX31856_set_tc_limits(float low_c, float high_c) {
    int32_t high = clamp_i32((int32_t)(high_c / MAX31856_TCTH_LSB), -32768, 32767);
    int32_t low = clamp_i32((int32_t)(low_c / MAX31856_TCTH_LSB), -32768, 32767);
    uint32_t uhigh = (uint32_t)high & 0xFFFFU;
    uint32_t ulow = (uint32_t)low & 0xFFFFU;

    MAX31856_write_reg(MAX31856_REG_LTHFTH, (uint8_t)(uhigh >> 8));
    MAX31856_write_reg(MAX31856_REG_LTHFTL, (uint8_t)(uhigh & 0xFFU));
    MAX31856_write_reg(MAX31856_REG_LTLFTH, (uint8_t)(ulow >> 8));
    MAX31856_write_reg(MAX31856_REG_LTLFTL, (uint8_t)(ulow & 0xFFU));
}

void MAX31856_set_cj_limits(int8_t low_c, int8_t high_c) {
    MAX31856_write_reg(MAX31856_REG_CJHF, (uint8_t)high_c);
    MAX31856_write_reg(MAX31856_REG_CJLF, (uint8_t)low_c);
}

void MAX31856_set_cj_offset(float offset_c) {
    int32_t code = clamp_i32((int32_t)(offset_c / MAX31856_CJTO_LSB), -128, 127);

    MAX31856_write_reg(MAX31856_REG_CJTO, (uint8_t)((uint32_t)code & 0xFFU));
}

void MAX31856_oneshot(void) {
    uint8_t cr0 = MAX31856_read_reg(MAX31856_REG_CR0);

    cr0 &= (uint8_t)(~MAX31856_CR0_CMODE);  /* leave automatic mode */
    cr0 |= MAX31856_CR0_1SHOT;              /* trigger a single conversion */

    MAX31856_write_reg(MAX31856_REG_CR0, cr0);
}

uint8_t MAX31856_conversion_done(void) {
    uint8_t cr0 = MAX31856_read_reg(MAX31856_REG_CR0);

    /* 1SHOT self-clears when the triggered conversion completes. */
    return ((cr0 & MAX31856_CR0_1SHOT) == 0U) ? 1U : 0U;
}

void MAX31856_clear_fault(void) {
    uint8_t cr0 = MAX31856_read_reg(MAX31856_REG_CR0);

    cr0 |= MAX31856_CR0_FAULTCLR;  /* self-clearing fault status reset */

    MAX31856_write_reg(MAX31856_REG_CR0, cr0);
}

uint8_t MAX31856_read_fault(void) {
    return MAX31856_read_reg(MAX31856_REG_SR);
}

int32_t MAX31856_read_temp_raw(void) {
    uint8_t buf[3] = {0U, 0U, 0U};
    uint32_t value = 0U;
    int32_t raw = 0;

    MAX31856_read_buf(MAX31856_REG_LTCBH, buf, 3U);

    value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    value >>= 5;  /* 19-bit linearized temperature, drop the 5 unused LSBs */

    raw = (int32_t)value;
    if ((value & 0x40000U) != 0U) {
        raw -= 0x80000;  /* sign-extend the 19-bit two's complement value */
    }

    return raw;
}

int32_t MAX31856_read_cj_temp_raw(void) {
    uint8_t buf[2] = {0U, 0U};
    uint16_t value = 0U;
    int32_t raw = 0;

    MAX31856_read_buf(MAX31856_REG_CJTH, buf, 2U);

    value = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    value >>= 2;  /* 14-bit cold-junction temperature, drop the 2 unused LSBs */

    raw = (int32_t)value;
    if ((value & 0x2000U) != 0U) {
        raw -= 0x4000;  /* sign-extend the 14-bit two's complement value */
    }

    return raw;
}

float MAX31856_read_temp(void) {
    return (float)MAX31856_read_temp_raw() * MAX31856_TC_LSB;
}

float MAX31856_read_cj_temp(void) {
    return (float)MAX31856_read_cj_temp_raw() * MAX31856_CJ_LSB;
}
