#include "max31856.h"

#define SPI_RXTX(data) SPI_TX(data); SPI_RX;

void MAX31856_write_reg(uint8_t addr, uint8_t value) {
    MAX31856_CS_LOW;

    SPI_RXTX(addr | MAX31856_WRITE_ADDR);
    SPI_RXTX(value);

    MAX31856_CS_HIGH;
}

uint8_t MAX31856_read_reg(uint8_t addr) {
    uint8_t value;

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
    for (uint32_t i = 0; i < len; i++) {
        SPI_TX(0);
        buf[i] = SPI_RX;
    }

    MAX31856_CS_HIGH;
}

void MAX31856_init(uint8_t tc_type, uint8_t avg) {
    /* CR0: automatic conversion, open-circuit detection on, 60 Hz rejection */
    MAX31856_write_reg(MAX31856_REG_CR0,
                       MAX31856_CR0_CMODE | MAX31856_CR0_OCFAULT0);

    /* CR1: averaging mode (bits 6:4) and thermocouple type (bits 3:0) */
    MAX31856_write_reg(MAX31856_REG_CR1,
                       (avg & 0x70) | (tc_type & 0x0F));

    /* MASK: unmask all faults (0 = fault asserts on the FAULT pin) */
    MAX31856_write_reg(MAX31856_REG_MASK, 0x00);
}

void MAX31856_oneshot(void) {
    uint8_t cr0 = MAX31856_read_reg(MAX31856_REG_CR0);

    cr0 &= (uint8_t)~MAX31856_CR0_CMODE;  // leave automatic mode
    cr0 |= MAX31856_CR0_1SHOT;            // trigger a single conversion

    MAX31856_write_reg(MAX31856_REG_CR0, cr0);
}

uint8_t MAX31856_read_fault(void) {
    return MAX31856_read_reg(MAX31856_REG_SR);
}

float MAX31856_read_temp(void) {
    uint8_t buf[3];
    uint32_t value;
    int32_t raw;

    MAX31856_read_buf(MAX31856_REG_LTCBH, buf, 3);

    value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    value >>= 5;  // 19-bit linearized temperature, drop the 5 unused LSBs

    raw = (int32_t)value;
    if ((value & 0x40000U) != 0U) {
        raw -= 0x80000;  // sign-extend the 19-bit two's complement value
    }

    return (float)raw * 0.0078125f;  // 2^-7 degC per LSB
}

float MAX31856_read_cj_temp(void) {
    uint8_t buf[2];
    uint16_t value;
    int32_t raw;

    MAX31856_read_buf(MAX31856_REG_CJTH, buf, 2);

    value = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    value >>= 2;  // 14-bit cold-junction temperature, drop the 2 unused LSBs

    raw = (int32_t)value;
    if ((value & 0x2000U) != 0U) {
        raw -= 0x4000;  // sign-extend the 14-bit two's complement value
    }

    return (float)raw * 0.015625f;  // 2^-6 degC per LSB
}
