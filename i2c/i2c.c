#include "i2c.h"

#include "gpio.h"

/* Bounded spin count so a stuck bus cannot hang the firmware forever.
   Iteration-based, not time-based: revisit when the core clock changes. */
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT  100000U
#endif

/* Half-period spin for the bus-recovery bit-bang clock (~10-100 kHz). */
#ifndef I2C_RECOVER_DELAY
#define I2C_RECOVER_DELAY  2000U
#endif

static I2C_Status i2c_wait_txis(I2C_TypeDef* i2c) {
    I2C_Status st = I2C_ERR_TIMEOUT;
    uint32_t t = I2C_TIMEOUT;

    while (t != 0U) {
        if (LL_I2C_IsActiveFlag_NACK(i2c) != 0U) {
            LL_I2C_ClearFlag_NACK(i2c);
            st = I2C_ERR_NACK;
            t = 0U;
        } else if (LL_I2C_IsActiveFlag_TXIS(i2c) != 0U) {
            st = I2C_OK;
            t = 0U;
        } else {
            t--;
        }
    }

    return st;
}

static I2C_Status i2c_wait_rxne(I2C_TypeDef* i2c) {
    I2C_Status st = I2C_ERR_TIMEOUT;
    uint32_t t = I2C_TIMEOUT;

    while (t != 0U) {
        if (LL_I2C_IsActiveFlag_NACK(i2c) != 0U) {
            LL_I2C_ClearFlag_NACK(i2c);
            st = I2C_ERR_NACK;
            t = 0U;
        } else if (LL_I2C_IsActiveFlag_RXNE(i2c) != 0U) {
            st = I2C_OK;
            t = 0U;
        } else {
            t--;
        }
    }

    return st;
}

static I2C_Status i2c_wait_flag(I2C_TypeDef* i2c, uint32_t is_tc) {
    I2C_Status st = I2C_ERR_TIMEOUT;
    uint32_t t = I2C_TIMEOUT;
    uint32_t done;

    while (t != 0U) {
        if (is_tc != 0U) {
            done = LL_I2C_IsActiveFlag_TC(i2c);
        } else {
            done = LL_I2C_IsActiveFlag_STOP(i2c);
        }

        if (LL_I2C_IsActiveFlag_NACK(i2c) != 0U) {
            LL_I2C_ClearFlag_NACK(i2c);
            st = I2C_ERR_NACK;
            t = 0U;
        } else if (done != 0U) {
            st = I2C_OK;
            t = 0U;
        } else {
            t--;
        }
    }

    return st;
}

I2C_Status I2C_Write(I2C_TypeDef* i2c, uint8_t addr, const uint8_t* data, uint32_t len) {
    I2C_Status st = I2C_OK;
    uint32_t i;

    LL_I2C_HandleTransfer(i2c, (uint32_t)addr << 1, LL_I2C_ADDRSLAVE_7BIT, len,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATION_START_WRITE);

    for (i = 0U; (i < len) && (st == I2C_OK); i++) {
        st = i2c_wait_txis(i2c);
        if (st == I2C_OK) {
            LL_I2C_TransmitData8(i2c, data[i]);
        }
    }

    if (st == I2C_OK) {
        st = i2c_wait_flag(i2c, 0U);
    }
    LL_I2C_ClearFlag_STOP(i2c);

    return st;
}

I2C_Status I2C_Read(I2C_TypeDef* i2c, uint8_t addr, uint8_t* data, uint32_t len) {
    I2C_Status st = I2C_OK;
    uint32_t i;

    LL_I2C_HandleTransfer(i2c, (uint32_t)addr << 1, LL_I2C_ADDRSLAVE_7BIT, len,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATION_START_READ);

    for (i = 0U; (i < len) && (st == I2C_OK); i++) {
        st = i2c_wait_rxne(i2c);
        if (st == I2C_OK) {
            data[i] = LL_I2C_ReceiveData8(i2c);
        }
    }

    if (st == I2C_OK) {
        st = i2c_wait_flag(i2c, 0U);
    }
    LL_I2C_ClearFlag_STOP(i2c);

    return st;
}

I2C_Status I2C_WriteRead(I2C_TypeDef* i2c, uint8_t addr,
                         const uint8_t* wdata, uint32_t wlen,
                         uint8_t* rdata, uint32_t rlen) {
    I2C_Status st = I2C_OK;
    uint32_t i;

    /* Phase 1: write with SOFTEND so no STOP is generated, then repeated start. */
    LL_I2C_HandleTransfer(i2c, (uint32_t)addr << 1, LL_I2C_ADDRSLAVE_7BIT, wlen,
                          LL_I2C_MODE_SOFTEND, LL_I2C_GENERATION_START_WRITE);

    for (i = 0U; (i < wlen) && (st == I2C_OK); i++) {
        st = i2c_wait_txis(i2c);
        if (st == I2C_OK) {
            LL_I2C_TransmitData8(i2c, wdata[i]);
        }
    }

    if (st == I2C_OK) {
        st = i2c_wait_flag(i2c, 1U);
    }

    /* Phase 2: repeated-start read with AUTOEND. */
    if (st == I2C_OK) {
        LL_I2C_HandleTransfer(i2c, (uint32_t)addr << 1, LL_I2C_ADDRSLAVE_7BIT, rlen,
                              LL_I2C_MODE_AUTOEND, LL_I2C_GENERATION_START_READ);

        for (i = 0U; (i < rlen) && (st == I2C_OK); i++) {
            st = i2c_wait_rxne(i2c);
            if (st == I2C_OK) {
                rdata[i] = LL_I2C_ReceiveData8(i2c);
            }
        }

        if (st == I2C_OK) {
            st = i2c_wait_flag(i2c, 0U);
        }
        LL_I2C_ClearFlag_STOP(i2c);
    }

    return st;
}

static void i2c_recover_delay(void) {
    volatile uint32_t d;

    for (d = 0U; d < I2C_RECOVER_DELAY; d++) {
    }
}

I2C_Status I2C_BusRecover(GPIO_TypeDef* scl_port, uint32_t scl_pin,
                          GPIO_TypeDef* sda_port, uint32_t sda_pin) {
    I2C_Status st = I2C_ERR_TIMEOUT;
    uint32_t i;

    /* Take both lines as open-drain GPIO, released (high). */
    GPIO_Config(scl_port, scl_pin, LL_GPIO_MODE_OUTPUT, LL_GPIO_OUTPUT_OPENDRAIN,
                LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0);
    GPIO_Config(sda_port, sda_pin, LL_GPIO_MODE_OUTPUT, LL_GPIO_OUTPUT_OPENDRAIN,
                LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0);
    LL_GPIO_SetOutputPin(scl_port, scl_pin);
    LL_GPIO_SetOutputPin(sda_port, sda_pin);
    i2c_recover_delay();

    /* Clock SCL until the stuck slave finishes its byte and releases SDA
       (9 pulses cover a full byte plus the ACK bit). */
    for (i = 0U; (i < 9U) && (LL_GPIO_IsInputPinSet(sda_port, sda_pin) == 0U); i++) {
        LL_GPIO_ResetOutputPin(scl_port, scl_pin);
        i2c_recover_delay();
        LL_GPIO_SetOutputPin(scl_port, scl_pin);
        i2c_recover_delay();
    }

    if (LL_GPIO_IsInputPinSet(sda_port, sda_pin) != 0U) {
        /* STOP condition: SDA low -> high while SCL stays high. */
        LL_GPIO_ResetOutputPin(sda_port, sda_pin);
        i2c_recover_delay();
        LL_GPIO_SetOutputPin(sda_port, sda_pin);
        i2c_recover_delay();
        st = I2C_OK;
    }

    return st;
}
