#include "i2c.h"

/* Bounded spin count so a stuck bus cannot hang the firmware forever. */
#define I2C_TIMEOUT  100000U

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
