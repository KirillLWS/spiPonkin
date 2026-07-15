/**
 * @file    aht20_bmp280_check.c
 * @brief   Board bring-up check of the AHT20 + BMP280 combo module.
 * @details Reference sequence for NUCLEO-H755ZI-Q / STM32H743: I2C1 on the
 *          Arduino header pins D15 = PB8 (SCL) and D14 = PB9 (SDA), AF4.
 *          Wiring of the combo module: VIN->3V3, GND->GND, SCL->D15,
 *          SDA->D14 (the module already carries pull-up resistors).
 *
 *          The file is a template: clock init (Clock_Init) comes from the
 *          main project, results land in live-watch variables. Not built as
 *          part of the libraries.
 */

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"

#include "gpio.h"
#include "i2c.h"
#include "aht20_bmp280.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/** I2C1 TIMINGR for 100 kHz standard mode at a 100 MHz kernel clock
 *  (APB1 = 100 MHz with the project 400 MHz clock tree). PRESC = 9 divides
 *  the kernel clock to 10 MHz; SCLL/SCLH stretch the low/high phases to
 *  ~5/4 us. For a different kernel clock recalculate (CubeMX I2C timing). */
#define I2C1_TIMINGR_100KHZ  0x90422731U

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/* No private data types. */

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/** Live-watch results: temperature from the AHT20, degC. */
static volatile float aht_temperature = 0.0f;
/** Live-watch results: relative humidity from the AHT20, %. */
static volatile float aht_humidity = 0.0f;
/** Live-watch results: temperature from the BMP280, degC. */
static volatile float bmp_temperature = 0.0f;
/** Live-watch results: pressure from the BMP280, Pa. */
static volatile float bmp_pressure = 0.0f;
/** Live-watch status of the last AHT20 transaction. */
static volatile I2C_Status aht_status = I2C_OK;
/** Live-watch status of the last BMP280 transaction. */
static volatile I2C_Status bmp_status = I2C_OK;

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

/**
 * @brief   Configure I2C1 pins and peripheral for the combo module.
 * @details PB8/PB9 -> AF4 open-drain with pull-ups, I2C1 kernel clock from
 *          PCLK1, 100 kHz timing, then enable.
 * @return  None.
 */
static void I2C1_Config(void) {
    /* SCL = PB8 (D15), SDA = PB9 (D14): alternate function 4, open drain */
    GPIO_Config(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
                LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_OPENDRAIN,
                LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_VERY_HIGH, LL_GPIO_AF_4);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    LL_I2C_Disable(I2C1);
    LL_I2C_SetTiming(I2C1, I2C1_TIMINGR_100KHZ);
    LL_I2C_Enable(I2C1);
}

/**
 * @brief   Bring-up entry: init both chips and poll them in a loop.
 * @details Call after Clock_Init() from the main project. Watch the
 *          aht_* / bmp_* variables in the debugger live watch.
 * @return  Never returns.
 */
void AHT20_BMP280_Check(void) {
    uint8_t busy = 0U;

    I2C1_Config();

    /* Optional: unstick the bus after an unclean reset, then re-init pins. */
    if (I2C_BusRecover(GPIOB, LL_GPIO_PIN_8, GPIOB, LL_GPIO_PIN_9) == I2C_OK) {
        I2C1_Config();
    }

    aht_status = AHT20_init();    /* expects I2C_OK; NACK = wiring/address    */
    bmp_status = BMP280_init();   /* expects I2C_OK; NACK = wiring or id != 0x58 */

    while (1) {
        /* --- AHT20, non-blocking pattern --- */
        aht_status = AHT20_trigger();
        if (aht_status == I2C_OK) {
            busy = 1U;
            while (busy == 1U) {
                LL_mDelay(10U);
                aht_status = AHT20_busy(&busy);
                if (aht_status != I2C_OK) {
                    busy = 0U;  /* bus error: leave the wait loop */
                }
            }
            float t = 0.0f;
            float h = 0.0f;
            aht_status = AHT20_fetch(&t, &h);
            if (aht_status == I2C_OK) {
                aht_temperature = t;
                aht_humidity = h;
            }
        }

        /* --- BMP280: normal mode converts on its own, just read --- */
        float bt = 0.0f;
        float bp = 0.0f;
        bmp_status = BMP280_read(&bt, &bp);
        if (bmp_status == I2C_OK) {
            bmp_temperature = bt;
            bmp_pressure = bp;
        }

        LL_mDelay(500U);
    }
}
