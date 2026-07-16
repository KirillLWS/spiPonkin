/**
 * @file    main_quick_check.c
 * @brief   Quick bring-up main for the AHT20+BMP280 combo on NUCLEO-H755ZI-Q.
 * @details Drop-in replacement of Src/main.c for the test_CM7 project.
 *          Requires Inc/aht20_bmp280_constants.h (copy of the .h_example
 *          with AHT_BMP_I2C set to I2C4).
 *          Watch aht_t / aht_h / bmp_t / bmp_p and the statuses in Live
 *          Expressions; LD1 (green, PB0) blinks as a heartbeat.
 */

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include <stdint.h>

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "aht20_bmp280.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_utils.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/** I2C4 TIMINGR for 100 kHz at a 100 MHz kernel clock (APB4 of the
 *  400 MHz project clock tree). Recalculate if the tree changes. */
#define I2C_TIMINGR_100KHZ  0x90422731U

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

static volatile float aht_t = 0.0f;            /**< AHT20 temperature, degC   */
static volatile float aht_h = 0.0f;            /**< AHT20 humidity, %         */
static volatile float bmp_t = 0.0f;            /**< BMP280 temperature, degC  */
static volatile float bmp_p = 0.0f;            /**< BMP280 pressure, Pa       */
static volatile I2C_Status aht_st = I2C_OK;    /**< Last AHT20 status         */
static volatile I2C_Status bmp_st = I2C_OK;    /**< Last BMP280 status        */
static volatile uint32_t cycle = 0U;           /**< Loop counter (liveness)   */

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

/**
 * @brief   Configure I2C4 on PF14 (SCL, D69) / PF15 (SDA, D68).
 * @details Zio label I2C_B; the silicon function on PF14/PF15 is I2C4 (AF4).
 * @return  None.
 */
static void I2C4_Setup(void) {
    GPIO_Config(GPIOF, LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
                LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_OPENDRAIN,
                LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_VERY_HIGH, LL_GPIO_AF_4);

    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_I2C4);

    LL_I2C_Disable(I2C4);
    LL_I2C_SetTiming(I2C4, I2C_TIMINGR_100KHZ);
    LL_I2C_Enable(I2C4);
}

/**
 * @brief   Entry point: init clocks, I2C and both sensor chips, then poll.
 * @return  Never returns.
 */
int main(void) {
    float t = 0.0f;
    float h = 0.0f;

    /* FPU on (floats are used below) */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    Clock_Init();
    LL_Init1msTick(SystemCoreClock);  /* 1 ms SysTick for LL_mDelay */

    GPIO_ConfigOutput(GPIOB, LL_GPIO_PIN_0);  /* LD1 heartbeat */
    I2C4_Setup();

    aht_st = AHT20_init();   /* I2C_OK expected; NACK = wiring          */
    bmp_st = BMP280_init();  /* I2C_OK expected; NACK = wiring or 0x77  */

    while (1) {
        aht_st = AHT20_read(&t, &h);
        if (aht_st == I2C_OK) {
            aht_t = t;
            aht_h = h;
        }

        bmp_st = BMP280_read(&t, &h);  /* t = degC, h reused as Pa */
        if (bmp_st == I2C_OK) {
            bmp_t = t;
            bmp_p = h;
        }

        LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
        cycle++;
        LL_mDelay(500U);
    }
}
