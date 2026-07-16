/**
 * @file    main_quick_check.c
 * @brief   Timer-driven bring-up check of AHT20+BMP280 on NUCLEO-H755ZI-Q.
 * @details Drop-in replacement of Src/main.c for the test_CM7 project.
 *          Sensor polling runs entirely in the TIM6 1 ms tick using the
 *          NON-BLOCKING driver API (trigger/busy/fetch) - the main loop
 *          stays free (main_loop_cnt keeps counting). Blocking calls are
 *          used only once, during init, before the timer starts.
 *
 *          Bus: I2C4 on PF14 (SCL, D69, CN9 pin 19) / PF15 (SDA, D68,
 *          CN9 pin 21), AF4. The Zio silk name is I2C_B; the silicon
 *          peripheral on these pins is I2C4 (APB4).
 *
 *          Requires Inc/aht20_bmp280_constants.h (copy of the .h_example
 *          with AHT_BMP_I2C set to I2C4).
 */

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include <stdint.h>

#include "clock.h"
#include "timer.h"
#include "gpio.h"
#include "i2c.h"
#include "aht20_bmp280.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_utils.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/** I2C4 TIMINGR for 100 kHz at a 100 MHz kernel clock (APB4 of the
 *  400 MHz project clock tree). Recalculate if the tree changes. */
#define I2C_TIMINGR_100KHZ   0x90422731U

#define AHT_PERIOD_MS        1000U  /**< AHT20 measurement cycle period, ms  */
#define AHT_FIRST_CHECK_MS   80U    /**< Typical AHT20 conversion time, ms   */
#define AHT_RECHECK_MS       20U    /**< Re-poll step while still busy, ms   */
#define AHT_GIVEUP_MS        200U   /**< Abort the cycle after this wait, ms */
#define BMP_PERIOD_MS        500U   /**< BMP280 read period, ms              */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/**
 * @brief AHT20 polling pipeline state.
 */
typedef enum {
    AHT_STATE_IDLE = 0,  /**< Waiting for the next measurement slot          */
    AHT_STATE_WAIT = 1   /**< Triggered, waiting for the conversion to end   */
} aht_state_t;

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* --- Live-watch results --- */
static volatile float aht_t = 0.0f;            /**< AHT20 temperature, degC  */
static volatile float aht_h = 0.0f;            /**< AHT20 humidity, %        */
static volatile float bmp_t = 0.0f;            /**< BMP280 temperature, degC */
static volatile float bmp_p = 0.0f;            /**< BMP280 pressure, Pa      */
static volatile I2C_Status aht_st = I2C_OK;    /**< Last AHT20 status        */
static volatile I2C_Status bmp_st = I2C_OK;    /**< Last BMP280 status       */
static volatile uint32_t aht_ok_cnt = 0U;      /**< Successful AHT20 reads   */
static volatile uint32_t bmp_ok_cnt = 0U;      /**< Successful BMP280 reads  */
static volatile uint32_t main_loop_cnt = 0U;   /**< Proves main() is not blocked */

/* --- Pipeline state (ISR context only) --- */
static aht_state_t aht_state = AHT_STATE_IDLE; /**< AHT20 pipeline stage     */
static uint32_t aht_ms = AHT_PERIOD_MS;        /**< ms since last AHT20 slot (start immediately) */
static uint32_t aht_wait_ms = 0U;              /**< ms since AHT20 trigger   */
static uint32_t bmp_ms = 0U;                   /**< ms since last BMP280 read */

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

/**
 * @brief   Configure I2C4 on PF14 (SCL) / PF15 (SDA), AF4, 100 kHz.
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
 * @brief   One 1 ms step of the AHT20 non-blocking pipeline.
 * @details IDLE: every AHT_PERIOD_MS starts a conversion (AHT20_trigger).
 *          WAIT: from AHT_FIRST_CHECK_MS on, tries AHT20_fetch every
 *          AHT_RECHECK_MS; I2C_ERR_TIMEOUT means "still converting".
 * @return  None.
 */
static void AHT20_Pipeline(void) {
    I2C_Status st = I2C_OK;
    float t = 0.0f;
    float h = 0.0f;

    switch (aht_state) {
    case AHT_STATE_IDLE:
        aht_ms++;
        if (aht_ms >= AHT_PERIOD_MS) {
            aht_ms = 0U;
            aht_st = AHT20_trigger();
            if (aht_st == I2C_OK) {
                aht_wait_ms = 0U;
                aht_state = AHT_STATE_WAIT;
            }
        }
        break;

    case AHT_STATE_WAIT:
        aht_wait_ms++;
        if ((aht_wait_ms >= AHT_FIRST_CHECK_MS) &&
            (((aht_wait_ms - AHT_FIRST_CHECK_MS) % AHT_RECHECK_MS) == 0U)) {
            st = AHT20_fetch(&t, &h);
            if (st == I2C_OK) {
                aht_t = t;
                aht_h = h;
                aht_ok_cnt++;
                aht_st = st;
                aht_state = AHT_STATE_IDLE;
            } else if (st != I2C_ERR_TIMEOUT) {
                aht_st = st;  /* bus/CRC error: abort this cycle */
                aht_state = AHT_STATE_IDLE;
            } else {
                /* still converting: keep waiting */
            }
        }
        if (aht_wait_ms >= AHT_GIVEUP_MS) {
            aht_st = I2C_ERR_TIMEOUT;
            aht_state = AHT_STATE_IDLE;
        }
        break;

    default:
        aht_state = AHT_STATE_IDLE;
        break;
    }
}

/**
 * @brief   One 1 ms step of the BMP280 polling.
 * @details BMP280 in normal mode converts on its own; every BMP_PERIOD_MS
 *          the compensated result is read out (no waits inside) and the
 *          heartbeat LED is toggled.
 * @return  None.
 */
static void BMP280_Pipeline(void) {
    I2C_Status st = I2C_OK;
    float t = 0.0f;
    float p = 0.0f;

    bmp_ms++;
    if (bmp_ms >= BMP_PERIOD_MS) {
        bmp_ms = 0U;
        st = BMP280_read(&t, &p);
        bmp_st = st;
        if (st == I2C_OK) {
            bmp_t = t;
            bmp_p = p;
            bmp_ok_cnt++;
        }
        LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);  /* LD1 heartbeat, 1 Hz */
    }
}

/**
 * @brief   Entry point: init clocks, I2C4 and both chips, then hand the
 *          polling over to the TIM6 tick.
 * @return  Never returns.
 */
int main(void) {
    /* FPU on (floats are used in the drivers and the ISR) */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    Clock_Init();
    LL_Init1msTick(SystemCoreClock);  /* 1 ms SysTick for the blocking init */

    GPIO_ConfigOutput(GPIOB, LL_GPIO_PIN_0);  /* LD1 heartbeat */
    I2C4_Setup();

    /* Blocking init is allowed here: the timer pipeline is not running yet. */
    aht_st = AHT20_init();   /* I2C_OK expected; NACK = wiring              */
    bmp_st = BMP280_init();  /* I2C_OK expected; NACK = wiring or addr 0x77 */

    TIM6_Setup();            /* 1 ms tick, same timer as before */

    while (1) {
        main_loop_cnt++;     /* free-running: proves nothing blocks main() */
    }
}

/**
 * @brief   TIM6 1 ms tick: the whole sensor polling pipeline.
 * @details Explicit pipeline, one stage per function, no blocking calls.
 * @return  None.
 */
void TIM6_DAC_IRQHandler(void) {
    AHT20_Pipeline();
    BMP280_Pipeline();

    LL_TIM_ClearFlag_UPDATE(TIM6);
}
