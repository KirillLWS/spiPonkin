/**
 * @file    main.c
 * @brief   Timer-driven continuous polling of AHT20+BMP280, NUCLEO-H755ZI-Q.
 * @details Демонстрационная циклограмма в аэрокосмическом стиле (DAL A
 *          практики): вся память статическая, циклы ограничены, из функций
 *          один выход, у автоматов защитные default-ветки, на каждый исход
 *          обмена - свой счётчик, шина планируется детерминированно (не
 *          более одной I2C-транзакции на тик 1 мс).
 *
 *          Телеметрия каждого датчика - одна структура (aht, bmp):
 *          - значения в нескольких единицах: температура °C и K, давление
 *            Па, гПа и мм рт. ст.;
 *          - скользящие min/max за последние MINMAX_WINDOW_MS миллисекунд;
 *          - медианный фильтр по MEDIAN_N = 100 измерениям на каждый
 *            физический канал (медиана считается в базовой единице, а
 *            производные единицы пересчитываются из неё: преобразования
 *            линейны и монотонны, поэтому результат тождествен отдельному
 *            фильтру в каждой единице, а память не дублируется);
 *          - счётчики всех исходов обмена и возраст данных.
 *          Структура sys_data - системная телеметрия: частоты ядра и всех
 *          шин МК, тики, аптайм, свободный счётчик main.
 *
 *          Шина: I2C4 на PF14 (SCL, D69, CN9 пин 19) / PF15 (SDA, D68,
 *          CN9 пин 21), AF4.
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
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_utils.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/** I2C4 TIMINGR для 100 кГц при кернел-клоке 100 МГц (APB4 дерева 400 МГц). */
#define I2C_TIMINGR_100KHZ   0x90422731U

#define AHT_PERIOD_MS        100U   /**< слот измерения AHT20 (10 Гц - максимум чипа), мс */
#define AHT_FIRST_CHECK_MS   80U    /**< типовое время преобразования AHT20, мс  */
#define AHT_RECHECK_MS       5U     /**< шаг повторной попытки fetch, мс         */
#define AHT_GIVEUP_MS        300U   /**< аварийное завершение цикла AHT20, мс    */
#define BMP_PERIOD_MS        10U    /**< период чтения BMP280 (100 Гц), мс       */
#define LED_PERIOD_MS        250U   /**< полупериод heartbeat LD1, мс            */
#define MS_PER_SECOND        1000U  /**< тиков в секунде                         */

#define MEDIAN_N             100U   /**< глубина медианного фильтра, измерений   */
#define MINMAX_WINDOW_MS     5000U  /**< окно скользящих min/max, мс. NB: окно
                                         ограничено и глубиной буфера: не более
                                         MEDIAN_N последних измерений (для BMP
                                         при 100 Гц это 1 с истории)            */

#define KELVIN_OFFSET        273.15f   /**< °C -> K                              */
#define PA_PER_HPA           100.0f    /**< Па в одном гПа                       */
#define PA_PER_MMHG          133.322f  /**< Па в одном мм рт. ст.                */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/**
 * @brief Этап конвейера опроса AHT20.
 */
typedef enum {
    AHT_STATE_IDLE = 0,  /**< ожидание следующего слота измерения              */
    AHT_STATE_WAIT = 1   /**< преобразование запущено, ожидание готовности     */
} aht_state_t;

/**
 * @brief Кольцевой буфер одного измерительного канала (медиана + окно).
 */
typedef struct {
    float sample[MEDIAN_N];      /**< последние измерения канала               */
    uint32_t stamp_ms[MEDIAN_N]; /**< тик записи каждого измерения             */
    uint32_t head;               /**< индекс следующей записи                  */
    uint32_t count;              /**< накоплено измерений (насыщается на N)    */
} chan_buf_t;

/**
 * @brief Полная телеметрия датчика AHT20.
 */
typedef struct {
    /* --- мгновенные значения --- */
    float temperature_c;      /**< температура, °C                             */
    float temperature_k;      /**< температура, K                              */
    float humidity_pct;       /**< влажность, %                                */
    /* --- медианный фильтр (100 измерений) --- */
    float t_med_c;            /**< медиана температуры, °C                     */
    float t_med_k;            /**< медиана температуры, K                      */
    float h_med_pct;          /**< медиана влажности, %                        */
    /* --- скользящие min/max за MINMAX_WINDOW_MS --- */
    float t_win_min_c;        /**< минимум температуры в окне, °C              */
    float t_win_max_c;        /**< максимум температуры в окне, °C             */
    float t_win_min_k;        /**< минимум температуры в окне, K               */
    float t_win_max_k;        /**< максимум температуры в окне, K              */
    float h_win_min_pct;      /**< минимум влажности в окне, %                 */
    float h_win_max_pct;      /**< максимум влажности в окне, %                */
    uint8_t valid;            /**< 1 = есть хотя бы одно валидное измерение    */
    /* --- здоровье канала --- */
    I2C_Status last_status;   /**< статус последней транзакции                 */
    uint32_t trigger_cnt;     /**< запущено преобразований                     */
    uint32_t ok_cnt;          /**< успешных измерений (CRC сошёлся)            */
    uint32_t err_nack_cnt;    /**< ошибок NACK (датчик не ответил)             */
    uint32_t err_crc_cnt;     /**< ошибок CRC (кадр повреждён)                 */
    uint32_t err_timeout_cnt; /**< циклов, прерванных по AHT_GIVEUP_MS         */
    uint32_t err_other_cnt;   /**< прочих ошибок шины                          */
    uint32_t last_conv_ms;    /**< длительность последнего преобразования, мс  */
    uint32_t data_age_ms;     /**< возраст последних валидных данных, мс       */
    /* --- состояние конвейера --- */
    aht_state_t state;        /**< текущий этап автомата                       */
    uint32_t period_ms;       /**< мс с начала текущего слота                  */
    uint32_t wait_ms;         /**< мс с момента запуска преобразования         */
} aht20_data_t;

/**
 * @brief Полная телеметрия датчика BMP280.
 */
typedef struct {
    /* --- мгновенные значения --- */
    float temperature_c;      /**< температура, °C                             */
    float temperature_k;      /**< температура, K                              */
    float pressure_pa;        /**< давление, Па                                */
    float pressure_hpa;       /**< давление, гПа                               */
    float pressure_mmhg;      /**< давление, мм рт. ст.                        */
    /* --- медианный фильтр (100 измерений) --- */
    float t_med_c;            /**< медиана температуры, °C                     */
    float t_med_k;            /**< медиана температуры, K                      */
    float p_med_pa;           /**< медиана давления, Па                        */
    float p_med_hpa;          /**< медиана давления, гПа                       */
    float p_med_mmhg;         /**< медиана давления, мм рт. ст.                */
    /* --- скользящие min/max за MINMAX_WINDOW_MS --- */
    float t_win_min_c;        /**< минимум температуры в окне, °C              */
    float t_win_max_c;        /**< максимум температуры в окне, °C             */
    float t_win_min_k;        /**< минимум температуры в окне, K               */
    float t_win_max_k;        /**< максимум температуры в окне, K              */
    float p_win_min_pa;       /**< минимум давления в окне, Па                 */
    float p_win_max_pa;       /**< максимум давления в окне, Па                */
    float p_win_min_hpa;      /**< минимум давления в окне, гПа                */
    float p_win_max_hpa;      /**< максимум давления в окне, гПа               */
    float p_win_min_mmhg;     /**< минимум давления в окне, мм рт. ст.         */
    float p_win_max_mmhg;     /**< максимум давления в окне, мм рт. ст.        */
    uint8_t valid;            /**< 1 = есть хотя бы одно валидное измерение    */
    /* --- здоровье канала --- */
    I2C_Status last_status;   /**< статус последней транзакции                 */
    uint32_t ok_cnt;          /**< успешных чтений                             */
    uint32_t err_nack_cnt;    /**< ошибок NACK (датчик не ответил)             */
    uint32_t err_data_cnt;    /**< пропущенных измерений (код 0x80000)         */
    uint32_t err_timeout_cnt; /**< таймаутов шины                              */
    uint32_t err_other_cnt;   /**< прочих ошибок                               */
    uint32_t data_age_ms;     /**< возраст последних валидных данных, мс       */
    /* --- состояние опроса --- */
    uint32_t period_ms;       /**< мс с последнего чтения                      */
} bmp280_data_t;

/**
 * @brief Системная телеметрия: частоты МК, тики, аптайм.
 */
typedef struct {
    /* --- частоты ядра и шин (заполняются на старте из RCC) --- */
    uint32_t cpu_hz;          /**< частота ядра CM7, Гц                        */
    uint32_t sysclk_hz;       /**< SYSCLK, Гц                                  */
    uint32_t ahb_hclk_hz;     /**< AHB (HCLK), Гц                              */
    uint32_t apb1_pclk_hz;    /**< APB1, Гц                                    */
    uint32_t apb2_pclk_hz;    /**< APB2, Гц                                    */
    uint32_t apb3_pclk_hz;    /**< APB3, Гц                                    */
    uint32_t apb4_pclk_hz;    /**< APB4, Гц                                    */
    uint32_t i2c4_kernel_hz;  /**< кернел-клок I2C4 (= APB4), Гц               */
    uint32_t tim6_clk_hz;     /**< тактирование TIM6 (2 x APB1 при делителе 2), Гц */
    uint32_t i2c_scl_hz;      /**< настроенная скорость шины I2C, Гц           */
    /* --- счётчики времени --- */
    uint32_t tick_cnt;        /**< тиков TIM6 с момента старта                 */
    uint32_t uptime_s;        /**< время работы, с                             */
    uint32_t ms_in_second;    /**< мс внутри текущей секунды                   */
    uint32_t main_loop_cnt;   /**< свободный счётчик main (не блокирован)      */
    uint32_t led_ms;          /**< мс с последнего переключения LD1            */
    uint8_t bus_used;         /**< 1 = в этом тике шина уже занята             */
} sys_data_t;

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/** Телеметрия AHT20 (выходные данные; нулевая инициализация, ненулевые
 *  стартовые значения выставляются в main до запуска таймера). */
static volatile aht20_data_t aht = {0};

/** Телеметрия BMP280. */
static volatile bmp280_data_t bmp = {0};

/** Системная телеметрия. */
static volatile sys_data_t sys_data = {0};

/** Кольцевые буферы каналов (контекст только прерывания). */
static chan_buf_t aht_t_chan = {{0.0f}, {0U}, 0U, 0U};  /**< AHT20: температура, °C */
static chan_buf_t aht_h_chan = {{0.0f}, {0U}, 0U, 0U};  /**< AHT20: влажность, %    */
static chan_buf_t bmp_t_chan = {{0.0f}, {0U}, 0U, 0U};  /**< BMP280: температура, °C */
static chan_buf_t bmp_p_chan = {{0.0f}, {0U}, 0U, 0U};  /**< BMP280: давление, Па   */

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

/**
 * @brief   Настройка I2C4 на PF14 (SCL) / PF15 (SDA), AF4, 100 кГц.
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
 * @brief   Заполнить частоты ядра и шин МК из RCC.
 * @param[in,out] s Системная телеметрия.
 * @return  None.
 */
static void Sys_FillClocks(volatile sys_data_t* s) {
    LL_RCC_ClocksTypeDef clk = {0U, 0U, 0U, 0U, 0U, 0U, 0U};

    LL_RCC_GetSystemClocksFreq(&clk);

    s->sysclk_hz = clk.SYSCLK_Frequency;
    s->cpu_hz = clk.CPUCLK_Frequency;
    s->ahb_hclk_hz = clk.HCLK_Frequency;
    s->apb1_pclk_hz = clk.PCLK1_Frequency;
    s->apb2_pclk_hz = clk.PCLK2_Frequency;
    s->apb3_pclk_hz = clk.PCLK3_Frequency;
    s->apb4_pclk_hz = clk.PCLK4_Frequency;
    s->i2c4_kernel_hz = clk.PCLK4_Frequency;   /* I2C4SEL по умолчанию = pclk4 */
    s->tim6_clk_hz = clk.PCLK1_Frequency * 2U; /* таймеры APB1 при делителе 2  */
    s->i2c_scl_hz = 100000U;                   /* по I2C_TIMINGR_100KHZ        */
}

/**
 * @brief   Записать измерение в кольцевой буфер канала.
 * @param[in,out] c   Буфер канала.
 * @param[in]     v   Значение измерения.
 * @param[in]     now Текущий тик, мс.
 * @return  None.
 */
static void Chan_Push(chan_buf_t* c, float v, uint32_t now) {
    c->sample[c->head] = v;
    c->stamp_ms[c->head] = now;
    c->head = (c->head + 1U) % MEDIAN_N;
    if (c->count < MEDIAN_N) {
        c->count++;
    }
}

/**
 * @brief   Медиана накопленных измерений канала.
 * @details Копия накопленной части буфера сортируется вставками (цикл
 *          ограничен MEDIAN_N); медиана - средний элемент, для чётного
 *          количества - среднее двух средних.
 * @param[in] c Буфер канала.
 * @return  Медиана, или 0.0f при пустом буфере.
 */
static float Chan_Median(const chan_buf_t* c) {
    float tmp[MEDIAN_N] = {0.0f};
    float result = 0.0f;
    float key = 0.0f;
    uint32_t i = 0U;
    int32_t j = 0;

    if (c->count != 0U) {
        for (i = 0U; i < c->count; i++) {
            tmp[i] = c->sample[i];
        }

        for (i = 1U; i < c->count; i++) {
            key = tmp[i];
            j = (int32_t)i - 1;
            while ((j >= 0) && (tmp[j] > key)) {
                tmp[j + 1] = tmp[j];
                j--;
            }
            tmp[j + 1] = key;
        }

        if ((c->count % 2U) != 0U) {
            result = tmp[c->count / 2U];
        } else {
            result = (tmp[c->count / 2U] + tmp[(c->count / 2U) - 1U]) / 2.0f;
        }
    }

    return result;
}

/**
 * @brief   Min/max измерений канала за последние MINMAX_WINDOW_MS.
 * @details Сканируются только записи моложе окна (цикл ограничен MEDIAN_N).
 *          Если в окне нет ни одной записи, возвращается последнее значение.
 * @param[in]  c    Буфер канала (count > 0).
 * @param[in]  now  Текущий тик, мс.
 * @param[out] pmin Минимум в окне.
 * @param[out] pmax Максимум в окне.
 * @return  None.
 */
static void Chan_WindowMinMax(const chan_buf_t* c, uint32_t now,
                              float* pmin, float* pmax) {
    float vmin = 0.0f;
    float vmax = 0.0f;
    float v = 0.0f;
    uint32_t found = 0U;
    uint32_t i = 0U;
    uint32_t last = (c->head + MEDIAN_N - 1U) % MEDIAN_N;

    for (i = 0U; i < c->count; i++) {
        if ((now - c->stamp_ms[i]) <= MINMAX_WINDOW_MS) {
            v = c->sample[i];
            if (found == 0U) {
                vmin = v;
                vmax = v;
                found = 1U;
            } else {
                if (v < vmin) { vmin = v; }
                if (v > vmax) { vmax = v; }
            }
        }
    }

    if (found == 0U) {
        vmin = c->sample[last];
        vmax = vmin;
    }

    *pmin = vmin;
    *pmax = vmax;
}

/**
 * @brief   Разнести ошибку шины AHT20 по счётчикам телеметрии.
 * @param[in,out] d  Телеметрия AHT20.
 * @param[in]     st Статус транзакции.
 * @return  None.
 */
static void AHT20_CountError(volatile aht20_data_t* d, I2C_Status st) {
    if (st == I2C_ERR_NACK) {
        d->err_nack_cnt++;
    } else if (st == I2C_ERR_DATA) {
        d->err_crc_cnt++;
    } else {
        d->err_other_cnt++;
    }
}

/**
 * @brief   Принять валидное измерение AHT20: единицы, медианы, окно.
 * @param[in,out] d   Телеметрия AHT20.
 * @param[in]     t   Температура, °C.
 * @param[in]     h   Влажность, %.
 * @param[in]     now Текущий тик, мс.
 * @return  None.
 */
static void AHT20_Accept(volatile aht20_data_t* d, float t, float h, uint32_t now) {
    float vmin = 0.0f;
    float vmax = 0.0f;

    d->temperature_c = t;
    d->temperature_k = t + KELVIN_OFFSET;
    d->humidity_pct = h;

    Chan_Push(&aht_t_chan, t, now);
    Chan_Push(&aht_h_chan, h, now);

    d->t_med_c = Chan_Median(&aht_t_chan);
    d->t_med_k = d->t_med_c + KELVIN_OFFSET;
    d->h_med_pct = Chan_Median(&aht_h_chan);

    Chan_WindowMinMax(&aht_t_chan, now, &vmin, &vmax);
    d->t_win_min_c = vmin;
    d->t_win_max_c = vmax;
    d->t_win_min_k = vmin + KELVIN_OFFSET;
    d->t_win_max_k = vmax + KELVIN_OFFSET;

    Chan_WindowMinMax(&aht_h_chan, now, &vmin, &vmax);
    d->h_win_min_pct = vmin;
    d->h_win_max_pct = vmax;

    d->valid = 1U;
    d->last_conv_ms = d->wait_ms;
    d->data_age_ms = 0U;
    d->ok_cnt++;
}

/**
 * @brief   Один шаг (1 мс) конвейера AHT20; не более одной транзакции.
 * @details IDLE: раз в AHT_PERIOD_MS запуск (trigger). WAIT: с 80-й мс
 *          каждые AHT_RECHECK_MS попытка fetch; I2C_ERR_TIMEOUT от fetch
 *          означает "ещё преобразует". Автомат с защитным default.
 * @param[in,out] d Телеметрия AHT20.
 * @return  None.
 */
static void AHT20_Pipeline(volatile aht20_data_t* d) {
    I2C_Status st = I2C_OK;
    float t = 0.0f;
    float h = 0.0f;

    if (d->valid == 1U) {
        d->data_age_ms++;
    }

    switch (d->state) {
    case AHT_STATE_IDLE:
        d->period_ms++;
        if ((d->period_ms >= AHT_PERIOD_MS) && (sys_data.bus_used == 0U)) {
            sys_data.bus_used = 1U;
            d->period_ms = 0U;
            st = AHT20_trigger();
            d->last_status = st;
            if (st == I2C_OK) {
                d->trigger_cnt++;
                d->wait_ms = 0U;
                d->state = AHT_STATE_WAIT;
            } else {
                AHT20_CountError(d, st);
            }
        }
        break;

    case AHT_STATE_WAIT:
        d->wait_ms++;
        if ((d->wait_ms >= AHT_FIRST_CHECK_MS) &&
            (((d->wait_ms - AHT_FIRST_CHECK_MS) % AHT_RECHECK_MS) == 0U) &&
            (sys_data.bus_used == 0U)) {
            sys_data.bus_used = 1U;
            st = AHT20_fetch(&t, &h);
            if (st == I2C_OK) {
                d->last_status = st;
                AHT20_Accept(d, t, h, sys_data.tick_cnt);
                d->state = AHT_STATE_IDLE;
            } else if (st != I2C_ERR_TIMEOUT) {
                d->last_status = st;
                AHT20_CountError(d, st);
                d->state = AHT_STATE_IDLE;
            } else {
                /* преобразование ещё идёт: ждём следующей попытки */
            }
        }
        if (d->wait_ms >= AHT_GIVEUP_MS) {
            d->last_status = I2C_ERR_TIMEOUT;
            d->err_timeout_cnt++;
            d->state = AHT_STATE_IDLE;
        }
        break;

    default:
        /* недостижимо; защитное восстановление автомата */
        d->state = AHT_STATE_IDLE;
        break;
    }
}

/**
 * @brief   Принять валидное чтение BMP280: единицы, медианы, окно.
 * @param[in,out] d   Телеметрия BMP280.
 * @param[in]     t   Температура, °C.
 * @param[in]     p   Давление, Па.
 * @param[in]     now Текущий тик, мс.
 * @return  None.
 */
static void BMP280_Accept(volatile bmp280_data_t* d, float t, float p, uint32_t now) {
    float vmin = 0.0f;
    float vmax = 0.0f;

    d->temperature_c = t;
    d->temperature_k = t + KELVIN_OFFSET;
    d->pressure_pa = p;
    d->pressure_hpa = p / PA_PER_HPA;
    d->pressure_mmhg = p / PA_PER_MMHG;

    Chan_Push(&bmp_t_chan, t, now);
    Chan_Push(&bmp_p_chan, p, now);

    d->t_med_c = Chan_Median(&bmp_t_chan);
    d->t_med_k = d->t_med_c + KELVIN_OFFSET;
    d->p_med_pa = Chan_Median(&bmp_p_chan);
    d->p_med_hpa = d->p_med_pa / PA_PER_HPA;
    d->p_med_mmhg = d->p_med_pa / PA_PER_MMHG;

    Chan_WindowMinMax(&bmp_t_chan, now, &vmin, &vmax);
    d->t_win_min_c = vmin;
    d->t_win_max_c = vmax;
    d->t_win_min_k = vmin + KELVIN_OFFSET;
    d->t_win_max_k = vmax + KELVIN_OFFSET;

    Chan_WindowMinMax(&bmp_p_chan, now, &vmin, &vmax);
    d->p_win_min_pa = vmin;
    d->p_win_max_pa = vmax;
    d->p_win_min_hpa = vmin / PA_PER_HPA;
    d->p_win_max_hpa = vmax / PA_PER_HPA;
    d->p_win_min_mmhg = vmin / PA_PER_MMHG;
    d->p_win_max_mmhg = vmax / PA_PER_MMHG;

    d->valid = 1U;
    d->data_age_ms = 0U;
    d->ok_cnt++;
}

/**
 * @brief   Один шаг (1 мс) опроса BMP280; не более одной транзакции.
 * @details Чип в normal mode меряет сам; раз в BMP_PERIOD_MS читается
 *          готовый компенсированный результат.
 * @param[in,out] d Телеметрия BMP280.
 * @return  None.
 */
static void BMP280_Pipeline(volatile bmp280_data_t* d) {
    I2C_Status st = I2C_OK;
    float t = 0.0f;
    float p = 0.0f;

    if (d->valid == 1U) {
        d->data_age_ms++;
    }

    d->period_ms++;
    if ((d->period_ms >= BMP_PERIOD_MS) && (sys_data.bus_used == 0U)) {
        sys_data.bus_used = 1U;
        d->period_ms = 0U;
        st = BMP280_read(&t, &p);
        d->last_status = st;
        if (st == I2C_OK) {
            BMP280_Accept(d, t, p, sys_data.tick_cnt);
        } else if (st == I2C_ERR_NACK) {
            d->err_nack_cnt++;
        } else if (st == I2C_ERR_DATA) {
            d->err_data_cnt++;
        } else if (st == I2C_ERR_TIMEOUT) {
            d->err_timeout_cnt++;
        } else {
            d->err_other_cnt++;
        }
    }
}

/**
 * @brief   Системный учёт: тики, аптайм, heartbeat LD1.
 * @param[in,out] s Системная телеметрия.
 * @return  None.
 */
static void Sys_Pipeline(volatile sys_data_t* s) {
    s->tick_cnt++;

    s->ms_in_second++;
    if (s->ms_in_second >= MS_PER_SECOND) {
        s->ms_in_second = 0U;
        s->uptime_s++;
    }

    s->led_ms++;
    if (s->led_ms >= LED_PERIOD_MS) {
        s->led_ms = 0U;
        LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);  /* LD1, 2 Гц */
    }
}

/**
 * @brief   Точка входа: тактирование, I2C4, инициализация датчиков,
 *          далее весь опрос в тике TIM6.
 * @return  Не возвращается.
 */
int main(void) {
    /* FPU (float в драйверах и прерывании) */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    Clock_Init();
    LL_Init1msTick(SystemCoreClock);  /* 1 мс SysTick для блокирующего init */

    GPIO_ConfigOutput(GPIOB, LL_GPIO_PIN_0);  /* LD1 heartbeat */
    I2C4_Setup();

    Sys_FillClocks(&sys_data);

    /* Стартовые значения счётчиков периодов: первый опрос без ожидания. */
    aht.period_ms = AHT_PERIOD_MS;
    bmp.period_ms = BMP_PERIOD_MS;

    /* Блокирующая инициализация допустима: конвейер ещё не запущен. */
    aht.last_status = AHT20_init();
    bmp.last_status = BMP280_init();

    TIM6_Setup();  /* тик 1 мс */

    while (1) {
        sys_data.main_loop_cnt++;  /* свободный цикл: main не блокирован */
    }
}

/**
 * @brief   Тик TIM6 (1 мс): системный учёт + конвейеры обоих датчиков.
 * @details Детерминированное планирование: флаг bus_used гарантирует не
 *          более одной I2C-транзакции на тик. Приоритет у AHT20 (его слот
 *          жёстче), BMP280 добирает свободные тики. Худший тик: транзакция
 *          (~0.8 мс на 100 кГц) + медианная сортировка двух каналов.
 * @return  None.
 */
void TIM6_DAC_IRQHandler(void) {
    sys_data.bus_used = 0U;

    Sys_Pipeline(&sys_data);
    AHT20_Pipeline(&aht);
    BMP280_Pipeline(&bmp);

    LL_TIM_ClearFlag_UPDATE(TIM6);
}
