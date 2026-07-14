#include "gpio.h"

#include "stm32h7xx_ll_bus.h"

static void gpio_enable_clock(const GPIO_TypeDef* port) {
#if defined(GPIOA)
    if (port == GPIOA) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    } else
#endif
#if defined(GPIOB)
    if (port == GPIOB) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
    } else
#endif
#if defined(GPIOC)
    if (port == GPIOC) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    } else
#endif
#if defined(GPIOD)
    if (port == GPIOD) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    } else
#endif
#if defined(GPIOE)
    if (port == GPIOE) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
    } else
#endif
#if defined(GPIOF)
    if (port == GPIOF) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
    } else
#endif
#if defined(GPIOG)
    if (port == GPIOG) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
    } else
#endif
#if defined(GPIOH)
    if (port == GPIOH) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
    } else
#endif
#if defined(GPIOI)
    if (port == GPIOI) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOI);
    } else
#endif
#if defined(GPIOJ)
    if (port == GPIOJ) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOJ);
    } else
#endif
#if defined(GPIOK)
    if (port == GPIOK) {
        LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOK);
    } else
#endif
    {
        /* unknown port: nothing to enable */
    }
}

void GPIO_Config(GPIO_TypeDef* port, uint32_t pins, uint32_t mode,
                 uint32_t output_type, uint32_t pull, uint32_t speed,
                 uint32_t alternate) {
    LL_GPIO_InitTypeDef init;

    gpio_enable_clock(port);

    init.Pin = pins;
    init.Mode = mode;
    init.OutputType = output_type;
    init.Pull = pull;
    init.Speed = speed;
    init.Alternate = alternate;

    (void)LL_GPIO_Init(port, &init);
}

void GPIO_ConfigOutput(GPIO_TypeDef* port, uint32_t pins) {
    GPIO_Config(port, pins, LL_GPIO_MODE_OUTPUT, LL_GPIO_OUTPUT_PUSHPULL,
                LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_VERY_HIGH, LL_GPIO_AF_0);
}

void GPIO_ConfigInput(GPIO_TypeDef* port, uint32_t pins, uint32_t pull) {
    GPIO_Config(port, pins, LL_GPIO_MODE_INPUT, LL_GPIO_OUTPUT_PUSHPULL,
                pull, LL_GPIO_SPEED_FREQ_VERY_HIGH, LL_GPIO_AF_0);
}

void GPIO_ConfigAlternate(GPIO_TypeDef* port, uint32_t pins, uint32_t alternate) {
    GPIO_Config(port, pins, LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_PUSHPULL,
                LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_VERY_HIGH, alternate);
}

void GPIO_ConfigAnalog(GPIO_TypeDef* port, uint32_t pins) {
    GPIO_Config(port, pins, LL_GPIO_MODE_ANALOG, LL_GPIO_OUTPUT_PUSHPULL,
                LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0);
}
