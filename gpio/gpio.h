#ifndef GPIO_H
#define GPIO_H

#include "stm32h7xx_ll_gpio.h"

/* Universal GPIO pin configuration.

   One call sets a pin (or several pins of one port) to any mode without
   filling the LL_GPIO_InitTypeDef by hand. All arguments map directly to the
   LL fields:

     pins        - LL_GPIO_PIN_x mask (may be OR-ed for several pins)
     mode        - LL_GPIO_MODE_INPUT / _OUTPUT / _ALTERNATE / _ANALOG
     output_type - LL_GPIO_OUTPUT_PUSHPULL / _OPENDRAIN
     pull        - LL_GPIO_PULL_NO / _UP / _DOWN
     speed       - LL_GPIO_SPEED_FREQ_LOW / _MEDIUM / _HIGH / _VERY_HIGH
     alternate   - LL_GPIO_AF_0 ... _AF_15 (used only when mode = ALTERNATE)

   The port clock (AHB4) is enabled automatically. */
void GPIO_Config(GPIO_TypeDef* port, uint32_t pins, uint32_t mode,
                 uint32_t output_type, uint32_t pull, uint32_t speed,
                 uint32_t alternate);

/* Shortcuts for the common cases (very-high speed, no pull unless asked). */
void GPIO_ConfigOutput(GPIO_TypeDef* port, uint32_t pins);
void GPIO_ConfigInput(GPIO_TypeDef* port, uint32_t pins, uint32_t pull);
void GPIO_ConfigAlternate(GPIO_TypeDef* port, uint32_t pins, uint32_t alternate);
void GPIO_ConfigAnalog(GPIO_TypeDef* port, uint32_t pins);

#endif
