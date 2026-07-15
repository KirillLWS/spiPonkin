/**
 * @file    gpio.h
 * @brief   Universal STM32H7 GPIO pin configuration on LL.
 * @details One call sets a pin (or several pins of one port) to any mode
 *          without filling LL_GPIO_InitTypeDef by hand and without opening
 *          the datasheet. The port clock (AHB4) is enabled automatically.
 */

#ifndef GPIO_H
#define GPIO_H

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "stm32h7xx_ll_gpio.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/* No public macros: all arguments map 1:1 to the LL_GPIO_* constants. */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/* No public data types. */

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* The module keeps no state. */

/*==============================================================================
 *                           FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief   Configure a pin (or a pin mask) to any mode in one call.
 * @details Enables the port AHB4 clock automatically, then applies the
 *          full LL configuration. All arguments map directly to LL fields.
 * @param[in,out] port        GPIO port (GPIOA..GPIOK).
 * @param[in]     pins        LL_GPIO_PIN_x mask; may be OR-ed for several pins.
 * @param[in]     mode        LL_GPIO_MODE_INPUT / _OUTPUT / _ALTERNATE / _ANALOG.
 * @param[in]     output_type LL_GPIO_OUTPUT_PUSHPULL / _OPENDRAIN.
 * @param[in]     pull        LL_GPIO_PULL_NO / _UP / _DOWN.
 * @param[in]     speed       LL_GPIO_SPEED_FREQ_LOW / _MEDIUM / _HIGH / _VERY_HIGH.
 * @param[in]     alternate   LL_GPIO_AF_0..LL_GPIO_AF_15 (used only when
 *                            mode = ALTERNATE).
 * @return  None.
 */
void GPIO_Config(GPIO_TypeDef* port, uint32_t pins, uint32_t mode,
                 uint32_t output_type, uint32_t pull, uint32_t speed,
                 uint32_t alternate);

/**
 * @brief   Configure pins as a push-pull output (very-high speed, no pull).
 * @param[in,out] port GPIO port.
 * @param[in]     pins LL_GPIO_PIN_x mask.
 * @return  None.
 */
void GPIO_ConfigOutput(GPIO_TypeDef* port, uint32_t pins);

/**
 * @brief   Configure pins as an input.
 * @param[in,out] port GPIO port.
 * @param[in]     pins LL_GPIO_PIN_x mask.
 * @param[in]     pull LL_GPIO_PULL_NO / _UP / _DOWN.
 * @return  None.
 */
void GPIO_ConfigInput(GPIO_TypeDef* port, uint32_t pins, uint32_t pull);

/**
 * @brief   Configure pins as a push-pull alternate function.
 * @details For open-drain peripherals (I2C) use the full GPIO_Config call.
 * @param[in,out] port      GPIO port.
 * @param[in]     pins      LL_GPIO_PIN_x mask.
 * @param[in]     alternate LL_GPIO_AF_0..LL_GPIO_AF_15.
 * @return  None.
 */
void GPIO_ConfigAlternate(GPIO_TypeDef* port, uint32_t pins, uint32_t alternate);

/**
 * @brief   Configure pins as analog (ADC input / lowest power).
 * @param[in,out] port GPIO port.
 * @param[in]     pins LL_GPIO_PIN_x mask.
 * @return  None.
 */
void GPIO_ConfigAnalog(GPIO_TypeDef* port, uint32_t pins);

#endif /* GPIO_H */
