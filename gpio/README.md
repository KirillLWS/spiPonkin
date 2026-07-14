# gpio

Универсальная настройка выводов STM32H7 на LL — чтобы не собирать
`LL_GPIO_InitTypeDef` вручную и не открывать даташит на каждую линию.

## Функции

```c
// Полный контроль: любой режим одним вызовом.
void GPIO_Config(GPIO_TypeDef* port, uint32_t pins, uint32_t mode,
                 uint32_t output_type, uint32_t pull, uint32_t speed,
                 uint32_t alternate);

// Частые случаи:
void GPIO_ConfigOutput(GPIO_TypeDef* port, uint32_t pins);              // push-pull выход
void GPIO_ConfigInput(GPIO_TypeDef* port, uint32_t pins, uint32_t pull);// вход
void GPIO_ConfigAlternate(GPIO_TypeDef* port, uint32_t pins, uint32_t af);// периферия (push-pull)
void GPIO_ConfigAnalog(GPIO_TypeDef* port, uint32_t pins);             // аналог
```

Тактирование порта (AHB4) включается автоматически по указателю на порт —
отдельный `LL_AHB4_GRP1_EnableClock` вызывать не нужно.

## Примеры

```c
GPIO_ConfigOutput(GPIOB, LL_GPIO_PIN_14);                 // светодиод
GPIO_ConfigInput(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_UP); // кнопка

// USART1 TX/RX (AF7):
GPIO_ConfigAlternate(GPIOA, LL_GPIO_PIN_9 | LL_GPIO_PIN_10, LL_GPIO_AF_7);

// I2C1 SCL/SDA — открытый сток + подтяжка, только через полный вызов:
GPIO_Config(GPIOB, LL_GPIO_PIN_6 | LL_GPIO_PIN_9,
            LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_OPENDRAIN,
            LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_VERY_HIGH, LL_GPIO_AF_4);
```

Аргументы 1:1 соответствуют полям LL (`LL_GPIO_MODE_*`, `LL_GPIO_OUTPUT_*`,
`LL_GPIO_PULL_*`, `LL_GPIO_SPEED_FREQ_*`, `LL_GPIO_AF_*`). Поддержаны порты
GPIOA…GPIOK (по наличию на конкретном МК).
