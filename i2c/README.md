# i2c

Блокирующий мастер-транспорт I²C на LL для STM32H7 — аналог слоя `spi.c` из
основного проекта, но с контролем ошибок. Сам периферийный блок (тактирование,
`TIMINGR`, выводы, `Enable`) настраивается в проекте; модуль только ведёт
транзакции. Адреса 7-битные (несдвинутые).

## Функции

```c
I2C_Status I2C_Write(I2C_TypeDef* i2c, uint8_t addr, const uint8_t* data, uint32_t len);
I2C_Status I2C_Read(I2C_TypeDef* i2c, uint8_t addr, uint8_t* data, uint32_t len);
I2C_Status I2C_WriteRead(I2C_TypeDef* i2c, uint8_t addr,
                         const uint8_t* wdata, uint32_t wlen,
                         uint8_t* rdata, uint32_t rlen);  // чтение регистра через repeated start
I2C_Status I2C_BusRecover(GPIO_TypeDef* scl_port, uint32_t scl_pin,
                          GPIO_TypeDef* sda_port, uint32_t sda_pin);
```

Статусы: `I2C_OK`, `I2C_ERR_TIMEOUT` (шина/флаг не дождались — прошивка не
виснет), `I2C_ERR_NACK` (устройство не ответило — можно детектировать
отсутствие датчика), `I2C_ERR_DATA` (кадр пришёл, но содержимое битое — CRC,
пропущенное измерение; выставляется драйверами датчиков).

## Восстановление зависшей шины

Классический отказ: МК сбросился посреди чтения, слейв держит SDA низко, и
обычные транзакции невозможны. `I2C_BusRecover` переводит SCL/SDA в
открытый сток GPIO, даёт до 9 тактов SCL, пока слейв не отпустит SDA, и
формирует STOP. После него выводы нужно вернуть в альтернативную функцию и
заново включить периферию:

```c
if (I2C_BusRecover(GPIOB, LL_GPIO_PIN_6, GPIOB, LL_GPIO_PIN_9) == I2C_OK) {
    GPIO_Config(GPIOB, LL_GPIO_PIN_6 | LL_GPIO_PIN_9,
                LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_OPENDRAIN,
                LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_VERY_HIGH, LL_GPIO_AF_4);
    /* ... re-init I2C1 ... */
}
```

## Замечания

- Таймаут `I2C_TIMEOUT` — счётчик итераций, не время: при смене тактовой
  частоты константу стоит пересмотреть (переопределяется до включения `i2c.c`).
- Функции не реентерабельные: при нескольких задачах RTOS на одной шине
  оберните вызовы мьютексом на стороне приложения.
- Зависимости: `gpio/` (для восстановления шины), LL I2C/GPIO.
