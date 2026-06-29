#ifndef MAX31856_H
#define MAX31856_H

#include "stdint.h"

#include "max31856_constants.h"

/* Register read addresses. Write address = read address | MAX31856_WRITE_ADDR */
#define MAX31856_REG_CR0     0x00  // Configuration 0 Register
#define MAX31856_REG_CR1     0x01  // Configuration 1 Register
#define MAX31856_REG_MASK    0x02  // Fault Mask Register
#define MAX31856_REG_CJHF    0x03  // Cold-Junction High Fault Threshold
#define MAX31856_REG_CJLF    0x04  // Cold-Junction Low Fault Threshold
#define MAX31856_REG_LTHFTH  0x05  // Linearized Temperature High Fault Threshold MSB
#define MAX31856_REG_LTHFTL  0x06  // Linearized Temperature High Fault Threshold LSB
#define MAX31856_REG_LTLFTH  0x07  // Linearized Temperature Low Fault Threshold MSB
#define MAX31856_REG_LTLFTL  0x08  // Linearized Temperature Low Fault Threshold LSB
#define MAX31856_REG_CJTO    0x09  // Cold-Junction Temperature Offset Register
#define MAX31856_REG_CJTH    0x0A  // Cold-Junction Temperature Register, MSB
#define MAX31856_REG_CJTL    0x0B  // Cold-Junction Temperature Register, LSB
#define MAX31856_REG_LTCBH   0x0C  // Linearized TC Temperature, Byte 2
#define MAX31856_REG_LTCBM   0x0D  // Linearized TC Temperature, Byte 1
#define MAX31856_REG_LTCBL   0x0E  // Linearized TC Temperature, Byte 0
#define MAX31856_REG_SR      0x0F  // Fault Status Register

#define MAX31856_WRITE_ADDR  0x80  // OR with a register address to form its write address
#define MAX31856_ADDR_MASK   0x7F  // mask to form a read address

/* Configuration 0 Register (CR0) bits */
#define MAX31856_CR0_CMODE     0x80  // 1 = automatic conversion mode, 0 = normally off
#define MAX31856_CR0_1SHOT     0x40  // one-shot conversion (self-clearing)
#define MAX31856_CR0_OCFAULT1  0x20  // open-circuit fault detection, bit 1
#define MAX31856_CR0_OCFAULT0  0x10  // open-circuit fault detection, bit 0
#define MAX31856_CR0_CJ        0x08  // 1 = cold-junction sensor disabled
#define MAX31856_CR0_FAULT     0x04  // fault mode (0 = comparator, 1 = interrupt)
#define MAX31856_CR0_FAULTCLR  0x02  // fault status clear (self-clearing)
#define MAX31856_CR0_50HZ      0x01  // 1 = 50 Hz rejection, 0 = 60 Hz rejection

/* Configuration 1 Register (CR1) thermocouple type (bits 3:0) */
#define MAX31856_TCTYPE_B   0x00
#define MAX31856_TCTYPE_E   0x01
#define MAX31856_TCTYPE_J   0x02
#define MAX31856_TCTYPE_K   0x03
#define MAX31856_TCTYPE_N   0x04
#define MAX31856_TCTYPE_R   0x05
#define MAX31856_TCTYPE_S   0x06
#define MAX31856_TCTYPE_T   0x07
#define MAX31856_VMODE_G8   0x08  // voltage mode, gain = 8
#define MAX31856_VMODE_G32  0x0C  // voltage mode, gain = 32

/* Configuration 1 Register (CR1) averaging mode (bits 6:4) */
#define MAX31856_AVG_1    0x00  // 1 sample
#define MAX31856_AVG_2    0x10  // 2 samples averaged
#define MAX31856_AVG_4    0x20  // 4 samples averaged
#define MAX31856_AVG_8    0x30  // 8 samples averaged
#define MAX31856_AVG_16   0x40  // 16 samples averaged

/* Fault Status Register (SR) bits */
#define MAX31856_FAULT_CJRANGE  0x80  // cold-junction out of range
#define MAX31856_FAULT_TCRANGE  0x40  // thermocouple out of range
#define MAX31856_FAULT_CJHIGH   0x20  // cold-junction above high threshold
#define MAX31856_FAULT_CJLOW    0x10  // cold-junction below low threshold
#define MAX31856_FAULT_TCHIGH   0x08  // thermocouple above high threshold
#define MAX31856_FAULT_TCLOW    0x04  // thermocouple below low threshold
#define MAX31856_FAULT_OVUV     0x02  // over-voltage or under-voltage
#define MAX31856_FAULT_OPEN     0x01  // open-circuit (broken thermocouple)

void MAX31856_write_reg(uint8_t addr, uint8_t value);
uint8_t MAX31856_read_reg(uint8_t addr);
void MAX31856_read_buf(uint8_t addr, uint8_t* buf, uint32_t len);

void MAX31856_init(uint8_t tc_type, uint8_t avg);
void MAX31856_oneshot(void);
uint8_t MAX31856_read_fault(void);

float MAX31856_read_temp(void);
float MAX31856_read_cj_temp(void);

#endif
