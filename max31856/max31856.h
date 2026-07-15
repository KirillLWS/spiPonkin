#ifndef MAX31856_H
#define MAX31856_H

#include "stdint.h"

#include "max31856_constants.h"

/* Register read addresses. Write address = read address | MAX31856_WRITE_ADDR */
#define MAX31856_REG_CR0     0x00U  // Configuration 0 Register
#define MAX31856_REG_CR1     0x01U  // Configuration 1 Register
#define MAX31856_REG_MASK    0x02U  // Fault Mask Register
#define MAX31856_REG_CJHF    0x03U  // Cold-Junction High Fault Threshold
#define MAX31856_REG_CJLF    0x04U  // Cold-Junction Low Fault Threshold
#define MAX31856_REG_LTHFTH  0x05U  // Linearized Temperature High Fault Threshold MSB
#define MAX31856_REG_LTHFTL  0x06U  // Linearized Temperature High Fault Threshold LSB
#define MAX31856_REG_LTLFTH  0x07U  // Linearized Temperature Low Fault Threshold MSB
#define MAX31856_REG_LTLFTL  0x08U  // Linearized Temperature Low Fault Threshold LSB
#define MAX31856_REG_CJTO    0x09U  // Cold-Junction Temperature Offset Register
#define MAX31856_REG_CJTH    0x0AU  // Cold-Junction Temperature Register, MSB
#define MAX31856_REG_CJTL    0x0BU  // Cold-Junction Temperature Register, LSB
#define MAX31856_REG_LTCBH   0x0CU  // Linearized TC Temperature, Byte 2
#define MAX31856_REG_LTCBM   0x0DU  // Linearized TC Temperature, Byte 1
#define MAX31856_REG_LTCBL   0x0EU  // Linearized TC Temperature, Byte 0
#define MAX31856_REG_SR      0x0FU  // Fault Status Register

#define MAX31856_WRITE_ADDR  0x80U  // OR with a register address to form its write address
#define MAX31856_ADDR_MASK   0x7FU  // mask to form a read address

/* Configuration 0 Register (CR0) bits */
#define MAX31856_CR0_CMODE     0x80U  // 1 = automatic conversion mode, 0 = normally off
#define MAX31856_CR0_1SHOT     0x40U  // one-shot conversion (self-clearing)
#define MAX31856_CR0_OCFAULT1  0x20U  // open-circuit fault detection, bit 1
#define MAX31856_CR0_OCFAULT0  0x10U  // open-circuit fault detection, bit 0
#define MAX31856_CR0_CJ        0x08U  // 1 = cold-junction sensor disabled
#define MAX31856_CR0_FAULT     0x04U  // fault mode (0 = comparator, 1 = interrupt)
#define MAX31856_CR0_FAULTCLR  0x02U  // fault status clear (self-clearing)
#define MAX31856_CR0_50HZ      0x01U  // 1 = 50 Hz rejection, 0 = 60 Hz rejection

/* Configuration 1 Register (CR1) thermocouple type (bits 3:0) */
#define MAX31856_TCTYPE_B   0x00U
#define MAX31856_TCTYPE_E   0x01U
#define MAX31856_TCTYPE_J   0x02U
#define MAX31856_TCTYPE_K   0x03U
#define MAX31856_TCTYPE_N   0x04U
#define MAX31856_TCTYPE_R   0x05U
#define MAX31856_TCTYPE_S   0x06U
#define MAX31856_TCTYPE_T   0x07U
#define MAX31856_VMODE_G8   0x08U  // voltage mode, gain = 8
#define MAX31856_VMODE_G32  0x0CU  // voltage mode, gain = 32

/* Configuration 1 Register (CR1) averaging mode (bits 6:4) */
#define MAX31856_AVG_1    0x00U  // 1 sample
#define MAX31856_AVG_2    0x10U  // 2 samples averaged
#define MAX31856_AVG_4    0x20U  // 4 samples averaged
#define MAX31856_AVG_8    0x30U  // 8 samples averaged
#define MAX31856_AVG_16   0x40U  // 16 samples averaged

/* Bit masks of the CR1 fields, used to sanitise the init arguments */
#define MAX31856_CR1_AVG_MASK   0x70U
#define MAX31856_CR1_TYPE_MASK  0x0FU

/* Fault Status Register (SR) bits */
#define MAX31856_FAULT_CJRANGE  0x80U  // cold-junction out of range
#define MAX31856_FAULT_TCRANGE  0x40U  // thermocouple out of range
#define MAX31856_FAULT_CJHIGH   0x20U  // cold-junction above high threshold
#define MAX31856_FAULT_CJLOW    0x10U  // cold-junction below low threshold
#define MAX31856_FAULT_TCHIGH   0x08U  // thermocouple above high threshold
#define MAX31856_FAULT_TCLOW    0x04U  // thermocouple below low threshold
#define MAX31856_FAULT_OVUV     0x02U  // over-voltage or under-voltage
#define MAX31856_FAULT_OPEN     0x01U  // open-circuit (broken thermocouple)

/* Resolution of the temperature registers, degC per LSB */
#define MAX31856_TC_LSB    0.0078125f  // linearized thermocouple reading, 2^-7 degC
#define MAX31856_CJ_LSB    0.015625f   // cold-junction reading, 2^-6 degC
#define MAX31856_TCTH_LSB  0.0625f     // TC fault thresholds LTxFTx, 2^-4 degC
#define MAX31856_CJTO_LSB  0.0625f     // cold-junction offset CJTO, 2^-4 degC
/* CJHF/CJLF cold-junction thresholds are signed 8-bit with 1 degC per LSB. */

typedef enum {
    MAX31856_OK   = 0,
    MAX31856_ERR  = 1   // no or wrong SPI response (chip missing, bad wiring/mode)
} MAX31856_Status;

/* Register access */
void MAX31856_write_reg(uint8_t addr, uint8_t value);
uint8_t MAX31856_read_reg(uint8_t addr);
void MAX31856_read_buf(uint8_t addr, uint8_t* buf, uint32_t len);

/* Presence check: write-readback of a scratch pattern (MASK register is
   saved and restored). Call after bus init to verify the chip answers. */
MAX31856_Status MAX31856_probe(void);

/* Configuration */
void MAX31856_init(uint8_t tc_type, uint8_t avg);
void MAX31856_set_filter(uint8_t use_50hz);
void MAX31856_clear_fault(void);

/* Fault thresholds and cold-junction offset (SR bits xxHIGH/xxLOW report
   crossings). Values are clamped to the register ranges. */
void MAX31856_set_tc_limits(float low_c, float high_c);   // LSB 0.0625 degC
void MAX31856_set_cj_limits(int8_t low_c, int8_t high_c); // LSB 1 degC
void MAX31856_set_cj_offset(float offset_c);              // LSB 0.0625 degC

/* One-shot flow (non-blocking, interrupt/state-machine friendly):
   MAX31856_oneshot() triggers, MAX31856_conversion_done() polls the
   self-clearing 1SHOT bit (~150-200 ms typical conversion). */
void MAX31856_oneshot(void);
uint8_t MAX31856_conversion_done(void);

/* Status and measurement (no waits inside, safe to call periodically) */
uint8_t MAX31856_read_fault(void);
int32_t MAX31856_read_temp_raw(void);
int32_t MAX31856_read_cj_temp_raw(void);
float MAX31856_read_temp(void);
float MAX31856_read_cj_temp(void);

#endif
