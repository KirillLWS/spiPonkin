/**
 * @file    max31856.h
 * @brief   MAX31856 precision thermocouple-to-digital converter driver.
 * @details SPI-connected driver built on the byte primitives declared in
 *          max31856_constants.h. Contains the register map, bit fields and
 *          the public API. No waits or delays inside any function, so the
 *          whole API is safe to call from a periodic interrupt; blocking
 *          usage under an RTOS is done by polling from a task.
 */

#ifndef MAX31856_H
#define MAX31856_H

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "stdint.h"

#include "max31856_constants.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/* --- Register read addresses (write address = read address | MAX31856_WRITE_ADDR) --- */
#define MAX31856_REG_CR0     0x00U  /**< Configuration 0 Register              */
#define MAX31856_REG_CR1     0x01U  /**< Configuration 1 Register              */
#define MAX31856_REG_MASK    0x02U  /**< Fault Mask Register                   */
#define MAX31856_REG_CJHF    0x03U  /**< Cold-Junction High Fault Threshold    */
#define MAX31856_REG_CJLF    0x04U  /**< Cold-Junction Low Fault Threshold     */
#define MAX31856_REG_LTHFTH  0x05U  /**< TC High Fault Threshold, MSB          */
#define MAX31856_REG_LTHFTL  0x06U  /**< TC High Fault Threshold, LSB          */
#define MAX31856_REG_LTLFTH  0x07U  /**< TC Low Fault Threshold, MSB           */
#define MAX31856_REG_LTLFTL  0x08U  /**< TC Low Fault Threshold, LSB           */
#define MAX31856_REG_CJTO    0x09U  /**< Cold-Junction Temperature Offset      */
#define MAX31856_REG_CJTH    0x0AU  /**< Cold-Junction Temperature, MSB        */
#define MAX31856_REG_CJTL    0x0BU  /**< Cold-Junction Temperature, LSB        */
#define MAX31856_REG_LTCBH   0x0CU  /**< Linearized TC Temperature, Byte 2     */
#define MAX31856_REG_LTCBM   0x0DU  /**< Linearized TC Temperature, Byte 1     */
#define MAX31856_REG_LTCBL   0x0EU  /**< Linearized TC Temperature, Byte 0     */
#define MAX31856_REG_SR      0x0FU  /**< Fault Status Register                 */

#define MAX31856_WRITE_ADDR  0x80U  /**< OR with a register address to form its write address */
#define MAX31856_ADDR_MASK   0x7FU  /**< AND with an address to form its read address         */

/* --- Configuration 0 Register (CR0) bits --- */
#define MAX31856_CR0_CMODE     0x80U  /**< 1 = automatic conversion mode, 0 = normally off  */
#define MAX31856_CR0_1SHOT     0x40U  /**< One-shot conversion trigger (self-clearing)      */
#define MAX31856_CR0_OCFAULT1  0x20U  /**< Open-circuit fault detection mode, bit 1         */
#define MAX31856_CR0_OCFAULT0  0x10U  /**< Open-circuit fault detection mode, bit 0         */
#define MAX31856_CR0_CJ        0x08U  /**< 1 = cold-junction sensor disabled                */
#define MAX31856_CR0_FAULT     0x04U  /**< Fault pin mode: 0 = comparator, 1 = interrupt    */
#define MAX31856_CR0_FAULTCLR  0x02U  /**< Fault status clear (self-clearing)               */
#define MAX31856_CR0_50HZ      0x01U  /**< 1 = 50 Hz mains rejection, 0 = 60 Hz             */

/* --- Configuration 1 Register (CR1), thermocouple type field (bits 3:0) --- */
#define MAX31856_TCTYPE_B   0x00U  /**< Thermocouple type B                    */
#define MAX31856_TCTYPE_E   0x01U  /**< Thermocouple type E                    */
#define MAX31856_TCTYPE_J   0x02U  /**< Thermocouple type J                    */
#define MAX31856_TCTYPE_K   0x03U  /**< Thermocouple type K                    */
#define MAX31856_TCTYPE_N   0x04U  /**< Thermocouple type N                    */
#define MAX31856_TCTYPE_R   0x05U  /**< Thermocouple type R                    */
#define MAX31856_TCTYPE_S   0x06U  /**< Thermocouple type S                    */
#define MAX31856_TCTYPE_T   0x07U  /**< Thermocouple type T                    */
#define MAX31856_VMODE_G8   0x08U  /**< Voltage mode, gain = 8                 */
#define MAX31856_VMODE_G32  0x0CU  /**< Voltage mode, gain = 32                */

/* --- Configuration 1 Register (CR1), averaging field (bits 6:4) --- */
#define MAX31856_AVG_1    0x00U  /**< No averaging (1 sample)                  */
#define MAX31856_AVG_2    0x10U  /**< Average of 2 samples                     */
#define MAX31856_AVG_4    0x20U  /**< Average of 4 samples                     */
#define MAX31856_AVG_8    0x30U  /**< Average of 8 samples                     */
#define MAX31856_AVG_16   0x40U  /**< Average of 16 samples                    */

/* --- CR1 field masks (used to sanitise the init arguments) --- */
#define MAX31856_CR1_AVG_MASK   0x70U  /**< Averaging field mask, CR1[6:4]     */
#define MAX31856_CR1_TYPE_MASK  0x0FU  /**< TC type field mask, CR1[3:0]       */

/* --- Fault Status Register (SR) bits --- */
#define MAX31856_FAULT_CJRANGE  0x80U  /**< Cold-junction out of range         */
#define MAX31856_FAULT_TCRANGE  0x40U  /**< Thermocouple out of range          */
#define MAX31856_FAULT_CJHIGH   0x20U  /**< Cold-junction above high threshold */
#define MAX31856_FAULT_CJLOW    0x10U  /**< Cold-junction below low threshold  */
#define MAX31856_FAULT_TCHIGH   0x08U  /**< Thermocouple above high threshold  */
#define MAX31856_FAULT_TCLOW    0x04U  /**< Thermocouple below low threshold   */
#define MAX31856_FAULT_OVUV     0x02U  /**< Over-voltage or under-voltage      */
#define MAX31856_FAULT_OPEN     0x01U  /**< Open circuit (broken thermocouple) */

/* --- Register resolutions, degC per LSB --- */
#define MAX31856_TC_LSB    0.0078125f  /**< Linearized TC reading, 2^-7 degC   */
#define MAX31856_CJ_LSB    0.015625f   /**< Cold-junction reading, 2^-6 degC   */
#define MAX31856_TCTH_LSB  0.0625f     /**< TC fault thresholds, 2^-4 degC     */
#define MAX31856_CJTO_LSB  0.0625f     /**< Cold-junction offset, 2^-4 degC    */
/* CJHF/CJLF cold-junction thresholds are signed 8-bit with 1 degC per LSB.    */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/**
 * @brief Driver status codes.
 */
typedef enum {
    MAX31856_OK  = 0,  /**< Operation completed, chip responded as expected    */
    MAX31856_ERR = 1   /**< No or wrong SPI response (chip missing, bad wiring/mode) */
} MAX31856_Status;

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* The driver keeps no state: all data lives in the chip registers. */

/*==============================================================================
 *                           FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief   Write one chip register.
 * @details Drives CS low, sends the write address (addr | MAX31856_WRITE_ADDR)
 *          and the data byte in a single frame, then releases CS.
 * @param[in] addr  Register read address (MAX31856_REG_x).
 * @param[in] value Byte to write.
 * @return  None.
 */
void MAX31856_write_reg(uint8_t addr, uint8_t value);

/**
 * @brief   Read one chip register.
 * @details Sends the read address and clocks one dummy byte to shift the
 *          register contents out.
 * @param[in] addr Register read address (MAX31856_REG_x).
 * @return  Register value.
 */
uint8_t MAX31856_read_reg(uint8_t addr);

/**
 * @brief   Read a block of consecutive registers.
 * @details Uses the chip address auto-increment, so multi-byte values
 *          (temperatures) are captured coherently within one CS frame.
 * @param[in]  addr Start register read address.
 * @param[out] buf  Destination buffer, at least len bytes.
 * @param[in]  len  Number of registers to read.
 * @return  None.
 */
void MAX31856_read_buf(uint8_t addr, uint8_t* buf, uint32_t len);

/**
 * @brief   Check that the chip is present and the SPI link works.
 * @details Writes a scratch pattern to the MASK register, reads it back and
 *          compares; the previous MASK value is saved and restored. Detects
 *          a missing chip, a wrong SPI mode and a stuck MISO line.
 * @return  MAX31856_OK when the readback matches, MAX31856_ERR otherwise.
 */
MAX31856_Status MAX31856_probe(void);

/**
 * @brief   Configure the chip and start continuous conversions.
 * @details Sets CR0 to automatic conversion with open-circuit detection and
 *          60 Hz rejection, programs the TC type and averaging into CR1 and
 *          unmasks all faults.
 * @param[in] tc_type Thermocouple type, MAX31856_TCTYPE_x or MAX31856_VMODE_x.
 * @param[in] avg     Averaging mode, MAX31856_AVG_x.
 * @return  None.
 */
void MAX31856_init(uint8_t tc_type, uint8_t avg);

/**
 * @brief   Select 50 or 60 Hz mains rejection.
 * @details The datasheet requires conversions to be stopped while the filter
 *          is changed, so automatic mode is suspended for the write and
 *          restored afterwards.
 * @param[in] use_50hz Non-zero selects 50 Hz rejection, zero selects 60 Hz.
 * @return  None.
 */
void MAX31856_set_filter(uint8_t use_50hz);

/**
 * @brief   Clear the latched fault status.
 * @details Sets the self-clearing FAULTCLR bit in CR0.
 * @return  None.
 */
void MAX31856_clear_fault(void);

/**
 * @brief   Program the thermocouple fault thresholds.
 * @details Converts degC to the 16-bit threshold code (LSB 0.0625 degC),
 *          clamps to the register range and writes LTHFTH..LTLFTL. Crossings
 *          are reported by the SR bits TCHIGH / TCLOW.
 * @param[in] low_c  Low threshold, degC.
 * @param[in] high_c High threshold, degC.
 * @return  None.
 */
void MAX31856_set_tc_limits(float low_c, float high_c);

/**
 * @brief   Program the cold-junction fault thresholds.
 * @details Thresholds are signed 8-bit degC values (LSB 1 degC). Crossings
 *          are reported by the SR bits CJHIGH / CJLOW.
 * @param[in] low_c  Low threshold, degC.
 * @param[in] high_c High threshold, degC.
 * @return  None.
 */
void MAX31856_set_cj_limits(int8_t low_c, int8_t high_c);

/**
 * @brief   Program the cold-junction temperature offset (CJTO).
 * @details Converts degC to the 8-bit offset code (LSB 0.0625 degC) with
 *          clamping; the chip adds the offset to its cold-junction reading.
 * @param[in] offset_c Offset, degC.
 * @return  None.
 */
void MAX31856_set_cj_offset(float offset_c);

/**
 * @brief   Trigger a single conversion (non-blocking).
 * @details Leaves automatic mode and sets the self-clearing 1SHOT bit. The
 *          conversion takes ~150-200 ms; poll MAX31856_conversion_done().
 * @return  None.
 */
void MAX31856_oneshot(void);

/**
 * @brief   Poll whether a one-shot conversion has finished.
 * @details Reads CR0 and tests the self-clearing 1SHOT bit.
 * @return  1 when the conversion is finished, 0 while it is running.
 */
uint8_t MAX31856_conversion_done(void);

/**
 * @brief   Read the fault status register.
 * @return  SR value; test against the MAX31856_FAULT_x bit masks.
 */
uint8_t MAX31856_read_fault(void);

/**
 * @brief   Read the linearized thermocouple temperature as a raw code.
 * @details Reads LTCBH..LTCBL in one frame and sign-extends the 19-bit
 *          value. For firmware that avoids floating point.
 * @return  Signed code, MAX31856_TC_LSB (2^-7) degC per LSB.
 */
int32_t MAX31856_read_temp_raw(void);

/**
 * @brief   Read the cold-junction temperature as a raw code.
 * @details Reads CJTH/CJTL in one frame and sign-extends the 14-bit value.
 * @return  Signed code, MAX31856_CJ_LSB (2^-6) degC per LSB.
 */
int32_t MAX31856_read_cj_temp_raw(void);

/**
 * @brief   Read the linearized thermocouple temperature.
 * @return  Temperature, degC.
 */
float MAX31856_read_temp(void);

/**
 * @brief   Read the cold-junction (chip) temperature.
 * @return  Temperature, degC.
 */
float MAX31856_read_cj_temp(void);

#endif /* MAX31856_H */
