/**
 * @file    gas_o2_h2o.h
 * @brief   O2-H2O gas analyzer math.
 * @details Pure computation module (no MCU peripheral access): converts raw
 *          cell/sensor readings into gas concentrations using the per-unit
 *          graduation constants from gas_o2_h2o_constants.h. The zirconia O2
 *          cell obeys the Nernst relation, so the O2 slope depends on the
 *          actual cell temperature, which is measured by the thermocouple
 *          (max31856 driver).
 */

#ifndef GAS_O2_H2O_H
#define GAS_O2_H2O_H

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "gas_o2_h2o_constants.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

#define GAS_PERCENT_TO_PPM  10000.0f  /**< 1 % = 1e4 ppm                      */
#define GAS_KELVIN_OFFSET   273.15f   /**< degC to K conversion offset        */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/**
 * @brief Validation status of the *_checked functions.
 */
typedef enum {
    GAS_OK        = 0,  /**< Inputs valid, output written                     */
    GAS_ERR_RANGE = 1   /**< An input is outside the GAS_*_MIN/MAX limits;
                             the output value was left untouched              */
} GAS_Status;

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* The module keeps no state: all functions are pure. */

/*==============================================================================
 *                           FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief   Flow rate from a flow-meter signal by an arbitrary quadratic.
 * @details Q = a + b*x + c*x^2; the coefficients come from the individual
 *          calibration of each channel.
 * @param[in] a Constant coefficient.
 * @param[in] b Linear coefficient.
 * @param[in] c Quadratic coefficient.
 * @param[in] x Flow-meter output signal.
 * @return  Flow rate, cm^3/min.
 */
float GAS_flow(float a, float b, float c, float x);

/**
 * @brief   Flow rate of the O2 channel (calibrated coefficients).
 * @param[in] x Flow-meter output signal.
 * @return  Flow rate, cm^3/min.
 */
float GAS_flow_o2(float x);

/**
 * @brief   Flow rate of the H2O channel (calibrated coefficients).
 * @param[in] x Flow-meter output signal.
 * @return  Flow rate, cm^3/min.
 */
float GAS_flow_h2o(float x);

/**
 * @brief   Convert a temperature from degC to K.
 * @details Helper for chaining with MAX31856_read_temp().
 * @param[in] t_celsius Temperature, degC.
 * @return  Temperature, K.
 */
float GAS_kelvin(float t_celsius);

/**
 * @brief   Nernst slope k(T) = 4F/(R*T) at the measured cell temperature.
 * @details Scaled from the nominal value: k(T) = GAS_O2_NERNST_K *
 *          GAS_O2_T_NOM / T. Guarded against T = 0 (returns the nominal k).
 * @param[in] t_kelvin Actual cell temperature, K.
 * @return  Nernst slope, 1/V.
 */
float GAS_o2_nernst_k(float t_kelvin);

/**
 * @brief   O2 volume fraction at the measured cell temperature.
 * @details Cx[%] = GAS_O2_C_REF * exp(-k(T) * (ex - et)).
 * @param[in] ex       Cell EMF, V.
 * @param[in] et       Thermo-EMF correction Et, V.
 * @param[in] t_kelvin Cell temperature, K (e.g. GAS_kelvin(MAX31856_read_temp())).
 * @return  O2 volume fraction, %.
 */
float GAS_o2_percent_at(float ex, float et, float t_kelvin);

/**
 * @brief   O2 volume fraction at the measured cell temperature, in ppm.
 * @param[in] ex       Cell EMF, V.
 * @param[in] et       Thermo-EMF correction Et, V.
 * @param[in] t_kelvin Cell temperature, K.
 * @return  O2 volume fraction, ppm.
 */
float GAS_o2_ppm_at(float ex, float et, float t_kelvin);

/**
 * @brief   O2 volume fraction at the nominal cell temperature GAS_O2_T_NOM.
 * @param[in] ex Cell EMF, V.
 * @param[in] et Thermo-EMF correction Et, V.
 * @return  O2 volume fraction, %.
 */
float GAS_o2_percent(float ex, float et);

/**
 * @brief   O2 volume fraction at the nominal cell temperature, in ppm.
 * @param[in] ex Cell EMF, V.
 * @param[in] et Thermo-EMF correction Et, V.
 * @return  O2 volume fraction, ppm.
 */
float GAS_o2_ppm(float ex, float et);

/**
 * @brief   H2O concentration with the default background correction.
 * @details C[ppmV] = GAS_H2O_K * (ux - GAS_H2O_UF) / q. Division is guarded:
 *          q = 0 returns 0.
 * @param[in] ux Voltage across the sense resistor, V.
 * @param[in] q  Flow rate, cm^3/min.
 * @return  H2O concentration, ppmV.
 */
float GAS_h2o_ppm(float ux, float q);

/**
 * @brief   H2O concentration with an explicit background voltage.
 * @details C[ppmV] = GAS_H2O_K * (ux - uf) / q. Division is guarded:
 *          q = 0 returns 0.
 * @param[in] ux Voltage across the sense resistor, V.
 * @param[in] uf Background-current voltage, V.
 * @param[in] q  Flow rate, cm^3/min.
 * @return  H2O concentration, ppmV.
 */
float GAS_h2o_ppm_corrected(float ux, float uf, float q);

/**
 * @brief   O2 computation with input validation.
 * @details Rejects EMF and temperature outside the GAS_*_MIN/MAX limits
 *          instead of producing plausible-looking garbage (a wrong-sign EMF
 *          feeds exp() and yields thousands of percent).
 * @param[in]  ex       Cell EMF, V.
 * @param[in]  et       Thermo-EMF correction Et, V.
 * @param[in]  t_kelvin Cell temperature, K.
 * @param[out] out      O2 volume fraction, %. Untouched on error.
 * @return  GAS_OK, or GAS_ERR_RANGE when an input (or out = NULL) is invalid.
 */
GAS_Status GAS_o2_percent_checked(float ex, float et, float t_kelvin, float* out);

/**
 * @brief   H2O computation with input validation.
 * @details Requires ux >= uf, ux <= GAS_H2O_UX_MAX and q >= GAS_FLOW_Q_MIN
 *          (a near-zero flow inflates the result unboundedly).
 * @param[in]  ux  Voltage across the sense resistor, V.
 * @param[in]  uf  Background-current voltage, V.
 * @param[in]  q   Flow rate, cm^3/min.
 * @param[out] out H2O concentration, ppmV. Untouched on error.
 * @return  GAS_OK, or GAS_ERR_RANGE when an input (or out = NULL) is invalid.
 */
GAS_Status GAS_h2o_ppm_checked(float ux, float uf, float q, float* out);

#endif /* GAS_O2_H2O_H */
