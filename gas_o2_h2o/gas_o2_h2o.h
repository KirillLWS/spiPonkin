#ifndef GAS_O2_H2O_H
#define GAS_O2_H2O_H

#include "gas_o2_h2o_constants.h"

/* O2-H2O gas analyzer math.

   The module contains only computations (no MCU peripheral access): it turns
   raw cell/sensor readings into gas concentrations using the per-unit
   graduation constants from gas_o2_h2o_constants.h. */

#define GAS_PERCENT_TO_PPM  10000.0f  // 1 % = 1e4 ppm
#define GAS_KELVIN_OFFSET   273.15f   // degC -> K

/* Flow rate from a flow-meter output signal x by the quadratic
   Q = a + b*x + c*x^2. */
float GAS_flow(float a, float b, float c, float x);
float GAS_flow_o2(float x);
float GAS_flow_h2o(float x);

/* Convert a cell temperature in degC (e.g. from MAX31856_read_temp) to K. */
float GAS_kelvin(float t_celsius);

/* Nernst slope k(T) = 4F/(R*T) scaled from its nominal value.
   t_kelvin - actual cell temperature, K. */
float GAS_o2_nernst_k(float t_kelvin);

/* O2 volume fraction from the solid-electrolyte cell.
   ex - cell EMF, V; et - thermo-EMF Et, V.
   The _at variants take the measured cell temperature (K); the plain
   variants use the nominal temperature GAS_O2_T_NOM. */
float GAS_o2_percent_at(float ex, float et, float t_kelvin);
float GAS_o2_ppm_at(float ex, float et, float t_kelvin);
float GAS_o2_percent(float ex, float et);
float GAS_o2_ppm(float ex, float et);

/* H2O volume fraction in ppmV.
   ux - voltage across the sense resistor, V; q - flow rate, cm^3/min.
   The _corrected variant subtracts the background-current voltage uf. */
float GAS_h2o_ppm(float ux, float q);
float GAS_h2o_ppm_corrected(float ux, float uf, float q);

/* Checked variants: validate the inputs against the GAS_*_MIN/MAX limits
   from the constants header before computing. On GAS_ERR_RANGE the output
   is left untouched, so a stale-but-valid value is not overwritten. */
typedef enum {
    GAS_OK        = 0,
    GAS_ERR_RANGE = 1
} GAS_Status;

GAS_Status GAS_o2_percent_checked(float ex, float et, float t_kelvin, float* out);
GAS_Status GAS_h2o_ppm_checked(float ux, float uf, float q, float* out);

#endif
