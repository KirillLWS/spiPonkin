#ifndef GAS_O2_H2O_H
#define GAS_O2_H2O_H

#include "gas_o2_h2o_constants.h"

/* O2-H2O gas analyzer math.

   The module contains only computations (no MCU peripheral access): it turns
   raw cell/sensor readings into gas concentrations using the per-unit
   graduation constants from gas_o2_h2o_constants.h. */

#define GAS_PERCENT_TO_PPM  10000.0f  // 1 % = 1e4 ppm

/* Flow rate from a flow-meter output signal x by the quadratic
   Q = a + b*x + c*x^2. */
float GAS_flow(float a, float b, float c, float x);
float GAS_flow_o2(float x);
float GAS_flow_h2o(float x);

/* O2 volume fraction from the solid-electrolyte cell.
   ex - cell EMF, V; et - thermo-EMF Et, V. */
float GAS_o2_percent(float ex, float et);
float GAS_o2_ppm(float ex, float et);

/* H2O volume fraction in ppmV.
   ux - voltage across the sense resistor, V; q - flow rate, cm^3/min.
   The _corrected variant subtracts the background-current voltage uf. */
float GAS_h2o_ppm(float ux, float q);
float GAS_h2o_ppm_corrected(float ux, float uf, float q);

#endif
