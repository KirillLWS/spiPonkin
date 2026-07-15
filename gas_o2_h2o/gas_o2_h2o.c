/**
 * @file    gas_o2_h2o.c
 * @brief   O2-H2O gas analyzer math implementation.
 * @details See gas_o2_h2o.h for the public API description.
 */

/*==============================================================================
 *                              INCLUDED FILES
 *============================================================================*/

#include "gas_o2_h2o.h"

#include "math.h"
#include "stddef.h"

/*==============================================================================
 *                            MACRO DEFINITIONS
 *============================================================================*/

/* No private macros. */

/*==============================================================================
 *                               DATA TYPES
 *============================================================================*/

/* No private data types. */

/*==============================================================================
 *                                VARIABLES
 *============================================================================*/

/* The module keeps no state: all functions are pure. */

/*==============================================================================
 *                                FUNCTIONS
 *============================================================================*/

float GAS_flow(float a, float b, float c, float x) {
    return a + (b * x) + (c * x * x);
}

float GAS_flow_o2(float x) {
    return GAS_flow(GAS_FLOW_O2_A, GAS_FLOW_O2_B, GAS_FLOW_O2_C, x);
}

float GAS_flow_h2o(float x) {
    return GAS_flow(GAS_FLOW_H2O_A, GAS_FLOW_H2O_B, GAS_FLOW_H2O_C, x);
}

float GAS_kelvin(float t_celsius) {
    return t_celsius + GAS_KELVIN_OFFSET;
}

float GAS_o2_nernst_k(float t_kelvin) {
    float k = GAS_O2_NERNST_K;

    /* k(T) = 4F/(R*T) scales inversely with the measured cell temperature. */
    if (t_kelvin != 0.0f) {
        k = (GAS_O2_NERNST_K * GAS_O2_T_NOM) / t_kelvin;
    }

    return k;
}

float GAS_o2_percent_at(float ex, float et, float t_kelvin) {
    float k = GAS_o2_nernst_k(t_kelvin);

    return GAS_O2_C_REF * expf(-k * (ex - et));
}

float GAS_o2_ppm_at(float ex, float et, float t_kelvin) {
    return GAS_o2_percent_at(ex, et, t_kelvin) * GAS_PERCENT_TO_PPM;
}

float GAS_o2_percent(float ex, float et) {
    return GAS_o2_percent_at(ex, et, GAS_O2_T_NOM);
}

float GAS_o2_ppm(float ex, float et) {
    return GAS_o2_percent(ex, et) * GAS_PERCENT_TO_PPM;
}

float GAS_h2o_ppm_corrected(float ux, float uf, float q) {
    float result = 0.0f;

    /* Q comes from a flow meter and must be non-zero to divide by it. */
    if (q != 0.0f) {
        result = GAS_H2O_K * ((ux - uf) / q);
    }

    return result;
}

float GAS_h2o_ppm(float ux, float q) {
    return GAS_h2o_ppm_corrected(ux, GAS_H2O_UF, q);
}

GAS_Status GAS_o2_percent_checked(float ex, float et, float t_kelvin, float* out) {
    GAS_Status st = GAS_ERR_RANGE;
    float de = ex - et;

    if ((out != NULL) &&
        (de >= GAS_O2_EX_MIN) && (de <= GAS_O2_EX_MAX) &&
        (t_kelvin >= GAS_O2_T_MIN) && (t_kelvin <= GAS_O2_T_MAX)) {
        *out = GAS_o2_percent_at(ex, et, t_kelvin);
        st = GAS_OK;
    }

    return st;
}

GAS_Status GAS_h2o_ppm_checked(float ux, float uf, float q, float* out) {
    GAS_Status st = GAS_ERR_RANGE;
    float du = ux - uf;

    if ((out != NULL) &&
        (du >= 0.0f) && (ux <= GAS_H2O_UX_MAX) &&
        (q >= GAS_FLOW_Q_MIN)) {
        *out = GAS_h2o_ppm_corrected(ux, uf, q);
        st = GAS_OK;
    }

    return st;
}
