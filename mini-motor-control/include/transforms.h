/**
 * @file transforms.h
 * @brief Clarke, Park, and inverse Park/Clarke coordinate transformations
 *
 * Covers L3 Mathematical Structures:
 *   - Clarke transform: abc -> alpha-beta (amplitude-invariant)
 *   - Inverse Clarke: alpha-beta -> abc
 *   - Park transform: alpha-beta -> d-q (rotating frame)
 *   - Inverse Park: d-q -> alpha-beta
 *   - Cartesian-to-polar conversion
 *   - Phase voltage reconstruction from PWM duty cycles
 *
 * Key formulas:
 *   Amplitude-invariant Clarke:
 *     alpha = (2/3) * (a - b/2 - c/2) = (2a - b - c)/3
 *     beta  = (2/3) * (sqrt(3)/2 * b - sqrt(3)/2 * c) = (b - c)/sqrt(3)
 *
 *   Park (forward, angle theta):
 *     [d]   [ cos(theta)  sin(theta)] [alpha]
 *     [q] = [-sin(theta)  cos(theta)] [beta ]
 *
 *   Inverse Park (backward, angle theta):
 *     [alpha]   [cos(theta) -sin(theta)] [d]
 *     [beta ] = [sin(theta)  cos(theta)] [q]
 *
 * Reference: Krause et al. (2013) Analysis of Electric Machinery
 * Course: MIT 6.685, ETH 227-0526, Berkeley EE217
 */

#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include "motor_types.h"

#define M_SQRT3      1.7320508075688772f
#define M_SQRT3_DIV2  0.8660254037844386f   /* sqrt(3)/2 */
#define M_1_DIV_SQRT3 0.5773502691896258f   /* 1/sqrt(3) */
#define M_2_DIV_3     0.6666666666666666f   /* 2/3 */
#define M_1_DIV_3     0.3333333333333333f   /* 1/3 */

/*=============================================================================
 * L3: Clarke Transform (abc -> alpha-beta)
 *============================================================================*/

/**
 * @brief Clarke transform: 3-phase to 2-phase stationary (amplitude-invariant)
 *
 * [alpha]   [ 2/3  -1/3  -1/3 ] [ia]
 * [beta ] = [  0   1/s3 -1/s3 ] [ib]
 * [zero ]   [ 1/3   1/3   1/3 ] [ic]
 *
 * Amplitude-invariant: |V_ab| = |V_abc| for balanced 3-phase.
 * Power-invariant version uses factor sqrt(2/3) instead of 2/3.
 *
 * Time complexity: O(1)
 *
 * @param ia  Phase A current/voltage
 * @param ib  Phase B current/voltage
 * @param ic  Phase C current/voltage
 * @return Clarke vector (alpha, beta, zero)
 */
clarke_vector_t clarke_transform(float ia, float ib, float ic);

/**
 * @brief Clarke transform with arbitrary zero-sequence
 * Same as above but handles zero-sequence separately.
 */
void clarke_transform_separate(float ia, float ib, float ic,
                                float *alpha, float *beta, float *zero);

/*=============================================================================
 * L3: Inverse Clarke Transform (alpha-beta -> abc)
 *============================================================================*/

/**
 * @brief Inverse Clarke transform: 2-phase stationary to 3-phase
 *
 * [a]   [ 1    0     1 ] [alpha]
 * [b] = [-1/2  s3/2  1 ] [beta ]
 * [c]   [-1/2 -s3/2  1 ] [zero ]
 */
phase_voltages_t inverse_clarke_transform(const clarke_vector_t *v_ab);

/**
 * @brief Inverse Clarke without zero sequence (balanced 3-phase)
 */
void inverse_clarke_no_zero(float alpha, float beta,
                             float *a, float *b, float *c);

/*=============================================================================
 * L3: Park Transform (alpha-beta -> d-q)
 *============================================================================*/

/**
 * @brief Park transform: stationary alpha-beta to synchronous d-q
 *
 * [d]   [ cos(theta_e)  sin(theta_e)] [alpha]
 * [q] = [-sin(theta_e)  cos(theta_e)] [beta ]
 *
 * @param v_ab   Clarke vector (alpha, beta)
 * @param theta_e Electrical angle [rad]
 * @return Park vector (d, q)
 */
park_vector_t park_transform(const clarke_vector_t *v_ab, float theta_e);

/**
 * @brief Direct Park transform from alpha, beta components
 */
void park_transform_direct(float alpha, float beta, float theta_e,
                            float *d, float *q);

/*=============================================================================
 * L3: Inverse Park Transform (d-q -> alpha-beta)
 *============================================================================*/

/**
 * @brief Inverse Park transform: synchronous d-q to stationary alpha-beta
 *
 * [alpha]   [cos(theta_e) -sin(theta_e)] [d]
 * [beta ] = [sin(theta_e)  cos(theta_e)] [q]
 *
 * @param v_dq   Park vector (d, q)
 * @param theta_e Electrical angle [rad]
 * @return Clarke vector (alpha, beta)
 */
clarke_vector_t inverse_park_transform(const park_vector_t *v_dq, float theta_e);

/**
 * @brief Direct inverse Park transform
 */
void inverse_park_direct(float d, float q, float theta_e,
                          float *alpha, float *beta);

/*=============================================================================
 * L3: Combination Transforms
 *============================================================================*/

/**
 * @brief Full forward transform chain: abc -> alpha-beta -> d-q
 *
 * Combines Clarke and Park in one step for efficiency.
 * Used in every FOC current loop iteration.
 */
park_vector_t abc_to_dq(phase_currents_t i_abc, float theta_e);

/**
 * @brief Full inverse transform chain: d-q -> alpha-beta -> abc
 *
 * Combines inverse Park and inverse Clarke.
 * Used to generate PWM duty cycles from FOC voltage outputs.
 */
phase_voltages_t dq_to_abc(park_vector_t v_dq, float theta_e);

/*=============================================================================
 * L3: Angle Normalization and Utilities
 *============================================================================*/

/**
 * @brief Normalize angle to range [-PI, PI]
 */
float normalize_angle(float theta);

/**
 * @brief Normalize angle to range [0, 2*PI]
 */
float normalize_angle_positive(float theta);

/**
 * @brief Compute angle difference: theta1 - theta2, normalized to [-PI, PI]
 */
float angle_difference(float theta1, float theta2);

/**
 * @brief Convert electrical angle to mechanical: theta_m = theta_e / P
 */
float electrical_to_mechanical_angle(float theta_e, float pole_pairs);

/**
 * @brief Convert mechanical angle to electrical: theta_e = P * theta_m
 */
float mechanical_to_electrical_angle(float theta_m, float pole_pairs);

/**
 * @brief Compute sqrt(d^2 + q^2) - current/voltage vector magnitude
 */
float dq_magnitude(park_vector_t v_dq);

/**
 * @brief Compute atan2(q, d) - current/voltage vector angle
 */
float dq_angle(park_vector_t v_dq);

/*=============================================================================
 * L3: Three-Phase System Utilities
 *============================================================================*/

/**
 * @brief Check if three-phase system is balanced: a + b + c ¡Ö 0
 *
 * For wye-connected systems without neutral, currents must sum to zero.
 */
bool is_balanced(float a, float b, float c, float tolerance);

/**
 * @brief Reconstruct phase C from phases A and B: c = -(a + b)
 *
 * Since ia + ib + ic = 0 for isolated neutral wye systems.
 */
float reconstruct_phase_c(float ia, float ib);

/**
 * @brief Three-phase RMS from amplitude
 *  V_rms_phase = V_peak / sqrt(2)  (sinusoidal)
 *  V_rms_line  = V_peak * sqrt(3/2) (line-to-line)
 */
float phase_peak_to_rms(float amplitude);
float line_peak_to_rms(float amplitude);

/**
 * @brief Sine-triangle PWM voltage to duty cycle conversion
 *  duty = 0.5 + 0.5 * (v_ref / (vdc/2))
 * Center-aligned PWM with bipolar modulation.
 */
float voltage_to_duty(float v_ref, float vdc);

#endif /* TRANSFORMS_H */
