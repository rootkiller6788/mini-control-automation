/**
 * @file transforms.c
 * @brief Implementation of Clarke, Park, and inverse coordinate transforms
 *
 * L3 Mathematical Structures - complete implementation of coordinate
 * frame transformations used in all field-oriented motor control.
 *
 * All transforms use amplitude-invariant convention (not power-invariant).
 * Amplitude-invariant: peak of alpha-beta equals peak of abc (balanced).
 * Power-invariant would use factor sqrt(2/3) instead of 2/3.
 */

#include "transforms.h"
#include <math.h>
#include <stdlib.h>

/* ========================================================================
 * L3: Clarke Transform (abc -> alpha-beta)
 * ========================================================================
 *
 * Amplitude-invariant Clarke (also known as "non-power-invariant"):
 *   alpha = (2/3)*[ a - (1/2)*b - (1/2)*c ] = (2a - b - c) / 3
 *   beta  = (2/3)*[ (sqrt(3)/2)*b - (sqrt(3)/2)*c ] = (b - c) / sqrt(3)
 *   zero  = (a + b + c) / 3
 *
 * Derivation: Project 3-phase 120-degree windings onto orthogonal axes.
 * For balanced system (a+b+c=0): alpha = a, beta = (b-c)/sqrt(3).
 */

clarke_vector_t clarke_transform(float ia, float ib, float ic)
{
    clarke_vector_t result;
    result.alpha = (2.0f * ia - ib - ic) / 3.0f;   /* (2a-b-c)/3 */
    result.beta  = (ib - ic) / M_SQRT3;              /* (b-c)/sqrt(3) */
    result.zero  = (ia + ib + ic) / 3.0f;            /* homopolar */
    return result;
}

void clarke_transform_separate(float ia, float ib, float ic,
                                float *alpha, float *beta, float *zero)
{
    *alpha = (2.0f * ia - ib - ic) / 3.0f;
    *beta  = (ib - ic) / M_SQRT3;
    *zero  = (ia + ib + ic) / 3.0f;
}

/* ========================================================================
 * L3: Inverse Clarke Transform (alpha-beta -> abc)
 * ========================================================================
 *
 * [a]   [ 1    0     1 ] [alpha]
 * [b] = [-1/2  s3/2  1 ] [beta ]
 * [c]   [-1/2 -s3/2  1 ] [zero ]
 *
 * For balanced 3-phase with no zero-sequence: a = alpha, b = c = -alpha/2 ¡À beta*sqrt(3)/2
 */

phase_voltages_t inverse_clarke_transform(const clarke_vector_t *v_ab)
{
    phase_voltages_t result;
    if (!v_ab) { result.va = result.vb = result.vc = 0.0f; return result; }
    result.va = v_ab->alpha + v_ab->zero;
    result.vb = -0.5f * v_ab->alpha + M_SQRT3_DIV2 * v_ab->beta + v_ab->zero;
    result.vc = -0.5f * v_ab->alpha - M_SQRT3_DIV2 * v_ab->beta + v_ab->zero;
    return result;
}

void inverse_clarke_no_zero(float alpha, float beta,
                             float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + M_SQRT3_DIV2 * beta;
    *c = -0.5f * alpha - M_SQRT3_DIV2 * beta;
}

/* ========================================================================
 * L3: Park Transform (alpha-beta -> d-q)
 * ========================================================================
 *
 * [d]   [ cos(theta)  sin(theta)] [alpha]
 * [q] = [-sin(theta)  cos(theta)] [beta ]
 *
 * Rotation matrix by angle theta. Maps stationary alpha-beta frame
 * onto the synchronously rotating d-q frame aligned with rotor flux.
 * theta is the electrical angle of the rotor (P * theta_mechanical).
 *
 * For theta = 0: d-axis aligns with alpha-axis (phase A magnetic axis).
 */

park_vector_t park_transform(const clarke_vector_t *v_ab, float theta_e)
{
    park_vector_t result;
    if (!v_ab) { result.d = 0.0f; result.q = 0.0f; return result; }
    float cos_t = cosf(theta_e);
    float sin_t = sinf(theta_e);
    result.d =  cos_t * v_ab->alpha + sin_t * v_ab->beta;
    result.q = -sin_t * v_ab->alpha + cos_t * v_ab->beta;
    return result;
}

void park_transform_direct(float alpha, float beta, float theta_e,
                            float *d, float *q)
{
    float cos_t = cosf(theta_e);
    float sin_t = sinf(theta_e);
    *d =  cos_t * alpha + sin_t * beta;
    *q = -sin_t * alpha + cos_t * beta;
}

/* ========================================================================
 * L3: Inverse Park Transform (d-q -> alpha-beta)
 * ========================================================================
 *
 * [alpha]   [cos(theta) -sin(theta)] [d]
 * [beta ] = [sin(theta)  cos(theta)] [q]
 *
 * Inverse rotation: maps d-q rotating frame back to stationary alpha-beta.
 */

clarke_vector_t inverse_park_transform(const park_vector_t *v_dq, float theta_e)
{
    clarke_vector_t result;
    if (!v_dq) { result.alpha = 0.0f; result.beta = 0.0f; result.zero = 0.0f; return result; }
    float cos_t = cosf(theta_e);
    float sin_t = sinf(theta_e);
    result.alpha = cos_t * v_dq->d - sin_t * v_dq->q;
    result.beta  = sin_t * v_dq->d + cos_t * v_dq->q;
    result.zero  = 0.0f;
    return result;
}

void inverse_park_direct(float d, float q, float theta_e,
                          float *alpha, float *beta)
{
    float cos_t = cosf(theta_e);
    float sin_t = sinf(theta_e);
    *alpha = cos_t * d - sin_t * q;
    *beta  = sin_t * d + cos_t * q;
}

/* ========================================================================
 * L3: Combined Transforms (for FOC inner loop efficiency)
 * ========================================================================
 *
 * In FOC, the typical current loop sequence is:
 *   1. Measure Ia, Ib (Ic = -Ia - Ib for wye configuration)
 *   2. abc_to_dq(I_abc, theta_e) -> Idq_measured
 *   3. PI controller on Id, Iq -> Vdq_reference
 *   4. dq_to_abc(Vdq_ref, theta_e) -> Vabc -> PWM duties
 *
 * These combined functions avoid intermediate struct copies.
 */

park_vector_t abc_to_dq(phase_currents_t i_abc, float theta_e)
{
    float cos_t = cosf(theta_e);
    float sin_t = sinf(theta_e);

    /* Step 1: Clarke transform */
    float alpha = (2.0f * i_abc.ia - i_abc.ib - i_abc.ic) / 3.0f;
    float beta  = (i_abc.ib - i_abc.ic) / M_SQRT3;

    /* Step 2: Park transform */
    park_vector_t result;
    result.d =  cos_t * alpha + sin_t * beta;
    result.q = -sin_t * alpha + cos_t * beta;
    return result;
}

phase_voltages_t dq_to_abc(park_vector_t v_dq, float theta_e)
{
    float cos_t = cosf(theta_e);
    float sin_t = sinf(theta_e);

    /* Step 1: Inverse Park */
    float alpha = cos_t * v_dq.d - sin_t * v_dq.q;
    float beta  = sin_t * v_dq.d + cos_t * v_dq.q;

    /* Step 2: Inverse Clarke (no zero-sequence) */
    phase_voltages_t result;
    result.va = alpha;
    result.vb = -0.5f * alpha + M_SQRT3_DIV2 * beta;
    result.vc = -0.5f * alpha - M_SQRT3_DIV2 * beta;
    return result;
}

/* ========================================================================
 * L3: Angle Normalization & Mechanical/Electrical Conversion
 * ========================================================================
 *
 * theta_e = P * theta_m  (electrical angle = pole_pairs * mechanical angle)
 * omega_e = P * omega_m  (electrical speed = pole_pairs * mechanical speed)
 *
 * Angles are periodic with period 2*PI.  Normalization wraps angles
 * to a canonical range to avoid numerical issues with large angles
 * accumulating in open-loop integration.
 */

float normalize_angle(float theta)
{
    /* Wrap to [-PI, PI) */
    theta = fmodf(theta + (float)M_PI, 2.0f * (float)M_PI);
    if (theta < 0.0f) theta += 2.0f * (float)M_PI;
    return theta - (float)M_PI;
}

float normalize_angle_positive(float theta)
{
    /* Wrap to [0, 2*PI) */
    theta = fmodf(theta, 2.0f * (float)M_PI);
    if (theta < 0.0f) theta += 2.0f * (float)M_PI;
    return theta;
}

float angle_difference(float theta1, float theta2)
{
    float diff = theta1 - theta2;
    /* Normalize to [-PI, PI] */
    diff = fmodf(diff + (float)M_PI, 2.0f * (float)M_PI);
    if (diff < 0.0f) diff += 2.0f * (float)M_PI;
    return diff - (float)M_PI;
}

float electrical_to_mechanical_angle(float theta_e, float pole_pairs)
{
    if (pole_pairs <= 0.0f) return 0.0f;
    return theta_e / pole_pairs;
}

float mechanical_to_electrical_angle(float theta_m, float pole_pairs)
{
    return theta_m * pole_pairs;
}

/* ========================================================================
 * L3: Vector Magnitude and Angle in dq Frame
 * ========================================================================
 *
 * |V_dq| = sqrt(d^2 + q^2)  ¡ª voltage/current magnitude
 * ang(V_dq) = atan2(q, d)   ¡ª angle of vector in dq plane
 */

float dq_magnitude(park_vector_t v_dq)
{
    return sqrtf(v_dq.d * v_dq.d + v_dq.q * v_dq.q);
}

float dq_angle(park_vector_t v_dq)
{
    return atan2f(v_dq.q, v_dq.d);
}

/* ========================================================================
 * L3: Three-Phase System Utilities
 * ========================================================================
 *
 * For isolated-neutral wye systems, Kirchhoff's Current Law requires:
 *   ia + ib + ic = 0
 * This allows reconstructing the third phase from two measurements.
 */

bool is_balanced(float a, float b, float c, float tolerance)
{
    float sum = a + b + c;
    return fabsf(sum) <= tolerance;
}

float reconstruct_phase_c(float ia, float ib)
{
    return -(ia + ib);
}

float phase_peak_to_rms(float amplitude)
{
    return amplitude / 1.4142135623730951f; /* amplitude / sqrt(2) */
}

float line_peak_to_rms(float amplitude)
{
    /* V_line_rms = V_phase_peak * sqrt(3/2) */
    return amplitude * M_SQRT3 / 1.4142135623730951f;
}

float voltage_to_duty(float v_ref, float vdc)
{
    /* Center-aligned PWM: duty = 0.5 + v_ref/Vdc
     * Safe clamping to [0, 1] range */
    if (vdc <= 0.001f) return 0.5f;
    float duty = 0.5f + v_ref / vdc;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    return duty;
}
