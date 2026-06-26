/**
 * @file pwm_modulation.c
 * @brief PWM modulation implementations for three-phase voltage source inverters
 *
 * L5 Algorithms �� complete PWM modulation techniques:
 *   - Space Vector PWM (SVPWM): maximum voltage utilization (15.5% more than SPWM)
 *   - Sinusoidal PWM (SPWM): simple, low harmonic distortion
 *   - Third-Harmonic Injection: extends linear range with simple math
 *   - Discontinuous PWM (DPWM0/1/2): reduced switching losses
 *   - Overmodulation handling (Mode I and Mode II)
 *
 * SVPWM principle:
 *   A 3-phase VSI has 8 switch states (6 active + 2 zero).
 *   Any reference voltage vector in the alpha-beta plane can be synthesized
 *   by time-averaging two adjacent active vectors plus zero vectors.
 *
 *   Sector determination uses the sign of three test quantities:
 *     Vref1 = Vbeta
 *     Vref2 = sqrt(3)/2 * Valpha - 0.5 * Vbeta
 *     Vref3 = -sqrt(3)/2 * Valpha - 0.5 * Vbeta
 *     sector_code = (Vref1>0)*4 + (Vref2>0)*2 + (Vref3>0)*1
 *
 *   Dwell times (for sector k, active vectors Vk and Vk+1):
 *     T1 = Ts * sqrt(3)*|Vref|/Vdc * sin(k*pi/3 - gamma)
 *     T2 = Ts * sqrt(3)*|Vref|/Vdc * sin(gamma - (k-1)*pi/3)
 *     where gamma is the angle of Vref within the sector.
 *
 * Reference: Holmes & Lipo (2003) Pulse Width Modulation for Power Converters
 *            Van der Broeck et al. (1988) IEEE Trans. Ind. Appl.
 */

#include "pwm_modulation.h"
#include <math.h>

/* ========================================================================
 * SVPWM Sector Lookup Table
 *
 * Maps the 3-bit digital word from sign detection to the geometric
 * sector number (1-6). This is the classic "reverse-Clarke" method.
 * ======================================================================== */

static const uint8_t sector_lookup[8] = {
    0,  /* 0b000 */
    4,  /* 0b001 -> sector 4 */
    6,  /* 0b010 -> sector 6 */
    5,  /* 0b011 -> sector 5 */
    2,  /* 0b100 -> sector 2 */
    3,  /* 0b101 -> sector 3 */
    1,  /* 0b110 -> sector 1 */
    0   /* 0b111 */
};

uint8_t svpwm_determine_sector(float v_alpha, float v_beta)
{
    float vref1 = v_beta;
    float vref2 = M_SQRT3_DIV2 * v_alpha - 0.5f * v_beta;
    float vref3 = -M_SQRT3_DIV2 * v_alpha - 0.5f * v_beta;

    uint8_t code = 0;
    if (vref1 > 0.0f) code |= 0x04;
    if (vref2 > 0.0f) code |= 0x02;
    if (vref3 > 0.0f) code |= 0x01;

    uint8_t sector = sector_lookup[code];
    return (sector >= 1 && sector <= 6) ? sector : 1;
}

/* ========================================================================
 * SVPWM Dwell Time Calculation
 *
 * For the reference vector in sector k, decomposed into adjacent
 * active vectors Vk and Vk+1:
 *
 *   Vref * Ts = Vk * T1 + Vk+1 * T2
 *
 * Solving for T1, T2 using geometry of the hexagon:
 *   T1 = Ts * sqrt(3) * |Vref|/Vdc * sin(k*60�� - gamma)
 *   T2 = Ts * sqrt(3) * |Vref|/Vdc * sin(gamma - (k-1)*60��)
 *
 * where gamma is the angle of Vref measured from the alpha axis.
 * ======================================================================== */

bool svpwm_calculate_timing(float v_alpha, float v_beta, float vdc,
                             float ts, svpwm_timing_t *timing)
{
    if (!timing || vdc <= 0.001f || ts <= 0.0f) return false;

    timing->sector = svpwm_determine_sector(v_alpha, v_beta);
    if (timing->sector < 1 || timing->sector > 6) return false;

    /* Magnitude and angle of reference vector */
    float v_mag = sqrtf(v_alpha*v_alpha + v_beta*v_beta);
    float v_angle = atan2f(v_beta, v_alpha);
    if (v_angle < 0.0f) v_angle += 2.0f * (float)M_PI;

    /* Angle within the sector (0 to 60 degrees) */
    float sector_angle = (float)(timing->sector - 1) * ((float)M_PI / 3.0f);
    float gamma = v_angle - sector_angle;

    /* Normalized dwell times: T1/Ts and T2/Ts */
    float k_factor = M_SQRT3 * v_mag / vdc;  /* sqrt(3) * |Vref| / Vdc */

    float t1_norm = k_factor * sinf(((float)M_PI / 3.0f) - gamma);
    float t2_norm = k_factor * sinf(gamma);

    /* Check for overmodulation (saturation):
       T1 + T2 > Ts means reference exceeds hexagon boundary */
    float t_sum = t1_norm + t2_norm;
    if (t_sum > 1.0f) {
        /* Overmodulation: scale T1 and T2 proportionally to fit within Ts
           while maintaining the correct angle (Mode I overmodulation) */
        float scale = 1.0f / t_sum;
        t1_norm *= scale;
        t2_norm *= scale;
    }

    timing->t1 = t1_norm;
    timing->t2 = t2_norm;
    timing->t0 = 1.0f - t1_norm - t2_norm;
    timing->t7 = timing->t0 * 0.5f;  /* Split evenly for symmetric PWM */

    return true;
}

/* ========================================================================
 * SVPWM Timing to Duty Cycle (7-Segment Symmetric Pattern)
 *
 * Each PWM period is divided into 7 segments:
 *   |--V0--|--Vk--|--Vk+1--|--V7--|--Vk+1--|--Vk--|--V0--|
 *
 * This symmetric pattern minimizes harmonic distortion and places
 * zero vectors at the beginning, middle, and end of each period.
 *
 * The duty cycles are computed by summing the ON times for each phase.
 * For each sector, different active vectors correspond to different
 * switch states. The mapping follows the standard VSI topology:
 *
 *   V1(100): A=1, B=0, C=0  -> ta = T1+T2+T0/2
 *   V2(110): A=1, B=1, C=0
 *   V3(010): A=0, B=1, C=0
 *   V4(011): A=0, B=1, C=1
 *   V5(001): A=0, B=0, C=1
 *   V6(101): A=1, B=0, C=1
 *   V0(000): A=0, B=0, C=0  (all low-side ON)
 *   V7(111): A=1, B=1, C=1  (all high-side ON)
 * ======================================================================== */

void svpwm_timing_to_duty(const svpwm_timing_t *timing, pwm_duty_t *duty)
{
    if (!timing || !duty) return;

    float ta, tb, tc;
    float t1 = timing->t1;
    float t2 = timing->t2;
    float t0_half = timing->t7;   /* T0/2 = T7/2 for symmetric */

    /* Phase duty = fraction of period where upper switch is ON.
       t_aon = T1 + T2 + T0/2 for V1(100), V2(110) sector 1 case */
    switch (timing->sector) {
        case 1:
            ta = t1 + t2 + t0_half;
            tb = t2 + t0_half;
            tc = t0_half;
            break;
        case 2:
            ta = t1 + t0_half;
            tb = t1 + t2 + t0_half;
            tc = t0_half;
            break;
        case 3:
            ta = t0_half;
            tb = t1 + t2 + t0_half;
            tc = t2 + t0_half;
            break;
        case 4:
            ta = t0_half;
            tb = t1 + t0_half;
            tc = t1 + t2 + t0_half;
            break;
        case 5:
            ta = t2 + t0_half;
            tb = t0_half;
            tc = t1 + t2 + t0_half;
            break;
        case 6:
            ta = t1 + t2 + t0_half;
            tb = t0_half;
            tc = t1 + t0_half;
            break;
        default:
            ta = tb = tc = 0.5f;
            break;
    }

    /* Clamp to [0, 1] for safety */
    duty->duty_a = (ta > 1.0f) ? 1.0f : ((ta < 0.0f) ? 0.0f : ta);
    duty->duty_b = (tb > 1.0f) ? 1.0f : ((tb < 0.0f) ? 0.0f : tb);
    duty->duty_c = (tc > 1.0f) ? 1.0f : ((tc < 0.0f) ? 0.0f : tc);
}

bool svpwm_compute(float v_alpha, float v_beta, float vdc, float ts,
                    pwm_duty_t *duty)
{
    svpwm_timing_t timing;
    if (!svpwm_calculate_timing(v_alpha, v_beta, vdc, ts, &timing)) {
        return false;
    }
    svpwm_timing_to_duty(&timing, duty);
    return true;
}

/* ========================================================================
 * L5: Sinusoidal PWM (SPWM)
 *
 * Simple modulation: compare sinusoidal reference to triangular carrier.
 * Phase voltage: v_ref(t) = V_amplitude * cos(theta_e - phase_offset)
 * Duty = 0.5 + v_ref / Vdc  (center-aligned, bipolar modulation)
 *
 * Limitation: Maximum linear phase voltage = Vdc/2 (modulation index = 1.0).
 * SVPWM extends this to Vdc/sqrt(3) (modulation index = 1.155).
 * ======================================================================== */

float sinusoidal_pwm_duty(float v_amplitude, float theta_e, float vdc)
{
    if (vdc <= 0.001f) return 0.5f;
    float v_ref = v_amplitude * cosf(theta_e);
    return voltage_to_duty(v_ref, vdc);
}

void sinusoidal_pwm_three_phase(float v_amplitude, float theta_e, float vdc,
                                 pwm_duty_t *duty)
{
    if (!duty) return;
    float offset = 2.0f * (float)M_PI / 3.0f;

    duty->duty_a = sinusoidal_pwm_duty(v_amplitude, theta_e, vdc);
    duty->duty_b = sinusoidal_pwm_duty(v_amplitude, theta_e - offset, vdc);
    duty->duty_c = sinusoidal_pwm_duty(v_amplitude, theta_e + offset, vdc);
}

/* ========================================================================
 * L5: Third-Harmonic Injection PWM (THIPWM)
 *
 * Adding a third-harmonic common-mode voltage flattens the peak of
 * the fundamental sine wave, allowing 15.47% more fundamental voltage
 * before clipping. This achieves SVPWM-equivalent voltage utilization
 * without the sector-by-sector computation.
 *
 * Common-mode injection: v_cm = -(max(v_a,v_b,v_c) + min(v_a,v_b,v_c)) / 2
 * This is the "min-max" method, mathematically equivalent to SVPWM.
 *
 * Alternatively: v_cm = (1/6) * V_amplitude * sin(3*theta_e)
 * This adds exactly the 3rd harmonic.
 * ======================================================================== */

void third_harmonic_injection(float *a, float *b, float *c)
{
    if (!a || !b || !c) return;

    /* Find min and max of the three phase references */
    float v_min = *a;
    if (*b < v_min) v_min = *b;
    if (*c < v_min) v_min = *c;

    float v_max = *a;
    if (*b > v_max) v_max = *b;
    if (*c > v_max) v_max = *c;

    /* Common-mode offset = -(v_max + v_min) / 2
       This centers the waveforms within [-Vdc/2, +Vdc/2] */
    float v_offset = -0.5f * (v_max + v_min);

    /* Add offset to all three phases */
    *a += v_offset;
    *b += v_offset;
    *c += v_offset;
}

void third_harmonic_pwm_three_phase(float v_amplitude, float theta_e, float vdc,
                                     pwm_duty_t *duty)
{
    if (!duty || vdc <= 0.001f) return;

    float offset = 2.0f * (float)M_PI / 3.0f;
    float a_ref = v_amplitude * cosf(theta_e);
    float b_ref = v_amplitude * cosf(theta_e - offset);
    float c_ref = v_amplitude * cosf(theta_e + offset);

    /* Apply 3rd harmonic injection before voltage-to-duty conversion */
    third_harmonic_injection(&a_ref, &b_ref, &c_ref);

    duty->duty_a = voltage_to_duty(a_ref, vdc);
    duty->duty_b = voltage_to_duty(b_ref, vdc);
    duty->duty_c = voltage_to_duty(c_ref, vdc);
}

/* ========================================================================
 * L5: Discontinuous PWM (DPWM)
 *
 * DPWM clamps one phase to either the positive or negative DC rail for
 * a portion of the fundamental period. This eliminates switching on
 * that phase during the clamped interval, reducing switching losses
 * by up to 33% compared to continuous SVPWM.
 *
 * DPWM0: Clamp to positive rail (duty=1) for 60�� around positive peak
 * DPWM1: Clamp to negative rail (duty=0) for 60�� around negative peak
 * DPWM2: Alternating 30��-30�� clamp, switching between positive and negative
 *
 * Trade-off: DPWM has higher harmonic distortion than SVPWM, especially
 * at low modulation indices.
 * ======================================================================== */

void dpwm0_modulate(float v_amplitude, float theta_e, float vdc,
                     pwm_duty_t *duty)
{
    if (!duty || vdc <= 0.001f) return;
    float offset = 2.0f * (float)M_PI / 3.0f;

    float a_ref = v_amplitude * cosf(theta_e);
    float b_ref = v_amplitude * cosf(theta_e - offset);
    float c_ref = v_amplitude * cosf(theta_e + offset);

    /* DPWM0: offset = Vdc/2 - max(a,b,c) --- clamps to positive rail */
    float v_max = a_ref;
    if (b_ref > v_max) v_max = b_ref;
    if (c_ref > v_max) v_max = c_ref;
    float v_offset = (vdc * 0.5f) - v_max;

    a_ref += v_offset;
    b_ref += v_offset;
    c_ref += v_offset;

    duty->duty_a = voltage_to_duty(a_ref, vdc);
    duty->duty_b = voltage_to_duty(b_ref, vdc);
    duty->duty_c = voltage_to_duty(c_ref, vdc);
}

void dpwm1_modulate(float v_amplitude, float theta_e, float vdc,
                     pwm_duty_t *duty)
{
    if (!duty || vdc <= 0.001f) return;
    float offset = 2.0f * (float)M_PI / 3.0f;

    float a_ref = v_amplitude * cosf(theta_e);
    float b_ref = v_amplitude * cosf(theta_e - offset);
    float c_ref = v_amplitude * cosf(theta_e + offset);

    /* DPWM1: offset = -Vdc/2 - min(a,b,c) --- clamps to negative rail */
    float v_min = a_ref;
    if (b_ref < v_min) v_min = b_ref;
    if (c_ref < v_min) v_min = c_ref;
    float v_offset = -(vdc * 0.5f) - v_min;

    a_ref += v_offset;
    b_ref += v_offset;
    c_ref += v_offset;

    duty->duty_a = voltage_to_duty(a_ref, vdc);
    duty->duty_b = voltage_to_duty(b_ref, vdc);
    duty->duty_c = voltage_to_duty(c_ref, vdc);
}

void dpwm2_modulate(float v_amplitude, float theta_e, float vdc,
                     pwm_duty_t *duty)
{
    if (!duty || vdc <= 0.001f) return;
    float offset = 2.0f * (float)M_PI / 3.0f;

    float a_ref = v_amplitude * cosf(theta_e);
    float b_ref = v_amplitude * cosf(theta_e - offset);
    float c_ref = v_amplitude * cosf(theta_e + offset);

    /* DPWM2: alternating 30�� clamp between + and - rails */
    /* This uses a time-varying offset based on the angle */
    float angle_6th = fmodf(6.0f * theta_e, 2.0f * (float)M_PI);
    bool clamp_pos;
    if (angle_6th < (float)M_PI) {
        clamp_pos = true;  /* First 30 degrees of each 60-degree sector */
    } else {
        clamp_pos = false; /* Second 30 degrees */
    }

    float v_offset;
    if (clamp_pos) {
        float v_max = a_ref;
        if (b_ref > v_max) v_max = b_ref;
        if (c_ref > v_max) v_max = c_ref;
        v_offset = (vdc * 0.5f) - v_max;
    } else {
        float v_min = a_ref;
        if (b_ref < v_min) v_min = b_ref;
        if (c_ref < v_min) v_min = c_ref;
        v_offset = -(vdc * 0.5f) - v_min;
    }

    a_ref += v_offset;
    b_ref += v_offset;
    c_ref += v_offset;

    duty->duty_a = voltage_to_duty(a_ref, vdc);
    duty->duty_b = voltage_to_duty(b_ref, vdc);
    duty->duty_c = voltage_to_duty(c_ref, vdc);
}

/* ========================================================================
 * L5: Overmodulation
 *
 * When the reference voltage magnitude exceeds the linear modulation
 * limit (Vdc/sqrt(3) for SVPWM), the inverter enters overmodulation.
 *
 * Overmodulation Mode I (MI = 0.907 to 0.952):
 *   The reference vector is limited to stay on the hexagon boundary.
 *   The angle is preserved; only the magnitude is reduced.
 *   This introduces low-order harmonics (5th, 7th) but maintains
 *   fundamental voltage increase.
 *
 * Overmodulation Mode II (MI = 0.952 to 1.0):
 *   Six-step operation: the inverter switches once per fundamental
 *   cycle, producing the maximum possible fundamental voltage.
 *   Output is a quasi-square wave with 120�� conduction.
 * ======================================================================== */

void overmodulation_mode1(pwm_duty_t *duty, float mi)
{
    if (!duty) return;
    /* In Mode I overmodulation, when T1+T2 > Ts, we scale both
       proportionally. This is handled in svpwm_calculate_timing().
       Here we apply additional scale if mi > 1.0 for SPWM mode. */
    if (mi <= 1.0f) return;

    /* Beyond MI=1.0 for SPWM, clamp and redistribute */
    float scale = 1.0f / mi;
    duty->duty_a = 0.5f + (duty->duty_a - 0.5f) * scale;
    duty->duty_b = 0.5f + (duty->duty_b - 0.5f) * scale;
    duty->duty_c = 0.5f + (duty->duty_c - 0.5f) * scale;

    /* Clamp */
    if (duty->duty_a > 1.0f) duty->duty_a = 1.0f;
    if (duty->duty_a < 0.0f) duty->duty_a = 0.0f;
    if (duty->duty_b > 1.0f) duty->duty_b = 1.0f;
    if (duty->duty_b < 0.0f) duty->duty_b = 0.0f;
    if (duty->duty_c > 1.0f) duty->duty_c = 1.0f;
    if (duty->duty_c < 0.0f) duty->duty_c = 0.0f;
}

float modulation_index(float v_ref_magnitude, float vdc)
{
    if (vdc <= 0.0f) return 0.0f;
    return v_ref_magnitude / (vdc * 0.5f);
}

float svpwm_max_linear_voltage(float vdc)
{
    return vdc / M_SQRT3;  /* Linear modulation limit for SVPWM */
}

float spwm_max_linear_voltage(float vdc)
{
    return vdc * 0.5f;    /* Linear modulation limit for SPWM */
}
