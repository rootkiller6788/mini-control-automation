/**
 * @file pwm_modulation.h
 * @brief PWM modulation techniques for three-phase inverters
 *
 * Covers L5 Algorithms/Methods & L6 Canonical Problems:
 *   - Space Vector PWM (SVPWM) with sector determination
 *   - Sine-triangle PWM (SPWM)
 *   - Third-harmonic injection PWM
 *   - Discontinuous PWM (DPWM0, DPWM1, DPWM2)
 *   - Overmodulation handling
 *
 * Key concepts:
 *   - Voltage hexagon with 6 active vectors (V1-V6) and 2 zero vectors (V0, V7)
 *   - Sector determination from V_alpha, V_beta
 *   - Dwell time calculation: T1 = Ts * sqrt(3) * |Vref|/Vdc * sin(pi/3 - gamma)
 *                             T2 = Ts * sqrt(3) * |Vref|/Vdc * sin(gamma)
 *   - where gamma = angle within sector
 *
 * Reference: Holmes & Lipo (2003) Pulse Width Modulation for Power Converters
 * Course: MIT 6.334, ETH 227-0530, Berkeley EE218
 */

#ifndef PWM_MODULATION_H
#define PWM_MODULATION_H

#include "motor_types.h"
#include "transforms.h"

/*=============================================================================
 * L5: Space Vector PWM
 *============================================================================*/

/**
 * @brief Determine SVPWM sector from alpha-beta voltage components
 *
 * Algorithm:
 *   v1 = Vbeta
 *   v2 = sqrt(3)/2 * Valpha - 0.5 * Vbeta
 *   v3 = -sqrt(3)/2 * Valpha - 0.5 * Vbeta
 *   sector = (v1>0)*1 + (v2>0)*2 + (v3>0)*4
 *   Maps result 1-6 using lookup table.
 *
 * @param v_alpha  Alpha-axis voltage [V]
 * @param v_beta   Beta-axis voltage [V]
 * @return Sector number 1-6
 */
uint8_t svpwm_determine_sector(float v_alpha, float v_beta);

/**
 * @brief Calculate SVPWM dwell times for active and zero vectors
 *
 * Given the reference voltage vector and sector, compute:
 *   T1 = Ts * (sqrt(3)*Vref/Vdc) * sin(pi/3 * k - gamma)
 *   T2 = Ts * (sqrt(3)*Vref/Vdc) * sin(gamma - pi/3 * (k-1))
 *   T0 = Ts - T1 - T2
 * where k = sector number, gamma = angle of Vref.
 *
 * @param v_alpha   Alpha-axis voltage [V]
 * @param v_beta    Beta-axis voltage [V]
 * @param vdc       DC bus voltage [V]
 * @param ts        Switching period Ts = 1/f_pwm [s]
 * @param timing    Output: sector, T1, T2, T0
 * @return true if within modulation range, false if overmodulation
 */
bool svpwm_calculate_timing(float v_alpha, float v_beta, float vdc,
                             float ts, svpwm_timing_t *timing);

/**
 * @brief Convert SVPWM timing to three-phase duty cycles
 *
 * Seven-segment symmetrical SVPWM pattern:
 *   V0(t0/4) -> Vk(t1/2) -> Vk+1(t2/2) -> V7(t0/2)
 *   -> Vk+1(t2/2) -> Vk(t1/2) -> V0(t0/4)
 *
 * @param timing    SVPWM sector and dwell times
 * @param duty      Output duty cycles for phases A, B, C
 */
void svpwm_timing_to_duty(const svpwm_timing_t *timing, pwm_duty_t *duty);

/**
 * @brief Full SVPWM computation: alpha-beta voltage -> 3-phase duty cycles
 *
 * Combines sector determination, timing calculation, and duty cycle generation.
 * This is THE function called in every FOC PWM update interrupt.
 *
 * @param v_alpha   Alpha-axis voltage reference [V]
 * @param v_beta    Beta-axis voltage reference [V]
 * @param vdc       DC bus voltage [V]
 * @param ts        PWM period [s]
 * @param duty      Output duty cycles
 * @return true if modulation is valid, false if saturated
 */
bool svpwm_compute(float v_alpha, float v_beta, float vdc, float ts,
                    pwm_duty_t *duty);

/*=============================================================================
 * L5: Sinusoidal PWM (SPWM)
 *============================================================================*/

/**
 * @brief Sine-triangle PWM: sinusoidal reference compared with triangular carrier
 *
 * Simple open-loop modulation: duty = 0.5 + 0.5 * (v_phase / (vdc/2)) * cos(angle)
 * Maximum phase voltage (linear): Vdc/2
 * Maximum phase voltage (overmodulation): 2*Vdc/pi
 *
 * @param v_amplitude  Desired phase voltage amplitude [V] (phase-to-neutral peak)
 * @param theta_e      Electrical angle [rad]
 * @param vdc          DC bus voltage [V]
 * @return Duty cycle for one phase [0, 1]
 */
float sinusoidal_pwm_duty(float v_amplitude, float theta_e, float vdc);

/**
 * @brief Generate three-phase sinusoidal PWM duties
 */
void sinusoidal_pwm_three_phase(float v_amplitude, float theta_e, float vdc,
                                 pwm_duty_t *duty);

/*=============================================================================
 * L5: Third-Harmonic Injection PWM
 *============================================================================*/

/**
 * @brief Third-harmonic injection PWM (extends linear range by 15.47%)
 *
 * v_phase += (1/6) * v_amplitude * sin(3 * theta_e)
 * This flattens the peak, allowing higher fundamental amplitude before clipping.
 * Maximum phase voltage (linear): Vdc/sqrt(3) �� 0.577*Vdc
 *
 * Uses: min-max method �� add (max(a,b,c) + min(a,b,c))/2 as common-mode offset.
 */
void third_harmonic_injection(float *a, float *b, float *c);

/**
 * @brief Generate three-phase third-harmonic injected PWM duties
 */
void third_harmonic_pwm_three_phase(float v_amplitude, float theta_e, float vdc,
                                     pwm_duty_t *duty);

/*=============================================================================
 * L5: Discontinuous PWM (DPWM)
 *============================================================================*/

/**
 * @brief DPWM0: clamping to positive DC rail for 60-degree intervals
 *
 * Reduces switching losses by 33% compared to continuous SVPWM.
 * Clamps one phase to Vdc (duty=1.0) during 60�� around the phase voltage peak.
 */
void dpwm0_modulate(float v_amplitude, float theta_e, float vdc,
                     pwm_duty_t *duty);

/**
 * @brief DPWM1: clamping to negative DC rail
 * Clamps one phase to GND (duty=0.0) during 60�� around the phase voltage minimum.
 */
void dpwm1_modulate(float v_amplitude, float theta_e, float vdc,
                     pwm_duty_t *duty);

/**
 * @brief DPWM2: alternating clamp (30�� to each rail)
 * Switches clamp between positive and negative rail every 30 degrees.
 */
void dpwm2_modulate(float v_amplitude, float theta_e, float vdc,
                     pwm_duty_t *duty);

/*=============================================================================
 * L5: Overmodulation
 *============================================================================*/

/**
 * @brief Overmodulation mode I: nonlinear amplitude compensation
 *
 * When Vref > Vdc/sqrt(3), the linear region is exceeded.
 * Mode I (0.577 < MI < 0.907): Maintain angle, reduce amplitude nonlinearly.
 *
 * @param duty      In/out duty cycles (modified if overmodulation detected)
 * @param mi        Modulation index (Vref / (Vdc/2))
 */
void overmodulation_mode1(pwm_duty_t *duty, float mi);

/**
 * @brief Calculate modulation index: MI = |Vref| / (Vdc/2)
 *
 * MI �� 1: Linear modulation (SPWM)
 * 1 < MI �� 1.155: Overmodulation mode I (SVPWM)
 * 1.155 < MI �� 1.273: Overmodulation mode II (six-step)
 */
float modulation_index(float v_ref_magnitude, float vdc);

/**
 * @brief Maximum linear phase voltage for given DC bus (amplitude-invariant)
 * V_phase_max = Vdc / sqrt(3)  (SVPWM linear limit)
 * V_phase_max = Vdc / 2        (SPWM linear limit)
 */
float svpwm_max_linear_voltage(float vdc);
float spwm_max_linear_voltage(float vdc);

/*=============================================================================
 * L5: Advanced SVPWM Features (implemented in svpwm.c)
 *============================================================================*/

/** SVPWM with automatic overmodulation (Mode I + II transition) */
bool svpwm_auto_overmodulation(float v_alpha, float v_beta, float vdc,
                                float ts, pwm_duty_t *duty, float *mi_out);

/** Reconstruct phase currents from DC link shunt resistor samples */
bool current_reconstruct_from_dc_link(uint8_t sector, float idc_sample1,
                                       float idc_sample2,
                                       phase_currents_t *currents);

/** Dead-time compensation for inverter non-linearity */
void dead_time_compensation(pwm_duty_t *duty, const phase_currents_t *currents,
                             float vdc, float t_dead, float t_pwm);

/** DC bus voltage ripple compensation (feed-forward) */
void dc_bus_ripple_compensation(pwm_duty_t *duty,
                                 float vdc_nominal, float vdc_instantaneous);

/** Spread-spectrum PWM frequency dithering for EMI reduction */
float spread_spectrum_pwm_frequency(float base_freq, float spread_percent,
                                     uint32_t *lfsr_state);

/** Clip PWM duties to physically realizable range (min pulse width) */
bool pwm_duty_clip(pwm_duty_t *duty, float min_pulse_width_ratio);

/** Six-step (block) modulation from electrical angle */
void six_step_from_angle(float theta_e, pwm_duty_t *duty);

#endif /* PWM_MODULATION_H */
