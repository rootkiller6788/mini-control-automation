/**
 * @file motor_control.h
 * @brief Control algorithms for motor drives
 *
 * Covers L5 Algorithms/Methods & L6 Canonical Problems:
 *   - PID controller with anti-windup (back-calculation + clamping)
 *   - Cascaded control loop structure (current/speed/position)
 *   - Feed-forward compensation
 *   - Current reference generation (MTPA, field weakening)
 *   - Speed trajectory generation (trapezoidal, S-curve)
 *   - BLDC six-step commutation table
 *   - Hall sensor decoding
 *
 * Reference: Astrom & Hagglund (2006) PID Controllers
 *            Vas (1998) Sensorless Vector and Direct Torque Control
 * Course: MIT 6.302, Stanford EE207 (Feedback Control)
 *         ETH 227-0216, Berkeley EE128 (Mechatronics)
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_types.h"
#include "transforms.h"

/*=============================================================================
 * L5: PID Controller with Anti-Windup
 *============================================================================*/

/**
 * @brief Initialize PID state to safe defaults (all zeros)
 * @param state    Output state structure
 * @param setpoint Initial setpoint value
 */
void pid_init(pid_state_t *state, float setpoint);

/**
 * @brief Update PID controller with back-calculation anti-windup
 *
 * Algorithm:
 *   1. error = setpoint - feedback
 *   2. P_out = Kp * error
 *   3. integral += Ki * error * dt
 *   4. Apply integral clamping: integral = clamp(integral, -limit, +limit)
 *   5. derivative = filter( (error - error_prev)/dt )
 *   6. total = P_out + integral + D_out
 *   7. Apply anti-windup back-calculation if total exceeds limits
 *
 * Time complexity: O(1), Space complexity: O(1)
 *
 * @param state    PID state (maintained between calls)
 * @param gains    PID gains configuration
 * @param dt       Sample time [s]
 * @return Total controller output
 */
float pid_update(pid_state_t *state, const pid_gains_t *gains, float dt);

/**
 * @brief Reset PID integral accumulator (bumpless transfer)
 *
 * Used when switching control modes to avoid integrator windup.
 */
void pid_reset_integral(pid_state_t *state);

/**
 * @brief Set PID output limits dynamically (e.g., current limit changes with DC bus)
 */
void pid_set_limits(pid_gains_t *gains, float out_min, float out_max);

/*=============================================================================
 * L5: Cascaded Control Structure
 *============================================================================*/

/**
 * @brief Execute current (torque) control loop
 *
 * For FOC: takes Id, Iq reference and measured, outputs Vd, Vq.
 * Uses two independent PI controllers for d and q axes.
 * Includes decoupling feed-forward terms.
 *
 * @param id_ref       d-axis current reference [A]
 * @param iq_ref       q-axis current reference [A]
 * @param id_meas      Measured d-axis current [A]
 * @param iq_meas      Measured q-axis current [A]
 * @param omega_e      Electrical angular velocity [rad/s]
 * @param vd_out       Output d-axis voltage reference [V]
 * @param vq_out       Output q-axis voltage reference [V]
 * @param cfg          Current controller gains
 * @param params       Motor parameters (for feed-forward)
 * @param dt           Sample time [s]
 */
void foc_current_control(float id_ref, float iq_ref, float id_meas, float iq_meas,
                          float omega_e, float *vd_out, float *vq_out,
                          const cascaded_control_cfg_t *cfg,
                          const pmsm_params_t *params, float dt);

/**
 * @brief Execute speed control loop
 *
 * Outer loop: PI speed controller generates Iq reference.
 * Inner loop: Current controller (caller must invoke separately).
 *
 * @param speed_ref    Desired mechanical speed [rad/s]
 * @param speed_meas   Measured mechanical speed [rad/s]
 * @param iq_ref_out   Output q-axis current reference [A]
 * @param cfg          Speed controller gains
 * @param dt           Sample time [s]
 */
void speed_control(float speed_ref, float speed_meas, float *iq_ref_out,
                    const cascaded_control_cfg_t *cfg, float dt);

/**
 * @brief Execute position control loop
 *
 * Outer loop: P controller on position generates speed reference.
 * This speed reference then feeds the speed loop.
 *
 * @param pos_ref      Desired mechanical position [rad]
 * @param pos_meas     Measured mechanical position [rad]
 * @param speed_ref_out Output speed reference [rad/s]
 * @param cfg          Position controller gains
 * @param dt           Sample time [s]
 */
void position_control(float pos_ref, float pos_meas, float *speed_ref_out,
                       const cascaded_control_cfg_t *cfg, float dt);

/*=============================================================================
 * L5: Trajectory Generation
 *============================================================================*/

/**
 * @brief Trapezoidal speed profile generator
 *
 * Generates smooth speed reference with acceleration and deceleration ramps.
 * Phases: acceleration -> constant speed -> deceleration.
 *
 * @param target_speed    Desired final speed [rad/s]
 * @param current_speed   Current reference speed (updated in-place) [rad/s]
 * @param acceleration    Acceleration rate [rad/s^2]
 * @param dt              Sample time [s]
 */
void speed_ramp_trapezoidal(float target_speed, float *current_speed,
                             float acceleration, float dt);

/**
 * @brief S-curve position profile generator (jerk-limited)
 *
 * Uses sigmoid-based profile: s(t) = smoothed step.
 * Phase 1: jerk (constant jmax)
 * Phase 2: constant acceleration
 * Phase 3: constant velocity
 * Phase 4: constant deceleration
 * Phase 5: constant negative jerk
 *
 * @param t               Current time [s]
 * @param total_time      Total move time [s]
 * @param target_position Target position [rad]
 * @return Position reference at time t [rad]
 */
float scurve_position_profile(float t, float total_time, float target_position);

/*=============================================================================
 * L5/L6: BLDC Six-Step Commutation
 *============================================================================*/

/**
 * @brief Decode Hall sensor signals to sector (1-6)
 *
 * Hall sensor pattern (120-degree spacing):
 *   H1 H2 H3 | Sector | A+ B+ C+ | A- B- C-
 *   0  0  1  |   1    | 1  0  0  | 0  1  0   (AB)
 *   0  1  0  |   2    | 1  0  0  | 0  0  1   (AC)
 *   ...
 *
 * @param hall_a  Hall sensor A (0 or 1)
 * @param hall_b  Hall sensor B (0 or 1)
 * @param hall_c  Hall sensor C (0 or 1)
 * @return Sector number 1-6, 0 for invalid (000 or 111)
 */
uint8_t hall_to_sector(uint8_t hall_a, uint8_t hall_b, uint8_t hall_c);

/**
 * @brief Get six-step commutation pattern for a given sector
 *
 * Returns on/off states for six switches:
 *   switches[0]=AH, [1]=AL, [2]=BH, [3]=BL, [4]=CH, [5]=CL
 *   1 = ON (high-side PWM or low-side ON), 0 = OFF
 *
 * @param sector    Sector 1-6
 * @param switches  Output switch state array (6 elements)
 * @return true if valid sector, false otherwise
 */
bool bldc_commutation_pattern(uint8_t sector, uint8_t switches[6]);

/**
 * @brief Advance to next commutation sector for given direction
 * @param current_sector Current sector (1-6)
 * @param direction      MOTOR_DIR_CW or MOTOR_DIR_CCW
 * @return Next sector (1-6)
 */
uint8_t bldc_next_sector(uint8_t current_sector, motor_direction_t direction);

/*=============================================================================
 * L5: Feed-Forward Compensation
 *============================================================================*/

/**
 * @brief PMSM dq-axis decoupling feed-forward terms
 *
 * Vd_ff = -omega_e * Lq * Iq
 * Vq_ff =  omega_e * Ld * Id + omega_e * psi_m
 *
 * These compensate for cross-coupling between d and q axes.
 */
void pmsm_decoupling_ff(float id, float iq, float omega_e,
                         const pmsm_params_t *params, float *vd_ff, float *vq_ff);

/**
 * @brief Calculate DC motor voltage feed-forward from desired speed
 *
 * V_ff = Ke * omega_ref + R * TL/Kt  (back-EMF + IR drop under load)
 */
float dc_motor_voltage_ff(float omega_ref, float t_load,
                           const dc_motor_params_t *params);

#endif /* MOTOR_CONTROL_H */
