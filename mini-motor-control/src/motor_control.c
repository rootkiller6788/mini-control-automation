/**
 * @file motor_control.c
 * @brief Control algorithm implementations for motor drive systems
 *
 * L5 Algorithms/Methods ¡ª complete implementations:
 *   - PID controller with back-calculation anti-windup
 *   - Cascaded FOC control loops (current / speed / position)
 *   - BLDC six-step hall-sensor commutation
 *   - Trajectory generators (trapezoidal, S-curve)
 *   - Decoupling feed-forward compensation
 *
 * The PID anti-windup method used is "back-calculation":
 * When the output saturates, the integral is reduced by a factor proportional
 * to the amount of saturation (u - u_saturated). This prevents "integral windup"
 * where the integrator accumulates error while the actuator is saturated.
 *
 * Reference:
 *   Astrom & Hagglund (2006) Advanced PID Control
 *   Vas (1998) Sensorless Vector and Direct Torque Control
 */

#include "motor_control.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * L5: PID Controller with Back-Calculation Anti-Windup
 * ========================================================================
 *
 * Standard form: u(t) = Kp*[e(t) + 1/Ti * integral(e)dt + Td * de/dt]
 * Discrete (backward Euler): 
 *   P(n) = Kp * e(n)
 *   I(n) = I(n-1) + Kp*Ki*e(n)*dt    (Ki = 1/Ti in continuous time)
 *   D(n) = Kd * (e(n) - e(n-1))/dt   (with low-pass filter)
 *
 * Anti-windup: If u > u_max, set u = u_max and back-calculate integral
 * to keep it at a level consistent with u_max:
 *   I(n) = I(n) + K_aw * (u_saturated - u_unsaturated) * dt
 * where K_aw = 1/Tt (tracking time constant), typically Tt = sqrt(Ti*Td).
 */

void pid_init(pid_state_t *state, float setpoint)
{
    if (!state) return;
    memset(state, 0, sizeof(pid_state_t));
    state->setpoint = setpoint;
}

void pid_reset_integral(pid_state_t *state)
{
    if (!state) return;
    state->integral = 0.0f;
    state->saturated = false;
}

void pid_set_limits(pid_gains_t *gains, float out_min, float out_max)
{
    if (!gains) return;
    gains->out_min = out_min;
    gains->out_max = out_max;
}

float pid_update(pid_state_t *state, const pid_gains_t *gains, float dt)
{
    if (!state || !gains || dt <= 0.0f) return 0.0f;

    /* 1. Compute error */
    state->error = state->setpoint - state->feedback;

    /* 2. Proportional term */
    state->output_p = gains->kp * state->error;

    /* 3. Integral term with clamping (conditional integration) */
    float integral_inc = gains->ki * state->error * dt;

    /* Anti-windup: skip integration if already saturated and pushing further */
    float tentative_integral = state->integral + integral_inc;
    if (tentative_integral > gains->integral_limit) {
        tentative_integral = gains->integral_limit;
    } else if (tentative_integral < -gains->integral_limit) {
        tentative_integral = -gains->integral_limit;
    }

    /* Back-calculation anti-windup */
    float u_unsat = state->output_p + tentative_integral + state->output_d;


    if (u_unsat > gains->out_max) {
        /* Reduce integral to match output limit */
        float excess = u_unsat - gains->out_max;
        tentative_integral -= excess;

    } else if (u_unsat < gains->out_min) {
        float excess = gains->out_min - u_unsat;
        tentative_integral += excess;

    }

    /* Only integrate if not in deep saturation (conditional integration) */
    state->integral = tentative_integral;
    state->output_i = state->integral;

    /* 4. Derivative term with low-pass filter (filtered derivative) */
    /* Transfer function: D(s) = Kd*s / (1 + s/N) * E(s)
       Discrete: D(n) = (N*Kd*(e(n)-e(n-1)) + D(n-1)) / (1 + N*dt) */
    float raw_deriv = (state->error - state->error_prev) / dt;
    state->derivative_raw = gains->kd * raw_deriv;

    if (gains->n_coeff > 0.0f) {
        float alpha = gains->n_coeff * dt / (1.0f + gains->n_coeff * dt);
        state->derivative = (1.0f - alpha) * state->derivative
                            + alpha * state->derivative_raw;
    } else {
        state->derivative = state->derivative_raw;
    }
    state->output_d = state->derivative;

    /* 5. Total output with final clamping */
    state->output_total = state->output_p + state->output_i + state->output_d;

    if (state->output_total >= gains->out_max) {
        state->output_total = gains->out_max;
        state->saturated = true;
    } else if (state->output_total <= gains->out_min) {
        state->output_total = gains->out_min;
        state->saturated = true;
    } else {
        state->saturated = false;
    }

    /* 6. Store state for next iteration */
    state->error_prev = state->error;

    return state->output_total;
}

/* ========================================================================
 * L5: FOC Cascaded Control Loops
 * ========================================================================
 *
 * Typical FOC cascaded structure (fastest to slowest):
 *   [Position Loop] -> Speed Ref -> [Speed Loop] -> Iq Ref -> [Current Loop] -> Vdq
 *   Sampling: Current @ 16-50 kHz, Speed @ 1-10 kHz, Position @ 100-1000 Hz
 *
 * Decoupling feed-forward is added to the current loop outputs to compensate
 * for cross-coupling between d and q axes caused by frame rotation.
 *
 * Id_ref is normally set to 0 for SPM motors (max torque per Ampere).
 * For IPM motors, MTPA angle adjusts Id to negative for reluctance torque.
 */

static pid_state_t pid_id;     /* d-axis current PI state */
static pid_state_t pid_iq;     /* q-axis current PI state */
static pid_state_t pid_speed;  /* Speed PI state */
static pid_state_t pid_pos;    /* Position P state */

void foc_current_control(float id_ref, float iq_ref, float id_meas, float iq_meas,
                          float omega_e, float *vd_out, float *vq_out,
                          const cascaded_control_cfg_t *cfg,
                          const pmsm_params_t *params, float dt)
{
    if (!cfg || !params) return;

    /* Id control (d-axis: flux regulation) */
    pid_id.setpoint = id_ref;
    pid_id.feedback = id_meas;
    float vd_pi = pid_update(&pid_id, &cfg->current_id_gains, dt);

    /* Iq control (q-axis: torque regulation) */
    pid_iq.setpoint = iq_ref;
    pid_iq.feedback = iq_meas;
    float vq_pi = pid_update(&pid_iq, &cfg->current_iq_gains, dt);

    /* Add decoupling feed-forward */
    float vd_ff, vq_ff;
    pmsm_decoupling_ff(id_meas, iq_meas, omega_e, params, &vd_ff, &vq_ff);

    *vd_out = vd_pi + vd_ff;
    *vq_out = vq_pi + vq_ff;
}

void speed_control(float speed_ref, float speed_meas, float *iq_ref_out,
                    const cascaded_control_cfg_t *cfg, float dt)
{
    if (!cfg) return;
    pid_speed.setpoint = speed_ref;
    pid_speed.feedback = speed_meas;
    float iq_ref = pid_update(&pid_speed, &cfg->speed_gains, dt);

    /* Clamp Iq reference to current limit */
    if (iq_ref > cfg->current_limit) {
        iq_ref = cfg->current_limit;
    } else if (iq_ref < -cfg->current_limit) {
        iq_ref = -cfg->current_limit;
    }
    *iq_ref_out = iq_ref;
}

void position_control(float pos_ref, float pos_meas, float *speed_ref_out,
                       const cascaded_control_cfg_t *cfg, float dt)
{
    if (!cfg) return;
    pid_pos.setpoint = pos_ref;
    pid_pos.feedback = pos_meas;
    *speed_ref_out = pid_update(&pid_pos, &cfg->position_gains, dt);
}

/* ========================================================================
 * L5: Trajectory Generators
 * ========================================================================
 *
 * Trapezoidal speed profile:
 *   Phase 1: Constant acceleration (+a_max) until target speed reached
 *   Phase 2: Constant speed
 *   Phase 3: Constant deceleration (-a_max) to zero
 *
 * S-curve position profile (jerk-limited):
 *   Uses sigmoid smoothing to eliminate infinite jerk at transitions.
 *   s(t) = s0 + (sf - s0) * f(t/T) where f is a 5th-order polynomial
 *   with zero velocity and acceleration at endpoints.
 *
 * The jerk-limited profile reduces mechanical stress and excitation
 * of structural resonances compared to trapezoidal profiles.
 */

void speed_ramp_trapezoidal(float target_speed, float *current_speed,
                             float acceleration, float dt)
{
    if (!current_speed || dt <= 0.0f) return;

    float speed_diff = target_speed - *current_speed;
    float max_step = acceleration * dt;

    if (speed_diff > max_step) {
        *current_speed += max_step;
    } else if (speed_diff < -max_step) {
        *current_speed -= max_step;
    } else {
        *current_speed = target_speed;
    }
}

float scurve_position_profile(float t, float total_time, float target_position)
{
    if (total_time <= 0.0f) return (t >= total_time) ? target_position : 0.0f;
    if (t <= 0.0f) return 0.0f;
    if (t >= total_time) return target_position;

    /* Normalized time [0, 1] */
    float tau = t / total_time;

    /* 5th-order polynomial with C2 continuity at endpoints:
       f(0)=0, f'(0)=0, f''(0)=0, f(1)=1, f'(1)=0, f''(1)=0
       f(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5 */
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;

    float profile = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
    return target_position * profile;
}

/* ========================================================================
 * L5/L6: BLDC Six-Step Commutation
 * ========================================================================
 *
 * Hall sensor decoding table (120-degree sensors):
 *   H3 H2 H1 | Sector | Forward (CW) Switch Pattern
 *   ---------|--------|---------------------------
 *   0  0  1  |   1    | AH=1, BL=1 (A+ B-)
 *   0  1  0  |   2    | AH=1, CL=1 (A+ C-)
 *   0  1  1  |   3    | BH=1, CL=1 (B+ C-)
 *   1  0  0  |   4    | BH=1, AL=1 (B+ A-)
 *   1  0  1  |   5    | CH=1, AL=1 (C+ A-)
 *   1  1  0  |   6    | CH=1, BL=1 (C+ B-)
 *   0  0  0  |   -    | Invalid
 *   1  1  1  |   -    | Invalid
 *
 * Switch index: [0]=AH, [1]=AL, [2]=BH, [3]=BL, [4]=CH, [5]=CL
 * where AH = phase A high-side, AL = phase A low-side, etc.
 */

uint8_t hall_to_sector(uint8_t hall_a, uint8_t hall_b, uint8_t hall_c)
{
    /* Combine 3 hall signals into a 3-bit code */
    uint8_t code = (hall_a << 2) | (hall_b << 1) | hall_c;

    /* Map 3-bit code to sector 1-6 */
    switch (code) {
        case 0x01: return 1;  /* 001 */
        case 0x02: return 2;  /* 010 */
        case 0x03: return 3;  /* 011 */
        case 0x04: return 4;  /* 100 */
        case 0x05: return 5;  /* 101 */
        case 0x06: return 6;  /* 110 */
        default:   return 0;  /* 000 or 111 = invalid */
    }
}

bool bldc_commutation_pattern(uint8_t sector, uint8_t switches[6])
{
    if (!switches || sector < 1 || sector > 6) return false;

    /* Clear all switches */
    for (int i = 0; i < 6; i++) switches[i] = 0;

    /* Six-step pattern: each sector energizes one high-side and one low-side.
       The PWM is applied to the active high-side switch for speed control.
       indices: 0=AH,1=AL,2=BH,3=BL,4=CH,5=CL */
    switch (sector) {
        case 1: switches[0] = 1; switches[3] = 1; break; /* A+ B- */
        case 2: switches[0] = 1; switches[5] = 1; break; /* A+ C- */
        case 3: switches[2] = 1; switches[5] = 1; break; /* B+ C- */
        case 4: switches[2] = 1; switches[1] = 1; break; /* B+ A- */
        case 5: switches[4] = 1; switches[1] = 1; break; /* C+ A- */
        case 6: switches[4] = 1; switches[3] = 1; break; /* C+ B- */
        default: return false;
    }
    return true;
}

uint8_t bldc_next_sector(uint8_t current_sector, motor_direction_t direction)
{
    if (current_sector < 1 || current_sector > 6) return 1;

    if (direction == MOTOR_DIR_CW) {
        return (current_sector % 6) + 1;
    } else {
        return (current_sector == 1) ? 6 : (current_sector - 1);
    }
}

/* ========================================================================
 * L5: Decoupling Feed-Forward Compensation
 * ========================================================================
 *
 * From the PMSM dq voltage equations:
 *   Vd = Rs*Id + Ld*dId/dt - omega_e*Lq*Iq
 *   Vq = Rs*Iq + Lq*dIq/dt + omega_e*Ld*Id + omega_e*psi_m
 *
 * The PI controllers handle the Rs*I + L*dI/dt terms (DC + transient).
 * The cross-coupling terms (-omega_e*Lq*Iq, +omega_e*Ld*Id) and the
 * permanent magnet flux term (+omega_e*psi_m) are compensated via
 * feed-forward to decouple the d and q axis dynamics.
 *
 * With perfect decoupling, the d and q axes become two independent
 * first-order systems: Id(s)/Vd(s) = 1/(Rs + s*Ld), etc.
 */

void pmsm_decoupling_ff(float id, float iq, float omega_e,
                         const pmsm_params_t *params, float *vd_ff, float *vq_ff)
{
    if (!params) return;
    /* Vd_ff = -omega_e * Lq * Iq  (cross-coupling to d-axis) */
    *vd_ff = -omega_e * params->lq * iq;
    /* Vq_ff = omega_e * Ld * Id + omega_e * psi_m  (cross-coupling + PM flux) */
    *vq_ff = omega_e * params->ld * id + omega_e * params->flux_linkage;
}

float dc_motor_voltage_ff(float omega_ref, float t_load,
                           const dc_motor_params_t *params)
{
    if (!params) return 0.0f;
    /* V = Ke*omega + R*TL/Kt  (back-EMF + IR compensation for load torque) */
    return params->ke_back_emf * omega_ref
           + params->winding_resistance * t_load / params->kt_torque;
}
