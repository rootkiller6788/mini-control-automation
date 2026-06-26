/**
 * @file motor_model.c
 * @brief Mathematical motor model implementations
 *
 * L3 Mathematical Structures & L4 Fundamental Laws - physically accurate
 * differential equation models for DC, PMSM, Induction, and Stepper motors.
 *
 * Core physical laws:
 *   L4a: Kirchhoff's Voltage Law �� sum of voltages around any loop = 0
 *   L4b: Faraday's Law �� epsilon = -d(Phi)/dt, linearized as back-EMF
 *   L4c: Lorentz Force �� F = B*I*L, producing torque: T = Kt * I
 *   L4d: Newton's 2nd Law (rotation) �� J * d(omega)/dt = sum(torques)
 *
 * The coupled ODE system for a DC motor:
 *   di/dt   = (V - R*i - Ke*omega) / L           [electrical]
 *   domega/dt = (Kt*i - B*omega - TL) / J        [mechanical]
 *
 * For PMSM in dq-frame (synchronous with rotor flux):
 *   d(Id)/dt  = (Vd - Rs*Id + omega_e*Lq*Iq) / Ld
 *   d(Iq)/dt  = (Vq - Rs*Iq - omega_e*Ld*Id - omega_e*psi_m) / Lq
 *   d(omega_m)/dt = (Te - B*omega_m - TL) / J
 *   d(theta_m)/dt = omega_m
 */

#include "motor_model.h"
#include "transforms.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * L3/L4: DC Motor Model �� Complete Dynamics
 * ========================================================================
 *
 * Equivalent circuit: Ra in series with La, Ke*omega as speed-dependent
 * back-EMF voltage source opposing the applied voltage.
 *
 * Steady-state from di/dt=0:  V = R*i + Ke*omega
 *   => omega = (V - R*i)/Ke  (speed drops linearly with current/torque)
 *
 * Transfer function (omega/V):
 *   G(s) = Kt / [J*L*s^2 + (J*R + B*L)*s + (B*R + Kt*Ke)]
 *
 * Mechanical time constant (dominant pole):
 *   tau_m = J*R / (B*R + Kt*Ke) �� J*R / (Kt*Ke) when friction is small
 */

float dc_motor_current_derivative(float i, float omega, float voltage,
                                   const dc_motor_params_t *params)
{
    if (!params || params->winding_inductance <= 0.0f) return 0.0f;
    /* di/dt = (V - R*i - Ke*omega) / L */
    return (voltage - params->winding_resistance * i
            - params->ke_back_emf * omega) / params->winding_inductance;
}

float dc_motor_speed_derivative(float i, float omega, float t_load,
                                 const dc_motor_params_t *params)
{
    if (!params || params->rotor_inertia <= 0.0f) return 0.0f;
    /* domega/dt = (Kt*i - B*omega - TL) / J */
    float t_em = params->kt_torque * i;
    float t_friction = params->friction_coefficient * omega;
    return (t_em - t_friction - t_load) / params->rotor_inertia;
}

float dc_motor_steady_state_speed(float voltage, const dc_motor_params_t *params)
{
    if (!params || params->ke_back_emf <= 0.0f) return 0.0f;
    /* No-load steady state: I �� 0 (ignoring friction), omega = V/Ke */
    /* With friction: omega = (Kt*V/R - TL) / (Kt*Ke/R + B) */
    float num = params->kt_torque * voltage / params->winding_resistance;
    float den = (params->kt_torque * params->ke_back_emf
                  / params->winding_resistance) + params->friction_coefficient;
    return num / den;
}

float dc_motor_torque_from_current(float i, const dc_motor_params_t *params)
{
    if (!params) return 0.0f;
    return params->kt_torque * i;
}

float dc_motor_back_emf(float omega, const dc_motor_params_t *params)
{
    if (!params) return 0.0f;
    return params->ke_back_emf * omega;
}

float dc_motor_mechanical_time_constant(const dc_motor_params_t *params)
{
    if (!params) return 0.0f;
    float denom = params->friction_coefficient * params->winding_resistance
                  + params->kt_torque * params->ke_back_emf;
    if (denom <= 0.0f) return 0.0f;
    return params->rotor_inertia * params->winding_resistance / denom;
}

float dc_motor_electrical_time_constant(const dc_motor_params_t *params)
{
    if (!params || params->winding_resistance <= 0.0f) return 0.0f;
    return params->winding_inductance / params->winding_resistance;
}

dc_motor_tf_t dc_motor_transfer_function(const dc_motor_params_t *params)
{
    dc_motor_tf_t tf = {0};
    if (!params) return tf;
    float R = params->winding_resistance;
    float L = params->winding_inductance;
    float Kt = params->kt_torque;
    float Ke = params->ke_back_emf;
    float J = params->rotor_inertia;
    float B = params->friction_coefficient;
    tf.a2 = J * L;
    tf.a1 = J * R + B * L;
    tf.a0 = B * R + Kt * Ke;
    tf.b0 = Kt;
    return tf;
}

void dc_motor_rk4_step(motor_state_t *state, float voltage, float t_load,
                        float dt, const dc_motor_params_t *params)
{
    if (!state || !params || dt <= 0.0f) return;

    /* Current state variables */
    float i0 = state->current_dq.q;   /* Repurposed: q = armature current */
    float w0 = state->rotor_speed_mechanical;

    /* --- RK4 k1 (slope at beginning) --- */
    float k1i = dc_motor_current_derivative(i0, w0, voltage, params);
    float k1w = dc_motor_speed_derivative(i0, w0, t_load, params);

    /* --- RK4 k2 (slope at midpoint t+dt/2) --- */
    float i1 = i0 + 0.5f * dt * k1i;
    float w1 = w0 + 0.5f * dt * k1w;
    float k2i = dc_motor_current_derivative(i1, w1, voltage, params);
    float k2w = dc_motor_speed_derivative(i1, w1, t_load, params);

    /* --- RK4 k3 (second midpoint estimate) --- */
    float i2 = i0 + 0.5f * dt * k2i;
    float w2 = w0 + 0.5f * dt * k2w;
    float k3i = dc_motor_current_derivative(i2, w2, voltage, params);
    float k3w = dc_motor_speed_derivative(i2, w2, t_load, params);

    /* --- RK4 k4 (slope at endpoint t+dt) --- */
    float i3 = i0 + dt * k3i;
    float w3 = w0 + dt * k3w;
    float k4i = dc_motor_current_derivative(i3, w3, voltage, params);
    float k4w = dc_motor_speed_derivative(i3, w3, t_load, params);

    /* --- Combine: y(t+dt) = y(t) + dt*(k1+2k2+2k3+k4)/6 --- */
    state->current_dq.q = i0 + dt * (k1i + 2.0f*k2i + 2.0f*k3i + k4i) / 6.0f;
    state->rotor_speed_mechanical = w0 + dt*(k1w + 2.0f*k2w + 2.0f*k3w + k4w)/6.0f;

    /* Update other state variables */
    state->rotor_angle_mechanical += dt * state->rotor_speed_mechanical;
    state->torque_electromagnetic = params->kt_torque * state->current_dq.q;
    state->rotor_speed_electrical = state->rotor_speed_mechanical * params->pole_pairs;
    state->rotor_angle_electrical = mechanical_to_electrical_angle(
        state->rotor_angle_mechanical, params->pole_pairs);
}

/* ========================================================================
 * L3/L4: PMSM dq-Axis Model
 * ========================================================================
 *
 * dq-axis voltage equations (Park-transformed from abc):
 *   Vd = Rs*Id + Ld * dId/dt - omega_e * Lq * Iq
 *   Vq = Rs*Iq + Lq * dIq/dt + omega_e * Ld * Id + omega_e * psi_m
 *
 * The terms -omega_e*Lq*Iq and +omega_e*Ld*Id are "speed EMF" terms
 * representing cross-coupling between axes due to frame rotation.
 *
 * Torque equation (general PMSM):
 *   Te = (3/2) * P * [psi_m*Iq + (Ld - Lq)*Id*Iq]
 *     ^ magnet torque          ^ reluctance torque (salient pole only)
 *
 * For Surface-Mount PMSM (SPM): Ld �� Lq, so Te = (3/2)*P*psi_m*Iq
 * For Interior PMSM (IPM): Ld < Lq, reluctance torque increases at high load.
 */

float pmsm_id_derivative(float id, float iq, float vd, float omega_e,
                          const pmsm_params_t *params)
{
    if (!params || params->ld <= 0.0f) return 0.0f;
    return (vd - params->rs * id + omega_e * params->lq * iq) / params->ld;
}

float pmsm_iq_derivative(float id, float iq, float vq, float omega_e,
                          const pmsm_params_t *params)
{
    if (!params || params->lq <= 0.0f) return 0.0f;
    return (vq - params->rs * iq - omega_e * params->ld * id
            - omega_e * params->flux_linkage) / params->lq;
}

float pmsm_torque(float id, float iq, const pmsm_params_t *params)
{
    if (!params) return 0.0f;
    float magnet_torque = params->flux_linkage * iq;
    float reluctance_torque = (params->ld - params->lq) * id * iq;
    return 1.5f * params->pole_pairs * (magnet_torque + reluctance_torque);
}

float pmsm_speed_derivative(float id, float iq, float omega_m, float t_load,
                             const pmsm_params_t *params)
{
    if (!params || params->rotor_inertia <= 0.0f) return 0.0f;
    float te = pmsm_torque(id, iq, params);
    return (te - params->friction_coefficient * omega_m - t_load)
            / params->rotor_inertia;
}

/* ========================================================================
 * L5: MTPA (Maximum Torque Per Ampere) �� Optimal Current Angle
 * ========================================================================
 *
 * Problem: Given total current limit Is_max, choose Id, Iq to maximize Te.
 * Constraint: Id^2 + Iq^2 <= Is_max^2
 *
 * Solution (for IPM, Ld < Lq):
 *   Id = psi_m / (2*(Lq-Ld)) - sqrt(psi_m^2/(4*(Lq-Ld)^2) + Iq^2)
 *
 * For SPM (Ld = Lq): Id = 0, Iq = Is (pure q-axis current)
 * MTPA angle gamma = atan2(-Id, Iq) �� typically 0-45 degrees for IPM.
 */

float pmsm_mtpa_id(float iq, const pmsm_params_t *params)
{
    if (!params) return 0.0f;
    float delta_L = params->lq - params->ld;
    if (fabsf(delta_L) < 1e-9f) {
        /* Surface-mount PMSM: MTPA is simply Id = 0 */
        return 0.0f;
    }
    float psi_over_delta = params->flux_linkage / (2.0f * delta_L);
    float discriminant = psi_over_delta * psi_over_delta + iq * iq;
    return psi_over_delta - sqrtf(discriminant);
}

/* ========================================================================
 * L5: Field Weakening �� High-Speed Operation
 * ========================================================================
 *
 * As speed increases, back-EMF approaches DC bus limit.
 * Field weakening reduces d-axis flux (psi_m + Ld*Id) by applying
 * negative Id, which opposes the PM flux and reduces Vq requirement.
 *
 * Voltage constraint: Vd^2 + Vq^2 <= Vmax^2 where Vmax = Vdc/sqrt(3)
 * At high speed, resistive drop is negligible:
 *   (omega_e*Lq*Iq)^2 + (omega_e*(Ld*Id+psi_m))^2 <= Vmax^2
 *
 * Solving for Id (steady-state, R neglected):
 *   Id = -psi_m/Ld + sqrt(Vmax^2/(omega_e^2) - (Lq*Iq)^2) / Ld
 *
 * Negative Id reduces net flux -> allows higher speed at same Vdc.
 */

float pmsm_field_weakening_id(float iq, float omega_e, float vdc,
                               const pmsm_params_t *params)
{
    if (!params) return 0.0f;
    if (omega_e < 1.0f) return 0.0f;  /* No FW at very low speed */

    float vmax = vdc / M_SQRT3;  /* Maximum phase voltage (amplitude invariance) */
    float vmax_over_omega = vmax / omega_e;
    float lq_iq_sq = params->lq * params->lq * iq * iq;

    float inner = vmax_over_omega * vmax_over_omega - lq_iq_sq;
    if (inner < 0.0f) inner = 0.0f;

    float id = -(params->flux_linkage / params->ld)
               + sqrtf(inner) / params->ld;
    /* Field weakening Id should be negative */
    if (id > 0.0f) id = 0.0f;
    return id;
}

bool pmsm_voltage_limit_check(float vd, float vq, float vdc)
{
    float vmax = vdc / M_SQRT3;
    return (vd * vd + vq * vq) <= (vmax * vmax);
}

/* ========================================================================
 * L3/L4: PMSM RK4 Integration Step
 * ========================================================================
 *
 * Integrates 4 coupled ODEs:
 *   dId/dt, dIq/dt, d(omega_m)/dt, d(theta_m)/dt
 * using classical 4th-order Runge-Kutta method (RK4).
 *
 * RK4 provides O(h^4) local truncation error, suitable for
 * motor simulation with time steps up to 100 us.
 */

void pmsm_rk4_step(motor_state_t *state, float vd, float vq,
                    float t_load, float dt, const pmsm_params_t *params)
{
    if (!state || !params || dt <= 0.0f) return;

    float id0 = state->current_dq.d;
    float iq0 = state->current_dq.q;
    float w0  = state->rotor_speed_mechanical;
    float we  = state->rotor_speed_electrical;

    /* --- k1 --- */
    float k1d = pmsm_id_derivative(id0, iq0, vd, we, params);
    float k1q = pmsm_iq_derivative(id0, iq0, vq, we, params);
    float k1w = pmsm_speed_derivative(id0, iq0, w0, t_load, params);
    float k1t = w0;

    /* --- k2 --- */
    float id1 = id0 + 0.5f*dt*k1d;
    float iq1 = iq0 + 0.5f*dt*k1q;
    float w1  = w0  + 0.5f*dt*k1w;
    float we1 = w1 * params->pole_pairs;
    float k2d = pmsm_id_derivative(id1, iq1, vd, we1, params);
    float k2q = pmsm_iq_derivative(id1, iq1, vq, we1, params);
    float k2w = pmsm_speed_derivative(id1, iq1, w1, t_load, params);
    float k2t = w1;

    /* --- k3 --- */
    float id2 = id0 + 0.5f*dt*k2d;
    float iq2 = iq0 + 0.5f*dt*k2q;
    float w2  = w0  + 0.5f*dt*k2w;
    float we2 = w2 * params->pole_pairs;
    float k3d = pmsm_id_derivative(id2, iq2, vd, we2, params);
    float k3q = pmsm_iq_derivative(id2, iq2, vq, we2, params);
    float k3w = pmsm_speed_derivative(id2, iq2, w2, t_load, params);
    float k3t = w2;

    /* --- k4 --- */
    float id3 = id0 + dt*k3d;
    float iq3 = iq0 + dt*k3q;
    float w3  = w0  + dt*k3w;
    float we3 = w3 * params->pole_pairs;
    float k4d = pmsm_id_derivative(id3, iq3, vd, we3, params);
    float k4q = pmsm_iq_derivative(id3, iq3, vq, we3, params);
    float k4w = pmsm_speed_derivative(id3, iq3, w3, t_load, params);
    float k4t = w3;

    /* --- Final combination --- */
    state->current_dq.d = id0 + dt*(k1d + 2.0f*k2d + 2.0f*k3d + k4d)/6.0f;
    state->current_dq.q = iq0 + dt*(k1q + 2.0f*k2q + 2.0f*k3q + k4q)/6.0f;
    state->rotor_speed_mechanical = w0 + dt*(k1w + 2.0f*k2w + 2.0f*k3w + k4w)/6.0f;
    state->rotor_angle_mechanical += dt*(k1t + 2.0f*k2t + 2.0f*k3t + k4t)/6.0f;

    /* Update derived quantities */
    state->rotor_speed_electrical = state->rotor_speed_mechanical
                                     * params->pole_pairs;
    state->rotor_angle_electrical = mechanical_to_electrical_angle(
        state->rotor_angle_mechanical, params->pole_pairs);
    state->torque_electromagnetic = pmsm_torque(state->current_dq.d,
                                                 state->current_dq.q, params);
    state->voltage_dq_ref.d = vd;
    state->voltage_dq_ref.q = vq;
}

/* ========================================================================
 * L3/L4: Induction Motor T-Model
 * ========================================================================
 *
 * Synchronous speed: ns = 120*f / P [RPM], or omega_s = 4*pi*f / P [rad/s elec]
 * Slip: s = (omega_s - omega_r) / omega_s
 *     s = 0: synchronous (no torque)
 *     s = 1: rotor at standstill (locked rotor)
 *     s < 0: generating
 *
 * Per-phase torque (simplified Thevenin equivalent at stator-airgap):
 *   T_phase = (V_th^2 / omega_s) * (Rr'/s) / [(R_th + Rr'/s)^2 + (X_th + Xlr')^2]
 *
 * Total torque = 3 * P * T_phase / 2
 */

float im_synchronous_speed(float frequency, float pole_pairs)
{
    if (pole_pairs <= 0.0f) return 0.0f;
    return 2.0f * (float)M_PI * frequency / pole_pairs;  /* rad/s electrical */
}

float im_slip(float omega_sync, float omega_rotor_electrical)
{
    if (fabsf(omega_sync) < 1e-6f) return 0.0f;
    return (omega_sync - omega_rotor_electrical) / omega_sync;
}

float im_torque_per_phase(float v_phase, float freq, float slip,
                           const induction_motor_params_t *params)
{
    if (!params || freq <= 0.0f) return 0.0f;

    float omega_s = 2.0f * (float)M_PI * freq;
    float xm = omega_s * params->lm;
    float xls = omega_s * params->lls;
    float xlr = omega_s * params->llr;

    /* Thevenin equivalent at air gap */
    if (fabsf(slip) < 1e-9f) return 0.0f;  /* synchronous: no torque */
    float rr_s = params->rr / slip;

    /* Thevenin voltage: V_th = Vph * Xm / sqrt(Rs^2 + (Xls+Xm)^2) */
    float den_th_sq = params->rs*params->rs + (xls + xm)*(xls + xm);
    float v_th = v_phase * xm / sqrtf(den_th_sq);

    /* Thevenin impedance: R_th + jX_th */
    float r_th = params->rs * xm * xm / den_th_sq;
    float x_th_num = xm * (params->rs*params->rs + xls*(xls + xm));
    float x_th = x_th_num / den_th_sq;

    /* Torque per phase */
    float num = v_th * v_th * rr_s;
    float den = omega_s * ((r_th + rr_s)*(r_th + rr_s) + (x_th + xlr)*(x_th + xlr));
    return num / den;
}

/* ========================================================================
 * L3/L4: Stepper Motor Model
 * ========================================================================
 *
 * Full-step angle: 360 / N_steps  (200 steps/rev -> 1.8 degrees)
 * Torque-displacement relation for 2-phase hybrid stepper:
 *   T(theta) = T_holding * sin(Nr * theta_error)
 *            + T_detent * sin(4*Nr * theta_error)
 * where Nr = steps_per_rev / 4 (number of rotor teeth per revolution).
 *
 * The first harmonic is the main holding torque (aligns rotor to stator).
 * The 4th harmonic is the detent (cogging) torque from slotting effects.
 */

float stepper_step_angle_deg(const stepper_params_t *params)
{
    if (!params || params->steps_per_rev <= 0) return 0.0f;
    return 360.0f / (float)params->steps_per_rev;
}

float stepper_torque_vs_angle(float angle_error_mech, const stepper_params_t *params)
{
    if (!params || params->steps_per_rev <= 0) return 0.0f;
    float Nr = (float)params->steps_per_rev / 4.0f; /* rotor tooth count */
    float main_torque = params->holding_torque * sinf(Nr * angle_error_mech);
    float detent_torque = params->detent_torque * sinf(4.0f * Nr * angle_error_mech);
    return main_torque + detent_torque;
}
