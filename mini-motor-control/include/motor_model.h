/**
 * @file motor_model.h
 * @brief Mathematical motor models in stationary and rotating reference frames
 *
 * Covers L3 Mathematical Structures & L4 Fundamental Laws:
 *   - DC motor differential equations (electrical + mechanical)
 *   - PMSM dq-axis voltage equations
 *   - Induction motor T-model equations
 *   - Stepper motor electromechanical model
 *
 * Key formulas:
 *   DC: V = R*i + L*di/dt + Ke*omega  (Kirchhoff voltage law + Faraday)
 *   DC: J*domega/dt = Kt*i - B*omega - TL (Newton second law rotation)
 *   PMSM d-axis: Vd = Rs*Id + Ld*dId/dt - omega_e*Lq*Iq
 *   PMSM q-axis: Vq = Rs*Iq + Lq*dIq/dt + omega_e*Ld*Id + omega_e*psi_m
 *   PMSM torque: Te = 1.5*P*(psi_m*Iq + (Ld-Lq)*Id*Iq)
 *   IM: Stator equation and rotor equation in arbitrary reference frame
 *
 * Reference: Krause (2013) Analysis of Electric Machinery
 * Course: MIT 6.685, Berkeley EE117, Purdue ECE 610
 */

#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include "motor_types.h"

/*=============================================================================
 * L3/L4: DC Motor Model
 *============================================================================*/

/**
 * @brief DC motor electrical derivative: di/dt = (V - R*i - Ke*omega) / L
 *
 * Based on Kirchhoff's Voltage Law applied to armature circuit.
 * @param i       Armature current [A]
 * @param omega   Rotor angular velocity [rad/s]
 * @param voltage Terminal voltage [V]
 * @param params  Motor parameters
 * @return di/dt [A/s]
 */
float dc_motor_current_derivative(float i, float omega, float voltage,
                                   const dc_motor_params_t *params);

/**
 * @brief DC motor mechanical derivative: domega/dt = (Kt*i - B*omega - TL) / J
 *
 * Newton's second law for rotation: J*domega/dt = sum of torques.
 * @param i       Armature current [A]
 * @param omega   Rotor angular velocity [rad/s]
 * @param t_load  Load torque [N*m]
 * @param params  Motor parameters
 * @return domega/dt [rad/s^2]
 */
float dc_motor_speed_derivative(float i, float omega, float t_load,
                                 const dc_motor_params_t *params);

/**
 * @brief DC motor steady-state speed: omega = (V - R*i) / Ke
 *
 * From di/dt = 0, solving for omega. No-load current approximated as 0.
 * @param voltage Terminal voltage [V]
 * @param params  Motor parameters
 * @return Steady-state no-load speed [rad/s]
 */
float dc_motor_steady_state_speed(float voltage, const dc_motor_params_t *params);

/**
 * @brief DC motor torque from current: T = Kt * i
 *
 * Lorentz force F = BIL applied to rotor conductors.
 * @param i      Armature current [A]
 * @param params Motor parameters
 * @return Electromagnetic torque [N*m]
 */
float dc_motor_torque_from_current(float i, const dc_motor_params_t *params);

/**
 * @brief DC motor back-EMF: E = Ke * omega
 *
 * Faraday's law of induction: epsilon = -dPhi/dt, linearized.
 * @param omega  Rotor angular velocity [rad/s]
 * @param params Motor parameters
 * @return Back-EMF voltage [V]
 */
float dc_motor_back_emf(float omega, const dc_motor_params_t *params);

/**
 * @brief DC motor mechanical time constant: tau_m = J*R / (Kt*Ke + B*R)
 *
 * Dominant time constant in speed response to voltage step.
 * For small B: tau_m ¡Ö J*R / (Kt*Ke)
 * @param params Motor parameters
 * @return Mechanical time constant [s]
 */
float dc_motor_mechanical_time_constant(const dc_motor_params_t *params);

/**
 * @brief DC motor electrical time constant: tau_e = L / R
 *
 * Governs current response time.
 * @param params Motor parameters
 * @return Electrical time constant [s]
 */
float dc_motor_electrical_time_constant(const dc_motor_params_t *params);

/**
 * @brief DC motor transfer function coefficients
 *
 * omega(s)/V(s) = Kt / [J*L*s^2 + (J*R + B*L)*s + (B*R + Kt*Ke)]
 * Returns struct { a2, a1, a0, b0 } for: b0 / (a2*s^2 + a1*s + a0)
 */
typedef struct {
    float a2;  /* s^2 coefficient: J*L */
    float a1;  /* s^1 coefficient: J*R + B*L */
    float a0;  /* s^0 coefficient: B*R + Kt*Ke */
    float b0;  /* numerator: Kt */
} dc_motor_tf_t;

dc_motor_tf_t dc_motor_transfer_function(const dc_motor_params_t *params);

/**
 * @brief RK4 integration step for DC motor
 *
 * Solves coupled ODEs: di/dt, domega/dt using 4th order Runge-Kutta.
 * @param state    Current state (modified in-place)
 * @param voltage  Applied terminal voltage [V]
 * @param t_load   Load torque [N*m]
 * @param dt       Time step [s]
 * @param params   Motor parameters
 */
void dc_motor_rk4_step(motor_state_t *state, float voltage, float t_load,
                        float dt, const dc_motor_params_t *params);

/*=============================================================================
 * L3/L4: PMSM dq-Axis Model (Synchronous Reference Frame)
 *============================================================================*/

/**
 * @brief PMSM d-axis current derivative: dId/dt = (Vd - Rs*Id + omega_e*Lq*Iq) / Ld
 *
 * From Park-transformed voltage equation (d-axis).
 * @param id        d-axis current [A]
 * @param iq        q-axis current [A]
 * @param vd        d-axis voltage [V]
 * @param omega_e   Electrical angular velocity [rad/s]
 * @param params    Motor parameters
 * @return dId/dt [A/s]
 */
float pmsm_id_derivative(float id, float iq, float vd, float omega_e,
                          const pmsm_params_t *params);

/**
 * @brief PMSM q-axis current derivative: dIq/dt = (Vq - Rs*Iq - omega_e*Ld*Id - omega_e*psi_m) / Lq
 *
 * From Park-transformed voltage equation (q-axis).
 */
float pmsm_iq_derivative(float id, float iq, float vq, float omega_e,
                          const pmsm_params_t *params);

/**
 * @brief PMSM electromagnetic torque: Te = 1.5*P * [psi_m*Iq + (Ld-Lq)*Id*Iq]
 *
 * For surface-mount PMSM (Ld ¡Ö Lq): Te = 1.5*P*psi_m*Iq
 * For interior PMSM (Ld < Lq): reluctance torque term appears.
 */
float pmsm_torque(float id, float iq, const pmsm_params_t *params);

/**
 * @brief PMSM mechanical speed derivative: domega_m/dt = (Te - B*omega_m - TL) / J
 *
 * Newton's second law applied to rotor.
 */
float pmsm_speed_derivative(float id, float iq, float omega_m, float t_load,
                             const pmsm_params_t *params);

/**
 * @brief PMSM maximum torque per ampere (MTPA) d-axis current
 *
 * Id_MTPA = psi_m / (2*(Lq-Ld)) - sqrt( psi_m^2 / (4*(Lq-Ld)^2) + Iq^2 )
 * Optimizes ratio of torque to current magnitude.
 *
 * @param iq        q-axis current [A]
 * @param params    Motor parameters
 * @return Optimal d-axis current for MTPA [A]
 */
float pmsm_mtpa_id(float iq, const pmsm_params_t *params);

/**
 * @brief PMSM field weakening d-axis current limit
 *
 * Id_fw = (psi_m/Ld) - sqrt( (Vdc/sqrt(3)/omega_e)^2 - (Lq*Iq)^2 ) / Ld
 * Ensures voltage does not exceed DC bus limit at high speed.
 *
 * @param iq        q-axis current [A]
 * @param omega_e   Electrical speed [rad/s]
 * @param vdc       DC bus voltage [V]
 * @param params    Motor parameters
 * @return Field-weakening d-axis current [A] (negative)
 */
float pmsm_field_weakening_id(float iq, float omega_e, float vdc,
                               const pmsm_params_t *params);

/**
 * @brief PMSM voltage limit check: Vd^2 + Vq^2 <= (Vdc/sqrt(3))^2
 *
 * Ensures operating point is within voltage hexagon.
 */
bool pmsm_voltage_limit_check(float vd, float vq, float vdc);

/**
 * @brief PMSM RK4 integration step
 */
void pmsm_rk4_step(motor_state_t *state, float vd, float vq,
                    float t_load, float dt, const pmsm_params_t *params);

/*=============================================================================
 * L3/L4: Induction Motor Model (T-equivalent circuit)
 *============================================================================*/

/**
 * @brief Induction motor synchronous speed: ns = 120*f / P (in RPM)
 *                                     or: omega_s = 2*pi*f / P (rad/s electrical)
 */
float im_synchronous_speed(float frequency, float pole_pairs);

/**
 * @brief Induction motor slip: s = (omega_s - omega_r) / omega_s
 */
float im_slip(float omega_sync, float omega_rotor_electrical);

/**
 * @brief Induction motor developed torque (per-phase equivalent):
 *   T = (3*P/2) * (V_th^2 * Rr'/s) / (omega_s * [(R_th + Rr'/s)^2 + (X_th + Xlr')^2])
 *
 * Using Thevenin equivalent of stator at air gap.
 */
float im_torque_per_phase(float v_phase, float freq, float slip,
                           const induction_motor_params_t *params);

/*=============================================================================
 * L3/L4: Stepper Motor Model
 *============================================================================*/

/**
 * @brief Stepper motor full-step angle: theta_step = 360 / steps_per_rev [degrees]
 */
float stepper_step_angle_deg(const stepper_params_t *params);

/**
 * @brief Stepper motor torque vs angle error (detent + holding)
 *   T = T_holding * sin(Nr * theta_error) + T_detent * sin(4*Nr * theta_error)
 * where Nr = steps_per_rev / 4 for 2-phase hybrid stepper.
 */
float stepper_torque_vs_angle(float angle_error_mech, const stepper_params_t *params);

#endif /* MOTOR_MODEL_H */
