/**
 * @file sensorless.h
 * @brief Sensorless rotor position and speed estimation for PMSM/BLDC
 *
 * Covers L5 Algorithms/Methods & L8 Advanced Topics:
 *   - Back-EMF based Sliding Mode Observer (SMO)
 *   - Phase-Locked Loop (PLL) for angle tracking
 *   - Extended Kalman Filter (EKF) for full state estimation
 *   - High-Frequency Injection (HFI) for low-speed operation
 *   - BLDC back-EMF zero-crossing detection
 *   - Flux linkage estimator
 *
 * Key concepts:
 *   - Sliding surface: s = I_hat - I_measured ¡ú 0
 *   - SMO law: dI_hat/dt = A*I_hat + B*(V - Z) where Z = K*sign(s)
 *   - EKF: predict step (model) + update step (measurement)
 *   - HFI: inject high-freq voltage, extract rotor saliency from current response
 *
 * Reference: Vas (1998) Sensorless Vector and Direct Torque Control
 *            Briz & Degner (2018) IEEE Trans. Ind. Appl.
 * Course: MIT 6.450, Stanford EE359, ETH 227-0530
 */

#ifndef SENSORLESS_H
#define SENSORLESS_H

#include "motor_types.h"

/*=============================================================================
 * L5: Sliding Mode Observer (SMO) for Back-EMF Estimation
 *============================================================================*/

/**
 * @brief SMO state variables for sensorless PMSM control
 */
typedef struct {
    /* Estimated currents in alpha-beta frame */
    float i_alpha_hat;  /* Estimated alpha-axis current [A] */
    float i_beta_hat;   /* Estimated beta-axis current [A] */

    /* Back-EMF estimates (output of SMO sign function, then filtered) */
    float e_alpha;      /* Estimated alpha-axis back-EMF [V] */
    float e_beta;       /* Estimated beta-axis back-EMF [V] */

    /* Internal SMO sliding surface */
    float s_alpha;      /* Alpha-axis sliding surface */
    float s_beta;       /* Beta-axis sliding surface */

    /* Low-pass filtered back-EMF */
    float e_alpha_filt; /* Filtered alpha BEMF */
    float e_beta_filt;  /* Filtered beta BEMF */
    float e_alpha_filt_prev; /* Previous filtered for LPF recursion */
    float e_beta_filt_prev;

    /* PLL state for angle/speed extraction */
    float theta_hat;    /* Estimated electrical angle [rad] */
    float omega_hat;    /* Estimated electrical speed [rad/s] */
    float pll_integral; /* PLL integral accumulator */

    /* SMO parameters */
    observer_config_t config;
} smo_state_t;

/**
 * @brief Initialize sliding mode observer state
 *
 * All estimates set to zero, ready for first update.
 */
void smo_init(smo_state_t *state, const observer_config_t *config);

/**
 * @brief Update sliding mode observer with new current/voltage measurements
 *
 * Algorithm:
 *   1. Compute current estimation error: i_tilde = i_hat - i_meas
 *   2. Apply sliding mode control law: Z = K * sign(i_tilde)
 *   3. Update current estimate: d(i_hat)/dt = -Rs/Ls*i_hat + (1/Ls)*(v - Z)
 *   4. Low-pass filter switching signal to get BEMF: e = LPF(Z)
 *   5. Extract angle via PLL: d(theta)/dt = omega; d(omega)/dt = PLL(e_q)
 *
 * Time complexity: O(1) fixed-point compatible
 *
 * @param state        SMO state (maintained between calls)
 * @param i_alpha      Measured alpha-axis current [A]
 * @param i_beta       Measured beta-axis current [A]
 * @param v_alpha      Applied alpha-axis voltage [V]
 * @param v_beta       Applied beta-axis voltage [V]
 * @param dt           Sample time [s]
 * @param params       Motor parameters (Rs, Ls used)
 */
void smo_update(smo_state_t *state, float i_alpha, float i_beta,
                 float v_alpha, float v_beta, float dt,
                 const pmsm_params_t *params);

/**
 * @brief Get estimated electrical angle from SMO
 */
float smo_get_angle(const smo_state_t *state);

/**
 * @brief Get estimated electrical speed from SMO
 */
float smo_get_speed(const smo_state_t *state);

/*=============================================================================
 * L5: Phase-Locked Loop for Angle Tracking
 *============================================================================*/

/**
 * @brief PLL state for tracking electrical angle from BEMF signals
 */
typedef struct {
    float theta_hat;     /* Estimated angle [rad] */
    float omega_hat;     /* Estimated speed [rad/s] */
    float integral;      /* Integral accumulator */
    float kp;            /* Proportional gain */
    float ki;            /* Integral gain */
    float dt_last;       /* Last sample time for normalization */
} pll_state_t;

/**
 * @brief Initialize PLL state
 */
void pll_init(pll_state_t *pll, float kp, float ki);

/**
 * @brief Update PLL with orthogonal BEMF components
 *
 * Error signal: epsilon = -e_alpha*sin(theta_hat) + e_beta*cos(theta_hat)
 * This is equivalent to projecting BEMF onto the estimated q-axis.
 * When aligned: e_d = 0, e_q = omega * psi_m.
 *
 * @param pll        PLL state
 * @param e_alpha    Alpha-axis back-EMF [V]
 * @param e_beta     Beta-axis back-EMF [V]
 * @param dt         Sample time [s]
 */
void pll_update(pll_state_t *pll, float e_alpha, float e_beta, float dt);

/**
 * @brief Get estimated angle from PLL
 */
float pll_get_angle(const pll_state_t *pll);

/**
 * @brief Get estimated speed from PLL
 */
float pll_get_speed(const pll_state_t *pll);

/*=============================================================================
 * L5/L8: Extended Kalman Filter for PMSM State Estimation
 *============================================================================*/

/**
 * @brief EKF state: [Id, Iq, omega_e, theta_e]^T (4-dimensional)
 */
typedef struct {
    float id_hat;       /* Estimated d-axis current */
    float iq_hat;       /* Estimated q-axis current */
    float omega_hat;    /* Estimated electrical speed */
    float theta_hat;    /* Estimated electrical angle */

    /* Covariance matrix P (4x4, stored row-major) */
    float P[16];

    /* Process noise covariance Q (4x4 diagonal) */
    float Q[4];

    /* Measurement noise covariance R (2x2 for Id, Iq measurements) */
    float R[2];

    /* Flag: first iteration (skip update step) */
    bool  initialized;
} ekf_state_t;

/**
 * @brief Initialize extended Kalman filter for PMSM
 *
 * Sets initial state estimate, initial covariance, and noise matrices.
 *
 * @param ekf    EKF state to initialize
 * @param params Motor parameters (used to set reasonable Q matrix)
 */
void ekf_init(ekf_state_t *ekf, const pmsm_params_t *params);

/**
 * @brief EKF prediction step (model-based time update)
 *
 * State transition:
 *   Id(k+1) = Id(k) + dt/Ld * (Vd - Rs*Id + omega*Lq*Iq)
 *   Iq(k+1) = Iq(k) + dt/Lq * (Vq - Rs*Iq - omega*Ld*Id - omega*psi_m)
 *   omega(k+1) = omega(k)
 *   theta(k+1) = theta(k) + dt * omega(k)
 *
 * Covariance prediction: P = F*P*F' + Q  where F is the Jacobian of f(x,u).
 */
void ekf_predict(ekf_state_t *ekf, float vd, float vq, float dt,
                  const pmsm_params_t *params);

/**
 * @brief EKF correction step (measurement update)
 *
 * Measurement: z = [Id_meas, Iq_meas]^T
 * Kalman gain: K = P*H'*(H*P*H' + R)^{-1}
 * State update: x = x + K*(z - h(x))
 * Covariance update: P = (I - K*H)*P
 */
void ekf_update(ekf_state_t *ekf, float id_meas, float iq_meas,
                 const pmsm_params_t *params);

/**
 * @brief Combined EKF predict + update step (run once per control cycle)
 */
void ekf_step(ekf_state_t *ekf, float vd, float vq, float id_meas, float iq_meas,
               float dt, const pmsm_params_t *params);

/*=============================================================================
 * L5: BLDC Back-EMF Zero-Crossing Detection
 *============================================================================*/

/**
 * @brief Detect back-EMF zero crossing in the unenergized phase
 *
 * For six-step BLDC commutation, the unenergized phase BEMF crosses
 * half the DC bus voltage (Vdc/2) at the commutation instant.
 * The zero crossing of (v_unenergized - Vdc/2) triggers commutation.
 *
 * @param v_phase      Unenergized phase voltage [V]
 * @param vdc          DC bus voltage [V]
 * @param v_phase_prev Previous sample of unenergized phase voltage
 * @return true if zero crossing detected, false otherwise
 */
bool bemf_zero_cross_detect(float v_phase, float vdc, float v_phase_prev);

/**
 * @brief Estimate speed from back-EMF zero-crossing period
 *
 * For BLDC with P pole pairs, 6 commutations per electrical cycle:
 *   omega_m = (pi/3) / (P * Tz)  [rad/s]
 * where Tz is the time between two consecutive zero crossings.
 *
 * @param t_between_zc  Time between zero crossings [s]
 * @param pole_pairs    Number of pole pairs
 * @return Estimated mechanical speed [rad/s]
 */
float bemf_speed_from_zc_time(float t_between_zc, float pole_pairs);

/*=============================================================================
 * L8: High-Frequency Injection (HFI)
 *============================================================================*/

/**
 * @brief HFI state for rotor position detection at low/zero speed
 *
 * Injects a high-frequency voltage signal into the d-axis.
 * The resulting current response contains rotor position information
 * in the q-axis due to saliency (Ld != Lq).
 */
typedef struct {
    float v_hf_d;        /* Injected HF d-axis voltage */
    float i_hf_d;        /* HF d-axis current response */
    float i_hf_q;        /* HF q-axis current response (carries saliency info) */
    float theta_hf;      /* HF injection angle [rad] */
    float frequency;     /* HF injection frequency [rad/s] */
    float amplitude;     /* HF injection voltage amplitude [V] */
    bool  enabled;       /* HFI active flag */
} hfi_state_t;

/**
 * @brief Initialize HFI state
 */
void hfi_init(hfi_state_t *hfi, float frequency, float amplitude);

/**
 * @brief Generate HF injection voltage for one control cycle
 *
 * @param hfi      HFI state
 * @param dt       Sample time [s]
 * @return HF d-axis voltage to inject [V]
 */
float hfi_get_injection_voltage(hfi_state_t *hfi, float dt);

/**
 * @brief Extract rotor position error from HF current response
 *
 * For pulsating HF injection in estimated d-axis:
 *   i_hf_q ¡Ø (Ld - Lq) * sin(2 * theta_error)
 * The error is used to drive a tracking observer.
 *
 * Uses heterodyning demodulation with band-pass filtering.
 *
 * @param i_d       Measured d-axis current (contains HF component)
 * @param i_q       Measured q-axis current (contains HF component)
 * @param hfi       HFI state
 * @return Position error signal (0 when aligned)
 */
float hfi_extract_position_error(float i_d, float i_q, const hfi_state_t *hfi);

/*=============================================================================
 * L5: Flux Linkage Estimator
 *============================================================================*/

/**
 * @brief Estimate stator flux linkage from voltage model
 *
 * Psi_s(t) = integral( V_s - Rs * I_s ) dt
 *
 * Used as an alternative to back-EMF integration for sensorless control.
 * Compensates for integration drift with a high-pass filter.
 *
 * @param v_alpha    Alpha-axis voltage [V]
 * @param v_beta     Beta-axis voltage [V]
 * @param i_alpha    Alpha-axis current [A]
 * @param i_beta     Beta-axis current [A]
 * @param rs         Stator resistance [Ohm]
 * @param dt         Integration time step [s]
 * @param psi_alpha  In/out: alpha-axis flux linkage [Wb]
 * @param psi_beta   In/out: beta-axis flux linkage [Wb]
 * @param cutoff_freq High-pass filter cutoff [rad/s] (drift compensation)
 */
void flux_estimator_voltage_model(float v_alpha, float v_beta,
                                   float i_alpha, float i_beta,
                                   float rs, float dt,
                                   float *psi_alpha, float *psi_beta,
                                   float cutoff_freq);

/**
 * @brief Estimate rotor angle from flux linkage
 *
 * theta = atan2(psi_beta, psi_alpha) for surface-mount PMSM.
 * For interior PMSM: subtract Lq*Iq from q-axis flux.
 */
float flux_to_angle(float psi_alpha, float psi_beta);

#endif /* SENSORLESS_H */
