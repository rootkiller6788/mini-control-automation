/**
 * @file sensorless.c
 * @brief Sensorless rotor position and speed estimation
 *
 * L5 Algorithms + L8 Advanced Topics:
 *   - Sliding Mode Observer (SMO) for back-EMF estimation
 *   - Phase-Locked Loop (PLL) angle tracking
 *   - Extended Kalman Filter (EKF) full state estimation
 *   - High-Frequency Injection (HFI) for zero/low-speed
 *   - Back-EMF zero-crossing for BLDC commutation
 *   - Flux linkage voltage-model estimator
 *
 * Core SMO equations (alpha-beta stationary frame):
 *   d(i_hat)/dt = -(Rs/Ls)*i_hat + (1/Ls)*v - (K/Ls)*sign(i_hat - i)
 *   Back-EMF: e = K * sign(i_hat - i)  [discontinuous signal]
 *   Filtered:  e_filt = LPF(e)  [continuous back-EMF estimate]
 *
 * The sliding surface is s = i_hat - i = 0.
 * When the system reaches the sliding surface (s=0, ds/dt=0),
 * the equivalent control signal equals the actual back-EMF.
 *
 * EKF state vector: x = [Id, Iq, omega_e, theta_e]^T
 * The EKF provides optimal state estimation with explicit handling
 * of measurement and process noise, but at higher computational cost
 * than SMO+PLL.
 *
 * Reference:
 *   Utkin et al. (2009) Sliding Mode Control in Electro-Mechanical Systems
 *   Bolognani et al. (2003) IEEE Trans. Ind. Appl. (EKF for PMSM)
 *   Corley & Lorenz (1998) IEEE Trans. Ind. Appl. (HFI)
 */

#include "sensorless.h"
#include "transforms.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * L5: Sliding Mode Observer (SMO) initialization
 * ======================================================================== */

void smo_init(smo_state_t *state, const observer_config_t *config)
{
    if (!state || !config) return;
    memset(state, 0, sizeof(smo_state_t));
    state->config = *config;
}

/* ========================================================================
 * L5: SMO Update Step
 *
 * Equation in alpha-beta frame (continuous-time):
 *   d(i_alpha_hat)/dt = -(Rs/Ls)*i_alpha_hat + (1/Ls)*v_alpha - (K/Ls)*sign(s_alpha)
 *   d(i_beta_hat)/dt  = -(Rs/Ls)*i_beta_hat  + (1/Ls)*v_beta  - (K/Ls)*sign(s_beta)
 *
 * where s = i_hat - i_meas is the sliding surface.
 *
 * Discrete-time (forward Euler):
 *   i_alpha_hat(k+1) = (1 - Rs*Ts/Ls)*i_alpha_hat(k)
 *                     + (Ts/Ls)*v_alpha(k) - (K*Ts/Ls)*sign(s_alpha(k))
 *
 * Back-EMF extraction:
 *   z = K * sign(s)  [switching signal contains BEMF + noise]
 *   e = LPF(z)       [filter out switching frequency to get smooth BEMF]
 *
 * Sign function implemented with boundary layer to reduce chattering:
 *   sign(x) �� x / (|x| + delta)  (saturation function)
 * or:
 *   sign(x) = 1 if x > +eps, -1 if x < -eps, x/eps otherwise
 * ======================================================================== */

static float smooth_sign(float x, float eps)
{
    if (x > eps) return 1.0f;
    if (x < -eps) return -1.0f;
    return x / eps;  /* Linear region inside boundary layer */
}

void smo_update(smo_state_t *state, float i_alpha, float i_beta,
                 float v_alpha, float v_beta, float dt,
                 const pmsm_params_t *params)
{
    if (!state || !params || dt <= 0.0f) return;

    float K  = state->config.gain_k;
    float Rs = params->rs;
    float Ls = params->ls;  /* Use average inductance */

    if (Ls <= 0.0f) return;

    float dt_over_L = dt / Ls;
    float rs_factor = 1.0f - Rs * dt_over_L;
    float k_factor  = K * dt_over_L;

    /* --- Current estimation error (sliding surface) --- */
    state->s_alpha = state->i_alpha_hat - i_alpha;
    state->s_beta  = state->i_beta_hat  - i_beta;

    /* --- Sliding mode control law (switching function) --- */
    float z_alpha = K * smooth_sign(state->s_alpha, 0.01f);
    float z_beta  = K * smooth_sign(state->s_beta,  0.01f);

    /* --- Update current estimates (forward Euler) --- */
    state->i_alpha_hat = rs_factor * state->i_alpha_hat
                         + dt_over_L * v_alpha - k_factor * smooth_sign(state->s_alpha, 0.01f);
    state->i_beta_hat  = rs_factor * state->i_beta_hat
                         + dt_over_L * v_beta  - k_factor * smooth_sign(state->s_beta,  0.01f);

    /* --- Back-EMF: low-pass filter the switching signal --- */
    float cutoff = state->config.filter_cutoff;
    float alpha_lpf;
    if (cutoff > 0.0f && dt > 0.0f) {
        alpha_lpf = cutoff * dt / (1.0f + cutoff * dt);
    } else {
        alpha_lpf = 1.0f;  /* No filtering */
    }

    state->e_alpha = (1.0f - alpha_lpf) * state->e_alpha + alpha_lpf * z_alpha;
    state->e_beta  = (1.0f - alpha_lpf) * state->e_beta  + alpha_lpf * z_beta;

    /* Store filtered BEMF for PLL */
    state->e_alpha_filt_prev = state->e_alpha_filt;
    state->e_beta_filt_prev  = state->e_beta_filt;
    state->e_alpha_filt = state->e_alpha;
    state->e_beta_filt  = state->e_beta;

    /* --- PLL-based angle extraction from BEMF --- */
    float ke_est = state->e_alpha * cosf(state->theta_hat)
                   + state->e_beta * sinf(state->theta_hat);  /* Projection onto d-axis */

    float kp = state->config.pll_kp;
    float ki = state->config.pll_ki;

    /* PLL error: BEMF d-axis component should be zero when aligned */
    float pll_error = ke_est;

    state->pll_integral += ki * pll_error * dt;
    if (state->pll_integral > 10000.0f) state->pll_integral = 10000.0f;
    if (state->pll_integral < -10000.0f) state->pll_integral = -10000.0f;

    state->omega_hat = kp * pll_error + state->pll_integral;
    state->theta_hat += state->omega_hat * dt;

    /* Normalize angle */
    state->theta_hat = normalize_angle_positive(state->theta_hat);
}

float smo_get_angle(const smo_state_t *state)
{
    return state ? state->theta_hat : 0.0f;
}

float smo_get_speed(const smo_state_t *state)
{
    return state ? state->omega_hat : 0.0f;
}

/* ========================================================================
 * L5: Phase-Locked Loop (PLL) for Angle Tracking
 *
 * PLL extracts rotor angle and speed from orthogonal sine/cosine
 * signals (back-EMF or flux estimates). The PLL locks the estimated
 * angle to the measured BEMF vector.
 *
 * Given BEMF components e_alpha and e_beta (sinusoidal at rotor frequency):
 *   e_alpha = -omega_e * psi_m * sin(theta_e)
 *   e_beta  =  omega_e * psi_m * cos(theta_e)
 *
 * PLL error: epsilon = -e_alpha*sin(theta_hat) + e_beta*cos(theta_hat)
 *                    = omega_e*psi_m * [sin(theta_e)*sin(theta_hat) + cos(theta_e)*cos(theta_hat)]
 *                    = omega_e*psi_m * cos(theta_e - theta_hat)
 *                    �� omega_e*psi_m * (theta_e - theta_hat)  for small errors
 *
 * The PI controller drives epsilon to zero, locking theta_hat to theta_e.
 * ======================================================================== */

void pll_init(pll_state_t *pll, float kp, float ki)
{
    if (!pll) return;
    memset(pll, 0, sizeof(pll_state_t));
    pll->kp = kp;
    pll->ki = ki;
}

void pll_update(pll_state_t *pll, float e_alpha, float e_beta, float dt)
{
    if (!pll || dt <= 0.0f) return;

    /* PLL error: cross-product of BEMF and estimated angle vector */
    float sin_hat = sinf(pll->theta_hat);
    float cos_hat = cosf(pll->theta_hat);

    /* epsilon = -e_alpha*sin(theta_hat) + e_beta*cos(theta_hat) */
    float epsilon = -e_alpha * sin_hat + e_beta * cos_hat;

    /* Normalize by BEMF magnitude to make gain independent of speed */
    float bemf_mag = sqrtf(e_alpha*e_alpha + e_beta*e_beta);
    if (bemf_mag > 0.01f) {
        epsilon /= bemf_mag;
    }

    /* PI controller */
    pll->integral += pll->ki * epsilon * dt;
    float omega_est = pll->kp * epsilon + pll->integral;
    pll->omega_hat = omega_est;
    pll->theta_hat += omega_est * dt;

    /* Wrap angle */
    pll->theta_hat = normalize_angle_positive(pll->theta_hat);
    pll->dt_last = dt;
}

float pll_get_angle(const pll_state_t *pll)
{
    return pll ? pll->theta_hat : 0.0f;
}

float pll_get_speed(const pll_state_t *pll)
{
    return pll ? pll->omega_hat : 0.0f;
}

/* ========================================================================
 * L8: Extended Kalman Filter (EKF) for PMSM
 *
 * State vector: x = [Id, Iq, omega_e, theta_e]^T
 * Input vector: u = [Vd, Vq]^T
 * Measurement:  z = [Id_meas, Iq_meas]^T
 *
 * Process model (nonlinear):
 *   f1: Id(k+1) = Id(k) + dt*(Vd - Rs*Id + omega_e*Lq*Iq)/Ld
 *   f2: Iq(k+1) = Iq(k) + dt*(Vq - Rs*Iq - omega_e*Ld*Id - omega_e*psi_m)/Lq
 *   f3: omega_e(k+1) = omega_e(k)  (assumed constant over one sample)
 *   f4: theta_e(k+1) = theta_e(k) + dt*omega_e(k)
 *
 * Jacobian F = df/dx:
 *   [1-dt*Rs/Ld,  dt*omega_e*Lq/Ld,      dt*Lq*Iq/Ld,   0]
 *   [-dt*omega_e*Ld/Lq, 1-dt*Rs/Lq,  -dt*(Ld*Id+psi_m)/Lq,  0]
 *   [0,              0,                1,                   0]
 *   [0,              0,               dt,                   1]
 *
 * Measurement matrix H = dh/dx (linear):
 *   [1, 0, 0, 0]
 *   [0, 1, 0, 0]
 *
 * Standard EKF recursion:
 *   Predict: x_pred = f(x, u)
 *            P_pred = F * P * F^T + Q
 *   Update:  K = P_pred * H^T * (H * P_pred * H^T + R)^{-1}
 *            x_new = x_pred + K * (z - H*x_pred)
 *            P_new = (I - K*H) * P_pred
 * ======================================================================== */

void ekf_init(ekf_state_t *ekf, const pmsm_params_t *params)
{
    if (!ekf || !params) return;
    memset(ekf, 0, sizeof(ekf_state_t));

    /* Initialize covariance matrix as diagonal with initial uncertainty */
    for (int i = 0; i < 4; i++) {
        ekf->P[i*4 + i] = 1.0f;
    }
    /* Higher uncertainty on theta (unknown initial position) */
    ekf->P[15] = 10.0f;

    /* Process noise (diagonal Q) */
    ekf->Q[0] = 0.01f;   /* Id process noise */
    ekf->Q[1] = 0.01f;   /* Iq process noise */
    ekf->Q[2] = 100.0f;  /* omega process noise (speed can change) */
    ekf->Q[3] = 0.001f;  /* theta process noise (deterministic) */

    /* Measurement noise */
    ekf->R[0] = 0.1f;   /* Id measurement variance */
    ekf->R[1] = 0.1f;   /* Iq measurement variance */

    ekf->initialized = true;
}

void ekf_predict(ekf_state_t *ekf, float vd, float vq, float dt,
                  const pmsm_params_t *params)
{
    if (!ekf || !params || dt <= 0.0f) return;

    float id  = ekf->id_hat;
    float iq  = ekf->iq_hat;
    float we  = ekf->omega_hat;
    float th  = ekf->theta_hat;
    float Rs  = params->rs;
    float Ld  = params->ld;
    float Lq  = params->lq;
    float psi = params->flux_linkage;

    /* ---- Predict state ---- */
    if (Ld > 0.0f) {
        ekf->id_hat = id + dt * (vd - Rs*id + we*Lq*iq) / Ld;
    }
    if (Lq > 0.0f) {
        ekf->iq_hat = iq + dt * (vq - Rs*iq - we*Ld*id - we*psi) / Lq;
    }
    /* Assume mechanical speed is approximately constant over one dt */
    /* ekf->omega_hat = we;  (no change in prediction) */
    ekf->theta_hat = normalize_angle_positive(th + dt * we);

    /* ---- Compute Jacobian F (4x4) ---- */
    float F[16] = {0};

    /* Row 0: df1/dx */
    F[0] = (Ld > 0.0f) ? (1.0f - dt * Rs / Ld) : 1.0f;
    F[1] = (Ld > 0.0f) ? (dt * we * Lq / Ld) : 0.0f;
    F[2] = (Ld > 0.0f) ? (dt * Lq * iq / Ld) : 0.0f;
    F[3] = 0.0f;

    /* Row 1: df2/dx */
    F[4] = (Lq > 0.0f) ? (-dt * we * Ld / Lq) : 0.0f;
    F[5] = (Lq > 0.0f) ? (1.0f - dt * Rs / Lq) : 1.0f;
    F[6] = (Lq > 0.0f) ? (-dt * (Ld*id + psi) / Lq) : 0.0f;
    F[7] = 0.0f;

    /* Row 2: df3/dx (omega is modeled as constant) */
    F[8] = 0.0f;
    F[9] = 0.0f;
    F[10] = 1.0f;
    F[11] = 0.0f;

    /* Row 3: df4/dx */
    F[12] = 0.0f;
    F[13] = 0.0f;
    F[14] = dt;
    F[15] = 1.0f;

    /* ---- P_pred = F * P * F^T + Q ---- */
    float FP[16] = {0};
    /* FP = F * P (4x4 matrix multiplication) */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += F[i*4 + k] * ekf->P[k*4 + j];
            }
            FP[i*4 + j] = sum;
        }
    }

    /* P_pred = FP * F^T + Q */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += FP[i*4 + k] * F[j*4 + k];  /* F^T(j,k) = F(k,j) */
            }
            ekf->P[i*4 + j] = sum + ((i == j) ? ekf->Q[i] : 0.0f);
        }
    }
}

void ekf_update(ekf_state_t *ekf, float id_meas, float iq_meas,
                 const pmsm_params_t *params)
{
    if (!ekf || !params) return;

    /* ---- Measurement residual ---- */
    float y[2];
    y[0] = id_meas - ekf->id_hat;
    y[1] = iq_meas - ekf->iq_hat;

    /* H = [[1,0,0,0], [0,1,0,0]] */
    /* S = H * P * H^T + R (2x2) */
    float S[4];
    S[0] = ekf->P[0] + ekf->R[0];  /* S11 = P11 + R1 */
    S[1] = ekf->P[1];               /* S12 = P12 */
    S[2] = ekf->P[4];               /* S21 = P21 */
    S[3] = ekf->P[5] + ekf->R[1];  /* S22 = P22 + R2 */

    /* Inverse of 2x2 matrix S: S^{-1} = [S22, -S12; -S21, S11] / det */
    float det = S[0]*S[3] - S[1]*S[2];
    if (fabsf(det) < 1e-10f) return;  /* singular, skip update */

    float inv_det = 1.0f / det;
    float Sinv[4];
    Sinv[0] =  S[3] * inv_det;
    Sinv[1] = -S[1] * inv_det;
    Sinv[2] = -S[2] * inv_det;
    Sinv[3] =  S[0] * inv_det;

    /* Kalman gain K = P * H^T * S^{-1} (4x2) */
    /* P*H^T is just the first two columns of P */
    float K[8];
    for (int i = 0; i < 4; i++) {
        K[i*2 + 0] = ekf->P[i*4 + 0] * Sinv[0] + ekf->P[i*4 + 1] * Sinv[2];
        K[i*2 + 1] = ekf->P[i*4 + 0] * Sinv[1] + ekf->P[i*4 + 1] * Sinv[3];
    }

    /* ---- State update: x = x + K*y ---- */
    ekf->id_hat    += K[0]*y[0] + K[1]*y[1];
    ekf->iq_hat    += K[2]*y[0] + K[3]*y[1];
    ekf->omega_hat += K[4]*y[0] + K[5]*y[1];
    ekf->theta_hat += K[6]*y[0] + K[7]*y[1];
    ekf->theta_hat = normalize_angle_positive(ekf->theta_hat);

    /* ---- Covariance update: P = (I - K*H) * P ---- */
    /* KH[i][j] = K[i][0]*H[0][j] + K[i][1]*H[1][j] = K[i][j] for first 2 cols */
    float KH[16] = {0};
    KH[0] = K[0];  KH[1] = K[1];
    KH[4] = K[2];  KH[5] = K[3];
    KH[8] = K[4];  KH[9] = K[5];
    KH[12]= K[6];  KH[13]= K[7];

    /* I - KH */
    float ImKH[16];
    for (int i = 0; i < 16; i++) ImKH[i] = -KH[i];
    ImKH[0]  += 1.0f;
    ImKH[5]  += 1.0f;
    ImKH[10] += 1.0f;
    ImKH[15] += 1.0f;

    /* P_new = ImKH * P */
    float P_new[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += ImKH[i*4 + k] * ekf->P[k*4 + j];
            }
            P_new[i*4 + j] = sum;
        }
    }
    for (int i = 0; i < 16; i++) ekf->P[i] = P_new[i];
}

void ekf_step(ekf_state_t *ekf, float vd, float vq, float id_meas, float iq_meas,
               float dt, const pmsm_params_t *params)
{
    ekf_predict(ekf, vd, vq, dt, params);
    ekf_update(ekf, id_meas, iq_meas, params);
}

/* ========================================================================
 * L5: BLDC Back-EMF Zero-Crossing Detection
 *
 * For BLDC six-step commutation, the unenergized phase's back-EMF
 * zero-crossing is used to determine commutation instants.
 *
 * In a wye-connected BLDC, when two phases are energized (e.g., A+ B-),
 * the neutral point voltage is Vdc/2. The unenergized phase C terminal
 * voltage is: Vc = Vdc/2 + e_c (where e_c is the back-EMF in phase C).
 *
 * A zero crossing of e_c (when Vc crosses Vdc/2) corresponds to
 * the electrical 30�� point after which commutation should occur.
 * After a 30�� delay from ZC detection, the next commutation step
 * is triggered for optimal torque production.
 * ======================================================================== */

bool bemf_zero_cross_detect(float v_phase, float vdc, float v_phase_prev)
{
    float v_neutral = vdc * 0.5f;
    float diff_curr = v_phase - v_neutral;
    float diff_prev = v_phase_prev - v_neutral;

    /* Zero crossing = sign change of (v_phase - Vdc/2) */
    return (diff_curr >= 0.0f && diff_prev < 0.0f)
        || (diff_curr < 0.0f && diff_prev >= 0.0f);
}

float bemf_speed_from_zc_time(float t_between_zc, float pole_pairs)
{
    if (t_between_zc <= 0.0f || pole_pairs <= 0.0f) return 0.0f;

    /* 6 zero-crossings per electrical cycle
       => electrical period = 6 * t_between_zc
       => omega_e = 2*pi / (6 * t_zc) = pi / (3 * t_zc)
       => omega_m = omega_e / P = pi / (3 * P * t_zc) */
    return (float)M_PI / (3.0f * pole_pairs * t_between_zc);
}

/* ========================================================================
 * L8: High-Frequency Injection (HFI)
 * ========================================================================
 *
 * At zero and low speeds, back-EMF is too small for reliable estimation.
 * HFI injects a high-frequency (500-2000 Hz) voltage signal into the
 * estimated d-axis. The resulting HF current in the q-axis contains
 * position error information due to magnetic saliency (Ld != Lq).
 *
 * For pulsating HF injection:
 *   v_d_hf = V_hf * cos(omega_hf * t)   [injected on estimated d-axis]
 *
 * The HF current response in the estimated q-axis:
 *   i_q_hf = (V_hf/omega_hf) * [(Ld-Lq)/(2*Ld*Lq)] * sin(2*theta_error)
 *
 * This signal is demodulated (heterodyne) and passed through a tracking
 * observer to extract the rotor position.
 *
 * HFI requires Ld != Lq (salient rotor). Surface-mount PMSMs with
 * Ld �� Lq cannot use standard HFI and must rely on saturation-induced
 * saliency or alternative methods.
 * ======================================================================== */

void hfi_init(hfi_state_t *hfi, float frequency, float amplitude)
{
    if (!hfi) return;
    memset(hfi, 0, sizeof(hfi_state_t));
    hfi->frequency = frequency;
    hfi->amplitude = amplitude;
    hfi->enabled = true;
}

float hfi_get_injection_voltage(hfi_state_t *hfi, float dt)
{
    if (!hfi || !hfi->enabled) return 0.0f;
    hfi->theta_hf += hfi->frequency * 2.0f * (float)M_PI * dt;
    if (hfi->theta_hf > 2.0f * (float)M_PI) {
        hfi->theta_hf -= 2.0f * (float)M_PI;
    }
    return hfi->amplitude * cosf(hfi->theta_hf);
}

float hfi_extract_position_error(float i_d, float i_q, const hfi_state_t *hfi)
{
    (void)i_d; /* HF d-axis current (may be used for advanced demodulation) */
    if (!hfi || !hfi->enabled) return 0.0f;

    /* Synchronous demodulation: multiply i_q by sin(omega_hf*t)
       and low-pass filter to extract the position-dependent component.
       The sin(2*theta_error) term appears after demodulation and LPF. */

    float sin_inj = sinf(hfi->theta_hf);
    float demod = i_q * sin_inj;  /* Heterodyne demodulation */

    /* The demodulated signal contains:
       - A DC component proportional to sin(2*theta_error) * (Ld-Lq)
       - A 2*f_hf component that will be filtered */

    /* In practice, a LPF extracts the DC component.
       Here we return the raw demodulated signal; the caller applies LPF. */
    return demod;
}

/* ========================================================================
 * L5: Flux Linkage Estimator (Voltage Model)
 *
 * The stator flux linkage is estimated by integrating the back-EMF:
 *   psi_s_alpha = integral( v_alpha - Rs * i_alpha ) dt
 *   psi_s_beta  = integral( v_beta  - Rs * i_beta  ) dt
 *
 * Pure integration suffers from DC offset drift (integrator windup).
 * A high-pass filter (HPF) in series with the integrator gives a
 * band-pass characteristic: psi_s = (1/s) * [s/(s+wc)] * (v - R*i)
 *
 * In discrete time (forward Euler):
 *   psi(k+1) = psi(k) + dt * [ (v(k) - R*i(k)) - wc * psi(k) ]
 * where wc is the HPF cutoff frequency (typ. 0.1-5 Hz).
 *
 * The rotor flux angle is then: theta = atan2(psi_beta, psi_alpha)
 *
 * For interior PMSM, the stator flux includes both PM flux and
 * armature reaction: psi_d = psi_m + Ld*Id, psi_q = Lq*Iq
 * The rotor angle is: theta = atan2(psi_beta, psi_alpha) - displacement
 *
 * Voltage model works well at medium/high speeds where back-EMF
 * dominates over the resistive drop and the HPF does not distort
 * the flux estimate significantly.
 * ======================================================================== */

void flux_estimator_voltage_model(float v_alpha, float v_beta,
                                   float i_alpha, float i_beta,
                                   float rs, float dt,
                                   float *psi_alpha, float *psi_beta,
                                   float cutoff_freq)
{
    if (!psi_alpha || !psi_beta || dt <= 0.0f) return;

    /* Compute back-EMF */
    float e_alpha = v_alpha - rs * i_alpha;
    float e_beta  = v_beta  - rs * i_beta;

    /* HPF compensation term: -wc * psi (drift correction) */
    float dc_corr_alpha = cutoff_freq * (*psi_alpha);
    float dc_corr_beta  = cutoff_freq * (*psi_beta);

    /* Integrate: psi += dt * (BEMF - wc*psi) */
    *psi_alpha += dt * (e_alpha - dc_corr_alpha);
    *psi_beta  += dt * (e_beta  - dc_corr_beta);

    /* Saturation prevention (safety clamp) */
    float psi_mag = sqrtf((*psi_alpha)*(*psi_alpha) + (*psi_beta)*(*psi_beta));
    if (psi_mag > 10.0f) {  /* Unrealistic flux, clamp */
        float scale = 10.0f / psi_mag;
        *psi_alpha *= scale;
        *psi_beta  *= scale;
    }
}

float flux_to_angle(float psi_alpha, float psi_beta)
{
    return atan2f(psi_beta, psi_alpha);
}
