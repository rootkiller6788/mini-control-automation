/**
 * control_analysis.h — Stability Analysis & Frequency Response
 *
 * L2: Stability concepts — BIBO, asymptotic, marginal stability
 * L4: Routh-Hurwitz criterion, Nyquist stability criterion,
 *     Bode stability criterion, Final Value Theorem
 * L5: Frequency response computation, margin evaluation
 *
 * Refs: Ogata Ch.5-8, Franklin Ch.6, Dorf Ch.7-9.
 */

#ifndef CONTROL_ANALYSIS_H
#define CONTROL_ANALYSIS_H

#include "control_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── L4: Routh-Hurwitz Stability Criterion ─── */

/**
 * Construct the Routh array for a polynomial.
 *
 * Given characteristic polynomial:
 *   a₀·sⁿ + a₁·s^{n-1} + ... + a_n = 0
 *
 * The Routh array determines the number of roots with positive real parts.
 * Theorem (Routh, 1874; Hurwitz, 1895):
 *   Number of RHP roots = number of sign changes in the first column.
 *
 * Returns: number of right-half-plane roots (0 = stable).
 * Returns -1 if the array encounters special cases (e.g., row of zeros).
 */
int routh_hurwitz(const double *coeff, int order, int *num_rhp_roots);

/**
 * Simplified Routh-Hurwitz for 2nd-order systems.
 * Characteristic: a₀·s² + a₁·s + a₂ = 0
 * Condition: all a_i > 0 for stability.
 */
int routh_2nd_order(double a0, double a1, double a2);

/**
 * Routh-Hurwitz for 3rd-order systems.
 * Characteristic: a₀·s³ + a₁·s² + a₂·s + a₃ = 0
 * Condition: a₀,a₁,a₂,a₃ > 0 AND a₁·a₂ > a₀·a₃
 */
int routh_3rd_order(double a0, double a1, double a2, double a3);

/**
 * Routh-Hurwitz for 4th-order systems.
 * Characteristic: a₀·s⁴ + a₁·s³ + a₂·s² + a₃·s + a₄ = 0
 * Condition: all a_i > 0, a₁·a₂ > a₀·a₃, a₁·a₂·a₃ > a₀·a₃² + a₁²·a₄
 */
int routh_4th_order(double a0, double a1, double a2, double a3, double a4);

/**
 * Find the critical gain K_crit using Routh array.
 * Determines the gain at which the system becomes marginally stable.
 * Useful for Ziegler-Nichols ultimate gain method (§control_design.h).
 */
double routh_critical_gain(const transfer_function_t *G);

/**
 * Check stability of closed-loop system defined by TF G(s).
 * 1. Forms closed-loop characteristic: 1 + G(s) = 0
 * 2. Applies Routh-Hurwitz to denominator of 1+G.
 *
 * Returns: 1 if stable, 0 if unstable, -1 on error.
 */
int tf_is_stable_cl(const transfer_function_t *G);

/* ─── L4: Nyquist Stability Criterion ─── */

/**
 * Compute the Nyquist contour values G(jω) for a range of frequencies.
 *
 * The Nyquist criterion (Nyquist, 1932):
 *   Z = N + P, where:
 *   Z = # of closed-loop RHP poles
 *   N = # of counterclockwise encirclements of (-1, j0) by G(jω) plot
 *   P = # of open-loop RHP poles
 *
 * If the open-loop system is stable (P=0), then closed-loop stability
 * requires N=0 (no encirclements of -1).
 */
int nyquist_contour(const transfer_function_t *G,
                    double *omega, int num_points,
                    double complex *G_jw);

/**
 * Count encirclements of the critical point (-1, j0) by the
 * Nyquist path G(jω) from ω = 0⁺ to +∞.
 * Also handles the mirror image for ω < 0 (symmetry).
 *
 * Returns: number of clockwise encirclements N.
 *   For stability with open-loop P=0: N must be 0.
 */
int nyquist_encirclements(const double complex *G_jw, int num_points);

/**
 * Full Nyquist stability check.
 * P = number of open-loop RHP poles
 * Returns 1 if closed-loop stable (Z = N + P = 0).
 */
int nyquist_stability(const transfer_function_t *G, int P);

/* ─── L5: Bode Plot & Frequency Response ─── */

/**
 * Compute Bode plot data: magnitude [dB] and phase [degrees]
 * for logarithmically spaced frequencies.
 *
 * Knowledge: Asymptotic Bode approximation:
 *   - Integrator/pole at origin: -20 dB/dec, -90°
 *   - Real pole at -1/τ: 0 → -20 dB/dec, 0° → -90° (over 2 decades)
 *   - Real zero at -1/τ: 0 → +20 dB/dec, 0° → +90°
 *   - Complex poles: resonance peak near ω_n, -40 dB/dec at high freq
 */
int bode_plot(const transfer_function_t *G,
              double omega_min, double omega_max, int num_points,
              double *omega, double *mag_db, double *phase_deg);

/**
 * Compute gain margin and phase margin from Bode data.
 *
 * Gain Margin (GM): reciprocal of |G(jω_pc)| where ∠G(jω_pc) = -180°
 *   GM_dB = -20·log₁₀(|G(jω_pc)|)
 *
 * Phase Margin (PM): ∠G(jω_gc) - (-180°) where |G(jω_gc)| = 1 (0 dB)
 *   PM_deg = 180° + ∠G(jω_gc)
 *
 * Theorem (Bode, 1945): For minimum-phase systems,
 *   PM > 0 and GM > 0 dB are necessary and sufficient for stability.
 */
int compute_margins(const transfer_function_t *G, freq_specs_t *specs);

/**
 * Compute the sensitivity function S(s) = 1/(1+G(s)).
 * Sensitivity indicates disturbance rejection: smaller |S(jω)| = better.
 *
 * Complementary sensitivity: T(s) = G(s)/(1+G(s)) = 1 - S(s).
 * Bode integral (waterbed effect): ∫₀^∞ ln|S(jω)| dω = π·Σ Re(p_i)
 *   for open-loop poles p_i in RHP.
 */
int sensitivity_function(const transfer_function_t *G, double omega,
                         double *S_mag_db, double *T_mag_db);

/** Compute closed-loop bandwidth (-3 dB point of T(jω)). */
double bandwidth_find(const transfer_function_t *G);

/* ─── L3: Time-Domain Response ─── */

/**
 * Simulate step response of a transfer function numerically.
 *
 * Method: discretized state-space via forward Euler or
 * 4th-order Runge-Kutta on controllable canonical form.
 *
 * Knowledge: The step response reveals transient characteristics.
 *   - Overdamped (ζ>1): monotonic, slow
 *   - Critically damped (ζ=1): fastest without overshoot
 *   - Underdamped (0<ζ<1): oscillatory with overshoot
 *   - Undamped (ζ=0): sustained oscillation
 */
int step_response(const transfer_function_t *G,
                  double t_final, double dt,
                  double *t, double *y, int max_points, int *num_points);

/**
 * Simulate impulse response of a transfer function numerically.
 * Impulse = derivative of step response (for linear system).
 */
int impulse_response(const transfer_function_t *G,
                     double t_final, double dt,
                     double *t, double *y, int max_points, int *num_points);

/**
 * Compute steady-state error for standard inputs.
 *
 * By the Final Value Theorem (L4):
 *   e_ss(step)   = lim_{s→0} s·(1/s)·1/(1+G(s)) = 1/(1+Kp)
 *   e_ss(ramp)   = lim_{s→0} s·(1/s²)·1/(1+G(s)) = 1/Kv
 *   e_ss(parab)  = lim_{s→0} s·(1/s³)·1/(1+G(s)) = 1/Ka
 *
 * Position error constant: Kp = lim_{s→0} G(s)
 * Velocity error constant: Kv = lim_{s→0} s·G(s)
 * Acceleration error constant: Ka = lim_{s→0} s²·G(s)
 */
int steady_state_errors(const transfer_function_t *G,
                        double *e_step, double *e_ramp, double *e_parabola);

/**
 * Compute the static error constants Kp, Kv, Ka.
 */
int error_constants(const transfer_function_t *G,
                    double *Kp, double *Kv, double *Ka);

/**
 * Extract equivalent 2nd-order parameters (ζ, ω_n) from dominant poles.
 * Finds the dominant complex pole pair closest to jω axis.
 */
int dominant_pole_params(const transfer_function_t *G,
                         double *zeta, double *omega_n);

/**
 * Determine stability by examining pole locations.
 * Stable: all poles in LHP (Re < 0).
 * Marginally stable: poles on jω axis, none in RHP.
 * Unstable: any pole in RHP (Re > 0).
 */
typedef enum { STABLE, MARGINALLY_STABLE, UNSTABLE } stability_t;
stability_t pole_stability(const pole_zero_t *pz);

/**
 * Compute damping ratio and natural frequency from a complex pole pair.
 * If s = -σ ± jω_d, then:
 *   ω_n = √(σ² + ω_d²)
 *   ζ = σ / ω_n
 */
void pole_to_zeta_omega(double complex pole, double *zeta, double *omega_n);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ANALYSIS_H */
