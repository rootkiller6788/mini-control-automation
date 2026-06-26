/**
 * control_design.h — Controller Design Methods
 *
 * L5: PID Controller Design
 *     - Ziegler-Nichols tuning (step response & ultimate gain)
 *     - Cohen-Coon tuning (FOPDT model)
 *     - Lead/Lag compensator design
 *     - Pole placement via Ackermann's formula
 * L6: Canonical design flows
 *
 * Refs: Ogata Ch.10, Astrom & Hagglund "PID Controllers" (1995),
 *       Ziegler & Nichols (1942), Cohen & Coon (1953).
 */

#ifndef CONTROL_DESIGN_H
#define CONTROL_DESIGN_H

#include "control_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── L5: PID Controller Tuning Methods ─── */

/**
 * Ziegler-Nichols Step Response Method (Open-Loop).
 *
 * Uses the process reaction curve from a step response.
 * The FOPDT model: G(s) = K·e^{-L·s} / (T·s + 1)
 * Parameters extracted from the inflection point tangent:
 *   L = apparent dead time
 *   T = apparent time constant
 *   K = static gain = Δy/Δu
 *   a = K·L/T
 *
 * Tuning rules (Ziegler-Nichols, 1942):
 *          |  Kp        |  Ti         |  Td
 *   ───────┼────────────┼─────────────┼────────
 *   P      |  1/a       |  —          |  —
 *   PI     |  0.9/a     |  3·L        |  —
 *   PID    |  1.2/a     |  2·L        |  0.5·L
 *
 * Returns PID params for the specified controller type (0=P, 1=PI, 2=PID).
 */
int zn_step_response(double K, double L, double T, int type,
                     pid_params_t *pid);

/**
 * Ziegler-Nichols Ultimate Gain Method (Closed-Loop).
 *
 * Procedure:
 *   1. Set Ki=0, Kd=0.
 *   2. Increase Kp until sustained oscillation (critical gain K_u).
 *   3. Measure oscillation period P_u.
 *
 * Tuning rules:
 *          |  Kp        |  Ti         |  Td
 *   ───────┼────────────┼─────────────┼────────
 *   P      |  0.5·K_u   |  —          |  —
 *   PI     |  0.45·K_u  |  P_u/1.2    |  —
 *   PID    |  0.6·K_u   |  P_u/2      |  P_u/8
 *
 * Knowledge: The ultimate gain method works by finding the stability
 * boundary via experiment or computation.
 */
int zn_ultimate_gain(double Ku, double Pu, int type, pid_params_t *pid);

/**
 * Cohen-Coon Tuning (Open-Loop FOPDT).
 *
 * More refined than ZN, designed for 1/4 decay ratio.
 * Minimizes: IAE (or ISE) for given decay ratio.
 *
 * Rules for PID (Cohen-Coon, 1953):
 *   Kp = (1/K)·(T/L)·(4/3 + L/(4T))
 *   Ti = L·(32 + 6·L/T) / (13 + 8·L/T)
 *   Td = L·4 / (11 + 2·L/T)
 */
int cohen_coon(double K, double L, double T, int type, pid_params_t *pid);

/**
 * Convert PID parallel form to standard ISA form.
 *
 * Parallel (independent gains):
 *   u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt
 *
 * ISA (standard) form:
 *   u(t) = Kc·[e(t) + (1/Ti)·∫e(t)dt + Td·de/dt]
 * where Kc=Kp, Ti=Kp/Ki, Td=Kd/Kp.
 */
int pid_parallel_to_isa(const pid_params_t *parallel, pid_params_t *isa);

/**
 * Apply derivative filtering to PID controller.
 * Replaces ideal D term Kd·s with Kd·s/(1 + s·Tf).
 * This limits high-frequency gain from the derivative.
 */
int pid_set_derivative_filter(pid_params_t *pid, double Tf);

/**
 * Anti-windup configuration for PID.
 * Sets the tracking time constant for back-calculation.
 */
int pid_antiwindup(pid_params_t *pid, double Tt);

/* ─── L5: Compensator Design ─── */

/**
 * Design a lead compensator: G_c(s) = Kc·(1 + α·τ·s)/(1 + τ·s), α > 1.
 *
 * Lead compensator adds phase lead (positive phase) at a target frequency.
 * Maximum phase lead: φ_max = arcsin((α-1)/(α+1))
 * Occurs at: ω_max = 1/(τ·√α)
 *
 * Design spec: desired phase margin PM_des at gain crossover ω_gc.
 *
 * Procedure (Ogata §10-3):
 *   1. Determine required phase lead: φ_lead = PM_des - PM_current + margin
 *   2. Compute α from φ_lead
 *   3. Place ω_max at desired crossover
 *   4. Determine K_c to achieve 0 dB at crossover
 *
 * Returns lead TF via Gc parameter.
 */
int lead_design(const transfer_function_t *G, double PM_des,
                transfer_function_t *Gc);

/**
 * Design a lag compensator: G_c(s) = Kc·(1 + τ·s)/(1 + β·τ·s), β > 1.
 *
 * Lag compensator reduces high-frequency gain without affecting
 * low-frequency gain much. Improves steady-state error.
 *
 * Design spec: desired static error constant improvement factor β.
 * The lag zero is placed 1 decade below gain crossover.
 */
int lag_design(const transfer_function_t *G, double beta,
               double gain_crossover, transfer_function_t *Gc);

/**
 * Design a lead-lag compensator combining both approaches.
 * Lead part: improves transient response (PM, bandwidth).
 * Lag part: improves steady-state accuracy.
 */
int lead_lag_design(const transfer_function_t *G, double PM_des,
                    double beta, transfer_function_t *Gc);

/* ─── L5: Pole Placement ─── */

/**
 * Pole placement via Ackermann's formula (SISO).
 *
 * Given (A,B) controllable, places closed-loop poles at desired locations
 * by state feedback u = -K·x.
 *
 * Ackermann's formula:
 *   K = [0 ... 0 1]·C^{-1}·φ_c(A)
 * where C = controllability matrix, φ_c(A) is the characteristic
 * polynomial of desired poles evaluated at A.
 *
 * Theorem (Wonham, 1967): Pole placement is possible for any
 * desired pole set iff (A,B) is controllable.
 */
int ackermann_pole_placement(const state_space_t *ss,
                              const double complex *desired_poles,
                              int num_poles, double *K);

/**
 * Place dominant 2nd-order poles for desired ζ, ω_n.
 * Remaining poles placed at 5× real-part distance (negligible effect).
 */
int dominant_pole_placement(const state_space_t *ss,
                            double zeta, double omega_n, double *K);

/**
 * Compute the state-feedback closed-loop A matrix: A_cl = A - B·K.
 */
int state_feedback_apply(const state_space_t *ss, const double *K,
                          double *A_cl);

/**
 * Feedforward gain for unity DC gain: N = 1/(C·(-(A-B·K)⁻¹)·B)
 * Ensures y_ss = r_ss for step reference.
 */
double feedforward_gain(const state_space_t *ss, const double *K);

/**
 * Implement a PID controller as a transfer function.
 * C(s) = Kp + Ki/s + Kd·s/(1 + s·Tf)
 * With anti-windup tracking time Tt.
 */
int pid_to_tf(const pid_params_t *pid, transfer_function_t *C);

/**
 * Compute the loop transfer function L(s) = C(s)·G(s).
 */
int loop_tf(const transfer_function_t *C, const transfer_function_t *G,
            transfer_function_t *L);

/**
 * Design PID by pole placement for a FOPDT model.
 * Dominant pole placement with pole-zero cancellation.
 */
int pid_pole_placement_fopdt(double K, double L, double T,
                              double zeta_des, double omega_n_des,
                              pid_params_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_DESIGN_H */
