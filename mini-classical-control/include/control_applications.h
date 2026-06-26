/**
 * control_applications.h — Classical Control Applications
 *
 * L6: Canonical control problems
 * L7: Real-world applications — DC motor, cruise control, temperature,
 *     position servo, process control
 *
 * Refs: Ogata Ch.11, Franklin Ch.9, Astrom & Murray "Feedback Systems" (2008).
 */

#ifndef CONTROL_APPLICATIONS_H
#define CONTROL_APPLICATIONS_H

#include "control_core.h"
#include "control_design.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── L6/L7: DC Motor Speed Control ─── */

/**
 * Armature-controlled DC motor transfer function:
 *   G(s) = Ω(s)/V_a(s) = K_m / [(J·s + b)·(L_a·s + R_a) + K_m·K_b]
 *
 * Parameters:
 *   J     = moment of inertia [kg·m²]
 *   b     = viscous friction [N·m·s]
 *   K_m   = motor torque constant [N·m/A]
 *   K_b   = back-EMF constant [V·s/rad] (≈ K_m in SI)
 *   R_a   = armature resistance [Ω]
 *   L_a   = armature inductance [H]
 *
 * Knowledge: The DC motor is the canonical electromechanical
 * system in classical control education.
 */
int dc_motor_model(double J, double b, double Km, double Kb,
                   double Ra, double La, transfer_function_t *G);

/**
 * Simplified DC motor (neglecting inductance L_a → 0):
 *   G(s) = (K_m/R_a) / (J·s + b + K_m·K_b/R_a)
 * = K / (τ·s + 1), a first-order system.
 */
int dc_motor_simplified(double J, double b, double Km, double Kb,
                        double Ra, transfer_function_t *G);

/**
 * Design a PI speed controller for a DC motor.
 * Uses pole-placement on simplified first-order model.
 * Target closed-loop: ω_n, ζ specified.
 */
int dc_motor_pi_speed_control(double J, double b, double Km, double Kb,
                               double Ra, double zeta, double omega_n,
                               pid_params_t *pid);

/* ─── L7: Automotive Cruise Control ─── */

/**
 * Vehicle longitudinal model:
 *   m·dv/dt + b·v = F_engine - F_drag
 *
 * Transfer function from engine force to speed:
 *   G(s) = V(s)/F(s) = 1 / (m·s + b)
 *
 * Where m = vehicle mass [kg], b = damping coefficient [N·s/m].
 */
int cruise_vehicle_model(double m, double b, transfer_function_t *G);

/**
 * Design a PI cruise controller with specified performance.
 * Assumes simplified vehicle model.
 */
int cruise_pi_design(double m, double b, double zeta, double omega_n,
                     pid_params_t *pid);

/* ─── L7: Temperature Control ─── */

/**
 * First-order plus dead time (FOPDT) thermal model:
 *   G(s) = K·e^{-L·s} / (T·s + 1)
 *
 * Used extensively in process control for thermal, flow, level loops.
 * K = process gain, T = time constant, L = dead time.
 */
int thermal_fopdt_model(double K, double T, double L,
                        transfer_function_t *G, transfer_function_t *G_delay);

/**
 * PID temperature controller design for FOPDT process.
 * Uses Cohen-Coon tuning rules for 1/4 decay ratio.
 */
int temperature_pid_design(double K, double T, double L,
                           pid_params_t *pid);

/* ─── L6: Position Servo System ─── */

/**
 * DC motor position servo (with gear train).
 * Position = integral of speed.
 *
 * G(s) = Θ(s)/V_a(s) = K_m / [s·((J·s+b)·(L_a·s+R_a) + K_m·K_b)]
 *
 * Includes integrator from velocity to position (type-1 system).
 */
int servo_position_model(double J, double b, double Km, double Kb,
                          double Ra, double La, transfer_function_t *G);

/**
 * Design a PD position controller for the servo.
 * PD adds damping (derivative) to improve stability.
 * Target closed-loop poles at specified ζ, ω_n.
 */
int servo_pd_design(double J, double b, double Km, double Kb,
                    double Ra, double zeta, double omega_n,
                    pid_params_t *pid);

/* ─── L6: Ball and Beam ─── */

/**
 * Ball and beam system transfer function (linearized):
 *   G(s) = X(s)/Θ(s) = (5g/7) / s²
 *
 * X = ball position on beam, Θ = beam angle.
 * Double integrator — marginally stable, needs derivative action.
 */
int ball_beam_model(transfer_function_t *G);

/* ─── L6: Inverted Pendulum (Linearized) ─── */

/**
 * Linearized inverted pendulum on cart:
 *
 * Transfer function from cart force to pendulum angle:
 *   G(s) = Θ(s)/F(s) = num / den
 *
 * Mass m [kg], length l [m], cart mass M [kg].
 */
int inverted_pendulum_model(double M, double m, double l,
                            transfer_function_t *G);

/* ─── L7: Industrial Process Control ─── */

/**
 * Generic industrial process model builder.
 * Supports: FOPDT, SOPDT (second-order plus dead time),
 * integrating process, inverse response.
 */
typedef enum {
    PROCESS_FOPDT,          /**< Ke^{-Ls}/(Ts+1)                */
    PROCESS_SOPDT,          /**< Ke^{-Ls}/((T1s+1)(T2s+1))     */
    PROCESS_INTEGRATING,    /**< Ke^{-Ls}/(s(Ts+1))            */
    PROCESS_INVERSE_RESP,   /**< K(-T3s+1)e^{-Ls}/((T1s+1)(T2s+1)) */
} process_model_type_t;

typedef struct {
    process_model_type_t type;
    double K, L;
    double T1, T2, T3;
} process_model_t;

/**
 * Build a transfer function from process model parameters.
 */
int process_model_to_tf(const process_model_t *pm, transfer_function_t *G);

/**
 * Auto-tune PID for industrial process using model-based rules.
 * Selects the best method based on process type.
 */
int process_auto_tune(const process_model_t *pm, pid_params_t *pid);

/**
 * Simulate closed-loop response of process + PID controller.
 * Includes saturation (output limits) and anti-windup.
 */
int process_closed_loop_sim(const process_model_t *pm,
                             const pid_params_t *pid,
                             double setpoint, double t_final, double dt,
                             double u_min, double u_max,
                             double *t, double *y, double *u,
                             int max_points, int *num_points);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_APPLICATIONS_H */
