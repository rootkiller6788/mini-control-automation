/**
 * @file pid_analysis.h
 * @brief PID Stability and Performance Analysis
 *
 * Covers knowledge levels:
 *   L3 -- Mathematical Structures: frequency response, Bode plots, Nyquist criterion
 *   L4 -- Fundamental Laws: Routh-Hurwitz stability criterion, gain/phase margin,
 *         Lyapunov stability for PID-controlled systems, Final Value Theorem
 *   L6 -- Canonical Problems: stability margin computation, sensitivity analysis
 *
 * Reference:
 *   Routh (1877), "A Treatise on the Stability of a Given State of Motion"
 *   Hurwitz (1895), "On the conditions under which an equation has only
 *     roots with negative real parts"
 *   Nyquist (1932), "Regeneration Theory"
 *   Bode (1945), "Network Analysis and Feedback Amplifier Design"
 */

#ifndef PID_ANALYSIS_H
#define PID_ANALYSIS_H

#include "pid_core.h"
#include "pid_tuning.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L3/L4 -- Stability Analysis Structures
 *===========================================================================*/

/**
 * @brief Polynomial for Routh-Hurwitz analysis
 *
 * a_n*s^n + a_{n-1}*s^{n-1} + ... + a_1*s + a_0
 */
typedef struct {
    double *coeffs;  /**< Coefficients a_0 (constant) to a_n (highest order) */
    int     order;   /**< Polynomial order n (1..10) */
} Polynomial;

/**
 * @brief Routh array entry for stability analysis
 */
typedef struct {
    double *rows[12];  /**< Routh array rows, each of length (order+1)/2 */
    int     order;     /**< Original polynomial order */
    int     sign_changes; /**< Number of sign changes in first column */
} RouthArray;

/**
 * @brief Frequency response analysis result
 */
typedef struct {
    double *freq;       /**< Frequency points (rad/s) */
    double *magnitude;  /**< Magnitude |G(jw)| */
    double *phase;      /**< Phase in radians */
    size_t  N;          /**< Number of frequency points */
    double  gain_margin;    /**< Gain margin in dB */
    double  phase_margin;   /**< Phase margin in radians */
    double  gain_crossover; /**< Frequency where phase = -pi (rad/s) */
    double  phase_crossover; /**< Frequency where |G| = 1 (rad/s) */
    double  bandwidth;      /**< Closed-loop bandwidth (-3dB, rad/s) */
} FrequencyAnalysis;

/**
 * @brief Step response performance metrics
 *
 * Metrics computed from the closed-loop step response:
 *
 * Rise time (tr):   Time to go from 10% to 90% of final value
 * Overshoot (Mp):   (peak_value - final_value) / final_value
 * Settling time (ts): Time to stay within +/- pct of final value (default 2%)
 * Steady-state error (ess): final_value - setpoint
 * Peak time (tp):   Time at which peak overshoot occurs
 * Delay time (td):  Time to reach 50% of final value
 * Decay ratio:      Ratio of second peak to first peak
 * IAE:              Integral of Absolute Error = integral(|e(t)| dt)
 * ISE:              Integral of Squared Error = integral(e(t)^2 dt)
 * ITAE:             Integral of Time-weighted Absolute Error = integral(t*|e(t)| dt)
 * ITSE:             Integral of Time-weighted Squared Error = integral(t*e(t)^2 dt)
 */
typedef struct {
    double rise_time;          /**< 10%-90% rise time (s) */
    double overshoot;          /**< Percent overshoot (fraction, e.g. 0.25 = 25%) */
    double settling_time;      /**< 2% settling time (s) */
    double steady_state_error; /**< Steady-state error */
    double peak_time;          /**< Time of first peak (s) */
    double delay_time;         /**< 50% rise time (s) */
    double decay_ratio;        /**< Second peak / First peak */
    double IAE;                /**< Integral of Absolute Error */
    double ISE;                /**< Integral of Squared Error */
    double ITAE;               /**< Integral of Time-weighted Absolute Error */
    double ITSE;               /**< Integral of Time-weighted Squared Error */
} StepResponseMetrics;

/*===========================================================================
 * L3 -- Frequency Analysis API
 *===========================================================================*/

/**
 * @brief Compute open-loop frequency response of PID + process
 *
 * L(s) = G_c(s) * G_p(s)  (loop transfer function)
 * Evaluates magnitude and phase at logarithmically spaced frequencies.
 *
 * @param pid_tf    PID transfer function.
 * @param model     Process FOPDT model.
 * @param f_min     Minimum frequency (rad/s).
 * @param f_max     Maximum frequency (rad/s).
 * @param n_points  Number of frequency points.
 * @param analysis  Output frequency analysis.
 * @return 0 on success, -1 on error.
 */
int pid_loop_frequency_analysis(const PIDTransferFunction *pid_tf,
                                const FOPDTModel *model,
                                double f_min, double f_max, size_t n_points,
                                FrequencyAnalysis *analysis);

/**
 * @brief Compute stability margins from frequency analysis
 *
 * Gain margin (Gm): 1/|L(j*w_pc)| where w_pc is phase crossover (phase = -180 deg)
 * Phase margin (Pm): 180 + phase(L(j*w_gc)) where w_gc is gain crossover (|L|=1)
 *
 * Stability conditions (Bode criterion):
 *   Gm > 0 dB (i.e., |L(j*w_pc)| < 1) AND Pm > 0 degrees
 *   Typically desired: Gm >= 6 dB AND Pm >= 45 degrees
 *
 * @param analysis Frequency analysis data (modified in-place to add margins/bw).
 * @return 0 on success.
 */
int pid_compute_stability_margins(FrequencyAnalysis *analysis);

/**
 * @brief Compute sensitivity functions
 *
 * S(s) = 1 / (1 + L(s))   -- sensitivity (disturbance rejection)
 * T(s) = L(s) / (1 + L(s)) -- complementary sensitivity (noise rejection, tracking)
 *
 * S(s) + T(s) = 1 (fundamental trade-off)
 *
 * @param loop_mag  Magnitude of L(jw).
 * @param loop_phase Phase of L(jw).
 * @param N         Number of points.
 * @param S_mag     Output: |S(jw)|.
 * @param T_mag     Output: |T(jw)|.
 */
void pid_compute_sensitivity(const double *loop_mag, const double *loop_phase,
                             size_t N, double *S_mag, double *T_mag);

/*===========================================================================
 * L4 -- Routh-Hurwitz Stability Criterion
 *===========================================================================*/

/**
 * @brief Construct Routh array from polynomial coefficients
 *
 * Routh-Hurwitz criterion: A polynomial is stable (all roots in LHP)
 * iff all elements in the first column of the Routh array have the same sign.
 * The number of sign changes = number of RHP roots.
 *
 * @param poly  Polynomial. a_0 + a_1*s + ... + a_n*s^n.
 * @param ra    Output Routh array. Caller must free with pid_routh_free().
 * @return 0 on success, -1 if polynomial order is too high.
 */
int pid_routh_construct(const Polynomial *poly, RouthArray *ra);

/**
 * @brief Check stability using Routh-Hurwitz criterion
 *
 * @param ra  Constructed Routh array.
 * @return 1 if stable, 0 if unstable, -1 if marginally stable.
 */
int pid_routh_is_stable(const RouthArray *ra);

/**
 * @brief Free memory allocated for Routh array
 * @param ra Routh array to free.
 */
void pid_routh_free(RouthArray *ra);

/*===========================================================================
 * L4 -- Lyapunov Stability for PID Systems
 *===========================================================================*/

/**
 * @brief Construct the closed-loop system matrix for PID + second-order process
 *
 * For a second-order process: y'' + a1*y' + a0*y = b0*u
 * with PID: u = Kp*(r-y) + Ki*integral(r-y) + Kd*(d/dt)(r-y)
 *
 * State vector: [y, y', integral_e]
 * System matrix A = [[0, 1, 0], [-a0-b0*Kp, -a1-b0*Kd, b0*Ki],
 *                    [-1, 0, 0]]
 *
 * Lyapunov stability: find P > 0 s.t. A'*P + P*A = -Q (Q > 0)
 * If such P exists, the system is asymptotically stable.
 *
 * @param a0,a1 Process parameters (y'' + a1*y' + a0*y = b0*u).
 * @param b0    Process input gain.
 * @param Kp,Ki,Kd PID gains.
 * @param A     Output 3x3 system matrix (row-major, size 9 doubles).
 */
void pid_lyapunov_system_matrix(double a0, double a1, double b0,
                                double Kp, double Ki, double Kd, double A[9]);

/**
 * @brief Solve continuous-time Lyapunov equation A'*P + P*A + Q = 0
 *
 * Uses Bartels-Stewart algorithm for small (3x3) matrices.
 *
 * @param A System matrix (n x n row-major).
 * @param Q Symmetric matrix (n x n row-major), must be positive definite.
 * @param P Output solution (n x n row-major).
 * @param n Matrix size (typically 3).
 * @return 0 if solution found, -1 if A has eigenvalues on imaginary axis.
 */
int pid_solve_lyapunov(const double *A, const double *Q, double *P, int n);

/**
 * @brief Check if matrix is positive definite using Cholesky decomposition
 *
 * @param M Symmetric matrix (n x n row-major).
 * @param n Size.
 * @return 1 if positive definite, 0 otherwise.
 */
int pid_is_positive_definite(const double *M, int n);

/*===========================================================================
 * L3/L6 -- Step Response Analysis
 *===========================================================================*/

/**
 * @brief Simulate closed-loop step response
 *
 * Simulates PID + FOPDT process response to a unit step in setpoint.
 * Uses Euler integration with dead-time handled by ring buffer.
 * Complexity: O(N) where N = duration/Ts.
 *
 * @param pid      PID controller (will be reset before simulation).
 * @param model    Process FOPDT model.
 * @param setpoint Target setpoint value (step from 0 to setpoint).
 * @param duration Simulation duration (seconds).
 * @param Ts       Sampling interval for simulation.
 * @param time     Output time vector [0..N-1] (caller allocates).
 * @param output   Output process output vector [0..N-1] (caller allocates).
 * @param control  Output control signal vector [0..N-1] (caller allocates).
 * @param N        Number of simulation steps = duration/Ts.
 * @return 0 on success.
 */
int pid_simulate_step_response(const PIDController *pid, const FOPDTModel *model,
                               double setpoint, double duration, double Ts,
                               double *time, double *output, double *control,
                               size_t N);

/**
 * @brief Compute step response performance metrics
 *
 * Analyzes the step response data to extract all standard performance metrics.
 * Reference: Astrom & Hagglund (2006), "Advanced PID Control", Chapter 4.
 *
 * @param time    Time vector.
 * @param output  Process output vector.
 * @param N       Number of data points.
 * @param setpoint Final desired value.
 * @param metrics Output performance metrics.
 * @return 0 on success, -1 if data is unusable.
 */
int pid_step_metrics(const double *time, const double *output, size_t N,
                     double setpoint, StepResponseMetrics *metrics);

/**
 * @brief Compute stability region in (Kp, Ki) plane for given Kd
 *
 * For P-controlled systems, determines the set of (Kp, Ki) gains
 * that result in closed-loop stability (using Routh-Hurwitz).
 *
 * @param model Process FOPDT model (first-order Pade approximation for delay).
 * @param Kd    Fixed derivative gain.
 * @param Kp    Output: array of stable Kp values.
 * @param Ki    Output: array of stable Ki values.
 * @param n     Number of boundary points to compute.
 * @return 0 on success.
 */
int pid_stability_region(const FOPDTModel *model, double Kd,
                         double *Kp, double *Ki, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* PID_ANALYSIS_H */
