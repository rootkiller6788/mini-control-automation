#ifndef MODERN_CONTROL_H
#define MODERN_CONTROL_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Core Definitions - State-Space Model
 *
 * Continuous: dx/dt = A*x(t) + B*u(t),  y(t) = C*x(t) + D*u(t)
 * Discrete:   x(k+1) = A*x(k) + B*u(k),  y(k) = C*x(k) + D*u(k)
 *
 * References: Ogata (2010), Chen (2013), Kalman (1963)
 * ========================================================================= */

typedef enum {
    MC_SYS_CONTINUOUS  = 0,
    MC_SYS_DISCRETE    = 1,
    MC_SYS_SAMPLED     = 2
} mc_system_type_t;

typedef enum {
    MC_STABLE            = 0,
    MC_UNSTABLE          = 1,
    MC_MARGINALLY_STABLE = 2,
    MC_UNDETERMINED      = 3
} mc_stability_t;

typedef enum {
    MC_FULLY_CONTROLLABLE     = 0,
    MC_NOT_CONTROLLABLE       = 1,
    MC_STABILIZABLE           = 2,
    MC_UNDETERMINED_CTRB      = 3
} mc_controllability_status_t;

typedef enum {
    MC_FULLY_OBSERVABLE    = 0,
    MC_NOT_OBSERVABLE      = 1,
    MC_DETECTABLE          = 2,
    MC_UNDETERMINED_OBSV   = 3
} mc_observability_status_t;

typedef enum {
    MC_DISC_ZOH           = 0,
    MC_DISC_FOH           = 1,
    MC_DISC_TUSTIN        = 2,
    MC_DISC_EULER_FWD     = 3,
    MC_DISC_EULER_BWD     = 4,
    MC_DISC_MATCHED_PZ    = 5
} mc_discretization_t;

/* L3: Mathematical Structures - Dense Matrix (row-major) */
typedef struct {
    size_t   rows;
    size_t   cols;
    size_t   stride;
    double  *data;
    int      owner;
} mc_matrix_t;

/** L3: Real Vector Structure */
typedef struct {
    size_t   length;
    double  *data;
    int      owner;
} mc_vector_t;

/* L1: Continuous-Time State-Space Model */
typedef struct {
    size_t       n_states;
    size_t       n_inputs;
    size_t       n_outputs;
    mc_matrix_t  A;
    mc_matrix_t  B;
    mc_matrix_t  C;
    mc_matrix_t  D;
    mc_system_type_t sys_type;
    char         name[64];
} mc_ss_system_t;

/* L1: Discrete-Time State-Space Model */
typedef struct {
    size_t       n_states;
    size_t       n_inputs;
    size_t       n_outputs;
    mc_matrix_t  A;
    mc_matrix_t  B;
    mc_matrix_t  C;
    mc_matrix_t  D;
    double       sample_time;
    mc_system_type_t sys_type;
    char         name[64];
} mc_ss_discrete_t;

/* L1: Transfer Function (SISO rational polynomial) */
typedef struct {
    size_t   num_order;
    size_t   den_order;
    double  *num_coeff;
    double  *den_coeff;
    double   dc_gain;
    int      is_proper;
    int      is_strictly_proper;
} mc_tf_t;

/* L2: Controllability Matrix (Kalman, 1960) */
typedef struct {
    mc_matrix_t           Ctrb;
    mc_matrix_t           Ctrb_gram;
    size_t                rank;
    double                min_singular;
    double                condition_number;
    mc_controllability_status_t   status;
} mc_controllability_t;

/* L2: Observability Matrix (Kalman, 1960) */
typedef struct {
    mc_matrix_t         Obsv;
    mc_matrix_t         Obsv_gram;
    size_t              rank;
    double              min_singular;
    double              condition_number;
    mc_observability_status_t   status;
} mc_observability_t;

/* L2: State Feedback Controller */
typedef struct {
    mc_matrix_t  K;
    mc_matrix_t  N;
    double      *poles;
    size_t       n_states;
    size_t       n_inputs;
    int          is_stabilizing;
} mc_state_feedback_t;

/* L2: Luenberger Observer (Luenberger, 1964) */
typedef struct {
    mc_matrix_t  L;
    double      *observer_poles;
    size_t       n_states;
    size_t       n_outputs;
    int          is_convergent;
} mc_luenberger_observer_t;

/* L4: Lyapunov Equation Solution */
typedef struct {
    mc_matrix_t  P;
    mc_matrix_t  Q;
    int          is_positive_definite;
    double       min_eigenvalue;
    double       max_eigenvalue;
    double       condition_number;
    int          convergence;
    size_t       iterations;
} mc_lyapunov_solution_t;

/* L4: CARE Solution - Linear Quadratic Regulator */
typedef struct {
    mc_matrix_t  P;
    mc_matrix_t  K;
    double       j_min;
    double       closed_loop_poles_real[32];
    double       closed_loop_poles_imag[32];
    size_t       n_poles;
    int          is_stabilizing;
    int          convergence;
    size_t       iterations;
    double       residual;
} mc_lqr_solution_t;

/* L4: DARE Solution - Discrete LQR */
typedef struct {
    mc_matrix_t  P;
    mc_matrix_t  K;
    double       j_min;
    int          is_stabilizing;
    int          convergence;
    size_t       iterations;
    double       residual;
} mc_dare_solution_t;

/* L3: Eigenvalue Structure */
typedef struct {
    size_t   n_values;
    double  *real_part;
    double  *imag_part;
    double  *magnitude;
    double   spectral_radius;
    mc_stability_t stability;
    double   damping_min;
    double   natural_freq_max;
} mc_eigenvalues_t;

/* L1: Step Response Metrics */
typedef struct {
    double   rise_time;
    double   settling_time;
    double   overshoot_percent;
    double   peak_time;
    double   steady_state_value;
    double   steady_state_error;
    double   dc_gain;
} mc_step_response_t;

typedef enum {
    MC_POLEPLACE_ACKERMANN   = 0,
    MC_POLEPLACE_BASS_GURA   = 1,
    MC_POLEPLACE_EIGENSTRUCT = 2,
    MC_POLEPLACE_LQR         = 3,
    MC_POLEPLACE_ROBUST      = 4
} mc_poleplace_method_t;

typedef enum {
    MC_LQR_SCHUR          = 0,
    MC_LQR_KLEINMAN       = 1,
    MC_LQR_MATRIX_SIGN    = 2,
    MC_LQR_EIGEN          = 3
} mc_lqr_method_t;

/* L6: Inverted Pendulum Parameters */
typedef struct {
    double   cart_mass;
    double   pendulum_mass;
    double   pendulum_len;
    double   gravity;
    double   friction_cart;
    double   friction_pend;
} mc_inverted_pendulum_params_t;

typedef struct {
    double   x_cart;
    double   v_cart;
    double   theta;
    double   omega;
    double   control_force;
} mc_inverted_pendulum_state_t;

/* L6: DC Motor Parameters */
typedef struct {
    double   R_a;
    double   L_a;
    double   K_b;
    double   K_t;
    double   J;
    double   B;
    double   gear_ratio;
} mc_dc_motor_params_t;

typedef struct {
    double   theta;
    double   omega;
    double   current;
    double   voltage;
} mc_dc_motor_state_t;

/* L6: Quadrotor Hover Model Parameters */
typedef struct {
    double   mass;
    double   I_xx;
    double   I_yy;
    double   I_zz;
    double   arm_length;
    double   thrust_coeff;
    double   torque_coeff;
    double   gravity;
} mc_quadrotor_params_t;

/* L5: Model Predictive Control (MPC) Structures */
typedef struct {
    double   u_min;
    double   u_max;
    double   du_max;
} mc_mpc_input_constraint_t;

typedef struct {
    double   x_min;
    double   x_max;
} mc_mpc_state_constraint_t;

typedef struct {
    size_t       horizon;
    mc_matrix_t  Q;
    mc_matrix_t  R;
    mc_matrix_t  Qf;
    mc_mpc_input_constraint_t *u_constraints;
    mc_mpc_state_constraint_t *x_constraints;
    double       *x_ref;
    size_t       max_iter;
    double       tol;
} mc_mpc_config_t;

typedef struct {
    mc_vector_t  u_opt;
    double       cost;
    int          convergence;
    size_t       iterations;
} mc_mpc_solution_t;

/* L2: Kalman Decomposition */
typedef struct {
    size_t      dim_co;
    size_t      dim_cno;
    size_t      dim_nco;
    size_t      dim_ncno;
    mc_matrix_t T;
    int        *partition;
} mc_kalman_decomposition_t;

/* L8: Robustness Margins (State-Space) */
typedef struct {
    double   gain_margin_dB;
    double   phase_margin_deg;
    double   disk_margin;
    double   gain_crossover_freq;
    double   phase_crossover_freq;
    double   delay_margin;
} mc_robustness_margins_t;

/* L8: H-infinity Solution */
typedef struct {
    double      gamma;
    mc_matrix_t X;
    mc_matrix_t Y;
    mc_matrix_t K_c;
    int         feasible;
    double      optimal_gamma;
} mc_hinf_solution_t;

/* L6: Kalman Filter (Linear) */
typedef struct {
    mc_ss_discrete_t  sys;
    mc_matrix_t        Q_kf;
    mc_matrix_t        R_kf;
    mc_vector_t        x_hat;
    mc_matrix_t        P_kf;
    mc_matrix_t        K_kf;
    int                initialized;
} mc_kalman_filter_t;

/* L8: Balanced Truncation Model Reduction */
typedef struct {
    mc_ss_system_t  reduced_sys;
    mc_vector_t     hankel_sv;
    double          error_bound;
    size_t          original_order;
    size_t          reduced_order;
} mc_balanced_truncation_t;
/* ===========================================================
 * Core API Declarations
 * =========================================================== */

/* --- Matrix Operations (L3) --- */
int  mc_matrix_alloc(size_t rows, size_t cols, mc_matrix_t *mat);
void mc_matrix_free(mc_matrix_t *mat);
void mc_matrix_set(mc_matrix_t *mat, size_t i, size_t j, double val);
double mc_matrix_get(const mc_matrix_t *mat, size_t i, size_t j);
void mc_matrix_zero(mc_matrix_t *mat);
void mc_matrix_identity(mc_matrix_t *mat);
void mc_matrix_copy(const mc_matrix_t *src, mc_matrix_t *dst);
void mc_matrix_transpose(const mc_matrix_t *A, mc_matrix_t *AT);
void mc_matrix_add(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C);
void mc_matrix_sub(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C);
void mc_matrix_scale(mc_matrix_t *A, double alpha);
int  mc_matrix_mul(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C);
double mc_matrix_norm_frobenius(const mc_matrix_t *A);
double mc_matrix_norm_inf(const mc_matrix_t *A);
double mc_matrix_trace(const mc_matrix_t *A);
double mc_matrix_det(const mc_matrix_t *A);
int  mc_matrix_inverse(const mc_matrix_t *A, mc_matrix_t *A_inv);
int  mc_matrix_rank(const mc_matrix_t *A, double tol);
int  mc_matrix_is_symmetric(const mc_matrix_t *A, double tol);
int  mc_matrix_is_positive_definite(const mc_matrix_t *A);
int  mc_matrix_solve(const mc_matrix_t *A, const mc_vector_t *b, mc_vector_t *x);
int  mc_matrix_cholesky(const mc_matrix_t *A, mc_matrix_t *L);

/* --- Vector Operations (L3) --- */
int  mc_vector_alloc(size_t length, mc_vector_t *vec);
void mc_vector_free(mc_vector_t *vec);
void mc_vector_zero(mc_vector_t *vec);
void mc_vector_set(mc_vector_t *vec, size_t i, double val);
double mc_vector_get(const mc_vector_t *vec, size_t i);
void mc_vector_copy(const mc_vector_t *src, mc_vector_t *dst);
double mc_vector_dot(const mc_vector_t *a, const mc_vector_t *b);
double mc_vector_norm2(const mc_vector_t *a);
double mc_vector_norm_inf(const mc_vector_t *a);
void mc_vector_scale(mc_vector_t *a, double alpha);
void mc_vector_add(const mc_vector_t *a, const mc_vector_t *b, mc_vector_t *c);
void mc_vector_sub(const mc_vector_t *a, const mc_vector_t *b, mc_vector_t *c);
void mc_matrix_vector_mul(const mc_matrix_t *A, const mc_vector_t *x, mc_vector_t *y);
void mc_vector_lincomb(double alpha, const mc_vector_t *a, double beta, const mc_vector_t *b, mc_vector_t *c);

/* --- System Lifecycle (L1) --- */
int  mc_ss_alloc(size_t n_states, size_t n_inputs, size_t n_outputs, mc_ss_system_t *sys);
void mc_ss_free(mc_ss_system_t *sys);
int  mc_ss_discrete_alloc(size_t n, size_t m, size_t p, mc_ss_discrete_t *sys, double Ts);
void mc_ss_discrete_free(mc_ss_discrete_t *sys);

/* --- Controllability and Observability (L2) --- */
int  mc_controllability_analyze(const mc_ss_system_t *sys, mc_controllability_t *ctrb);
int  mc_controllability_discrete(const mc_ss_discrete_t *sys, mc_controllability_t *ctrb);
void mc_controllability_free(mc_controllability_t *ctrb);
int  mc_observability_analyze(const mc_ss_system_t *sys, mc_observability_t *obsv);
int  mc_observability_discrete(const mc_ss_discrete_t *sys, mc_observability_t *obsv);
void mc_observability_free(mc_observability_t *obsv);
int  mc_controllability_gramian(const mc_ss_system_t *sys, mc_matrix_t *Wc);
int  mc_observability_gramian(const mc_ss_system_t *sys, mc_matrix_t *Wo);

/* --- Eigenvalues (L3) --- */
int  mc_eigenvalues_compute(const mc_matrix_t *A, mc_eigenvalues_t *eig);
void mc_eigenvalues_free(mc_eigenvalues_t *eig);
int  mc_is_hurwitz(const mc_matrix_t *A);
int  mc_is_schur(const mc_matrix_t *A);

/* --- Pole Placement (L5) --- */
int  mc_pole_place_ackermann(const mc_ss_system_t *sys, const double *desired_poles, mc_state_feedback_t *fb);
int  mc_pole_place_acker_discrete(const mc_ss_discrete_t *sys, const double *desired_poles, mc_state_feedback_t *fb);
void mc_state_feedback_free(mc_state_feedback_t *fb);

/* --- LQR (L4-L5) --- */
int  mc_lqr_solve(const mc_matrix_t *A, const mc_matrix_t *B, const mc_matrix_t *Q, const mc_matrix_t *R, mc_lqr_method_t method, mc_lqr_solution_t *sol);
int  mc_lqr_discrete(const mc_matrix_t *A, const mc_matrix_t *B, const mc_matrix_t *Q, const mc_matrix_t *R, mc_dare_solution_t *sol);
void mc_lqr_solution_free(mc_lqr_solution_t *sol);
void mc_dare_solution_free(mc_dare_solution_t *sol);

/* --- Lyapunov Equation (L4) --- */
int  mc_lyapunov_solve(const mc_matrix_t *A, const mc_matrix_t *Q, mc_lyapunov_solution_t *sol);
int  mc_lyapunov_solve_discrete(const mc_matrix_t *A, const mc_matrix_t *Q, mc_lyapunov_solution_t *sol);
void mc_lyapunov_solution_free(mc_lyapunov_solution_t *sol);

/* --- Observer Design (L5) --- */
int  mc_observer_luenberger_design(const mc_ss_system_t *sys, const double *observer_poles, mc_luenberger_observer_t *obs);
int  mc_observer_luenberger_discrete(const mc_ss_discrete_t *sys, const double *observer_poles, mc_luenberger_observer_t *obs);
void mc_luenberger_observer_free(mc_luenberger_observer_t *obs);

/* --- Discretization (L2) --- */
int  mc_c2d(const mc_ss_system_t *cont_sys, mc_ss_discrete_t *disc_sys, double Ts, mc_discretization_t method);
int  mc_d2c(const mc_ss_discrete_t *disc_sys, mc_ss_system_t *cont_sys);

/* --- Simulation (L6) --- */
int  mc_simulate_step(const mc_ss_system_t *sys, const mc_vector_t *x, const mc_vector_t *u, double dt, mc_vector_t *x_next, mc_vector_t *y);
int  mc_simulate_discrete_step(const mc_ss_discrete_t *sys, const mc_vector_t *x, const mc_vector_t *u, mc_vector_t *x_next, mc_vector_t *y);
int  mc_simulate_run(const mc_ss_system_t *sys, double t0, double tf, double dt, const mc_vector_t *x0, double (*u_func)(double t, void *ctx), void *u_ctx, double *t_out, mc_matrix_t *x_traj, int *n_steps);
int  mc_simulate_closed_loop(const mc_ss_system_t *sys, const mc_state_feedback_t *fb, double t0, double tf, double dt, const mc_vector_t *x0, mc_matrix_t *x_traj, int *n_steps);

/* --- Step Response (L6) --- */
int  mc_step_response(const mc_ss_system_t *sys, size_t output_idx, double t_final, double dt, mc_step_response_t *resp);

/* --- Transfer Function Conversions (L3) --- */
int  mc_tf_to_ss_controllable(const mc_tf_t *tf, mc_ss_system_t *sys);
int  mc_tf_to_ss_observable(const mc_tf_t *tf, mc_ss_system_t *sys);
int  mc_ss_to_tf(const mc_ss_system_t *sys, size_t output_idx, size_t input_idx, mc_tf_t *tf);
void mc_tf_free(mc_tf_t *tf);
double mc_tf_evaluate(const mc_tf_t *tf, double s);

/* --- Canonical Forms (L3) --- */
int  mc_to_controllable_canonical(const mc_ss_system_t *sys, mc_ss_system_t *canon);
int  mc_to_observable_canonical(const mc_ss_system_t *sys, mc_ss_system_t *canon);
int  mc_to_jordan_form(const mc_ss_system_t *sys, mc_ss_system_t *jordan);

/* --- Kalman Decomposition (L2) --- */
int  mc_kalman_decomposition(const mc_ss_system_t *sys, mc_kalman_decomposition_t *decomp);
void mc_kalman_decomposition_free(mc_kalman_decomposition_t *decomp);

/* --- Robustness Margins (L8) --- */
int  mc_robustness_margins(const mc_ss_system_t *sys, const mc_state_feedback_t *fb, mc_robustness_margins_t *margins);

/* --- H-infinity Design (L8) --- */
int  mc_hinf_design(const mc_ss_system_t *sys, const mc_matrix_t *B_w, const mc_matrix_t *C_z, double gamma, mc_hinf_solution_t *sol);
void mc_hinf_solution_free(mc_hinf_solution_t *sol);

/* --- System Interconnection (L3) --- */
int  mc_series(const mc_ss_system_t *sys1, const mc_ss_system_t *sys2, mc_ss_system_t *sys_out);
int  mc_parallel(const mc_ss_system_t *sys1, const mc_ss_system_t *sys2, mc_ss_system_t *sys_out);
int  mc_feedback_connect(const mc_ss_system_t *G, const mc_ss_system_t *H, int negative_feedback, mc_ss_system_t *sys_cl);
int  mc_ss_similarity_transform(const mc_ss_system_t *sys, const mc_matrix_t *T, mc_ss_system_t *sys_transformed);

/* --- Model Predictive Control (L5) --- */
int  mc_mpc_step(const mc_ss_discrete_t *sys, const mc_mpc_config_t *cfg, const mc_vector_t *x_current, mc_mpc_solution_t *sol);
void mc_mpc_solution_free(mc_mpc_solution_t *sol);

/* --- Canonical Problem Builders (L6) --- */
int  mc_build_inverted_pendulum(const mc_inverted_pendulum_params_t *params, mc_ss_system_t *sys);
int  mc_build_dc_motor(const mc_dc_motor_params_t *params, mc_ss_system_t *sys);
int  mc_build_quadrotor_hover(const mc_quadrotor_params_t *params, mc_ss_system_t *sys);

/* --- Canonical LQR Design (L6) --- */
int  mc_inverted_pendulum_lqr(const mc_inverted_pendulum_params_t *params, const double *q_weights, const double *r_weights, mc_ss_system_t *sys_out, mc_lqr_solution_t *lqr_out);
int  mc_dc_motor_lqr(const mc_dc_motor_params_t *params, const double *q_weights, const double *r_weights, mc_ss_system_t *sys_out, mc_lqr_solution_t *lqr_out);
int  mc_dc_motor_lqr_position(const mc_dc_motor_params_t *params, const double *q_weights, const double *r_weights, mc_ss_system_t *sys_out, mc_lqr_solution_t *lqr_out);

/* --- Kalman Filter (L6) --- */
int  mc_kalman_filter_init(mc_ss_discrete_t *sys, const mc_matrix_t *Q, const mc_matrix_t *R, const mc_vector_t *x0, const mc_matrix_t *P0, mc_kalman_filter_t *kf);
int  mc_kalman_filter_predict(mc_kalman_filter_t *kf, const mc_vector_t *u);
int  mc_kalman_filter_update(mc_kalman_filter_t *kf, const mc_vector_t *y);
void mc_kalman_filter_free(mc_kalman_filter_t *kf);

/* --- Balanced Truncation (L8) --- */
int  mc_balanced_truncation(const mc_ss_system_t *sys, size_t r, mc_balanced_truncation_t *result);
void mc_balanced_truncation_free(mc_balanced_truncation_t *result);
int  mc_balanced_realization(const mc_ss_system_t *sys, mc_ss_system_t *balanced);

/* --- Minimal Realization (L4) --- */
int  mc_minimal_realization(const mc_ss_system_t *sys, mc_ss_system_t *minimal);

/* --- Utility Functions (L3) --- */
int  mc_matrix_power(const mc_matrix_t *A, size_t power, mc_matrix_t *A_pow);
int  mc_char_poly(const mc_matrix_t *A, double *coeffs);
int  mc_ctrb_matrix(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *Ctrb);
int  mc_obsv_matrix(const mc_matrix_t *A, const mc_matrix_t *C, mc_matrix_t *Obsv);
int  mc_dlyap(const mc_matrix_t *A, const mc_matrix_t *Q, mc_matrix_t *P, size_t max_iter, double tol);
int  mc_blkdiag(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C);
int  mc_kron(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *K);
void mc_poles_to_poly(const double *poles, size_t n_poles, double *coeffs);

#ifdef __cplusplus
}
#endif

#endif /* MODERN_CONTROL_H */
