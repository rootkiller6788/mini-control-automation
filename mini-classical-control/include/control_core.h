/**
 * control_core.h — Classical Control Theory: Core Data Structures & Operations
 *
 * Covers L1-L3: transfer functions, state-space models, system specifications,
 * Laplace domain algebra, pole-zero analysis, and model conversions.
 *
 * References:
 *   - K. Ogata, "Modern Control Engineering", 5th ed., Prentice Hall, 2010.
 *   - R.C. Dorf & R.H. Bishop, "Modern Control Systems", 13th ed., Pearson, 2017.
 *   - G.F. Franklin, J.D. Powell & A. Emami-Naeini,
 *     "Feedback Control of Dynamic Systems", 8th ed., Pearson, 2019.
 */

#ifndef CONTROL_CORE_H
#define CONTROL_CORE_H

#include <stddef.h>
#include <complex.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTRL_MAX_ORDER    32

/* ─── Transfer function: G(s) = num(s) / den(s) ─── */
typedef struct {
    int      num_order;
    int      den_order;
    double   num[CTRL_MAX_ORDER + 1];
    double   den[CTRL_MAX_ORDER + 1];
    double   gain;
} transfer_function_t;

/* ─── State-space model (continuous): dx=A·x+B·u, y=C·x+D·u ─── */
typedef struct {
    int     n, m, p;
    double *A, *B, *C, *D;
} state_space_t;

/* ─── Pole-zero description ─── */
typedef struct {
    int      num_poles, num_zeros;
    double complex poles[CTRL_MAX_ORDER];
    double complex zeros[CTRL_MAX_ORDER];
    double   dc_gain;
} pole_zero_t;

/* ─── Time-domain step response specifications ─── */
typedef struct {
    double   rise_time;
    double   settling_time;
    double   peak_time;
    double   overshoot_pct;
    double   steady_state_err;
    double   delay_time;
    double   final_value;
    double   damping_ratio;
    double   natural_freq;
} step_specs_t;

/* ─── Frequency-domain specifications ─── */
typedef struct {
    double   gain_margin_db;
    double   phase_margin_deg;
    double   gain_crossover;
    double   phase_crossover;
    double   bandwidth;
    double   dc_gain_db;
    double   resonant_peak_db;
    double   resonant_freq;
} freq_specs_t;

/* ─── PID Controller parameters ─── */
typedef struct {
    double   Kp, Ki, Kd;
    double   N, Tf;
    double   b, c;
    double   Ts;
} pid_params_t;

/* ─── System type classification ─── */
typedef enum {
    SYSTEM_TYPE_0 = 0,
    SYSTEM_TYPE_1 = 1,
    SYSTEM_TYPE_2 = 2,
    SYSTEM_TYPE_3 = 3,
} system_type_t;

/* ─── L2: Core Analysis Operations ─── */
system_type_t tf_system_type(const transfer_function_t *G);
int compute_step_specs(double omega_n, double zeta, step_specs_t *specs);
double complex tf_evaluate(const transfer_function_t *G, double complex s);
int tf_freq_response(const transfer_function_t *G, double omega,
                     double *mag_db, double *phase_deg);

/* ─── L3: Algebra & Conversions ─── */
int tf_series(const transfer_function_t *G1, const transfer_function_t *G2,
              transfer_function_t *result);
int tf_parallel(const transfer_function_t *G1, const transfer_function_t *G2,
                transfer_function_t *result);
int tf_unity_feedback(const transfer_function_t *G, transfer_function_t *T);
int tf_feedback(const transfer_function_t *G, const transfer_function_t *H,
                transfer_function_t *T);
int tf2ss(const transfer_function_t *G, state_space_t *ss);
int ss2tf(const state_space_t *ss, transfer_function_t *G);
int tf_find_poles(const transfer_function_t *G, pole_zero_t *pz);
int tf_find_zeros(const transfer_function_t *G, pole_zero_t *pz);
int tf_partial_fraction(const transfer_function_t *G,
                        double complex residues[],
                        double complex roots[],
                        int *num_terms);
int controllability_matrix(const state_space_t *ss, double *Cmat, int *rank);
int observability_matrix(const state_space_t *ss, double *Omat, int *rank);
int matrix_rank(const double *A, int rows, int cols, double tol);
int tf_init(transfer_function_t *G, const double *num, int num_order,
             const double *den, int den_order);
void tf_normalize(transfer_function_t *G);
double complex poly_eval(const double *coeff, int order, double complex s);
int poly_mul(const double *a, int oa, const double *b, int ob,
             double *c, int *oc);
int poly_add(const double *a, int oa, const double *b, int ob,
             double *c, int *oc);
int tf_first_order(transfer_function_t *G, double K, double tau);
int tf_second_order(transfer_function_t *G, double K, double omega_n, double zeta);
int tf_integrator(transfer_function_t *G);
int tf_pade_delay(double Td, transfer_function_t *G);
void ss_free(state_space_t *ss);
int ss_init(state_space_t *ss, int n, int m, int p);
int tf_snprint(char *buf, size_t size, const transfer_function_t *G);
int tf_is_proper(const transfer_function_t *G);
int tf_is_strictly_proper(const transfer_function_t *G);
double tf_dc_gain(const transfer_function_t *G);
void tf_scale(transfer_function_t *G, double factor);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_CORE_H */
