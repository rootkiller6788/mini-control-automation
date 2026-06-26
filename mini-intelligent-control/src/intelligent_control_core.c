#define _USE_MATH_DEFINES
#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* intelligent_control_core.c - L1:L6 */

/* Logistic sigmoid: 1/(1+e^{-x}) */
double ic_sigmoid(double x) {
    if (x > 20.0) return 1.0;
    if (x < -20.0) return 0.0;
    return 1.0 / (1.0 + exp(-x));
}

/* Sigmoid derivative: s*(1-s) */
double ic_sigmoid_derivative(double x) {
    double s = ic_sigmoid(x); return s * (1.0 - s);
}

/* Hyperbolic tangent */
double ic_tanh_custom(double x) {
    return tanh(x);
}

/* Tanh derivative: 1-tanh^2 */
double ic_tanh_derivative(double x) {
    double t = tanh(x); return 1.0 - t * t;
}

/* ReLU: max(0,x) */
double ic_relu(double x) {
    return x > 0.0 ? x : 0.0;
}

/* ReLU derivative */
double ic_relu_derivative(double x) {
    return x > 0.0 ? 1.0 : 0.0;
}

/* Leaky ReLU */
double ic_leaky_relu(double x, double alpha) {
    return x > 0.0 ? x : alpha * x;
}

/* Leaky ReLU derivative */
double ic_leaky_relu_derivative(double x, double alpha) {
    return x > 0.0 ? 1.0 : alpha;
}

/* Linear activation */
double ic_linear(double x) {
    return x;
}

/* Linear derivative */
double ic_linear_derivative(double x) {
    (void)x; return 1.0;
}

/* Softmax: stable via max-subtraction */
void ic_softmax(const double *x, size_t n, double *out) {
    if (n == 0) return;
    double max_val = x[0]; size_t i;
    for (i = 1; i < n; i++) if (x[i] > max_val) max_val = x[i];
    double sum = 0.0;
    for (i = 0; i < n; i++) { out[i] = exp(x[i] - max_val); sum += out[i]; }
    if (sum > 0.0) for (i = 0; i < n; i++) out[i] /= sum;
}

/* Matrix-vector multiply y=A*x, O(mn) */
void ic_matrix_vector_mul(const double *A, const double *x, size_t m, size_t n, double *y) {
    size_t i, j;
    for (i = 0; i < m; i++) { double acc = 0.0; for (j = 0; j < n; j++) acc += A[i * n + j] * x[j]; y[i] = acc; }
}

/* Matrix transpose AT=A^T, O(mn) */
void ic_matrix_transpose(const double *A, size_t m, size_t n, double *AT) {
    size_t i, j;
    for (i = 0; i < m; i++) for (j = 0; j < n; j++) AT[j * m + i] = A[i * n + j];
}

/* Cholesky-based inverse, SPD only, O(n^3/3) */
int ic_matrix_inverse_cholesky(const double *A, size_t n, double *Ainv) {
    size_t i, j, k, col; double *L = (double*)calloc(n * n, sizeof(double)); if (!L) return -1;
    for (i = 0; i < n; i++) { for (j = 0; j <= i; j++) { double sum = A[i * n + j]; for (k = 0; k < j; k++) sum -= L[i * n + k] * L[j * n + k];
        if (i == j) { if (sum <= 0.0) { free(L); return -1; } L[i * n + i] = sqrt(sum); } else { L[i * n + j] = sum / L[j * n + j]; } } }
    for (col = 0; col < n; col++) { double *y = (double*)calloc(n, sizeof(double)); if (!y) { free(L); return -1; }
        for (i = 0; i < n; i++) { double sum = (i == col) ? 1.0 : 0.0; for (j = 0; j < i; j++) sum -= L[i * n + j] * y[j]; y[i] = sum / L[i * n + i]; }
        for (i = n; i > 0; i--) { size_t ri = i - 1; double sum = y[ri]; for (j = ri + 1; j < n; j++) sum -= L[j * n + ri] * Ainv[col * n + j]; Ainv[col * n + ri] = sum / L[ri * n + ri]; }
        free(y); }
    free(L); return 0;
}

/* Gaussian elimination with partial pivoting, O(n^3) */
int ic_solve_linear_system(const double *A, const double *b, size_t n, double *x) {
    size_t i, j, col, row; if (n == 0) return -1;
    double *aug = (double*)malloc(n * (n + 1) * sizeof(double)); if (!aug) return -1;
    for (i = 0; i < n; i++) { for (j = 0; j < n; j++) aug[i*(n+1)+j] = A[i*n+j]; aug[i*(n+1)+n] = b[i]; }
    for (col = 0; col < n; col++) { size_t max_row = col; double max_val = fabs(aug[col*(n+1)+col]);
        for (row=col+1; row<n; row++) { double v = fabs(aug[row*(n+1)+col]); if (v>max_val) { max_val=v; max_row=row; } }
        if (max_val < 1e-15) { free(aug); return -1; }
        if (max_row != col) for (j=0; j<=n; j++) { double t=aug[col*(n+1)+j]; aug[col*(n+1)+j]=aug[max_row*(n+1)+j]; aug[max_row*(n+1)+j]=t; }
        for (row=col+1; row<n; row++) { double f = aug[row*(n+1)+col]/aug[col*(n+1)+col]; for (j=col; j<=n; j++) aug[row*(n+1)+j] -= f*aug[col*(n+1)+j]; } }
    for (i=n; i>0; i--) { size_t ri=i-1; double sum=aug[ri*(n+1)+n]; for (j=ri+1; j<n; j++) sum -= aug[ri*(n+1)+j]*x[j]; x[ri] = sum/aug[ri*(n+1)+ri]; }
    free(aug); return 0;
}

/* Vector add c=a+b */
void ic_vector_add(const double *a, const double *b, size_t n, double *c) {
    size_t i; for (i=0; i<n; i++) c[i]=a[i]+b[i];
}

/* Vector sub c=a-b */
void ic_vector_sub(const double *a, const double *b, size_t n, double *c) {
    size_t i; for (i=0; i<n; i++) c[i]=a[i]-b[i];
}

/* Dot product a^T*b */
double ic_vector_dot(const double *a, const double *b, size_t n) {
    double s=0.0; size_t i; for (i=0; i<n; i++) s+=a[i]*b[i]; return s;
}

/* L2 norm ||a||_2 */
double ic_vector_norm(const double *a, size_t n) {
    return sqrt(ic_vector_dot(a,a,n));
}

/* Vector scale a*=alpha */
void ic_vector_scale(double *a, double alpha, size_t n) {
    size_t i; for (i=0; i<n; i++) a[i]*=alpha;
}

double ic_activation(ic_activation_t type, double x) {
    switch (type) {
    case IC_ACT_SIGMOID: return ic_sigmoid(x);
    case IC_ACT_TANH: return ic_tanh_custom(x);
    case IC_ACT_RELU: return ic_relu(x);
    case IC_ACT_LEAKY_RELU: return ic_leaky_relu(x, 0.01);
    case IC_ACT_LINEAR: return ic_linear(x);
    case IC_ACT_GAUSSIAN: return exp(-x * x);
    default: return x;
    }
}

double ic_activation_derivative(ic_activation_t type, double x) {
    switch (type) {
    case IC_ACT_SIGMOID: return ic_sigmoid_derivative(x);
    case IC_ACT_TANH: return ic_tanh_derivative(x);
    case IC_ACT_RELU: return ic_relu_derivative(x);
    case IC_ACT_LEAKY_RELU: return ic_leaky_relu_derivative(x, 0.01);
    case IC_ACT_LINEAR: return ic_linear_derivative(x);
    case IC_ACT_GAUSSIAN: return -2.0 * x * exp(-x * x);
    default: return 1.0;
    }
}
#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* intelligent_control_core_extra.c - Lyapunov, RNG, Performance */


/* Lyapunov, RNG, Performance, Convergence */

uint64_t ic_rng_state = 123456789ULL;

double ic_lyapunov_quadratic(const double *x, const double *P, size_t n) {
    double *Px = (double*)malloc(n * sizeof(double));
    if (!Px) return INFINITY;
    ic_matrix_vector_mul(P, x, n, n, Px);
    double result = ic_vector_dot(x, Px, n);
    free(Px);
    return result;
}

int ic_lyapunov_solve(const double *A, size_t n, double *P) {
    size_t i, j, k;
    if (n == 0 || n > 64) return -1;
    size_t n2 = n * n;
    double *M = (double*)calloc(n2 * n2, sizeof(double));
    double *q = (double*)calloc(n2, sizeof(double));
    double *pvec = (double*)calloc(n2, sizeof(double));
    if (!M || !q || !pvec) { free(M); free(q); free(pvec); return -1; }
    for (i = 0; i < n; i++) q[i * n + i] = -1.0;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            size_t row = i * n + j;
            for (k = 0; k < n; k++) {
                M[row * n2 + i * n + k] += A[k * n + j];
                M[row * n2 + k * n + j] += A[k * n + i];
            }
        }
    }
    int ret = ic_solve_linear_system(M, q, n2, pvec);
    if (ret == 0) {
        for (i = 0; i < n2; i++) P[i] = -pvec[i];
    }
    free(M); free(q); free(pvec);
    return ret;
}

void ic_random_seed(uint64_t seed) {
    ic_rng_state = seed ? seed : 123456789ULL;
}

double ic_random_uniform(void) {
    ic_rng_state ^= ic_rng_state >> 12;
    ic_rng_state ^= ic_rng_state << 25;
    ic_rng_state ^= ic_rng_state >> 27;
    return (double)(ic_rng_state * 0x2545F4914F6CDD1DULL) / 18446744073709551616.0;
}

double ic_random_gaussian(void) {
    double u1 = ic_random_uniform();
    double u2 = ic_random_uniform();
    if (u1 < 1e-15) u1 = 1e-15;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

void ic_evaluate_performance(const double *setpoint, const double *output,
                              const double *control, size_t n,
                              double dt, ic_performance_t *perf) {
    size_t i, j;
    if (n < 2) return;
    memset(perf, 0, sizeof(ic_performance_t));
    double final_val = output[n - 1];
    perf->steady_state_error = fabs(setpoint[n - 1] - final_val);
    for (i = 0; i < n; i++) {
        double e = setpoint[i] - output[i];
        double ae = fabs(e);
        perf->ise += e * e * dt;
        perf->iae += ae * dt;
        perf->itae += (double)i * dt * ae * dt;
    }
    double y_final = output[n - 1];
    double t_lo = -1.0, t_hi = -1.0;
    for (i = 1; i < n; i++) {
        if (t_lo < 0.0 && output[i] >= 0.1 * y_final) t_lo = (double)i * dt;
        if (t_hi < 0.0 && output[i] >= 0.9 * y_final) t_hi = (double)i * dt;
    }
    if (t_lo >= 0.0 && t_hi >= 0.0) perf->rise_time = t_hi - t_lo;
    for (i = 0; i < n; i++) {
        int settled = 1;
        for (j = i; j < n; j++) {
            if (fabs(output[j] - y_final) > 0.02 * fabs(y_final) + 1e-9) {
                settled = 0; break;
            }
        }
        if (settled) { perf->settling_time = (double)i * dt; break; }
    }
    double y_max = output[0];
    for (i = 1; i < n; i++) { if (output[i] > y_max) y_max = output[i]; }
    if (fabs(y_final) > 1e-9) {
        perf->overshoot = 100.0 * (y_max - y_final) / fabs(y_final);
        if (perf->overshoot < 0.0) perf->overshoot = 0.0;
    }
    for (i = 0; i < n; i++) perf->control_effort_rms += control[i] * control[i];
    perf->control_effort_rms = sqrt(perf->control_effort_rms / (double)n);
    perf->robustness_margin = 1.0 / (1.0 + perf->overshoot / 100.0);
}

int ic_check_convergence(const double *error_sequence, size_t n,
                          double tol, ic_convergence_t *conv) {
    size_t i, offset, window;
    if (n < 10) return 0;
    memset(conv, 0, sizeof(ic_convergence_t));
    conv->curve_length = n < 100 ? n : 100;
    offset = n - conv->curve_length;
    for (i = 0; i < conv->curve_length; i++)
        conv->learning_curve[i] = fabs(error_sequence[offset + i]);
    window = 10 < conv->curve_length ? 10 : conv->curve_length;
    double avg_end = 0.0;
    for (i = 0; i < window; i++)
        avg_end += conv->learning_curve[conv->curve_length - 1 - i];
    avg_end /= (double)window;
    conv->asymptotic_error = avg_end;
    conv->converged = (avg_end < tol) ? 1 : 0;
    if (conv->converged) {
        for (i = 1; i < conv->curve_length; i++) {
            if (fabs(error_sequence[i]) < tol) {
                conv->convergence_samples = i + offset; break;
            }
        }
        double initial = fabs(error_sequence[offset]);
        if (initial > 1e-9 && conv->convergence_samples > 0) {
            conv->convergence_rate = -log(conv->asymptotic_error / initial)
                                     / (double)conv->convergence_samples;
            if (conv->convergence_rate < 0.0) conv->convergence_rate = 0.0;
        }
    }
    return conv->converged;
}

int ic_system_init(ic_strategy_t strategy, void *controller) {
    if (!controller) return -1;
    (void)strategy;
    return 0;
}

void ic_system_free(ic_strategy_t strategy, void *controller) {
    (void)strategy;
    (void)controller;
}
