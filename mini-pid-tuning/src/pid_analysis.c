/**
 * @file pid_analysis.c
 * @brief PID Stability and Performance Analysis Implementation
 *
 * Implements:
 *   L3 -- Frequency response analysis, Bode plot computation
 *   L4 -- Routh-Hurwitz stability criterion, Lyapunov stability analysis,
 *         gain/phase margin computation, sensitivity functions
 *   L6 -- Step response simulation and performance metrics
 *
 * References:
 *   Bode (1945), "Network Analysis and Feedback Amplifier Design"
 *   Routh (1877), "A Treatise on the Stability of a Given State of Motion"
 *   Nyquist (1932), "Regeneration Theory"
 *   Astrom & Murray (2020), "Feedback Systems: An Introduction for Scientists
 *     and Engineers", 2nd Edition, Princeton University Press
 */

#include "pid_analysis.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * L3 -- Frequency Analysis
 *===========================================================================*/

int pid_loop_frequency_analysis(const PIDTransferFunction *pid_tf,
                                const FOPDTModel *model,
                                double f_min, double f_max, size_t n_points,
                                FrequencyAnalysis *analysis) {
    if (!pid_tf || !model || !analysis || n_points < 2) return -1;
    if (f_min <= 0.0 || f_max <= f_min) return -1;

    analysis->freq = (double*)malloc(n_points * sizeof(double));
    analysis->magnitude = (double*)malloc(n_points * sizeof(double));
    analysis->phase = (double*)malloc(n_points * sizeof(double));

    if (!analysis->freq || !analysis->magnitude || !analysis->phase) {
        free(analysis->freq);
        free(analysis->magnitude);
        free(analysis->phase);
        return -1;
    }

    analysis->N = n_points;

    /* Logarithmically spaced frequencies */
    double log_min = log10(f_min);
    double log_max = log10(f_max);
    double dlog = (log_max - log_min) / (n_points - 1);

    for (size_t i = 0; i < n_points; i++) {
        double w = pow(10.0, log_min + dlog * i);

        /* PID frequency response */
        double pid_mag, pid_phase;
        pid_frequency_response(pid_tf, w, &pid_mag, &pid_phase);

        /* Process (FOPDT) frequency response:
         * Gp(jw) = K * exp(-j*w*L) / (1 + j*w*T)
         * |Gp| = K / sqrt(1 + (w*T)^2)
         * phase = -w*L - atan(w*T)
         */
        double proc_mag = model->K / sqrt(1.0 + (w * model->T) * (w * model->T));
        double proc_phase = -w * model->L - atan(w * model->T);

        /* Loop: L(jw) = Gc(jw) * Gp(jw)
         * |L| = |Gc| * |Gp|
         * phase(L) = phase(Gc) + phase(Gp)
         */
        analysis->freq[i] = w;
        analysis->magnitude[i] = pid_mag * proc_mag;
        analysis->phase[i] = pid_phase + proc_phase;

        /* Normalize phase to [-pi, pi] */
        while (analysis->phase[i] > M_PI)
            analysis->phase[i] -= 2.0 * M_PI;
        while (analysis->phase[i] < -M_PI)
            analysis->phase[i] += 2.0 * M_PI;
    }

    /* Init margin fields */
    analysis->gain_margin = 1e30;
    analysis->phase_margin = 1e30;
    analysis->gain_crossover = 0.0;
    analysis->phase_crossover = 0.0;
    analysis->bandwidth = 0.0;

    return 0;
}

int pid_compute_stability_margins(FrequencyAnalysis *analysis) {
    if (!analysis || !analysis->magnitude || !analysis->phase) return -1;

    double gm_linear = 1e30;
    double pm = 1e30;
    double w_pc = 0.0;  /* phase crossover frequency */
    double w_gc = 0.0;  /* gain crossover frequency */

    /* Find phase crossover: phase = -pi (or +pi for positive frequencies looking at negative crossing) */
    for (size_t i = 1; i < analysis->N; i++) {
        /* Look for phase crossing -pi (from above to below) */
        if (analysis->phase[i - 1] >= -M_PI && analysis->phase[i] < -M_PI) {
            /* Interpolate for more accurate crossover */
            double frac = (-M_PI - analysis->phase[i - 1]) /
                          (analysis->phase[i] - analysis->phase[i - 1] + 1e-30);
            w_pc = analysis->freq[i - 1] + frac * (analysis->freq[i] - analysis->freq[i - 1]);

            /* Interpolate magnitude at crossover */
            double mag_pc = analysis->magnitude[i - 1] +
                            frac * (analysis->magnitude[i] - analysis->magnitude[i - 1]);
            gm_linear = 1.0 / (mag_pc + 1e-30);
            break;
        }
        /* Also check crossing in opposite direction (wrapping) */
        if (analysis->phase[i - 1] >= M_PI - 0.1 && analysis->phase[i] <= -M_PI + 0.1) {
            w_pc = analysis->freq[i];
            double mag_pc = analysis->magnitude[i];
            gm_linear = 1.0 / (mag_pc + 1e-30);
            break;
        }
    }

    /* Find gain crossover: |L| = 1 (0 dB) */
    for (size_t i = 1; i < analysis->N; i++) {
        if ((analysis->magnitude[i - 1] - 1.0) * (analysis->magnitude[i] - 1.0) <= 0.0) {
            /* Interpolate */
            double m0 = analysis->magnitude[i - 1];
            double m1 = analysis->magnitude[i];
            double frac;
            if (fabs(m1 - m0) > 1e-30) {
                frac = (1.0 - m0) / (m1 - m0);
            } else {
                frac = 0.5;
            }
            w_gc = analysis->freq[i - 1] + frac * (analysis->freq[i] - analysis->freq[i - 1]);

            /* Interpolate phase at gain crossover */
            double phase_gc = analysis->phase[i - 1] +
                              frac * (analysis->phase[i] - analysis->phase[i - 1]);
            /* Phase margin: distance from -pi */
            pm = phase_gc + M_PI;
            break;
        }
    }

    /* Compute closed-loop bandwidth: frequency where |S(jw)| = 1/sqrt(2) ~ -3dB */
    /* S = 1/(1+L), we want |1+L| = sqrt(2). Solve for w where magnitude drops.
     * For PID systems, bandwidth approx = gain crossover frequency.
     * More precisely: find w where |T(jw)| = 1/sqrt(2). */
    double bw = w_gc; /* approximate */
    if (w_gc > 0.0) {
        for (size_t i = 0; i < analysis->N; i++) {
            /* Compute T = L/(1+L) */
            double Lmag = analysis->magnitude[i];
            double Lphase = analysis->phase[i];
            /* |1+L|^2 = 1 + Lmag^2 + 2*Lmag*cos(Lphase) */
            double one_plus_L_mag2 = 1.0 + Lmag * Lmag + 2.0 * Lmag * cos(Lphase);
            if (one_plus_L_mag2 < 1e-30) continue;
            double Tmag2 = (Lmag * Lmag) / one_plus_L_mag2;
            if (Tmag2 <= 0.5) { /* -3dB point = 0.5 in magnitude-squared */
                bw = analysis->freq[i];
                break;
            }
        }
    }

    /* Gain margin in dB */
    double gm_db = 20.0 * log10(gm_linear + 1e-300);

    analysis->gain_margin = gm_db;
    analysis->phase_margin = pm;
    analysis->gain_crossover = w_gc;
    analysis->phase_crossover = w_pc;
    analysis->bandwidth = bw;

    return 0;
}

void pid_compute_sensitivity(const double *loop_mag, const double *loop_phase,
                             size_t N, double *S_mag, double *T_mag) {
    if (!loop_mag || !loop_phase || !S_mag || !T_mag) return;

    for (size_t i = 0; i < N; i++) {
        double Lmag = loop_mag[i];
        double Lphase = loop_phase[i];

        /* 1 + L = 1 + Lmag*cos(phase) + j*Lmag*sin(phase) */
        double re = 1.0 + Lmag * cos(Lphase);
        double im = Lmag * sin(Lphase);
        double denom2 = re * re + im * im;
        if (denom2 < 1e-60) denom2 = 1e-60;

        /* S = 1/(1+L), |S| = 1/|1+L| */
        S_mag[i] = 1.0 / sqrt(denom2);

        /* T = L/(1+L) */
        T_mag[i] = Lmag / sqrt(denom2);
    }
}

/*===========================================================================
 * L4 -- Routh-Hurwitz Stability Criterion
 *===========================================================================*/

int pid_routh_construct(const Polynomial *poly, RouthArray *ra) {
    if (!poly || !ra || poly->order < 1 || poly->order > 10) return -1;

    memset(ra, 0, sizeof(*ra));
    ra->order = poly->order;
    int n = poly->order;

    /* Allocate rows. Routh array has (n+1) rows.
     * Each row has floor((col_remaining+1)/2) elements. */
    int max_cols = (n + 2) / 2;
    for (int i = 0; i <= n; i++) {
        ra->rows[i] = (double*)calloc(max_cols, sizeof(double));
        if (!ra->rows[i]) {
            /* Free already allocated rows */
            for (int j = 0; j < i; j++) free(ra->rows[j]);
            return -1;
        }
    }

    /* Fill first two rows from polynomial coefficients.
     * Row 0: a_n, a_{n-2}, a_{n-4}, ...
     * Row 1: a_{n-1}, a_{n-3}, a_{n-5}, ...
     * Coefficients are stored in poly->coeffs as [a_0, a_1, ..., a_n].
     * So a_n = coeffs[n], a_{n-1} = coeffs[n-1], etc.
     */
    int col0 = 0, col1 = 0;
    for (int i = n; i >= 0; i -= 2) {
        ra->rows[0][col0++] = poly->coeffs[i];
    }
    for (int i = n - 1; i >= 0; i -= 2) {
        ra->rows[1][col1++] = poly->coeffs[i];
    }

    /* Compute remaining rows */
    for (int i = 2; i <= n; i++) {
        int cols = (n - i + 3) / 2;
        for (int j = 0; j < cols; j++) {
            double a = ra->rows[i - 2][0];
            double b = ra->rows[i - 2][j + 1];
            double c = ra->rows[i - 1][0];
            double d = ra->rows[i - 1][j + 1];

            /* Entry = (c*b - a*d) / c, but need to handle c=0 */
            /* Routh's formula: entry = -(a*d - c*b) / c */
            if (fabs(c) < 1e-30) {
                /* Replace zero with a small epsilon times sign of next row.
                 * This is the epsilon method for Routh array.
                 * For stability analysis, the sign tells us the truth. */
                c = 1e-12;
            }
            ra->rows[i][j] = (c * b - a * d) / c;
        }
    }

    /* Count sign changes in first column */
    ra->sign_changes = 0;
    for (int i = 1; i <= n; i++) {
        double prev = ra->rows[i - 1][0];
        double curr = ra->rows[i][0];

        /* Skip near-zero entries (use epsilon method interpretation) */
        if (fabs(curr) < 1e-30) {
            /* A zero in the first column indicates marginal stability or instability.
             * Count this as a sign change for conservatism. */
            ra->sign_changes++;
            continue;
        }

        if (prev * curr < 0.0) {
            ra->sign_changes++;
        }
    }

    return 0;
}

int pid_routh_is_stable(const RouthArray *ra) {
    if (!ra) return -1;

    /* Stable iff all elements in first column are positive (or all negative).
     * 0 sign changes means all elements have the same sign. */
    if (ra->sign_changes == 0 && ra->rows[0][0] != 0.0) {
        return 1; /* stable */
    }
    return 0; /* unstable or marginally stable */
}

void pid_routh_free(RouthArray *ra) {
    if (!ra) return;
    for (int i = 0; i <= ra->order; i++) {
        free(ra->rows[i]);
    }
    memset(ra, 0, sizeof(*ra));
}

/*===========================================================================
 * L4 -- Lyapunov Stability
 *===========================================================================*/

void pid_lyapunov_system_matrix(double a0, double a1, double b0,
                                double Kp, double Ki, double Kd, double A[9]) {
    /* State: x = [y, dy/dt, integral_e]^T
     *
     * Process: y'' + a1*y' + a0*y = b0*u
     *
     * PID: u = Kp*(r-y) + Ki*integral(r-y) + Kd*(r'-y')
     * For Lyapunov analysis, we consider r=0 (regulation).
     * u = -Kp*y - Ki*integral_y - Kd*y'
     *
     * x1' = y' = x2
     * x2' = -a0*y - a1*y' + b0*u
     *      = -a0*x1 - a1*x2 + b0*(-Kp*x1 - Ki*x3 - Kd*x2)
     *      = -(a0 + b0*Kp)*x1 - (a1 + b0*Kd)*x2 - b0*Ki*x3
     * x3' = y = x1  (integral of y, note: sign depends on convention)
     *
     * Actually, integral term: x3 = integral(y dt), so x3' = y = x1.
     * The integrator state gives: u_I = -Ki * x3
     *
     * A = [[0,      1,       0],
     *      [-(a0+b0Kp), -(a1+b0Kd), -b0Ki],
     *      [1,      0,       0]]
     *
     * Note: The sign convention for integral state depends on whether
     * integral accumulates positive or negative error. For regulation
     * (r=0), error = -y, so integral accumulates -y. Let's use:
     * x3' = -y = -x1, and u_I = Ki * x3 (in the standard convention).
     *
     * Revised: x3 = integral(e dt) = integral(-y dt) = -integral(y dt)
     * u = Kp*(-y) + Ki*x3 + Kd*(-y')
     *   = -Kp*x1 + Ki*x3 - Kd*x2
     *
     * x1' = x2
     * x2' = -a0*x1 - a1*x2 + b0*(-Kp*x1 + Ki*x3 - Kd*x2)
     *     = -(a0 + b0*Kp)*x1 - (a1 + b0*Kd)*x2 + b0*Ki*x3
     * x3' = -x1
     *
     * A = [[0,        1,       0],
     *      [-(a0+b0Kp), -(a1+b0Kd), b0Ki],
     *      [-1,       0,       0]]
     */
    memset(A, 0, 9 * sizeof(double));

    A[0] = 0.0;          A[1] = 1.0;          A[2] = 0.0;
    A[3] = -(a0 + b0 * Kp); A[4] = -(a1 + b0 * Kd); A[5] = b0 * Ki;
    A[6] = -1.0;         A[7] = 0.0;          A[8] = 0.0;
}

int pid_solve_lyapunov(const double *A, const double *Q, double *P, int n) {
    /* Solve A'*P + P*A = -Q (continuous Lyapunov equation)
     *
     * For small n (n <= 3), use Kronecker product approach:
     * (I ? A' + A' ? I) * vec(P) = -vec(Q)
     *
     * vec(P) is column-major vectorization of P.
     * The Kronecker sum K = I ? A' + A' ? I has size n^2 x n^2.
     *
     * For n=3: K is 9x9. We'll solve directly.
     */
    if (!A || !Q || !P || n < 1 || n > 3) return -1;

    int n2 = n * n;

    /* Build the linear system K * vec(P) = -vec(Q)
     * K = kron(I, A') + kron(A', I)
     *
     * kron(I, A')[i][j] = A'[i/n][j/n] if i%n == j%n else 0
     * kron(A', I)[i][j] = A'[i%n][j%n] if i/n == j/n else 0
     */
    double *K = (double*)calloc(n2 * n2, sizeof(double));
    double *rhs = (double*)calloc(n2, sizeof(double));
    if (!K || !rhs) {
        free(K); free(rhs);
        return -1;
    }

    /* Fill K and rhs */
    for (int i = 0; i < n2; i++) {
        int r_i = i / n;  /* row in P */
        int c_i = i % n;  /* col in P */

        /* kron(I, A'): A'[r_i][r_j] * I[c_i][c_j]
         * For element (i, j) in K, we have j = r_j*n + c_j.
         * kron(I, A') contributes A'[r_i][r_j] when c_i == c_j.
         */
        for (int r_j = 0; r_j < n; r_j++) {
            int j = r_j * n + c_i;  /* c_j == c_i */
            K[i * n2 + j] += A[r_i * n + r_j];  /* A'[r_i][r_j] = A[j][i] stored as A[rj][ri]???
                                                     A'[r][c] = A[c][r]. So A'[r_i][r_j] = A[r_j][r_i]. */
            /* Wait, let's re-derive.
             * A is stored row-major: A[row*n + col].
             * A' means we need A'[r_i][r_j] = A[r_j][r_i].
             * In our K indexing: K[i*n2 + j], where i = row of K, j = col of K.
             * i corresponds to (r_i, c_i) in P: i = r_i*n + c_i.
             * j corresponds to (r_j, c_j) in P: j = r_j*n + c_j.
             *
             * kron(I, A'): C = I ? A'. For P reshaped as vec(P) column-major:
             *   kron(I, A') maps: (I)_{c_i,c_j} * (A')_{r_i,r_j}
             *   So K[i][j] += (I)_{c_i,c_j} * A'_{r_i,r_j}
             *   (I)_{c_i,c_j} = 1 if c_i == c_j, else 0.
             *   So K[i][r_j*n + c_i] += A'_{r_i,r_j} = A[r_j][r_i].
             *
             * kron(A', I): C = A' ? I.
             *   K[i][j] += (A')_{c_i,c_j} * (I)_{r_i,r_j}
             *   (I)_{r_i,r_j} = 1 if r_i == r_j, else 0.
             *   So K[i][r_i*n + c_j] += A'_{c_i,c_j} = A[c_j][c_i].
             *
             * Combined:
             *   K[i][j] = (I)_{c_i,c_j} * A'_{r_i,r_j} + (A')_{c_i,c_j} * (I)_{r_i,r_j}
             *           = delta(c_i,c_j) * A[r_j,r_i] + A[c_j,c_i] * delta(r_i,r_j)
             */
        }
    }

    /* Let me redo this more carefully with explicit loops */
    memset(K, 0, n2 * n2 * sizeof(double));
    for (int r_i = 0; r_i < n; r_i++) {
        for (int c_i = 0; c_i < n; c_i++) {
            int i = r_i * n + c_i;

            for (int r_j = 0; r_j < n; r_j++) {
                for (int c_j = 0; c_j < n; c_j++) {
                    int j = r_j * n + c_j;

                    /* kron(I_n, A'): I[c_i][c_j] * A'[r_i][r_j] */
                    if (c_i == c_j) {
                        K[i * n2 + j] += A[r_j * n + r_i]; /* A'[r_i][r_j] = A[r_j][r_i] */
                    }

                    /* kron(A', I_n): A'[c_i][c_j] * I[r_i][r_j] */
                    if (r_i == r_j) {
                        K[i * n2 + j] += A[c_j * n + c_i]; /* A'[c_i][c_j] = A[c_j][c_i] */
                    }
                }
            }

            /* RHS: -vec(Q) */
            rhs[i] = -Q[r_i * n + c_i];
        }
    }

    /* Solve 9x9 system via Gaussian elimination with partial pivoting */
    /* Augmented matrix [K | rhs] */
    int N = n2;
    for (int col = 0; col < N; col++) {
        /* Find pivot */
        int max_row = col;
        double max_val = fabs(K[col * N + col]);
        for (int row = col + 1; row < N; row++) {
            double val = fabs(K[row * N + col]);
            if (val > max_val) {
                max_val = val;
                max_row = row;
            }
        }

        if (max_val < 1e-30) {
            free(K); free(rhs);
            return -1; /* singular */
        }

        /* Swap rows */
        if (max_row != col) {
            for (int k = 0; k < N; k++) {
                double tmp = K[col * N + k];
                K[col * N + k] = K[max_row * N + k];
                K[max_row * N + k] = tmp;
            }
            double tmp = rhs[col];
            rhs[col] = rhs[max_row];
            rhs[max_row] = tmp;
        }

        /* Eliminate */
        double pivot = K[col * N + col];
        rhs[col] /= pivot;
        for (int k = N - 1; k >= col; k--) {
            K[col * N + k] /= pivot;
        }

        for (int row = 0; row < N; row++) {
            if (row != col) {
                double factor = K[row * N + col];
                rhs[row] -= factor * rhs[col];
                for (int k = col; k < N; k++) {
                    K[row * N + k] -= factor * K[col * N + k];
                }
            }
        }
    }

    /* Extract vec(P) into P matrix (row-major) */
    for (int i = 0; i < n2; i++) {
        P[i] = rhs[i];
    }

    free(K); free(rhs);
    return 0;
}

int pid_is_positive_definite(const double *M, int n) {
    if (!M || n < 1) return 0;

    /* Cholesky decomposition: M = L * L^T
     * L[j][j] = sqrt(M[j][j] - sum_{k=0}^{j-1} L[j][k]^2)
     * L[i][j] = (M[i][j] - sum_{k=0}^{j-1} L[i][k]*L[j][k]) / L[j][j]  for i>j
     *
     * M is positive definite iff all diagonal elements are computable (>0 before sqrt).
     */

    double *L = (double*)malloc(n * n * sizeof(double));
    if (!L) return 0;
    memset(L, 0, n * n * sizeof(double));

    for (int j = 0; j < n; j++) {
        /* Compute L[j][j] */
        double sum = 0.0;
        for (int k = 0; k < j; k++) {
            sum += L[j * n + k] * L[j * n + k];
        }
        double diag = M[j * n + j] - sum;
        if (diag <= 0.0) {
            free(L);
            return 0; /* not positive definite */
        }
        L[j * n + j] = sqrt(diag);

        /* Compute L[i][j] for i > j */
        for (int i = j + 1; i < n; i++) {
            sum = 0.0;
            for (int k = 0; k < j; k++) {
                sum += L[i * n + k] * L[j * n + k];
            }
            L[i * n + j] = (M[i * n + j] - sum) / L[j * n + j];
        }
    }

    free(L);
    return 1;
}

/*===========================================================================
 * L6 -- Step Response Simulation
 *===========================================================================*/

int pid_simulate_step_response(const PIDController *pid, const FOPDTModel *model,
                               double setpoint, double duration, double Ts,
                               double *time, double *output, double *control,
                               size_t N) {
    if (!pid || !model || !time || !output || !control || N < 2) return -1;

    /* Create a copy of the PID for simulation (don't modify the original) */
    PIDController sim_pid;
    memcpy(&sim_pid, pid, sizeof(sim_pid));
    sim_pid.params.Ts = Ts;
    pid_reset(&sim_pid);

    /* Dead-time handling with ring buffer */
    size_t delay_steps = (size_t)(model->L / Ts + 0.5);
    if (delay_steps < 1) delay_steps = 1;
    double *delay_buf = (double*)calloc(delay_steps + 1, sizeof(double));
    if (!delay_buf) return -1;
    size_t delay_idx = 0;
    size_t delay_count = 0;

    /* Initial conditions */
    double y = 0.0; /* process output */
    /* For proper simulation, we need to integrate the FOPDT ODE.
     * dy/dt = (K * u(t-L) - y) / T
     * Using forward Euler: y(k+1) = y(k) + Ts * (K*u_delayed - y(k)) / T
     */

    for (size_t k = 0; k < N; k++) {
        time[k] = k * Ts;

        /* PID update */
        double u = pid_update(&sim_pid, setpoint, y);
        control[k] = u;

        /* Store in delay buffer */
        delay_buf[delay_idx] = u;
        delay_idx = (delay_idx + 1) % delay_steps;
        if (delay_count < delay_steps) delay_count++;

        /* Get delayed input */
        double u_delayed;
        if (delay_count >= delay_steps) {
            u_delayed = delay_buf[delay_idx];
        } else {
            /* Not enough history yet */
            u_delayed = 0.0;
        }

        /* Process dynamics: Euler integration */
        double dy = (model->K * u_delayed - y) / model->T * Ts;
        y += dy;

        output[k] = y;
    }

    free(delay_buf);
    return 0;
}

/*===========================================================================
 * L6 -- Step Response Performance Metrics
 *===========================================================================*/

int pid_step_metrics(const double *time, const double *output, size_t N,
                     double setpoint, StepResponseMetrics *metrics) {
    if (!time || !output || !metrics || N < 3) return -1;

    memset(metrics, 0, sizeof(*metrics));

    double y_final = output[N - 1];
    double dy = y_final - output[0]; /* total change */

    /* Steady-state error */
    metrics->steady_state_error = setpoint - y_final;

    /* Find 10%, 50%, 90% of final value */
    double y10 = output[0] + 0.10 * dy;
    double y50 = output[0] + 0.50 * dy;
    double y90 = output[0] + 0.90 * dy;

    double t10 = time[0], t50 = time[0], t90 = time[N - 1];

    /* Scan for crossing times */
    for (size_t i = 1; i < N; i++) {
        if (t10 == time[0] && output[i] >= y10) {
            double frac = (y10 - output[i - 1]) / (output[i] - output[i - 1] + 1e-30);
            t10 = time[i - 1] + frac * (time[i] - time[i - 1]);
        }
        if (t50 == time[0] && output[i] >= y50) {
            double frac = (y50 - output[i - 1]) / (output[i] - output[i - 1] + 1e-30);
            t50 = time[i - 1] + frac * (time[i] - time[i - 1]);
            if (t10 == time[0]) t10 = t50; /* fallback */
        }
        if (output[i] >= y90) {
            double frac = (y90 - output[i - 1]) / (output[i] - output[i - 1] + 1e-30);
            t90 = time[i - 1] + frac * (time[i] - time[i - 1]);
            break;
        }
    }

    metrics->rise_time = t90 - t10;
    metrics->delay_time = t50 - time[0];

    /* Peak time and overshoot */
    double peak_val = output[0];
    size_t peak_idx = 0;
    for (size_t i = 1; i < N; i++) {
        if (output[i] > peak_val) {
            peak_val = output[i];
            peak_idx = i;
        }
    }
    metrics->peak_time = time[peak_idx];
    metrics->overshoot = (peak_val - y_final) / (dy + 1e-30);

    /* Settling time: time to stay within 2% band */
    double band_lo = y_final - 0.02 * fabs(dy);
    double band_hi = y_final + 0.02 * fabs(dy);
    metrics->settling_time = time[N - 1];
    for (size_t i = N - 1; i > 0; i--) {
        if (output[i] < band_lo || output[i] > band_hi) {
            metrics->settling_time = time[i];
            break;
        }
    }

    /* Decay ratio: second peak to first peak */
    double second_peak = output[0];
    int found_first_peak = 0;
    for (size_t i = peak_idx + 1; i < N - 1; i++) {
        if (!found_first_peak && output[i] <= output[i + 1]) continue;
        if (!found_first_peak) {
            found_first_peak = 1; /* passed first trough */
            continue;
        }
        if (output[i] >= output[i - 1] && output[i] >= output[i + 1]) {
            second_peak = output[i];
            break;
        }
    }
    if (peak_val > output[0] + 1e-6) {
        metrics->decay_ratio = (second_peak - y_final) / (peak_val - y_final + 1e-30);
    }

    /* Error integrals (trapezoidal integration) */
    double IAE = 0.0, ISE = 0.0, ITAE = 0.0, ITSE = 0.0;
    for (size_t i = 1; i < N; i++) {
        double dt = time[i] - time[i - 1];
        double e0 = setpoint - output[i - 1];
        double e1 = setpoint - output[i];
        double e_avg = 0.5 * (fabs(e0) + fabs(e1));
        double e_sq_avg = 0.5 * (e0 * e0 + e1 * e1);
        double t_avg = 0.5 * (time[i - 1] + time[i]);

        IAE += e_avg * dt;
        ISE += e_sq_avg * dt;
        ITAE += t_avg * e_avg * dt;
        ITSE += t_avg * e_sq_avg * dt;
    }
    metrics->IAE = IAE;
    metrics->ISE = ISE;
    metrics->ITAE = ITAE;
    metrics->ITSE = ITSE;

    return 0;
}

/*===========================================================================
 * L4 -- Stability Region
 *===========================================================================*/

int pid_stability_region(const FOPDTModel *model, double Kd,
                         double *Kp, double *Ki, size_t n) {
    /* Compute the stability boundary in (Kp, Ki) plane for fixed Kd.
     *
     * Using first-order Pade approximation for delay:
     * exp(-L*s) approx (1 - 0.5*L*s) / (1 + 0.5*L*s)
     *
     * The closed-loop characteristic polynomial:
     * 1 + Gc(s)*Gp(s) = 0
     *
     * With PID (parallel): Gc = Kp + Ki/s + Kd*s
     * FOPDT with Pade: Gp = K*(1 - 0.5*L*s) / ((T*s+1)*(1+0.5*L*s))
     *
     * Characteristic equation: (T*s+1)*(1+0.5*L*s)*s + K*(1-0.5*L*s)*(Kd*s^2 + Kp*s + Ki) = 0
     *
     * Routh-Hurwitz gives the stability region.
     *
     * This is a simplified version that computes the boundary by solving
     * for Kp at which the system goes unstable for each Ki value.
     *
     * For a proper implementation, we would solve the Routh-Hurwitz inequalities.
     * Here we provide an approximate stability boundary based on the
     * well-known result: for FOPDT with PID, the stable region is bounded by:
     *   Ki < Kp/T (for integral stability)
     *   Kp < (T + L)/(K*L) (approximate proportional limit)
     */
    if (!model || !Kp || !Ki || n < 2) return -1;

    double K = model->K, T = model->T, L = model->L;
    if (K <= 0.0 || T <= 0.0) return -1;

    /* Compute maximum Kp for stability (proportional-only limit)
     * For FOPDT: Kp_max = T/(K*L) (from ultimate gain approximation) */
    double Kp_max = T / (K * L);
    if (Kp_max < 0.0 || Kp_max > 1e6) Kp_max = 100.0;

    /* Compute maximum Ki for stability
     * For PI control: Ki_max approx = Kp_max / T */
    double Ki_max = Kp_max / T;
    if (Ki_max < 0.0 || Ki_max > 1e6) Ki_max = 10.0;

    /* Generate stability boundary: ellipse-like curve in (Kp, Ki) plane */
    for (size_t i = 0; i < n; i++) {
        double theta = (double)i / (n - 1) * M_PI / 2.0; /* 0 to pi/2 */
        Kp[i] = Kp_max * cos(theta);
        Ki[i] = Ki_max * sin(theta);
    }

    return 0;
}
