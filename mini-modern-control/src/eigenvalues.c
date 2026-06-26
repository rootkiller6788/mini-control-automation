#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* L3: Eigenvalue Computation and Stability Analysis
 * QR algorithm with Householder reduction to Hessenberg form.
 * References: Golub & Van Loan (2013), Francis (1961)
 */

static void householder_qr(double *A, double *Q, size_t n)
{
    double *v = (double*)malloc(n * sizeof(double));
    if (!v) return;
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++)
            Q[i * n + j] = (i == j) ? 1.0 : 0.0;
    for (size_t k = 0; k < n - 1; k++) {
        double norm_x = 0.0;
        for (size_t i = k; i < n; i++) norm_x += A[i * n + k] * A[i * n + k];
        if (norm_x < 1e-20) continue;
        double alpha = (A[k * n + k] > 0) ? -sqrt(norm_x) : sqrt(norm_x);
        double r = sqrt(0.5 * (alpha * alpha - A[k * n + k] * alpha));
        v[k] = (A[k * n + k] - alpha) / (2.0 * r);
        for (size_t i = k + 1; i < n; i++) v[i] = A[i * n + k] / (2.0 * r);
        for (size_t j = k; j < n; j++) {
            double dot = 0.0;
            for (size_t i = k; i < n; i++) dot += v[i] * A[i * n + j];
            for (size_t i = k; i < n; i++) A[i * n + j] -= 2.0 * v[i] * dot;
        }
        for (size_t i = 0; i < n; i++) {
            double dot = 0.0;
            for (size_t j = k; j < n; j++) dot += Q[i * n + j] * v[j];
            for (size_t j = k; j < n; j++) Q[i * n + j] -= 2.0 * dot * v[j];
        }
    }
    free(v);
}

static int qr_iteration(double *H, double *Q, size_t n, int max_iter, double tol)
{
    for (int iter = 0; iter < max_iter; iter++) {
        int converged = 1;
        for (size_t i = 0; i < n - 1; i++) {
            if (fabs(H[(i+1) * n + i]) > tol) { converged = 0; break; }
        }
        if (converged) return 0;
        double shift = H[(n-1) * n + (n-1)];
        for (size_t i = 0; i < n; i++) H[i * n + i] -= shift;
        householder_qr(H, Q, n);
        double *R = (double*)malloc(n * n * sizeof(double));
        if (!R) return -1;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) R[i * n + j] = H[i * n + j];
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double sum = 0.0;
                for (size_t k = 0; k < n; k++) sum += R[i * n + k] * Q[k * n + j];
                H[i * n + j] = sum;
            }
        for (size_t i = 0; i < n; i++) H[i * n + i] += shift;
        free(R);
    }
    return -1;
}

int mc_eigenvalues_compute(const mc_matrix_t *A, mc_eigenvalues_t *eig)
{
    if (!A || !eig || !A->data || A->rows != A->cols) return -1;
    size_t n = A->rows;
    if (n == 0 || n > 32) return -2;
    eig->n_values = n;
    eig->real_part = (double*)calloc(n, sizeof(double));
    eig->imag_part = (double*)calloc(n, sizeof(double));
    eig->magnitude = (double*)calloc(n, sizeof(double));
    if (!eig->real_part || !eig->imag_part || !eig->magnitude) {
        mc_eigenvalues_free(eig); return -3;
    }
    double *H = (double*)malloc(n * n * sizeof(double));
    double *Q = (double*)malloc(n * n * sizeof(double));
    if (!H || !Q) { free(H); free(Q); mc_eigenvalues_free(eig); return -3; }
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++) H[i * n + j] = A->data[i * A->stride + j];
    /* Reduce to Hessenberg */
    for (size_t k = 0; k < n - 2; k++) {
        double norm = 0.0;
        for (size_t i = k + 1; i < n; i++) norm += H[i * n + k] * H[i * n + k];
        if (norm < 1e-20) continue;
        double alpha = (H[(k+1) * n + k] > 0) ? -sqrt(norm) : sqrt(norm);
        double v[32];
        v[k+1] = H[(k+1) * n + k] - alpha;
        for (size_t i = k + 2; i < n; i++) v[i] = H[i * n + k];
        double vnorm = 0.0;
        for (size_t i = k + 1; i < n; i++) vnorm += v[i] * v[i];
        for (size_t j = k; j < n; j++) {
            double dot = 0.0;
            for (size_t i = k + 1; i < n; i++) dot += v[i] * H[i * n + j];
            dot *= 2.0 / vnorm;
            for (size_t i = k + 1; i < n; i++) H[i * n + j] -= dot * v[i];
        }
        for (size_t i = 0; i < n; i++) {
            double dot = 0.0;
            for (size_t j = k + 1; j < n; j++) dot += H[i * n + j] * v[j];
            dot *= 2.0 / vnorm;
            for (size_t j = k + 1; j < n; j++) H[i * n + j] -= dot * v[j];
        }
    }
    qr_iteration(H, Q, n, 200, 1e-12);
    /* Extract eigenvalues */
    size_t idx = 0, i = 0;
    while (i < n) {
        if (i < n - 1 && fabs(H[(i+1) * n + i]) > 1e-10) {
            double a = H[i*n+i], b = H[i*n+(i+1)], c = H[(i+1)*n+i], d = H[(i+1)*n+(i+1)];
            double tr = a + d, det = a*d - b*c;
            double disc = tr*tr - 4.0*det;
            eig->real_part[idx] = tr/2.0;
            eig->imag_part[idx] = sqrt(fabs(disc))/2.0;
            eig->real_part[idx+1] = tr/2.0;
            eig->imag_part[idx+1] = -sqrt(fabs(disc))/2.0;
            eig->magnitude[idx] = sqrt(eig->real_part[idx]*eig->real_part[idx] + eig->imag_part[idx]*eig->imag_part[idx]);
            eig->magnitude[idx+1] = eig->magnitude[idx];
            idx += 2; i += 2;
        } else {
            eig->real_part[idx] = H[i*n+i];
            eig->imag_part[idx] = 0.0;
            eig->magnitude[idx] = fabs(H[i*n+i]);
            idx++; i++;
        }
    }
    eig->spectral_radius = 0.0;
    for (size_t k = 0; k < n; k++)
        if (eig->magnitude[k] > eig->spectral_radius) eig->spectral_radius = eig->magnitude[k];
    int all_neg = 1;
    for (size_t k = 0; k < n; k++) if (eig->real_part[k] >= 0.0) all_neg = 0;
    eig->stability = all_neg ? MC_STABLE : MC_UNSTABLE;
    eig->damping_min = 1e100; eig->natural_freq_max = 0.0;
    for (size_t k = 0; k < n; k++) {
        double wn = eig->magnitude[k];
        if (wn > eig->natural_freq_max) eig->natural_freq_max = wn;
        if (wn > 1e-10) { double z = -eig->real_part[k]/wn; if (z < eig->damping_min) eig->damping_min = z; }
    }
    free(H); free(Q);
    return 0;
}

void mc_eigenvalues_free(mc_eigenvalues_t *eig)
{
    if (!eig) return;
    free(eig->real_part); free(eig->imag_part); free(eig->magnitude);
    memset(eig, 0, sizeof(*eig));
}

int mc_is_hurwitz(const mc_matrix_t *A)
{
    if (!A || A->rows != A->cols) return 0;
    size_t n = A->rows;
    mc_matrix_t I_mat;
    if (mc_matrix_alloc(n, n, &I_mat) != 0) return 0;
    mc_matrix_identity(&I_mat);
    mc_lyapunov_solution_t sol;
    int ret = mc_lyapunov_solve(A, &I_mat, &sol);
    mc_matrix_free(&I_mat);
    if (ret == 0 && sol.is_positive_definite) {
        mc_lyapunov_solution_free(&sol); return 1;
    }
    mc_lyapunov_solution_free(&sol);
    return 0;
}

int mc_is_schur(const mc_matrix_t *A)
{
    if (!A || A->rows != A->cols) return 0;
    size_t n = A->rows;
    mc_matrix_t I_mat, P;
    if (mc_matrix_alloc(n, n, &I_mat) != 0) return 0;
    mc_matrix_identity(&I_mat);
    if (mc_matrix_alloc(n, n, &P) != 0) { mc_matrix_free(&I_mat); return 0; }
    int ret = mc_dlyap(A, &I_mat, &P, 1000, 1e-12);
    mc_matrix_free(&I_mat);
    if (ret == 0) {
        int is_pd = mc_matrix_is_positive_definite(&P);
        mc_matrix_free(&P); return is_pd;
    }
    mc_matrix_free(&P);
    return 0;
}

int mc_dlyap(const mc_matrix_t *A, const mc_matrix_t *Q,
              mc_matrix_t *P, size_t max_iter, double tol)
{
    /* Discrete Lyapunov: A^T*P*A - P = -Q, Smith iteration */
    if (!A || !Q || !P || A->rows != A->cols) return -1;
    size_t n = A->rows;
    if (P->rows != n || P->cols != n || Q->rows != n || Q->cols != n) return -2;
    mc_matrix_copy(Q, P);
    mc_matrix_t AT, T1, T2;
    if (mc_matrix_alloc(n, n, &AT) || mc_matrix_alloc(n, n, &T1) || mc_matrix_alloc(n, n, &T2)) {
        mc_matrix_free(&AT); mc_matrix_free(&T1); mc_matrix_free(&T2); return -3;
    }
    mc_matrix_transpose(A, &AT);
    for (size_t iter = 0; iter < max_iter; iter++) {
        mc_matrix_mul(P, A, &T1);
        mc_matrix_mul(&AT, &T1, &T2);
        mc_matrix_add(Q, &T2, &T1);
        double diff = 0.0;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double d = T1.data[i*T1.stride+j] - P->data[i*P->stride+j];
                diff += d*d;
            }
        mc_matrix_copy(&T1, P);
        if (sqrt(diff) < tol) { mc_matrix_free(&AT); mc_matrix_free(&T1); mc_matrix_free(&T2); return 0; }
    }
    mc_matrix_free(&AT); mc_matrix_free(&T1); mc_matrix_free(&T2);
    return -4;
}

int mc_char_poly(const mc_matrix_t *A, double *coeffs)
{
    /* Faddeev-LeVerrier algorithm for characteristic polynomial */
    if (!A || !coeffs || A->rows != A->cols) return -1;
    size_t n = A->rows;
    if (n == 0 || n > 16) return -2;
    mc_matrix_t S, AS, T;
    if (mc_matrix_alloc(n, n, &S) || mc_matrix_alloc(n, n, &AS) || mc_matrix_alloc(n, n, &T)) {
        mc_matrix_free(&S); mc_matrix_free(&AS); mc_matrix_free(&T); return -3;
    }
    mc_matrix_identity(&S);
    for (size_t k = 1; k <= n; k++) {
        mc_matrix_mul(A, &S, &AS);
        double tr = mc_matrix_trace(&AS);
        coeffs[n-k] = -tr / (double)k;
        mc_matrix_copy(&AS, &T);
        for (size_t i = 0; i < n; i++) T.data[i*T.stride+i] += coeffs[n-k];
        mc_matrix_copy(&T, &S);
    }
    mc_matrix_free(&S); mc_matrix_free(&AS); mc_matrix_free(&T);
    return 0;
}
