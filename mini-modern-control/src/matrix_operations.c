#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

/* =========================================================================
 * L3: Mathematical Structures - Matrix Operations
 *
 * All matrices stored in row-major order.
 * Core building block for state-space computations.
 *
 * Matrix multiplication: C = A * B
 *   C[i][j] = sum_k A[i][k] * B[k][j]
 *   Complexity: O(rows_A * cols_A * cols_B)
 *
 * Gaussian elimination (LU decomposition):
 *   Used for matrix inversion and solving Ax = b.
 *   Complexity: O(n^3)
 *
 * Cholesky decomposition: A = L * L^T (for A > 0 symmetric)
 *   Complexity: O(n^3/3), twice as efficient as LU for SPD matrices.
 *
 * References:
 *   Golub & Van Loan, "Matrix Computations", 4th ed. (2013)
 *   Strang, "Linear Algebra and Its Applications", 4th ed. (2006)
 * ========================================================================= */

/* ---------- Matrix Lifecycle ---------- */

int mc_matrix_alloc(size_t rows, size_t cols, mc_matrix_t *mat)
{
    if (!mat || rows == 0 || cols == 0) return -1;
    mat->rows   = rows;
    mat->cols   = cols;
    mat->stride = cols;
    mat->data   = (double*)calloc(rows * cols, sizeof(double));
    mat->owner  = 1;
    if (!mat->data) return -2;
    return 0;
}

void mc_matrix_free(mc_matrix_t *mat)
{
    if (!mat) return;
    if (mat->owner && mat->data) {
        free(mat->data);
        mat->data = NULL;
    }
    mat->rows = mat->cols = mat->stride = 0;
    mat->owner = 0;
}

void mc_matrix_set(mc_matrix_t *mat, size_t i, size_t j, double val)
{
    if (!mat || !mat->data || i >= mat->rows || j >= mat->cols) return;
    mat->data[i * mat->stride + j] = val;
}

double mc_matrix_get(const mc_matrix_t *mat, size_t i, size_t j)
{
    if (!mat || !mat->data || i >= mat->rows || j >= mat->cols) return 0.0;
    return mat->data[i * mat->stride + j];
}

void mc_matrix_zero(mc_matrix_t *mat)
{
    if (!mat || !mat->data) return;
    memset(mat->data, 0, mat->rows * mat->stride * sizeof(double));
}

void mc_matrix_identity(mc_matrix_t *mat)
{
    if (!mat || !mat->data || mat->rows != mat->cols) return;
    mc_matrix_zero(mat);
    for (size_t i = 0; i < mat->rows; i++)
        mat->data[i * mat->stride + i] = 1.0;
}

void mc_matrix_copy(const mc_matrix_t *src, mc_matrix_t *dst)
{
    if (!src || !dst || !src->data || !dst->data) return;
    if (src->rows != dst->rows || src->cols != dst->cols) return;
    for (size_t i = 0; i < src->rows; i++)
        memcpy(&dst->data[i * dst->stride], &src->data[i * src->stride],
               src->cols * sizeof(double));
}

void mc_matrix_transpose(const mc_matrix_t *A, mc_matrix_t *AT)
{
    if (!A || !AT || !A->data || !AT->data) return;
    if (AT->rows != A->cols || AT->cols != A->rows) return;
    for (size_t i = 0; i < A->rows; i++)
        for (size_t j = 0; j < A->cols; j++)
            AT->data[j * AT->stride + i] = A->data[i * A->stride + j];
}

void mc_matrix_add(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C)
{
    if (!A || !B || !C || !A->data || !B->data || !C->data) return;
    if (A->rows != B->rows || A->cols != B->cols) return;
    if (C->rows != A->rows || C->cols != A->cols) return;
    for (size_t i = 0; i < A->rows; i++)
        for (size_t j = 0; j < A->cols; j++)
            C->data[i * C->stride + j] =
                A->data[i * A->stride + j] + B->data[i * B->stride + j];
}

void mc_matrix_sub(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C)
{
    if (!A || !B || !C || !A->data || !B->data || !C->data) return;
    if (A->rows != B->rows || A->cols != B->cols) return;
    if (C->rows != A->rows || C->cols != A->cols) return;
    for (size_t i = 0; i < A->rows; i++)
        for (size_t j = 0; j < A->cols; j++)
            C->data[i * C->stride + j] =
                A->data[i * A->stride + j] - B->data[i * B->stride + j];
}

void mc_matrix_scale(mc_matrix_t *A, double alpha)
{
    if (!A || !A->data) return;
    for (size_t i = 0; i < A->rows; i++)
        for (size_t j = 0; j < A->cols; j++)
            A->data[i * A->stride + j] *= alpha;
}

int mc_matrix_mul(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C)
{
    if (!A || !B || !C || !A->data || !B->data || !C->data) return -1;
    if (A->cols != B->rows) return -2;
    if (C->rows != A->rows || C->cols != B->cols) return -3;
    mc_matrix_zero(C);
    for (size_t i = 0; i < A->rows; i++) {
        for (size_t k = 0; k < A->cols; k++) {
            double aik = A->data[i * A->stride + k];
            if (aik == 0.0) continue;
            for (size_t j = 0; j < B->cols; j++) {
                C->data[i * C->stride + j] += aik * B->data[k * B->stride + j];
            }
        }
    }
    return 0;
}

double mc_matrix_norm_frobenius(const mc_matrix_t *A)
{
    if (!A || !A->data) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < A->rows; i++)
        for (size_t j = 0; j < A->cols; j++) {
            double v = A->data[i * A->stride + j];
            sum += v * v;
        }
    return sqrt(sum);
}

double mc_matrix_norm_inf(const mc_matrix_t *A)
{
    if (!A || !A->data) return 0.0;
    double max_sum = 0.0;
    for (size_t i = 0; i < A->rows; i++) {
        double row_sum = 0.0;
        for (size_t j = 0; j < A->cols; j++)
            row_sum += fabs(A->data[i * A->stride + j]);
        if (row_sum > max_sum) max_sum = row_sum;
    }
    return max_sum;
}

double mc_matrix_trace(const mc_matrix_t *A)
{
    if (!A || !A->data) return 0.0;
    size_t n = (A->rows < A->cols) ? A->rows : A->cols;
    double tr = 0.0;
    for (size_t i = 0; i < n; i++)
        tr += A->data[i * A->stride + i];
    return tr;
}

double mc_matrix_det(const mc_matrix_t *A)
{
    /* LU decomposition-based determinant computation.
     * det(A) = product of diagonal entries of U * (-1)^(row swaps).
     * O(n^3) complexity.
     *
     * Uses Doolittle algorithm: L has unit diagonal.
     */
    if (!A || !A->data || A->rows != A->cols || A->rows == 0) return 0.0;
    size_t n = A->rows;

    /* Allocate working copy */
    double *LU = (double*)malloc(n * n * sizeof(double));
    if (!LU) return 0.0;
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++)
            LU[i * n + j] = A->data[i * A->stride + j];

    double det = 1.0;
    int sign = 1;

    for (size_t k = 0; k < n; k++) {
        /* Find pivot */
        size_t pivot = k;
        double max_val = fabs(LU[k * n + k]);
        for (size_t i = k + 1; i < n; i++) {
            double val = fabs(LU[i * n + k]);
            if (val > max_val) { max_val = val; pivot = i; }
        }
        if (max_val < 1e-15) {
            free(LU);
            return 0.0;  /* Singular matrix */
        }
        /* Swap rows if needed */
        if (pivot != k) {
            for (size_t j = 0; j < n; j++) {
                double tmp = LU[k * n + j];
                LU[k * n + j] = LU[pivot * n + j];
                LU[pivot * n + j] = tmp;
            }
            sign = -sign;
        }
        det *= LU[k * n + k];
        /* Eliminate below */
        for (size_t i = k + 1; i < n; i++) {
            double factor = LU[i * n + k] / LU[k * n + k];
            for (size_t j = k + 1; j < n; j++)
                LU[i * n + j] -= factor * LU[k * n + j];
        }
    }
    free(LU);
    return sign * det;
}

/* ---------- Gaussian Elimination with Partial Pivoting ---------- */

static int gaussian_elimination(double *A, double *b, size_t n, size_t stride)
{
    /* Solves Ax = b in-place using Gaussian elimination with partial pivoting.
     * A is an n x n matrix stored with given stride (stride >= n).
     * b is modified in-place to contain the solution x.
     * Returns 0 on success, -1 if singular.
     *
     * Theorem (Gauss, 1809): A unique solution exists iff det(A) != 0.
     */
    for (size_t k = 0; k < n; k++) {
        /* Partial pivoting: find largest pivot in column k */
        size_t pivot = k;
        double max_val = fabs(A[k * stride + k]);
        for (size_t i = k + 1; i < n; i++) {
            double val = fabs(A[i * stride + k]);
            if (val > max_val) { max_val = val; pivot = i; }
        }
        if (max_val < 1e-15) return -1;  /* Singular */

        /* Swap rows */
        if (pivot != k) {
            for (size_t j = k; j < n; j++) {
                double tmp = A[k * stride + j];
                A[k * stride + j] = A[pivot * stride + j];
                A[pivot * stride + j] = tmp;
            }
            double tmp = b[k]; b[k] = b[pivot]; b[pivot] = tmp;
        }

        /* Eliminate below */
        double pivot_val = A[k * stride + k];
        for (size_t i = k + 1; i < n; i++) {
            double factor = A[i * stride + k] / pivot_val;
            for (size_t j = k + 1; j < n; j++)
                A[i * stride + j] -= factor * A[k * stride + j];
            b[i] -= factor * b[k];
        }
    }

    /* Back substitution */
    for (size_t i = n; i-- > 0; ) {
        double sum = b[i];
        for (size_t j = i + 1; j < n; j++)
            sum -= A[i * stride + j] * b[j];
        b[i] = sum / A[i * stride + i];
    }
    return 0;
}

int mc_matrix_solve(const mc_matrix_t *A, const mc_vector_t *b, mc_vector_t *x)
{
    if (!A || !b || !x || !A->data || !b->data || !x->data) return -1;
    if (A->rows != A->cols || A->rows != b->length || b->length != x->length)
        return -2;

    size_t n = A->rows;
    /* Copy A and b for in-place solve */
    double *A_copy = (double*)malloc(n * n * sizeof(double));
    if (!A_copy) return -3;
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++)
            A_copy[i * n + j] = A->data[i * A->stride + j];
    for (size_t i = 0; i < n; i++)
        x->data[i] = b->data[i];

    int ret = gaussian_elimination(A_copy, x->data, n, n);
    free(A_copy);
    return ret;
}

int mc_matrix_inverse(const mc_matrix_t *A, mc_matrix_t *A_inv)
{
    /* Compute inverse using Gauss-Jordan elimination.
     * Augment [A | I], reduce to [I | A^{-1}].
     * O(n^3) complexity.
     */
    if (!A || !A_inv || !A->data || !A_inv->data) return -1;
    if (A->rows != A->cols || A_inv->rows != A->rows || A_inv->cols != A->cols)
        return -2;

    size_t n = A->rows;
    /* Allocate augmented matrix [A | I] as n x 2n */
    double *aug = (double*)calloc(n * (2 * n), sizeof(double));
    if (!aug) return -3;

    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++)
            aug[i * (2 * n) + j] = A->data[i * A->stride + j];
        aug[i * (2 * n) + n + i] = 1.0;  /* Identity on right */
    }

    /* Forward elimination with partial pivoting */
    for (size_t k = 0; k < n; k++) {
        size_t pivot = k;
        double max_val = fabs(aug[k * (2 * n) + k]);
        for (size_t i = k + 1; i < n; i++) {
            double val = fabs(aug[i * (2 * n) + k]);
            if (val > max_val) { max_val = val; pivot = i; }
        }
        if (max_val < 1e-15) { free(aug); return -4; }

        if (pivot != k) {
            for (size_t j = 0; j < 2 * n; j++) {
                double tmp = aug[k * (2 * n) + j];
                aug[k * (2 * n) + j] = aug[pivot * (2 * n) + j];
                aug[pivot * (2 * n) + j] = tmp;
            }
        }

        double pivot_val = aug[k * (2 * n) + k];
        for (size_t j = k; j < 2 * n; j++)
            aug[k * (2 * n) + j] /= pivot_val;

        for (size_t i = 0; i < n; i++) {
            if (i == k) continue;
            double factor = aug[i * (2 * n) + k];
            for (size_t j = k; j < 2 * n; j++)
                aug[i * (2 * n) + j] -= factor * aug[k * (2 * n) + j];
        }
    }

    /* Extract inverse from right half */
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++)
            A_inv->data[i * A_inv->stride + j] = aug[i * (2 * n) + n + j];

    free(aug);
    return 0;
}

int mc_matrix_rank(const mc_matrix_t *A, double tol)
{
    /* Rank via Gaussian elimination (numerical rank).
     * Count non-zero pivots after elimination.
     * Rank is invariant under elementary row operations (Strang, 2006).
     */
    if (!A || !A->data) return -1;
    size_t m = A->rows, n = A->cols;
    size_t min_mn = (m < n) ? m : n;

    /* Working copy */
    double *U = (double*)malloc(m * n * sizeof(double));
    if (!U) return -1;
    for (size_t i = 0; i < m; i++)
        for (size_t j = 0; j < n; j++)
            U[i * n + j] = A->data[i * A->stride + j];

    if (tol <= 0.0) tol = 1e-10;

    size_t rank = 0;
    size_t row = 0;
    for (size_t col = 0; col < n && row < m; col++) {
        /* Find pivot */
        size_t pivot = row;
        double max_val = fabs(U[row * n + col]);
        for (size_t i = row + 1; i < m; i++) {
            double val = fabs(U[i * n + col]);
            if (val > max_val) { max_val = val; pivot = i; }
        }
        if (max_val <= tol) continue;

        /* Swap */
        if (pivot != row) {
            for (size_t j = col; j < n; j++) {
                double tmp = U[row * n + j];
                U[row * n + j] = U[pivot * n + j];
                U[pivot * n + j] = tmp;
            }
        }

        /* Eliminate below */
        double pivot_val = U[row * n + col];
        for (size_t i = row + 1; i < m; i++) {
            double factor = U[i * n + col] / pivot_val;
            for (size_t j = col; j < n; j++)
                U[i * n + j] -= factor * U[row * n + j];
        }
        rank++;
        row++;
    }

    free(U);
    return (int)rank;
}

int mc_matrix_is_symmetric(const mc_matrix_t *A, double tol)
{
    if (!A || !A->data || A->rows != A->cols) return 0;
    if (tol <= 0.0) tol = 1e-12;
    for (size_t i = 0; i < A->rows; i++)
        for (size_t j = i + 1; j < A->cols; j++)
            if (fabs(A->data[i * A->stride + j] - A->data[j * A->stride + i]) > tol)
                return 0;
    return 1;
}

int mc_matrix_is_positive_definite(const mc_matrix_t *A)
{
    /* Attempt Cholesky decomposition.
     * Theorem: A > 0 iff Cholesky exists (no zero/negative pivots).
     * Sylvester's criterion: all leading principal minors > 0.
     */
    if (!A || !A->data || A->rows != A->cols) return 0;
    size_t n = A->rows;

    double *L = (double*)calloc(n * n, sizeof(double));
    if (!L) return 0;

    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j <= i; j++) {
            double sum = 0.0;
            for (size_t k = 0; k < j; k++)
                sum += L[i * n + k] * L[j * n + k];
            if (i == j) {
                double diag = A->data[i * A->stride + i] - sum;
                if (diag <= 1e-15) { free(L); return 0; }
                L[i * n + i] = sqrt(diag);
            } else {
                L[i * n + j] = (A->data[i * A->stride + j] - sum) / L[j * n + j];
            }
        }
    }
    free(L);
    return 1;
}

int mc_matrix_cholesky(const mc_matrix_t *A, mc_matrix_t *L)
{
    /* Cholesky decomposition: A = L * L^T
     * Requires A symmetric positive definite.
     * L is lower triangular with positive diagonal.
     * Complexity: O(n^3/3)
     *
     * Theorem (Cholesky, 1924): If A is SPD, unique L exists with L_ii > 0.
     */
    if (!A || !L || !A->data || !L->data) return -1;
    if (A->rows != A->cols || L->rows != A->rows || L->cols != A->cols) return -2;
    size_t n = A->rows;

    mc_matrix_zero(L);
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j <= i; j++) {
            double sum = 0.0;
            for (size_t k = 0; k < j; k++)
                sum += L->data[i * L->stride + k] * L->data[j * L->stride + k];
            if (i == j) {
                double diag = A->data[i * A->stride + i] - sum;
                if (diag <= 1e-15) return -3;  /* Not SPD */
                L->data[i * L->stride + i] = sqrt(diag);
            } else {
                L->data[i * L->stride + j] =
                    (A->data[i * A->stride + j] - sum) / L->data[j * L->stride + j];
            }
        }
    }
    return 0;
}

/* ---------- Power Method for Spectral Radius ---------- */

static double power_method(const mc_matrix_t *A, size_t max_iter, double tol)
{
    /* Power iteration: v_{k+1} = A*v_k / ||A*v_k||.
     * Converges to dominant eigenvalue for non-defective matrices.
     * Rayleigh quotient: lambda = v^T * A * v / (v^T * v).
     */
    if (!A || A->rows != A->cols || A->rows == 0) return 0.0;
    size_t n = A->rows;

    double *v = (double*)malloc(n * sizeof(double));
    double *Av = (double*)malloc(n * sizeof(double));
    if (!v || !Av) { free(v); free(Av); return 0.0; }

    /* Initialize v to all ones */
    for (size_t i = 0; i < n; i++) v[i] = 1.0;

    double lambda = 0.0;
    for (size_t iter = 0; iter < max_iter; iter++) {
        /* v = A * v */
        for (size_t i = 0; i < n; i++) {
            Av[i] = 0.0;
            for (size_t j = 0; j < n; j++)
                Av[i] += A->data[i * A->stride + j] * v[j];
        }
        /* Normalize */
        double norm = 0.0;
        for (size_t i = 0; i < n; i++) norm += Av[i] * Av[i];
        if (norm < 1e-20) break;
        norm = sqrt(norm);
        double lambda_new = 0.0;
        for (size_t i = 0; i < n; i++) {
            v[i] = Av[i] / norm;
            double dot = 0.0;
            for (size_t j = 0; j < n; j++)
                dot += A->data[i * A->stride + j] * v[j];
            lambda_new += v[i] * dot;
        }
        if (fabs(lambda_new - lambda) < tol) {
            lambda = lambda_new;
            break;
        }
        lambda = lambda_new;
    }

    free(v); free(Av);
    return lambda;
}

double mc_matrix_power_iteration(const double *A_data, size_t n, size_t stride,
                                  size_t max_iter, double tol)
{
    /* Internal helper: power iteration on raw matrix data.
     * Used by eigenvalue computation.
     */
    (void)A_data; (void)n; (void)stride; (void)max_iter; (void)tol;
    return 0.0;  /* Placeholder; full implementation in eigenvalues.c */
}

/* ---------- Kronecker Product ---------- */

int mc_kron(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *K)
{
    /* Kronecker product: K = A (x) B
     * K is (A->rows * B->rows) x (A->cols * B->cols).
     * Block structure: K_{ik,jl} = A_{ij} * B_{kl}
     *
     * Used in solving Lyapunov equations via vectorization.
     * Key identity: vec(A*X*B) = (B^T (x) A) * vec(X)
     */
    if (!A || !B || !K || !A->data || !B->data || !K->data) return -1;
    size_t rA = A->rows, cA = A->cols;
    size_t rB = B->rows, cB = B->cols;
    if (K->rows != rA * rB || K->cols != cA * cB) return -2;

    for (size_t i = 0; i < rA; i++)
        for (size_t j = 0; j < cA; j++) {
            double a_ij = A->data[i * A->stride + j];
            for (size_t k = 0; k < rB; k++)
                for (size_t l = 0; l < cB; l++)
                    K->data[(i * rB + k) * K->stride + (j * cB + l)] =
                        a_ij * B->data[k * B->stride + l];
        }
    return 0;
}

int mc_blkdiag(const mc_matrix_t *A, const mc_matrix_t *B, mc_matrix_t *C)
{
    /* Block diagonal: C = diag(A, B)
     * C = [A  0]
     *     [0  B]
     */
    if (!A || !B || !C || !A->data || !B->data || !C->data) return -1;
    size_t rA = A->rows, cA = A->cols;
    size_t rB = B->rows, cB = B->cols;
    if (C->rows != rA + rB || C->cols != cA + cB) return -2;

    mc_matrix_zero(C);
    /* Copy A into top-left */
    for (size_t i = 0; i < rA; i++)
        for (size_t j = 0; j < cA; j++)
            C->data[i * C->stride + j] = A->data[i * A->stride + j];
    /* Copy B into bottom-right */
    for (size_t i = 0; i < rB; i++)
        for (size_t j = 0; j < cB; j++)
            C->data[(rA + i) * C->stride + (cA + j)] = B->data[i * B->stride + j];
    return 0;
}

int mc_matrix_power(const mc_matrix_t *A, size_t power, mc_matrix_t *A_pow)
{
    /* Compute A^power using binary exponentiation.
     * Complexity: O(log(power) * n^3) for matrix multiplication.
     *
     * Used in Ackermann's formula: p(A) = A^n + a_{n-1}*A^{n-1} + ... + a_0*I
     * where p(lambda) = det(lambda*I - A) is the characteristic polynomial.
     */
    if (!A || !A_pow || !A->data || !A_pow->data) return -1;
    if (A->rows != A->cols || A_pow->rows != A->rows || A_pow->cols != A->cols)
        return -2;

    size_t n = A->rows;
    /* Start with identity */
    mc_matrix_identity(A_pow);

    if (power == 0) return 0;

    /* Binary exponentiation */
    mc_matrix_t base;
    mc_matrix_t temp;
    if (mc_matrix_alloc(n, n, &base) != 0) return -3;
    if (mc_matrix_alloc(n, n, &temp) != 0) {
        mc_matrix_free(&base);
        return -3;
    }
    mc_matrix_copy(A, &base);

    size_t p = power;
    while (p > 0) {
        if (p & 1) {
            mc_matrix_mul(A_pow, &base, &temp);
            mc_matrix_copy(&temp, A_pow);
        }
        p >>= 1;
        if (p > 0) {
            mc_matrix_mul(&base, &base, &temp);
            mc_matrix_copy(&temp, &base);
        }
    }

    mc_matrix_free(&base);
    mc_matrix_free(&temp);
    return 0;
}

/* ---------- Vector Operations ---------- */

int mc_vector_alloc(size_t length, mc_vector_t *vec)
{
    if (!vec || length == 0) return -1;
    vec->length = length;
    vec->data = (double*)calloc(length, sizeof(double));
    vec->owner = 1;
    if (!vec->data) return -2;
    return 0;
}

void mc_vector_free(mc_vector_t *vec)
{
    if (!vec) return;
    if (vec->owner && vec->data) {
        free(vec->data);
        vec->data = NULL;
    }
    vec->length = 0;
    vec->owner = 0;
}

void mc_vector_zero(mc_vector_t *vec)
{
    if (!vec || !vec->data) return;
    memset(vec->data, 0, vec->length * sizeof(double));
}

void mc_vector_set(mc_vector_t *vec, size_t i, double val)
{
    if (!vec || !vec->data || i >= vec->length) return;
    vec->data[i] = val;
}

double mc_vector_get(const mc_vector_t *vec, size_t i)
{
    if (!vec || !vec->data || i >= vec->length) return 0.0;
    return vec->data[i];
}

void mc_vector_copy(const mc_vector_t *src, mc_vector_t *dst)
{
    if (!src || !dst || !src->data || !dst->data) return;
    if (src->length != dst->length) return;
    memcpy(dst->data, src->data, src->length * sizeof(double));
}

double mc_vector_dot(const mc_vector_t *a, const mc_vector_t *b)
{
    if (!a || !b || !a->data || !b->data || a->length != b->length) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < a->length; i++)
        sum += a->data[i] * b->data[i];
    return sum;
}

double mc_vector_norm2(const mc_vector_t *a)
{
    if (!a || !a->data) return 0.0;
    return sqrt(mc_vector_dot(a, a));
}

double mc_vector_norm_inf(const mc_vector_t *a)
{
    if (!a || !a->data) return 0.0;
    double max_val = 0.0;
    for (size_t i = 0; i < a->length; i++) {
        double abs_val = fabs(a->data[i]);
        if (abs_val > max_val) max_val = abs_val;
    }
    return max_val;
}

void mc_vector_scale(mc_vector_t *a, double alpha)
{
    if (!a || !a->data) return;
    for (size_t i = 0; i < a->length; i++)
        a->data[i] *= alpha;
}

void mc_vector_add(const mc_vector_t *a, const mc_vector_t *b, mc_vector_t *c)
{
    if (!a || !b || !c || !a->data || !b->data || !c->data) return;
    if (a->length != b->length || b->length != c->length) return;
    for (size_t i = 0; i < a->length; i++)
        c->data[i] = a->data[i] + b->data[i];
}

void mc_vector_sub(const mc_vector_t *a, const mc_vector_t *b, mc_vector_t *c)
{
    if (!a || !b || !c || !a->data || !b->data || !c->data) return;
    if (a->length != b->length || b->length != c->length) return;
    for (size_t i = 0; i < a->length; i++)
        c->data[i] = a->data[i] - b->data[i];
}

void mc_matrix_vector_mul(const mc_matrix_t *A, const mc_vector_t *x, mc_vector_t *y)
{
    /* y = A * x
     * For state-space: x_dot = A*x, y_out = C*x
     */
    if (!A || !x || !y || !A->data || !x->data || !y->data) return;
    if (A->cols != x->length || A->rows != y->length) return;
    for (size_t i = 0; i < A->rows; i++) {
        double sum = 0.0;
        for (size_t j = 0; j < A->cols; j++)
            sum += A->data[i * A->stride + j] * x->data[j];
        y->data[i] = sum;
    }
}

void mc_vector_lincomb(double alpha, const mc_vector_t *a,
                        double beta, const mc_vector_t *b, mc_vector_t *c)
{
    /* c = alpha*a + beta*b
     * Fundamental BLAS-1 operation (axpy variant).
     */
    if (!a || !b || !c || !a->data || !b->data || !c->data) return;
    if (a->length != b->length || b->length != c->length) return;
    for (size_t i = 0; i < a->length; i++)
        c->data[i] = alpha * a->data[i] + beta * b->data[i];
}
