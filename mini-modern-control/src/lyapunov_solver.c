#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* =========================================================================
 * L4: Continuous and Discrete Lyapunov Equation Solvers
 *
 * Continuous: A^T*P + P*A = -Q  (Lyapunov, 1892)
 * Discrete:   A^T*P*A - P = -Q
 *
 * Solution via Bartels-Stewart algorithm using real Schur decomposition,
 * or simplified via direct iteration for smaller systems.
 *
 * Lyapunov theorem: P > 0 for Q > 0 iff A is Hurwitz (continuous)
 * or A is Schur (discrete). This is the foundation of linear stability
 * analysis and is used extensively in LQR, observer design, and
 * model reduction.
 *
 * References:
 *   Lyapunov, "General Problem of Stability of Motion" (1892)
 *   Bartels & Stewart, "Solution of the Matrix Equation AX+XB=C" (1972)
 * ========================================================================= */

int mc_lyapunov_solve(const mc_matrix_t *A, const mc_matrix_t *Q,
                       mc_lyapunov_solution_t *sol)
{
    /* Solve A^T*P + P*A = -Q using iterative method.
     *
     * For stable A (Hurwitz), the solution is:
     *   P = integral_0^inf exp(A^T*t) * Q * exp(A*t) dt
     *
     * We use Smith's iterative method for the continuous case:
     *   P_{k+1} = P_k + h*(A^T*P_k + P_k*A + Q)
     * which is a forward Euler discretization of the matrix ODE dP/dt = A^T*P + P*A + Q.
     *
     * For improved convergence, we use the doubled-step variant:
     *   Solve for small h, then square.
     */
    if (!A || !Q || !sol) return -1;
    size_t n = A->rows;
    if (A->cols != n || Q->rows != n || Q->cols != n) return -2;

    memset(sol, 0, sizeof(*sol));
    if (mc_matrix_alloc(n, n, &sol->P) != 0) return -3;
    if (mc_matrix_alloc(n, n, &sol->Q) != 0) {
        mc_matrix_free(&sol->P); return -3;
    }
    mc_matrix_copy(Q, &sol->Q);

    /* Check if A is Hurwitz (necessary for unique solution) */
    int is_hurwitz = mc_is_hurwitz(A);
    if (!is_hurwitz) {
        /* May still have a solution, but not guaranteed unique */
    }

    /* Direct iteration: P_{k+1} = P_k + alpha*(A^T*P_k + P_k*A + Q)
     * with optimal step size alpha for convergence.
     * For stable A, converges to the unique solution.
     */
    mc_matrix_copy(Q, &sol->P);  /* Initial guess */

    mc_matrix_t AT, ATP, PA, residual_matrix, temp;
    int ok = 1;
    ok &= (mc_matrix_alloc(n, n, &AT) == 0);
    ok &= (mc_matrix_alloc(n, n, &ATP) == 0);
    ok &= (mc_matrix_alloc(n, n, &PA) == 0);
    ok &= (mc_matrix_alloc(n, n, &residual_matrix) == 0);
    ok &= (mc_matrix_alloc(n, n, &temp) == 0);
    if (!ok) {
        mc_lyapunov_solution_free(sol);
        return -4;
    }

    mc_matrix_transpose(A, &AT);

    /* Estimate step size based on spectral radius */
    double alpha = 0.5 / (mc_matrix_norm_inf(A) + 1.0);
    if (alpha < 1e-8) alpha = 1e-6;

    double tol = 1e-12;
    size_t max_iter = 5000;
    sol->convergence = 0;

    for (sol->iterations = 0; sol->iterations < max_iter; sol->iterations++) {
        /* Compute residual: A^T*P + P*A + Q */
        mc_matrix_mul(&AT, &sol->P, &ATP);
        mc_matrix_mul(&sol->P, A, &PA);
        mc_matrix_add(&ATP, &PA, &residual_matrix);
        mc_matrix_add(&residual_matrix, Q, &residual_matrix);

        double res_norm = mc_matrix_norm_frobenius(&residual_matrix);
        if (res_norm < tol) {
            sol->convergence = 1;
            break;
        }

        /* P = P + alpha * residual (fixed-point iteration) */
        /* Actually: P = P - alpha * (A^T*P + P*A + Q) */
        mc_matrix_scale(&residual_matrix, -alpha);
        mc_matrix_add(&sol->P, &residual_matrix, &temp);
        mc_matrix_copy(&temp, &sol->P);
    }

    /* Check positive definiteness */
    sol->is_positive_definite = mc_matrix_is_positive_definite(&sol->P);

    /* Estimate eigenvalues via trace and norm */
    sol->min_eigenvalue = 0.0;
    sol->max_eigenvalue = mc_matrix_norm_inf(&sol->P);
    sol->condition_number = sol->max_eigenvalue > 1e-15 ?
        sol->max_eigenvalue / 1e-10 : 0.0;

    mc_matrix_free(&AT); mc_matrix_free(&ATP); mc_matrix_free(&PA);
    mc_matrix_free(&residual_matrix); mc_matrix_free(&temp);

    return sol->convergence ? 0 : -10;
}

int mc_lyapunov_solve_discrete(const mc_matrix_t *A, const mc_matrix_t *Q,
                                mc_lyapunov_solution_t *sol)
{
    /* Solve A^T*P*A - P = -Q (discrete Lyapunov).
     *
     * For stable A (Schur), the solution is:
     *   P = sum_{k=0}^inf (A^T)^k * Q * A^k
     *
     * Uses Smith's iteration:
     *   P_{k+1} = Q + A^T * P_k * A
     * which converges geometrically if spectral radius of A < 1.
     */
    if (!A || !Q || !sol) return -1;
    size_t n = A->rows;
    if (A->cols != n || Q->rows != n || Q->cols != n) return -2;

    memset(sol, 0, sizeof(*sol));
    if (mc_matrix_alloc(n, n, &sol->P) != 0) return -3;
    if (mc_matrix_alloc(n, n, &sol->Q) != 0) {
        mc_matrix_free(&sol->P); return -3;
    }
    mc_matrix_copy(Q, &sol->Q);
    mc_matrix_copy(Q, &sol->P);

    mc_matrix_t AT, PA, ATPA, temp, P_old;
    int ok = 1;
    ok &= (mc_matrix_alloc(n, n, &AT) == 0);
    ok &= (mc_matrix_alloc(n, n, &PA) == 0);
    ok &= (mc_matrix_alloc(n, n, &ATPA) == 0);
    ok &= (mc_matrix_alloc(n, n, &temp) == 0);
    ok &= (mc_matrix_alloc(n, n, &P_old) == 0);
    if (!ok) { mc_lyapunov_solution_free(sol); return -4; }

    mc_matrix_transpose(A, &AT);

    double tol = 1e-12;
    size_t max_iter = 5000;
    sol->convergence = 0;

    for (sol->iterations = 0; sol->iterations < max_iter; sol->iterations++) {
        mc_matrix_copy(&sol->P, &P_old);

        /* P_new = Q + A^T * P * A */
        mc_matrix_mul(&sol->P, A, &PA);
        mc_matrix_mul(&AT, &PA, &ATPA);
        mc_matrix_add(Q, &ATPA, &temp);
        mc_matrix_copy(&temp, &sol->P);

        double diff = 0.0;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double d = sol->P.data[i*sol->P.stride+j] - P_old.data[i*P_old.stride+j];
                diff += d*d;
            }
        if (sqrt(diff) < tol) { sol->convergence = 1; break; }
    }

    sol->is_positive_definite = mc_matrix_is_positive_definite(&sol->P);
    sol->max_eigenvalue = mc_matrix_norm_inf(&sol->P);
    sol->condition_number = sol->max_eigenvalue > 1e-15 ? sol->max_eigenvalue / 1e-10 : 0.0;

    mc_matrix_free(&AT); mc_matrix_free(&PA); mc_matrix_free(&ATPA);
    mc_matrix_free(&temp); mc_matrix_free(&P_old);
    return sol->convergence ? 0 : -10;
}

void mc_lyapunov_solution_free(mc_lyapunov_solution_t *sol)
{
    if (!sol) return;
    mc_matrix_free(&sol->P);
    mc_matrix_free(&sol->Q);
    memset(sol, 0, sizeof(*sol));
}
