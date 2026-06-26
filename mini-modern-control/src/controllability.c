#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L2: Controllability and Observability Analysis
 *
 * Controllability (Kalman, 1960):
 *   Ctrb = [B, A*B, A^2*B, ..., A^{n-1}*B]
 *   System is controllable iff rank(Ctrb) = n.
 *
 * Observability (Kalman, 1960):
 *   Obsv = [C; C*A; C*A^2; ...; C*A^{n-1}]
 *   System is observable iff rank(Obsv) = n.
 *
 * PBH test (Popov-Belevitch-Hautus):
 *   Controllable iff rank([lambda*I - A, B]) = n for all lambda.
 *   Observable iff rank([lambda*I - A; C]) = n for all lambda.
 *
 * Gramians:
 *   Controllability Gramian: Wc = integral_0^inf exp(A*t)*B*B^T*exp(A^T*t) dt
 *   Observability Gramian:  Wo = integral_0^inf exp(A^T*t)*C^T*C*exp(A*t) dt
 *   Wc and Wo satisfy Lyapunov equations:
 *     A*Wc + Wc*A^T + B*B^T = 0
 *     A^T*Wo + Wo*A + C^T*C = 0
 *
 * References:
 *   Kalman, "On the General Theory of Control Systems" (1960)
 *   Gilbert, "Controllability and Observability" (1963)
 * ========================================================================= */

int mc_ctrb_matrix(const mc_matrix_t *A, const mc_matrix_t *B,
                    mc_matrix_t *Ctrb)
{
    /* Build controllability matrix: Ctrb = [B, A*B, A^2*B, ..., A^{n-1}*B]
     * Size: n x (n*m), where n = A.rows, m = B.cols.
     * O(n^3 * m) complexity.
     */
    if (!A || !B || !Ctrb || !A->data || !B->data || !Ctrb->data) return -1;
    size_t n = A->rows;
    size_t m = B->cols;
    if (A->cols != n || B->rows != n) return -2;
    if (Ctrb->rows != n || Ctrb->cols != n * m) return -3;

    mc_matrix_zero(Ctrb);
    /* First block: B */
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < m; j++)
            Ctrb->data[i * Ctrb->stride + j] = B->data[i * B->stride + j];

    /* Compute A^k * B for k = 1..n-1 using repeated multiplication */
    mc_matrix_t A_pow, temp;
    if (mc_matrix_alloc(n, n, &A_pow) != 0) return -4;
    if (mc_matrix_alloc(n, m, &temp) != 0) {
        mc_matrix_free(&A_pow); return -4;
    }
    mc_matrix_copy(A, &A_pow);

    for (size_t k = 1; k < n; k++) {
        /* temp = A_pow * B */
        mc_matrix_mul(&A_pow, B, &temp);
        /* Copy into Ctrb at column block k*m */
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < m; j++)
                Ctrb->data[i * Ctrb->stride + k * m + j] =
                    temp.data[i * temp.stride + j];
        /* A_pow = A_pow * A (for next iteration, skip last) */
        if (k < n - 1) {
            mc_matrix_t A_next;
            if (mc_matrix_alloc(n, n, &A_next) == 0) {
                mc_matrix_mul(&A_pow, A, &A_next);
                mc_matrix_copy(&A_next, &A_pow);
                mc_matrix_free(&A_next);
            }
        }
    }

    mc_matrix_free(&A_pow);
    mc_matrix_free(&temp);
    return 0;
}

int mc_obsv_matrix(const mc_matrix_t *A, const mc_matrix_t *C,
                    mc_matrix_t *Obsv)
{
    /* Build observability matrix: Obsv = [C; C*A; C*A^2; ...; C*A^{n-1}]
     * Size: (n*p) x n, where n = A.rows, p = C.rows.
     * Dual of controllability: Obsv(A,C) = Ctrb(A^T, C^T)^T.
     */
    if (!A || !C || !Obsv || !A->data || !C->data || !Obsv->data) return -1;
    size_t n = A->rows;
    size_t p = C->rows;
    if (A->cols != n || C->cols != n) return -2;
    if (Obsv->rows != n * p || Obsv->cols != n) return -3;

    mc_matrix_zero(Obsv);
    /* First block: C */
    for (size_t i = 0; i < p; i++)
        for (size_t j = 0; j < n; j++)
            Obsv->data[i * Obsv->stride + j] = C->data[i * C->stride + j];

    mc_matrix_t A_pow, temp;
    if (mc_matrix_alloc(n, n, &A_pow) != 0) return -4;
    if (mc_matrix_alloc(p, n, &temp) != 0) {
        mc_matrix_free(&A_pow); return -4;
    }
    mc_matrix_copy(A, &A_pow);

    for (size_t k = 1; k < n; k++) {
        /* temp = C * A_pow */
        mc_matrix_mul(C, &A_pow, &temp);
        for (size_t i = 0; i < p; i++)
            for (size_t j = 0; j < n; j++)
                Obsv->data[(k * p + i) * Obsv->stride + j] =
                    temp.data[i * temp.stride + j];
        if (k < n - 1) {
            mc_matrix_t A_next;
            if (mc_matrix_alloc(n, n, &A_next) == 0) {
                mc_matrix_mul(&A_pow, A, &A_next);
                mc_matrix_copy(&A_next, &A_pow);
                mc_matrix_free(&A_next);
            }
        }
    }

    mc_matrix_free(&A_pow);
    mc_matrix_free(&temp);
    return 0;
}

int mc_controllability_analyze(const mc_ss_system_t *sys,
                                mc_controllability_t *ctrb)
{
    if (!sys || !ctrb) return -1;
    size_t n = sys->n_states;
    size_t m = sys->n_inputs;
    if (m == 0) { ctrb->status = MC_NOT_CONTROLLABLE; return 0; }

    /* Allocate and build controllability matrix */
    if (mc_matrix_alloc(n, n * m, &ctrb->Ctrb) != 0) return -2;
    mc_ctrb_matrix(&sys->A, &sys->B, &ctrb->Ctrb);

    /* Compute rank */
    int rank = mc_matrix_rank(&ctrb->Ctrb, 1e-10);
    ctrb->rank = (size_t)rank;

    if (rank == (int)n) {
        ctrb->status = MC_FULLY_CONTROLLABLE;
    } else {
        /* Check stabilizability: uncontrollable modes must be stable */
        ctrb->status = MC_NOT_CONTROLLABLE;
        /* Full stabilizability check requires eigenvalue decomposition */
    }

    /* Compute condition number estimate */
    double frob_norm = mc_matrix_norm_frobenius(&ctrb->Ctrb);
    ctrb->condition_number = (frob_norm > 1e-15) ? frob_norm : 0.0;
    ctrb->min_singular = 0.0; /* Would need SVD for accurate value */

    return 0;
}

int mc_controllability_discrete(const mc_ss_discrete_t *sys,
                                 mc_controllability_t *ctrb)
{
    if (!sys || !ctrb) return -1;
    size_t n = sys->n_states;
    size_t m = sys->n_inputs;
    if (m == 0) { ctrb->status = MC_NOT_CONTROLLABLE; return 0; }

    if (mc_matrix_alloc(n, n * m, &ctrb->Ctrb) != 0) return -2;
    mc_ctrb_matrix(&sys->A, &sys->B, &ctrb->Ctrb);

    int rank = mc_matrix_rank(&ctrb->Ctrb, 1e-10);
    ctrb->rank = (size_t)rank;
    ctrb->status = (rank == (int)n) ? MC_FULLY_CONTROLLABLE : MC_NOT_CONTROLLABLE;
    ctrb->condition_number = mc_matrix_norm_frobenius(&ctrb->Ctrb);
    ctrb->min_singular = 0.0;
    return 0;
}

void mc_controllability_free(mc_controllability_t *ctrb)
{
    if (!ctrb) return;
    mc_matrix_free(&ctrb->Ctrb);
    mc_matrix_free(&ctrb->Ctrb_gram);
    memset(ctrb, 0, sizeof(*ctrb));
}

int mc_observability_analyze(const mc_ss_system_t *sys,
                              mc_observability_t *obsv)
{
    if (!sys || !obsv) return -1;
    size_t n = sys->n_states;
    size_t p = sys->n_outputs;
    if (p == 0) { obsv->status = MC_NOT_OBSERVABLE; return 0; }

    if (mc_matrix_alloc(n * p, n, &obsv->Obsv) != 0) return -2;
    mc_obsv_matrix(&sys->A, &sys->C, &obsv->Obsv);

    int rank = mc_matrix_rank(&obsv->Obsv, 1e-10);
    obsv->rank = (size_t)rank;
    obsv->status = (rank == (int)n) ? MC_FULLY_OBSERVABLE : MC_NOT_OBSERVABLE;
    obsv->condition_number = mc_matrix_norm_frobenius(&obsv->Obsv);
    obsv->min_singular = 0.0;
    return 0;
}

int mc_observability_discrete(const mc_ss_discrete_t *sys,
                               mc_observability_t *obsv)
{
    if (!sys || !obsv) return -1;
    size_t n = sys->n_states;
    size_t p = sys->n_outputs;
    if (p == 0) { obsv->status = MC_NOT_OBSERVABLE; return 0; }

    if (mc_matrix_alloc(n * p, n, &obsv->Obsv) != 0) return -2;
    mc_obsv_matrix(&sys->A, &sys->C, &obsv->Obsv);

    int rank = mc_matrix_rank(&obsv->Obsv, 1e-10);
    obsv->rank = (size_t)rank;
    obsv->status = (rank == (int)n) ? MC_FULLY_OBSERVABLE : MC_NOT_OBSERVABLE;
    obsv->condition_number = mc_matrix_norm_frobenius(&obsv->Obsv);
    return 0;
}

void mc_observability_free(mc_observability_t *obsv)
{
    if (!obsv) return;
    mc_matrix_free(&obsv->Obsv);
    mc_matrix_free(&obsv->Obsv_gram);
    memset(obsv, 0, sizeof(*obsv));
}

/* ---------- Gramian Computation via Lyapunov Equation ---------- */

int mc_controllability_gramian(const mc_ss_system_t *sys, mc_matrix_t *Wc)
{
    /* Solve A*Wc + Wc*A^T + B*B^T = 0 for Wc.
     * Wc > 0 iff (A,B) is controllable (for stable A).
     * Uses simple iterative method: Wc[k+1] derived from integral.
     */
    if (!sys || !Wc || !sys->A.data) return -1;
    size_t n = sys->n_states;
    if (Wc->rows != n || Wc->cols != n) return -2;

    /* Compute B*B^T */
    mc_matrix_t BBT;
    if (mc_matrix_alloc(n, n, &BBT) != 0) return -3;
    if (sys->n_inputs > 0) {
        /* BBT = B * B^T */
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double sum = 0.0;
                for (size_t k = 0; k < sys->n_inputs; k++)
                    sum += sys->B.data[i * sys->B.stride + k] *
                           sys->B.data[j * sys->B.stride + k];
                BBT.data[i * BBT.stride + j] = sum;
            }
    }

    /* Solve Lyapunov equation: A*Wc + Wc*A^T = -B*B^T */
    mc_lyapunov_solution_t sol;
    int ret = mc_lyapunov_solve(&sys->A, &BBT, &sol);
    if (ret == 0) {
        mc_matrix_copy(&sol.P, Wc);
        mc_lyapunov_solution_free(&sol);
    }
    mc_matrix_free(&BBT);
    return ret;
}

int mc_observability_gramian(const mc_ss_system_t *sys, mc_matrix_t *Wo)
{
    /* Solve A^T*Wo + Wo*A + C^T*C = 0 for Wo.
     * Wo > 0 iff (A,C) is observable (for stable A).
     */
    if (!sys || !Wo || !sys->A.data) return -1;
    size_t n = sys->n_states;
    if (Wo->rows != n || Wo->cols != n) return -2;

    /* Compute C^T*C */
    mc_matrix_t CTC;
    if (mc_matrix_alloc(n, n, &CTC) != 0) return -3;
    if (sys->n_outputs > 0) {
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double sum = 0.0;
                for (size_t k = 0; k < sys->n_outputs; k++)
                    sum += sys->C.data[k * sys->C.stride + i] *
                           sys->C.data[k * sys->C.stride + j];
                CTC.data[i * CTC.stride + j] = sum;
            }
    }

    /* Solve A^T*Wo + Wo*A = -C^T*C.
     * This is equivalent to: A*X + X*A^T = Q with X=Wo, then transpose.
     */
    mc_matrix_t A_T;
    if (mc_matrix_alloc(n, n, &A_T) != 0) { mc_matrix_free(&CTC); return -3; }
    mc_matrix_transpose(&sys->A, &A_T);

    mc_lyapunov_solution_t sol;
    int ret = mc_lyapunov_solve(&A_T, &CTC, &sol);
    if (ret == 0) {
        mc_matrix_copy(&sol.P, Wo);
        mc_lyapunov_solution_free(&sol);
    }
    mc_matrix_free(&CTC);
    mc_matrix_free(&A_T);
    return ret;
}
