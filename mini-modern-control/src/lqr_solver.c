#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* L4-L5: Linear Quadratic Regulator - CARE via Kleinman-Newton iteration
 * CARE: A^T*P + P*A - P*B*R^{-1}*B^T*P + Q = 0
 * References: Kalman (1964), Kleinman (1968), Laub (1979)
 */

int mc_lqr_solve(const mc_matrix_t *A, const mc_matrix_t *B,
                  const mc_matrix_t *Q, const mc_matrix_t *R,
                  mc_lqr_method_t method, mc_lqr_solution_t *sol)
{
    if (!A || !B || !Q || !R || !sol) return -1;
    size_t n = A->rows;
    size_t m = B->cols;
    if (A->cols != n || B->rows != n) return -2;
    if (Q->rows != n || Q->cols != n) return -3;
    if (R->rows != m || R->cols != m) return -4;
    (void)method;

    memset(sol, 0, sizeof(*sol));
    if (mc_matrix_alloc(n, n, &sol->P) != 0) return -5;
    if (mc_matrix_alloc(m, n, &sol->K) != 0) { mc_matrix_free(&sol->P); return -5; }

    /* Compute R^{-1} */
    mc_matrix_t R_inv;
    if (mc_matrix_alloc(m, m, &R_inv) != 0) { mc_lqr_solution_free(sol); return -6; }
    mc_matrix_inverse(R, &R_inv);

    /* Initialize P = Q, K = R^{-1}*B^T*Q (initial guess) */
    mc_matrix_copy(Q, &sol->P);
    mc_matrix_t BT, temp;
    mc_matrix_alloc(n, m, &BT);
    mc_matrix_alloc(m, n, &temp);
    mc_matrix_transpose(B, &BT);
    /* K0 = R^{-1}*B^T*Q */
    mc_matrix_t BTQ;
    mc_matrix_alloc(m, n, &BTQ);
    mc_matrix_mul(&BT, Q, &BTQ);
    mc_matrix_mul(&R_inv, &BTQ, &sol->K);
    mc_matrix_free(&BTQ);

    mc_matrix_t A_cl, BK, QK, P_old;
    mc_matrix_alloc(n, n, &A_cl);
    mc_matrix_alloc(n, n, &BK);
    mc_matrix_alloc(n, n, &QK);
    mc_matrix_alloc(n, n, &P_old);

    size_t max_iter = 100;
    double tol = 1e-10;
    sol->convergence = 0;

    for (sol->iterations = 0; sol->iterations < max_iter; sol->iterations++) {
        mc_matrix_copy(&sol->P, &P_old);

        /* K = R^{-1} * B^T * P */
        mc_matrix_t BTP;
        mc_matrix_alloc(m, n, &BTP);
        mc_matrix_mul(&BT, &sol->P, &BTP);
        mc_matrix_mul(&R_inv, &BTP, &sol->K);
        mc_matrix_free(&BTP);

        /* QK = Q + K^T*R*K */
        mc_matrix_t KT, RK, KTRK;
        mc_matrix_alloc(n, m, &KT);
        mc_matrix_alloc(m, n, &RK);
        mc_matrix_alloc(n, n, &KTRK);
        mc_matrix_transpose(&sol->K, &KT);
        mc_matrix_mul(R, &sol->K, &RK);
        mc_matrix_mul(&KT, &RK, &KTRK);
        mc_matrix_add(Q, &KTRK, &QK);
        mc_matrix_free(&KT); mc_matrix_free(&RK); mc_matrix_free(&KTRK);

        /* Solve Lyapunov: (A-B*K)^T*P + P*(A-B*K) = -QK */
        mc_matrix_mul(B, &sol->K, &BK);
        mc_matrix_sub(A, &BK, &A_cl);

        mc_lyapunov_solution_t ls;
        int lr = mc_lyapunov_solve(&A_cl, &QK, &ls);
        if (lr == 0) {
            mc_matrix_copy(&ls.P, &sol->P);
            mc_lyapunov_solution_free(&ls);
        } else { goto done; }

        /* Convergence check */
        double diff = 0.0;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double d = sol->P.data[i*sol->P.stride+j] - P_old.data[i*P_old.stride+j];
                diff += d*d;
            }
        if (sqrt(diff) < tol) { sol->convergence = 1; break; }
    }

done:
    sol->j_min = mc_matrix_trace(&sol->P);
    mc_matrix_mul(B, &sol->K, &BK);
    mc_matrix_sub(A, &BK, &A_cl);
    sol->is_stabilizing = mc_is_hurwitz(&A_cl);

    /* Residual: ||A^T*P + P*A - P*B*R^{-1}*B^T*P + Q|| */
    mc_matrix_t AT;
    mc_matrix_alloc(n, n, &AT);
    mc_matrix_transpose(A, &AT);
    mc_matrix_t ATP, PA, PBRBTP;
    mc_matrix_alloc(n, n, &ATP);
    mc_matrix_alloc(n, n, &PA);
    mc_matrix_alloc(n, n, &PBRBTP);
    mc_matrix_mul(&AT, &sol->P, &ATP);
    mc_matrix_mul(&sol->P, A, &PA);
    mc_matrix_add(&ATP, &PA, &temp);
    /* P*B*R^{-1}*B^T*P */
    mc_matrix_t PB;
    mc_matrix_alloc(n, m, &PB);
    mc_matrix_mul(&sol->P, B, &PB);
    mc_matrix_mul(&PB, &sol->K, &PBRBTP);  /* K = R^{-1}*B^T*P */
    mc_matrix_sub(&temp, &PBRBTP, &temp);
    mc_matrix_add(&temp, Q, &temp);
    sol->residual = mc_matrix_norm_frobenius(&temp);
    mc_matrix_free(&AT); mc_matrix_free(&ATP); mc_matrix_free(&PA);
    mc_matrix_free(&PBRBTP); mc_matrix_free(&PB);

    mc_matrix_free(&R_inv); mc_matrix_free(&BT); mc_matrix_free(&temp);
    mc_matrix_free(&A_cl); mc_matrix_free(&BK); mc_matrix_free(&QK);
    mc_matrix_free(&P_old);
    return sol->convergence ? 0 : -10;
}

void mc_lqr_solution_free(mc_lqr_solution_t *sol)
{
    if (!sol) return;
    mc_matrix_free(&sol->P);
    mc_matrix_free(&sol->K);
    memset(sol, 0, sizeof(*sol));
}

int mc_lqr_discrete(const mc_matrix_t *A, const mc_matrix_t *B,
                     const mc_matrix_t *Q, const mc_matrix_t *R,
                     mc_dare_solution_t *sol)
{
    /* DARE: P = A^T*P*A - A^T*P*B*(R+B^T*P*B)^{-1}*B^T*P*A + Q
     * Solved via value iteration (discrete Kleinman).
     */
    if (!A || !B || !Q || !R || !sol) return -1;
    size_t n = A->rows, m = B->cols;
    if (A->cols != n || B->rows != n) return -2;
    if (Q->rows != n || Q->cols != n) return -3;
    if (R->rows != m || R->cols != m) return -4;

    memset(sol, 0, sizeof(*sol));
    if (mc_matrix_alloc(n, n, &sol->P) != 0) return -5;
    if (mc_matrix_alloc(m, n, &sol->K) != 0) { mc_matrix_free(&sol->P); return -5; }

    mc_matrix_copy(Q, &sol->P);
    mc_matrix_t AT, BT, temp, P_old, BTPB_R, BTPB_R_inv, BTPA, Acl;
    int ok = 1;
    ok &= (mc_matrix_alloc(n, n, &AT) == 0);
    ok &= (mc_matrix_alloc(n, m, &BT) == 0);
    ok &= (mc_matrix_alloc(n, n, &temp) == 0);
    ok &= (mc_matrix_alloc(n, n, &P_old) == 0);
    ok &= (mc_matrix_alloc(m, m, &BTPB_R) == 0);
    ok &= (mc_matrix_alloc(m, m, &BTPB_R_inv) == 0);
    ok &= (mc_matrix_alloc(m, n, &BTPA) == 0);
    ok &= (mc_matrix_alloc(n, n, &Acl) == 0);
    if (!ok) { mc_dare_solution_free(sol); return -6; }

    mc_matrix_transpose(A, &AT);
    mc_matrix_transpose(B, &BT);

    size_t max_iter = 200;
    double tol = 1e-10;
    sol->convergence = 0;

    for (sol->iterations = 0; sol->iterations < max_iter; sol->iterations++) {
        mc_matrix_copy(&sol->P, &P_old);

        /* B^T*P*B + R */
        mc_matrix_t PB, BTPB;
        mc_matrix_alloc(n, m, &PB);
        mc_matrix_alloc(m, m, &BTPB);
        mc_matrix_mul(&sol->P, B, &PB);
        mc_matrix_mul(&BT, &PB, &BTPB);
        mc_matrix_add(&BTPB, R, &BTPB_R);

        if (mc_matrix_inverse(&BTPB_R, &BTPB_R_inv) != 0) {
            mc_matrix_free(&PB); mc_matrix_free(&BTPB); break;
        }

        /* K = (R + B^T*P*B)^{-1} * B^T * P * A */
        mc_matrix_t PA;
        mc_matrix_alloc(n, n, &PA);
        mc_matrix_mul(&sol->P, A, &PA);
        mc_matrix_mul(&BT, &PA, &BTPA);
        mc_matrix_mul(&BTPB_R_inv, &BTPA, &sol->K);
        mc_matrix_free(&PA);

        /* P = A^T*P*(A - B*K) + Q */
        mc_matrix_mul(B, &sol->K, &Acl);
        mc_matrix_t AmBK;
        mc_matrix_alloc(n, n, &AmBK);
        mc_matrix_sub(A, &Acl, &AmBK);
        mc_matrix_t P_AmBK;
        mc_matrix_alloc(n, n, &P_AmBK);
        mc_matrix_mul(&sol->P, &AmBK, &P_AmBK);
        mc_matrix_mul(&AT, &P_AmBK, &temp);
        mc_matrix_add(&temp, Q, &sol->P);
        mc_matrix_free(&AmBK); mc_matrix_free(&P_AmBK);

        double diff = 0.0;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                double d = sol->P.data[i*sol->P.stride+j] - P_old.data[i*P_old.stride+j];
                diff += d*d;
            }
        mc_matrix_free(&PB); mc_matrix_free(&BTPB);

        if (sqrt(diff) < tol) { sol->convergence = 1; break; }
    }

    sol->j_min = mc_matrix_trace(&sol->P);
    mc_matrix_mul(B, &sol->K, &Acl);
    mc_matrix_t AmBK2;
    mc_matrix_alloc(n, n, &AmBK2);
    mc_matrix_sub(A, &Acl, &AmBK2);
    sol->is_stabilizing = mc_is_schur(&AmBK2);
    mc_matrix_free(&AmBK2);

    mc_matrix_free(&AT); mc_matrix_free(&BT); mc_matrix_free(&temp);
    mc_matrix_free(&P_old); mc_matrix_free(&BTPB_R);
    mc_matrix_free(&BTPB_R_inv); mc_matrix_free(&BTPA); mc_matrix_free(&Acl);
    return sol->convergence ? 0 : -10;
}

void mc_dare_solution_free(mc_dare_solution_t *sol)
{
    if (!sol) return;
    mc_matrix_free(&sol->P);
    mc_matrix_free(&sol->K);
    memset(sol, 0, sizeof(*sol));
}
