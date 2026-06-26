#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* =========================================================================
 * L6: Kalman Filter for Linear Discrete-Time Systems
 *
 * System: x[k+1] = A*x[k] + B*u[k] + w[k]   (w ~ N(0, Q))
 *         y[k]   = C*x[k] + v[k]             (v ~ N(0, R))
 *
 * Prediction step:
 *   x_hat[k|k-1] = A * x_hat[k-1|k-1] + B * u[k-1]
 *   P[k|k-1]     = A * P[k-1|k-1] * A^T + Q
 *
 * Update step:
 *   K[k]          = P[k|k-1] * C^T * (C * P[k|k-1] * C^T + R)^{-1}
 *   x_hat[k|k]    = x_hat[k|k-1] + K[k] * (y[k] - C * x_hat[k|k-1])
 *   P[k|k]        = (I - K[k] * C) * P[k|k-1]
 *
 * The Kalman filter is the optimal state estimator for linear Gaussian
 * systems (minimum mean-square error). It is the dual of the LQR:
 *   Kalman filter (estimation): A, C, Q, R -> P, K
 *   LQR (control):             A^T, B^T, Q, R -> P, K^T
 *
 * References:
 *   Kalman, "A New Approach to Linear Filtering and Prediction" (1960)
 *   Kalman & Bucy, "New Results in Linear Filtering and Prediction" (1961)
 *   Anderson & Moore, "Optimal Filtering" (1979)
 * ========================================================================= */

int mc_kalman_filter_init(mc_ss_discrete_t *sys,
                            const mc_matrix_t *Q, const mc_matrix_t *R,
                            const mc_vector_t *x0, const mc_matrix_t *P0,
                            mc_kalman_filter_t *kf)
{
    if (!sys || !Q || !R || !x0 || !kf) return -1;
    size_t n = sys->n_states;
    size_t p = sys->n_outputs;

    memset(kf, 0, sizeof(*kf));

    /* Copy system matrices */
    if (mc_ss_discrete_alloc(n, sys->n_inputs, p, &kf->sys, sys->sample_time) != 0)
        return -2;
    mc_matrix_copy(&sys->A, &kf->sys.A);
    mc_matrix_copy(&sys->B, &kf->sys.B);
    mc_matrix_copy(&sys->C, &kf->sys.C);

    /* Allocate Q, R, x_hat, P, K */
    if (mc_matrix_alloc(n, n, &kf->Q_kf) != 0 ||
        mc_matrix_alloc(p, p, &kf->R_kf) != 0 ||
        mc_vector_alloc(n, &kf->x_hat) != 0 ||
        mc_matrix_alloc(n, n, &kf->P_kf) != 0 ||
        mc_matrix_alloc(n, p, &kf->K_kf) != 0) {
        mc_kalman_filter_free(kf); return -3;
    }

    mc_matrix_copy(Q, &kf->Q_kf);
    mc_matrix_copy(R, &kf->R_kf);
    mc_vector_copy(x0, &kf->x_hat);
    mc_matrix_copy(P0, &kf->P_kf);
    kf->initialized = 1;
    return 0;
}

int mc_kalman_filter_predict(mc_kalman_filter_t *kf,
                               const mc_vector_t *u)
{
    /* Prediction step (time update).
     * x_hat = A*x_hat + B*u
     * P = A*P*A^T + Q
     */
    if (!kf || !kf->initialized) return -1;
    size_t n = kf->sys.n_states;

    mc_vector_t x_pred;
    if (mc_vector_alloc(n, &x_pred) != 0) return -2;

    /* x_pred = A * x_hat */
    mc_matrix_vector_mul(&kf->sys.A, &kf->x_hat, &x_pred);

    /* Add B*u if control input provided */
    if (u && kf->sys.n_inputs > 0) {
        mc_vector_t Bu;
        if (mc_vector_alloc(n, &Bu) == 0) {
            mc_matrix_vector_mul(&kf->sys.B, u, &Bu);
            mc_vector_add(&x_pred, &Bu, &x_pred);
            mc_vector_free(&Bu);
        }
    }

    mc_vector_copy(&x_pred, &kf->x_hat);
    mc_vector_free(&x_pred);

    /* P = A * P * A^T + Q */
    mc_matrix_t AT, PA, APAT;
    if (mc_matrix_alloc(n, n, &AT) != 0 ||
        mc_matrix_alloc(n, n, &PA) != 0 ||
        mc_matrix_alloc(n, n, &APAT) != 0) {
        mc_matrix_free(&AT); mc_matrix_free(&PA); mc_matrix_free(&APAT);
        return -2;
    }

    mc_matrix_transpose(&kf->sys.A, &AT);
    mc_matrix_mul(&kf->P_kf, &AT, &PA);     /* PA = P * A^T */
    mc_matrix_mul(&kf->sys.A, &PA, &APAT);  /* APAT = A * P * A^T */
    mc_matrix_add(&APAT, &kf->Q_kf, &kf->P_kf);

    mc_matrix_free(&AT); mc_matrix_free(&PA); mc_matrix_free(&APAT);
    return 0;
}

int mc_kalman_filter_update(mc_kalman_filter_t *kf,
                              const mc_vector_t *y)
{
    /* Update step (measurement update).
     * S = C*P*C^T + R          (innovation covariance)
     * K = P*C^T * S^{-1}      (Kalman gain)
     * x_hat = x_hat + K*(y - C*x_hat)
     * P = (I - K*C) * P
     */
    if (!kf || !kf->initialized || !y) return -1;
    size_t n = kf->sys.n_states;
    size_t p = kf->sys.n_outputs;

    mc_matrix_t CT, PCT, S, S_inv, KC, I_KC, P_new;
    mc_vector_t y_pred, innov;

    int ok = 1;
    ok &= (mc_matrix_alloc(p, n, &CT) == 0);
    ok &= (mc_matrix_alloc(n, p, &PCT) == 0);
    ok &= (mc_matrix_alloc(p, p, &S) == 0);
    ok &= (mc_matrix_alloc(p, p, &S_inv) == 0);
    ok &= (mc_matrix_alloc(n, n, &KC) == 0);
    ok &= (mc_matrix_alloc(n, n, &I_KC) == 0);
    ok &= (mc_matrix_alloc(n, n, &P_new) == 0);
    ok &= (mc_vector_alloc(p, &y_pred) == 0);
    ok &= (mc_vector_alloc(p, &innov) == 0);
    if (!ok) goto update_cleanup;

    /* C^T */
    mc_matrix_transpose(&kf->sys.C, &CT);

    /* Innovation: y - C*x_hat */
    mc_matrix_vector_mul(&kf->sys.C, &kf->x_hat, &y_pred);
    mc_vector_sub(y, &y_pred, &innov);

    /* S = C*P*C^T + R */
    mc_matrix_mul(&kf->P_kf, &CT, &PCT);
    mc_matrix_mul(&kf->sys.C, &PCT, &S);
    mc_matrix_add(&S, &kf->R_kf, &S);

    /* K = P*C^T * S^{-1} */
    if (mc_matrix_inverse(&S, &S_inv) == 0) {
        mc_matrix_mul(&PCT, &S_inv, &kf->K_kf);
    }

    /* x_hat = x_hat + K * innov */
    mc_vector_t K_innov;
    if (mc_vector_alloc(n, &K_innov) == 0) {
        mc_matrix_vector_mul(&kf->K_kf, &innov, &K_innov);
        mc_vector_add(&kf->x_hat, &K_innov, &kf->x_hat);
        mc_vector_free(&K_innov);
    }

    /* P = (I - K*C) * P */
    mc_matrix_mul(&kf->K_kf, &kf->sys.C, &KC);
    mc_matrix_identity(&I_KC);
    mc_matrix_sub(&I_KC, &KC, &I_KC);
    mc_matrix_mul(&I_KC, &kf->P_kf, &P_new);
    mc_matrix_copy(&P_new, &kf->P_kf);

update_cleanup:
    mc_matrix_free(&CT); mc_matrix_free(&PCT); mc_matrix_free(&S);
    mc_matrix_free(&S_inv); mc_matrix_free(&KC); mc_matrix_free(&I_KC);
    mc_matrix_free(&P_new); mc_vector_free(&y_pred); mc_vector_free(&innov);
    return ok ? 0 : -10;
}

void mc_kalman_filter_free(mc_kalman_filter_t *kf)
{
    if (!kf) return;
    mc_ss_discrete_free(&kf->sys);
    mc_matrix_free(&kf->Q_kf);
    mc_matrix_free(&kf->R_kf);
    mc_vector_free(&kf->x_hat);
    mc_matrix_free(&kf->P_kf);
    mc_matrix_free(&kf->K_kf);
    memset(kf, 0, sizeof(*kf));
}
