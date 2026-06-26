#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* L5: Pole Placement via Ackermann Formula */

void mc_poles_to_poly(const double *poles, size_t n_poles, double *coeffs)
{
    if (!poles || !coeffs || n_poles == 0) return;
    coeffs[0] = -poles[0];
    coeffs[1] = 1.0;
    for (size_t i = 2; i <= n_poles; i++) coeffs[i] = 0.0;
    for (size_t i = 1; i < n_poles; i++) {
        double neg_p = -poles[i];
        double prev = 0.0;
        for (size_t j = 0; j <= i + 1; j++) {
            double cur = coeffs[j];
            coeffs[j] = prev + neg_p * cur;
            prev = cur;
        }
    }
}

int mc_pole_place_ackermann(const mc_ss_system_t *sys,
                             const double *desired_poles,
                             mc_state_feedback_t *fb)
{
    if (!sys || !desired_poles || !fb) return -1;
    if (sys->n_inputs != 1) return -2;
    size_t n = sys->n_states;
    if (n == 0) return -3;

    fb->n_states  = n;
    fb->n_inputs  = sys->n_inputs;
    fb->poles = (double*)malloc(n * sizeof(double));
    if (!fb->poles) return -4;
    memcpy(fb->poles, desired_poles, n * sizeof(double));
    if (mc_matrix_alloc(sys->n_inputs, n, &fb->K) != 0) {
        free(fb->poles); return -5;
    }

    double *poly_coeffs = (double*)calloc(n + 1, sizeof(double));
    if (!poly_coeffs) { mc_state_feedback_free(fb); return -6; }
    mc_poles_to_poly(desired_poles, n, poly_coeffs);

    mc_matrix_t Ctrb, Ctrb_inv, pA, A_pow, temp;
    if (mc_matrix_alloc(n, n, &Ctrb) != 0 ||
        mc_matrix_alloc(n, n, &Ctrb_inv) != 0 ||
        mc_matrix_alloc(n, n, &pA) != 0 ||
        mc_matrix_alloc(n, n, &A_pow) != 0 ||
        mc_matrix_alloc(n, n, &temp) != 0) {
        free(poly_coeffs); mc_state_feedback_free(fb); return -7;
    }
    mc_ctrb_matrix(&sys->A, &sys->B, &Ctrb);

    int rank = mc_matrix_rank(&Ctrb, 1e-10);
    if (rank < (int)n) {
        mc_matrix_free(&Ctrb); mc_matrix_free(&Ctrb_inv);
        mc_matrix_free(&pA); mc_matrix_free(&A_pow); mc_matrix_free(&temp);
        free(poly_coeffs); fb->is_stabilizing = 0; return 0;
    }

    mc_matrix_zero(&pA);
    for (size_t i = 0; i < n; i++)
        pA.data[i * pA.stride + i] = poly_coeffs[0];
    mc_matrix_identity(&A_pow);

    for (size_t k = 1; k <= n; k++) {
        mc_matrix_mul(&A_pow, &sys->A, &temp);
        mc_matrix_copy(&temp, &A_pow);
        if (poly_coeffs[k] != 0.0) {
            for (size_t i = 0; i < n; i++)
                for (size_t j = 0; j < n; j++)
                    pA.data[i * pA.stride + j] +=
                        poly_coeffs[k] * A_pow.data[i * A_pow.stride + j];
        }
    }

    int inv_ok = mc_matrix_inverse(&Ctrb, &Ctrb_inv);
    if (inv_ok == 0) {
        mc_matrix_t temp2;
        if (mc_matrix_alloc(n, n, &temp2) == 0) {
            mc_matrix_mul(&Ctrb_inv, &pA, &temp2);
            for (size_t j = 0; j < n; j++)
                fb->K.data[j] = temp2.data[(n - 1) * temp2.stride + j];
            mc_matrix_free(&temp2);
        }
    }

    mc_matrix_t BK, A_cl;
    if (mc_matrix_alloc(n, n, &BK) == 0 && mc_matrix_alloc(n, n, &A_cl) == 0) {
        mc_matrix_mul(&sys->B, &fb->K, &BK);
        mc_matrix_sub(&sys->A, &BK, &A_cl);
        fb->is_stabilizing = mc_is_hurwitz(&A_cl);
        mc_matrix_free(&BK); mc_matrix_free(&A_cl);
    }

    mc_matrix_free(&Ctrb); mc_matrix_free(&Ctrb_inv);
    mc_matrix_free(&pA); mc_matrix_free(&A_pow); mc_matrix_free(&temp);
    free(poly_coeffs);
    return 0;
}

int mc_pole_place_acker_discrete(const mc_ss_discrete_t *sys,
                                  const double *desired_poles,
                                  mc_state_feedback_t *fb)
{
    if (!sys || !desired_poles || !fb) return -1;
    if (sys->n_inputs != 1) return -2;
    size_t n = sys->n_states;

    fb->n_states = n; fb->n_inputs = sys->n_inputs;
    fb->poles = (double*)malloc(n * sizeof(double));
    if (!fb->poles) return -3;
    memcpy(fb->poles, desired_poles, n * sizeof(double));
    if (mc_matrix_alloc(sys->n_inputs, n, &fb->K) != 0) {
        free(fb->poles); return -4;
    }

    double *pc = (double*)calloc(n + 1, sizeof(double));
    mc_poles_to_poly(desired_poles, n, pc);

    mc_matrix_t C, Ci, pA, Ap, T;
    mc_matrix_alloc(n, n, &C); mc_matrix_alloc(n, n, &Ci);
    mc_matrix_alloc(n, n, &pA); mc_matrix_alloc(n, n, &Ap);
    mc_matrix_alloc(n, n, &T);
    mc_ctrb_matrix(&sys->A, &sys->B, &C);
    mc_matrix_inverse(&C, &Ci);

    mc_matrix_zero(&pA);
    for (size_t i = 0; i < n; i++) pA.data[i * pA.stride + i] = pc[0];
    mc_matrix_identity(&Ap);
    for (size_t k = 1; k <= n; k++) {
        mc_matrix_mul(&Ap, &sys->A, &T);
        mc_matrix_copy(&T, &Ap);
        if (pc[k] != 0.0)
            for (size_t i = 0; i < n; i++)
                for (size_t j = 0; j < n; j++)
                    pA.data[i * pA.stride + j] += pc[k] * Ap.data[i * Ap.stride + j];
    }

    mc_matrix_t T2;
    mc_matrix_alloc(n, n, &T2);
    mc_matrix_mul(&Ci, &pA, &T2);
    for (size_t j = 0; j < n; j++)
        fb->K.data[j] = T2.data[(n-1) * T2.stride + j];

    mc_matrix_t BK, A_cl;
    mc_matrix_alloc(n, n, &BK); mc_matrix_alloc(n, n, &A_cl);
    mc_matrix_mul(&sys->B, &fb->K, &BK);
    mc_matrix_sub(&sys->A, &BK, &A_cl);
    fb->is_stabilizing = mc_is_schur(&A_cl);

    mc_matrix_free(&C); mc_matrix_free(&Ci); mc_matrix_free(&pA);
    mc_matrix_free(&Ap); mc_matrix_free(&T); mc_matrix_free(&T2);
    mc_matrix_free(&BK); mc_matrix_free(&A_cl);
    free(pc);
    return 0;
}

void mc_state_feedback_free(mc_state_feedback_t *fb)
{
    if (!fb) return;
    free(fb->poles);
    mc_matrix_free(&fb->K);
    mc_matrix_free(&fb->N);
    memset(fb, 0, sizeof(*fb));
}
