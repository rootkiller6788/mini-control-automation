#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* L5: Luenberger Observer Design
 * Error dynamics: de/dt = (A - L*C)*e
 * Design via duality to pole placement: place poles of (A^T, C^T), L = K_dual^T
 * References: Luenberger (1964, 1971), Chen (2013)
 */

static int dual_ackermann_design(const mc_matrix_t *A, const mc_matrix_t *C,
                                  const double *poles, size_t n,
                                  mc_matrix_t *L)
{
    mc_matrix_t AD, BD, Ctrb, Cn, Ci, pA, Ap, T, T2;
    mc_matrix_alloc(n, n, &AD); mc_matrix_alloc(n, 1, &BD);
    mc_matrix_transpose(A, &AD);
    for (size_t i = 0; i < n; i++) BD.data[i] = C->data[i];

    double *pc = (double*)calloc(n + 1, sizeof(double));
    mc_poles_to_poly(poles, n, pc);

    mc_matrix_alloc(n, n, &Ctrb); mc_matrix_alloc(n, n, &Cn);
    mc_ctrb_matrix(&AD, &BD, &Ctrb);
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++)
            Cn.data[i*Cn.stride+j] = Ctrb.data[i*Ctrb.stride+j];

    mc_matrix_alloc(n, n, &Ci); mc_matrix_alloc(n, n, &pA);
    mc_matrix_alloc(n, n, &Ap); mc_matrix_alloc(n, n, &T);
    mc_matrix_alloc(n, n, &T2);

    mc_matrix_zero(&pA);
    for (size_t i = 0; i < n; i++) pA.data[i*pA.stride+i] = pc[0];
    mc_matrix_identity(&Ap);
    for (size_t k = 1; k <= n; k++) {
        mc_matrix_mul(&Ap, &AD, &T);
        mc_matrix_copy(&T, &Ap);
        if (pc[k] != 0.0)
            for (size_t i = 0; i < n; i++)
                for (size_t j = 0; j < n; j++)
                    pA.data[i*pA.stride+j] += pc[k]*Ap.data[i*Ap.stride+j];
    }

    if (mc_matrix_inverse(&Cn, &Ci) == 0) {
        mc_matrix_mul(&Ci, &pA, &T2);
        for (size_t j = 0; j < n; j++)
            L->data[j*L->stride+0] = T2.data[(n-1)*T2.stride+j];
    }

    mc_matrix_free(&AD); mc_matrix_free(&BD); mc_matrix_free(&Ctrb);
    mc_matrix_free(&Cn); mc_matrix_free(&Ci); mc_matrix_free(&pA);
    mc_matrix_free(&Ap); mc_matrix_free(&T); mc_matrix_free(&T2);
    free(pc);
    return 0;
}

int mc_observer_luenberger_design(const mc_ss_system_t *sys,
                                   const double *observer_poles,
                                   mc_luenberger_observer_t *obs)
{
    if (!sys || !observer_poles || !obs) return -1;
    size_t n = sys->n_states;
    size_t p = sys->n_outputs;
    if (p != 1 || n == 0) return -2;

    memset(obs, 0, sizeof(*obs));
    obs->n_states = n; obs->n_outputs = p;
    obs->observer_poles = (double*)malloc(n * sizeof(double));
    if (!obs->observer_poles) return -3;
    memcpy(obs->observer_poles, observer_poles, n * sizeof(double));
    if (mc_matrix_alloc(n, p, &obs->L) != 0) {
        free(obs->observer_poles); return -4;
    }

    dual_ackermann_design(&sys->A, &sys->C, observer_poles, n, &obs->L);

    /* Verify convergence */
    mc_matrix_t LC, A_cl;
    mc_matrix_alloc(n, n, &LC);
    mc_matrix_alloc(n, n, &A_cl);
    mc_matrix_mul(&obs->L, &sys->C, &LC);
    mc_matrix_sub(&sys->A, &LC, &A_cl);
    obs->is_convergent = mc_is_hurwitz(&A_cl);
    mc_matrix_free(&LC); mc_matrix_free(&A_cl);
    return 0;
}

int mc_observer_luenberger_discrete(const mc_ss_discrete_t *sys,
                                     const double *observer_poles,
                                     mc_luenberger_observer_t *obs)
{
    if (!sys || !observer_poles || !obs) return -1;
    size_t n = sys->n_states;
    size_t p = sys->n_outputs;
    if (p != 1 || n == 0) return -2;

    memset(obs, 0, sizeof(*obs));
    obs->n_states = n; obs->n_outputs = p;
    obs->observer_poles = (double*)malloc(n * sizeof(double));
    if (!obs->observer_poles) return -3;
    memcpy(obs->observer_poles, observer_poles, n * sizeof(double));
    if (mc_matrix_alloc(n, p, &obs->L) != 0) {
        free(obs->observer_poles); return -4;
    }

    dual_ackermann_design(&sys->A, &sys->C, observer_poles, n, &obs->L);

    mc_matrix_t LC, A_cl;
    mc_matrix_alloc(n, n, &LC); mc_matrix_alloc(n, n, &A_cl);
    mc_matrix_mul(&obs->L, &sys->C, &LC);
    mc_matrix_sub(&sys->A, &LC, &A_cl);
    obs->is_convergent = mc_is_schur(&A_cl);
    mc_matrix_free(&LC); mc_matrix_free(&A_cl);
    return 0;
}

void mc_luenberger_observer_free(mc_luenberger_observer_t *obs)
{
    if (!obs) return;
    free(obs->observer_poles);
    mc_matrix_free(&obs->L);
    memset(obs, 0, sizeof(*obs));
}
