#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L1-L2: State-Space System Lifecycle, Simulation, and Discretization
 *
 * Continuous:  dx/dt = A*x + B*u,  y = C*x + D*u
 * Discrete:    x[k+1] = A*x[k] + B*u[k],  y[k] = C*x[k] + D*u[k]
 *
 * References: Kalman (1963), Ogata (2010), Chen (2013)
 * ========================================================================= */

int mc_ss_alloc(size_t n_states, size_t n_inputs, size_t n_outputs,
                 mc_ss_system_t *sys)
{
    if (!sys || n_states == 0) return -1;
    memset(sys, 0, sizeof(*sys));
    sys->n_states  = n_states;
    sys->n_inputs  = n_inputs;
    sys->n_outputs = n_outputs;
    sys->sys_type  = MC_SYS_CONTINUOUS;
    snprintf(sys->name, sizeof(sys->name), "sys_%zux%zux%zu",
             n_states, n_inputs, n_outputs);
    int ret;
    ret = mc_matrix_alloc(n_states, n_states, &sys->A);
    if (ret) goto fail_A;
    if (n_inputs > 0) {
        ret = mc_matrix_alloc(n_states, n_inputs, &sys->B);
        if (ret) goto fail_B;
    }
    if (n_outputs > 0) {
        ret = mc_matrix_alloc(n_outputs, n_states, &sys->C);
        if (ret) goto fail_C;
    }
    if (n_outputs > 0 && n_inputs > 0) {
        ret = mc_matrix_alloc(n_outputs, n_inputs, &sys->D);
        if (ret) goto fail_D;
    }
    return 0;
fail_D:
    if (sys->C.data) mc_matrix_free(&sys->C);
fail_C:
    if (sys->B.data) mc_matrix_free(&sys->B);
fail_B:
    mc_matrix_free(&sys->A);
fail_A:
    return ret;
}

void mc_ss_free(mc_ss_system_t *sys)
{
    if (!sys) return;
    mc_matrix_free(&sys->A);
    mc_matrix_free(&sys->B);
    mc_matrix_free(&sys->C);
    mc_matrix_free(&sys->D);
    memset(sys, 0, sizeof(*sys));
}

int mc_ss_discrete_alloc(size_t n, size_t m, size_t p,
                          mc_ss_discrete_t *sys, double Ts)
{
    if (!sys || n == 0 || Ts <= 0.0) return -1;
    memset(sys, 0, sizeof(*sys));
    sys->n_states    = n;
    sys->n_inputs    = m;
    sys->n_outputs   = p;
    sys->sample_time = Ts;
    sys->sys_type    = MC_SYS_DISCRETE;
    snprintf(sys->name, sizeof(sys->name), "dsys_%zux%zux%zu_Ts%.4f",
             n, m, p, Ts);
    int ret;
    ret = mc_matrix_alloc(n, n, &sys->A);
    if (ret) goto fail_A_d;
    if (m > 0) {
        ret = mc_matrix_alloc(n, m, &sys->B);
        if (ret) goto fail_B_d;
    }
    if (p > 0) {
        ret = mc_matrix_alloc(p, n, &sys->C);
        if (ret) goto fail_C_d;
    }
    if (p > 0 && m > 0) {
        ret = mc_matrix_alloc(p, m, &sys->D);
        if (ret) goto fail_D_d;
    }
    return 0;
fail_D_d:
    if (sys->C.data) mc_matrix_free(&sys->C);
fail_C_d:
    if (sys->B.data) mc_matrix_free(&sys->B);
fail_B_d:
    mc_matrix_free(&sys->A);
fail_A_d:
    return ret;
}

void mc_ss_discrete_free(mc_ss_discrete_t *sys)
{
    if (!sys) return;
    mc_matrix_free(&sys->A);
    mc_matrix_free(&sys->B);
    mc_matrix_free(&sys->C);
    mc_matrix_free(&sys->D);
    memset(sys, 0, sizeof(*sys));
}

/* ---------- Continuous-Time Simulation (Forward Euler) ---------- */

int mc_simulate_step(const mc_ss_system_t *sys, const mc_vector_t *x,
                      const mc_vector_t *u, double dt,
                      mc_vector_t *x_next, mc_vector_t *y)
{
    /* Forward Euler: x_next = x + dt * (A*x + B*u), y = C*x + D*u */
    if (!sys || !x || !x_next || dt <= 0.0) return -1;
    if (x->length != sys->n_states || x_next->length != sys->n_states) return -2;
    mc_vector_t dx, Bu;
    if (mc_vector_alloc(sys->n_states, &dx) != 0) return -3;
    mc_matrix_vector_mul(&sys->A, x, &dx);
    if (sys->n_inputs > 0 && u) {
        if (mc_vector_alloc(sys->n_states, &Bu) != 0) {
            mc_vector_free(&dx); return -3;
        }
        mc_matrix_vector_mul(&sys->B, u, &Bu);
        mc_vector_add(&dx, &Bu, &dx);
        mc_vector_free(&Bu);
    }
    mc_vector_lincomb(1.0, x, dt, &dx, x_next);
    mc_vector_free(&dx);
    if (y && sys->n_outputs > 0) {
        if (y->length != sys->n_outputs) return -4;
        mc_matrix_vector_mul(&sys->C, x, y);
        if (sys->n_inputs > 0 && u && sys->D.data) {
            mc_vector_t Du;
            if (mc_vector_alloc(sys->n_outputs, &Du) == 0) {
                mc_matrix_vector_mul(&sys->D, u, &Du);
                mc_vector_add(y, &Du, y);
                mc_vector_free(&Du);
            }
        }
    }
    return 0;
}

int mc_simulate_discrete_step(const mc_ss_discrete_t *sys,
                               const mc_vector_t *x, const mc_vector_t *u,
                               mc_vector_t *x_next, mc_vector_t *y)
{
    /* Discrete step: x_next = A*x + B*u, y = C*x + D*u */
    if (!sys || !x || !x_next) return -1;
    if (x->length != sys->n_states || x_next->length != sys->n_states) return -2;
    mc_vector_t Ax, Bu;
    if (mc_vector_alloc(sys->n_states, &Ax) != 0) return -3;
    mc_matrix_vector_mul(&sys->A, x, &Ax);
    mc_vector_copy(&Ax, x_next);
    if (sys->n_inputs > 0 && u) {
        if (mc_vector_alloc(sys->n_states, &Bu) == 0) {
            mc_matrix_vector_mul(&sys->B, u, &Bu);
            mc_vector_add(x_next, &Bu, x_next);
            mc_vector_free(&Bu);
        }
    }
    mc_vector_free(&Ax);
    if (y && sys->n_outputs > 0) {
        mc_matrix_vector_mul(&sys->C, x, y);
        if (sys->n_inputs > 0 && u && sys->D.data) {
            mc_vector_t Du;
            if (mc_vector_alloc(sys->n_outputs, &Du) == 0) {
                mc_matrix_vector_mul(&sys->D, u, &Du);
                mc_vector_add(y, &Du, y);
                mc_vector_free(&Du);
            }
        }
    }
    return 0;
}
#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Discretization and Step Response for State-Space Systems */

int mc_c2d(const mc_ss_system_t *cont_sys, mc_ss_discrete_t *disc_sys,
            double Ts, mc_discretization_t method)
{
    if (!cont_sys || !disc_sys || Ts <= 0.0) return -1;
    if (disc_sys->n_states != cont_sys->n_states) return -2;
    size_t n = cont_sys->n_states;
    size_t m = cont_sys->n_inputs;
    size_t p = cont_sys->n_outputs;

    if (method == MC_DISC_EULER_FWD) {
        mc_matrix_identity(&disc_sys->A);
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++)
                disc_sys->A.data[i * disc_sys->A.stride + j] +=
                    Ts * cont_sys->A.data[i * cont_sys->A.stride + j];
        if (m > 0)
            for (size_t i = 0; i < n; i++)
                for (size_t j = 0; j < m; j++)
                    disc_sys->B.data[i * disc_sys->B.stride + j] =
                        Ts * cont_sys->B.data[i * cont_sys->B.stride + j];
    } else {
        /* ZOH using augmented matrix exponential via Taylor series */
        size_t N = n + m;
        double *M = (double*)calloc(N * N, sizeof(double));
        if (!M) return -3;
        for (size_t i = 0; i < n; i++) {
            for (size_t j = 0; j < n; j++)
                M[i * N + j] = cont_sys->A.data[i * cont_sys->A.stride + j] * Ts;
            for (size_t j = 0; j < m; j++)
                M[i * N + n + j] = cont_sys->B.data[i * cont_sys->B.stride + j] * Ts;
        }
        double *expM = (double*)calloc(N * N, sizeof(double));
        double *Mpow = (double*)calloc(N * N, sizeof(double));
        double *tempM = (double*)malloc(N * N * sizeof(double));

        for (size_t i = 0; i < N; i++) { 
            expM[i * N + i] = 1.0; 
            Mpow[i * N + i] = 1.0; 
        }
        double fact = 1.0;
        for (int k = 1; k <= 15; k++) {
            for (size_t i = 0; i < N; i++)
                for (size_t j = 0; j < N; j++) {
                    double sum = 0.0;
                    for (size_t kk = 0; kk < N; kk++)
                        sum += Mpow[i * N + kk] * M[kk * N + j];
                    tempM[i * N + j] = sum;
                }
            for (size_t i = 0; i < N * N; i++) Mpow[i] = tempM[i];
            fact *= k;
            for (size_t i = 0; i < N * N; i++) expM[i] += Mpow[i] / fact;
        }
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++)
                disc_sys->A.data[i * disc_sys->A.stride + j] = expM[i * N + j];
        if (m > 0)
            for (size_t i = 0; i < n; i++)
                for (size_t j = 0; j < m; j++)
                    disc_sys->B.data[i * disc_sys->B.stride + j] = expM[i * N + n + j];
        free(M); free(expM); free(Mpow); free(tempM);
    }
    if (p > 0) mc_matrix_copy(&cont_sys->C, &disc_sys->C);
    if (p > 0 && m > 0 && cont_sys->D.data)
        mc_matrix_copy(&cont_sys->D, &disc_sys->D);
    disc_sys->sample_time = Ts;
    disc_sys->sys_type = MC_SYS_DISCRETE;
    return 0;
}

int mc_d2c(const mc_ss_discrete_t *disc_sys, mc_ss_system_t *cont_sys)
{
    if (!disc_sys || !cont_sys || disc_sys->sample_time <= 0.0) return -1;
    size_t n = disc_sys->n_states;
    double Ts = disc_sys->sample_time;
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++) {
            double a_ij = disc_sys->A.data[i * disc_sys->A.stride + j];
            cont_sys->A.data[i * cont_sys->A.stride + j] =
                ((i == j) ? (a_ij - 1.0) / Ts : a_ij / Ts);
        }
    size_t m = disc_sys->n_inputs;
    if (m > 0)
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < m; j++)
                cont_sys->B.data[i * cont_sys->B.stride + j] =
                    disc_sys->B.data[i * disc_sys->B.stride + j] / Ts;
    size_t p = disc_sys->n_outputs;
    if (p > 0) mc_matrix_copy(&disc_sys->C, &cont_sys->C);
    if (p > 0 && m > 0 && disc_sys->D.data)
        mc_matrix_copy(&disc_sys->D, &cont_sys->D);
    cont_sys->sys_type = MC_SYS_CONTINUOUS;
    return 0;
}

/* ---------- Step Response ---------- */

int mc_step_response(const mc_ss_system_t *sys, size_t output_idx,
                      double t_final, double dt,
                      mc_step_response_t *resp)
{
    if (!sys || !resp || output_idx >= sys->n_outputs) return -1;
    if (t_final <= 0.0 || dt <= 0.0) return -2;
    memset(resp, 0, sizeof(*resp));

    int n_steps = (int)(t_final / dt) + 1;
    if (n_steps > 100000) n_steps = 100000;
    double *y_arr = (double*)malloc(n_steps * sizeof(double));
    if (!y_arr) return -3;

    mc_vector_t x, x_next, y_vec, u_vec;
    mc_vector_alloc(sys->n_states, &x);
    mc_vector_alloc(sys->n_states, &x_next);
    mc_vector_alloc(sys->n_outputs, &y_vec);
    mc_vector_t *u_ptr = NULL;
    if (sys->n_inputs > 0) {
        mc_vector_alloc(sys->n_inputs, &u_vec);
        mc_vector_zero(&u_vec);
        u_vec.data[0] = 1.0;
        u_ptr = &u_vec;
    }

    mc_vector_zero(&x);
    double t = 0.0, y_max = -1e100;
    int peak_idx = 0;

    for (int k = 0; k < n_steps; k++) {
        mc_simulate_step(sys, &x, u_ptr, dt, &x_next, &y_vec);
        y_arr[k] = y_vec.data[output_idx];
        if (y_arr[k] > y_max) { y_max = y_arr[k]; peak_idx = k; }
        mc_vector_copy(&x_next, &x);
        t += dt;
    }

    double y_ss = y_arr[n_steps - 1];
    resp->steady_state_value = y_ss;
    resp->dc_gain = y_ss;
    resp->steady_state_error = 1.0 - y_ss;

    if (fabs(y_ss) > 1e-10) {
        double overshoot = (y_max - y_ss) / fabs(y_ss);
        resp->overshoot_percent = (overshoot > 0) ? overshoot * 100.0 : 0.0;
        resp->peak_time = peak_idx * dt;
    }

    double y10 = 0.1 * y_ss, y90 = 0.9 * y_ss;
    int idx10 = -1, idx90 = -1;
    for (int k = 0; k < n_steps; k++) {
        if (idx10 < 0 && y_arr[k] >= y10) idx10 = k;
        if (idx90 < 0 && y_arr[k] >= y90) idx90 = k;
    }
    if (idx10 >= 0 && idx90 >= 0 && idx90 > idx10)
        resp->rise_time = (idx90 - idx10) * dt;

    double tol_band = 0.02 * fabs(y_ss);
    if (tol_band < 1e-8) tol_band = 0.02;
    for (int k = n_steps - 1; k >= 0; k--) {
        if (fabs(y_arr[k] - y_ss) > tol_band) {
            resp->settling_time = (k + 1 < n_steps) ? (k + 1) * dt : t_final;
            break;
        }
    }

    mc_vector_free(&x); mc_vector_free(&x_next); mc_vector_free(&y_vec);
    if (sys->n_inputs > 0) mc_vector_free(&u_vec);
    free(y_arr);
    return 0;
}

/* ---------- Transfer Function Conversions ---------- */

int mc_tf_to_ss_controllable(const mc_tf_t *tf, mc_ss_system_t *sys)
{
    if (!tf || !sys) return -1;
    size_t n = tf->den_order;
    if (n == 0 || sys->n_states != n) return -2;
    if (sys->n_inputs != 1 || sys->n_outputs != 1) return -3;
    mc_matrix_zero(&sys->A);
    for (size_t i = 0; i < n - 1; i++) mc_matrix_set(&sys->A, i, i + 1, 1.0);
    for (size_t j = 0; j < n; j++) mc_matrix_set(&sys->A, n - 1, j, -tf->den_coeff[j]);
    mc_matrix_zero(&sys->B);
    mc_matrix_set(&sys->B, n - 1, 0, 1.0);
    for (size_t j = 0; j < n && j <= tf->num_order; j++)
        mc_matrix_set(&sys->C, 0, j, tf->num_coeff[j]);
    mc_matrix_zero(&sys->D);
    if (tf->num_order == n) mc_matrix_set(&sys->D, 0, 0, tf->num_coeff[n]);
    return 0;
}

int mc_tf_to_ss_observable(const mc_tf_t *tf, mc_ss_system_t *sys)
{
    if (!tf || !sys) return -1;
    size_t n = tf->den_order;
    if (n == 0 || sys->n_states != n) return -2;
    if (sys->n_inputs != 1 || sys->n_outputs != 1) return -3;
    mc_matrix_zero(&sys->A);
    for (size_t i = 0; i < n - 1; i++) mc_matrix_set(&sys->A, i + 1, i, 1.0);
    for (size_t i = 0; i < n; i++) mc_matrix_set(&sys->A, i, n - 1, -tf->den_coeff[i]);
    for (size_t i = 0; i < n && i <= tf->num_order; i++)
        mc_matrix_set(&sys->B, i, 0, tf->num_coeff[i]);
    mc_matrix_set(&sys->C, 0, n - 1, 1.0);
    mc_matrix_zero(&sys->D);
    if (tf->num_order == n) mc_matrix_set(&sys->D, 0, 0, tf->num_coeff[n]);
    return 0;
}

void mc_tf_free(mc_tf_t *tf)
{
    if (!tf) return;
    free(tf->num_coeff);
    free(tf->den_coeff);
    memset(tf, 0, sizeof(*tf));
}

double mc_tf_evaluate(const mc_tf_t *tf, double s)
{
    if (!tf) return 0.0;
    double num = 0.0;
    for (size_t i = tf->num_order + 1; i-- > 0; )
        num = num * s + tf->num_coeff[i];
    double den = 0.0;
    for (size_t i = tf->den_order + 1; i-- > 0; )
        den = den * s + tf->den_coeff[i];
    den = den * s + 1.0;
    return (fabs(den) > 1e-15) ? num / den : 0.0;
}
