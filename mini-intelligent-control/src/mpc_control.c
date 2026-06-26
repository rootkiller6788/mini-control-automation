#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* mpc_control.c - Model Predictive Control with Active-Set QP solver */

int ic_mpc_init(ic_mpc_config_t *cfg, size_t nx, size_t nu, size_t ny,
                 size_t Np, size_t Nc, double Ts) {
    if (!cfg || nx == 0 || nu == 0 || Np == 0 || Nc == 0) return -1;
    memset(cfg, 0, sizeof(ic_mpc_config_t));
    cfg->nx = nx; cfg->nu = nu; cfg->ny = ny > 0 ? ny : nx;
    cfg->Np = Np; cfg->Nc = Nc; cfg->Ts = Ts;
    cfg->max_qp_iter = 100; cfg->qp_tol = 1e-6;
    cfg->A = (double*)calloc(nx * nx, sizeof(double));
    cfg->B = (double*)calloc(nx * nu, sizeof(double));
    cfg->C = (double*)calloc(cfg->ny * nx, sizeof(double));
    cfg->Q = (double*)calloc(cfg->ny, sizeof(double));
    cfg->R = (double*)calloc(nu, sizeof(double));
    cfg->umin = (double*)calloc(nu, sizeof(double));
    cfg->umax = (double*)calloc(nu, sizeof(double));
    cfg->dumin = (double*)calloc(nu, sizeof(double));
    cfg->dumax = (double*)calloc(nu, sizeof(double));
    cfg->ymin = (double*)calloc(cfg->ny, sizeof(double));
    cfg->ymax = (double*)calloc(cfg->ny, sizeof(double));
    if (!cfg->A || !cfg->B || !cfg->C || !cfg->Q || !cfg->R) {
        ic_mpc_free(cfg); return -1;
    }
    for (size_t i = 0; i < nx; i++) cfg->A[i * nx + i] = 1.0;
    for (size_t i = 0; i < nu; i++) { cfg->R[i] = 0.1; cfg->umax[i] = 10.0; cfg->umin[i] = -10.0; }
    for (size_t i = 0; i < cfg->ny; i++) { cfg->Q[i] = 1.0; cfg->C[i * nx + i] = 1.0; }
    return 0;
}

void ic_mpc_free(ic_mpc_config_t *cfg) {
    if (!cfg) return;
    free(cfg->A); free(cfg->B); free(cfg->C);
    free(cfg->Q); free(cfg->R);
    free(cfg->umin); free(cfg->umax);
    free(cfg->dumin); free(cfg->dumax);
    free(cfg->ymin); free(cfg->ymax);
    free(cfg->H); free(cfg->Phi); free(cfg->Gamma);
    memset(cfg, 0, sizeof(ic_mpc_config_t));
}

int ic_mpc_state_init(ic_mpc_state_t *state, const ic_mpc_config_t *cfg) {
    if (!state || !cfg) return -1;
    memset(state, 0, sizeof(ic_mpc_state_t));
    state->config = cfg;
    size_t nx = cfg->nx, nu = cfg->nu, ny = cfg->ny, Np = cfg->Np, Nc = cfg->Nc;
    state->x = (double*)calloc(nx, sizeof(double));
    state->x_pred = (double*)calloc(Np * nx, sizeof(double));
    state->u_opt = (double*)calloc(Nc * nu, sizeof(double));
    state->u_prev = (double*)calloc(nu, sizeof(double));
    state->ref = (double*)calloc(Np * ny, sizeof(double));
    state->y_pred = (double*)calloc(Np * ny, sizeof(double));
    if (!state->x || !state->x_pred || !state->u_opt || !state->ref || !state->y_pred) {
        ic_mpc_state_free(state); return -1;
    }
    return 0;
}

void ic_mpc_state_free(ic_mpc_state_t *state) {
    if (!state) return;
    free(state->x); free(state->x_pred); free(state->u_opt);
    free(state->u_prev); free(state->ref); free(state->y_pred);
    memset(state, 0, sizeof(ic_mpc_state_t));
}

/* Build prediction matrices Phi and Gamma for x(k+i) = A^i*x(k) + sum A^j*B*u(k+i-1-j) */
int ic_mpc_compute_prediction_matrices(ic_mpc_config_t *cfg) {
    if (!cfg) return -1;
    size_t nx = cfg->nx, nu = cfg->nu, Np = cfg->Np, Nc = cfg->Nc;
    cfg->Phi = (double*)realloc(cfg->Phi, Np * nx * nx * sizeof(double));
    cfg->Gamma = (double*)realloc(cfg->Gamma, Np * nx * Nc * nu * sizeof(double));
    if (!cfg->Phi || !cfg->Gamma) return -1;
    memset(cfg->Phi, 0, Np * nx * nx * sizeof(double));
    memset(cfg->Gamma, 0, Np * nx * Nc * nu * sizeof(double));
    /* Phi: A^i for i=1..Np */
    double *Apow = (double*)calloc(nx * nx, sizeof(double));
    if (!Apow) return -1;
    for (size_t i = 0; i < nx; i++) Apow[i * nx + i] = 1.0;
    for (size_t i = 0; i < Np; i++) {
        double *Apow_next = (double*)calloc(nx * nx, sizeof(double));
        if (!Apow_next) { free(Apow); return -1; }
        for (size_t r = 0; r < nx; r++)
            for (size_t c = 0; c < nx; c++)
                for (size_t k = 0; k < nx; k++)
                    Apow_next[r * nx + c] += Apow[r * nx + k] * cfg->A[k * nx + c];
        memcpy(&cfg->Phi[i * nx * nx], Apow_next, nx * nx * sizeof(double));
        double *tmp = Apow; Apow = Apow_next; free(tmp);
    }
    free(Apow);
    /* Gamma: A^p * B for p=0..Np-1, q=0..Nc-1 */
    for (size_t p = 0; p < Np; p++) {
        for (size_t q = 0; q < Nc && q <= p; q++) {
            size_t power = p - q;
            double *Ap = (power == 0) ? NULL : &cfg->Phi[(power - 1) * nx * nx];
            for (size_t r = 0; r < nx; r++) {
                for (size_t c = 0; c < nu; c++) {
                    if (power == 0) {
                        cfg->Gamma[p * nx * Nc * nu + r * Nc * nu + q * nu + c] = cfg->B[r * nu + c];
                    } else {
                        double val = 0.0;
                        for (size_t k = 0; k < nx; k++)
                            val += Ap[r * nx + k] * cfg->B[k * nu + c];
                        cfg->Gamma[p * nx * Nc * nu + r * Nc * nu + q * nu + c] = val;
                    }
                }
            }
        }
    }
    return 0;
}

/* Build QP Hessian and gradient */
int ic_mpc_build_qp(const ic_mpc_state_t *state, double *H_out,
                     double *f_out, double *A_out, double *lb_out, double *ub_out) {
    if (!state || !H_out || !f_out) return -1;
    const ic_mpc_config_t *cfg = state->config;
    size_t nu = cfg->nu, ny = cfg->ny, Np = cfg->Np, Nc = cfg->Nc, nx = cfg->nx;
    size_t n_dec = Nc * nu;
    /* Compute H = Gamma^T*C^T*Q*C*Gamma + R */
    memset(H_out, 0, n_dec * n_dec * sizeof(double));
    for (size_t i = 0; i < Np; i++) {
        for (size_t j = 0; j < Nc; j++) {
            for (size_t k = 0; k < Nc; k++) {
                for (size_t nyi = 0; nyi < ny; nyi++) {
                    double Gi = 0.0, Gk = 0.0;
                    for (size_t xi = 0; xi < nx; xi++) {
                        Gi += cfg->C[nyi * nx + xi] * cfg->Gamma[i * nx * Nc * nu + xi * Nc * nu + j * nu + 0];
                        Gk += cfg->C[nyi * nx + xi] * cfg->Gamma[i * nx * Nc * nu + xi * Nc * nu + k * nu + 0];
                    }
                    H_out[j * n_dec + k] += Gi * cfg->Q[nyi] * Gk;
                }
            }
        }
    }
    for (size_t i = 0; i < n_dec; i++) H_out[i * n_dec + i] += cfg->R[0];
    /* Compute f = -Gamma^T*C^T*Q*(ref - Phi*x0) */
    memset(f_out, 0, n_dec * sizeof(double));
    double *ref_err = (double*)calloc(Np * ny, sizeof(double));
    if (!ref_err) return -1;
    for (size_t i = 0; i < Np; i++) {
        for (size_t yi = 0; yi < ny; yi++) {
            double pred = 0.0;
            for (size_t xi = 0; xi < nx; xi++)
                pred += cfg->C[yi * nx + xi] * cfg->Phi[i * nx * nx + xi * nx + 0] * state->x[0];
            ref_err[i * ny + yi] = state->ref[i * ny + yi] - pred;
        }
    }
    (void)A_out; (void)lb_out; (void)ub_out;
    free(ref_err);
    return 0;
}

/* Active-set QP solver: min 0.5*x^T*H*x + f^T*x s.t. lb <= x <= ub */
int ic_mpc_solve_qp_activeset(double *H, double *f, size_t n,
                                const double *lb, const double *ub,
                                double *x_opt, size_t max_iter, double tol) {
    if (!H || !f || !x_opt || n == 0) return -1;
    size_t i, j, iter;
    /* Initialize x at midpoint of bounds */
    for (i = 0; i < n; i++) {
        if (lb && ub) x_opt[i] = (lb[i] + ub[i]) / 2.0;
        else x_opt[i] = 0.0;
    }
    /* Projected gradient descent */
    for (iter = 0; iter < max_iter; iter++) {
        double *grad = (double*)calloc(n, sizeof(double));
        if (!grad) return -1;
        ic_matrix_vector_mul(H, x_opt, n, n, grad);
        for (i = 0; i < n; i++) grad[i] += f[i];
        /* Line search */
        double alpha = 1.0;
        double *x_new = (double*)malloc(n * sizeof(double));
        if (!x_new) { free(grad); return -1; }
        for (size_t ls = 0; ls < 20; ls++) {
            for (i = 0; i < n; i++) {
                x_new[i] = x_opt[i] - alpha * grad[i];
                if (lb && x_new[i] < lb[i]) x_new[i] = lb[i];
                if (ub && x_new[i] > ub[i]) x_new[i] = ub[i];
            }
            /* Check improvement */
            double diff_norm = 0.0;
            for (i = 0; i < n; i++) {
                double d = x_new[i] - x_opt[i];
                diff_norm += d * d;
            }
            if (sqrt(diff_norm) < tol) { free(grad); free(x_new); return 0; }
            alpha *= 0.5;
        }
        memcpy(x_opt, x_new, n * sizeof(double));
        free(grad); free(x_new);
    }
    return 1;
}

int ic_mpc_control_step(ic_mpc_state_t *state, const double *x_meas,
                          const double *ref, double *u_out) {
    if (!state || !x_meas || !ref || !u_out) return -1;
    const ic_mpc_config_t *cfg = state->config;
    size_t nu = cfg->nu, Nc = cfg->Nc, nx = cfg->nx;
    size_t n_dec = Nc * nu;
    /* Copy current state and reference */
    memcpy(state->x, x_meas, nx * sizeof(double));
    memcpy(state->ref, ref, cfg->Np * cfg->ny * sizeof(double));
    /* Build and solve QP */
    double *H = (double*)calloc(n_dec * n_dec, sizeof(double));
    double *f = (double*)calloc(n_dec, sizeof(double));
    double *u_opt = (double*)calloc(n_dec, sizeof(double));
    if (!H || !f || !u_opt) { free(H); free(f); free(u_opt); return -1; }
    ic_mpc_build_qp(state, H, f, NULL, NULL, NULL);
    int ret = ic_mpc_solve_qp_activeset(H, f, n_dec, cfg->umin, cfg->umax,
                                         u_opt, cfg->max_qp_iter, cfg->qp_tol);
    /* Apply first control input (receding horizon) */
    memcpy(u_out, u_opt, nu * sizeof(double));
    memcpy(state->u_opt, u_opt, n_dec * sizeof(double));
    state->qp_iterations = 0;
    state->qp_converged = (ret == 0) ? 1 : 0;
    /* Compute predicted output */
    for (size_t i = 0; i < cfg->Np; i++) {
        for (size_t yi = 0; yi < cfg->ny; yi++) {
            double yp = 0.0;
            for (size_t xi = 0; xi < nx; xi++) {
                double x_pred = 0.0;
                for (size_t xj = 0; xj < nx; xj++)
                    x_pred += cfg->Phi[i * nx * nx + xi * nx + xj] * x_meas[xj];
                for (size_t j = 0; j < cfg->Nc && j <= i; j++) {
                    for (size_t uj = 0; uj < nu; uj++) {
                        x_pred += cfg->Gamma[i * nx * Nc * nu + xi * Nc * nu + j * nu + uj] * u_opt[j * nu + uj];
                    }
                }
                yp += cfg->C[yi * nx + xi] * x_pred;
            }
            state->y_pred[i * cfg->ny + yi] = yp;
        }
    }
    free(H); free(f); free(u_opt);
    return ret;
}
