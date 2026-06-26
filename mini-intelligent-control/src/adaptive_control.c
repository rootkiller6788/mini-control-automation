#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* adaptive_control.c - MRAC, STR, Gain Scheduling */

/* ---- L6: Model Reference Adaptive Control (MRAC) ---- */

int ic_mrac_init(ic_mrac_t *mrac, double zeta_ref, double wn_ref,
                  double a0, double a1, double b,
                  const double *gamma, int use_lyapunov) {
    if (!mrac || !gamma) return -1;
    memset(mrac, 0, sizeof(ic_mrac_t));
    mrac->zeta_ref = zeta_ref; mrac->wn_ref = wn_ref;
    mrac->a0_nom = a0; mrac->a1_nom = a1; mrac->b_nom = b;
    mrac->gamma_theta[0] = gamma[0];
    mrac->gamma_theta[1] = gamma[1];
    mrac->gamma_theta[2] = gamma[2];
    mrac->use_lyapunov = use_lyapunov;
    mrac->theta[0] = wn_ref * wn_ref / b;
    mrac->theta[1] = (a0 - wn_ref * wn_ref) / b;
    mrac->theta[2] = (a1 - 2.0 * zeta_ref * wn_ref) / b;
    mrac->sigma_mod = 0.001;
    return 0;
}

void ic_mrac_step(ic_mrac_t *mrac, double r, double dt, double y_meas,
                   double *u_out, double *y_out) {
    if (!mrac || !u_out || !y_out) return;
    double zeta = mrac->zeta_ref, wn = mrac->wn_ref;
    double *theta = mrac->theta;
    /* Reference model dynamics */
    double ym_ddot = wn * wn * (r - mrac->ym) - 2.0 * zeta * wn * mrac->ym_dot;
    mrac->ym_ddot = ym_ddot;
    mrac->ym_dot += ym_ddot * dt;
    mrac->ym += mrac->ym_dot * dt;
    /* Plant output measurement */
    mrac->y = y_meas;
    mrac->y_dot = (y_meas - mrac->y) / (dt > 1e-10 ? dt : 1e-10);
    if (dt < 1e-10) mrac->y_dot = mrac->y_ddot;
    /* Control law: u = theta1*r - theta2*y - theta3*ydot */
    mrac->u = theta[0] * r - theta[1] * mrac->y - theta[2] * mrac->y_dot;
    mrac->error = mrac->y - mrac->ym;
    /* Adaptation law */
    if (mrac->use_lyapunov) {
        double phi0 = r, phi1 = -mrac->y, phi2 = -mrac->y_dot;
        theta[0] -= mrac->gamma_theta[0] * mrac->error * phi0 * dt;
        theta[1] -= mrac->gamma_theta[1] * mrac->error * phi1 * dt;
        theta[2] -= mrac->gamma_theta[2] * mrac->error * phi2 * dt;
        for (int i = 0; i < 3; i++)
            theta[i] -= mrac->sigma_mod * theta[i] * dt;
    } else {
        theta[0] -= mrac->gamma_theta[0] * mrac->error * r * dt;
        theta[1] -= mrac->gamma_theta[1] * mrac->error * (-mrac->y) * dt;
        theta[2] -= mrac->gamma_theta[2] * mrac->error * (-mrac->y_dot) * dt;
    }
    *u_out = mrac->u; *y_out = mrac->y;
}

void ic_mrac_reset(ic_mrac_t *mrac) {
    if (!mrac) return;
    mrac->ym = 0.0; mrac->ym_dot = 0.0; mrac->ym_ddot = 0.0;
    mrac->y = 0.0; mrac->y_dot = 0.0; mrac->error = 0.0; mrac->u = 0.0;
}

int ic_mrac_lyapunov_check(const ic_mrac_t *mrac) {
    if (!mrac) return 0;
    double V = 0.5 * mrac->error * mrac->error;
    return (V < 100.0) ? 1 : 0;
}

/* ---- L5: Self-Tuning Regulator (STR) ---- */

int ic_str_init(ic_str_t *str, size_t na, size_t nb, size_t nc,
                 size_t nk, double lambda) {
    if (!str) return -1;
    memset(str, 0, sizeof(ic_str_t));
    str->na = na; str->nb = nb; str->nc = nc; str->nk = nk;
    str->lambda_forget = lambda;
    str->param_dim = na + nb + nc;
    str->theta_hat = (double*)calloc(str->param_dim, sizeof(double));
    str->P_matrix = (double*)calloc(str->param_dim * str->param_dim, sizeof(double));
    str->phi_vector = (double*)calloc(str->param_dim, sizeof(double));
    if (!str->theta_hat || !str->P_matrix || !str->phi_vector) {
        ic_str_free(str); return -1;
    }
    for (size_t i = 0; i < str->param_dim; i++)
        str->P_matrix[i * str->param_dim + i] = 1000.0;
    str->theta_hat[0] = 0.1;
    return 0;
}

void ic_str_free(ic_str_t *str) {
    if (!str) return;
    free(str->A); free(str->B); free(str->C);
    free(str->theta_hat); free(str->P_matrix);
    free(str->phi_vector);
    free(str->R); free(str->S); free(str->T);
    memset(str, 0, sizeof(ic_str_t));
}

void ic_str_identify_rls(ic_str_t *str, double u, double y) {
    if (!str) return;
    size_t dim = str->param_dim;
    double lambda = str->lambda_forget;
    for (size_t i = 0; i < dim - 1; i++)
        str->phi_vector[dim - 1 - i] = str->phi_vector[dim - 2 - i];
    str->phi_vector[0] = y;
    str->u = u; str->y = y;
    double y_hat = ic_vector_dot(str->phi_vector, str->theta_hat, dim);
    double error = y - y_hat;
    double *Pphi = (double*)malloc(dim * sizeof(double));
    if (!Pphi) return;
    ic_matrix_vector_mul(str->P_matrix, str->phi_vector, dim, dim, Pphi);
    double phiTPphi = ic_vector_dot(str->phi_vector, Pphi, dim);
    double denom = lambda + phiTPphi;
    if (fabs(denom) < 1e-15) { free(Pphi); return; }
    for (size_t i = 0; i < dim; i++)
        str->theta_hat[i] += Pphi[i] * error / denom;
    for (size_t i = 0; i < dim; i++)
        for (size_t j = 0; j < dim; j++)
            str->P_matrix[i * dim + j] =
                (str->P_matrix[i * dim + j] - Pphi[i] * Pphi[j] / denom) / lambda;
    free(Pphi);
}

int ic_str_solve_diophantine(ic_str_t *str) {
    if (!str) return -1;
    str->nr = 1; str->ns = 1; str->nt = 1;
    str->R = (double*)realloc(str->R, str->nr * sizeof(double));
    str->S = (double*)realloc(str->S, str->ns * sizeof(double));
    str->T = (double*)realloc(str->T, str->nt * sizeof(double));
    if (!str->R || !str->S || !str->T) return -1;
    str->R[0] = 1.0; str->S[0] = -str->theta_hat[0]; str->T[0] = 1.0;
    return 0;
}

double ic_str_compute_control(ic_str_t *str, double ref) {
    if (!str) return 0.0;
    double u = (str->T[0] * ref - str->S[0] * str->y) /
               (fabs(str->R[0]) > 1e-15 ? str->R[0] : 1.0);
    return u;
}

/* ---- L5: Gain Scheduling ---- */

int ic_gain_sched_init(ic_gain_schedule_t *gs, size_t num_vars,
                        size_t poly_order, size_t num_gains) {
    if (!gs) return -1;
    memset(gs, 0, sizeof(ic_gain_schedule_t));
    gs->num_sched_vars = num_vars;
    gs->poly_order = poly_order;
    gs->num_gains = num_gains;
    gs->sched_vars = (double*)calloc(num_vars, sizeof(double));
    gs->sched_ranges = (double*)calloc(num_vars * 2, sizeof(double));
    gs->current_gains = (double*)calloc(num_gains, sizeof(double));
    size_t coeff_size = num_gains * (poly_order + 1) * num_vars;
    gs->coeff_table = (double*)calloc(coeff_size, sizeof(double));
    if (!gs->sched_vars || !gs->sched_ranges || !gs->current_gains || !gs->coeff_table) {
        ic_gain_sched_free(gs); return -1;
    }
    return 0;
}

void ic_gain_sched_free(ic_gain_schedule_t *gs) {
    if (!gs) return;
    free(gs->sched_vars); free(gs->sched_ranges);
    free(gs->current_gains); free(gs->coeff_table);
    memset(gs, 0, sizeof(ic_gain_schedule_t));
}

void ic_gain_sched_interpolate(ic_gain_schedule_t *gs,
                                const double *sched_vars, double *gains) {
    if (!gs || !sched_vars || !gains) return;
    memcpy(gs->sched_vars, sched_vars, gs->num_sched_vars * sizeof(double));
    size_t i, j, v;
    for (i = 0; i < gs->num_gains; i++) {
        gains[i] = 0.0;
        for (v = 0; v < gs->num_sched_vars; v++) {
            double alpha = sched_vars[v];
            if (gs->sched_ranges[2*v+1] > gs->sched_ranges[2*v]) {
                alpha = (alpha - gs->sched_ranges[2*v]) /
                        (gs->sched_ranges[2*v+1] - gs->sched_ranges[2*v]);
            }
            double poly_val = 0.0;
            double alpha_pow = 1.0;
            for (j = 0; j <= gs->poly_order; j++) {
                size_t idx = ((i * gs->num_sched_vars + v) *
                             (gs->poly_order + 1) + j);
                poly_val += gs->coeff_table[idx] * alpha_pow;
                alpha_pow *= alpha;
            }
            gains[i] += poly_val;
        }
    }
    memcpy(gs->current_gains, gains, gs->num_gains * sizeof(double));
}

/* ---- L7: Adaptive DC Motor Speed Control ---- */

typedef struct {
    ic_mrac_t mrac;
    double J, b, Kt, Ke, R, L;
    double omega, current;
    double target_omega;
} ic_adaptive_dc_motor_t;

int ic_adaptive_dc_motor_init(ic_adaptive_dc_motor_t *motor,
                                double J, double b, double Kt,
                                double Ke, double R, double L) {
    if (!motor) return -1;
    memset(motor, 0, sizeof(ic_adaptive_dc_motor_t));
    motor->J = J; motor->b = b; motor->Kt = Kt;
    motor->Ke = Ke; motor->R = R; motor->L = L;
    double gamma[3] = {10.0, 1.0, 0.1};
    ic_mrac_init(&motor->mrac, 1.0, 20.0,
                 b/J, (Kt*Ke)/(J*R), Kt/(J*R), gamma, 1);
    return 0;
}

void ic_adaptive_dc_motor_step(ic_adaptive_dc_motor_t *motor,
                                double voltage, double dt,
                                double *omega_out, double *u_out) {
    if (!motor) return;
    /* DC motor dynamics: */
    double omega_dot = (motor->Kt * motor->current - motor->b * motor->omega) / motor->J;
    double i_dot = (voltage - motor->R * motor->current - motor->Ke * motor->omega) / motor->L;
    motor->omega += omega_dot * dt;
    motor->current += i_dot * dt;
    double y_meas = motor->omega;
    double u, y;
    ic_mrac_step(&motor->mrac, motor->target_omega, dt, y_meas, &u, &y);
    if (omega_out) *omega_out = motor->omega;
    if (u_out) *u_out = u;
}

/* ---- L7: Adaptive Quadrotor Attitude Control ---- */

typedef struct {
    ic_mrac_t mrac_roll, mrac_pitch, mrac_yaw;
    double Ixx, Iyy, Izz;
    double roll, pitch, yaw;
    double roll_rate, pitch_rate, yaw_rate;
} ic_quadrotor_attitude_t;

int ic_quadrotor_attitude_init(ic_quadrotor_attitude_t *quad,
                                 double Ixx, double Iyy, double Izz) {
    if (!quad) return -1;
    memset(quad, 0, sizeof(ic_quadrotor_attitude_t));
    quad->Ixx = Ixx; quad->Iyy = Iyy; quad->Izz = Izz;
    double gamma[3] = {5.0, 2.0, 0.5};
    ic_mrac_init(&quad->mrac_roll, 0.8, 10.0, 0.0, 0.0, 1.0/Ixx, gamma, 1);
    ic_mrac_init(&quad->mrac_pitch, 0.8, 10.0, 0.0, 0.0, 1.0/Iyy, gamma, 1);
    ic_mrac_init(&quad->mrac_yaw, 0.9, 5.0, 0.0, 0.0, 1.0/Izz, gamma, 1);
    return 0;
}

void ic_quadrotor_attitude_step(ic_quadrotor_attitude_t *quad,
                                 const double *torques, double dt,
                                 double *angles_out, double *u_out) {
    if (!quad || !torques) return;
    double *U = (double*)malloc(3 * sizeof(double));
    if (!U) return;
    quad->roll_rate += torques[0] / quad->Ixx * dt;
    quad->pitch_rate += torques[1] / quad->Iyy * dt;
    quad->yaw_rate += torques[2] / quad->Izz * dt;
    quad->roll += quad->roll_rate * dt;
    quad->pitch += quad->pitch_rate * dt;
    quad->yaw += quad->yaw_rate * dt;
    ic_mrac_step(&quad->mrac_roll, 0.0, dt, quad->roll, &U[0], NULL);
    ic_mrac_step(&quad->mrac_pitch, 0.0, dt, quad->pitch, &U[1], NULL);
    ic_mrac_step(&quad->mrac_yaw, 0.0, dt, quad->yaw, &U[2], NULL);
    if (angles_out) { angles_out[0] = quad->roll; angles_out[1] = quad->pitch; angles_out[2] = quad->yaw; }
    if (u_out) memcpy(u_out, U, 3 * sizeof(double));
    free(U);
}
