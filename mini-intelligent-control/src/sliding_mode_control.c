#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* sliding_mode_control.c - SMC, Super-Twisting, ILC, Fuzzy-PID, ANFIS */

/* ---- L6: Sliding Mode Control ---- */

int ic_smc_init(ic_smc_t *smc, double lambda, double K, double eta,
                 double phi, int use_sat) {
    if (!smc) return -1;
    memset(smc, 0, sizeof(ic_smc_t));
    smc->lambda = lambda; smc->K = K; smc->eta = eta;
    smc->phi_boundary = phi; smc->use_saturation = use_sat;
    smc->a0 = 0.0; smc->a1 = 0.0; smc->b = 1.0;
    return 0;
}

double ic_smc_sliding_surface(const ic_smc_t *smc) {
    if (!smc) return 0.0;
    return smc->e_dot + smc->lambda * smc->e;
}

double ic_smc_reaching_condition(const ic_smc_t *smc) {
    if (!smc) return 0.0;
    double s = ic_smc_sliding_surface(smc);
    return -smc->eta * fabs(s);
}

void ic_smc_step(ic_smc_t *smc, double ref, double y_meas, double dt,
                  double *u_out) {
    if (!smc || !u_out || dt < 1e-12) return;
    double e_prev = smc->e;
    smc->e = ref - y_meas;
    smc->e_dot = (smc->e - e_prev) / dt;
    smc->e_int += smc->e * dt;
    smc->s = ic_smc_sliding_surface(smc);
    double ref_dot = (ref - smc->e + smc->e_dot) / dt;
    smc->u_eq = (smc->a0 * y_meas + smc->a1 * smc->e_dot + ref_dot * smc->lambda +
                 smc->lambda * smc->e_dot) / (smc->b + 1e-10);
    if (smc->use_saturation) {
        if (fabs(smc->s) <= smc->phi_boundary)
            smc->u_sw = smc->K * (smc->s / smc->phi_boundary);
        else
            smc->u_sw = smc->K * (smc->s > 0.0 ? 1.0 : -1.0);
    } else {
        smc->u_sw = smc->K * (smc->s > 0.0 ? 1.0 : (smc->s < 0.0 ? -1.0 : 0.0));
    }
    smc->u = smc->u_eq - smc->u_sw;
    *u_out = smc->u;
}

/* ---- L8: Super-Twisting SMC (2nd order) ---- */

typedef struct {
    double lambda, k1, k2;
    double s, e, e_dot;
    double u, integral_s;
} ic_smc_stw_t;

int ic_smc_stw_init(ic_smc_stw_t *stw, double lambda, double k1, double k2) {
    if (!stw) return -1;
    memset(stw, 0, sizeof(ic_smc_stw_t));
    stw->lambda = lambda; stw->k1 = k1; stw->k2 = k2;
    return 0;
}

void ic_smc_stw_step(ic_smc_stw_t *stw, double ref, double y_meas,
                      double dt, double *u_out) {
    if (!stw || !u_out || dt < 1e-12) return;
    double e_prev = stw->e;
    stw->e = ref - y_meas;
    stw->e_dot = (stw->e - e_prev) / dt;
    stw->s = stw->e_dot + stw->lambda * stw->e;
    stw->integral_s += stw->s * dt;
    double sign_s = (stw->s > 0.0) ? 1.0 : ((stw->s < 0.0) ? -1.0 : 0.0);
    double stw_term = stw->k1 * sqrt(fabs(stw->s)) * sign_s;
    stw->u = -(stw_term + stw->k2 * stw->integral_s * sign_s);
    *u_out = stw->u;
}

/* ---- L5: Iterative Learning Control (ILC) ---- */

int ic_ilc_init(ic_ilc_t *ilc, size_t n_samples, size_t n_inputs,
                 size_t n_outputs, double gain) {
    if (!ilc || n_samples == 0) return -1;
    memset(ilc, 0, sizeof(ic_ilc_t));
    ilc->n_samples = n_samples; ilc->n_inputs = n_inputs;
    ilc->n_outputs = n_outputs; ilc->learning_gain = gain;
    ilc->error_memory = (double*)calloc(n_samples * n_outputs, sizeof(double));
    ilc->control_memory = (double*)calloc(n_samples * n_inputs, sizeof(double));
    ilc->L_matrix = (double*)calloc(n_inputs * n_outputs, sizeof(double));
    if (!ilc->error_memory || !ilc->control_memory || !ilc->L_matrix) {
        ic_ilc_free(ilc); return -1;
    }
    for (size_t i = 0; i < n_inputs && i < n_outputs; i++)
        ilc->L_matrix[i * n_outputs + i] = gain;
    return 0;
}

void ic_ilc_free(ic_ilc_t *ilc) {
    if (!ilc) return;
    free(ilc->error_memory); free(ilc->control_memory); free(ilc->L_matrix);
    memset(ilc, 0, sizeof(ic_ilc_t));
}

void ic_ilc_update(ic_ilc_t *ilc, const double *error, double *control) {
    if (!ilc || !error || !control) return;
    size_t i, j;
    for (i = 0; i < ilc->n_samples; i++) {
        for (j = 0; j < ilc->n_inputs; j++) {
            double correction = 0.0;
            for (size_t o = 0; o < ilc->n_outputs; o++) {
                correction += ilc->L_matrix[j * ilc->n_outputs + o] *
                              error[i * ilc->n_outputs + o];
            }
            control[i * ilc->n_inputs + j] =
                ilc->control_memory[i * ilc->n_inputs + j] + correction;
        }
    }
    memcpy(ilc->error_memory, error, ilc->n_samples * ilc->n_outputs * sizeof(double));
    memcpy(ilc->control_memory, control, ilc->n_samples * ilc->n_inputs * sizeof(double));
}

void ic_ilc_reset(ic_ilc_t *ilc) {
    if (!ilc) return;
    memset(ilc->error_memory, 0, ilc->n_samples * ilc->n_outputs * sizeof(double));
    memset(ilc->control_memory, 0, ilc->n_samples * ilc->n_inputs * sizeof(double));
}

/* ---- L6: Fuzzy-PID Hybrid Controller ---- */

int ic_fuzzy_pid_init(ic_fuzzy_pid_t *fpid, double Kp0, double Ki0,
                       double Kd0, double u_min, double u_max) {
    if (!fpid) return -1;
    memset(fpid, 0, sizeof(ic_fuzzy_pid_t));
    fpid->Kp0 = Kp0; fpid->Ki0 = Ki0; fpid->Kd0 = Kd0;
    fpid->Kp = Kp0; fpid->Ki = Ki0; fpid->Kd = Kd0;
    fpid->u_min = u_min; fpid->u_max = u_max;
    fpid->anti_windup = 1;
    return 0;
}

void ic_fuzzy_pid_free(ic_fuzzy_pid_t *fpid) {
    if (!fpid) return;
    if (fpid->fis_kp) { ic_fuzzy_free(fpid->fis_kp); free(fpid->fis_kp); }
    if (fpid->fis_ki) { ic_fuzzy_free(fpid->fis_ki); free(fpid->fis_ki); }
    if (fpid->fis_kd) { ic_fuzzy_free(fpid->fis_kd); free(fpid->fis_kd); }
    memset(fpid, 0, sizeof(ic_fuzzy_pid_t));
}

double ic_fuzzy_pid_control(ic_fuzzy_pid_t *fpid, double setpoint,
                              double measurement, double dt) {
    if (!fpid || dt < 1e-12) return 0.0;
    fpid->e_prev = fpid->e;
    fpid->e = setpoint - measurement;
    fpid->de = (fpid->e - fpid->e_prev) / dt;
    fpid->integral += fpid->e * dt;
    if (fpid->anti_windup) {
        double u_p = fpid->Kp * fpid->e;
        double max_i = (fpid->u_max - u_p) / (fpid->Ki + 1e-10);
        double min_i = (fpid->u_min - u_p) / (fpid->Ki + 1e-10);
        if (fpid->integral > max_i) fpid->integral = max_i;
        if (fpid->integral < min_i) fpid->integral = min_i;
    }
    double u = fpid->Kp * fpid->e + fpid->Ki * fpid->integral + fpid->Kd * fpid->de;
    if (u > fpid->u_max) u = fpid->u_max;
    if (u < fpid->u_min) u = fpid->u_min;
    if (fpid->fis_kp && fpid->fis_kp->num_rules > 0) {
        double fi[2] = {fpid->e, fpid->de};
        double dK;
        ic_fuzzy_control_step(fpid->fis_kp, fi, &dK);
        fpid->Kp = fpid->Kp0 * (1.0 + 0.5 * dK);
    }
    return u;
}

/* ---- L5: ANFIS (Jang 1993) ---- */

int ic_anfis_init(ic_anfis_t *anfis, size_t num_inputs, size_t num_rules) {
    if (!anfis || num_inputs == 0 || num_rules == 0) return -1;
    memset(anfis, 0, sizeof(ic_anfis_t));
    anfis->num_mf = num_inputs * num_rules;
    anfis->learning_rate_premise = 0.001;
    anfis->learning_rate_consequent = 0.01;
    anfis->premise_params = (double*)calloc(anfis->num_mf * 3, sizeof(double));
    anfis->consequent_params = (double*)calloc(num_rules * (num_inputs + 1), sizeof(double));
    if (!anfis->premise_params || !anfis->consequent_params) {
        ic_anfis_free(anfis); return -1;
    }
    ic_fuzzy_init(&anfis->fis, num_inputs, 1, 1);
    return 0;
}

void ic_anfis_free(ic_anfis_t *anfis) {
    if (!anfis) return;
    ic_fuzzy_free(&anfis->fis);
    free(anfis->premise_params); free(anfis->consequent_params);
    memset(anfis, 0, sizeof(ic_anfis_t));
}

double ic_anfis_output(ic_anfis_t *anfis, const double *inputs) {
    if (!anfis || !inputs) return 0.0;
    double outputs[1];
    ic_fuzzy_infer_tsk(&anfis->fis, inputs, outputs);
    return outputs[0];
}

void ic_anfis_train_step(ic_anfis_t *anfis, const double *inputs, double target) {
    if (!anfis || !inputs) return;
    double output = ic_anfis_output(anfis, inputs);
    double error = target - output;
    for (size_t r = 0; r < anfis->fis.num_rules; r++) {
        for (size_t j = 0; j <= anfis->fis.num_inputs; j++) {
            double xj = (j == 0) ? 1.0 : inputs[j - 1];
            anfis->consequent_params[r * (anfis->fis.num_inputs + 1) + j] +=
                anfis->learning_rate_consequent * error * xj;
        }
    }
    for (size_t i = 0; i < anfis->num_mf; i++) {
        anfis->premise_params[i * 3] += anfis->learning_rate_premise * error * 0.01;
    }
}
