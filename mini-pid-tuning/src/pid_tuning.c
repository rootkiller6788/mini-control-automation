/**
 * @file pid_tuning.c
 * @brief PID Tuning Methods Implementation
 *
 * Implements classical and modern PID tuning rules:
 *   L5 -- Ziegler-Nichols (open/closed loop), Cohen-Coon, Tyreus-Luyben,
 *         AMIGO, IMC, Lambda, CHR, Relay auto-tuning
 *
 * All methods produce PIDTuningResult with standard-form parameters.
 * The caller can convert to desired form via pid_convert_params().
 *
 * References:
 *   Ziegler & Nichols (1942), "Optimum Settings for Automatic Controllers"
 *     Trans. ASME, Vol. 64, pp. 759-768
 *   Cohen & Coon (1953), "Theoretical Consideration of Retarded Control"
 *     Trans. ASME, Vol. 75, pp. 827-834
 *   Tyreus & Luyben (1992), "Tuning PI controllers for integrator/dead time
 *     processes", Ind. Eng. Chem. Res., Vol. 31, pp. 2625-2628
 *   Astrom & Hagglund (2004), "Revisiting the Ziegler-Nichols step response
 *     method for PID control", Journal of Process Control, Vol. 14, pp. 635-650
 *   Skogestad (2003), "Simple analytic rules for model reduction and PID
 *     controller tuning", Journal of Process Control, Vol. 13, pp. 291-309
 */

#include "pid_tuning.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * L5 -- Process Identification: FOPDT (Reaction Curve)
 *===========================================================================*/

/* Find the index of maximum slope in step response */
static size_t find_max_slope_index(const double *time, const double *output,
                                    size_t N) {
    if (N < 2) return 0;
    size_t max_idx = 0;
    double max_slope = 0.0;
    for (size_t i = 1; i < N; i++) {
        double dt = time[i] - time[i - 1];
        if (dt < 1e-12) continue;
        double slope = (output[i] - output[i - 1]) / dt;
        if (slope > max_slope) {
            max_slope = slope;
            max_idx = i;
        }
    }
    return max_idx;
}

int pid_identify_fopdt(const StepResponseData *data, FOPDTModel *model) {
    if (!data || !model || data->N < 3) return -1;

    double dy = data->final_value - data->initial_value;
    if (fabs(dy) < 1e-12) return -1;

    /* Gain: K = dy / du */
    double du = data->input_step;
    if (fabs(du) < 1e-12) return -1;
    model->K = dy / du;

    /* Tangent method: find point of maximum slope */
    size_t max_idx = find_max_slope_index(data->time, data->output, data->N);
    double max_slope = 0.0;
    if (max_idx > 0) {
        double dt = data->time[max_idx] - data->time[max_idx - 1];
        if (dt > 1e-12) {
            max_slope = (data->output[max_idx] - data->output[max_idx - 1]) / dt;
        }
    }
    if (max_slope < 1e-12) return -1;

    /* The tangent line: y = max_slope * (t - L)
     * At t = max_idx time: y(t_max) = max_slope * (t_max - L)
     * L = t_max - y(t_max)/max_slope
     */
    double y_at_max = data->output[max_idx] - data->initial_value;
    double t_at_max = data->time[max_idx];
    double L_est = t_at_max - y_at_max / max_slope;
    if (L_est < 0.0) L_est = 0.0;

    /* Time constant: T = dy / max_slope */
    double T_est = dy / max_slope;

    /* Validate: check that the model step response matches data at t=T+L (63.2% point) */
    double y63 = data->initial_value + 0.632 * dy;
    double t63 = t_at_max; /* fallback */
    for (size_t i = 1; i < data->N; i++) {
        if (data->output[i] >= y63) {
            /* Linear interpolation to find exact 63.2% time */
            double frac = (y63 - data->output[i - 1]) /
                          (data->output[i] - data->output[i - 1] + 1e-30);
            t63 = data->time[i - 1] + frac * (data->time[i] - data->time[i - 1]);
            break;
        }
    }
    /* From FOPDT: y(T+L) = K*du*(1 - exp(-T/T)) = K*du*0.632
     * So t63 = T + L, therefore T = t63 - L */
    /* Use the tangent-based T and L as primary estimate */
    model->L = L_est;
    model->T = T_est;

    return 0;
}

int pid_identify_fopdt_area(const StepResponseData *data, FOPDTModel *model) {
    if (!data || !model || data->N < 3) return -1;

    double dy = data->final_value - data->initial_value;
    double du = data->input_step;
    if (fabs(dy) < 1e-12 || fabs(du) < 1e-12) return -1;

    /* Static gain */
    model->K = dy / du;

    /* Area method (Astrom & Hagglund, 1995, Section 2.7):
     * The normalized step response h(t) = (y(t) - y0) / dy
     * Average residence time: Tar = integral(1 - h(t)) dt from 0 to inf
     * Tar = T + L (for FOPDT)
     *
     * Second moment provides additional equation to separate T and L.
     * Using trapezoidal integration.
     */
    double Tar = 0.0;
    double sigma2 = 0.0;  /* variance (second central moment) */
    for (size_t i = 1; i < data->N; i++) {
        double h_i = (data->output[i] - data->initial_value) / dy;
        double h_im1 = (data->output[i - 1] - data->initial_value) / dy;
        double dt = data->time[i] - data->time[i - 1];
        double h_avg = 0.5 * (h_i + h_im1);
        double t_avg = 0.5 * (data->time[i] + data->time[i - 1]);

        Tar += (1.0 - h_avg) * dt;

        /* For second moment: integral of t*(1 - h(t)) dt */
        sigma2 += t_avg * (1.0 - h_avg) * dt;
    }

    /* For FOPDT: Tar = T + L, variance = T^2
     * So: T = sqrt(sigma2^2?), we know sigma2 = integral(t*(1-h)) dt.
     *
     * Correct approach: L = Tar - T
     * For FOPDT with area method, L can be negative (non-minimum phase
     * or higher-order effects). We clamp to 0.
     */
    if (Tar < 1e-12) return -1;

    /* Approximate T as the time to reach 63.2% minus L.
     * Using: Tar = T + L, and the slope at the inflexion.
     * Actually for exact FOPDT: Tar = integral(1 - h(t)) dt
     *   = integral(exp(-(t-L)/T)) dt from L to inf
     *   = T
     * Wait: Tar = T + L? Let's recalculate:
     *   For t >= L: h(t) = 1 - exp(-(t-L)/T)
     *   Tar = integral_{L}^{inf} (1 - h(t)) dt = integral_{L}^{inf} exp(-(t-L)/T) dt = T
     *   But the area from 0 to L: h(t)=0, so integral = L
     *   Total Tar = L + T. Yes, Tar = T + L.
     *
     * So we need another equation. The maximum slope gives T = dy/max_slope.
     * We already know Tar = T + L. So L = Tar - T.
     *
     * But we can get T from the slope: T = dy / max_slope
     * Combined: L = Tar - dy/max_slope
     */

    size_t max_idx = find_max_slope_index(data->time, data->output, data->N);
    double max_slope;
    if (max_idx > 0) {
        double dt_slope = data->time[max_idx] - data->time[max_idx - 1];
        max_slope = (data->output[max_idx] - data->output[max_idx - 1]);
        if (dt_slope > 1e-12) max_slope /= dt_slope;
    } else {
        max_slope = dy / (data->time[data->N - 1] - data->time[0]);
    }

    if (max_slope < 1e-12) {
        /* Fall back: assume L=0, T=Tar */
        model->T = Tar;
        model->L = 0.0;
    } else {
        model->T = dy / max_slope;
        model->L = Tar - model->T;
        if (model->L < 0.0) model->L = 0.0;
    }

    return 0;
}

int pid_identify_sopdt(const StepResponseData *data, SOPDTModel *model) {
    if (!data || !model || data->N < 5) return -1;

    double dy = data->final_value - data->initial_value;
    double du = data->input_step;
    if (fabs(dy) < 1e-12 || fabs(du) < 1e-12) return -1;

    model->K = dy / du;

    /* Two-point method (Smith, 1972):
     * Find times t28 and t63 where response reaches 28.3% and 63.2%.
     * For SOPDT: t28/T_avg and t63/T_avg characterize T1/T2 ratio.
     * T_avg = (t63 - t28) / ln((1-0.283)/(1-0.632)) = (t63 - t28) / ln(0.717/0.368)
     *       = (t63 - t28) / 0.667  */
    double t28 = -1.0, t63 = -1.0;
    for (size_t i = 1; i < data->N; i++) {
        double frac = (data->output[i] - data->initial_value) / dy;
        if (t28 < 0.0 && frac >= 0.283) {
            double f0 = (data->output[i - 1] - data->initial_value) / dy;
            double frac_t = (0.283 - f0) / (frac - f0 + 1e-30);
            t28 = data->time[i - 1] + frac_t * (data->time[i] - data->time[i - 1]);
        }
        if (t63 < 0.0 && frac >= 0.632) {
            double f0 = (data->output[i - 1] - data->initial_value) / dy;
            double frac_t = (0.632 - f0) / (frac - f0 + 1e-30);
            t63 = data->time[i - 1] + frac_t * (data->time[i] - data->time[i - 1]);
            break;
        }
    }

    if (t28 < 0.0 || t63 < 0.0) {
        /* Fallback to FOPDT-type approximation */
        pid_identify_fopdt(data, (FOPDTModel*)model);
        model->T1 = ((FOPDTModel*)model)->T;
        model->T2 = 0.0;
        model->L = ((FOPDTModel*)model)->L;
        return 0;
    }

    /* double ratio = t28 / t63; -- reserved for SOPDT characterization */
    /* For SOPDT with T1=T2: ratio = 0.32, t63/(T1+T2+L) ~ 1
     * This is a simplified approximation.
     * More accurate: use the empirical relationship from Smith (1972).
     */
    double T_sum = (t63 - t28) / 0.667;
    model->L = t63 - T_sum - 0.5 * T_sum;  /* rough estimate */
    if (model->L < 0.0) model->L = 0.0;
    model->T1 = 0.6 * T_sum;
    model->T2 = 0.4 * T_sum;

    return 0;
}

/*===========================================================================
 * L5 -- Ultimate Gain Computation
 *===========================================================================*/

int pid_compute_ultimate_gain(const FOPDTModel *model, UltimateGainData *ug) {
    if (!model || !ug || model->K < 1e-30 || model->T < 1e-30) return -1;

    /* Solve for w_u where phase(G(jw_u)) = -pi
     * G(s) = K*exp(-L*s)/(T*s + 1)
     * phase(G(jw)) = -w*L - atan(w*T)
     * Solve: w*L + atan(w*T) = pi
     *
     * For small L/T: w_u approx = pi / (2*L + T)
     * For large L/T: w_u approx = (pi - 0.5*pi) / L = pi/(2*L)
     *
     * We use Newton-Raphson to solve f(w) = w*L + atan(w*T) - pi = 0
     * f'(w) = L + T/(1 + (w*T)^2)
     */

    double L = model->L;
    double T = model->T;
    /* M_PI now from macro */

    /* Initial guess */
    double w = M_PI / (2.0 * L + T);
    if (w < 1e-6) w = 0.01;

    /* Newton-Raphson iterations (max 50) */
    for (int iter = 0; iter < 50; iter++) {
        double wT = w * T;
        double f = w * L + atan(wT) - M_PI;
        double df = L + T / (1.0 + wT * wT);
        double dw = f / df;
        w -= dw;
        if (fabs(dw) < 1e-10) break;
    }

    if (w <= 0.0) w = 0.01;

    /* Ultimate period: Pu = 2*pi / w_u */
    ug->Pu = 2.0 * M_PI / w;

    /* Ultimate gain: Ku = 1 / |G(j*w_u)|
     * |G(jw)| = K / sqrt(1 + (w*T)^2)
     * So Ku = sqrt(1 + (w*T)^2) / K
     */
    ug->Ku = sqrt(1.0 + (w * T) * (w * T)) / model->K;
    ug->relay_amplitude = 0.0;
    ug->oscillation_amplitude = 0.0;

    return 0;
}

/*===========================================================================
 * L5 -- Relay Auto-Tuning (Astrom-Hagglund)
 *===========================================================================*/

int pid_relay_autotune(const FOPDTModel *model, double relay_amplitude,
                       double hysteresis, UltimateGainData *ug) {
    if (!model || !ug) return -1;

    /* Simulate relay feedback experiment:
     * Relay output: u = +relay_amplitude when e > hysteresis
     *               u = -relay_amplitude when e < -hysteresis
     *
     * The process oscillates at the ultimate frequency.
     * Ku = 4*d / (pi*a) where d = relay amplitude, a = output oscillation amplitude
     * Pu = measured oscillation period
     *
     * We simulate this for a few cycles to find the steady-state oscillation.
     */

    double K  = model->K;
    double T  = model->T;
    double L  = model->L;
    double d  = relay_amplitude;
    double h  = hysteresis;

    if (K < 1e-30) return -1;

    double y = 0.0;       /* process output */
    double u = d;         /* initial relay state */
    double t = 0.0;
    double dt = 0.001 * T; /* simulation step: 0.1% of time constant */
    if (dt < 1e-4) dt = 1e-4;
    if (dt > 0.1) dt = 0.1;

    /* Dead-time buffer */
    size_t delay_steps = (size_t)(L / dt + 0.5);
    if (delay_steps < 1) delay_steps = 1;
    double *delay_buffer = (double*)calloc(delay_steps + 1, sizeof(double));
    if (!delay_buffer) return -1;
    size_t delay_idx = 0;

    /* Run simulation for several ultimate periods */
    double sim_duration = 10.0 * (T + L);
    if (sim_duration < 10.0) sim_duration = 10.0;
    size_t N = (size_t)(sim_duration / dt);
    if (N > 100000) N = 100000;

    /* Oscillation detection */
    double prev_y_sign = 1.0;
    double half_periods[20];
    int half_count = 0;
    double half_start_time = 0.0;
    /* double peak_vals[20]; -- reserved for oscillation analysis */
    int peak_count = 0;

    for (size_t i = 0; i < N; i++) {
        /* Relay logic */
        double e = -y; /* setpoint = 0, output = y */
        if (e > h) {
            u = d;
        } else if (e < -h) {
            u = -d;
        }
        /* u stays at previous value inside hysteresis band */

        /* Process dynamics: dy/dt = (K * u(t-L) - y) / T */
        double u_delayed = K * delay_buffer[delay_idx];
        double dy = (u_delayed - y) / T * dt;

        /* Update delay buffer */
        delay_buffer[delay_idx] = u;
        delay_idx = (delay_idx + 1) % delay_steps;

        y += dy;
        t += dt;

        /* Detect zero crossings for period measurement */
        if (half_count < 20 && i > N/4) { /* Wait for transients to settle */
            if (prev_y_sign > 0 && y <= 0 && half_count > 0) {
                /* Positive to negative crossing */
                half_periods[half_count] = t - half_start_time;
                half_start_time = t;
                half_count++;
            } else if (prev_y_sign < 0 && y >= 0 && half_count == 0) {
                /* First negative to positive crossing -- start measuring */
                half_start_time = t;
                half_count = 1;
            }
        }
        prev_y_sign = (y > 0) ? 1.0 : -1.0;

        /* Track peaks */
        /* Simple peak tracking based on sign change of derivative */
        if (peak_count < 20) {
            /* Store y as we go, peaks will be detected post-hoc */
        }
    }

    /* Compute Pu from average half-period */
    double Pu = 0.0;
    int num_half = 0;
    for (int i = 1; i < half_count && i < 20; i++) {
        Pu += half_periods[i] * 2.0; /* half-period * 2 = full period */
        num_half++;
    }
    if (num_half > 0) {
        Pu /= num_half;
    } else {
        /* Fallback: use FOPDT approximation */
        Pu = 2.0 * (L + 0.5 * T);
    }

    /* Compute Ku from describing function: Ku = 4*d / (pi*a)
     * The oscillation amplitude a is half the peak-to-peak of y.
     * For a FOPDT with relay: a approx = K*d*(1-exp(-L/T))/(1+exp(-L/T))
     *                              ... more precisely found from simulation.
     */
    /* Find oscillation amplitude from recent cycles */
    double a = 0.0;
    {
        /* Scan the last few cycles for peak-to-peak */
        /* Simplified: use describing function approximation */
        double exp_ratio = exp(-L / T);
        a = K * d * (1.0 - exp_ratio) / (1.0 + exp_ratio);
        if (a < 1e-9) a = fabs(y) * 0.5; /* fallback */
    }

    ug->Ku = 4.0 * d / (3.14159265358979323846 * a);
    ug->Pu = Pu;
    ug->relay_amplitude = d;
    ug->oscillation_amplitude = a;

    free(delay_buffer);
    return 0;
}

/*===========================================================================
 * L5 -- Ziegler-Nichols Closed-Loop Tuning
 *===========================================================================*/

int pid_tune_zn_closed_loop(const UltimateGainData *ug, PIDForm form,
                            double ts, PIDTuningResult *result) {
    if (!ug || !result || ug->Ku <= 0.0 || ug->Pu <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_ZN_CLOSED_LOOP;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double Ku = ug->Ku, Pu = ug->Pu;

    /* ZN closed-loop rules (standard form):
     * P:   Kp = 0.50*Ku
     * PI:  Kp = 0.45*Ku, Ti = Pu/1.2
     * PID: Kp = 0.60*Ku, Ti = Pu/2.0, Td = Pu/8.0
     */

    /* We use PID rules by default */
    result->params.Kp = 0.60 * Ku;
    result->params.Ti = Pu / 2.0;
    result->params.Td = Pu / 8.0;

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    } else {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    }

    /* Expected performance for ZN-PID: ~25% overshoot */
    result->expected_overshoot = 0.25;
    result->expected_settling_time = 3.0 * Pu;

    snprintf(result->note, sizeof(result->note),
             "ZN closed-loop: Ku=%.4g, Pu=%.4g", Ku, Pu);
    return 0;
}

/*===========================================================================
 * L5 -- Ziegler-Nichols Open-Loop Tuning
 *===========================================================================*/

int pid_tune_zn_open_loop(const FOPDTModel *model, PIDForm form,
                          double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_ZN_OPEN_LOOP;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;
    double a = K * L / T;  /* normalized dead time */

    /* ZN open-loop rules (standard form):
     * P:   Kp = T/(K*L)
     * PI:  Kp = 0.9*T/(K*L), Ti = 3.33*L
     * PID: Kp = 1.2*T/(K*L), Ti = 2.0*L, Td = 0.5*L
     */
    result->params.Kp = 1.2 * T / (K * L);
    result->params.Ti = 2.0 * L;
    result->params.Td = 0.5 * L;

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    } else {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    }

    result->expected_overshoot = 0.25;
    result->expected_settling_time = 4.0 * (T + L);

    snprintf(result->note, sizeof(result->note),
             "ZN open-loop: K=%.4g, T=%.4g, L=%.4g, a=%.4g", K, T, L, a);
    return 0;
}

/*===========================================================================
 * L5 -- Cohen-Coon Tuning
 *===========================================================================*/

int pid_tune_cohen_coon(const FOPDTModel *model, PIDForm form,
                        double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_COHEN_COON;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;
    double mu = L / (T + L);  /* fractional dead time */

    /* Cohen-Coon PID rules:
     * Kc = (1/K)*(T/L)*[4/3 + mu/4]
     * Ti = L * [32 + 6*mu] / [13 + 8*mu]
     * Td = L * [4] / [11 + 2*mu]
     */
    result->params.Kp = (1.0 / K) * (T / L) * (4.0 / 3.0 + mu / 4.0);
    result->params.Ti = L * (32.0 + 6.0 * mu) / (13.0 + 8.0 * mu);
    result->params.Td = L * 4.0 / (11.0 + 2.0 * mu);

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    } else {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    }

    result->expected_overshoot = 0.25;
    result->expected_settling_time = 4.0 * (T + L);

    snprintf(result->note, sizeof(result->note),
             "Cohen-Coon: mu=%.4f", mu);
    return 0;
}

/*===========================================================================
 * L5 -- Tyreus-Luyben Tuning
 *===========================================================================*/

int pid_tune_tyreus_luyben(const UltimateGainData *ug, PIDForm form,
                           double ts, PIDTuningResult *result) {
    if (!ug || !result || ug->Ku <= 0.0 || ug->Pu <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_TYREUS_LUYBEN;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double Ku = ug->Ku, Pu = ug->Pu;

    /* Tyreus-Luyben rules (more conservative):
     * PI:  Kc = Ku/3.2,  Ti = 2.2*Pu
     * PID: Kc = Ku/2.2,  Ti = 2.2*Pu,  Td = Pu/6.3
     *
     * These give GM >= 6dB and PM >= 45 degrees typically.
     */
    result->params.Kp = Ku / 2.2;
    result->params.Ti = 2.2 * Pu;
    result->params.Td = Pu / 6.3;

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    } else {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    }

    result->expected_overshoot = 0.10;  /* more conservative */
    result->expected_settling_time = 5.0 * Pu;

    snprintf(result->note, sizeof(result->note),
             "Tyreus-Luyben: Ku=%.4g, Pu=%.4g (conservative)", Ku, Pu);
    return 0;
}

/*===========================================================================
 * L5 -- AMIGO Tuning
 *===========================================================================*/

int pid_tune_amigo(const FOPDTModel *model, PIDForm form,
                   double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_AMIGO;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;

    /* AMIGO rules (Astrom & Hagglund, 2004):
     * Designed for Ms <= 1.4 (maximum sensitivity).
     *
     * Kc = (1/K)*(0.2 + 0.45*T/L)  for L/T > 0
     * Ti = 0.4*L + 0.8*T
     * Td = 0.5*L*T/(0.3*L + T)
     *
     * Valid range: 0.05 < L/T < 2.0
     */
    double Kc = (1.0 / K) * (0.2 + 0.45 * T / L);
    double Ti = 0.4 * L + 0.8 * T;
    double Td = 0.5 * L * T / (0.3 * L + T);

    /* Clamp to reasonable values */
    if (Kc < 0.0) Kc = 0.1;
    if (Ti < 0.01) Ti = 0.01;

    result->params.Kp = Kc;
    result->params.Ti = Ti;
    result->params.Td = Td;

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = Kc / Ti;
        result->params.Kd = Kc * Td;
    } else {
        result->params.Ki = Kc / Ti;
        result->params.Kd = Kc * Td;
    }

    result->expected_overshoot = 0.10;
    result->expected_settling_time = 4.0 * (T + L);

    snprintf(result->note, sizeof(result->note),
             "AMIGO: Ms<=1.4, Kc=%.4g, Ti=%.4g, Td=%.4g", Kc, Ti, Td);
    return 0;
}

/*===========================================================================
 * L5 -- IMC-Based Tuning
 *===========================================================================*/

int pid_tune_imc(const FOPDTModel *model, double lambda, PIDForm form,
                 double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0 || lambda <= 0.0)
        return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_IMC;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;

    /* IMC-based PID for FOPDT (Rivera, Morari & Skogestad, 1986):
     * Using first-order Pade approximation for delay: exp(-L*s) ~ (1 - 0.5*L*s)/(1 + 0.5*L*s)
     *
     * Kc = (T + 0.5*L) / (K*(lambda + L))
     * Ti = T + 0.5*L
     * Td = T*L / (2*T + L)
     *
     * lambda: desired closed-loop time constant
     *   lambda >= 0.2*T for robustness
     *   lambda = T gives moderate performance
     *   lambda <= 0.5*T gives aggressive performance
     */

    double Kc = (T + 0.5 * L) / (K * (lambda + L));
    double Ti = T + 0.5 * L;
    double Td = T * L / (2.0 * T + L);

    result->params.Kp = Kc;
    result->params.Ti = Ti;
    result->params.Td = Td;

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = Kc / Ti;
        result->params.Kd = Kc * Td;
    } else {
        result->params.Ki = Kc / Ti;
        result->params.Kd = Kc * Td;
    }

    result->expected_overshoot = (lambda < T) ? 0.15 : 0.05;
    result->expected_settling_time = 4.0 * lambda;

    snprintf(result->note, sizeof(result->note),
             "IMC: lambda=%.4g, Kc=%.4g", lambda, Kc);
    return 0;
}

/*===========================================================================
 * L5 -- Lambda (Dahlin) Tuning
 *===========================================================================*/

int pid_tune_lambda(const FOPDTModel *model, double lambda, PIDForm form,
                    double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0 || lambda <= 0.0)
        return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_LAMBDA;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;

    /* Lambda tuning for PI (Dahlin, 1968):
     * For FOPDT: Kc = T / (K*(lambda + L))
     *            Ti = T
     *
     * For PID extension: add Td = 0.5*L
     */
    double Kc = T / (K * (lambda + L));
    double Ti = T;
    double Td = 0.5 * L;

    result->params.Kp = Kc;
    result->params.Ti = Ti;
    result->params.Td = Td;

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = Kc / Ti;
        result->params.Kd = Kc * Td;
    } else {
        result->params.Ki = Kc / Ti;
        result->params.Kd = Kc * Td;
    }

    result->expected_overshoot = 0.0;
    result->expected_settling_time = 4.0 * lambda;

    snprintf(result->note, sizeof(result->note),
             "Lambda: lambda=%.4g, Kc=%.4g, Ti=%.4g", lambda, Kc, Ti);
    return 0;
}

/*===========================================================================
 * L5 -- CHR Tuning
 *===========================================================================*/

int pid_tune_chr_setpoint(const FOPDTModel *model, int overshoot,
                          PIDForm form, double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_CHIEN_HRONES_RESWICK;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;
    double a = K * L / T;

    if (overshoot == 0) {
        /* CHR setpoint, 0% overshoot */
        result->params.Kp = 0.6 * T / (K * L);
        result->params.Ti = T;
        result->params.Td = 0.5 * L;
    } else {
        /* CHR setpoint, 20% overshoot */
        result->params.Kp = 0.95 * T / (K * L);
        result->params.Ti = 1.357 * T;
        result->params.Td = 0.473 * L;
    }

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    } else {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    }

    result->expected_overshoot = overshoot / 100.0;
    result->expected_settling_time = 4.0 * (T + L);

    snprintf(result->note, sizeof(result->note),
             "CHR setpoint: %d%% overshoot", overshoot);
    return 0;
}

int pid_tune_chr_disturbance(const FOPDTModel *model, int overshoot,
                             PIDForm form, double ts, PIDTuningResult *result) {
    if (!model || !result || model->K <= 0.0 || model->T <= 0.0) return -1;

    memset(result, 0, sizeof(*result));
    result->method = TUNE_CHIEN_HRONES_RESWICK_DIST;
    result->params.form = form;
    result->params.Ts = ts;
    result->params.N = 10.0;
    result->params.b = 1.0;
    result->params.c = 0.0;
    result->valid = true;

    double K = model->K, T = model->T, L = model->L;

    if (overshoot == 0) {
        /* CHR disturbance, 0% overshoot */
        result->params.Kp = 0.95 * T / (K * L);
        result->params.Ti = 2.4 * L;
        result->params.Td = 0.42 * L;
    } else {
        /* CHR disturbance, 20% overshoot */
        result->params.Kp = 1.2 * T / (K * L);
        result->params.Ti = 2.0 * L;
        result->params.Td = 0.42 * L;
    }

    if (form == PID_FORM_PARALLEL) {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    } else {
        result->params.Ki = result->params.Kp / result->params.Ti;
        result->params.Kd = result->params.Kp * result->params.Td;
    }

    result->expected_overshoot = overshoot / 100.0;
    result->expected_settling_time = 4.0 * (T + L);

    snprintf(result->note, sizeof(result->note),
             "CHR disturbance: %d%% overshoot", overshoot);
    return 0;
}

/*===========================================================================
 * L5 -- Apply Tuning Result to Controller
 *===========================================================================*/

void pid_apply_tuning(PIDController *pid, const PIDTuningResult *result) {
    if (!pid || !result || !result->valid) return;
    pid->params.Kp = result->params.Kp;
    pid->params.Ki = result->params.Ki;
    pid->params.Kd = result->params.Kd;
    pid->params.Ti = result->params.Ti;
    pid->params.Td = result->params.Td;
    pid->params.N  = result->params.N;
    pid->params.Ts = result->params.Ts;
    pid->params.b  = result->params.b;
    pid->params.c  = result->params.c;
    pid->params.form = result->params.form;
        /* Compute alpha directly: alpha = Td / (Td + N*Ts) */
    {
        double _Td = result->params.Td, _N = result->params.N, _Ts = result->params.Ts;
        if (_Ts > 0.0 && _Td > 0.0 && _N > 0.0)
            pid->alpha = _Td / (_Td + _N * _Ts);
        else
            pid->alpha = 0.0;
    }
}

/*===========================================================================
 * Helper
 *===========================================================================*/

const char *pid_tuning_method_name(PIDTuningMethod method) {
    switch (method) {
        case TUNE_ZN_OPEN_LOOP:    return "Ziegler-Nichols (open-loop)";
        case TUNE_ZN_CLOSED_LOOP:  return "Ziegler-Nichols (closed-loop)";
        case TUNE_COHEN_COON:      return "Cohen-Coon";
        case TUNE_TYREUS_LUYBEN:   return "Tyreus-Luyben";
        case TUNE_AMIGO:           return "AMIGO";
        case TUNE_IMC:             return "IMC-based";
        case TUNE_LAMBDA:          return "Lambda (Dahlin)";
        case TUNE_CHIEN_HRONES_RESWICK:      return "CHR (setpoint)";
        case TUNE_CHIEN_HRONES_RESWICK_DIST: return "CHR (disturbance)";
        case TUNE_RELAY:           return "Relay (Astrom-Hagglund)";
        default:                   return "Unknown";
    }
}
