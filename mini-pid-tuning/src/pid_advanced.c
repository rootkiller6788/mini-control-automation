/**
 * @file pid_advanced.c
 * @brief Advanced PID Control Structures Implementation
 *
 * Implements:
 *   L6 -- Cascade control, feedforward control, gain scheduling,
 *         ratio control, override control
 *   L8 -- Nonlinear PID, fractional-order PID, event-based PID,
 *         MRAC adaptive PID
 *
 * References:
 *   Astrom & Hagglund (2006), "Advanced PID Control", ISA
 *   Podlubny (1999), "Fractional-order systems and PID controllers",
 *     IEEE Trans. Automatic Control, 44(1), pp. 208-214
 *   Han (1994), "Nonlinear PID Controller", ACTA Automatica Sinica
 *   Astrom & Wittenmark (1995), "Adaptive Control", 2nd Ed., Addison-Wesley
 */

#include "pid_advanced.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/*===========================================================================
 * L6 -- Cascade PID
 *===========================================================================*/

void cascade_pid_init(CascadePID *cas, PIDForm p_form, PIDForm s_form) {
    if (!cas) return;
    pid_init(&cas->primary, p_form);
    pid_init(&cas->secondary, s_form);
    cas->secondary_setpoint = 0.0;
    cas->secondary_measurement = 0.0;
    cas->cascade_active = true;

    /* Default: secondary loop faster than primary */
    cas->primary.params.Ts = 0.1;
    cas->secondary.params.Ts = 0.01;
}

double cascade_pid_update(CascadePID *cas, double setpoint,
                          double primary_measurement,
                          double secondary_measurement) {
    if (!cas) return 0.0;

    /* Primary loop: computes setpoint for secondary */
    double u_primary = pid_update(&cas->primary, setpoint, primary_measurement);
    cas->secondary_setpoint = u_primary;
    cas->secondary_measurement = secondary_measurement;

    /* Secondary loop: controls the actual process input */
    double u_secondary = pid_update(&cas->secondary, u_primary, secondary_measurement);

    /* Anti-windup coordination: if secondary saturates, inform primary */
    /* (Simplified: the secondary anti-windup handles this internally) */

    return u_secondary;
}

/*===========================================================================
 * L6 -- Feedforward PID
 *===========================================================================*/

void feedforward_pid_init(FeedforwardPID *ffpid, PIDForm form) {
    if (!ffpid) return;
    pid_init(&ffpid->pid, form);
    ffpid->Kff_static = 0.0;
    ffpid->Kff_dynamic = 0.0;
    ffpid->ff_lead = 0.0;
    ffpid->ff_lag = 1.0;
    ffpid->prev_disturbance = 0.0;
    ffpid->ff_output = 0.0;
}

double feedforward_pid_update(FeedforwardPID *ffpid, double setpoint,
                              double measurement, double disturbance) {
    if (!ffpid) return 0.0;

    /* Static feedforward */
    double u_ff_static = ffpid->Kff_static * disturbance;

    /* Dynamic feedforward: (ff_lead*s + 1)/(ff_lag*s + 1) * disturbance
     * Discretized using backward Euler:
     *   y(k) = (ff_lag*y(k-1) + ff_lead*(d(k)-d(k-1))/Ts + d(k)*Ts)
     *               / (ff_lag + Ts)   ... approximately
     *
     * Using simplified discrete-time filter:
     *   y(k) = alpha*y(k-1) + beta*d(k) + gamma*d(k-1)
     *   where alpha = ff_lag/(ff_lag+Ts)
     *         beta = (ff_lead + Ts)/(ff_lag + Ts) * Kff_dynamic
     *         gamma = -ff_lead/(ff_lag + Ts) * Kff_dynamic
     */
    double Ts = ffpid->pid.params.Ts;
    double u_ff_dynamic = 0.0;
    if (Ts > 0.0 && ffpid->ff_lag > 1e-12) {
        double alpha = ffpid->ff_lag / (ffpid->ff_lag + Ts);
        double beta = (ffpid->ff_lead + Ts) / (ffpid->ff_lag + Ts) * ffpid->Kff_dynamic;
        double gamma = -ffpid->ff_lead / (ffpid->ff_lag + Ts) * ffpid->Kff_dynamic;

        u_ff_dynamic = alpha * ffpid->ff_output +
                       beta * disturbance +
                       gamma * ffpid->prev_disturbance;
        ffpid->ff_output = u_ff_dynamic;
    }

    ffpid->prev_disturbance = disturbance;

    double u_fb = pid_update(&ffpid->pid, setpoint, measurement);

    return u_fb + u_ff_static + u_ff_dynamic;
}

/*===========================================================================
 * L6 -- Gain-Scheduled PID
 *===========================================================================*/

void gs_pid_init(GainScheduledPID *gspid, PIDForm form,
                 const GainScheduleEntry *table, size_t table_size) {
    if (!gspid) return;
    pid_init(&gspid->pid, form);
    gspid->table = (GainScheduleEntry*)table;
    gspid->table_size = table_size;
    gspid->current_sv = 0.0;
    gspid->enabled = (table != NULL && table_size >= 2);
}

static void interpolate_gains(const GainScheduleEntry *table, size_t size,
                              double sv, double *Kp, double *Ki, double *Kd) {
    if (!table || size == 0) return;

    /* Handle out-of-range */
    if (sv <= table[0].scheduling_variable) {
        *Kp = table[0].Kp;
        *Ki = table[0].Ki;
        *Kd = table[0].Kd;
        return;
    }
    if (sv >= table[size - 1].scheduling_variable) {
        *Kp = table[size - 1].Kp;
        *Ki = table[size - 1].Ki;
        *Kd = table[size - 1].Kd;
        return;
    }

    /* Binary search for interval */
    size_t lo = 0, hi = size - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (table[mid].scheduling_variable <= sv) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    /* Linear interpolation */
    double x0 = table[lo].scheduling_variable;
    double x1 = table[hi].scheduling_variable;
    double frac = (sv - x0) / (x1 - x0 + 1e-30);
    frac = frac < 0.0 ? 0.0 : (frac > 1.0 ? 1.0 : frac);

    *Kp = table[lo].Kp + frac * (table[hi].Kp - table[lo].Kp);
    *Ki = table[lo].Ki + frac * (table[hi].Ki - table[lo].Ki);
    *Kd = table[lo].Kd + frac * (table[hi].Kd - table[lo].Kd);
}

double gs_pid_update(GainScheduledPID *gspid, double setpoint,
                     double measurement, double sched_var) {
    if (!gspid) return 0.0;

    gspid->current_sv = sched_var;

    if (gspid->enabled) {
        double Kp = 0.0, Ki = 0.0, Kd = 0.0;
        interpolate_gains(gspid->table, gspid->table_size, sched_var, &Kp, &Ki, &Kd);
        gspid->pid.params.Kp = Kp;
        gspid->pid.params.Ki = Ki;
        gspid->pid.params.Kd = Kd;
    }

    return pid_update(&gspid->pid, setpoint, measurement);
}

/*===========================================================================
 * L8 -- Nonlinear PID
 *===========================================================================*/

void nonlinear_pid_init(NonlinearPID *nlpid, PIDForm form) {
    if (!nlpid) return;
    pid_init(&nlpid->pid, form);
    nlpid->nonlinear_gain = nl_gain_tanh;
    nlpid->nlp_param = 1.0;
    nlpid->error_threshold = 0.1;
    nlpid->use_nonlinear_integral = false;
}

double nonlinear_pid_update(NonlinearPID *nlpid, double setpoint,
                            double measurement) {
    if (!nlpid) return 0.0;
    if (!nlpid->nonlinear_gain) return pid_update(&nlpid->pid, setpoint, measurement);

    double error = setpoint - measurement;
    double abs_error = fabs(error);

    /* Modify gain based on error magnitude */
    double gain_factor = nlpid->nonlinear_gain(abs_error, nlpid->nlp_param);

    /* Save original gains */
    double orig_Kp = nlpid->pid.params.Kp;
    double orig_Ki = nlpid->pid.params.Ki;
    double orig_Kd = nlpid->pid.params.Kd;

    /* Apply nonlinear scaling */
    nlpid->pid.params.Kp *= gain_factor;

    /* Conditional integration: only integrate when error is small */
    if (nlpid->use_nonlinear_integral && abs_error > nlpid->error_threshold) {
        nlpid->pid.params.Ki = 0.0;
    } else if (nlpid->use_nonlinear_integral) {
        nlpid->pid.params.Ki = orig_Ki * gain_factor;
    }

    nlpid->pid.params.Kd *= gain_factor;

    double u = pid_update(&nlpid->pid, setpoint, measurement);

    /* Restore gains */
    nlpid->pid.params.Kp = orig_Kp;
    nlpid->pid.params.Ki = orig_Ki;
    nlpid->pid.params.Kd = orig_Kd;

    return u;
}

/* Nonlinear gain functions */

double nl_gain_tanh(double error, double param) {
    /* Hyperbolic tangent: gain increases with error, saturates.
     * f(|e|) = 1 + param * tanh(|e|)
     * Small errors: gain ~ 1
     * Large errors: gain ~ 1 + param
     */
    return 1.0 + param * tanh(error);
}

double nl_gain_deadzone(double error, double param) {
    /* Deadzone: no gain change within threshold.
     * f(|e|) = 1.0                               if |e| < param
     * f(|e|) = 1.0 + (|e| - param) / param      if |e| >= param
     */
    if (error < param) return 1.0;
    return 1.0 + (error - param) / param;
}

double nl_gain_quadratic(double error, double param) {
    /* Quadratic: gain ~ |e|^2
     * f(|e|) = 1.0 + param * |e|^2
     */
    return 1.0 + param * error * error;
}

/*===========================================================================
 * L8 -- Fractional-Order PID
 *===========================================================================*/

void fopid_init(FractionalOrderPID *fopid, PIDForm form,
                double lambda, double mu, size_t N_order) {
    if (!fopid) return;
    pid_init(&fopid->pid, form);
    fopid->lambda = lambda;
    fopid->mu = mu;
    fopid->filter_order = (N_order < 2) ? 2 : (N_order > 10 ? 10 : N_order);

    /* Allocate Oustaloup filter state
     * The fractional integrator s^(-lambda) and differentiator s^mu
     * are each approximated by an Nth-order transfer function:
     *   s^alpha ? K * prod_{k=1}^{N} (s + w'_k) / (s + w_k)
     *
     * In state-space, this requires 2*N_order states each.
     */
    size_t n = fopid->filter_order;
    fopid->frac_int_state = (double*)calloc(2 * n, sizeof(double));
    fopid->frac_diff_state = (double*)calloc(2 * n, sizeof(double));

    if (!fopid->frac_int_state || !fopid->frac_diff_state) {
        fopid_free(fopid);
    }
}

double fopid_update(FractionalOrderPID *fopid, double setpoint,
                    double measurement) {
    if (!fopid || !fopid->frac_int_state || !fopid->frac_diff_state) return 0.0;

    double error = setpoint - measurement;
    double Ts = fopid->pid.params.Ts;
    if (Ts <= 0.0) Ts = 0.01;

    /* Fractional integral: s^(-lambda)
     * In discrete time, this is approximated by a truncated
     * Grunwald-Letnikov series:
     *   I_f(k) = Ts^lambda * sum_{j=0}^{k} w_j * e(k-j)
     *   where w_j = (1 - (1-lambda)/j) * w_{j-1}, w_0 = 1
     *
     * For efficiency, we approximate with an IIR filter (Oustaloup).
     * Since full Oustaloup requires frequency-domain design, here we
     * provide a simplified version using the GL coefficients truncated
     * to the filter order.
     *
     * Simplified: use a first-order IIR approximation:
     *   I_f(k) = alpha_I * I_f(k-1) + (1-alpha_I) * Ts^lambda * error
     *   where alpha_I = exp(-Ts / T_I) for some equivalent time constant.
     * For lambda=1: alpha_I = 1 (pure integrator)
     * For lambda=0: alpha_I = 0 (no memory)
     */

    /* Fractional derivative: s^mu
     * Similarly approximated:
     *   D_f(k) = alpha_D * D_f(k-1) + (1-alpha_D) * (error - prev_error) / Ts^mu
     */

    /* Compute fractional terms using Grunwald-Letnikov with binomial
     * coefficient recurrence truncated to filter_order terms.
     * w_0 = 1, w_j = w_{j-1} * (j - 1 - alpha) / j
     *
     * Implementation uses the state buffer as a FIFO of past errors.
     */
    double lambda = fopid->lambda;
    double mu = fopid->mu;
    size_t N = fopid->filter_order;

    /* Shift error history (FIFO) */
    /* State layout: [e(k), e(k-1), ..., e(k-N+1)] for integrator
     *               [e(k), e(k-1), ..., e(k-N+1)] for differentiator */
    for (size_t i = N - 1; i > 0; i--) {
        fopid->frac_int_state[i] = fopid->frac_int_state[i - 1];
        fopid->frac_diff_state[i] = fopid->frac_diff_state[i - 1];
    }
    fopid->frac_int_state[0] = error;
    fopid->frac_diff_state[0] = error;

    /* GL coefficients for integrator (s^{-lambda}):
     * w_0 = 1, w_j = w_{j-1} * (j - 1 + lambda) / j */
    double wi[12] = {1.0, 0};
    for (size_t j = 1; j < N && j < 12; j++) {
        wi[j] = wi[j - 1] * (j - 1.0 + lambda) / j;
    }

    /* GL coefficients for differentiator (s^{mu}):
     * w_0 = 1, w_j = w_{j-1} * (j - 1 - mu) / j */
    double wd[12] = {1.0, 0};
    for (size_t j = 1; j < N && j < 12; j++) {
        wd[j] = wd[j - 1] * (j - 1.0 - mu) / j;
    }

    /* Compute fractional integral and derivative */
    double I_frac = 0.0, D_frac = 0.0;
    double Ts_pow_lambda = pow(Ts, lambda);
    double Ts_pow_mu = pow(Ts, mu);

    for (size_t j = 0; j < N && j < 12; j++) {
        I_frac += wi[j] * fopid->frac_int_state[j];
        D_frac += wd[j] * fopid->frac_diff_state[j];
    }
    I_frac *= Ts_pow_lambda;
    D_frac /= Ts_pow_mu;

    /* PID output: u = Kp*e + Ki*I_frac + Kd*D_frac */
    double Kp = fopid->pid.params.Kp;
    double Ki = fopid->pid.params.Ki;
    double Kd = fopid->pid.params.Kd;

    double u = Kp * error + Ki * I_frac + Kd * D_frac;

    /* Output limits */
    if (u < fopid->pid.out_min) u = fopid->pid.out_min;
    if (u > fopid->pid.out_max) u = fopid->pid.out_max;

    fopid->pid.state.prev_output = u;
    fopid->pid.state.prev_error = error;
    fopid->pid.state.sample_count++;

    return u;
}

void fopid_free(FractionalOrderPID *fopid) {
    if (!fopid) return;
    free(fopid->frac_int_state);
    free(fopid->frac_diff_state);
    fopid->frac_int_state = NULL;
    fopid->frac_diff_state = NULL;
}

/*===========================================================================
 * L8 -- Event-Based PID
 *===========================================================================*/

void eb_pid_init(EventBasedPID *ebpid, PIDForm form,
                 double delta_threshold, double timeout) {
    if (!ebpid) return;
    pid_init(&ebpid->pid, form);
    ebpid->delta_threshold = delta_threshold;
    ebpid->timeout = timeout;
    ebpid->last_update_time = 0.0;
    ebpid->last_sent_measurement = 0.0;
    ebpid->time_since_update = 0.0;
    ebpid->last_output = 0.0;
    ebpid->update_count = 0;
    ebpid->sample_count = 0;
}

double eb_pid_update(EventBasedPID *ebpid, double setpoint,
                     double measurement, double dt, int *updated) {
    if (!ebpid) {
        if (updated) *updated = 0;
        return 0.0;
    }

    ebpid->sample_count++;
    ebpid->time_since_update += dt;

    int trigger = 0;

    /* Trigger condition 1: measurement change exceeds threshold */
    if (fabs(measurement - ebpid->last_sent_measurement) >= ebpid->delta_threshold) {
        trigger = 1;
    }

    /* Trigger condition 2: timeout exceeded */
    if (ebpid->timeout > 0.0 && ebpid->time_since_update >= ebpid->timeout) {
        trigger = 1;
    }

    if (trigger) {
        ebpid->last_output = pid_update(&ebpid->pid, setpoint, measurement);
        ebpid->last_sent_measurement = measurement;
        ebpid->last_update_time += ebpid->time_since_update;
        ebpid->time_since_update = 0.0;
        ebpid->update_count++;
        if (updated) *updated = 1;
    } else {
        if (updated) *updated = 0;
    }

    return ebpid->last_output;
}

/*===========================================================================
 * L6 -- Ratio PID
 *===========================================================================*/

void ratio_pid_init(RatioPID *rpid, PIDForm form) {
    if (!rpid) return;
    pid_init(&rpid->pid, form);
    rpid->ratio_setpoint = 1.0;
    rpid->y1_measurement = 0.0;
    rpid->y2_measurement = 0.0;
}

double ratio_pid_update(RatioPID *rpid, double y1, double y2) {
    if (!rpid) return 0.0;

    rpid->y1_measurement = y1;
    rpid->y2_measurement = y2;

    /* Target: y1 = ratio * y2
     * Error: e = y1_desired - y1_actual = ratio*y2 - y1
     * Or equivalently: e = ratio - y1/y2 (normalized)
     *
     * Using the first form (linear in measurements):
     */
    double setpoint_y1 = rpid->ratio_setpoint * y2;
    return pid_update(&rpid->pid, setpoint_y1, y1);
}

/*===========================================================================
 * L8 -- MRAC-PID (Model Reference Adaptive Control)
 *===========================================================================*/

void mrac_pid_init(MRACPID *mrac, PIDForm form,
                   const FOPDTModel *ref_model, double adapt_gain) {
    if (!mrac) return;
    pid_init(&mrac->pid, form);
    mrac->process_model.K = 1.0;
    mrac->process_model.T = 1.0;
    mrac->process_model.L = 0.0;
    mrac->ref_model = *ref_model;
    mrac->adaptation_gain = adapt_gain;
    mrac->prev_error = 0.0;
    mrac->model_output = 0.0;
    mrac->model_state = 0.0;
}

double mrac_pid_update(MRACPID *mrac, double setpoint, double measurement,
                       double dt) {
    if (!mrac) return 0.0;

    /* 1. PID output */
    double u = pid_update(&mrac->pid, setpoint, measurement);

    /* 2. Update reference model output
     * Reference model: 1/(ref_T*s + 1) * exp(-ref_L*s)
     * Discretized:
     *   model_state = model_state + dt*(setpoint - model_state)/ref_T
     *   (delay omitted for simplicity)
     */
    double ref_T = mrac->ref_model.T;
    if (ref_T > 1e-12 && dt > 0.0) {
        mrac->model_state += dt * (setpoint - mrac->model_state) / ref_T;
    } else {
        mrac->model_state = setpoint;
    }
    mrac->model_output = mrac->model_state;

    /* 3. Adaptation error */
    double adapt_error = mrac->model_output - measurement;

    /* 4. Gradient (MIT rule): dKp/dt = -gamma * adapt_error * sensitivity
     * sensitivity = partial derivative of output w.r.t. Kp
     * Approximate: sensitivity ~ e (error signal)
     *
     * dKp = -gamma * adapt_error * e * dt
     */
    double error = setpoint - measurement;
    double dKp = -mrac->adaptation_gain * adapt_error * error * dt;

    /* Apply with bounds */
    double new_Kp = mrac->pid.params.Kp + dKp;
    if (new_Kp < 0.001) new_Kp = 0.001;
    if (new_Kp > 1000.0) new_Kp = 1000.0;
    mrac->pid.params.Kp = new_Kp;

    mrac->prev_error = adapt_error;

    return u;
}
