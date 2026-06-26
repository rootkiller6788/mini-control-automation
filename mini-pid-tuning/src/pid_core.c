/**
 * @file pid_core.c
 * @brief PID Controller Core Implementation
 *
 * Implements the standard discrete-time PID controller with:
 *   - Backward Euler integration
 *   - Backward difference derivative (optionally filtered)
 *   - Configurable anti-windup (clamping, back-calculation, combined)
 *   - Output saturation
 *   - Bumpless transfer (manual/auto switching, external tracking)
 *   - 2-DOF setpoint weighting
 *   - Derivative on measurement (to avoid derivative kick)
 *
 * References:
 *   Astrom & Hagglund (1995), "PID Controllers: Theory, Design, and Tuning"
 *   Astrom & Hagglund (2006), "Advanced PID Control"
 *   Visioli (2006), "Practical PID Control"
 *   Wescott (2016), "PID Without a PhD"
 */

#include "pid_core.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

/*===========================================================================
 * Internal helpers
 *===========================================================================*/

/** Clamp a value to [lo, hi] */
static double clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/** Safe division: return 0 if denominator is near zero */
static double safe_div(double num, double den) {
    if (fabs(den) < 1e-30) return 0.0;
    return num / den;
}

/** Compute derivative filter coefficient alpha from N, Td, Ts.
 *  alpha = Td / (Td + N*Ts) or equivalently 1 / (1 + N*Ts/Td).
 *  When Ts=0 (continuous), alpha=0 (no discrete filtering needed). */
static double compute_alpha(double Td, double N, double Ts) {
    if (Ts <= 0.0 || Td <= 0.0 || N <= 0.0) return 0.0;
    double denom = Td + N * Ts;
    if (denom < 1e-30) return 0.0;
    return Td / denom;
}

/*===========================================================================
 * L2 -- Initialization Functions
 *===========================================================================*/

void pid_init(PIDController *pid, PIDForm form) {
    if (!pid) return;
    memset(pid, 0, sizeof(*pid));
    pid->params.Kp = 1.0;
    pid->params.Ki = 0.0;
    pid->params.Kd = 0.0;
    pid->params.Ti = 0.0;
    pid->params.Td = 0.0;
    pid->params.N  = 10.0;
    pid->params.Ts = 0.01;
    pid->params.b  = 1.0;
    pid->params.c  = 0.0;
    pid->params.Kff = 0.0;
    pid->params.form = form;
    pid->mode = PID_MODE_AUTO;
    pid->dterm_mode = PID_DTERM_MEASUREMENT;
    pid->aw_method = PID_AW_CLAMPING;
    pid->out_min = -DBL_MAX;
    pid->out_max = DBL_MAX;
    pid->int_min = -DBL_MAX;
    pid->int_max = DBL_MAX;
    pid->aw_gain = 1.0;
    pid->alpha = compute_alpha(pid->params.Td, pid->params.N, pid->params.Ts);
    pid->tracking_enabled = false;
    pid->state.initialized = true;
    pid->state.sample_count = 0;
}

void pid_init_params(PIDController *pid, PIDForm form,
                     double Kp, double Ki, double Kd, double Ts) {
    pid_init(pid, form);
    if (!pid) return;
    pid->params.Kp = Kp;
    pid->params.Ki = Ki;
    pid->params.Kd = Kd;
    pid->params.Ts = Ts;
    /* Convert to standard form for internal consistency */
    if (form == PID_FORM_PARALLEL && Kp > 1e-30) {
        pid->params.Ti = safe_div(Kp, Ki);
        pid->params.Td = safe_div(Kd, Kp);
    }
    pid->alpha = compute_alpha(pid->params.Td, pid->params.N, Ts);
}

void pid_set_standard_gains(PIDController *pid, double Kp, double Ti, double Td) {
    if (!pid) return;
    pid->params.Kp = Kp;
    pid->params.Ti = Ti;
    pid->params.Td = Td;
    if (pid->params.form == PID_FORM_PARALLEL) {
        pid->params.Ki = safe_div(Kp, Ti);
        pid->params.Kd = Kp * Td;
    } else if (pid->params.form == PID_FORM_STANDARD) {
        pid->params.Ki = safe_div(Kp, Ti);
        pid->params.Kd = Kp * Td;
    }
    pid->alpha = compute_alpha(pid->params.Td, pid->params.N, pid->params.Ts);
}

void pid_set_output_limits(PIDController *pid, double out_min, double out_max) {
    if (!pid) return;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_set_integral_limits(PIDController *pid, double int_min, double int_max) {
    if (!pid) return;
    pid->int_min = int_min;
    pid->int_max = int_max;
}

void pid_set_antiwindup(PIDController *pid, PIDAntiWindup method, double aw_gain) {
    if (!pid) return;
    pid->aw_method = method;
    if (aw_gain > 0.0) {
        pid->aw_gain = aw_gain;
    }
}

void pid_set_derivative_mode(PIDController *pid, PIDDerivativeMode mode) {
    if (!pid) return;
    pid->dterm_mode = mode;
}

void pid_set_setpoint_weights(PIDController *pid, double b, double c) {
    if (!pid) return;
    pid->params.b = clamp(b, 0.0, 1.0);
    pid->params.c = clamp(c, 0.0, 1.0);
}

void pid_set_derivative_filter(PIDController *pid, double N) {
    if (!pid) return;
    if (N < 1.0) N = 1.0;
    pid->params.N = N;
    pid->alpha = compute_alpha(pid->params.Td, N, pid->params.Ts);
}

/*===========================================================================
 * L2 -- Runtime: pid_update() ? the core algorithm
 *===========================================================================*/

double pid_update(PIDController *pid, double setpoint, double measurement) {
    if (!pid || !pid->state.initialized) return 0.0;

    double Kp = pid->params.Kp;
    double Ki = pid->params.Ki;
    double Kd = pid->params.Kd;
    double Ts = pid->params.Ts;
    double b  = pid->params.b;
    /* double c = pid->params.c; -- reserved for 2-DOF derivative setpoint weighting */

    /* ---- 1. Compute error ---- */
    double error = setpoint - measurement;

    /* ---- 2. Proportional term with 2-DOF setpoint weighting ----
     * Standard 1-DOF: P = Kp * error
     * 2-DOF:          P = Kp * (b * setpoint - measurement)
     *   b=0 gives I-PD (no proportional kick on setpoint change)
     *   b=1 gives standard PID
     */
    double p_term = Kp * (b * setpoint - measurement);

    /* ---- 3. Integral term with anti-windup ----
     * Backward Euler: I(k) = I(k-1) + Ki * Ts * e(k)
     *
     * Anti-windup methods (executed AFTER computing raw I):
     *   CLAMPING:      I = clamp(I, int_min, int_max)
     *   BACK_CALC:     I += Ts * Kaw * (u_sat - u_unsat)  (tracking)
     *   COMBINED:      clamp first, then back-calc
     */
    double integral = pid->state.integral;
    integral += Ki * Ts * error;

    /* Clamping anti-windup: limit integrator directly */
    if (pid->aw_method == PID_AW_CLAMPING || pid->aw_method == PID_AW_COMBINED) {
        integral = clamp(integral, pid->int_min, pid->int_max);
    }

    double i_term = integral;

    /* ---- 4. Derivative term ----
     * D_term on measurement (default): D = Kd * (y(k-1) - y(k)) / Ts
     *   This avoids derivative kick because setpoint changes don't
     *   affect the derivative term. Equivalent to c=0 in 2-DOF.
     *
     * D_term on error: D = Kd * (e(k) - e(k-1)) / Ts
     *
     * With low-pass filter: D_f(k) = alpha * D_f(k-1) + (1-alpha) * D(k)
     *   where alpha = Td / (Td + N*Ts)
     */

    double d_term_raw;
    if (pid->dterm_mode == PID_DTERM_MEASUREMENT) {
        /* Derivative on measurement: -Kd * (y(k) - y(k-1)) / Ts */
        if (Ts > 0.0 && pid->state.sample_count > 0) {
            d_term_raw = Kd * (pid->state.prev_measurement - measurement) / Ts;
        } else {
            d_term_raw = 0.0;
        }
    } else {
        /* Derivative on error: Kd * (e(k) - e(k-1)) / Ts */
        if (Ts > 0.0 && pid->state.sample_count > 0) {
            d_term_raw = Kd * (error - pid->state.prev_error) / Ts;
        } else {
            d_term_raw = 0.0;
        }
    }

    /* Apply low-pass filter to derivative */
    double alpha = pid->alpha;
    double d_term;
    if (alpha > 0.0 && alpha < 1.0) {
        d_term = alpha * pid->state.filtered_derivative + (1.0 - alpha) * d_term_raw;
        pid->state.filtered_derivative = d_term;
    } else {
        d_term = d_term_raw;
        pid->state.filtered_derivative = d_term_raw;
    }

    /* ---- 5. Feedforward ---- */
    double ff_term = pid->params.Kff * setpoint;

    /* ---- 6. Compute unsaturated output ---- */
    double u_unsat = p_term + i_term + d_term + ff_term;

    /* ---- 7. Output saturation ---- */
    double u_sat = clamp(u_unsat, pid->out_min, pid->out_max);

    /* ---- 8. Back-calculation anti-windup ----
     * Adjust integral so that integrated output matches saturated output.
     * I_new = I_old + Kaw * Ts * (u_sat - u_unsat)
     * where Kaw = 1/Tt (Tt = tracking time constant)
     */
    if (pid->aw_method == PID_AW_BACK_CALC || pid->aw_method == PID_AW_COMBINED) {
        integral += pid->aw_gain * Ts * (u_sat - u_unsat);
    }

    /* ---- 9. Bumpless tracking ----
     * When tracking is enabled, force integral so that output = tracking_input.
     * I = tracking_input - P - D - FF  (solved from u = P + I + D + FF)
     */
    if (pid->mode == PID_MODE_TRACKING || pid->tracking_enabled) {
        double track_target = pid->state.tracking_input;
        /* Solve: track_target = p_term + integral + d_term + ff_term */
        integral = track_target - p_term - d_term - ff_term;
        u_sat = clamp(track_target, pid->out_min, pid->out_max);
    }

    /* ---- 10. Manual mode ---- */
    if (pid->mode == PID_MODE_MANUAL) {
        u_sat = clamp(pid->state.tracking_input, pid->out_min, pid->out_max);
        /* Preload integrator for bumpless transfer back to auto */
        integral = u_sat - p_term - d_term - ff_term;
    }

    /* ---- 11. Final integral clamping ---- */
    integral = clamp(integral, pid->int_min, pid->int_max);

    /* ---- 12. Update state ---- */
    pid->state.integral = integral;
    pid->state.prev_error = error;
    pid->state.prev_measurement = measurement;
    pid->state.prev_output = u_sat;
    pid->state.sample_count++;
    /* Store the actual P/I/D contributions for query API */
    /* (We'll compute them on the fly in the query functions) */

    return u_sat;
}

/*===========================================================================
 * L2 -- Runtime: Mode Switching and Reset
 *===========================================================================*/

void pid_reset(PIDController *pid) {
    if (!pid) return;
    pid->state.integral = 0.0;
    pid->state.prev_error = 0.0;
    pid->state.prev_measurement = 0.0;
    pid->state.prev_output = 0.0;
    pid->state.tracking_input = 0.0;
    pid->state.filtered_derivative = 0.0;
    pid->state.sample_count = 0;
}

void pid_set_manual(PIDController *pid, double manual_out) {
    if (!pid) return;
    pid->mode = PID_MODE_MANUAL;
    pid->state.tracking_input = manual_out;
    /* The integrator will be preloaded on the next update call */
}

void pid_set_auto(PIDController *pid) {
    if (!pid) return;
    /* When switching from MANUAL to AUTO, the integrator was already
     * preloaded during the last MANUAL update call, ensuring bumpless transfer. */
    pid->mode = PID_MODE_AUTO;
}

void pid_set_tracking(PIDController *pid, double tracking_value) {
    if (!pid) return;
    pid->state.tracking_input = tracking_value;
    pid->tracking_enabled = true;
}

/*===========================================================================
 * L2 -- Query API
 *===========================================================================*/

double pid_get_p_term(const PIDController *pid) {
    if (!pid) return 0.0;
    double setpoint = pid->state.prev_measurement + pid->state.prev_error;
    return pid->params.Kp * (pid->params.b * setpoint - pid->state.prev_measurement);
}

double pid_get_i_term(const PIDController *pid) {
    if (!pid) return 0.0;
    return pid->state.integral;
}

double pid_get_d_term(const PIDController *pid) {
    if (!pid) return 0.0;
    return pid->state.filtered_derivative;
}

double pid_get_integral(const PIDController *pid) {
    if (!pid) return 0.0;
    return pid->state.integral;
}

/*===========================================================================
 * L2 -- Parameter Conversion
 *===========================================================================*/

void pid_convert_params(PIDParams *dst, const PIDParams *src,
                        PIDForm src_form, PIDForm dst_form) {
    if (!dst || !src) return;
    *dst = *src;

    if (src_form == dst_form) return;

    if (src_form == PID_FORM_PARALLEL && dst_form == PID_FORM_STANDARD) {
        dst->Ti = safe_div(src->Kp, src->Ki);
        dst->Td = safe_div(src->Kd, src->Kp);
        dst->form = PID_FORM_STANDARD;
    } else if (src_form == PID_FORM_STANDARD && dst_form == PID_FORM_PARALLEL) {
        dst->Ki = safe_div(src->Kp, src->Ti);
        dst->Kd = src->Kp * src->Td;
        dst->form = PID_FORM_PARALLEL;
    } else if (src_form == PID_FORM_SERIES && dst_form == PID_FORM_PARALLEL) {
        /* Series -> Parallel: Kp' = Kp*(1 + Td/Ti), Ki' = Kp/Ti, Kd' = Kp*Td
         * Valid only when Ti >> Td (interaction negligible) */
        double ratio = safe_div(src->Td, src->Ti);
        dst->Kp = src->Kp * (1.0 + ratio);
        dst->Ki = safe_div(src->Kp, src->Ti);
        dst->Kd = src->Kp * src->Td;
        dst->form = PID_FORM_PARALLEL;
    } else if (src_form == PID_FORM_PARALLEL && dst_form == PID_FORM_SERIES) {
        /* Parallel -> Series: Kp' = Kp/2*(1+sqrt(1-4*Ki*Kd/Kp^2)),
         * Ti' = ..., Td' = ... (requires Kp^2 >= 4*Ki*Kd) */
        double disc = src->Kp * src->Kp - 4.0 * src->Ki * src->Kd;
        if (disc >= 0.0) {
            dst->Kp = 0.5 * (src->Kp + sqrt(disc));
            dst->Ti = safe_div(dst->Kp, src->Ki);
            dst->Td = safe_div(src->Kd, dst->Kp);
        } else {
            /* Complex roots: use approximation */
            dst->Kp = src->Kp;
            dst->Ti = safe_div(src->Kp, src->Ki);
            dst->Td = safe_div(src->Kd, src->Kp);
        }
        dst->form = PID_FORM_SERIES;
    }
}

int pid_transfer_function_str(const PIDController *pid, char *buf, size_t size) {
    if (!pid || !buf || size == 0) return -1;
    double Kp = pid->params.Kp;
    double Ti = pid->params.Ti;
    double Td = pid->params.Td;
    int n;

    if (pid->params.form == PID_FORM_PARALLEL) {
        /* G(s) = Kp + Ki/s + Kd*s */
        if (pid->params.Ki > 0.0 && pid->params.Kd > 0.0) {
            n = snprintf(buf, size, "G(s) = %.4g + %.4g/s + %.4g*s",
                         pid->params.Kp, pid->params.Ki, pid->params.Kd);
        } else if (pid->params.Ki > 0.0) {
            n = snprintf(buf, size, "G(s) = %.4g + %.4g/s (PI)",
                         pid->params.Kp, pid->params.Ki);
        } else if (pid->params.Kd > 0.0) {
            n = snprintf(buf, size, "G(s) = %.4g + %.4g*s (PD)",
                         pid->params.Kp, pid->params.Kd);
        } else {
            n = snprintf(buf, size, "G(s) = %.4g (P-only)", pid->params.Kp);
        }
    } else {
        /* Standard form: G(s) = Kp*(1 + 1/(Ti s) + Td s) */
        n = snprintf(buf, size,
                     "G(s) = %.4g * (1 + 1/(%.4g s) + %.4g s)",
                     Kp, Ti, Td);
    }
    if (n < 0 || (size_t)n >= size) return -1;
    return n;
}

/*===========================================================================
 * L3 -- Transfer Function Extraction
 *===========================================================================*/

void pid_get_transfer_function(const PIDController *pid,
                               PIDTransferFunction *tf, PIDForm form) {
    if (!pid || !tf) return;
    memset(tf, 0, sizeof(*tf));

    double Kp = pid->params.Kp;
    double Ki = pid->params.Ki;
    double Kd = pid->params.Kd;
    double Ti = pid->params.Ti;
    double Td = pid->params.Td;

    if (form == PID_FORM_PARALLEL) {
        /* G_c(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
         * num: [Ki, Kp, Kd]  (a0 + a1*s + a2*s^2)
         * den: [0, 1]         (s)
         */
        tf->n0 = Ki;
        tf->n1 = Kp;
        tf->n2 = Kd;
        tf->d0 = 0.0;
        tf->d1 = 1.0;
        tf->d2 = 0.0;
    } else {
        /* Standard: G_c(s) = Kp*(1 + 1/(Ti*s) + Td*s)
         * = Kp * (Ti*Td*s^2 + Ti*s + 1) / (Ti*s)
         * num: Kp * [1, Ti, Ti*Td]
         * den: [0, Ti]
         */
        tf->n0 = Kp;
        tf->n1 = Kp * Ti;
        tf->n2 = Kp * Ti * Td;
        tf->d0 = 0.0;
        tf->d1 = Ti;
        tf->d2 = 0.0;
    }
}

void pid_frequency_response(const PIDTransferFunction *tf, double omega,
                            double *mag, double *phase) {
    /* Evaluate numerator and denominator at s = j*omega
     * num(jw) = (n0 - n2*w^2) + j*(n1*w)
     * den(jw) = (d0 - d2*w^2) + j*(d1*w)
     *
     * |G(jw)| = |num| / |den|
     * phase(G) = atan2(im(num), re(num)) - atan2(im(den), re(den))
     */
    if (!tf) {
        if (mag) *mag = 0.0;
        if (phase) *phase = 0.0;
        return;
    }

    double w = omega;
    double w2 = w * w;

    double num_re = tf->n0 - tf->n2 * w2;
    double num_im = tf->n1 * w;
    double den_re = tf->d0 - tf->d2 * w2;
    double den_im = tf->d1 * w;

    if (mag) {
        double num_mag2 = num_re * num_re + num_im * num_im;
        double den_mag2 = den_re * den_re + den_im * den_im;
        if (den_mag2 < 1e-60) {
            *mag = 1e300; /* near-pole, very large */
        } else {
            *mag = sqrt(num_mag2 / den_mag2);
        }
    }

    if (phase) {
        double num_phase = atan2(num_im, num_re);
        double den_phase = atan2(den_im, den_re);
        *phase = num_phase - den_phase;
    }
}
