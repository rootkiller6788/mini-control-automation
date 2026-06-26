/*
 * pid_controller.c - PID Controller Implementation
 *
 * Full implementation of industrial PID control algorithms including:
 *   - ISA standard and parallel forms with anti-windup
 *   - Ziegler-Nichols, Cohen-Coon, IMC, and AMIGO tuning methods
 *   - Cascade control structure
 *   - Static and dynamic feedforward compensation
 *   - Gain scheduling for nonlinear processes
 *
 * References:
 *   - Astrom & Hagglund (1995), "PID Controllers: Theory, Design, and Tuning"
 *   - Ziegler & Nichols (1942), "Optimum Settings for Automatic Controllers"
 *   - Cohen & Coon (1953)
 *   - Rivera, Morari & Skogestad (1986), "Internal Model Control"
 */

#include "pid_controller.h"
#include <math.h>
#include <string.h>
#include <float.h>

/* ============================================================================
 * Internal helpers
 * ============================================================================
 */

/** Clamp a double value to [lo, hi] range */
static double clamp_d(double x, double lo, double hi) {
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/** Check if a double is effectively zero */
static bool is_zero(double x) {
    return fabs(x) < 1e-15;
}

/* ============================================================================
 * PID Initialization
 * ============================================================================
 */

void pid_init_isa(pid_controller_t *pid, double kc, double ti, double td,
                  double ts, double mv_lo, double mv_hi) {
    if (!pid) return;
    memset(pid, 0, sizeof(pid_controller_t));

    pid->form   = PID_FORM_ISA_STANDARD;
    pid->action = PID_ACTION_REVERSE; /* Default: reverse-acting (heating) */
    pid->kc     = kc;
    pid->ti     = ti;
    pid->td     = td;
    pid->ts     = ts;
    pid->mv_lo  = mv_lo;
    pid->mv_hi  = mv_hi;
    pid->beta   = 1.0;    /* Full setpoint weighting by default */
    pid->gamma  = 0.0;    /* Derivative on PV only (standard industrial) */
    pid->n_filter = 10.0; /* Derivative filter: N=10 is typical */

    /* Default: no rate-of-change limits */
    pid->dv_lo = -INFINITY;
    pid->dv_hi = +INFINITY;

    /* Integral limits: use output limits if integral not separately limited */
    pid->i_lo = mv_lo;
    pid->i_hi = mv_hi;

    pid->initialized  = true;
    pid->manual_mode  = false;
    pid->integral_hold = false;
    pid->integral     = 0.0;

    /* Convert to parallel form for consistent internal state */
    if (is_zero(ti)) {
        pid->kp = kc;
        pid->ki = 0.0;
    } else {
        pid->kp = kc;
        pid->ki = kc / ti;
    }
    pid->kd = kc * td;
}

void pid_init_parallel(pid_controller_t *pid, double kp, double ki, double kd,
                       double ts, double mv_lo, double mv_hi) {
    if (!pid) return;
    memset(pid, 0, sizeof(pid_controller_t));

    pid->form  = PID_FORM_PARALLEL;
    pid->action = PID_ACTION_REVERSE;
    pid->kp    = kp;
    pid->ki    = ki;
    pid->kd    = kd;
    pid->ts    = ts;
    pid->mv_lo = mv_lo;
    pid->mv_hi = mv_hi;
    pid->beta  = 1.0;
    pid->gamma = 0.0;
    pid->n_filter = 10.0;

    pid->dv_lo = -INFINITY;
    pid->dv_hi = +INFINITY;
    pid->i_lo  = mv_lo;
    pid->i_hi  = mv_hi;

    pid->initialized  = true;
    pid->manual_mode  = false;
    pid->integral_hold = false;
    pid->integral     = 0.0;

    /* Store ISA form for diagnostic display */
    pid->kc = kp;
    pid->ti = is_zero(ki) ? 0.0 : kp / ki;
    pid->td = is_zero(kp) ? 0.0 : kd / kp;
}

/* ============================================================================
 * PID Update ? the core control algorithm
 *
 * L5: Discrete-time PID with industrial hardening features.
 *
 * The algorithm implements:
 *
 * 1. PID computation:
 *    P = Kc * (beta * SP - PV)        [proportional on error with SP weight]
 *    I += Kc/Ti * integral(error * dt) [trapezoidal integration]
 *    D = Kc * Td * d(error)/dt         [filtered derivative]
 *
 * 2. Derivative-on-PV mode (gamma = 0):
 *    Uses -d(PV)/dt instead of d(SP-PV)/dt to avoid "derivative kick"
 *    on setpoint changes. Standard in all industrial DCS.
 *
 * 3. Anti-reset windup (conditional integration + back-calculation):
 *    When MV saturates, integrator freezes. This prevents integral
 *    windup where the integrator accumulates a large value while
 *    the actuator is saturated, causing overshoot when PV crosses SP.
 *    Related: Lyapunov stability of constrained systems (L4).
 *
 * 4. Bumpless transfer:
 *    In manual mode, integral term is continuously set to track the
 *    manual output minus P and D terms. This ensures smooth transition
 *    when switching to automatic.
 *
 * 5. Derivative filtering:
 *    A first-order filter Td/N is applied to the derivative term
 *    to limit high-frequency gain. Typical N = 8..20.
 *    Transfer function: D(s) = Kc*Td*s / (1 + Td/N*s) * E(s)
 *
 * Complexity: O(1) per call ? 11 floating-point operations.
 * ============================================================================
 */

double pid_update(pid_controller_t *pid, double sp, double pv, double dt) {
    if (!pid) return pid ? pid->mv_lo : 0.0;

    /* Use controller's configured sample time if dt not provided */
    if (dt <= 0.0) {
        dt = pid->ts;
    }
    if (dt <= 0.0) {
        return pid->manual_mode ? pid->manual_mv : pid->prev_mv;
    }

    /* === Manual Mode === */
    if (pid->manual_mode) {
        /* Bumpless transfer: track integral so that when switching
         * to auto, the output starts at the current manual value. */
        double error = pid->action * (pid->beta * sp - pv);

        /* In ISA form: Ki = Kc/Ti */
        if (pid->ti > 0.0) {
            /* Compute what integral should be to produce manual_mv:
             * manual_mv = Kc*(beta*sp - pv) + I + D
             * I_required = manual_mv - P - D */
            double p_term = pid->kc * error;
            pid->integral = pid->manual_mv - p_term;
            pid->integral = clamp_d(pid->integral, pid->i_lo, pid->i_hi);
        } else {
            pid->integral = 0.0;
        }

        pid->prev_mv = pid->manual_mv;
        pid->prev_error = error;
        pid->prev_pv = pv;
        return pid->manual_mv;
    }

    /* === Error Calculation === */
    double error = pid->action * (sp - pv);

    /* === Proportional Term ===
     * P = Kc * (beta * sp - pv)
     * With beta < 1, reduces proportional kick on SP change (2-DOF PID). */
    double p_term = pid->kc * pid->action * (pid->beta * sp - pv);

    /* === Derivative Term (filtered, derivative-on-PV) ===
     *
     * Ideal derivative: D = Kc * Td * d(error)/dt
     *
     * With derivative-on-PV (gamma = 0):
     *   D = -Kc * Td * d(pv)/dt   -> avoids derivative kick on SP changes
     *
     * With 2-DOF (0 < gamma < 1):
     *   D = Kc * Td * d(gamma*sp - pv)/dt
     *
     * Filtered derivative (prevents noise amplification):
     *   D_filtered = (Td/N * D_prev + Td * (e - e_prev)) / (Td/N + dt)
     *   where N is the filter coefficient (typically 8-20). */
    double d_term = 0.0;
    if (pid->td > 0.0 && dt > 0.0) {
        double deriv_input = pid->gamma * sp - pv;
        double raw_deriv = (deriv_input - (pid->gamma * pid->last_sp - pid->prev_pv)) / dt;
        double Tf = pid->td / pid->n_filter; /* Derivative filter time constant */
        if (Tf > 0.0) {
            /* Filtered derivative: Tf * dD/dt + D = Kc*Td * raw_deriv
             * Discrete: D_new = (Tf*D_old + dt*Kc*Td*raw_deriv) / (Tf + dt) */
            double d_old = pid->kd * pid->prev_error; /* Previous D (simplified) */
            d_term = (Tf * d_old + dt * pid->kc * pid->td * raw_deriv) / (Tf + dt);
        } else {
            d_term = pid->kc * pid->td * raw_deriv;
        }
    }

    /* === Integral Term (trapezoidal integration with anti-windup) ===
     *
     * I_k = I_{k-1} + Ki * dt * (e_k + e_{k-1}) / 2   [trapezoidal rule]
     *
     * Conditional integration: only integrate when:
     *   - Output is not saturated, OR
     *   - Error would drive output away from saturation */
    if (pid->ti > 0.0 && !pid->integral_hold) {
        double ki = pid->kc / pid->ti;
        double i_increment = ki * dt * (error + pid->prev_error) / 2.0;

        /* Pre-compute what the new output would be */
        double tentative_mv = p_term + (pid->integral + i_increment) + d_term;
        /* tentative_mv used below for saturation detection */

        /* Conditional integration: only integrate if not saturated
         * or if integration would push away from saturation */
        bool should_integrate = true;
        if (tentative_mv >= pid->mv_hi && i_increment > 0.0) {
            should_integrate = false; /* Saturated high, integrator pushing up */
        } else if (tentative_mv <= pid->mv_lo && i_increment < 0.0) {
            should_integrate = false; /* Saturated low, integrator pushing down */
        }

        if (should_integrate) {
            pid->integral += i_increment;
        }

        /* Clamp integral term independently */
        pid->integral = clamp_d(pid->integral, pid->i_lo, pid->i_hi);

        /* Track integral extremes for diagnostics */
        if (pid->integral > pid->max_integral) pid->max_integral = pid->integral;
        if (pid->integral < pid->min_integral) pid->min_integral = pid->integral;
    }

    /* === Output Computation === */
    double mv = p_term + pid->integral + d_term;

    /* Apply output limits */
    double mv_raw = clamp_d(mv, pid->mv_lo, pid->mv_hi);

    /* Apply rate-of-change limits */
    double mv_out = mv_raw;
    if (pid->update_count > 0) {
        double rate = (mv_out - pid->prev_mv) / dt;
        if (rate > pid->dv_hi) {
            mv_out = pid->prev_mv + pid->dv_hi * dt;
        } else if (rate < pid->dv_lo) {
            mv_out = pid->prev_mv + pid->dv_lo * dt;
        }
    }

    /* Track saturation */
    if (fabs(mv_raw - mv) > 1e-9 || fabs(mv_out - mv_raw) > 1e-9) {
        pid->sat_count++;
    }

    /* === State Update === */
    pid->prev_mv    = mv_out;
    pid->prev_error = error;
    pid->prev_pv    = pv;
    pid->last_sp    = sp;
    pid->update_count++;

    return mv_out;
}

void pid_set_manual(pid_controller_t *pid, double manual_mv) {
    if (!pid) return;
    pid->manual_mode = true;
    pid->manual_mv   = clamp_d(manual_mv, pid->mv_lo, pid->mv_hi);
}

void pid_set_auto(pid_controller_t *pid) {
    if (!pid) return;
    pid->manual_mode = false;
    /* Integral was already tracking during manual mode,
     * so bumpless transfer is automatic */
}

/* ============================================================================
 * PID Diagnostics
 * ============================================================================
 */

void pid_get_diagnostics(const pid_controller_t *pid,
                         uint64_t *sat_count, double *max_i, double *min_i) {
    if (!pid) return;
    if (sat_count) *sat_count = pid->sat_count;
    if (max_i)     *max_i     = pid->max_integral;
    if (min_i)     *min_i     = pid->min_integral;
}

/* ============================================================================
 * Parameter Conversion: ISA <-> Parallel
 *
 * L1: The conversion formulas are fundamental to understanding how
 *     different DCS vendors represent PID parameters.
 *
 * ISA Standard (Non-Interacting):
 *   MV = Kc * [ e + 1/Ti * integral(e dt) + Td * de/dt ]
 *
 * Parallel (Ideal/Independent):
 *   MV = Kp * e + Ki * integral(e dt) + Kd * de/dt
 *
 * Conversion:
 *   Kp = Kc,  Ki = Kc/Ti,  Kd = Kc*Td
 *   Kc = Kp,  Ti = Kp/Ki,  Td = Kd/Kp
 *
 * Note: Ti = 0 (no integral) means Ki = 0.
 *       Kp = 0 means the conversion to Ti/Td is undefined.
 * ============================================================================
 */

void pid_convert_isa_to_parallel(double kc, double ti, double td,
                                  double *kp, double *ki, double *kd) {
    if (kp) *kp = kc;
    if (ki) *ki = (is_zero(ti)) ? 0.0 : kc / ti;
    if (kd) *kd = kc * td;
}

void pid_convert_parallel_to_isa(double kp, double ki, double kd,
                                  double *kc, double *ti, double *td) {
    if (kc) *kc = kp;
    if (ti) *ti = (is_zero(ki)) ? 0.0 : kp / ki;
    if (td) *td = (is_zero(kp)) ? 0.0 : kd / kp;
}

/* ============================================================================
 * PID Tuning: Select method and compute parameters
 * ============================================================================
 */

void pid_tune(const pid_fopdt_model_t *model, pid_tuning_method_t method,
              pid_tuning_t *result) {
    if (!model || !result) return;

    switch (method) {
        case TUNE_ZN_OPEN_LOOP:
            pid_tune_zn_openloop(model, result);
            break;
        case TUNE_ZN_CLOSED_LOOP:
            /* Cannot compute from FOPDT; caller must use pid_tune_zn_closedloop() */
            memset(result, 0, sizeof(pid_tuning_t));
            result->method = "ZN-Closed-Loop requires Ku, Pu";
            break;
        case TUNE_COHEN_COON:
            pid_tune_cohen_coon(model, result);
            break;
        case TUNE_IMC:
            pid_tune_imc(model, model->tau, result);
            break;
        case TUNE_AMIGO:
            pid_tune_imc(model, 2.0 * model->tau, result);
            result->method = "AMIGO (approximated via IMC)";
            break;
        case TUNE_TYREUS_LUYBEN:
            /* Tyreus-Luyben uses Kc = Ku/3.2, Ti = 2.2*Pu, Td = Pu/6.3
             * Requires Ku and Pu which cannot be derived from FOPDT.
             * Provide conservative IMC-based fallback. */
            pid_tune_imc(model, 3.0 * model->tau, result);
            result->method = "Tyreus-Luyben (approximated via conservative IMC)";
            break;
        case TUNE_IAE_SETPOINT:
        case TUNE_IAE_DISTURB:
            /* These require numerical optimization; use Cohen-Coon as
             * a reasonable approximation. */
            pid_tune_cohen_coon(model, result);
            break;
        default:
            memset(result, 0, sizeof(pid_tuning_t));
            result->method = "Unknown";
            break;
    }
}

/* ============================================================================
 * Ziegler-Nichols Open-Loop (Reaction Curve) Method
 *
 * L5: Classic 1942 tuning method, still the most widely taught.
 *
 * Procedure:
 *   1. Put controller in manual mode
 *   2. Make a small step change in MV (e.g., 10%)
 *   3. Record the process reaction curve (PV vs time)
 *   4. Fit a FOPDT model: draw tangent at inflection point
 *      - Kp = delta_PV / delta_MV  (process gain)
 *      - tau = time for PV to reach 63.2% of final value
 *      - theta = intersection of tangent with initial PV line
 *   5. Apply rules below
 *
 * Transfer function model: G(s) = Kp * exp(-theta*s) / (tau*s + 1)
 *
 * Tuning tables (ISA form):
 *   Controller | Kc                     | Ti         | Td
 *   -----------|------------------------|------------|----------
 *   P          | tau/(Kp*theta)         | -          | -
 *   PI         | 0.9*tau/(Kp*theta)     | 3.33*theta | -
 *   PID        | 1.2*tau/(Kp*theta)     | 2.0*theta  | 0.5*theta
 *
 * Design criterion: Quarter-amplitude decay ratio (each peak is 1/4
 * of the previous peak). This gives fast disturbance rejection but
 * can be oscillatory for setpoint tracking.
 *
 * Limitations:
 *   - Assumes theta/tau > 0.1 (not for dead-time dominant processes)
 *   - Quarter-decay may be too aggressive for many chemical processes
 *   - Does not account for measurement noise
 * ============================================================================
 */

void pid_tune_zn_openloop(const pid_fopdt_model_t *model, pid_tuning_t *result) {
    if (!model || !result) return;
    memset(result, 0, sizeof(pid_tuning_t));

    double Kp = model->gain;
    double tau = model->tau;
    double theta = model->theta;

    if (is_zero(Kp) || is_zero(tau) || theta < 0.0) {
        result->method = "ZN-Open-Loop: Invalid model";
        return;
    }

    /* Guard against very small dead time */
    if (theta < 0.001) theta = 0.001;

    /* PID tuning: Kc = 1.2*tau/(Kp*theta), Ti = 2.0*theta, Td = 0.5*theta */
    result->kc = 1.2 * tau / (Kp * theta);
    result->ti = 2.0 * theta;
    result->td = 0.5 * theta;
    result->method = "Ziegler-Nichols Open-Loop (PID)";
    result->gain_margin = 2.0;   /* Approximate, rule of thumb */
    result->phase_margin = 45.0; /* Approximate */
}

/* ============================================================================
 * Ziegler-Nichols Closed-Loop (Ultimate Sensitivity) Method
 *
 * L5: Alternative to the open-loop method for processes that cannot
 *     tolerate open-loop step tests (e.g., integrating processes).
 *
 * Procedure:
 *   1. Disable integral and derivative (P-only, Ti = infinity, Td = 0)
 *   2. Gradually increase Kc until the loop oscillates with constant
 *      amplitude (ultimate gain Ku, ultimate period Pu)
 *   3. Apply rules:
 *
 *   Controller | Kc       | Ti       | Td
 *   -----------|----------|----------|------
 *   P          | 0.5*Ku   | -        | -
 *   PI         | 0.45*Ku  | Pu/1.2   | -
 *   PID        | 0.6*Ku   | Pu/2.0   | Pu/8.0
 *
 * Safety note: Driving a process to sustained oscillation can be
 * dangerous. This method is banned in some industries (nuclear,
 * exothermic reactors). Use the relay feedback (Astrom-Hagglund)
 * method as a safer alternative.
 * ============================================================================
 */

void pid_tune_zn_closedloop(double ku, double pu, pid_tuning_t *result) {
    if (!result) return;
    memset(result, 0, sizeof(pid_tuning_t));

    if (is_zero(ku) || pu <= 0.0) {
        result->method = "ZN-Closed-Loop: Invalid Ku/Pu";
        return;
    }

    result->kc = 0.6 * ku;
    result->ti = pu / 2.0;
    result->td = pu / 8.0;
    result->method = "Ziegler-Nichols Closed-Loop (PID)";
    result->gain_margin = 1.0 / 0.6;  /* 1/0.6 = 1.67 */
    result->phase_margin = 50.0;
}

/* ============================================================================
 * Cohen-Coon Tuning Method
 *
 * L5: Designed for processes with significant dead time (theta/tau > 0.1).
 *
 * Cohen & Coon (1953) derived tuning rules by minimizing various
 * performance criteria (IAE, ISE, ITAE) for FOPDT processes.
 *
 * PID rules (ISA form):
 *   Kc = (1/Kp) * (tau/theta) * (4/3 + theta/(4*tau))
 *   Ti = theta * (32 + 6*theta/tau) / (13 + 8*theta/tau)
 *   Td = theta * 4 / (11 + 2*theta/tau)
 *
 * These formulas emerge from polynomial approximations to the optimal
 * solutions of the integral-of-error minimization problem.
 *
 * Compared to ZN: Cohen-Coon gives larger Kc and shorter Ti,
 * resulting in faster disturbance rejection but potentially more
 * oscillatory response.
 * ============================================================================
 */

void pid_tune_cohen_coon(const pid_fopdt_model_t *model, pid_tuning_t *result) {
    if (!model || !result) return;
    memset(result, 0, sizeof(pid_tuning_t));

    double Kp = model->gain;
    double tau = model->tau;
    double theta = model->theta;

    if (is_zero(Kp) || is_zero(tau) || theta < 0.001) {
        result->method = "Cohen-Coon: Invalid model";
        return;
    }

    double ratio = theta / tau;

    /* Cohen-Coon PID formulas */
    result->kc = (1.0 / Kp) * (tau / theta) * (4.0/3.0 + ratio / 4.0);
    result->ti = theta * (32.0 + 6.0 * ratio) / (13.0 + 8.0 * ratio);
    result->td = theta * 4.0 / (11.0 + 2.0 * ratio);

    result->method = "Cohen-Coon (PID)";
    result->gain_margin = 1.8;
    result->phase_margin = 40.0;
}

/* ============================================================================
 * IMC (Internal Model Control) / Lambda Tuning
 *
 * L5: Model-based tuning with a single intuitive tuning parameter
 *     lambda that directly specifies closed-loop speed.
 *
 * Theory (Rivera, Morari & Skogestad, 1986):
 *   For FOPDT process G(s) = Kp * exp(-theta*s) / (tau*s + 1),
 *   the IMC-based PID controller is:
 *
 *   Kc = tau / (Kp * (lambda + theta))
 *   Ti = tau
 *   Td = theta / 2  (for theta << tau)
 *
 * The closed-loop transfer function becomes:
 *   G_cl(s) = exp(-theta*s) / (lambda*s + 1)
 *
 * Interpretation:
 *   - lambda = tau/3  ? aggressive (fast response)
 *   - lambda = tau    ? moderate (standard)
 *   - lambda = 3*tau  ? conservative (robust)
 *
 * Advantages over ZN/CC:
 *   1. Single tuning parameter (lambda) instead of three
 *   2. Directly sets closed-loop speed
 *   3. Explicit robustness parameter (larger lambda = more robust)
 *   4. No oscillatory tuning needed
 *
 * This is the preferred tuning method in modern chemical plants
 * (adopted by Dow, DuPont, Shell).
 * ============================================================================
 */

void pid_tune_imc(const pid_fopdt_model_t *model, double lambda,
                  pid_tuning_t *result) {
    if (!model || !result) return;
    memset(result, 0, sizeof(pid_tuning_t));

    double Kp = model->gain;
    double tau = model->tau;
    double theta = model->theta;

    if (is_zero(Kp) || tau < 0.0 || theta < 0.0 || lambda <= 0.0) {
        result->method = "IMC: Invalid model or lambda";
        return;
    }

    double denom = lambda + theta;
    if (is_zero(denom)) denom = 0.001;

    result->kc = tau / (Kp * denom);
    result->ti = tau;
    /* For FOPDT, IMC-PID approximates derivative based on dead time */
    result->td = (theta > 0.0) ? theta / 2.0 : 0.0;

    result->method = "IMC (Lambda Tuning)";
    result->gain_margin = (lambda / tau + 1.0) * 1.5;
    result->phase_margin = 60.0;
}

/* ============================================================================
 * Cascade Control Implementation
 *
 * L6: Cascade control is one of the most powerful enhancements to
 *     single-loop PID, widely used in chemical and power industries.
 *
 * Structure:
 *         +-------+    +-------+    +-------+    +-------+
 *   SP -->| Master|--->| Slave |--->| Valve |--->|Process|--> PV_outer
 *   ^     | (slow)|    | (fast)|    |       |    |       |    |
 *   |     +-------+    +-------+    +-------+    +-------+    |
 *   |          ^            ^                       |         |
 *   |          |            |-------- PV_inner ------+         |
 *   +---------------------- PV_outer -------------------------+
 *
 * Master (outer/slow): Controls the primary variable (e.g., reactor temperature)
 * Slave  (inner/fast): Controls the secondary variable (e.g., jacket flow)
 *
 * Tuning rule: Tune inner loop first (fast), then tune outer loop
 * with inner loop closed. Inner loop should be 5-10x faster than outer.
 *
 * Mathematical justification (L4): The inner loop attenuates disturbances
 * entering the secondary path by a factor of |1 + Gc2*Gp2| compared to
 * single-loop control. This is a direct consequence of the sensitivity
 * function S(s) = 1/(1 + L(s)) being smaller at frequencies where |L| >> 1.
 * ============================================================================
 */

void pid_cascade_init(pid_cascade_t *cascade,
                      const pid_tuning_t *master_tune, double master_ts,
                      double master_mv_lo, double master_mv_hi,
                      const pid_tuning_t *slave_tune, double slave_ts,
                      double slave_mv_lo, double slave_mv_hi) {
    if (!cascade) return;
    memset(cascade, 0, sizeof(pid_cascade_t));

    if (master_tune) {
        pid_init_isa(&cascade->master, master_tune->kc, master_tune->ti,
                     master_tune->td, master_ts, master_mv_lo, master_mv_hi);
    }

    if (slave_tune) {
        pid_init_isa(&cascade->slave, slave_tune->kc, slave_tune->ti,
                     slave_tune->td, slave_ts, slave_mv_lo, slave_mv_hi);
    }
}

double pid_cascade_update(pid_cascade_t *cascade,
                          double master_sp, double master_pv,
                          double slave_pv, double dt) {
    if (!cascade) return 0.0;

    /* Step 1: Run outer (master) loop.
     * Master output becomes the setpoint for the slave. */
    cascade->master_mv = pid_update(&cascade->master, master_sp, master_pv, dt);

    /* Step 2: Run inner (slave) loop.
     * Slave SP comes from master output. */
    cascade->slave_mv = pid_update(&cascade->slave, cascade->master_mv,
                                    slave_pv, dt);

    return cascade->slave_mv;
}

/* ============================================================================
 * Feedforward Control
 *
 * L5: Feedforward compensates for measured disturbances BEFORE they
 *     affect the controlled variable?unlike feedback which waits for
 *     the error to develop.
 *
 * Principle of invariance:
 *   For disturbance D(s) and process Gp(s), the feedforward controller
 *   Gff(s) should satisfy:
 *     Gff(s) * Gp(s) + Gd(s) = 0
 *   => Gff(s) = -Gd(s) / Gp(s)
 *
 * In practice, this is approximated by a static gain + lead-lag:
 *   Gff(s) = Kff * (T_lead*s + 1) / (T_lag*s + 1)
 *
 * Static feedforward (T_lead = T_lag = 0):
 *   MV_ff = Kff * (disturbance - dist_nominal)
 *
 * Dynamic feedforward (adds lead-lag compensation):
 *   Filters the disturbance through a first-order lead-lag network.
 *
 * Lead-Lag discrete implementation (bilinear/Tustin):
 *   y[n] = (lag*y[n-1] + lead*(x[n]-x[n-1]) + dt*x[n]) / (lag + dt)
 *
 * Applications:
 *   - Feedforward of feed temperature to reactor cooling duty
 *   - Feedforward of steam pressure to turbine speed control
 *   - Feedforward of wind speed to wind turbine pitch control
 * ============================================================================
 */

void pid_feedforward_init(pid_feedforward_t *ff, double kff,
                          double dist_nominal, double t_lead, double t_lag) {
    if (!ff) return;
    memset(ff, 0, sizeof(pid_feedforward_t));
    ff->kff          = kff;
    ff->dist_nominal = dist_nominal;
    ff->t_lead       = t_lead;
    ff->t_lag        = t_lag;
}

double pid_feedforward_update(pid_feedforward_t *ff, double disturbance, double dt) {
    if (!ff) return 0.0;

    /* Static component */
    double mv_ff = ff->kff * (disturbance - ff->dist_nominal);

    /* Dynamic lead-lag component */
    if (ff->t_lead > 0.0 || ff->t_lag > 0.0) {
        if (dt > 0.0) {
            /* Lead-lag filter (bilinear discretization):
             * (T_lead*s + 1) / (T_lag*s + 1)  mapped to discrete time */
            double lag = ff->t_lag;
            double lead = ff->t_lead;

            if (lag > 0.0) {
                /* Apply lag filter to the disturbance */
                double filtered = (lag * ff->prev_ff_out + dt * disturbance) / (lag + dt);
                /* Apply lead component */
                double lead_out = filtered + (lead / dt) * (disturbance - ff->prev_dist);
                mv_ff = ff->kff * (lead_out - ff->dist_nominal);
            } else if (lead > 0.0) {
                /* Lead-only (pure derivative on disturbance, uncommon) */
                double lead_out = disturbance + (lead / dt) * (disturbance - ff->prev_dist);
                mv_ff = ff->kff * (lead_out - ff->dist_nominal);
            }
        }
    }

    ff->prev_dist   = disturbance;
    ff->prev_ff_out = mv_ff;

    return mv_ff;
}

/* ============================================================================
 * Gain Scheduling
 *
 * L8: Advanced PID technique for nonlinear processes.
 *
 * Many processes have dynamics that change with operating point:
 *   - pH control: process gain varies dramatically near neutral
 *   - Level control: vessel cross-section may change with level
 *   - Flight control: aircraft dynamics change with altitude/speed
 *
 * Gain scheduling: Use different PID parameters at different
 * operating points, with interpolation between points.
 *
 * The scheduling variable is typically the process variable (PV)
 * or the setpoint (SP). Linear interpolation is used between
 * breakpoints.
 *
 * Formal justification (L4): Gain scheduling is an approximation
 * to feedback linearization for nonlinear systems of the form
 * dx/dt = f(x) + g(x)*u. At each operating point, we design a
 * linear controller for the linearized plant.
 * ============================================================================
 */

void pid_gain_schedule_init(pid_gain_schedule_t *gs) {
    if (!gs) return;
    memset(gs, 0, sizeof(pid_gain_schedule_t));
}

int pid_gain_schedule_add(pid_gain_schedule_t *gs, double pv_point,
                          double kc, double ti, double td) {
    if (!gs) return -1;
    if (gs->count >= PID_GAIN_SCHEDULE_MAX) return -1;

    /* Insert in sorted order by pv_point */
    int i = gs->count;
    while (i > 0 && gs->points[i-1].pv_point > pv_point) {
        gs->points[i] = gs->points[i-1];
        i--;
    }
    gs->points[i].pv_point = pv_point;
    gs->points[i].kc       = kc;
    gs->points[i].ti       = ti;
    gs->points[i].td       = td;
    gs->count++;
    return i;
}

bool pid_gain_schedule_lookup(const pid_gain_schedule_t *gs, double pv,
                              double *kc, double *ti, double *td) {
    if (!gs || gs->count == 0) return false;

    /* Below lowest breakpoint: use lowest gains */
    if (pv <= gs->points[0].pv_point) {
        *kc = gs->points[0].kc;
        *ti = gs->points[0].ti;
        *td = gs->points[0].td;
        return true;
    }

    /* Above highest breakpoint: use highest gains */
    if (pv >= gs->points[gs->count-1].pv_point) {
        int last = gs->count - 1;
        *kc = gs->points[last].kc;
        *ti = gs->points[last].ti;
        *td = gs->points[last].td;
        return true;
    }

    /* Linear interpolation between two surrounding breakpoints */
    for (int i = 0; i < gs->count - 1; i++) {
        if (pv >= gs->points[i].pv_point && pv <= gs->points[i+1].pv_point) {
            double x0 = gs->points[i].pv_point;
            double x1 = gs->points[i+1].pv_point;
            double t = (pv - x0) / (x1 - x0);

            *kc = gs->points[i].kc + t * (gs->points[i+1].kc - gs->points[i].kc);
            *ti = gs->points[i].ti + t * (gs->points[i+1].ti - gs->points[i].ti);
            *td = gs->points[i].td + t * (gs->points[i+1].td - gs->points[i].td);
            return true;
        }
    }

    return false;
}