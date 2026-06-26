/*
 * pid_controller.h - PID Controller: algorithms, tuning, and advanced forms
 *
 * The PID (Proportional-Integral-Derivative) controller is the backbone
 * of industrial process control, used in >95% of all control loops.
 *
 * This header implements the ISA standard form, parallel form, and
 * advanced variants including anti-windup, bumpless transfer, gain
 * scheduling, and cascade structures.
 *
 * References:
 *   - Astrom & Hagglund, "PID Controllers: Theory, Design, and Tuning" (1995)
 *   - Ziegler & Nichols, "Optimum Settings for Automatic Controllers" (1942)
 *   - Cohen & Coon, "Theoretical Consideration of Retarded Control" (1953)
 *   - ISA-5.1 / ISA-75 series on control valve standards
 *
 * Course Alignment:
 *   MIT 6.302 - Feedback System Design (PID tuning, loop shaping)
 *   Stanford EE392 - Digital Control (discrete PID, anti-windup)
 *   Berkeley EE128 - Feedback Control (state-space PID, cascade)
 *   Tsinghua - Process Control (PID parameter tuning)
 *   ETH 227-0216 - Control Systems II (advanced PID structures)
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * L1: PID Forms - ISA Standard vs Parallel vs Ideal
 * ============================================================================
 */

/**
 * PID Controller Form enumeration.
 *
 * ISA Standard (Non-Interacting):
 *   MV(s) = Kc * [ 1 + 1/(Ti*s) + Td*s ] * E(s)
 *   Where Kc = proportional gain, Ti = integral time (seconds/repeat),
 *   Td = derivative time (seconds). Also known as the "standard form."
 *
 * Parallel (Ideal/Independent):
 *   MV(s) = [ Kp + Ki/s + Kd*s ] * E(s)
 *   Where Kp, Ki, Kd are independent gains.
 *
 * Conversion between forms:
 *   Kp = Kc,  Ki = Kc/Ti,  Kd = Kc*Td
 *   Kc = Kp,  Ti = Kp/Ki,  Td = Kd/Kp
 */
typedef enum {
    PID_FORM_ISA_STANDARD = 0,  /* Kc, Ti, Td */
    PID_FORM_PARALLEL     = 1,  /* Kp, Ki, Kd */
    PID_FORM_SERIES       = 2   /* Series/interacting form */
} pid_form_t;

/**
 * Controller action direction.
 * Reverse: MV increases as PV drops below SP (e.g., heating valve)
 *   MV(s) = Kc * (SP(s) - PV(s)) + ...
 * Direct: MV increases as PV rises above SP (e.g., cooling valve)
 *   MV(s) = Kc * (PV(s) - SP(s)) + ...
 */
typedef enum {
    PID_ACTION_REVERSE = 1,   /* Heating: MV = Kc*(SP-PV) + ... */
    PID_ACTION_DIRECT  = -1   /* Cooling: MV = Kc*(PV-SP) + ... */
} pid_action_t;

/* ============================================================================
 * L2: PID Controller Parameters and State
 * ============================================================================
 */

/** PID controller main structure.
 *  Contains both configuration (tuning) parameters and runtime state.
 *  All integral/derivative computations use trapezoidal approximation
 *  for discrete-time implementation.
 */
typedef struct {
    /* === Configuration === */
    pid_form_t   form;           /* ISA standard or parallel */
    pid_action_t action;         /* Direct or reverse acting */
    double       kc;             /* Controller gain (proportional) */
    double       ti;             /* Integral time [seconds/repeat] */
    double       td;             /* Derivative time [seconds] */
    double       kp;             /* Parallel form proportional gain */
    double       ki;             /* Parallel form integral gain [1/s] */
    double       kd;             /* Parallel form derivative gain [s] */
    double       ts;             /* Sample time [seconds] */
    double       beta;           /* Setpoint weighting (0..1), for 2-DOF */
    double       gamma;          /* Derivative setpoint weight (0..1), for 2-DOF */
    double       n_filter;       /* Derivative filter coefficient (N in Td/(1+Td/N*s)) */

    /* === Limits === */
    double       mv_lo;          /* Output low limit */
    double       mv_hi;          /* Output high limit */
    double       dv_lo;          /* Rate-of-change low limit */
    double       dv_hi;          /* Rate-of-change high limit */
    double       i_lo;           /* Integral term low limit */
    double       i_hi;           /* Integral term high limit */

    /* === Runtime State === */
    double       integral;       /* Accumulated integral term */
    double       prev_error;     /* Previous error for derivative */
    double       prev_pv;        /* Previous PV for derivative-on-PV mode */
    double       prev_mv;        /* Previous output */
    double       last_sp;        /* Last setpoint (for bumpless transfer) */
    uint64_t     last_update;    /* Timestamp of last update */
    bool         initialized;    /* Controller has been initialized */
    bool         manual_mode;    /* Manual mode flag */
    double       manual_mv;      /* Manual output setpoint */
    bool         integral_hold;  /* Freeze integrator (e.g., during saturation) */

    /* === Diagnostics === */
    uint64_t     update_count;   /* Number of PID updates */
    double       max_integral;   /* Peak integral term observed */
    double       min_integral;   /* Minimum integral term observed */
    uint64_t     sat_count;      /* Number of saturation events */
} pid_controller_t;

/* ============================================================================
 * L4: Tuning Rules - PID parameter calculation methods
 * ============================================================================
 */

/**
 * Process model for PID tuning.
 * Uses the First-Order Plus Dead Time (FOPDT) model:
 *   G(s) = Kp * exp(-theta*s) / (tau*s + 1)
 *
 * Where:
 *   Kp    = process gain (delta_PV / delta_MV)
 *   tau   = time constant (63.2% response time)
 *   theta = dead time / transport delay
 *
 * This is the most common model in process control; it captures
 * the dominant dynamics of >80% of industrial processes.
 */
typedef struct {
    double gain;     /* Kp: Process gain (dimensionless or PV/MV units) */
    double tau;      /* tau: Time constant (seconds) */
    double theta;    /* theta: Dead time (seconds) */
} pid_fopdt_model_t;

/**
 * Tuning rule enumeration.
 * Each method is optimal for a different performance criterion.
 */
typedef enum {
    TUNE_ZN_OPEN_LOOP   = 0,  /* Ziegler-Nichols open-loop (reaction curve) */
    TUNE_ZN_CLOSED_LOOP = 1,  /* Ziegler-Nichols closed-loop (ultimate gain) */
    TUNE_COHEN_COON     = 2,  /* Cohen-Coon for FOPDT with large dead time */
    TUNE_IMC            = 3,  /* Internal Model Control (Lambda tuning) */
    TUNE_IAE_SETPOINT   = 4,  /* Minimum IAE for setpoint changes */
    TUNE_IAE_DISTURB    = 5,  /* Minimum IAE for disturbance rejection */
    TUNE_AMIGO          = 6,  /* Approximate M-constrained Integral Gain Optimization */
    TUNE_TYREUS_LUYBEN  = 7   /* Tyreus-Luyben (conservative, for cascade inner) */
} pid_tuning_method_t;

/**
 * PID tuning result.
 * Contains the computed PID parameters from any tuning method.
 */
typedef struct {
    double kc;          /* Controller gain */
    double ti;          /* Integral time [s] (INFINITY if no integral) */
    double td;          /* Derivative time [s] (0.0 if no derivative) */
    const char *method; /* Name of tuning method used */
    double gain_margin; /* Estimated gain margin */
    double phase_margin;/* Estimated phase margin [degrees] */
} pid_tuning_t;

/* ============================================================================
 * L5: PID Controller API
 * ============================================================================
 */

/**
 * Initialize a PID controller with ISA standard form parameters.
 *
 * @param pid       Pointer to controller structure
 * @param kc        Controller gain (dimensionless)
 * @param ti        Integral time [seconds/repeat], 0 to disable
 * @param td        Derivative time [seconds], 0 to disable
 * @param ts        Sample time [seconds]
 * @param mv_lo     Output low limit
 * @param mv_hi     Output high limit
 */
void pid_init_isa(pid_controller_t *pid, double kc, double ti, double td,
                  double ts, double mv_lo, double mv_hi);

/**
 * Initialize a PID controller with parallel form parameters.
 */
void pid_init_parallel(pid_controller_t *pid, double kp, double ki, double kd,
                       double ts, double mv_lo, double mv_hi);

/**
 * Compute one PID iteration.
 *
 * Discrete-time PID with:
 *   - Backward Euler integration (robust to noise)
 *   - Filtered derivative (avoids derivative kick)
 *   - Derivative-on-PV mode (standard industrial practice)
 *   - Anti-reset windup (clamping + back-calculation)
 *   - Bumpless transfer (tracks manual MV in manual mode)
 *
 * @param pid     Controller state
 * @param sp      Setpoint
 * @param pv      Process variable (measurement)
 * @param dt      Time step [seconds] (0 = use ts from init)
 * @return        Manipulated variable (controller output)
 *
 * Algorithm complexity: O(1) per call.
 *
 * Mathematical basis:
 *   P = Kc * (beta*sp - pv)
 *   I += Ki * dt * error  (clamped to [i_lo, i_hi])
 *   D = Kc * Td * (prev_pv - pv) / dt  (filtered, derivative on PV)
 *   MV = P + I + D  (clamped to [mv_lo, mv_hi])
 */
double pid_update(pid_controller_t *pid, double sp, double pv, double dt);

/**
 * Place controller in manual mode.
 * Output tracks manual_mv. Integral term resets for bumpless transfer.
 */
void pid_set_manual(pid_controller_t *pid, double manual_mv);

/**
 * Place controller in automatic mode.
 * Initializes integral term for bumpless transfer from manual.
 */
void pid_set_auto(pid_controller_t *pid);

/**
 * Compute PID tuning parameters using specified method.
 *
 * @param model   FOPDT process model parameters
 * @param method  Tuning method to apply
 * @param result  Output: computed PID parameters
 */
void pid_tune(const pid_fopdt_model_t *model, pid_tuning_method_t method,
              pid_tuning_t *result);

/**
 * Ziegler-Nichols Open-Loop Tuning (Reaction Curve Method).
 *
 * Process:
 *   1. Place controller in manual, step MV by delta_MV
 *   2. Record PV response (process reaction curve)
 *   3. Fit FOPDT model: Kp = delta_PV/delta_MV, tau, theta from curve
 *   4. Apply tuning rules based on Kp, tau, theta
 *
 * Rules (for ISA form):
 *   P-only:   Kc = tau/(Kp*theta)
 *   PI:       Kc = 0.9*tau/(Kp*theta), Ti = 3.33*theta
 *   PID:      Kc = 1.2*tau/(Kp*theta), Ti = 2.0*theta, Td = 0.5*theta
 */
void pid_tune_zn_openloop(const pid_fopdt_model_t *model, pid_tuning_t *result);

/**
 * Ziegler-Nichols Closed-Loop Tuning (Ultimate Gain Method).
 *
 * Process:
 *   1. Set Ti = infinity, Td = 0 (P-only control)
 *   2. Increase Kc until sustained oscillation (ultimate gain Ku, period Pu)
 *   3. Apply rules:
 *      P-only:   Kc = 0.5*Ku
 *      PI:       Kc = 0.45*Ku, Ti = Pu/1.2
 *      PID:      Kc = 0.6*Ku, Ti = Pu/2.0, Td = Pu/8.0
 *
 * @param ku   Ultimate gain
 * @param pu   Ultimate period [seconds]
 */
void pid_tune_zn_closedloop(double ku, double pu, pid_tuning_t *result);

/**
 * Cohen-Coon Tuning for FOPDT processes.
 * Designed for processes with significant dead time (theta/tau > 0.1).
 *
 * Rules:
 *   Kc = (1/Kp)*(tau/theta)*(4/3 + theta/(4*tau))
 *   Ti = theta*(32 + 6*theta/tau)/(13 + 8*theta/tau)
 *   Td = theta*4/(11 + 2*theta/tau)
 *
 * This method minimizes the 1/4 decay ratio criterion.
 */
void pid_tune_cohen_coon(const pid_fopdt_model_t *model, pid_tuning_t *result);

/**
 * IMC (Internal Model Control) / Lambda Tuning.
 *
 * Provides a single tuning parameter lambda that adjusts the closed-loop
 * speed of response. lambda = tau gives "normal" tuning; lambda = 3*tau
 * gives conservative tuning suitable for cascade inner loops.
 *
 *   Kc = tau / (Kp * (lambda + theta))
 *   Ti = tau
 *   Td = theta / 2
 *
 * @param model   FOPDT model
 * @param lambda  Desired closed-loop time constant [s]
 */
void pid_tune_imc(const pid_fopdt_model_t *model, double lambda,
                  pid_tuning_t *result);

/**
 * Convert PID parameters between ISA and Parallel forms.
 * From ISA: Kp = Kc, Ki = Kc/Ti, Kd = Kc*Td
 * From Parallel: Kc = Kp, Ti = Kp/Ki, Td = Kd/Kp
 */
void pid_convert_isa_to_parallel(double kc, double ti, double td,
                                  double *kp, double *ki, double *kd);
void pid_convert_parallel_to_isa(double kp, double ki, double kd,
                                  double *kc, double *ti, double *td);

/**
 * Get PID diagnostics: saturation count, integral extremes.
 */
void pid_get_diagnostics(const pid_controller_t *pid,
                         uint64_t *sat_count, double *max_i, double *min_i);

/* ============================================================================
 * L5: Cascade Control - Master/Slave PID structure
 * ============================================================================
 */

/**
 * Cascade control structure: outer (primary/master) loop feeds its
 * output as the setpoint to the inner (secondary/slave) loop.
 *
 * Benefit: Inner loop rejects disturbances before they affect the
 * outer loop. Typically 10x faster response.
 *
 * Example: Reactor temperature (outer) -> jacket flow (inner).
 */
typedef struct {
    pid_controller_t master;     /* Outer/primary controller */
    pid_controller_t slave;      /* Inner/secondary controller */
    double           master_mv;  /* Cached master output */
    double           slave_mv;   /* Cached slave output */
} pid_cascade_t;

void pid_cascade_init(pid_cascade_t *cascade,
                      const pid_tuning_t *master_tune, double master_ts,
                      double master_mv_lo, double master_mv_hi,
                      const pid_tuning_t *slave_tune, double slave_ts,
                      double slave_mv_lo, double slave_mv_hi);

double pid_cascade_update(pid_cascade_t *cascade,
                          double master_sp, double master_pv,
                          double slave_pv, double dt);

/* ============================================================================
 * L5: Feedforward Control
 * ============================================================================
 */

/**
 * Feedforward controller: compensates for measured disturbances before
 * they affect the process variable.
 *
 * Static feedforward: MV_ff = Kff * (disturbance - dist_nominal)
 * Dynamic feedforward: adds lead-lag compensation Gff(s) = Kff*(Tld*s+1)/(Tlg*s+1)
 *
 * Used with PID feedback for combined feedforward-feedback control.
 */
typedef struct {
    double kff;         /* Feedforward gain */
    double dist_nominal;/* Nominal disturbance value */
    double t_lead;      /* Lead time constant [s] (0 = static only) */
    double t_lag;       /* Lag time constant [s] (0 = static only) */
    double prev_dist;   /* Previous disturbance value */
    double prev_ff_out; /* Previous feedforward output */
} pid_feedforward_t;

void pid_feedforward_init(pid_feedforward_t *ff, double kff,
                          double dist_nominal, double t_lead, double t_lag);

double pid_feedforward_update(pid_feedforward_t *ff, double disturbance, double dt);

/* ============================================================================
 * L6: Gain Scheduling - Parameter-varying PID
 * ============================================================================
 */

#define PID_GAIN_SCHEDULE_MAX 10

/** Gain scheduling point: PID parameters at a specific operating condition */
typedef struct {
    double pv_point;    /* PV value at which these gains apply */
    double kc;          /* Controller gain at this point */
    double ti;          /* Integral time at this point */
    double td;          /* Derivative time at this point */
} pid_gain_point_t;

/** Gain schedule: piecewise-linear interpolation of PID gains vs PV */
typedef struct {
    pid_gain_point_t points[PID_GAIN_SCHEDULE_MAX];
    int              count;
} pid_gain_schedule_t;

void pid_gain_schedule_init(pid_gain_schedule_t *gs);

int pid_gain_schedule_add(pid_gain_schedule_t *gs, double pv_point,
                          double kc, double ti, double td);

/** Interpolate PID gains for current PV */
bool pid_gain_schedule_lookup(const pid_gain_schedule_t *gs, double pv,
                              double *kc, double *ti, double *td);

#endif /* PID_CONTROLLER_H */