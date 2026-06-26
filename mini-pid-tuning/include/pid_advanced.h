/**
 * @file pid_advanced.h
 * @brief Advanced PID Control Structures
 *
 * Covers knowledge levels:
 *   L6 -- Canonical Problems: cascade control, feedforward control,
 *         gain scheduling, ratio control, override/selector control
 *   L8 -- Advanced Topics: adaptive PID, fuzzy PID, fractional-order PID,
 *         nonlinear PID, MPC-inspired PID, event-based PID
 *
 * Reference:
 *   Astrom & Hagglund (2006), "Advanced PID Control"
 *   Visioli (2006), "Practical PID Control"
 *   Yu (2006), "Autotuning of PID Controllers"
 */

#ifndef PID_ADVANCED_H
#define PID_ADVANCED_H

#include "pid_core.h"
#include "pid_tuning.h"
#include "pid_analysis.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L6 -- Cascade Control
 *===========================================================================*/

/**
 * @brief Cascade PID control structure (primary + secondary loop)
 *
 * Primary loop (outer):   controls the main process variable (e.g., temperature)
 * Secondary loop (inner): controls an intermediate variable (e.g., flow rate)
 *
 * Advantages: faster disturbance rejection, reduced effect of nonlinearities
 * in the inner loop, simpler tuning (tune inner first, then outer).
 */
typedef struct {
    PIDController primary;    /**< Primary (outer) PID controller */
    PIDController secondary;  /**< Secondary (inner) PID controller */
    double secondary_setpoint; /**< Setpoint computed by primary for secondary */
    double secondary_measurement; /**< Current secondary process variable */
    bool   cascade_active;    /**< Whether cascade mode is active */
} CascadePID;

/**
 * @brief Initialize cascade PID controller
 * @param cas          Cascade controller.
 * @param p_form, s_form PID forms for primary and secondary.
 */
void cascade_pid_init(CascadePID *cas, PIDForm p_form, PIDForm s_form);

/**
 * @brief Execute one cascade PID iteration
 *
 * 1. Primary PID: e1 = setpoint - primary_measurement -> u1
 * 2. Secondary PID: e2 = u1 - secondary_measurement -> u2 (actual output)
 *
 * @param cas                  Cascade controller.
 * @param setpoint             Main setpoint.
 * @param primary_measurement  Primary (outer) process variable.
 * @param secondary_measurement Secondary (inner) process variable.
 * @return double              Final control output (secondary output).
 */
double cascade_pid_update(CascadePID *cas, double setpoint,
                          double primary_measurement,
                          double secondary_measurement);

/*===========================================================================
 * L6 -- Feedforward Control
 *===========================================================================*/

/**
 * @brief PID with feedforward control
 *
 * u = u_pid + Kff * d
 * where d is a measured disturbance.
 *
 * Feedforward provides immediate compensation for measurable disturbances,
 * without waiting for feedback to detect the error.
 */
typedef struct {
    PIDController pid;       /**< Feedback PID controller */
    double Kff_static;       /**< Static feedforward gain */
    double Kff_dynamic;      /**< Dynamic feedforward gain (lead-lag) */
    double ff_lead;          /**< Feedforward lead time constant */
    double ff_lag;           /**< Feedforward lag time constant */
    double prev_disturbance; /**< Previous disturbance value for dynamic FF */
    double ff_output;        /**< Feedforward contribution */
} FeedforwardPID;

/**
 * @brief Initialize feedforward PID
 */
void feedforward_pid_init(FeedforwardPID *ffpid, PIDForm form);

/**
 * @brief Update feedforward PID
 *
 * Static FF:  u_ff = Kff_static * disturbance
 * Dynamic FF: u_ff = Kff_dynamic * (ff_lead*s + 1)/(ff_lag*s + 1) * disturbance
 *
 * @param ffpid       Feedforward PID controller.
 * @param setpoint    Control setpoint.
 * @param measurement Process value.
 * @param disturbance Measured disturbance.
 * @return double     Combined output = u_pid + u_ff.
 */
double feedforward_pid_update(FeedforwardPID *ffpid, double setpoint,
                              double measurement, double disturbance);

/*===========================================================================
 * L6 -- Gain Scheduling
 *===========================================================================*/

/**
 * @brief Gain scheduling table entry
 */
typedef struct {
    double scheduling_variable; /**< Value of the scheduling variable */
    double Kp;                  /**< Proportional gain at this point */
    double Ki;                  /**< Integral gain at this point */
    double Kd;                  /**< Derivative gain at this point */
} GainScheduleEntry;

/**
 * @brief Gain-scheduled PID controller
 *
 * PID gains are automatically adjusted based on a scheduling variable
 * (e.g., process output, setpoint, or an external measurement).
 * Between table entries, linear interpolation is used.
 */
typedef struct {
    PIDController       pid;           /**< Base PID controller */
    GainScheduleEntry  *table;         /**< Gain schedule table (sorted by scheduling variable) */
    size_t              table_size;    /**< Number of entries in table */
    double              current_sv;    /**< Current scheduling variable value */
    bool                enabled;       /**< Whether gain scheduling is active */
} GainScheduledPID;

/**
 * @brief Initialize gain-scheduled PID
 *
 * @param gspid      Gain-scheduled PID.
 * @param form       Base PID form.
 * @param table      Gain schedule table (must outlive the controller).
 * @param table_size Number of table entries.
 */
void gs_pid_init(GainScheduledPID *gspid, PIDForm form,
                 const GainScheduleEntry *table, size_t table_size);

/**
 * @brief Update gain-scheduled PID
 *
 * Before computing PID output, interpolates Kp/Ki/Kd from the table
 * based on current scheduling variable value.
 *
 * @param gspid       Gain-scheduled PID.
 * @param setpoint    Setpoint.
 * @param measurement Process value.
 * @param sched_var   Current scheduling variable value.
 * @return double     Control output.
 */
double gs_pid_update(GainScheduledPID *gspid, double setpoint,
                     double measurement, double sched_var);

/*===========================================================================
 * L8 -- Nonlinear PID
 *===========================================================================*/

/**
 * @brief Nonlinear PID with error-dependent gains
 *
 * Uses a nonlinear function to modify gains based on error magnitude:
 * - Large error: higher proportional gain for fast response
 * - Small error: lower proportional gain to reduce overshoot
 *
 * Also supports nonlinear integral action (conditional integration)
 * and nonlinear derivative (derivative on PV with variable gain).
 *
 * Reference: Han (1994), "Nonlinear PID Controller"
 */
typedef struct {
    PIDController pid;           /**< Base PID controller */
    double (*nonlinear_gain)(double error, double param); /**< Gain-modifying function */
    double nlp_param;            /**< Parameter for nonlinear function */
    double error_threshold;      /**< Error threshold for gain modification */
    bool   use_nonlinear_integral; /**< Whether to use conditional integration */
} NonlinearPID;

/**
 * @brief Initialize nonlinear PID
 */
void nonlinear_pid_init(NonlinearPID *nlpid, PIDForm form);

/**
 * @brief Update nonlinear PID
 *
 * Modifies effective gain based on error magnitude:
 *   Kp_eff = Kp * f(|error|, nlp_param)
 *
 * @return double Control output.
 */
double nonlinear_pid_update(NonlinearPID *nlpid, double setpoint,
                            double measurement);

/**
 * @brief Common nonlinear gain functions
 */

/** Hyperbolic tangent soft gain: large errors get saturating gain */
double nl_gain_tanh(double error, double param);

/** Linear-with-deadzone: no gain change within threshold */
double nl_gain_deadzone(double error, double param);

/** Quadratic gain: gain increases with error squared */
double nl_gain_quadratic(double error, double param);

/*===========================================================================
 * L8 -- Fractional-Order PID (FO-PID)
 *===========================================================================*/

/**
 * @brief Fractional-Order PID controller
 *
 * PI^lambda D^mu: G_c(s) = Kp + Ki/s^lambda + Kd*s^mu
 * where lambda, mu are real numbers in [0, 1].
 *
 * lambda = mu = 1: standard PID
 * lambda = 1, mu = 0: PI
 * lambda = 0, mu = 1: PD
 * lambda = mu = 0.5: half-order PID
 *
 * Discrete approximation uses Oustaloup recursive filter for s^alpha.
 *
 * Reference: Podlubny (1999), "Fractional-order systems and PID controllers"
 */
typedef struct {
    PIDController pid;       /**< Base PID (for storage and compatibility) */
    double lambda;           /**< Fractional integral order (0 < lambda <= 1) */
    double mu;               /**< Fractional derivative order (0 < mu <= 1) */
    double *frac_int_state;  /**< Fractional integrator state (Oustaloup filter) */
    double *frac_diff_state; /**< Fractional differentiator state */
    size_t filter_order;     /**< Oustaloup filter approximation order */
} FractionalOrderPID;

/**
 * @brief Initialize fractional-order PID
 *
 * @param fopid   Fractional-order PID.
 * @param form    Base PID form.
 * @param lambda  Integral order (0..1].
 * @param mu      Derivative order (0..1].
 * @param N_order Oustaloup filter order (typically 2..5).
 */
void fopid_init(FractionalOrderPID *fopid, PIDForm form,
                double lambda, double mu, size_t N_order);

/**
 * @brief Update fractional-order PID
 * @return double Control output.
 */
double fopid_update(FractionalOrderPID *fopid, double setpoint,
                    double measurement);

/**
 * @brief Free fractional-order PID internal state
 */
void fopid_free(FractionalOrderPID *fopid);

/*===========================================================================
 * L8 -- Event-Based PID
 *===========================================================================*/

/**
 * @brief Event-based (send-on-delta) PID controller
 *
 * PID output is only updated when the measurement changes by more than a
 * specified threshold, or when a timeout expires. Reduces computation
 * and communication in networked control systems.
 *
 * Reference: Astrom & Bernhardsson (2002), "Comparison of Riemann and
 * Lebesgue sampling for first order stochastic systems"
 */
typedef struct {
    PIDController pid;           /**< Base PID controller */
    double delta_threshold;      /**< Measurement change threshold */
    double timeout;              /**< Maximum time between updates (seconds) */
    double last_update_time;     /**< Time of last output update */
    double last_sent_measurement; /**< Last measurement that triggered update */
    double time_since_update;    /**< Elapsed time since last update */
    double last_output;          /**< Held output value */
    size_t update_count;         /**< Number of triggered updates */
    size_t sample_count;         /**< Total number of samples seen */
} EventBasedPID;

/**
 * @brief Initialize event-based PID
 */
void eb_pid_init(EventBasedPID *ebpid, PIDForm form,
                 double delta_threshold, double timeout);

/**
 * @brief Update event-based PID (call at each sampling instant)
 *
 * Only actually recomputes and updates the output when the triggering
 * condition is met.
 *
 * @param ebpid       Event-based PID.
 * @param setpoint    Setpoint.
 * @param measurement Current process value.
 * @param dt          Time since last call (seconds).
 * @param updated     Output: set to 1 if output was updated, 0 if held.
 * @return double     Control output (held or newly computed).
 */
double eb_pid_update(EventBasedPID *ebpid, double setpoint,
                     double measurement, double dt, int *updated);

/*===========================================================================
 * L6 -- Ratio Control
 *===========================================================================*/

/**
 * @brief Ratio PID controller
 *
 * Maintains a fixed ratio between two process variables:
 *   y1 / y2 = ratio_setpoint
 * Typically used for mixing/blending processes (e.g., fuel/air ratio).
 */
typedef struct {
    PIDController pid;       /**< PID controller acting on ratio error */
    double ratio_setpoint;   /**< Desired ratio y1/y2 */
    double y1_measurement;   /**< Numerator process variable */
    double y2_measurement;   /**< Denominator process variable (wild flow) */
} RatioPID;

void ratio_pid_init(RatioPID *rpid, PIDForm form);

/**
 * @brief Update ratio PID
 *
 * Computes: error = y1 - ratio_setpoint * y2
 * (or ratio-based: setpoint_y1 = ratio_setpoint * y2)
 *
 * @return double Control output.
 */
double ratio_pid_update(RatioPID *rpid, double y1, double y2);

/*===========================================================================
 * L8 -- Adaptive PID (Model Reference)
 *===========================================================================*/

/**
 * @brief Model Reference Adaptive PID (MRAC-PID)
 *
 * Adjusts PID gains online to make the closed-loop response match
 * a specified reference model.
 *
 * Uses MIT rule or gradient descent to update gains.
 *
 * Reference: Astrom & Wittenmark (1995), "Adaptive Control"
 */
typedef struct {
    PIDController pid;         /**< Base PID controller */
    FOPDTModel    process_model; /**< Estimated process model */
    FOPDTModel    ref_model;     /**< Desired reference model */
    double adaptation_gain;     /**< Adaptation rate (learning rate) */
    double prev_error;          /**< Error from previous step */
    double model_output;        /**< Reference model output */
    double model_state;         /**< Reference model state */
} MRACPID;

void mrac_pid_init(MRACPID *mrac, PIDForm form,
                   const FOPDTModel *ref_model, double adapt_gain);

/**
 * @brief Update MRAC-PID
 *
 * 1. Compute PID output
 * 2. Update reference model
 * 3. Compute adaptation error: model_output - actual_measurement
 * 4. Adjust Kp using MIT rule: dKp/dt = -gamma * error * sensitivity
 *
 * @return double Control output.
 */
double mrac_pid_update(MRACPID *mrac, double setpoint, double measurement,
                       double dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_ADVANCED_H */
