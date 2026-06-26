/**
 * @file pid_core.h
 * @brief PID Controller Core -- Definitions, Structures, and Basic Operations
 *
 * Covers knowledge levels:
 *   L1 -- PID definitions: Kp/Ki/Kd, setpoint, process variable, error, output
 *   L2 -- Core concepts: proportional/integral/derivative action, feedback loop
 *   L3 -- Math structures: Laplace transfer function G(s)=Kp*(1+1/(Ti*s)+Td*s)
 *   L4 -- Fundamental laws: BIBO stability, Final Value Theorem for steady-state error
 *
 * Reference curriculum:
 *   MIT 6.302 -- Feedback Systems
 *   Stanford EE267 -- Digital Control
 *   Berkeley EE128 -- Feedback Control Systems
 *   ETH 227-0216 -- Control Systems II
 */

#ifndef PID_CORE_H
#define PID_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L1 -- Core Definitions
 *===========================================================================*/

/**
 * @brief PID controller form / algorithm type
 *
 * Parallel:  u(t) = Kp*e(t) + Ki*integral(e)dt + Kd*de/dt
 * Standard:  u(t) = Kp*(e(t) + 1/Ti*integral(e)dt + Td*de/dt)
 * Series:    u(t) = Kp*(1 + 1/(Ti*s))*(1 + Td*s)*e(t)
 */
typedef enum {
    PID_FORM_PARALLEL = 0,  /**< Parallel/ideal form: independent gains */
    PID_FORM_STANDARD = 1,  /**< Standard/ISA form: Kp, Ti, Td */
    PID_FORM_SERIES   = 2   /**< Series/interacting form */
} PIDForm;

/**
 * @brief PID controller operation mode
 */
typedef enum {
    PID_MODE_AUTO     = 0,  /**< Automatic: full PID control */
    PID_MODE_MANUAL   = 1,  /**< Manual: output set by operator */
    PID_MODE_TRACKING = 2   /**< Bumpless tracking: PID follows external signal */
} PIDMode;

/**
 * @brief Derivative action mode
 */
typedef enum {
    PID_DTERM_ERROR       = 0,  /**< Derivative on error (standard) */
    PID_DTERM_MEASUREMENT = 1   /**< Derivative on measurement (PV) */
} PIDDerivativeMode;

/**
 * @brief Anti-windup method selection
 */
typedef enum {
    PID_AW_NONE       = 0,  /**< No anti-windup */
    PID_AW_CLAMPING   = 1,  /**< Conditional integration (clamping) */
    PID_AW_BACK_CALC  = 2,  /**< Back-calculation tracking */
    PID_AW_COMBINED   = 3   /**< Combined clamping + back-calculation */
} PIDAntiWindup;

/*===========================================================================
 * L1 -- PID Parameter Structure
 *===========================================================================*/

/**
 * @brief PID tuning parameters
 */
typedef struct {
    double Kp;        /**< Proportional gain */
    double Ki;        /**< Integral gain (1/s for parallel) */
    double Kd;        /**< Derivative gain (s for parallel) */
    double Ti;        /**< Integral time (s) -- standard form */
    double Td;        /**< Derivative time (s) -- standard form */
    double N;         /**< Derivative filter pole ratio (typically 2..20) */
    double Ts;        /**< Sampling period (s) -- 0 for continuous-time */
    double b;         /**< Setpoint weight on proportional term */
    double c;         /**< Setpoint weight on derivative term */
    double Kff;       /**< Feedforward gain */
    PIDForm form;     /**< Controller form */
} PIDParams;

/*===========================================================================
 * L1 -- PID Internal State
 *===========================================================================*/

/**
 * @brief PID controller internal state
 */
typedef struct {
    double integral;            /**< Accumulated integral term */
    double prev_error;          /**< Error at previous sample, e(k-1) */
    double prev_measurement;    /**< Measurement at previous sample, y(k-1) */
    double prev_output;         /**< Output at previous sample, u(k-1) */
    double tracking_input;      /**< External tracking value for bumpless transfer */
    double filtered_derivative; /**< Low-pass filtered derivative term */
    uint64_t sample_count;      /**< Number of sampling iterations executed */
    bool initialized;           /**< Whether state has been properly initialized */
} PIDState;

/*===========================================================================
 * L1 -- PID Controller Object
 *===========================================================================*/

/**
 * @brief PID Controller -- complete runtime object
 */
typedef struct {
    PIDParams params;              /**< Tuning parameters */
    PIDState  state;               /**< Internal state */
    PIDMode   mode;                /**< Operating mode */
    PIDDerivativeMode dterm_mode;  /**< Derivative computation target */
    PIDAntiWindup  aw_method;      /**< Anti-windup method */
    double  out_min;               /**< Output lower saturation limit */
    double  out_max;               /**< Output upper saturation limit */
    double  int_min;               /**< Integrator lower limit */
    double  int_max;               /**< Integrator upper limit */
    double  aw_gain;               /**< Anti-windup back-calculation gain */
    double  alpha;                 /**< Derivative filter smoothing factor [0,1] */
    bool    tracking_enabled;      /**< Whether bumpless tracking is active */
} PIDController;

/*===========================================================================
 * L2 -- Core API: Initialization and Configuration
 *===========================================================================*/

/**
 * @brief Initialize PID controller with default parameters
 * Defaults: Kp=1.0, Ki=0.0, Kd=0.0, Ts=0.01, N=10, b=1, c=0,
 *           derivative on measurement, no output limits.
 */
void pid_init(PIDController *pid, PIDForm form);

/**
 * @brief Initialize PID controller with explicit parameters
 */
void pid_init_params(PIDController *pid, PIDForm form,
                     double Kp, double Ki, double Kd, double Ts);

/**
 * @brief Set PID gains from standard (ISA) form parameters
 * Standard form: u = Kp*(e + 1/Ti*integral(e)dt + Td*de/dt)
 */
void pid_set_standard_gains(PIDController *pid, double Kp, double Ti, double Td);

/**
 * @brief Configure output saturation limits
 * Reference: Astrom & Hagglund (2006), Advanced PID Control, Chapter 3.
 */
void pid_set_output_limits(PIDController *pid, double out_min, double out_max);

/**
 * @brief Configure integral windup limits
 */
void pid_set_integral_limits(PIDController *pid, double int_min, double int_max);

/**
 * @brief Configure anti-windup method
 * @param aw_gain Back-calculation gain (1/Tt). For PID_AW_BACK_CALC/COMBINED.
 */
void pid_set_antiwindup(PIDController *pid, PIDAntiWindup method, double aw_gain);

/**
 * @brief Configure derivative mode
 */
void pid_set_derivative_mode(PIDController *pid, PIDDerivativeMode mode);

/**
 * @brief Configure setpoint weights for 2-DOF PID
 * u = Kp*(b*r - y) + Ki*integral(r-y)dt + Kd*(c*dr/dt - dy/dt)
 * b=0 gives I-PD control (no proportional kick on setpoint change)
 * Reference: Astrom & Hagglund, PID Controllers: Theory, Design, and Tuning, 1995.
 */
void pid_set_setpoint_weights(PIDController *pid, double b, double c);

/**
 * @brief Set derivative filter pole ratio
 * D(s) = s*Kd*Td / (1 + s*Td/N), N typically 2..20
 */
void pid_set_derivative_filter(PIDController *pid, double N);

/*===========================================================================
 * L2 -- Core API: Runtime Operations
 *===========================================================================*/

/**
 * @brief Execute one PID iteration (discrete-time update)
 * Implements backward Euler integration and backward difference derivative
 * with configurable anti-windup, output clamping, and bumpless transfer.
 * Complexity: O(1) per call.
 */
double pid_update(PIDController *pid, double setpoint, double measurement);

/**
 * @brief Reset PID controller state (clear integrator, history)
 */
void pid_reset(PIDController *pid);

/**
 * @brief Set controller to manual mode with bumpless transfer
 */
void pid_set_manual(PIDController *pid, double manual_out);

/**
 * @brief Switch controller to automatic mode
 */
void pid_set_auto(PIDController *pid);

/**
 * @brief Enable bumpless tracking to an external signal
 * Used for cascade control and override control.
 */
void pid_set_tracking(PIDController *pid, double tracking_value);

/*===========================================================================
 * L2 -- Query API
 *===========================================================================*/

double pid_get_p_term(const PIDController *pid);
double pid_get_i_term(const PIDController *pid);
double pid_get_d_term(const PIDController *pid);
double pid_get_integral(const PIDController *pid);

/**
 * @brief Convert PID parameters between forms
 * Parallel -> Standard:  Ti = Kp/Ki,  Td = Kd/Kp
 * Standard -> Parallel:  Ki = Kp/Ti,  Kd = Kp*Td
 */
void pid_convert_params(PIDParams *dst, const PIDParams *src,
                        PIDForm src_form, PIDForm dst_form);

/**
 * @brief Compute the PID transfer function as a string
 */
int pid_transfer_function_str(const PIDController *pid, char *buf, size_t size);

/*===========================================================================
 * L3 -- Mathematical Structures
 *===========================================================================*/

/**
 * @brief Continuous-time PID transfer function coefficients
 * num(s) = n2*s^2 + n1*s + n0
 * den(s) = d2*s^2 + d1*s + d0
 */
typedef struct {
    double n2;  /**< Numerator s^2 coefficient */
    double n1;  /**< Numerator s^1 coefficient */
    double n0;  /**< Numerator s^0 coefficient */
    double d2;  /**< Denominator s^2 coefficient */
    double d1;  /**< Denominator s^1 coefficient */
    double d0;  /**< Denominator s^0 coefficient */
} PIDTransferFunction;

/**
 * @brief Extract the continuous-time transfer function coefficients
 */
void pid_get_transfer_function(const PIDController *pid,
                               PIDTransferFunction *tf, PIDForm form);

/**
 * @brief Evaluate the PID frequency response G(j*omega) at a given frequency
 * @param mag Output: magnitude |G(j*omega)|
 * @param phase Output: phase angle(G(j*omega)) in radians
 */
void pid_frequency_response(const PIDTransferFunction *tf, double omega,
                            double *mag, double *phase);

#ifdef __cplusplus
}
#endif

#endif /* PID_CORE_H */
