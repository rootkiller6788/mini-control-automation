/**
 * @file pid_tuning.h
 * @brief PID Tuning Methods -- Structures and API
 *
 * Covers knowledge levels:
 *   L5 -- Algorithms/Methods: Ziegler-Nichols, Cohen-Coon, Tyreus-Luyben,
 *         AMIGO, IMC-based, Relay auto-tuning, Lambda tuning
 *   L6 -- Canonical Problems: First-order-plus-dead-time (FOPDT) identification,
 *         Ultimate gain/period estimation, reaction curve analysis
 *
 * Reference:
 *   Ziegler & Nichols (1942), "Optimum Settings for Automatic Controllers"
 *   Cohen & Coon (1953), "Theoretical Consideration of Retarded Control"
 *   Astrom & Hagglund (1995), "PID Controllers: Theory, Design, and Tuning"
 *   Skogestad (2003), "Simple analytic rules for model reduction and PID controller tuning"
 */

#ifndef PID_TUNING_H
#define PID_TUNING_H

#include "pid_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L5 -- Tuning Method Enumeration
 *===========================================================================*/

/**
 * @brief PID tuning method identifiers
 */
typedef enum {
    TUNE_ZN_OPEN_LOOP   = 0,   /**< Ziegler-Nichols open-loop (reaction curve) */
    TUNE_ZN_CLOSED_LOOP = 1,   /**< Ziegler-Nichols closed-loop (ultimate gain) */
    TUNE_COHEN_COON     = 2,   /**< Cohen-Coon method */
    TUNE_TYREUS_LUYBEN  = 3,   /**< Tyreus-Luyben (conservative, for integrating processes) */
    TUNE_AMIGO          = 4,   /**< AMIGO (Approximate M-constrained Integral Gain Optimization) */
    TUNE_IMC            = 5,   /**< Internal Model Control based tuning */
    TUNE_LAMBDA         = 6,   /**< Lambda tuning (Dahlin) for first-order processes */
    TUNE_CHIEN_HRONES_RESWICK = 7, /**< CHR setpoint tracking tuning */
    TUNE_CHIEN_HRONES_RESWICK_DIST = 8, /**< CHR disturbance rejection tuning */
    TUNE_RELAY          = 9    /**< Relay (Astrom-Hagglund) auto-tuning */
} PIDTuningMethod;

/*===========================================================================
 * L5 -- Process Model: First-Order Plus Dead Time (FOPDT)
 *===========================================================================*/

/**
 * @brief FOPDT model parameters
 *
 * G(s) = K * exp(-L*s) / (T*s + 1)
 *
 * K  = static gain (steady-state output change / input change)
 * T  = time constant (seconds) -- time to reach 63.2% of final value
 * L  = dead time / transport delay (seconds)
 *
 * Ratio tau = L/T characterizes controllability:
 *   tau < 0.1: easy to control
 *   tau 0.1-1.0: controllable with PID
 *   tau > 1.0: difficult, may need Smith predictor or advanced control
 */
typedef struct {
    double K;   /**< Static gain */
    double T;   /**< Time constant (seconds) */
    double L;   /**< Dead time (seconds) */
} FOPDTModel;

/*===========================================================================
 * L5 -- Second-Order Plus Dead Time (SOPDT)
 *===========================================================================*/

/**
 * @brief SOPDT model parameters
 *
 * G(s) = K * exp(-L*s) / ((T1*s + 1)*(T2*s + 1))
 */
typedef struct {
    double K;    /**< Static gain */
    double T1;   /**< First time constant (seconds) */
    double T2;   /**< Second time constant (seconds) */
    double L;    /**< Dead time (seconds) */
} SOPDTModel;

/*===========================================================================
 * L5 -- Reaction Curve (Step Response) Data
 *===========================================================================*/

/**
 * @brief Step response experiment data for process identification
 *
 * The reaction curve method (Ziegler-Nichols open-loop):
 *   1. Put controller in manual mode
 *   2. Apply a step change delta_u to the process input
 *   3. Record the process output y(t)
 *   4. Fit FOPDT model from the step response
 *
 * The tangent method: draw the steepest tangent to the step response.
 *   L = intercept of tangent with initial value line
 *   T = (final_value - initial_value) / (slope_of_tangent)
 *   K = (final_value - initial_value) / delta_u
 */
typedef struct {
    double *time;          /**< Time vector [0..N-1] */
    double *output;        /**< Process output vector [0..N-1] */
    size_t  N;             /**< Number of data points */
    double  input_step;    /**< Magnitude of input step change */
    double  initial_value; /**< Process output before step */
    double  final_value;   /**< Steady-state process output after step */
} StepResponseData;

/*===========================================================================
 * L5 -- Ultimate Gain Experiment Data
 *===========================================================================*/

/**
 * @brief Closed-loop relay / ultimate gain experiment
 *
 * Ziegler-Nichols closed-loop method:
 *   1. Set Ki = Kd = 0 (P-only control)
 *   2. Increase Kp until the loop oscillates with constant amplitude
 *   3. Record ultimate gain Ku and ultimate period Pu
 *
 * Tuning rules:
 *   P:   Kp = 0.50 * Ku
 *   PI:  Kp = 0.45 * Ku,  Ti = Pu / 1.2
 *   PID: Kp = 0.60 * Ku,  Ti = Pu / 2.0,  Td = Pu / 8.0
 *
 * For relay auto-tuning (Astrom-Hagglund):
 *   Ku = 4*d / (pi*a)
 *   Pu = measured oscillation period
 *   where d = relay amplitude, a = process output oscillation amplitude
 */
typedef struct {
    double Ku;          /**< Ultimate gain (proportional gain at stability limit) */
    double Pu;          /**< Ultimate period (seconds) -- oscillation period at Ku */
    double relay_amplitude;  /**< Relay output amplitude (for relay method) */
    double oscillation_amplitude; /**< Process output oscillation amplitude */
} UltimateGainData;

/*===========================================================================
 * L5 -- Tuning Result
 *===========================================================================*/

/**
 * @brief PID tuning result with method and parameters
 */
typedef struct {
    PIDTuningMethod method; /**< Method used */
    PIDParams params;       /**< Tuned parameters */
    double expected_overshoot; /**< Expected overshoot (fraction, e.g., 0.25 = 25%) */
    double expected_settling_time; /**< Expected settling time (seconds) */
    bool   valid;           /**< Whether tuning produced valid parameters */
    char   note[256];       /**< Method-specific notes or warnings */
} PIDTuningResult;

/*===========================================================================
 * L5 -- Tuning API
 *===========================================================================*/

/**
 * @brief Identify FOPDT model from step response data (reaction curve)
 *
 * Uses the tangent method (maximum slope) to estimate K, T, L.
 * Reference: Ziegler & Nichols (1942)
 * Complexity: O(N) where N is the number of data points.
 *
 * @param data  Step response experiment data.
 * @param model Output FOPDT model.
 * @return 0 on success, -1 if data is insufficient.
 */
int pid_identify_fopdt(const StepResponseData *data, FOPDTModel *model);

/**
 * @brief Identify FOPDT model using the area method (more robust to noise)
 *
 * Uses the integral of the step response (area method) instead of
 * maximum-slope tangent. More noise-robust than the tangent method.
 * Reference: Astrom & Hagglund (1995), Section 2.7
 * Complexity: O(N).
 *
 * @param data  Step response experiment data.
 * @param model Output FOPDT model.
 * @return 0 on success, -1 if data is insufficient.
 */
int pid_identify_fopdt_area(const StepResponseData *data, FOPDTModel *model);

/**
 * @brief Identify SOPDT model from step response data
 *
 * Uses two-point method: measures times t1, t2 at which response reaches
 * y1, y2 (typically 28.3% and 63.2% of final value).
 * Reference: Smith (1972), "Digital Computer Process Control"
 * Complexity: O(N).
 *
 * @param data  Step response experiment data.
 * @param model Output SOPDT model.
 * @return 0 on success, -1 if data is insufficient.
 */
int pid_identify_sopdt(const StepResponseData *data, SOPDTModel *model);

/**
 * @brief Compute ultimate gain and period from the FOPDT model
 *
 * Using the describing function approximation:
 *   Ku = 2*pi*T / (K * L)  (approximate, for FOPDT)
 *   Pu = 2 * L             (approximate)
 *
 * More accurate: solve Ku * |G(j*wu)| = 1 and phase(G(j*wu)) = -pi
 * to find Ku and Pu = 2*pi/wu.
 *
 * @param model FOPDT process model.
 * @param ug    Output ultimate gain data.
 * @return 0 on success, -1 if model is invalid.
 */
int pid_compute_ultimate_gain(const FOPDTModel *model, UltimateGainData *ug);

/**
 * @brief Run relay auto-tuning experiment simulation
 *
 * Simulates the Astrom-Hagglund relay feedback experiment on a given
 * process model to determine ultimate gain and period.
 * Reference: Astrom & Hagglund (1984), "Automatic Tuning of Simple Regulators"
 *
 * @param model Process model (FOPDT).
 * @param relay_amplitude Relay output amplitude.
 * @param hysteresis Relay hysteresis band.
 * @param ug Output ultimate gain data.
 * @return 0 on success, -1 on error.
 */
int pid_relay_autotune(const FOPDTModel *model, double relay_amplitude,
                       double hysteresis, UltimateGainData *ug);

/**
 * @brief Tune PID using Ziegler-Nichols closed-loop method
 *
 * @param ug      Ultimate gain data.
 * @param form    Desired PID form.
 * @param ts      Sampling period for discrete implementation.
 * @param result  Output tuning result.
 * @return 0 on success, -1 if data is invalid.
 */
int pid_tune_zn_closed_loop(const UltimateGainData *ug, PIDForm form,
                            double ts, PIDTuningResult *result);

/**
 * @brief Tune PID using Ziegler-Nichols open-loop method
 *
 * @param model   FOPDT process model.
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_tune_zn_open_loop(const FOPDTModel *model, PIDForm form,
                          double ts, PIDTuningResult *result);

/**
 * @brief Tune PID using Cohen-Coon method
 *
 * Cohen-Coon tunes for quarter-amplitude decay ratio.
 * Better for processes with significant dead time than ZN.
 * Reference: Cohen & Coon (1953)
 *
 * @param model   FOPDT process model.
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_tune_cohen_coon(const FOPDTModel *model, PIDForm form,
                        double ts, PIDTuningResult *result);

/**
 * @brief Tune PID using Tyreus-Luyben method
 *
 * More conservative than ZN, designed for integrating processes
 * and chemical process control. Larger gain/phase margins.
 * Reference: Tyreus & Luyben (1992)
 *
 * @param ug      Ultimate gain data.
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_tune_tyreus_luyben(const UltimateGainData *ug, PIDForm form,
                           double ts, PIDTuningResult *result);

/**
 * @brief Tune PID using AMIGO method
 *
 * Approximate M-constrained Integral Gain Optimization.
 * Based on extensive simulation with robustness constraints (Ms <= 1.4).
 * Generally yields better performance than ZN for most processes.
 * Reference: Astrom & Hagglund (2004), "Revisiting the Ziegler-Nichols
 *            step response method for PID control"
 *
 * @param model   FOPDT process model.
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_tune_amigo(const FOPDTModel *model, PIDForm form,
                   double ts, PIDTuningResult *result);

/**
 * @brief Tune PID using IMC (Internal Model Control) method
 *
 * IMC-based tuning with a user-specified closed-loop time constant lambda.
 * Larger lambda = more robust, slower response.
 * Smaller lambda = more aggressive, faster response.
 * Reference: Rivera, Morari & Skogestad (1986), "Internal Model Control.
 *            4. PID Controller Design"
 *
 * @param model   FOPDT process model.
 * @param lambda  Desired closed-loop time constant (>= 0.1*T typical).
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_tune_imc(const FOPDTModel *model, double lambda, PIDForm form,
                 double ts, PIDTuningResult *result);

/**
 * @brief Tune PID using Lambda (Dahlin) method
 *
 * Similar to IMC but specifically derived for first-order processes.
 * Closed-loop response approximates: Y(s)/R(s) = exp(-L*s) / (lambda*s + 1)
 * Reference: Dahlin (1968), "Designing and tuning digital controllers"
 *
 * @param model   FOPDT process model.
 * @param lambda  Desired closed-loop time constant.
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_tune_lambda(const FOPDTModel *model, double lambda, PIDForm form,
                    double ts, PIDTuningResult *result);

/**
 * @brief CHR (Chien-Hrones-Reswick) tuning for setpoint tracking
 *
 * Designed for 0% or 20% overshoot with setpoint changes.
 * Reference: Chien, Hrones & Reswick (1952)
 *
 * @param model      FOPDT process model.
 * @param overshoot  0 for no overshoot, 20 for 20% overshoot.
 * @param form       Desired PID form.
 * @param ts         Sampling period.
 * @param result     Output tuning result.
 * @return 0 on success.
 */
int pid_tune_chr_setpoint(const FOPDTModel *model, int overshoot,
                          PIDForm form, double ts, PIDTuningResult *result);

/**
 * @brief CHR tuning for disturbance rejection
 *
 * @param model      FOPDT process model.
 * @param overshoot  0 for no overshoot, 20 for 20% overshoot.
 * @param form       Desired PID form.
 * @param ts         Sampling period.
 * @param result     Output tuning result.
 * @return 0 on success.
 */
int pid_tune_chr_disturbance(const FOPDTModel *model, int overshoot,
                             PIDForm form, double ts, PIDTuningResult *result);

/**
 * @brief Apply tuned parameters to a PID controller
 *
 * Converts the tuning result into the controller's configured form
 * and applies all parameters including N, b, c defaults.
 *
 * @param pid    Controller to configure.
 * @param result Tuning result to apply.
 */
void pid_apply_tuning(PIDController *pid, const PIDTuningResult *result);

/**
 * @brief Get human-readable name for a tuning method
 * @param method Tuning method enum value.
 * @return const char* Method name string.
 */
const char *pid_tuning_method_name(PIDTuningMethod method);

#ifdef __cplusplus
}
#endif

#endif /* PID_TUNING_H */
