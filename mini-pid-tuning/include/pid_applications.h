/**
 * @file pid_applications.h
 * @brief PID Control Applications -- Domain-Specific Models and Helpers
 *
 * Covers knowledge levels:
 *   L6 -- Canonical Problems: DC motor speed/position control,
 *         temperature control (Peltier/heater), quadrotor attitude,
 *         liquid level control, inverted pendulum
 *   L7 -- Applications: industrial process control, automotive cruise control,
 *         drone stabilization, HVAC control, chemical reactor temperature
 *
 * Reference:
 *   Ogata (2010), "Modern Control Engineering", Chapter 8
 *   Franklin, Powell & Emami-Naeini (2014), "Feedback Control of Dynamic Systems"
 *   Dorf & Bishop (2016), "Modern Control Systems"
 */

#ifndef PID_APPLICATIONS_H
#define PID_APPLICATIONS_H

#include "pid_core.h"
#include "pid_tuning.h"
#include "pid_analysis.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L6 -- DC Motor Model
 *===========================================================================*/

/**
 * @brief DC motor dynamic model
 *
 * Electrical: L*di/dt + R*i + Ke*omega = V
 * Mechanical: J*domega/dt + b*omega = Kt*i - T_load
 *
 * Transfer function (voltage to speed):
 *   G(s) = Kt / ((L*s + R)*(J*s + b) + Kt*Ke)
 *
 * Simplified (L << R): first-order
 *   G(s) = K / (tau*s + 1)
 *   where K = Kt/(R*b + Kt*Ke), tau = R*J/(R*b + Kt*Ke)
 */
typedef struct {
    double R;       /**< Armature resistance (ohm) */
    double L;       /**< Armature inductance (H) */
    double Ke;      /**< Back-EMF constant (V/(rad/s)) */
    double Kt;      /**< Torque constant (N*m/A) */
    double J;       /**< Rotor inertia (kg*m^2) */
    double b;       /**< Viscous friction (N*m/(rad/s)) */
    double T_load;  /**< Load torque (N*m) */
} DCMotorModel;

/**
 * @brief Initialize DC motor with typical parameters
 *
 * @param motor DC motor model.
 * @param type  0: small hobby motor (1-10W), 1: medium industrial (100W-1kW),
 *              2: large (1kW+), 3: micro (<1W).
 */
void dc_motor_init(DCMotorModel *motor, int type);

/**
 * @brief Get FOPDT approximation of DC motor for PID tuning
 *
 * @param motor DC motor model.
 * @param fopdt Output FOPDT model.
 * @param voltage_range Input voltage range (V).
 * @return 0 on success.
 */
int dc_motor_to_fopdt(const DCMotorModel *motor, FOPDTModel *fopdt,
                      double voltage_range);

/**
 * @brief Simulate DC motor open-loop step response
 *
 * Solves the ODE using Euler integration.
 *
 * @param motor     Motor model.
 * @param voltage   Applied voltage.
 * @param dt        Time step.
 * @param duration  Total simulation time.
 * @param time      Output time array (caller allocated).
 * @param speed     Output speed array (rad/s).
 * @param current   Output current array (A).
 * @param N         Number of time steps.
 * @return 0 on success.
 */
int dc_motor_simulate(const DCMotorModel *motor, double voltage,
                      double dt, double duration,
                      double *time, double *speed, double *current, size_t N);

/*===========================================================================
 * L6 -- Thermal System Model (First-Order)
 *===========================================================================*/

/**
 * @brief Thermal process model (lumped capacitance)
 *
 * m*Cp*dT/dt = Q_in - Q_out
 * Q_out = (T - T_ambient) / Rth
 *
 * G(s) = Kth / (tau_th * s + 1)
 * where Kth = Rth, tau_th = m*Cp*Rth
 */
typedef struct {
    double mass;        /**< Thermal mass (kg) */
    double Cp;          /**< Specific heat capacity (J/(kg*K)) */
    double Rth;         /**< Thermal resistance to ambient (K/W) */
    double T_ambient;   /**< Ambient temperature (K or C) */
    double max_power;   /**< Maximum heater power (W) */
} ThermalModel;

/**
 * @brief Initialize thermal model
 *
 * @param tm   Thermal model.
 * @param type 0: small oven/incubator, 1: water bath,
 *             2: large industrial furnace, 3: Peltier/thermoelectric.
 */
void thermal_model_init(ThermalModel *tm, int type);

/**
 * @brief Get FOPDT model for thermal system
 *
 * @param tm    Thermal model.
 * @param fopdt Output FOPDT model.
 */
void thermal_to_fopdt(const ThermalModel *tm, FOPDTModel *fopdt);

/**
 * @brief Simulate thermal system response
 *
 * @param tm     Thermal model.
 * @param power  Heater power (W), clamped to [0, max_power].
 * @param dt     Time step.
 * @param duration Total time.
 * @param time   Output time array.
 * @param temp   Output temperature array.
 * @param N      Number of steps.
 * @return 0 on success.
 */
int thermal_simulate(const ThermalModel *tm, double power,
                     double dt, double duration,
                     double *time, double *temp, size_t N);

/*===========================================================================
 * L6 -- Quadrotor Attitude Model (Single Axis)
 *===========================================================================*/

/**
 * @brief Single-axis quadrotor attitude model
 *
 * J*ddtheta = tau - b*dtheta
 * where tau is the torque from differential motor thrust.
 *
 * Transfer function (torque to angle):
 *   G(s) = 1 / (J*s^2 + b*s)
 */
typedef struct {
    double J;         /**< Moment of inertia about axis (kg*m^2) */
    double b;         /**< Aerodynamic damping (N*m/(rad/s)) */
    double arm_length; /**< Motor arm length (m) */
    double thrust_coeff; /**< Thrust coefficient per motor (N/(PWM^2)) */
    double max_torque;   /**< Maximum available torque (N*m) */
} QuadrotorAxisModel;

/**
 * @brief Initialize quadrotor axis model
 *
 * @param qm   Quadrotor model.
 * @param type 0: micro (e.g., Crazyflie), 1: mini (250mm),
 *             2: medium (450mm), 3: large (650mm+).
 */
void quadrotor_axis_init(QuadrotorAxisModel *qm, int type);

/**
 * @brief Get SOPDT model for quadrotor axis
 *
 * @param qm    Quadrotor model.
 * @param sopdt Output SOPDT model.
 */
void quadrotor_to_sopdt(const QuadrotorAxisModel *qm, SOPDTModel *sopdt);

/**
 * @brief Simulate quadrotor axis dynamics
 *
 * @param qm      Quadrotor model.
 * @param torque  Applied torque (N*m).
 * @param dt      Time step.
 * @param duration Total time.
 * @param time    Output time array.
 * @param angle   Output angle array (rad).
 * @param rate    Output angular rate array (rad/s).
 * @param N       Number of steps.
 * @return 0 on success.
 */
int quadrotor_simulate(const QuadrotorAxisModel *qm, double torque,
                       double dt, double duration,
                       double *time, double *angle, double *rate, size_t N);

/*===========================================================================
 * L6 -- Liquid Level Control (Tank System)
 *===========================================================================*/

/**
 * @brief Single-tank liquid level model
 *
 * A*dh/dt = q_in - q_out
 * q_out = Cv * sqrt(2*g*h)  (for free discharge)
 *
 * Linearized about operating point h0:
 *   tau*d(delta_h)/dt + delta_h = K*delta_q_in
 * where tau = 2*A*sqrt(h0/g)/Cv, K = 2*sqrt(h0/g)/Cv
 */
typedef struct {
    double A;          /**< Tank cross-sectional area (m^2) */
    double Cv;         /**< Outlet valve coefficient (m^2) */
    double g;          /**< Gravitational acceleration (m/s^2) */
    double max_level;  /**< Maximum tank level (m) */
    double operating_level; /**< Operating point level for linearization (m) */
} TankModel;

/**
 * @brief Initialize tank level model
 */
void tank_model_init(TankModel *tm, int type);

/**
 * @brief Get FOPDT linearized model around operating point
 */
void tank_to_fopdt(const TankModel *tm, FOPDTModel *fopdt);

/**
 * @brief Simulate nonlinear tank dynamics
 *
 * @param tm     Tank model.
 * @param q_in   Inflow rate (m^3/s).
 * @param dt     Time step.
 * @param duration Total time.
 * @param time   Output time array.
 * @param level  Output level array (m).
 * @param N      Number of steps.
 * @return 0 on success.
 */
int tank_simulate(const TankModel *tm, double q_in,
                  double dt, double duration,
                  double *time, double *level, size_t N);

/*===========================================================================
 * L6 -- Inverted Pendulum Model
 *===========================================================================*/

/**
 * @brief Inverted pendulum on a cart (linearized)
 *
 * State: [x, theta, dx/dt, dtheta/dt]
 * Linearized about theta = 0 (upright position).
 *
 * M*ddx + m*l*ddtheta = F  (cart dynamics)
 * (I + m*l^2)*ddtheta + m*l*ddx - m*g*l*theta = 0  (pendulum dynamics)
 *
 * Classical control benchmark problem.
 * Reference: Ogata (2010), "Modern Control Engineering", Example 10-2.
 */
typedef struct {
    double M;      /**< Cart mass (kg) */
    double m;      /**< Pendulum mass (kg) */
    double l;      /**< Pendulum length from pivot to CoM (m) */
    double I;      /**< Pendulum moment of inertia about CoM (kg*m^2) */
    double g;      /**< Gravitational acceleration (m/s^2) */
    double max_force; /**< Maximum cart force (N) */
} InvertedPendulumModel;

/**
 * @brief Initialize inverted pendulum model
 */
void inv_pendulum_init(InvertedPendulumModel *ipm, int type);

/**
 * @brief Simulate inverted pendulum dynamics (nonlinear)
 *
 * Uses Runge-Kutta 4th order integration.
 *
 * @param ipm   Inverted pendulum model.
 * @param force Cart force (N).
 * @param dt    Time step.
 * @param duration Total time.
 * @param time  Output time array.
 * @param theta Output pendulum angle array (rad, 0 = upright).
 * @param x     Output cart position array (m).
 * @param N     Number of steps.
 * @return 0 on success.
 */
int inv_pendulum_simulate(const InvertedPendulumModel *ipm, double force,
                          double dt, double duration,
                          double *time, double *theta, double *x, size_t N);

/*===========================================================================
 * L7 -- Application Helper Functions
 *===========================================================================*/

/**
 * @brief Auto-tune PID for a given application and model
 *
 * Selects the best tuning method based on process characteristics.
 * Decision logic based on the L/T ratio:
 *   L/T < 0.1:  Lambda/IMC tuning (easy process)
 *   0.1 <= L/T < 0.5: AMIGO (medium difficulty)
 *   0.5 <= L/T < 1.0: Cohen-Coon (significant dead time)
 *   L/T >= 1.0: Ziegler-Nichols closed-loop (dead-time dominant)
 *
 * @param model   FOPDT process model.
 * @param form    Desired PID form.
 * @param ts      Sampling period.
 * @param result  Output tuning result.
 * @return 0 on success.
 */
int pid_autotune(const FOPDTModel *model, PIDForm form,
                 double ts, PIDTuningResult *result);

/**
 * @brief Evaluate PID closed-loop performance via simulation
 *
 * Runs a full step-response simulation, computes metrics, and provides
 * a quality score (0-100).
 *
 * @param pid    Configured PID controller.
 * @param model  Process model.
 * @param score  Output performance score (0 = worst, 100 = best).
 * @param metrics Output detailed metrics.
 * @return 0 on success.
 */
int pid_performance_score(const PIDController *pid, const FOPDTModel *model,
                          double *score, StepResponseMetrics *metrics);

/**
 * @brief Print PID tuning comparison table
 *
 * Runs multiple tuning methods on the same FOPDT model and prints
 * a comparison of resulting parameters and expected performance.
 *
 * @param model Process model.
 * @param ts    Sampling period.
 */
void pid_tuning_comparison(const FOPDTModel *model, double ts);

/**
 * @brief Get process type name for a given FOPDT model
 *
 * Based on L/T ratio:
 *   L/T < 0.1    -> "Lag-dominant"
 *   0.1..0.3     -> "Balanced"
 *   0.3..0.7     -> "Moderate dead-time"
 *   0.7..1.5     -> "Dead-time dominant"
 *   > 1.5        -> "Severe dead-time (Smith predictor recommended)"
 *
 * @param model   FOPDT model.
 * @param buf     Output buffer.
 * @param size    Buffer size.
 * @return const char* buf pointer.
 */
const char *pid_process_type_name(const FOPDTModel *model, char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* PID_APPLICATIONS_H */
