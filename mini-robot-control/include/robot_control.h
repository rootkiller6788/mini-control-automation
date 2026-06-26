#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "robot_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L5-L6: Robot Motion Control Algorithms
 *
 * Joint-space PD:        tau = Kp*e + Kd*e_dot
 * PD + Gravity:          tau = Kp*e + Kd*e_dot + G(q)
 * Computed Torque (CTC): tau = M*(qdd_d + Kd*e_dot + Kp*e) + C*qd + G
 * Jacobian Transpose:    tau = J^T * (Kp*(x_d - x) + Kd*(xd_d - xd))
 * Impedance:             tau = J^T * (K*(x_eq - x) - D*xd - M*xdd)
 *
 * Stability (Takegaki & Arimoto, 1981):
 *   PD + gravity compensation is globally asymptotically stable for
 *   setpoint control because V = 1/2*qd^T*M*qd + 1/2*e^T*Kp*e is a
 *   Lyapunov function with V_dot = -qd^T*Kd*qd <= 0.
 *
 * References:
 *   Craig (2018) Ch. 10
 *   Siciliano et al. (2010) Ch. 8
 *   Spong, Hutchinson, Vidyasagar (2006) "Robot Modeling and Control"
 *   Takegaki & Arimoto (1981) ASME JDSMC 103(2):119-125
 *   Hogan (1985) "Impedance Control", ASME JDSMC 107:1-24
 *   Slotine & Li (1987) "Adaptive Manipulator Control", IJRR 6(3)
 * ========================================================================= */

/* PID Controller */
typedef struct {
    double *kp; double *ki; double *kd;
    double *integral; double *prev_error;
    double integral_limit; double output_limit;
} robot_pid_t;

/* Computed Torque Controller */
typedef struct {
    double *kp; double *kd; double *desired_qdd;
} robot_ctc_t;

/* Impedance Controller */
typedef struct {
    double mass; double stiffness; double damping;
    double force_deadband; double max_displacement;
    int axis;
} robot_impedance_params_t;

typedef struct {
    robot_impedance_params_t *axes; size_t n_axes;
    vec3_t equilibrium_position; quat_t equilibrium_orientation;
    vec3_t current_displacement; vec3_t velocity;
    vec3_t external_force; double time_constant;
} robot_impedance_t;

/* Force Controller */
typedef struct {
    double desired_force; double force_gain;
    double force_integral_gain; double force_derivative_gain;
    double force_integral; double prev_force_error;
    int axis; int is_torque;
} robot_force_ctrl_t;

/* Adaptive Controller (Slotine & Li, 1987) */
typedef struct {
    double *theta_hat; double *Gamma; double *regressor;
    double *s; double *Lambda; double *kd;
    size_t n_params; double sigma_mod;
} robot_adaptive_ctrl_t;

/* ---- PID ---- */
int robot_pid_init(robot_pid_t *pid, size_t n_dof, double kp, double ki, double kd);
void robot_pid_free(robot_pid_t *pid);
void robot_pid_control(robot_pid_t *pid, const double *q, const double *q_des,
                       const double *qd, const double *qd_des,
                       double dt, double *tau_out);
void robot_pid_reset(robot_pid_t *pid);

/* ---- CTC ---- */
int robot_ctc_init(robot_ctc_t *ctc, size_t n_dof, double kp, double kd);
void robot_ctc_free(robot_ctc_t *ctc);
int robot_ctc_control(robot_ctc_t *ctc, const robot_model_t *model,
                      const double *q, const double *qd,
                      const double *q_des, const double *qd_des,
                      const double *qdd_des, double *tau_out);

/* ---- PD+Gravity ---- */
int robot_pd_gravity_control(const robot_model_t *model, const double *q,
                             const double *qd, const double *q_des,
                             const double *kp, const double *kd,
                             double *tau_out);

/* ---- Jacobian Transpose ---- */
int robot_jacobian_transpose_control(const robot_model_t *model,
                                     const double *q, const double *qd,
                                     const pose3d_t *x_des,
                                     const twist_t *xd_des,
                                     const double *kp, const double *kd,
                                     double *tau_out);

/* ---- Impedance ---- */
int robot_impedance_init(robot_impedance_t *imp, size_t n_axes,
                         double mass, double stiffness, double damping);
void robot_impedance_free(robot_impedance_t *imp);
int robot_impedance_control(robot_impedance_t *imp, const robot_model_t *model,
                            const double *q, const double *qd,
                            const wrench_t *external_wrench,
                            double dt, double *tau_out);

/* ---- Force ---- */
int robot_force_ctrl_init(robot_force_ctrl_t *fc, double desired_force,
                          double gain, int axis, int is_torque);
void robot_force_ctrl_free(robot_force_ctrl_t *fc);
int robot_force_control(robot_force_ctrl_t *fc, const robot_model_t *model,
                        const double *q, const double *qd,
                        double measured_force, double dt, double *tau_out);

/* ---- Adaptive ---- */
int robot_adaptive_init(robot_adaptive_ctrl_t *adapt, size_t n_dof,
                        size_t n_params);
void robot_adaptive_free(robot_adaptive_ctrl_t *adapt);
int robot_adaptive_control(robot_adaptive_ctrl_t *adapt,
                           const robot_model_t *model,
                           const double *q, const double *qd,
                           const double *q_des, const double *qd_des,
                           const double *qdd_des, double dt,
                           double *tau_out);

/* ---- Joint Limit Avoidance ---- */
int robot_joint_limit_avoidance(const robot_model_t *model,
                                const double *q, double K_limit,
                                double *tau_repulsive);

/* ---- Performance Evaluation ---- */
int robot_control_evaluate(const robot_model_t *model,
                           const double *q_history, const double *q_des_history,
                           const double *tau_history,
                           size_t n_steps, double dt,
                           control_performance_t *perf);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_CONTROL_H */
