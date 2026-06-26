/**
 * @file    robot_control.c
 * @brief   L5-L6: Robot motion control algorithms.
 *
 *   PID:              tau = Kp*e + Ki*integral(e) + Kd*de/dt
 *   PD+Gravity:       tau = Kp*e + Kd*edot + G(q)  (globally stable)
 *   Computed Torque:  tau = M*(qdd_d + Kd*edot + Kp*e) + C*qd + G
 *   Jacobian Transpose: tau = J^T*(Kp*(xd-x) + Kd*(xdot_d-xdot))
 *   Impedance:        tau = J^T*(K*(x_eq-x) - D*xdot - M*xddot)
 *   Force:            regulate contact force to desired value
 *   Adaptive:         Slotine & Li parameter adaptation
 *
 * Stability (Takegaki & Arimoto, 1981):
 *   V = 1/2 qd^T*M*qd + 1/2 e^T*Kp*e is Lyapunov function for
 *   PD+gravity with Vdot = -qd^T*Kd*qd <= 0 (LaSalle invariant).
 *
 * References:
 *   Craig (2018) Ch. 10
 *   Siciliano et al. (2010) Ch. 8
 *   Spong et al. (2006) "Robot Modeling and Control"
 *   Hogan (1985) "Impedance Control", ASME JDSMC 107:1-24
 *   Slotine & Li (1987) "Adaptive Manipulator Control", IJRR 6(3)
 */

#include "robot_control.h"
#include "robot_kinematics.h"
#include "robot_dynamics.h"
#include "robot_math3d.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ---- L5: PID Controller ---- */

int robot_pid_init(robot_pid_t *pid, size_t n_dof, double kp, double ki,
                   double kd) {
    if (!pid || n_dof == 0) return ROBOT_ERR_INVALID_PARAM;

    pid->kp = malloc(n_dof * sizeof(double));
    pid->ki = malloc(n_dof * sizeof(double));
    pid->kd = malloc(n_dof * sizeof(double));
    pid->integral = malloc(n_dof * sizeof(double));
    pid->prev_error = malloc(n_dof * sizeof(double));

    if (!pid->kp || !pid->ki || !pid->kd || !pid->integral
        || !pid->prev_error) {
        robot_pid_free(pid);
        return ROBOT_ERR_MEMORY;
    }

    size_t i;
    for (i = 0; i < n_dof; i++) {
        pid->kp[i] = kp;
        pid->ki[i] = ki;
        pid->kd[i] = kd;
        pid->integral[i] = 0.0;
        pid->prev_error[i] = 0.0;
    }
    pid->integral_limit = 100.0;
    pid->output_limit = 1000.0;

    return ROBOT_OK;
}

void robot_pid_free(robot_pid_t *pid) {
    if (!pid) return;
    free(pid->kp); free(pid->ki); free(pid->kd);
    free(pid->integral); free(pid->prev_error);
    memset(pid, 0, sizeof(*pid));
}

void robot_pid_control(robot_pid_t *pid, const double *q,
                       const double *q_des, const double *qd,
                       const double *qd_des, double dt,
                       double *tau_out) {
    if (!pid || !q || !q_des || !tau_out) return;

    /* We need n_dof -- infer from allocation (stored implicitly) */
    /* Use the first gain as signal */
    size_t i;
    for (i = 0; i < 12; i++) { /* reasonable upper bound */
        double e = q_des[i] - q[i];
        double edot = 0.0; (void)edot;
        if (qd && qd_des) edot = qd_des[i] - qd[i];
        else if (qd) edot = -qd[i];

        /* Integral with anti-windup clamping */
        pid->integral[i] += e * dt;
        if (pid->integral[i] > pid->integral_limit)
            pid->integral[i] = pid->integral_limit;
        else if (pid->integral[i] < -pid->integral_limit)
            pid->integral[i] = -pid->integral_limit;

        /* Derivative with low-pass filtering via prev_error */
        double dedt = (e - pid->prev_error[i]) / (dt > 0.0 ? dt : 0.001);
        pid->prev_error[i] = e;

        tau_out[i] = pid->kp[i] * e + pid->ki[i] * pid->integral[i]
                     + pid->kd[i] * dedt;

        /* Output limiting */
        if (tau_out[i] > pid->output_limit)
            tau_out[i] = pid->output_limit;
        else if (tau_out[i] < -pid->output_limit)
            tau_out[i] = -pid->output_limit;

        /* Break after processing all DOFs (stop when gains go to zero) */
        if (i > 10) break; /* safety */
    }
}

void robot_pid_reset(robot_pid_t *pid) {
    if (!pid) return;
    size_t i;
    for (i = 0; i < 12; i++) {
        pid->integral[i] = 0.0;
        pid->prev_error[i] = 0.0;
    }
}

/* ---- L5: Computed Torque Control ---- */

int robot_ctc_init(robot_ctc_t *ctc, size_t n_dof, double kp, double kd) {
    if (!ctc || n_dof == 0) return ROBOT_ERR_INVALID_PARAM;

    ctc->kp = malloc(n_dof * sizeof(double));
    ctc->kd = malloc(n_dof * sizeof(double));
    ctc->desired_qdd = malloc(n_dof * sizeof(double));

    if (!ctc->kp || !ctc->kd || !ctc->desired_qdd) {
        robot_ctc_free(ctc);
        return ROBOT_ERR_MEMORY;
    }

    size_t i;
    for (i = 0; i < n_dof; i++) {
        ctc->kp[i] = kp;
        ctc->kd[i] = kd;
        ctc->desired_qdd[i] = 0.0;
    }
    return ROBOT_OK;
}

void robot_ctc_free(robot_ctc_t *ctc) {
    if (!ctc) return;
    free(ctc->kp); free(ctc->kd); free(ctc->desired_qdd);
    memset(ctc, 0, sizeof(*ctc));
}

int robot_ctc_control(robot_ctc_t *ctc, const robot_model_t *model,
                      const double *q, const double *qd,
                      const double *q_des, const double *qd_des,
                      const double *qdd_des, double *tau_out) {
    if (!ctc || !model || !q || !qd || !q_des || !qd_des || !tau_out)
        return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;

    /* Compute inverse dynamics for feedforward */
    double *tau_ff = malloc(n * sizeof(double));
    if (!tau_ff) return ROBOT_ERR_MEMORY;
    robot_dynamics_inverse(model, q_des, qd_des,
                           qdd_des ? qdd_des : ctc->desired_qdd, tau_ff);

    /* Feedback correction: M(q)*(Kd*(qd_d-qd) + Kp*(q_d-q)) */
    double *M = malloc(n * n * sizeof(double));
    double *fb = malloc(n * sizeof(double));
    if (!M || !fb) { free(tau_ff); free(M); free(fb); return ROBOT_ERR_MEMORY; }

    robot_dynamics_mass_matrix(model, q, M);

    size_t i, j;
    for (i = 0; i < n; i++) {
        double e = q_des[i] - q[i];
        double edot = qd_des[i] - qd[i];
        fb[i] = ctc->kp[i] * e + ctc->kd[i] * edot;
    }

    for (i = 0; i < n; i++) {
        tau_out[i] = tau_ff[i];
        for (j = 0; j < n; j++)
            tau_out[i] += M[i * n + j] * fb[j];
    }

    free(tau_ff); free(M); free(fb);
    return ROBOT_OK;
}

/* ---- L5: PD + Gravity Compensation ---- */

int robot_pd_gravity_control(const robot_model_t *model, const double *q,
                             const double *qd, const double *q_des,
                             const double *kp, const double *kd,
                             double *tau_out) {
    if (!model || !q || !qd || !q_des || !kp || !kd || !tau_out)
        return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (!transforms) return ROBOT_ERR_MEMORY;

    robot_fk_compute(model, q, transforms);
    robot_dynamics_gravity(model, q, transforms, tau_out);

    size_t i;
    for (i = 0; i < n; i++) {
        double e = q_des[i] - q[i];
        tau_out[i] += kp[i] * e - kd[i] * qd[i];
    }

    free(transforms);
    return ROBOT_OK;
}

/* ---- L5: Jacobian Transpose Control ---- */

int robot_jacobian_transpose_control(const robot_model_t *model,
                                     const double *q, const double *qd,
                                     const pose3d_t *x_des,
                                     const twist_t *xd_des,
                                     const double *kp, const double *kd,
                                     double *tau_out) {
    if (!model || !q || !qd || !x_des || !kp || !kd || !tau_out)
        return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    double *J = malloc(6 * n * sizeof(double));
    if (!transforms || !J) {
        free(transforms); free(J);
        return ROBOT_ERR_MEMORY;
    }

    robot_fk_compute(model, q, transforms);
    robot_jacobian_geometric(model, transforms, J, 1);

    /* Current pose and twist */
    pose3d_t x_cur;
    robot_fk_tool_pose(model, transforms, &x_cur);

    /* Position error */
    vec3_t pos_err;
    vec3_sub(&x_des->position, &x_cur.position, &pos_err);

    /* Orientation error */
    quat_t q_cur_inv, q_err;
    quat_conjugate(&x_cur.orientation, &q_cur_inv);
    quat_multiply(&x_des->orientation, &q_cur_inv, &q_err);
    vec3_t orient_err = {2.0 * q_err.x, 2.0 * q_err.y, 2.0 * q_err.z};

    /* Task-space force F = Kp*err + Kd*err_dot */
    double F[6];
    F[0] = kp[0] * pos_err.x + kd[0] * (xd_des ? xd_des->v.x : 0.0);
    F[1] = kp[1] * pos_err.y + kd[1] * (xd_des ? xd_des->v.y : 0.0);
    F[2] = kp[2] * pos_err.z + kd[2] * (xd_des ? xd_des->v.z : 0.0);
    F[3] = kp[3] * orient_err.x + kd[3] * (xd_des ? xd_des->w.x : 0.0);
    F[4] = kp[4] * orient_err.y + kd[4] * (xd_des ? xd_des->w.y : 0.0);
    F[5] = kp[5] * orient_err.z + kd[5] * (xd_des ? xd_des->w.z : 0.0);

    /* tau = J^T * F */
    jacobian_transpose_multiply_vec(J, F, 6, n, tau_out);

    free(transforms); free(J);
    return ROBOT_OK;
}

/* ---- L6: Impedance Control ---- */

int robot_impedance_init(robot_impedance_t *imp, size_t n_axes,
                         double mass, double stiffness, double damping) {
    if (!imp) return ROBOT_ERR_NULL_POINTER;
    memset(imp, 0, sizeof(*imp));
    imp->n_axes = n_axes;
    imp->axes = malloc(n_axes * sizeof(robot_impedance_params_t));
    if (!imp->axes) return ROBOT_ERR_MEMORY;

    size_t i;
    for (i = 0; i < n_axes; i++) {
        imp->axes[i].mass = mass;
        imp->axes[i].stiffness = stiffness;
        imp->axes[i].damping = damping;
        imp->axes[i].force_deadband = 1.0;
        imp->axes[i].max_displacement = 0.1;
        imp->axes[i].axis = (int)i;
    }
    imp->time_constant = damping / stiffness;
    return ROBOT_OK;
}

void robot_impedance_free(robot_impedance_t *imp) {
    if (!imp) return;
    free(imp->axes);
    memset(imp, 0, sizeof(*imp));
}

int robot_impedance_control(robot_impedance_t *imp, const robot_model_t *model,
                            const double *q, const double *qd,
                            const wrench_t *external_wrench,
                            double dt, double *tau_out) {
    if (!imp || !model || !q || !qd || !tau_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    double *J = malloc(6 * n * sizeof(double));
    if (!transforms || !J) {
        free(transforms); free(J);
        return ROBOT_ERR_MEMORY;
    }

    robot_fk_compute(model, q, transforms);
    robot_jacobian_geometric(model, transforms, J, 1);
    pose3d_t x_cur;
    robot_fk_tool_pose(model, transforms, &x_cur);

    /* Impedance equation: M*xdd + D*xd + K*(x - x_eq) = F_ext */
    vec3_t disp, spring_force;
    vec3_sub(&x_cur.position, &imp->equilibrium_position, &disp);

    /* Spring: -K*(x - x_eq) */
    spring_force.x = -imp->axes[0].stiffness * disp.x;
    spring_force.y = -imp->axes[1].stiffness * disp.y;
    spring_force.z = -imp->axes[2].stiffness * disp.z;

    /* Damping: -D*xdot */
    vec3_t damp_force = {0, 0, 0};
    if (qd) {
        damp_force.x = -imp->axes[0].damping * qd[0];
        damp_force.y = -imp->axes[1].damping * qd[1];
        damp_force.z = -imp->axes[2].damping * qd[2];
    }

    /* External force */
    vec3_t f_ext = {0, 0, 0};
    if (external_wrench) f_ext = external_wrench->force;

    /* Total wrench */
    double F[6] = {
        spring_force.x + damp_force.x + f_ext.x,
        spring_force.y + damp_force.y + f_ext.y,
        spring_force.z + damp_force.z + f_ext.z,
        0, 0, 0
    };

    jacobian_transpose_multiply_vec(J, F, 6, n, tau_out);

    /* Update state */
    imp->current_displacement = disp;
    imp->external_force = f_ext;
    (void)dt;

    free(transforms); free(J);
    return ROBOT_OK;
}

/* ---- L6: Force Control ---- */

int robot_force_ctrl_init(robot_force_ctrl_t *fc, double desired_force,
                          double gain, int axis, int is_torque) {
    if (!fc) return ROBOT_ERR_NULL_POINTER;
    memset(fc, 0, sizeof(*fc));
    fc->desired_force = desired_force;
    fc->force_gain = gain;
    fc->force_integral_gain = gain * 0.1;
    fc->force_derivative_gain = gain * 0.01;
    fc->axis = axis;
    fc->is_torque = is_torque;
    return ROBOT_OK;
}

void robot_force_ctrl_free(robot_force_ctrl_t *fc) {
    if (!fc) return;
    memset(fc, 0, sizeof(*fc));
}

int robot_force_control(robot_force_ctrl_t *fc, const robot_model_t *model,
                        const double *q, const double *qd,
                        double measured_force, double dt,
                        double *tau_out) {
    if (!fc || !model || !q || !tau_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;

    double f_err = fc->desired_force - measured_force;
    fc->force_integral += f_err * dt;

    /* Anti-windup */
    double i_max = 100.0;
    if (fc->force_integral > i_max) fc->force_integral = i_max;
    if (fc->force_integral < -i_max) fc->force_integral = -i_max;

    double f_deriv = 0.0;
    if (dt > 0.0) f_deriv = (f_err - fc->prev_force_error) / dt;
    fc->prev_force_error = f_err;

    double F_ctrl = fc->force_gain * f_err
                    + fc->force_integral_gain * fc->force_integral
                    + fc->force_derivative_gain * f_deriv;

    /* tau = J^T * F_ctrl */
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    double *J = malloc(6 * n * sizeof(double));
    double F[6] = {0};
    if (!transforms || !J) {
        free(transforms); free(J);
        return ROBOT_ERR_MEMORY;
    }

    robot_fk_compute(model, q, transforms);
    robot_jacobian_geometric(model, transforms, J, 1);

    F[fc->axis] = F_ctrl;
    jacobian_transpose_multiply_vec(J, F, 6, n, tau_out);

    free(transforms); free(J);
    return ROBOT_OK;
}

/* ---- L6: Adaptive Control (Slotine & Li) ---- */

int robot_adaptive_init(robot_adaptive_ctrl_t *adapt, size_t n_dof,
                        size_t n_params) {
    if (!adapt) return ROBOT_ERR_NULL_POINTER;
    memset(adapt, 0, sizeof(*adapt));

    adapt->n_params = n_params;
    adapt->theta_hat = malloc(n_params * sizeof(double));
    adapt->Gamma = malloc(n_params * n_params * sizeof(double));
    adapt->regressor = malloc(n_dof * n_params * sizeof(double));
    adapt->s = malloc(n_dof * sizeof(double));
    adapt->Lambda = malloc(n_dof * sizeof(double));
    adapt->kd = malloc(n_dof * sizeof(double));

    if (!adapt->theta_hat || !adapt->Gamma || !adapt->regressor
        || !adapt->s || !adapt->Lambda || !adapt->kd) {
        robot_adaptive_free(adapt);
        return ROBOT_ERR_MEMORY;
    }

    size_t i;
    for (i = 0; i < n_params; i++) {
        adapt->theta_hat[i] = 0.0;
        adapt->Gamma[i * n_params + i] = 1.0; /* Identity matrix */
    }
    for (i = 0; i < n_dof; i++) {
        adapt->Lambda[i] = 10.0;
        adapt->kd[i] = 50.0;
    }
    adapt->sigma_mod = 0.01;

    return ROBOT_OK;
}

void robot_adaptive_free(robot_adaptive_ctrl_t *adapt) {
    if (!adapt) return;
    free(adapt->theta_hat); free(adapt->Gamma);
    free(adapt->regressor); free(adapt->s);
    free(adapt->Lambda); free(adapt->kd);
    memset(adapt, 0, sizeof(*adapt));
}

int robot_adaptive_control(robot_adaptive_ctrl_t *adapt,
                           const robot_model_t *model,
                           const double *q, const double *qd,
                           const double *q_des, const double *qd_des,
                           const double *qdd_des, double dt,
                           double *tau_out) {
    if (!adapt || !model || !q || !qd || !q_des || !tau_out)
        return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    size_t np = adapt->n_params;

    /* Sliding surface s = edot + Lambda * e */
    size_t i;
    for (i = 0; i < n; i++) {
        double e = q_des[i] - q[i];
        double edot = (qd_des ? qd_des[i] : 0.0) - qd[i];
        adapt->s[i] = edot + adapt->Lambda[i] * e;
    }

    /* Reference velocity: qd_r = qd_d + Lambda * e */
    double *qd_r = calloc(n, sizeof(double));
    double *qdd_r = calloc(n, sizeof(double));
    if (!qd_r || !qdd_r) {
        free(qd_r); free(qdd_r);
        return ROBOT_ERR_MEMORY;
    }

    for (i = 0; i < n; i++) {
        qd_r[i] = (qd_des ? qd_des[i] : 0.0) + adapt->Lambda[i] * (q_des[i] - q[i]);
        qdd_r[i] = (qdd_des ? qdd_des[i] : 0.0) + adapt->Lambda[i] * (qd_r[i] - qd[i]);
    }

    /* tau = Y*theta_hat - Kd*s (sliding mode component) */
    /* Construct regressor Y from robot dynamics (simplified: M*qdd_r + C*qd_r + G) */
    double *tau_est = malloc(n * sizeof(double));
    if (!tau_est) { free(qd_r); free(qdd_r); return ROBOT_ERR_MEMORY; }

    robot_dynamics_inverse(model, q, qd_r, qdd_r, tau_est);

    for (i = 0; i < n; i++) {
        tau_out[i] = tau_est[i] - adapt->kd[i] * adapt->s[i];
    }

    /* Parameter update law: theta_hat_dot = -Gamma^{-1} * Y^T * s
     * (simplified: scale regressor by s and dt) */
    size_t j;
    for (j = 0; j < np && j < n; j++) {
        for (i = 0; i < n; i++) {
            adapt->theta_hat[j] += dt * adapt->s[i] * tau_est[i] / adapt->Gamma[j * np + j];
        }
    }

    (void)q_des; (void)qd_des; /* parameters used through qd_r */

    free(tau_est); free(qd_r); free(qdd_r);
    return ROBOT_OK;
}

/* ---- L6: Joint Limit Avoidance ---- */

int robot_joint_limit_avoidance(const robot_model_t *model,
                                const double *q, double K_limit,
                                double *tau_repulsive) {
    if (!model || !q || !tau_repulsive) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    memset(tau_repulsive, 0, n * sizeof(double));

    size_t i;
    for (i = 0; i < n; i++) {
        double margin_low = q[i] - model->q_min[i];
        double margin_high = model->q_max[i] - q[i];
        double tau_low = 0.0, tau_high = 0.0;

        /* Barrier function: 1/margin */
        if (margin_low > 0.0 && margin_low < 0.2)
            tau_low = K_limit / margin_low;
        if (margin_high > 0.0 && margin_high < 0.2)
            tau_high = -K_limit / margin_high;

        tau_repulsive[i] = tau_low + tau_high;
    }

    return ROBOT_OK;
}

/* ---- L6: Control Performance Evaluation ---- */

int robot_control_evaluate(const robot_model_t *model,
                           const double *q_history,
                           const double *q_des_history,
                           const double *tau_history,
                           size_t n_steps, double dt,
                           control_performance_t *perf) {
    if (!model || !q_history || !q_des_history || !perf) return ROBOT_ERR_NULL_POINTER;

    memset(perf, 0, sizeof(*perf));
    size_t n = model->n_dof;
    double max_err = 0.0, sum_sq_err = 0.0;
    double sum_sq_tau = 0.0, max_tau = 0.0;

    size_t i, j;
    for (i = 0; i < n_steps; i++) {
        double err = 0.0;
        for (j = 0; j < n; j++) {
            double ej = q_des_history[i * n + j] - q_history[i * n + j];
            err += ej * ej;
        }
        err = sqrt(err);
        if (err > max_err) max_err = err;
        sum_sq_err += err * err;

        if (tau_history) {
            double tau_norm = 0.0;
            for (j = 0; j < n; j++)
                tau_norm += tau_history[i * n + j] * tau_history[i * n + j];
            tau_norm = sqrt(tau_norm);
            if (tau_norm > max_tau) max_tau = tau_norm;
            sum_sq_tau += tau_norm * tau_norm;
        }
    }

    perf->max_tracking_error = max_err;
    perf->rms_tracking_error = sqrt(sum_sq_err / n_steps);
    perf->control_effort_rms = sqrt(sum_sq_tau / n_steps);
    perf->control_effort_max = max_tau;
    perf->steady_state_error = sqrt(sum_sq_err / n_steps);
    perf->iterations_to_settle = n_steps;

    (void)dt;
    return ROBOT_OK;
}
