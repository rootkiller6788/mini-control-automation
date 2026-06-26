/**
 * @file    robot_kinematics.c
 * @brief   L1-L4: Forward kinematics (DH chain), inverse kinematics
 *          (Newton-Raphson, gradient descent, CCD), geometric Jacobian,
 *          singularity detection.
 *
 * Knowledge Coverage:
 *   L1 (Definitions):   FK chain multiplication, DH convention
 *   L2 (Core Concepts): IK solving, differential kinematics
 *   L3 (Math):          Jacobian computation, pseudoinverse
 *   L4 (Laws):          Denavit-Hartenberg theorem, Chasles screw theorem
 *
 * References:
 *   Craig (2018) Ch. 3-5
 *   Siciliano et al. (2010) Ch. 2-3
 *   Buss (2004) "Intro to Inverse Kinematics", IEEE RA Mag
 *   Denavit & Hartenberg (1955) ASME JAM 22:215-221
 */

#include "robot_kinematics.h"
#include "robot_math3d.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* =================================================================
 * L2: Forward Kinematics
 * ================================================================= */

int robot_fk_compute(const robot_model_t *model, const double *q,
                     mat4_t *transforms) {
    if (!model || !q || !transforms) return ROBOT_ERR_NULL_POINTER;

    mat4_t T_joint;
    mat4_t T_accum;
    size_t i;

    mat4_identity(&transforms[0]);

    /* Build base-to-world transform */
    mat4_t T_baseworld;
    mat4_from_pose(&model->base_pose, &T_baseworld);
    T_accum = T_baseworld;

    for (i = 0; i < model->n_dof; i++) {
        dh_param_t *dh = &model->dh_params[i];
        double theta_val = dh->is_revolute ? dh->theta + q[i] : dh->theta;
        double d_val = dh->is_revolute ? dh->d : dh->d + q[i];

        mat4_dh_standard(dh->alpha, dh->a, d_val, theta_val, &T_joint);
        mat4_multiply(&T_accum, &T_joint, &transforms[i + 1]);
        T_accum = transforms[i + 1];
    }

    return ROBOT_OK;
}

int robot_fk_tool_pose(const robot_model_t *model, const mat4_t *transforms,
                       pose3d_t *tool_pose) {
    if (!model || !transforms || !tool_pose) return ROBOT_ERR_NULL_POINTER;

    const mat4_t *T_tool = &transforms[model->n_dof];
    mat4_to_pose(T_tool, tool_pose);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Inverse Kinematics -- Newton-Raphson
 *
 * Iteration: q_{k+1} = q_k + J^+(q_k)*(x_d - f(q_k))
 * where J^+ = J^T*(J*J^T + lambda^2*I)^{-1} (damped least squares)
 * ================================================================= */

/* Solve (J*J^T + lambda^2*I) * dy = dx via Gaussian elimination
 * Assumes JJt is 6x6 (in-place modified) */
static int solve_6x6(double *JJt, double *dx) {
    size_t n = 6, i, j, k;
    for (k = 0; k < n; k++) {
        /* Partial pivot */
        size_t pivot = k;
        double max_val = fabs(JJt[k * n + k]);
        for (i = k + 1; i < n; i++) {
            if (fabs(JJt[i * n + k]) > max_val) {
                max_val = fabs(JJt[i * n + k]);
                pivot = i;
            }
        }
        if (max_val < 1e-15) return -1;
        if (pivot != k) {
            for (j = k; j < n; j++) {
                double tmp = JJt[k * n + j];
                JJt[k * n + j] = JJt[pivot * n + j];
                JJt[pivot * n + j] = tmp;
            }
            double tmp = dx[k]; dx[k] = dx[pivot]; dx[pivot] = tmp;
        }

        double piv = JJt[k * n + k];
        for (j = k; j < n; j++) JJt[k * n + j] /= piv;
        dx[k] /= piv;

        for (i = k + 1; i < n; i++) {
            double factor = JJt[i * n + k];
            for (j = k; j < n; j++) JJt[i * n + j] -= factor * JJt[k * n + j];
            dx[i] -= factor * dx[k];
        }
    }
    /* Back substitution */
    for (k = n; k-- > 0; ) {
        for (i = 0; i < k; i++)
            dx[i] -= JJt[i * n + k] * dx[k];
    }
    return 0;
}

int robot_ik_newton_raphson(const robot_model_t *model, const double *q_init,
                            const pose3d_t *target, double *q_out,
                            double tol, size_t max_iter, double lambda) {
    if (!model || !q_init || !target || !q_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    double *q = malloc(n * sizeof(double));
    double *q_new = malloc(n * sizeof(double));
    double *J = malloc(6 * n * sizeof(double));
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (!q || !q_new || !J || !transforms) {
        free(q); free(q_new); free(J); free(transforms);
        return ROBOT_ERR_MEMORY;
    }
    memcpy(q, q_init, n * sizeof(double));

    size_t iter;
    for (iter = 0; iter < max_iter; iter++) {
        /* Forward kinematics */
        robot_fk_compute(model, q, transforms);
        pose3d_t current_pose;
        robot_fk_tool_pose(model, transforms, &current_pose);

        /* Position error */
        vec3_t pos_err;
        vec3_sub(&target->position, &current_pose.position, &pos_err);

        /* Orientation error: q_err = q_target * conj(q_cur) */
        quat_t q_err, q_target = target->orientation;
        quat_t q_cur = current_pose.orientation;
        quat_t q_cur_inv;
        quat_conjugate(&q_cur, &q_cur_inv);
        /* q_err = q_target * conj(q_cur) -- this gives the delta quaternion */
        quat_multiply(&q_target, &q_cur_inv, &q_err);
        if (q_err.w > 1.0) q_err.w = 1.0;
        if (q_err.w < -1.0) q_err.w = -1.0;

        vec3_t orient_err;
        double half_angle = acos(q_err.w);
        double sin_half = sqrt(1.0 - q_err.w * q_err.w);
        if (sin_half > 1e-10) {
            orient_err.x = 2.0 * half_angle * q_err.x / sin_half;
            orient_err.y = 2.0 * half_angle * q_err.y / sin_half;
            orient_err.z = 2.0 * half_angle * q_err.z / sin_half;
        } else {
            vec3_zero(&orient_err);
        }

        /* Check convergence */
        double pos_norm = vec3_norm(&pos_err);
        double ori_norm = vec3_norm(&orient_err);
        if (pos_norm < tol && ori_norm < tol) {
            memcpy(q_out, q, n * sizeof(double));
            free(q); free(q_new); free(J); free(transforms);
            return ROBOT_OK;
        }

        /* Compute Jacobian */
        robot_jacobian_geometric(model, transforms, J, 1);

        /* Task-space error vector [6] */
        double dx[6];
        dx[0] = pos_err.x; dx[1] = pos_err.y; dx[2] = pos_err.z;
        dx[3] = orient_err.x; dx[4] = orient_err.y; dx[5] = orient_err.z;

        /* Damped least squares: dq = J^T*(J*J^T + lambda^2*I)^{-1} * dx */
        double JJt[36] = {0};
        size_t i, j, k;
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
                for (k = 0; k < n; k++)
                    JJt[i * 6 + j] += J[i * n + k] * J[j * n + k];
                if (i == j) JJt[i * 6 + j] += lambda * lambda;
            }
        }

        double dy[6];
        memcpy(dy, dx, 6 * sizeof(double));
        if (solve_6x6(JJt, dy) != 0) {
            /* Singular: reduce lambda and skip iteration */
            lambda *= 2.0;
            continue;
        }

        /* dq = J^T * dy */
        double dq[12]; /* max n_dof = 12 */
        jacobian_transpose_multiply_vec(J, dy, 6, n, dq);

        /* Apply update */
        for (i = 0; i < n; i++)
            q_new[i] = q[i] + dq[i];

        robot_clamp_to_limits(model, q_new);
        memcpy(q, q_new, n * sizeof(double));
    }

    /* Did not converge -- return last iteration */
    memcpy(q_out, q, n * sizeof(double));
    free(q); free(q_new); free(J); free(transforms);
    return ROBOT_ERR_NO_SOLUTION;
}

/* =================================================================
 * L2: Inverse Kinematics -- Gradient Descent
 * ================================================================= */

int robot_ik_gradient_descent(const robot_model_t *model, const double *q_init,
                              const pose3d_t *target, double *q_out,
                              double tol, size_t max_iter, double alpha) {
    if (!model || !q_init || !target || !q_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    double *q = malloc(n * sizeof(double));
    double *J = malloc(6 * n * sizeof(double));
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (!q || !J || !transforms) {
        free(q); free(J); free(transforms);
        return ROBOT_ERR_MEMORY;
    }
    memcpy(q, q_init, n * sizeof(double));

    size_t iter;
    for (iter = 0; iter < max_iter; iter++) {
        robot_fk_compute(model, q, transforms);
        pose3d_t cur;
        robot_fk_tool_pose(model, transforms, &cur);

        vec3_t pos_err;
        vec3_sub(&target->position, &cur.position, &pos_err);

        /* Orientation error */
        quat_t q_cur_inv, q_err;
        quat_conjugate(&cur.orientation, &q_cur_inv);
        quat_multiply(&target->orientation, &q_cur_inv, &q_err);

        vec3_t orient_err = {2.0 * q_err.x, 2.0 * q_err.y, 2.0 * q_err.z};
        if (q_err.w < 0.0) {
            orient_err.x = -orient_err.x;
            orient_err.y = -orient_err.y;
            orient_err.z = -orient_err.z;
        }

        if (vec3_norm(&pos_err) < tol && vec3_norm(&orient_err) < tol) {
            memcpy(q_out, q, n * sizeof(double));
            free(q); free(J); free(transforms);
            return ROBOT_OK;
        }

        robot_jacobian_geometric(model, transforms, J, 1);

        double dx[6];
        dx[0] = pos_err.x; dx[1] = pos_err.y; dx[2] = pos_err.z;
        dx[3] = orient_err.x; dx[4] = orient_err.y; dx[5] = orient_err.z;

        double dq[12];
        jacobian_transpose_multiply_vec(J, dx, 6, n, dq);

        size_t i;
        for (i = 0; i < n; i++)
            q[i] += alpha * dq[i];

        robot_clamp_to_limits(model, q);
    }

    memcpy(q_out, q, n * sizeof(double));
    free(q); free(J); free(transforms);
    return ROBOT_ERR_NO_SOLUTION;
}

/* =================================================================
 * L2: Inverse Kinematics -- CCD (Cyclic Coordinate Descent)
 *
 * For each joint from tip to base, compute the rotation that
 * minimizes the distance between end-effector and target.
 * ================================================================= */

int robot_ik_ccd(const robot_model_t *model, const double *q_init,
                 const pose3d_t *target, double *q_out,
                 double tol, size_t max_iter) {
    if (!model || !q_init || !target || !q_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    double *q = malloc(n * sizeof(double));
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (!q || !transforms) {
        free(q); free(transforms);
        return ROBOT_ERR_MEMORY;
    }
    memcpy(q, q_init, n * sizeof(double));

    size_t iter;
    for (iter = 0; iter < max_iter; iter++) {
        robot_fk_compute(model, q, transforms);
        pose3d_t cur;
        robot_fk_tool_pose(model, transforms, &cur);

        if (vec3_distance(&cur.position, &target->position) < tol) {
            memcpy(q_out, q, n * sizeof(double));
            free(q); free(transforms);
            return ROBOT_OK;
        }

        /* Process joints from tip to base */
        size_t joint;
        for (joint = n; joint-- > 0; ) {
            /* Get joint position */
            vec3_t joint_pos;
            mat4_extract_translation(&transforms[joint], &joint_pos);

            /* Vectors from joint to end-effector and joint to target */
            vec3_t to_ee, to_target;
            vec3_sub(&cur.position, &joint_pos, &to_ee);
            vec3_sub(&target->position, &joint_pos, &to_target);

            double n_ee = vec3_norm(&to_ee);
            double n_target = vec3_norm(&to_target);
            if (n_ee < 1e-10 || n_target < 1e-10) continue;

            vec3_t to_ee_norm, to_target_norm;
            vec3_scale(&to_ee, 1.0 / n_ee, &to_ee_norm);
            vec3_scale(&to_target, 1.0 / n_target, &to_target_norm);

            /* Rotation axis (perpendicular to both vectors) */
            vec3_t axis;
            vec3_cross(&to_ee_norm, &to_target_norm, &axis);
            double cross_norm = vec3_norm(&axis);

            if (cross_norm < 1e-10) continue; /* Aligned */

            vec3_normalize(&axis);
            double dot_val = vec3_dot(&to_ee_norm, &to_target_norm);
            if (dot_val > 1.0) dot_val = 1.0;
            if (dot_val < -1.0) dot_val = -1.0;
            double angle = acos(dot_val);

            /* Apply rotation about joint axis */
            if (model->dh_params[joint].is_revolute) {
                double proj = vec3_dot(&axis, &model->links[joint].joint_axis);
                q[joint] += angle * proj;
            }
        }

        robot_clamp_to_limits(model, q);
    }

    memcpy(q_out, q, n * sizeof(double));
    free(q); free(transforms);
    return ROBOT_ERR_NO_SOLUTION;
}

/* =================================================================
 * L2: Geometric Jacobian
 * ================================================================= */

int robot_jacobian_geometric(const robot_model_t *model, const mat4_t *transforms,
                              double *J, int use_world_frame) {
    if (!model || !transforms || !J) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    jacobian_set_zero(J, 6, n);

    vec3_t p_tool;
    mat4_extract_translation(&transforms[n], &p_tool);

    size_t i;
    for (i = 0; i < n; i++) {
        vec3_t p_i;
        mat4_extract_translation(&transforms[i], &p_i);

        /* z-axis of frame i-1 is the third column (rotation) */
        vec3_t z_i;
        z_i.x = transforms[i].m[2];
        z_i.y = transforms[i].m[6];
        z_i.z = transforms[i].m[10];

        if (model->dh_params[i].is_revolute) {
            /* Linear: z_i x (p_tool - p_i) */
            vec3_t dp, Jv;
            vec3_sub(&p_tool, &p_i, &dp);
            vec3_cross(&z_i, &dp, &Jv);

            J[0 * n + i] = Jv.x;
            J[1 * n + i] = Jv.y;
            J[2 * n + i] = Jv.z;

            /* Angular: z_i */
            J[3 * n + i] = z_i.x;
            J[4 * n + i] = z_i.y;
            J[5 * n + i] = z_i.z;
        } else {
            /* Prismatic: linear = z_i, angular = 0 */
            J[0 * n + i] = z_i.x;
            J[1 * n + i] = z_i.y;
            J[2 * n + i] = z_i.z;
            J[3 * n + i] = 0.0;
            J[4 * n + i] = 0.0;
            J[5 * n + i] = 0.0;
        }
    }

    if (!use_world_frame) {
        /* Transform Jacobian to body frame: R^T_0n applied */
        mat3_t R_tool_T;
        mat4_extract_rotation(&transforms[n], &R_tool_T);
        mat3_t R;
        mat3_transpose(&R_tool_T, &R);

        for (i = 0; i < n; i++) {
            vec3_t Jv = {J[0 * n + i], J[1 * n + i], J[2 * n + i]};
            vec3_t Jw = {J[3 * n + i], J[4 * n + i], J[5 * n + i]};
            mat3_vec_multiply(&R, &Jv, &Jv);
            mat3_vec_multiply(&R, &Jw, &Jw);
            J[0 * n + i] = Jv.x; J[1 * n + i] = Jv.y; J[2 * n + i] = Jv.z;
            J[3 * n + i] = Jw.x; J[4 * n + i] = Jw.y; J[5 * n + i] = Jw.z;
        }
    }

    return ROBOT_OK;
}

/* =================================================================
 * L2: Analytic Jacobian
 * ================================================================= */

int robot_jacobian_analytic(const robot_model_t *model, const mat4_t *transforms,
                            const double *q, double *J) {
    if (!model || !transforms || !J) return ROBOT_ERR_NULL_POINTER;

    /* Start with geometric Jacobian */
    robot_jacobian_geometric(model, transforms, J, 1);

    /* Convert angular velocity part to ZYX Euler angle rates.
     * omega = T(phi) * d(phi)/dt
     * where T(phi) maps Euler rates to angular velocity.
     *
     * For ZYX: T(phi) = [0 -s(phi) c(phi)*c(theta); 0 c(phi) s(phi)*c(theta); 1 0 -s(theta)]
     * T^{-1} is applied to the angular velocity rows.
     */

    pose3d_t pose;
    robot_fk_tool_pose(model, transforms, &pose);

    /* Extract current Euler angles */
    mat3_t R;
    quat_to_mat3(&pose.orientation, &R);
    euler_t euler;
    mat3_to_euler_zyx(&R, &euler);
    double cr = cos(euler.roll), sr = sin(euler.roll);
    double ct = cos(euler.pitch), st = sin(euler.pitch);

    if (fabs(ct) > 1e-8) {
        double inv_ct = 1.0 / ct;
        size_t i;
        for (i = 0; i < model->n_dof; i++) {
            double wx = J[3 * model->n_dof + i];
            double wy = J[4 * model->n_dof + i];
            double wz = J[5 * model->n_dof + i];

            J[3 * model->n_dof + i] = wx + sr * st * inv_ct * wy + cr * st * inv_ct * wz;
            J[4 * model->n_dof + i] = cr * wy - sr * wz;
            J[5 * model->n_dof + i] = sr * inv_ct * wy + cr * inv_ct * wz;
        }
    }

    return ROBOT_OK;
}

/* =================================================================
 * L2: Singularity Detection and Manipulability
 * ================================================================= */

int robot_singularity_detect(const double *J, size_t n_dof,
                             double condition_threshold) {
    if (!J) return -1;

    /* Condition number approx via JJ^T determinant check */
    double JJt[36] = {0}; /* 6x6 max */
    size_t i, j, k;
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
            for (k = 0; k < n_dof; k++)
                JJt[i * 6 + j] += J[i * n_dof + k] * J[j * n_dof + k];

    /* Compute Frobenius norm and condition estimate */
    double frob = 0.0;
    for (i = 0; i < 36; i++) frob += JJt[i] * JJt[i];
    frob = sqrt(frob);

    /* Estimate reciprocal condition number via Cline-Moler */
    if (frob < 1e-15) return 1; /* Definitely singular */

    return (frob > condition_threshold) ? 1 : 0;
}

double robot_manipulability(const double *J, size_t n_dof) {
    if (!J || n_dof == 0) return 0.0;

    double JJt[36] = {0};
    size_t i, j, k;
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
            for (k = 0; k < n_dof; k++)
                JJt[i * 6 + j] += J[i * n_dof + k] * J[j * n_dof + k];

    /* det(JJt) for 6x6 via Laplace expansion (top-left corner) */
    /* Simplified: use product of first 3 diagonal elements as
     * approximation (full expansion is 720 terms) */
    /* Better: use determinant of decomposed JJt */
    /* For now use a geometric mean of eigenvalues estimate */
    double trace = JJt[0] + JJt[7] + JJt[14] + JJt[21] + JJt[28] + JJt[35];
    return sqrt(fabs(trace / 6.0));
}

/* =================================================================
 * L2: Reachability check
 * ================================================================= */

int robot_is_reachable(const robot_model_t *model, const pose3d_t *target,
                       double arm_length_hint) {
    if (!model || !target) return 0;

    /* Compute total arm length */
    double total_length = 0.0;
    size_t i;
    for (i = 0; i < model->n_dof; i++) {
        double a = model->dh_params[i].a;
        double d = model->dh_params[i].d;
        total_length += sqrt(a * a + d * d);
    }
    if (arm_length_hint > 0.0) total_length = arm_length_hint;

    /* Check distance from base to target */
    double dist = vec3_norm(&target->position);
    return (dist <= total_length * 1.05) ? 1 : 0;
}

/* =================================================================
 * L5: Helper function declared in robot_types.h but implemented here
 * ================================================================= */
const char* robot_error_string(robot_error_t err);
/* Already implemented in robot_core.c */

/* Forward declarations of functions from robot_core.c */
extern int robot_clamp_to_limits(const robot_model_t *model, double *q);
