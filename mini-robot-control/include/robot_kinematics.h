#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H
#include "robot_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1-L4: Robot Kinematics -- Forward, Inverse, Jacobian
 *
 * FK: x = f(q), f: Q subset R^n -> SE(3)   (unique, continuous)
 * IK: q = f^{-1}(x)   (non-unique, nonlinear, may not exist)
 *
 * Differential kinematics:  x_dot = J(q)*q_dot
 *                           q_dot = J^+(q)*x_dot  (pseudoinverse)
 *
 * Geometric Jacobian J in R^{6xn}:
 *   Revolute:  J_i = [ z_{i-1} x (p_n - p_{i-1}) ; z_{i-1} ]
 *   Prismatic: J_i = [ z_{i-1} ; 0 ]
 *
 * References:
 *   Craig (2018) Ch. 3-5
 *   Siciliano et al. (2010) Ch. 2-3
 *   Denavit & Hartenberg (1955) ASME JAM 22:215-221
 *   Buss (2004) "Introduction to Inverse Kinematics", IEEE RA Magazine
 * ========================================================================= */

/* ---- Forward Kinematics ---- */
int robot_fk_compute(const robot_model_t *model, const double *q,
                     mat4_t *transforms);
int robot_fk_tool_pose(const robot_model_t *model, const mat4_t *transforms,
                       pose3d_t *tool_pose);

/* ---- Inverse Kinematics ---- */
int robot_ik_newton_raphson(const robot_model_t *model, const double *q_init,
                            const pose3d_t *target, double *q_out,
                            double tol, size_t max_iter, double lambda);
int robot_ik_gradient_descent(const robot_model_t *model, const double *q_init,
                              const pose3d_t *target, double *q_out,
                              double tol, size_t max_iter, double alpha);
int robot_ik_ccd(const robot_model_t *model, const double *q_init,
                 const pose3d_t *target, double *q_out,
                 double tol, size_t max_iter);

/* ---- Jacobian & Singularity ---- */
int robot_jacobian_geometric(const robot_model_t *model, const mat4_t *transforms,
                              double *J, int use_world_frame);
int robot_jacobian_analytic(const robot_model_t *model, const mat4_t *transforms,
                            const double *q, double *J);
int robot_singularity_detect(const double *J, size_t n_dof,
                             double condition_threshold);
double robot_manipulability(const double *J, size_t n_dof);
int robot_is_reachable(const robot_model_t *model, const pose3d_t *target,
                       double arm_length_hint);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_KINEMATICS_H */
