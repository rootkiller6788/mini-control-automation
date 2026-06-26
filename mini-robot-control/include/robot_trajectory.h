#ifndef ROBOT_TRAJECTORY_H
#define ROBOT_TRAJECTORY_H
#include "robot_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L5-L6: Trajectory Planning for Robot Motion
 *
 * Cubic:     q(t) = a0 + a1*t + a2*t^2 + a3*t^3
 * Quintic:   q(t) = a0 + a1*t + ... + a5*t^5
 * Trapezoidal: constant accel - constant vel - constant decel
 * S-curve:    trapezoidal acceleration (jerk-limited)
 * LSPB:       Linear Segment with Parabolic Blend
 *
 * References:
 *   Craig (2018) Ch. 7
 *   Siciliano et al. (2010) Ch. 4
 *   Biagiotti & Melchiorri (2008) "Trajectory Planning", Springer
 * ========================================================================= */

int robot_trajectory_alloc(trajectory_t *traj, size_t n_dof, size_t max_points);
void robot_trajectory_free(trajectory_t *traj);

int robot_traj_cubic(const double *q0, const double *qf,
                     const double *v0, const double *vf,
                     double T, size_t n_steps, size_t n_dof,
                     trajectory_t *traj);

int robot_traj_quintic_rest_to_rest(const double *q0, const double *qf,
                                    double T, size_t n_steps, size_t n_dof,
                                    trajectory_t *traj);

int robot_traj_quintic_general(const double *q0, const double *qf,
                               const double *v0, const double *vf,
                               const double *a0, const double *af,
                               double T, size_t n_steps, size_t n_dof,
                               trajectory_t *traj);

int robot_traj_trapezoidal(const double *q0, const double *qf,
                           double v_max, double a_max,
                           size_t n_steps, size_t n_dof,
                           trajectory_t *traj);

int robot_traj_s_curve(const double *q0, const double *qf,
                       double v_max, double a_max, double j_max,
                       size_t n_steps, size_t n_dof,
                       trajectory_t *traj);

int robot_traj_lspb(const double *via_points, size_t n_points, size_t n_dof,
                    double v_default, double a_blend,
                    size_t n_steps_per_segment, trajectory_t *traj);

int robot_traj_cubic_spline(const double *via_points, const double *times,
                            size_t n_points, size_t n_dof,
                            size_t n_steps_per_segment, trajectory_t *traj);

int robot_traj_cartesian_line(const pose3d_t *start_pose,
                              const pose3d_t *end_pose,
                              double v_max, double a_max,
                              double freq, size_t n_dof,
                              trajectory_t *traj);

int robot_traj_cartesian_circle(const pose3d_t *center_pose,
                                double radius, double start_angle,
                                double end_angle,
                                const vec3_t *plane_normal,
                                double angular_speed,
                                double freq, size_t n_dof,
                                trajectory_t *traj);

int robot_traj_evaluate(const trajectory_t *traj, double t,
                        trajectory_point_t *point);

int robot_traj_step(trajectory_t *traj, double dt,
                    double *q_ref, double *qd_ref, double *qdd_ref);

int robot_traj_concatenate(trajectory_t *dest,
                           const trajectory_t **segments,
                           size_t n_segments);

int robot_traj_time_optimal(const robot_model_t *model,
                            const double *q0, const double *qf,
                            size_t n_steps, trajectory_t *traj);

int robot_traj_cartesian_to_joint(const robot_model_t *model,
                                  const trajectory_t *cart_traj,
                                  const double *q_init,
                                  trajectory_t *joint_traj);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_TRAJECTORY_H */
