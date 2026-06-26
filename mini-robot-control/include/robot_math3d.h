#ifndef ROBOT_MATH3D_H
#define ROBOT_MATH3D_H
#include "robot_types.h"
#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L3: Mathematical Structures for 3D Robot Geometry
 *
 * SO(3):  Rotation group { R in R^{3x3} | R^T R = I, det(R) = 1 }
 * SE(3):  Special Euclidean = { (R, p) | R in SO(3), p in R^3 }
 * Unit quaternions double-cover SO(3): S^3 -> SO(3)
 *
 * Key identities:
 *   R_{ZYX}(phi,theta,psi) = R_z(phi)*R_y(theta)*R_x(psi)
 *   q = cos(theta/2) + u*sin(theta/2)  (axis-angle)
 *   T = [R p; 0 1] in SE(3)
 *
 * References:
 *   Craig (2018) Ch. 2, Siciliano (2010) Ch. 2
 *   Hamilton (1844), "On Quaternions"
 *   Shoemake (1985), "Animating Rotation with Quaternion Curves"
 * ========================================================================= */

/* ---- L3: vec3 operations ---- */
void vec3_zero(vec3_t *v);
void vec3_set(vec3_t *v, double x, double y, double z);
void vec3_add(const vec3_t *a, const vec3_t *b, vec3_t *out);
void vec3_sub(const vec3_t *a, const vec3_t *b, vec3_t *out);
void vec3_scale(const vec3_t *v, double s, vec3_t *out);
double vec3_dot(const vec3_t *a, const vec3_t *b);
void vec3_cross(const vec3_t *a, const vec3_t *b, vec3_t *out);
double vec3_norm(const vec3_t *v);
double vec3_norm_sq(const vec3_t *v);
int vec3_normalize(vec3_t *v);
double vec3_distance(const vec3_t *a, const vec3_t *b);
void vec3_lerp(const vec3_t *a, const vec3_t *b, double t, vec3_t *out);
void vec3_negate(const vec3_t *v, vec3_t *out);

/* ---- L3: mat3 operations (SO(3) rotation matrices) ---- */
void mat3_identity(mat3_t *R);
void mat3_zero(mat3_t *R);
void mat3_set_col(mat3_t *R, int col, double x, double y, double z);
void mat3_multiply(const mat3_t *A, const mat3_t *B, mat3_t *out);
void mat3_transpose(const mat3_t *R, mat3_t *out);
void mat3_vec_multiply(const mat3_t *R, const vec3_t *v, vec3_t *out);
double mat3_determinant(const mat3_t *R);
int mat3_inverse(const mat3_t *R, mat3_t *out);
int mat3_is_rotation(const mat3_t *R, double tol);
int mat3_is_identity(const mat3_t *R, double tol);

/* ---- L3: Rotation matrix generators ---- */
void mat3_rot_x(double theta, mat3_t *R);
void mat3_rot_y(double theta, mat3_t *R);
void mat3_rot_z(double theta, mat3_t *R);
void mat3_rot_euler_zyx(double roll, double pitch, double yaw, mat3_t *R);

/* ---- L3: mat4 operations (SE(3) homogeneous transforms) ---- */
void mat4_identity(mat4_t *T);
void mat4_zero(mat4_t *T);
void mat4_multiply(const mat4_t *A, const mat4_t *B, mat4_t *out);
void mat4_inverse(const mat4_t *T, mat4_t *out);
void mat4_transform_point(const mat4_t *T, const vec3_t *p, vec3_t *out);
void mat4_transform_vector(const mat4_t *T, const vec3_t *v, vec3_t *out);
void mat4_from_translation(double x, double y, double z, mat4_t *T);
void mat4_from_rotation(const mat3_t *R, mat4_t *T);
void mat4_from_pose(const pose3d_t *pose, mat4_t *T);
void mat4_to_pose(const mat4_t *T, pose3d_t *pose);
void mat4_extract_rotation(const mat4_t *T, mat3_t *R);
void mat4_extract_translation(const mat4_t *T, vec3_t *p);

/* ---- L3: DH transformation (Craig convention) ---- */
void mat4_dh_standard(double alpha, double a, double d, double theta, mat4_t *T);

/* ---- L3: Quaternion operations ---- */
void quat_identity(quat_t *q);
void quat_set(quat_t *q, double w, double x, double y, double z);
void quat_from_axis_angle(const vec3_t *axis, double angle, quat_t *q);
void quat_from_mat3(const mat3_t *R, quat_t *q);
void quat_to_mat3(const quat_t *q, mat3_t *R);
void quat_multiply(const quat_t *a, const quat_t *b, quat_t *out);
void quat_conjugate(const quat_t *q, quat_t *out);
double quat_norm(const quat_t *q);
int quat_normalize(quat_t *q);
void quat_inverse(const quat_t *q, quat_t *out);
void quat_rotate_vector(const quat_t *q, const vec3_t *v, vec3_t *out);
void quat_slerp(const quat_t *a, const quat_t *b, double t, quat_t *out);

/* ---- L3: Euler angle conversions ---- */
void mat3_to_euler_zyx(const mat3_t *R, euler_t *euler);
void euler_to_mat3_zyx(const euler_t *euler, mat3_t *R);

/* ---- L3: Twist / Wrench operations ---- */
void twist_zero(twist_t *tw);
void twist_from_vecs(const vec3_t *v, const vec3_t *w, twist_t *tw);
void wrench_zero(wrench_t *wr);
void wrench_sensor_transform(const mat4_t *T, const wrench_t *wr, wrench_t *out);

/* ---- L3: Pose operations ---- */
void pose_identity(pose3d_t *pose);
void pose_multiply(const pose3d_t *a, const pose3d_t *b, pose3d_t *out);
void pose_inverse(const pose3d_t *pose, pose3d_t *out);
void pose_transform_point(const pose3d_t *pose, const vec3_t *p, vec3_t *out);

/* ---- L3: SE(3) exponential and logarithm ---- */
void mat4_exp_twist(const twist_t *tw, double theta, mat4_t *T);

/* ---- L3: Jacobian utilities ---- */
void jacobian_set_zero(double *J, size_t rows, size_t cols);
void jacobian_multiply_vec(const double *J, const double *v, size_t rows,
                            size_t cols, double *out);
void jacobian_transpose_multiply_vec(const double *J, const double *v,
                                      size_t rows, size_t cols, double *out);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_MATH3D_H */
