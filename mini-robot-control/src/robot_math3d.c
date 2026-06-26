/**
 * @file    robot_math3d.c
 * @brief   L3: 3D mathematical structures for robotics -- vec3, mat3,
 *          mat4 (SE(3)), quaternions, DH transforms, Jacobian utilities.
 *
 * Knowledge Coverage:
 *   L3 (Math Structures): SO(3), SE(3), quaternion algebra, homogeneous
 *                          transforms, Euler angles, twist/wrench, Jacobian
 *   L4 (Fundamental Laws): Chasles theorem (any rigid motion = screw),
 *                           Euler rotation theorem, DH convention
 *
 * References:
 *   Craig (2018) Ch. 2
 *   Siciliano et al. (2010) Ch. 2
 *   Shoemake (1985) "Animating Rotation with Quaternion Curves"
 *   Hamilton (1844) "On Quaternions"
 */

#include "robot_math3d.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* =================================================================
 * L3: vec3 operations
 * ================================================================= */

void vec3_zero(vec3_t *v) {
    if (!v) return;
    v->x = v->y = v->z = 0.0;
}

void vec3_set(vec3_t *v, double x, double y, double z) {
    if (!v) return;
    v->x = x; v->y = y; v->z = z;
}

void vec3_add(const vec3_t *a, const vec3_t *b, vec3_t *out) {
    if (!a || !b || !out) return;
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
}

void vec3_sub(const vec3_t *a, const vec3_t *b, vec3_t *out) {
    if (!a || !b || !out) return;
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}

void vec3_scale(const vec3_t *v, double s, vec3_t *out) {
    if (!v || !out) return;
    out->x = v->x * s;
    out->y = v->y * s;
    out->z = v->z * s;
}

double vec3_dot(const vec3_t *a, const vec3_t *b) {
    if (!a || !b) return 0.0;
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3_cross(const vec3_t *a, const vec3_t *b, vec3_t *out) {
    if (!a || !b || !out) return;
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

double vec3_norm(const vec3_t *v) {
    if (!v) return 0.0;
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double vec3_norm_sq(const vec3_t *v) {
    if (!v) return 0.0;
    return v->x * v->x + v->y * v->y + v->z * v->z;
}

int vec3_normalize(vec3_t *v) {
    if (!v) return -1;
    double n = vec3_norm(v);
    if (n < 1e-15) return -1;
    v->x /= n; v->y /= n; v->z /= n;
    return 0;
}

double vec3_distance(const vec3_t *a, const vec3_t *b) {
    if (!a || !b) return 0.0;
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dz = a->z - b->z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void vec3_lerp(const vec3_t *a, const vec3_t *b, double t, vec3_t *out) {
    if (!a || !b || !out) return;
    double t1 = 1.0 - t;
    out->x = t1 * a->x + t * b->x;
    out->y = t1 * a->y + t * b->y;
    out->z = t1 * a->z + t * b->z;
}

void vec3_negate(const vec3_t *v, vec3_t *out) {
    if (!v || !out) return;
    out->x = -v->x; out->y = -v->y; out->z = -v->z;
}

/* =================================================================
 * L3: mat3 operations -- SO(3) rotation matrices
 * ================================================================= */

void mat3_identity(mat3_t *R) {
    if (!R) return;
    memset(R->m, 0, sizeof(R->m));
    R->m[0] = R->m[4] = R->m[8] = 1.0;
}

void mat3_zero(mat3_t *R) {
    if (!R) return;
    memset(R->m, 0, sizeof(R->m));
}

void mat3_set_col(mat3_t *R, int col, double x, double y, double z) {
    if (!R || col < 0 || col > 2) return;
    R->m[col] = x;
    R->m[3 + col] = y;
    R->m[6 + col] = z;
}

void mat3_multiply(const mat3_t *A, const mat3_t *B, mat3_t *out) {
    if (!A || !B || !out) return;
    mat3_t tmp;
    size_t i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            double sum = 0.0;
            for (k = 0; k < 3; k++) {
                sum += A->m[i * 3 + k] * B->m[k * 3 + j];
            }
            tmp.m[i * 3 + j] = sum;
        }
    }
    memcpy(out, &tmp, sizeof(mat3_t));
}

void mat3_transpose(const mat3_t *R, mat3_t *out) {
    if (!R || !out) return;
    size_t i, j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            out->m[j * 3 + i] = R->m[i * 3 + j];
}

void mat3_vec_multiply(const mat3_t *R, const vec3_t *v, vec3_t *out) {
    if (!R || !v || !out) return;
    out->x = R->m[0] * v->x + R->m[1] * v->y + R->m[2] * v->z;
    out->y = R->m[3] * v->x + R->m[4] * v->y + R->m[5] * v->z;
    out->z = R->m[6] * v->x + R->m[7] * v->y + R->m[8] * v->z;
}

double mat3_determinant(const mat3_t *R) {
    if (!R) return 0.0;
    double a = R->m[0], b = R->m[1], c = R->m[2];
    double d = R->m[3], e = R->m[4], f = R->m[5];
    double g = R->m[6], h = R->m[7], i = R->m[8];
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

int mat3_inverse(const mat3_t *R, mat3_t *out) {
    if (!R || !out) return -1;
    double det = mat3_determinant(R);
    if (fabs(det) < 1e-15) return -1;

    double inv_det = 1.0 / det;
    double a = R->m[0], b = R->m[1], c = R->m[2];
    double d = R->m[3], e = R->m[4], f = R->m[5];
    double g = R->m[6], h = R->m[7], i = R->m[8];

    out->m[0] = (e * i - f * h) * inv_det;
    out->m[1] = (c * h - b * i) * inv_det;
    out->m[2] = (b * f - c * e) * inv_det;
    out->m[3] = (f * g - d * i) * inv_det;
    out->m[4] = (a * i - c * g) * inv_det;
    out->m[5] = (c * d - a * f) * inv_det;
    out->m[6] = (d * h - e * g) * inv_det;
    out->m[7] = (b * g - a * h) * inv_det;
    out->m[8] = (a * e - b * d) * inv_det;
    return 0;
}

int mat3_is_rotation(const mat3_t *R, double tol) {
    if (!R) return 0;
    /* Check R^T * R = I */
    mat3_t Rt, RtR;
    mat3_transpose(R, &Rt);
    mat3_multiply(&Rt, R, &RtR);
    /* Check diagonal near 1, off-diagonal near 0 */
    double diag_err = fabs(RtR.m[0] - 1.0) + fabs(RtR.m[4] - 1.0)
                    + fabs(RtR.m[8] - 1.0);
    double off_err = fabs(RtR.m[1]) + fabs(RtR.m[2]) + fabs(RtR.m[3])
                   + fabs(RtR.m[5]) + fabs(RtR.m[6]) + fabs(RtR.m[7]);
    return (diag_err + off_err) < tol && fabs(mat3_determinant(R) - 1.0) < tol;
}

int mat3_is_identity(const mat3_t *R, double tol) {
    if (!R) return 0;
    return fabs(R->m[0] - 1.0) < tol && fabs(R->m[1]) < tol
        && fabs(R->m[2]) < tol && fabs(R->m[3]) < tol
        && fabs(R->m[4] - 1.0) < tol && fabs(R->m[5]) < tol
        && fabs(R->m[6]) < tol && fabs(R->m[7]) < tol
        && fabs(R->m[8] - 1.0) < tol;
}

/* =================================================================
 * L3: Rotation matrix generators
 * ================================================================= */

void mat3_rot_x(double theta, mat3_t *R) {
    if (!R) return;
    double c = cos(theta), s = sin(theta);
    mat3_identity(R);
    R->m[4] = c;  R->m[5] = -s;
    R->m[7] = s;  R->m[8] = c;
}

void mat3_rot_y(double theta, mat3_t *R) {
    if (!R) return;
    double c = cos(theta), s = sin(theta);
    mat3_identity(R);
    R->m[0] = c;  R->m[2] = s;
    R->m[6] = -s; R->m[8] = c;
}

void mat3_rot_z(double theta, mat3_t *R) {
    if (!R) return;
    double c = cos(theta), s = sin(theta);
    mat3_identity(R);
    R->m[0] = c;  R->m[1] = -s;
    R->m[3] = s;  R->m[4] = c;
}

void mat3_rot_euler_zyx(double roll, double pitch, double yaw, mat3_t *R) {
    if (!R) return;
    /* R = R_z(yaw) * R_y(pitch) * R_x(roll) */
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    R->m[0] = cp * cy;
    R->m[1] = sr * sp * cy - cr * sy;
    R->m[2] = cr * sp * cy + sr * sy;
    R->m[3] = cp * sy;
    R->m[4] = sr * sp * sy + cr * cy;
    R->m[5] = cr * sp * sy - sr * cy;
    R->m[6] = -sp;
    R->m[7] = sr * cp;
    R->m[8] = cr * cp;
}

/* =================================================================
 * L3: mat4 operations -- SE(3) homogeneous transforms
 * ================================================================= */

void mat4_identity(mat4_t *T) {
    if (!T) return;
    memset(T->m, 0, sizeof(T->m));
    T->m[0] = T->m[5] = T->m[10] = T->m[15] = 1.0;
}

void mat4_zero(mat4_t *T) {
    if (!T) return;
    memset(T->m, 0, sizeof(T->m));
}

void mat4_multiply(const mat4_t *A, const mat4_t *B, mat4_t *out) {
    if (!A || !B || !out) return;
    mat4_t tmp;
    size_t i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            double sum = 0.0;
            for (k = 0; k < 4; k++)
                sum += A->m[i * 4 + k] * B->m[k * 4 + j];
            tmp.m[i * 4 + j] = sum;
        }
    }
    memcpy(out, &tmp, sizeof(mat4_t));
}

void mat4_inverse(const mat4_t *T, mat4_t *out) {
    if (!T || !out) return;
    /* For SE(3), T^{-1} = [R^T  -R^T*p; 0 0 0 1] */
    mat4_identity(out);
    /* Extract rotation (top-left 3x3) and transpose */
    out->m[0] = T->m[0]; out->m[1] = T->m[4]; out->m[2] = T->m[8];
    out->m[4] = T->m[1]; out->m[5] = T->m[5]; out->m[6] = T->m[9];
    out->m[8] = T->m[2]; out->m[9] = T->m[6]; out->m[10] = T->m[10];
    /* Translation: -R^T * p */
    double px = T->m[3], py = T->m[7], pz = T->m[11];
    out->m[3] = -(out->m[0] * px + out->m[1] * py + out->m[2] * pz);
    out->m[7] = -(out->m[4] * px + out->m[5] * py + out->m[6] * pz);
    out->m[11] = -(out->m[8] * px + out->m[9] * py + out->m[10] * pz);
}

void mat4_transform_point(const mat4_t *T, const vec3_t *p, vec3_t *out) {
    if (!T || !p || !out) return;
    double x = T->m[0] * p->x + T->m[1] * p->y + T->m[2] * p->z + T->m[3];
    double y = T->m[4] * p->x + T->m[5] * p->y + T->m[6] * p->z + T->m[7];
    double z = T->m[8] * p->x + T->m[9] * p->y + T->m[10] * p->z + T->m[11];
    out->x = x; out->y = y; out->z = z;
}

void mat4_transform_vector(const mat4_t *T, const vec3_t *v, vec3_t *out) {
    if (!T || !v || !out) return;
    /* Vectors transform with rotation only (no translation) */
    out->x = T->m[0] * v->x + T->m[1] * v->y + T->m[2] * v->z;
    out->y = T->m[4] * v->x + T->m[5] * v->y + T->m[6] * v->z;
    out->z = T->m[8] * v->x + T->m[9] * v->y + T->m[10] * v->z;
}

void mat4_from_translation(double x, double y, double z, mat4_t *T) {
    mat4_identity(T);
    T->m[3] = x; T->m[7] = y; T->m[11] = z;
}

void mat4_from_rotation(const mat3_t *R, mat4_t *T) {
    if (!R || !T) return;
    mat4_identity(T);
    T->m[0] = R->m[0]; T->m[1] = R->m[1]; T->m[2] = R->m[2];
    T->m[4] = R->m[3]; T->m[5] = R->m[4]; T->m[6] = R->m[5];
    T->m[8] = R->m[6]; T->m[9] = R->m[7]; T->m[10] = R->m[8];
}

void mat4_from_pose(const pose3d_t *pose, mat4_t *T) {
    if (!pose || !T) return;
    mat3_t R;
    quat_to_mat3(&pose->orientation, &R);
    mat4_from_rotation(&R, T);
    T->m[3] = pose->position.x;
    T->m[7] = pose->position.y;
    T->m[11] = pose->position.z;
}

void mat4_to_pose(const mat4_t *T, pose3d_t *pose) {
    if (!T || !pose) return;
    pose->position.x = T->m[3];
    pose->position.y = T->m[7];
    pose->position.z = T->m[11];
    mat3_t R;
    mat4_extract_rotation(T, &R);
    quat_from_mat3(&R, &pose->orientation);
}

void mat4_extract_rotation(const mat4_t *T, mat3_t *R) {
    if (!T || !R) return;
    R->m[0] = T->m[0]; R->m[1] = T->m[1]; R->m[2] = T->m[2];
    R->m[3] = T->m[4]; R->m[4] = T->m[5]; R->m[5] = T->m[6];
    R->m[6] = T->m[8]; R->m[7] = T->m[9]; R->m[8] = T->m[10];
}

void mat4_extract_translation(const mat4_t *T, vec3_t *p) {
    if (!T || !p) return;
    p->x = T->m[3]; p->y = T->m[7]; p->z = T->m[11];
}

/* =================================================================
 * L3/L4: DH transformation (Craig standard convention)
 *
 * T = Rot(x, alpha)*Trans(x, a)*Rot(z, theta)*Trans(z, d)
 *
 *     [ c_theta          -s_theta           0           a        ]
 * T = [ s_theta*c_alpha   c_theta*c_alpha  -s_alpha   -d*s_alpha]
 *     [ s_theta*s_alpha   c_theta*s_alpha   c_alpha    d*c_alpha ]
 *     [ 0                 0                 0          1         ]
 * ================================================================= */

void mat4_dh_standard(double alpha, double a, double d, double theta,
                      mat4_t *T) {
    if (!T) return;
    double ca = cos(alpha), sa = sin(alpha);
    double ct = cos(theta), st = sin(theta);

    mat4_identity(T);
    T->m[0] = ct;
    T->m[1] = -st;
    T->m[3] = a;
    T->m[4] = st * ca;
    T->m[5] = ct * ca;
    T->m[6] = -sa;
    T->m[7] = -d * sa;
    T->m[8] = st * sa;
    T->m[9] = ct * sa;
    T->m[10] = ca;
    T->m[11] = d * ca;
}

/* =================================================================
 * L3: Quaternion operations (Hamilton convention: q = w + xi + yj + zk)
 * ================================================================= */

void quat_identity(quat_t *q) {
    if (!q) return;
    q->w = 1.0; q->x = 0.0; q->y = 0.0; q->z = 0.0;
}

void quat_set(quat_t *q, double w, double x, double y, double z) {
    if (!q) return;
    q->w = w; q->x = x; q->y = y; q->z = z;
}

void quat_from_axis_angle(const vec3_t *axis, double angle, quat_t *q) {
    if (!axis || !q) return;
    double half = 0.5 * angle;
    double s = sin(half);
    double c = cos(half);
    vec3_t u = *axis;
    vec3_normalize(&u);
    q->w = c;
    q->x = u.x * s;
    q->y = u.y * s;
    q->z = u.z * s;
}

void quat_from_mat3(const mat3_t *R, quat_t *q) {
    if (!R || !q) return;
    /* Shepperd's method: select largest of trace, R(0,0), R(1,1), R(2,2) */
    double trace = R->m[0] + R->m[4] + R->m[8];
    double s, w, x, y, z;

    if (trace > 0.0) {
        s = 0.5 / sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (R->m[7] - R->m[5]) * s;
        y = (R->m[2] - R->m[6]) * s;
        z = (R->m[3] - R->m[1]) * s;
    } else if (R->m[0] > R->m[4] && R->m[0] > R->m[8]) {
        s = 2.0 * sqrt(1.0 + R->m[0] - R->m[4] - R->m[8]);
        w = (R->m[7] - R->m[5]) / s;
        x = 0.25 * s;
        y = (R->m[1] + R->m[3]) / s;
        z = (R->m[2] + R->m[6]) / s;
    } else if (R->m[4] > R->m[8]) {
        s = 2.0 * sqrt(1.0 + R->m[4] - R->m[0] - R->m[8]);
        w = (R->m[2] - R->m[6]) / s;
        x = (R->m[1] + R->m[3]) / s;
        y = 0.25 * s;
        z = (R->m[5] + R->m[7]) / s;
    } else {
        s = 2.0 * sqrt(1.0 + R->m[8] - R->m[0] - R->m[4]);
        w = (R->m[3] - R->m[1]) / s;
        x = (R->m[2] + R->m[6]) / s;
        y = (R->m[5] + R->m[7]) / s;
        z = 0.25 * s;
    }
    q->w = w; q->x = x; q->y = y; q->z = z;
}

void quat_to_mat3(const quat_t *q, mat3_t *R) {
    if (!q || !R) return;
    double w = q->w, x = q->x, y = q->y, z = q->z;
    double xx = x * x, yy = y * y, zz = z * z;
    double wx = w * x, wy = w * y, wz = w * z;
    double xy = x * y, xz = x * z, yz = y * z;

    R->m[0] = 1.0 - 2.0 * (yy + zz);
    R->m[1] = 2.0 * (xy - wz);
    R->m[2] = 2.0 * (xz + wy);
    R->m[3] = 2.0 * (xy + wz);
    R->m[4] = 1.0 - 2.0 * (xx + zz);
    R->m[5] = 2.0 * (yz - wx);
    R->m[6] = 2.0 * (xz - wy);
    R->m[7] = 2.0 * (yz + wx);
    R->m[8] = 1.0 - 2.0 * (xx + yy);
}

void quat_multiply(const quat_t *a, const quat_t *b, quat_t *out) {
    if (!a || !b || !out) return;
    /* q1*q2 = (w1*w2 - v1.v2, w1*v2 + w2*v1 + v1 x v2) */
    out->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    out->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    out->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    out->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

void quat_conjugate(const quat_t *q, quat_t *out) {
    if (!q || !out) return;
    out->w = q->w;
    out->x = -q->x; out->y = -q->y; out->z = -q->z;
}

double quat_norm(const quat_t *q) {
    if (!q) return 0.0;
    return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

int quat_normalize(quat_t *q) {
    if (!q) return -1;
    double n = quat_norm(q);
    if (n < 1e-15) return -1;
    q->w /= n; q->x /= n; q->y /= n; q->z /= n;
    return 0;
}

void quat_inverse(const quat_t *q, quat_t *out) {
    if (!q || !out) return;
    double n2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    quat_conjugate(q, out);
    if (n2 > 1e-15) {
        out->w /= n2; out->x /= n2; out->y /= n2; out->z /= n2;
    }
}

void quat_rotate_vector(const quat_t *q, const vec3_t *v, vec3_t *out) {
    if (!q || !v || !out) return;
    /* v_rot = q * v_q * q^{-1}, where v_q = (0, v) */
    quat_t vq = {0.0, v->x, v->y, v->z};
    quat_t q_inv, tmp, result;
    quat_inverse(q, &q_inv);
    quat_multiply(q, &vq, &tmp);
    quat_multiply(&tmp, &q_inv, &result);
    out->x = result.x; out->y = result.y; out->z = result.z;
}

/**
 * Spherical Linear Interpolation (SLERP) between two unit quaternions.
 *
 * SLERP(q0, q1; t) = (sin((1-t)*Omega)*q0 + sin(t*Omega)*q1) / sin(Omega)
 * where Omega = acos(q0.q1)
 *
 * (Shoemake, 1985)
 */
void quat_slerp(const quat_t *a, const quat_t *b, double t, quat_t *out) {
    if (!a || !b || !out) return;
    /* Compute cosine of half-angle */
    double dot = a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z;
    /* Clamp to [-1, 1] for numerical stability */
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    double omega = acos(dot);
    double sin_omega = sin(omega);

    if (fabs(sin_omega) < 1e-12) {
        /* Very close: use linear interpolation */
        out->w = (1.0 - t) * a->w + t * b->w;
        out->x = (1.0 - t) * a->x + t * b->x;
        out->y = (1.0 - t) * a->y + t * b->y;
        out->z = (1.0 - t) * a->z + t * b->z;
        quat_normalize(out);
        return;
    }

    double s0 = sin((1.0 - t) * omega) / sin_omega;
    double s1 = sin(t * omega) / sin_omega;
    out->w = s0 * a->w + s1 * b->w;
    out->x = s0 * a->x + s1 * b->x;
    out->y = s0 * a->y + s1 * b->y;
    out->z = s0 * a->z + s1 * b->z;
}

/* =================================================================
 * L3: Euler angle conversions (ZYX convention)
 * ================================================================= */

void mat3_to_euler_zyx(const mat3_t *R, euler_t *euler) {
    if (!R || !euler) return;
    /* Pitch: -asin(R[2,0]) */
    euler->pitch = atan2(-R->m[6], sqrt(R->m[0] * R->m[0] + R->m[3] * R->m[3]));
    if (cos(euler->pitch) > 1e-10) {
        euler->roll = atan2(R->m[7], R->m[8]);
        euler->yaw = atan2(R->m[3], R->m[0]);
    } else {
        /* Gimbal lock */
        euler->roll = 0.0;
        euler->yaw = atan2(-R->m[1], R->m[4]);
    }
}

void euler_to_mat3_zyx(const euler_t *euler, mat3_t *R) {
    mat3_rot_euler_zyx(euler->roll, euler->pitch, euler->yaw, R);
}

/* =================================================================
 * L3: Twist and Wrench operations
 * ================================================================= */

void twist_zero(twist_t *tw) {
    if (!tw) return;
    vec3_zero(&tw->v);
    vec3_zero(&tw->w);
}

void twist_from_vecs(const vec3_t *v, const vec3_t *w, twist_t *tw) {
    if (!v || !w || !tw) return;
    tw->v = *v;
    tw->w = *w;
}

void wrench_zero(wrench_t *wr) {
    if (!wr) return;
    vec3_zero(&wr->force);
    vec3_zero(&wr->torque);
}

void wrench_sensor_transform(const mat4_t *T, const wrench_t *wr,
                             wrench_t *out) {
    if (!T || !wr || !out) return;
    /* Rotate force and torque using the rotation part of T */
    mat3_t R;
    mat4_extract_rotation(T, &R);
    mat3_vec_multiply(&R, &wr->force, &out->force);
    mat3_vec_multiply(&R, &wr->torque, &out->torque);
}

/* =================================================================
 * L3: Pose operations
 * ================================================================= */

void pose_identity(pose3d_t *pose) {
    if (!pose) return;
    vec3_zero(&pose->position);
    quat_identity(&pose->orientation);
}

void pose_multiply(const pose3d_t *a, const pose3d_t *b, pose3d_t *out) {
    if (!a || !b || !out) return;
    /* Compose: T_a_b = T_a * T_b */
    mat4_t Ta, Tb, Tout;
    mat4_from_pose(a, &Ta);
    mat4_from_pose(b, &Tb);
    mat4_multiply(&Ta, &Tb, &Tout);
    mat4_to_pose(&Tout, out);
}

void pose_inverse(const pose3d_t *pose, pose3d_t *out) {
    if (!pose || !out) return;
    /* Inverse pose: T^{-1} = [R^T  -R^T*p] */
    mat3_t R, Rt;
    quat_to_mat3(&pose->orientation, &R);
    mat3_transpose(&R, &Rt);

    out->orientation.w = pose->orientation.w;
    out->orientation.x = -pose->orientation.x;
    out->orientation.y = -pose->orientation.y;
    out->orientation.z = -pose->orientation.z;

    vec3_t p;
    vec3_negate(&pose->position, &p);
    mat3_vec_multiply(&Rt, &p, &out->position);
}

void pose_transform_point(const pose3d_t *pose, const vec3_t *p,
                          vec3_t *out) {
    if (!pose || !p || !out) return;
    mat4_t T;
    mat4_from_pose(pose, &T);
    mat4_transform_point(&T, p, out);
}

/* =================================================================
 * L3: SE(3) exponential map
 *
 * exp(xi * theta) = I + xi*sin(theta) + xi^2*(1-cos(theta))
 * where xi is the se(3) matrix form of the twist.
 *
 * Chasles theorem: every rigid body motion is equivalent to a
 * rotation about some axis plus a translation along that axis.
 * ================================================================= */

void mat4_exp_twist(const twist_t *tw, double theta, mat4_t *T) {
    if (!tw || !T) return;
    mat4_identity(T);

    double w_norm = vec3_norm(&tw->w);
    if (w_norm < 1e-15) {
        /* Pure translation */
        T->m[3] = tw->v.x * theta;
        T->m[7] = tw->v.y * theta;
        T->m[11] = tw->v.z * theta;
        return;
    }

    /* Normalize rotation axis */
    vec3_t w_hat = tw->w;
    vec3_normalize(&w_hat);

    /* Rotation part: Rodrigues formula */
    double c = cos(w_norm * theta);
    double s = sin(w_norm * theta);

    mat3_t R, K, K2;
    mat3_zero(&K);
    K.m[1] = -w_hat.z; K.m[2] = w_hat.y;
    K.m[3] = w_hat.z; K.m[5] = -w_hat.x;
    K.m[6] = -w_hat.y; K.m[7] = w_hat.x;

    mat3_multiply(&K, &K, &K2);

    /* R = I + sin(angle)*K + (1-cos(angle))*K^2 */
    size_t i;
    for (i = 0; i < 9; i++)
        R.m[i] = K.m[i] * s + K2.m[i] * (1.0 - c);
    R.m[0] += 1.0; R.m[4] += 1.0; R.m[8] += 1.0;

    mat4_from_rotation(&R, T);

    /* Translation: (I - R)*(w x v) + w*w^T*v*theta */
    vec3_t wcrossv;
    vec3_cross(&tw->w, &tw->v, &wcrossv);
    vec3_t R_wcrossv;
    mat3_vec_multiply(&R, &wcrossv, &R_wcrossv);

    vec3_t tmp;
    vec3_sub(&wcrossv, &R_wcrossv, &tmp);
    double wtv = vec3_dot(&tw->w, &tw->v);
    T->m[3] = tmp.x / (w_norm * w_norm) + w_hat.x * wtv * theta / w_norm;
    T->m[7] = tmp.y / (w_norm * w_norm) + w_hat.y * wtv * theta / w_norm;
    T->m[11] = tmp.z / (w_norm * w_norm) + w_hat.z * wtv * theta / w_norm;
}

/* =================================================================
 * L3: Jacobian utility functions
 * ================================================================= */

void jacobian_set_zero(double *J, size_t rows, size_t cols) {
    if (!J) return;
    memset(J, 0, rows * cols * sizeof(double));
}

void jacobian_multiply_vec(const double *J, const double *v,
                            size_t rows, size_t cols, double *out) {
    if (!J || !v || !out) return;
    size_t i, j;
    for (i = 0; i < rows; i++) {
        out[i] = 0.0;
        for (j = 0; j < cols; j++)
            out[i] += J[i * cols + j] * v[j];
    }
}

void jacobian_transpose_multiply_vec(const double *J, const double *v,
                                      size_t rows, size_t cols, double *out) {
    if (!J || !v || !out) return;
    size_t i, j;
    for (j = 0; j < cols; j++) {
        out[j] = 0.0;
        for (i = 0; i < rows; i++)
            out[j] += J[i * cols + j] * v[i];
    }
}
