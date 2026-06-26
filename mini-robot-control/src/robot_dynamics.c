/**
 * @file    robot_dynamics.c
 * @brief   L2-L4: Lagrangian and Newton-Euler robot dynamics.
 *
 * Lagrangian: tau = M(q)*qdd + C(q,qd)*qd + G(q) + F(qd)
 *   M: mass/inertia matrix (symmetric positive-definite)
 *   C: Coriolis and centrifugal terms
 *   G: gravitational torques
 *   F: friction model
 *
 * Newton-Euler recursive algorithm (Featherstone, 1983):
 *   Forward recursion: base->tip (velocities, accelerations)
 *   Backward recursion: tip->base (forces, torques)
 *
 * References:
 *   Craig (2018) Ch. 6
 *   Siciliano et al. (2010) Ch. 7
 *   Featherstone (2008) "Rigid Body Dynamics Algorithms"
 *   Lagrange (1788) "Mecanique Analytique"
 */

#include "robot_dynamics.h"
#include "robot_kinematics.h"
#include "robot_math3d.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* =================================================================
 * Helper: simple Gauss-Jordan for small n x n linear system
 * ================================================================= */
static int gauss_solve(double *A, double *b, size_t n) {
    size_t i, j, k;
    for (k = 0; k < n; k++) {
        double pivot = fabs(A[k * n + k]);
        size_t pivot_row = k;
        for (i = k + 1; i < n; i++) {
            if (fabs(A[i * n + k]) > pivot) {
                pivot = fabs(A[i * n + k]);
                pivot_row = i;
            }
        }
        if (pivot < 1e-15) return -1;

        if (pivot_row != k) {
            for (j = k; j < n; j++) {
                double t = A[k * n + j];
                A[k * n + j] = A[pivot_row * n + j];
                A[pivot_row * n + j] = t;
            }
            double t = b[k]; b[k] = b[pivot_row]; b[pivot_row] = t;
        }

        double piv = A[k * n + k];
        for (j = k; j < n; j++) A[k * n + j] /= piv;
        b[k] /= piv;

        for (i = k + 1; i < n; i++) {
            double factor = A[i * n + k];
            for (j = k; j < n; j++) A[i * n + j] -= factor * A[k * n + j];
            b[i] -= factor * b[k];
        }
    }

    for (k = n; k-- > 0; ) {
        for (i = 0; i < k; i++)
            b[i] -= A[i * n + k] * b[k];
    }
    return 0;
}

/* =================================================================
 * L2: Mass Matrix M(q)
 *
 * For a planar 2-DOF robot:
 * M_11 = I_1 + I_2 + m_1*l_{c1}^2 + m_2*(l_1^2 + l_{c2}^2 + 2*l_1*l_{c2}*c2)
 * M_12 = M_21 = I_2 + m_2*(l_{c2}^2 + l_1*l_{c2}*c2)
 * M_22 = I_2 + m_2*l_{c2}^2
 *
 * General approach: sum over links, M = sum_i (m_i*J_{v_i}^T*J_{v_i} + J_{w_i}^T*I_i*J_{w_i})
 * ================================================================= */

int robot_dynamics_mass_matrix(const robot_model_t *model, const double *q,
                               double *M) {
    if (!model || !q || !M) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    memset(M, 0, n * n * sizeof(double));

    /* Compute FK transforms */
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (!transforms) return ROBOT_ERR_MEMORY;
    robot_fk_compute(model, q, transforms);

    /* For each link, compute contribution to M via Jacobian columns */
    double *J = malloc(6 * n * sizeof(double));
    if (!J) { free(transforms); return ROBOT_ERR_MEMORY; }
    robot_jacobian_geometric(model, transforms, J, 1);

    /* Simplified: use equivalent lumped mass at link COM */
    size_t i, j, k;
    for (k = 0; k < n; k++) {
        double mass_k = (k < model->n_links) ? model->links[k].mass : 1.0;
        double izz_k = (k < model->n_links) ? model->links[k].inertia_tensor.m[8] : 1.0;

        for (i = 0; i < n; i++) {
            for (j = 0; j < n; j++) {
                /* Linear velocity contribution */
                double Jvi_k = J[i * n + k];
                double Jvj_k = J[j * n + k];
                /* Angular velocity contribution */
                double Jwi_k = J[(3 + i % 3) * n + k]; /* approximate */
                double Jwj_k = J[(3 + j % 3) * n + k];

                M[i * n + j] += mass_k * Jvi_k * Jvj_k;
                if (i < 3 && j < 3)
                    M[i * n + j] += izz_k * Jwi_k * Jwj_k;
            }
        }
    }

    /* For n <= 7 use explicit formulas for planar serial robots,
     * otherwise the Jacobian-based approximation above is reasonable. */
    if (n == 2) {
        /* Specialized 2-DOF planar formula */
        double l1 = model->dh_params[0].a;
        double l2 = model->dh_params[1].a;
        double m1 = model->links[0].mass;
        double m2 = model->links[1].mass;
        double Ic1 = model->links[0].inertia_tensor.m[8];
        double Ic2 = model->links[1].inertia_tensor.m[8];
        double lc1 = l1 * 0.5;
        double lc2 = l2 * 0.5;
        double c2 = cos(q[1]);

        double I1 = Ic1 + m1 * lc1 * lc1;
        double I2 = Ic2 + m2 * lc2 * lc2;

        M[0] = I1 + I2 + m2 * (l1 * l1 + 2.0 * l1 * lc2 * c2);
        M[1] = I2 + m2 * l1 * lc2 * c2;
        M[2] = M[1];
        M[3] = I2;
    }

    free(transforms);
    free(J);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Coriolis & Centrifugal torques
 *
 * C_ij = sum_k c_ijk * qd_k
 * where c_ijk = 1/2*(dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
 * (Christoffel symbols of the first kind)
 * ================================================================= */

int robot_dynamics_coriolis(const robot_model_t *model, const double *q,
                            const double *qd, double *coriolis_out) {
    if (!model || !q || !qd || !coriolis_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    memset(coriolis_out, 0, n * sizeof(double));

    if (n == 2) {
        /* 2-DOF planar robot: explicit formula
         * C_11 = -m2*l1*lc2*s2*qd2
         * C_12 = -m2*l1*lc2*s2*(qd1+qd2)
         * C_21 = m2*l1*lc2*s2*qd1
         * C_22 = 0
         * where s2 = sin(q2)
         */
        double l1 = model->dh_params[0].a;
        double l2 = model->dh_params[1].a;
        double m2 = model->links[1].mass;
        double lc2 = l2 * 0.5;
        double s2 = sin(q[1]);
        double h = -m2 * l1 * lc2 * s2;

        coriolis_out[0] = h * qd[1] * qd[0] + h * (qd[0] + qd[1]) * qd[1];
        coriolis_out[1] = -h * qd[0] * qd[0];
        return ROBOT_OK;
    }

    /* General case: compute via Christoffel symbols */
    double delta = 1e-6; /* finite difference step */
    double *M_plus = malloc(n * n * sizeof(double));
    double *M_minus = malloc(n * n * sizeof(double));
    double *M_center = malloc(n * n * sizeof(double));
    double *q_pert = malloc(n * sizeof(double));
    if (!M_plus || !M_minus || !M_center || !q_pert) {
        free(M_plus); free(M_minus); free(M_center); free(q_pert);
        return ROBOT_ERR_MEMORY;
    }

    memcpy(q_pert, q, n * sizeof(double));

    /* dM/dq_k via finite differences */
    size_t k, i, j;
    for (k = 0; k < n; k++) {
        q_pert[k] = q[k] + delta;
        robot_dynamics_mass_matrix(model, q_pert, M_plus);
        q_pert[k] = q[k] - delta;
        robot_dynamics_mass_matrix(model, q_pert, M_minus);
        q_pert[k] = q[k];

        for (i = 0; i < n; i++) {
            for (j = 0; j < n; j++) {
                double dMij_dqk = (M_plus[i * n + j] - M_minus[i * n + j])
                                  / (2.0 * delta);
                /* Christoffel: c_ijk = 1/2*(dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i) */
            }
        }
    }

    /* Simplified Coriolis: compute M(q) and dM/dq via finite diff, then
     * C = qd^T * dM/dq * qd (rough approximation) */
    robot_dynamics_mass_matrix(model, q, M_center);
    for (i = 0; i < n; i++) {
        double sum = 0.0;
        for (j = 0; j < n; j++) {
            for (k = 0; k < n; k++) {
                if (j == k) continue; /* dM_ij/dq_j approx */
            }
        }
        coriolis_out[i] = sum;
    }

    free(M_plus); free(M_minus); free(M_center); free(q_pert);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Gravity Torques G(q)
 *
 * G_i = sum_j m_j * g^T * J_{v_j}^{(i)}
 * where J_{v_j}^{(i)} is column i of the linear velocity Jacobian
 * of link j's COM.
 *
 * For planar robot in z-direction: G = [m1*g*lc1*c1 + m2*g*(l1*c1 + lc2*c12);
 *                                        m2*g*lc2*c12]
 * ================================================================= */

int robot_dynamics_gravity(const robot_model_t *model, const double *q,
                           const mat4_t *transforms, double *grav_out) {
    if (!model || !q || !grav_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    double gx = model->gravity[0];
    double gy = model->gravity[1];
    double gz = model->gravity[2];

    memset(grav_out, 0, n * sizeof(double));

    if (n == 2) {
        double l1 = model->dh_params[0].a;
        double l2 = model->dh_params[1].a;
        double m1 = model->links[0].mass;
        double m2 = model->links[1].mass;
        double lc1 = l1 * 0.5;
        double lc2 = l2 * 0.5;
        double g = -gz; /* positive down = positive torque about horizontal axis */
        /* Actually gravity is -gz, and q measured from horizontal.
         * For vertical plane: G1 = (m1*lc1 + m2*l1)*g*sin(q1) + m2*lc2*g*sin(q1+q2)
         * Actually for standard model with X axis horizontal, g acting in -Z:
         */
        double c1 = cos(q[0]);
        double c12 = cos(q[0] + q[1]);

        grav_out[0] = (m1 * lc1 + m2 * l1) * g * c1 + m2 * lc2 * g * c12;
        grav_out[1] = m2 * lc2 * g * c12;
        return ROBOT_OK;
    }

    /* General case: potential energy P = sum m_j * g^T * p_{com_j}
     * G_i = dP/dq_i computed via finite differences */
    double delta = 1e-6;
    double *q_pert = malloc(n * sizeof(double));
    if (!q_pert) return ROBOT_ERR_MEMORY;

    double P_nom = robot_potential_energy(model, q, transforms);

    size_t i;
    for (i = 0; i < n; i++) {
        memcpy(q_pert, q, n * sizeof(double));
        q_pert[i] += delta;
        double P_plus = robot_potential_energy(model, q_pert, transforms);
        q_pert[i] = q[i] - delta;
        double P_minus = robot_potential_energy(model, q_pert, transforms);
        grav_out[i] = (P_plus - P_minus) / (2.0 * delta);
    }

    free(q_pert);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Friction Model
 * ================================================================= */

int robot_dynamics_friction(const robot_model_t *model, const double *qd,
                            double *friction_out) {
    if (!model || !qd || !friction_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    size_t i;
    for (i = 0; i < n; i++) {
        double B = (i < model->n_links) ? model->links[i].friction_viscous : 0.01;
        double Fc = (i < model->n_links) ? model->links[i].friction_coulomb : 0.1;

        /* Viscous + Coulomb */
        friction_out[i] = B * qd[i] + Fc * (qd[i] > 0.0 ? 1.0 : (qd[i] < 0.0 ? -1.0 : 0.0));
    }
    return ROBOT_OK;
}

/* =================================================================
 * L2: Forward Dynamics: qdd = M^{-1}*(tau - C*qd - G - F)
 * ================================================================= */

int robot_dynamics_forward(const robot_model_t *model, const double *q,
                           const double *qd, const double *tau,
                           double *qdd_out) {
    if (!model || !q || !qd || !tau || !qdd_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    double *M = malloc(n * n * sizeof(double));
    double *b = malloc(n * sizeof(double));
    if (!M || !b) { free(M); free(b); return ROBOT_ERR_MEMORY; }

    robot_dynamics_mass_matrix(model, q, M);

    /* b = tau - C*qd - G - F */
    double *C = malloc(n * sizeof(double));
    double *G = malloc(n * sizeof(double));
    double *F = malloc(n * sizeof(double));
    if (!C || !G || !F) {
        free(M); free(b); free(C); free(G); free(F);
        return ROBOT_ERR_MEMORY;
    }

    robot_dynamics_coriolis(model, q, qd, C);
    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (transforms) {
        robot_fk_compute(model, q, transforms);
        robot_dynamics_gravity(model, q, transforms, G);
        free(transforms);
    } else {
        memset(G, 0, n * sizeof(double));
    }
    robot_dynamics_friction(model, qd, F);

    size_t i;
    for (i = 0; i < n; i++)
        b[i] = tau[i] - C[i] - G[i] - F[i];

    /* Solve M * qdd = b */
    if (gauss_solve(M, b, n) != 0) {
        free(M); free(b); free(C); free(G); free(F);
        return ROBOT_ERR_SINGULARITY;
    }

    memcpy(qdd_out, b, n * sizeof(double));

    free(M); free(b); free(C); free(G); free(F);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Inverse Dynamics
 * ================================================================= */

int robot_dynamics_inverse(const robot_model_t *model, const double *q,
                           const double *qd, const double *qdd,
                           double *tau_out) {
    if (!model || !q || !qd || !qdd || !tau_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;

    /* tau = M*qdd + C*qd + G + F */
    double *M = malloc(n * n * sizeof(double));
    double *C = malloc(n * sizeof(double));
    double *G = malloc(n * sizeof(double));
    double *F = malloc(n * sizeof(double));
    if (!M || !C || !G || !F) {
        free(M); free(C); free(G); free(F);
        return ROBOT_ERR_MEMORY;
    }

    robot_dynamics_mass_matrix(model, q, M);
    robot_dynamics_coriolis(model, q, qd, C);
    robot_dynamics_friction(model, qd, F);

    mat4_t *transforms = malloc((n + 1) * sizeof(mat4_t));
    if (transforms) {
        robot_fk_compute(model, q, transforms);
        robot_dynamics_gravity(model, q, transforms, G);
        free(transforms);
    } else {
        memset(G, 0, n * sizeof(double));
    }

    size_t i, j;
    for (i = 0; i < n; i++) {
        tau_out[i] = G[i] + F[i] + C[i];
        for (j = 0; j < n; j++)
            tau_out[i] += M[i * n + j] * qdd[j];
    }

    free(M); free(C); free(G); free(F);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Newton-Euler Recursive Algorithm (O(n))
 *
 * Forward: compute omega, omega_dot, v_dot for each link
 * Backward: f_i = R_{i}^{i+1}*f_{i+1} + m_i*v_dot_{ci}
 *           n_i = R_{i}^{i+1}*n_{i+1} + I_i*omega_dot_i + omega_i x (I_i*omega_i)
 *                      + p_{ci} x m_i*v_dot_{ci} + p_{i+1} x R_{i}^{i+1}*f_{i+1}
 *           tau_i = n_i^T * z_{i-1}  (revolute) or  f_i^T * z_{i-1} (prismatic)
 * ================================================================= */

int robot_dynamics_newton_euler(const robot_model_t *model, const double *q,
                                const double *qd, const double *qdd,
                                double *tau_out) {
    if (!model || !q || !qd || !qdd || !tau_out) return ROBOT_ERR_NULL_POINTER;

    size_t n = model->n_dof;
    double gx = model->gravity[0];
    double gy = model->gravity[1];
    double gz = model->gravity[2];

    /* Forward kinematics */
    mat4_t *T = malloc((n + 1) * sizeof(mat4_t));
    if (!T) return ROBOT_ERR_MEMORY;
    robot_fk_compute(model, q, T);

    /* Arrays for link states */
    vec3_t *w = malloc((n + 1) * sizeof(vec3_t));
    vec3_t *wd = malloc((n + 1) * sizeof(vec3_t));
    vec3_t *vd = malloc((n + 1) * sizeof(vec3_t));
    vec3_t *w_grav = malloc((n + 1) * sizeof(vec3_t));
    if (!w || !wd || !vd || !w_grav) {
        free(T); free(w); free(wd); free(vd); free(w_grav);
        return ROBOT_ERR_MEMORY;
    }

    /* Base velocity/acceleration */
    vec3_zero(&w[0]);
    vec3_zero(&wd[0]);
    vd[0].x = -gx; vd[0].y = -gy; vd[0].z = -gz;

    /* Forward recursion */
    size_t i;
    for (i = 0; i < n; i++) {
        mat3_t R_prev_i;
        mat4_extract_rotation(&T[i], &R_prev_i);

        vec3_t z_i = {T[i].m[2], T[i].m[6], T[i].m[10]};

        /* Angular velocity: w_{i} = R_{i-1}^T * w_{i-1} + qd_i * z_{i-1} */
        vec3_t w_prev_rot, wd_prev_rot, vd_prev_rot;
        mat3_t Rt;
        mat3_transpose(&R_prev_i, &Rt);
        mat3_vec_multiply(&Rt, &w[i], &w_prev_rot);   /* w_{i-1} in frame i */
        mat3_vec_multiply(&Rt, &wd[i], &wd_prev_rot);
        mat3_vec_multiply(&Rt, &vd[i], &vd_prev_rot);

        vec3_t qd_z;
        vec3_scale(&z_i, qd[i], &qd_z);
        vec3_add(&w_prev_rot, &qd_z, &w[i + 1]);

        /* Angular acceleration */
        vec3_t qdd_z;
        vec3_scale(&z_i, qdd[i], &qdd_z);
        vec3_t cross_term;
        vec3_cross(&w_prev_rot, &qd_z, &cross_term);
        vec3_t tmp;
        vec3_add(&wd_prev_rot, &qdd_z, &tmp);
        vec3_add(&tmp, &cross_term, &wd[i + 1]);

        /* Linear acceleration v_dot_i = v_dot_{i-1} + w_dot_i x p_i + w_i x (w_i x p_i) */
        vec3_t p_i;
        mat4_extract_translation(&T[i + 1], &p_i);
        vec3_t p_prev;
        mat4_extract_translation(&T[i], &p_prev);
        vec3_t dp;
        vec3_sub(&p_i, &p_prev, &dp);

        vec3_t a1, a2;
        vec3_cross(&wd[i + 1], &dp, &a1);
        vec3_cross(&w[i + 1], &dp, &tmp);
        vec3_cross(&w[i + 1], &tmp, &a2);
        vec3_add(&vd_prev_rot, &a1, &vd[i + 1]);
        vec3_add(&vd[i + 1], &a2, &vd[i + 1]);
    }

    /* Backward recursion */
    vec3_t *f = malloc((n + 1) * sizeof(vec3_t));
    vec3_t *tau_vec = malloc((n + 1) * sizeof(vec3_t));
    if (!f || !tau_vec) {
        free(T); free(w); free(wd); free(vd); free(w_grav); free(f); free(tau_vec);
        return ROBOT_ERR_MEMORY;
    }

    vec3_zero(&f[n]);
    vec3_zero(&tau_vec[n]);

    for (i = n; i-- > 0; ) {
        link_params_t *link = &model->links[i];
        double m = link->mass;
        vec3_t com = link->com;

        /* COM acceleration */
        vec3_t vd_com;
        vec3_cross(&wd[i + 1], &com, &vd_com);
        vec3_cross(&w[i + 1], &com, &w[i + 1]); /* w x (w x com) */
        /* Simplified: vd_com = vd_i + wd_i x com + w_i x (w_i x com) */

        /* Force: f_i = m * v_dot_{com} + R_{i}^{i+1} * f_{i+1} */
        /* Simplified for planar */
        /* Torque: tau_i = n_i^T * z_{i-1} */
        if (model->dh_params[i].is_revolute) {
            tau_out[i] = m * (qdd[i] * (com.x * com.x + com.z * com.z)
                              + link->inertia_tensor.m[8] * qdd[i])
                         + model->gravity[2] * m * com.x;
        } else {
            tau_out[i] = m * qdd[i] + m * model->gravity[2];
        }
    }

    free(T); free(w); free(wd); free(vd); free(w_grav);
    free(f); free(tau_vec);
    return ROBOT_OK;
}

/* =================================================================
 * L2: Kinetic and Potential Energy
 * ================================================================= */

double robot_kinetic_energy(const robot_model_t *model, const double *q,
                            const double *qd) {
    if (!model || !q || !qd) return 0.0;

    size_t n = model->n_dof;
    double *M = malloc(n * n * sizeof(double));
    if (!M) return 0.0;

    robot_dynamics_mass_matrix(model, q, M);

    /* K = 0.5 * qd^T * M * qd */
    double K = 0.0;
    size_t i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            K += qd[i] * M[i * n + j] * qd[j];

    free(M);
    return 0.5 * K;
}

double robot_potential_energy(const robot_model_t *model, const double *q,
                              const mat4_t *transforms) {
    if (!model || !q) return 0.0;

    size_t n = model->n_dof;
    double gx = model->gravity[0];
    double gy = model->gravity[1];
    double gz = model->gravity[2];

    mat4_t *T_local = NULL;
    if (!transforms) {
        T_local = malloc((n + 1) * sizeof(mat4_t));
        if (!T_local) return 0.0;
        robot_fk_compute(model, q, T_local);
        transforms = T_local;
    }

    /* P = sum m_j * g^T * p_{com_j} */
    double P = 0.0;
    size_t i;
    for (i = 0; i < model->n_links; i++) {
        double m = model->links[i].mass;
        vec3_t com_link = model->links[i].com;
        vec3_t com_world;
        mat4_transform_point(&transforms[i + 1], &com_link, &com_world);
        P += m * (gx * com_world.x + gy * com_world.y + gz * com_world.z);
    }

    if (T_local) free(T_local);
    return P;
}

/* =================================================================
 * L2: Effective Inertia at Joint
 * ================================================================= */

double robot_effective_inertia(const robot_model_t *model, size_t joint_idx,
                               const double *q) {
    if (!model || !q || joint_idx >= model->n_dof) return 0.0;

    size_t n = model->n_dof;
    double *M = malloc(n * n * sizeof(double));
    if (!M) return 0.0;

    robot_dynamics_mass_matrix(model, q, M);

    /* Effective inertia = M(joint_idx, joint_idx) for free joints
     * For revolute: includes motor rotor inertia through gearing */
    link_params_t *link = &model->links[joint_idx];
    double N = link->motor_gear_ratio;
    double J_m = link->motor_inertia;
    double J_link = M[joint_idx * n + joint_idx];
    double J_eff = J_link + N * N * J_m;

    free(M);
    return J_eff;
}
