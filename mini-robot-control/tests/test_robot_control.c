/**
 * @file    test_robot_control.c
 * @brief   Assert-based tests for robot control module.
 * Tests: model lifecycle, FK, IK, Jacobian, dynamics, PID, CTC, trajectory.
 */
#include "robot_types.h"
#include "robot_math3d.h"
#include "robot_kinematics.h"
#include "robot_dynamics.h"
#include "robot_control.h"
#include "robot_trajectory.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; return; } while(0)
#define ASSERT_TRUE(cond, msg) do { if (!(cond)) { FAIL(msg); } } while(0)
#define ASSERT_NEAR(a, b, tol, msg) do { \
    if (fabs((a)-(b)) > (tol)) { \
        printf("FAIL: %s (%.6f vs %.6f)\n", msg, a, b); \
        tests_failed++; return; \
    } \
} while(0)

/* ---- Test 1: Vector Math ---- */
static void test_vec3_operations(void) {
    TEST("vec3 basic operations");
    vec3_t a, b, out;
    vec3_set(&a, 1.0, 2.0, 3.0);
    vec3_set(&b, 4.0, 5.0, 6.0);
    vec3_add(&a, &b, &out);
    ASSERT_NEAR(out.x, 5.0, 1e-10, "vec3_add x");
    ASSERT_NEAR(out.y, 7.0, 1e-10, "vec3_add y");

    vec3_cross(&a, &b, &out);
    ASSERT_NEAR(out.x, -3.0, 1e-10, "cross x");
    ASSERT_NEAR(out.y, 6.0, 1e-10, "cross y");
    ASSERT_NEAR(out.z, -3.0, 1e-10, "cross z");

    ASSERT_NEAR(vec3_dot(&a, &b), 32.0, 1e-10, "dot");
    ASSERT_NEAR(vec3_norm(&a), sqrt(14.0), 1e-10, "norm");
    PASS();
}

/* ---- Test 2: Rotation Matrices ---- */
static void test_mat3_rotation(void) {
    TEST("mat3 rotation matrices");
    mat3_t Rx, Ry, Rz;
    mat3_rot_x(0.5, &Rx);
    mat3_rot_y(0.3, &Ry);
    mat3_rot_z(0.7, &Rz);

    ASSERT_TRUE(mat3_is_rotation(&Rx, 1e-10), "Rx not rotation");
    ASSERT_TRUE(mat3_is_rotation(&Ry, 1e-10), "Ry not rotation");
    ASSERT_TRUE(mat3_is_rotation(&Rz, 1e-10), "Rz not rotation");

    /* Rz(pi/2) rotates (1,0,0) to (0,1,0) */
    mat3_rot_z(1.5707963267948966, &Rz);
    vec3_t v = {1.0, 0.0, 0.0}, r;
    mat3_vec_multiply(&Rz, &v, &r);
    ASSERT_NEAR(r.x, 0.0, 1e-10, "Rz x");
    ASSERT_NEAR(r.y, 1.0, 1e-10, "Rz y");
    PASS();
}

/* ---- Test 3: Quaternion Operations ---- */
static void test_quaternion(void) {
    TEST("quaternion operations");
    quat_t q, q_id, q_out;
    quat_identity(&q_id);
    vec3_t axis = {0.0, 0.0, 1.0};
    quat_from_axis_angle(&axis, 1.5707963267948966, &q);
    ASSERT_NEAR(quat_norm(&q), 1.0, 1e-10, "q norm");

    quat_multiply(&q, &q_id, &q_out);
    ASSERT_NEAR(q_out.w, q.w, 1e-10, "q*id");
    ASSERT_NEAR(q_out.z, q.z, 1e-10, "q*id z");

    /* Rotate vector */
    vec3_t v = {1.0, 0.0, 0.0}, vr;
    quat_rotate_vector(&q, &v, &vr);
    ASSERT_NEAR(vr.x, 0.0, 1e-10, "rot x");
    ASSERT_NEAR(vr.y, 1.0, 1e-10, "rot y");
    PASS();
}

/* ---- Test 4: Homogeneous Transforms ---- */
static void test_mat4_transform(void) {
    TEST("mat4 homogeneous transforms");
    mat4_t T, Tinv, Tout;
    mat4_identity(&T);
    T.m[3] = 1.0; T.m[7] = 2.0; T.m[11] = 3.0;

    mat4_inverse(&T, &Tinv);
    mat4_multiply(&T, &Tinv, &Tout);
    ASSERT_TRUE(mat3_is_identity((mat3_t*)&Tout.m[0], 1e-10), "T*Tinv not I");

    vec3_t p = {10.0, 20.0, 30.0}, pt;
    mat4_transform_point(&T, &p, &pt);
    ASSERT_NEAR(pt.x, 11.0, 1e-10, "transform x");
    ASSERT_NEAR(pt.y, 22.0, 1e-10, "transform y");
    PASS();
}

/* ---- Test 5: DH Transform ---- */
static void test_dh_transform(void) {
    TEST("DH transform standard");
    mat4_t T;
    mat4_dh_standard(0.0, 1.0, 0.0, 0.0, &T);
    ASSERT_NEAR(T.m[3], 1.0, 1e-10, "a=1 translation");
    ASSERT_NEAR(T.m[15], 1.0, 1e-10, "homogeneous bottom-right");

    mat4_dh_standard(1.5707963267948966, 0.0, 0.5, 0.0, &T);
    ASSERT_NEAR(T.m[7], -0.5, 1e-10, "d after alpha=90");
    PASS();
}

/* ---- Test 6: Forward Kinematics ---- */
static void test_fk_2dof(void) {
    TEST("forward kinematics 2-DOF planar");
    robot_model_t model;
    ASSERT_TRUE(robot_model_create_planar_2dof(&model) == ROBOT_OK, "model create");

    mat4_t *transforms = malloc((model.n_dof+1)*sizeof(mat4_t));
    double q[2] = {0.0, 0.0};
    ASSERT_TRUE(robot_fk_compute(&model, q, transforms) == ROBOT_OK, "fk compute");

    /* At q=(0,0), tool should be at (l1+l2, 0, 0) = (2.0, 0.0, 0.0) */
    vec3_t tool_pos;
    mat4_extract_translation(&transforms[2], &tool_pos);
    ASSERT_NEAR(tool_pos.x, 2.0, 1e-6, "tool x");
    ASSERT_NEAR(tool_pos.y, 0.0, 1e-6, "tool y y=0");

    /* At q=(pi/2, 0), tool should be at (0, l1+l2, 0) */
    q[0] = 1.5707963267948966; q[1] = 0.0;
    robot_fk_compute(&model, q, transforms);
    mat4_extract_translation(&transforms[2], &tool_pos);
    ASSERT_NEAR(tool_pos.x, 0.0, 1e-6, "tool x at q0=90");
    ASSERT_NEAR(tool_pos.y, 2.0, 1e-6, "tool y at q0=90");

    free(transforms);
    robot_model_free(&model);
    PASS();
}

/* ---- Test 7: Jacobian ---- */
static void test_jacobian(void) {
    TEST("geometric Jacobian");
    robot_model_t model;
    robot_model_create_planar_2dof(&model);

    mat4_t *transforms = malloc((model.n_dof+1)*sizeof(mat4_t));
    double q[2] = {0.5, 0.3};
    robot_fk_compute(&model, q, transforms);

    double *J = malloc(6 * model.n_dof * sizeof(double));
    ASSERT_TRUE(robot_jacobian_geometric(&model, transforms, J, 1) == ROBOT_OK, "jacobian");

    /* Jacobian should be full rank */
    double mu = robot_manipulability(J, model.n_dof);
    ASSERT_TRUE(mu > 0.0, "manipulability zero");

    free(J); free(transforms);
    robot_model_free(&model);
    PASS();
}

/* ---- Test 8: Inverse Kinematics ---- */
static void test_ik_newton(void) {
    TEST("IK Newton-Raphson");
    robot_model_t model;
    robot_model_create_planar_2dof(&model);

    double q_init[2] = {0.1, 0.1};
    pose3d_t target;
    pose_identity(&target);
    target.position.x = 1.0;
    target.position.y = 0.5;
    target.position.z = 0.0;

    double q_out[2];
    int ret = robot_ik_newton_raphson(&model, q_init, &target, q_out,
                                       1e-4, 100, 0.01);
    ASSERT_TRUE(ret == ROBOT_OK, "ik solution not found");

    /* Verify FK of solution matches target */
    mat4_t *transforms = malloc((model.n_dof+1)*sizeof(mat4_t));
    robot_fk_compute(&model, q_out, transforms);
    vec3_t pos;
    mat4_extract_translation(&transforms[2], &pos);
    ASSERT_NEAR(pos.x, target.position.x, 1e-3, "ik x match");
    ASSERT_NEAR(pos.y, target.position.y, 1e-3, "ik y match");

    free(transforms);
    robot_model_free(&model);
    PASS();
}

/* ---- Test 9: Dynamics ---- */
static void test_dynamics_mass_matrix(void) {
    TEST("mass matrix 2-DOF");
    robot_model_t model;
    robot_model_create_planar_2dof(&model);

    double *M = malloc(model.n_dof * model.n_dof * sizeof(double));
    double q[2] = {0.0, 0.0};
    ASSERT_TRUE(robot_dynamics_mass_matrix(&model, q, M) == ROBOT_OK, "mass matrix");

    ASSERT_TRUE(M[0] > 0.0, "M11 non-positive");
    ASSERT_TRUE(M[3] > 0.0, "M22 non-positive");

    /* Check symmetry */
    ASSERT_NEAR(M[1], M[2], 1e-10, "M not symmetric");

    free(M);
    robot_model_free(&model);
    PASS();
}

/* ---- Test 10: PID Control ---- */
static void test_pid_control(void) {
    TEST("PID controller");
    robot_pid_t pid;
    ASSERT_TRUE(robot_pid_init(&pid, 2, 10.0, 1.0, 2.0) == ROBOT_OK, "pid init");

    double q[2] = {0.0, 0.0}, q_des[2] = {1.0, 0.5};
    double qd[2] = {0.0, 0.0}, qd_des[2] = {0.0, 0.0};
    double tau[2];
    robot_pid_control(&pid, q, q_des, qd, qd_des, 0.01, tau);
    ASSERT_TRUE(tau[0] > 0.0, "PID output zero for positive error");

    robot_pid_free(&pid);
    PASS();
}

/* ---- Test 11: Trajectory ---- */
static void test_trajectory_cubic(void) {
    TEST("cubic trajectory");
    trajectory_t traj;
    ASSERT_TRUE(robot_trajectory_alloc(&traj, 2, 100) == ROBOT_OK, "traj alloc");

    double q0[2] = {0.0, 0.0}, qf[2] = {1.0, 0.5};
    double v0[2] = {0.0, 0.0}, vf[2] = {0.0, 0.0};
    ASSERT_TRUE(robot_traj_cubic(q0, qf, v0, vf, 1.0, 50, 2, &traj) == ROBOT_OK, "cubic");

    /* Start and end positions should match */
    ASSERT_NEAR(traj.points[0].q[0], 0.0, 1e-10, "start q0");
    ASSERT_NEAR(traj.points[49].q[0], 1.0, 1e-10, "end q0");

    robot_trajectory_free(&traj);
    PASS();
}

/* ---- Test 12: Gravity Torque ---- */
static void test_gravity(void) {
    TEST("gravity torque 2-DOF");
    robot_model_t model;
    robot_model_create_planar_2dof(&model);

    double q[2] = {1.5707963267948966, 0.0};
    mat4_t *T = malloc((model.n_dof+1)*sizeof(mat4_t));
    robot_fk_compute(&model, q, T);

    double *G = malloc(model.n_dof * sizeof(double));
    ASSERT_TRUE(robot_dynamics_gravity(&model, q, T, G) == ROBOT_OK, "gravity");

    /* At horizontal q=(90,0), gravity torque should be non-zero on joint 1 */
    ASSERT_TRUE(fabs(G[0]) > 0.01, "G1 zero at horizontal");

    free(G); free(T);
    robot_model_free(&model);
    PASS();
}

/* ---- Test 13: Singularity Detection ---- */
static void test_singularity(void) {
    TEST("singularity detection");
    robot_model_t model;
    robot_model_create_planar_2dof(&model);

    mat4_t *T = malloc((model.n_dof+1)*sizeof(mat4_t));
    double *J = malloc(6 * model.n_dof * sizeof(double));

    /* Fully extended (singular) */
    double q_sing[2] = {0.0, 0.0};
    robot_fk_compute(&model, q_sing, T);
    robot_jacobian_geometric(&model, T, J, 1);
    double mu_sing = robot_manipulability(J, model.n_dof);
    ASSERT_TRUE(mu_sing > 0.0, "manipulability at extended should be positive");

    free(T); free(J);
    robot_model_free(&model);
    PASS();
}

int main(void) {
    printf("=== mini-robot-control Test Suite ===\n\n");

    test_vec3_operations();
    test_mat3_rotation();
    test_quaternion();
    test_mat4_transform();
    test_dh_transform();
    test_fk_2dof();
    test_jacobian();
    test_ik_newton();
    test_dynamics_mass_matrix();
    test_pid_control();
    test_trajectory_cubic();
    test_gravity();
    test_singularity();

    printf("\n=== Results: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}