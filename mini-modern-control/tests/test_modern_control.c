#include "modern_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

static int tests_run = 0;
static int tests_passed = 0;
#define TEST(name) do { tests_run++; printf("  TEST %s... ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASSED\n"); } while(0)
#define FAIL(msg) do { printf("FAILED: %s\n", msg); } while(0)
#define ASSERT_EQ(a,b) do { if((a)!=(b)){FAIL("assertion");return;} } while(0)
#define ASSERT_NEAR(a,b,tol) do { if(fabs((a)-(b))>(tol)){FAIL("not near");return;} } while(0)

static void test_matrix_alloc_free(void)
{
    TEST("matrix alloc/free");
    mc_matrix_t mat;
    ASSERT_EQ(mc_matrix_alloc(3, 3, &mat), 0);
    ASSERT_EQ(mat.rows, 3);
    ASSERT_EQ(mat.cols, 3);
    mc_matrix_free(&mat);
    ASSERT_EQ(mat.data, (void*)0);
    PASS();
}

static void test_matrix_ops(void)
{
    TEST("matrix operations");
    mc_matrix_t A, B, C;
    mc_matrix_alloc(2, 2, &A);
    mc_matrix_alloc(2, 2, &B);
    mc_matrix_alloc(2, 2, &C);

    mc_matrix_set(&A, 0, 0, 1.0); mc_matrix_set(&A, 0, 1, 2.0);
    mc_matrix_set(&A, 1, 0, 3.0); mc_matrix_set(&A, 1, 1, 4.0);

    mc_matrix_identity(&B);
    mc_matrix_mul(&A, &B, &C);
    ASSERT_NEAR(mc_matrix_get(&C, 0, 0), 1.0, 1e-10);
    ASSERT_NEAR(mc_matrix_get(&C, 0, 1), 2.0, 1e-10);
    ASSERT_NEAR(mc_matrix_get(&C, 1, 0), 3.0, 1e-10);
    ASSERT_NEAR(mc_matrix_get(&C, 1, 1), 4.0, 1e-10);

    mc_matrix_free(&A); mc_matrix_free(&B); mc_matrix_free(&C);
    PASS();
}

static void test_matrix_inverse(void)
{
    TEST("matrix inverse");
    mc_matrix_t A, A_inv, I, C;
    mc_matrix_alloc(2, 2, &A);
    mc_matrix_alloc(2, 2, &A_inv);
    mc_matrix_alloc(2, 2, &I);
    mc_matrix_alloc(2, 2, &C);

    mc_matrix_set(&A, 0, 0, 4.0); mc_matrix_set(&A, 0, 1, 7.0);
    mc_matrix_set(&A, 1, 0, 2.0); mc_matrix_set(&A, 1, 1, 6.0);

    ASSERT_EQ(mc_matrix_inverse(&A, &A_inv), 0);
    mc_matrix_mul(&A, &A_inv, &C);
    ASSERT_NEAR(mc_matrix_get(&C, 0, 0), 1.0, 1e-8);
    ASSERT_NEAR(mc_matrix_get(&C, 0, 1), 0.0, 1e-8);
    ASSERT_NEAR(mc_matrix_get(&C, 1, 0), 0.0, 1e-8);
    ASSERT_NEAR(mc_matrix_get(&C, 1, 1), 1.0, 1e-8);

    mc_matrix_free(&A); mc_matrix_free(&A_inv);
    mc_matrix_free(&I); mc_matrix_free(&C);
    PASS();
}

static void test_controllability(void)
{
    TEST("controllability");
    mc_ss_system_t sys;
    mc_ss_alloc(2, 1, 1, &sys);
    /* Double integrator: A=[0 1;0 0], B=[0;1] */
    mc_matrix_set(&sys.A, 0, 1, 1.0);
    mc_matrix_set(&sys.B, 1, 0, 1.0);
    mc_matrix_set(&sys.C, 0, 0, 1.0);

    mc_controllability_t ctrb;
    ASSERT_EQ(mc_controllability_analyze(&sys, &ctrb), 0);
    ASSERT_EQ(ctrb.status, MC_FULLY_CONTROLLABLE);
    ASSERT_EQ(ctrb.rank, 2);

    mc_controllability_free(&ctrb);
    mc_ss_free(&sys);
    PASS();
}

static void test_observability(void)
{
    TEST("observability");
    mc_ss_system_t sys;
    mc_ss_alloc(2, 1, 1, &sys);
    mc_matrix_set(&sys.A, 0, 1, 1.0);
    mc_matrix_set(&sys.B, 1, 0, 1.0);
    mc_matrix_set(&sys.C, 0, 0, 1.0);

    mc_observability_t obsv;
    ASSERT_EQ(mc_observability_analyze(&sys, &obsv), 0);
    ASSERT_EQ(obsv.status, MC_FULLY_OBSERVABLE);
    ASSERT_EQ(obsv.rank, 2);

    mc_observability_free(&obsv);
    mc_ss_free(&sys);
    PASS();
}

static void test_pole_placement(void)
{
    TEST("pole placement Ackermann");
    mc_ss_system_t sys;
    mc_ss_alloc(2, 1, 1, &sys);
    mc_matrix_set(&sys.A, 0, 1, 1.0);
    mc_matrix_set(&sys.B, 1, 0, 1.0);
    mc_matrix_set(&sys.C, 0, 0, 1.0);

    double poles[2] = {-2.0, -3.0};
    mc_state_feedback_t fb;
    ASSERT_EQ(mc_pole_place_ackermann(&sys, poles, &fb), 0);
    ASSERT_NEAR(fb.poles[0], -2.0, 1e-10);
    ASSERT_NEAR(fb.poles[1], -3.0, 1e-10);

    mc_state_feedback_free(&fb);
    mc_ss_free(&sys);
    PASS();
}

static void test_lqr_dc_motor(void)
{
    TEST("LQR DC motor");
    mc_dc_motor_params_t params = {1.0, 0.5, 0.01, 0.01, 0.01, 0.001, 1.0};
    double qw[3] = {10.0, 1.0, 0.1};
    double rw[1] = {0.1};
    mc_lqr_solution_t sol;
    int ret = mc_dc_motor_lqr(&params, qw, rw, NULL, &sol);
    printf("(ret=%d,conv=%d,stab=%d) ", ret, sol.convergence, sol.is_stabilizing);
    mc_lqr_solution_free(&sol);
    PASS();
}

static void test_kalman_filter(void)
{
    TEST("Kalman filter");
    mc_ss_discrete_t sys;
    mc_ss_discrete_alloc(2, 1, 1, &sys, 0.01);
    mc_matrix_set(&sys.A, 0, 0, 1.0); mc_matrix_set(&sys.A, 0, 1, 0.01);
    mc_matrix_set(&sys.A, 1, 0, 0.0); mc_matrix_set(&sys.A, 1, 1, 1.0);
    mc_matrix_set(&sys.B, 0, 0, 0.0);
    mc_matrix_set(&sys.C, 0, 0, 1.0);

    mc_matrix_t Q, R, P0;
    mc_vector_t x0, y_meas;
    mc_matrix_alloc(2, 2, &Q); mc_matrix_alloc(1, 1, &R);
    mc_matrix_alloc(2, 2, &P0);
    mc_vector_alloc(2, &x0); mc_vector_alloc(1, &y_meas);

    mc_matrix_set(&Q, 0, 0, 0.01); mc_matrix_set(&Q, 1, 1, 0.01);
    mc_matrix_set(&R, 0, 0, 0.1);
    mc_matrix_identity(&P0);
    mc_vector_zero(&x0);
    mc_vector_set(&y_meas, 0, 1.0);

    mc_kalman_filter_t kf;
    ASSERT_EQ(mc_kalman_filter_init(&sys, &Q, &R, &x0, &P0, &kf), 0);

    /* Predict step */
    mc_vector_t u;
    mc_vector_alloc(1, &u);
    mc_vector_zero(&u);
    ASSERT_EQ(mc_kalman_filter_predict(&kf, &u), 0);

    /* Update step */
    ASSERT_EQ(mc_kalman_filter_update(&kf, &y_meas), 0);

    mc_kalman_filter_free(&kf);
    mc_matrix_free(&Q); mc_matrix_free(&R); mc_matrix_free(&P0);
    mc_vector_free(&x0); mc_vector_free(&y_meas); mc_vector_free(&u);
    mc_ss_discrete_free(&sys);
    PASS();
}

static void test_lyapunov_stability(void)
{
    TEST("Lyapunov stability (Hurwitz check)");
    mc_matrix_t A;
    mc_matrix_alloc(2, 2, &A);
    mc_matrix_set(&A, 0, 0, -2.0); mc_matrix_set(&A, 0, 1, 0.0);
    mc_matrix_set(&A, 1, 0, 0.0); mc_matrix_set(&A, 1, 1, -3.0);
    int stable = mc_is_hurwitz(&A);
    ASSERT_EQ(stable, 1);
    mc_matrix_free(&A);
    PASS();
}

static void test_inverted_pendulum_build(void)
{
    TEST("inverted pendulum build");
    mc_inverted_pendulum_params_t p = {0.5, 0.2, 0.3, 9.81, 0.1, 0.0};
    mc_ss_system_t sys;
    mc_ss_alloc(4, 1, 2, &sys);
    ASSERT_EQ(mc_build_inverted_pendulum(&p, &sys), 0);
    ASSERT_NEAR(mc_matrix_get(&sys.A, 0, 1), 1.0, 1e-10);
    mc_ss_free(&sys);
    PASS();
}

static void test_c2d(void)
{
    TEST("c2d discretization");
    mc_ss_system_t csys;
    mc_ss_alloc(2, 1, 1, &csys);
    mc_matrix_set(&csys.A, 0, 1, 1.0);
    mc_matrix_set(&csys.B, 1, 0, 1.0);
    mc_matrix_set(&csys.C, 0, 0, 1.0);

    mc_ss_discrete_t dsys;
    mc_ss_discrete_alloc(2, 1, 1, &dsys, 0.01);
    ASSERT_EQ(mc_c2d(&csys, &dsys, 0.01, MC_DISC_ZOH), 0);

    mc_ss_free(&csys); mc_ss_discrete_free(&dsys);
    PASS();
}

int main(void)
{
    printf("=== mini-modern-control Test Suite ===\n\n");

    test_matrix_alloc_free();
    test_matrix_ops();
    test_matrix_inverse();
    test_controllability();
    test_observability();
    test_pole_placement();
    test_lqr_dc_motor();
    test_kalman_filter();
    test_lyapunov_stability();
    test_inverted_pendulum_build();
    test_c2d();

    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
