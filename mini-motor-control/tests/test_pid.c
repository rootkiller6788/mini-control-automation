/**
 * test_pid.c - Tests for PID controller with anti-windup
 * L5: Algorithms - validates PID computation correctness.
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "motor_control.h"
#include "motor_types.h"

#define TOL 1e-5f

static int tests_run = 0;
static int tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; if (cond) { tests_passed++; } \
    else { printf("FAIL %s\n", msg); } \
} while(0)

#define ASSERT_NEAR(a, b, msg) do { \
    tests_run++; \
    if (fabsf((a) - (b)) < TOL) { tests_passed++; } \
    else { printf("FAIL %s: got %.6f, expected %.6f\n", msg, (double)(a), (double)(b)); } \
} while(0)

static void test_pid_proportional_only(void)
{
    pid_state_t state;
    pid_init(&state, 10.0f);
    state.feedback = 5.0f;

    pid_gains_t gains = { .kp = 2.0f, .ki = 0.0f, .kd = 0.0f,
        .n_coeff = 0.0f, .out_min = -100.0f, .out_max = 100.0f,
        .integral_limit = 100.0f };

    float out = pid_update(&state, &gains, 0.001f);
    ASSERT_NEAR(out, 10.0f, "P-only: output = Kp*error = 2*5 = 10");
}

static void test_pid_integral_accumulation(void)
{
    pid_state_t state;
    pid_init(&state, 0.0f);
    state.feedback = 10.0f;

    pid_gains_t gains = { .kp = 1.0f, .ki = 100.0f, .kd = 0.0f,
        .n_coeff = 0.0f, .out_min = -100.0f, .out_max = 100.0f,
        .integral_limit = 100.0f };

    pid_update(&state, &gains, 0.001f);
    ASSERT_NEAR(state.integral, -1.0f, "I term after 1 step");

    pid_update(&state, &gains, 0.001f);
    ASSERT_NEAR(state.integral, -2.0f, "I term after 2 steps");
}

static void test_pid_integral_limit(void)
{
    pid_state_t state;
    pid_init(&state, 0.0f);
    state.feedback = 100.0f;

    pid_gains_t gains = { .kp = 0.0f, .ki = 1000.0f, .kd = 0.0f,
        .n_coeff = 0.0f, .out_min = -10.0f, .out_max = 10.0f,
        .integral_limit = 5.0f };

    for (int i = 0; i < 100; i++) {
        pid_update(&state, &gains, 0.001f);
    }
    CHECK(fabsf(state.integral) <= 5.0f + 0.01f, "Integral clamped to limit");
}

static void test_pid_output_saturation(void)
{
    pid_state_t state;
    pid_init(&state, 0.0f);
    state.feedback = 50.0f; /* error = -50 */

    pid_gains_t gains = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f,
        .n_coeff = 0.0f, .out_min = -5.0f, .out_max = 5.0f,
        .integral_limit = 100.0f };

    float out = pid_update(&state, &gains, 0.001f);
    ASSERT_NEAR(out, -5.0f, "Output saturated to lower limit");
    CHECK(state.saturated, "Saturation flag set");
}

static void test_pid_reset_integral(void)
{
    pid_state_t state;
    pid_init(&state, 0.0f);
    state.feedback = 10.0f;

    pid_gains_t gains = { .kp = 0.0f, .ki = 100.0f, .kd = 0.0f,
        .n_coeff = 0.0f, .out_min = -100.0f, .out_max = 100.0f,
        .integral_limit = 100.0f };

    pid_update(&state, &gains, 0.01f);
    CHECK(fabsf(state.integral) > 0.01f, "Integral accumulated");

    pid_reset_integral(&state);
    ASSERT_NEAR(state.integral, 0.0f, "Integral reset to zero");
}

static void test_pid_derivative(void)
{
    pid_state_t state;
    pid_init(&state, 0.0f);
    state.feedback = 0.0f;

    pid_gains_t gains = { .kp = 0.0f, .ki = 0.0f, .kd = 1.0f,
        .n_coeff = 1000.0f, .out_min = -100.0f, .out_max = 100.0f,
        .integral_limit = 100.0f };

    pid_update(&state, &gains, 0.001f);

    /* Change setpoint so error changes - derivative should respond */
    state.setpoint = 10.0f;
    state.feedback = 0.0f;
    pid_update(&state, &gains, 0.001f);

    CHECK(fabsf(state.output_d) > 0.0f, "Derivative non-zero when error changes");
}

static void test_hall_to_sector(void)
{
    /* H1 H2 H3 = a b c in function params */
    CHECK(hall_to_sector(0, 0, 1) == 1, "Hall 001 -> sector 1");
    CHECK(hall_to_sector(0, 1, 0) == 2, "Hall 010 -> sector 2");
    CHECK(hall_to_sector(0, 1, 1) == 3, "Hall 011 -> sector 3");
    CHECK(hall_to_sector(1, 0, 0) == 4, "Hall 100 -> sector 4");
    CHECK(hall_to_sector(1, 0, 1) == 5, "Hall 101 -> sector 5");
    CHECK(hall_to_sector(1, 1, 0) == 6, "Hall 110 -> sector 6");
    CHECK(hall_to_sector(0, 0, 0) == 0, "Hall 000 -> invalid (0)");
    CHECK(hall_to_sector(1, 1, 1) == 0, "Hall 111 -> invalid (0)");
}

static void test_bldc_commutation(void)
{
    uint8_t sw[6];
    CHECK(bldc_commutation_pattern(1, sw), "Valid sector 1");
    CHECK(sw[0] == 1 && sw[3] == 1, "Sector 1: A+ B-");

    CHECK(bldc_commutation_pattern(6, sw), "Valid sector 6");
    CHECK(sw[4] == 1 && sw[3] == 1, "Sector 6: C+ B-");

    CHECK(!bldc_commutation_pattern(0, sw), "Invalid sector 0");
    CHECK(!bldc_commutation_pattern(7, sw), "Invalid sector 7");
}

static void test_bldc_next_sector(void)
{
    CHECK(bldc_next_sector(1, MOTOR_DIR_CW) == 2, "CW: 1->2");
    CHECK(bldc_next_sector(6, MOTOR_DIR_CW) == 1, "CW: 6->1 (wrap)");
    CHECK(bldc_next_sector(1, MOTOR_DIR_CCW) == 6, "CCW: 1->6 (wrap)");
    CHECK(bldc_next_sector(3, MOTOR_DIR_CCW) == 2, "CCW: 3->2");
}

static void test_trapezoidal_ramp(void)
{
    float speed = 0.0f;
    float accel = 100.0f;
    float dt = 0.01f;

    speed_ramp_trapezoidal(100.0f, &speed, accel, dt);
    ASSERT_NEAR(speed, 1.0f, "Speed ramp step 1: 100*0.01=1");

    for (int i = 0; i < 99; i++) {
        speed_ramp_trapezoidal(100.0f, &speed, accel, dt);
    }
    ASSERT_NEAR(speed, 100.0f, "Speed reached target after 100 steps");
}

static void test_scurve_profile(void)
{
    float pos = scurve_position_profile(0.5f, 1.0f, 10.0f);
    ASSERT_NEAR(pos, 5.0f, "S-curve half time -> half position");

    pos = scurve_position_profile(0.0f, 1.0f, 10.0f);
    ASSERT_NEAR(pos, 0.0f, "S-curve t=0 -> pos=0");

    pos = scurve_position_profile(1.0f, 1.0f, 10.0f);
    ASSERT_NEAR(pos, 10.0f, "S-curve t=T -> pos=target");
}

static void test_feedforward(void)
{
    pmsm_params_t p = { .rs = 1.0f, .ld = 0.01f, .lq = 0.01f,
        .flux_linkage = 0.1f, .pole_pairs = 2.0f };

    float vd_ff, vq_ff;
    pmsm_decoupling_ff(0.0f, 2.0f, 100.0f, &p, &vd_ff, &vq_ff);

    ASSERT_NEAR(vd_ff, -2.0f, "Decoupling Vd_ff");
    ASSERT_NEAR(vq_ff, 10.0f, "Decoupling Vq_ff");
}

int main(void)
{
    printf("=== test_pid ===\n");

    test_pid_proportional_only();
    test_pid_integral_accumulation();
    test_pid_integral_limit();
    test_pid_output_saturation();
    test_pid_reset_integral();
    test_pid_derivative();
    test_hall_to_sector();
    test_bldc_commutation();
    test_bldc_next_sector();
    test_trapezoidal_ramp();
    test_scurve_profile();
    test_feedforward();

    printf("Results: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
