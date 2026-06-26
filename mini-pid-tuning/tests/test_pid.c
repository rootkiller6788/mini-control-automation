/**
 * @file test_pid.c
 * @brief Comprehensive PID Controller Test Suite
 *
 * Tests all core API functions with assert-based verification.
 * Covers: initialization, parameter setting, runtime update,
 * anti-windup, bumpless transfer, parameter conversion,
 * transfer function, frequency response, tuning methods,
 * step response simulation, and Routh-Hurwitz stability.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <float.h>

#include "pid_core.h"
#include "pid_tuning.h"
#include "pid_analysis.h"
#include "pid_advanced.h"
#include "pid_applications.h"
#include <stdlib.h>

#define ASSERT_NEAR(a, b, tol) \
    assert(fabs((a) - (b)) < (tol))

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(name) do { \
    tests_run++; \
    printf("  %s ... ", name); \
    fflush(stdout); \
} while(0)

#define TEST_PASS() do { \
    tests_passed++; \
    printf("PASS\n"); \
} while(0)

/*===========================================================================
 * pid_core.h tests
 *===========================================================================*/

static void test_pid_init_defaults(void) {
    RUN_TEST("pid_init defaults");
    PIDController pid;
    pid_init(&pid, PID_FORM_PARALLEL);
    assert(pid.params.Kp == 1.0);
    assert(pid.params.Ki == 0.0);
    assert(pid.params.Kd == 0.0);
    assert(pid.params.Ts == 0.01);
    assert(pid.params.N == 10.0);
    assert(pid.params.b == 1.0);
    assert(pid.params.c == 0.0);
    assert(pid.mode == PID_MODE_AUTO);
    assert(pid.dterm_mode == PID_DTERM_MEASUREMENT);
    assert(pid.aw_method == PID_AW_CLAMPING);
    assert(pid.state.initialized == true);
    TEST_PASS();
}

static void test_pid_init_params(void) {
    RUN_TEST("pid_init_params");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 2.0, 0.5, 0.1, 0.02);
    assert(pid.params.Kp == 2.0);
    assert(pid.params.Ki == 0.5);
    assert(pid.params.Kd == 0.1);
    assert(pid.params.Ts == 0.02);
    /* Standard form conversion: Ti = Kp/Ki = 2/0.5 = 4 */
    ASSERT_NEAR(pid.params.Ti, 4.0, 0.01);
    /* Td = Kd/Kp = 0.1/2 = 0.05 */
    ASSERT_NEAR(pid.params.Td, 0.05, 0.001);
    TEST_PASS();
}

static void test_pid_set_standard_gains(void) {
    RUN_TEST("pid_set_standard_gains");
    PIDController pid;
    pid_init(&pid, PID_FORM_PARALLEL);
    pid_set_standard_gains(&pid, 3.0, 10.0, 2.0);
    assert(pid.params.Kp == 3.0);
    /* Ki = Kp/Ti = 3/10 = 0.3 */
    ASSERT_NEAR(pid.params.Ki, 0.3, 0.01);
    /* Kd = Kp*Td = 3*2 = 6 */
    ASSERT_NEAR(pid.params.Kd, 6.0, 0.01);
    TEST_PASS();
}

static void test_pid_set_integral_limits(void) {
    RUN_TEST("pid_set_integral_limits");
    PIDController pid;
    pid_init(&pid, PID_FORM_PARALLEL);
    pid_set_integral_limits(&pid, -100.0, 100.0);
    assert(pid.int_min == -100.0);
    assert(pid.int_max == 100.0);
    TEST_PASS();
}

/*===========================================================================
 * pid_update tests
 *===========================================================================*/

static void test_pid_update_p_only(void) {
    RUN_TEST("pid_update P-only");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 2.0, 0.0, 0.0, 0.1);
    /* With P-only: u = Kp*(setpoint - measurement) = 2*(10-0) = 20 */
    double u = pid_update(&pid, 10.0, 0.0);
    ASSERT_NEAR(u, 20.0, 0.01);
    /* Next step: measurement goes to 5 */
    u = pid_update(&pid, 10.0, 5.0);
    ASSERT_NEAR(u, 10.0, 0.01);  /* 2*(10-5) = 10 */
    TEST_PASS();
}

static void test_pid_update_pi(void) {
    RUN_TEST("pid_update PI ? integral accumulation");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 1.0, 0.5, 0.0, 0.1);
    /* Step 1: error=10, I += 0.5*0.1*10 = 0.5, u=1*10+0.5=10.5 */
    double u = pid_update(&pid, 10.0, 0.0);
    ASSERT_NEAR(u, 10.5, 0.01);
    /* Step 2: error still 10, I += 0.5, total I=1.0, u=10+1=11 */
    u = pid_update(&pid, 10.0, 0.0);
    ASSERT_NEAR(u, 11.0, 0.01);
    /* Step 3: I += 0.5, total=1.5, u=11.5 */
    u = pid_update(&pid, 10.0, 0.0);
    ASSERT_NEAR(u, 11.5, 0.01);
    TEST_PASS();
}

static void test_pid_update_output_clamping(void) {
    RUN_TEST("pid_update output clamping");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 100.0, 0.0, 0.0, 0.1);
    pid_set_output_limits(&pid, -5.0, 5.0);
    /* error=10, Kp=100 -> unsaturated=1000, clamped to 5 */
    double u = pid_update(&pid, 10.0, 0.0);
    ASSERT_NEAR(u, 5.0, 0.01);
    /* Negative: error=-10, unsaturated=-1000, clamped to -5 */
    u = pid_update(&pid, -10.0, 0.0);
    ASSERT_NEAR(u, -5.0, 0.01);
    TEST_PASS();
}

static void test_pid_update_antiwindup_clamping(void) {
    RUN_TEST("pid_update anti-windup clamping");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 1.0, 2.0, 0.0, 0.1);
    pid_set_output_limits(&pid, -10.0, 10.0);
    pid_set_integral_limits(&pid, -5.0, 5.0);
    pid_set_antiwindup(&pid, PID_AW_CLAMPING, 1.0);
    /* Large error would cause integrator windup but it gets clamped */
    for (int i = 0; i < 100; i++) {
        pid_update(&pid, 100.0, 0.0);
    }
    /* Integral should be clamped at 5.0 */
    double integral = pid_get_integral(&pid);
    assert(integral <= 5.0001);
    assert(integral >= -5.0001);
    TEST_PASS();
}

static void test_pid_update_bumpless_manual(void) {
    RUN_TEST("pid_update bumpless manual -> auto");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 2.0, 1.0, 0.0, 0.1);
    /* Run a few steps */
    pid_update(&pid, 10.0, 0.0);
    pid_update(&pid, 10.0, 0.0);
    double old_output = pid_update(&pid, 10.0, 0.0);
    /* Switch to manual */
    pid_set_manual(&pid, 7.0);
    double u_manual = pid_update(&pid, 10.0, 0.0);
    ASSERT_NEAR(u_manual, 7.0, 0.01);
    /* Switch back to auto ? output should transition smoothly */
    pid_set_auto(&pid);
    double u_auto = pid_update(&pid, 10.0, 0.0);
    /* First auto output should be close to manual output (bumpless) */
    assert(fabs(u_auto - 7.0) < 10.0);  /* within reasonable range */
    TEST_PASS();
}

/*===========================================================================
 * pid_convert_params tests
 *===========================================================================*/

static void test_pid_convert_parallel_to_standard(void) {
    RUN_TEST("pid_convert parallel -> standard");
    PIDParams src = {.Kp = 2.0, .Ki = 0.5, .Kd = 0.2};
    PIDParams dst;
    pid_convert_params(&dst, &src, PID_FORM_PARALLEL, PID_FORM_STANDARD);
    /* Ti = Kp/Ki = 2/0.5 = 4 */
    ASSERT_NEAR(dst.Ti, 4.0, 0.01);
    /* Td = Kd/Kp = 0.2/2 = 0.1 */
    ASSERT_NEAR(dst.Td, 0.1, 0.001);
    assert(dst.form == PID_FORM_STANDARD);
    TEST_PASS();
}

static void test_pid_convert_standard_to_parallel(void) {
    RUN_TEST("pid_convert standard -> parallel");
    PIDParams src = {.Kp = 3.0, .Ti = 6.0, .Td = 1.5};
    PIDParams dst;
    pid_convert_params(&dst, &src, PID_FORM_STANDARD, PID_FORM_PARALLEL);
    /* Ki = Kp/Ti = 3/6 = 0.5 */
    ASSERT_NEAR(dst.Ki, 0.5, 0.01);
    /* Kd = Kp*Td = 3*1.5 = 4.5 */
    ASSERT_NEAR(dst.Kd, 4.5, 0.01);
    TEST_PASS();
}

/*===========================================================================
 * transfer function tests
 *===========================================================================*/

static void test_pid_transfer_function(void) {
    RUN_TEST("pid_get_transfer_function");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 2.0, 0.5, 0.1, 0.01);
    PIDTransferFunction tf;
    pid_get_transfer_function(&pid, &tf, PID_FORM_PARALLEL);
    /* G(s) = 0.5 + 2.0*s + 0.1*s^2 all over s
     * num: n0=0.5, n1=2.0, n2=0.1
     * den: d0=0, d1=1, d2=0 */
    ASSERT_NEAR(tf.n0, 0.5, 0.001);
    ASSERT_NEAR(tf.n1, 2.0, 0.001);
    ASSERT_NEAR(tf.n2, 0.1, 0.001);
    ASSERT_NEAR(tf.d1, 1.0, 0.001);
    TEST_PASS();
}

static void test_pid_frequency_response(void) {
    RUN_TEST("pid_frequency_response");
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 2.0, 0.5, 0.1, 0.01);
    PIDTransferFunction tf;
    pid_get_transfer_function(&pid, &tf, PID_FORM_PARALLEL);
    double mag, phase;
    /* At w=0 (DC): G(0) -> infinite (integrator) */
    pid_frequency_response(&tf, 0.1, &mag, &phase);
    assert(isfinite(mag) && mag > 0.0);
    /* Phase at low freq should be near -90 deg for integrator */
    assert(phase < 0.0);
    TEST_PASS();
}

/*===========================================================================
 * pid_tuning.h tests
 *===========================================================================*/

static void test_identify_fopdt_ideal(void) {
    RUN_TEST("pid_identify_fopdt ideal data");
    /* Generate ideal FOPDT step response:
     * y(t) = K * du * (1 - exp(-(t-L)/T)) for t >= L, else 0 */
    double K = 2.0, T = 5.0, L = 1.0, du = 1.0;
    size_t N = 1000;
    double *time = (double*)malloc(N * sizeof(double));
    double *output = (double*)malloc(N * sizeof(double));
    double dt = 0.05;
    for (size_t i = 0; i < N; i++) {
        time[i] = i * dt;
        double t = time[i];
        if (t < L) {
            output[i] = 0.0;
        } else {
            output[i] = K * du * (1.0 - exp(-(t - L) / T));
        }
    }
    StepResponseData data = {
        .time = time, .output = output, .N = N,
        .input_step = du, .initial_value = 0.0,
        .final_value = K * du
    };
    FOPDTModel model;
    int ret = pid_identify_fopdt(&data, &model);
    assert(ret == 0);
    /* Check that identified K, T, L are reasonable */
    ASSERT_NEAR(model.K, K, 0.5);
    assert(model.T > 0.0);
    assert(model.L >= 0.0);
    free(time); free(output);
    TEST_PASS();
}

static void test_tune_zn_open_loop(void) {
    RUN_TEST("pid_tune_zn_open_loop");
    FOPDTModel model = {.K = 2.0, .T = 10.0, .L = 2.0};
    PIDTuningResult result;
    int ret = pid_tune_zn_open_loop(&model, PID_FORM_STANDARD, 0.1, &result);
    assert(ret == 0);
    assert(result.valid);
    /* Kp = 1.2*T/(K*L) = 1.2*10/(2*2) = 1.2*10/4 = 3.0 */
    ASSERT_NEAR(result.params.Kp, 3.0, 0.1);
    /* Ti = 2.0*L = 4.0 */
    ASSERT_NEAR(result.params.Ti, 4.0, 0.1);
    TEST_PASS();
}

static void test_tune_cohen_coon(void) {
    RUN_TEST("pid_tune_cohen_coon");
    FOPDTModel model = {.K = 2.0, .T = 10.0, .L = 2.0};
    PIDTuningResult result;
    int ret = pid_tune_cohen_coon(&model, PID_FORM_STANDARD, 0.1, &result);
    assert(ret == 0);
    assert(result.valid);
    assert(result.params.Kp > 0.0);
    assert(result.params.Ti > 0.0);
    assert(result.params.Td > 0.0);
    TEST_PASS();
}

static void test_tune_amigo(void) {
    RUN_TEST("pid_tune_amigo");
    FOPDTModel model = {.K = 1.0, .T = 20.0, .L = 3.0};
    PIDTuningResult result;
    int ret = pid_tune_amigo(&model, PID_FORM_STANDARD, 0.1, &result);
    assert(ret == 0);
    assert(result.valid);
    assert(result.params.Kp > 0.0);
    TEST_PASS();
}

static void test_tune_imc(void) {
    RUN_TEST("pid_tune_imc");
    FOPDTModel model = {.K = 1.0, .T = 5.0, .L = 0.5};
    PIDTuningResult result;
    int ret = pid_tune_imc(&model, 5.0, PID_FORM_STANDARD, 0.1, &result);
    assert(ret == 0);
    assert(result.valid);
    /* IMC with lambda=T should give moderate Kp */
    assert(result.params.Kp > 0.0 && result.params.Kp < 20.0);
    TEST_PASS();
}

static void test_apply_tuning(void) {
    RUN_TEST("pid_apply_tuning");
    PIDController pid;
    pid_init(&pid, PID_FORM_STANDARD);
    PIDTuningResult result;
    FOPDTModel model = {.K = 1.0, .T = 10.0, .L = 1.0};
    pid_tune_zn_open_loop(&model, PID_FORM_STANDARD, 0.1, &result);
    pid_apply_tuning(&pid, &result);
    ASSERT_NEAR(pid.params.Kp, result.params.Kp, 0.001);
    ASSERT_NEAR(pid.params.Ti, result.params.Ti, 0.001);
    TEST_PASS();
}

/*===========================================================================
 * pid_analysis.h tests
 *===========================================================================*/

static void test_routh_hurwitz_stable(void) {
    RUN_TEST("Routh-Hurwitz: stable polynomial");
    /* s^3 + 6*s^2 + 11*s + 6 ? roots at -1, -2, -3 (stable) */
    Polynomial poly;
    poly.order = 3;
    poly.coeffs = (double*)malloc(4 * sizeof(double));
    poly.coeffs[0] = 6.0;
    poly.coeffs[1] = 11.0;
    poly.coeffs[2] = 6.0;
    poly.coeffs[3] = 1.0;
    RouthArray ra;
    int ret = pid_routh_construct(&poly, &ra);
    assert(ret == 0);
    int stable = pid_routh_is_stable(&ra);
    assert(stable == 1);
    pid_routh_free(&ra);
    free(poly.coeffs);
    TEST_PASS();
}

static void test_routh_hurwitz_unstable(void) {
    RUN_TEST("Routh-Hurwitz: unstable polynomial");
    /* s^3 + s^2 + s + 6 ? has RHP roots */
    Polynomial poly;
    poly.order = 3;
    poly.coeffs = (double*)malloc(4 * sizeof(double));
    poly.coeffs[0] = 6.0;
    poly.coeffs[1] = 1.0;
    poly.coeffs[2] = 1.0;
    poly.coeffs[3] = 1.0;
    RouthArray ra;
    pid_routh_construct(&poly, &ra);
    int stable = pid_routh_is_stable(&ra);
    /* This polynomial is unstable */
    assert(stable == 0);
    pid_routh_free(&ra);
    free(poly.coeffs);
    TEST_PASS();
}

static void test_lyapunov_system_matrix(void) {
    RUN_TEST("Lyapunov system matrix construction");
    double A[9];
    /* Process: y'' + 2*y' + 3*y = 1*u, PID: Kp=5, Ki=2, Kd=1 */
    pid_lyapunov_system_matrix(3.0, 2.0, 1.0, 5.0, 2.0, 1.0, A);
    /* Check A matrix structure:
     * A[0][0]=0, A[0][1]=1, A[0][2]=0
     * A[1][0]=-(a0+b0*Kp)=-(3+5)=-8
     * A[1][1]=-(a1+b0*Kd)=-(2+1)=-3
     * A[1][2]=b0*Ki=2
     * A[2][0]=-1, A[2][1]=0, A[2][2]=0
     */
    ASSERT_NEAR(A[0*3+0], 0.0, 0.001);
    ASSERT_NEAR(A[0*3+1], 1.0, 0.001);
    ASSERT_NEAR(A[1*3+0], -8.0, 0.001);
    ASSERT_NEAR(A[1*3+2], 2.0, 0.001);
    ASSERT_NEAR(A[2*3+0], -1.0, 0.001);
    TEST_PASS();
}

static void test_compute_stability_margins(void) {
    RUN_TEST("pid_compute_stability_margins");
    /* Create a PID + FOPDT loop and analyze */
    PIDController pid;
    pid_init_params(&pid, PID_FORM_PARALLEL, 1.0, 0.2, 0.05, 0.01);
    PIDTransferFunction tf;
    pid_get_transfer_function(&pid, &tf, PID_FORM_PARALLEL);
    FOPDTModel model = {.K = 1.0, .T = 10.0, .L = 1.0};
    FrequencyAnalysis analysis;
    int ret = pid_loop_frequency_analysis(&tf, &model, 0.01, 100.0, 500, &analysis);
    assert(ret == 0);
    pid_compute_stability_margins(&analysis);
    /* Should have some gain and phase margins */
    assert(isfinite(analysis.gain_margin));
    assert(isfinite(analysis.phase_margin));
    free(analysis.freq);
    free(analysis.magnitude);
    free(analysis.phase);
    TEST_PASS();
}

/*===========================================================================
 * pid_applications.h tests
 *===========================================================================*/

static void test_dc_motor_model(void) {
    RUN_TEST("dc_motor_init + to_fopdt");
    DCMotorModel motor;
    dc_motor_init(&motor, 0);  /* small hobby motor */
    assert(motor.R > 0.0);
    assert(motor.J > 0.0);
    FOPDTModel fopdt;
    int ret = dc_motor_to_fopdt(&motor, &fopdt, 12.0);
    assert(ret == 0);
    assert(fopdt.K > 0.0);
    assert(fopdt.T > 0.0);
    TEST_PASS();
}

static void test_thermal_model(void) {
    RUN_TEST("thermal_model_init + to_fopdt");
    ThermalModel tm;
    thermal_model_init(&tm, 1);  /* water bath */
    assert(tm.Cp == 4186.0);
    assert(tm.max_power > 0);
    FOPDTModel fopdt;
    thermal_to_fopdt(&tm, &fopdt);
    assert(fopdt.K > 0.0);
    assert(fopdt.T > 0.0);
    TEST_PASS();
}

static void test_pid_autotune(void) {
    RUN_TEST("pid_autotune selection");
    FOPDTModel model = {.K = 2.0, .T = 10.0, .L = 0.5};  /* L/T=0.05, easy */
    PIDTuningResult result;
    int ret = pid_autotune(&model, PID_FORM_STANDARD, 0.1, &result);
    assert(ret == 0);
    assert(result.valid);
    /* Should have selected IMC since L/T < 0.1 */
    assert(result.method == TUNE_IMC);
    TEST_PASS();
}

static void test_process_type_name(void) {
    RUN_TEST("pid_process_type_name");
    FOPDTModel model = {.K = 1, .T = 10, .L = 15};  /* L/T=1.5, severe dead time */
    char buf[128];
    const char *name = pid_process_type_name(&model, buf, sizeof(buf));
    assert(strstr(name, "Smith predictor") != NULL);
    TEST_PASS();
}

/*===========================================================================
 * pid_advanced.h tests
 *===========================================================================*/

static void test_cascade_pid(void) {
    RUN_TEST("cascade_pid_init + update");
    CascadePID cas;
    cascade_pid_init(&cas, PID_FORM_PARALLEL, PID_FORM_PARALLEL);
    assert(cas.cascade_active);
    cas.primary.params.Kp = 1.0;
    cas.secondary.params.Kp = 2.0;
    /* Primary measures 5, setpoint=10: error=5, u_primary=5
     * Secondary measures 2, setpoint=5: error=3, u_secondary=6 */
    double u = cascade_pid_update(&cas, 10.0, 5.0, 2.0);
    /* u_primary = 1*(10-5) = 5, u_secondary = 2*(5-2) = 6 */
    assert(u > 0.0);
    TEST_PASS();
}

static void test_feedforward_pid(void) {
    RUN_TEST("feedforward_pid_init + update");
    FeedforwardPID ffpid;
    feedforward_pid_init(&ffpid, PID_FORM_PARALLEL);
    ffpid.Kff_static = 0.5;
    ffpid.pid.params.Kp = 1.0;
    /* Feedback: error=10, u_fb=10
     * Feedforward: disturbance=5, u_ff=0.5*5=2.5
     * Total: 12.5 */
    double u = feedforward_pid_update(&ffpid, 10.0, 0.0, 5.0);
    ASSERT_NEAR(u, 12.5, 0.1);
    TEST_PASS();
}

static void test_gain_scheduled_pid(void) {
    RUN_TEST("gain_scheduled PID");
    GainScheduleEntry table[3] = {
        {0.0, 1.0, 0.1, 0.0},
        {5.0, 5.0, 0.5, 0.1},
        {10.0, 10.0, 1.0, 0.2}
    };
    GainScheduledPID gspid;
    gs_pid_init(&gspid, PID_FORM_PARALLEL, table, 3);
    /* At sched_var=5.0, Kp should be 5.0 */
    double u = gs_pid_update(&gspid, 10.0, 0.0, 5.0);
    assert(gspid.pid.params.Kp == 5.0);
    /* At sched_var=7.5 (midpoint), Kp should be 7.5 */
    u = gs_pid_update(&gspid, 10.0, 0.0, 7.5);
    ASSERT_NEAR(gspid.pid.params.Kp, 7.5, 0.01);
    TEST_PASS();
}

static void test_nonlinear_pid(void) {
    RUN_TEST("nonlinear PID");
    NonlinearPID nlpid;
    nonlinear_pid_init(&nlpid, PID_FORM_PARALLEL);
    nlpid.pid.params.Kp = 2.0;
    /* Small error (0.1): gain ~ 1.0 + 1.0*tanh(0.1) ~ 1.1 */
    double u = nonlinear_pid_update(&nlpid, 1.0, 0.9);
    assert(isfinite(u));
    /* Large error (5.0): gain ~ 1.0 + tanh(5) ~ 2.0 */
    u = nonlinear_pid_update(&nlpid, 10.0, 5.0);
    assert(isfinite(u));
    TEST_PASS();
}

static void test_event_based_pid(void) {
    RUN_TEST("event-based PID");
    EventBasedPID ebpid;
    eb_pid_init(&ebpid, PID_FORM_PARALLEL, 0.5, 2.0);
    ebpid.pid.params.Kp = 1.0;
    int updated;
    /* First call: should trigger (initial state) */
    double u = eb_pid_update(&ebpid, 10.0, 0.0, 0.1, &updated);
    /* May or may not trigger depending on initial delta - the first call
     * has last_sent_measurement=0, measurement=0, delta=0 < 0.5, so may not trigger */
    /* Call again with large delta */
    u = eb_pid_update(&ebpid, 10.0, 2.0, 0.1, &updated);
    /* delta = |2-0|=2 >= 0.5, should trigger */
    assert(ebpid.sample_count == 2);
    TEST_PASS();
}

/*===========================================================================
 * main
 *===========================================================================*/

int main(void) {
    printf("\n=== PID Controller Test Suite ===\n\n");

    /* pid_core tests */
    test_pid_init_defaults();
    test_pid_init_params();
    test_pid_set_standard_gains();
    test_pid_set_integral_limits();
    test_pid_update_p_only();
    test_pid_update_pi();
    test_pid_update_output_clamping();
    test_pid_update_antiwindup_clamping();
    test_pid_update_bumpless_manual();
    test_pid_convert_parallel_to_standard();
    test_pid_convert_standard_to_parallel();
    test_pid_transfer_function();
    test_pid_frequency_response();

    /* pid_tuning tests */
    test_identify_fopdt_ideal();
    test_tune_zn_open_loop();
    test_tune_cohen_coon();
    test_tune_amigo();
    test_tune_imc();
    test_apply_tuning();

    /* pid_analysis tests */
    test_routh_hurwitz_stable();
    test_routh_hurwitz_unstable();
    test_lyapunov_system_matrix();
    test_compute_stability_margins();

    /* pid_applications tests */
    test_dc_motor_model();
    test_thermal_model();
    test_pid_autotune();
    test_process_type_name();

    /* pid_advanced tests */
    test_cascade_pid();
    test_feedforward_pid();
    test_gain_scheduled_pid();
    test_nonlinear_pid();
    test_event_based_pid();

    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
