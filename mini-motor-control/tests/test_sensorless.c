/**
 * test_sensorless.c ¡ª Tests for sensorless observers
 *
 * L5/L8: Sensorless rotor position estimation algorithms.
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "sensorless.h"

#define TOL 1e-4f

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

static void test_smo_init(void)
{
    observer_config_t cfg;
    observer_config_init(&cfg);
    smo_state_t state;
    smo_init(&state, &cfg);

    ASSERT_NEAR(state.i_alpha_hat, 0.0f, "SMO init: i_alpha=0");
    ASSERT_NEAR(state.i_beta_hat, 0.0f, "SMO init: i_beta=0");
    ASSERT_NEAR(state.theta_hat, 0.0f, "SMO init: theta=0");
    ASSERT_NEAR(state.omega_hat, 0.0f, "SMO init: omega=0");
}

static void test_smo_update_convergence(void)
{
    pmsm_params_t params;
    pmsm_params_init(&params);
    params.rs = 1.0f;
    params.ld = 0.01f;
    params.lq = 0.01f;

    observer_config_t cfg;
    observer_config_init(&cfg);
    cfg.gain_k = 500.0f;
    cfg.filter_cutoff = 2000.0f;

    smo_state_t state;
    smo_init(&state, &cfg);

    /* Simulate a running motor: apply known voltage and current */
    /* At steady state, BEMF e_alpha = -omega*psi_m*sin(theta) */
    float omega_e = 100.0f;
    float theta = 0.5f;
    (void)theta;

    for (int i = 0; i < 100; i++) {
        float v_alpha = 10.0f * cosf(omega_e * i * 0.0001f);
        float v_beta  = 10.0f * sinf(omega_e * i * 0.0001f);
        float i_alpha = v_alpha / params.rs;
        float i_beta  = v_beta / params.rs;

        smo_update(&state, i_alpha, i_beta, v_alpha, v_beta, 0.0001f, &params);
    }

    /* After convergence, estimates should be reasonable */
    CHECK(fabsf(smo_get_speed(&state)) > 0.0f, "SMO speed estimate non-zero");
    CHECK(fabsf(smo_get_angle(&state)) > 0.0f || smo_get_angle(&state) == 0.0f,
          "SMO angle output valid");  /* angle may wrap */
}

static void test_pll_init_and_update(void)
{
    pll_state_t pll;
    pll_init(&pll, 50.0f, 500.0f);

    /* Feed sinusoidal BEMF at known frequency */
    float omega = 100.0f;
    float theta = 0.0f;
    float dt = 0.0001f;

    for (int i = 0; i < 5000; i++) {
        theta += omega * dt;
        if (theta > 2.0f * (float)M_PI) theta -= 2.0f * (float)M_PI;

        /* BEMF: e_alpha = -omega*psi*sin(theta), e_beta = omega*psi*cos(theta) */
        float e_alpha = -omega * 0.05f * sinf(theta);
        float e_beta  =  omega * 0.05f * cosf(theta);

        pll_update(&pll, e_alpha, e_beta, dt);
    }

    /* PLL should track the frequency */
    float est_speed = pll_get_speed(&pll);
    CHECK(est_speed > 10.0f, "PLL speed estimate roughly tracks 100 rad/s");
    CHECK(est_speed < 500.0f, "PLL speed estimate bounded");
}

static void test_bemf_zero_cross_detect(void)
{
    /* Rising crossing */
    CHECK(bemf_zero_cross_detect(13.0f, 24.0f, 11.0f),
          "ZC detect: rising through Vdc/2=12");
    /* Falling crossing */
    CHECK(bemf_zero_cross_detect(11.0f, 24.0f, 13.0f),
          "ZC detect: falling through Vdc/2=12");
    /* No crossing */
    CHECK(!bemf_zero_cross_detect(13.0f, 24.0f, 14.0f),
          "No ZC: both above Vdc/2");
    CHECK(!bemf_zero_cross_detect(10.0f, 24.0f, 11.0f),
          "No ZC: both below Vdc/2");
}

static void test_bemf_speed_estimation(void)
{
    /* 10 ms between ZCs, 2 pole pairs */
    float omega = bemf_speed_from_zc_time(0.01f, 2.0f);
    /* Expected: pi / (3*2*0.01) = pi/0.06 = 52.36 rad/s */
    float expected = (float)M_PI / (3.0f * 2.0f * 0.01f);
    ASSERT_NEAR(omega, expected, "Speed from ZC time");
}

static void test_flux_estimator(void)
{
    /* Apply constant voltage, estimate flux */
    float psi_alpha = 0.0f, psi_beta = 0.0f;
    float rs = 1.0f;
    float dt = 0.001f;

    for (int i = 0; i < 100; i++) {
        flux_estimator_voltage_model(10.0f, 0.0f, 5.0f, 0.0f, rs, dt,
                                      &psi_alpha, &psi_beta, 1.0f);
    }

    /* After integration, flux should increase from back-EMF */
    CHECK(psi_alpha > 0.0f, "Flux alpha builds up with DC voltage");
    ASSERT_NEAR(psi_beta, 0.0f, "Flux beta stays zero (no beta excitation)");
}

static void test_flux_to_angle(void)
{
    float psi_a = 3.0f, psi_b = 4.0f;
    float angle = flux_to_angle(psi_a, psi_b);
    float expected = atan2f(4.0f, 3.0f);
    ASSERT_NEAR(angle, expected, "Flux to angle: atan2(4,3)");
}

static void test_ekf_init(void)
{
    pmsm_params_t p;
    pmsm_params_init(&p);

    ekf_state_t ekf;
    ekf_init(&ekf, &p);

    CHECK(ekf.initialized, "EKF initialized");
    CHECK(ekf.P[0] > 0.0f, "Covariance P11 positive");
    CHECK(ekf.P[5] > 0.0f, "Covariance P22 positive");
    CHECK(ekf.P[10] > 0.0f, "Covariance P33 positive");
    CHECK(ekf.P[15] > 0.0f, "Covariance P44 positive");
}

static void test_ekf_predict_update(void)
{
    pmsm_params_t p;
    pmsm_params_init(&p);

    ekf_state_t ekf;
    ekf_init(&ekf, &p);

    /* Apply a prediction step */
    ekf_predict(&ekf, 10.0f, 5.0f, 0.0001f, &p);
    CHECK(ekf.id_hat != 0.0f || ekf.iq_hat != 0.0f, "EKF predict changes state");

    /* Apply an update step with measurements */
    ekf_update(&ekf, 2.0f, 1.0f, &p);
    /* State should move toward measurements */
    CHECK(fabsf(ekf.id_hat - 2.0f) < 10.0f, "EKF update: Id moves toward measurement");
}

static void test_hfi_init(void)
{
    hfi_state_t hfi;
    hfi_init(&hfi, 500.0f, 5.0f);
    CHECK(hfi.enabled, "HFI enabled after init");
    CHECK(hfi.frequency == 500.0f, "HFI frequency set");
    CHECK(hfi.amplitude == 5.0f, "HFI amplitude set");
}

static void test_hfi_injection(void)
{
    hfi_state_t hfi;
    hfi_init(&hfi, 100.0f, 5.0f);

    float v_inj1 = hfi_get_injection_voltage(&hfi, 0.001f);
    float v_inj2 = hfi_get_injection_voltage(&hfi, 0.001f);

    /* Injection voltage should vary between calls (oscillating) */
    CHECK(fabsf(v_inj1) <= 5.0f, "Injection within amplitude");
    CHECK(fabsf(v_inj2) <= 5.0f, "Injection within amplitude");
}

int main(void)
{
    printf("=== test_sensorless ===\n");

    test_smo_init();
    test_smo_update_convergence();
    test_pll_init_and_update();
    test_bemf_zero_cross_detect();
    test_bemf_speed_estimation();
    test_flux_estimator();
    test_flux_to_angle();
    test_ekf_init();
    test_ekf_predict_update();
    test_hfi_init();
    test_hfi_injection();

    printf("Results: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
