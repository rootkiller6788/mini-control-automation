/**
 * test_svpwm.c ¡ª Tests for SVPWM and PWM modulation
 *
 * L5: Algorithms ¡ª validates sector determination, dwell times, duty generation.
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "pwm_modulation.h"

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

static void test_svpwm_sector_angles(void)
{
    /* Test known angles in each sector */
    /* Sector 1: 0 to 60 deg. At 30 deg: Valpha>0, Vbeta>0 */
    float v_alpha = 0.866f;  /* cos(30¡ã) */
    float v_beta  = 0.5f;    /* sin(30¡ã) */
    uint8_t s = svpwm_determine_sector(v_alpha, v_beta);
    CHECK(s == 1, "30 deg -> sector 1");

    /* Sector 2: 60 to 120 deg. At 90 deg: Valpha=0, Vbeta=1 */
    s = svpwm_determine_sector(0.0f, 1.0f);
    CHECK(s == 2, "90 deg -> sector 2");

    /* Sector 3: 120 to 180 deg. At 150 deg: Valpha<0, Vbeta>0 */
    s = svpwm_determine_sector(-0.866f, 0.5f);
    CHECK(s == 3, "150 deg -> sector 3");

    /* Sector 4: 180 to 240 deg. At 210 deg: Valpha<0, Vbeta<0 */
    s = svpwm_determine_sector(-0.866f, -0.5f);
    CHECK(s == 4, "210 deg -> sector 4");

    /* Sector 5: 240 to 300 deg. At 270 deg: Valpha=0, Vbeta=-1 */
    s = svpwm_determine_sector(0.0f, -1.0f);
    CHECK(s == 5, "270 deg -> sector 5");

    /* Sector 6: 300 to 360 deg. At 330 deg: Valpha>0, Vbeta<0 */
    s = svpwm_determine_sector(0.866f, -0.5f);
    CHECK(s == 6, "330 deg -> sector 6");
}

static void test_svpwm_timing_linear(void)
{
    /* Test at half modulation index with known Vdc */
    float vdc = 100.0f;
    float ts = 0.0001f;  /* 100 us = 10 kHz */
    /* Vref = 30V at 30 deg in sector 1 */
    float v_alpha = 30.0f * cosf(30.0f * (float)M_PI / 180.0f);
    float v_beta  = 30.0f * sinf(30.0f * (float)M_PI / 180.0f);

    svpwm_timing_t timing;
    bool ok = svpwm_calculate_timing(v_alpha, v_beta, vdc, ts, &timing);
    CHECK(ok, "SVPWM timing calculation succeeded");
    CHECK(timing.sector == 1, "30 deg -> sector 1");

    /* T1 + T2 + T0 should equal 1 (normalized) */
    float sum = timing.t1 + timing.t2 + timing.t0;
    ASSERT_NEAR(sum, 1.0f, "T1 + T2 + T0 = 1");

    /* T1 and T2 should be positive */
    CHECK(timing.t1 >= 0.0f, "T1 >= 0");
    CHECK(timing.t2 >= 0.0f, "T2 >= 0");
}

static void test_svpwm_duty_generation(void)
{
    float vdc = 48.0f;
    float ts = 0.0001f;
    float v_alpha = 10.0f;  /* moderate Vref in sector 1 */
    float v_beta = 5.0f;

    pwm_duty_t duty;
    bool ok = svpwm_compute(v_alpha, v_beta, vdc, ts, &duty);
    CHECK(ok, "SVPWM duty generation succeeded");

    /* Duties should be in [0, 1] */
    CHECK(duty.duty_a >= 0.0f && duty.duty_a <= 1.0f, "duty_a in [0,1]");
    CHECK(duty.duty_b >= 0.0f && duty.duty_b <= 1.0f, "duty_b in [0,1]");
    CHECK(duty.duty_c >= 0.0f && duty.duty_c <= 1.0f, "duty_c in [0,1]");
}

static void test_svpwm_overmodulation(void)
{
    /* Request voltage that exceeds DC bus */
    float vdc = 48.0f;
    float ts = 0.0001f;
    float v_alpha = 30.0f;  /* > Vdc/sqrt(3) = 27.7 */
    float v_beta = 5.0f;

    pwm_duty_t duty;
    float mi;
    bool ok = svpwm_auto_overmodulation(v_alpha, v_beta, vdc, ts, &duty, &mi);
    CHECK(ok, "Overmodulation handled");
    CHECK(mi > 0.9f, "Modulation index high");

    /* Duties should still be in [0, 1] */
    CHECK(duty.duty_a >= 0.0f && duty.duty_a <= 1.0f, "duty_a clamped");
    CHECK(duty.duty_b >= 0.0f && duty.duty_b <= 1.0f, "duty_b clamped");
    CHECK(duty.duty_c >= 0.0f && duty.duty_c <= 1.0f, "duty_c clamped");
}

static void test_sinusoidal_pwm(void)
{
    float duty = sinusoidal_pwm_duty(24.0f, 0.0f, 48.0f);
    /* cos(0)=1: v_ref = 24, vdc = 48 => duty = 0.5 + 24/48 = 1.0 */
    CHECK(duty >= 0.99f && duty <= 1.01f, "SPWM duty at peak");

    duty = sinusoidal_pwm_duty(24.0f, (float)M_PI, 48.0f);
    /* cos(pi)=-1: v_ref = -24 => duty = 0.5 - 24/48 = 0.0 */
    ASSERT_NEAR(duty, 0.0f, "SPWM duty at trough");
}

static void test_third_harmonic_injection(void)
{
    float a = 0.9f, b = -0.2f, c = -0.7f;
    float a_orig = a, b_orig = b, c_orig = c;
    third_harmonic_injection(&a, &b, &c);

    /* Injection preserves sum? Actually it adds common-mode.
       a+b+c = original + 3*offset.
       This is by design -- the common-mode voltage doesn't affect
       motor phase-to-phase voltages. */
    CHECK(a != a_orig || b != b_orig || c != c_orig, "THI modifies references");
}

static void test_dpwm(void)
{
    pwm_duty_t duty_spwm, duty_dpwm0, duty_dpwm1; (void)duty_dpwm0; (void)duty_dpwm1;

    sinusoidal_pwm_three_phase(20.0f, 0.5f, 48.0f, &duty_spwm);
    dpwm0_modulate(20.0f, 0.5f, 48.0f, &duty_dpwm0);
    dpwm1_modulate(20.0f, 0.5f, 48.0f, &duty_dpwm1);

    /* At least one DPWM should have a different pattern than SPWM */
    int diff_count = 0;
    if (fabsf(duty_dpwm0.duty_a - duty_spwm.duty_a) > 0.01f) diff_count++;
    if (fabsf(duty_dpwm0.duty_b - duty_spwm.duty_b) > 0.01f) diff_count++;
    if (fabsf(duty_dpwm0.duty_c - duty_spwm.duty_c) > 0.01f) diff_count++;
    CHECK(diff_count > 0, "DPWM differs from SPWM");
}

static void test_modulation_index(void)
{
    float mi = modulation_index(24.0f, 48.0f);
    ASSERT_NEAR(mi, 1.0f, "MI = 24/(48/2) = 1.0");

    mi = modulation_index(0.0f, 48.0f);
    ASSERT_NEAR(mi, 0.0f, "Zero voltage -> MI = 0");
}

static void test_max_linear_voltage(void)
{
    float v_max_svpwm = svpwm_max_linear_voltage(48.0f);
    float expected_svpwm = 48.0f / M_SQRT3;
    ASSERT_NEAR(v_max_svpwm, expected_svpwm, "SVPWM Vmax = Vdc/sqrt(3)");

    float v_max_spwm = spwm_max_linear_voltage(48.0f);
    ASSERT_NEAR(v_max_spwm, 24.0f, "SPWM Vmax = Vdc/2");
}

static void test_dc_bus_ripple_compensation(void)
{
    pwm_duty_t duty = {0.75f, 0.25f, 0.25f};
    dc_bus_ripple_compensation(&duty, 48.0f, 40.0f);  /* Vdc drooped */
    /* duty should increase to compensate for lower Vdc */
    CHECK(duty.duty_a > 0.75f, "Duty increases when Vdc drops");
    CHECK(duty.duty_b < 0.25f, "Lower duty decreases further");
}

static void test_dead_time_compensation(void)
{
    pwm_duty_t duty = {0.5f, 0.5f, 0.5f};
    phase_currents_t currents = {2.0f, -1.0f, -1.0f};  /* A pos, B,C neg */

    dead_time_compensation(&duty, &currents, 48.0f, 1e-6f, 1e-4f);

    /* Phase A: positive current -> slightly increased duty */
    CHECK(duty.duty_a > 0.5f, "Phase A comp: duty increases");
    /* Phase B,C: negative current -> slightly decreased duty */
    CHECK(duty.duty_b < 0.5f, "Phase B comp: duty decreases");
}

int main(void)
{
    printf("=== test_svpwm ===\n");

    test_svpwm_sector_angles();
    test_svpwm_timing_linear();
    test_svpwm_duty_generation();
    test_svpwm_overmodulation();
    test_sinusoidal_pwm();
    test_third_harmonic_injection();
    test_dpwm();
    test_modulation_index();
    test_max_linear_voltage();
    test_dc_bus_ripple_compensation();
    test_dead_time_compensation();

    printf("Results: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
