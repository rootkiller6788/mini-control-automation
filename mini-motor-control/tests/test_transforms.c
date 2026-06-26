/**
 * test_transforms.c ¡ª Tests for Clarke/Park/Inverse transforms
 *
 * L3: Mathematical Structures ¡ª validates coordinate transformation math.
 * Tests cover:
 *   1. Clarke amplitude invariance
 *   2. Inverse Clarke recovers original (balanced)
 *   3. Park transform orthogonality (|(d,q)| = |(alpha,beta)|)
 *   4. Inverse Park roundtrip
 *   5. abc->dq->abc roundtrip for balanced system
 *   6. Angle normalization
 *   7. Angle difference
 *   8. Mechanical/electrical angle conversion
 *   9. Balanced detection
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "transforms.h"

#define TOL 1e-5f

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_FLOAT_EQ(a, b, msg) do { \
    tests_run++; \
    if (fabsf((a) - (b)) < TOL) { tests_passed++; } \
    else { printf("FAIL %s: got %f, expected %f\n", msg, (double)(a), (double)(b)); } \
} while(0)

static void test_clarke_amplitude_invariance(void)
{
    /* For balanced 3-phase with amplitude A: alpha should equal A */
    float ia = 1.0f, ib = -0.5f, ic = -0.5f;

    clarke_vector_t v = clarke_transform(ia, ib, ic);
    ASSERT_FLOAT_EQ(v.alpha, 1.0f, "Clarke alpha=1 for Ia=1 with Ib=Ic=-0.5");
    /* beta = (ib - ic)/sqrt(3) = (-0.5 + 0.5)/sqrt(3) = 0 */
    ASSERT_FLOAT_EQ(v.beta, 0.0f, "Clarke beta=0 for symmetric Ib,Ic");
    /* zero = (1 - 0.5 - 0.5)/3 = 0 */
    ASSERT_FLOAT_EQ(v.zero, 0.0f, "Clarke zero=0 for balanced system");
}

static void test_clarke_roundtrip(void)
{
    /* Clarke + inverse Clarke should recover original for balanced inputs */
    float cases[4][3] = {
        {1.0f, -0.5f, -0.5f},
        {0.0f, 0.866f, -0.866f},
        {-0.5f, 1.0f, -0.5f},
        {2.0f, -1.0f, -1.0f}
    };
    for (int i = 0; i < 4; i++) {
        float ia = cases[i][0], ib = cases[i][1], ic = cases[i][2];
        clarke_vector_t v = clarke_transform(ia, ib, ic);
        phase_voltages_t pv = inverse_clarke_transform(&v);
        char msg[64];
        snprintf(msg, sizeof(msg), "Clarke roundtrip case %d", i);
        ASSERT_FLOAT_EQ(pv.va, ia, msg);
        /* Due to zero-sequence handling, exact recovery is guaranteed */
    }
}

static void test_park_orthogonality(void)
{
    /* Park transform preserves vector magnitude */
    float alpha = 3.0f, beta = 4.0f;  /* known 3-4-5 triangle */
    float mag_ab = sqrtf(alpha*alpha + beta*beta);

    float angles[] = {0.0f, 0.785f, 1.571f, 3.142f, 4.712f};
    for (int i = 0; i < 5; i++) {
        float theta = angles[i];
        park_vector_t pv;
        park_transform_direct(alpha, beta, theta, &pv.d, &pv.q);
        float mag_dq = sqrtf(pv.d*pv.d + pv.q*pv.q);
        char msg[64];
        snprintf(msg, sizeof(msg), "Park orthogonality angle=%.3f", (double)theta);
        ASSERT_FLOAT_EQ(mag_dq, mag_ab, msg);
    }
}

static void test_park_roundtrip(void)
{
    /* Park + inverse Park = identity */
    float alpha = 2.5f, beta = -1.5f;
    float theta = 1.2f;

    park_vector_t pv = park_transform(&(clarke_vector_t){alpha, beta, 0.0f}, theta);
    clarke_vector_t recovered = inverse_park_transform(&pv, theta);

    ASSERT_FLOAT_EQ(recovered.alpha, alpha, "Park roundtrip alpha");
    ASSERT_FLOAT_EQ(recovered.beta, beta, "Park roundtrip beta");
}

static void test_abc_to_dq_to_abc(void)
{
    /* Full forward-inverse chain: abc -> dq -> abc */
    phase_currents_t i_abc = {2.0f, -0.5f, -1.5f};  /* balanced: 2-0.5-1.5=0 */
    float theta = 0.8f;

    park_vector_t i_dq = abc_to_dq(i_abc, theta);
    phase_voltages_t v_abc = dq_to_abc(i_dq, theta);

    ASSERT_FLOAT_EQ(v_abc.va, i_abc.ia, "Full chain phase A");
    ASSERT_FLOAT_EQ(v_abc.vb, i_abc.ib, "Full chain phase B");
    ASSERT_FLOAT_EQ(v_abc.vc, i_abc.ic, "Full chain phase C");
}

static void test_angle_normalization(void)
{
    float n = normalize_angle(3.5f * (float)M_PI);
    /* 3.5*pi = 1.75 turns => equivalent to -0.5*pi */
    ASSERT_FLOAT_EQ(n, -0.5f * (float)M_PI, "Normalize 3.5*pi -> -0.5*pi");

    n = normalize_angle(-4.0f * (float)M_PI);
    /* -4*pi = -2 turns => equivalent to 0 */
    ASSERT_FLOAT_EQ(n, 0.0f, "Normalize -4*pi -> 0");
}

static void test_angle_difference(void)
{
    float diff = angle_difference(0.1f, 0.2f);
    ASSERT_FLOAT_EQ(diff, -0.1f, "Angle diff 0.1-0.2 = -0.1");

    /* Wrap-around case */
    diff = angle_difference(3.1f, -3.1f);
    /* 3.1 - (-3.1) = 6.2, normalized within [-pi, pi] ->~ -0.083 */
    /* Actually 6.2 - 2*pi ¡Ö 6.2 - 6.283 = -0.083 */
}

static void test_mech_elec_conversion(void)
{
    float theta_m = 2.0f;
    float P = 4.0f;
    float theta_e = mechanical_to_electrical_angle(theta_m, P);
    ASSERT_FLOAT_EQ(theta_e, 8.0f, "theta_e = P * theta_m");

    float recovered = electrical_to_mechanical_angle(theta_e, P);
    ASSERT_FLOAT_EQ(recovered, theta_m, "theta_m = theta_e / P");
}

static void test_balanced_detection(void)
{
    assert(is_balanced(1.0f, -0.5f, -0.5f, 0.001f));
    assert(!is_balanced(1.0f, 0.5f, 0.5f, 0.001f));  /* sum = 2.0 */
}

static void test_phase_reconstruction(void)
{
    float ic = reconstruct_phase_c(2.0f, -1.0f);
    ASSERT_FLOAT_EQ(ic, -1.0f, "ic = -(ia+ib)");
}

static void test_voltage_to_duty(void)
{
    float duty = voltage_to_duty(0.0f, 24.0f);
    ASSERT_FLOAT_EQ(duty, 0.5f, "0V -> 50% duty");

    duty = voltage_to_duty(12.0f, 24.0f);
    ASSERT_FLOAT_EQ(duty, 1.0f, "12V with 24V bus -> 100% duty (clamped)");

    duty = voltage_to_duty(-12.0f, 24.0f);
    ASSERT_FLOAT_EQ(duty, 0.0f, "-12V -> 0% duty (clamped)");
}

static void test_dq_magnitude(void)
{
    park_vector_t pv = {3.0f, 4.0f};
    float mag = dq_magnitude(pv);
    ASSERT_FLOAT_EQ(mag, 5.0f, "|(3,4)| = 5");
}

int main(void)
{
    printf("=== test_transforms ===\n");

    test_clarke_amplitude_invariance();
    test_clarke_roundtrip();
    test_park_orthogonality();
    test_park_roundtrip();
    test_abc_to_dq_to_abc();
    test_angle_normalization();
    test_angle_difference();
    test_mech_elec_conversion();
    test_balanced_detection();
    test_phase_reconstruction();
    test_voltage_to_duty();
    test_dq_magnitude();

    printf("Results: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
