/**
 * test_motor_model.c ¡ª Tests for motor model functions
 *
 * L3/L4: Mathematical Structures & Fundamental Laws
 * Tests DC motor, PMSM, induction motor, and stepper models.
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "motor_model.h"

#define TOL 1e-4f

static int tests_run = 0;
static int tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("FAIL %s\n", msg); } \
} while(0)

#define ASSERT_NEAR(a, b, msg) do { \
    tests_run++; \
    if (fabsf((a) - (b)) < TOL) { tests_passed++; } \
    else { printf("FAIL %s: got %.6f, expected %.6f\n", msg, (double)(a), (double)(b)); } \
} while(0)

static void test_dc_motor_steady_state(void)
{
    dc_motor_params_t p = {
        .winding_resistance = 0.5f,
        .winding_inductance = 0.002f,
        .ke_back_emf = 0.05f,
        .kt_torque = 0.05f,
        .rotor_inertia = 0.0001f,
        .friction_coefficient = 0.00001f,
        .pole_pairs = 1.0f
    };

    /* No-load steady state speed at 24V */
    float w_ss = dc_motor_steady_state_speed(24.0f, &p);
    CHECK(w_ss > 0.0f, "DC motor steady-state speed positive");
    CHECK(w_ss < 500.0f, "DC motor steady-state speed reasonable");

    /* Back-EMF at speed */
    float bemf = dc_motor_back_emf(100.0f, &p);
    ASSERT_NEAR(bemf, 5.0f, "Back-EMF at 100 rad/s");

    /* Torque from current */
    float torque = dc_motor_torque_from_current(2.0f, &p);
    ASSERT_NEAR(torque, 0.1f, "Torque at 2A");

    /* Time constants */
    float tau_e = dc_motor_electrical_time_constant(&p);
    ASSERT_NEAR(tau_e, 0.004f, "Electrical time constant L/R");
    float tau_m = dc_motor_mechanical_time_constant(&p);
    CHECK(tau_m > 0.0f, "Mechanical time constant positive");
    CHECK(tau_m > tau_e, "Mechanical >> Electrical for this motor");
}

static void test_dc_motor_derivatives(void)
{
    dc_motor_params_t p = {
        .winding_resistance = 1.0f,
        .winding_inductance = 0.01f,
        .ke_back_emf = 0.1f,
        .kt_torque = 0.1f,
        .rotor_inertia = 0.001f,
        .friction_coefficient = 0.0f,
        .pole_pairs = 1.0f
    };

    /* At startup (omega=0, V=10V): di/dt = (10 - 1*0 - 0.1*0)/0.01 = 1000 A/s */
    float di = dc_motor_current_derivative(0.0f, 0.0f, 10.0f, &p);
    ASSERT_NEAR(di, 1000.0f, "di/dt at startup");

    /* With 5A current at startup: dw/dt = (0.1*5 - 0 - 0)/0.001 = 500 rad/s^2 */
    float dw = dc_motor_speed_derivative(5.0f, 0.0f, 0.0f, &p);
    ASSERT_NEAR(dw, 500.0f, "dw/dt at 5A startup");
}

static void test_dc_motor_transfer_function(void)
{
    dc_motor_params_t p = {
        .winding_resistance = 1.0f,
        .winding_inductance = 0.001f,
        .ke_back_emf = 0.1f,
        .kt_torque = 0.1f,
        .rotor_inertia = 0.0001f,
        .friction_coefficient = 0.0f,
        .pole_pairs = 1.0f
    };
    dc_motor_tf_t tf = dc_motor_transfer_function(&p);

    /* a2 = J*L = 0.0001 * 0.001 = 1e-7 */
    ASSERT_NEAR(tf.a2, 1e-7f, "TF a2 = J*L");
    /* a1 = J*R + B*L = 0.0001*1 + 0 = 1e-4 */
    ASSERT_NEAR(tf.a1, 1e-4f, "TF a1 = J*R");
    /* a0 = B*R + Kt*Ke = 0 + 0.01 = 0.01 */
    ASSERT_NEAR(tf.a0, 0.01f, "TF a0 = Kt*Ke");
    /* b0 = Kt = 0.1 */
    ASSERT_NEAR(tf.b0, 0.1f, "TF b0 = Kt");
}

static void test_dc_motor_rk4(void)
{
    dc_motor_params_t p = {
        .winding_resistance = 1.0f,
        .winding_inductance = 0.01f,
        .ke_back_emf = 0.1f,
        .kt_torque = 0.1f,
        .rotor_inertia = 0.001f,
        .friction_coefficient = 0.0f,
        .pole_pairs = 1.0f
    };
    motor_state_t state = {0};
    state.dc_bus_voltage = 10.0f;

    /* Apply 10V, no load, simulate 1ms */
    dc_motor_rk4_step(&state, 10.0f, 0.0f, 0.001f, &p);
    CHECK(state.current_dq.q > 0.0f, "Current increases with positive voltage");
    CHECK(state.rotor_speed_mechanical > 0.0f, "Speed increases with positive voltage");
    CHECK(state.rotor_angle_mechanical > 0.0f, "Angle advances");
}

static void test_pmsm_torque(void)
{
    pmsm_params_t p = {
        .rs = 0.3f, .ld = 0.001f, .lq = 0.0012f,
        .flux_linkage = 0.05f, .pole_pairs = 2.0f,
        .rotor_inertia = 0.0001f, .friction_coefficient = 0.0f
    };

    /* SPM approximation: Te = 1.5*P*psi_m*Iq (Id=0) */
    float te = pmsm_torque(0.0f, 1.0f, &p);
    float te_expected = 1.5f * 2.0f * 0.05f * 1.0f; /* = 0.15 */
    ASSERT_NEAR(te, te_expected, "PMSM torque Id=0, Iq=1");

    /* With negative Id (IPM), reluctance torque adds */
    float te_ipm = pmsm_torque(-0.5f, 1.0f, &p);
    CHECK(te_ipm > te, "IPM torque > SPM torque with negative Id (Ld<Lq)");
}

static void test_pmsm_derivatives(void)
{
    pmsm_params_t p = {
        .rs = 1.0f, .ld = 0.01f, .lq = 0.01f,
        .flux_linkage = 0.1f, .pole_pairs = 2.0f,
        .rotor_inertia = 0.001f, .friction_coefficient = 0.0f
    };

    /* At standstill: dId/dt = (Vd - Rs*Id) / Ld */
    float did = pmsm_id_derivative(0.0f, 0.0f, 10.0f, 0.0f, &p);
    ASSERT_NEAR(did, 1000.0f, "PMSM dId/dt at startup");

    /* dIq/dt = (Vq - Rs*Iq - omega*psi_m) / Lq */
    float diq = pmsm_iq_derivative(0.0f, 0.0f, 10.0f, 0.0f, &p);
    ASSERT_NEAR(diq, 1000.0f, "PMSM dIq/dt at startup");
}

static void test_pmsm_mtpa(void)
{
    pmsm_params_t p = {
        .rs = 0.3f, .ld = 0.0008f, .lq = 0.0012f,  /* Lq > Ld (IPM) */
        .flux_linkage = 0.05f, .pole_pairs = 2.0f,
        .rotor_inertia = 0.0001f, .friction_coefficient = 0.0f
    };

    /* MTPA Id should be negative for IPM (Lq > Ld) when Iq > 0 */
    float id_mtpa = pmsm_mtpa_id(1.0f, &p);
    CHECK(id_mtpa < 0.0f, "MTPA Id is negative for IPM");

    /* For SPM (Ld = Lq), MTPA Id should be 0 */
    pmsm_params_t p_spm = p;
    p_spm.lq = p_spm.ld;
    float id_mtpa_spm = pmsm_mtpa_id(1.0f, &p_spm);
    ASSERT_NEAR(id_mtpa_spm, 0.0f, "MTPA Id = 0 for SPM");
}

static void test_pmsm_field_weakening(void)
{
    pmsm_params_t p = {
        .rs = 0.3f, .ld = 0.001f, .lq = 0.001f,
        .flux_linkage = 0.05f, .pole_pairs = 2.0f,
        .rotor_inertia = 0.0001f, .friction_coefficient = 0.0f
    };

    /* At high speed, FW Id should be negative */
    float id_fw = pmsm_field_weakening_id(0.0f, 1000.0f, 48.0f, &p);
    CHECK(id_fw < 0.0f, "Field weakening Id is negative");
    /* At low speed, FW Id should be 0 */
    id_fw = pmsm_field_weakening_id(0.0f, 1.0f, 48.0f, &p);
    ASSERT_NEAR(id_fw, 0.0f, "No field weakening at low speed");
}

static void test_im_synchronous_speed(void)
{
    float ns = im_synchronous_speed(50.0f, 2.0f);
    /* ns = 2*pi*50/2 = 157.08 rad/s electrical */
    float expected = 2.0f * (float)M_PI * 50.0f / 2.0f;
    ASSERT_NEAR(ns, expected, "IM synchronous speed");
}

static void test_im_slip(void)
{
    float ns = 157.08f;
    float nr = 150.0f;
    float s = im_slip(ns, nr);
    ASSERT_NEAR(s, (ns-nr)/ns, "IM slip calculation");
}

static void test_stepper(void)
{
    stepper_params_t p = {
        .winding_resistance = 1.5f, .winding_inductance = 0.003f,
        .holding_torque = 0.45f, .detent_torque = 0.02f,
        .steps_per_rev = 200, .rated_current = 1.7f
    };

    float step_angle = stepper_step_angle_deg(&p);
    ASSERT_NEAR(step_angle, 1.8f, "200 steps/rev -> 1.8 deg per step");

    /* At zero error, torque should be 0 */
    float t0 = stepper_torque_vs_angle(0.0f, &p);
    ASSERT_NEAR(t0, 0.0f, "Zero torque at zero angle error");
}

int main(void)
{
    printf("=== test_motor_model ===\n");

    test_dc_motor_steady_state();
    test_dc_motor_derivatives();
    test_dc_motor_transfer_function();
    test_dc_motor_rk4();
    test_pmsm_torque();
    test_pmsm_derivatives();
    test_pmsm_mtpa();
    test_pmsm_field_weakening();
    test_im_synchronous_speed();
    test_im_slip();
    test_stepper();

    printf("Results: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
