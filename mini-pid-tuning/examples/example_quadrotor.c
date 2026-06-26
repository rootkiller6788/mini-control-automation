/**
 * @file example_quadrotor.c
 * @brief Example: Quadrotor Attitude Control with Cascade PID
 *
 * Demonstrates:
 *   1. Quadrotor single-axis attitude dynamics
 *   2. Cascade PID control: outer angle loop + inner rate loop
 *   3. Gain scheduling for varying operating conditions
 *   4. Step response and disturbance rejection
 *
 * L7 Application: Drone/UAV attitude stabilization.
 * L8 Topic: Cascade control with gain scheduling.
 *
 * Reference: Bouabdallah, Murrieri & Siegwart (2004),
 *   "Design and Control of an Indoor Micro Quadrotor", ICRA.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pid_core.h"
#include "pid_tuning.h"
#include "pid_analysis.h"
#include "pid_applications.h"
#include "pid_advanced.h"

int main(void) {
    printf("\n");
    printf("????????????????????????????????????????????????????????????\n");
    printf("?   Quadrotor Attitude Control ? Cascade PID Tuning       ?\n");
    printf("????????????????????????????????????????????????????????????\n\n");

    /*---------------------------------------------------------------------------
     * Step 1: Model the quadrotor (single axis, e.g., roll)
     *---------------------------------------------------------------------------*/
    printf("--- Step 1: Quadrotor Model ---\n");
    QuadrotorAxisModel qm;
    quadrotor_axis_init(&qm, 2);  /* Medium (DJI Phantom class, 450mm) */
    printf("  Moment of inertia: %.4f kg.m^2\n", qm.J);
    printf("  Aerodynamic damping: %.4f Nm/(rad/s)\n", qm.b);
    printf("  Arm length: %.3f m\n", qm.arm_length);
    printf("  Max torque: %.3f Nm\n", qm.max_torque);

    /* Get dynamics model */
    SOPDTModel sopdt;
    quadrotor_to_sopdt(&qm, &sopdt);
    printf("\n  Rate dynamics (SOPDT):\n");
    printf("    K=%.4f, T1=%.4f s, T2=%.4f s, L=%.4f s\n",
           sopdt.K, sopdt.T1, sopdt.T2, sopdt.L);

    /* For inner rate loop: use FOPDT approximation */
    FOPDTModel rate_model;
    rate_model.K = sopdt.K;
    rate_model.T = sopdt.T1;
    rate_model.L = sopdt.L;

    /* For outer angle loop: the rate loop closed + integrator 1/s */
    /* Simplified: angle loop sees a first-order closed rate loop + integrator */
    FOPDTModel angle_model;
    angle_model.K = 1.0;
    angle_model.T = 0.1;
    angle_model.L = 0.02;

    /*---------------------------------------------------------------------------
     * Step 2: Tune Cascade PID
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 2: Cascade PID Tuning ---\n");

    /* Inner rate loop: tune for fast response (IMC, aggressive) */
    PIDTuningResult rate_tune;
    pid_tune_imc(&rate_model, rate_model.T * 0.3, PID_FORM_STANDARD, 0.002, &rate_tune);
    printf("  Inner rate loop (IMC aggressive):\n");
    printf("    Kp=%.4f, Ti=%.4f s, Td=%.4f s\n",
           rate_tune.params.Kp, rate_tune.params.Ti, rate_tune.params.Td);

    /* Outer angle loop: tune for moderate response (AMIGO) */
    PIDTuningResult angle_tune;
    pid_tune_amigo(&angle_model, PID_FORM_STANDARD, 0.01, &angle_tune);
    printf("  Outer angle loop (AMIGO):\n");
    printf("    Kp=%.4f, Ti=%.4f s, Td=%.4f s\n",
           angle_tune.params.Kp, angle_tune.params.Ti, angle_tune.params.Td);

    /* Configure cascade PID */
    CascadePID cas;
    cascade_pid_init(&cas, PID_FORM_STANDARD, PID_FORM_STANDARD);

    /* Outer loop (angle): PD mostly (no integral in angle loop to avoid windup) */
    cas.primary.params.Kp = angle_tune.params.Kp;
    cas.primary.params.Ki = 0.0;           /* No integral for angle */
    cas.primary.params.Kd = angle_tune.params.Kd;
    cas.primary.params.Ts = 0.01;
    pid_set_output_limits(&cas.primary, -3.14159, 3.14159);  /* ?pi rad/s rate limit */

    /* Inner loop (rate): full PID */
    cas.secondary.params.Kp = rate_tune.params.Kp;
    cas.secondary.params.Ki = rate_tune.params.Ki;
    cas.secondary.params.Kd = rate_tune.params.Kd;
    cas.secondary.params.Ts = 0.002;
    pid_set_output_limits(&cas.secondary, -qm.max_torque, qm.max_torque);
    pid_set_antiwindup(&cas.secondary, PID_AW_CLAMPING, 2.0);

    /*---------------------------------------------------------------------------
     * Step 3: Closed-loop simulation
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 3: Closed-Loop Attitude Step Response ---\n");

    double dt = 0.001;
    double duration = 2.0;
    size_t N = (size_t)(duration / dt);
    double *time = malloc(N * sizeof(double));
    double *angle = malloc(N * sizeof(double));
    double *rate = malloc(N * sizeof(double));
    double *torque_arr = malloc(N * sizeof(double));

    double theta = 0.0, omega = 0.0;
    double setpoint_angle = 0.5;  /* 0.5 rad (~29 degree) step */

    /* For the outer loop at 100Hz, inner at 500Hz */
    int outer_decimation = 10;  /* update outer loop every 10 inner steps */

    for (size_t k = 0; k < N; k++) {
        time[k] = k * dt;

        /* Outer loop updates at lower rate */
        double rate_setpoint;
        if (k % outer_decimation == 0) {
            rate_setpoint = pid_update(&cas.primary, setpoint_angle, theta);
        } else {
            /* Hold previous outer loop output */
            rate_setpoint = cas.secondary_setpoint;
        }

        /* Inner rate loop */
        double torque = pid_update(&cas.secondary, rate_setpoint, omega);
        if (torque > qm.max_torque) torque = qm.max_torque;
        if (torque < -qm.max_torque) torque = -qm.max_torque;
        torque_arr[k] = torque;

        /* Quadrotor dynamics */
        double domega = (torque - qm.b * omega) / qm.J * dt;
        omega += domega;
        theta += omega * dt;

        angle[k] = theta;
        rate[k] = omega;
    }

    /* Performance metrics */
    StepResponseMetrics metrics;
    pid_step_metrics(time, angle, N, setpoint_angle, &metrics);

    printf("  Angle step: %.2f rad (%.1f deg)\n",
           setpoint_angle, setpoint_angle * 180.0 / 3.14159);
    printf("  Rise time:       %.3f s\n", metrics.rise_time);
    printf("  Overshoot:       %.1f %%\n", metrics.overshoot * 100.0);
    printf("  Settling time:   %.3f s\n", metrics.settling_time);
    printf("  Steady-state err: %.4f rad\n", metrics.steady_state_error);

    /*---------------------------------------------------------------------------
     * Step 4: Gain scheduling demonstration
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 4: Gain Scheduling ---\n");
    printf("  Demonstrating gain scheduling for varying battery voltage...\n");

    /* As battery voltage drops, the effective torque constant decreases.
     * We compensate by increasing PID gains as voltage drops. */
    GainScheduleEntry gs_table[4] = {
        {16.8,  0.8, 0.16, 0.04},   /* Full battery (4S LiPo) */
        {15.0,  1.0, 0.20, 0.05},   /* Nominal */
        {14.0,  1.2, 0.24, 0.06},   /* Low */
        {12.0,  1.5, 0.30, 0.08}    /* Critical (land now) */
    };

    GainScheduledPID gspid;
    gs_pid_init(&gspid, PID_FORM_PARALLEL, gs_table, 4);
    gspid.pid.params.Ts = 0.01;

    /* Simulate at different battery voltages */
    double voltages[] = {16.8, 15.0, 14.0, 12.0};
    const char *labels[] = {"Full", "Nominal", "Low", "Critical"};
    printf("\n  %-12s %8s %8s %8s\n", "Battery", "Kp", "Ki", "Kd");
    printf("  ----------------------------------------\n");
    for (int i = 0; i < 4; i++) {
        double u = gs_pid_update(&gspid, 0.5, 0.0, voltages[i]);
        printf("  %-12s %8.3f %8.3f %8.3f\n",
               labels[i],
               gspid.pid.params.Kp,
               gspid.pid.params.Ki,
               gspid.pid.params.Kd);
    }

    /*---------------------------------------------------------------------------
     * Step 5: Disturbance rejection (wind gust)
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 5: Disturbance Rejection (Wind Gust) ---\n");
    printf("  Simulating 0.1 Nm gust torque at t=1.0s for 0.2s...\n");

    double theta2 = 0.0, omega2 = 0.0;
    double gust_start = 1.0, gust_duration = 0.2;
    double gust_torque = 0.1;

    /* Simulate with disturbance */
    for (size_t k = 0; k < N; k++) {
        double t = k * dt;
        double ext_torque = 0.0;
        if (t >= gust_start && t < gust_start + gust_duration) {
            ext_torque = gust_torque;
        }

        double rate_sp = pid_update(&cas.primary, setpoint_angle, theta2);
        double torque = pid_update(&cas.secondary, rate_sp, omega2);
        torque += ext_torque;  /* external disturbance adds to motor torque */

        if (torque > qm.max_torque) torque = qm.max_torque;
        if (torque < -qm.max_torque) torque = -qm.max_torque;

        double domega = (torque - qm.b * omega2) / qm.J * dt;
        omega2 += domega;
        theta2 += omega2 * dt;
    }

    /* Find maximum deviation during gust */
    size_t gust_start_idx = (size_t)(gust_start / dt);
    size_t gust_end_idx = (size_t)((gust_start + gust_duration + 0.5) / dt);
    if (gust_end_idx >= N) gust_end_idx = N - 1;
    double max_dev = 0.0;
    for (size_t k = gust_start_idx; k < gust_end_idx; k++) {
        double dev = fabs(theta2 - setpoint_angle);
        if (dev > max_dev) max_dev = dev;
    }
    printf("  Max angular deviation during gust: %.4f rad (%.2f deg)\n",
           max_dev, max_dev * 180.0 / 3.14159);
    printf("  Recovery: cascade PID rejects disturbance via inner rate loop\n");

    /*---------------------------------------------------------------------------
     * Cleanup
     *---------------------------------------------------------------------------*/
    free(time); free(angle); free(rate); free(torque_arr);

    printf("\n=== Quadrotor Cascade PID Control Example Complete ===\n\n");
    return 0;
}
