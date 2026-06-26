/**
 * @file example_temperature.c
 * @brief Example: Temperature Control of a Water Bath with PID
 *
 * Demonstrates:
 *   1. Thermal system modeling (lumped capacitance)
 *   2. PID tuning for slow thermal processes
 *   3. Anti-windup importance in heating systems (output saturation at 0/100%)
 *   4. Comparison of P, PI, PID control for temperature regulation
 *   5. Disturbance rejection (ambient temperature changes)
 *
 * L7 Application: HVAC and industrial process temperature control.
 * Reference: Astrom & Hagglund (2006), "Advanced PID Control", Chapter 6.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pid_core.h"
#include "pid_tuning.h"
#include "pid_analysis.h"
#include "pid_applications.h"

/* Simulate a thermal process with PID in the loop (not using the library
 * simulator, to demonstrate direct real-time-style PID usage) */
static void simulate_thermal_loop(const PIDController *pid_template,
                                   const ThermalModel *tm,
                                   double setpoint, double duration, double Ts,
                                   double *time, double *temp, double *power,
                                   size_t N) {
    PIDController pid;
    memcpy(&pid, pid_template, sizeof(pid));
    pid.params.Ts = Ts;
    pid_reset(&pid);

    double T = tm->T_ambient;

    for (size_t k = 0; k < N; k++) {
        time[k] = k * Ts;

        /* PID computes heater power */
        double P = pid_update(&pid, setpoint, T);
        /* Clamp power to [0, max_power] (heater cannot cool) */
        if (P < 0.0) P = 0.0;
        if (P > tm->max_power) P = tm->max_power;
        power[k] = P;

        /* Thermal dynamics: dT/dt = (P - (T-T_ambient)/Rth) / (m*Cp) */
        double mCp = tm->mass * tm->Cp;
        double dT = (P - (T - tm->T_ambient) / tm->Rth) / mCp * Ts;
        T += dT;
        temp[k] = T;
    }
}

int main(void) {
    printf("\n");
    printf("????????????????????????????????????????????????????????????\n");
    printf("?   Temperature Control ? PID Tuning for Water Bath       ?\n");
    printf("????????????????????????????????????????????????????????????\n\n");

    /*---------------------------------------------------------------------------
     * Step 1: Model a 20L water bath
     *---------------------------------------------------------------------------*/
    printf("--- Step 1: Thermal Model ---\n");
    ThermalModel tm;
    thermal_model_init(&tm, 1);  /* 20L water bath */
    printf("  Water mass: %.1f kg\n", tm.mass);
    printf("  Specific heat: %.0f J/(kg.K)\n", tm.Cp);
    printf("  Thermal resistance: %.2f K/W\n", tm.Rth);
    printf("  Max heater power: %.0f W\n", tm.max_power);
    printf("  Ambient temperature: %.1f ?C\n", tm.T_ambient);

    /* Time constant */
    double tau = tm.mass * tm.Cp * tm.Rth;
    printf("  Thermal time constant: %.0f s (%.1f min)\n", tau, tau / 60.0);

    /* Get FOPDT model for tuning */
    FOPDTModel model;
    thermal_to_fopdt(&tm, &model);
    printf("\n  FOPDT model: K=%.3f, T=%.1f s, L=%.1f s\n",
           model.K, model.T, model.L);

    /*---------------------------------------------------------------------------
     * Step 2: Tune PID for temperature control
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 2: PID Tuning ---\n");

    /* Run tuning comparison */
    pid_tuning_comparison(&model, 1.0);

    /* Select Lambda tuning for conservative temperature control (no overshoot) */
    PIDTuningResult result;
    pid_tune_lambda(&model, model.T * 1.5, PID_FORM_STANDARD, 1.0, &result);

    printf("\n  Selected: %s\n", pid_tuning_method_name(result.method));
    printf("  Kp=%.4f, Ti=%.1f s, Td=%.1f s\n",
           result.params.Kp, result.params.Ti, result.params.Td);

    /* Configure PID with anti-windup (critical for heating ? cannot cool) */
    PIDController pid;
    pid_init(&pid, PID_FORM_STANDARD);
    pid_apply_tuning(&pid, &result);
    pid_set_output_limits(&pid, 0.0, tm.max_power);
    pid_set_integral_limits(&pid, -tm.max_power * 0.5, tm.max_power * 0.5);
    pid_set_antiwindup(&pid, PID_AW_COMBINED, 0.1);

    /*---------------------------------------------------------------------------
     * Step 3: Compare P, PI, PID control
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 3: P vs PI vs PID Comparison ---\n");
    printf("  Simulating step from %.0f to %.0f ?C...\n",
           tm.T_ambient, tm.T_ambient + 50.0);

    double setpoint = tm.T_ambient + 50.0;
    double Ts = 2.0;
    double duration = 7200.0; /* 2 hours */
    size_t N = (size_t)(duration / Ts);
    if (N > 100000) N = 100000;

    double *time = malloc(N * sizeof(double));
    double *temp_P = malloc(N * sizeof(double));
    double *temp_PI = malloc(N * sizeof(double));
    double *temp_PID = malloc(N * sizeof(double));
    double *power = malloc(N * sizeof(double));

    /* P-only */
    PIDController pid_P;
    pid_init(&pid_P, PID_FORM_STANDARD);
    pid_P.params.Kp = result.params.Kp * 0.5;  /* reduce Kp without I/D */
    pid_P.params.Ki = 0.0;
    pid_P.params.Kd = 0.0;
    pid_set_output_limits(&pid_P, 0.0, tm.max_power);
    simulate_thermal_loop(&pid_P, &tm, setpoint, duration, Ts,
                          time, temp_P, power, N);

    /* PI */
    PIDController pid_PI;
    pid_init(&pid_PI, PID_FORM_STANDARD);
    pid_PI.params.Kp = result.params.Kp;
    pid_PI.params.Ki = result.params.Ki;
    pid_PI.params.Kd = 0.0;
    pid_set_output_limits(&pid_PI, 0.0, tm.max_power);
    pid_set_antiwindup(&pid_PI, PID_AW_COMBINED, 0.1);
    simulate_thermal_loop(&pid_PI, &tm, setpoint, duration, Ts,
                          time, temp_PI, power, N);

    /* PID */
    PIDController pid_PID;
    pid_init(&pid_PID, PID_FORM_STANDARD);
    pid_apply_tuning(&pid_PID, &result);
    pid_set_output_limits(&pid_PID, 0.0, tm.max_power);
    pid_set_antiwindup(&pid_PID, PID_AW_COMBINED, 0.1);
    simulate_thermal_loop(&pid_PID, &tm, setpoint, duration, Ts,
                          time, temp_PID, power, N);

    /* Compute metrics for each */
    StepResponseMetrics m_P, m_PI, m_PID;
    pid_step_metrics(time, temp_P, N, setpoint, &m_P);
    pid_step_metrics(time, temp_PI, N, setpoint, &m_PI);
    pid_step_metrics(time, temp_PID, N, setpoint, &m_PID);

    printf("\n  %-20s %12s %12s %12s\n", "Metric", "P-only", "PI", "PID");
    printf("  ------------------------------------------------------------\n");
    printf("  %-20s %11.1f%% %11.1f%% %11.1f%%\n",
           "Overshoot", m_P.overshoot*100, m_PI.overshoot*100, m_PID.overshoot*100);
    printf("  %-20s %11.0fs %11.0fs %11.0fs\n",
           "Settling time", m_P.settling_time, m_PI.settling_time, m_PID.settling_time);
    printf("  %-20s %11.2f?C %11.2f?C %11.2f?C\n",
           "Steady-state err", m_P.steady_state_error,
           m_PI.steady_state_error, m_PID.steady_state_error);
    printf("  %-20s %11.0f %11.0f %11.0f\n",
           "IAE", m_P.IAE, m_PI.IAE, m_PID.IAE);

    /*---------------------------------------------------------------------------
     * Step 4: Disturbance rejection test
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 4: Disturbance Rejection ---\n");
    printf("  Adding 10?C ambient temperature drop at t=2000s...\n");

    /* Simulate with PI and disturbance */
    PIDController pid_dist;
    memcpy(&pid_dist, &pid_PID, sizeof(pid_dist));
    pid_reset(&pid_dist);

    double *temp_dist = malloc(N * sizeof(double));
    double *power_dist = malloc(N * sizeof(double));
    double T_dist = tm.T_ambient;
    double amb_temp = tm.T_ambient;

    for (size_t k = 0; k < N; k++) {
        double t = k * Ts;
        /* Ambient temperature drops by 10?C at t=2000s */
        if (t >= 2000.0) {
            amb_temp = tm.T_ambient - 10.0;
        } else {
            amb_temp = tm.T_ambient;
        }

        double P = pid_update(&pid_dist, setpoint, T_dist);
        if (P < 0.0) P = 0.0;
        if (P > tm.max_power) P = tm.max_power;
        power_dist[k] = P;

        double mCp = tm.mass * tm.Cp;
        double dT = (P - (T_dist - amb_temp) / tm.Rth) / mCp * Ts;
        T_dist += dT;
        temp_dist[k] = T_dist;
    }

    /* Find max deviation after disturbance */
    double max_dev = 0.0;
    for (size_t k = (size_t)(2000.0/Ts); k < N; k++) {
        double dev = fabs(temp_dist[k] - setpoint);
        if (dev > max_dev) max_dev = dev;
    }
    printf("  Maximum deviation after disturbance: %.2f?C\n", max_dev);
    printf("  Recovery behavior: disturbance rejected by integral action\n");

    /*---------------------------------------------------------------------------
     * Cleanup
     *---------------------------------------------------------------------------*/
    free(time); free(temp_P); free(temp_PI); free(temp_PID);
    free(power); free(temp_dist); free(power_dist);

    printf("\n=== Temperature PID Control Example Complete ===\n\n");
    return 0;
}
