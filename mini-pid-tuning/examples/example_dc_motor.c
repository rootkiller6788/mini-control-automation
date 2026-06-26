/**
 * @file example_dc_motor.c
 * @brief Example: DC Motor Speed Control with PID Tuning
 *
 * Demonstrates:
 *   1. DC motor model initialization and FOPDT identification
 *   2. Multiple tuning method comparison (ZN, Cohen-Coon, AMIGO, IMC)
 *   3. Closed-loop step response simulation
 *   4. Performance evaluation and method selection
 *
 * This is a canonical L6 problem: motor speed regulation with PID.
 *
 * Reference: Ogata (2010), "Modern Control Engineering", Chapter 8
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pid_core.h"
#include "pid_tuning.h"
#include "pid_analysis.h"
#include "pid_applications.h"

int main(void) {
    printf("\n");
    printf("????????????????????????????????????????????????????????????\n");
    printf("?     DC Motor Speed Control ? PID Tuning Example         ?\n");
    printf("????????????????????????????????????????????????????????????\n\n");

    /*---------------------------------------------------------------------------
     * Step 1: Model the DC motor
     *---------------------------------------------------------------------------*/
    printf("--- Step 1: DC Motor Model ---\n");
    DCMotorModel motor;
    dc_motor_init(&motor, 1);  /* Medium industrial (100W) DC servo */
    printf("  Motor parameters:\n");
    printf("    R = %.4f ohm, L = %.4f H\n", motor.R, motor.L);
    printf("    Ke = %.4f V/(rad/s), Kt = %.4f Nm/A\n", motor.Ke, motor.Kt);
    printf("    J = %.6f kg.m^2, b = %.6f Nm/(rad/s)\n", motor.J, motor.b);

    /* Get FOPDT approximation */
    FOPDTModel model;
    dc_motor_to_fopdt(&motor, &model, 12.0);

    /* Simulate open-loop step response to verify model */
    printf("\n  Identified FOPDT model:\n");
    printf("    K = %.4f, T = %.4f s, L = %.4f s\n", model.K, model.T, model.L);
    printf("    L/T ratio = %.3f", model.L / (model.T + 1e-30));
    char type_buf[128];
    printf("  ?  %s\n", pid_process_type_name(&model, type_buf, sizeof(type_buf)));

    /*---------------------------------------------------------------------------
     * Step 2: Open-loop step response
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 2: Open-Loop Step Response ---\n");
    printf("  Applying 12V step to motor...\n");
    size_t N = 500;
    double *time = (double*)malloc(N * sizeof(double));
    double *speed = (double*)malloc(N * sizeof(double));
    double *current = (double*)malloc(N * sizeof(double));
    dc_motor_simulate(&motor, 12.0, 0.001, 0.5, time, speed, current, N);
    printf("  Final speed: %.2f rad/s (%.1f RPM)\n",
           speed[N-1], speed[N-1] * 60.0 / (2.0 * 3.14159));
    printf("  Peak current: %.3f A\n", current[10]);
    printf("  Rise time (open-loop): %.3f s\n",
           time[N-1] * 0.632);  /* approximate */

    /*---------------------------------------------------------------------------
     * Step 3: Tune PID using multiple methods
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 3: PID Tuning ---\n");
    /* Run tuning comparison (prints table internally) */
    pid_tuning_comparison(&model, 0.01);

    /*---------------------------------------------------------------------------
     * Step 4: Select best method and simulate closed-loop
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 4: Closed-Loop Step Response ---\n");

    /* Tune using IMC (best for L/T < 0.1 lag-dominant processes) */
    PIDTuningResult result;
    pid_tune_amigo(&model, PID_FORM_STANDARD, 0.01, &result);

    printf("  Selected: %s\n", pid_tuning_method_name(result.method));
    printf("  Parameters: Kp=%.4f, Ti=%.4f, Td=%.4f\n",
           result.params.Kp, result.params.Ti, result.params.Td);

    /* Configure PID */
    PIDController pid;
    pid_init(&pid, PID_FORM_STANDARD);
    pid_apply_tuning(&pid, &result);
    pid_set_output_limits(&pid, 0.0, 12.0);  /* 0-12V output */
    pid_set_antiwindup(&pid, PID_AW_COMBINED, 2.0);

    /* Simulate closed-loop */
    double *cl_time = (double*)malloc(N * sizeof(double));
    double *cl_output = (double*)malloc(N * sizeof(double));
    double *cl_control = (double*)malloc(N * sizeof(double));
    pid_simulate_step_response(&pid, &model, 100.0, 0.5, 0.001,
                               cl_time, cl_output, cl_control, N);

    /* Compute performance metrics */
    StepResponseMetrics metrics;
    pid_step_metrics(cl_time, cl_output, N, 100.0, &metrics);

    printf("\n  Performance Metrics:\n");
    printf("    Rise time:      %.4f s\n", metrics.rise_time);
    printf("    Overshoot:      %.1f %%\n", metrics.overshoot * 100.0);
    printf("    Settling time:  %.4f s\n", metrics.settling_time);
    printf("    Steady-state error: %.4f\n", metrics.steady_state_error);
    printf("    IAE:            %.4f\n", metrics.IAE);
    printf("    ITAE:           %.4f\n", metrics.ITAE);

    /* Performance score */
    double score;
    pid_performance_score(&pid, &model, &score, &metrics);
    printf("\n  Overall Performance Score: %.1f / 100\n", score);

    /*---------------------------------------------------------------------------
     * Step 5: Frequency domain analysis
     *---------------------------------------------------------------------------*/
    printf("\n--- Step 5: Frequency Domain Analysis ---\n");
    PIDTransferFunction tf;
    pid_get_transfer_function(&pid, &tf, PID_FORM_STANDARD);
    FrequencyAnalysis analysis;
    pid_loop_frequency_analysis(&tf, &model, 0.01, 500.0, 200, &analysis);
    pid_compute_stability_margins(&analysis);

    printf("  Gain margin:    %.2f dB\n", analysis.gain_margin);
    printf("  Phase margin:   %.1f deg\n", analysis.phase_margin * 180.0 / 3.14159);
    printf("  Bandwidth:      %.2f rad/s\n", analysis.bandwidth);

    if (analysis.gain_margin > 6.0 && analysis.phase_margin > 0.785) {
        printf("  ? Good stability margins (GM > 6dB, PM > 45deg)\n");
    } else {
        printf("  ? Stability margins may be insufficient\n");
    }

    /*---------------------------------------------------------------------------
     * Cleanup
     *---------------------------------------------------------------------------*/
    free(time); free(speed); free(current);
    free(cl_time); free(cl_output); free(cl_control);
    free(analysis.freq); free(analysis.magnitude); free(analysis.phase);

    printf("\n=== DC Motor PID Control Example Complete ===\n\n");
    return 0;
}
