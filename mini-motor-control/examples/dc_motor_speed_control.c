/**
 * dc_motor_speed_control.c
 * L6 Canonical Problem: DC motor speed control with PID
 *
 * Simulates a DC motor with a PI speed controller.
 * Demonstrates:
 *   - Motor parameter configuration
 *   - PID speed control loop
 *   - Step response with speed ramp
 *   - Torque limiting
 *
 * Model: 24V 100W DC brushed motor controlling an inertial load.
 */

#include <stdio.h>
#include <math.h>
#include "motor_model.h"
#include "motor_control.h"
#include "motor_types.h"

/* Forward declaration */
static float apply_ramp(float target_ref_unused);

int main(void)
{
    printf("========================================\n");
    printf(" DC Motor Speed Control Simulation\n");
    printf("========================================\n\n");

    /* Motor parameters: 24V 100W DC brushed motor */
    dc_motor_params_t motor;
    dc_motor_params_init(&motor);
    motor.nominal_voltage = 24.0f;
    motor.nominal_current = 5.0f;
    motor.winding_resistance = 0.5f;
    motor.winding_inductance = 0.002f;
    motor.ke_back_emf = 0.048f;   /* ~3000 RPM no-load at 24V */
    motor.kt_torque = 0.048f;
    motor.rotor_inertia = 0.0001f;
    motor.friction_coefficient = 0.00001f;

    printf("Motor: %.0fV, %.0fW, R=%.2f Ohm, L=%.0f uH\n",
            motor.nominal_voltage, motor.nominal_voltage * motor.nominal_current,
            motor.winding_resistance, motor.winding_inductance * 1e6f);

    /* Validate parameters */
    if (dc_motor_params_validate(&motor) != 0) {
        printf("ERROR: Invalid motor parameters\n");
        return 1;
    }

    /* Time constants */
    float tau_e = dc_motor_electrical_time_constant(&motor);
    float tau_m = dc_motor_mechanical_time_constant(&motor);
    printf("Time constants: tau_e=%.3f ms, tau_m=%.1f ms\n",
            tau_e * 1000.0f, tau_m * 1000.0f);

    /* Transfer function */
    dc_motor_tf_t tf = dc_motor_transfer_function(&motor);
    printf("TF: %.3e / (%.3e*s^2 + %.3e*s + %.3f)\n\n",
            (double)tf.b0, (double)tf.a2, (double)tf.a1, (double)tf.a0);

    /* Initialize state */
    motor_state_t state;
    motor_state_init(&state);

    /* Setup PI speed controller */
    pid_state_t speed_pid;
    pid_init(&speed_pid, 300.0f);  /* Target: 300 rad/s (~2865 RPM) */

    pid_gains_t speed_gains = {
        .kp = 0.5f, .ki = 20.0f, .kd = 0.0f,
        .n_coeff = 100.0f,
        .out_min = -10.0f,   /* Current limit */
        .out_max = 10.0f,
        .integral_limit = 10.0f
    };

    /* Load torque profile: 0.1 N*m constant */
    float load_torque = 0.1f;

    float dt = 0.001f;  /* 1 ms control loop */
    printf("Simulation: dt=%.0f us, target=%d rad/s, load=%.2f N*m\n",
            dt*1e6f, 300, load_torque);
    printf("%-8s  %-10s  %-10s  %-10s  %-10s\n",
           "Time[s]", "Speed[r/s]", "Current[A]", "V_term[V]", "Error");

    /* Run simulation for 1 second */
    float sim_time = 1.0f;
    int steps = (int)(sim_time / dt);

    for (int k = 0; k < steps; k++) {
        float t = k * dt;

        /* Speed measurement (feedback) */
        float speed_meas = state.rotor_speed_mechanical;

        /* PI speed controller: generates current reference */
        speed_pid.setpoint = apply_ramp(speed_pid.setpoint);
        speed_pid.feedback = speed_meas;
        float i_ref = pid_update(&speed_pid, &speed_gains, dt);

        /* Voltage feed-forward for better tracking */
        float v_ff = dc_motor_voltage_ff(speed_pid.setpoint,
                                          load_torque, &motor);

        /* Apply voltage (simple proportional current control) */
        float voltage = v_ff + 2.0f * (i_ref - state.current_dq.q);

        /* Clamp voltage */
        if (voltage > 24.0f) voltage = 24.0f;
        if (voltage < -24.0f) voltage = -24.0f;

        /* Integrate motor dynamics */
        dc_motor_rk4_step(&state, voltage, load_torque, dt, &motor);

        /* Print every 50 ms */
        if (k % 50 == 0) {
            printf("%8.3f  %10.2f  %10.3f  %10.2f  %10.2f\n",
                   t, state.rotor_speed_mechanical, state.current_dq.q,
                   voltage, speed_pid.error);
        }
    }

    /* Final state */
    float final_rpm = rad_per_sec_to_rpm(state.rotor_speed_mechanical);
    printf("\n=== Final State ===\n");
    printf("Speed:     %.1f rad/s  (%.0f RPM)\n",
           state.rotor_speed_mechanical, final_rpm);
    printf("Current:   %.2f A\n", state.current_dq.q);
    printf("Torque:    %.4f N*m\n", state.torque_electromagnetic);
    printf("Angle:     %.2f rad  (%.0f rev)\n",
           state.rotor_angle_mechanical,
           state.rotor_angle_mechanical / (2.0f * (float)M_PI));
    printf("Efficiency: %.1f %%\n",
           efficiency(mechanical_power(state.torque_electromagnetic,
                    state.rotor_speed_mechanical),
                    electrical_power_dc(24.0f, state.current_dq.q)) * 100.0f);

    printf("\nDC Motor Speed Control simulation complete.\n");
    return 0;
}

/* Simple ramp generator (inline for example clarity) */
static float apply_ramp(float target_ref_unused)
    (void)target_ref_unused;
{
    /* For this example, ramp is handled by speed_ramp_trapezoidal in control.c */
    /* Here we use a simple version */
    static float ramp_setpoint = 0.0f;
    float accel = 500.0f;  /* rad/s^2 */
    float dt = 0.001f;
    float diff = 300.0f - ramp_setpoint;
    float step = accel * dt;
    if (diff > step) ramp_setpoint += step;
    else if (diff < -step) ramp_setpoint -= step;
    else ramp_setpoint = 300.0f;
    return ramp_setpoint;
}
