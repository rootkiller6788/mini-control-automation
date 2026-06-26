/**
 * pmsm_foc.c
 * L6 Canonical Problem: PMSM Field-Oriented Control (FOC) simulation
 *
 * Full FOC chain simulation:
 *   Clarke -> Park -> PI current control -> Inverse Park -> SVPWM
 * with MTPA optimization and field weakening at high speed.
 *
 * This demonstrates the complete FOC algorithm as used in
 * modern industrial servo drives and electric vehicle traction motors.
 */

#include <stdio.h>
#include <math.h>
#include "motor_model.h"
#include "motor_control.h"
#include "pwm_modulation.h"
#include "transforms.h"
#include "motor_types.h"

int main(void)
{
    printf("========================================\n");
    printf(" PMSM Field-Oriented Control (FOC)\n");
    printf("========================================\n\n");

    /* PMSM parameters: 200W, 4-pole, 48V bus */
    pmsm_params_t motor;
    pmsm_params_init(&motor);
    motor.rs = 0.3f;
    motor.ld = 0.0008f;
    motor.lq = 0.0010f;
    motor.flux_linkage = 0.05f;
    motor.pole_pairs = 2.0f;
    motor.rotor_inertia = 0.00005f;
    motor.friction_coefficient = 0.00001f;
    motor.max_dc_voltage = 48.0f;

    printf("Motor: %d poles, Rs=%.2f Ohm, Ld=%.0f uH, Lq=%.0f uH\n",
            (int)(2*motor.pole_pairs), motor.rs,
            motor.ld*1e6f, motor.lq*1e6f);
    printf("Flux: %.0f mWb, J=%.0f g*cm^2\n",
            motor.flux_linkage*1e3f, motor.rotor_inertia*1e4f);

    /* Configure cascaded control */
    cascaded_control_cfg_t cfg;
    cascaded_control_cfg_init(&cfg, CONTROL_MODE_SPEED);
    cfg.current_limit = 5.0f;

    /* Initialize motor state */
    motor_state_t state;
    motor_state_init(&state);
    state.dc_bus_voltage = 48.0f;

    float dt = 0.0001f;    /* 100 us = 10 kHz FOC loop */
    float sim_time = 0.5f;
    int steps = (int)(sim_time / dt);

    printf("\nSimulation: dt=%.0f us, %d steps, Vdc=%.0fV\n",
            dt*1e6f, steps, state.dc_bus_voltage);
    printf("Control: SVPWM @ 10 kHz, cascaded PI (current+speed)\n\n");
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s\n",
           "Time[s]", "Wm[r/s]", "Id[A]", "Iq[A]", "Te[N*m]", "Vd[V]");

    /* Speed profile: ramp from 0 to 200 rad/s */
    float speed_target = 200.0f;   /* ~1910 RPM */

    for (int k = 0; k < steps; k++) {
        float t = k * dt;

        /* Ramped speed reference (position profile not active in speed mode) */
        float speed_ref;
        if (t < 0.1f) {
            speed_ref = t / 0.1f * speed_target;
        } else {
            speed_ref = speed_target;
        }

        /* Measure currents (from state) */
        float id_meas = state.current_dq.d;
        float iq_meas = state.current_dq.q;

        /* Speed control: generates Iq reference */
        float iq_ref;
        speed_control(speed_ref, state.rotor_speed_mechanical, &iq_ref, &cfg, dt);

        /* MTPA: compute optimal Id for given Iq */
        float id_ref = pmsm_mtpa_id(iq_ref, &motor);

        /* Check field weakening if speed is high */
        float vmag = dq_magnitude(state.voltage_dq_ref);
        if (vmag > pmsm_field_weakening_id(iq_ref, state.rotor_speed_electrical,
                                            state.dc_bus_voltage, &motor)) {
            /* The field weakening function returns the FW Id directly */
            id_ref = pmsm_field_weakening_id(iq_ref, state.rotor_speed_electrical,
                                              state.dc_bus_voltage, &motor);
        }

        /* Current control: generates Vd, Vq */
        float vd, vq;
        foc_current_control(id_ref, iq_ref, id_meas, iq_meas,
                            state.rotor_speed_electrical, &vd, &vq,
                            &cfg, &motor, dt);

        /* SVPWM: Vdq -> duty cycles */
        /* First inverse Park to alpha-beta */
        float v_alpha, v_beta;
        inverse_park_direct(vd, vq, state.rotor_angle_electrical,
                            &v_alpha, &v_beta);

        pwm_duty_t duty;
        bool svpwm_ok = svpwm_compute(v_alpha, v_beta, state.dc_bus_voltage,
                                       1.0f/10000.0f, &duty);

        /* Integrate motor dynamics */
        float load_torque = 0.05f;  /* small constant load */
        pmsm_rk4_step(&state, vd, vq, load_torque, dt, &motor);

        /* Print every 25 ms */
        if (k % 250 == 0) {
            /* float rpm = */ rad_per_sec_to_rpm(state.rotor_speed_mechanical);
            printf("%8.3f  %10.1f  %10.3f  %10.3f  %10.4f  %10.2f\n",
                   t, state.rotor_speed_mechanical,
                   id_meas, iq_meas,
                   state.torque_electromagnetic, vd);
            if (svpwm_ok) {
                /* duties OK */
            }
        }
    }

    /* Results */
    printf("\n=== Final State ===\n");
    printf("Speed:     %.1f rad/s  (%.0f RPM)\n",
           state.rotor_speed_mechanical,
           rad_per_sec_to_rpm(state.rotor_speed_mechanical));
    printf("Id:        %.3f A\n", state.current_dq.d);
    printf("Iq:        %.3f A\n", state.current_dq.q);
    printf("Is:        %.3f A  (%.1f %% of rated)\n",
           dq_magnitude(state.current_dq),
           dq_magnitude(state.current_dq) / motor.rated_current * 100.0f);
    printf("Torque:    %.4f N*m\n", state.torque_electromagnetic);
    printf("Power:     %.1f W  (mechanical)\n",
           mechanical_power(state.torque_electromagnetic,
                           state.rotor_speed_mechanical));
    printf("Angle:     %.2f rad  (%.1f rev)\n",
           state.rotor_angle_mechanical,
           state.rotor_angle_mechanical / (2.0f * (float)M_PI));

    float mi = modulation_index(dq_magnitude(state.voltage_dq_ref),
                                 state.dc_bus_voltage);
    printf("Mod Index: %.3f  (%s)\n", mi,
           mi < 0.907f ? "linear" : mi < 0.952f ? "OV-I" : "OV-II/six-step");

    printf("\nPMSM FOC simulation complete.\n");
    return 0;
}
