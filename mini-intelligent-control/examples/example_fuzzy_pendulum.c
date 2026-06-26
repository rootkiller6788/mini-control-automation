#include "intelligent_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Fuzzy control of an inverted pendulum (L6 canonical problem) */

int main(void) {
    printf("=== Fuzzy Inverted Pendulum Control ===\n\n");
    /* Create fuzzy system: 2 inputs (angle, angular velocity), 1 output (force) */
    ic_fuzzy_system_t fis;
    ic_fuzzy_init(&fis, 2, 1, 0);
    ic_fuzzy_add_input(&fis, "angle", -0.5, 0.5);
    ic_fuzzy_add_input(&fis, "angvel", -2.0, 2.0);
    ic_fuzzy_add_output(&fis, "force", -10.0, 10.0);
    /* Membership functions for angle */
    ic_mf_triangular_t ang_N = {-0.5, -0.5, 0.0, "N"};
    ic_mf_triangular_t ang_Z = {-0.1, 0.0, 0.1, "Z"};
    ic_mf_triangular_t ang_P = {0.0, 0.5, 0.5, "P"};
    ic_fuzzy_add_mf(&fis.inputs[0], IC_MF_TRIANGULAR, &ang_N, "N");
    ic_fuzzy_add_mf(&fis.inputs[0], IC_MF_TRIANGULAR, &ang_Z, "Z");
    ic_fuzzy_add_mf(&fis.inputs[0], IC_MF_TRIANGULAR, &ang_P, "P");
    /* Membership functions for angular velocity */
    ic_mf_triangular_t vel_N = {-2.0, -2.0, 0.0, "N"};
    ic_mf_triangular_t vel_Z = {-0.5, 0.0, 0.5, "Z"};
    ic_mf_triangular_t vel_P = {0.0, 2.0, 2.0, "P"};
    ic_fuzzy_add_mf(&fis.inputs[1], IC_MF_TRIANGULAR, &vel_N, "N");
    ic_fuzzy_add_mf(&fis.inputs[1], IC_MF_TRIANGULAR, &vel_Z, "Z");
    ic_fuzzy_add_mf(&fis.inputs[1], IC_MF_TRIANGULAR, &vel_P, "P");
    /* Membership functions for output force */
    ic_mf_triangular_t f_NB = {-10.0, -10.0, -3.0, "NB"};
    ic_mf_triangular_t f_NS = {-8.0, -3.0, 0.0, "NS"};
    ic_mf_triangular_t f_Z  = {-3.0, 0.0, 3.0, "Z"};
    ic_mf_triangular_t f_PS = {0.0, 3.0, 8.0, "PS"};
    ic_mf_triangular_t f_PB = {3.0, 10.0, 10.0, "PB"};
    ic_fuzzy_add_mf(&fis.outputs[0], IC_MF_TRIANGULAR, &f_NB, "NB");
    ic_fuzzy_add_mf(&fis.outputs[0], IC_MF_TRIANGULAR, &f_NS, "NS");
    ic_fuzzy_add_mf(&fis.outputs[0], IC_MF_TRIANGULAR, &f_Z, "Z");
    ic_fuzzy_add_mf(&fis.outputs[0], IC_MF_TRIANGULAR, &f_PS, "PS");
    ic_fuzzy_add_mf(&fis.outputs[0], IC_MF_TRIANGULAR, &f_PB, "PB");
    /* Define rules */
    size_t rules[9][2] = {{0,0},{0,1},{0,2},{1,0},{1,1},{1,2},{2,0},{2,1},{2,2}};
    size_t consequents[9] = {0, 1, 2, 1, 2, 3, 2, 3, 4};
    for (int i = 0; i < 9; i++)
        ic_fuzzy_add_rule(&fis, rules[i], consequents[i], 1.0, 0);
    /* Simulate pendulum */
    double theta = 0.2, theta_dot = 0.0;
    double dt = 0.01;
    printf("Simulating inverted pendulum with fuzzy control...\n");
    printf("Initial: theta=%.3f rad\n", theta);
    for (int step = 0; step < 500; step++) {
        double inputs[2] = {theta, theta_dot};
        double outputs[1];
        ic_fuzzy_control_step(&fis, inputs, outputs);
        double F = outputs[0];
        /* Pendulum dynamics: theta_ddot = (g/l)*sin(theta) + F/(m*l^2) */
        double g = 9.81, l = 0.5, m = 0.2;
        double theta_ddot = (g/l)*sin(theta) + F/(m*l*l);
        theta_dot += theta_ddot * dt;
        theta += theta_dot * dt;
        if (step % 100 == 0)
            printf("  t=%.1f: theta=%.4f, theta_dot=%.4f, F=%.3f\n",
                   step*dt, theta, theta_dot, F);
    }
    printf("Final: theta=%.4f rad\n", theta);
    ic_fuzzy_free(&fis);
    printf("\nFuzzy pendulum control complete.\n");
    return 0;
}
