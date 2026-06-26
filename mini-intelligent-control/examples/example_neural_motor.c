#include "intelligent_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Neural network control of a DC motor (L6 canonical problem) */

int main(void) {
    printf("=== Neural Network DC Motor Control ===\n\n");
    /* Create neural network: 2 inputs (speed error, error derivative),
       4 hidden, 1 output (voltage) */
    size_t layers[] = {2, 4, 1};
    ic_neural_network_t nn;
    ic_nn_create(&nn, layers, 3, IC_ACT_TANH, IC_ACT_LINEAR);
    /* Training data: linear mapping from error states to control */
    double inputs[20][2], targets[20][1];
    for (int i = 0; i < 20; i++) {
        double e = (i - 10) * 0.1;
        double de = (i - 10) * 0.05;
        inputs[i][0] = e;
        inputs[i][1] = de;
        targets[i][0] = 5.0 * e + 0.5 * de;
    }
    /* Train network */
    const double *in_ptrs[20], *tgt_ptrs[20];
    for (int i = 0; i < 20; i++) {
        in_ptrs[i] = inputs[i];
        tgt_ptrs[i] = targets[i];
    }
    printf("Training neural network...\n");
    ic_nn_train_batch(&nn, in_ptrs, tgt_ptrs, 20, 50);
    /* Simulate DC motor */
    double omega = 0.0, omega_ref = 100.0;
    double e_prev = omega_ref - omega;
    double dt = 0.001;
    printf("Simulating DC motor (ref=%.1f rad/s)...\n", omega_ref);
    for (int step = 0; step < 1000; step++) {
        double e = omega_ref - omega;
        double de = (e - e_prev) / dt;
        double nn_in[2] = {e / 100.0, de / 100.0};
        double nn_out[1];
        ic_nn_forward(&nn, nn_in, nn_out);
        double voltage = nn_out[0] * 10.0;
        if (voltage > 24.0) voltage = 24.0;
        if (voltage < -24.0) voltage = -24.0;
        /* Motor dynamics: J*dω/dt + b*ω = K*V */
        double J = 0.01, b = 0.1, K = 0.05;
        double omega_dot = (K * voltage - b * omega) / J;
        omega += omega_dot * dt;
        e_prev = e;
        if (step % 200 == 0)
            printf("  t=%.2f: omega=%.2f, V=%.2f\n", step*dt, omega, voltage);
    }
    printf("Final speed: %.2f rad/s (target: %.1f)\n", omega, omega_ref);
    ic_nn_free(&nn);
    printf("\nNeural motor control complete.\n");
    return 0;
}
