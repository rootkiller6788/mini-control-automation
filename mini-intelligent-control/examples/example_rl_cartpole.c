#include "intelligent_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Q-Learning for Cart-Pole balancing (L6 canonical RL problem) */

#define N_STATES 16
#define N_ACTIONS 2

static int discretize_state(double theta, double theta_dot) {
    int t_idx = (int)((theta + 0.2) / 0.1);
    if (t_idx < 0) t_idx = 0; if (t_idx > 3) t_idx = 3;
    int td_idx = (int)((theta_dot + 1.0) / 0.5);
    if (td_idx < 0) td_idx = 0; if (td_idx > 3) td_idx = 3;
    return t_idx * 4 + td_idx;
}

int main(void) {
    printf("=== Q-Learning Cart-Pole Control ===\n\n");
    ic_qlearning_t ql;
    ic_ql_init(&ql, N_STATES, N_ACTIONS, 0.1, 0.95, 0.2);
    printf("Training Q-Learning agent...\n");
    int episodes = 200;
    int total_steps = 0;
    for (int ep = 0; ep < episodes; ep++) {
        double theta = 0.05, theta_dot = 0.0;
        int state = discretize_state(theta, theta_dot);
        int steps = 0;
        for (steps = 0; steps < 200; steps++) {
            size_t action = ic_ql_select_action(&ql, (size_t)state);
            double F = (action == 0) ? -10.0 : 10.0;
            /* Simplified cart-pole dynamics */
            double theta_ddot = 15.0 * theta + F * 0.1;
            theta_dot += theta_ddot * 0.02;
            theta += theta_dot * 0.02;
            int next_state = discretize_state(theta, theta_dot);
            double reward = (fabs(theta) < 0.2) ? 1.0 : -1.0;
            ic_ql_update(&ql, (size_t)state, action, reward, (size_t)next_state);
            state = next_state;
            if (fabs(theta) > 0.5) break;
        }
        total_steps += steps;
        if (ep % 50 == 0)
            printf("  Episode %d: %d steps, epsilon=%.3f\n", ep, steps, ql.epsilon);
    }
    printf("\nTraining complete: avg %.1f steps/episode\n",
           (double)total_steps / episodes);
    /* Test learned policy */
    printf("Testing learned policy...\n");
    double theta = 0.1, theta_dot = 0.0;
    int state = discretize_state(theta, theta_dot);
    for (int step = 0; step < 100; step++) {
        size_t best_a = ic_ql_best_action(&ql, (size_t)state);
        double F = (best_a == 0) ? -10.0 : 10.0;
        double theta_ddot = 15.0 * theta + F * 0.1;
        theta_dot += theta_ddot * 0.02;
        theta += theta_dot * 0.02;
        state = discretize_state(theta, theta_dot);
        if (step % 20 == 0)
            printf("  t=%.2f: theta=%.4f\n", step*0.02, theta);
        if (fabs(theta) > 0.5) { printf("  Pole fell!\n"); break; }
    }
    ic_ql_free(&ql);
    printf("\nRL cart-pole control complete.\n");
    return 0;
}
