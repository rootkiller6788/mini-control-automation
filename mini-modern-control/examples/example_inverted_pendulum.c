#include "modern_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(void)
{
    printf("=== Inverted Pendulum LQR Control ===\n\n");

    mc_inverted_pendulum_params_t params = {
        .cart_mass     = 0.5,
        .pendulum_mass = 0.2,
        .pendulum_len  = 0.3,
        .gravity       = 9.81,
        .friction_cart = 0.1,
        .friction_pend = 0.0
    };

    mc_ss_system_t sys;
    mc_ss_alloc(4, 1, 2, &sys);
    mc_build_inverted_pendulum(&params, &sys);

    printf("System built: %zu states, %zu inputs, %zu outputs\n",
           sys.n_states, sys.n_inputs, sys.n_outputs);

    /* Check open-loop stability */
    mc_eigenvalues_t eig;
    mc_eigenvalues_compute(&sys.A, &eig);
    printf("\nOpen-loop eigenvalues:\n");
    for (size_t i = 0; i < eig.n_values; i++) {
        printf("  lambda[%zu] = %.4f %+.4fj  (|lambda|=%.4f)\n",
               i, eig.real_part[i], eig.imag_part[i], eig.magnitude[i]);
    }
    printf("Stability: %s\n",
           eig.stability == MC_STABLE ? "STABLE" :
           eig.stability == MC_UNSTABLE ? "UNSTABLE" : "MARGINAL");

    /* LQR design */
    double q_weights[4] = {10.0, 1.0, 100.0, 10.0};
    double r_weights[1] = {0.1};
    mc_lqr_solution_t lqr;
    int ret = mc_inverted_pendulum_lqr(&params, q_weights, r_weights, NULL, &lqr);

    printf("\nLQR Design: %s (iterations=%zu, residual=%.2e)\n",
           ret == 0 ? "CONVERGED" : "FAILED",
           lqr.iterations, lqr.residual);

    if (ret == 0) {
        printf("\nFeedback gain K:\n");
        for (size_t j = 0; j < 4; j++)
            printf("  K[%zu] = %+.6f\n", j, mc_matrix_get(&lqr.K, 0, j));

        printf("\nClosed-loop eigenvalues (A - B*K):\n");
        for (size_t i = 0; i < lqr.n_poles && i < 4; i++)
            printf("  lambda_cl[%zu] = %.4f %+.4fj\n",
                   i, lqr.closed_loop_poles_real[i], lqr.closed_loop_poles_imag[i]);
        printf("Closed-loop stable: %s\n", lqr.is_stabilizing ? "YES" : "NO");

        /* Step response */
        mc_state_feedback_t fb;
        fb.n_states = 4; fb.n_inputs = 1;
        mc_matrix_alloc(1, 4, &fb.K);
        for (size_t j = 0; j < 4; j++)
            mc_matrix_set(&fb.K, 0, j, mc_matrix_get(&lqr.K, 0, j));

        mc_step_response_t resp;
        mc_step_response(&sys, 0, 10.0, 0.001, &resp);
        printf("\nStep Response (cart position, output 0):\n");
        printf("  Rise time:     %.3f s\n", resp.rise_time);
        printf("  Settling time: %.3f s\n", resp.settling_time);
        printf("  Overshoot:     %.1f%%\n", resp.overshoot_percent);
        printf("  DC gain:       %.4f\n", resp.dc_gain);

        mc_state_feedback_free(&fb);
    }

    mc_eigenvalues_free(&eig);
    mc_lqr_solution_free(&lqr);
    mc_ss_free(&sys);

    printf("\nDone.\n");
    return 0;
}
