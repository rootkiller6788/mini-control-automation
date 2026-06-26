#include "modern_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(void)
{
    printf("=== DC Motor LQR Speed Control ===\n\n");

    mc_dc_motor_params_t params = {
        .R_a = 1.0, .L_a = 0.5, .K_b = 0.01,
        .K_t = 0.01, .J = 0.01, .B = 0.001, .gear_ratio = 1.0
    };

    mc_ss_system_t sys;
    mc_ss_alloc(3, 1, 1, &sys);
    mc_build_dc_motor(&params, &sys);

    printf("DC Motor model: A (3x3), B (3x1), C (1x3)\n");

    /* Open-loop eigenvalues */
    mc_eigenvalues_t eig;
    mc_eigenvalues_compute(&sys.A, &eig);
    printf("Open-loop poles:\n");
    for (size_t i = 0; i < eig.n_values; i++)
        printf("  s[%zu] = %.4f %+.4fj\n", i, eig.real_part[i], eig.imag_part[i]);

    /* LQR velocity control */
    double qw[3] = {0.0, 100.0, 1.0};
    double rw[1] = {0.01};
    mc_lqr_solution_t lqr;
    int ret = mc_dc_motor_lqr(&params, qw, rw, NULL, &lqr);

    printf("\nLQR velocity control: %s (iter=%zu, conv=%d, stab=%d)\n",
           ret==0?"OK":"FAIL", lqr.iterations, lqr.convergence, lqr.is_stabilizing);
    if (ret == 0) {
        printf("  K = [%.6f, %.6f, %.6f]\n",
               mc_matrix_get(&lqr.K, 0, 0),
               mc_matrix_get(&lqr.K, 0, 1),
               mc_matrix_get(&lqr.K, 0, 2));
        printf("  J_min = %.6f\n", lqr.j_min);

        mc_state_feedback_t fb;
        fb.n_states = 3; fb.n_inputs = 1;
        mc_matrix_alloc(1, 3, &fb.K);
        for (size_t j = 0; j < 3; j++)
            mc_matrix_set(&fb.K, 0, j, mc_matrix_get(&lqr.K, 0, j));

        mc_step_response_t resp;
        mc_step_response(&sys, 0, 5.0, 0.001, &resp);
        printf("  Step response: Tr=%.3fs, Ts=%.3fs, Mp=%.1f%%\n",
               resp.rise_time, resp.settling_time, resp.overshoot_percent);

        mc_state_feedback_free(&fb);
    }

    mc_eigenvalues_free(&eig);
    mc_lqr_solution_free(&lqr);
    mc_ss_free(&sys);
    printf("\nDone.\n");
    return 0;
}
