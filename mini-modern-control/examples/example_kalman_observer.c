#include "modern_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(void)
{
    printf("=== Kalman Filter + Luenberger Observer Example ===\n\n");

    /* Build discrete double integrator with noise */
    mc_ss_discrete_t sys;
    mc_ss_discrete_alloc(2, 1, 1, &sys, 0.01);
    /* A = [1, Ts; 0, 1] */
    mc_matrix_set(&sys.A, 0, 0, 1.0); mc_matrix_set(&sys.A, 0, 1, 0.01);
    mc_matrix_set(&sys.A, 1, 0, 0.0); mc_matrix_set(&sys.A, 1, 1, 1.0);
    mc_matrix_set(&sys.B, 0, 0, 0.0);
    mc_matrix_set(&sys.C, 0, 0, 1.0);

    /* Process and measurement noise */
    mc_matrix_t Q, R, P0;
    mc_vector_t x0, y_meas;
    mc_matrix_alloc(2, 2, &Q); mc_matrix_alloc(1, 1, &R);
    mc_matrix_alloc(2, 2, &P0);
    mc_vector_alloc(2, &x0); mc_vector_alloc(1, &y_meas);

    mc_matrix_set(&Q, 0, 0, 0.01); mc_matrix_set(&Q, 1, 1, 0.01);
    mc_matrix_set(&R, 0, 0, 0.1);
    mc_matrix_identity(&P0);
    mc_vector_set(&x0, 0, 1.0); mc_vector_set(&x0, 1, 0.5);
    mc_vector_set(&y_meas, 0, 1.2);  /* Noisy measurement */

    /* Initialize Kalman filter */
    mc_kalman_filter_t kf;
    mc_kalman_filter_init(&sys, &Q, &R, &x0, &P0, &kf);

    printf("Initial state estimate: [%.4f, %.4f]\n",
           kf.x_hat.data[0], kf.x_hat.data[1]);

    /* Predict */
    mc_vector_t u;
    mc_vector_alloc(1, &u);
    mc_vector_zero(&u);
    mc_kalman_filter_predict(&kf, &u);
    printf("After predict: [%.4f, %.4f]\n", kf.x_hat.data[0], kf.x_hat.data[1]);

    /* Update with measurement */
    mc_kalman_filter_update(&kf, &y_meas);
    printf("After update:  [%.4f, %.4f]\n", kf.x_hat.data[0], kf.x_hat.data[1]);
    printf("Kalman gain:   [%.6f, %.6f]\n",
           mc_matrix_get(&kf.K_kf, 0, 0), mc_matrix_get(&kf.K_kf, 1, 0));

    /* Multiple steps */
    printf("\nRunning 10 predict-update cycles:\n");
    for (int k = 0; k < 10; k++) {
        mc_kalman_filter_predict(&kf, &u);
        y_meas.data[0] = kf.x_hat.data[0] + 0.05 * ((double)rand()/RAND_MAX - 0.5);
        mc_kalman_filter_update(&kf, &y_meas);
        if (k < 3 || k >= 7)
            printf("  k=%d: x_hat=[%.4f, %.4f], P_trace=%.6f\n",
                   k, kf.x_hat.data[0], kf.x_hat.data[1],
                   mc_matrix_trace(&kf.P_kf));
    }

    /* Now design a Luenberger observer for the original continuous system */
    mc_ss_system_t csys;
    mc_ss_alloc(2, 1, 1, &csys);
    mc_matrix_set(&csys.A, 0, 1, 1.0);
    mc_matrix_set(&csys.B, 1, 0, 1.0);
    mc_matrix_set(&csys.C, 0, 0, 1.0);

    double obs_poles[2] = {-5.0, -6.0};  /* Faster than system dynamics */
    mc_luenberger_observer_t obs;
    mc_observer_luenberger_design(&csys, obs_poles, &obs);

    printf("\nLuenberger observer gain L:\n");
    printf("  L = [%.6f, %.6f]^T\n",
           mc_matrix_get(&obs.L, 0, 0), mc_matrix_get(&obs.L, 1, 0));
    printf("  Observer convergent: %s\n", obs.is_convergent ? "YES" : "NO");

    mc_kalman_filter_free(&kf);
    mc_luenberger_observer_free(&obs);
    mc_matrix_free(&Q); mc_matrix_free(&R); mc_matrix_free(&P0);
    mc_vector_free(&x0); mc_vector_free(&y_meas); mc_vector_free(&u);
    mc_ss_free(&csys); mc_ss_discrete_free(&sys);
    printf("\nDone.\n");
    return 0;
}
