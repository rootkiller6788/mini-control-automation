/**
 * ex1_planar_2dof_control.c - Planar 2-DOF robot PD+gravity control demo
 */
#include "robot_types.h"
#include "robot_control.h"
#include "robot_kinematics.h"
#include "robot_dynamics.h"
#include "robot_trajectory.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int main(void) {
    printf("=== Example 1: Planar 2-DOF Robot PD+Gravity Control ===\n\n");

    robot_model_t model;
    if (robot_model_create_planar_2dof(&model) != ROBOT_OK) {
        printf("Failed to create robot model\n"); return 1;
    }

    robot_state_t state;
    if (robot_state_alloc(&model, &state) != ROBOT_OK) {
        printf("Failed to allocate state\n"); return 1;
    }

    double q_start[2] = {0.0, 0.0};
    double q_goal[2] = {1.0, -0.5};
    double kp[2] = {100.0, 100.0}, kd[2] = {20.0, 20.0};
    double dt = model.control_period;

    size_t n_steps = 200;
    double *q_hist = calloc(n_steps * 2, sizeof(double));
    double *tau_hist = calloc(n_steps * 2, sizeof(double));

    printf("Moving from (%.2f, %.2f) to (%.2f, %.2f)\n",
           q_start[0], q_start[1], q_goal[0], q_goal[1]);
    printf("Gains: Kp=%.0f, Kd=%.0f\n\n", kp[0], kd[0]);

    memcpy(state.joint_state.q, q_start, 2 * sizeof(double));
    memset(state.joint_state.qd, 0, 2 * sizeof(double));

    trajectory_t traj;
    robot_trajectory_alloc(&traj, 2, n_steps);
    robot_traj_quintic_rest_to_rest(q_start, q_goal, 2.0, n_steps, 2, &traj);

    printf(" t[s]   q1     q2     q1_des  q2_des  tau1    tau2\n");
    printf("-------------------------------------------------------\n");

    size_t i;
    for (i = 0; i < n_steps; i++) {
        double *q_des = traj.points[i].q;
        double *qd_des = traj.points[i].qd;

        robot_pd_gravity_control(&model, state.joint_state.q,
                                 state.joint_state.qd, q_des, kp, kd,
                                 state.joint_state.tau);

        /* Simple Euler integration */
        double *qdd = malloc(2 * sizeof(double));
        robot_dynamics_forward(&model, state.joint_state.q,
                               state.joint_state.qd, state.joint_state.tau, qdd);
        size_t j;
        for (j = 0; j < 2; j++) {
            state.joint_state.qd[j] += qdd[j] * dt;
            state.joint_state.q[j] += state.joint_state.qd[j] * dt;
        }
        free(qdd);

        q_hist[i * 2] = state.joint_state.q[0];
        q_hist[i * 2 + 1] = state.joint_state.q[1];
        tau_hist[i * 2] = state.joint_state.tau[0];
        tau_hist[i * 2 + 1] = state.joint_state.tau[1];

        if (i % 20 == 0) {
            printf(" %.2f   %.3f  %.3f  %.3f   %.3f   %.3f  %.3f\n",
                   traj.points[i].time,
                   state.joint_state.q[0], state.joint_state.q[1],
                   q_des[0], q_des[1],
                   state.joint_state.tau[0], state.joint_state.tau[1]);
        }
    }

    control_performance_t perf;
    robot_control_evaluate(&model, q_hist, traj.points[0].q,
                           tau_hist, n_steps, dt, &perf);
    printf("\nPerformance:\n");
    printf("  RMS tracking error: %.6f rad\n", perf.rms_tracking_error);
    printf("  Max tracking error: %.6f rad\n", perf.max_tracking_error);

    robot_trajectory_free(&traj);
    robot_state_free(&state);
    robot_model_free(&model);
    free(q_hist); free(tau_hist);
    printf("\nDone.\n");
    return 0;
}