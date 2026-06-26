/**
 * ex3_mobile_robot.c - Differential-drive mobile robot trajectory tracking
 */
#include "robot_types.h"
#include "robot_trajectory.h"
#include "robot_math3d.h"
#include "robot_kinematics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int main(void) {
    printf("=== Example 3: Differential-Drive Mobile Robot ===\n\n");

    /* Unicycle model: x_dot=v*cos(theta), y_dot=v*sin(theta), theta_dot=omega */
    double x = 0.0, y = 0.0, theta = 0.0;
    double wheelbase = 0.3;     /* Distance between wheels [m] */
    double wheel_radius = 0.05; /* [m] */
    double dt = 0.05;           /* Time step [s] */
    size_t n_steps = 200;

    /* Desired path: figure-8 pattern */
    double *traj_x = calloc(n_steps, sizeof(double));
    double *traj_y = calloc(n_steps, sizeof(double));
    double *traj_theta = calloc(n_steps, sizeof(double));

    printf("Tracking figure-8 path for %.1f seconds...\n", n_steps * dt);

    size_t i;
    for (i = 0; i < n_steps; i++) {
        double t = i * dt;

        /* Figure-8: x = sin(omega*t), y = sin(2*omega*t) */
        double omega = 0.5;
        double x_des = sin(omega * t);
        double y_des = 0.5 * sin(2.0 * omega * t);

        /* Desired heading: tangent direction */
        double dx = omega * cos(omega * t);
        double dy = omega * cos(2.0 * omega * t);
        double theta_des = atan2(dy, dx);

        /* Simple P controller */
        double kp_pos = 2.0, kp_head = 1.0;
        double ex = x_des - x;
        double ey = y_des - y;
        double v_cmd = kp_pos * (ex * cos(theta) + ey * sin(theta));
        double omega_cmd = kp_head * atan2(sin(theta_des - theta),
                                           cos(theta_des - theta));

        /* Wheel velocities from inverse kinematics */
        double w_left = (v_cmd - omega_cmd * wheelbase / 2.0) / wheel_radius;
        double w_right = (v_cmd + omega_cmd * wheelbase / 2.0) / wheel_radius;

        /* Forward kinematics (update pose) */
        double v_actual = (w_left + w_right) * wheel_radius / 2.0;
        double omega_actual = (w_right - w_left) * wheel_radius / wheelbase;

        x += v_actual * cos(theta) * dt;
        y += v_actual * sin(theta) * dt;
        theta += omega_actual * dt;

        traj_x[i] = x;
        traj_y[i] = y;
        traj_theta[i] = theta;

        if (i % 20 == 0) {
            printf(" t=%.1fs: pos=(%.3f,%.3f) hdg=%.2f  err=(%.3f,%.3f)\n",
                   t, x, y, theta, x_des - x, y_des - y);
        }
    }

    /* Compute tracking error */
    double rms_err = 0.0;
    for (i = 0; i < n_steps; i++) {
        double t = i * dt;
        double x_des = sin(0.5 * t);
        double y_des = 0.5 * sin(t);
        double err = (traj_x[i] - x_des) * (traj_x[i] - x_des)
                   + (traj_y[i] - y_des) * (traj_y[i] - y_des);
        rms_err += err;
    }
    rms_err = sqrt(rms_err / n_steps);
    printf("\nRMS tracking error: %.4f m\n", rms_err);

    free(traj_x); free(traj_y); free(traj_theta);
    printf("\nDone.\n");
    return 0;
}