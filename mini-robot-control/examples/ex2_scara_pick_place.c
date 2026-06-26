/**
 * ex2_scara_pick_place.c - SCARA robot pick-and-place demo
 */
#include "robot_types.h"
#include "robot_kinematics.h"
#include "robot_math3d.h"
#include "robot_trajectory.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    printf("=== Example 2: SCARA Robot Pick-and-Place ===\n\n");

    robot_model_t model;
    if (robot_model_create_scara(&model) != ROBOT_OK) {
        printf("Failed to create SCARA model\n"); return 1;
    }
    printf("SCARA model: %zu DOF, %zu links\n", model.n_dof, model.n_links);

    /* Define pick and place positions (joint space, approximate) */
    double q_pick[3] = {0.5, -0.3, 0.02};   /* theta1, theta2, z */
    double q_place[3] = {-0.4, 0.6, 0.0};   /* target location */

    /* Create via-point trajectory */
    size_t n_points = 4;
    double via[12] = {
        0.0, 0.0, 0.05,    /* Home (above work area) */
        0.5, -0.3, 0.05,   /* Above pick */
        0.5, -0.3, 0.02,   /* At pick (down) */
        0.5, -0.3, 0.05    /* Lift */
    };

    trajectory_t traj;
    robot_trajectory_alloc(&traj, 3, 100);
    robot_traj_lspb(via, n_points, 3, 0.5, 2.0, 25, &traj);

    printf("Trajectory: %zu points, %.2f s total\n",
           traj.n_points, traj.total_duration);

    /* Forward kinematics for key points */
    mat4_t *T = malloc((model.n_dof + 1) * sizeof(mat4_t));

    robot_fk_compute(&model, q_pick, T);
    vec3_t pick_pos;
    mat4_extract_translation(&T[3], &pick_pos);
    printf("Pick position:  (%.3f, %.3f, %.3f)\n",
           pick_pos.x, pick_pos.y, pick_pos.z);

    robot_fk_compute(&model, q_place, T);
    vec3_t place_pos;
    mat4_extract_translation(&T[3], &place_pos);
    printf("Place position: (%.3f, %.3f, %.3f)\n",
           place_pos.x, place_pos.y, place_pos.z);

    printf("Cartesian distance: %.3f m\n",
           vec3_distance(&pick_pos, &place_pos));

    printf("\nTrajectory via-points:\n");
    size_t i;
    for (i = 0; i < n_points; i++) {
        robot_fk_compute(&model, &via[i * 3], T);
        vec3_t pos;
        mat4_extract_translation(&T[3], &pos);
        printf("  Point %zu: joint=(%.3f,%.3f,%.3f) cart=(%.3f,%.3f,%.3f)\n",
               i, via[i * 3], via[i * 3 + 1], via[i * 3 + 2],
               pos.x, pos.y, pos.z);
    }

    free(T);
    robot_trajectory_free(&traj);
    robot_model_free(&model);
    printf("\nDone.\n");
    return 0;
}