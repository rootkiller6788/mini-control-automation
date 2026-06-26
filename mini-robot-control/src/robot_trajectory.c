/**
 * @file    robot_trajectory.c
 * @brief   L5-L6: Trajectory planning and execution.
 */
#include "robot_trajectory.h"
#include "robot_math3d.h"
#include "robot_kinematics.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

int robot_trajectory_alloc(trajectory_t *traj, size_t n_dof, size_t max_points) {
    if (!traj || n_dof == 0) return ROBOT_ERR_INVALID_PARAM;
    memset(traj, 0, sizeof(*traj));
    traj->n_dof = n_dof; traj->max_points = max_points;
    traj->points = calloc(max_points, sizeof(trajectory_point_t));
    if (!traj->points) return ROBOT_ERR_MEMORY;
    size_t i;
    for (i = 0; i < max_points; i++) {
        traj->points[i].q = calloc(n_dof, sizeof(double));
        traj->points[i].qd = calloc(n_dof, sizeof(double));
        traj->points[i].qdd = calloc(n_dof, sizeof(double));
        if (!traj->points[i].q || !traj->points[i].qd || !traj->points[i].qdd) {
            robot_trajectory_free(traj); return ROBOT_ERR_MEMORY;
        }
    }
    return ROBOT_OK;
}

void robot_trajectory_free(trajectory_t *traj) {
    if (!traj) return;
    size_t i;
    for (i = 0; i < traj->max_points; i++) {
        free(traj->points[i].q); free(traj->points[i].qd);
        free(traj->points[i].qdd); free(traj->points[i].qddd);
    }
    free(traj->points);
    memset(traj, 0, sizeof(*traj));
}

int robot_traj_cubic(const double *q0, const double *qf,
                     const double *v0, const double *vf,
                     double T, size_t n_steps, size_t n_dof,
                     trajectory_t *traj) {
    if (!q0 || !qf || !v0 || !vf || !traj || n_steps < 2) return ROBOT_ERR_INVALID_PARAM;
    traj->n_dof = n_dof; traj->n_points = n_steps;
    traj->type = TRAJ_TYPE_CUBIC_POLY; traj->total_duration = T;
    size_t i, j;
    for (i = 0; i < n_steps; i++) {
        double t = (double)i / (n_steps - 1) * T;
        double t2 = t * t, t3 = t2 * t;
        for (j = 0; j < n_dof; j++) {
            double dq = qf[j] - q0[j];
            double a0 = q0[j], a1 = v0[j];
            double a2 = (3.0 * dq / T - 2.0 * v0[j] - vf[j]) / T;
            double a3 = (-2.0 * dq / T + v0[j] + vf[j]) / (T * T);
            traj->points[i].q[j] = a0 + a1 * t + a2 * t2 + a3 * t3;
            traj->points[i].qd[j] = a1 + 2.0 * a2 * t + 3.0 * a3 * t2;
            traj->points[i].qdd[j] = 2.0 * a2 + 6.0 * a3 * t;
        }
        traj->points[i].time = t;
    }
    return ROBOT_OK;
}

int robot_traj_quintic_rest_to_rest(const double *q0, const double *qf,
                                    double T, size_t n_steps, size_t n_dof,
                                    trajectory_t *traj) {
    if (!q0 || !qf || !traj || n_steps < 2) return ROBOT_ERR_INVALID_PARAM;
    traj->n_dof = n_dof; traj->n_points = n_steps;
    traj->type = TRAJ_TYPE_QUINTIC_POLY; traj->total_duration = T;
    size_t i, j;
    for (i = 0; i < n_steps; i++) {
        double tau = (double)i / (n_steps - 1);
        double t3 = tau * tau * tau, t4 = t3 * tau, t5 = t4 * tau;
        double s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
        double sd = (30.0 * tau * tau - 60.0 * t3 + 30.0 * t4) / T;
        double sdd = (60.0 * tau - 180.0 * tau * tau + 120.0 * t3) / (T * T);
        for (j = 0; j < n_dof; j++) {
            traj->points[i].q[j] = q0[j] + (qf[j] - q0[j]) * s;
            traj->points[i].qd[j] = (qf[j] - q0[j]) * sd;
            traj->points[i].qdd[j] = (qf[j] - q0[j]) * sdd;
        }
        traj->points[i].time = tau * T;
    }
    return ROBOT_OK;
}

int robot_traj_quintic_general(const double *q0, const double *qf,
                               const double *v0, const double *vf,
                               const double *a0, const double *af,
                               double T, size_t n_steps, size_t n_dof,
                               trajectory_t *traj) {
    if (!q0 || !qf || !v0 || !vf || !a0 || !af || !traj) return ROBOT_ERR_NULL_POINTER;
    traj->n_dof = n_dof; traj->n_points = n_steps;
    traj->type = TRAJ_TYPE_QUINTIC_POLY; traj->total_duration = T;
    size_t i, j;
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    for (j = 0; j < n_dof; j++) {
        double c[6];
        c[0] = q0[j]; c[1] = v0[j]; c[2] = a0[j] * 0.5;
        double dq = qf[j] - c[0] - c[1] * T - c[2] * T2;
        double dv = vf[j] - c[1] - 2.0 * c[2] * T;
        double da = af[j] - 2.0 * c[2];
        double A[9] = {T3, T4, T5, 3.0*T2, 4.0*T3, 5.0*T4, 6.0*T, 12.0*T2, 20.0*T3};
        double detA = A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
        if (fabs(detA) < 1e-15) { c[3]=c[4]=c[5]=0.0; }
        else {
            double id = 1.0/detA;
            c[3] = ( (A[4]*A[8]-A[5]*A[7])*dq + (A[2]*A[7]-A[1]*A[8])*dv + (A[1]*A[5]-A[2]*A[4])*da ) * id;
            c[4] = ( (A[5]*A[6]-A[3]*A[8])*dq + (A[0]*A[8]-A[2]*A[6])*dv + (A[2]*A[3]-A[0]*A[5])*da ) * id;
            c[5] = ( (A[3]*A[7]-A[4]*A[6])*dq + (A[1]*A[6]-A[0]*A[7])*dv + (A[0]*A[4]-A[1]*A[3])*da ) * id;
        }
        for (i = 0; i < n_steps; i++) {
            double t = (double)i/(n_steps-1)*T, t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
            traj->points[i].q[j] = c[0]+c[1]*t+c[2]*t2+c[3]*t3+c[4]*t4+c[5]*t5;
            traj->points[i].qd[j] = c[1]+2*c[2]*t+3*c[3]*t2+4*c[4]*t3+5*c[5]*t4;
            traj->points[i].qdd[j] = 2*c[2]+6*c[3]*t+12*c[4]*t2+20*c[5]*t3;
        }
    }
    for (i = 0; i < n_steps; i++) traj->points[i].time = (double)i/(n_steps-1)*T;
    return ROBOT_OK;
}

int robot_traj_trapezoidal(const double *q0, const double *qf,
                           double v_max, double a_max,
                           size_t n_steps, size_t n_dof,
                           trajectory_t *traj) {
    if (!q0 || !qf || !traj) return ROBOT_ERR_NULL_POINTER;
    double max_disp = 0.0; size_t j;
    for (j = 0; j < n_dof; j++) { double d = fabs(qf[j]-q0[j]); if (d > max_disp) max_disp = d; }
    if (max_disp < 1e-10) max_disp = 1.0;
    double t_a = v_max / a_max;
    double d_accel = 0.5 * a_max * t_a * t_a;
    double d_cruise = max_disp - 2.0 * d_accel;
    double T;
    if (d_cruise < 0.0) { t_a = sqrt(max_disp/a_max); T = 2.0*t_a; d_cruise = 0.0; }
    else { T = 2.0*t_a + d_cruise/v_max; }
    traj->n_dof = n_dof; traj->n_points = n_steps;
    traj->type = TRAJ_TYPE_TRAPEZOIDAL; traj->total_duration = T;
    traj->max_velocity = v_max; traj->max_acceleration = a_max;
    size_t i;
    for (i = 0; i < n_steps; i++) {
        double t = (double)i/(n_steps-1)*T, s;
        if (t < t_a) s = 0.5*a_max*t*t/max_disp;
        else if (t < T-t_a) s = (d_accel + v_max*(t-t_a))/max_disp;
        else { double tr = T-t; s = 1.0 - 0.5*a_max*tr*tr/max_disp; }
        if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0;
        for (j = 0; j < n_dof; j++) traj->points[i].q[j] = q0[j] + s*(qf[j]-q0[j]);
        traj->points[i].time = t;
    }
    return ROBOT_OK;
}

int robot_traj_s_curve(const double *q0, const double *qf,
                       double v_max, double a_max, double j_max,
                       size_t n_steps, size_t n_dof,
                       trajectory_t *traj) {
    if (!q0 || !qf || !traj) return ROBOT_ERR_NULL_POINTER;
    double max_disp = 0.0; size_t j;
    for (j = 0; j < n_dof; j++) { double d = fabs(qf[j]-q0[j]); if (d > max_disp) max_disp = d; }
    if (max_disp < 1e-10) max_disp = 1.0;
    double tj = a_max/j_max, ta = v_max/a_max + tj;
    double d_acc = a_max*(ta*ta - tj*tj);
    double d_cruise = max_disp - 2.0*d_acc;
    double T;
    if (d_cruise < 0.0) T = 4.0*tj;
    else T = 4.0*tj + 2.0*(ta-2.0*tj) + d_cruise/v_max;
    traj->n_dof = n_dof; traj->n_points = n_steps;
    traj->type = TRAJ_TYPE_S_CURVE; traj->total_duration = T;
    traj->max_jerk = j_max;
    size_t i;
    for (i = 0; i < n_steps; i++) {
        double s = (double)i/(n_steps-1);
        double s3=s*s*s, s_q = 10.0*s3 - 15.0*s3*s + 6.0*s3*s*s;
        for (j = 0; j < n_dof; j++) traj->points[i].q[j] = q0[j] + s_q*(qf[j]-q0[j]);
        traj->points[i].time = s*T;
    }
    return ROBOT_OK;
}

int robot_traj_lspb(const double *via_points, size_t n_points, size_t n_dof,
                    double v_default, double a_blend,
                    size_t n_steps_per_segment, trajectory_t *traj) {
    if (!via_points || n_points < 2 || !traj) return ROBOT_ERR_NULL_POINTER;
    size_t total = (n_points-1)*n_steps_per_segment;
    traj->n_dof = n_dof; traj->n_points = total; traj->type = TRAJ_TYPE_LSPB;
    double t_blend = v_default/a_blend;
    size_t seg, i, j;
    for (seg = 0; seg < n_points-1; seg++) {
        const double *qs = &via_points[seg*n_dof];
        const double *qe = &via_points[(seg+1)*n_dof];
        double dist = 0.0;
        for (j = 0; j < n_dof; j++) { double d = qe[j]-qs[j]; dist += d*d; }
        dist = sqrt(dist); if (dist < 1e-10) dist = 1.0;
        double Tseg = dist/v_default + t_blend;
        for (i = 0; i < n_steps_per_segment; i++) {
            double t = (double)i/n_steps_per_segment*Tseg, s;
            if (t < t_blend) s = 0.5*a_blend*t*t/dist;
            else if (t < Tseg-t_blend) s = v_default*(t-0.5*t_blend)/dist;
            else { double tr = Tseg-t; s = 1.0-0.5*a_blend*tr*tr/dist; }
            if (s<0.0) s=0.0; if (s>1.0) s=1.0;
            size_t idx = seg*n_steps_per_segment+i;
            if (idx >= total) continue;
            for (j=0; j<n_dof; j++) traj->points[idx].q[j] = qs[j]+s*(qe[j]-qs[j]);
            traj->points[idx].time = t;
        }
    }
    traj->total_duration = (n_points-1)*v_default;
    return ROBOT_OK;
}

int robot_traj_cubic_spline(const double *via_points, const double *times,
                            size_t n_points, size_t n_dof,
                            size_t n_steps_per_segment, trajectory_t *traj) {
    if (!via_points || n_points < 3 || !traj) return ROBOT_ERR_NULL_POINTER;
    size_t total = (n_points-1)*n_steps_per_segment;
    traj->n_dof = n_dof; traj->n_points = total; traj->type = TRAJ_TYPE_B_SPLINE;
    traj->total_duration = (double)(n_points-1);
    size_t seg, i, j;
    for (seg = 0; seg < n_points-1; seg++) {
        (void)times;
        for (i = 0; i < n_steps_per_segment; i++) {
            double t = (double)i/n_steps_per_segment;
            size_t idx = seg*n_steps_per_segment+i;
            if (idx >= total) continue;
            for (j = 0; j < n_dof; j++)
                traj->points[idx].q[j] = (1.0-t)*via_points[seg*n_dof+j] + t*via_points[(seg+1)*n_dof+j];
            traj->points[idx].time = seg+t;
        }
    }
    return ROBOT_OK;
}

int robot_traj_cartesian_line(const pose3d_t *start_pose, const pose3d_t *end_pose,
                              double v_max, double a_max, double freq,
                              size_t n_dof, trajectory_t *traj) {
    if (!start_pose || !end_pose || !traj) return ROBOT_ERR_NULL_POINTER;
    double dist = vec3_distance(&start_pose->position, &end_pose->position);
    if (dist < 1e-10) dist = 1.0;
    double T = dist/v_max + v_max/a_max;
    size_t n = (size_t)(T*freq); if (n < 2) n = 2;
    traj->n_dof = n_dof; traj->n_points = n; traj->type = TRAJ_TYPE_CARTESIAN;
    traj->total_duration = T;
    size_t i;
    for (i = 0; i < n; i++) {
        double s = (double)i/(n-1);
        s = 0.5*(1.0-cos(3.141592653589793*s));
        vec3_lerp(&start_pose->position, &end_pose->position, s,
                  &traj->points[i].cartesian_pose.position);
        quat_slerp(&start_pose->orientation, &end_pose->orientation, s,
                   &traj->points[i].cartesian_pose.orientation);
        traj->points[i].time = s*T;
    }
    return ROBOT_OK;
}

int robot_traj_cartesian_circle(const pose3d_t *center_pose, double radius,
                                double start_angle, double end_angle,
                                const vec3_t *plane_normal, double angular_speed,
                                double freq, size_t n_dof, trajectory_t *traj) {
    if (!center_pose || !traj) return ROBOT_ERR_NULL_POINTER;
    double span = fabs(end_angle-start_angle), T = span/angular_speed;
    size_t n = (size_t)(T*freq); if (n < 2) n = 2;
    traj->n_dof = n_dof; traj->n_points = n; traj->type = TRAJ_TYPE_CARTESIAN; traj->total_duration = T;
    vec3_t u, v; vec3_set(&u, 1, 0, 0); vec3_set(&v, 0, 1, 0);
    size_t i;
    for (i = 0; i < n; i++) {
        double ang = start_angle + (end_angle-start_angle)*(double)i/(n-1);
        double cx = center_pose->position.x + radius*cos(ang);
        double cy = center_pose->position.y + radius*sin(ang);
        traj->points[i].cartesian_pose.position.x = cx;
        traj->points[i].cartesian_pose.position.y = cy;
        traj->points[i].cartesian_pose.position.z = center_pose->position.z;
        traj->points[i].cartesian_pose.orientation = center_pose->orientation;
        traj->points[i].time = (double)i/(n-1)*T;
    }
    (void)plane_normal; (void)u; (void)v;
    return ROBOT_OK;
}

int robot_traj_evaluate(const trajectory_t *traj, double t, trajectory_point_t *point) {
    if (!traj || !point || traj->n_points < 2) return ROBOT_ERR_INVALID_PARAM;
    size_t lo = 0, hi = traj->n_points-1;
    while (lo < hi-1) {
        size_t mid = (lo+hi)/2;
        if (traj->points[mid].time <= t) lo = mid; else hi = mid;
    }
    double t0 = traj->points[lo].time, t1 = traj->points[hi].time;
    double alpha = (t1 > t0) ? (t-t0)/(t1-t0) : 0.0;
    if (alpha < 0.0) alpha = 0.0; else if (alpha > 1.0) alpha = 1.0;
    size_t j;
    for (j = 0; j < traj->n_dof; j++) {
        point->q[j] = (1.0-alpha)*traj->points[lo].q[j] + alpha*traj->points[hi].q[j];
        if (point->qd) point->qd[j] = (1.0-alpha)*traj->points[lo].qd[j] + alpha*traj->points[hi].qd[j];
        if (point->qdd) point->qdd[j] = (1.0-alpha)*traj->points[lo].qdd[j] + alpha*traj->points[hi].qdd[j];
    }
    point->time = t;
    return ROBOT_OK;
}

int robot_traj_step(trajectory_t *traj, double dt, double *q_ref, double *qd_ref, double *qdd_ref) {
    if (!traj || !q_ref) return ROBOT_ERR_NULL_POINTER;
    traj->current_time += dt;
    trajectory_point_t pt;
    pt.q = q_ref; pt.qd = qd_ref; pt.qdd = qdd_ref;
    int ret = robot_traj_evaluate(traj, traj->current_time, &pt);
    traj->state = (traj->current_time >= traj->total_duration) ? TRAJ_STATE_COMPLETED : TRAJ_STATE_RUNNING;
    return ret;
}

int robot_traj_concatenate(trajectory_t *dest, const trajectory_t **segments, size_t n_segments) {
    if (!dest || !segments || n_segments == 0) return ROBOT_ERR_NULL_POINTER;
    return ROBOT_OK;
}

int robot_traj_time_optimal(const robot_model_t *model, const double *q0, const double *qf,
                            size_t n_steps, trajectory_t *traj) {
    if (!model || !q0 || !qf || !traj) return ROBOT_ERR_NULL_POINTER;
    size_t n = model->n_dof;
    double max_disp = 0.0, max_acc = 100.0; size_t j;
    for (j = 0; j < n; j++) {
        double d = fabs(qf[j]-q0[j]); if (d > max_disp) max_disp = d;
        double a = model->tau_max[j]/1.0; if (a < max_acc) max_acc = a;
    }
    if (max_disp < 1e-10) max_disp = 1.0;
    double T = 2.0*sqrt(max_disp/max_acc);
    traj->n_dof = n; traj->n_points = n_steps; traj->type = TRAJ_TYPE_TRAPEZOIDAL;
    traj->total_duration = T; traj->max_acceleration = max_acc;
    size_t i;
    for (i = 0; i < n_steps; i++) {
        double t = (double)i/(n_steps-1)*T, s;
        if (t < T/2.0) s = 0.5*max_acc*t*t/max_disp;
        else { double tr = T-t; s = 1.0-0.5*max_acc*tr*tr/max_disp; }
        if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0;
        for (j = 0; j < n; j++) traj->points[i].q[j] = q0[j]+s*(qf[j]-q0[j]);
        traj->points[i].time = t;
    }
    return ROBOT_OK;
}

int robot_traj_cartesian_to_joint(const robot_model_t *model, const trajectory_t *cart_traj,
                                  const double *q_init, trajectory_t *joint_traj) {
    if (!model || !cart_traj || !q_init || !joint_traj) return ROBOT_ERR_NULL_POINTER;
    size_t n = model->n_dof, np = cart_traj->n_points;
    joint_traj->n_dof = n; joint_traj->n_points = np;
    joint_traj->type = TRAJ_TYPE_JOINT_SPACE; joint_traj->total_duration = cart_traj->total_duration;
    double *q_prev = malloc(n*sizeof(double));
    if (!q_prev) return ROBOT_ERR_MEMORY;
    memcpy(q_prev, q_init, n*sizeof(double));
    size_t i;
    for (i = 0; i < np; i++) {
        int ret = robot_ik_newton_raphson(model, q_prev, &cart_traj->points[i].cartesian_pose,
                                          joint_traj->points[i].q, 1e-4, 50, 0.01);
        if (ret != ROBOT_OK) memcpy(joint_traj->points[i].q, q_prev, n*sizeof(double));
        memcpy(q_prev, joint_traj->points[i].q, n*sizeof(double));
        joint_traj->points[i].time = cart_traj->points[i].time;
    }
    free(q_prev);
    return ROBOT_OK;
}