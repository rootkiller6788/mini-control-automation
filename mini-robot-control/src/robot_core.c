/**
 * @file    robot_core.c
 * @brief   Core robot model lifecycle, memory management, error strings,
 *          and basic utility functions.
 *
 * Knowledge Coverage:
 *   L1 (Definitions): robot model lifecycle, joint/state initialization
 *   L2 (Core Concepts): robot model validation, joint limit enforcement
 *
 * References:
 *   Craig (2018), Siciliano et al. (2010)
 */

#include "robot_types.h"
#include "robot_math3d.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef ROBOT_MIN
#define ROBOT_MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef ROBOT_MAX
#define ROBOT_MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef ROBOT_CLAMP
#define ROBOT_CLAMP(x, lo, hi) (ROBOT_MIN(ROBOT_MAX((x), (lo)), (hi)))
#endif

/* ---- L1: Error string conversion ---- */
const char* robot_error_string(robot_error_t err) {
    switch (err) {
    case ROBOT_OK:                 return "Success";
    case ROBOT_ERR_NULL_POINTER:   return "Null pointer argument";
    case ROBOT_ERR_INVALID_DOF:    return "Invalid degrees of freedom";
    case ROBOT_ERR_MEMORY:         return "Memory allocation failure";
    case ROBOT_ERR_SINGULARITY:    return "Kinematic singularity detected";
    case ROBOT_ERR_JOINT_LIMIT:    return "Joint limit violation";
    case ROBOT_ERR_VELOCITY_LIMIT: return "Velocity limit violation";
    case ROBOT_ERR_TORQUE_LIMIT:   return "Torque limit violation";
    case ROBOT_ERR_NO_SOLUTION:    return "No IK solution found";
    case ROBOT_ERR_DIVERGED:       return "Algorithm diverged";
    case ROBOT_ERR_NOT_INIT:       return "Robot model not initialized";
    case ROBOT_ERR_INVALID_PARAM:  return "Invalid parameter";
    case ROBOT_ERR_TIMEOUT:        return "Operation timed out";
    case ROBOT_ERR_COLLISION:      return "Collision detected";
    case ROBOT_ERR_WORKSPACE:      return "Target outside workspace";
    default:                       return "Unknown error";
    }
}

/* ---- L1: Memory allocation helpers ---- */
static void* robot_malloc_safe(size_t size) {
    void *p = malloc(size);
    if (!p) fprintf(stderr, "robot: malloc(%zu) failed\n", size);
    return p;
}

static void robot_free_safe(void **p) {
    if (p && *p) { free(*p); *p = NULL; }
}

/* ---- L1: Model validation ---- */
static int robot_model_validate(const robot_model_t *model) {
    if (!model) return ROBOT_ERR_NULL_POINTER;
    if (model->n_dof == 0 || model->n_dof > 100) return ROBOT_ERR_INVALID_DOF;
    if (!model->dh_params) return ROBOT_ERR_NULL_POINTER;
    if (!model->links) return ROBOT_ERR_NULL_POINTER;
    return ROBOT_OK;
}

/* ---- L1: Allocate robot model ---- */
int robot_model_alloc(robot_model_t *model, robot_type_t type,
                      size_t n_dof, size_t n_links) {
    if (!model) return ROBOT_ERR_NULL_POINTER;
    if (n_dof == 0 || n_dof > 100) return ROBOT_ERR_INVALID_DOF;

    memset(model, 0, sizeof(*model));
    model->type = type;
    model->n_dof = n_dof;
    model->n_links = n_links > 0 ? n_links : n_dof;
    model->has_floating_base = 0;
    model->gravity[0] = 0.0;
    model->gravity[1] = 0.0;
    model->gravity[2] = -9.81;
    model->control_period = 0.001;

    model->dh_params = robot_malloc_safe(model->n_dof * sizeof(dh_param_t));
    model->links = robot_malloc_safe(model->n_links * sizeof(link_params_t));
    model->q_min = robot_malloc_safe(model->n_dof * sizeof(double));
    model->q_max = robot_malloc_safe(model->n_dof * sizeof(double));
    model->qd_max = robot_malloc_safe(model->n_dof * sizeof(double));
    model->tau_max = robot_malloc_safe(model->n_dof * sizeof(double));

    if (!model->dh_params || !model->links || !model->q_min ||
        !model->q_max || !model->qd_max || !model->tau_max) {
        robot_model_free(model);
        return ROBOT_ERR_MEMORY;
    }

    /* Initialize DH params to identity */
    size_t i;
    for (i = 0; i < model->n_dof; i++) {
        model->dh_params[i].alpha = 0.0;
        model->dh_params[i].a = 0.0;
        model->dh_params[i].d = 0.0;
        model->dh_params[i].theta = 0.0;
        model->dh_params[i].is_revolute = 1;
    }

    /* Initialize links as unit-mass point masses at origin */
    for (i = 0; i < model->n_links; i++) {
        model->links[i].link_id = i;
        model->links[i].mass = 1.0;
        model->links[i].com.x = 0.0;
        model->links[i].com.y = 0.0;
        model->links[i].com.z = 0.0;
        memset(&model->links[i].inertia_tensor, 0, sizeof(mat3_t));
        model->links[i].inertia_tensor.m[0] = 1.0;
        model->links[i].inertia_tensor.m[4] = 1.0;
        model->links[i].inertia_tensor.m[8] = 1.0;
        model->links[i].joint_axis.x = 0.0;
        model->links[i].joint_axis.y = 0.0;
        model->links[i].joint_axis.z = 1.0;
        model->links[i].friction_viscous = 0.01;
        model->links[i].friction_coulomb = 0.1;
        model->links[i].motor_gear_ratio = 1.0;
        model->links[i].motor_inertia = 0.0;
    }

    /* Default joint limits: +/-pi for revolute */
    for (i = 0; i < model->n_dof; i++) {
        model->q_min[i] = -3.141592653589793;
        model->q_max[i] = 3.141592653589793;
        model->qd_max[i] = 10.0;
        model->tau_max[i] = 100.0;
    }

    return ROBOT_OK;
}

/* ---- L1: Free robot model ---- */
void robot_model_free(robot_model_t *model) {
    if (!model) return;
    robot_free_safe((void**)&model->dh_params);
    robot_free_safe((void**)&model->links);
    robot_free_safe((void**)&model->q_min);
    robot_free_safe((void**)&model->q_max);
    robot_free_safe((void**)&model->qd_max);
    robot_free_safe((void**)&model->tau_max);
    memset(model, 0, sizeof(*model));
}

/* ---- L1: Joint state operations ---- */
int robot_state_alloc(const robot_model_t *model, robot_state_t *state) {
    if (!model || !state) return ROBOT_ERR_NULL_POINTER;
    int err = robot_model_validate(model);
    if (err != ROBOT_OK) return err;

    memset(state, 0, sizeof(*state));
    state->model = model;
    state->joint_state.n_dof = model->n_dof;
    state->joint_state.q = robot_malloc_safe(model->n_dof * sizeof(double));
    state->joint_state.qd = robot_malloc_safe(model->n_dof * sizeof(double));
    state->joint_state.qdd = robot_malloc_safe(model->n_dof * sizeof(double));
    state->joint_state.tau = robot_malloc_safe(model->n_dof * sizeof(double));
    state->link_transforms = robot_malloc_safe(
        (model->n_links + 1) * sizeof(mat4_t));
    state->time = 0.0;
    state->iteration = 0;

    if (!state->joint_state.q || !state->joint_state.qd ||
        !state->joint_state.qdd || !state->joint_state.tau ||
        !state->link_transforms) {
        robot_state_free(state);
        return ROBOT_ERR_MEMORY;
    }

    size_t i;
    for (i = 0; i < model->n_dof; i++) {
        state->joint_state.q[i] = 0.0;
        state->joint_state.qd[i] = 0.0;
        state->joint_state.qdd[i] = 0.0;
        state->joint_state.tau[i] = 0.0;
    }
    for (i = 0; i <= model->n_links; i++) {
        mat4_identity(&state->link_transforms[i]);
    }

    return ROBOT_OK;
}

void robot_state_free(robot_state_t *state) {
    if (!state) return;
    robot_free_safe((void**)&state->joint_state.q);
    robot_free_safe((void**)&state->joint_state.qd);
    robot_free_safe((void**)&state->joint_state.qdd);
    robot_free_safe((void**)&state->joint_state.tau);
    robot_free_safe((void**)&state->link_transforms);
    robot_free_safe(&state->ctrl_state);
    memset(state, 0, sizeof(*state));
}

/* ---- L1: Set DH parameters for a specific joint ---- */
void robot_set_dh_param(robot_model_t *model, size_t joint_idx,
                        double alpha, double a, double d, double theta,
                        int is_revolute) {
    if (!model || joint_idx >= model->n_dof) return;
    model->dh_params[joint_idx].alpha = alpha;
    model->dh_params[joint_idx].a = a;
    model->dh_params[joint_idx].d = d;
    model->dh_params[joint_idx].theta = theta;
    model->dh_params[joint_idx].is_revolute = is_revolute;
}

/* ---- L1: Set link parameters ---- */
void robot_set_link_param(robot_model_t *model, size_t link_idx,
                          double mass, double com_x, double com_y, double com_z,
                          double ixx, double iyy, double izz) {
    if (!model || link_idx >= model->n_links) return;
    link_params_t *link = &model->links[link_idx];
    link->mass = mass;
    link->com.x = com_x;
    link->com.y = com_y;
    link->com.z = com_z;
    link->inertia_tensor.m[0] = ixx;
    link->inertia_tensor.m[4] = iyy;
    link->inertia_tensor.m[8] = izz;
}

/* ---- L1: Joint limit enforcement ---- */
int robot_clamp_to_limits(const robot_model_t *model, double *q) {
    if (!model || !q) return ROBOT_ERR_NULL_POINTER;
    size_t i;
    int clamped = 0;
    for (i = 0; i < model->n_dof; i++) {
        if (q[i] < model->q_min[i]) {
            q[i] = model->q_min[i];
            clamped = 1;
        } else if (q[i] > model->q_max[i]) {
            q[i] = model->q_max[i];
            clamped = 1;
        }
    }
    return clamped ? ROBOT_ERR_JOINT_LIMIT : ROBOT_OK;
}

int robot_check_limits(const robot_model_t *model, const double *q,
                       const double *qd, const double *tau) {
    if (!model) return ROBOT_ERR_NULL_POINTER;
    size_t i;
    if (q) {
        for (i = 0; i < model->n_dof; i++) {
            if (q[i] < model->q_min[i] || q[i] > model->q_max[i])
                return ROBOT_ERR_JOINT_LIMIT;
        }
    }
    if (qd) {
        for (i = 0; i < model->n_dof; i++) {
            if (qd[i] > model->qd_max[i] || qd[i] < -model->qd_max[i])
                return ROBOT_ERR_VELOCITY_LIMIT;
        }
    }
    if (tau) {
        for (i = 0; i < model->n_dof; i++) {
            if (tau[i] > model->tau_max[i] || tau[i] < -model->tau_max[i])
                return ROBOT_ERR_TORQUE_LIMIT;
        }
    }
    return ROBOT_OK;
}

/* ---- L1: 2-DOF planar robot template (common benchmark) ---- */
int robot_model_create_planar_2dof(robot_model_t *model) {
    int ret = robot_model_alloc(model, ROBOT_TYPE_SERIAL_MANIPULATOR, 2, 2);
    if (ret != ROBOT_OK) return ret;

    /* Link 1: length 1m, mass 1kg, Izz=1/12 (rod) */
    robot_set_dh_param(model, 0, 0.0, 1.0, 0.0, 0.0, 1);
    robot_set_link_param(model, 0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0833);
    model->links[0].joint_axis.z = 1.0;

    /* Link 2: length 1m, mass 1kg */
    robot_set_dh_param(model, 1, 0.0, 1.0, 0.0, 0.0, 1);
    robot_set_link_param(model, 1, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0833);
    model->links[1].joint_axis.z = 1.0;

    model->gravity[0] = 0.0;
    model->gravity[1] = 0.0;
    model->gravity[2] = -9.81;
    model->control_period = 0.001;

    return ROBOT_OK;
}

/* ---- L1: 3-DOF SCARA robot template ---- */
int robot_model_create_scara(robot_model_t *model) {
    int ret = robot_model_alloc(model, ROBOT_TYPE_SCARA, 3, 3);
    if (ret != ROBOT_OK) return ret;

    /* Joint 1: rotation about Z */
    robot_set_dh_param(model, 0, 0.0, 0.5, 0.0, 0.0, 1);
    robot_set_link_param(model, 0, 2.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.1);

    /* Joint 2: rotation about Z */
    robot_set_dh_param(model, 1, 0.0, 0.4, 0.0, 0.0, 1);
    robot_set_link_param(model, 1, 1.5, 0.20, 0.0, 0.0, 0.0, 0.0, 0.05);

    /* Joint 3: prismatic along Z */
    robot_set_dh_param(model, 2, 0.0, 0.0, 0.15, 0.0, 0);
    robot_set_link_param(model, 2, 0.5, 0.0, 0.0, 0.075, 0.01, 0.01, 0.001);
    model->links[2].joint_axis.z = 1.0;
    model->q_min[2] = 0.0;
    model->q_max[2] = 0.2;

    return ROBOT_OK;
}

/* ---- L1: Differential-drive mobile robot template ---- */
int robot_model_create_diff_drive(robot_model_t *model) {
    int ret = robot_model_alloc(model, ROBOT_TYPE_MOBILE_WHEELED, 2, 1);
    if (ret != ROBOT_OK) return ret;

    /* Non-holonomic differential drive: 2 wheel DOFs, 3 task DOFs (x,y,theta) */
    model->has_floating_base = 1;

    /* Body link */
    robot_set_link_param(model, 0, 5.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.2);
    model->links[0].joint_axis.z = 0.0;
    model->links[0].joint_axis.x = 1.0;

    /* Joint 0: forward velocity, Joint 1: angular velocity */
    model->q_min[0] = -2.0; model->q_max[0] = 2.0;
    model->q_min[1] = -3.14; model->q_max[1] = 3.14;
    model->qd_max[0] = 1.0; model->qd_max[1] = 3.0;
    model->tau_max[0] = 10.0; model->tau_max[1] = 5.0;
    model->control_period = 0.01;

    return ROBOT_OK;
}

/* ---- L1: Print robot configuration (debug) ---- */
void robot_print_config(const robot_model_t *model) {
    if (!model) { printf("robot_print_config: null model\n"); return; }
    printf("Robot Model: type=%d, DOF=%zu, links=%zu\n",
           model->type, model->n_dof, model->n_links);
    printf("  Gravity: [%.2f, %.2f, %.2f]\n",
           model->gravity[0], model->gravity[1], model->gravity[2]);
    printf("  Control period: %.4f s\n", model->control_period);
    size_t i;
    for (i = 0; i < model->n_dof; i++) {
        dh_param_t *dh = &model->dh_params[i];
        printf("  Joint %zu: alpha=%.3f a=%.3f d=%.3f theta=%.3f %s\n",
               i, dh->alpha, dh->a, dh->d, dh->theta,
               dh->is_revolute ? "revolute" : "prismatic");
    }
    for (i = 0; i < model->n_links; i++) {
        link_params_t *l = &model->links[i];
        printf("  Link %zu: mass=%.3f kg, com=(%.3f,%.3f,%.3f),"
               " Izz=%.4f\n",
               i, l->mass, l->com.x, l->com.y, l->com.z,
               l->inertia_tensor.m[8]);
    }
}
