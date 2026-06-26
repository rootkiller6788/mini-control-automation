#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Core Definitions 鈥?Robot Type Taxonomies
 *
 * Robot control centers on the interplay between configuration space
 * (joint space Q) and task space (Cartesian/se(3)). The fundamental
 * mapping is:
 *
 *   x = f(q)         Forward kinematics:  Q 鈫?SE(3)
 *   蟿 = M(q)q虉 + C(q,q虈)q虈 + G(q)   Lagrangian dynamics
 *
 * References:
 *   Craig, "Introduction to Robotics", 4th ed. (2018): Ch. 2-6
 *   Siciliano et al., "Robotics", 2nd ed. (2010): Ch. 2-7
 *   Murray, Li, Sastry, "A Mathematical Introduction to Robotic
 *     Manipulation", CRC (1994): Ch. 2-4
 * ========================================================================= */

typedef enum {
    ROBOT_TYPE_SERIAL_MANIPULATOR = 0,
    ROBOT_TYPE_PARALLEL           = 1,
    ROBOT_TYPE_MOBILE_WHEELED     = 2,
    ROBOT_TYPE_MOBILE_LEGGED      = 3,
    ROBOT_TYPE_UAV                = 4,
    ROBOT_TYPE_AUV                = 5,
    ROBOT_TYPE_SCARA              = 6,
    ROBOT_TYPE_CARTESIAN          = 7,
    ROBOT_TYPE_COLLABORATIVE      = 8,
    ROBOT_TYPE_SOFT               = 9,
    ROBOT_TYPE_HUMANOID           = 10
} robot_type_t;

typedef enum {
    JOINT_TYPE_REVOLUTE   = 0,
    JOINT_TYPE_PRISMATIC  = 1,
    JOINT_TYPE_CONTINUOUS = 2,
    JOINT_TYPE_SPHERICAL  = 3,
    JOINT_TYPE_FIXED      = 4,
    JOINT_TYPE_PLANAR     = 5
} joint_type_t;

typedef enum {
    CONTROL_MODE_POSITION     = 0,
    CONTROL_MODE_VELOCITY     = 1,
    CONTROL_MODE_TORQUE       = 2,
    CONTROL_MODE_IMPEDANCE    = 3,
    CONTROL_MODE_ADMITTANCE   = 4,
    CONTROL_MODE_HYBRID       = 5,
    CONTROL_MODE_COMPLIANT    = 6
} control_mode_t;

typedef enum {
    FRAME_WORLD    = -1,
    FRAME_BASE     = 0,
    FRAME_TOOL     = -2,
    FRAME_SENSOR   = -3,
    FRAME_OBJECT   = -4
} frame_id_t;

typedef enum {
    TRAJ_TYPE_JOINT_SPACE   = 0,
    TRAJ_TYPE_CARTESIAN     = 1,
    TRAJ_TYPE_CUBIC_POLY    = 2,
    TRAJ_TYPE_QUINTIC_POLY  = 3,
    TRAJ_TYPE_TRAPEZOIDAL   = 4,
    TRAJ_TYPE_S_CURVE       = 5,
    TRAJ_TYPE_LSPB          = 6,
    TRAJ_TYPE_B_SPLINE      = 7,
    TRAJ_TYPE_MINIMUM_JERK  = 8
} trajectory_type_t;

typedef enum {
    TRAJ_STATE_IDLE       = 0,
    TRAJ_STATE_RUNNING    = 1,
    TRAJ_STATE_PAUSED     = 2,
    TRAJ_STATE_COMPLETED  = 3,
    TRAJ_STATE_ABORTED    = 4,
    TRAJ_STATE_HOLDING    = 5
} trajectory_state_t;

typedef struct { double x, y, z; } vec3_t;

typedef struct { double m[9]; } mat3_t;

typedef struct { double m[16]; } mat4_t;

typedef struct { double w, x, y, z; } quat_t;

typedef struct { double roll, pitch, yaw; } euler_t;

typedef struct { vec3_t v; vec3_t w; } twist_t;

typedef struct { vec3_t force; vec3_t torque; } wrench_t;

typedef struct { vec3_t position; quat_t orientation; } pose3d_t;

typedef struct {
    size_t   n_dof;
    double  *q;
    double  *qd;
    double  *qdd;
    double  *tau;
} joint_state_t;

typedef enum {
    DH_CONVENTION_STANDARD  = 0,
    DH_CONVENTION_MODIFIED  = 1
} dh_convention_t;

typedef struct {
    double  alpha;
    double  a;
    double  d;
    double  theta;
    int     is_revolute;
} dh_param_t;

typedef struct {
    size_t   link_id;
    double   mass;
    vec3_t   com;
    mat3_t   inertia_tensor;
    vec3_t   joint_axis;
    vec3_t   joint_origin;
    double   friction_viscous;
    double   friction_coulomb;
    double   motor_gear_ratio;
    double   motor_inertia;
} link_params_t;

typedef struct {
    robot_type_t    type;
    size_t          n_dof;
    size_t          n_links;
    dh_param_t     *dh_params;
    link_params_t  *links;
    pose3d_t        base_pose;
    int             has_floating_base;
    double          gravity[3];
    double          control_period;
    double         *q_min;
    double         *q_max;
    double         *qd_max;
    double         *tau_max;
    void           *user_data;
} robot_model_t;

typedef struct {
    const robot_model_t  *model;
    joint_state_t         joint_state;
    pose3d_t              tool_pose;
    twist_t               tool_twist;
    mat4_t               *link_transforms;
    double                time;
    size_t                iteration;
    void                 *ctrl_state;
} robot_state_t;

typedef struct {
    double   time;
    double  *q;
    double  *qd;
    double  *qdd;
    double  *qddd;
    pose3d_t cartesian_pose;
    trajectory_state_t state;
} trajectory_point_t;

typedef struct {
    trajectory_type_t type;
    size_t            n_dof;
    size_t            n_points;
    size_t            max_points;
    trajectory_point_t *points;
    double            total_duration;
    double            max_velocity;
    double            max_acceleration;
    double            max_jerk;
    trajectory_state_t state;
    double            current_time;
} trajectory_t;

typedef struct {
    double   settling_time;
    double   overshoot_percent;
    double   steady_state_error;
    double   rms_tracking_error;
    double   max_tracking_error;
    double   control_effort_rms;
    double   control_effort_max;
    size_t   iterations_to_settle;
} control_performance_t;

typedef enum {
    ROBOT_OK                 = 0,
    ROBOT_ERR_NULL_POINTER   = -1,
    ROBOT_ERR_INVALID_DOF    = -2,
    ROBOT_ERR_MEMORY         = -3,
    ROBOT_ERR_SINGULARITY    = -4,
    ROBOT_ERR_JOINT_LIMIT    = -5,
    ROBOT_ERR_VELOCITY_LIMIT = -6,
    ROBOT_ERR_TORQUE_LIMIT   = -7,
    ROBOT_ERR_NO_SOLUTION    = -8,
    ROBOT_ERR_DIVERGED       = -9,
    ROBOT_ERR_NOT_INIT       = -10,
    ROBOT_ERR_INVALID_PARAM  = -11,
    ROBOT_ERR_TIMEOUT        = -12,
    ROBOT_ERR_COLLISION      = -13,
    ROBOT_ERR_WORKSPACE      = -14
} robot_error_t;

const char* robot_error_string(robot_error_t err);


/* ---- Extended API declarations for robot model lifecycle ---- */
int robot_model_alloc(robot_model_t *model, robot_type_t type, size_t n_dof, size_t n_links);
void robot_model_free(robot_model_t *model);
int robot_state_alloc(const robot_model_t *model, robot_state_t *state);
void robot_state_free(robot_state_t *state);
void robot_set_dh_param(robot_model_t *model, size_t joint_idx, double alpha, double a, double d, double theta, int is_revolute);
void robot_set_link_param(robot_model_t *model, size_t link_idx, double mass, double com_x, double com_y, double com_z, double ixx, double iyy, double izz);
int robot_clamp_to_limits(const robot_model_t *model, double *q);
int robot_check_limits(const robot_model_t *model, const double *q, const double *qd, const double *tau);
int robot_model_create_planar_2dof(robot_model_t *model);
int robot_model_create_scara(robot_model_t *model);
int robot_model_create_diff_drive(robot_model_t *model);
void robot_print_config(const robot_model_t *model);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_TYPES_H */