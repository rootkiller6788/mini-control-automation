#ifndef ROBOT_DYNAMICS_H
#define ROBOT_DYNAMICS_H
#include "robot_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1-L4: Robot Dynamics -- Lagrangian & Newton-Euler Formulations
 *
 * Lagrangian: tau = M(q)*qdd + C(q,qd)*qd + G(q) + F(qd)
 *   M(q): symmetric positive-definite mass matrix
 *   C(q,qd)*qd: Coriolis & centrifugal torques
 *   G(q): gravitational torques = partial P / partial q
 *   F(qd): friction torques
 *
 * Newton-Euler: O(n) recursive algorithm (Featherstone, 1983)
 *
 * References:
 *   Craig (2018) Ch. 6
 *   Siciliano et al. (2010) Ch. 7
 *   Featherstone (2008) "Rigid Body Dynamics Algorithms", Springer
 *   Lagrange (1788) "Mecanique Analytique"
 * ========================================================================= */

typedef enum {
    DYNAMICS_METHOD_LAGRANGIAN    = 0,
    DYNAMICS_METHOD_NEWTON_EULER  = 1,
    DYNAMICS_METHOD_COMPOSITE_RB  = 2
} dynamics_method_t;

typedef struct {
    dynamics_method_t method;
    int include_friction;
    int include_external_forces;
    double integration_dt;
    void *external_wrenches;
} dynamics_config_t;

int robot_dynamics_mass_matrix(const robot_model_t *model, const double *q,
                               double *M);
int robot_dynamics_coriolis(const robot_model_t *model, const double *q,
                            const double *qd, double *coriolis_out);
int robot_dynamics_gravity(const robot_model_t *model, const double *q,
                           const mat4_t *transforms, double *grav_out);
int robot_dynamics_friction(const robot_model_t *model, const double *qd,
                            double *friction_out);
int robot_dynamics_forward(const robot_model_t *model, const double *q,
                           const double *qd, const double *tau,
                           double *qdd_out);
int robot_dynamics_inverse(const robot_model_t *model, const double *q,
                           const double *qd, const double *qdd,
                           double *tau_out);
int robot_dynamics_newton_euler(const robot_model_t *model, const double *q,
                                const double *qd, const double *qdd,
                                double *tau_out);
double robot_kinetic_energy(const robot_model_t *model, const double *q,
                            const double *qd);
double robot_potential_energy(const robot_model_t *model, const double *q,
                              const mat4_t *transforms);
double robot_effective_inertia(const robot_model_t *model, size_t joint_idx,
                               const double *q);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_DYNAMICS_H */
