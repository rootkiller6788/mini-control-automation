#include "modern_control.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* L6: Canonical Control Problems - Inverted Pendulum, DC Motor, Quadrotor */

int mc_build_inverted_pendulum(const mc_inverted_pendulum_params_t *p, mc_ss_system_t *sys)
{
    if (!p || !sys || sys->n_states != 4 || sys->n_inputs != 1) return -1;
    double M = p->cart_mass, m = p->pendulum_mass, l = p->pendulum_len;
    double g = p->gravity, b = p->friction_cart;
    double I = m*l*l/3.0;
    double It = (M+m)*I + M*m*l*l;
    if (It < 1e-12) return -2;
    mc_matrix_zero(&sys->A);
    mc_matrix_set(&sys->A, 0, 1, 1.0);
    mc_matrix_set(&sys->A, 1, 1, -(I+m*l*l)*b/It);
    mc_matrix_set(&sys->A, 1, 2, m*m*g*l*l/It);
    mc_matrix_set(&sys->A, 2, 3, 1.0);
    mc_matrix_set(&sys->A, 3, 1, -m*l*b/It);
    mc_matrix_set(&sys->A, 3, 2, (M+m)*m*g*l/It);
    mc_matrix_set(&sys->B, 1, 0, (I+m*l*l)/It);
    mc_matrix_set(&sys->B, 3, 0, m*l/It);
    mc_matrix_set(&sys->C, 0, 0, 1.0);
    mc_matrix_set(&sys->C, 1, 2, 1.0);
    return 0;
}

int mc_build_dc_motor(const mc_dc_motor_params_t *p, mc_ss_system_t *sys)
{
    if (!p || !sys || sys->n_states != 3 || sys->n_inputs != 1) return -1;
    double Ra=p->R_a, La=p->L_a, Kb=p->K_b, Kt=p->K_t, J=p->J, B=p->B;
    if (La<1e-12 || J<1e-12) return -2;
    mc_matrix_zero(&sys->A);
    mc_matrix_set(&sys->A, 0, 1, 1.0);
    mc_matrix_set(&sys->A, 1, 1, -B/J);
    mc_matrix_set(&sys->A, 1, 2, Kt/J);
    mc_matrix_set(&sys->A, 2, 1, -Kb/La);
    mc_matrix_set(&sys->A, 2, 2, -Ra/La);
    mc_matrix_set(&sys->B, 2, 0, 1.0/La);
    mc_matrix_set(&sys->C, 0, 1, 1.0);
    return 0;
}

int mc_build_quadrotor_hover(const mc_quadrotor_params_t *p, mc_ss_system_t *sys)
{
    if (!p || !sys || sys->n_states != 12 || sys->n_inputs != 4) return -1;
    double g=p->gravity, Ix=p->I_xx, Iy=p->I_yy, Iz=p->I_zz;
    if (Ix<1e-12 || Iy<1e-12 || Iz<1e-12) return -2;
    mc_matrix_zero(&sys->A);
    mc_matrix_set(&sys->A, 0, 1, 1.0);
    mc_matrix_set(&sys->A, 1, 8, -g);
    mc_matrix_set(&sys->A, 2, 3, 1.0);
    mc_matrix_set(&sys->A, 3, 6, g);
    mc_matrix_set(&sys->A, 4, 5, 1.0);
    mc_matrix_set(&sys->A, 6, 7, 1.0);
    mc_matrix_set(&sys->A, 8, 9, 1.0);
    mc_matrix_set(&sys->A, 10, 11, 1.0);
    double m = p->mass;
    mc_matrix_set(&sys->B, 5, 0, 1.0/m);
    mc_matrix_set(&sys->B, 7, 1, 1.0/Ix);
    mc_matrix_set(&sys->B, 9, 2, 1.0/Iy);
    mc_matrix_set(&sys->B, 11, 3, 1.0/Iz);
    mc_matrix_set(&sys->C, 0, 0, 1.0);
    mc_matrix_set(&sys->C, 1, 2, 1.0);
    mc_matrix_set(&sys->C, 2, 4, 1.0);
    mc_matrix_set(&sys->C, 3, 6, 1.0);
    mc_matrix_set(&sys->C, 4, 8, 1.0);
    mc_matrix_set(&sys->C, 5, 10, 1.0);
    return 0;
}

int mc_inverted_pendulum_lqr(const mc_inverted_pendulum_params_t *p,
                              const double *qw, const double *rw,
                              mc_ss_system_t *so, mc_lqr_solution_t *lo)
{
    mc_ss_system_t sys;
    if (mc_ss_alloc(4,1,2,&sys) != 0) return -1;
    mc_build_inverted_pendulum(p, &sys);
    mc_matrix_t Q, R;
    mc_matrix_alloc(4,4,&Q); mc_matrix_alloc(1,1,&R);
    mc_matrix_zero(&Q);
    for(size_t i=0;i<4;i++) mc_matrix_set(&Q,i,i,qw[i]);
    mc_matrix_set(&R,0,0,rw[0]);
    int ret = mc_lqr_solve(&sys.A,&sys.B,&Q,&R,MC_LQR_KLEINMAN,lo);
    if(so){mc_matrix_copy(&sys.A,&so->A);mc_matrix_copy(&sys.B,&so->B);mc_matrix_copy(&sys.C,&so->C);}
    mc_matrix_free(&Q); mc_matrix_free(&R); mc_ss_free(&sys);
    return ret;
}

int mc_dc_motor_lqr(const mc_dc_motor_params_t *p,
                     const double *qw, const double *rw,
                     mc_ss_system_t *so, mc_lqr_solution_t *lo)
{
    mc_ss_system_t sys;
    if(mc_ss_alloc(3,1,1,&sys)!=0) return -1;
    mc_build_dc_motor(p,&sys);
    mc_matrix_t Q,R;
    mc_matrix_alloc(3,3,&Q); mc_matrix_alloc(1,1,&R);
    mc_matrix_zero(&Q);
    for(size_t i=0;i<3;i++) mc_matrix_set(&Q,i,i,qw[i]);
    mc_matrix_set(&R,0,0,rw[0]);
    int ret=mc_lqr_solve(&sys.A,&sys.B,&Q,&R,MC_LQR_KLEINMAN,lo);
    if(so){mc_matrix_copy(&sys.A,&so->A);mc_matrix_copy(&sys.B,&so->B);mc_matrix_copy(&sys.C,&so->C);}
    mc_matrix_free(&Q);mc_matrix_free(&R);mc_ss_free(&sys);
    return ret;
}

int mc_dc_motor_lqr_position(const mc_dc_motor_params_t *p,
                              const double *qw, const double *rw,
                              mc_ss_system_t *so, mc_lqr_solution_t *lo)
{
    mc_ss_system_t sys;
    if(mc_ss_alloc(3,1,1,&sys)!=0) return -1;
    mc_build_dc_motor(p,&sys);
    mc_matrix_set(&sys.C,0,0,1.0);
    mc_matrix_t Q,R;
    mc_matrix_alloc(3,3,&Q);mc_matrix_alloc(1,1,&R);
    mc_matrix_zero(&Q);
    for(size_t i=0;i<3;i++) mc_matrix_set(&Q,i,i,qw[i]);
    mc_matrix_set(&R,0,0,rw[0]);
    int ret=mc_lqr_solve(&sys.A,&sys.B,&Q,&R,MC_LQR_KLEINMAN,lo);
    if(so){mc_matrix_copy(&sys.A,&so->A);mc_matrix_copy(&sys.B,&so->B);mc_matrix_copy(&sys.C,&so->C);}
    mc_matrix_free(&Q);mc_matrix_free(&R);mc_ss_free(&sys);
    return ret;
}

int mc_ss_similarity_transform(const mc_ss_system_t *sys,
                                const mc_matrix_t *T,
                                mc_ss_system_t *st)
{
    if(!sys||!T||!st) return -1;
    size_t n=sys->n_states;
    if(T->rows!=n||T->cols!=n||st->n_states!=n) return -2;
    mc_matrix_t Ti,TA,TATi;
    mc_matrix_alloc(n,n,&Ti);mc_matrix_alloc(n,n,&TA);mc_matrix_alloc(n,n,&TATi);
    if(mc_matrix_inverse(T,&Ti)!=0){mc_matrix_free(&Ti);mc_matrix_free(&TA);mc_matrix_free(&TATi);return-3;}
    mc_matrix_mul(T,&sys->A,&TA);
    mc_matrix_mul(&TA,&Ti,&TATi);
    mc_matrix_copy(&TATi,&st->A);
    if(sys->n_inputs>0){mc_matrix_t TB;mc_matrix_alloc(n,sys->n_inputs,&TB);mc_matrix_mul(T,&sys->B,&TB);mc_matrix_copy(&TB,&st->B);mc_matrix_free(&TB);}
    if(sys->n_outputs>0){mc_matrix_t CTi;mc_matrix_alloc(sys->n_outputs,n,&CTi);mc_matrix_mul(&sys->C,&Ti,&CTi);mc_matrix_copy(&CTi,&st->C);mc_matrix_free(&CTi);}
    mc_matrix_free(&Ti);mc_matrix_free(&TA);mc_matrix_free(&TATi);
    return 0;
}
