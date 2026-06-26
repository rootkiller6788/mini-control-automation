#include "control_core.h"
#include "control_analysis.h"
#include "control_design.h"
#include "control_rootlocus.h"
#include "control_applications.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static int tests_passed=0,tests_total=0;
#define TEST(n) do{tests_total++;printf("  TEST %-30s ... ",n);}while(0)
#define PASS() do{printf("PASS\n");tests_passed++;}while(0)
#define FAIL(m) printf("FAIL: %s\n",m)

static void test_tf_init(void) {
    TEST("tf_init & normalization");
    transfer_function_t G;
    double num[]={5.0,2.0},den[]={3.0,4.0,1.0};
    assert(tf_init(&G,num,1,den,2)==0);
    assert(G.den_order==2);
    assert(fabs(G.den[0]-1.0)<1e-10);
    PASS();
}
static void test_tf_eval(void) {
    TEST("tf_evaluate DC gain");
    transfer_function_t G;
    double num[]={2.0},den[]={1.0,1.0};
    tf_init(&G,num,0,den,1);
    double complex v=tf_evaluate(&G,0.0);
    assert(fabs(creal(v)-2.0)<1e-10);
    PASS();
}
static void test_sys_type(void) {
    TEST("tf_system_type");
    transfer_function_t G;
    double n[]={1.0},d1[]={1.0,1.0},d2[]={1.0,0.0};
    tf_init(&G,n,0,d1,1); assert(tf_system_type(&G)==SYSTEM_TYPE_0);
    tf_init(&G,n,0,d2,1); assert(tf_system_type(&G)==SYSTEM_TYPE_1);
    PASS();
}
static void test_step_specs(void) {
    TEST("compute_step_specs zeta=0.5");
    step_specs_t s;
    compute_step_specs(2.0,0.5,&s);
    assert(s.overshoot_pct>15.0&&s.overshoot_pct<17.0);
    assert(s.settling_time>3.5&&s.settling_time<4.5);
    PASS();
}
static void test_tf_series(void) {
    TEST("tf_series");
    transfer_function_t G1,G2,Gs;
    double n1[]={1.0},d1[]={1.0,1.0},n2[]={2.0},d2[]={2.0,1.0};
    tf_init(&G1,n1,0,d1,1); tf_init(&G2,n2,0,d2,1);
    assert(tf_series(&G1,&G2,&Gs)==0);
    assert(Gs.den_order==2);
    PASS();
}
static void test_tf_feedback(void) {
    TEST("tf_unity_feedback");
    transfer_function_t G,T;
    double num[]={1.0},den[]={1.0,1.0};
    tf_init(&G,num,0,den,1);
    tf_unity_feedback(&G,&T);
    assert(fabs(T.den[1]-2.0)<1e-10);
    PASS();
}
static void test_ss_conv(void) {
    TEST("tf2ss->ss2tf roundtrip");
    transfer_function_t G1,G2; state_space_t ss;
    double num[]={2.0,3.0},den[]={1.0,4.0,3.0};
    tf_init(&G1,num,1,den,2);
    tf2ss(&G1,&ss); ss2tf(&ss,&G2);
    assert(fabs(tf_dc_gain(&G1)-tf_dc_gain(&G2))<1e-6);
    ss_free(&ss); PASS();
}
static void test_controllability(void) {
    TEST("controllability canonical form");
    transfer_function_t G; state_space_t ss;
    double num[]={1.0},den[]={1.0,3.0,2.0};
    tf_init(&G,num,0,den,2); tf2ss(&G,&ss);
    double *Cm=malloc(4*sizeof(double)); int rank;
    controllability_matrix(&ss,Cm,&rank);
    assert(rank==2); free(Cm); ss_free(&ss); PASS();
}
static void test_routh_stable(void) {
    TEST("Routh-Hurwitz stable (s+1)^3");
    double c[]={1.0,3.0,3.0,1.0}; int rhp;
    routh_hurwitz(c,3,&rhp); assert(rhp==0); PASS();
}
static void test_routh_unstable(void) {
    TEST("Routh-Hurwitz unstable");
    double c[]={1.0,-1.0,2.0,4.0}; int rhp;
    routh_hurwitz(c,3,&rhp); assert(rhp>0); PASS();
}
static void test_err_const(void) {
    TEST("error constants Kp,Kv,Ka");
    transfer_function_t G; double n[]={10.0},d[]={1.0,2.0,1.0};
    tf_init(&G,n,0,d,2); double Kp,Kv,Ka;
    error_constants(&G,&Kp,&Kv,&Ka);
    assert(fabs(Kp-10.0)<1e-10); assert(fabs(Kv)<1e-10); PASS();
}
static void test_zn(void) {
    TEST("Ziegler-Nichols PID");
    pid_params_t p; zn_step_response(1.0,0.5,2.0,2,&p);
    assert(p.Kp>0&&p.Ki>0&&p.Kd>0); PASS();
}
static void test_cc(void) {
    TEST("Cohen-Coon PID");
    pid_params_t p; cohen_coon(1.5,0.3,4.0,2,&p);
    assert(p.Kp>0); PASS();
}
static void test_dc_motor(void) {
    TEST("DC motor model");
    transfer_function_t G; dc_motor_model(0.01,0.1,0.05,0.05,1.0,0.5,&G);
    assert(G.den_order==2); PASS();
}
static void test_margins(void) {
    TEST("Gain/phase margins");
    transfer_function_t G; double n[]={1.0},d[]={1.0,1.0,0.0};
    tf_init(&G,n,0,d,2); freq_specs_t fs; compute_margins(&G,&fs);
    PASS();
}
static void test_rl_real(void) {
    TEST("Root locus real-axis rule");
    double complex p[]={0.0,-2.0},z[]={-4.0};
    /* Between 0 and -2: 1 pole to right → ON RL */
    assert(rl_on_real_axis(-1.0,p,2,z,1)==1);
    /* Between -2 and -4: 2 items to right → OFF RL */
    assert(rl_on_real_axis(-3.0,p,2,z,1)==0);
    PASS();
}
static void test_rl_asym(void) {
    TEST("Root locus asymptotes");
    double a[3]; int na; rl_asymptote_angles(3,1,a,&na);
    assert(na==2); assert(fabs(a[0]-M_PI/2.0)<0.1); PASS();
}
static void test_cruise(void) {
    TEST("Cruise control PI");
    pid_params_t p; cruise_pi_design(1500.0,50.0,0.8,0.5,&p);
    assert(p.Kp>0&&p.Ki>0); PASS();
}
static void test_proc_sim(void) {
    TEST("Process closed-loop sim");
    process_model_t pm={PROCESS_FOPDT,1.0,0.2,3.0,0,0};
    pid_params_t p; cohen_coon(1.0,0.2,3.0,2,&p);
    double t[200],y[200],u[200]; int np;
    process_closed_loop_sim(&pm,&p,1.0,10.0,0.05,-5,5,t,y,u,200,&np);
    assert(np>10);
    assert(fabs(y[np-1]-1.0)<0.5); PASS();
}
int main(void) {
    printf("=== mini-classical-control Test Suite ===\n\n");
    test_tf_init(); test_tf_eval(); test_sys_type();
    test_step_specs(); test_tf_series(); test_tf_feedback();
    test_ss_conv(); test_controllability();
    test_routh_stable(); test_routh_unstable();
    test_err_const(); test_zn(); test_cc();
    test_dc_motor(); test_margins();
    test_rl_real(); test_rl_asym();
    test_cruise(); test_proc_sim();
    printf("\n=== Results: %d/%d tests passed ===\n",tests_passed,tests_total);
    return (tests_passed==tests_total)?0:1;
}
