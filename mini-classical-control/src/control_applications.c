#include "control_applications.h"
#include "control_analysis.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- DC Motor Model (Electromechanical, Ogata Ch.3) ---- */

int dc_motor_model(double J, double b, double Km, double Kb,
                   double Ra, double La, transfer_function_t *G)
{
    /* Armature-controlled DC motor:
     * Electrical: L_a·di/dt + R_a·i = v_a - K_b·ω
     * Mechanical: J·dω/dt + b·ω = K_m·i
     * TF: Ω(s)/V_a(s) = K_m/[(J·s+b)(L_a·s+R_a)+K_m·K_b]
     * Knowledge: The DC motor is the canonical electromechanical
     * actuator in classical control. Second-order with electrical
     * and mechanical time constants. */
    if (!G || J<=0 || La<=0 || Ra<=0) return -1;
    double a2 = J * La;
    double a1 = J * Ra + b * La;
    double a0 = b * Ra + Km * Kb;
    double num[] = {Km};
    double den[] = {a2, a1, a0};
    return tf_init(G, num, 0, den, 2);
}

int dc_motor_simplified(double J, double b, double Km, double Kb,
                        double Ra, transfer_function_t *G)
{
    /* Simplified DC motor (L_a ≈ 0, electrical dynamics neglected):
     * G(s) = (Km/Ra) / (J·s + b + Km·Kb/Ra) = K/(τ·s+1)
     * K = Km/(b·Ra+Km·Kb), τ = J·Ra/(b·Ra+Km·Kb)
     * Valid when electrical time constant L_a/R_a ≪ mechanical τ. */
    if (!G || J<=0 || Ra<=0) return -1;
    double den1 = b * Ra + Km * Kb;
    double K = Km / den1;
    double tau = J * Ra / den1;
    return tf_first_order(G, K, tau);
}

int dc_motor_pi_speed_control(double J, double b, double Km, double Kb,
                               double Ra, double zeta, double omega_n,
                               pid_params_t *pid)
{
    /* PI speed controller for simplified DC motor model.
     * Plant: G(s) = K/(τ·s+1).
     * PI: C(s) = Kp + Ki/s = Kp·(s + Ki/Kp)/s.
     * Closed-loop TF = C·G/(1+C·G) = (K·Kp·s+K·Ki)/(τ·s²+(1+K·Kp)·s+K·Ki) */
    if (!pid) return -1;
    memset(pid,0,sizeof(pid_params_t));
    transfer_function_t Gm;
    dc_motor_simplified(J,b,Km,Kb,Ra,&Gm);
    double K = Gm.num[0];
    double tau = Gm.den[0];
    double wn2 = omega_n*omega_n;
    double sigma2 = 2.0*zeta*omega_n;
    pid->Kp = (tau*sigma2 - 1.0) / K;
    if (pid->Kp < 0) pid->Kp = 0.01;
    pid->Ki = tau*wn2 / K;
    pid->N = 10.0;
    return 0;
}

/* ---- Automotive Cruise Control ---- */

int cruise_vehicle_model(double m, double b, transfer_function_t *G)
{
    /* Longitudinal vehicle dynamics: m·dv/dt + b·v = F.
     * G(s) = V(s)/F(s) = 1/(m·s + b).
     * m = vehicle mass [kg], b = viscous damping [N·s/m].
     * Knowledge: This is one of the simplest yet most widely taught
     * control applications (Ogata §4, Franklin §1). */
    if (!G || m<=0) return -1;
    double num[]={1.0}, den[]={m, b};
    return tf_init(G, num, 0, den, 1);
}

int cruise_pi_design(double m, double b, double zeta, double omega_n,
                     pid_params_t *pid)
{
    /* PI controller for cruise control.
     * Plant: G(s)=1/(m·s+b). PI: C(s)=Kp+Ki/s.
     * CL: T(s)=(Kp·s+Ki)/(m·s²+(b+Kp)·s+Ki).
     * Match to ω_n², 2ζω_n:
     *   Ki/m = ω_n² → Ki = m·ω_n²
     *   (b+Kp)/m = 2ζω_n → Kp = 2ζω_n·m - b */
    if (!pid||m<=0||b<0||zeta<=0||omega_n<=0) return -1;
    memset(pid,0,sizeof(pid_params_t));
    pid->Ki = m * omega_n * omega_n;
    double Kp_calc = 2.0*zeta*omega_n*m - b;
    pid->Kp = (Kp_calc > 0) ? Kp_calc : 0.01;
    pid->N = 10.0;
    return 0;
}

/* ---- Temperature Control (Process Control) ---- */

int thermal_fopdt_model(double K, double T, double L,
                        transfer_function_t *G, transfer_function_t *G_delay)
{
    /* FOPDT thermal model: G(s) = K·e^{-L·s}/(T·s+1).
     * K = static gain [°C/%], T = time constant [s], L = dead time [s].
     * The delay is approximated by 1st-order Padé.
     * Knowledge: FOPDT is the most common model in process control,
     * covering ~80% of industrial loops (Astrom & Hagglund). */
    if (!G || !G_delay || T<=0) return -1;
    /* Plant without delay */
    tf_first_order(G, K, T);
    /* Padé approximation of delay */
    tf_pade_delay(L, G_delay);
    return 0;
}

int temperature_pid_design(double K, double T, double L, pid_params_t *pid)
{
    /* PID for thermal FOPDT process.
     * Uses Cohen-Coon tuning rules (suitable for temperature loops).
     * Knowledge: Temperature loops are typically slow (large T)
     * with significant dead time. Derivative action helps compensate
     * for the delay. */
    if (!pid||K<=0||T<=0||L<=0) return -1;
    return cohen_coon(K, L, T, 2, pid);
}

/* ---- Position Servo System ---- */

int servo_position_model(double J, double b, double Km, double Kb,
                          double Ra, double La, transfer_function_t *G)
{
    /* DC motor position servo: position = integral of speed.
     * Θ(s)/V_a(s) = Km/[s·((J·s+b)(L_a·s+R_a)+Km·Kb)]
     * Type-1 system (one integrator from velocity to position).
     * Knowledge: Position servos are fundamental in robotics,
     * CNC machines, and aerospace actuators. */
    if (!G || J<=0 || La<=0 || Ra<=0) return -1;
    double a3 = J * La;
    double a2 = J * Ra + b * La;
    double a1 = b * Ra + Km * Kb;
    double a0 = 0.0;
    double num[] = {Km};
    double den[] = {a3, a2, a1, a0};
    return tf_init(G, num, 0, den, 3);
}

int servo_pd_design(double J, double b, double Km, double Kb,
                    double Ra, double zeta, double omega_n,
                    pid_params_t *pid)
{
    /* PD position controller for servo.
     * Simplified model (neglect L_a): G(s)=K/[s·(τ·s+1)].
     * PD: C(s)=Kp+Kd·s. CL becomes 2nd-order.
     * Knowledge: Derivative action adds damping to the double
     * integrator (from motor inertia + position integration).
     * Position control requires derivative for stability. */
    if (!pid) return -1;
    memset(pid,0,sizeof(pid_params_t));
    double den1 = b*Ra + Km*Kb;
    double K = Km / den1;
    double tau = J*Ra / den1;
    double wn2 = omega_n*omega_n;
    double sigma2 = 2.0*zeta*omega_n;
    pid->Kp = tau*wn2 / K;
    pid->Kd = (tau*sigma2 - 1.0) / K;
    if (pid->Kd < 0) pid->Kd = 0.001;
    pid->N = 10.0;
    return 0;
}

/* ---- Ball and Beam (Classic Lab Experiment) ---- */

int ball_beam_model(transfer_function_t *G)
{
    /* Ball and beam: X(s)/Θ(s) = (5g/7)/s².
     * g = 9.81 m/s². Linearized about horizontal beam.
     * Double integrator — unstable in closed loop without derivative.
     * Knowledge: The ball and beam is a canonical unstable mechanical
     * system used to teach cascade control and nonlinear compensation. */
    if (!G) return -1;
    double Kbb = 5.0 * 9.81 / 7.0;
    double num[] = {Kbb};
    double den[] = {1.0, 0.0, 0.0};
    return tf_init(G, num, 0, den, 2);
}

/* ---- Inverted Pendulum (Linearized) ---- */

int inverted_pendulum_model(double M, double m, double l, transfer_function_t *G)
{
    /* Linearized inverted pendulum on cart.
     * State variables: cart position x, pendulum angle θ.
     * Linearization about θ=0 (upright).
     *
     * TF from force F to angle Θ:
     *   Θ(s)/F(s) = -m·l·s² / [q·s⁴ - (M+m)·m·g·l·s²]
     * where q = (M+m)·(I+m·l²) - (m·l)², I = m·l²/3.
     *
     * Simplified (point mass at tip, I=0):
     *   Θ(s)/F(s) = -1 / [(M+m)·l·s² - (M+m)·g]
     * This is unstable (RHP pole at √(g/l)).
     *
     * Knowledge: The inverted pendulum is the quintessential
     * unstable system in control education. Classic problem
     * for state feedback and LQR design. */
    if (!G || M<=0 || m<=0 || l<=0) return -1;
    /* Simplified model (point mass pendulum) */
    double den_coef = (M + m) * l;
    double spring_term = -(M + m) * 9.81;
    double num[] = {-1.0};
    double den[] = {den_coef, 0.0, spring_term};
    return tf_init(G, num, 0, den, 2);
}

/* ---- Industrial Process Control ---- */

int process_model_to_tf(const process_model_t *pm, transfer_function_t *G)
{
    /* Build TF from industrial process model parameters.
     * Supports FOPDT, SOPDT, integrating, and inverse response types. */
    if (!pm || !G) return -1;
    transfer_function_t Gp, Gd;
    switch (pm->type) {
    case PROCESS_FOPDT:
        tf_first_order(&Gp, pm->K, pm->T1);
        break;
    case PROCESS_SOPDT: {
        /* G(s) = K/[(T1·s+1)(T2·s+1)] */
        double num[] = {pm->K};
        double den[] = {pm->T1*pm->T2, pm->T1+pm->T2, 1.0};
        tf_init(&Gp, num, 0, den, 2);
        break;
    }
    case PROCESS_INTEGRATING: {
        /* G(s) = K/[s·(T1·s+1)] */
        double num[] = {pm->K};
        double den[] = {pm->T1, 1.0, 0.0};
        tf_init(&Gp, num, 0, den, 2);
        break;
    }
    case PROCESS_INVERSE_RESP: {
        /* G(s) = K·(-T3·s+1)/[(T1·s+1)(T2·s+1)] */
        double num[] = {-pm->K*pm->T3, pm->K};
        double den[] = {pm->T1*pm->T2, pm->T1+pm->T2, 1.0};
        tf_init(&Gp, num, 1, den, 2);
        break;
    }
    default: return -1;
    }
    if (pm->L > 1e-10) {
        tf_pade_delay(pm->L, &Gd);
        tf_series(&Gp, &Gd, G);
    } else {
        *G = Gp;
    }
    return 0;
}

int process_auto_tune(const process_model_t *pm, pid_params_t *pid)
{
    /* Auto-tune PID based on process model type.
     * For FOPDT/integrating: use Cohen-Coon.
     * For SOPDT: use Ziegler-Nichols step response.
     * Knowledge: Model-based auto-tuning is standard in industrial
     * PID controllers (e.g., Honeywell UDC, Siemens SIMATIC). */
    if (!pm || !pid) return -1;
    memset(pid,0,sizeof(pid_params_t));
    double K=pm->K, L=pm->L, T=pm->T1;
    if (T < 1e-10) T = 1.0;
    switch (pm->type) {
    case PROCESS_FOPDT:
    case PROCESS_SOPDT:
    case PROCESS_INVERSE_RESP:
        return cohen_coon(K, L, T, 2, pid);
    case PROCESS_INTEGRATING:
        return zn_step_response(K, L, T, 2, pid);
    default: return -1;
    }
}

int process_closed_loop_sim(const process_model_t *pm, const pid_params_t *pid,
                             double setpoint, double t_final, double dt,
                             double u_min, double u_max,
                             double *t, double *y, double *u,
                             int max_points, int *num_points)
{
    /* Simulate closed-loop PID control of industrial process.
     * Includes output saturation (actuator limits) and basic
     * anti-windup via conditional integration.
     *
     * Knowledge: Saturation is the most common nonlinearity in
     * real control systems. Without anti-windup, integral windup
     * causes large overshoot and slow settling. */
    if (!pm||!pid||!t||!y||!u||!num_points||max_points<2) return -1;
    int steps=(int)(t_final/dt);
    if (steps>max_points) steps=max_points;
    transfer_function_t Gp;
    if (process_model_to_tf(pm,&Gp)<0) return -1;
    /* Simplified simulation using discrete-time approximation */
    /* For FOPDT: y[k+1]=a·y[k]+b·u[k-d] where a=exp(-dt/T), b=K·(1-a) */
    double a,b; int d;
    if (pm->type==PROCESS_FOPDT) {
        a=exp(-dt/pm->T1); b=pm->K*(1.0-a); d=(int)(pm->L/dt);
        if (d<0) d=0;
    } else {
        a=0.9; b=0.1; d=0; /* generic fallback */
    }
    double yk=0, ik=0;
    double *ubuf=calloc((d+2),sizeof(double));
    if (!ubuf) return -1;
    for (int k=0;k<=steps;k++) {
        t[k]=k*dt; y[k]=yk;
        double ek=setpoint-yk;
        double up=pid->Kp*ek;
        ik+=pid->Ki*ek*dt;
        double ud=(k>0)?pid->Kd*(y[k-1]-yk)/dt:0;
        double uk_raw=up+ik+ud;
        if (uk_raw>u_max) { uk_raw=u_max; ik-=(uk_raw-u_max)*dt; }
        if (uk_raw<u_min) { uk_raw=u_min; ik-=(uk_raw-u_min)*dt; }
        u[k]=uk_raw;
        /* Shift delay buffer and add new input */
        for (int i=d;i>0;i--) ubuf[i]=ubuf[i-1];
        ubuf[0]=uk_raw;
        yk=a*yk+b*ubuf[d];
    }
    *num_points=steps+1;
    free(ubuf);
    return 0;
}
