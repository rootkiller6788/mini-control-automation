/**
 * @file pid_applications.c
 * @brief PID Control Applications Implementation
 *
 * Implements domain-specific models and helpers for:
 *   L6 -- DC motor speed/position control, temperature control,
 *         quadrotor attitude, liquid level, inverted pendulum
 *   L7 -- Auto-tuning selection, performance scoring, tuning comparison
 *
 * References:
 *   Ogata (2010), "Modern Control Engineering", 5th Ed., Prentice Hall
 *   Franklin, Powell & Emami-Naeini (2014), "Feedback Control of Dynamic
 *     Systems", 7th Ed., Pearson
 *   Dorf & Bishop (2016), "Modern Control Systems", 13th Ed., Pearson
 *   Astrom & Hagglund (2006), "Advanced PID Control", ISA
 */

#include "pid_applications.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

/*===========================================================================
 * L6 -- DC Motor Model
 *===========================================================================*/

void dc_motor_init(DCMotorModel *motor, int type) {
    if (!motor) return;
    memset(motor, 0, sizeof(*motor));

    switch (type) {
        case 0: /* Small hobby motor (e.g., Mabuchi FA-130) */
            motor->R = 3.5;
            motor->L = 0.0015;
            motor->Ke = 0.005;
            motor->Kt = 0.005;
            motor->J = 1e-6;
            motor->b = 1e-7;
            break;
        case 1: /* Medium industrial (e.g., 100W DC servo) */
            motor->R = 1.0;
            motor->L = 0.005;
            motor->Ke = 0.05;
            motor->Kt = 0.05;
            motor->J = 1e-4;
            motor->b = 1e-5;
            break;
        case 2: /* Large industrial (e.g., 1kW DC motor) */
            motor->R = 0.5;
            motor->L = 0.01;
            motor->Ke = 0.5;
            motor->Kt = 0.5;
            motor->J = 0.01;
            motor->b = 0.001;
            break;
        case 3: /* Micro motor (e.g., pager/vibration) */
            motor->R = 10.0;
            motor->L = 0.0005;
            motor->Ke = 0.001;
            motor->Kt = 0.001;
            motor->J = 1e-8;
            motor->b = 1e-9;
            break;
        default:
            motor->R = 1.0;
            motor->L = 0.001;
            motor->Ke = 0.01;
            motor->Kt = 0.01;
            motor->J = 1e-5;
            motor->b = 1e-6;
    }
}

int dc_motor_to_fopdt(const DCMotorModel *motor, FOPDTModel *fopdt,
                      double voltage_range) {
    if (!motor || !fopdt) return -1;

    /* Simplified first-order model (L << R):
     * G(s) = Kt / ((L*s+R)*(J*s+b) + Kt*Ke)
     *      ? Kt / (R*(J*s+b) + Kt*Ke)        [neglect Ls terms]
     *      = Kt / (R*J*s + R*b + Kt*Ke)
     *      = (Kt/(R*b+Kt*Ke)) / (R*J/(R*b+Kt*Ke) * s + 1)
     *
     * K = Kt / (R*b + Kt*Ke)
     * tau = R*J / (R*b + Kt*Ke)
     */
    double denom = motor->R * motor->b + motor->Kt * motor->Ke;
    if (denom < 1e-30) denom = 1e-30;
    double K_speed = motor->Kt / denom;
    double tau = motor->R * motor->J / denom;

    fopdt->K = K_speed * voltage_range;
    fopdt->T = tau;
    fopdt->L = motor->L / motor->R; /* electrical time constant as dead time approx */
    if (fopdt->L < 0.0) fopdt->L = 0.0;

    return 0;
}

int dc_motor_simulate(const DCMotorModel *motor, double voltage,
                      double dt, double duration,
                      double *time, double *speed, double *current, size_t N) {
    if (!motor || !time || !speed || !current || N < 2) return -1;

    double i = 0.0;   /* current */
    double w = 0.0;   /* angular velocity */
    double V = voltage;

    /* Euler integration of coupled ODEs:
     * L*di/dt = V - R*i - Ke*w
     * J*dw/dt = Kt*i - b*w - T_load
     */
    for (size_t k = 0; k < N; k++) {
        time[k] = k * dt;

        double di = (V - motor->R * i - motor->Ke * w) / motor->L * dt;
        double dw = (motor->Kt * i - motor->b * w - motor->T_load) / motor->J * dt;

        i += di;
        w += dw;

        speed[k] = w;
        current[k] = i;
    }

    return 0;
}

/*===========================================================================
 * L6 -- Thermal Model
 *===========================================================================*/

void thermal_model_init(ThermalModel *tm, int type) {
    if (!tm) return;
    memset(tm, 0, sizeof(*tm));

    switch (type) {
        case 0: /* Small oven/incubator (e.g., lab oven, 1L) */
            tm->mass = 0.1;
            tm->Cp = 900.0;    /* aluminum */
            tm->Rth = 5.0;
            tm->T_ambient = 25.0;
            tm->max_power = 100.0;
            break;
        case 1: /* Water bath (e.g., 20L) */
            tm->mass = 20.0;
            tm->Cp = 4186.0;   /* water */
            tm->Rth = 0.5;
            tm->T_ambient = 25.0;
            tm->max_power = 2000.0;
            break;
        case 2: /* Large industrial furnace (e.g., 1000L) */
            tm->mass = 500.0;
            tm->Cp = 500.0;    /* steel */
            tm->Rth = 0.01;
            tm->T_ambient = 25.0;
            tm->max_power = 50000.0;
            break;
        case 3: /* Peltier/thermoelectric */
            tm->mass = 0.01;
            tm->Cp = 385.0;    /* copper */
            tm->Rth = 10.0;
            tm->T_ambient = 25.0;
            tm->max_power = 50.0;
            break;
        default:
            tm->mass = 1.0;
            tm->Cp = 1000.0;
            tm->Rth = 1.0;
            tm->T_ambient = 25.0;
            tm->max_power = 100.0;
    }
}

void thermal_to_fopdt(const ThermalModel *tm, FOPDTModel *fopdt) {
    if (!tm || !fopdt) return;
    fopdt->K = tm->Rth;    /* steady-state temp rise per watt */
    fopdt->T = tm->mass * tm->Cp * tm->Rth; /* thermal time constant */
    fopdt->L = 0.1 * fopdt->T; /* small delay for sensor/actuator */
    if (fopdt->L < 0.1) fopdt->L = 1.0;
}

int thermal_simulate(const ThermalModel *tm, double power,
                     double dt, double duration,
                     double *time, double *temp, size_t N) {
    if (!tm || !time || !temp || N < 2) return -1;

    double T = tm->T_ambient;
    double P = (power < 0.0) ? 0.0 : (power > tm->max_power ? tm->max_power : power);

    /* dT/dt = (P - (T - T_ambient)/Rth) / (m*Cp) */
    double mCp = tm->mass * tm->Cp;
    if (mCp < 1e-30) mCp = 1.0;

    for (size_t k = 0; k < N; k++) {
        time[k] = k * dt;
        double dT = (P - (T - tm->T_ambient) / tm->Rth) / mCp * dt;
        T += dT;
        temp[k] = T;
    }

    return 0;
}

/*===========================================================================
 * L6 -- Quadrotor Axis Model
 *===========================================================================*/

void quadrotor_axis_init(QuadrotorAxisModel *qm, int type) {
    if (!qm) return;
    memset(qm, 0, sizeof(*qm));

    switch (type) {
        case 0: /* Micro (e.g., Crazyflie 2.0, 30g) */
            qm->J = 1.4e-5;
            qm->b = 1e-6;
            qm->arm_length = 0.04;
            qm->thrust_coeff = 1.3e-8;
            qm->max_torque = 0.002;
            break;
        case 1: /* Mini (e.g., 250mm racing drone, 500g) */
            qm->J = 0.0015;
            qm->b = 0.001;
            qm->arm_length = 0.125;
            qm->thrust_coeff = 1e-6;
            qm->max_torque = 0.1;
            break;
        case 2: /* Medium (e.g., DJI Phantom, 450mm, 1.5kg) */
            qm->J = 0.02;
            qm->b = 0.005;
            qm->arm_length = 0.225;
            qm->thrust_coeff = 5e-6;
            qm->max_torque = 0.5;
            break;
        case 3: /* Large (e.g., 650mm+, 5kg) */
            qm->J = 0.15;
            qm->b = 0.02;
            qm->arm_length = 0.35;
            qm->thrust_coeff = 2e-5;
            qm->max_torque = 2.0;
            break;
        default:
            qm->J = 0.01;
            qm->b = 0.002;
            qm->arm_length = 0.2;
            qm->thrust_coeff = 1e-6;
            qm->max_torque = 0.2;
    }
}

void quadrotor_to_sopdt(const QuadrotorAxisModel *qm, SOPDTModel *sopdt) {
    if (!qm || !sopdt) return;

    /* G(s) = 1/(J*s^2 + b*s) = (1/b) / (s*(J/b*s + 1))
     *
     * For position control (cascaded): the inner rate loop is first-order:
     *   G_rate(s) = (1/b) / (J/b*s + 1)
     *   K_rate = 1/b, T_rate = J/b
     */
    sopdt->K = 1.0 / qm->b;
    sopdt->T1 = qm->J / qm->b;
    sopdt->T2 = 0.01; /* actuator dynamics */
    sopdt->L = 0.005; /* sensor/ESC delay */
}

int quadrotor_simulate(const QuadrotorAxisModel *qm, double torque,
                       double dt, double duration,
                       double *time, double *angle, double *rate, size_t N) {
    if (!qm || !time || !angle || !rate || N < 2) return -1;

    double theta = 0.0;   /* angle */
    double omega = 0.0;   /* angular rate */
    double tau = torque;
    if (tau > qm->max_torque) tau = qm->max_torque;
    if (tau < -qm->max_torque) tau = -qm->max_torque;

    for (size_t k = 0; k < N; k++) {
        time[k] = k * dt;

        /* J*domega = tau - b*omega */
        double domega = (tau - qm->b * omega) / qm->J * dt;
        omega += domega;
        theta += omega * dt;

        angle[k] = theta;
        rate[k] = omega;
    }

    return 0;
}

/*===========================================================================
 * L6 -- Tank Level Model
 *===========================================================================*/

void tank_model_init(TankModel *tm, int type) {
    if (!tm) return;
    memset(tm, 0, sizeof(*tm));
    tm->g = 9.81;

    switch (type) {
        case 0: /* Small lab tank */
            tm->A = 0.01;
            tm->Cv = 1e-5;
            tm->max_level = 0.5;
            tm->operating_level = 0.25;
            break;
        case 1: /* Medium process tank */
            tm->A = 1.0;
            tm->Cv = 0.001;
            tm->max_level = 3.0;
            tm->operating_level = 1.5;
            break;
        case 2: /* Large storage tank */
            tm->A = 50.0;
            tm->Cv = 0.05;
            tm->max_level = 10.0;
            tm->operating_level = 5.0;
            break;
        default:
            tm->A = 1.0;
            tm->Cv = 0.001;
            tm->max_level = 2.0;
            tm->operating_level = 1.0;
    }
}

void tank_to_fopdt(const TankModel *tm, FOPDTModel *fopdt) {
    if (!tm || !fopdt) return;

    double h0 = tm->operating_level;
    if (h0 < 0.01) h0 = 0.01;
    double g = tm->g;

    /* Linearized around h0:
     * A*d(delta_h)/dt = delta_q_in - Cv*g/sqrt(2*g*h0) * delta_h
     *
     * tau = A * sqrt(2*h0/g) / Cv
     * K = sqrt(2*h0/g) / Cv
     */
    double sqrt_term = sqrt(2.0 * h0 / g);
    double tau = tm->A * sqrt_term / tm->Cv;
    double K = sqrt_term / tm->Cv;

    fopdt->K = K;
    fopdt->T = tau;
    fopdt->L = 0.0;
}

int tank_simulate(const TankModel *tm, double q_in,
                  double dt, double duration,
                  double *time, double *level, size_t N) {
    if (!tm || !time || !level || N < 2) return -1;

    double h = tm->operating_level;
    if (h < 0.0) h = 0.0;

    for (size_t k = 0; k < N; k++) {
        time[k] = k * dt;

        /* dh/dt = (q_in - Cv*sqrt(2*g*h)) / A */
        double outflow;
        if (h > 0.0) {
            outflow = tm->Cv * sqrt(2.0 * tm->g * h);
        } else {
            outflow = 0.0;
        }

        double dh = (q_in - outflow) / tm->A * dt;
        h += dh;
        if (h < 0.0) h = 0.0;
        if (h > tm->max_level) h = tm->max_level;

        level[k] = h;
    }

    return 0;
}

/*===========================================================================
 * L6 -- Inverted Pendulum Model
 *===========================================================================*/

void inv_pendulum_init(InvertedPendulumModel *ipm, int type) {
    if (!ipm) return;
    memset(ipm, 0, sizeof(*ipm));
    ipm->g = 9.81;

    switch (type) {
        case 0: /* Classic cart-pendulum (lab scale) */
            ipm->M = 0.5;
            ipm->m = 0.1;
            ipm->l = 0.3;
            ipm->I = ipm->m * ipm->l * ipm->l / 3.0; /* rod about pivot */
            ipm->max_force = 5.0;
            break;
        case 1: /* Small segway-type */
            ipm->M = 5.0;
            ipm->m = 1.0;
            ipm->l = 0.5;
            ipm->I = ipm->m * ipm->l * ipm->l / 3.0;
            ipm->max_force = 20.0;
            break;
        case 2: /* Larger platform */
            ipm->M = 20.0;
            ipm->m = 5.0;
            ipm->l = 1.0;
            ipm->I = ipm->m * ipm->l * ipm->l / 3.0;
            ipm->max_force = 100.0;
            break;
        default:
            ipm->M = 1.0;
            ipm->m = 0.2;
            ipm->l = 0.5;
            ipm->I = ipm->m * ipm->l * ipm->l / 3.0;
            ipm->max_force = 10.0;
    }
}

int inv_pendulum_simulate(const InvertedPendulumModel *ipm, double force,
                          double dt, double duration,
                          double *time, double *theta_array, double *x_array,
                          size_t N) {
    if (!ipm || !time || !theta_array || !x_array || N < 2) return -1;

    /* State: [x, theta, x_dot, theta_dot]
     *
     * Nonlinear equations (from Lagrangian):
     * (M+m)*ddx + m*l*ddtheta*cos(theta) - m*l*dtheta^2*sin(theta) = F
     * (I+m*l^2)*ddtheta + m*l*ddx*cos(theta) - m*g*l*sin(theta) = 0
     *
     * Solved for ddtheta and ddx at each step.
     */

    double x = 0.0, theta = 0.01; /* small initial offset */
    double x_dot = 0.0, theta_dot = 0.0;

    double M = ipm->M, m = ipm->m, l = ipm->l;
    double I = ipm->I, g = ipm->g;
    double F = force;
    if (F > ipm->max_force) F = ipm->max_force;
    if (F < -ipm->max_force) F = -ipm->max_force;

    for (size_t k = 0; k < N; k++) {
        time[k] = k * dt;

        /* RK4 integration step */
        double s[4] = {x, theta, x_dot, theta_dot};

        /* Single Euler step (simplified)
         *
         * Mass matrix: [M+m,      m*l*cos(theta)]
         *              [m*l*cos(theta), I+m*l^2  ]
         *
         * RHS: [F + m*l*dtheta^2*sin(theta)]
         *      [m*g*l*sin(theta)           ]
         *
         * Solve 2x2 system for [ddx; ddtheta] */
        double ct = cos(theta);
        double st = sin(theta);
        double m11 = M + m;
        double m12 = m * l * ct;
        double m22 = I + m * l * l;
        double det = m11 * m22 - m12 * m12;

        if (fabs(det) < 1e-30) det = 1e-30;

        double rhs1 = F + m * l * theta_dot * theta_dot * st;
        double rhs2 = m * g * l * st;

        double ddx = (m22 * rhs1 - m12 * rhs2) / det;
        double ddtheta = (m11 * rhs2 - m12 * rhs1) / det;

        /* Euler update */
        x += x_dot * dt;
        theta += theta_dot * dt;
        x_dot += ddx * dt;
        theta_dot += ddtheta * dt;

        theta_array[k] = theta;
        x_array[k] = x;
    }

    return 0;
}

/*===========================================================================
 * L7 -- Application Helper Functions
 *===========================================================================*/

int pid_autotune(const FOPDTModel *model, PIDForm form,
                 double ts, PIDTuningResult *result) {
    if (!model || !result) return -1;

    double L = model->L, T = model->T;
    if (T < 1e-30) T = 1e-30;
    double ratio = L / T;

    /* Select tuning method based on L/T ratio */
    if (ratio < 0.1) {
        /* Easy process: use IMC with moderate lambda */
        return pid_tune_imc(model, T, form, ts, result);
    } else if (ratio < 0.5) {
        /* Medium difficulty: use AMIGO */
        return pid_tune_amigo(model, form, ts, result);
    } else if (ratio < 1.0) {
        /* Significant dead time: Cohen-Coon */
        return pid_tune_cohen_coon(model, form, ts, result);
    } else {
        /* Dead-time dominant: Ziegler-Nichols */
        return pid_tune_zn_open_loop(model, form, ts, result);
    }
}

int pid_performance_score(const PIDController *pid, const FOPDTModel *model,
                          double *score, StepResponseMetrics *metrics) {
    if (!pid || !model || !score || !metrics) return -1;

    /* Run step response simulation */
    double duration = 10.0 * (model->T + model->L);
    if (duration < 10.0) duration = 10.0;
    if (duration > 1000.0) duration = 1000.0;

    double Ts = pid->params.Ts;
    if (Ts <= 0.0) Ts = 0.01;
    size_t N = (size_t)(duration / Ts);
    if (N < 100) N = 100;
    if (N > 100000) N = 100000;

    double *time = (double*)malloc(N * sizeof(double));
    double *output = (double*)malloc(N * sizeof(double));
    double *control = (double*)malloc(N * sizeof(double));

    if (!time || !output || !control) {
        free(time); free(output); free(control);
        return -1;
    }

    int ret = pid_simulate_step_response(pid, model, 1.0, duration, Ts,
                                         time, output, control, N);
    if (ret != 0) {
        free(time); free(output); free(control);
        return -1;
    }

    ret = pid_step_metrics(time, output, N, 1.0, metrics);

    /* Compute score (0-100):
     * - Overshoot penalty: -2 points per % overshoot (max -50)
     * - Settling time: -1 point per unit beyond T+L (max -30)
     * - Steady-state error: -10 per % error (max -20)
     * Start from 100, subtract penalties. */
    double sc = 100.0;

    /* Overshoot penalty */
    double os_pct = metrics->overshoot * 100.0;
    if (os_pct > 0.0) {
        sc -= 2.0 * os_pct;
        if (sc < 50.0) sc = 50.0 + (sc - 50.0) * 0.5; /* soft floor at 50 */
    }

    /* Settling time penalty (relative to open-loop time constant) */
    double tau_ref = model->T + model->L;
    if (tau_ref > 0.0 && metrics->settling_time > tau_ref) {
        double excess = (metrics->settling_time - tau_ref) / tau_ref;
        sc -= 10.0 * excess;
    }

    /* Steady-state error penalty */
    double ess_pct = fabs(metrics->steady_state_error) * 100.0;
    sc -= 5.0 * ess_pct;

    /* Clamp */
    if (sc < 0.0) sc = 0.0;
    if (sc > 100.0) sc = 100.0;

    *score = sc;

    free(time); free(output); free(control);
    return 0;
}

const char *pid_process_type_name(const FOPDTModel *model, char *buf, size_t size) {
    if (!model || !buf || size == 0) return "Invalid";
    double ratio = model->L / (model->T + 1e-30);

    if (ratio < 0.1)      snprintf(buf, size, "Lag-dominant (L/T=%.3f)", ratio);
    else if (ratio < 0.3)  snprintf(buf, size, "Balanced (L/T=%.3f)", ratio);
    else if (ratio < 0.7)  snprintf(buf, size, "Moderate dead-time (L/T=%.3f)", ratio);
    else if (ratio < 1.5)  snprintf(buf, size, "Dead-time dominant (L/T=%.3f)", ratio);
    else                   snprintf(buf, size, "Severe dead-time: Smith predictor recommended (L/T=%.3f)", ratio);

    return buf;
}

void pid_tuning_comparison(const FOPDTModel *model, double ts) {
    if (!model) return;
    printf("\n============================================================\n");
    printf("  PID Tuning Comparison\n");
    printf("  Process: K=%.4g  T=%.4g  L=%.4g  (L/T=%.3f)\n",
           model->K, model->T, model->L, model->L / (model->T + 1e-30));
    printf("============================================================\n");
    printf("%-30s %8s %8s %8s %8s\n",
           "Method", "Kp", "Ti", "Td", "Score");
    printf("------------------------------------------------------------\n");

    /* Run each tuning method and evaluate */
    PIDTuningResult results[10];
    int n_results = 0;

    /* Collect tuning results */
    if (pid_tune_zn_open_loop(model, PID_FORM_STANDARD, ts,
                              &results[n_results]) == 0) n_results++;
    if (pid_tune_cohen_coon(model, PID_FORM_STANDARD, ts,
                            &results[n_results]) == 0) n_results++;
    if (pid_tune_amigo(model, PID_FORM_STANDARD, ts,
                       &results[n_results]) == 0) n_results++;
    if (pid_tune_imc(model, model->T, PID_FORM_STANDARD, ts,
                     &results[n_results]) == 0) n_results++;
    if (pid_tune_lambda(model, model->T, PID_FORM_STANDARD, ts,
                        &results[n_results]) == 0) n_results++;
    if (pid_tune_chr_setpoint(model, 0, PID_FORM_STANDARD, ts,
                              &results[n_results]) == 0) n_results++;
    if (pid_tune_chr_disturbance(model, 0, PID_FORM_STANDARD, ts,
                                 &results[n_results]) == 0) n_results++;

    /* Evaluate and print each */
    for (int i = 0; i < n_results; i++) {
        PIDController pid;
        pid_init(&pid, PID_FORM_STANDARD);
        pid_apply_tuning(&pid, &results[i]);

        double score;
        StepResponseMetrics metrics;
        pid_performance_score(&pid, model, &score, &metrics);

        printf("%-30s %8.4f %8.4f %8.4f %7.1f\n",
               pid_tuning_method_name(results[i].method),
               results[i].params.Kp,
               results[i].params.Ti,
               results[i].params.Td,
               score);
    }
    printf("============================================================\n");
}
