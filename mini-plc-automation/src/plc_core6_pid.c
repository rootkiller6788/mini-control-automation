#include "plc_core.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void plc_pid_init(plc_pid_t *pid, double kp, double ki, double kd,
                  double ts_sec, double out_min, double out_max)
{
    if (!pid) return;
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->ts_sec = ts_sec;
    pid->setpoint = 0.0; pid->integral = 0.0;
    pid->last_error = 0.0; pid->output = 0.0;
    pid->out_min = out_min; pid->out_max = out_max;
    pid->anti_windup = 1; pid->auto_mode = 1;
}

double plc_pid_update(plc_pid_t *pid, double setpoint, double measurement)
{
    if (!pid || !pid->auto_mode) return pid ? pid->output : 0.0;
    double error = setpoint - measurement;
    double ts = pid->ts_sec;
    if (ts < 1e-12) ts = 0.001;
    double p_term = pid->kp * error;
    double new_integral = pid->integral + pid->ki * ts * error;
    double d_term = (pid->kd / ts) * (error - pid->last_error);
    double u = p_term + new_integral + d_term;
    if (u > pid->out_max) {
        u = pid->out_max;
        if (!pid->anti_windup || error <= 0) pid->integral = new_integral;
    } else if (u < pid->out_min) {
        u = pid->out_min;
        if (!pid->anti_windup || error >= 0) pid->integral = new_integral;
    } else {
        pid->integral = new_integral;
    }
    pid->last_error = error;
    pid->output = u;
    pid->setpoint = setpoint;
    return u;
}

void plc_pid_reset(plc_pid_t *pid)
{ if (pid) { pid->integral = 0.0; pid->last_error = 0.0; pid->output = 0.0; } }

void plc_pid_ziegler_nichols_open(double K, double tau, double theta,
                                  double ts_sec, double out_min, double out_max,
                                  plc_pid_t *pid)
{
    if (!pid) return;
    if (fabs(K) < 1e-12 || fabs(theta) < 1e-12) {
        plc_pid_init(pid, 1.0, 0.0, 0.0, ts_sec, out_min, out_max);
        return;
    }
    double kp = 1.2 * tau / (K * theta);
    double ti = 2.0 * theta;
    double td = 0.5 * theta;
    plc_pid_init(pid, kp, kp/ti, kp*td, ts_sec, out_min, out_max);
}

void plc_pid_ziegler_nichols_closed(double amplitude, double osc_period,
                                    double osc_ampl, double ts_sec,
                                    double out_min, double out_max,
                                    plc_pid_t *pid)
{
    if (!pid) return;
    double Ku = (fabs(osc_ampl) < 1e-12) ? 1.0 :
                 4.0 * amplitude / (M_PI * osc_ampl);
    double Tu = osc_period;
    double kp = 0.60 * Ku;
    double ti = Tu / 2.0;
    double td = Tu / 8.0;
    plc_pid_init(pid, kp, kp/ti, kp*td, ts_sec, out_min, out_max);
}
