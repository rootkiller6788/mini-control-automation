#include "plc_core.h"
#include <stdio.h>
#include <math.h>

/* Water Tank Level Control using PID on PLC
 * Plant: A*dh/dt = K_valve*u - K_drain*sqrt(h)
 * PID tuned via Ziegler-Nichols open-loop method */

static double tank_level = 0.5;
static double k_drain = 0.02;

static double sim_tank(double valve_pos, double dt_sec)
{
    double area = 2.0, k_valve = 0.05, h_max = 3.0;
    double q_in = k_valve * valve_pos;
    double q_out = k_drain * sqrt(tank_level);
    if (tank_level < 0.0) q_out = 0.0;
    tank_level += (q_in - q_out) * dt_sec / area;
    if (tank_level < 0.0) tank_level = 0.0;
    if (tank_level > h_max) tank_level = h_max;
    return tank_level;
}

static int level_ctrl(plc_system_t *plc, double dt_ms, void *user_data)
{
    plc_pid_t *pid = (plc_pid_t*)user_data;
    if (!pid) return -1;
    double dt_sec = dt_ms / 1000.0;
    double level = sim_tank(pid->output, dt_sec);
    plc_write_internal_reg(plc, 0, level);
    double valve = plc_pid_update(pid, pid->setpoint, level);
    plc_write_analog_output(plc, 0, valve);
    plc_write_internal_reg(plc, 1, valve);
    return 0;
}

int main(void)
{
    plc_system_t plc; plc_init(&plc);

    plc_pid_t pid;
    /* ZN-OL: K=5, tau=100s, theta=5s -> Kp=4.8, Ki=0.48, Kd=12.0 */
    plc_pid_ziegler_nichols_open(5.0, 100.0, 5.0, 1.0, 0.0, 1.0, &pid);
    pid.setpoint = 2.0;

    plc_set_mode(&plc, PLC_MODE_RUN);

    printf("Water Tank Level PID Control Demo\n");
    printf("================================\n");
    printf("PID gains: Kp=%.3f Ki=%.4f Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
    printf("Setpoint: %.1f m\n\n", pid.setpoint);

    for (int i = 0; i < 500; i++) {
        if (i == 250) {
            printf("--- Disturbance: drain increased ---\n");
            k_drain = 0.03;
        }
        plc_scan_cycle(&plc, level_ctrl, &pid, 1000.0);
        if (i % 50 == 0) {
            double lvl = plc_read_internal_reg(&plc, 0);
            double vlv = plc_read_analog_output(&plc, 0) * 100.0;
            printf("t=%4ds  Level=%.3fm  Valve=%.1f%%\n", i+1, lvl, vlv);
        }
    }

    printf("\nFinal level: %.3f m (setpoint: %.1f m)\n",
           plc_read_internal_reg(&plc, 0), pid.setpoint);
    printf("Demo complete.\n");
    return 0;
}
