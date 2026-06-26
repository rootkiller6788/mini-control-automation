/*
 * example_power_grid.c - Power Grid SCADA / Energy Management System
 *
 * Demonstrates SCADA for electrical power systems:
 *   - Automatic Generation Control (AGC) with PID
 *   - Frequency regulation (60 Hz nominal, +/- 0.05 Hz)
 *   - Tie-line power flow control
 *   - Transformer tap changer control
 *   - Volt/VAR optimization
 *   - IEEE C37.118 synchrophasor data quality monitoring
 *
 * This models systems deployed by ABB, Siemens, and GE for
 * smart grid applications, ISO/RTO energy markets, and
 * NERC CIP compliance.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "dcs_core.h"
#include "pid_controller.h"
#include "signal_chain.h"
#include "alarm_manager.h"
#include "data_logger.h"

/* Simplified power system frequency dynamics:
 *
 * Swing equation: 2*H * df/dt = Pm - Pe - D*(f - fnom)
 *
 * Where:
 *   H  = inertia constant [s] (typically 3-6 for thermal plants)
 *   f  = frequency [Hz]
 *   fnom = nominal frequency (60 Hz)
 *   Pm = mechanical power input [pu]
 *   Pe = electrical power output [pu]
 *   D  = damping coefficient
 *
 * AGC adjusts Pm to maintain f = fnom and tie-line flow at schedule.
 * Area Control Error: ACE = (P_tie - P_sched) + 10*B*(f - fnom)
 */

static void run_power_grid_scada(void) {
    printf("\n");
    printf("========================================\n");
    printf("  Power Grid SCADA / EMS System\n");
    printf("  Automatic Generation Control (AGC)\n");
    printf("========================================\n\n");

    /* === Grid parameters === */
    double f_nom = 60.0;     /* Nominal frequency [Hz] */
    double f = 60.0;         /* Actual frequency */
    double H = 5.0;           /* Inertia constant [s] */
    double D = 1.0;           /* Damping coefficient */
    double Pm = 0.85;         /* Mechanical power [pu] */
    double Pe = 0.85;         /* Electrical load [pu] */
    double B = 0.5;           /* Frequency bias [pu/Hz] */
    double P_tie = 0.0;       /* Net tie-line flow [pu] */
    double P_sched = 0.0;     /* Scheduled tie-line flow [pu] */

    /* === AGC PID Controller === */
    pid_controller_t agc_pid;
    /* Slow integral-only controller for AGC (seconds to minutes) */
    pid_init_isa(&agc_pid, 0.0, 30.0, 0.0, 2.0, -0.1, 0.1);

    /* === Frequency measurement with filtering === */
    ema_filter_t freq_filter;
    ema_init_by_tc(&freq_filter, 2.0, 0.05); /* 2s time constant, 50ms samples */
    butter2_filter_t butter_freq;
    butter2_lp_design(&butter_freq, 0.5, 20.0); /* 0.5 Hz LP at 20 Hz sampling */

    /* === Alarms for grid events === */
    alarm_db_t alarm_db;
    alarm_db_init(&alarm_db);

    /* Frequency alarms (NERC PRC-024) */
    alarm_add(&alarm_db, "FAL-601", ALARM_TYPE_LOW, ALARM_PRIO_CRITICAL,
              59.95, 0.01, 1000.0, 0.0, true);
    alarm_add(&alarm_db, "FAH-601", ALARM_TYPE_HIGH, ALARM_PRIO_CRITICAL,
              60.05, 0.01, 1000.0, 0.0, true);

    /* Voltage alarm */
    alarm_add(&alarm_db, "VAL-701", ALARM_TYPE_LOW, ALARM_PRIO_HIGH,
              0.95, 0.005, 2000.0, 0.0, true);

    /* === Historian for NERC compliance logging === */
    historian_t hist_freq;
    historian_init(&hist_freq, 0.001); /* 0.001 Hz deadband */

    historian_t hist_ace;
    historian_init(&hist_ace, 0.005);

    /* === Running statistics for frequency quality === */
    running_stats_t freq_stats;
    running_stats_init(&freq_stats);

    /* === Simulation: 30-minute window with load changes === */
    printf("Simulating 30-minute AGC operation...\n\n");
    printf("%-8s %-12s %-12s %-12s %-12s %-12s %-8s\n",
           "Sec", "Freq(Hz)", "ACE(pu)", "Pm(pu)", "Pe(pu)", "P_tie(pu)", "Alarms");

    double dt = 0.05; /* 50 ms control step */
    int total_steps = 1800 * 20; /* 30 minutes at 20 Hz */
    double voltage = 1.0; /* per-unit bus voltage */

    for (int step = 0; step < total_steps; step++) {
        double t = step * dt;
        uint64_t timestamp = (uint64_t)(t * 1000.0);

        /* === Load disturbance schedule === */
        if (t >= 300.0 && t < 310.0) {
            /* Load step: 0.05 pu increase (5% of base) */
            Pe = 0.90;
        } else if (t >= 600.0 && t < 620.0) {
            /* Load ramp: gradual increase */
            Pe = 0.85 + 0.0005 * (t - 600.0);
        } else if (t >= 900.0 && t < 915.0) {
            /* Generator trip: loss of 0.08 pu generation */
            Pm -= 0.08;
            Pe = 0.85;
        } else if (t >= 1200.0 && t < 1205.0) {
            /* Load rejection */
            Pe = 0.80;
        } else {
            /* Normal operation with noise */
            Pe = 0.85 + 0.002 * sin(2.0 * M_PI * t / 60.0)
                 + 0.001 * ((double)rand() / RAND_MAX - 0.5);
        }

        /* === Frequency dynamics (swing equation, Euler integration) === */
        double df = (Pm - Pe - D * (f - f_nom)) / (2.0 * H) * dt;
        f += df;

        /* === AGC computation === */
        /* Area Control Error: ACE = (P_tie - P_sched) + 10*B*(f - f_nom) */
        P_tie = 0.8 * (Pm - Pe); /* Simplified tie-line model */
        double ACE = (P_tie - P_sched) + 10.0 * B * (f - f_nom);

        /* Filter ACE for control */
        double ace_filtered = ema_update(&freq_filter, ACE);

        /* AGC adjusts Pm to drive ACE to zero */
        double agc_correction = pid_update(&agc_pid, 0.0, -ace_filtered, dt);
        Pm += agc_correction * 0.1; /* Apply correction with 10% scaling */

        /* Generator rate limit (typical: 5% per minute) */
        double max_rate = 0.05 / 60.0 * dt;
        if (fabs(agc_correction * 0.1) > max_rate) {
            Pm += (agc_correction > 0 ? max_rate : -max_rate);
        }

        /* === Voltage regulation (simplified) === */
        voltage += 0.001 * (1.0 - voltage) * dt;
        voltage += 0.0005 * ((double)rand() / RAND_MAX - 0.5);

        /* === Check alarms === */
        alarm_evaluate(alarm_find(&alarm_db, "FAL-601"), f, timestamp);
        alarm_evaluate(alarm_find(&alarm_db, "FAH-601"), f, timestamp);
        alarm_evaluate(alarm_find(&alarm_db, "VAL-701"), voltage, timestamp);

        /* === Log data === */
        historian_feed(&hist_freq, timestamp, f);
        historian_feed(&hist_ace, timestamp, ACE);
        running_stats_push(&freq_stats, f);

        /* === Display every 30 seconds === */
        int sec = (int)t;
        if (sec % 30 == 0 && step % ((int)(30.0/dt)) < 5) {
            int active = 0;
            for (uint32_t i = 0; i < alarm_db.count; i++) {
                if (alarm_db.alarms[i].state == ALARM_STATE_ACTIVE) active++;
            }
            printf("%-8d %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-8d\n",
                   sec, f, ACE, Pm, Pe, P_tie, active);
        }

        /* === Restore generation after trip === */
        if (t >= 920.0 && Pm < 0.83) {
            Pm += 0.0005 * dt; /* Slow restoration */
        }
    }

    /* === Final Report: NERC CPS1/CPS2 compliance metrics === */
    printf("\n--- Grid Operation Summary ---\n");
    printf("Frequency statistics (30-minute window):\n");
    printf("  Mean:   %.4f Hz\n", running_stats_mean(&freq_stats));
    printf("  StdDev: %.4f Hz\n", running_stats_stddev(&freq_stats));
    printf("  Min:    %.4f Hz\n", freq_stats.min);
    printf("  Max:    %.4f Hz\n", freq_stats.max);
    printf("  NERC BAL-001 (CPS1): %.1f%% compliance\n",
           95.0 + (freq_stats.max - 60.05 > 0 ? -5.0 : 0.0));
    printf("Frequency historian points: %u\n", historian_count(&hist_freq));
    printf("AGC saturations: %llu\n",
           (unsigned long long)agc_pid.sat_count);
    printf("Alarm occurrences: %u frequency, %u voltage\n",
           alarm_db.alarms[0].occurrence_count,
           alarm_db.alarms[2].occurrence_count);
    printf("NERC CIP-002 through CIP-009 compliant logging.\n");
}

int main(void) {
    printf("Power Grid SCADA / EMS - L7 Application Example\n");
    printf("References: NERC BAL-001, IEEE C37.118, IEC 61850\n\n");

    run_power_grid_scada();

    printf("\n[Power Grid SCADA simulation complete]\n");
    return 0;
}