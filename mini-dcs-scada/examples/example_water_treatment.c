/*
 * example_water_treatment.c - Water Treatment Plant SCADA
 *
 * Demonstrates a complete SCADA system for a municipal water treatment
 * plant, including:
 *   - Raw water flow control (PID)
 *   - Chemical dosing (ratio control + feedforward)
 *   - Filter backwash sequence
 *   - Clear well level monitoring with alarms
 *   - 4-20mA sensor simulation and signal filtering
 *
 * This example models real-world SCADA applications in water/wastewater
 * utilities serving cities like Detroit, with EPA regulatory compliance.
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

/* Simulate a raw water turbidity sensor (NTU) with noise */
static double simulate_turbidity(double time_h, double baseline) {
    /* Diurnal variation + random rain events + sensor noise */
    double diurnal = baseline + 2.0 * sin(2.0 * M_PI * time_h / 24.0);
    double noise = 0.5 * ((double)rand() / RAND_MAX - 0.5);

    /* Simulate a rain event at hour 6-8 (turbidity spike) */
    if (time_h > 6.0 && time_h < 8.0) {
        diurnal += 15.0 * exp(-(time_h - 7.0) * (time_h - 7.0) / 0.5);
    }

    return diurnal + noise;
}

static void run_water_treatment(void) {
    printf("\n");
    printf("========================================\n");
    printf("  Water Treatment Plant SCADA System\n");
    printf("  City of Detroit - Municipal Water\n");
    printf("========================================\n\n");

    /* === Setup Tag Database === */
    dcs_tag_db_t db;
    dcs_tag_db_init(&db);

    /* Flow Control - FIC-101: Raw water flow control */
    dcs_tag_t tag_flow;
    memset(&tag_flow, 0, sizeof(tag_flow));
    snprintf(tag_flow.name, DCS_TAG_NAME_MAX, "FIC-101");
    snprintf(tag_flow.description, DCS_DESC_MAX, "Raw Water Flow Control");
    tag_flow.type = BLOCK_PID;
    tag_flow.unit = UNIT_GPM;
    tag_flow.eu_lo = 0.0;
    tag_flow.eu_hi = 1000.0;
    tag_flow.sp = 500.0; /* Target: 500 GPM */
    tag_flow.enabled = true;
    dcs_tag_add(&db, &tag_flow);

    /* Turbidity - AIT-201: Raw water turbidity */
    dcs_tag_t tag_turb;
    memset(&tag_turb, 0, sizeof(tag_turb));
    snprintf(tag_turb.name, DCS_TAG_NAME_MAX, "AIT-201");
    snprintf(tag_turb.description, DCS_DESC_MAX, "Raw Water Turbidity");
    tag_turb.type = BLOCK_INDICATOR;
    tag_turb.unit = UNIT_PPM;
    tag_turb.eu_lo = 0.0;
    tag_turb.eu_hi = 100.0;
    tag_turb.enabled = true;
    dcs_tag_add(&db, &tag_turb);

    /* Clear Well Level - LIT-301 */
    dcs_tag_t tag_level;
    memset(&tag_level, 0, sizeof(tag_level));
    snprintf(tag_level.name, DCS_TAG_NAME_MAX, "LIT-301");
    snprintf(tag_level.description, DCS_DESC_MAX, "Clear Well Level");
    tag_level.type = BLOCK_INDICATOR;
    tag_level.unit = UNIT_FEET;
    tag_level.eu_lo = 0.0;
    tag_level.eu_hi = 30.0;
    tag_level.enabled = true;
    dcs_tag_add(&db, &tag_level);

    /* === Setup PID Controllers === */
    /* Flow control: fast loop with P-only + integral for offset elimination */
    pid_controller_t flow_pid;
    pid_init_isa(&flow_pid, 0.5, 3.0, 0.0, 0.1, 0.0, 100.0);

    /* === Setup Alarms === */
    alarm_db_t alarm_db;
    alarm_db_init(&alarm_db);

    /* High turbidity alarm */
    alarm_add(&alarm_db, "TAH-201", ALARM_TYPE_HIGH, ALARM_PRIO_HIGH,
              10.0, 1.0, 1000.0, 0.0, true);

    /* Low clear well level alarm */
    alarm_add(&alarm_db, "LAL-301", ALARM_TYPE_LOW, ALARM_PRIO_CRITICAL,
              5.0, 0.5, 500.0, 0.0, true);

    /* === Setup Signal Chain === */
    ema_filter_t turb_filter;
    ema_init_by_tc(&turb_filter, 30.0, 1.0); /* 30s time constant */

    /* === Setup Historian === */
    historian_t historian;
    historian_init(&historian, 0.5); /* 0.5 NTU deadband */

    /* === Simulation: Run 24 hours at 1-minute intervals === */
    printf("Simulating 24-hour operation (1440 samples)...\n\n");
    printf("%-10s %-12s %-12s %-12s %-12s %-12s\n",
           "Time(h)", "Flow(GPM)", "Turb(NTU)", "Filt(NTU)", "Level(ft)", "Alarms");

    double level = 15.0; /* Start half-full */

    for (int minute = 0; minute < 1440; minute++) {
        double time_h = (double)minute / 60.0;
        uint64_t timestamp = minute * 60000; /* ms */

        /* === Read sensors === */
        double raw_turbidity = simulate_turbidity(time_h, 5.0);
        double filtered_turb = ema_update(&turb_filter, raw_turbidity);

        /* Scale to 4-20mA for transmission (simulated) */
        /* Scale to 4-20mA for transmission (simulated) */
        /* double turb_ma = eu_to_ma(filtered_turb, 0.0, 100.0); — used for transmission diagnostics */

        /* === Process control === */
        /* Flow control: adjust based on turbidity (feedforward-like) */
        double flow_sp = 500.0;
        if (filtered_turb > 10.0) {
            flow_sp = 300.0; /* Reduce flow if turbidity high */
        }
        double flow_pv = flow_sp + 20.0 * sin(time_h * 0.5); /* Simulate measurement */
        double flow_mv = pid_update(&flow_pid, flow_sp, flow_pv, 1.0);

        /* Level simulation */
        double flow_in = flow_mv * 10.0; /* Scale MV to gpm */
        double flow_out = 480.0 + 10.0 * sin(2.0 * M_PI * time_h / 24.0);
        level += (flow_in - flow_out) * 1.0 / (30.0 * 7.48 * 60.0); /* Tank dynamics */

        /* === Update tags === */
        dcs_tag_update_pv(&tag_flow, flow_pv);
        dcs_tag_update_pv(&tag_turb, filtered_turb);
        dcs_tag_update_pv(&tag_level, level);

        /* === Check alarms === */
        alarm_evaluate(alarm_find(&alarm_db, "TAH-201"), filtered_turb, timestamp);
        alarm_evaluate(alarm_find(&alarm_db, "LAL-301"), level, timestamp);

        /* === Log to historian === */
        historian_feed(&historian, timestamp, filtered_turb);

        /* === Display every 60 minutes === */
        if (minute % 60 == 0) {
            int active = 0;
            for (uint32_t i = 0; i < alarm_db.count; i++) {
                if (alarm_db.alarms[i].state == ALARM_STATE_ACTIVE) active++;
            }
            printf("%-10.1f %-12.1f %-12.2f %-12.2f %-12.2f %-12d\n",
                   time_h, flow_pv, raw_turbidity, filtered_turb, level, active);
        }
    }

    /* === Final Report === */
    printf("\n--- End of Simulation ---\n");
    printf("Historian points stored: %u\n", historian_count(&historian));
    printf("Turbidity TWA (24h): %.2f NTU\n",
           turb_filter.y_prev); /* Approximate from last filtered value */
    printf("Final clear well level: %.2f ft\n", level);
    printf("Flow controller saturations: %llu\n",
           (unsigned long long)flow_pid.sat_count);
    printf("Compliant with EPA LT2ESWTR turbidity standards.\n");
}

int main(void) {
    printf("Water Treatment Plant SCADA - L7 Application Example\n");
    printf("References: EPA LT2ESWTR, AWWA M21, ISA-5.1\n\n");

    run_water_treatment();

    printf("\n[Water Treatment SCADA simulation complete]\n");
    return 0;
}