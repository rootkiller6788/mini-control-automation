#include "plc_core.h"
#include <stdio.h>

/* Traffic Light Controller using SFC (Sequential Function Chart)
 * Standard 4-phase cycle: Main Green->Main Yellow->Side Green->Side Yellow */

#define TIMER_P1 0
#define TIMER_P2 1
#define TIMER_P3 2
#define TIMER_P4 3

static int traffic_logic(plc_system_t *plc, double dt_ms, void *user_data)
{
    plc_sfc_engine_t *sfc = (plc_sfc_engine_t*)user_data;
    if (!sfc) return -1;
    plc_timer_update(plc, TIMER_P1, dt_ms);
    plc_timer_update(plc, TIMER_P2, dt_ms);
    plc_timer_update(plc, TIMER_P3, dt_ms);
    plc_timer_update(plc, TIMER_P4, dt_ms);
    int active = plc_sfc_get_active_step(sfc);
    plc_sfc_set_condition(sfc, 12, (active==1 && plc_timer_get_output(plc,TIMER_P1)));
    plc_sfc_set_condition(sfc, 23, (active==2 && plc_timer_get_output(plc,TIMER_P2)));
    plc_sfc_set_condition(sfc, 34, (active==3 && plc_timer_get_output(plc,TIMER_P3)));
    plc_sfc_set_condition(sfc, 41, (active==4 && plc_timer_get_output(plc,TIMER_P4)));
    plc_sfc_scan(sfc, dt_ms);
    active = plc_sfc_get_active_step(sfc);
    /* Output: Main R/Y/G = Q0/Q1/Q2, Side R/Y/G = Q3/Q4/Q5 */
    switch (active) {
    case 1: plc_write_digital_output(plc,0,0);plc_write_digital_output(plc,1,0);plc_write_digital_output(plc,2,1);
            plc_write_digital_output(plc,3,1);plc_write_digital_output(plc,4,0);plc_write_digital_output(plc,5,0); break;
    case 2: plc_write_digital_output(plc,0,0);plc_write_digital_output(plc,1,1);plc_write_digital_output(plc,2,0);
            plc_write_digital_output(plc,3,1);plc_write_digital_output(plc,4,0);plc_write_digital_output(plc,5,0); break;
    case 3: plc_write_digital_output(plc,0,1);plc_write_digital_output(plc,1,0);plc_write_digital_output(plc,2,0);
            plc_write_digital_output(plc,3,0);plc_write_digital_output(plc,4,0);plc_write_digital_output(plc,5,1); break;
    case 4: plc_write_digital_output(plc,0,1);plc_write_digital_output(plc,1,0);plc_write_digital_output(plc,2,0);
            plc_write_digital_output(plc,3,0);plc_write_digital_output(plc,4,1);plc_write_digital_output(plc,5,0); break;
    default: plc_sfc_reset(sfc); break;
    }
    return 0;
}

int main(void)
{
    plc_system_t plc; plc_init(&plc);
    plc_timer_config(&plc, TIMER_P1, PLC_TIMER_TON, 30000.0);
    plc_timer_config(&plc, TIMER_P2, PLC_TIMER_TON, 5000.0);
    plc_timer_config(&plc, TIMER_P3, PLC_TIMER_TON, 20000.0);
    plc_timer_config(&plc, TIMER_P4, PLC_TIMER_TON, 5000.0);

    plc_sfc_engine_t sfc;
    plc_sfc_init(&sfc, 5, 5);
    plc_sfc_add_step(&sfc, 1, 1);plc_sfc_add_step(&sfc, 2, 0);
    plc_sfc_add_step(&sfc, 3, 0);plc_sfc_add_step(&sfc, 4, 0);
    plc_sfc_add_transition(&sfc, 12, 1, 2);
    plc_sfc_add_transition(&sfc, 23, 2, 3);
    plc_sfc_add_transition(&sfc, 34, 3, 4);
    plc_sfc_add_transition(&sfc, 41, 4, 1);

    plc_set_mode(&plc, PLC_MODE_RUN);

    printf("Traffic Light Control Demo (60-second cycle)\n");
    printf("=============================================\n");

    double total_time = 0.0;
    int prev_step = -1;
    for (int i = 0; i < 60; i++) {
        plc_scan_cycle(&plc, traffic_logic, &sfc, 1000.0);
        total_time += 1000.0;
        int step = plc_sfc_get_active_step(&sfc);
        if (step != prev_step) {
            printf("t=%.0fs: Step %d\n", total_time/1000.0, step);
            prev_step = step;
        }
    }

    plc_sfc_free(&sfc);
    printf("\nDemo complete.\n");
    return 0;
}
