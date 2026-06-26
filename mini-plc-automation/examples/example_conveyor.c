#include "plc_core.h"
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L6: Conveyor Belt Control System
 *
 * Classic PLC application: a conveyor belt with start/stop buttons,
 * overload protection, and run indication.
 *
 * I/O mapping:
 *   I0 = Start pushbutton (NO, momentary)
 *   I1 = Stop pushbutton (NC, momentary)
 *   I2 = Overload sensor (NC)
 *   Q0 = Motor contactor
 *   Q1 = Run indicator lamp
 *   Q2 = Fault indicator lamp
 *
 * Logic (ladder equivalent):
 *   Q0 = (I0 OR Q0) AND I1 AND I2  (start/stop with seal-in)
 *   Q1 = Q0                         (run lamp)
 *   Q2 = NOT I2                     (fault lamp when overload trips)
 *
 * This implements the classic motor start/stop circuit with
 * overload protection per IEC 60204-1 (Safety of machinery).
 * ========================================================================= */

/* User logic callback: executed each PLC scan cycle */
static int conveyor_logic(plc_system_t *plc, double dt_ms, void *user_data)
{
    (void)dt_ms;
    int *scan_count = (int*)user_data;
    if (scan_count) (*scan_count)++;

    int start   = plc_read_digital_input(plc, 0);  /* I0 - Start PB */
    int stop    = plc_read_digital_input(plc, 1);  /* I1 - Stop PB (NC, opened = stop) */
    int overload = plc_read_digital_input(plc, 2); /* I2 - Overload (NC, opened = trip) */

    /* Motor run logic: start seals in, stop or overload drops out */
    int motor_on = plc_read_digital_output(plc, 0);
    motor_on = (start || motor_on) && stop && overload;

    plc_write_digital_output(plc, 0, motor_on);   /* Q0 - Motor */
    plc_write_digital_output(plc, 1, motor_on);   /* Q1 - Run lamp */
    plc_write_digital_output(plc, 2, !overload);  /* Q2 - Fault lamp */

    return 0;
}

int main(void)
{
    plc_system_t plc;
    plc_init(&plc);

    /* Configure I/O */
    plc_config_digital_io(&plc, 0, "START_PB", PLC_IO_DISCRETE_INPUT);
    plc_config_digital_io(&plc, 1, "STOP_PB", PLC_IO_DISCRETE_INPUT);
    plc_config_digital_io(&plc, 2, "OVERLOAD", PLC_IO_DISCRETE_INPUT);
    plc_config_digital_io(&plc, 0, "MOTOR", PLC_IO_DISCRETE_OUTPUT);
    plc_config_digital_io(&plc, 1, "RUN_LAMP", PLC_IO_DISCRETE_OUTPUT);
    plc_config_digital_io(&plc, 2, "FAULT_LAMP", PLC_IO_DISCRETE_OUTPUT);

    plc_set_mode(&plc, PLC_MODE_RUN);

    int scan_count = 0;
    printf("Conveyor Belt Control Demo\n");
    printf("===========================\n");

    /* Initial state: stop pressed, overload OK */
    plc.digital_inputs[0].value = 0;  /* Start not pressed */
    plc.digital_inputs[1].value = 1;  /* Stop NC = closed (not pressed) */
    plc.digital_inputs[2].value = 1;  /* Overload NC = closed (OK) */

    /* Scan 1: Motor should be off */
    plc_scan_cycle(&plc, conveyor_logic, &scan_count, 10.0);
    printf("Scan %d: Motor=%d, Run=%d, Fault=%d\n", scan_count,
           plc_read_digital_output(&plc, 0),
           plc_read_digital_output(&plc, 1),
           plc_read_digital_output(&plc, 2));

    /* Press start button */
    plc.digital_inputs[0].value = 1;
    plc_scan_cycle(&plc, conveyor_logic, &scan_count, 10.0);
    printf("Scan %d: Motor=%d (start pressed)\n", scan_count,
           plc_read_digital_output(&plc, 0));

    /* Release start button (seal-in holds) */
    plc.digital_inputs[0].value = 0;
    plc_scan_cycle(&plc, conveyor_logic, &scan_count, 10.0);
    printf("Scan %d: Motor=%d (start released, seal-in holds)\n", scan_count,
           plc_read_digital_output(&plc, 0));

    /* Simulate overload trip */
    plc.digital_inputs[2].value = 0;  /* Overload NC opens */
    plc_scan_cycle(&plc, conveyor_logic, &scan_count, 10.0);
    printf("Scan %d: Motor=%d, Fault=%d (overload tripped!)\n", scan_count,
           plc_read_digital_output(&plc, 0),
           plc_read_digital_output(&plc, 2));

    /* Press stop */
    plc.digital_inputs[1].value = 0;  /* Stop NC opens */
    plc.digital_inputs[2].value = 1;  /* Reset overload */
    plc_scan_cycle(&plc, conveyor_logic, &scan_count, 10.0);
    printf("Scan %d: Motor=%d (stop pressed, overload reset)\n", scan_count,
           plc_read_digital_output(&plc, 0));

    printf("\nTotal scan cycles: %d\n", scan_count);
    printf("Demo complete.\n");
    return 0;
}
