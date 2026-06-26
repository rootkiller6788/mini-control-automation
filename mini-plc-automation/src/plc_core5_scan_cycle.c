#include "plc_core.h"
#include <string.h>
#include <stdio.h>

/* plc_init: Initialize PLC to safe state (STOP mode, all outputs off).
 * Configuration: 10ms scan, 20ms deadline, 100ms watchdog.
 * Reference: IEC 61131-3 Section 4.2. */

int plc_init(plc_system_t *plc)
{
    if (!plc) return -1;
    memset(plc, 0, sizeof(plc_system_t));
    plc->mode = PLC_MODE_STOP;
    plc->firmware_version = 0x00010000;
    plc->scan_cfg.target_scan_time_ms  = 10.0;
    plc->scan_cfg.max_scan_time_ms     = 20.0;
    plc->scan_cfg.watchdog_timeout_ms  = 100.0;
    plc->scan_cfg.input_phase_budget_ms  = 1.0;
    plc->scan_cfg.exec_phase_budget_ms   = 8.0;
    plc->scan_cfg.output_phase_budget_ms = 0.5;
    plc->scan_cfg.min_scan_time_ms       = 1e9;
    plc->scan_cfg.max_scan_time_ms_hist  = 0.0;
    plc->cpu_temperature_c = 25.0;
    return 0;
}

/* plc_set_mode: State machine for IEC 61131-3 operating modes.
 *
 * Allowed transitions:
 *   STOP -> RUN, PROGRAM, MAINTENANCE, FAULT
 *   RUN -> STOP, MAINTENANCE, FAULT
 *   PROGRAM -> STOP, FAULT
 *   MAINTENANCE -> STOP, RUN, FAULT
 *   any -> FAULT (unconditional)
 *
 * On STOP entry: all digital outputs de-energized (safe state).
 * On RUN entry: scan counter reset, execution begins next call to plc_scan_cycle. */

int plc_set_mode(plc_system_t *plc, plc_mode_t new_mode)
{
    if (!plc) return -1;
    if (new_mode == PLC_MODE_FAULT) {
        plc->mode = PLC_MODE_FAULT;
        plc->fault_active = 1;
        for (uint16_t i = 0; i < plc->num_digital_out; i++)
            plc->digital_outputs[i].value = 0;
        return 0;
    }
    switch (plc->mode) {
    case PLC_MODE_STOP:
        if (new_mode == PLC_MODE_RUN || new_mode == PLC_MODE_PROGRAM ||
            new_mode == PLC_MODE_MAINTENANCE) {
            plc->mode = new_mode;
            if (new_mode == PLC_MODE_RUN) plc->scan_cfg.scan_counter = 0;
            return 0;
        }
        break;
    case PLC_MODE_RUN:
        if (new_mode == PLC_MODE_STOP || new_mode == PLC_MODE_MAINTENANCE) {
            if (new_mode == PLC_MODE_STOP)
                for (uint16_t i = 0; i < plc->num_digital_out; i++)
                    plc->digital_outputs[i].value = 0;
            plc->mode = new_mode;
            return 0;
        }
        break;
    case PLC_MODE_PROGRAM:
        if (new_mode == PLC_MODE_STOP) { plc->mode = new_mode; return 0; }
        break;
    case PLC_MODE_MAINTENANCE:
        if (new_mode == PLC_MODE_STOP || new_mode == PLC_MODE_RUN) {
            plc->mode = new_mode; return 0;
        }
        break;
    default: break;
    }
    return -1;
}

/* plc_reset: Force STOP mode, clear faults, reset all outputs/timers/counters.
 * Restores PLC to known-safe initial state. Equivalent to power-cycle. */

int plc_reset(plc_system_t *plc)
{
    if (!plc) return -1;
    plc->mode = PLC_MODE_STOP;
    plc->fault_active = 0;
    plc->fault_message[0] = '\0';
    for (uint16_t i = 0; i < plc->num_digital_out; i++) {
        plc->digital_outputs[i].value = 0;
        plc->digital_outputs[i].forced = 0;
    }
    for (uint16_t i = 0; i < plc->num_analog_out; i++) {
        plc->analog_outputs[i].eng_value = 0.0;
        plc->analog_outputs[i].forced = 0;
    }
    for (uint16_t i = 0; i < plc->num_timers; i++) {
        plc->timers[i].elapsed_time_ms = 0.0;
        plc->timers[i].output = 0;
        plc->timers[i].timing = 0;
    }
    for (uint16_t i = 0; i < plc->num_counters; i++) {
        plc->counters[i].current_value = 0;
        plc->counters[i].output_up = 0;
        plc->counters[i].output_dn = 0;
    }
    return 0;
}

/* plc_scan_cycle: Execute one complete scan per IEC 61131-3 cyclic model.
 *
 * Scan phases (sequential, non-preemptive):
 *   Phase 1 (INPUT_SCAN): Read physical inputs -> input image table.
 *     - Digital: copy physical value to image (respecting force flags).
 *     - Analog: apply scaling (ADC->engineering) and digital filtering.
 *   Phase 2 (PROGRAM_EXEC): Execute user-provided logic callback.
 *     - Callback receives (plc*, delta_ms, user_data).
 *     - Non-zero return -> PLC_FAULT.
 *   Phase 3 (OUTPUT_SCAN): Write output image table -> physical outputs.
 *   Phase 4 (HOUSEKEEPING): Update scan stats, check watchdog.
 *     - Exponential moving average of scan time (alpha=0.1).
 *     - Jitter tracking: max deviation from target.
 *     - Watchdog: if T_scan > watchdog_timeout -> FAULT.
 *     - Overrun counter: if T_scan > max_scan_time_ms.
 *
 * Input-to-output latency model:
 *   L_io = T_input + T_program + T_output
 *   Worst-case: L_io_max = T_scan (if input changes just after INPUT_SCAN)
 */

int plc_scan_cycle(plc_system_t *plc,
                   int (*logic_exec)(plc_system_t*, double, void*),
                   void *user_data, double delta_ms)
{
    if (!plc) return -1;
    if (plc->mode != PLC_MODE_RUN && plc->mode != PLC_MODE_MAINTENANCE)
        return -1;

    /* Phase 1: INPUT_SCAN */
    plc->current_phase = PLC_PHASE_INPUT_SCAN;
    for (uint16_t i = 0; i < plc->num_digital_in; i++)
        if (plc->digital_inputs[i].forced)
            plc->digital_inputs[i].value = plc->digital_inputs[i].forced_value;
    for (uint16_t i = 0; i < plc->num_analog_in; i++) {
        if (plc->analog_inputs[i].forced)
            plc->analog_inputs[i].eng_value = plc->analog_inputs[i].raw_value;
        plc_analog_scale_and_filter(&plc->analog_inputs[i]);
    }

    /* Phase 2: PROGRAM_EXEC */
    plc->current_phase = PLC_PHASE_PROGRAM_EXEC;
    if (logic_exec) {
        int ret = logic_exec(plc, delta_ms, user_data);
        if (ret != 0) {
            plc_set_mode(plc, PLC_MODE_FAULT);
            snprintf(plc->fault_message, sizeof(plc->fault_message),
                     "Logic error code %d", ret);
            return -1;
        }
    }

    /* Phase 3: OUTPUT_SCAN */
    plc->current_phase = PLC_PHASE_OUTPUT_SCAN;
    for (uint16_t i = 0; i < plc->num_digital_out; i++)
        if (plc->digital_outputs[i].forced)
            plc->digital_outputs[i].value = plc->digital_outputs[i].forced_value;
    for (uint16_t i = 0; i < plc->num_analog_out; i++)
        if (plc->analog_outputs[i].forced)
            plc->analog_outputs[i].eng_value = plc->analog_outputs[i].forced_value;

    /* Phase 4: HOUSEKEEPING */
    plc->current_phase = PLC_PHASE_HOUSEKEEPING;
    plc->scan_cfg.scan_counter++;
    plc->total_scans++;
    plc->uptime_seconds += delta_ms / 1000.0;

    double st = delta_ms;
    plc->scan_cfg.last_scan_time_ms = st;
    if (st < plc->scan_cfg.min_scan_time_ms) plc->scan_cfg.min_scan_time_ms = st;
    if (st > plc->scan_cfg.max_scan_time_ms_hist) plc->scan_cfg.max_scan_time_ms_hist = st;
    if (plc->scan_cfg.avg_scan_time_ms < 1e-9)
        plc->scan_cfg.avg_scan_time_ms = st;
    else
        plc->scan_cfg.avg_scan_time_ms = 0.9 * plc->scan_cfg.avg_scan_time_ms + 0.1 * st;

    double jit = fabs(st - plc->scan_cfg.target_scan_time_ms);
    if (jit > plc->scan_cfg.jitter_ms) plc->scan_cfg.jitter_ms = jit;

    if (st > plc->scan_cfg.watchdog_timeout_ms) {
        plc_set_mode(plc, PLC_MODE_FAULT);
        snprintf(plc->fault_message, sizeof(plc->fault_message),
                 "Watchdog: %.1fms > %.1fms", st, plc->scan_cfg.watchdog_timeout_ms);
        return -1;
    }
    if (st > plc->scan_cfg.max_scan_time_ms)
        plc->scan_cfg.overrun_count++;

    plc->current_phase = PLC_PHASE_IDLE;
    return 0;
}
