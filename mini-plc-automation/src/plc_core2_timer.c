#include "plc_core.h"

/* Timer implementations: TON, TOF, TP, RTO per IEC 61131-3
 *
 * TON (Timer ON Delay): Q becomes TRUE when IN has been TRUE for PT ms.
 *   IN=0 -> Q=0, ET=0.  IN=1, ET<PT -> Q=0, ET+=dt.  IN=1, ET>=PT -> Q=1.
 *
 * TOF (Timer OFF Delay): Q becomes FALSE when IN has been FALSE for PT ms.
 *   IN=1 -> Q=1, ET=0.  IN=0, ET<PT -> Q=1, ET+=dt.  IN=0, ET>=PT -> Q=0.
 *
 * TP (Pulse Timer): Q=TRUE for PT ms following rising edge of IN.
 *   IN rising -> Q=1, ET=0.  ET<PT -> Q=1, ET+=dt.  ET>=PT -> Q=0.
 *
 * RTO (Retentive TON): accumulates IN=TRUE time across IN=FALSE periods.
 *   Reset input clears accumulated time to 0.
 */

int plc_timer_config(plc_system_t *plc, uint16_t idx,
                     plc_timer_type_t type, double pt_ms)
{
    if (!plc || idx >= PLC_MAX_TIMERS) return -1;
    plc->timers[idx].type = type;
    plc->timers[idx].preset_time_ms = pt_ms;
    plc->timers[idx].elapsed_time_ms = 0.0;
    plc->timers[idx].input = 0;
    plc->timers[idx].output = 0;
    plc->timers[idx].reset = 0;
    plc->timers[idx].timing = 0;
    if (idx >= plc->num_timers) plc->num_timers = idx + 1;
    return 0;
}

int plc_timer_update(plc_system_t *plc, uint16_t idx, double dt_ms)
{
    if (!plc || idx >= plc->num_timers) return -1;
    plc_timer_t *t = &plc->timers[idx];
    switch (t->type) {
    case PLC_TIMER_TON:
        if (t->input) {
            t->elapsed_time_ms += dt_ms;
            if (t->elapsed_time_ms >= t->preset_time_ms) {
                t->elapsed_time_ms = t->preset_time_ms; t->output = 1;
            } else t->output = 0;
        } else { t->elapsed_time_ms = 0.0; t->output = 0; }
        break;
    case PLC_TIMER_TOF:
        if (t->input) { t->elapsed_time_ms = 0.0; t->output = 1; }
        else {
            t->elapsed_time_ms += dt_ms;
            if (t->elapsed_time_ms >= t->preset_time_ms) {
                t->elapsed_time_ms = t->preset_time_ms; t->output = 0;
            } else t->output = 1;
        }
        break;
    case PLC_TIMER_TP:
        if (t->input && !t->timing) {
            t->elapsed_time_ms = 0.0; t->output = 1; t->timing = 1;
        }
        if (t->timing) {
            t->elapsed_time_ms += dt_ms;
            if (t->elapsed_time_ms >= t->preset_time_ms) {
                t->elapsed_time_ms = t->preset_time_ms;
                t->output = 0; t->timing = 0;
            } else t->output = 1;
        }
        break;
    case PLC_TIMER_RTO:
        if (t->reset) {
            t->elapsed_time_ms = 0.0; t->output = 0; t->timing = 0;
        } else if (t->input) {
            t->elapsed_time_ms += dt_ms;
            if (t->elapsed_time_ms >= t->preset_time_ms) {
                t->elapsed_time_ms = t->preset_time_ms; t->output = 1;
            } else t->output = 0;
        }
        break;
    }
    t->last_update_ms += dt_ms;
    return t->output;
}

int plc_timer_set_input(plc_system_t *plc, uint16_t idx, int in_value)
{ if (!plc || idx >= plc->num_timers) return -1; plc->timers[idx].input = (in_value != 0); return 0; }

int plc_timer_get_output(const plc_system_t *plc, uint16_t idx)
{ if (!plc || idx >= plc->num_timers) return 0; return plc->timers[idx].output; }

int plc_timer_reset(plc_system_t *plc, uint16_t idx)
{
    if (!plc || idx >= plc->num_timers) return -1;
    plc->timers[idx].elapsed_time_ms = 0.0;
    plc->timers[idx].output = 0;
    plc->timers[idx].timing = 0;
    return 0;
}
