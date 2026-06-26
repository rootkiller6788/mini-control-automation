#include "plc_core.h"
#include <string.h>

/* Counter implementations: CTU, CTD, CTUD per IEC 61131-3
 *
 * CTU (Up Counter): CU rising edge -> CV++. Q=1 when CV >= PV. Reset -> CV=0.
 * CTD (Down Counter): CD rising edge -> CV--. Q=1 when CV <= 0. Load -> CV=PV.
 * CTUD (Up/Down Counter): Both counting directions with shared CV.
 */

int plc_counter_config(plc_system_t *plc, uint16_t idx,
                       plc_counter_type_t type, int32_t pv)
{
    if (!plc || idx >= PLC_MAX_COUNTERS) return -1;
    memset(&plc->counters[idx], 0, sizeof(plc_counter_t));
    plc->counters[idx].type = type;
    plc->counters[idx].preset_value = pv;
    if (idx >= plc->num_counters) plc->num_counters = idx + 1;
    return 0;
}

int plc_counter_update(plc_system_t *plc, uint16_t idx,
                       int cu_signal, int cd_signal,
                       int reset_signal, int load_signal)
{
    if (!plc || idx >= plc->num_counters) return -1;
    plc_counter_t *c = &plc->counters[idx];
    /* Reset has highest priority */
    if (reset_signal) {
        c->current_value = 0; c->output_up = 0; c->output_dn = 0;
        c->last_cu = 0; c->last_cd = 0;
        return 0;
    }
    if (load_signal)
        c->current_value = c->preset_value;
    /* Rising edge detection */
    int cu_rise = (cu_signal && !c->last_cu);
    int cd_rise = (cd_signal && !c->last_cd);
    c->last_cu = cu_signal; c->last_cd = cd_signal;
    switch (c->type) {
    case PLC_CTU:
        if (cu_rise) c->current_value++;
        c->output_up = (c->current_value >= c->preset_value);
        c->output_dn = 0;
        break;
    case PLC_CTD:
        if (cd_rise) c->current_value--;
        c->output_dn = (c->current_value <= 0);
        c->output_up = 0;
        break;
    case PLC_CTUD:
        if (cu_rise && !cd_rise) c->current_value++;
        else if (cd_rise && !cu_rise) c->current_value--;
        c->output_up = (c->current_value >= c->preset_value);
        c->output_dn = (c->current_value <= 0);
        break;
    }
    return 0;
}

int plc_counter_get_qu(const plc_system_t *plc, uint16_t idx)
{ if (!plc || idx >= plc->num_counters) return 0; return plc->counters[idx].output_up; }

int plc_counter_get_qd(const plc_system_t *plc, uint16_t idx)
{ if (!plc || idx >= plc->num_counters) return 0; return plc->counters[idx].output_dn; }

int32_t plc_counter_get_cv(const plc_system_t *plc, uint16_t idx)
{ if (!plc || idx >= plc->num_counters) return 0; return plc->counters[idx].current_value; }

int plc_counter_reset(plc_system_t *plc, uint16_t idx)
{
    if (!plc || idx >= plc->num_counters) return -1;
    plc->counters[idx].current_value = 0;
    plc->counters[idx].output_up = 0;
    plc->counters[idx].output_dn = 0;
    plc->counters[idx].last_cu = 0;
    plc->counters[idx].last_cd = 0;
    return 0;
}
