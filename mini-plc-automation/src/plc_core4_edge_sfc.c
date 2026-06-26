#include "plc_core.h"
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * L3: Edge Detection -- R_TRIG and F_TRIG
 *
 * R_TRIG: output = input AND NOT(last_input)
 *   Produces exactly ONE scan TRUE per 0->1 transition.
 * F_TRIG: output = NOT(input) AND last_input
 *   Produces exactly ONE scan TRUE per 1->0 transition.
 *
 * Mathematical model: discrete-time backward difference of Boolean signal.
 *   x[k] = u[k] AND NOT(u[k-1])  for R_TRIG
 *   x[k] = NOT(u[k]) AND u[k-1]  for F_TRIG
 * ========================================================================= */

void plc_r_trig_reset(plc_r_trig_t *rt)
{ if (rt) { rt->last_input = 0; rt->output = 0; } }

int plc_r_trig_update(plc_r_trig_t *rt, int input)
{
    if (!rt) return 0;
    rt->output = (input && !rt->last_input);
    rt->last_input = input;
    return rt->output;
}

void plc_f_trig_reset(plc_f_trig_t *ft)
{ if (ft) { ft->last_input = 0; ft->output = 0; } }

int plc_f_trig_update(plc_f_trig_t *ft, int input)
{
    if (!ft) return 0;
    ft->output = (!input && ft->last_input);
    ft->last_input = input;
    return ft->output;
}

/* =========================================================================
 * L3: SR and RS Flip-Flops per IEC 61131-3
 *
 * SR (Set-dominant): S=1,R=1 => Q=1
 *   Q[k] = S[k] OR (NOT R[k] AND Q[k-1])
 *
 * RS (Reset-dominant): S=1,R=1 => Q=0
 *   Q[k] = NOT R[k] AND (S[k] OR Q[k-1])
 * ========================================================================= */

void plc_sr_ff_reset(plc_sr_ff_t *ff) { if (ff) ff->output = 0; }

int plc_sr_ff_update(plc_sr_ff_t *ff, int set, int reset)
{
    if (!ff) return 0;
    if (set) ff->output = 1;
    else if (reset) ff->output = 0;
    return ff->output;
}

void plc_rs_ff_reset(plc_rs_ff_t *ff) { if (ff) ff->output = 0; }

int plc_rs_ff_update(plc_rs_ff_t *ff, int set, int reset)
{
    if (!ff) return 0;
    if (reset) ff->output = 0;
    else if (set) ff->output = 1;
    return ff->output;
}

/* =========================================================================
 * L6: Sequential Function Chart (SFC) Engine
 *
 * SFC (IEC 61131-3) is a graphical language for sequential control.
 * Steps represent states; transitions represent conditions.
 * At most one step is active at a time in a single sequence.
 *
 * Execution algorithm per scan:
 *   1. Evaluate all transition conditions.
 *   2. For each TRUE transition with active from-step:
 *      deactivate from-step, activate to-step.
 *   3. Execute active step actions.
 *   4. Increment active step elapsed time.
 *
 * Mutual exclusion: when a step activates, the previous step deactivates.
 * This guarantees at most one active step per sequence.
 * ========================================================================= */

int plc_sfc_init(plc_sfc_engine_t *sfc, size_t max_steps, size_t max_trans)
{
    if (!sfc || max_steps == 0 || max_trans == 0) return -1;
    sfc->steps = (plc_sfc_step_t*)calloc(max_steps, sizeof(plc_sfc_step_t));
    sfc->transitions = (plc_sfc_transition_t*)calloc(max_trans, sizeof(plc_sfc_transition_t));
    if (!sfc->steps || !sfc->transitions) {
        free(sfc->steps); free(sfc->transitions);
        sfc->steps = NULL; sfc->transitions = NULL;
        return -1;
    }
    sfc->step_capacity = max_steps;
    sfc->trans_capacity = max_trans;
    sfc->num_steps = 0;
    sfc->num_transitions = 0;
    sfc->initial_step = -1;
    sfc->initialized = 1;
    return 0;
}

void plc_sfc_free(plc_sfc_engine_t *sfc)
{
    if (!sfc) return;
    free(sfc->steps); sfc->steps = NULL;
    free(sfc->transitions); sfc->transitions = NULL;
    sfc->initialized = 0;
}

int plc_sfc_add_step(plc_sfc_engine_t *sfc, int step_id, int initial)
{
    if (!sfc || !sfc->initialized || sfc->num_steps >= sfc->step_capacity)
        return -1;
    sfc->steps[sfc->num_steps].step_id = step_id;
    sfc->steps[sfc->num_steps].active = (initial ? 1 : 0);
    sfc->steps[sfc->num_steps].active_time_ms = 0.0;
    if (initial) sfc->initial_step = (int)sfc->num_steps;
    sfc->num_steps++;
    return 0;
}

int plc_sfc_add_transition(plc_sfc_engine_t *sfc, int trans_id,
                            int from_step, int to_step)
{
    if (!sfc || !sfc->initialized ||
        sfc->num_transitions >= sfc->trans_capacity) return -1;
    sfc->transitions[sfc->num_transitions].trans_id = trans_id;
    sfc->transitions[sfc->num_transitions].from_step = from_step;
    sfc->transitions[sfc->num_transitions].to_step = to_step;
    sfc->transitions[sfc->num_transitions].condition = 0;
    sfc->num_transitions++;
    return 0;
}

int plc_sfc_set_condition(plc_sfc_engine_t *sfc, int trans_id, int cond)
{
    if (!sfc) return -1;
    for (size_t i = 0; i < sfc->num_transitions; i++) {
        if (sfc->transitions[i].trans_id == trans_id) {
            sfc->transitions[i].condition = (cond != 0);
            return 0;
        }
    }
    return -1;
}

int plc_sfc_scan(plc_sfc_engine_t *sfc, double dt_ms)
{
    if (!sfc || !sfc->initialized) return -1;
    /* Evaluate all transitions: if condition=TRUE and from-step active, fire */
    for (size_t t = 0; t < sfc->num_transitions; t++) {
        plc_sfc_transition_t *tr = &sfc->transitions[t];
        if (!tr->condition) continue;
        int from_idx = -1, to_idx = -1;
        for (size_t s = 0; s < sfc->num_steps; s++) {
            if (sfc->steps[s].step_id == tr->from_step) from_idx = (int)s;
            if (sfc->steps[s].step_id == tr->to_step) to_idx = (int)s;
        }
        if (from_idx >= 0 && to_idx >= 0 && sfc->steps[from_idx].active) {
            sfc->steps[from_idx].active = 0;
            sfc->steps[from_idx].active_time_ms = 0.0;
            sfc->steps[to_idx].active = 1;
            sfc->steps[to_idx].active_time_ms = 0.0;
        }
    }
    /* Update active step timers */
    for (size_t s = 0; s < sfc->num_steps; s++)
        if (sfc->steps[s].active)
            sfc->steps[s].active_time_ms += dt_ms;
    return 0;
}

int plc_sfc_get_active_step(const plc_sfc_engine_t *sfc)
{
    if (!sfc) return -1;
    for (size_t s = 0; s < sfc->num_steps; s++)
        if (sfc->steps[s].active) return sfc->steps[s].step_id;
    return -1;
}

void plc_sfc_reset(plc_sfc_engine_t *sfc)
{
    if (!sfc) return;
    for (size_t s = 0; s < sfc->num_steps; s++) {
        sfc->steps[s].active = 0;
        sfc->steps[s].active_time_ms = 0.0;
    }
    if (sfc->initial_step >= 0 && (size_t)sfc->initial_step < sfc->num_steps)
        sfc->steps[sfc->initial_step].active = 1;
}
