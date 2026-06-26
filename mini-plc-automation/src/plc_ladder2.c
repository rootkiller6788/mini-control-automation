#include "plc_ladder.h"
#include "plc_core.h"
#include <string.h>
#include <stdio.h>

static int ld_read_bit(const plc_system_t *plc, plc_io_type_t t, uint16_t a)
{
    if (!plc) return 0;
    switch (t) {
    case PLC_IO_DISCRETE_INPUT:  return plc_read_digital_input(plc, a);
    case PLC_IO_DISCRETE_OUTPUT: return plc_read_digital_output(plc, a);
    case PLC_IO_INTERNAL_BIT:    return plc_read_internal_bit(plc, a);
    default: return 0;
    }
}

static void ld_write_bit(plc_system_t *plc, plc_io_type_t t, uint16_t a, int v)
{
    if (!plc) return;
    switch (t) {
    case PLC_IO_DISCRETE_OUTPUT: plc_write_digital_output(plc, a, v); break;
    case PLC_IO_INTERNAL_BIT:    plc_write_internal_bit(plc, a, v); break;
    default: break;
    }
}

/* =========================================================================
 * L5: Rung Evaluation -- Power Flow Algorithm
 *
 * Evaluates series-parallel contact network left-to-right.
 * Element power propagation:
 *   NO Contact:  out = in AND ref_bit
 *   NC Contact:  out = in AND (NOT ref_bit)
 *   POS Contact: out = in AND rising_edge(ref_bit)
 *   NEG Contact: out = in AND falling_edge(ref_bit)
 *   Coil:        apply action to ref_bit, return coil_power
 *   Horiz Link:  out = in (pass-through)
 *   Vert Link:   creates/exits parallel OR branch
 *
 * Parallel OR: when vertical link is encountered, current power
 * is saved and separate OR accumulator tracks parallel path.
 * On exit, power = saved_power OR accumulated_or.
 * ========================================================================= */

int ld_rung_evaluate(const ld_rung_t *rung, plc_system_t *plc,
                     ld_program_t *prog)
{
    if (!rung || !plc || !rung->enabled) return 0;
    int power = 1, or_state = 0, in_parallel = 0;

    for (size_t i = 0; i < rung->num_elements; i++) {
        ld_element_t *e = (ld_element_t*)&rung->elements[i];
        switch (e->elem_type) {
        case LD_ELEM_CONTACT: {
            int bit = ld_read_bit(plc, e->data.contact.ref_type,
                                  e->data.contact.ref_addr);
            int cr = 0;
            switch (e->data.contact.ctype) {
            case LD_CONTACT_NO:  cr = bit; break;
            case LD_CONTACT_NC:  cr = !bit; break;
            case LD_CONTACT_POS:
                if (prog && e->data.contact.ref_addr < PLC_MAX_DIGITAL_IO)
                    cr = plc_r_trig_update(
                        &prog->pos_edge_states[e->data.contact.ref_addr], bit);
                break;
            case LD_CONTACT_NEG:
                if (prog && e->data.contact.ref_addr < PLC_MAX_DIGITAL_IO)
                    cr = plc_f_trig_update(
                        &prog->neg_edge_states[e->data.contact.ref_addr], bit);
                break;
            }
            if (in_parallel) or_state = or_state || (power && cr);
            else power = power && cr;
            break;
        }
        case LD_ELEM_COIL: {
            int cp = in_parallel ? (power || or_state) : power;
            switch (e->data.coil.ctype) {
            case LD_COIL_NORMAL:  ld_write_bit(plc, e->data.coil.ref_type, e->data.coil.ref_addr, cp); break;
            case LD_COIL_NEGATED: ld_write_bit(plc, e->data.coil.ref_type, e->data.coil.ref_addr, !cp); break;
            case LD_COIL_SET:     if (cp) ld_write_bit(plc, e->data.coil.ref_type, e->data.coil.ref_addr, 1); break;
            case LD_COIL_RESET:   if (cp) ld_write_bit(plc, e->data.coil.ref_type, e->data.coil.ref_addr, 0); break;
            case LD_COIL_POS:
                if (prog && e->data.coil.ref_addr < PLC_MAX_DIGITAL_IO) {
                    int r = plc_r_trig_update(&prog->pos_edge_states[e->data.coil.ref_addr], cp);
                    if (r) ld_write_bit(plc, e->data.coil.ref_type, e->data.coil.ref_addr, 1);
                }
                break;
            case LD_COIL_NEG:
                if (prog && e->data.coil.ref_addr < PLC_MAX_DIGITAL_IO) {
                    int r = plc_f_trig_update(&prog->neg_edge_states[e->data.coil.ref_addr], cp);
                    if (r) ld_write_bit(plc, e->data.coil.ref_type, e->data.coil.ref_addr, 0);
                }
                break;
            }
            return cp;
        }
        case LD_ELEM_HORIZ_LINK: break;
        case LD_ELEM_VERT_LINK:
            if (!in_parallel) { in_parallel = 1; or_state = 0; }
            else { power = power || or_state; in_parallel = 0; }
            break;
        case LD_ELEM_FUNCTION_BLOCK:
            if (e->data.fb.fbtype == LD_FB_COMPARE_EQ) {
                uint16_t ra = e->data.fb.instance_id & 0xFFFF;
                uint16_t rb = (e->data.fb.instance_id >> 16) & 0xFFFF;
                power = power && (fabs(plc_read_internal_reg(plc,ra)-plc_read_internal_reg(plc,rb)) < 1e-9);
            }
            break;
        }
    }
    return in_parallel ? (power || or_state) : power;
}

/* =========================================================================
 * L5: Program-Level Scan
 * ========================================================================= */

void ld_program_init(ld_program_t *prog)
{ if (prog) { memset(prog, 0, sizeof(*prog)); prog->mcr_enabled = 1; } }

int ld_program_add_rung(ld_program_t *prog, ld_rung_t *rung)
{
    if (!prog || !rung || prog->num_rungs >= LD_MAX_RUNGS) return -1;
    memcpy(&prog->rungs[prog->num_rungs], rung, sizeof(ld_rung_t));
    prog->num_rungs++;
    return (int)(prog->num_rungs - 1);
}

int ld_program_scan(ld_program_t *prog, plc_system_t *plc, double dt_ms)
{
    if (!prog || !plc || !prog->mcr_enabled) return prog ? 0 : -1;
    (void)dt_ms;
    for (size_t r = 0; r < prog->num_rungs; r++)
        ld_rung_evaluate(&prog->rungs[r], plc, prog);
    return 0;
}

/* =========================================================================
 * L3: Boolean Expression Export and Rung Validation
 * ========================================================================= */

int ld_rung_to_boolean_expr(const ld_rung_t *rung, char *buf, size_t bufsz)
{
    if (!rung || !buf || bufsz < 4) return 0;
    size_t pos = 0; int first = 1;
    for (size_t i = 0; i < rung->num_elements; i++) {
        const ld_element_t *e = &rung->elements[i];
        if (e->elem_type == LD_ELEM_CONTACT) {
            if (!first) pos += snprintf(buf + pos, bufsz - pos, " AND ");
            first = 0;
            const char *nm = (e->data.contact.ref_type == PLC_IO_DISCRETE_INPUT) ? "I" :
                             (e->data.contact.ref_type == PLC_IO_DISCRETE_OUTPUT) ? "Q" : "M";
            if (e->data.contact.ctype == LD_CONTACT_NC)
                pos += snprintf(buf+pos, bufsz-pos, "(NOT %s%d)", nm, e->data.contact.ref_addr);
            else
                pos += snprintf(buf+pos, bufsz-pos, "%s%d", nm, e->data.contact.ref_addr);
            if (pos >= bufsz-1) break;
        } else if (e->elem_type == LD_ELEM_COIL) {
            const char *nm = (e->data.coil.ref_type == PLC_IO_DISCRETE_OUTPUT) ? "Q" : "M";
            pos += snprintf(buf+pos, bufsz-pos, " => %s%d", nm, e->data.coil.ref_addr);
            break;
        }
    }
    buf[pos] = '\0';
    return (int)pos;
}

int ld_rung_validate(const ld_rung_t *rung, const plc_system_t *plc)
{
    if (!rung || !plc) return 0;
    int hc=0, hcoil=0;
    for (size_t i=0; i<rung->num_elements; i++) {
        if (rung->elements[i].elem_type == LD_ELEM_CONTACT) hc=1;
        if (rung->elements[i].elem_type == LD_ELEM_COIL) hcoil=1;
    }
    return hc && hcoil;
}

/* L3: Adjacency Matrix and Analysis */

void ld_rung_adjacency_matrix(const ld_rung_t *rung, int *matrix, size_t max_dim)
{
    if (!rung || !matrix || max_dim < rung->num_elements) return;
    size_t n = rung->num_elements;
    memset(matrix, 0, n * max_dim * sizeof(int));
    for (size_t i = 0; i + 1 < n; i++)
        matrix[i * (int)max_dim + (i+1)] = 1;
}

int ld_rung_is_dead(const ld_rung_t *rung)
{
    if (!rung) return 1;
    for (size_t i = 0; i < rung->num_elements; i++)
        if (rung->elements[i].elem_type == LD_ELEM_COIL) return 0;
    return 1;
}

int ld_rung_topological_sort(const ld_rung_t *rung, int *order, size_t max_order)
{
    if (!rung || !order || max_order < rung->num_elements) return -1;
    for (size_t i = 0; i < rung->num_elements; i++)
        order[i] = (int)i;
    return (int)rung->num_elements;
}