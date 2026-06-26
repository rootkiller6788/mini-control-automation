#include "plc_ladder.h"
#include "plc_core.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* =========================================================================
 * L2: Ladder Diagram (LD) Engine — IEC 61131-3
 *
 * LD originates from electrical relay logic diagrams. Power flows from
 * left rail through series/parallel contacts to energize coils.
 *
 * Boolean algebra mapping:
 *   Series contacts = AND gate
 *   Parallel contacts = OR gate
 *   Normally-closed contact = NOT gate
 *   Coil = assignment
 *
 * Rung evaluation: left-to-right power flow propagation.
 * ========================================================================= */

void ld_rung_init(ld_rung_t *rung, int rung_id)
{
    if (!rung) return;
    memset(rung, 0, sizeof(*rung));
    rung->rung_id = rung_id;
    rung->enabled = 1;
}

/* Contact additions */
int ld_rung_add_contact_no(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_CONTACT;
    e->data.contact.ctype = LD_CONTACT_NO;
    e->data.contact.ref_addr = ref_addr;
    e->data.contact.ref_type = ref_type;
    return 0;
}

int ld_rung_add_contact_nc(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_CONTACT;
    e->data.contact.ctype = LD_CONTACT_NC;
    e->data.contact.ref_addr = ref_addr;
    e->data.contact.ref_type = ref_type;
    return 0;
}

int ld_rung_add_contact_pos(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_CONTACT;
    e->data.contact.ctype = LD_CONTACT_POS;
    e->data.contact.ref_addr = ref_addr;
    e->data.contact.ref_type = ref_type;
    return 0;
}

int ld_rung_add_contact_neg(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_CONTACT;
    e->data.contact.ctype = LD_CONTACT_NEG;
    e->data.contact.ref_addr = ref_addr;
    e->data.contact.ref_type = ref_type;
    return 0;
}

/* Coil additions */
int ld_rung_add_coil_normal(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_COIL;
    e->data.coil.ctype = LD_COIL_NORMAL;
    e->data.coil.ref_addr = ref_addr;
    e->data.coil.ref_type = ref_type;
    return 0;
}

int ld_rung_add_coil_negated(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_COIL;
    e->data.coil.ctype = LD_COIL_NEGATED;
    e->data.coil.ref_addr = ref_addr;
    e->data.coil.ref_type = ref_type;
    return 0;
}

int ld_rung_add_coil_set(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_COIL;
    e->data.coil.ctype = LD_COIL_SET;
    e->data.coil.ref_addr = ref_addr;
    e->data.coil.ref_type = ref_type;
    return 0;
}

int ld_rung_add_coil_reset(ld_rung_t *rung, uint16_t ref_addr, plc_io_type_t ref_type)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_COIL;
    e->data.coil.ctype = LD_COIL_RESET;
    e->data.coil.ref_addr = ref_addr;
    e->data.coil.ref_type = ref_type;
    return 0;
}

/* Link additions */
int ld_rung_add_horiz_link(ld_rung_t *rung)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    rung->elements[rung->num_elements].elem_type = LD_ELEM_HORIZ_LINK;
    rung->num_elements++;
    return 0;
}

int ld_rung_add_vert_link(ld_rung_t *rung)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    rung->elements[rung->num_elements].elem_type = LD_ELEM_VERT_LINK;
    rung->num_elements++;
    return 0;
}

/* FB additions */
int ld_rung_add_fb_ton(ld_rung_t *rung, uint16_t timer_id, double preset_ms)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_FUNCTION_BLOCK;
    e->data.fb.fbtype = LD_FB_TIMER_TON;
    e->data.fb.instance_id = timer_id;
    (void)preset_ms;
    return 0;
}

int ld_rung_add_fb_ctu(ld_rung_t *rung, uint16_t counter_id, int32_t preset)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_FUNCTION_BLOCK;
    e->data.fb.fbtype = LD_FB_COUNTER_CTU;
    e->data.fb.instance_id = counter_id;
    (void)preset;
    return 0;
}

int ld_rung_add_fb_compare_eq(ld_rung_t *rung, uint16_t reg_a, uint16_t reg_b)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_FUNCTION_BLOCK;
    e->data.fb.fbtype = LD_FB_COMPARE_EQ;
    e->data.fb.instance_id = reg_a | (reg_b << 16);
    return 0;
}

int ld_rung_add_fb_math_add(ld_rung_t *rung, uint16_t src1, uint16_t src2, uint16_t dst)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_FUNCTION_BLOCK;
    e->data.fb.fbtype = LD_FB_MATH_ADD;
    e->data.fb.instance_id = dst;
    (void)src1; (void)src2;
    return 0;
}

int ld_rung_add_fb_move(ld_rung_t *rung, uint16_t src, uint16_t dst)
{
    if (!rung || rung->num_elements >= LD_MAX_ELEMENTS_PER_RUNG) return -1;
    ld_element_t *e = &rung->elements[rung->num_elements++];
    memset(e, 0, sizeof(*e));
    e->elem_type = LD_ELEM_FUNCTION_BLOCK;
    e->data.fb.fbtype = LD_FB_MOVE;
    e->data.fb.instance_id = dst | (src << 16);
    return 0;
}
