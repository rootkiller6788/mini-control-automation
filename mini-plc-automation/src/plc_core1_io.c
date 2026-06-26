#include "plc_core.h"
#include <string.h>
/* Digital and Analog I/O, Timer, Counter implementations */

int plc_read_digital_input(const plc_system_t *plc, uint16_t addr)
{ if (!plc || addr >= plc->num_digital_in) return 0; return plc->digital_inputs[addr].value; }

int plc_read_digital_output(const plc_system_t *plc, uint16_t addr)
{ if (!plc || addr >= plc->num_digital_out) return 0; return plc->digital_outputs[addr].value; }

int plc_read_internal_bit(const plc_system_t *plc, uint16_t addr)
{ if (!plc || addr >= plc->num_internal_bits) return 0; return plc->internal_bits[addr]; }

int plc_write_digital_output(plc_system_t *plc, uint16_t addr, int value)
{
    if (!plc || addr >= PLC_MAX_DIGITAL_IO) return -1;
    if (addr >= plc->num_digital_out) plc->num_digital_out = addr + 1;
    plc->digital_outputs[addr].value = (value != 0);
    return 0;
}

int plc_write_internal_bit(plc_system_t *plc, uint16_t addr, int value)
{
    if (!plc || addr >= PLC_MAX_INTERNAL_BITS) return -1;
    if (addr >= plc->num_internal_bits) plc->num_internal_bits = addr + 1;
    plc->internal_bits[addr] = (value != 0);
    return 0;
}

int plc_config_digital_io(plc_system_t *plc, uint16_t addr,
                          const char *tag, plc_io_type_t io_type)
{
    if (!plc) return -1;
    switch (io_type) {
    case PLC_IO_DISCRETE_INPUT:
        if (addr >= PLC_MAX_DIGITAL_IO) return -1;
        strncpy(plc->digital_inputs[addr].tag, tag, 31);
        plc->digital_inputs[addr].tag[31] = '\0';
        plc->digital_inputs[addr].address = addr;
        plc->digital_inputs[addr].io_type = io_type;
        if (addr >= plc->num_digital_in) plc->num_digital_in = addr + 1;
        break;
    case PLC_IO_DISCRETE_OUTPUT:
        if (addr >= PLC_MAX_DIGITAL_IO) return -1;
        strncpy(plc->digital_outputs[addr].tag, tag, 31);
        plc->digital_outputs[addr].tag[31] = '\0';
        plc->digital_outputs[addr].address = addr;
        plc->digital_outputs[addr].io_type = io_type;
        if (addr >= plc->num_digital_out) plc->num_digital_out = addr + 1;
        break;
    case PLC_IO_INTERNAL_BIT:
        if (addr >= PLC_MAX_INTERNAL_BITS) return -1;
        if (addr >= plc->num_internal_bits) plc->num_internal_bits = addr + 1;
        break;
    default: return -1;
    }
    return 0;
}

/* Analog I/O */

double plc_read_analog_input(const plc_system_t *plc, uint16_t addr)
{ if (!plc || addr >= plc->num_analog_in) return 0.0; return plc->analog_inputs[addr].filtered_value; }

double plc_read_analog_output(const plc_system_t *plc, uint16_t addr)
{ if (!plc || addr >= plc->num_analog_out) return 0.0; return plc->analog_outputs[addr].eng_value; }

double plc_read_internal_reg(const plc_system_t *plc, uint16_t addr)
{ if (!plc || addr >= plc->num_internal_regs) return 0.0; return plc->internal_regs[addr]; }

int plc_write_analog_output(plc_system_t *plc, uint16_t addr, double value)
{
    if (!plc || addr >= PLC_MAX_ANALOG_IO) return -1;
    if (addr >= plc->num_analog_out) plc->num_analog_out = addr + 1;
    plc->analog_outputs[addr].eng_value = value;
    return 0;
}

int plc_write_internal_reg(plc_system_t *plc, uint16_t addr, double value)
{
    if (!plc || addr >= PLC_MAX_INTERNAL_REGS) return -1;
    if (addr >= plc->num_internal_regs) plc->num_internal_regs = addr + 1;
    plc->internal_regs[addr] = value;
    return 0;
}

int plc_config_analog_io(plc_system_t *plc, uint16_t addr, const char *tag,
                         plc_io_type_t io_type, double scale_min,
                         double scale_max, double eng_min, double eng_max,
                         double filter_alpha)
{
    if (!plc) return -1;
    plc_analog_io_t *aio = NULL;
    switch (io_type) {
    case PLC_IO_ANALOG_INPUT:
        if (addr >= PLC_MAX_ANALOG_IO) return -1;
        aio = &plc->analog_inputs[addr];
        if (addr >= plc->num_analog_in) plc->num_analog_in = addr + 1;
        break;
    case PLC_IO_ANALOG_OUTPUT:
        if (addr >= PLC_MAX_ANALOG_IO) return -1;
        aio = &plc->analog_outputs[addr];
        if (addr >= plc->num_analog_out) plc->num_analog_out = addr + 1;
        break;
    default: return -1;
    }
    strncpy(aio->tag, tag, 31); aio->tag[31] = '\0';
    aio->address = addr; aio->io_type = io_type;
    aio->scale_min = scale_min; aio->scale_max = scale_max;
    aio->eng_min = eng_min; aio->eng_max = eng_max;
    aio->filter_alpha = filter_alpha;
    aio->filtered_value = 0.0; aio->rate_of_change = 0.0;
    return 0;
}

void plc_analog_scale_and_filter(plc_analog_io_t *aio)
{
    if (!aio) return;
    double denom = aio->scale_max - aio->scale_min;
    double scaled = (fabs(denom) < 1e-12) ? 0.0 :
        aio->eng_min + (aio->raw_value - aio->scale_min) *
        (aio->eng_max - aio->eng_min) / denom;
    double alpha = aio->filter_alpha;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    double prev = aio->filtered_value;
    aio->filtered_value = alpha * scaled + (1.0 - alpha) * prev;
    aio->rate_of_change = aio->filtered_value - prev;
    aio->eng_value = scaled;
}
