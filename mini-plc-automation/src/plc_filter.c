#include "plc_core.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* First-order IIR filter */
void plc_iir_filter_init(plc_iir_filter_t *f, double alpha)
{
    if (!f) return;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    f->alpha = alpha;
    f->last_output = 0.0;
    f->initialized = 0;
}

double plc_iir_filter_update(plc_iir_filter_t *f, double input)
{
    if (!f) return 0.0;
    if (!f->initialized) {
        f->last_output = input;
        f->initialized = 1;
        return input;
    }
    f->last_output = f->alpha * input + (1.0 - f->alpha) * f->last_output;
    return f->last_output;
}

double plc_iir_alpha_from_fc(double fc_hz, double fs_hz)
{
    if (fc_hz < 0.0 || fs_hz < 1e-12) return 1.0;
    double omega = 2.0 * M_PI * fc_hz / fs_hz;
    if (omega > 20.0) return 1.0;
    return 1.0 - exp(-omega);
}

/* Moving average filter */
int plc_moving_avg_init(plc_moving_avg_t *ma, size_t window_size)
{
    if (!ma || window_size == 0) return -1;
    ma->buffer = (double*)calloc(window_size, sizeof(double));
    if (!ma->buffer) return -1;
    ma->window_size = window_size;
    ma->index = 0; ma->count = 0; ma->sum = 0.0;
    return 0;
}

void plc_moving_avg_free(plc_moving_avg_t *ma)
{
    if (!ma) return;
    free(ma->buffer); ma->buffer = NULL; ma->window_size = 0;
}

double plc_moving_avg_update(plc_moving_avg_t *ma, double input)
{
    if (!ma || !ma->buffer) return 0.0;
    size_t n = ma->window_size;
    if (ma->count < n) {
        ma->buffer[ma->index] = input;
        ma->sum += input;
        ma->count++;
        ma->index = (ma->index + 1) % n;
        return ma->sum / (double)ma->count;
    }
    double old = ma->buffer[ma->index];
    ma->buffer[ma->index] = input;
    ma->sum = ma->sum - old + input;
    ma->index = (ma->index + 1) % n;
    return ma->sum / (double)n;
}

void plc_moving_avg_reset(plc_moving_avg_t *ma)
{
    if (!ma || !ma->buffer) return;
    memset(ma->buffer, 0, ma->window_size * sizeof(double));
    ma->index = 0; ma->count = 0; ma->sum = 0.0;
}
