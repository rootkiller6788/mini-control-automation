/*
 * data_logger.c - Process Data Historian Implementation
 *
 * Implements circular buffers, time-weighted averages, deadband-compressed
 * historians, batch records (ISA-88), and CSV export.
 */

#include "data_logger.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ============================================================================
 * Circular Buffer
 *
 * L2: Fixed-size FIFO buffer for real-time trend data.
 *
 * The circular buffer stores the most recent N samples and provides
 * O(1) statistical queries (average, variance) by maintaining
 * running sums.
 *
 * This is the fundamental data structure behind every DCS trend
 * display: the "last hour" trend shows the last 3600 samples
 * (at 1 Hz), implemented as a circular buffer.
 * ============================================================================
 */

void circ_buf_init(circ_buf_t *cb) {
    if (!cb) return;
    memset(cb, 0, sizeof(circ_buf_t));
}

bool circ_buf_push(circ_buf_t *cb, const data_sample_t *sample) {
    if (!cb || !sample) return false;

    /* If buffer is full, remove oldest sample from running statistics */
    if (cb->count >= CIRC_BUF_SIZE) {
        uint32_t oldest_idx = (cb->head + CIRC_BUF_SIZE - cb->count) % CIRC_BUF_SIZE;
        double old_val = cb->samples[oldest_idx].value;
        cb->sum   -= old_val;
        cb->sum_sq -= old_val * old_val;
    } else {
        cb->count++;
    }

    /* Write new sample */
    cb->samples[cb->head] = *sample;
    cb->sum   += sample->value;
    cb->sum_sq += sample->value * sample->value;

    if (cb->count == 1) {
        cb->first_time = sample->timestamp_ms;
    }
    cb->last_time = sample->timestamp_ms;

    cb->head = (cb->head + 1) % CIRC_BUF_SIZE;
    return true;
}

const data_sample_t *circ_buf_get(const circ_buf_t *cb, uint32_t index) {
    if (!cb || index >= cb->count) return NULL;
    /* Index 0 = oldest sample */
    uint32_t oldest = (cb->head + CIRC_BUF_SIZE - cb->count) % CIRC_BUF_SIZE;
    uint32_t actual = (oldest + index) % CIRC_BUF_SIZE;
    return &cb->samples[actual];
}

uint32_t circ_buf_count(const circ_buf_t *cb) {
    if (!cb) return 0;
    return cb->count;
}

double circ_buf_average(const circ_buf_t *cb) {
    if (!cb || cb->count == 0) return 0.0;
    return cb->sum / (double)cb->count;
}

double circ_buf_stddev(const circ_buf_t *cb) {
    if (!cb || cb->count < 2) return 0.0;
    double mean = circ_buf_average(cb);
    double variance = (cb->sum_sq / (double)cb->count) - (mean * mean);
    if (variance < 0.0) variance = 0.0; /* Numerical guard */
    return sqrt(variance);
}

double circ_buf_min(const circ_buf_t *cb) {
    if (!cb || cb->count == 0) return 0.0;
    double min_val = INFINITY;
    for (uint32_t i = 0; i < cb->count; i++) {
        const data_sample_t *s = circ_buf_get(cb, i);
        if (s && s->value < min_val) min_val = s->value;
    }
    return min_val;
}

double circ_buf_max(const circ_buf_t *cb) {
    if (!cb || cb->count == 0) return 0.0;
    double max_val = -INFINITY;
    for (uint32_t i = 0; i < cb->count; i++) {
        const data_sample_t *s = circ_buf_get(cb, i);
        if (s && s->value > max_val) max_val = s->value;
    }
    return max_val;
}

const data_sample_t *circ_buf_oldest(const circ_buf_t *cb) {
    return circ_buf_get(cb, 0);
}

const data_sample_t *circ_buf_newest(const circ_buf_t *cb) {
    if (!cb || cb->count == 0) return NULL;
    return circ_buf_get(cb, cb->count - 1);
}

/* ============================================================================
 * Time-Weighted Average and Total
 *
 * L3: Integral-based statistics for regulatory compliance.
 *
 * Time-Weighted Average (TWA):
 *   TWA = integral_{t0}^{tN} x(t) dt / (tN - t0)
 *   Numerically: TWA = sum_{i=1}^{N-1} (x_i + x_{i-1})/2 * (t_i - t_{i-1}) / (tN - t0)
 *
 * This is the mathematically correct average for non-uniformly sampled
 * data. The naive arithmetic mean assumes equal spacing, which is wrong
 * for compressed historian data.
 *
 * TWA is required by:
 *   - EPA: ambient air quality monitoring
 *   - OSHA: workplace exposure limits (8-hour TWA)
 *   - FDA: sterilization validation (F0 calculation)
 * ============================================================================
 */

double time_weighted_average(const data_sample_t *samples, uint32_t count) {
    if (!samples || count < 2) {
        return (samples && count == 1) ? samples[0].value : 0.0;
    }

    double integral = 0.0;
    for (uint32_t i = 1; i < count; i++) {
        double dt = (double)(samples[i].timestamp_ms - samples[i-1].timestamp_ms) / 1000.0;
        /* Trapezoidal rule */
        double avg_val = (samples[i].value + samples[i-1].value) / 2.0;
        integral += avg_val * dt;
    }

    double total_time = (double)(samples[count-1].timestamp_ms - samples[0].timestamp_ms) / 1000.0;

    if (total_time <= 0.0) return 0.0;
    return integral / total_time;
}

double time_weighted_total(const data_sample_t *samples, uint32_t count) {
    if (!samples || count < 2) return 0.0;

    double integral = 0.0;
    for (uint32_t i = 1; i < count; i++) {
        double dt = (double)(samples[i].timestamp_ms - samples[i-1].timestamp_ms) / 1000.0;
        double avg_val = (samples[i].value + samples[i-1].value) / 2.0;
        integral += avg_val * dt;
    }

    return integral;
}

/* ============================================================================
 * Rate of Change via Linear Regression
 *
 * L5: Robust derivative estimation using least-squares.
 *
 * Instead of simple finite difference (y2-y1)/(t2-t1), which amplifies
 * noise, we fit a line to a window of points and take its slope.
 *
 * Regression formula (minimizing squared error):
 *   slope = (N * sum(x_i*y_i) - sum(x_i) * sum(y_i))
 *         / (N * sum(x_i^2) - sum(x_i)^2)
 *
 * where x_i = time (centered for numerical stability), y_i = value.
 *
 * This provides noise-robust rate estimation for derivative control,
 * trend alarms, and process diagnostics.
 * ============================================================================
 */

double rate_of_change(const data_sample_t *samples, uint32_t count) {
    if (!samples || count < 2) return 0.0;

    /* Use time relative to first sample for numerical stability */
    double t0 = (double)samples[0].timestamp_ms / 1000.0;
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;

    for (uint32_t i = 0; i < count; i++) {
        double t = (double)samples[i].timestamp_ms / 1000.0 - t0;
        double y = samples[i].value;
        sum_x  += t;
        sum_y  += y;
        sum_xy += t * y;
        sum_xx += t * t;
    }

    double n = (double)count;
    double denom = n * sum_xx - sum_x * sum_x;

    if (fabs(denom) < 1e-15) return 0.0;

    double slope = (n * sum_xy - sum_x * sum_y) / denom;
    return slope; /* Units: value/second */
}

/* ============================================================================
 * Interpolation
 * ============================================================================
 */

double linear_interp(const data_sample_t *samples, uint32_t count,
                     uint64_t target_time_ms) {
    if (!samples || count == 0) return NAN;
    if (count == 1) return samples[0].value;

    /* Before first sample */
    if (target_time_ms <= samples[0].timestamp_ms) {
        return samples[0].value;
    }

    /* After last sample */
    if (target_time_ms >= samples[count-1].timestamp_ms) {
        return samples[count-1].value;
    }

    /* Find bracketing interval */
    for (uint32_t i = 0; i < count - 1; i++) {
        if (target_time_ms >= samples[i].timestamp_ms &&
            target_time_ms <= samples[i+1].timestamp_ms) {

            double dt = (double)(samples[i+1].timestamp_ms - samples[i].timestamp_ms);
            if (dt < 0.001) return samples[i].value;

            double t = (double)(target_time_ms - samples[i].timestamp_ms) / dt;
            return samples[i].value + t * (samples[i+1].value - samples[i].value);
        }
    }

    return NAN;
}

double nearest_interp(const data_sample_t *samples, uint32_t count,
                      uint64_t target_time_ms) {
    if (!samples || count == 0) return NAN;

    double best_val = samples[0].value;
    uint64_t best_diff = (target_time_ms > samples[0].timestamp_ms)
        ? target_time_ms - samples[0].timestamp_ms
        : samples[0].timestamp_ms - target_time_ms;

    for (uint32_t i = 1; i < count; i++) {
        uint64_t diff = (target_time_ms > samples[i].timestamp_ms)
            ? target_time_ms - samples[i].timestamp_ms
            : samples[i].timestamp_ms - target_time_ms;

        if (diff < best_diff) {
            best_diff = diff;
            best_val = samples[i].value;
        }
    }

    return best_val;
}

/* ============================================================================
 * Historian
 *
 * L6: Deadband-compressed long-term data storage.
 *
 * The process historian is the most data-intensive subsystem of SCADA.
 * A medium-sized plant with 10,000 tags at 1 second scan rate would
 * generate 864 million samples per day (~69 GB of raw double-precision
 * data). Deadband compression reduces this by 10-100x.
 *
 * The historian also serves as the "system of record" for:
 *   - Regulatory compliance (EPA continuous emissions monitoring)
 *   - Quality assurance (batch records for pharma/food)
 *   - Process optimization (identifying inefficiencies)
 *   - Forensic analysis (what happened before the incident?)
 * ============================================================================
 */

void historian_init(historian_t *hist, double deadband) {
    if (!hist) return;
    memset(hist, 0, sizeof(historian_t));
    hist->deadband = fabs(deadband);
}

bool historian_feed(historian_t *hist, uint64_t timestamp_ms, double value) {
    if (!hist) return false;
    if (hist->count >= HISTORIAN_MAX_POINTS) return false;

    /* First point: always store */
    if (hist->count == 0) {
        hist->points[0].timestamp_ms = timestamp_ms;
        hist->points[0].value        = value;
        hist->count++;
        hist->last_stored_value = value;
        hist->last_stored_time  = timestamp_ms;
        hist->first_time        = timestamp_ms;
        hist->last_time         = timestamp_ms;
        return true;
    }

    /* Deadband check: store only if value changed significantly */
    if (fabs(value - hist->last_stored_value) >= hist->deadband) {
        hist->points[hist->count].timestamp_ms = timestamp_ms;
        hist->points[hist->count].value        = value;
        hist->count++;

        hist->last_stored_value = value;
        hist->last_stored_time  = timestamp_ms;
        hist->last_time         = timestamp_ms;
        return true;
    }

    /* Always update last_time to track time range */
    hist->last_time = timestamp_ms;
    return false;
}

uint32_t historian_count(const historian_t *hist) {
    if (!hist) return 0;
    return hist->count;
}

const historian_point_t *historian_get(const historian_t *hist, uint32_t index) {
    if (!hist || index >= hist->count) return NULL;
    return &hist->points[index];
}

uint32_t historian_compact(historian_t *hist) {
    if (!hist || hist->count < 2) return 0;

    uint32_t write_idx = 1;
    double last_val = hist->points[0].value;
    uint32_t removed = 0;

    for (uint32_t read_idx = 1; read_idx < hist->count; read_idx++) {
        double val = hist->points[read_idx].value;
        if (fabs(val - last_val) >= hist->deadband) {
            /* Keep this point */
            if (write_idx != read_idx) {
                hist->points[write_idx] = hist->points[read_idx];
            }
            last_val = val;
            write_idx++;
        } else {
            removed++;
        }
    }

    hist->count = write_idx;
    return removed;
}

/* ============================================================================
 * Batch Record (ISA-88)
 *
 * L7: Batch process data capture for pharmaceutical manufacturing.
 *
 * ISA-88 defines the standard model for batch control:
 *   Procedure > Unit Procedure > Operation > Phase
 *
 * Each batch run produces a batch record documenting:
 *   - Recipe parameters (setpoints, timings)
 *   - Actual process data (temperatures, pressures, flows)
 *   - Alarm and deviation events
 *   - Operator actions and electronic signatures
 *   - Final yield and quality metrics
 *
 * 21 CFR Part 11 (FDA) requires:
 *   - Electronic records equivalent to paper records
 *   - Electronic signatures with traceability
 *   - Audit trails of all changes
 *   - Records retention for specified periods
 * ============================================================================
 */

void batch_record_init(batch_record_t *batch, const char *batch_id,
                        const char *product, double setpoint) {
    if (!batch) return;
    memset(batch, 0, sizeof(batch_record_t));

    if (batch_id) strncpy(batch->batch_id, batch_id, BATCH_ID_MAX - 1);
    if (product) strncpy(batch->product, product, BATCH_PRODUCT_MAX - 1);
    batch->setpoint = setpoint;
    batch->completed = false;
}

void batch_record_close(batch_record_t *batch, uint64_t end_time_ms,
                         double yield, double energy, double feedstock) {
    if (!batch) return;

    batch->end_time_ms        = end_time_ms;
    batch->final_yield        = yield;
    batch->total_energy_kwh   = energy;
    batch->total_feedstock_kg = feedstock;
    batch->completed          = true;
}

double batch_cycle_efficiency(const batch_record_t *batch, double planned_time_ms) {
    if (!batch || !batch->completed || planned_time_ms <= 0.0) return 0.0;
    double actual = (double)(batch->end_time_ms - batch->start_time_ms);
    return planned_time_ms / actual; /* >1 = faster than planned, <1 = slower */
}

/* ============================================================================
 * CSV Export
 *
 * L7: Data export in universal format for regulatory and analysis needs.
 *
 * Format: ISO8601_timestamp, value, quality, flags
 * Compatible with Excel, MATLAB, Python pandas, R.
 * ============================================================================
 */

int historian_export_csv(const historian_t *hist, char *buffer, size_t buf_size) {
    if (!hist || !buffer || buf_size == 0) return 0;

    int written = 0;
    written = snprintf(buffer, buf_size, "timestamp,value\n");

    for (uint32_t i = 0; i < hist->count && written < (int)buf_size - 100; i++) {
        int n = snprintf(buffer + written, buf_size - written,
                         "%llu,%.6f\n",
                         (unsigned long long)hist->points[i].timestamp_ms,
                         hist->points[i].value);
        if (n < 0 || (size_t)n >= buf_size - written) break;
        written += n;
    }

    return written;
}

/* ============================================================================
 * Daily Statistics
 *
 * L7: Aggregation for regulatory reporting and process KPIs.
 *
 * Many environmental regulations require daily statistical summaries:
 *   - Daily maximum (e.g., SO2 emissions must not exceed X ppm)
 *   - Daily average (e.g., effluent temperature < 30 C average)
 *   - Daily minimum
 *
 * This function uses simplified even binning rather than true
 * calendar-day alignment (which requires timezone awareness).
 * ============================================================================
 */

uint32_t daily_stats_compute(const historian_t *hist, daily_stats_t *stats,
                              uint32_t max_days) {
    if (!hist || !stats || hist->count == 0 || max_days == 0) return 0;

    /* Simplified: divide the entire time range into max_days bins */
    uint64_t total_ms = hist->last_time - hist->first_time;
    if (total_ms == 0) total_ms = 86400000; /* Default 1 day */

    uint64_t bin_ms = total_ms / max_days;
    if (bin_ms == 0) bin_ms = 1;

    uint32_t day_count = 0;
    uint64_t day_start = hist->first_time;

    for (uint32_t d = 0; d < max_days && day_count < HISTORIAN_MAX_POINTS / 2; d++) {
        uint64_t day_end = day_start + bin_ms;

        /* Collect stats for this bin */
        double day_min = INFINITY;
        double day_max = -INFINITY;
        double day_sum = 0.0;
        double day_sum_sq = 0.0;
        uint32_t count = 0;

        for (uint32_t i = 0; i < hist->count; i++) {
            if (hist->points[i].timestamp_ms >= day_start &&
                hist->points[i].timestamp_ms < day_end) {
                double v = hist->points[i].value;
                if (v < day_min) day_min = v;
                if (v > day_max) day_max = v;
                day_sum += v;
                day_sum_sq += v * v;
                count++;
            }
        }

        if (count > 0) {
            stats[day_count].date_ms   = day_start;
            stats[day_count].day_min   = day_min;
            stats[day_count].day_max   = day_max;
            stats[day_count].day_avg   = day_sum / (double)count;
            stats[day_count].sample_count = count;

            double mean = stats[day_count].day_avg;
            double var = (day_sum_sq / (double)count) - (mean * mean);
            stats[day_count].day_stddev = sqrt(var > 0.0 ? var : 0.0);

            day_count++;
        }

        day_start = day_end;
    }

    return day_count;
}