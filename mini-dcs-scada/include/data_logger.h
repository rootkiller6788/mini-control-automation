/*
 * data_logger.h - Process Data Historian and Logging
 *
 * Implements time-series data storage for process historians, the
 * foundational technology behind SCADA data analysis and reporting.
 *
 * The historian captures process data over long periods (months to years)
 * enabling trend analysis, regulatory compliance reporting, and
 * process optimization.
 *
 * References:
 *   - OSIsoft PI System architecture
 *   - ISA-88 batch data logging
 *   - 21 CFR Part 11 (FDA electronic records for pharma)
 *
 * Course Alignment:
 *   Stanford EE392 - Digital Control (discrete-time data logging)
 *   Tsinghua - Industrial Process Data Analysis
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * L1: Data Sample Types
 * ============================================================================
 */

/** A single time-stamped data sample.
 *  The fundamental unit of a process historian. */
typedef struct {
    uint64_t timestamp_ms;    /* UNIX timestamp in milliseconds */
    double   value;           /* Process value in engineering units */
    uint16_t quality;         /* OPC quality flags */
    uint8_t  flags;           /* User-defined flags (e.g., manual entry) */
} data_sample_t;

/* ============================================================================
 * L2: Circular Buffer - Fixed-size recent data store
 * ============================================================================
 */

#define CIRC_BUF_SIZE 1024

/**
 * Circular buffer for real-time trending data.
 *
 * Stores the most recent N samples. When full, oldest data is overwritten.
 * Used for real-time trend displays (last hour, last shift, etc.).
 *
 * O(1) operations for read, write, and statistical queries over windows.
 */
typedef struct {
    data_sample_t samples[CIRC_BUF_SIZE];
    uint32_t      head;             /* Write position */
    uint32_t      count;            /* Number of valid samples */
    uint64_t      first_time;       /* Timestamp of oldest sample */
    uint64_t      last_time;        /* Timestamp of newest sample */
    double        sum;              /* Running sum for fast average */
    double        sum_sq;           /* Running sum of squares for variance */
} circ_buf_t;

void circ_buf_init(circ_buf_t *cb);
bool circ_buf_push(circ_buf_t *cb, const data_sample_t *sample);
const data_sample_t *circ_buf_get(const circ_buf_t *cb, uint32_t index);
uint32_t circ_buf_count(const circ_buf_t *cb);

/** Compute average of all samples in buffer. O(1). */
double circ_buf_average(const circ_buf_t *cb);

/** Compute standard deviation of all samples in buffer. O(1). */
double circ_buf_stddev(const circ_buf_t *cb);

/** Find minimum value in buffer. O(n). */
double circ_buf_min(const circ_buf_t *cb);

/** Find maximum value in buffer. O(n). */
double circ_buf_max(const circ_buf_t *cb);

/** Get the oldest sample (FIFO peek) */
const data_sample_t *circ_buf_oldest(const circ_buf_t *cb);

/** Get the newest sample */
const data_sample_t *circ_buf_newest(const circ_buf_t *cb);

/* ============================================================================
 * L3: Time-Weighted Average
 * ============================================================================
 */

/**
 * Time-weighted average (integral average over time).
 *
 * TWA = (1 / (t_end - t_start)) * integral_{t_start}^{t_end} x(t) dt
 *
 * Uses trapezoidal integration between samples.
 * Required for environmental compliance reporting (EPA, EU directives).
 */
double time_weighted_average(const data_sample_t *samples, uint32_t count);

/**
 * Time-weighted total (integral over time).
 *
 * TWT = integral_{t_start}^{t_end} x(t) dt
 *
 * Used for flow totalization and energy metering:
 *   Total Volume = integral Q(t) dt
 *   Total Energy  = integral P(t) dt
 */
double time_weighted_total(const data_sample_t *samples, uint32_t count);

/* ============================================================================
 * L5: Rate-of-Change Computation
 * ============================================================================
 */

/**
 * Calculate rate of change from sample array.
 * Uses linear regression over a window for noise robustness.
 *
 * dy/dt = (N*sum(xy) - sum(x)*sum(y)) / (N*sum(x^2) - sum(x)^2)
 *
 * @return Rate in units/second
 */
double rate_of_change(const data_sample_t *samples, uint32_t count);

/* ============================================================================
 * L5: Interpolation for Missing Data
 * ============================================================================
 */

/**
 * Linear interpolation at a target time.
 * Assumes samples are sorted by timestamp. Returns interpolated value,
 * or NaN if target_time is outside the sample range.
 */
double linear_interp(const data_sample_t *samples, uint32_t count,
                     uint64_t target_time_ms);

/**
 * Nearest-neighbor interpolation (zero-order hold).
 * Returns the value of the sample closest in time to target_time.
 */
double nearest_interp(const data_sample_t *samples, uint32_t count,
                      uint64_t target_time_ms);

/* ============================================================================
 * L6: Historian Buffer - Long-term compressed storage
 * ============================================================================
 */

#define HISTORIAN_MAX_POINTS 10000

/**
 * Deadband-compressed historian point.
 *
 * Stores only significant changes rather than every sample.
 * This is the "archive" compression used by process historians.
 * Typical compression ratio: 10:1 to 100:1 depending on deadband.
 */
typedef struct {
    uint64_t timestamp_ms;
    double   value;
} historian_point_t;

/**
 * Process historian: long-term storage with deadband compression.
 *
 * Writes a new point only when the value changes by more than
 * the specified deadband from the last stored point.
 */
typedef struct {
    historian_point_t points[HISTORIAN_MAX_POINTS];
    uint32_t          count;
    double            deadband;
    double            last_stored_value;
    uint64_t          last_stored_time;
    uint64_t          first_time;
    uint64_t          last_time;
} historian_t;

void historian_init(historian_t *hist, double deadband);

/**
 * Feed a new sample to the historian.
 * @return true if the sample was stored (passed deadband check)
 */
bool historian_feed(historian_t *hist, uint64_t timestamp_ms, double value);

/** Get count of stored points */
uint32_t historian_count(const historian_t *hist);

/** Get a stored point by index */
const historian_point_t *historian_get(const historian_t *hist, uint32_t index);

/**
 * Compact the historian: re-apply deadband filtering to the stored data
 * to remove any points that no longer meet the deadband criterion.
 * @return Number of points removed
 */
uint32_t historian_compact(historian_t *hist);

/* ============================================================================
 * L7: Batch Record - ISA-88 Batch Data Model
 * ============================================================================
 */

#define BATCH_ID_MAX 32
#define BATCH_PRODUCT_MAX 64

/**
 * Batch record: captures all relevant data for a batch process run.
 * Required by FDA 21 CFR Part 11 for pharmaceutical manufacturing.
 */
typedef struct {
    char        batch_id[BATCH_ID_MAX];
    char        product[BATCH_PRODUCT_MAX];
    uint64_t    start_time_ms;
    uint64_t    end_time_ms;
    double      setpoint;
    double      final_yield;
    double      total_energy_kwh;
    double      total_feedstock_kg;
    double      peak_temperature;
    double      min_temperature;
    double      avg_pressure;
    uint32_t    alarm_count;
    uint32_t    deviation_count;
    bool        completed;
    uint8_t     operator_signature[32];  /* Simplified digital signature */
} batch_record_t;

void batch_record_init(batch_record_t *batch, const char *batch_id,
                        const char *product, double setpoint);
void batch_record_close(batch_record_t *batch, uint64_t end_time_ms,
                         double yield, double energy, double feedstock);

/** Compute cycle time efficiency: actual_time / planned_time */
double batch_cycle_efficiency(const batch_record_t *batch, double planned_time_ms);

/* ============================================================================
 * L7: Data Export - CSV format for regulatory reporting
 * ============================================================================
 */

#define CSV_EXPORT_MAX 4096

/**
 * Export historian data to CSV format.
 * Timestamp,Value format suitable for Excel, MATLAB, Python pandas.
 *
 * @return Number of characters written (excluding null terminator)
 */
int historian_export_csv(const historian_t *hist, char *buffer, size_t buf_size);

/**
 * Compute daily statistics from historian data.
 * For each calendar day: min, max, avg, stddev.
 */
#define DAILY_STATS_DAYS 365

typedef struct {
    uint64_t date_ms;       /* Midnight timestamp */
    double   day_min;
    double   day_max;
    double   day_avg;
    double   day_stddev;
    uint32_t sample_count;
} daily_stats_t;

/** Compute daily statistics (simplified: uses even binning) */
uint32_t daily_stats_compute(const historian_t *hist, daily_stats_t *stats,
                              uint32_t max_days);

#endif /* DATA_LOGGER_H */