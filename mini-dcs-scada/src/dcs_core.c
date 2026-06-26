/*
 * dcs_core.c - DCS/SCADA Core Implementation
 *
 * Implements tag database operations, scan engine, and actuator
 * command shaping utilities.
 */

#include "dcs_core.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * Tag Database Operations
 * ============================================================================
 */

void dcs_tag_db_init(dcs_tag_db_t *db) {
    if (!db) return;
    memset(db, 0, sizeof(dcs_tag_db_t));
    db->scan_active = false;
    db->last_scan_time_ms = 0;
    db->scan_counter = 0;
}

int dcs_tag_add(dcs_tag_db_t *db, const dcs_tag_t *tag) {
    if (!db || !tag) return -1;
    if (db->count >= DCS_MAX_TAGS) return -1;

    uint32_t idx = db->count;
    memcpy(&db->tags[idx], tag, sizeof(dcs_tag_t));
    db->count++;
    return (int)idx;
}

dcs_tag_t *dcs_tag_find(dcs_tag_db_t *db, const char *name) {
    if (!db || !name) return NULL;
    for (uint32_t i = 0; i < db->count; i++) {
        if (strncmp(db->tags[i].name, name, DCS_TAG_NAME_MAX) == 0) {
            return &db->tags[i];
        }
    }
    return NULL;
}

dcs_tag_t *dcs_tag_at(dcs_tag_db_t *db, uint32_t index) {
    if (!db || index >= db->count) return NULL;
    return &db->tags[index];
}

void dcs_tag_update_pv(dcs_tag_t *tag, double raw_value) {
    if (!tag) return;
    tag->pv = raw_value;
    tag->timestamp_ms = 0; /* Caller must set real timestamp */
    tag->update_count++;
}

bool dcs_tag_eu_to_pct(const dcs_tag_t *tag, double eu_value, double *pct_out) {
    if (!tag || !pct_out) return false;
    double eu_range = tag->eu_hi - tag->eu_lo;
    if (fabs(eu_range) < 1e-12) return false;
    double pct_range = tag->pct_hi - tag->pct_lo;
    *pct_out = tag->pct_lo + (eu_value - tag->eu_lo) * pct_range / eu_range;
    return true;
}

bool dcs_tag_pct_to_eu(const dcs_tag_t *tag, double pct_value, double *eu_out) {
    if (!tag || !eu_out) return false;
    double pct_range = tag->pct_hi - tag->pct_lo;
    if (fabs(pct_range) < 1e-12) return false;
    double eu_range = tag->eu_hi - tag->eu_lo;
    *eu_out = tag->eu_lo + (pct_value - tag->pct_lo) * eu_range / pct_range;
    return true;
}

/* ============================================================================
 * Scan Engine
 *
 * Models the cyclic execution engine of a DCS. In real systems (Honeywell
 * TDC, Emerson DeltaV, Siemens PCS7), the scan engine is a real-time
 * scheduler that executes control blocks at their configured scan rates.
 *
 * L2: Core concept of cyclic execution in real-time control systems.
 * L4: The scan rate must satisfy the Nyquist criterion for each loop:
 *      scan_rate >= 2 * loop_bandwidth.
 *
 * For PID loops, practical rule: scan rate >= 10 * loop_bandwidth for
 * the continuous-time tuning rules to be valid in discrete time.
 * ============================================================================
 */

void dcs_scan_engine_init(dcs_scan_engine_t *engine, dcs_tag_db_t *db) {
    if (!engine) return;
    memset(engine, 0, sizeof(dcs_scan_engine_t));
    engine->db = db;
    engine->running = false;
    engine->stats.min_scan_time_ms = 1e12;
}

/* Map scan rate enum to milliseconds */
static uint32_t scan_rate_to_ms(dcs_scan_rate_t rate) {
    switch (rate) {
        case SCAN_10MS:  return 10;
        case SCAN_50MS:  return 50;
        case SCAN_100MS: return 100;
        case SCAN_250MS: return 250;
        case SCAN_500MS: return 500;
        case SCAN_1S:    return 1000;
        case SCAN_5S:    return 5000;
        case SCAN_10S:   return 10000;
        case SCAN_30S:   return 30000;
        case SCAN_1M:    return 60000;
        default:         return 1000;
    }
}

uint32_t dcs_scan_engine_run(dcs_scan_engine_t *engine) {
    if (!engine || !engine->db) return 0;

    uint32_t processed = 0;

    engine->running = true;

    for (uint32_t i = 0; i < engine->db->count; i++) {
        dcs_tag_t *tag = &engine->db->tags[i];
        if (!tag->enabled) continue;

        /* Check if this tag's scan period has elapsed.
         * In a real DCS, this uses a multi-rate scheduler with
         * phase-shifted scan groups to spread CPU load.
         * Here we implement a simplified single-rate scan. */
        uint32_t period_ms = scan_rate_to_ms(tag->scan_rate);
        if ((engine->db->scan_counter % (period_ms / 10)) != 0) {
            continue; /* Skip if not this tag's scan cycle */
        }

        /* === Execute control block logic based on tag type === */

        /* For simplicity, the scan engine marks tags as processed.
         * Actual control computation (PID, etc.) is done by the
         * application calling pid_update() between scan cycles.
         *
         * The scan engine's role is:
         *   1. Read inputs (AI/DI cards)
         *   2. Execute control blocks
         *   3. Write outputs (AO/DO cards)
         *   4. Check alarms
         *   5. Log data
         *
         * In a real DCS, steps 1-5 are orchestrated by the scan engine
         * with strict timing guarantees. Here we model the scheduling
         * logic layer. */

        tag->timestamp_ms = 0; /* Would be set by I/O subsystem */
        processed++;
    }

    engine->db->scan_counter++;
    engine->running = false;

    /* Update scan statistics */
    engine->stats.total_scans++;
    if (processed > 0) {
        /* In real system: scan_time_ms = read_hardware_timer() - start_ticks */
        double scan_time_ms = 0.0; /* Placeholder for real timer read */
        if (scan_time_ms < engine->stats.min_scan_time_ms) {
            engine->stats.min_scan_time_ms = scan_time_ms;
        }
        if (scan_time_ms > engine->stats.max_scan_time_ms) {
            engine->stats.max_scan_time_ms = scan_time_ms;
        }
        /* Exponential moving average for scan time */
        engine->stats.avg_scan_time_ms =
            0.95 * engine->stats.avg_scan_time_ms + 0.05 * scan_time_ms;
    }

    return processed;
}

const dcs_scan_stats_t *dcs_scan_engine_stats(const dcs_scan_engine_t *engine) {
    if (!engine) return NULL;
    return &engine->stats;
}

/* ============================================================================
 * Slew Rate Limiter
 *
 * L2: Actuator command shaping to prevent mechanical stress.
 *
 * Physical motivation: A control valve has finite actuation speed.
 * Commanding a step change can cause:
 *   - Water hammer in liquid pipelines
 *   - Compressor surge
 *   - Excessive wear on valve stem and seat
 *
 * The slew rate limiter enforces:
 *   |output_new - output_prev| / dt <= max_rate
 *
 * This is equivalent to rate limiting in signal processing,
 * applied to the actuation path.
 * ============================================================================
 */

double dcs_slew_limit(dcs_slew_limiter_t *lim, double desired, double dt) {
    if (!lim || dt <= 0.0) return desired;

    double max_step = lim->max_rate * dt;
    double diff = desired - lim->last_output;
    double limited;

    if (diff > max_step) {
        limited = lim->last_output + max_step;
    } else if (diff < -max_step) {
        limited = lim->last_output - max_step;
    } else {
        limited = desired;
    }

    lim->last_output = limited;
    lim->dt_seconds = dt;
    return limited;
}

/* ============================================================================
 * Reversal Detector
 *
 * L2: Detects when an actuator changes direction.
 *
 * Mechanical significance: Each reversal causes backlash take-up and
 * static friction (stiction) breakaway. Tracking reversals helps
 * predict maintenance needs.
 *
 * A reversal is counted when the direction of MV change flips sign
 * and the magnitude exceeds the deadband (to avoid counting noise).
 * ============================================================================
 */

bool dcs_reversal_detect(dcs_reversal_detector_t *det, double current_output) {
    if (!det) return false;

    double diff = current_output - det->prev_direction;

    /* Update prev_direction to track the current output for next call */
    double new_direction;
    if (diff > det->deadband) {
        new_direction = 1.0;
    } else if (diff < -det->deadband) {
        new_direction = -1.0;
    } else {
        /* Within deadband, maintain previous direction */
        return false;
    }

    /* Check if direction actually reversed */
    if (det->prev_direction * new_direction < 0.0) {
        det->reversal_count++;
        det->prev_direction = new_direction;
        return true;
    }

    det->prev_direction = new_direction;
    return false;
}