/*
 * dcs_core.h - DCS/SCADA Core Definitions
 *
 * Defines the fundamental data types for industrial Distributed Control
 * Systems (DCS) and Supervisory Control and Data Acquisition (SCADA).
 *
 * References:
 *   - ISA-5.1 Instrumentation Symbols and Identification
 *   - ISA-88 Batch Control Standards
 *   - ISA-95 Enterprise-Control System Integration
 *   - IEC 61131-3 Programming Languages for Industrial Control
 *
 * Course Alignment:
 *   MIT 6.302 - Feedback System Design
 *   Stanford EE392 - Digital Control
 *   ETH 227-0216 - Control Systems II
 *   Tsinghua - Process Control Systems
 */

#ifndef DCS_CORE_H
#define DCS_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ============================================================================
 * L1: Core Definitions - DCS/SCADA fundamental data structures
 * ============================================================================
 */

/** Signal quality enumeration per OPC UA Part 8 / IEC 62541 */
typedef enum {
    DCS_QUALITY_GOOD              = 0x00,
    DCS_QUALITY_GOOD_OVERRANGE    = 0x01,
    DCS_QUALITY_GOOD_CASCADE      = 0x02,
    DCS_QUALITY_UNCERTAIN         = 0x40,
    DCS_QUALITY_UNCERTAIN_SUB     = 0x44,
    DCS_QUALITY_BAD               = 0x80,
    DCS_QUALITY_BAD_COMM_FAIL     = 0x84,
    DCS_QUALITY_BAD_SENSOR_FAIL   = 0x88,
    DCS_QUALITY_BAD_CONFIG_ERROR  = 0x8C,
    DCS_QUALITY_INIT              = 0xFF
} dcs_quality_t;

/** Engineering unit type identifiers for process variables */
typedef enum {
    UNIT_NONE = 0,
    UNIT_CELSIUS,
    UNIT_FAHRENHEIT,
    UNIT_KELVIN,
    UNIT_PASCAL,
    UNIT_BAR,
    UNIT_PSI,
    UNIT_ATM,
    UNIT_METERS,
    UNIT_FEET,
    UNIT_MM,
    UNIT_KG_PER_S,
    UNIT_LB_PER_H,
    UNIT_M3_PER_H,
    UNIT_GPM,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_MILLIVOLT,
    UNIT_MILLIAMP,
    UNIT_PH,
    UNIT_PPM,
    UNIT_RPM,
    UNIT_COUNT
} dcs_unit_t;

/** Control block type - ISA-5.1 function block taxonomy */
typedef enum {
    BLOCK_PID          = 0x01,
    BLOCK_PI           = 0x02,
    BLOCK_PD           = 0x03,
    BLOCK_P_ONLY       = 0x04,
    BLOCK_CASCADE_PRI  = 0x10,
    BLOCK_CASCADE_SEC  = 0x11,
    BLOCK_FEEDFORWARD  = 0x20,
    BLOCK_RATIO        = 0x21,
    BLOCK_SPLIT_RANGE  = 0x30,
    BLOCK_SELECTOR     = 0x40,
    BLOCK_INDICATOR    = 0x50,
    BLOCK_TOTALIZER    = 0x60,
    BLOCK_FUNCTION     = 0x70,
    BLOCK_SEQUENCE     = 0x80,
    BLOCK_MOTOR        = 0x90,
    BLOCK_VALVE        = 0x91,
    BLOCK_INTERLOCK    = 0xA0
} dcs_block_type_t;

/** DCS execution cycle (scan period) configuration */
typedef enum {
    SCAN_10MS    = 0,
    SCAN_50MS    = 1,
    SCAN_100MS   = 2,
    SCAN_250MS   = 3,
    SCAN_500MS   = 4,
    SCAN_1S      = 5,
    SCAN_5S      = 6,
    SCAN_10S     = 7,
    SCAN_30S     = 8,
    SCAN_1M      = 9
} dcs_scan_rate_t;

/* ============================================================================
 * L1: DCS Tag - the fundamental addressable entity in a DCS/SCADA system
 * ============================================================================
 */

#define DCS_TAG_NAME_MAX   32
#define DCS_DESC_MAX      128

/** A DCS Tag is the atomic data point in the control system database.
 *  Every sensor, actuator, setpoint, and computed value is a Tag.
 *  This is the ISA-88 / ISA-95 Control Recipe Parameter primitive.
 */
typedef struct {
    char            name[DCS_TAG_NAME_MAX];
    char            description[DCS_DESC_MAX];
    dcs_block_type_t type;
    dcs_scan_rate_t  scan_rate;
    dcs_unit_t      unit;
    double          pv;
    double          sp;
    double          mv;
    double          eu_lo;
    double          eu_hi;
    double          pct_lo;
    double          pct_hi;
    dcs_quality_t   quality;
    uint64_t        timestamp_ms;
    uint32_t        update_count;
    bool            enabled;
    bool            triggered;
    double          trigger_value;
    uint64_t        trigger_time;
} dcs_tag_t;

/* ============================================================================
 * L1: Tag Database - the real-time process image
 * ============================================================================
 */

#define DCS_MAX_TAGS  1024

/** Tag database: the in-memory real-time process image of the DCS.
 *  Central data structure that holds all scannable points.
 */
typedef struct {
    dcs_tag_t  tags[DCS_MAX_TAGS];
    uint32_t   count;
    uint64_t   scan_counter;
    uint64_t   last_scan_time_ms;
    bool       scan_active;
} dcs_tag_db_t;

/* ============================================================================
 * API: Tag Database Operations
 * ============================================================================
 */

void dcs_tag_db_init(dcs_tag_db_t *db);

int dcs_tag_add(dcs_tag_db_t *db, const dcs_tag_t *tag);

dcs_tag_t *dcs_tag_find(dcs_tag_db_t *db, const char *name);

dcs_tag_t *dcs_tag_at(dcs_tag_db_t *db, uint32_t index);

void dcs_tag_update_pv(dcs_tag_t *tag, double raw_value);

bool dcs_tag_eu_to_pct(const dcs_tag_t *tag, double eu_value, double *pct_out);
bool dcs_tag_pct_to_eu(const dcs_tag_t *tag, double pct_value, double *eu_out);

/* ============================================================================
 * L2: Scan Engine - cyclic execution of control blocks
 * ============================================================================
 */

/** Scan statistics for diagnostics and tuning */
typedef struct {
    uint64_t total_scans;
    uint64_t overrun_count;
    double   min_scan_time_ms;
    double   max_scan_time_ms;
    double   avg_scan_time_ms;
    double   cpu_load_percent;
} dcs_scan_stats_t;

typedef struct {
    dcs_tag_db_t   *db;
    dcs_scan_stats_t stats;
    uint64_t        cycle_start_ms;
    uint64_t        cycle_deadline_ms;
    bool            running;
} dcs_scan_engine_t;

void dcs_scan_engine_init(dcs_scan_engine_t *engine, dcs_tag_db_t *db);

uint32_t dcs_scan_engine_run(dcs_scan_engine_t *engine);

const dcs_scan_stats_t *dcs_scan_engine_stats(const dcs_scan_engine_t *engine);

/* ============================================================================
 * L2: Controller Output - actuator command shaping
 * ============================================================================
 */

typedef struct {
    double max_rate;
    double last_output;
    double dt_seconds;
} dcs_slew_limiter_t;

double dcs_slew_limit(dcs_slew_limiter_t *lim, double desired, double dt);

typedef struct {
    uint32_t reversal_count;
    double   prev_direction;
    double   deadband;
} dcs_reversal_detector_t;

bool dcs_reversal_detect(dcs_reversal_detector_t *det, double current_output);

#endif /* DCS_CORE_H */