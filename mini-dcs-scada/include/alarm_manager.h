/*
 * alarm_manager.h - Industrial Alarm Management System
 *
 * Implements alarm and event management per ISA-18.2 "Management of Alarm
 * Systems for the Process Industries" and EEMUA 191 guidelines.
 *
 * Key concepts:
 *   - Alarm states and lifecycle
 *   - Alarm prioritization (critical/warning/advisory)
 *   - Debounce/hysteresis to prevent nuisance alarms
 *   - Alarm flood suppression
 *   - Shelving and suppression
 *
 * References:
 *   - ISA-18.2-2016 Management of Alarm Systems
 *   - EEMUA 191 Alarm Systems: A Guide to Design, Management, Procurement
 *   - IEC 62682 Alarm Management in Process Industries
 *
 * Course Alignment:
 *   MIT 6.302 - Feedback System Design (alarm rationalization)
 *   Tsinghua - Process Control Systems (alarm management)
 */

#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

/* ============================================================================
 * L1: Alarm Definitions
 * ============================================================================
 */

#define ALARM_NAME_MAX   40
#define ALARM_BUF_MAX    256

/** Alarm priority levels per ISA-18.2 */
typedef enum {
    ALARM_PRIO_DIAGNOSTIC = 0,  /* Maintenance/diagnostic, no operator action */
    ALARM_PRIO_LOW        = 1,  /* Low priority advisory */
    ALARM_PRIO_MEDIUM     = 2,  /* Medium priority warning */
    ALARM_PRIO_HIGH       = 3,  /* High priority alarm */
    ALARM_PRIO_CRITICAL   = 4   /* Critical/Emergency alarm */
} alarm_priority_t;

/** Alarm type enumeration */
typedef enum {
    ALARM_TYPE_HIGH       = 0,  /* PV exceeded high limit */
    ALARM_TYPE_HIGH_HIGH  = 1,  /* PV exceeded high-high limit */
    ALARM_TYPE_LOW        = 2,  /* PV fell below low limit */
    ALARM_TYPE_LOW_LOW    = 3,  /* PV fell below low-low limit */
    ALARM_TYPE_RATE       = 4,  /* Rate of change exceeded limit */
    ALARM_TYPE_DEVIATION  = 5,  /* PV deviated from setpoint by too much */
    ALARM_TYPE_BAD_QUALITY = 6, /* Signal quality degraded */
    ALARM_TYPE_DIGITAL    = 7,  /* Digital/binary state change */
    ALARM_TYPE_SYSTEM     = 8   /* System/communication failure */
} alarm_type_t;

/** Alarm state machine states */
typedef enum {
    ALARM_STATE_NORMAL    = 0,  /* No alarm condition */
    ALARM_STATE_PENDING   = 1,  /* Condition detected, waiting for debounce */
    ALARM_STATE_ACTIVE    = 2,  /* Alarm is active and annunciated */
    ALARM_STATE_ACKED     = 3,  /* Operator acknowledged, condition persists */
    ALARM_STATE_RETURNING = 4,  /* Condition cleared, waiting for debounce */
    ALARM_STATE_SHELVED   = 5,  /* Temporarily suppressed by operator */
    ALARM_STATE_DISABLED  = 6   /* Disabled by maintenance */
} alarm_state_t;

/** A single alarm point */
typedef struct {
    char            name[ALARM_NAME_MAX];   /* Alarm tag name */
    char            description[128];        /* Human description */
    alarm_type_t    type;                    /* Alarm type */
    alarm_priority_t priority;               /* Priority level */
    alarm_state_t   state;                   /* Current state */
    double          limit;                   /* Trip limit value */
    double          limit2;                  /* Second limit (e.g., rate limit) */
    double          deadband;                /* Return deadband */
    double          on_delay_ms;             /* Debounce time when entering alarm */
    double          off_delay_ms;            /* Debounce time when leaving alarm */
    double          trigger_value;           /* Value at trigger */
    uint64_t        trigger_time_ms;         /* Timestamp when triggered */
    uint64_t        ack_time_ms;             /* Timestamp when acknowledged */
    uint64_t        clear_time_ms;           /* Timestamp when cleared */
    uint32_t        occurrence_count;        /* How many times it has triggered */
    bool            requires_ack;            /* Does this alarm need acknowledgment */
    char            operator_msg[128];       /* Operator acknowledgment message */
} alarm_t;

/** Alarm statistics for performance monitoring per ISA-18.2 */
typedef struct {
    uint32_t total_alarms;          /* Total configured alarms */
    uint32_t active_alarms;         /* Currently active */
    uint32_t unacked_alarms;        /* Active but not acknowledged */
    uint32_t shelved_alarms;        /* Currently shelved */
    uint32_t alarms_per_hour;       /* Alarm rate (target < 6 per ISA-18.2) */
    uint32_t peak_alarms_per_10min; /* Flood detection metric */
    uint64_t last_reset_time;
} alarm_stats_t;

/* ============================================================================
 * L2: Alarm Database - manages all alarm points
 * ============================================================================
 */

#define ALARM_DB_MAX 512

/** Alarm database: central registry of all alarm points */
typedef struct {
    alarm_t        alarms[ALARM_DB_MAX];
    uint32_t       count;
    alarm_stats_t  stats;
    bool           flood_suppress;        /* Flood suppression active */
    uint32_t       flood_threshold;        /* Alarms/10min to trigger flood */
} alarm_db_t;

/* ============================================================================
 * L5: Alarm Management API
 * ============================================================================
 */

/** Initialize the alarm database */
void alarm_db_init(alarm_db_t *db);

/**
 * Add a new alarm point to the database.
 * @return Index of new alarm, or -1 if full.
 */
int alarm_add(alarm_db_t *db, const char *name, alarm_type_t type,
              alarm_priority_t priority, double limit, double deadband,
              double on_delay_ms, double off_delay_ms, bool requires_ack);

/** Find alarm by name */
alarm_t *alarm_find(alarm_db_t *db, const char *name);

/** Get alarm by index */
alarm_t *alarm_at(alarm_db_t *db, uint32_t index);

/**
 * Evaluate an alarm: check if the process value triggers or clears the alarm.
 *
 * Implements the full ISA-18.2 alarm state machine with:
 *   Normal -> Pending (on_delay) -> Active -> Acked -> Returning (off_delay) -> Normal
 *
 * @param alarm      Alarm to evaluate
 * @param value      Current process variable value
 * @param timestamp  Current timestamp in ms
 * @return           true if alarm state changed
 */
bool alarm_evaluate(alarm_t *alarm, double value, uint64_t timestamp_ms);

/**
 * Acknowledge an active alarm.
 * Operator confirms they have seen it. Required for EEMUA 191 compliance.
 *
 * @return true on success, false if alarm not in acknowledgeable state
 */
bool alarm_acknowledge(alarm_t *alarm, uint64_t timestamp_ms,
                       const char *operator_msg);

/** Shelve an alarm (temporary suppression, automatic unshelve after timeout) */
bool alarm_shelve(alarm_t *alarm, uint64_t timestamp_ms, uint64_t timeout_ms);

/** Unshelve an alarm */
bool alarm_unshelve(alarm_t *alarm, uint64_t timestamp_ms);

/** Disable an alarm (requires maintenance override) */
bool alarm_disable(alarm_t *alarm);

/** Enable a disabled alarm */
bool alarm_enable(alarm_t *alarm);

/** Get current alarm statistics */
const alarm_stats_t *alarm_get_stats(const alarm_db_t *db);

/** Update alarm statistics (should be called periodically) */
void alarm_update_stats(alarm_db_t *db, uint64_t timestamp_ms);

/* ============================================================================
 * L5: Alarm Flood Detection & Suppression
 * ============================================================================
 */

/**
 * Alarm flood: >10 alarms in 10 minutes per ISA-18.2.
 * During flood, non-critical alarms are suppressed to prevent operator
 * overload. Critical alarms are always annunciated.
 */
bool alarm_flood_detect(alarm_db_t *db, uint64_t timestamp_ms);

/**
 * Check if a specific alarm should be suppressed during flood.
 * Only critical alarms pass through during flood suppression.
 */
bool alarm_is_suppressed(const alarm_db_t *db, const alarm_t *alarm);

/* ============================================================================
 * L6: Event Logging
 * ============================================================================
 */

/** Event types for the audit trail */
typedef enum {
    EVENT_ALARM_TRIGGER  = 0,
    EVENT_ALARM_CLEAR    = 1,
    EVENT_ALARM_ACK      = 2,
    EVENT_ALARM_SHELVE   = 3,
    EVENT_ALARM_UNSHELVE = 4,
    EVENT_OPERATOR_ACTION = 5,
    EVENT_SYSTEM_START   = 6,
    EVENT_SYSTEM_STOP    = 7,
    EVENT_CONFIG_CHANGE  = 8,
    EVENT_COMM_FAIL      = 9
} event_type_t;

#define EVENT_DESC_MAX  256

/** An event record for the sequence of events (SOE) log */
typedef struct {
    event_type_t  type;
    uint64_t      timestamp_ms;
    char          source[ALARM_NAME_MAX];
    char          description[EVENT_DESC_MAX];
    double        value;
    char          operator_id[32];
} event_record_t;

#define EVENT_LOG_MAX 1024

/** Circular buffer event log (SOE recorder) */
typedef struct {
    event_record_t events[EVENT_LOG_MAX];
    uint32_t       head;
    uint32_t       count;
    uint32_t       lost_count;  /* Events lost due to buffer overflow */
} event_log_t;

void event_log_init(event_log_t *log);
void event_log_record(event_log_t *log, event_type_t type,
                      const char *source, const char *description,
                      double value, const char *operator_id);

/* ============================================================================
 * L2: First-Out / Root Cause Detection
 * ============================================================================
 */

/**
 * First-Out Detection: when multiple alarms trigger in rapid succession,
 * identifies which alarm triggered FIRST as the root cause.
 *
 * Common in trip/shutdown sequences: the first alarm to trigger is
 * usually the initiating cause, subsequent alarms are consequences.
 *
 * Resolution: millisecond timestamps (SOE resolution).
 */

typedef struct {
    alarm_t *first_alarm;
    uint64_t first_time_ms;
    uint32_t count_in_window;
    uint64_t window_start_ms;
} first_out_t;

void first_out_init(first_out_t *fo);
void first_out_record(first_out_t *fo, alarm_t *alarm, uint64_t timestamp_ms);
const alarm_t *first_out_get(const first_out_t *fo);
void first_out_reset(first_out_t *fo);

#endif /* ALARM_MANAGER_H */