/*
 * test_alarm.c - Alarm Manager Tests
 *
 * Tests for alarm state machine, database operations, flood detection,
 * first-out detection, and event logging.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "alarm_manager.h"

static int passed = 0, failed = 0;
#define T(n) printf("  TEST: %s ... ", n)
#define P() do { printf("PASS\n"); passed++; } while(0)
#define F(m) do { printf("FAIL: %s\n", m); failed++; } while(0)

static void test_alarm_db_init(void) {
    T("Alarm database initialization");
    alarm_db_t db;
    alarm_db_init(&db);
    assert(db.count == 0);
    assert(db.stats.total_alarms == 0);
    assert(!db.flood_suppress);
    P();
}

static void test_alarm_add_find(void) {
    T("Add and find alarm by name");
    alarm_db_t db;
    alarm_db_init(&db);

    int idx = alarm_add(&db, "TAH-101", ALARM_TYPE_HIGH,
                         ALARM_PRIO_HIGH, 80.0, 2.0, 500.0, 0.0, true);
    assert(idx == 0);

    alarm_t *a = alarm_find(&db, "TAH-101");
    assert(a != NULL);
    assert(a->type == ALARM_TYPE_HIGH);
    assert(a->priority == ALARM_PRIO_HIGH);
    assert(fabs(a->limit - 80.0) < 1e-9);

    alarm_t *notfound = alarm_find(&db, "DOES_NOT_EXIST");
    assert(notfound == NULL);
    P();
}

static void test_alarm_high_trigger_clear(void) {
    T("High alarm trigger and clear");
    alarm_t alarm;
    memset(&alarm, 0, sizeof(alarm_t));
    strncpy(alarm.name, "TAH-101", ALARM_NAME_MAX - 1);
    alarm.type        = ALARM_TYPE_HIGH;
    alarm.priority    = ALARM_PRIO_HIGH;
    alarm.limit       = 80.0;
    alarm.deadband    = 2.0;
    alarm.on_delay_ms = 0.0; /* No delay for test */
    alarm.off_delay_ms = 0.0;
    alarm.state       = ALARM_STATE_NORMAL;
    alarm.requires_ack = true;

    /* PV below limit: no alarm */
    bool changed = alarm_evaluate(&alarm, 75.0, 1000);
    assert(!changed);
    assert(alarm.state == ALARM_STATE_NORMAL);

    /* PV above limit: alarm triggers */
    changed = alarm_evaluate(&alarm, 85.0, 2000);
    assert(changed);
    assert(alarm.state == ALARM_STATE_ACTIVE);

    /* PV drops below limit but above deadband: alarm holds */
    changed = alarm_evaluate(&alarm, 79.0, 3000);
    assert(!changed);
    assert(alarm.state == ALARM_STATE_ACTIVE);

    /* PV drops below limit - deadband: alarm clears */
    changed = alarm_evaluate(&alarm, 77.5, 4000);
    assert(changed);
    assert(alarm.state == ALARM_STATE_NORMAL);
    P();
}

static void test_alarm_acknowledge(void) {
    T("Alarm acknowledge");
    alarm_t alarm;
    memset(&alarm, 0, sizeof(alarm_t));
    strncpy(alarm.name, "PAL-201", ALARM_NAME_MAX - 1);
    alarm.type        = ALARM_TYPE_LOW;
    alarm.limit       = 10.0;
    alarm.deadband    = 1.0;
    alarm.on_delay_ms = 0.0;
    alarm.state       = ALARM_STATE_ACTIVE;
    alarm.requires_ack = true;

    bool ok = alarm_acknowledge(&alarm, 5000, "Operator John");
    assert(ok);
    assert(alarm.state == ALARM_STATE_ACKED);

    /* Cannot acknowledge again */
    ok = alarm_acknowledge(&alarm, 6000, "Operator John");
    assert(!ok);
    P();
}

static void test_alarm_shelve_unshelve(void) {
    T("Alarm shelve and unshelve");
    alarm_t alarm;
    memset(&alarm, 0, sizeof(alarm_t));
    alarm.state = ALARM_STATE_ACTIVE;

    assert(alarm_shelve(&alarm, 1000, 60000)); /* 60s timeout */
    assert(alarm.state == ALARM_STATE_SHELVED);

    assert(alarm_unshelve(&alarm, 2000));
    assert(alarm.state == ALARM_STATE_NORMAL);
    P();
}

static void test_alarm_disable_enable(void) {
    T("Alarm disable and enable");
    alarm_t alarm;
    memset(&alarm, 0, sizeof(alarm_t));
    alarm.state = ALARM_STATE_NORMAL;

    assert(alarm_disable(&alarm));
    assert(alarm.state == ALARM_STATE_DISABLED);

    assert(alarm_enable(&alarm));
    assert(alarm.state == ALARM_STATE_NORMAL);
    P();
}

static void test_alarm_debounce(void) {
    T("Alarm on-delay debounce");
    alarm_t alarm;
    memset(&alarm, 0, sizeof(alarm_t));
    strncpy(alarm.name, "TAH-102", ALARM_NAME_MAX - 1);
    alarm.type        = ALARM_TYPE_HIGH;
    alarm.limit       = 80.0;
    alarm.deadband    = 2.0;
    alarm.on_delay_ms = 1000.0; /* 1 second debounce */
    alarm.off_delay_ms = 0.0;
    alarm.state       = ALARM_STATE_NORMAL;

    /* First: condition true but delay not expired */
    bool changed = alarm_evaluate(&alarm, 85.0, 0);
    assert(changed);
    assert(alarm.state == ALARM_STATE_PENDING);

    /* 500ms: still pending */
    changed = alarm_evaluate(&alarm, 85.0, 500);
    assert(!changed);
    assert(alarm.state == ALARM_STATE_PENDING);

    /* 1000ms: delay expired, alarm triggers */
    changed = alarm_evaluate(&alarm, 85.0, 1000);
    assert(changed);
    assert(alarm.state == ALARM_STATE_ACTIVE);
    P();
}

static void test_alarm_stats(void) {
    T("Alarm statistics");
    alarm_db_t db;
    alarm_db_init(&db);

    alarm_add(&db, "A1", ALARM_TYPE_HIGH, ALARM_PRIO_HIGH, 80.0, 2.0, 0.0, 0.0, true);
    alarm_add(&db, "A2", ALARM_TYPE_LOW, ALARM_PRIO_MEDIUM, 10.0, 1.0, 0.0, 0.0, true);
    alarm_add(&db, "A3", ALARM_TYPE_HIGH_HIGH, ALARM_PRIO_CRITICAL, 90.0, 2.0, 0.0, 0.0, true);

    /* Trigger A1 and A3 */
    alarm_evaluate(&db.alarms[0], 85.0, 1000);
    alarm_evaluate(&db.alarms[2], 95.0, 1000);

    alarm_update_stats(&db, 1000);
    const alarm_stats_t *s = alarm_get_stats(&db);

    assert(s->active_alarms == 2);
    assert(s->unacked_alarms == 2);
    P();
}

static void test_first_out(void) {
    T("First-out detection");
    first_out_t fo;
    first_out_init(&fo);

    alarm_t a1, a2, a3;
    memset(&a1, 0, sizeof(alarm_t));
    memset(&a2, 0, sizeof(alarm_t));
    memset(&a3, 0, sizeof(alarm_t));
    strncpy(a1.name, "ALARM1", ALARM_NAME_MAX - 1);
    strncpy(a2.name, "ALARM2", ALARM_NAME_MAX - 1);
    strncpy(a3.name, "ALARM3", ALARM_NAME_MAX - 1);

    first_out_record(&fo, &a1, 1000);
    first_out_record(&fo, &a2, 1005);
    first_out_record(&fo, &a3, 1010);

    const alarm_t *first = first_out_get(&fo);
    assert(first == &a1); /* ALARM1 was first */
    P();
}

static void test_event_log(void) {
    T("Event logging (SOE recorder)");
    event_log_t log;
    event_log_init(&log);

    event_log_record(&log, EVENT_ALARM_TRIGGER, "TAH-101",
                     "High temperature alarm", 85.5, "Ops");
    assert(log.count == 1);

    event_log_record(&log, EVENT_OPERATOR_ACTION, "TAH-101",
                     "Alarm acknowledged", 85.5, "John");
    assert(log.count == 2);
    P();
}

int main(void) {
    printf("=== Alarm Manager Tests ===\n\n");
    test_alarm_db_init();
    test_alarm_add_find();
    test_alarm_high_trigger_clear();
    test_alarm_acknowledge();
    test_alarm_shelve_unshelve();
    test_alarm_disable_enable();
    test_alarm_debounce();
    test_alarm_stats();
    test_first_out();
    test_event_log();

    printf("\n=== Alarm Tests: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}