/*
 * test_core.c - DCS Core Tests
 *
 * Tests for tag database, scan engine, slew limiter,
 * and reversal detector.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "dcs_core.h"

static int passed = 0, failed = 0;
#define T(n) printf("  TEST: %s ... ", n)
#define P() do { printf("PASS\n"); passed++; } while(0)
#define F(m) do { printf("FAIL: %s\n", m); failed++; } while(0)

static void test_tag_db_init(void) {
    T("Tag database initialization");
    dcs_tag_db_t db;
    dcs_tag_db_init(&db);
    assert(db.count == 0);
    assert(!db.scan_active);
    P();
}

static void test_tag_add_find(void) {
    T("Tag add and find by name");
    dcs_tag_db_t db;
    dcs_tag_db_init(&db);

    dcs_tag_t tag;
    memset(&tag, 0, sizeof(tag));
    strncpy(tag.name, "FIC-101", DCS_TAG_NAME_MAX - 1);
    strncpy(tag.description, "Flow Indicator Controller", DCS_DESC_MAX - 1);
    tag.type = BLOCK_PID;
    tag.unit = UNIT_GPM;
    tag.eu_lo = 0.0;
    tag.eu_hi = 100.0;
    tag.pct_lo = 0.0;
    tag.pct_hi = 100.0;
    tag.enabled = true;

    int idx = dcs_tag_add(&db, &tag);
    assert(idx == 0);

    dcs_tag_t *found = dcs_tag_find(&db, "FIC-101");
    assert(found != NULL);
    assert(found->type == BLOCK_PID);

    dcs_tag_t *by_idx = dcs_tag_at(&db, 0);
    assert(by_idx == found);
    P();
}

static void test_tag_eu_pct_conversion(void) {
    T("Engineering unit to percent conversion");
    dcs_tag_t tag;
    memset(&tag, 0, sizeof(tag));
    tag.eu_lo = 0.0;
    tag.eu_hi = 100.0;
    tag.pct_lo = 0.0;
    tag.pct_hi = 100.0;

    double pct;
    bool ok = dcs_tag_eu_to_pct(&tag, 50.0, &pct);
    assert(ok);
    assert(fabs(pct - 50.0) < 1e-9);

    double eu;
    ok = dcs_tag_pct_to_eu(&tag, 75.0, &eu);
    assert(ok);
    assert(fabs(eu - 75.0) < 1e-9);

    /* Reverse with asymmetric scales */
    tag.pct_lo = -5.0;
    tag.pct_hi = 105.0;
    ok = dcs_tag_eu_to_pct(&tag, 0.0, &pct);
    assert(ok);
    assert(fabs(pct - (-5.0)) < 0.01);

    ok = dcs_tag_eu_to_pct(&tag, 100.0, &pct);
    assert(ok);
    assert(fabs(pct - 105.0) < 0.01);

    /* Error case: eu_hi == eu_lo */
    tag.eu_hi = 0.0;
    ok = dcs_tag_eu_to_pct(&tag, 50.0, &pct);
    assert(!ok);
    P();
}

static void test_tag_update_pv(void) {
    T("Tag PV update");
    dcs_tag_t tag;
    memset(&tag, 0, sizeof(tag));
    tag.update_count = 0;

    dcs_tag_update_pv(&tag, 42.5);
    assert(fabs(tag.pv - 42.5) < 1e-9);
    assert(tag.update_count == 1);
    P();
}

static void test_slew_limiter(void) {
    T("Slew rate limiter");
    dcs_slew_limiter_t lim;
    lim.max_rate = 10.0;
    lim.last_output = 0.0;
    lim.dt_seconds = 0.1;

    /* Step of 5: within rate limit (5/0.1 = 50 > 10).
     * Max allowed: 10*0.1 = 1.0, so output = 1.0 */
    double out = dcs_slew_limit(&lim, 5.0, 0.1);
    assert(fabs(out - 1.0) < 0.01);

    /* Step of -0.5: within rate limit */
    out = dcs_slew_limit(&lim, -0.5, 0.1);
    /* Max negative: 1.0 - 10*0.1 = 0.0 */
    assert(fabs(out - 0.0) < 0.01);
    P();
}

static void test_reversal_detector(void) {
    T("Actuator reversal detector");
    dcs_reversal_detector_t det;
    det.reversal_count = 0;
    det.prev_direction = 1.0;
    det.deadband = 0.01;

    /* Moving forward: no reversal */
    bool rev = dcs_reversal_detect(&det, 10.0);
    assert(!rev);
    assert(det.reversal_count == 0);

    /* Moving backward past deadband: reversal */
    rev = dcs_reversal_detect(&det, 0.0);
    assert(rev);
    assert(det.reversal_count == 1);
    P();
}

static void test_scan_engine(void) {
    T("Scan engine basic operation");
    dcs_tag_db_t db;
    dcs_tag_db_init(&db);

    dcs_scan_engine_t engine;
    dcs_scan_engine_init(&engine, &db);

    /* Add a few tags */
    for (int i = 0; i < 5; i++) {
        dcs_tag_t tag;
        memset(&tag, 0, sizeof(tag));
        snprintf(tag.name, DCS_TAG_NAME_MAX, "TAG-%d", i);
        tag.type = BLOCK_INDICATOR;
        tag.scan_rate = SCAN_1S;
        tag.enabled = true;
        dcs_tag_add(&db, &tag);
    }

    uint32_t processed = dcs_scan_engine_run(&engine);
    /* Some tags should have been processed */
    assert(engine.stats.total_scans == 1);
    (void)processed;

    const dcs_scan_stats_t *stats = dcs_scan_engine_stats(&engine);
    assert(stats != NULL);
    P();
}

int main(void) {
    printf("=== DCS Core Tests ===\n\n");
    test_tag_db_init();
    test_tag_add_find();
    test_tag_eu_pct_conversion();
    test_tag_update_pv();
    test_slew_limiter();
    test_reversal_detector();
    test_scan_engine();

    printf("\n=== Core Tests: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}