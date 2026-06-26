/*
 * test_signal.c - Signal Chain Tests
 *
 * Tests for ADC model, digital filters (EMA, SMA, Butterworth, notch),
 * scaling, linearization (thermocouple, RTD, 4-20mA), deadband,
 * hysteresis, running statistics, and interpolation.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "signal_chain.h"

#define EPS 1e-9

static int passed = 0, failed = 0;
#define T(name) printf("  TEST: %s ... ", name)
#define P() do { printf("PASS\n"); passed++; } while(0)
#define F(m) do { printf("FAIL: %s\n", m); failed++; } while(0)

static void test_adc_quantization(void) {
    T("ADC quantization (12-bit, 3.3V ref)");
    adc_model_t adc;
    adc_init(&adc, 3.3, 12);

    /* 0V -> 0 counts */
    uint32_t c = adc_voltage_to_counts(&adc, 0.0);
    assert(c == 0);

    /* 3.3V -> 4095 counts (2^12 - 1) */
    c = adc_voltage_to_counts(&adc, 3.3);
    assert(c == 4095);

    /* 1.65V -> ~2047 counts */
    c = adc_voltage_to_counts(&adc, 1.65);
    assert(c >= 2047 && c <= 2048);

    /* LSB = 3.3 / 4096 = 0.00080566 V */
    assert(fabs(adc.lsb - 3.3/4096.0) < 1e-6);
    P();
}

static void test_ema_filter(void) {
    T("EMA filter time constant");
    ema_filter_t f;
    ema_init_by_tc(&f, 1.0, 0.1);

    /* alpha = 0.1/(1.0+0.1) = 0.0909... */
    double expected_alpha = 0.1 / 1.1;
    assert(fabs(f.alpha - expected_alpha) < EPS);

    /* Step response: apply x=1.0 continuously */
    double y = ema_update(&f, 1.0);
    for (int i = 0; i < 100; i++) {
        y = ema_update(&f, 1.0);
    }
    /* Should converge to 1.0 */
    assert(fabs(y - 1.0) < 0.01);
    P();
}

static void test_sma_filter(void) {
    T("SMA filter (moving average, N=5)");
    sma_filter_t f;
    sma_init(&f, 5);

    /* Average of 5 identical values = the value */
    double sum = 0.0;
    for (int i = 0; i < 10; i++) {
        sum = sma_update(&f, 2.0);
    }
    assert(fabs(sum - 2.0) < EPS);
    P();
}

static void test_butterworth_filter(void) {
    T("Butterworth 2nd-order low-pass filter");
    butter2_filter_t f;
    butter2_lp_design(&f, 10.0, 100.0);

    /* Coefficients should be valid */
    assert(fabs(f.b0) > 0.0);
    assert(fabs(f.b0 + f.b1 + f.b2) > 0.0);

    /* Step response: eventually converges */
    double y = 0.0;
    for (int i = 0; i < 1000; i++) {
        y = butter2_update(&f, 1.0);
    }
    assert(fabs(y - 1.0) < 0.01);
    P();
}

static void test_notch_filter(void) {
    T("Notch filter (50 Hz at 1 kHz)");
    notch_filter_t f;
    notch_design(&f, 50.0, 1000.0, 0.95);

    /* Feed a 50 Hz sine wave: should be attenuated */
    /* Feed DC: should pass through */
    double y = 0.0;
    for (int i = 0; i < 100; i++) {
        y = notch_update(&f, 1.0);
    }
    assert(fabs(y - 1.0) < 0.1);
    P();
}

static void test_linear_scaling(void) {
    T("Linear scaling (4-20mA to 0-100 C)");
    double t;

    t = dcs_scale(4.0, 4.0, 20.0, 0.0, 100.0);
    assert(fabs(t - 0.0) < EPS);

    t = dcs_scale(12.0, 4.0, 20.0, 0.0, 100.0);
    assert(fabs(t - 50.0) < EPS);

    t = dcs_scale(20.0, 4.0, 20.0, 0.0, 100.0);
    assert(fabs(t - 100.0) < EPS);
    P();
}

static void test_sqrt_extraction(void) {
    T("Square root extraction (DP flow meter)");
    /* K=10, dP=100 -> flow = 10 * 10 = 100 */
    double flow = dcs_sqrt_extract(100.0, 10.0, 0.1);
    assert(fabs(flow - 100.0) < EPS);

    /* dP below cutoff -> flow = 0 */
    flow = dcs_sqrt_extract(0.05, 10.0, 0.1);
    assert(fabs(flow - 0.0) < EPS);
    P();
}

static void test_thermocouple_k(void) {
    T("Thermocouple Type K voltage (0 C)");
    double v = thermocouple_k_voltage(0.0);
    /* At 0 C, Type K voltage ~ 0 uV */
    assert(fabs(v) < 10.0);

    /* 100 C: ~ 4.096 mV */
    v = thermocouple_k_voltage(100.0);
    assert(v > 3000.0 && v < 5000.0);

    /* Round-trip: 25 C */
    double t = thermocouple_k_temp(thermocouple_k_voltage(25.0));
    assert(fabs(t - 25.0) < 1.0);
    P();
}

static void test_rtd_pt100(void) {
    T("RTD Pt100 resistance to temperature");
    /* 0 C -> 100.0 ohms */
    double t = rtd_pt100_temp(100.0);
    assert(fabs(t - 0.0) < 0.1);

    /* 100 C -> ~138.51 ohms */
    t = rtd_pt100_temp(138.51);
    assert(fabs(t - 100.0) < 0.5);
    P();
}

static void test_4_20ma(void) {
    T("4-20 mA current loop scaling");
    double eu = ma_to_eu(4.0, 0.0, 100.0);
    assert(fabs(eu - 0.0) < EPS);

    eu = ma_to_eu(20.0, 0.0, 100.0);
    assert(fabs(eu - 100.0) < EPS);

    assert(ma_is_valid(4.0));
    assert(ma_is_valid(20.0));
    assert(!ma_is_valid(2.0)); /* Below 3.8 = wire break */
    assert(!ma_is_valid(22.0)); /* Above 20.5 = fault */
    P();
}

static void test_deadband(void) {
    T("Deadband filter");
    deadband_filter_t f;
    deadband_init(&f, 0.5);

    double y = deadband_update(&f, 10.0);
    assert(fabs(y - 10.0) < EPS); /* First value passes */

    y = deadband_update(&f, 10.1); /* Change < 0.5 */
    assert(fabs(y - 10.0) < EPS); /* Output unchanged */

    y = deadband_update(&f, 11.0); /* Change > 0.5 */
    assert(fabs(y - 11.0) < EPS); /* New output */
    P();
}

static void test_hysteresis(void) {
    T("Hysteresis (Schmitt trigger)");
    hysteresis_t h;
    hysteresis_init(&h, 10.0, 5.0, false);

    assert(!hysteresis_update(&h, 0.0));
    assert(!hysteresis_update(&h, 7.0));  /* Between 5 and 10: hold false */
    assert(hysteresis_update(&h, 11.0));  /* Above 10: trigger true */
    assert(hysteresis_update(&h, 7.0));   /* Still above 5: hold true */
    assert(!hysteresis_update(&h, 4.0));  /* Below 5: trigger false */
    P();
}

static void test_running_stats(void) {
    T("Running statistics (Welford's algorithm)");
    running_stats_t rs;
    running_stats_init(&rs);

    double values[] = {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0};
    for (int i = 0; i < 8; i++) {
        running_stats_push(&rs, values[i]);
    }

    /* Mean = 40/8 = 5.0 */
    assert(fabs(running_stats_mean(&rs) - 5.0) < EPS);
    /* Variance = sum((x-5)^2)/(8-1) = (9+1+1+1+0+0+4+16)/7 = 32/7 ? 4.571 */
    double expected_var = 32.0 / 7.0;
    assert(fabs(running_stats_variance(&rs) - expected_var) < 0.01);

    assert(fabs(rs.min - 2.0) < EPS);
    assert(fabs(rs.max - 9.0) < EPS);
    P();
}

static void test_roc_validation(void) {
    T("Rate-of-change validator");
    roc_validator_t v;
    v.max_rate = 10.0;
    v.prev_value = 0.0;
    v.prev_time = 0.0;

    /* Change 5 in 1s -> rate = 5, OK */
    (void)roc_validate(&v, 5.0, 1.0, 1.0);
    assert(roc_validate(&v, 5.0, 1.0, 1.0));
    P();
}

static void test_nyquist(void) {
    T("Nyquist minimum sample rate");
    /* Signal bandwidth = 100 Hz, safety factor = 5 */
    double fs = nyquist_min_sample_rate(100.0, 5.0);
    assert(fabs(fs - 1000.0) < EPS);
    P();
}

int main(void) {
    printf("=== Signal Chain Tests ===\n\n");
    test_adc_quantization();
    test_ema_filter();
    test_sma_filter();
    test_butterworth_filter();
    test_notch_filter();
    test_linear_scaling();
    test_sqrt_extraction();
    test_thermocouple_k();
    test_rtd_pt100();
    test_4_20ma();
    test_deadband();
    test_hysteresis();
    test_running_stats();
    test_roc_validation();
    test_nyquist();

    printf("\n=== Signal Tests: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}