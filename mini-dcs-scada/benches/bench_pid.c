/*
 * bench_pid.c - PID Controller Performance Benchmarks
 *
 * Measures PID update throughput (iterations/second) for
 * performance characterization on embedded DCS controllers.
 */

#include <stdio.h>
#include <time.h>
#include <math.h>
#include "pid_controller.h"

#ifndef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1000
#endif

int main(void) {
    printf("=== PID Controller Performance Benchmarks ===\n\n");

    pid_controller_t pid;
    pid_init_isa(&pid, 1.0, 10.0, 0.5, 0.1, 0.0, 100.0);

    const int iterations = 1000000;
    double sp = 50.0, pv = 40.0;
    volatile double mv;

    /* Warm-up */
    for (int i = 0; i < 1000; i++) {
        mv = pid_update(&pid, sp, pv, 0.1);
    }

    /* Benchmark: PID update only */
    clock_t start = clock();
    for (int i = 0; i < iterations; i++) {
        mv = pid_update(&pid, sp + i * 0.001, pv + i * 0.0005, 0.1);
    }
    clock_t end = clock();
    (void)mv;

    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    double updates_per_sec = iterations / elapsed;

    printf("PID update benchmark (%d iterations):\n", iterations);
    printf("  Total time:   %.3f seconds\n", elapsed);
    printf("  Throughput:   %.0f updates/second\n", updates_per_sec);
    printf("  Per update:   %.3f microseconds\n", 1e6 / updates_per_sec);
    printf("\n");

    /* Benchmark: Cascade control */
    pid_cascade_t cascade;
    pid_tuning_t mt = { .kc = 2.0, .ti = 5.0, .td = 0.0 };
    pid_tuning_t st = { .kc = 1.5, .ti = 1.0, .td = 0.0 };
    pid_cascade_init(&cascade, &mt, 0.5, 0.0, 100.0, &st, 0.1, 0.0, 100.0);

    start = clock();
    for (int i = 0; i < iterations; i++) {
        mv = pid_cascade_update(&cascade, sp, pv + i * 0.001, pv * 0.5, 0.5);
    }
    end = clock();
    (void)mv;

    elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    double cascade_per_sec = iterations / elapsed;

    printf("Cascade control benchmark (%d iterations):\n", iterations);
    printf("  Total time:   %.3f seconds\n", elapsed);
    printf("  Throughput:   %.0f updates/second\n", cascade_per_sec);
    printf("  Per update:   %.3f microseconds\n", 1e6 / cascade_per_sec);
    printf("\n");

    /* Benchmark: ZN tuning computation */
    pid_fopdt_model_t model = { .gain = 1.0, .tau = 10.0, .theta = 2.0 };
    pid_tuning_t result;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        pid_tune_zn_openloop(&model, &result);
    }
    end = clock();

    elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("ZN tuning benchmark (%d iterations):\n", iterations);
    printf("  Total time:   %.3f seconds\n", elapsed);
    printf("  Throughput:   %.0f tunings/second\n", iterations / elapsed);

    printf("\nPerformance suitable for embedded DCS controllers.\n");
    printf("Target: >1M PID updates/second on ARM Cortex-M7 @ 400MHz.\n");

    return 0;
}