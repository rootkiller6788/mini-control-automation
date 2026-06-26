#include "plc_core.h"
#include <string.h>

int plc_analyze_timing(const plc_scan_config_t *cfg,
                       plc_timing_metrics_t *metrics)
{
    if (!cfg || !metrics) return -1;
    double T = cfg->max_scan_time_ms;
    if (T < 1e-12) { memset(metrics, 0, sizeof(*metrics)); return -1; }
    double C = cfg->input_phase_budget_ms +
               cfg->exec_phase_budget_ms +
               cfg->output_phase_budget_ms;
    if (C < 1e-12) C = T;
    metrics->utilization = C / T;
    metrics->rms_bound = 1.0; /* n=1 */
    metrics->response_time_ms = C;
    metrics->latency_jitter_ms = cfg->jitter_ms;
    metrics->is_schedulable = (metrics->utilization <= metrics->rms_bound);
    return 0;
}

double plc_min_scan_rate_hz(double f_max_signal_hz, double safety_factor)
{
    if (f_max_signal_hz < 0.0) return 0.0;
    if (safety_factor < 2.0) safety_factor = 2.0;
    return safety_factor * f_max_signal_hz;
}

int plc_check_nyquist(double scan_rate_hz, double f_max_signal_hz)
{
    if (scan_rate_hz < 0.0 || f_max_signal_hz < 0.0) return 0;
    return (scan_rate_hz >= 2.0 * f_max_signal_hz);
}

int plc_rms_schedulable(const double *periods_ms,
                        const double *budgets_ms, size_t n)
{
    if (!periods_ms || !budgets_ms || n == 0) return 0;
    double total_util = 0.0;
    for (size_t i = 0; i < n; i++) {
        if (periods_ms[i] < 1e-12) return 0;
        total_util += budgets_ms[i] / periods_ms[i];
    }
    double bound = (double)n * (pow(2.0, 1.0/(double)n) - 1.0);
    return (total_util <= bound + 1e-9);
}
