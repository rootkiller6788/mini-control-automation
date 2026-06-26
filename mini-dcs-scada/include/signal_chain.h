/*
 * signal_chain.h - Signal Processing Chain for DCS/SCADA
 *
 * Implements the complete signal path from field sensor to control algorithm:
 *   Sensor -> ADC -> Filter -> Scale -> Linearize -> Validate -> PV
 *
 * References:
 *   - Oppenheim & Schafer, "Discrete-Time Signal Processing" (2010)
 *   - Smith, "Digital Signal Processing" (2003)
 *   - ISA-67.04.01 Setpoints for Nuclear Safety-Related Instrumentation
 *
 * Course Alignment:
 *   MIT 6.003 - Signals and Systems (Fourier, filtering, sampling)
 *   Berkeley EE123 - Digital Signal Processing (FIR/IIR filter design)
 *   Stanford EE264 - Digital Signal Processing
 *   Michigan EECS 351 - DSP
 */

#ifndef SIGNAL_CHAIN_H
#define SIGNAL_CHAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * L1/L2: Signal Types and ADC Simulation
 * ============================================================================
 */

/** Analog-to-Digital converter model with quantization */
typedef struct {
    double   vref;           /* Reference voltage */
    uint16_t resolution;     /* Bits of resolution (e.g., 12, 16, 24) */
    double   lsb;            /* Least significant bit = Vref / 2^resolution */
    double   offset;         /* DC offset error (volts) */
    double   gain_error;     /* Gain error (ratio, 1.0 = perfect) */
    double   noise_rms;      /* RMS noise (volts) */
} adc_model_t;

void adc_init(adc_model_t *adc, double vref, uint16_t resolution);

/* Convert voltage to ADC counts with quantization */
uint32_t adc_voltage_to_counts(const adc_model_t *adc, double voltage);

/* Convert ADC counts back to voltage */
double adc_counts_to_voltage(const adc_model_t *adc, uint32_t counts);

/* Add simulated noise (Gaussian approximation via Box-Muller) */
double adc_add_noise(const adc_model_t *adc, double voltage);

/* Full ADC conversion: voltage -> noisy quantized -> counts */
uint32_t adc_convert(const adc_model_t *adc, double voltage);

/* ============================================================================
 * L3: Digital Filters - FIR and IIR Implementations
 * ============================================================================
 */

/* --- Exponential Moving Average (1st-order IIR low-pass) --- */

/**
 * EMA: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
 *
 * Corner frequency: fc = alpha / (2*pi*Ts*(1-alpha)) approx for alpha << 1
 * Or: alpha = 2*pi*Ts*fc / (1 + 2*pi*Ts*fc)
 *
 * Equivalent to an RC low-pass filter with RC = Ts*(1-alpha)/alpha
 */
typedef struct {
    double alpha;       /* Smoothing factor [0, 1] */
    double y_prev;      /* Previous output */
    bool   initialized;
} ema_filter_t;

void ema_init(ema_filter_t *f, double alpha);
void ema_init_by_tc(ema_filter_t *f, double time_constant_s, double sample_time_s);
double ema_update(ema_filter_t *f, double x);

/* --- Double EMA (2nd-order, for trend filtering) --- */
typedef struct {
    ema_filter_t ema1;
    ema_filter_t ema2;
    double       alpha;
} dema_filter_t;

void dema_init(dema_filter_t *f, double alpha);
double dema_update(dema_filter_t *f, double x);

/* --- Moving Average Filter (FIR) --- */

#define MA_FILTER_MAX 64

/** Simple moving average (SMA): y[n] = (1/N) * sum_{k=0}^{N-1} x[n-k]
 *  Frequency response: H(f) = (1/N) * sin(pi*f*N*Ts) / sin(pi*f*Ts) */
typedef struct {
    double buffer[MA_FILTER_MAX];
    int    size;
    int    index;
    int    count;
    double sum;
} sma_filter_t;

void sma_init(sma_filter_t *f, int window_size);
double sma_update(sma_filter_t *f, double x);

/* --- Butterworth 2nd-order Low-Pass IIR Filter --- */

/**
 * Digital Butterworth filter via bilinear transform.
 *
 * H(s) = 1 / (s^2 + sqrt(2)*s + 1)  (normalized 2nd-order low-pass)
 *
 * Bilinear prewarp: omega_a = (2/Ts) * tan(omega_d*Ts/2)
 *
 * Difference equation:
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */
typedef struct {
    double b0, b1, b2;   /* Feedforward coefficients */
    double a1, a2;       /* Feedback coefficients */
    double x1, x2;        /* Delayed inputs */
    double y1, y2;        /* Delayed outputs */
} butter2_filter_t;

/**
 * Design a 2nd-order Butterworth low-pass filter.
 * @param fc     Cutoff frequency [Hz]
 * @param fs     Sample frequency [Hz]
 */
void butter2_lp_design(butter2_filter_t *f, double fc, double fs);

double butter2_update(butter2_filter_t *f, double x);

/* --- Notch Filter (2nd-order IIR, for mains hum rejection) --- */

/**
 * Notch filter at frequency fn (e.g., 50/60 Hz power line interference).
 *
 * H(z) = (1 - 2*cos(omega)*z^-1 + z^-2) / (1 - 2*r*cos(omega)*z^-1 + r^2*z^-2)
 *
 * Parameter r controls notch width: r -> 1 gives narrower notch.
 */
typedef struct {
    double r;            /* Pole radius (0.9..0.99) */
    double cos_omega;    /* cos(2*pi*fn/fs) */
    double x1, x2, y1, y2;
} notch_filter_t;

void notch_design(notch_filter_t *f, double fn, double fs, double r);
double notch_update(notch_filter_t *f, double x);

/* ============================================================================
 * L2/L3: Signal Scaling and Linearization
 * ============================================================================
 */

/**
 * Linear scaling: maps [in_lo, in_hi] -> [out_lo, out_hi]
 *
 *   out = (in - in_lo) / (in_hi - in_lo) * (out_hi - out_lo) + out_lo
 *
 * Clamps to output range unless no_clamp is true.
 */
double dcs_scale(double in, double in_lo, double in_hi,
                 double out_lo, double out_hi);

/**
 * Square root extraction (for differential pressure flow meters).
 *
 * Flow = K * sqrt(dP)
 *
 * Implements low-flow cutoff: if dP < cutoff^2, flow = 0.
 * This prevents "negative flow" noise at zero flow.
 *
 * Bernoulli's principle: v = sqrt(2*dP/rho) -> Q = A*v
 */
double dcs_sqrt_extract(double dp, double k_factor, double low_cutoff);

/**
 * Thermocouple linearization: Type K (Chromel-Alumel), -200 to 1372 C.
 *
 * Uses the NIST ITS-90 polynomial:
 *   V = sum_{i=0}^{n} c_i * t^i + a0 * exp(a1*(t - 126.9686)^2)
 *
 * Inverse (voltage to temperature) uses Newton-Raphson iteration.
 */
double thermocouple_k_voltage(double temp_c);
double thermocouple_k_temp(double voltage_mv);

/**
 * RTD (Pt100) resistance to temperature.
 *
 * Callendar-Van Dusen equation:
 *   For T >= 0: R = R0 * (1 + A*T + B*T^2)
 *   For T < 0:  R = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Where: A=3.9083e-3, B=-5.775e-7, C=-4.183e-12, R0=100
 */
double rtd_pt100_temp(double resistance_ohm);

/**
 * 4-20 mA current loop: the standard industrial analog signal.
 *
 * 4 mA = 0% = process low
 * 20 mA = 100% = process high
 *
 * The 4 mA "live zero" allows detection of wire breaks (0 mA = fault).
 */
double ma_to_eu(double ma_current, double eu_lo, double eu_hi);
double eu_to_ma(double eu_value, double eu_lo, double eu_hi);
bool ma_is_valid(double ma_current);  /* Returns false if < 3.5 mA (wire break) */

/* ============================================================================
 * L5: Deadband and Hysteresis
 * ============================================================================
 */

/** Deadband filter: suppresses small signal changes.
 *  If |x - last| < deadband, output = last (no change).
 *  Reduces actuator wear and communication bandwidth. */
typedef struct {
    double deadband;
    double last_value;
    double last_output;
} deadband_filter_t;

void deadband_init(deadband_filter_t *f, double deadband);
double deadband_update(deadband_filter_t *f, double x);

/** Hysteresis (Schmitt trigger): different thresholds for rising/falling.
 *  Commonly used in on/off control (thermostats, level switches). */
typedef struct {
    double on_point;
    double off_point;
    bool   state;
} hysteresis_t;

void hysteresis_init(hysteresis_t *h, double on_point, double off_point, bool initial);
bool hysteresis_update(hysteresis_t *h, double x);

/* ============================================================================
 * L3: Statistical Signal Processing
 * ============================================================================
 */

/** Running statistics (Welford's online algorithm).
 *  Computes mean and variance in a single pass with numerical stability. */
typedef struct {
    uint64_t count;
    double   mean;
    double   m2;     /* Sum of squared differences */
    double   min;
    double   max;
} running_stats_t;

void running_stats_init(running_stats_t *rs);
void running_stats_push(running_stats_t *rs, double x);
double running_stats_mean(const running_stats_t *rs);
double running_stats_variance(const running_stats_t *rs);
double running_stats_stddev(const running_stats_t *rs);

/* ============================================================================
 * L3: Signal Validation
 * ============================================================================
 */

/** Rate-of-change validation: detects sensor faults by checking if
 *  |dx/dt| exceeds physical limits. E.g., a temperature sensor cannot
 *  change more than 10 C/s. */
typedef struct {
    double max_rate;       /* Maximum allowed rate (units/s) */
    double prev_value;
    double prev_time;
} roc_validator_t;

bool roc_validate(roc_validator_t *v, double value, double time_s, double dt);

/** Spike/glitch removal: median filter that rejects single-sample outliers. */
double median3_filter(double x1, double x2, double x3);

/* ============================================================================
 * L5: Interpolation for non-uniformly sampled data
 * ============================================================================
 */

/** Linear interpolation between two points */
double dcs_lerp(double y0, double y1, double t);

/** Bilinear interpolation on a 2D table (e.g., compressor map) */
typedef struct {
    double *data;          /* Row-major 2D data: data[row * cols + col] */
    double *x_axis;        /* X-axis breakpoints */
    double *y_axis;        /* Y-axis breakpoints */
    int     rows;
    int     cols;
} table2d_t;

void table2d_init(table2d_t *t, double *data, double *x_axis, double *y_axis,
                  int rows, int cols);
double table2d_lookup(const table2d_t *t, double x, double y);

/* ============================================================================
 * L4: Shannon-Nyquist Sampling Theorem Verification
 * ============================================================================
 */

/**
 * Determines the minimum sample rate for a given signal bandwidth.
 *
 * Nyquist: fs >= 2 * fmax
 *
 * In practice, fs >= 5*fmax is recommended for industrial control
 * to avoid aliasing from non-ideal anti-aliasing filters.
 *
 * @param fmax       Maximum signal frequency [Hz]
 * @param safety_factor  Safety factor (>= 1.0, typically 5-10)
 * @return           Minimum sample rate [Hz]
 */
double nyquist_min_sample_rate(double fmax, double safety_factor);

#endif /* SIGNAL_CHAIN_H */