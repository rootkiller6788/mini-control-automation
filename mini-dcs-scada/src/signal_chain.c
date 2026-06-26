/*
 * signal_chain.c - Signal Processing Chain for DCS/SCADA
 *
 * Implements the full signal path from field sensor to validated PV:
 *   ADC modeling -> digital filtering -> scaling/linearization -> validation
 *
 * Includes Butterworth, EMA, moving average, notch filters; thermocouple
 * and RTD linearization; 4-20mA scaling; deadband and hysteresis.
 */

#include "signal_chain.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================================
 * ADC Model
 *
 * L1/L2: Analog-to-Digital converter simulates quantization effects.
 *
 * Quantization: The mapping from continuous voltage to discrete counts
 * introduces quantization error bounded by +/- 0.5 LSB.
 *
 * ENOB (Effective Number of Bits):
 *   ENOB = (SINAD_dB - 1.76) / 6.02
 *   where SINAD is Signal-to-Noise-And-Distortion ratio.
 *
 * Quantization noise power: sigma_q^2 = LSB^2 / 12
 * (derived from uniform distribution of quantization error).
 * ============================================================================
 */

void adc_init(adc_model_t *adc, double vref, uint16_t resolution) {
    if (!adc) return;
    memset(adc, 0, sizeof(adc_model_t));
    adc->vref       = vref;
    adc->resolution = resolution;
    adc->lsb        = vref / ((double)(1U << resolution));
    adc->offset     = 0.0;
    adc->gain_error = 1.0;
    adc->noise_rms  = 0.0;
}

uint32_t adc_voltage_to_counts(const adc_model_t *adc, double voltage) {
    if (!adc) return 0;
    double max_counts = (double)((1U << adc->resolution) - 1);
    double raw = (voltage / adc->vref) * max_counts;
    if (raw < 0.0) raw = 0.0;
    if (raw > max_counts) raw = max_counts;
    return (uint32_t)(raw + 0.5); /* Round to nearest integer */
}

double adc_counts_to_voltage(const adc_model_t *adc, uint32_t counts) {
    if (!adc) return 0.0;
    double max_counts = (double)((1U << adc->resolution) - 1);
    return (double)counts * adc->vref / max_counts;
}

/* Box-Muller transform: generate approximately Gaussian noise */
static double box_muller(void) {
    /* Use a simple approximation: sum of 12 uniform randoms
     * gives mean=0, variance=1 (Central Limit Theorem).
     * This avoids transcendental function calls for speed. */
    double sum = 0.0;
    for (int i = 0; i < 12; i++) {
        sum += ((double)rand() / (double)RAND_MAX);
    }
    return sum - 6.0;
}

double adc_add_noise(const adc_model_t *adc, double voltage) {
    if (!adc || adc->noise_rms <= 0.0) return voltage;
    return voltage + adc->noise_rms * box_muller();
}

uint32_t adc_convert(const adc_model_t *adc, double voltage) {
    if (!adc) return 0;
    double noisy = adc_add_noise(adc, voltage);
    double calibrated = adc->gain_error * noisy + adc->offset;
    return adc_voltage_to_counts(adc, calibrated);
}

/* ============================================================================
 * Exponential Moving Average (EMA) Filter
 *
 * L3: First-order IIR low-pass filter.
 *
 * Difference equation: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
 *
 * Transfer function: H(z) = alpha / (1 - (1-alpha)*z^-1)
 *
 * Frequency response (magnitude):
 *   |H(e^jw)| = alpha / sqrt(1 + (1-alpha)^2 - 2*(1-alpha)*cos(w))
 *
 * Phase delay at DC: tau = Ts*(1-alpha)/alpha
 *
 * Step response: y[n] = 1 - (1-alpha)^n (converges in ~4*tau/Ts samples)
 *
 * Applications:
 *   - Sensor noise filtering in PLC analog input cards
 *   - Process variable smoothing for display
 *   - Feedforward signal conditioning
 * ============================================================================
 */

void ema_init(ema_filter_t *f, double alpha) {
    if (!f) return;
    memset(f, 0, sizeof(ema_filter_t));
    f->alpha = (alpha < 0.0) ? 0.0 : (alpha > 1.0) ? 1.0 : alpha;
}

void ema_init_by_tc(ema_filter_t *f, double time_constant_s, double sample_time_s) {
    if (!f) return;
    /* alpha = dt / (RC + dt) = Ts / (tau + Ts) */
    if (sample_time_s <= 0.0 || time_constant_s < 0.0) {
        memset(f, 0, sizeof(ema_filter_t));
        return;
    }
    double alpha = sample_time_s / (time_constant_s + sample_time_s);
    ema_init(f, alpha);
}

double ema_update(ema_filter_t *f, double x) {
    if (!f) return x;
    if (!f->initialized) {
        f->y_prev = x;
        f->initialized = true;
        return x;
    }
    f->y_prev = f->alpha * x + (1.0 - f->alpha) * f->y_prev;
    return f->y_prev;
}

/* ============================================================================
 * Double EMA (DEMA)
 *
 * L5: Two cascaded EMA filters provide 2nd-order filtering.
 *
 * DEMA = 2*EMA1 - EMA2  (where EMA2 = EMA(EMA1))
 *
 * Properties:
 *   - Reduces lag compared to a single EMA with the same smoothing
 *   - 2nd-order rolloff (-12 dB/octave)
 *   - Approximates a 2nd-order critically damped low-pass
 *
 * Used in financial time-series analysis (MACD) and can be
 * adapted for process trend filtering in DCS.
 * ============================================================================
 */

void dema_init(dema_filter_t *f, double alpha) {
    if (!f) return;
    memset(f, 0, sizeof(dema_filter_t));
    f->alpha = alpha;
    ema_init(&f->ema1, alpha);
    ema_init(&f->ema2, alpha);
}

double dema_update(dema_filter_t *f, double x) {
    if (!f) return x;
    double e1 = ema_update(&f->ema1, x);
    double e2 = ema_update(&f->ema2, e1);
    return 2.0 * e1 - e2;
}

/* ============================================================================
 * Simple Moving Average (FIR)
 *
 * L3: Finite Impulse Response filter of length N.
 *
 * y[n] = (1/N) * sum_{k=0}^{N-1} x[n-k]
 *
 * Frequency response: H(e^jw) = (1/N) * sin(wN/2) / sin(w/2) * e^{-jw(N-1)/2}
 *
 * Nulls at: f = k*fs/N for k = 1, 2, ..., N-1
 *
 * This is the optimal filter for reducing white noise while
 * preserving a DC signal (maximum SNR improvement = sqrt(N)).
 *
 * Applications:
 *   - Filtering 50/60 Hz mains hum (choose N so fs/N = 50Hz or 60Hz)
 *   - Reducing ADC quantization noise
 *   - Flow meter pulse counting integration
 *
 * Implementation: Maintains running sum for O(1) updates.
 * ============================================================================
 */

void sma_init(sma_filter_t *f, int window_size) {
    if (!f) return;
    memset(f, 0, sizeof(sma_filter_t));
    if (window_size < 1) window_size = 1;
    if (window_size > MA_FILTER_MAX) window_size = MA_FILTER_MAX;
    f->size = window_size;
}

double sma_update(sma_filter_t *f, double x) {
    if (!f) return x;
    if (f->size == 0) f->size = 1;

    /* Remove oldest sample from running sum */
    if (f->count >= f->size) {
        f->sum -= f->buffer[f->index];
    }

    /* Add new sample */
    f->buffer[f->index] = x;
    f->sum += x;

    f->index = (f->index + 1) % f->size;
    if (f->count < f->size) f->count++;

    return f->sum / (double)f->count;
}

/* ============================================================================
 * Butterworth 2nd-Order Low-Pass Filter
 *
 * L5: The Butterworth filter has maximally flat passband response.
 *
 * Normalized transfer function (2nd order):
 *   H(s) = 1 / (s^2 + sqrt(2)*s + 1)
 *
 * Magnitude: |H(jw)| = 1 / sqrt(1 + w^{2N}) for order N.
 * At w = 1 (cutoff): |H| = 1/sqrt(2) = -3 dB.
 *
 * Bilinear transform (Tustin's method):
 *   s = (2/Ts) * (z-1)/(z+1)
 *
 * Prewarping ensures correct cutoff after bilinear mapping:
 *   w_a = (2/Ts) * tan(w_d*Ts/2)
 *
 * Coefficient formulas (derived via bilinear transform of H(s)):
 *   Let c = tan(pi*fc/fs)
 *   b0 = c^2 / (1 + sqrt(2)*c + c^2)
 *   b1 = 2*b0
 *   b2 = b0
 *   a1 = 2*(c^2 - 1) / (1 + sqrt(2)*c + c^2)
 *   a2 = (1 - sqrt(2)*c + c^2) / (1 + sqrt(2)*c + c^2)
 *
 * Difference equation:
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * This filter is essential for anti-aliasing before downsampling
 * and for removing sensor noise in critical measurements.
 * ============================================================================
 */

void butter2_lp_design(butter2_filter_t *f, double fc, double fs) {
    if (!f) return;
    memset(f, 0, sizeof(butter2_filter_t));

    if (fc <= 0.0 || fs <= 0.0 || fc >= fs/2.0) {
        /* Pass-through: b0 = 1 */
        f->b0 = 1.0;
        f->b1 = 0.0;
        f->b2 = 0.0;
        f->a1 = 0.0;
        f->a2 = 0.0;
        return;
    }

    /* Prewarped cutoff */
    double omega = tan(M_PI * fc / fs);
    double omega2 = omega * omega;
    double sqrt2 = 1.4142135623730951;

    double denom = omega2 + sqrt2 * omega + 1.0;

    f->b0 = omega2 / denom;
    f->b1 = 2.0 * omega2 / denom;
    f->b2 = omega2 / denom;
    f->a1 = 2.0 * (omega2 - 1.0) / denom;
    f->a2 = (omega2 - sqrt2 * omega + 1.0) / denom;
}

double butter2_update(butter2_filter_t *f, double x) {
    if (!f) return x;

    /* Direct Form I difference equation */
    double y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
               - f->a1 * f->y1 - f->a2 * f->y2;

    /* Shift delay lines */
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

/* ============================================================================
 * Notch Filter
 *
 * L5: Removes a specific frequency (e.g., 50/60 Hz power line interference).
 *
 * Transfer function:
 *   H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)
 *
 * where w0 = 2*pi*fn/fs (notch frequency in rad/sample),
 * r is the pole radius (closer to 1 = narrower notch, slower response).
 *
 * Zeros are exactly on the unit circle at angle w0.
 * Poles are slightly inside the unit circle at radius r.
 *
 * The notch depth is theoretically infinite at fn (zeros cancel),
 * but limited by coefficient quantization in fixed-point implementations.
 *
 * Recommended r values:
 *   - r = 0.90: wide notch, fast response, for rapidly changing noise
 *   - r = 0.95: moderate width, typical choice
 *   - r = 0.99: very narrow notch, for stable mains frequency
 * ============================================================================
 */

void notch_design(notch_filter_t *f, double fn, double fs, double r) {
    if (!f) return;
    memset(f, 0, sizeof(notch_filter_t));

    if (fn <= 0.0 || fs <= 0.0 || r <= 0.0 || r >= 1.0) {
        f->r = 0.95;
        f->cos_omega = cos(2.0 * M_PI * 50.0 / 1000.0); /* Default 50Hz at 1kHz */
        return;
    }

    f->r = r;
    f->cos_omega = cos(2.0 * M_PI * fn / fs);
}

double notch_update(notch_filter_t *f, double x) {
    if (!f) return x;

    double y = x - 2.0 * f->cos_omega * f->x1 + f->x2
               + 2.0 * f->r * f->cos_omega * f->y1 - f->r * f->r * f->y2;

    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

/* ============================================================================
 * Linear Scaling
 *
 * L1: The most fundamental signal conditioning operation.
 *
 * Maps input range [in_lo, in_hi] to output range [out_lo, out_hi].
 *
 * out = out_lo + (in - in_lo) * (out_hi - out_lo) / (in_hi - in_lo)
 *
 * Used everywhere: from 4-20mA to engineering units, ADC counts to
 * voltage, percent to valve position.
 * ============================================================================
 */

double dcs_scale(double in, double in_lo, double in_hi,
                 double out_lo, double out_hi) {
    double in_range = in_hi - in_lo;
    if (fabs(in_range) < 1e-15) return out_lo;
    double out_range = out_hi - out_lo;
    return out_lo + (in - in_lo) * out_range / in_range;
}

/* ============================================================================
 * Square Root Extraction
 *
 * L2: Differential pressure (DP) flow meters measure dP across an
 *     orifice plate. Flow rate Q is proportional to sqrt(dP).
 *
 * Bernoulli: p1 + 1/2*rho*v1^2 = p2 + 1/2*rho*v2^2
 * Continuity: A1*v1 = A2*v2
 * => v2 = sqrt(2*(p1-p2) / (rho*(1 - (A2/A1)^2)))
 * => Q = A2 * v2 = K * sqrt(dP)
 *
 * Low-flow cutoff: For dP < cutoff, return 0. This prevents
 * the sqrt function from producing non-zero flow when the
 * differential pressure transmitter reads near-zero noise.
 *
 * The cutoff is typically 0.5-1% of full-scale dP.
 * ============================================================================
 */

double dcs_sqrt_extract(double dp, double k_factor, double low_cutoff) {
    if (dp <= low_cutoff) return 0.0;
    return k_factor * sqrt(dp);
}

/* ============================================================================
 * Thermocouple Type K Linearization
 *
 * L5: NIST ITS-90 polynomial for Type K (Chromel-Alumel).
 *
 * Type K is the most common industrial thermocouple:
 *   - Range: -200 C to +1372 C
 *   - Sensitivity: ~41 uV/C at 25 C
 *   - Accuracy: +/- 2.2 C or 0.75% (standard grade)
 *
 * The voltage-to-temperature relationship is non-linear.
 * Two polynomial ranges are used for accuracy:
 *   Range 1: -200 C to 0 C:  9th-order polynomial
 *   Range 2: 0 C to 500 C:   9th-order polynomial
 *   Range 3: 500 C to 1372 C: same form with different coefficients
 *
 * Plus an exponential term for the 0-1372 C range.
 *
 * For simplicity, we implement the 0-1372 C range (most common):
 *   V = sum_{i=0}^{8} c_i * T^i + a0 * exp(a1 * (T - 126.9686)^2)
 *
 * where V is in microvolts and T is in degrees Celsius.
 * ============================================================================
 */

/* Coefficients for Type K, range 0 to 1372 C (in uV vs C) */
static const double k_coeff[10] = {
    -0.176004136860e-1,
     0.389212049750e-1,
     0.185587700320e-4,
    -0.994575928740e-7,
     0.318409457190e-9,
    -0.560728448890e-12,
     0.560750590590e-15,
    -0.320207200030e-18,
     0.971511471520e-22,
    -0.121047212750e-25
};
static const double k_a0 = 0.118597600000e0;
static const double k_a1 = -0.118343200000e-3;
static const double k_t0 = 126.9686;

double thermocouple_k_voltage(double temp_c) {
    if (temp_c < -200.0) temp_c = -200.0;
    if (temp_c > 1372.0) temp_c = 1372.0;

    double v = 0.0;
    double t_pow = 1.0;
    for (int i = 0; i < 10; i++) {
        v += k_coeff[i] * t_pow;
        t_pow *= temp_c;
    }

    /* Exponential term for 0-1372 C range */
    v += k_a0 * exp(k_a1 * (temp_c - k_t0) * (temp_c - k_t0));

    return v * 1000.0; /* Convert mV to uV */
}

/* Newton-Raphson for inverse: V(uV) -> T(C) */
double thermocouple_k_temp(double voltage_uv) {
    double v_mv = voltage_uv / 1000.0;
    double t = 25.0; /* Initial guess: room temperature */
    double tolerance = 0.001;

    for (int iter = 0; iter < 20; iter++) {
        double t_pow = 1.0;
        double v = 0.0;
        double dvdt = 0.0;
        for (int i = 0; i < 10; i++) {
            v += k_coeff[i] * t_pow;
            if (i > 0) dvdt += i * k_coeff[i] * t_pow / t;
            t_pow *= t;
        }

        double exp_part = k_a0 * exp(k_a1 * (t - k_t0) * (t - k_t0));
        v += exp_part;
        dvdt += exp_part * 2.0 * k_a1 * (t - k_t0);

        double f = v - v_mv;
        double df = dvdt;

        if (fabs(f) < tolerance) return t;

        if (fabs(df) > 1e-15) {
            t = t - f / df;
        } else {
            break;
        }

        if (t < -200.0) t = -200.0;
        if (t > 1372.0) t = 1372.0;
    }

    return t;
}

/* ============================================================================
 * RTD Pt100 Linearization
 *
 * L5: Platinum Resistance Thermometer, the most accurate industrial
 *     temperature sensor.
 *
 * Callendar-Van Dusen equation (IEC 60751):
 *
 * For T >= 0 C:
 *   R(T) = R0 * (1 + A*T + B*T^2)
 *
 * For T < 0 C:
 *   R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Constants for standard Pt100 (alpha = 0.00385):
 *   R0 = 100 ohms at 0 C
 *   A  = 3.9083e-3 / C
 *   B  = -5.775e-7 / C^2
 *   C  = -4.183e-12 / C^4 (only for T < 0)
 *
 * Given a measured resistance, solve for temperature.
 * For T >= 0, this is a quadratic in T: B*T^2 + A*T + (1 - R/R0) = 0
 * For T < 0, this is a 4th-order equation (solve iteratively).
 * ============================================================================
 */

#define PT100_R0  100.0
#define PT100_A   3.9083e-3
#define PT100_B  -5.775e-7
#define PT100_C  -4.183e-12

double rtd_pt100_temp(double resistance_ohm) {
    double ratio = resistance_ohm / PT100_R0;

    /* Try positive temperature solution first:
     * B*T^2 + A*T + (1 - ratio) = 0
     * T = (-A + sqrt(A^2 - 4*B*(1-ratio))) / (2*B)  [since B < 0, positive root] */
    double discriminant = PT100_A * PT100_A - 4.0 * PT100_B * (1.0 - ratio);

    if (discriminant >= 0.0) {
        /* Positive root */
        double t_pos = (-PT100_A + sqrt(discriminant)) / (2.0 * PT100_B);
        if (t_pos >= 0.0) return t_pos;
    }

    /* Negative temperature: need to solve the 4th-order CVD equation.
     * Use Newton-Raphson starting from negative discriminant case. */
    double t = -50.0; /* Initial guess */
    for (int i = 0; i < 30; i++) {
        double r_calc;
        double drdt;

        if (t >= 0.0) {
            r_calc = PT100_R0 * (1.0 + PT100_A * t + PT100_B * t * t);
            drdt   = PT100_R0 * (PT100_A + 2.0 * PT100_B * t);
        } else {
            double t3 = t * t * t;
            r_calc = PT100_R0 * (1.0 + PT100_A * t + PT100_B * t * t
                      + PT100_C * (t - 100.0) * t3);
            drdt   = PT100_R0 * (PT100_A + 2.0 * PT100_B * t
                      + PT100_C * (4.0 * t3 - 300.0 * t * t));
        }

        double error = r_calc - resistance_ohm;
        if (fabs(error) < 0.001) return t;

        if (fabs(drdt) > 1e-15) {
            t = t - error / drdt;
        }
    }

    return t;
}

/* ============================================================================
 * 4-20 mA Current Loop
 *
 * L1: The universal industrial analog signal standard.
 *
 * Advantages of 4-20 mA:
 *   1. Live zero: 4 mA = 0%, so 0 mA = wire break (fault detection)
 *   2. Current signal is immune to voltage drops in long cables
 *   3. Two-wire operation: power and signal on same pair
 *   4. Intrinsic safety: limited energy for hazardous areas
 *
 * NAMUR NE43 recommendation:
 *   3.8-20.5 mA = valid measurement
 *   < 3.6 mA or > 21.0 mA = fault
 * ============================================================================
 */

double ma_to_eu(double ma_current, double eu_lo, double eu_hi) {
    return dcs_scale(ma_current, 4.0, 20.0, eu_lo, eu_hi);
}

double eu_to_ma(double eu_value, double eu_lo, double eu_hi) {
    return dcs_scale(eu_value, eu_lo, eu_hi, 4.0, 20.0);
}

bool ma_is_valid(double ma_current) {
    /* NAMUR NE43: valid range is 3.8 to 20.5 mA */
    return (ma_current >= 3.8 && ma_current <= 20.5);
}

/* ============================================================================
 * Deadband Filter
 *
 * L5: Suppresses small changes to reduce actuator wear and network traffic.
 *
 * If |x - last_output| < deadband, output remains unchanged.
 * This is a form of non-linear filtering.
 *
 * Typical deadbands (as % of span):
 *   - Control valves: 0.5-1.0%
 *   - Analog inputs: 0.1-0.5%
 *   - SCADA telemetry: 0.5-2.0% (to reduce communication cost)
 *
 * The deadband introduces hysteresis, which can cause limit cycles
 * in feedback loops if too large. Describing function analysis (L4)
 * can predict the amplitude and frequency of these limit cycles.
 * ============================================================================
 */

void deadband_init(deadband_filter_t *f, double deadband) {
    if (!f) return;
    memset(f, 0, sizeof(deadband_filter_t));
    f->deadband = deadband;
}

double deadband_update(deadband_filter_t *f, double x) {
    if (!f) return x;
    if (fabs(x - f->last_value) > f->deadband) {
        f->last_output = x;
        f->last_value  = x;
    }
    /* Note: last_value always tracks input; last_output is filtered */
    f->last_value = x;
    return f->last_output;
}

/* ============================================================================
 * Hysteresis (Schmitt Trigger)
 *
 * L2: Two-threshold comparator used in on/off control.
 *
 * When input rises above "on_point", output = true.
 * When input falls below "off_point", output = false.
 * Between on_point and off_point, output maintains previous state.
 *
 * This prevents rapid cycling (chatter) that would occur with a
 * single threshold, protecting contactors and compressors.
 *
 * Mathematical description: This is a relay with hysteresis,
 * a common nonlinearity analyzed via describing functions (L4)
 * and Popov criterion for stability (L4).
 * ============================================================================
 */

void hysteresis_init(hysteresis_t *h, double on_point, double off_point, bool initial) {
    if (!h) return;
    memset(h, 0, sizeof(hysteresis_t));
    h->on_point  = on_point;
    h->off_point = off_point;
    h->state     = initial;
}

bool hysteresis_update(hysteresis_t *h, double x) {
    if (!h) return false;
    if (x >= h->on_point) {
        h->state = true;
    } else if (x <= h->off_point) {
        h->state = false;
    }
    /* Between off_point and on_point: hold state */
    return h->state;
}

/* ============================================================================
 * Running Statistics (Welford's Algorithm)
 *
 * L3: Online computation of mean and variance without storing all data.
 *
 * Welford (1962) algorithm:
 *   delta = x - mean
 *   mean  = mean + delta / count
 *   M2    = M2 + delta * (x - mean)
 *   variance = M2 / count       (population) or M2 / (count-1) (sample)
 *
 * This algorithm is numerically stable (unlike the naive two-pass
 * method) and requires O(1) memory and O(1) time per sample.
 *
 * Used in DCS for:
 *   - Online sensor calibration drift detection
 *   - Control performance monitoring (minimum variance benchmarking)
 *   - Statistical process control (SPC) in manufacturing
 * ============================================================================
 */

void running_stats_init(running_stats_t *rs) {
    if (!rs) return;
    memset(rs, 0, sizeof(running_stats_t));
    rs->min = INFINITY;
    rs->max = -INFINITY;
}

void running_stats_push(running_stats_t *rs, double x) {
    if (!rs) return;
    rs->count++;

    /* Welford's algorithm for mean and variance */
    double delta = x - rs->mean;
    rs->mean += delta / (double)rs->count;
    rs->m2   += delta * (x - rs->mean);

    if (x < rs->min) rs->min = x;
    if (x > rs->max) rs->max = x;
}

double running_stats_mean(const running_stats_t *rs) {
    if (!rs || rs->count == 0) return 0.0;
    return rs->mean;
}

double running_stats_variance(const running_stats_t *rs) {
    if (!rs || rs->count < 2) return 0.0;
    return rs->m2 / (double)(rs->count - 1); /* Sample variance */
}

double running_stats_stddev(const running_stats_t *rs) {
    return sqrt(running_stats_variance(rs));
}

/* ============================================================================
 * Rate-of-Change Validation
 *
 * L3: Detects sensor faults by physically impossible changes.
 *
 * If |dx/dt| > max_rate, the sensor is likely faulty or the
 * signal line is compromised.
 *
 * Typical max rates:
 *   - Temperature: 10 C/s (thermal mass limits change speed)
 *   - Pressure: 100 bar/s
 *   - Level: vessel geometry-dependent
 *   - Flow: can change rapidly (limited only by pipe dynamics)
 *
 * This is a simple form of model-based fault detection (L8).
 * ============================================================================
 */

bool roc_validate(roc_validator_t *v, double value, double time_s, double dt) {
    if (!v) return true;
    (void)time_s; /* Time passed in, dt already captures interval */

    if (dt > 0.0 && v->prev_time > 0.0) {
        double rate = fabs(value - v->prev_value) / dt;
        if (rate > v->max_rate) {
            return false; /* ROC violation */
        }
    }

    return true;
}

/* ============================================================================
 * Median-3 Filter
 *
 * L5: Simple nonlinear filter that removes single-sample spikes.
 *
 * Takes three consecutive samples and returns the median.
 * This eliminates isolated outliers while preserving edges.
 *
 * For larger windows, median filtering provides excellent
 * salt-and-pepper noise removal but blurs edges.
 *
 * Efficiently implemented as a sorting network for N=3:
 *   Comparisons: 3 (optimal)
 * ============================================================================
 */

double median3_filter(double x1, double x2, double x3) {
    /* Sorting network for 3 elements: median is the middle one */
    if (x1 > x2) { double t = x1; x1 = x2; x2 = t; }
    if (x2 > x3) { double t = x2; x2 = x3; x3 = t; }
    if (x1 > x2) { double t = x1; x1 = x2; x2 = t; }
    return x2; /* x2 is the median */
}

/* ============================================================================
 * Linear Interpolation
 * ============================================================================
 */

double dcs_lerp(double y0, double y1, double t) {
    return y0 + t * (y1 - y0);
}

/* ============================================================================
 * 2D Table Lookup with Bilinear Interpolation
 *
 * L5: Essential for nonlinear sensor characterization and
 *     compressor/turbine performance maps.
 *
 * Bilinear interpolation on a rectangular grid:
 *   f(x,y) = (1-tx)*(1-ty)*f00 + tx*(1-ty)*f10 + (1-tx)*ty*f01 + tx*ty*f11
 *
 * where tx, ty are normalized coordinates within the cell.
 * ============================================================================
 */

void table2d_init(table2d_t *t, double *data, double *x_axis, double *y_axis,
                  int rows, int cols) {
    if (!t) return;
    t->data    = data;
    t->x_axis  = x_axis;
    t->y_axis  = y_axis;
    t->rows    = rows;
    t->cols    = cols;
}

double table2d_lookup(const table2d_t *t, double x, double y) {
    if (!t || !t->data || t->rows < 2 || t->cols < 2) return 0.0;

    /* Find x interval */
    int ix = 0;
    if (x <= t->x_axis[0]) {
        ix = 0;
    } else if (x >= t->x_axis[t->rows - 1]) {
        ix = t->rows - 2;
    } else {
        for (int i = 0; i < t->rows - 1; i++) {
            if (x >= t->x_axis[i] && x <= t->x_axis[i + 1]) {
                ix = i;
                break;
            }
        }
    }

    /* Find y interval */
    int iy = 0;
    if (y <= t->y_axis[0]) {
        iy = 0;
    } else if (y >= t->y_axis[t->cols - 1]) {
        iy = t->cols - 2;
    } else {
        for (int j = 0; j < t->cols - 1; j++) {
            if (y >= t->y_axis[j] && y <= t->y_axis[j + 1]) {
                iy = j;
                break;
            }
        }
    }

    /* Get corner values */
    double f00 = t->data[ix * t->cols + iy];
    double f10 = t->data[(ix + 1) * t->cols + iy];
    double f01 = t->data[ix * t->cols + (iy + 1)];
    double f11 = t->data[(ix + 1) * t->cols + (iy + 1)];

    /* Normalized coordinates */
    double dx = t->x_axis[ix + 1] - t->x_axis[ix];
    double dy = t->y_axis[iy + 1] - t->y_axis[iy];
    double tx = (fabs(dx) < 1e-15) ? 0.0 : (x - t->x_axis[ix]) / dx;
    double ty = (fabs(dy) < 1e-15) ? 0.0 : (y - t->y_axis[iy]) / dy;

    /* Bilinear interpolation */
    return (1.0 - tx) * (1.0 - ty) * f00
         + tx * (1.0 - ty) * f10
         + (1.0 - tx) * ty * f01
         + tx * ty * f11;
}

/* ============================================================================
 * Nyquist Minimum Sample Rate
 *
 * L4: Shannon-Nyquist Sampling Theorem
 *
 * Theorem: A bandlimited signal with maximum frequency f_max can be
 * perfectly reconstructed from samples taken at a rate f_s >= 2*f_max.
 *
 * In practice, a safety factor of 5-10 is used because:
 *   1. Real signals are not perfectly bandlimited
 *   2. Anti-aliasing filters have finite roll-off (not brick-wall)
 *   3. Control systems need to respond faster than the signal bandwidth
 *
 * For industrial loops:
 *   - Flow control: f_s >= 10 Hz (f_max ~ 0.2-0.5 Hz)
 *   - Pressure control: f_s >= 50 Hz (f_max ~ 1-5 Hz)
 *   - Temperature control: f_s >= 1 Hz (f_max ~ 0.01-0.05 Hz)
 *
 * Undersampling leads to aliasing: high frequencies fold back into
 * the passband as false low-frequency signals, corrupting control.
 * ============================================================================
 */

double nyquist_min_sample_rate(double fmax, double safety_factor) {
    if (fmax <= 0.0 || safety_factor < 1.0) return 0.0;
    return 2.0 * fmax * safety_factor;
}