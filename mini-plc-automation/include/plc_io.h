#ifndef PLC_IO_H
#define PLC_IO_H
#include "plc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2: I/O Subsystem — Physical I/O Management
 *
 * PLC I/O subsystem manages:
 *   1. Discrete I/O: on/off sensors and actuators (limit switches,
 *      pushbuttons, contactors, solenoids, indicator lamps)
 *   2. Analog I/O: continuous signals via ADC/DAC (4–20 mA, 0–10 V,
 *      thermocouples, RTDs, strain gauges, pressure transmitters)
 *   3. High-Speed I/O: pulse trains, encoders, PWM outputs
 *
 * I/O mapping model:
 *   Physical I/O → I/O Image Table → User Program → Output Image → Physical Output
 *
 * The I/O image table is updated during INPUT_SCAN and OUTPUT_SCAN phases.
 * This double-buffering prevents inconsistent data mid-scan.
 * ========================================================================= */

/* =========================================================================
 * L2: I/O Map — Complete I/O Configuration
 * ========================================================================= */

/** Analog input signal type (for scaling configuration) */
typedef enum {
    AIO_SIG_4_20mA     = 0,  /* 4–20 mA current loop              */
    AIO_SIG_0_20mA     = 1,  /* 0–20 mA current                   */
    AIO_SIG_0_10V      = 2,  /* 0–10 V voltage                    */
    AIO_SIG_0_5V       = 3,  /* 0–5 V voltage                     */
    AIO_SIG_M10_10V    = 4,  /* -10 to +10 V                      */
    AIO_SIG_THERMOCOUPLE_K = 5,  /* Type K: -200 to 1372 C        */
    AIO_SIG_THERMOCOUPLE_J = 6,  /* Type J: -210 to 1200 C        */
    AIO_SIG_RTD_PT100  = 7,  /* PT100 RTD: -200 to 850 C          */
    AIO_SIG_POTENTIOMETER = 8, /* 3-wire potentiometer            */
    AIO_SIG_CUSTOM     = 9   /* User-defined scaling              */
} aio_signal_type_t;

/** Analog output signal type */
typedef enum {
    AOO_SIG_4_20mA  = 0,
    AOO_SIG_0_20mA  = 1,
    AOO_SIG_0_10V   = 2,
    AOO_SIG_M10_10V = 3,
    AOO_SIG_PWM      = 4
} aoo_signal_type_t;

/**
 * @brief Thermocouple linearization polynomial coefficients
 *
 * Type K (Chromel-Alumel) polynomial for 0–1372 C:
 *   V(T) = Σ c_i * T^i  (i = 0..n)
 * where V is in microvolts, T in Celsius.
 *
 * Inverse (T from V) uses piecewise polynomial per ITS-90.
 */
typedef struct {
    double   coeffs[10];   /* Polynomial coefficients c_0..c_9  */
    int      degree;       /* Polynomial degree                */
    double   t_min_c;      /* Valid range min (Celsius)        */
    double   t_max_c;      /* Valid range max (Celsius)        */
    double   v_min_uv;     /* Valid range min (microvolts)     */
    double   v_max_uv;     /* Valid range max (microvolts)     */
} plc_thermocouple_cal_t;

/**
 * @brief RTD (PT100) linearization
 *
 * Callendar-Van Dusen equation:
 *   For T ≥ 0 C: R(T) = R0 * (1 + A*T + B*T^2)
 *   For T < 0 C: R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Where:
 *   R0 = 100 Ω,  A = 3.9083e-3,  B = -5.775e-7,  C = -4.183e-12
 */
typedef struct {
    double   r0_ohms;       /* Resistance at 0 C (100.0)       */
    double   a_coeff;       /* A coefficient                   */
    double   b_coeff;       /* B coefficient                   */
    double   c_coeff;       /* C coefficient                   */
    double   t_min_c;
    double   t_max_c;
} plc_rtd_cal_t;

/* =========================================================================
 * L2: High-Speed Counter (HSC) Configuration
 * ========================================================================= */

/**
 * @brief High-speed counter for encoder/pulse inputs
 *
 * Unlike standard counters, HSC operates in hardware/firmware
 * and can track pulses at kHz–MHz rates independent of scan cycle.
 */
typedef enum {
    HSC_MODE_PULSE_DIR   = 0,  /* Pulse on A, direction on B   */
    HSC_MODE_QUADRATURE  = 1,  /* Quadrature (A leads B = CW)  */
    HSC_MODE_UP_DOWN     = 2,  /* Up pulses on A, down on B    */
    HSC_MODE_FREQUENCY   = 3   /* Measure frequency of A       */
} plc_hsc_mode_t;

typedef struct {
    uint16_t         address;      /* HSC instance number        */
    plc_hsc_mode_t   mode;
    int32_t          current_count;
    int32_t          preset;
    int32_t          rollover;     /* 0=no rollover, >0=mod N   */
    int              output;       /* Internal output bit        */
    double           frequency_hz; /* Measured in frequency mode */
    int              enabled;
    double           last_capture_time_ms;
} plc_hsc_t;

/* =========================================================================
 * L2: PWM Output Configuration
 * ========================================================================= */

typedef struct {
    uint16_t address;
    double   frequency_hz;     /* PWM carrier frequency         */
    double   duty_cycle;       /* 0.0 (0%) to 1.0 (100%)       */
    double   period_ms;        /* 1/frequency * 1000            */
    int      enabled;
} plc_pwm_output_t;

/* =========================================================================
 * L3: Analog Scaling Functions
 * ========================================================================= */

/**
 * @brief Linear scaling: raw ADC counts to engineering units
 *
 *   eng = eng_min + (raw - raw_min) * (eng_max - eng_min)
 *                  / (raw_max - raw_min)
 *
 * This is an affine transformation (y = ax + b):
 *   a = (eng_max - eng_min) / (raw_max - raw_min)
 *   b = eng_min - a * raw_min
 *
 * @param raw_value   ADC/DAC counts
 * @param raw_min     Minimum raw value (e.g., 0 or 6241 for 4mA)
 * @param raw_max     Maximum raw value (e.g., 4095 or 31206 for 20mA)
 * @param eng_min     Minimum engineering value
 * @param eng_max     Maximum engineering value
 * @return Scaled engineering value
 */
double plc_scale_linear(double raw_value, double raw_min, double raw_max,
                        double eng_min, double eng_max);

/**
 * @brief Square-root scaling for differential pressure flow meters
 *
 * Flow rate ∝ √(ΔP):
 *   Q = eng_min + (eng_max - eng_min) * √((raw - raw_min)/(raw_max - raw_min))
 *
 * @param raw_value   Raw differential pressure reading
 * @param raw_zero    Raw value at zero flow (typically 4 mA → 6241)
 * @param raw_span    Raw span (20 mA value - 4 mA value)
 * @param eng_min     Minimum flow rate
 * @param eng_max     Maximum flow rate
 * @param cutoff      Below this fraction of span, output = eng_min (prevents noise)
 * @return Computed flow rate
 */
double plc_scale_sqrt(double raw_value, double raw_zero, double raw_span,
                      double eng_min, double eng_max, double cutoff);

/**
 * @brief Type K Thermocouple: microvolts to Celsius
 *
 * Uses ITS-90 inverse polynomial (piecewise).
 *
 * @param uv   Measured EMF in microvolts
 * @return Temperature in Celsius
 */
double plc_thermocouple_k_uv_to_c(double uv);

/**
 * @brief PT100 RTD: resistance to Celsius
 *
 * Solves Callendar-Van Dusen equation for T given R.
 *
 * @param r_ohms  Measured resistance
 * @return Temperature in Celsius
 */
double plc_rtd_pt100_r_to_c(double r_ohms);

/* =========================================================================
 * L4: I/O Scan Determinism — Real-Time I/O Theory
 * ========================================================================= */

/**
 * @brief Compute I/O latency budget for a given scan rate
 *
 * Maximum I/O latency L_max in a PLC:
 *   L_max = T_scan + T_input + T_output + T_network_delay
 *
 * For control loop stability:
 *   L_max < τ / 5  (rule of thumb: 5x margin over process time constant τ)
 *
 * From sampled-data control theory:
 *   Phase margin degradation = ω_c * T_s / 2
 *   where ω_c = crossover frequency, T_s = sample time
 *
 * @param scan_period_ms   T_scan
 * @param io_delay_ms      T_input + T_output
 * @param process_tau_ms   Process time constant τ
 * @param phase_margin_deg Desired minimum phase margin
 * @return 1 if latency is acceptable, 0 if too large
 */
int plc_io_latency_check(double scan_period_ms, double io_delay_ms,
                         double process_tau_ms, double phase_margin_deg);

/**
 * @brief I/O jitter analysis
 *
 * Jitter J is the variation in I/O update time.
 * For time-critical I/O (motion control, precise timing):
 *   J_max < T_scan / 10  (acceptable)
 *
 * Jitter causes:
 *   - Input aliasing (variable sampling interval)
 *   - Output timing error (PWM distortion)
 *   - Control loop detuning (effective Kp varies with T_s)
 *
 * @param jitter_samples   Array of measured jitter values
 * @param n_samples        Number of samples
 * @param mean_jitter_out  Computed mean jitter
 * @param max_jitter_out   Max absolute jitter
 * @param std_jitter_out   Standard deviation of jitter
 */
void plc_analyze_jitter(const double *jitter_samples, size_t n_samples,
                        double *mean_jitter_out, double *max_jitter_out,
                        double *std_jitter_out);

/* =========================================================================
 * L5: Input Debounce Algorithm
 * ========================================================================= */

/**
 * @brief Software debounce for digital inputs
 *
 * Mechanical contacts bounce on make/break, producing multiple
 * transitions within ~5–20 ms. Debounce rejects these glitches.
 *
 * Algorithm: Integration debounce
 *   1. Sample input at high rate (e.g., 1 kHz)
 *   2. Integrate: if input=1, counter++; if input=0, counter--
 *   3. Saturation: counter clamped to [0, period_samples]
 *   4. Output = (counter >= threshold)
 *
 * Equivalent to: moving average > threshold ratio.
 *
 * @param raw_signal     Current raw input sample (0 or 1)
 * @param state          Debounce state (caller maintains across calls)
 * @param period_samples Number of samples for debounce window
 * @param threshold      Samples needed to confirm state change
 * @return Debounced output (0 or 1)
 */
int plc_debounce(int raw_signal, int *counter, int period_samples,
                 int threshold);

/* =========================================================================
 * L5: 4–20 mA Loop Diagnostics
 * ========================================================================= */

/**
 * @brief Check 4–20 mA signal health
 *
 * Wire break detection:
 *   current < 3.5 mA  → open circuit (broken wire)
 *   current > 21.0 mA  → short circuit or sensor fault
 *   3.5 ≤ I ≤ 21.0    → normal operating range
 *
 * NAMUR NE43 recommendation:
 *   < 3.6 mA  — fail low (wire break)
 *   3.8–20.5 mA — normal
 *   > 21.0 mA — fail high (short)
 *
 * @param current_ma   Measured loop current in mA
 * @param status_out   Output status: -1=fail low, 0=normal, 1=fail high
 * @return 0 if normal, non-zero if fault
 */
int plc_check_4_20ma_health(double current_ma, int *status_out);

/* =========================================================================
 * L6: I/O Mapping — Tag-based addressing
 * ========================================================================= */

/**
 * @brief Resolve a tag name to I/O address and type
 *
 * Searches all I/O banks for a matching tag.
 * Tags are assigned via plc_config_digital_io / plc_config_analog_io.
 *
 * @param tag       Tag name (e.g., "MOTOR_START", "TANK_LEVEL")
 * @param io_type   Output: I/O type (%I, %Q, %MW, etc.)
 * @param addr      Output: I/O address
 * @return 0 if found, -1 if not found
 */
int plc_resolve_tag(const plc_system_t *plc, const char *tag,
                    plc_io_type_t *io_type, uint16_t *addr);

/**
 * @brief Force an I/O point (maintenance override)
 *
 * When forced, the I/O value is overridden regardless of
 * physical input or program output. Used for testing/debug.
 *
 * Safety: forces are automatically cleared on PLC_MODE_RUN entry
 * unless explicitly re-enabled.
 *
 * @return 0 on success, -1 if point doesn't exist
 */
int plc_force_digital(plc_system_t *plc, plc_io_type_t io_type,
                      uint16_t addr, int forced_value);
int plc_force_analog(plc_system_t *plc, plc_io_type_t io_type,
                     uint16_t addr, double forced_value);

/** Clear all forces */
void plc_clear_all_forces(plc_system_t *plc);

/* =========================================================================
 * L6: I/O Simulation — Virtual plant I/O model
 * ========================================================================= */

/**
 * @brief Simulate a first-order process with dead time (FOPDT)
 *
 * Plant transfer function:
 *   G(s) = K * e^{-θ*s} / (τ * s + 1)
 *
 * Discrete-time approximation (forward Euler):
 *   PV[k] = PV[k-1] + (T_s/τ) * (K * MV[k-d] - PV[k-1])
 *
 * where d = round(θ / T_s) is the number of delay steps.
 *
 * This is the standard model used for PID tuning (Ziegler-Nichols,
 * Cohen-Coon, IMC) and control loop simulation.
 *
 * @param mv             Manipulated variable input (controller output)
 * @param K              Process gain (ΔPV/ΔMV in steady state)
 * @param tau            Time constant
 * @param theta          Dead time
 * @param ts             Sample time (must equal scan period)
 * @param prev_pv        Previous process variable value
 * @param delay_buffer   Ring buffer for dead time (size = ceil(theta/ts)+1)
 * @param delay_len      Buffer length
 * @param delay_idx      Current buffer position (updated by call)
 * @param new_pv         Output: new process variable
 */
void plc_simulate_fopdt(double mv, double K, double tau, double theta,
                        double ts, double prev_pv,
                        double *delay_buffer, size_t delay_len,
                        size_t *delay_idx, double *new_pv);

/**
 * @brief Simulate a water tank level
 *
 * Tank dynamics (mass balance):
 *   A * dh/dt = Q_in - Q_out
 *
 * where:
 *   A = tank cross-sectional area
 *   Q_in = K_valve * u  (valve flow, u ∈ [0,1])
 *   Q_out = K_drain * sqrt(h)  (gravity drain)
 *
 * Level limiting: 0 ≤ h ≤ h_max
 *
 * @param u              Valve position (0=closed, 1=fully open)
 * @param area_m2        Tank cross-section area [m^2]
 * @param k_valve        Valve flow coefficient [m^3/s]
 * @param k_drain        Drain coefficient [m^2.5/s]
 * @param h_max_m        Maximum tank height [m]
 * @param ts_sec         Sample time [s]
 * @param h_m            Prev/next level [m] (updated in place)
 */
void plc_simulate_tank(double u, double area_m2, double k_valve,
                       double k_drain, double h_max_m, double ts_sec,
                       double *h_m);

#ifdef __cplusplus
}
#endif

#endif /* PLC_IO_H */