#ifndef PLC_CORE_H
#define PLC_CORE_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Core Definitions - PLC Architecture & IEC 61131-3 Model
 * ========================================================================= */

typedef enum {
    PLC_MODE_RUN        = 0,
    PLC_MODE_STOP       = 1,
    PLC_MODE_PROGRAM    = 2,
    PLC_MODE_DEBUG      = 3,
    PLC_MODE_FAULT      = 4,
    PLC_MODE_MAINTENANCE= 5
} plc_mode_t;

typedef enum {
    PLC_PHASE_INPUT_SCAN    = 0,
    PLC_PHASE_PROGRAM_EXEC  = 1,
    PLC_PHASE_OUTPUT_SCAN   = 2,
    PLC_PHASE_HOUSEKEEPING  = 3,
    PLC_PHASE_IDLE          = 4
} plc_scan_phase_t;

typedef enum {
    PLC_TYPE_BOOL=0, PLC_TYPE_BYTE=1, PLC_TYPE_WORD=2, PLC_TYPE_DWORD=3,
    PLC_TYPE_LWORD=4, PLC_TYPE_SINT=5, PLC_TYPE_INT=6, PLC_TYPE_DINT=7,
    PLC_TYPE_LINT=8, PLC_TYPE_USINT=9, PLC_TYPE_UINT=10, PLC_TYPE_UDINT=11,
    PLC_TYPE_ULINT=12, PLC_TYPE_REAL=13, PLC_TYPE_LREAL=14,
    PLC_TYPE_TIME=15, PLC_TYPE_DATE=16, PLC_TYPE_TOD=17, PLC_TYPE_DT=18,
    PLC_TYPE_STRING=19
} plc_data_type_t;

typedef enum {
    PLC_IO_DISCRETE_INPUT=0, PLC_IO_DISCRETE_OUTPUT=1,
    PLC_IO_ANALOG_INPUT=2, PLC_IO_ANALOG_OUTPUT=3,
    PLC_IO_INTERNAL_BIT=4, PLC_IO_INTERNAL_REG=5,
    PLC_IO_TIMER=6, PLC_IO_COUNTER=7, PLC_IO_SYSTEM=8
} plc_io_type_t;

#define PLC_MAX_DIGITAL_IO   256
#define PLC_MAX_ANALOG_IO    64
#define PLC_MAX_TIMERS       64
#define PLC_MAX_COUNTERS     64
#define PLC_MAX_INTERNAL_BITS  512
#define PLC_MAX_INTERNAL_REGS  256

typedef struct {
    double   target_scan_time_ms;
    double   max_scan_time_ms;
    double   watchdog_timeout_ms;
    double   input_phase_budget_ms;
    double   exec_phase_budget_ms;
    double   output_phase_budget_ms;
    uint32_t scan_counter;
    double   last_scan_time_ms;
    double   min_scan_time_ms;
    double   max_scan_time_ms_hist;
    double   avg_scan_time_ms;
    double   jitter_ms;
    int      overrun_count;
} plc_scan_config_t;

typedef struct {
    double   utilization;
    double   rms_bound;
    double   response_time_ms;
    double   latency_jitter_ms;
    int      is_schedulable;
} plc_timing_metrics_t;

typedef struct {
    uint16_t address;
    plc_io_type_t io_type;
    int      value;
    int      forced;
    int      forced_value;
    int      fault;
    char     tag[32];
} plc_digital_io_t;

typedef struct {
    uint16_t address;
    plc_io_type_t io_type;
    double   raw_value;
    double   eng_value;
    double   scale_min;
    double   scale_max;
    double   eng_min;
    double   eng_max;
    double   filter_alpha;
    double   filtered_value;
    double   rate_of_change;
    int      overrange;
    int      underrange;
    int      forced;
    double   forced_value;
    int      fault;
    char     tag[32];
} plc_analog_io_t;

typedef enum { PLC_TIMER_TON=0, PLC_TIMER_TOF=1, PLC_TIMER_TP=2, PLC_TIMER_RTO=3 } plc_timer_type_t;

typedef struct {
    plc_timer_type_t type;
    double   preset_time_ms;
    double   elapsed_time_ms;
    int      input;
    int      output;
    int      reset;
    double   last_update_ms;
    int      timing;
} plc_timer_t;

typedef enum { PLC_CTU=0, PLC_CTD=1, PLC_CTUD=2 } plc_counter_type_t;

typedef struct {
    plc_counter_type_t type;
    int32_t preset_value;
    int32_t current_value;
    int     count_up, count_dn;
    int     reset, load;
    int     output_up, output_dn;
    int     last_cu, last_cd;
} plc_counter_t;

typedef struct {
    plc_mode_t        mode;
    plc_scan_config_t scan_cfg;
    plc_scan_phase_t  current_phase;
    uint64_t          firmware_version;
    plc_digital_io_t  digital_inputs[PLC_MAX_DIGITAL_IO];
    plc_digital_io_t  digital_outputs[PLC_MAX_DIGITAL_IO];
    plc_analog_io_t   analog_inputs[PLC_MAX_ANALOG_IO];
    plc_analog_io_t   analog_outputs[PLC_MAX_ANALOG_IO];
    uint16_t          num_digital_in;
    uint16_t          num_digital_out;
    uint16_t          num_analog_in;
    uint16_t          num_analog_out;
    int               internal_bits[PLC_MAX_INTERNAL_BITS];
    double            internal_regs[PLC_MAX_INTERNAL_REGS];
    uint16_t          num_internal_bits;
    uint16_t          num_internal_regs;
    plc_timer_t       timers[PLC_MAX_TIMERS];
    plc_counter_t     counters[PLC_MAX_COUNTERS];
    uint16_t          num_timers;
    uint16_t          num_counters;
    uint32_t          total_scans;
    uint32_t          fault_count;
    double            uptime_seconds;
    double            cpu_temperature_c;
    int               fault_active;
    char              fault_message[128];
} plc_system_t;

/* L2: Scan Cycle Engine */
int plc_init(plc_system_t *plc);
int plc_scan_cycle(plc_system_t *plc,
                   int (*logic_exec)(plc_system_t*, double, void*),
                   void *user_data, double delta_ms);
int plc_set_mode(plc_system_t *plc, plc_mode_t new_mode);
int plc_reset(plc_system_t *plc);

/* L2: Digital I/O */
int plc_read_digital_input(const plc_system_t *plc, uint16_t addr);
int plc_read_digital_output(const plc_system_t *plc, uint16_t addr);
int plc_read_internal_bit(const plc_system_t *plc, uint16_t addr);
int plc_write_digital_output(plc_system_t *plc, uint16_t addr, int value);
int plc_write_internal_bit(plc_system_t *plc, uint16_t addr, int value);
int plc_config_digital_io(plc_system_t *plc, uint16_t addr,
                          const char *tag, plc_io_type_t io_type);

/* L2: Analog I/O */
double plc_read_analog_input(const plc_system_t *plc, uint16_t addr);
double plc_read_analog_output(const plc_system_t *plc, uint16_t addr);
double plc_read_internal_reg(const plc_system_t *plc, uint16_t addr);
int plc_write_analog_output(plc_system_t *plc, uint16_t addr, double value);
int plc_write_internal_reg(plc_system_t *plc, uint16_t addr, double value);
int plc_config_analog_io(plc_system_t *plc, uint16_t addr, const char *tag,
                         plc_io_type_t io_type, double scale_min,
                         double scale_max, double eng_min, double eng_max,
                         double filter_alpha);
void plc_analog_scale_and_filter(plc_analog_io_t *aio);

/* L2: Timer API */
int plc_timer_config(plc_system_t *plc, uint16_t idx,
                     plc_timer_type_t type, double pt_ms);
int plc_timer_update(plc_system_t *plc, uint16_t idx, double dt_ms);
int plc_timer_set_input(plc_system_t *plc, uint16_t idx, int in_value);
int plc_timer_get_output(const plc_system_t *plc, uint16_t idx);
int plc_timer_reset(plc_system_t *plc, uint16_t idx);

/* L2: Counter API */
int plc_counter_config(plc_system_t *plc, uint16_t idx,
                       plc_counter_type_t type, int32_t pv);
int plc_counter_update(plc_system_t *plc, uint16_t idx,
                       int cu_signal, int cd_signal,
                       int reset_signal, int load_signal);
int plc_counter_get_qu(const plc_system_t *plc, uint16_t idx);
int plc_counter_get_qd(const plc_system_t *plc, uint16_t idx);
int32_t plc_counter_get_cv(const plc_system_t *plc, uint16_t idx);
int plc_counter_reset(plc_system_t *plc, uint16_t idx);

/* L3: Edge Detection */
typedef struct { int last_input; int output; } plc_r_trig_t;
void plc_r_trig_reset(plc_r_trig_t *rt);
int  plc_r_trig_update(plc_r_trig_t *rt, int input);

typedef struct { int last_input; int output; } plc_f_trig_t;
void plc_f_trig_reset(plc_f_trig_t *ft);
int  plc_f_trig_update(plc_f_trig_t *ft, int input);

/* L3: Flip-Flops */
typedef struct { int output; } plc_sr_ff_t;
void plc_sr_ff_reset(plc_sr_ff_t *ff);
int  plc_sr_ff_update(plc_sr_ff_t *ff, int set, int reset);

typedef struct { int output; } plc_rs_ff_t;
void plc_rs_ff_reset(plc_rs_ff_t *ff);
int  plc_rs_ff_update(plc_rs_ff_t *ff, int set, int reset);

/* L4: Timing Analysis */
int plc_analyze_timing(const plc_scan_config_t *cfg,
                       plc_timing_metrics_t *metrics);
double plc_min_scan_rate_hz(double f_max_signal_hz, double safety_factor);
int plc_check_nyquist(double scan_rate_hz, double f_max_signal_hz);

/* L5: RMS Scheduler */
int plc_rms_schedulable(const double *periods_ms,
                        const double *budgets_ms, size_t n);

/* L5: PID Controller */
typedef struct {
    double   kp, ki, kd;
    double   ts_sec;
    double   setpoint;
    double   integral;
    double   last_error;
    double   output;
    double   out_min, out_max;
    int      anti_windup;
    int      auto_mode;
} plc_pid_t;

void plc_pid_init(plc_pid_t *pid, double kp, double ki, double kd,
                  double ts_sec, double out_min, double out_max);
double plc_pid_update(plc_pid_t *pid, double setpoint, double measurement);
void plc_pid_reset(plc_pid_t *pid);
void plc_pid_ziegler_nichols_open(double K, double tau, double theta,
                                  double ts_sec, double out_min, double out_max,
                                  plc_pid_t *pid);
void plc_pid_ziegler_nichols_closed(double amplitude, double osc_period,
                                    double osc_ampl, double ts_sec,
                                    double out_min, double out_max,
                                    plc_pid_t *pid);

/* L5: Digital Filtering */
typedef struct {
    double   alpha;
    double   last_output;
    int      initialized;
} plc_iir_filter_t;

void plc_iir_filter_init(plc_iir_filter_t *f, double alpha);
double plc_iir_filter_update(plc_iir_filter_t *f, double input);
double plc_iir_alpha_from_fc(double fc_hz, double fs_hz);

typedef struct {
    size_t   window_size;
    size_t   index;
    size_t   count;
    double  *buffer;
    double   sum;
} plc_moving_avg_t;

int  plc_moving_avg_init(plc_moving_avg_t *ma, size_t window_size);
void plc_moving_avg_free(plc_moving_avg_t *ma);
double plc_moving_avg_update(plc_moving_avg_t *ma, double input);
void plc_moving_avg_reset(plc_moving_avg_t *ma);

/* L6: Sequential Function Chart Engine */
typedef struct {
    int      step_id;
    int      active;
    double   active_time_ms;
    int      action_qualifier;
    void    *action_data;
} plc_sfc_step_t;

typedef struct {
    int      trans_id;
    int      condition;
    int      from_step;
    int      to_step;
} plc_sfc_transition_t;

typedef struct {
    plc_sfc_step_t       *steps;
    plc_sfc_transition_t *transitions;
    size_t                num_steps;
    size_t                num_transitions;
    size_t                step_capacity;
    size_t                trans_capacity;
    int                   initial_step;
    int                   initialized;
} plc_sfc_engine_t;

int  plc_sfc_init(plc_sfc_engine_t *sfc, size_t max_steps, size_t max_trans);
void plc_sfc_free(plc_sfc_engine_t *sfc);
int  plc_sfc_add_step(plc_sfc_engine_t *sfc, int step_id, int initial);
int  plc_sfc_add_transition(plc_sfc_engine_t *sfc, int trans_id,
                            int from_step, int to_step);
int  plc_sfc_set_condition(plc_sfc_engine_t *sfc, int trans_id, int cond);
int  plc_sfc_scan(plc_sfc_engine_t *sfc, double dt_ms);
int  plc_sfc_get_active_step(const plc_sfc_engine_t *sfc);
void plc_sfc_reset(plc_sfc_engine_t *sfc);

/* L7: Modbus CRC-16 */
uint16_t plc_modbus_crc16(const uint8_t *data, size_t length);
int plc_modbus_crc16_check(const uint8_t *frame, size_t total_length);

/* L8: Redundancy */
typedef enum {
    PLC_REDUN_PRIMARY   = 0,
    PLC_REDUN_SECONDARY = 1,
    PLC_REDUN_OFFLINE   = 2
} plc_redundancy_role_t;

typedef struct {
    plc_redundancy_role_t role;
    int      primary_healthy;
    int      secondary_healthy;
    double   sync_data_timestamp_ms;
    double   switchover_time_ms;
    int      switchover_count;
    int      heartbeat_missed;
    uint32_t sequence_number;
    int      sync_buffer[64];
    size_t   sync_buffer_len;
} plc_redundancy_t;

int  plc_redundancy_init(plc_redundancy_t *red, plc_redundancy_role_t role);
int  plc_redundancy_heartbeat(plc_redundancy_t *red, int peer_alive);
int  plc_redundancy_sync(plc_redundancy_t *red,
                         const plc_system_t *source, plc_system_t *target);
int  plc_redundancy_switchover(plc_redundancy_t *red);

/* L8: Safety Integrity Level */
double plc_sil_compute_pfd(double lambda_du,
                           double proof_test_interval_hours,
                           int architecture);
int plc_sil_get_level(double pfd);

#ifdef __cplusplus
}
#endif

#endif /* PLC_CORE_H */