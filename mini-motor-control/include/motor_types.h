/**
 * @file motor_types.h
 * @brief Motor type definitions, enumerations, and core data structures
 *
 * Covers L1 Definitions:
 *   - Motor type enumeration (DC, BLDC, PMSM, Induction, Stepper)
 *   - Motor state structure (electrical + mechanical state)
 *   - Control mode enumeration (torque, speed, position)
 *   - PWM configuration structures
 *   - Fault type bitfield definitions
 *
 * Key concepts: Lorentz force (F = BIL), Faraday is law (epsilon = -dPhi/dt),
 *   back-EMF constant (Ke), torque constant (Kt), winding types
 *
 * Reference: Sedra & Smith (2020), Erickson & Maksimovic (2001)
 * Course: MIT 6.450, Stanford EE359, Berkeley EE105
 */

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Ensure M_PI is defined (POSIX extension, not in strict C11) */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*=============================================================================
 * L1: Motor Type Definitions
 *============================================================================*/

/** Motor topologies supported by this library */
typedef enum {
    MOTOR_DC_BRUSHED      = 0,  /* Permanent magnet DC brushed motor */
    MOTOR_DC_BRUSHLESS    = 1,  /* Brushless DC (BLDC) with trapezoidal BEMF */
    MOTOR_PMSM            = 2,  /* Permanent Magnet Synchronous Motor */
    MOTOR_INDUCTION       = 3,  /* AC Induction Motor (IM) */
    MOTOR_STEPPER         = 4,  /* Stepper motor (hybrid/VR/PM) */
    MOTOR_SRM             = 5,  /* Switched Reluctance Motor */
    MOTOR_SYNCHRONOUS_REL = 6,  /* Synchronous Reluctance Motor */
    MOTOR_COUNT
} motor_type_t;

/** Winding connection configuration */
typedef enum {
    WINDING_WYE   = 0,  /* Star/Wye connection */
    WINDING_DELTA = 1   /* Delta connection */
} winding_type_t;

/** Control mode for cascaded control structure */
typedef enum {
    CONTROL_MODE_TORQUE   = 0,  /* Inner current/torque loop only */
    CONTROL_MODE_SPEED    = 1,  /* Speed loop + current loop */
    CONTROL_MODE_POSITION = 2   /* Position loop + speed loop + current loop */
} control_mode_t;

/** Commutation method */
typedef enum {
    COMMUTATION_SIX_STEP     = 0,  /* Six-step block commutation (BLDC) */
    COMMUTATION_SINUSOIDAL   = 1,  /* Sinusoidal PWM (PMSM) */
    COMMUTATION_FOC          = 2,  /* Field-Oriented Control */
    COMMUTATION_DTC          = 3,  /* Direct Torque Control */
    COMMUTATION_SPACE_VECTOR = 4   /* Space Vector PWM */
} commutation_method_t;

/** Motor direction */
typedef enum {
    MOTOR_DIR_CW  = 0,  /* Clockwise */
    MOTOR_DIR_CCW = 1   /* Counter-clockwise */
} motor_direction_t;

/** Hall sensor commutation sector (6 states for BLDC 120 sensors) */
typedef enum {
    HALL_SECTOR_0 = 0,  /* 000 - invalid/error */
    HALL_SECTOR_1 = 1,  /* 001 */
    HALL_SECTOR_2 = 2,  /* 010 */
    HALL_SECTOR_3 = 3,  /* 011 */
    HALL_SECTOR_4 = 4,  /* 100 */
    HALL_SECTOR_5 = 5,  /* 101 */
    HALL_SECTOR_6 = 6,  /* 110 */
    HALL_SECTOR_7 = 7   /* 111 - invalid/error */
} hall_sector_t;

/*=============================================================================
 * L1: PWM Configuration
 *============================================================================*/

/** PWM carrier and dead-time configuration */
typedef struct {
    uint32_t pwm_frequency_hz;   /* PWM carrier frequency (typ. 8-20 kHz) */
    uint32_t dead_time_ns;       /* Dead time for complementary outputs */
    uint16_t timer_period;       /* Timer auto-reload value */
    float    dc_bus_voltage;     /* DC bus voltage in volts */
    bool     complementary_mode; /* Enable complementary CHx/CHxN outputs */
} pwm_config_t;

/** Duty cycles for three-phase output */
typedef struct {
    float duty_a;  /* Phase A duty cycle [0, 1] */
    float duty_b;  /* Phase B duty cycle [0, 1] */
    float duty_c;  /* Phase C duty cycle [0, 1] */
} pwm_duty_t;

/** Space vector PWM sector timing */
typedef struct {
    uint8_t sector;    /* Sector number 1-6 */
    float    t1;        /* Active vector 1 time (normalized to Ts) */
    float    t2;        /* Active vector 2 time (normalized to Ts) */
    float    t0;        /* Zero vector time sum (t0 + t7, normalized) */
    float    t7;        /* Zero vector 7 time (for symmetrical PWM) */
} svpwm_timing_t;

/*=============================================================================
 * L1: Motor Electrical Parameters
 *============================================================================*/

/** DC brushed motor parameters */
typedef struct {
    float nominal_voltage;       /* Rated voltage [V] */
    float nominal_current;       /* Rated current [A] */
    float winding_resistance;    /* Armature resistance Ra [Ohm] */
    float winding_inductance;    /* Armature inductance La [H] */
    float ke_back_emf;           /* Back-EMF constant Ke [V/(rad/s)] */
    float kt_torque;             /* Torque constant Kt [N*m/A] */
    float rotor_inertia;         /* Rotor moment of inertia J [kg*m^2] */
    float friction_coefficient;  /* Viscous friction B [N*m/(rad/s)] */
    float pole_pairs;            /* Number of pole pairs */
    float max_speed;             /* Maximum mechanical speed [rad/s] */
} dc_motor_params_t;

/** PMSM / BLDC three-phase motor parameters */
typedef struct {
    float rs;                  /* Stator resistance per phase [Ohm] */
    float ld;                  /* d-axis inductance [H] */
    float lq;                  /* q-axis inductance [H] */
    float ls;                  /* Average inductance Ls = (Ld+Lq)/2 [H] */
    float flux_linkage;        /* PM flux linkage psi_m [Wb] */
    float pole_pairs;          /* Number of pole pairs P */
    float rotor_inertia;       /* Moment of inertia J [kg*m^2] */
    float friction_coefficient;/* Viscous friction B [N*m/(rad*s)] */
    float rated_current;       /* Rated phase current amplitude [A] */
    float rated_speed;         /* Rated mechanical speed [rad/s] */
    float rated_torque;        /* Rated torque [N*m] */
    float max_dc_voltage;      /* Maximum DC bus voltage [V] */
} pmsm_params_t;

/** Induction motor T-model parameters */
typedef struct {
    float rs;              /* Stator resistance [Ohm] */
    float rr;              /* Rotor resistance referred to stator [Ohm] */
    float lls;             /* Stator leakage inductance [H] */
    float llr;             /* Rotor leakage inductance [H] */
    float lm;              /* Magnetizing inductance [H] */
    float pole_pairs;      /* Number of pole pairs */
    float rotor_inertia;   /* Moment of inertia J [kg*m^2] */
    float rated_power;     /* Rated power [W] */
    float rated_frequency; /* Rated frequency [Hz] */
    float rated_slip;      /* Rated slip (s = (ns-nr)/ns) */
} induction_motor_params_t;

/** Stepper motor parameters */
typedef struct {
    float winding_resistance;    /* Per phase [Ohm] */
    float winding_inductance;    /* Per phase [H] */
    float holding_torque;        /* Maximum static torque [N*m] */
    float detent_torque;         /* Detent (cogging) torque [N*m] */
    uint16_t steps_per_rev;      /* Full steps per revolution (typ. 200) */
    float rated_current;         /* Rated phase current [A] */
    float rotor_inertia;         /* Rotor inertia [kg*m^2] */
} stepper_params_t;

/*=============================================================================
 * L1: Motor State (Electrical + Mechanical)
 *============================================================================*/

/** Three-phase current measurements */
typedef struct {
    float ia;   /* Phase A current [A] */
    float ib;   /* Phase B current [A] */
    float ic;   /* Phase C current [A] */
} phase_currents_t;

/** Three-phase voltage references */
typedef struct {
    float va;   /* Phase A voltage [V] */
    float vb;   /* Phase B voltage [V] */
    float vc;   /* Phase C voltage [V] */
} phase_voltages_t;

/** Stationary reference frame (alpha-beta, Clarke transform domain) */
typedef struct {
    float alpha;  /* alpha-axis component */
    float beta;   /* beta-axis component */
    float zero;   /* Zero-sequence (homopolar) component */
} clarke_vector_t;

/** Synchronous rotating reference frame (d-q, Park transform domain) */
typedef struct {
    float d;      /* d-axis (direct) - flux producing component */
    float q;      /* q-axis (quadrature) - torque producing component */
} park_vector_t;

/** Complete motor runtime state */
typedef struct {
    /* Electrical state */
    phase_currents_t phase_currents;       /* Measured phase currents */
    phase_voltages_t phase_voltages;       /* Applied phase voltages */
    clarke_vector_t  current_ab;           /* Clarke-transformed currents */
    park_vector_t    current_dq;           /* Park-transformed currents */
    park_vector_t    voltage_dq_ref;       /* Voltage references in dq frame */

    /* Mechanical state */
    float rotor_angle_electrical;          /* Electrical angle theta_e = P*theta_m */
    float rotor_angle_mechanical;          /* Mechanical angle theta_m [rad] */
    float rotor_speed_electrical;          /* Electrical speed omega_e [rad/s] */
    float rotor_speed_mechanical;          /* Mechanical speed omega_m [rad/s] */
    float torque_electromagnetic;          /* Produced torque Te [N*m] */
    float torque_load;                     /* Load torque TL [N*m] */

    /* System state */
    float dc_bus_voltage;                  /* DC link voltage [V] */
    float temperature_windings;            /* Winding temperature [C] */
    uint32_t fault_flags;                  /* Bitfield of fault conditions */
} motor_state_t;

/*=============================================================================
 * L1: Fault Types
 *============================================================================*/

/** Fault flag bit definitions */
#define FAULT_OVERCURRENT       (1U << 0)
#define FAULT_OVERVOLTAGE       (1U << 1)
#define FAULT_UNDERVOLTAGE      (1U << 2)
#define FAULT_OVERTEMPERATURE   (1U << 3)
#define FAULT_HALL_SEQUENCE     (1U << 4)
#define FAULT_STALL_DETECTED    (1U << 5)
#define FAULT_ENCODER_FAULT     (1U << 6)
#define FAULT_PHASE_LOSS        (1U << 7)
#define FAULT_GROUND_FAULT      (1U << 8)
#define FAULT_DESATURATION      (1U << 9)  /* IGBT desaturation protection */
#define FAULT_TIMEOUT           (1U << 10) /* Communication timeout */
#define FAULT_BRAKE_OVERTEMP    (1U << 11)
#define FAULT_BEARING_WEAR      (1U << 12)
#define FAULT_INSULATION_FAIL   (1U << 13)

/** Fault severity levels */
typedef enum {
    FAULT_SEV_WARNING   = 0,   /* Non-critical, motor can continue */
    FAULT_SEV_LIMITED   = 1,   /* Reduce power/derate */
    FAULT_SEV_CRITICAL  = 2,   /* Immediate controlled stop */
    FAULT_SEV_EMERGENCY = 3    /* Emergency stop, disable PWM */
} fault_severity_t;

/*=============================================================================
 * L1: Controller Configuration
 *============================================================================*/

/** PID controller gains with low-pass filtered derivative */
typedef struct {
    float kp;             /* Proportional gain */
    float ki;             /* Integral gain (units: per-second) */
    float kd;             /* Derivative gain (units: seconds) */
    float n_coeff;        /* Derivative filter coefficient (low-pass) */
    float out_min;        /* Output saturation lower limit */
    float out_max;        /* Output saturation upper limit */
    float integral_limit; /* Anti-windup integral clamp */
} pid_gains_t;

/** PID runtime state with anti-windup back-calculation tracking */
typedef struct {
    float setpoint;         /* Desired value */
    float feedback;         /* Measured value */
    float error;            /* Current error = setpoint - feedback */
    float error_prev;       /* Previous error for derivative calculation */
    float integral;         /* Accumulated integral term */
    float derivative;       /* Filtered derivative term */
    float derivative_raw;   /* Unfiltered derivative */
    float output_p;         /* Proportional contribution */
    float output_i;         /* Integral contribution */
    float output_d;         /* Derivative contribution */
    float output_total;     /* Total controller output */
    bool  saturated;        /* Saturation flag for anti-windup */
} pid_state_t;

/** Cascaded control loop configuration */
typedef struct {
    pid_gains_t current_iq_gains;   /* q-axis current PI */
    pid_gains_t current_id_gains;   /* d-axis current PI */
    pid_gains_t speed_gains;        /* Speed loop PI */
    pid_gains_t position_gains;     /* Position loop P (or PID) */
    control_mode_t active_mode;     /* Which loops are active */
    float speed_ramp_rate;          /* Speed ramp limit [rad/s^2] */
    float current_limit;            /* Maximum current amplitude [A] */
} cascaded_control_cfg_t;

/*=============================================================================
 * L1: Observer/Sensorless Configuration
 *============================================================================*/

/** Sliding mode observer + PLL configuration for sensorless control */
typedef struct {
    float gain_k;          /* SMO gain */
    float filter_cutoff;   /* Low-pass filter cutoff for BEMF [rad/s] */
    float pll_kp;          /* PLL proportional gain for angle tracking */
    float pll_ki;          /* PLL integral gain for angle tracking */
    float min_speed;       /* Minimum speed for sensorless operation */
    bool  enable_hfi;      /* Enable high-frequency injection at low speed */
    float hfi_frequency;   /* HF injection frequency [Hz] */
    float hfi_amplitude;   /* HF injection voltage amplitude [V] */
} observer_config_t;

/*=============================================================================
 * L1: Function Declarations (implemented in motor_types.c)
 *============================================================================*/

/* Parameter initialization */
void dc_motor_params_init(dc_motor_params_t *params);
void pmsm_params_init(pmsm_params_t *params);
void induction_motor_params_init(induction_motor_params_t *params);
void stepper_params_init(stepper_params_t *params);

/* Parameter validation */
int dc_motor_params_validate(const dc_motor_params_t *params);
int pmsm_params_validate(const pmsm_params_t *params);

/* State management */
void motor_state_init(motor_state_t *state);
void motor_state_reset_faults(motor_state_t *state);

/* Fault management */
bool motor_has_fault(const motor_state_t *state);
bool motor_has_critical_fault(const motor_state_t *state);
fault_severity_t motor_fault_severity(const motor_state_t *state);
void motor_set_fault(motor_state_t *state, uint32_t fault_bit);
void motor_clear_fault(motor_state_t *state, uint32_t fault_bit);

/* Unit conversions */
float rpm_to_rad_per_sec(float rpm);
float rad_per_sec_to_rpm(float rad_per_sec);
float deg_to_rad(float degrees);
float rad_to_deg(float radians);
float mechanical_power(float torque, float speed_rad_per_sec);
float electrical_power_dc(float voltage, float current);
float efficiency(float p_out_mech, float p_in_elec);

/* String conversion */
const char *motor_type_to_string(motor_type_t type);
const char *control_mode_to_string(control_mode_t mode);

/* Configuration initialization */
void pwm_config_init(pwm_config_t *cfg, float vdc);
void cascaded_control_cfg_init(cascaded_control_cfg_t *cfg, control_mode_t mode);
void observer_config_init(observer_config_t *cfg);

/* Fault checking */
bool check_overcurrent(const phase_currents_t *currents, float limit);
bool check_overtemperature(float temperature, float limit);
bool check_dc_bus_overvoltage(float vdc, float vdc_max);
bool check_dc_bus_undervoltage(float vdc, float vdc_min);

/* State summary */
void motor_state_summary(const motor_state_t *state);

#endif /* MOTOR_TYPES_H */
