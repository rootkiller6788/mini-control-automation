/**
 * @file motor_types.c
 * @brief Motor type utility functions: parameter initialization, validation, fault handling
 *
 * L1 Definitions ¡ª utility functions for motor parameter structs
 * and motor state management, including:
 *   - Default parameter initialization for common motor types
 *   - Parameter validation and sanity checking
 *   - Fault flag management
 *   - Motor state initialization and reset
 *   - Unit conversion utilities (RPM <-> rad/s, etc.)
 */

#include "motor_types.h"
#include <string.h>

/* ========================================================================
 * L1: Default Parameter Initialization
 *
 * These functions provide reasonable default values for common
 * motor types. Values are based on typical small-to-medium-sized
 * industrial motors. For production use, load parameters from
 * a motor datasheet or identification procedure.
 * ======================================================================== */

void dc_motor_params_init(dc_motor_params_t *params)
{
    if (!params) return;
    memset(params, 0, sizeof(dc_motor_params_t));
    /* Typical 24V, 100W DC brushed motor defaults */
    params->nominal_voltage = 24.0f;
    params->nominal_current = 5.0f;
    params->winding_resistance = 0.5f;
    params->winding_inductance = 0.002f;
    params->ke_back_emf = 0.05f;       /* V / (rad/s) */
    params->kt_torque = 0.05f;         /* N*m / A (¡Ö Ke for SI units) */
    params->rotor_inertia = 0.0001f;   /* kg*m^2 */
    params->friction_coefficient = 0.00001f;
    params->pole_pairs = 1.0f;
    params->max_speed = 500.0f;        /* rad/s */
}

void pmsm_params_init(pmsm_params_t *params)
{
    if (!params) return;
    memset(params, 0, sizeof(pmsm_params_t));
    /* Typical 200W, 3000 RPM, 4-pole PMSM defaults (similar to a small servo) */
    params->rs = 0.3f;
    params->ld = 0.001f;
    params->lq = 0.0012f;
    params->ls = (params->ld + params->lq) / 2.0f;
    params->flux_linkage = 0.05f;      /* Wb */
    params->pole_pairs = 2.0f;
    params->rotor_inertia = 0.00005f;
    params->friction_coefficient = 0.000005f;
    params->rated_current = 4.0f;
    params->rated_speed = 314.0f;      /* 3000 RPM in rad/s */
    params->rated_torque = 0.64f;      /* N*m */
    params->max_dc_voltage = 48.0f;
}

void induction_motor_params_init(induction_motor_params_t *params)
{
    if (!params) return;
    memset(params, 0, sizeof(induction_motor_params_t));
    /* Typical 750W, 4-pole, 50Hz induction motor defaults */
    params->rs = 5.0f;
    params->rr = 3.0f;
    params->lls = 0.01f;
    params->llr = 0.01f;
    params->lm = 0.3f;
    params->pole_pairs = 2.0f;
    params->rotor_inertia = 0.002f;
    params->rated_power = 750.0f;
    params->rated_frequency = 50.0f;
    params->rated_slip = 0.04f;
}

void stepper_params_init(stepper_params_t *params)
{
    if (!params) return;
    memset(params, 0, sizeof(stepper_params_t));
    /* Typical NEMA17 stepper defaults */
    params->winding_resistance = 1.5f;
    params->winding_inductance = 0.003f;
    params->holding_torque = 0.45f;
    params->detent_torque = 0.02f;
    params->steps_per_rev = 200;
    params->rated_current = 1.7f;
    params->rotor_inertia = 0.000005f;
}

/* ========================================================================
 * L1: Motor Parameter Validation
 *
 * Performs sanity checks on motor parameters. Returns the number
 * of invalid fields found. A value of 0 means all checks passed.
 * ======================================================================== */

int dc_motor_params_validate(const dc_motor_params_t *params)
{
    int errors = 0;
    if (!params) return -1;
    if (params->winding_resistance <= 0.0f) errors++;
    if (params->winding_inductance <= 0.0f) errors++;
    if (params->ke_back_emf <= 0.0f) errors++;
    if (params->kt_torque <= 0.0f) errors++;
    if (params->rotor_inertia <= 0.0f) errors++;
    if (params->pole_pairs <= 0.0f) errors++;
    return errors;
}

int pmsm_params_validate(const pmsm_params_t *params)
{
    int errors = 0;
    if (!params) return -1;
    if (params->rs <= 0.0f) errors++;
    if (params->ld <= 0.0f) errors++;
    if (params->lq <= 0.0f) errors++;
    if (params->flux_linkage <= 0.0f) errors++;
    if (params->pole_pairs <= 0.0f) errors++;
    if (params->rotor_inertia <= 0.0f) errors++;
    return errors;
}

/* ========================================================================
 * L1: Motor State Initialization
 * ======================================================================== */

void motor_state_init(motor_state_t *state)
{
    if (!state) return;
    memset(state, 0, sizeof(motor_state_t));
    state->dc_bus_voltage = 24.0f;  /* Assume typical DC bus */
    state->temperature_windings = 25.0f;  /* Ambient */
}

void motor_state_reset_faults(motor_state_t *state)
{
    if (!state) return;
    state->fault_flags = 0;
}

/* ========================================================================
 * L1: Fault Management
 * ======================================================================== */

bool motor_has_fault(const motor_state_t *state)
{
    if (!state) return true;  /* Null state treated as faulted */
    return state->fault_flags != 0;
}

bool motor_has_critical_fault(const motor_state_t *state)
{
    if (!state) return true;
    /* Critical faults require immediate stop */
    uint32_t critical_mask = FAULT_OVERCURRENT | FAULT_OVERTEMPERATURE
                           | FAULT_DESATURATION | FAULT_PHASE_LOSS;
    return (state->fault_flags & critical_mask) != 0;
}

fault_severity_t motor_fault_severity(const motor_state_t *state)
{
    if (!state || state->fault_flags == 0) return FAULT_SEV_WARNING;

    if (state->fault_flags & (FAULT_OVERCURRENT | FAULT_DESATURATION
                            | FAULT_GROUND_FAULT)) {
        return FAULT_SEV_EMERGENCY;
    }
    if (state->fault_flags & (FAULT_OVERTEMPERATURE | FAULT_PHASE_LOSS
                            | FAULT_STALL_DETECTED)) {
        return FAULT_SEV_CRITICAL;
    }
    if (state->fault_flags & (FAULT_OVERVOLTAGE | FAULT_UNDERVOLTAGE
                            | FAULT_ENCODER_FAULT | FAULT_HALL_SEQUENCE)) {
        return FAULT_SEV_LIMITED;
    }
    return FAULT_SEV_WARNING;
}

void motor_set_fault(motor_state_t *state, uint32_t fault_bit)
{
    if (!state) return;
    state->fault_flags |= fault_bit;
}

void motor_clear_fault(motor_state_t *state, uint32_t fault_bit)
{
    if (!state) return;
    state->fault_flags &= ~fault_bit;
}

/* ========================================================================
 * L1: Unit Conversion Utilities
 * ======================================================================== */

float rpm_to_rad_per_sec(float rpm)
{
    return rpm * 2.0f * (float)M_PI / 60.0f;
}

float rad_per_sec_to_rpm(float rad_per_sec)
{
    return rad_per_sec * 60.0f / (2.0f * (float)M_PI);
}

float deg_to_rad(float degrees)
{
    return degrees * (float)M_PI / 180.0f;
}

float rad_to_deg(float radians)
{
    return radians * 180.0f / (float)M_PI;
}

float mechanical_power(float torque, float speed_rad_per_sec)
{
    /* P_mech = T * omega [W] */
    return torque * speed_rad_per_sec;
}

float electrical_power_dc(float voltage, float current)
{
    return voltage * current;
}

float efficiency(float p_out_mech, float p_in_elec)
{
    if (p_in_elec <= 0.0f) return 0.0f;
    return p_out_mech / p_in_elec;
}

/* ========================================================================
 * L1: Motor Type String Conversion (for diagnostics)
 * ======================================================================== */

const char *motor_type_to_string(motor_type_t type)
{
    switch (type) {
        case MOTOR_DC_BRUSHED:      return "DC Brushed";
        case MOTOR_DC_BRUSHLESS:    return "BLDC (Brushless DC)";
        case MOTOR_PMSM:            return "PMSM (Synchronous)";
        case MOTOR_INDUCTION:       return "Induction Motor";
        case MOTOR_STEPPER:         return "Stepper";
        case MOTOR_SRM:             return "Switched Reluctance";
        case MOTOR_SYNCHRONOUS_REL: return "Synchronous Reluctance";
        default:                    return "Unknown";
    }
}

const char *control_mode_to_string(control_mode_t mode)
{
    switch (mode) {
        case CONTROL_MODE_TORQUE:   return "Torque";
        case CONTROL_MODE_SPEED:    return "Speed";
        case CONTROL_MODE_POSITION: return "Position";
        default:                    return "Unknown";
    }
}

/* ========================================================================
 * L1: PWM Configuration Initialization
 * ======================================================================== */

void pwm_config_init(pwm_config_t *cfg, float vdc)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(pwm_config_t));
    cfg->pwm_frequency_hz = 16000;     /* 16 kHz typical */
    cfg->dead_time_ns = 1000;          /* 1 us dead time */
    cfg->timer_period = 4500;          /* For 72 MHz timer @ 16 kHz */
    cfg->dc_bus_voltage = vdc;
    cfg->complementary_mode = true;
}

/* ========================================================================
 * L1: Cascaded Control Configuration Initialization
 * ======================================================================== */

void cascaded_control_cfg_init(cascaded_control_cfg_t *cfg, control_mode_t mode)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(cascaded_control_cfg_t));

    /* Default current loop PI gains (tuned for 1 kHz bandwidth) */
    cfg->current_iq_gains.kp = 10.0f;
    cfg->current_iq_gains.ki = 500.0f;
    cfg->current_iq_gains.kd = 0.0f;
    cfg->current_iq_gains.n_coeff = 1000.0f;
    cfg->current_iq_gains.out_min = -48.0f;
    cfg->current_iq_gains.out_max = 48.0f;
    cfg->current_iq_gains.integral_limit = 48.0f;

    cfg->current_id_gains = cfg->current_iq_gains;  /* Same gains for d-axis */

    /* Default speed loop PI gains (tuned for 100 Hz bandwidth) */
    cfg->speed_gains.kp = 0.5f;
    cfg->speed_gains.ki = 10.0f;
    cfg->speed_gains.kd = 0.0f;
    cfg->speed_gains.n_coeff = 100.0f;
    cfg->speed_gains.out_min = -10.0f;
    cfg->speed_gains.out_max = 10.0f;
    cfg->speed_gains.integral_limit = 10.0f;

    /* Default position loop P gain */
    cfg->position_gains.kp = 50.0f;
    cfg->position_gains.ki = 0.0f;
    cfg->position_gains.kd = 0.0f;
    cfg->position_gains.n_coeff = 10.0f;
    cfg->position_gains.out_min = -500.0f;
    cfg->position_gains.out_max = 500.0f;
    cfg->position_gains.integral_limit = 500.0f;

    cfg->active_mode = mode;
    cfg->speed_ramp_rate = 500.0f;   /* rad/s^2 */
    cfg->current_limit = 10.0f;       /* A */
}

/* ========================================================================
 * L1: Observer Configuration Initialization
 * ======================================================================== */

void observer_config_init(observer_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(observer_config_t));
    cfg->gain_k = 200.0f;
    cfg->filter_cutoff = 1000.0f;     /* rad/s (~160 Hz) */
    cfg->pll_kp = 0.5f;
    cfg->pll_ki = 10.0f;
    cfg->min_speed = 5.0f;            /* rad/s (~50 RPM) */
    cfg->enable_hfi = true;
    cfg->hfi_frequency = 500.0f;      /* Hz */
    cfg->hfi_amplitude = 5.0f;       /* V */
}

/* ========================================================================
 * L1: Motor State Summary (for diagnostics/logging)
 * ======================================================================== */

void motor_state_summary(const motor_state_t *state)
{
    if (!state) return;
    /* This function populates the state with derived quantities
       that can be computed from the raw measurements. Used for
       monitoring and diagnostics. */
    /* No-op for now - state is maintained by control loops */
    (void)state;  /* State is valid, suppress unused warning */
}

/* ========================================================================
 * L1: Overcurrent Detection
 * ======================================================================== */

bool check_overcurrent(const phase_currents_t *currents, float limit)
{
    if (!currents) return false;
    float mag_a = fabsf(currents->ia);
    float mag_b = fabsf(currents->ib);
    float mag_c = fabsf(currents->ic);
    return (mag_a > limit || mag_b > limit || mag_c > limit);
}

/* ========================================================================
 * L1: Overtemperature Detection
 * ======================================================================== */

bool check_overtemperature(float temperature, float limit)
{
    return temperature > limit;
}

/* ========================================================================
 * L1: DC Bus Voltage Monitoring
 * ======================================================================== */

bool check_dc_bus_overvoltage(float vdc, float vdc_max)
{
    return vdc > vdc_max;
}

bool check_dc_bus_undervoltage(float vdc, float vdc_min)
{
    return vdc < vdc_min;
}
