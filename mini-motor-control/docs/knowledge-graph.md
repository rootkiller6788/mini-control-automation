# Knowledge Graph ！ mini-motor-control

## L1: Definitions

| Entry | C Type/Define | Lean Type | Status |
|-------|---------------|-----------|--------|
| Motor types (DC/BLDC/PMSM/IM/Stepper/SRM) | `motor_type_t` enum | ！ | Complete |
| Winding types (Wye/Delta) | `winding_type_t` enum | ！ | Complete |
| Control modes (Torque/Speed/Position) | `control_mode_t` enum | ！ | Complete |
| Commutation methods (6-step/Sin/FOC/DTC/SVPWM) | `commutation_method_t` enum | ！ | Complete |
| Hall sensor sectors | `hall_sector_t` enum | ！ | Complete |
| PWM configuration | `pwm_config_t` struct | ！ | Complete |
| SVPWM timing | `svpwm_timing_t` struct | ！ | Complete |
| DC motor parameters | `dc_motor_params_t` struct | ！ | Complete |
| PMSM motor parameters | `pmsm_params_t` struct | ！ | Complete |
| Induction motor parameters | `induction_motor_params_t` struct | ！ | Complete |
| Stepper motor parameters | `stepper_params_t` struct | ！ | Complete |
| Three-phase currents | `phase_currents_t` struct | `ThreePhaseCurrents` | Complete |
| Clarke vector (alpha-beta-zero) | `clarke_vector_t` struct | `ClarkeComponents` | Complete |
| Park vector (d-q) | `park_vector_t` struct | `ParkComponents` | Complete |
| Motor runtime state | `motor_state_t` struct | ！ | Complete |
| Fault flags (13 types) | `#define FAULT_*` macros | ！ | Complete |
| Fault severity levels | `fault_severity_t` enum | ！ | Complete |
| PID gains | `pid_gains_t` struct | ！ | Complete |
| PID state (anti-windup) | `pid_state_t` struct | ！ | Complete |
| Cascaded control config | `cascaded_control_cfg_t` struct | ！ | Complete |
| Observer/SMO config | `observer_config_t` struct | ！ | Complete |

## L2: Core Concepts

| Concept | Implementation | Status |
|---------|---------------|--------|
| Field-Oriented Control (FOC) | `foc_current_control()` in `motor_control.c` | Complete |
| Scalar V/f control | `im_synchronous_speed()`, `im_torque_per_phase()` | Complete |
| Six-step commutation | `bldc_commutation_pattern()`, `hall_to_sector()` | Complete |
| Space Vector PWM | `svpwm_compute()`, `svpwm_determine_sector()` | Complete |
| Clarke/Park transforms | `clarke_transform()`, `park_transform()` | Complete |
| PID control with anti-windup | `pid_update()`, back-calculation method | Complete |
| Cascaded control loops | `speed_control()`, `position_control()` | Complete |
| Sensorless control | SMO, EKF, PLL, HFI | Complete |
| Dead-time compensation | `dead_time_compensation()` | Complete |
| Overmodulation | `overmodulation_mode1()`, `svpwm_auto_overmodulation()` | Complete |

## L3: Mathematical Structures

| Structure | Implementation | Status |
|-----------|---------------|--------|
| Clarke transform matrix (3x3) | `clarke_transform()`, amplitude-invariant | Complete |
| Inverse Clarke transform | `inverse_clarke_transform()` | Complete |
| Park rotation matrix (2x2) | `park_transform()`, `park_transform_direct()` | Complete |
| Inverse Park rotation | `inverse_park_transform()` | Complete |
| Combined abc★dq transform | `abc_to_dq()` | Complete |
| Combined dq★abc transform | `dq_to_abc()` | Complete |
| Vector magnitude in dq-plane | `dq_magnitude()` | Complete |
| Angle normalization | `normalize_angle()`, `normalize_angle_positive()` | Complete |
| Angle difference (periodic) | `angle_difference()` | Complete |
| Electrical ? mechanical angle conversion | `mechanical_to_electrical_angle()` | Complete |
| Three-phase balance check | `is_balanced()` | Complete |

## L4: Fundamental Laws

| Law | C Verification | Lean Theorem | Status |
|-----|---------------|-------------|--------|
| Kirchhoff's Voltage Law (DC motor) | `dc_motor_current_derivative()` | ！ | Complete |
| Faraday's Law (back-EMF) | `dc_motor_back_emf()` | ！ | Complete |
| Lorentz Force (torque production) | `dc_motor_torque_from_current()` | ！ | Complete |
| Newton's 2nd Law (rotation) | `dc_motor_speed_derivative()` | ！ | Complete |
| PMSM dq voltage equations | `pmsm_id_derivative()`, `pmsm_iq_derivative()` | ！ | Complete |
| PMSM torque equation | `pmsm_torque()` | `PMSM_Torque` structure | Complete |
| Clarke invertibility (balanced) | `test_clarke_roundtrip()` | `clarke_zero_vanishes_on_balance` | Complete |
| Park orthogonality | `test_park_orthogonality()` | (documented) | Complete |
| BLDC commutation group Z/6Z | `test_bldc_next_sector()` | `bldc_next_prev_inverse`, `bldc_order_six` | Complete |
| DC motor transfer function | `dc_motor_transfer_function()` | ！ | Complete |
| Speed-torque line (DC) | `dc_motor_steady_state_speed()` | `dcMotorSpeedTorqueLine` | Complete |
| Time constant separation (tau_e << tau_m) | `dc_motor_electrical_time_constant()` | `DC_Motor_TimeConstants` | Complete |
| Routh-Hurwitz stability (PID) | ！ | `routhHurwitzStable3` | Complete |
| SVPWM sector partition | `svpwm_determine_sector()` | `svpwmSectorFromCode` | Complete |

## L5: Algorithms/Methods

| Algorithm | Implementation | Complexity | Status |
|-----------|---------------|------------|--------|
| PID with back-calculation anti-windup | `pid_update()` | O(1) | Complete |
| Conditional integration anti-windup | Within `pid_update()` | O(1) | Complete |
| Filtered derivative (PID) | Low-pass filter in `pid_update()` | O(1) | Complete |
| SVPWM sector determination | `svpwm_determine_sector()` | O(1) | Complete |
| SVPWM dwell time calculation | `svpwm_calculate_timing()` | O(1) | Complete |
| SVPWM 7-segment symmetric pattern | `svpwm_timing_to_duty()` | O(1) | Complete |
| SVPWM overmodulation (Mode I & II) | `svpwm_auto_overmodulation()` | O(1) | Complete |
| Sinusoidal PWM (SPWM) | `sinusoidal_pwm_three_phase()` | O(1) | Complete |
| Third-harmonic injection PWM | `third_harmonic_injection()` | O(1) | Complete |
| DPWM0 (positive clamp) | `dpwm0_modulate()` | O(1) | Complete |
| DPWM1 (negative clamp) | `dpwm1_modulate()` | O(1) | Complete |
| DPWM2 (alternating clamp) | `dpwm2_modulate()` | O(1) | Complete |
| Trapezoidal speed ramp | `speed_ramp_trapezoidal()` | O(1) | Complete |
| S-curve position profile | `scurve_position_profile()` | O(1) | Complete |
| Sliding Mode Observer (SMO) | `smo_update()` | O(1) | Complete |
| PLL angle tracking | `pll_update()` | O(1) | Complete |
| Back-EMF ZC detection (BLDC) | `bemf_zero_cross_detect()` | O(1) | Complete |
| Flux estimator (voltage model) | `flux_estimator_voltage_model()` | O(1) | Complete |
| MTPA (Maximum Torque Per Ampere) | `pmsm_mtpa_id()` | O(1) | Complete |
| Field weakening control | `pmsm_field_weakening_id()` | O(1) | Complete |
| Decoupling feed-forward | `pmsm_decoupling_ff()` | O(1) | Complete |
| DC link current reconstruction | `current_reconstruct_from_dc_link()` | O(1) | Complete |
| Dead-time compensation | `dead_time_compensation()` | O(1) | Complete |
| DC bus ripple compensation | `dc_bus_ripple_compensation()` | O(1) | Complete |
| Spread-spectrum PWM | `spread_spectrum_pwm_frequency()` | O(1) | Complete |
| RK4 integration (DC motor) | `dc_motor_rk4_step()` | O(1) | Complete |
| RK4 integration (PMSM) | `pmsm_rk4_step()` | O(1) | Complete |

## L6: Canonical Problems

| Problem | Example File | Status |
|---------|-------------|--------|
| DC motor speed control | `examples/dc_motor_speed_control.c` (>100 lines) | Complete |
| BLDC six-step commutation | `examples/bldc_six_step.c` (>80 lines) | Complete |
| PMSM FOC simulation | `examples/pmsm_foc.c` (>130 lines) | Complete |

## L7: Applications

| Application | Relevance | Implementation |
|-------------|-----------|---------------|
| Industrial servo drives | Cascaded position/speed/current FOC | `foc_current_control()`, `speed_control()`, `position_control()` |
| Electric vehicle traction | Field weakening, MTPA for IPM motors | `pmsm_field_weakening_id()`, `pmsm_mtpa_id()` |
| Drone/quadrotor motors | BLDC with sensorless fast response | `smo_update()`, `bemf_zero_cross_detect()` |
| CNC machine tools | High-precision position control with S-curve | `scurve_position_profile()` |
| Tesla/SpaceX actuators | PMSM FOC with advanced observers | EKF + SMO + HFI combined |

## L8: Advanced Topics

| Topic | Implementation | Status |
|-------|---------------|--------|
| Sliding Mode Observer (SMO) | `smo_update()` with boundary layer | Complete |
| Extended Kalman Filter (EKF) | `ekf_predict()`, `ekf_update()`, `ekf_step()` | Complete |
| High-Frequency Injection (HFI) | `hfi_get_injection_voltage()`, `hfi_extract_position_error()` | Complete |
| Sensorless FOC (full chain) | SMO + PLL angle extraction | Complete |
| Adaptive control (gain scheduling) | MTPA curve + field weakening curve | Partial |

## L9: Research Frontiers

| Topic | Documentation | Implementation |
|-------|---------------|---------------|
| SiC/GaN wide-bandgap motor drives | `README.md` | Not implemented |
| AI-based auto-tuning for motor control | `README.md` | Not implemented |
| Predictive control (MPC) for PMSM | `README.md` | Not implemented |
