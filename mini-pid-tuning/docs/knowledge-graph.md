# Knowledge Graph ? mini-pid-tuning

## L1: Definitions (Complete ?)

| # | Concept | C Definition | Location |
|---|---------|-------------|----------|
| 1 | PID Form (Parallel/Standard/Series) | `PIDForm` enum | include/pid_core.h |
| 2 | PID Controller Mode | `PIDMode` (AUTO/MANUAL/TRACKING) | include/pid_core.h |
| 3 | Derivative Action Mode | `PIDDerivativeMode` | include/pid_core.h |
| 4 | Anti-Windup Method | `PIDAntiWindup` (4 methods) | include/pid_core.h |
| 5 | PID Parameters | `PIDParams` struct | include/pid_core.h |
| 6 | PID Internal State | `PIDState` struct | include/pid_core.h |
| 7 | PID Controller Object | `PIDController` struct | include/pid_core.h |
| 8 | FOPDT Model | `FOPDTModel` struct | include/pid_tuning.h |
| 9 | SOPDT Model | `SOPDTModel` struct | include/pid_tuning.h |
| 10 | Step Response Data | `StepResponseData` struct | include/pid_tuning.h |
| 11 | Ultimate Gain Data | `UltimateGainData` struct | include/pid_tuning.h |
| 12 | Tuning Result | `PIDTuningResult` struct | include/pid_tuning.h |
| 13 | Transfer Function Polynomial | `PIDTransferFunction` struct | include/pid_core.h |
| 14 | Polynomial | `Polynomial` struct | include/pid_analysis.h |
| 15 | Routh Array | `RouthArray` struct | include/pid_analysis.h |
| 16 | Frequency Analysis | `FrequencyAnalysis` struct | include/pid_analysis.h |
| 17 | Step Response Metrics | `StepResponseMetrics` struct | include/pid_analysis.h |
| 18 | DC Motor Model | `DCMotorModel` struct | include/pid_applications.h |
| 19 | Thermal Model | `ThermalModel` struct | include/pid_applications.h |
| 20 | Quadrotor Axis Model | `QuadrotorAxisModel` struct | include/pid_applications.h |
| 21 | Tank Model | `TankModel` struct | include/pid_applications.h |
| 22 | Inverted Pendulum Model | `InvertedPendulumModel` struct | include/pid_applications.h |
| 23 | PID Gains (Lean) | `PIDGains` structure | src/pid_lean.lean |
| 24 | Transfer Function (Lean) | `TransferFunction` structure | src/pid_lean.lean |

## L2: Core Concepts (Complete ?)

| # | Concept | Implementation |
|---|---------|---------------|
| 1 | Proportional Action | `pid_update()` ? Kp * error term |
| 2 | Integral Action | Backward Euler integration in `pid_update()` |
| 3 | Derivative Action | Backward difference in `pid_update()` |
| 4 | Feedback Loop | PID + process in closed-loop |
| 5 | Anti-Windup (Clamping) | Conditional integration clamping |
| 6 | Anti-Windup (Back-Calculation) | Tracking-based correction |
| 7 | Anti-Windup (Combined) | Both methods combined |
| 8 | Bumpless Transfer | Manual ? Auto integrator preloading |
| 9 | Derivative Filtering | First-order low-pass on D-term |
| 10 | Setpoint Weighting (2-DOF) | b, c weights for P and D terms |
| 11 | Output Saturation | Hard limits on control signal |
| 12 | Tracking Mode | External signal following |
| 13 | PID Form Conversion | Parallel ? Standard ? Series |
| 14 | Transfer Function Representation | Symbolic string output |
| 15 | Lean: Steady-state error elimination | Theorem: PI eliminates steady-state error |

## L3: Mathematical Structures (Complete ?)

| # | Structure | Implementation |
|---|-----------|---------------|
| 1 | Laplace Transform (TF) | `PIDTransferFunction` + polynomial evaluation |
| 2 | Frequency Response | `pid_frequency_response()` ? |G(j?)|, ?G(j?) |
| 3 | Polynomial Operations | `Polynomial` struct + evaluation |
| 4 | Bode Plot Data | `FrequencyAnalysis` with log-spaced frequencies |
| 5 | Sensitivity Functions | S(j?), T(j?) computation |
| 6 | Gain/Phase Margins | Crossover frequency interpolation |
| 7 | Bandwidth | -3dB closed-loop bandwidth |
| 8 | Lean: Polynomial type | `Polynomial` in Lean 4 |
| 9 | Lean: Transfer function algebra | Series/feedback operations |

## L4: Fundamental Laws (Complete ?)

| # | Theorem/Law | Implementation |
|---|------------|---------------|
| 1 | Routh-Hurwitz Criterion | `pid_routh_construct()` + `pid_routh_is_stable()` |
| 2 | Nyquist Stability Criterion | Implicit in frequency analysis (gain/phase margins) |
| 3 | Bode Stability Criterion | `pid_compute_stability_margins()` |
| 4 | Lyapunov Stability | `pid_lyapunov_system_matrix()` + `pid_solve_lyapunov()` |
| 5 | Sylvester's Criterion | `pid_is_positive_definite()` via Cholesky |
| 6 | Final Value Theorem | `pi_eliminates_steady_state_error` theorem |
| 7 | Lean: Routh-Hurwitz (orders 1-3) | `first_order_stable`, `second_order_stable_condition`, `third_order_stable_condition` |
| 8 | Lean: Lyapunov ? Stability | `lyapunov_implies_stable_2x2` theorem |
| 9 | Lean: Lyapunov equation statement | Formal definition of A'P + PA + Q = 0 |

## L5: Algorithms/Methods (Complete ?)

| # | Algorithm | Implementation |
|---|-----------|---------------|
| 1 | FOPDT Identification (Tangent) | `pid_identify_fopdt()` |
| 2 | FOPDT Identification (Area) | `pid_identify_fopdt_area()` |
| 3 | SOPDT Identification (Two-Point) | `pid_identify_sopdt()` |
| 4 | Ultimate Gain (Newton-Raphson) | `pid_compute_ultimate_gain()` |
| 5 | Relay Auto-Tuning | `pid_relay_autotune()` |
| 6 | Ziegler-Nichols (Closed-Loop) | `pid_tune_zn_closed_loop()` |
| 7 | Ziegler-Nichols (Open-Loop) | `pid_tune_zn_open_loop()` |
| 8 | Cohen-Coon | `pid_tune_cohen_coon()` |
| 9 | Tyreus-Luyben | `pid_tune_tyreus_luyben()` |
| 10 | AMIGO | `pid_tune_amigo()` |
| 11 | IMC-Based | `pid_tune_imc()` |
| 12 | Lambda (Dahlin) | `pid_tune_lambda()` |
| 13 | CHR Setpoint | `pid_tune_chr_setpoint()` |
| 14 | CHR Disturbance | `pid_tune_chr_disturbance()` |
| 15 | Auto-Tuning Selection | `pid_autotune()` |
| 16 | Step Response Simulation | `pid_simulate_step_response()` |
| 17 | Performance Scoring | `pid_performance_score()` |
| 18 | Lean: ZN vs TL aggressiveness | `zn_more_aggressive_than_tl`, `zn_faster_integral_than_tl` |

## L6: Canonical Problems (Complete ?)

| # | Problem | Example |
|---|---------|---------|
| 1 | DC Motor Speed Control | `examples/example_dc_motor.c` |
| 2 | Temperature Control (Water Bath) | `examples/example_temperature.c` |
| 3 | Quadrotor Attitude Control | `examples/example_quadrotor.c` |
| 4 | Liquid Level Control | `tank_simulate()` in pid_applications.c |
| 5 | Inverted Pendulum | `inv_pendulum_simulate()` in pid_applications.c |
| 6 | Cascade Control | `cascade_pid_update()` in pid_advanced.c |
| 7 | Feedforward Control | `feedforward_pid_update()` in pid_advanced.c |
| 8 | Ratio Control | `ratio_pid_update()` in pid_advanced.c |

## L7: Applications (Complete ?, 5 domains)

| # | Application | Implementation |
|---|------------|---------------|
| 1 | Industrial Process Control | Tank level, temperature, flow control |
| 2 | Automotive Cruise Control | DC motor = throttle actuator model |
| 3 | Drone Stabilization | Quadrotor attitude cascade PID |
| 4 | HVAC Control | Thermal model + PID temperature regulation |
| 5 | Chemical Reactor | Thermal process with disturbance rejection |

## L8: Advanced Topics (Complete ?, 5 methods)

| # | Topic | Implementation |
|---|-------|---------------|
| 1 | Nonlinear PID | `NonlinearPID` + 3 gain functions (tanh, deadzone, quadratic) |
| 2 | Fractional-Order PID (FO-PID) | `FractionalOrderPID` with Gr?nwald-Letnikov |
| 3 | Event-Based PID | `EventBasedPID` with send-on-delta |
| 4 | MRAC Adaptive PID | `MRACPID` with MIT rule adaptation |
| 5 | Gain Scheduling | `GainScheduledPID` with linear interpolation |
| 6 | Lean: Event-based update bound | `event_based_update_bound` theorem |
| 7 | Lean: Event-based minimum inter-update | `event_based_min_interupdate` theorem |

## L9: Research Frontiers (Partial)

| # | Topic | Status |
|---|-------|--------|
| 1 | RL-Based PID Tuning | Documented, not implemented |
| 2 | Neural PID Controllers | Documented, not implemented |
| 3 | Digital Twin-Based Tuning | Documented, not implemented |
| 4 | 6G RIS Control | Documented, not implemented |
