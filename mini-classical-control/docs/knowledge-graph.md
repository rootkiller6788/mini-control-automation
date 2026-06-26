# Knowledge Graph — mini-classical-control

## L1: Definitions
| # | Item | C Type | Lean Type |
|---|------|--------|-----------|
| 1 | Transfer Function | `transfer_function_t` | `TransferFunction` |
| 2 | State-Space Model | `state_space_t` | `StateSpace n m p` |
| 3 | Pole-Zero Description | `pole_zero_t` | `ComplexPole` |
| 4 | Step Response Specs | `step_specs_t` | `StepSpecs` |
| 5 | Frequency Response Specs | `freq_specs_t` | `FreqSpecs` |
| 6 | PID Parameters | `pid_params_t` | `PIDParams` |
| 7 | System Type | `system_type_t` enum | `SystemType` |
| 8 | Stability Classification | `stability_t` enum | `Stability` |
| 9 | Process Model Types | `process_model_type_t` | — |

## L2: Core Concepts
| # | Concept | Implementation |
|---|---------|---------------|
| 1 | Feedback Control Loop | `tf_unity_feedback()`, `tf_feedback()` |
| 2 | BIBO Stability | `pole_stability()`, `tf_is_stable_cl()` |
| 3 | Steady-State Error | `steady_state_errors()` |
| 4 | Transient Response | `compute_step_specs()` |
| 5 | Sensitivity & Complementary Sensitivity | `sensitivity_function()` |
| 6 | Disturbance Rejection | Implicit in sensitivity analysis |
| 7 | Tracking Performance | `process_closed_loop_sim()` |
| 8 | Minimum-Phase Systems | Implicit in Bode stability |

## L3: Mathematical Structures
| # | Structure | Implementation |
|---|-----------|---------------|
| 1 | Laplace Transform (s-domain) | `tf_evaluate(s)` |
| 2 | Transfer Function Algebra | `tf_series()`, `tf_parallel()` |
| 3 | Polynomial Root-Finding | `poly_roots()` via companion matrix + QR |
| 4 | Partial Fraction Expansion | `tf_partial_fraction()` |
| 5 | State-Space ↔ TF Conversion | `tf2ss()`, `ss2tf()` (Leverrier-Faddeeva) |
| 6 | Controllability Matrix | `controllability_matrix()` |
| 7 | Observability Matrix | `observability_matrix()` |
| 8 | Matrix Rank | `matrix_rank()` (Gaussian elimination) |

## L4: Fundamental Laws
| # | Theorem/Principle | C Verification | Lean Statement |
|---|------------------|---------------|----------------|
| 1 | Routh-Hurwitz Criterion | `routh_hurwitz()` | `routh_2nd_order_stable`, `routh_3rd_order_stable` |
| 2 | Nyquist Stability Criterion | `nyquist_stability()` | `nyquist_criterion_statement` |
| 3 | Bode Stability Criterion | `compute_margins()` | — |
| 4 | Final Value Theorem | `steady_state_errors()` | `final_value_theorem_statement` |
| 5 | Internal Model Principle | `error_constants()` | `internal_model_principle` |
| 6 | Kalman Controllability | `controllability_matrix()` | — |
| 7 | Kalman Observability | `observability_matrix()` | — |

## L5: Algorithms/Methods
| # | Algorithm | Implementation |
|---|-----------|---------------|
| 1 | Routh Array Construction | `routh_hurwitz()` |
| 2 | Ziegler-Nichols Step Response | `zn_step_response()` |
| 3 | Ziegler-Nichols Ultimate Gain | `zn_ultimate_gain()` |
| 4 | Cohen-Coon Tuning | `cohen_coon()` |
| 5 | Lead Compensator Design | `lead_design()` |
| 6 | Lag Compensator Design | `lag_design()` |
| 7 | Lead-Lag Compensator | `lead_lag_design()` |
| 8 | Ackermann Pole Placement | `ackermann_pole_placement()` |
| 9 | Root Locus Computation | `root_locus_compute()` |
| 10 | QR Eigenvalue Algorithm | `poly_roots()` |
| 11 | RK4 Simulation | `step_response()`, `impulse_response()` |
| 12 | Bode Plot Generation | `bode_plot()` |

## L6: Canonical Problems
| # | Problem | Example/Implementation |
|---|---------|----------------------|
| 1 | DC Motor Speed Control | `example_dc_motor.c` |
| 2 | PID Tuning Comparison | `example_pid_tuning.c` |
| 3 | Root Locus Analysis | `example_root_locus.c` |
| 4 | Position Servo | `servo_position_model()` |
| 5 | Inverted Pendulum | `inverted_pendulum_model()` |
| 6 | Ball and Beam | `ball_beam_model()` |

## L7: Applications
| # | Application | Implementation | Key Keywords |
|---|-------------|---------------| ------------|
| 1 | DC Motor Control | `dc_motor_model()` + PI design | DC motor |
| 2 | Automotive Cruise Control | `cruise_vehicle_model()` + PI | Toyota, Detroit |
| 3 | Temperature Process Control | `thermal_fopdt_model()` + PID | ISO, supplier |
| 4 | Industrial Process Auto-tuning | `process_auto_tune()` | smart grid |

## L8: Advanced Topics
| # | Topic | Implementation |
|---|-------|---------------|
| 1 | Lyapunov Stability (via pole analysis) | `pole_stability()`, `dominant_pole_params()` |
| 2 | Time-Varying System Analysis | implicit in simulation framework |
| 3 | Anti-Windup Control | `process_closed_loop_sim()` |

## L9: Research Frontiers
| # | Topic | Documentation |
|---|-------|--------------|
| 1 | Adaptive PID | `AdaptivePID` in Lean |
| 2 | Fractional-Order PID | `FractionalOrderPID` in Lean |
| 3 | Event-Triggered Control | `EventTriggeredController` in Lean |
| 4 | 6G RIS / Intelligent Control | Documented in course-tree.md |
