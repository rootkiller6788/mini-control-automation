# Knowledge Graph — mini-dcs-scada

## L1: Definitions (Complete)

| Entry | C Struct/Enum | File |
|-------|--------------|------|
| DCS Tag (ISA-88/95 primitive) | `dcs_tag_t` | include/dcs_core.h |
| Signal Quality (OPC UA) | `dcs_quality_t` | include/dcs_core.h |
| Engineering Units | `dcs_unit_t` | include/dcs_core.h |
| Control Block Types (ISA-5.1) | `dcs_block_type_t` | include/dcs_core.h |
| PID Controller Parameters (ISA Form) | `pid_controller_t` | include/pid_controller.h |
| PID Forms (ISA/Parallel/Series) | `pid_form_t` | include/pid_controller.h |
| FOPDT Process Model | `pid_fopdt_model_t` | include/pid_controller.h |
| Modbus Data Model | `modbus_data_model_t` | include/comm_protocol.h |
| Modbus ADU/PDU | `modbus_adu_t` | include/comm_protocol.h |
| Alarm State (ISA-18.2) | `alarm_state_t` | include/alarm_manager.h |
| Alarm Type/Priority | `alarm_type_t`, `alarm_priority_t` | include/alarm_manager.h |
| Data Sample | `data_sample_t` | include/data_logger.h |
| Batch Record (ISA-88) | `batch_record_t` | include/data_logger.h |
| ADC Model | `adc_model_t` | include/signal_chain.h |
| 4-20mA Current Loop | `ma_to_eu()`, `ma_is_valid()` | include/signal_chain.h |

## L2: Core Concepts (Complete)

| Concept | Implementation | File |
|---------|---------------|------|
| Feedback/PID Control Loop | `pid_update()` | src/pid_controller.c |
| Cascade Control | `pid_cascade_update()` | src/pid_controller.c |
| Feedforward Compensation | `pid_feedforward_update()` | src/pid_controller.c |
| Ratio/Split-Range Control | `BLOCK_RATIO`, `BLOCK_SPLIT_RANGE` | include/dcs_core.h |
| Scan Engine (Cyclic Execution) | `dcs_scan_engine_run()` | src/dcs_core.c |
| Tag Database (Process Image) | `dcs_tag_db_t` | src/dcs_core.c |
| Modbus RTU Protocol | `modbus_crc16()`, PDU builders | src/comm_protocol.c |
| OPC UA Variables | `opc_variable_t` | src/comm_protocol.c |
| Alarm Lifecycle (ISA-18.2) | `alarm_evaluate()` | src/alarm_manager.c |
| Alarm Flood Detection | `alarm_flood_detect()` | src/alarm_manager.c |
| First-Out Detection | `first_out_record()` | src/alarm_manager.c |
| Deadband & Hysteresis | `deadband_update()`, `hysteresis_update()` | src/signal_chain.c |
| Engineering Unit Scaling | `dcs_scale()`, `ma_to_eu()` | src/signal_chain.c |

## L3: Mathematical Structures (Complete)

| Structure | Implementation | File |
|-----------|---------------|------|
| Discrete-Time Signals (Z-domain) | `Signal` type in Lean | src/dcs_scada.lean |
| Difference Equations (FIR/IIR) | `ema_update()`, `butter2_update()` | src/signal_chain.c |
| Trapezoidal Integration | `pid_update()` integral term | src/pid_controller.c |
| Bilinear Transform (s-to-z) | `butter2_lp_design()` | src/signal_chain.c |
| Lead-Lag Compensation | `pid_feedforward_update()` | src/pid_controller.c |
| Linear Interpolation / Bilinear | `dcs_lerp()`, `table2d_lookup()` | src/signal_chain.c |
| Running Statistics (Welford) | `running_stats_push()` | src/signal_chain.c |
| Time-Weighted Average (Integral) | `time_weighted_average()` | src/data_logger.c |
| Linear Regression (Rate) | `rate_of_change()` | src/data_logger.c |

## L4: Fundamental Laws (Complete)

| Law/Theorem | C Implementation | Lean Formalization |
|-------------|-----------------|-------------------|
| Nyquist-Shannon Sampling Theorem | `nyquist_min_sample_rate()` | `nyquist_min_two_samples`, `nyquist_period_bound` |
| Routh-Hurwitz Stability Criterion | `pid_update()` (stable if Kc>0) | `rh_stable_example`, `rh_unstable_example` |
| Final Value Theorem (PI steady-state) | `pid_update()` integral term | `pi_eliminates_offset` |
| Hamming Code Minimum Distance | `hamming74_encode/decode()` | `hamming_perfect_decode` |
| Stability Margin (Gain/Phase) | `pid_tuning_t` margins | Cascade bandwidth separation theorem |
| CRC-16 Error Detection | `modbus_crc16()` | (arithmetic, not formalized) |

## L5: Algorithms/Methods (Complete)

| Algorithm | Implementation | File |
|-----------|---------------|------|
| Ziegler-Nichols Open-Loop Tuning | `pid_tune_zn_openloop()` | src/pid_controller.c |
| Ziegler-Nichols Closed-Loop Tuning | `pid_tune_zn_closedloop()` | src/pid_controller.c |
| Cohen-Coon Tuning | `pid_tune_cohen_coon()` | src/pid_controller.c |
| IMC/Lambda Tuning | `pid_tune_imc()` | src/pid_controller.c |
| Anti-Reset Windup (Conditional Integration) | `pid_update()` | src/pid_controller.c |
| Bumpless Transfer (Manual/Auto) | `pid_set_manual/auto()` | src/pid_controller.c |
| Gain Scheduling | `pid_gain_schedule_lookup()` | src/pid_controller.c |
| EMA/DEMA/SMA Filters | `ema_update()`, `dema_update()`, `sma_update()` | src/signal_chain.c |
| Butterworth 2nd-Order LP Design | `butter2_lp_design()` | src/signal_chain.c |
| Notch Filter (Mains Hum) | `notch_design()` | src/signal_chain.c |
| Thermocouple Linearization (ITS-90) | `thermocouple_k_temp()` | src/signal_chain.c |
| RTD Pt100 (Callendar-Van Dusen) | `rtd_pt100_temp()` | src/signal_chain.c |
| Swinging Door Compression | `swinging_door_should_archive()` | src/comm_protocol.c |
| Hamming(7,4) Error Correction | `hamming74_encode/decode()` | src/comm_protocol.c |

## L6: Canonical Problems (Complete)

| Problem | Example/Implementation | File |
|---------|----------------------|------|
| Flow Control Loop (FOPDT) | `demo_control_loop.c` | demos/ |
| Temperature Control (CSTR) | `example_chemical_reactor.c` | examples/ |
| Level Control (Clear Well) | `example_water_treatment.c` | examples/ |
| Frequency Regulation (AGC) | `example_power_grid.c` | examples/ |
| Cascade Loop Tuning | `pid_cascade_init/update()` | src/pid_controller.c |
| Alarm Rationalization (ISA-18.2) | `alarm_evaluate()` state machine | src/alarm_manager.c |

## L7: Applications (Complete — 3 applications)

| Application | Example File | Keywords |
|-------------|-------------|----------|
| Municipal Water Treatment SCADA | `example_water_treatment.c` | Detroit, EPA, turbidity, chlorine |
| Chemical Reactor DCS (Pharma) | `example_chemical_reactor.c` | FDA 21CFR11, F-35 supplier, ISA-88 |
| Power Grid SCADA / AGC | `example_power_grid.c` | NERC, ISO, smart grid, 60Hz |

## L8: Advanced Topics (Partial — 5 topics)

| Topic | Implementation |
|-------|---------------|
| Gain Scheduling (Nonlinear PID) | `pid_gain_schedule_t` — parameter-varying control |
| Fuzzy Logic Control | `FuzzySet`, `fuzzify()`, `ruleOutput()` in Lean |
| Model Predictive Control (concept) | Documented in course-alignment.md |
| Digital Twin Synchronization | `DigitalTwin`, `syncError` theorems in Lean |
| Time-Varying Process Control | Feedforward with lead-lag compensation |

## L9: Research Frontiers (Partial — documented)

| Frontier | Formalization |
|----------|--------------|
| Digital Twin for Process Industries | `DigitalTwin` structure + synchronization proof |
| AI-Based PID Auto-Tuning | Documented in gap-report.md |
| Cyber-Physical SCADA Security | Documented in gap-report.md |
| Edge Computing in SCADA | Documented, not implemented |
