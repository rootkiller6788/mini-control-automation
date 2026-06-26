# Course Alignment ? mini-pid-tuning

## MIT ? 6.302 Feedback Systems

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| PID Control Fundamentals | L1 (PIDParams, PIDController), L2 (pid_update) |
| Root Locus Method | L3 (Polynomial), L4 (Routh-Hurwitz) |
| Frequency Response | L3 (FrequencyAnalysis), L4 (stability margins) |
| Nyquist Criterion | L4 (gain/phase margin) |
| Bode Plots | L3 (pid_loop_frequency_analysis) |
| Stability Margins | L4 (pid_compute_stability_margins) |
| Lead-Lag Compensation | Implicit in PID (derivative = lead, integral = lag) |

## Stanford ? EE267 Digital Control

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| Discrete-Time PID | L1 (Ts parameter), L2 (backward Euler/BDF) |
| Anti-Windup | L2 (3 methods: clamping, back-calc, combined) |
| Bumpless Transfer | L2 (pid_set_manual, pid_set_auto, pid_set_tracking) |
| Sampling Effects | L1 (Ts, N filter parameters) |
| Digital Implementation | src/pid_core.c ? full discrete-time implementation |

## Berkeley ? EE128 Feedback Control Systems

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| PID Tuning Methods | L5 (10 methods) |
| Ziegler-Nichols | L5 (open + closed loop) |
| Cohen-Coon | L5 |
| IMC Tuning | L5 (Rivera-Morari-Skogestad) |
| Cascade Control | L6 (CascadePID) |
| Feedforward | L6 (FeedforwardPID) |

## Caltech ? CDS 110 Introduction to Control

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| First-Order Systems | L1 (FOPDTModel) |
| Second-Order Systems | L1 (SOPDTModel) |
| Step Response | L6 (pid_simulate_step_response, StepResponseMetrics) |
| PID Tuning | L5 |
| Lyapunov Stability | L4 (pid_lyapunov_system_matrix, pid_solve_lyapunov) |

## ETH ? 227-0216 Control Systems II

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| Advanced PID Structures | L8 (Nonlinear, Fractional-Order, Event-Based) |
| Gain Scheduling | L6 (GainScheduledPID) |
| Adaptive Control | L8 (MRACPID) |
| Process Identification | L5 (tangent + area methods) |

## Cambridge ? 3F2 Control Systems

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| Classical Control Design | L1-L5 |
| Frequency Domain Design | L3-L4 |
| Routh-Hurwitz | L4 |
| PID Controller Implementation | L2 |

## TU Munich ? Regelungstechnik

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| PID-Regler | L1-L2 |
| Einstellregeln | L5 (ZN, Cohen-Coon, CHR) |
| St?rgr??enaufschaltung | L6 (FeedforwardPID) |
| Kaskadenregelung | L6 (CascadePID) |

## Tsinghua ? ??????

| Topic | mini-pid-tuning Coverage |
|-------|-------------------------|
| PID?? | L1-L5 |
| ???? | L5 (FOPDT, SOPDT identification) |
| ????? | L4 (Routh, Lyapunov, margins) |
| ???PID | L8 (NonlinearPID) |
