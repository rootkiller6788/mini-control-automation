# Mini Control & Automation

A collection of **from-scratch, zero-dependency C implementations** covering control theory and industrial automation—from classical PID tuning and state-space methods to intelligent control, motor drives, PLC programming, DCS/SCADA systems, and robot kinematics/dynamics. Each module maps to MIT, Stanford, and other top-tier university courses, translating textbook equations and industrial standards into runnable C code.

## Sub-Modules

| Sub-Module | Topics | Key Courses |
|------------|--------|-------------|
| [mini-classical-control](mini-classical-control/) | Transfer functions, state-space models, Laplace domain, pole-zero analysis, Routh-Hurwitz criterion, Nyquist/Bode stability, root locus, PID compensator design, DC motor/cruise/temperature/position servo control | MIT 6.302, MIT 6.241J |
| [mini-dcs-scada](mini-dcs-scada/) | Modbus RTU/TCP, OPC UA, IEC 61158 fieldbus, ISA-18.2 alarm management (debounce, flood suppression, shelving), EEMUA 191, process data historian (trend analysis, 21 CFR Part 11), industrial communication protocols | MIT 6.02, Stanford EE379 |
| [mini-intelligent-control](mini-intelligent-control/) | Fuzzy logic control, neural network control, adaptive control (MRAC, STR), model predictive control (MPC), reinforcement learning (Q-learning, SARSA), sliding mode control, iterative learning control, genetic algorithms | MIT 6.885, Stanford CS229 |
| [mini-modern-control](mini-modern-control/) | State-space models (continuous/discrete/sampled), controllability & observability, LQR, Kalman filter, pole placement, Lyapunov stability, Ackermann's formula, Luenberger observer, separation principle | MIT 6.241J, MIT 16.30, Stanford EE363 |
| [mini-motor-control](mini-motor-control/) | DC motor / PMSM / induction / stepper motor models, Park/Clarke transforms, FOC vector control, PID with anti-windup, cascaded current/speed/position loops, MTPA, field weakening, BLDC six-step commutation, Hall sensor decoding | MIT 6.685, Berkeley EE117, ETH 227-0216 |
| [mini-pid-tuning](mini-pid-tuning/) | PID structures (series/parallel/ideal), Ziegler-Nichols & Cohen-Coon tuning, cascade control, feedforward, gain scheduling, ratio/override control, adaptive PID, fuzzy PID, fractional-order PID, Bode/Nyquist stability margins, autotuning | MIT 6.302, Stanford EE207 |
| [mini-plc-automation](mini-plc-automation/) | IEC 61131-3 (Ladder LD, FBD, ST, SFC, IL), PLC scan cycle (input scan → program exec → output scan), digital I/O, timers/counters, DAG-based FBD evaluation, task scheduling, industrial automation patterns | MIT 6.302 |
| [mini-robot-control](mini-robot-control/) | Forward/inverse kinematics (DH parameters), differential kinematics (Jacobian, pseudoinverse), Lagrangian & Newton-Euler dynamics, PD+gravity, computed torque control (CTC), impedance control, adaptive manipulator control, trajectory generation | Stanford CS223A, MIT 6.834 |

## Design Philosophy

- **Zero external dependencies** — pure C (C99/C11), only `libc` and `libm`
- **Self-contained modules** — each directory has its own `Makefile`, `include/`, `src/`, `examples/`, `demos/`, `tests/`
- **Theory-to-code mapping** — every module maps directly to university-level control theory textbooks and industrial standards (IEC, ISA)
- **Practical applications** — motor drive simulators, PLC runtime engines, SCADA historians, robot trajectory planners, PID autotuners, and more

## Building

Each module is standalone. Navigate to a module directory and run:

```bash
cd mini-classical-control
make all    # build everything
make test   # run tests
```

Requires **GCC** and **GNU Make**.

## Project Structure

```
mini-control-automation/
├── mini-classical-control/     # Classical Control Theory (Laplace, Bode, Nyquist, Root Locus)
├── mini-dcs-scada/             # DCS/SCADA — Industrial Communication, Alarms, Data Historian
├── mini-intelligent-control/   # Intelligent Control — Fuzzy, Neural, Adaptive, MPC, RL, SMC
├── mini-modern-control/        # Modern Control — State-Space, LQR, Kalman, Observers
├── mini-motor-control/         # Motor Control — FOC, PMSM, BLDC, Cascaded Loops
├── mini-pid-tuning/            # PID Tuning — Advanced Structures, Analysis, Autotuning
├── mini-plc-automation/        # PLC Automation — IEC 61131-3, LD, FBD, ST, Scan Cycle
└── mini-robot-control/         # Robot Control — Kinematics, Dynamics, CTC, Impedance
```

## License

MIT
