# Course Tree — mini-classical-control

## Prerequisites
```
Mathematics
├── Calculus (differentiation, integration, ODEs)
├── Linear Algebra (matrices, eigenvalues, rank)
├── Complex Analysis (complex numbers, poles, residues)
└── Laplace Transform (s-domain, transfer functions)
         ↓
mini-classical-control
├── Transfer Functions
│   ├── Series/Parallel/Feedback algebra
│   ├── Poles and Zeros
│   └── Partial Fraction Expansion
├── Stability Analysis
│   ├── Routh-Hurwitz Criterion
│   ├── Root Locus (Evans Rules)
│   ├── Nyquist Criterion
│   └── Bode Plot → Gain/Phase Margins
├── Controller Design
│   ├── PID (ZN, Cohen-Coon, Pole Placement)
│   ├── Lead/Lag Compensators
│   └── Pole Placement (Ackermann)
├── State-Space Methods
│   ├── SS ↔ TF Conversion
│   └── Controllability & Observability
└── Applications
    ├── DC Motor Control
    ├── Cruise Control
    ├── Temperature Control
    └── Position Servo
         ↓
mini-modern-control (LQR, Kalman Filter, Robust Control)
mini-pid-tuning (Advanced PID, auto-tuning, gain scheduling)
mini-intelligent-control (Fuzzy, Neural, Adaptive)
mini-motor-control (Field-oriented, sensorless, stepper)
mini-robot-control (Kinematics, dynamics, impedance)
```

## Research Frontiers (L9)
- **Adaptive PID**: Online gain adjustment via system identification
- **Fractional-Order PID**: Non-integer integration/differentiation orders
- **Event-Triggered Control**: Aperiodic updates for networked systems
- **6G RIS Control**: Reconfigurable intelligent surfaces for wireless
