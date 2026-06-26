# Course Dependency Tree — mini-dcs-scada

## Prerequisites (What You Need Before This Module)

```
mini-dcs-scada
├── Mathematics
│   ├── Calculus (ODE, integration, Laplace transform)
│   ├── Linear Algebra (state-space, matrices)
│   ├── Probability & Statistics (noise, random processes)
│   └── Numerical Methods (Newton-Raphson, trapezoidal integration)
│
├── Signals & Systems
│   ├── Fourier Transform (frequency domain analysis)
│   ├── Laplace Transform (transfer functions, stability)
│   ├── Z-Transform (discrete-time systems)
│   └── Sampling Theory (Nyquist-Shannon theorem)
│
├── Control Theory
│   ├── Feedback Systems (PID, stability criteria)
│   ├── Classical Control (root locus, Bode, Nyquist)
│   ├── Digital Control (discrete PID, ZOH, bilinear transform)
│   └── Process Control (FOPDT models, tuning rules)
│
├── Electronics
│   ├── Analog Electronics (op-amps, 4-20mA, ADC)
│   ├── Digital Electronics (logic, memory, MCU)
│   └── Sensor Technology (thermocouples, RTDs, pressure)
│
└── Computer Science
    ├── C Programming (structs, pointers, memory management)
    ├── Networking (TCP/IP, serial communication)
    ├── Data Structures (circular buffer, tables, trees)
    └── Real-Time Systems (scheduling, deadlines)
```

## Module Internal Dependencies

```
dcs_core.h/c          ← Base: tags, scan engine, actuators
    ↓
pid_controller.h/c    ← PID algorithms, tuning, cascade, FF
    ↓
signal_chain.h/c      ← ADC, filters, scaling, linearization
    ↓                   ↘
comm_protocol.h/c     ← Modbus, OPC, CRC, Hamming  → alarm_manager.h/c
    ↓                                                   ↓
data_logger.h/c       ← Historian, batch, CSV      event_logging, first-out
```

## Postrequisites (What This Module Enables)

```
mini-dcs-scada (this module)
├── Advanced Process Control (APC/MPC)
├── Safety Instrumented Systems (SIS, IEC 61511)
├── Industrial IoT / Industry 4.0
├── SCADA Security (NERC CIP, IEC 62443)
├── Process Optimization (RTO, data analytics)
├── Digital Twin Systems
└── Regulatory Compliance Systems (FDA, EPA)
```

## L9 Research Frontiers Dependencies

```
Digital Twin Research
├── mini-dcs-scada (this module)
├── Real-time data streaming (Kafka, MQTT)
├── Machine learning for anomaly detection
├── High-fidelity process simulation (CFD, FEM)
└── Cloud/edge computing architecture

AI-Based Control
├── mini-dcs-scada (PID foundation)
├── Reinforcement learning (policy gradient methods)
├── System identification (subspace, neural ODE)
└── Safe exploration (barrier functions, Lyapunov constraints)
```
