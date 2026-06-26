# Course Tree — mini-plc-automation

Prerequisite dependency tree for PLC automation knowledge.

```
PLC Automation
├── Digital Logic (Boolean algebra, gates)
│   └── Electrical Circuits (relay logic origin)
├── Computer Architecture (CPU, memory, I/O)
│   └── Real-Time Systems (scheduling, deadlines)
├── Control Theory
│   ├── PID Control (proportional, integral, derivative)
│   ├── Ziegler-Nichols Tuning (open-loop & closed-loop)
│   └── Sampled-Data Control (discrete-time approximation)
├── Signal Processing
│   ├── Sampling Theory (Nyquist-Shannon)
│   └── Digital Filtering (IIR, FIR, moving average)
├── Industrial Communication
│   ├── Modbus (RTU, TCP, CRC-16)
│   └── Fieldbus Concepts (Profinet, EtherCAT — referenced)
├── Functional Safety
│   ├── IEC 61508 (SIL, PFD, redundancy)
│   └── Safety PLC Architecture
└── Software Engineering
    ├── IEC 61131-3 Languages (LD, FBD, ST, IL, SFC)
    ├── Parsing & Compilation (lexer, parser, AST)
    └── Graph Algorithms (topological sort, DAG)
```

## Immediate Prerequisites (must know before this module)
1. Boolean algebra and logic gates
2. Basic control theory (feedback, PID)
3. C programming and data structures

## Leads To (modules that depend on this)
1. mini-dcs-scada — Distributed Control Systems
2. mini-industrial-fieldbus — Industrial Communication Protocols
3. mini-motor-control — Motor Drive Control
4. mini-robot-control — Robot Control Systems
