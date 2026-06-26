# Course Alignment — mini-dcs-scada

## Nine-School Curriculum Mapping

### MIT
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| 6.302 Feedback System Design | PID tuning, stability margins, loop shaping | `pid_controller.c`: all tuning methods, gain/phase margins |
| 6.003 Signals and Systems | Fourier, Laplace, sampling | `signal_chain.c`: EMA, Butterworth, notch filters |
| 6.450 Digital Communications | Error correction, protocol design | `comm_protocol.c`: Hamming(7,4), Modbus CRC-16 |

### Stanford
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| EE392 Digital Control | Discrete PID, anti-windup, bumpless | `pid_controller.c`: anti-windup, manual/auto transfer |
| EE359 Wireless Communications | Wireless SCADA | `comm_protocol.c`: Hamming ECC for wireless |
| EE264 DSP | Digital filter design | `signal_chain.c`: Butterworth, notch filter design |

### Berkeley
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| EE16A/B Circuits | Sensor interfaces, 4-20mA | `signal_chain.c`: ADC model, 4-20mA scaling |
| EE123 Digital Signal Processing | FIR/IIR, bilinear transform | `signal_chain.c`: SMA, Butterworth IIR design |
| EE128 Feedback Control | Cascade, feedforward | `pid_controller.c`: cascade, feedforward |

### Illinois (UIUC)
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| ECE 310 DSP | Filter banks, multirate | `signal_chain.c`: EMA, SMA filter bank concepts |
| ECE 459 Communications | Industrial protocols | `comm_protocol.c`: Modbus, OPC UA |
| ECE 451 EM | EMI in industrial environments | Not directly covered |

### Michigan
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| EECS 351 DSP | Adaptive filtering | Not yet (gap: LMS adaptive filter) |
| EECS 455 Comm | Error correction | `comm_protocol.c`: Hamming(7,4) |
| EECS 411 Microwave | Not directly covered | — |

### Georgia Tech
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| ECE 4270 DSP | Statistical signal processing | `signal_chain.c`: Welford running statistics |
| ECE 6601 Comm | Error control coding | `comm_protocol.c`: Hamming, CRC |
| ECE 6350 EM | EMC in control systems | Not directly covered |

### TU Munich
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| Signal Processing | Industrial signal conditioning | `signal_chain.c`: full chain |
| Communications | Fieldbus, Profibus | `comm_protocol.c`: Modbus (foundation) |
| High-Frequency Eng. | Not directly covered | — |

### ETH Zurich
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| 227-0216 Control Systems II | Advanced PID, cascade | `pid_controller.c`: full cascade + FF |
| 227-0427 Signal Processing | Filter banks, wavelets | `signal_chain.c`: digital filters |
| 227-0436 Communications | Industrial networks | `comm_protocol.c`: Modbus, OPC UA |

### Tsinghua University
| Course | Topic | Module Coverage |
|--------|-------|----------------|
| 过程控制系统 (Process Control) | PID tuning, cascade, DCS architecture | Full module |
| 信号与系统 (Signals & Systems) | Sampling, filtering | `signal_chain.c` |
| 工业通信与现场总线 (Industrial Comm) | Modbus, OPC | `comm_protocol.c` |
| 通信原理 (Communication Principles) | Error correction | `comm_protocol.c`: Hamming, CRC |

## Reference Textbooks

| Textbook | Authors | Coverage in Module |
|----------|---------|-------------------|
| PID Controllers: Theory, Design, and Tuning | Astrom & Hagglund (1995) | Full PID implementation |
| Discrete-Time Signal Processing | Oppenheim & Schafer (2010) | Digital filter design |
| Digital Communications | Proakis & Salehi (2008) | Hamming, CRC |
| Alarm Management (ISA-18.2) | ISA (2016) | Full alarm lifecycle |
| Modbus Protocol Specification | Modbus Organization | Full Modbus implementation |
| Embedded Systems: RTOS for ARM Cortex-M | Valvano (2019) | Scan engine, real-time concepts |
