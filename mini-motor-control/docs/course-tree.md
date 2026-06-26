# Course Tree ¡ª mini-motor-control

## Prerequisites (what this module depends on)

```
Control Theory (PID, stability, state-space)
©À©¤©¤ MIT 6.302: Feedback Systems
©À©¤©¤ Stanford EE207: Feedback Control Design
©¸©¤©¤ ETH 227-0216: Control Systems II

Electric Machinery (motor physics)
©À©¤©¤ MIT 6.685: Electric Machines
©À©¤©¤ Berkeley EE117: Electromagnetics
©¸©¤©¤ ETH 227-0530: Electric Drive Systems

Power Electronics (inverters, PWM)
©À©¤©¤ MIT 6.334: Power Electronics
©À©¤©¤ Berkeley EE218: Power Electronics
©¸©¤©¤ ETH 227-0526: Power Electronics

Signal Processing (filters, observers)
©À©¤©¤ MIT 6.003: Signals and Systems
©¸©¤©¤ Stanford EE102A: Signal Processing
```

## Core Dependencies

```
mini-motor-control/
©À©¤©¤ depends on: Control Theory fundamentals
©¦   ©À©¤©¤ PID with anti-windup
©¦   ©À©¤©¤ State observers (Luenberger, SMO, EKF)
©¦   ©¸©¤©¤ Stability analysis (Routh-Hurwitz, Lyapunov)
©À©¤©¤ depends on: Electric machine physics
©¦   ©À©¤©¤ Lorentz force, Faraday's law
©¦   ©À©¤©¤ dq-axis modeling
©¦   ©¸©¤©¤ Flux linkage, back-EMF
©À©¤©¤ depends on: Power electronics
©¦   ©À©¤©¤ Three-phase VSI topology
©¦   ©À©¤©¤ PWM modulation (SPWM, SVPWM, DPWM)
©¦   ©¸©¤©¤ Dead-time, overmodulation
©¸©¤©¤ depends on: Mathematics
    ©À©¤©¤ Clarke/Park transforms (linear algebra)
    ©À©¤©¤ ODE integration (RK4)
    ©À©¤©¤ Optimization (MTPA)
    ©¸©¤©¤ Stochastic estimation (EKF)
```

## What Depends on This Module

```
Industrial Applications
©À©¤©¤ servo_drive_control
©À©¤©¤ ev_traction_control
©¸©¤©¤ drone_motor_control

Advanced Topics
©À©¤©¤ predictive_current_control
©À©¤©¤ sensorless_drive_full
©¸©¤©¤ ai_auto_tuning_motor
```

## Knowledge Flow

```
Physics ¡ú Modeling ¡ú Control ¡ú Implementation
  ©¦          ©¦          ©¦           ©¦
Faraday    DC/PM/IM   PID/FOC    SVPWM/PWM
Lorentz    dq-model   Cascaded   Inverter
Newton     TF/time    Observer   Gate drive
  ©¦          ©¦          ©¦           ©¦
  ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
                  ©¦
           motor_control.h/.c
           Complete FOC chain
```
