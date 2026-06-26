# Course Alignment — mini-motor-control

## MIT

| Course | Topic | Implementation |
|--------|-------|---------------|
| 6.302 Feedback Control | PID, anti-windup, cascaded loops | `pid_update()`, `speed_control()` |
| 6.450 Digital Communications | (peripheral - control system commutation) | BLDC commutation timing |
| 6.685 Electric Machines | DC/PM/IM motor models, dq theory | `motor_model.c` RK4 models |
| 6.334 Power Electronics | Three-phase inverters, PWM, SVPWM | `pwm_modulation.c` |

## Stanford

| Course | Topic | Implementation |
|--------|-------|---------------|
| EE207 Feedback Control | PID design, Routh-Hurwitz stability | `pid_update()`, Lean stability theorem |
| EE359 Wireless (peripheral) | Motor drives for antenna positioning | Position control loop |
| EE253 Power Electronics | DC-DC + inverter topologies for motor drives | DC bus + inverter models |

## Berkeley

| Course | Topic | Implementation |
|--------|-------|---------------|
| EE105 Analog Circuits | Current sense amplifiers, gate drivers | (indirect - fault detection) |
| EE117 Electromagnetics | Magnetic circuit analysis, flux linkage | `flux_estimator_voltage_model()` |
| EE128 Mechatronics | Motor selection, sensor interfaces | `motor_types.h` parameter validation |
| EE218 Power Electronics | SVPWM, DPWM, overmodulation | `pwm_modulation.c` |

## ETH Zurich

| Course | Topic | Implementation |
|--------|-------|---------------|
| 227-0216 Control Systems II | State-space control, observers | EKF implementation |
| 227-0526 Power Electronics | Advanced PWM techniques | THIPWM, DPWM variants |
| 227-0530 Electric Drive Systems | FOC, sensorless control, MTPA | Full FOC chain |

## TU Munich

| Course | Topic | Implementation |
|--------|-------|---------------|
| High-Frequency Engineering | EMI considerations, spread-spectrum PWM | `spread_spectrum_pwm_frequency()` |
| Advanced Control | Sliding mode control theory | SMO implementation |

## Tsinghua / 清华

| Course | Topic | Implementation |
|--------|-------|---------------|
| 信号与系统 | Transfer functions, time constants | `dc_motor_transfer_function()` |
| 通信原理 | PWM as modulation | PWM techniques |
| 电磁场 | Maxwell -> motor flux | Flux estimation |
| 数字信号处理 | Discrete-time control, EKF | `ekf_step()` |

## Cross-Cutting Coverage

| Skill | Multiple Courses | Implementation |
|-------|-----------------|---------------|
| PID Control | MIT 6.302, Stanford EE207, ETH 227-0216 | Complete |
| Motor Modeling | MIT 6.685, Berkeley EE117, ETH 227-0530 | Complete |
| PWM Modulation | MIT 6.334, Berkeley EE218, ETH 227-0526 | Complete |
| Sensorless Control | ETH 227-0530, TUM Advanced Control | Complete |
| Coordinate Transforms | All motor control courses | Complete |
