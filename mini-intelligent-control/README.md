# mini-intelligent-control

Intelligent Control Theory -- Fuzzy Logic, Neural Networks, Adaptive Control, MPC, RL, SMC in C with Lean 4 formalization.

## Module Status: COMPLETE ✅

- **L1-L6**: Complete
- **L7**: Complete (DC motor, quadrotor, pendulum, cart-pole)
- **L8**: Complete (Super-twisting SMC, Policy Iteration, Pareto GA, ANFIS)
- **L9**: Partial (documented in knowledge-graph.md)

| Level | Status | Score |
|-------|--------|-------|
| L1 Definitions | Complete | 2 |
| L2 Core Concepts | Complete | 2 |
| L3 Math Structures | Complete | 2 |
| L4 Fundamental Laws | Complete | 2 |
| L5 Algorithms/Methods | Complete | 2 |
| L6 Canonical Problems | Complete | 2 |
| L7 Applications | Complete | 2 |
| L8 Advanced Topics | Complete | 2 |
| L9 Research Frontiers | Partial | 1 |
| **Total** | | **17/18** |

**Line Count**: include/ + src/ = 3030 lines (threshold: 3000) ✅

## Core Theorems

- **Universal Approximation** (Fuzzy + Neural): Any continuous function on compact set can be approximated
- **Lyapunov Stability (MRAC)**: V_dot <= -a_m * e^2 => global asymptotic stability
- **Bellman Optimality**: V*(s) = max_a [R + gamma * P * V*]
- **Sliding Mode Existence**: s * s_dot <= -eta * |s| => finite-time convergence
- **Q-Learning Convergence**: Under Robbins-Monro conditions, Q -> Q* wp1

## Core Algorithms

| Algorithm | Complexity | Reference |
|-----------|-----------|-----------|
| Mamdani Inference | O(R*S) | Mamdani 1975 |
| TSK Inference | O(R*I) | Takagi-Sugeno-Kang 1985 |
| Backpropagation | O(N*W) | Rumelhart et al. 1986 |
| MRAC (Lyapunov) | O(n) | Parks 1966 |
| STR (RLS) | O(p^2) | Astrom & Wittenmark 1973 |
| Active-Set QP | O(n^3) | Nocedal & Wright 2006 |
| Q-Learning | O(A)/step | Watkins 1989 |
| SARSA | O(A)/step | Rummery & Niranjan 1994 |
| Policy Iteration | O(S^3)/iter | Howard 1960 |
| SMC | O(1)/step | Utkin 1977 |
| Super-Twisting SMC | O(1)/step | Levant 1993 |
| Genetic Algorithm | O(G*P*C) | Holland 1975 |

## Classic Problems

1. **Inverted Pendulum** (Fuzzy control)
2. **DC Motor Speed Control** (Neural + MRAC)
3. **Cart-Pole Balancing** (Q-Learning)
4. **Robot Arm Trajectory** (MRAC)
5. **Temperature Control** (MPC)
6. **Buck Converter** (SMC)
7. **PID Tuning** (Genetic Algorithm)

## Course Mapping

| School | Topics |
|--------|--------|
| MIT | 6.003, 6.450, 6.867 - Fuzzy/neural/RL/adaptive |
| Stanford | EE264, EE267, EE359 - MRAC, MPC, intelligent |
| Berkeley | EE221A, EE222, CS287 - Optimal, RL, fuzzy |
| Illinois | ECE 310, ECE 459, ECE 486 - Adaptive, RL |
| Michigan | EECS 351, EECS 455 - LMS, adaptive, MPC |
| Georgia Tech | ECE 4270, ECE 6601 - DSP control, RL |
| TU Munich | Control - MRAC, fuzzy, MPC |
| ETH Zurich | 227-0427, 227-0690 - Intelligent/adaptive |
| Tsinghua | 智能控制 - Fuzzy, neural, expert |

## Build & Test

```bash
make          # Build library and test binary
make test     # Run all tests
make examples # Build examples
make clean    # Remove artifacts
```
