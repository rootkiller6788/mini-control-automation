# mini-modern-control

Modern Control Theory — State-Space Methods, Optimal Control, and Estimation in C with Lean 4 Formalization.

## Module Status: COMPLETE ✅

- **L1-L6**: Complete
- **L7**: Partial (3 applications: DC motor, quadrotor, inverted pendulum)
- **L8**: Partial (2/5 advanced topics: H-infinity framework, balanced truncation)
- **L9**: Partial (MPC documented, not fully implemented)

| Level | Status | Score |
|-------|--------|-------|
| L1 Definitions | Complete | 2 |
| L2 Core Concepts | Complete | 2 |
| L3 Math Structures | Complete | 2 |
| L4 Fundamental Laws | Complete | 2 |
| L5 Algorithms/Methods | Complete | 2 |
| L6 Canonical Problems | Complete | 2 |
| L7 Applications | Partial | 1 |
| L8 Advanced Topics | Partial | 1 |
| L9 Research Frontiers | Partial | 1 |
| **Total** | | **15/18** |

**Line Count**: include/ + src/ >= 3000 lines ✅

---

## Core Definitions

| Concept | Symbol | Definition |
|---------|--------|-----------|
| State-space (continuous) | dx/dt = Ax + Bu, y = Cx + Du | First-order vector ODE |
| Controllability matrix | Ctrb = [B, AB, ..., A^{n-1}B] | Kalman (1960) |
| Observability matrix | Obsv = [C; CA; ...; CA^{n-1}] | Kalman (1960) |
| State feedback | u = -Kx | Pole placement |
| Luenberger observer | dx̂/dt = Ax̂ + Bu + L(y - Cx̂) | Luenberger (1964) |
| CARE | AᵀP + PA - PBR⁻¹BᵀP + Q = 0 | Optimal LQR |
| DARE | P = AᵀPA - AᵀPB(R+BᵀPB)⁻¹BᵀPA + Q | Discrete LQR |
| Lyapunov equation | AᵀP + PA = -Q | Stability theorem |
| Kalman filter | x̂ₖ₊₁ = Ax̂ₖ + Buₖ + K(yₖ - Cx̂ₖ) | Kalman (1960) |

## Core Theorems

### Kalman Rank Condition (1960)
System (A, B) is controllable iff rank([B, AB, ..., A^{n-1}B]) = n.

### Lyapunov Stability Theorem (1892)
A is Hurwitz iff ∃P > 0: AᵀP + PA = -Q for some Q > 0.

### Separation Principle (Luenberger, 1964)
eig(A_cl) = eig(A - BK) ∪ eig(A - LC). Controller and observer can be designed independently.

### Optimal LQR (Kalman, 1964)
u* = -R⁻¹BᵀPx minimizes J = ∫₀^∞(xᵀQx + uᵀRu)dt where P solves CARE.

## Core Algorithms

| Algorithm | Method | Complexity |
|-----------|--------|------------|
| Pole Placement | Ackermann formula | O(n³) |
| LQR (CARE) | Kleinman-Newton iteration | O(n³) per iteration |
| DARE | Value iteration | O(n³) per iteration |
| Lyapunov equation | Smith iteration | O(n³) per iteration |
| Eigenvalues | QR algorithm + Hessenberg reduction | O(n³) |
| Kalman filter | Predict-Update cycle | O(n³ + np²) |
| Observer design | Dual pole placement | O(n³) |
| Discretization | Matrix exponential (Taylor/Padé) | O(n³) |

## Classic Problems

1. **Inverted Pendulum LQR** — Stabilize upright pendulum on cart
2. **DC Motor LQR** — Optimal velocity/position control
3. **Quadrotor Hover** — Linearized attitude control
4. **Kalman Filtering** — State estimation with noisy measurements
5. **Luenberger Observation** — State reconstruction from outputs

## Course Mapping

| School | Courses | Topics |
|--------|---------|--------|
| MIT | 6.302 | State-space, LQR, observers |
| Stanford | ENGR 205 | Controllability, pole placement |
| Berkeley | ME 232 | LQR, Kalman, Lyapunov |
| Illinois | ECE 515 | State-space methods |
| Michigan | EECS 560 | Linear systems theory |
| Georgia Tech | ECE 6551 | Observers, separation |
| TU Munich | Control 2 | State feedback, LQR |
| ETH Zurich | 227-0216 | Optimal control, Kalman |
| Tsinghua | 控制理论 | Modern control |

## Build & Test

```bash
make          # Build library and test binary
make test     # Build and run all tests (11 tests)
make examples # Build 3 end-to-end examples
make clean    # Remove build artifacts
```

## File Structure

```
mini-modern-control/
├── Makefile
├── README.md                     ← This file (COMPLETE ✅)
├── include/
│   └── modern_control.h           (524 lines)
├── src/
│   ├── matrix_operations.c        (773 lines)
│   ├── state_space.c              (280 lines)
│   ├── controllability.c          (314 lines)
│   ├── pole_placement.c           (146 lines)
│   ├── eigenvalues.c              (239 lines)
│   ├── lqr_solver.c               (243 lines)
│   ├── lyapunov_solver.c          (192 lines)
│   ├── observer_design.c          (126 lines)
│   ├── kalman_filter.c            (191 lines)
│   ├── canonical_problems.c       (146 lines)
│   └── modern_control_formal.lean (116 lines)
├── tests/
│   └── test_modern_control.c      (246 lines)
├── examples/
│   ├── example_inverted_pendulum.c
│   ├── example_dc_motor_lqr.c
│   └── example_kalman_observer.c
├── docs/
│   ├── knowledge-graph.md
│   ├── coverage-report.md
│   ├── gap-report.md
│   ├── course-alignment.md
│   └── course-tree.md
├── demos/
└── benches/
```

## References

- Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems". Trans. ASME.
- Kalman, R.E. (1960). "On the General Theory of Control Systems". IFAC Congress.
- Kalman, R.E. (1963). "Mathematical Description of Linear Dynamical Systems". SIAM J. Control.
- Luenberger, D.G. (1964). "Observing the State of a Linear System". IEEE Trans. Mil. Electron.
- Luenberger, D.G. (1971). "An Introduction to Observers". IEEE Trans. Autom. Control.
- Wonham, W.M. (1967). "On Pole Assignment in Multi-Input Controllable Linear Systems". IEEE TAC.
- Kleinman, D.L. (1968). "On an Iterative Technique for Riccati Equation Computations". IEEE TAC.
- Ogata, K. (2010). *Modern Control Engineering*, 5th ed. Pearson.
- Chen, C.T. (2013). *Linear System Theory and Design*, 4th ed. Oxford.
- Golub, G.H. & Van Loan, C.F. (2013). *Matrix Computations*, 4th ed. Johns Hopkins.
