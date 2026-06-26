# Knowledge Graph — mini-modern-control

## L1: Definitions
- State-space model (continuous & discrete): mc_ss_system_t, mc_ss_discrete_t
- Transfer function: mc_tf_t
- Step response metrics: mc_step_response_t
- System types, stability, controllability/observability enums

## L2: Core Concepts
- Controllability matrix (Kalman rank condition)
- Observability matrix
- Controllability/Observability Gramians
- State feedback (u = -Kx)
- Luenberger observer (dx_hat/dt = A x_hat + B u + L(y - C x_hat))
- Kalman decomposition
- Discretization methods (ZOH, Tustin, Euler)
- Separation principle

## L3: Mathematical Structures
- Dense matrix operations (row-major, Gaussian elimination, Cholesky)
- Vector operations (dot, norm, axpy)
- Eigenvalue computation (QR algorithm, Householder reduction)
- Characteristic polynomial (Faddeev-LeVerrier)
- Kronecker product, block diagonal
- Canonical forms (controllable, observable, Jordan)

## L4: Fundamental Laws
- Lyapunov stability theorem (A^T P + P A = -Q, P > 0 => Hurwitz)
- Continuous Algebraic Riccati Equation (CARE)
- Discrete Algebraic Riccati Equation (DARE)
- Kalman rank condition for controllability/observability
- PBH test
- Cayley-Hamilton theorem

## L5: Algorithms/Methods
- Ackermann formula for pole placement
- Kleinman-Newton iteration for CARE
- Smith iteration for Lyapunov equation
- QR algorithm for eigenvalues
- Discretization (matrix exponential via Taylor/Pade)
- Dual pole placement for observer design
- Kalman filter predict/update cycle

## L6: Canonical Problems
- Inverted pendulum LQR stabilization
- DC motor velocity/position LQR control
- Quadrotor hover linearization
- Kalman filter for noisy state estimation
- Luenberger observer design

## L7: Applications
- DC motor control (industrial automation, Toyota servo systems)
- Quadrotor attitude control (drone stabilization, SpaceX landing)
- Inverted pendulum (robotics, Segway balancing)

## L8: Advanced Topics
- H-infinity control formulation (gamma iteration)
- Balanced truncation model reduction
- Robustness margins (gain, phase, disk margins)
- Kalman decomposition (structural decomposition)
- Minimal realization

## L9: Research Frontiers
- Model predictive control (MPC) structures
- Robust control for uncertain systems
- 6G RIS (reconfigurable intelligent surfaces) control
- Quantum control systems
