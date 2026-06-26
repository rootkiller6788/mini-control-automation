# Prerequisite Dependency Tree — mini-robot-control

```
Robot Control (this module)
├── Linear Algebra
│   ├── Vector spaces, dot/cross products
│   ├── Matrix multiplication, transpose, inverse
│   └── Eigenvalues, singular values (for manipulability)
├── Calculus
│   ├── Multivariable differentiation (Jacobian)
│   ├── Ordinary differential equations (dynamics integration)
│   └── Taylor series (Newton-Raphson IK)
├── Classical Mechanics
│   ├── Newton's laws (Newton-Euler dynamics)
│   ├── Lagrangian mechanics (energy-based dynamics)
│   └── Rigid body kinematics (rotation matrices, quaternions)
├── Control Theory
│   ├── PID control (mini-classical-control)
│   ├── State-space methods (mini-modern-control)
│   ├── Lyapunov stability theory
│   └── Feedback linearization (computed torque)
├── Differential Geometry
│   ├── Lie groups SO(3), SE(3)
│   ├── Exponential map, screw theory
│   └── Configuration space topology
├── Numerical Methods
│   ├── Newton's method (IK solving)
│   ├── Gaussian elimination (matrix inversion)
│   ├── Spline interpolation (trajectory)
│   └── RLS estimation (parameter adaptation)
├── Optimization
│   ├── Gradient descent (IK)
│   ├── Time-optimal control (bang-bang)
│   └── Workspace Monte Carlo sampling
└── Embedded Systems
    ├── Real-time control loops
    ├── Sensor integration (joint encoders, force sensors)
    └── Motor control (mini-motor-control)
```
