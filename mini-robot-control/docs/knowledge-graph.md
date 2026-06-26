# Knowledge Graph — mini-robot-control

## L1: Definitions (Complete)
- Robot types (serial, parallel, mobile, UAV, AUV, SCARA, Cartesian, collaborative, soft, humanoid)
- Joint types (revolute, prismatic, continuous, spherical, fixed, planar)
- Control modes (position, velocity, torque, impedance, admittance, hybrid, compliant)
- Coordinate frames (world, base, tool, sensor, object)
- DH parameters (alpha, a, d, theta) + standard vs modified convention
- Link parameters (mass, COM, inertia tensor, friction, gear ratio)
- Robot model (DOF, links, DH table, joint limits, gravity)
- Robot state (joint positions, velocities, accelerations, torques)
- Trajectory types (joint space, Cartesian, cubic, quintic, trapezoidal, S-curve, LSPB, B-spline)
- Performance metrics (settling time, overshoot, steady-state error, RMS tracking, control effort)
- Error codes (null pointer, invalid DOF, memory, singularity, joint limits, etc.)

## L2: Core Concepts (Complete)
- Forward kinematics as DH chain
- Inverse kinematics as nonlinear equation solving
- Differential kinematics (Jacobian)
- Lagrangian dynamics
- Newton-Euler recursive dynamics
- Geometric vs analytic Jacobian
- Kinematic singularity and manipulability
- Workspace analysis (reachable, dexterous)
- Joint-space vs task-space control
- Trajectory planning in joint and Cartesian space
- Passive vs active compliance
- Friction models (viscous + Coulomb)

## L3: Mathematical Structures (Complete)
- vec3: 3D vector algebra (add, sub, cross, dot, norm)
- mat3: SO(3) rotation matrices (Rodrigues, Euler angles, axis-angle)
- mat4: SE(3) homogeneous transforms (composition, inverse, point/vector transform)
- Quaternions: S^3 covering SO(3) (slerp, multiply, conjugate, normalize)
- Euler angles: ZYX convention with gimbal lock handling
- twist/wrench: 6D spatial vectors
- pose3d: unified position + orientation representation
- Jacobian utilities (multiply, transpose-multiply, pseudoinverse)
- SE(3) exponential map

## L4: Fundamental Laws (Complete)
- Denavit-Hartenberg theorem (1955)
- Chasles screw theorem (every rigid motion = rotation + translation along axis)
- Euler rotation theorem
- Lagrangian mechanics (d/dt(∂L/∂q_dot) - ∂L/∂q = tau)
- Newton-Euler equations
- Lyapunov stability (Takegaki & Arimoto, 1981) for PD+gravity
- LaSalle invariance principle
- Orthogonality principle in inverse kinematics
- Yoshikawa manipulability measure

## L5: Algorithms/Methods (Complete)
- Newton-Raphson IK with damped least-squares
- Gradient descent IK
- Cyclic Coordinate Descent (CCD) IK
- Czochralski (6x6 Gaussian elimination for IK)
- Cubic polynomial trajectory (4 constraints)
- Quintic polynomial trajectory (6 constraints)
- Trapezoidal velocity profile (3 phases)
- S-curve velocity profile (7 phases)
- LSPB (linear segment with parabolic blend)
- Cubic spline interpolation
- Cartesian line trajectory with SLERP
- Cartesian circle trajectory
- PID control with anti-windup
- Computed Torque Control (feedback linearization)
- Slotine & Li adaptive control
- RLS parameter estimation

## L6: Canonical Problems (Complete)
- Planar 2-DOF forward/inverse kinematics
- SCARA robot pick-and-place
- Differential-drive mobile robot trajectory tracking
- Quadrotor waypoint navigation
- PD+gravity setpoint regulation
- Impedance-controlled peg-in-hole
- Force control for surface following
- Time-optimal trajectory with torque limits
- Joint limit barrier avoidance

## L7: Applications (Partial)
- KUKA/ABB-style industrial joint move (with gravity compensation)
- SCARA assembly pick-and-place
- Mobile robot figure-8 tracking
- Quadrotor waypoint PID control

## L8: Advanced Topics (Partial)
- Sliding mode control for robust tracking
- Monte Carlo workspace sampling
- Fuzzy gain scheduling for PID adaptation
- Lyapunov stability numerical verification
- RLS parameter estimation with exponential forgetting

## L9: Research Frontiers (Partial)
- Soft robot control (documented, not implemented)
- Humanoid locomotion (documented, not implemented)
- Collaborative robot force control (documented, not implemented)
- 6G-connected telerobotics (documented, not implemented)
