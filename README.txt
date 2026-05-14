ELEN 6760 Final Code Submission
Author : Ian Pichs (ifp2107) — ian.pichs@columbia.edu
Course : Discrete-Time Control Systems, Spring 2026

Per discussion with Prof. Beigi I am not submitting the simulation that
accompanies these control nodes, as it is proprietary to the Delft Student
Formula Team (DUT25). The two ROS 2 control nodes described below contain
all original ifp2107 code and are available on GitHub (links below).

===============================================================================
REPOSITORY
===============================================================================
GitHub: https://github.com/IanPichs/CAR-Control-Node
Branch: main

The two packages submitted are:
  1. lqr_pid_controller/   — LQR lateral + PID longitudinal controller
  2. coupled_mpc_controller/ — Coupled Nonlinear MPC (acados OCP)

===============================================================================
FILES TOUCHED BY ifp2107
===============================================================================
Search the codebase for "ifp2107" to locate every line authored by Ian Pichs.

lqr_pid_controller/
  lqr_pid_controller/controller_node.py   [ALL — original ifp2107 work]
  package.xml                              [author/maintainer fields]
  setup.py                                 [description and maintainer]

coupled_mpc_controller/
  coupled_mpc_controller/ocp_definition.py  [ALL — original ifp2107 work]
  coupled_mpc_controller/controller_node.py [ALL — original ifp2107 work]
  package.xml                               [author/maintainer fields]
  setup.py                                  [description and maintainer]

===============================================================================
PACKAGE STRUCTURE
===============================================================================

lqr_pid_controller/
├── lqr_pid_controller/
│   ├── controller_node.py   Main ROS 2 node: LQR lateral + PID longitudinal
│   └── __init__.py
├── launch/
│   └── lqr_pid_controller.launch.py
├── package.xml
├── setup.py
└── CMakeLists.txt

coupled_mpc_controller/
├── coupled_mpc_controller/
│   ├── ocp_definition.py    acados OCP definition: 7-state nonlinear MPC
│   ├── controller_node.py   ROS 2 node: solver loop + topic I/O
│   └── __init__.py
├── launch/
│   └── coupled_mpc_controller.launch.py
├── package.xml
└── setup.py

===============================================================================
CONTROLLER DESCRIPTIONS
===============================================================================

1. LQR + PID (lqr_pid_controller)
   - Lateral: discrete-time LQR on a 5-state linearized dynamic bicycle model
     [e_y, e_psi, vy, r, steer]. Gain-scheduled via DARE at 5 speed set-points
     (3, 5, 7, 9, 11.25 m/s). Curvature feedforward with 40-waypoint lookahead.
   - Longitudinal: gain-scheduled PID with quadratic feedforward, anti-windup,
     and rate limiter.
   - Input:  /controllers/opt_requests  (controller_msgs/OptRequest)
             /controllers/accel_request (controller_msgs/AccRequest)
   - Output: /controllers/opt_results   (controller_msgs/OptResult)
             /controllers/long          (controller_msgs/PIDErrors)

2. Coupled Nonlinear MPC (coupled_mpc_controller)
   - 7-state, 2-input acados OCP: x = [px, py, psi, vy, r, delta, vx],
     u = [delta_dot, D]. Solved via SQP-RTI with PARTIAL_CONDENSING_HPIPM.
   - Horizon: N=40 stages, Ts=0.025 s (Tf=1.0 s). ERK4 integrator.
   - Soft friction circle constraint: (D/ax_max)^2 + (ay/ay_max)^2 <= 1 + eps.
   - Input:  /controllers/opt_requests  (controller_msgs/OptRequest)
             /controllers/accel_request (controller_msgs/AccRequest)
   - Output: /controllers/opt_results   (controller_msgs/OptResult)
             /controllers/long          (controller_msgs/PIDErrors)

===============================================================================
HOW TO RUN (inside DUT25 Docker container)
===============================================================================
NOTE: The DUT25 simulation environment is NOT included — it is proprietary
to the Delft Student Formula Team and cannot be distributed.

To build the two packages:
  source /opt/ros/humble/setup.bash
  colcon build --packages-select lqr_pid_controller coupled_mpc_controller
  source install/setup.bash

To launch the LQR controller standalone:
  ros2 launch lqr_pid_controller lqr_pid_controller.launch.py

To launch the Coupled MPC controller standalone:
  ros2 launch coupled_mpc_controller coupled_mpc_controller.launch.py

Within the full DUT25 stack, controllers are selected via the mission_control
launch system:
  LQR mode:  ros2 launch mission_control simulation.launch.xml \
               controller_mode:=lqr mission_name:=skidpad \
               state_estimation:=sim
  MPC mode:  ros2 launch mission_control simulation.launch.xml \
               controller_mode:=mpcc mission_name:=skidpad \
               state_estimation:=sim

===============================================================================
DEPENDENCIES
===============================================================================
  ROS 2 Humble
  controller_msgs  (DUT25 interfaces — OptRequest, OptResult, AccRequest, PIDErrors)
  numpy, scipy     (LQR: DARE solver, ZOH discretization)
  acados           (Coupled MPC: SQP-RTI solver)
  casadi           (Coupled MPC: symbolic OCP formulation)

===============================================================================
OPEN-SOURCE REFERENCES
===============================================================================
[1] R. Verschueren et al., "acados: a modular open-source framework for fast
    embedded optimal control," Math. Prog. Comp., vol. 14, pp. 147-176, 2022.
    https://github.com/acados/acados

[2] J. Andersson et al., "CasADi: A software framework for nonlinear
    optimization and optimal control," Math. Prog. Comp., 2019.
    https://github.com/casadi/casadi

[3] R. Rajamani, Vehicle Dynamics and Control, 2nd ed. Springer, 2012.
    (Dynamic bicycle model and cornering stiffness parameters)

[4] J. Betz et al., "TUM autonomous motorsport: An autonomous racing software
    for the Indy Autonomous Challenge," J. Field Robot., 2023.
    (Curvilinear MPC formulation reference — mpc_python baseline by DUT team)

[5] V. Mitev, "Better State Estimation: Project Definition & Concept
    Selection," Formula Student Team Delft, Tech. Rep., Nov. 2024, unpublished.
    (DUT25 state estimation subsystem design)
