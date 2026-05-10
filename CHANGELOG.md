# Changelog

All notable changes to this project are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
Versioning: [Semantic Versioning](https://semver.org/) — MAJOR.MINOR.PATCH

---

## [0.2.0] — 2026-05-10 17:00 UTC

### Added
- Dynamic lateral bicycle model with DUT25 vehicle parameters:
  m=180 kg, Iz=294 kg·m², lf=0.872 m, lr=0.658 m, Cf=Cr=20 000 N/rad
  (linearized at vx=11.25 m/s skidpad operating point)
- 5-state LQR error vector: [e_y, e_psi, vy, r, steer] — replaces the
  3-state kinematic model; now includes explicit lateral velocity and yaw
  rate dynamics driven by tyre cornering stiffness
- N-step trajectory propagation (N=100, Tf=1.0 s, dt=10 ms) using
  nonlinear Euler integration — produces the 101-element OptResult.steer
  array required by skidpad_manager's time-interpolated latency compensator
- Gain-scheduled longitudinal PID with three speed-range gain sets matching
  DUT25 conventional/config/longitudinal_pid_parameters.yaml exactly
- Quadratic speed feed-forward term (FF_A·v² + FF_B·v + FF_C) derived from
  DUT25 measured vehicle data in speed_ff_term_calc.py
- All Q/R weights and PID gains exposed as individual ROS 2 parameters for
  runtime tuning without recompilation
- Full inline documentation: module-level docstring covers system model,
  ROS interface, how skidpad_manager uses OptResult, and tuning guide

### Changed
- ROS interface completely replaced to match DUT25 skidpad pipeline:
    Subscribes: /controllers/opt_requests (OptRequest), /controllers/accel_request (AccRequest)
    Publishes:  /controllers/opt_results (OptResult), /controllers/long (PIDErrors)
- LQR control output changed from steering angle (δ) to steering rate (dδ/dt);
  steering angle is now a state that integrates the control input
- Control architecture changed from timer-driven to callback-driven:
  each OptRequest triggers an immediate LQR response (microsecond latency
  vs 25 ms timer); each AccRequest triggers an immediate PID response
- Vehicle model discretization timestep changed from DT=0.05 s (20 Hz) to
  DT_TRAJ=0.01 s (100 Hz) to match the manager's interpolation time axis
- package.xml version bumped to 0.2.0; dependencies updated (removed
  nav_msgs, geometry_msgs, std_msgs; added controller_msgs)

### Fixed
- Entry point bug in setup.py: module path was lqr_pid_controller.lqr_pid_controller
  (non-existent); corrected to lqr_pid_controller.controller_node

### Removed
- Kinematic bicycle model (wheelbase-only parameterisation, valid only at low speed)
- Generic nav_msgs/Odometry + geometry_msgs/PoseStamped subscriber interface
- Standalone throttle [0, 1] output (replaced by DUT25-compatible PIDErrors)
- Standalone steering angle output (replaced by steering_rate + steer sequence in OptResult)

---

## [0.1.0] — 2026-02-26

### Added
- Initial implementation: discrete LQR steering + PID throttle
- 3-state kinematic bicycle model: state = [e_lat, e_yaw, ė_lat],
  control = steering angle δ; linearized using wheelbase L and speed v
- DARE solver (scipy.linalg.solve_discrete_are) for optimal gain computation
- Zero-Order Hold (ZOH) discretization via scipy.signal.cont2discrete
- Fixed-rate timer control loop decoupled from topic publish rates
- ROS 2 parameter system for vehicle parameters and cost weights

### Fixed
- Integral windup: added explicit clamp to PID accumulator
- Triple-callback publishing: moved control output to timer; callbacks
  only cache incoming data
- Logger API: corrected to ROS 2 Python throttled logging syntax
  (warning(..., throttle_duration_sec=N))
