# Changelog

All notable changes to this project are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
Versioning: [Semantic Versioning](https://semver.org/) — MAJOR.MINOR.PATCH

---

## [0.2.3] — 2026-05-11 UTC

### Changed (tuning)
- **Confirmed best Q/R parameters (R4)**: after six systematic one-change-at-a-time rounds,
  the optimal weights are:
  `q_e_y=8.0, q_e_psi=1.0, q_vy=5.0, q_r=4.0, q_steer=1.0, r_steer_rate=1.0`
  Both launch files (`controllers.launch.xml` and `lqr_pid_controller.launch.py`) set to these values.

### Findings
- **DARE insensitivity to q_steer**: Raising q_steer from 1.0 → 4.0 (Round 5) produced only a
  0.5% change in K[steer] (K[steer] ≈ 15.53 is already the dominant eigenvalue). Performance
  regressed: C1 3.40m, C2 9.19m vs R4 baseline 2.56m / 7.66m. q_steer is a dead knob in this
  operating regime. Reverted to 1.0.
- **q_e_psi floor = 1.0**: Lowering q_e_psi from 1.0 → 0.5 (Round 6) produced only 0.1% change
  in K[e_psi]. Car failed to complete track — C2 error reached −13.7m with heading errors ±112°
  as reduced heading correction allowed oscillations to grow compoundingly. 1.0 is the minimum
  stable value. Reverted to 1.0.
- **max_steer_rate is the primary bottleneck**: At circle entries u_raw reaches 5–20 rad/s but
  is clamped to 0.5 rad/s. C1 peak (~2.56m) and C2 peak (~7.66m) are both driven by steer rate
  saturation, not by Q/R weights. Q/R tuning has reached its ceiling.
- **Steady-state offset (~−0.35m)**: Caused by feedforward model mismatch; not eliminable via
  Q/R without integral action.

### Notes
- Tuning round history (all at request_interval=0.004s, N=100, Tf=1.0s, vx_op=11.25 m/s):

| Round | q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | C1 peak | SS offset | C2 peak | Completed |
|-------|-------|---------|------|-----|---------|---------|---------|-----------|---------|-----------|
| R1 | 7.0 | 1.0 | 2.0 | 2.0 | 1.0 | 1.5 | ~2.9m | ~−0.47m | diverged | No |
| R2 | 4.0 | 1.0 | 5.0 | 4.0 | 1.0 | 1.5 | ~2.3m | ~−0.47m | ~7.3m | Yes |
| R3 | 8.0 | 2.0 | 5.0 | 4.0 | 1.0 | 1.0 | ~3.1m | ~−0.35m | ~8.1m | Yes |
| **R4** | **8.0** | **1.0** | **5.0** | **4.0** | **1.0** | **1.0** | **~2.56m** | **~−0.35m** | **~7.66m** | **Yes ← BEST** |
| R5 | 8.0 | 1.0 | 5.0 | 4.0 | 4.0 | 1.0 | ~3.40m | ~−0.35m | ~9.19m | Yes |
| R6 | 8.0 | 0.5 | 5.0 | 4.0 | 1.0 | 1.0 | — | — | −13.7m | No |

- Next experiment: raise `max_steer_rate` from 0.5 → 1.0 rad/s (single-parameter change).
  Simulator physically allows ~1.3 rad/s; DUT25 actual ~4.0 rad/s.

---

## [0.2.2] — 2026-05-11 UTC

### Changed
- **Control frequency: 40 Hz → 250 Hz**: `request_interval` for skidpad_manager
  overridden to 0.004 s in the `mode=lqr` launch block. skidpad_manager's
  pose_callback already fires at 250 Hz (driven by `/se/vehicle/pose`); the
  previous 0.025 s interval was the only bottleneck. The LQR now receives fresh
  x0 state and recomputed waypoints every 4 ms instead of every 25 ms.
  At 11.25 m/s this reduces the arc traveled between corrections from 28 cm to
  4.5 cm, cutting per-step error accumulation by 6×.
- **MultiThreadedExecutor**: `main()` now uses `rclpy.executors.MultiThreadedExecutor`
  so that `_on_accel_request` callbacks are not starved by `_on_opt_request`
  at the higher 250 Hz rate.

### Notes
- MPC mode is unaffected: the `request_interval` override exists only inside the
  `mode=lqr` launch group; MPC loads `skidpad_mpc_python_params.yaml` which
  retains `request_interval: 0.025`.
- `max_steer_rate` remains 0.5 rad/s for this release. The simulator enforces
  ~1.3 rad/s at the front wheel (5.24 rad/s column ÷ ~4:1 ratio); the
  `full-24` measured value is 4.0 rad/s. Rate increase is deferred until
  250 Hz baseline performance is evaluated.

---

## [0.2.1] — 2026-05-10 22:30 UTC

### Fixed
- **Path curvature feed-forward**: LQR error state now subtracts steady-state
  reference values for r, steer, and vy before applying K. Without this, K[steer]
  and K[r] commanded opposing the turn at the moment circular motion began (u_ss ≈
  −3.2 rad/s against the right circle), causing immediate divergence into random
  spins. Reference values derived from path curvature κ computed locally from
  adjacent waypoints in each OptRequest:
    r_ref = vx · κ,  steer_ref = −(LF+LR) · κ,  vy_ref = 0.2429 · vx · r_ref
  Handles both skidpad circles and straights automatically (κ≈0 on straights).
- **Corrected cornering stiffness**: Cf=Cr=20 000 N/rad replaced with values
  interpolated from the DUT25 measured tyre stiffness curve (same data used by
  LPVMPC.get_tyre_stiffness): Cf=18 877 N/rad (front, 379.6 N/tyre load),
  Cr=24 293 N/rad (rear, 503.3 N/tyre load). This corrects a 17 % underestimate
  of rear stiffness that caused model/plant mismatch in the K matrix.

### Added
- `_compute_path_curvature(ref_idx, msg)` method: estimates signed path curvature
  κ = dθ/ds from heading angle difference between adjacent waypoints, used for the
  feed-forward reference computation above.

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
