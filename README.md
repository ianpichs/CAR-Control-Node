# DUT25 LQR + PID Controller

ROS 2 node implementing discrete-time LQR lateral control and gain-scheduled
PID longitudinal control for the DUT25 Formula Student autonomous car.

Designed as a drop-in replacement for `mpc_python_exec` + `longitudinal_control`
in the DUT25 skidpad pipeline. The goal is a direct performance comparison between
LQR (lower computational cost, fixed operating point) and the existing MPC
(higher computational cost, adaptive LPV formulation).

---

## ROS Interface

| Direction | Topic | Message Type | Description |
|-----------|-------|-------------|-------------|
| Subscribe | `/controllers/opt_requests` | `controller_msgs/OptRequest` | Reference path (N+1 waypoints) + current state x0 |
| Subscribe | `/controllers/accel_request` | `controller_msgs/AccRequest` | Speed setpoint + current speed |
| Publish | `/controllers/opt_results` | `controller_msgs/OptResult` | Predicted steer sequence + trajectory |
| Publish | `/controllers/long` | `controller_msgs/PIDErrors` | Throttle command + PID diagnostics |

`skidpad_manager_node` requires no modification — it reads the same topics
and message types that the MPC uses.

---

## System Model — Lateral (LQR)

Dynamic bicycle model linearized at the skidpad operating speed (11.25 m/s).

**State:** `x = [e_y, e_psi, vy, r, steer]`

| State | Description | Units |
|-------|-------------|-------|
| `e_y` | Lateral deviation from reference path | m |
| `e_psi` | Heading error (vehicle yaw − path heading) | rad |
| `vy` | Lateral velocity | m/s |
| `r` | Yaw rate | rad/s |
| `steer` | Front wheel steering angle | rad |

**Control input:** `u = steering_rate` (dδ/dt) [rad/s]

The LQR gain K is computed once at startup by solving the Discrete Algebraic
Riccati Equation (DARE). K remains constant for the full mission.

**Vehicle parameters (DUT25):**

| Parameter | Value | Source |
|-----------|-------|--------|
| Mass m | 180 kg | parameters_LPV.yaml |
| Yaw inertia Iz | 294 kg·m² | parameters_LPV.yaml |
| CoG–front axle lf | 0.872 m | wbase × (1 − x_cg) |
| CoG–rear axle lr | 0.658 m | wbase × x_cg |
| Front stiffness Cf | 18 877 N/rad | Interpolated from DUT25 tyre load curve |
| Rear stiffness Cr | 24 293 N/rad | Interpolated from DUT25 tyre load curve |
| Max steer angle | 0.4 rad | parameters_LPV.yaml |
| Max steer rate | 1.3 rad/s | Simulator maximum (DUT25 actual ~4.0 rad/s) |

---

## Tuning — LQR (lateral)

All Q/R weights are ROS parameters. Change them in the launch file or a YAML
config; relaunch the node to apply (K is recomputed at startup).

| Parameter | Session best (v0.2.9) | Effect / Notes |
|-----------|----------------------|----------------|
| `q_e_y` | **4.0** | Lateral tracking tightness. Raising to 8.0 regressed C1 (3.1m→2.56m at R4, then hurt again in session). 4.0 is optimal. |
| `q_e_psi` | **1.0** | Heading correction. **Floor is 1.0** — lowering to 0.5 causes oscillation divergence on circle 2. Do not adjust. |
| `q_vy` | **5.0** | Lateral velocity damping. 7.0 gives C1=1.7m (best ever) but regresses C2 to 8.0m — net negative. 5.0 is the balanced value. |
| `q_r` | **8.0** | Yaw rate penalty. Raising from 4→8 was the clearest session improvement: visible reduction in circle entry oscillation. |
| `q_steer` | **1.0** | **Dead knob in this regime.** DARE responds <0.5% to changes. Do not adjust. |
| `r_steer_rate` | **1.0** | Control effort penalty. Lowering from 1.5→1.0 was the single largest gain (SS offset eliminated). 0.5 gives C1=1.7m but C2 regression. |

> **Session best result (v0.2.9):** C1=2.0m, SS≈0m, C2=7.5m at `max_steer_rate=1.3 rad/s`.
> Raising `max_steer_rate` from 0.5→1.3 rad/s eliminated steady-state offset and reduced entry peaks.
> Remaining C1/C2 transient peaks are the active tuning frontier; lookahead feedforward was tested
> and reverted (abrupt straight-to-circle curvature transition makes it net negative on skidpad).

---

## Tuning — PID (longitudinal)

Gain parameters match `longitudinal_pid_parameters.yaml` from the DUT25
conventional controller stack. Three gain sets cover low / mid / high speed:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `kp_list` | [4.0, 4.0, 4.0] | Proportional gains per speed range |
| `ki_list` | [1.3, 1.3, 1.3] | Integral gains per speed range |
| `kd_list` | [0.0, 0.0, 0.0] | Derivative gains (disabled — matches baseline) |
| `lower_speed` | 4.0 m/s | Threshold between gain sets 0 and 1 |
| `upper_speed` | 8.0 m/s | Threshold between gain sets 1 and 2 |

---

## DUT25 Integration

### Build inside the container

```bash
# Inside Docker container (./run_container.sh)
cd /dut
colcon build --packages-select lqr_pid_controller \
    --packages-skip spinnaker_camera_driver spinnaker_synchronized_camera_driver state_planning
source install/setup.bash
```

### Add LQR mode to the launch system

Edit `src/mission_control/launch/base_pipeline/controllers.launch.xml`
and add a block for `mode == "lqr"` pointing at this package's launch file.

Then launch with:

```bash
ros2 launch simulator simulation.launch.xml \
    mission_name:=skidpad perception:=sim state_estimation:=sim \
    rviz:=false controller_mode:=lqr
```

### Monitoring in Foxglove Studio

Connect to `ws://localhost:8765`. Key topics for LQR vs MPC comparison:

| Topic | Content |
|-------|---------|
| `/controllers/mpc_error_actual` | MPC lateral tracking error (baseline) |
| `/controllers/opt_results` | LQR predicted trajectory + steer sequence |
| `/controllers/long` | Longitudinal PID errors + throttle |
| `/embedded/to/TrajectorySetpoints` | Combined steer + throttle to simulator |
| `/viz/sim/real_car_pose` | Car position for 3D view |

---

## Repository Structure

```
lqr_pid_controller/
  controller_node.py     Main node — LQR lateral + PID longitudinal
launch/
  lqr_pid_controller.launch.py   Launch file with all tuning parameters
CHANGELOG.md             Full change history with dates and times
package.xml              ROS 2 package manifest (version 0.2.0)
setup.py                 Python package entry point
```

---

## Comparison Goals

| Metric | Expected LQR result |
|--------|-------------------|
| Computation time per step | < 1 ms (matrix multiply only) |
| Control frequency | Matches OptRequest rate (~40 Hz) |
| Lateral tracking error | Comparable to MPC on skidpad (constant operating point) |
| Steering smoothness | May show more oscillation than MPC (no constraint handling) |
| Constraint satisfaction | Soft — steer and rate are clamped post-computation |
