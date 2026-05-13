# DUT25 Autonomous — Controller Suite

ROS 2 controller implementations for the DUT25 Formula Student autonomous car, skidpad mission.
Three architectures compared: LQR+PID, Decoupled MPC (team baseline), and Coupled Nonlinear MPC.

Both controllers authored by ifp2107 (Ian Pichs, Columbia ELEN 6760, Spring 2026) are designed
as drop-in replacements for `mpc_python_exec` + `longitudinal_control` in the DUT25 skidpad
pipeline. The ROS interface is identical to the existing MPC — `skidpad_manager_node` requires
no modification.

---

# LQR + PID Controller

`lqr_pid_controller/` — Discrete-time LQR lateral control + gain-scheduled PID longitudinal.

## ROS Interface

| Direction | Topic | Message Type | Description |
|-----------|-------|-------------|-------------|
| Subscribe | `/controllers/opt_requests` | `controller_msgs/OptRequest` | Reference path (N+1 waypoints) + current state x0 |
| Subscribe | `/controllers/accel_request` | `controller_msgs/AccRequest` | Speed setpoint + current speed |
| Publish | `/controllers/opt_results` | `controller_msgs/OptResult` | Predicted steer sequence + trajectory |
| Publish | `/controllers/long` | `controller_msgs/PIDErrors` | Throttle command + PID diagnostics |

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

The LQR gain K is pre-computed at startup for each speed in `vx_schedule`
(default: [3, 5, 7, 9, 11.25] m/s) by solving the Discrete Algebraic Riccati
Equation (DARE). At runtime K is linearly interpolated from the two bracketing
schedule entries using the current measured speed from AccRequest. Speeds below
the lowest schedule entry clamp to the lowest K. This gain scheduling keeps the
controller near-optimal during the acceleration phase into the skidpad circles,
not just at the 11.25 m/s cruise point.

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

## Curvature Feedforward

The LQR error state is computed relative to speed-dependent steady-state reference values derived from path curvature κ. Without this, K[steer] and K[r] would command a net input opposing the turn at the moment circular motion begins (at circle entry this causes immediate divergence into large-amplitude oscillations).

**κ computation** — estimated from adjacent waypoints at the lookahead index:

```
κ = (θ_{i+1} − θ_i) / ds
```

where θ is the path heading and ds is the arc-length between waypoints (= vx × dt_traj = 0.1125 m at 11.25 m/s).

**Reference signals:**

| Signal | Formula | Notes |
|--------|---------|-------|
| `r_ref` | `vx · κ` | Expected yaw rate — scales linearly with speed |
| `steer_ref` | `−(lf + lr) · κ` | Kinematic steer — **speed-independent** |
| `vy_ref` | `0.2429 · vx · r_ref` | Lateral velocity proxy — scales with vx² · κ |

`steer_ref` does not depend on speed — only on path geometry and wheelbase. `vy_ref` is the steady-state lateral velocity predicted by the bicycle model for circular motion at the current speed.

**Reference values at skidpad circle (κ = 1/7.5 m⁻¹) across gain-schedule speeds:**

| vx [m/s] | r_ref [rad/s] | steer_ref [rad] | vy_ref [m/s] |
|----------|--------------|-----------------|--------------|
| 3.0 | 0.400 | −0.204 | 0.291 |
| 5.0 | 0.667 | −0.204 | 0.810 |
| 7.0 | 0.933 | −0.204 | 1.587 |
| 9.0 | 1.200 | −0.204 | 2.623 |
| 11.25 | 1.500 | −0.204 | 4.099 |

These are the values subtracted from the measured state before applying K. At 11.25 m/s on the skidpad the LQR is tracking a yaw rate of 1.5 rad/s and steering angle of 0.204 rad — without feedforward, both would appear as large error signals driving the wrong correction direction.

**Lookahead mechanism** — `preview_idx = ref_idx + lookahead_steps`. At the current optimum of 40 steps:

- Preview distance: 40 × 0.1125 m = **4.5 m** (≈ 0.4 s at 11.25 m/s)
- The error states (e_y, e_psi) still use the nearest waypoint — only the feedforward references (r_ref, steer_ref, vy_ref) use the preview index

---

## Tuning — LQR (lateral)

All Q/R weights are ROS parameters. Change them in the launch file or a YAML
config; relaunch the node to apply (K is recomputed at startup).

| Parameter | Best (v0.3.7) | Effect / Notes |
|-----------|--------------|----------------|
| `q_e_y` | **4.0** | Lateral tracking tightness. Ceiling at 4.0 for current damping — higher causes oscillation. |
| `q_e_psi` | **1.0** | Heading correction. **Floor is 1.0** — lower causes divergence on C2. Do not adjust. |
| `q_vy` | **5.0** | Lateral velocity damping. 7.0 improves C1 but regresses C2 — net negative. |
| `q_r` | **8.0** | Yaw rate penalty. Raising 4→8 was the clearest single improvement in tuning. |
| `q_steer` | **1.0** | **Dead knob in this regime.** DARE responds <0.5% to changes. Do not adjust. |
| `r_steer_rate` | **1.0** | Control effort penalty. Lowering 1.5→1.0 was the largest single gain (SS offset eliminated). |
| `lookahead_steps` | **40** | Waypoints ahead for curvature feedforward. 40 = 4.5 m preview — primary lever for C2 transient. |

> **LQR Best result (v0.3.7):** C1≈0.76m, SS≈−0.47m, C2≈0.52m at `max_steer_rate=1.3 rad/s`, `lookahead_steps=40`.
> Lookahead feedforward at 40 steps reduced C2 from 7.5m to 0.52m (×14 improvement over baseline).
> The straight-to-circle transition is abrupt (no clothoid), so lookahead >60 pre-steers on the
> approach straight and worsens C1. 40 steps is the current optimum.

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

## DUT25 Integration (LQR+PID)

### Prerequisites — manual fixes required on a fresh machine

Two packages in the DUT25 repo declare incomplete dependencies, causing
`colcon build` to fail when `lqr_pid_controller` is present. These fixes
are applied directly to the host-mounted files and are **not committed to
the DUT25 repo** — they must be reapplied on any new machine:

```xml
<!-- src/perception_25/pointcloud_processing/package.xml — add inside <package>: -->
<depend>cv_msgs</depend>

<!-- src/state_planning/package.xml — add inside <package>: -->
<depend>controller_msgs</depend>
```

Once applied they persist across container restarts (host-mounted volume).

### Build inside the container

```bash
# Inside Docker container (./run_container.sh)
cd /dut
colcon build --packages-select lqr_pid_controller \
    --packages-skip spinnaker_camera_driver spinnaker_synchronized_camera_driver state_planning
source install/setup.bash
```

### Launch

```bash
ros2 launch simulator simulation.launch.xml \
    mission_name:=skidpad perception:=sim state_estimation:=sim \
    rviz:=false controller_mode:=lqr
```

### Monitoring in Foxglove Studio

Connect to `ws://localhost:8765`. Key topics:

| Topic | Content |
|-------|---------|
| `/controllers/mpc_error_actual` | Lateral tracking error |
| `/controllers/opt_results` | Predicted trajectory + steer sequence |
| `/controllers/long` | Longitudinal PID errors + throttle |
| `/embedded/to/TrajectorySetpoints` | Combined steer + throttle to simulator |
| `/viz/sim/real_car_pose` | Car position for 3D view |

<p align="center">
  <img src="0_3_7_output.png" alt="LQR v0.3.7 controller visualization" width="900">
</p>

---

# Coupled Nonlinear MPC (MPCC)

`coupled_mpc_controller/` — Coupled nonlinear MPC solving lateral and longitudinal control jointly in a single OCP. Uses acados SQP_RTI with ERK4 integration and PARTIAL_CONDENSING_HPIPM QP solver.

## System Model — Coupled MPC

**State:** `x = [pos_x, pos_y, psi, vy, r, steer, vx]` — 7 states (global position, heading, lateral velocity, yaw rate, steering angle, longitudinal speed)

**Controls:** `u = [steer_rate, throttle]` — 2 inputs

| Parameter | Value |
|-----------|-------|
| Horizon N | 40 steps |
| dt | 0.025 s |
| Horizon Tf | 1.0 s |
| Solver | SQP_RTI + ERK4 + PARTIAL_CONDENSING_HPIPM |
| Friction circle | (ax/15)² + (ay/15)² ≤ 1 (soft constraint) |

**Cost weights** `W_diag = [pos_x, pos_y, psi, vy, r, steer, vx, steer_rate, throttle]`:

| Weight | Value | Notes |
|--------|-------|-------|
| W_pos_x | 10 | Along-track position (vehicle body frame) |
| W_pos_y | 10 | Cross-track position (primary lateral tracking metric) |
| W_psi | 1 | Heading — kept low; heading changes continuously on a circle, high weight causes oscillation |
| W_vy | 0 | Lateral velocity — not penalized (naturally regulated via pos_y) |
| W_r | 0 | Yaw rate — not penalized directly |
| W_steer | 1 | Steering angle |
| W_vx | 10 | Longitudinal speed — high weight prevents speed runaway and panic braking at changeover |
| W_steer_rate | 1 | Control effort on steering |
| W_throttle | 0.5 | Control effort on throttle |

**Vehicle parameters (DUT25):**

| Parameter | Value |
|-----------|-------|
| Mass m | 180 kg |
| Yaw inertia Iz | 294 kg·m² |
| CoG–front axle lf | 0.872 m |
| CoG–rear axle lr | 0.658 m |
| Front stiffness Cf | 18 877 N/rad |
| Rear stiffness Cr | 24 293 N/rad |
| Drag coefficient | 0.0075 m/s² per (m/s)² |
| Max steer angle | ±0.4 rad |
| Max steer rate | 1.3 rad/s |
| Max throttle | 15 m/s² |
| Max braking | 20 m/s² |

## Key Implementation Details

**Vehicle body frame convention:** All waypoints in OptRequest are in the vehicle's locally-centred body frame — `x0.pos_x = 0`, `x0.pos_y = 0`, `x0.heading = 0` always. The OCP reference is set in this frame each timestep.

**vx_safe clamp:** `vx_safe = ca.fmax(vx, 1.0)` — prevents Jacobian degeneracy in the bicycle model dynamics at near-zero speed. Clamp at 1.0 m/s rather than 0.5 m/s avoids a gradient singularity at the boundary.

**Warm-start mismatch guard:** At circle changeover the previous optimal trajectory curves right for all 40 stages while the new reference curves left. Mid-horizon mismatch reaches ~3.4 m geometrically. When mismatch at k=20 exceeds `WARM_START_MISMATCH_THRESHOLD = 3.0 m`, the warm-start is reset to a neutral cruise trajectory (zero steer_rate, drag-compensating throttle). The 3.0 m threshold fires at changeover without triggering during normal tracking (errors ≤2 m).

**Changeover physics (documented limitation):** At changeover steer must reverse from +0.168 to −0.168 rad (0.336 rad total). At MAX_STEER_RATE = 1.3 rad/s this takes 0.258 s → car travels ~2.9 m. The ~2.5 m changeover overshoot is an irreducible physical actuation constraint, not a tuning issue.

## Performance (v0.4.0)

| Phase | Lateral error |
|-------|--------------|
| Circle 1 (steady-state) | ±0.3 m |
| Changeover overshoot | ~2.5 m peak |
| Circle 2 | ±0.3 m (after recovery) |

## ROS Interface

Same topics as LQR+PID — no modifications to `skidpad_manager_node` required:

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| Subscribe | `/controllers/opt_requests` | `controller_msgs/OptRequest` |
| Subscribe | `/controllers/accel_request` | `controller_msgs/AccRequest` |
| Publish | `/controllers/opt_results` | `controller_msgs/OptResult` |
| Publish | `/controllers/long` | `controller_msgs/PIDErrors` |

## DUT25 Integration (Coupled MPC)

### Build — after controller_node.py change only

```bash
cd /dut
colcon build --packages-select coupled_mpc_controller
source install/setup.bash
```

### Build — after ocp_definition.py change (regenerates C code)

```bash
cd /dut
python3 src/controllers/coupled_mpc_controller/coupled_mpc_controller/ocp_definition.py
colcon build --packages-select coupled_mpc_controller
source install/setup.bash
```

### Launch

```bash
ros2 launch simulator simulation.launch.xml \
    mission_name:=skidpad perception:=sim state_estimation:=sim \
    rviz:=false controller_mode:=mpcc
```

### Monitoring in Foxglove Studio

Connect to `ws://localhost:8765`. Key topics:

| Topic | Content |
|-------|---------|
| `/controllers/opt_results` | Predicted trajectory + steer sequence |
| `/controllers/long` | Throttle command + PID diagnostics |
| `/embedded/to/TrajectorySetpoints` | Combined steer + throttle to simulator |
| `/viz/sim/real_car_pose` | Car position for 3D view |

---

## Repository Structure

```
lqr_pid_controller/
  controller_node.py               LQR lateral + PID longitudinal node
coupled_mpc_controller/
  ocp_definition.py                Acados OCP definition + C code generation
  controller_node.py               Coupled MPC ROS node
launch/
  lqr_pid_controller.launch.py     Launch file — LQR+PID parameters
  coupled_mpc_controller.launch.py Launch file — Coupled MPC parameters
CHANGELOG.md                       Full change history with dates and times
package.xml                        ROS 2 package manifest
setup.py                           Python package entry point
```

---

## Controller Comparison

Measured on DUT25 skidpad simulator (FSG layout, cruise speed 11.25 m/s).
`mpc_error_actual.error` field read from Foxglove Studio.

| Controller | C1 peak | SS | Changeover / C2 peak | Completes run |
|------------|---------|-----|----------------------|---------------|
| LQR+PID (lookahead=0, no feedforward) | 0.35 m | −0.10 m | diverges >9.5 m | No |
| LQR+PID v0.3.7 (lookahead=40) | 0.76 m | −0.47 m | 0.52 m | Yes |
| Decoupled MPC (mpc_python, DUT team) | 0.15 m | ~0 m | −0.85 m | Yes |
| Coupled MPC v0.4.0 (ifp2107) | 0.15 m | −0.10 m | −2.8 m | No |

**Key findings:**
- Curvature feedforward (lookahead=40) is essential for LQR — without it the controller diverges at C2 entry (>9.5 m error)
- Decoupled MPC achieves the best steady-state tracking (SS≈0) and completes both circles
- Coupled MPC matches Decoupled MPC on C1/SS but exits track at changeover — actuation-limited steer reversal (0.336 rad at 1.3 rad/s max = 0.26 s, ~2.9 m travel) is an irreducible physical constraint
- Computation: LQR <1 ms (matrix multiply); Coupled MPC ~3–8 ms (SQP_RTI NLP)
