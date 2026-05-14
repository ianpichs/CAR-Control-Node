# ifp2107 — Ian Pichs, Columbia University, ELEN 6760 Spring 2026
"""
controller_node.py — DUT25 LQR Lateral + PID Longitudinal Controller
Author: Ian Pichs (ifp2107) — Columbia University
=====================================================================

PURPOSE
-------
Drop-in replacement for the mpc_python_exec + longitudinal_control node pair
in the DUT25 skidpad pipeline. Implements:
  - Discrete-time LQR for lateral steering control
  - Gain-scheduled PID with speed feed-forward for longitudinal control

SYSTEM MODEL — LATERAL
-----------------------
The lateral controller uses a linearized dynamic bicycle model in the
path-following error frame. The state vector is:

    x = [e_y, e_psi, vy, r, steer]

where:
    e_y    = lateral deviation from the reference path         [m]
    e_psi  = heading error (vehicle yaw − path heading)        [rad]
    vy     = lateral velocity (body frame)                     [m/s]
    r      = yaw rate                                          [rad/s]
    steer  = front wheel steering angle                        [rad]

The control input is:
    u = steering_rate  (dsteer/dt)                             [rad/s]

Because steering_rate is the input (not steer directly), the steering angle
integrates the control command and appears as a state. This matches the DUT25
OptResult interface which expects a steering angle prediction sequence.

Continuous-time dynamics (linearized at constant vx, small-angle assumption):

    ė_y    =  vy + vx · e_psi
    ė_psi  =  r
    d_vy   = −(Cf+Cr)/(m·vx)·vy  +  (−vx + (Cr·lr−Cf·lf)/(m·vx))·r  − Cf/m·steer
    d_r    =  (lr·Cr−lf·Cf)/(Iz·vx)·vy  −  (lf²·Cf+lr²·Cr)/(Iz·vx)·r  − lf·Cf/Iz·steer
    d_steer =  u

This is discretized with Zero-Order Hold (ZOH) at timestep DT_TRAJ and the
optimal gain K is solved offline via the Discrete Algebraic Riccati Equation
(DARE):

    P = A_d^T P A_d − (A_d^T P B_d)(R + B_d^T P B_d)^−1(B_d^T P A_d) + Q
    K = (R + B_d^T P B_d)^−1 (B_d^T P A_d)

Control law:  u* = −K x   (drives all error states to zero)

SYSTEM MODEL — LONGITUDINAL
-----------------------------
A gain-scheduled PID with anti-windup tracks the reference speed published
by skidpad_manager_node. Three gain sets cover low / mid / high speed ranges,
matching the existing DUT25 longitudinal_control node parameters. A quadratic
feed-forward term (derived from measured vehicle throttle vs speed data) reduces
steady-state error at cruise speed.

ROS INTERFACE
-------------
Subscribes:
    /controllers/opt_requests     (controller_msgs/OptRequest)
        Reference path (N+1 waypoints) + current vehicle state x0.
        Published by skidpad_manager_node every ~25 ms (40 Hz).

    /controllers/accel_request    (controller_msgs/AccRequest)
        Speed setpoint + current speed. Published by skidpad_manager_node
        at ~250 Hz (every ASControlsEstimations tick).

Publishes:
    /controllers/opt_results      (controller_msgs/OptResult)
        Predicted steer angle sequence (N+1 values) used by skidpad_manager
        for time-interpolated latency compensation. Also contains predicted
        trajectory for Foxglove visualization and tracking error comparison.

    /controllers/long             (controller_msgs/PIDErrors)
        Longitudinal PID diagnostics + throttle command. skidpad_manager reads
        PIDErrors.input as the acceleration/throttle value and combines it
        with steer from OptResult to publish TrajectorySetpoints to the simulator.

HOW skidpad_manager USES OptResult
------------------------------------
The manager stores OptResult.steer as a 101-element buffer indexed over
[0, 1.0] seconds. When it needs to actuate, it interpolates steer at:

    t = ascontrols_since_opt_start × 0.004 + 0.15   [s]

The 0.15 s offset (buffer_delay_advance) pre-compensates for processing latency.
Our node must therefore produce exactly N_HORIZON+1 = 101 steer values whose
indices represent predicted wheel angle at each 10 ms step over 1 second.

TUNING GUIDE
------------
LQR (lateral) — adjust Q/R via ROS parameters or launch file:
    Start from MPC Q/R weights (already used as defaults):
      Q[e_y]=10, Q[e_psi]=1, Q[steer]=1, R=1
    Increase Q[e_y]   → tighter path tracking, more aggressive corrections.
    Increase Q[e_psi] → faster heading alignment.
    Increase Q[vy]    → damps lateral oscillation (start at 0, raise slowly).
    Increase R        → smoother steering at the cost of tracking bandwidth.
    Relaunch node after each change; K is recomputed at startup.

Lookahead feedforward — adjust via lookahead_steps in launch file:
    Controls how many waypoints ahead the curvature feedforward reads.
    Waypoint spacing = vx_op × dt_traj = 11.25 × 0.01 = 0.1125 m/waypoint.
    lookahead_steps=40 → 4.5 m / 0.4 s preview at 11.25 m/s (current optimum).
    Increase → earlier pre-steering before circle entries; reduces entry peak
               at the cost of pre-steering while still on the straight if too
               large (car begins curving before the geometric entry point).
    Tested range: 20/25 (worse — pre-steer on straight), 40 (best — C2 peak ÷14
    vs baseline), 60 (first early-turn at C2 but C1 regression +1.3m).

PID (longitudinal) — adjust via kp_list / ki_list in launch file or yaml:
    Values match DUT25 longitudinal_control node — only change if longitudinal
    performance differs from the MPC baseline.

DEPENDENCIES (inside DUT25 Docker container)
--------------------------------------------
    controller_msgs  (DUT25 interfaces package — provides OptRequest, OptResult,
                      AccRequest, PIDErrors)
    numpy, scipy, rclpy
"""

# ifp2107 — all code in this file is original work by Ian Pichs (ifp2107)

import math

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

from controller_msgs.msg import AccRequest, OptRequest, OptResult, PIDErrors


# ifp2107: VEHICLE PARAMETERS — DUT25 measured values
# =============================================================================
# VEHICLE PARAMETERS — DUT25 measured values
# Sources: parameters_LPV.yaml (m, Iz, wbase, x_cg, max_steering_angle,
#          max_steering_rate) and skidpad speed reference
# =============================================================================

M = 180.0    # total vehicle mass                         [kg]
IZ = 294.0   # yaw moment of inertia                      [kg·m²]
LF = 0.872   # CoG to front axle  = wbase × (1 − x_cg)   [m]
LR = 0.658   # CoG to rear axle   = wbase × x_cg          [m]

# Cornering stiffness at the skidpad operating point.
# Computed from the DUT25 tyre stiffness curve used by LPVMPC.get_tyre_stiffness:
#   C_data_x = [300, 500, 700, 900] N  (normal load per tyre)
#   C_data_y = [15374, 24177, 31211, 36360] N/rad
# Front per-tyre load = m·g·(1−x_cg)/2 = 180·9.81·0.43/2 ≈ 379.6 N → 18 877 N/rad
# Rear  per-tyre load = m·g·x_cg/2     = 180·9.81·0.57/2 ≈ 503.3 N → 24 293 N/rad
# (x_cg = 0.57 = rear weight fraction; LF = 0.872 m, LR = 0.658 m)
CF = 18877.0   # front axle cornering stiffness           [N/rad]
CR = 24293.0   # rear axle cornering stiffness            [N/rad]

# Skidpad constant-speed target — the LQR is linearized at this speed.
VX_OP = 11.25  # [m/s]

# Gain-scheduling lookup table: vx values at which K is pre-computed.
# At runtime the two bracketing entries are linearly interpolated using the
# current measured speed from AccRequest. Covers the full acceleration phase
# (startup → cruise). Values below VX_SCHEDULE[0] clamp to the first entry.
VX_SCHEDULE = [3.0, 5.0, 7.0, 9.0, 11.25]  # [m/s]


# ifp2107: ACTUATOR LIMITS — DUT25 spec (from parameters_LPV.yaml)
# =============================================================================
# ACTUATOR LIMITS — DUT25 spec (from parameters_LPV.yaml)
# =============================================================================

MAX_STEER_ANGLE = 0.4   # maximum front wheel steering angle   [rad]
MAX_STEER_RATE = 1.3    # maximum steering rate                 [rad/s]
LOOKAHEAD_STEPS = 40    # waypoints ahead for curvature feedforward preview (0 = nearest waypoint only)


# ifp2107: TRAJECTORY PREDICTION PARAMETERS — must match skidpad_manager_node
# =============================================================================
# TRAJECTORY PREDICTION PARAMETERS
# Must match skidpad_manager_node parameters (Nt=100, Tf=1.0) exactly.
# The manager creates optimization_node_times = linspace(0, Tf, Nt+1) and
# builds an interp1d over OptResult.steer — array length must be Nt+1 = 101.
# =============================================================================

N_HORIZON = 100     # prediction steps (Nt in skidpad_manager)
T_HORIZON = 1.0     # prediction time window                   [s]
DT_TRAJ = T_HORIZON / N_HORIZON  # step size = 0.01 s (10 ms)


# ifp2107: LQR COST MATRICES — State: [e_y, e_psi, vy, r, steer]  Control: [steering_rate]
# =============================================================================
# LQR COST MATRICES
# State: [e_y, e_psi, vy, r, steer]   Control: [steering_rate]
#
# Initial values mirror the MPC Q/R weights from parameters_LPV.yaml:
#   MPC Q: pos_y=10, heading=1, steer=1  (pos_x, vy, r not penalised)
#   MPC R: steering_rate=1
# All entries are exposed as ROS parameters for runtime tuning.
# =============================================================================

Q_MATRIX = np.diag([
    4.0,    # e_y   — lateral path error     (tuned v0.3.7; ceiling at 4.0 for current damping)
    1.0,    # e_psi — heading error           (floor at 1.0; lower causes divergence)
    5.0,    # vy    — lateral velocity        (tuned v0.3.7; 7.0 gives C1 best but C2 regression)
    8.0,    # r     — yaw rate               (tuned v0.3.7; raising 4→8 clearest session gain)
    1.0,    # steer — steering angle          (DARE-insensitive in this regime; leave at 1.0)
])

R_MATRIX = np.array([[1.0]])   # steering_rate cost  (matches MPC R = 1)


# ifp2107: LONGITUDINAL PID PARAMETERS — gain-scheduled, three speed ranges
# =============================================================================
# LONGITUDINAL PID — parameters matching DUT25 longitudinal_control node
# Source: conventional/config/longitudinal_pid_parameters.yaml
#
# Three gain sets indexed by speed range:
#   index 0: speed < LOWER_SPEED     (low-speed / startup)
#   index 1: LOWER_SPEED ≤ speed < UPPER_SPEED  (transition)
#   index 2: speed ≥ UPPER_SPEED     (cruise / skidpad operating range)
# =============================================================================

LONG_KP_LIST = [4.0, 4.0, 4.0]     # proportional gains per speed range
LONG_KI_LIST = [1.3, 1.3, 1.3]     # integral gains per speed range
LONG_KD_LIST = [0.0, 0.0, 0.0]     # derivative gains — disabled to match
                                     # baseline (set >0 with caution; amplifies
                                     # speed measurement noise)
LOWER_SPEED  = 4.0    # gain-schedule lower threshold   [m/s]
UPPER_SPEED  = 8.0    # gain-schedule upper threshold   [m/s]
MAX_THROTTLE = 15.0   # maximum throttle output (acceleration units)
MAX_BRAKING  = 20.0   # maximum braking magnitude (stored as −MAX_BRAKING)
MAX_ACC_RATE = 3.0    # max throttle Δ per PID step — jerk limiter

# PID integration timestep.
# AccRequest is published inside skidpad_manager's ASControlsEstimations
# callback which fires at 250 Hz (0.004 s per tick).
DT_PID = 0.004   # [s]

# Feed-forward: quadratic throttle vs speed curve.
# Fitted from measured DUT25 data in speed_ff_term_calc.py.
# throttle_ff = FF_A·v² + FF_B·v + FF_C
_FF_SPEED    = np.array([1.937, 4.097, 9.598, 13.004, 15.959, 20.223, 22.07])
_FF_THROTTLE = np.array([0.189, 0.3081, 0.721, 1.189,  1.624,  2.361,  2.76])
_FF_COEFS    = np.linalg.lstsq(
    np.vstack([_FF_SPEED ** 2, _FF_SPEED, np.ones_like(_FF_SPEED)]).T,
    _FF_THROTTLE,
    rcond=None,
)[0]
FF_A, FF_B, FF_C = _FF_COEFS


# ifp2107: HELPER FUNCTIONS
# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def normalize_angle(angle: float) -> float:
    """Wrap an angle to [−π, π] to prevent discontinuities near ±180°."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def build_dynamic_lateral_system(vx: float) -> tuple:
    """
    Build continuous-time (A_c, B_c) for the linearized dynamic bicycle model.

    State:   x = [e_y, e_psi, vy, r, steer]
    Control: u = [steering_rate]

    Linearization point: vx = operating speed, all error states = 0.
    Path curvature feed-forward (r_ref = vx/R_path) is omitted because the
    reference curvature is already encoded in the waypoint path supplied by
    skidpad_manager; the LQR regulates deviations from that path to zero.

    Dynamics:
      ė_y    =  vy + vx·e_psi           (geometry: lateral error grows with heading)
      ė_psi  =  r                        (heading error integrates yaw rate)
      d_vy   = −(Cf+Cr)/(m·vx)·vy
               + (−vx + (Cr·lr−Cf·lf)/(m·vx))·r
               − Cf/m·steer             (Newton: lateral force from tyre slip)
      d_r    =  (lr·Cr−lf·Cf)/(Iz·vx)·vy
               − (lf²·Cf+lr²·Cr)/(Iz·vx)·r
               − lf·Cf/Iz·steer         (Newton: yaw moment from tyre forces)
      d_steer =  u                       (steer integrates the control rate input)

    Returns:
        A_c: (5×5) continuous state matrix
        B_c: (5×1) continuous input matrix
    """
    A_c = np.array([
        [0.0,  vx,   1.0,
         0.0,
         0.0],
        [0.0,  0.0,  0.0,
         1.0,
         0.0],
        [0.0,  0.0,  -(CF + CR) / (M * vx),
         (-vx + (CR * LR - CF * LF) / (M * vx)),
         -CF / M],
        [0.0,  0.0,  (LR * CR - LF * CF) / (IZ * vx),
         -(LF ** 2 * CF + LR ** 2 * CR) / (IZ * vx),
         -LF * CF / IZ],
        [0.0,  0.0,  0.0,
         0.0,
         0.0],
    ], dtype=float)

    # steering_rate enters only row 4 (d_steer = u); its effect on vy and r
    # propagates through the A matrix over successive timesteps.
    B_c = np.array([[0.0], [0.0], [0.0], [0.0], [1.0]], dtype=float)

    return A_c, B_c


def discretize_system(A_c: np.ndarray, B_c: np.ndarray, dt: float) -> tuple:
    """
    Discretize (A_c, B_c) using Zero-Order Hold (ZOH) at timestep dt.

    ZOH assumes the control input is constant between samples, which is the
    correct assumption for a digital controller that outputs one command per
    timestep and holds it until the next update.

    Returns:
        A_d: (5×5) discrete state matrix
        B_d: (5×1) discrete input matrix
    """
    A_d, B_d, _, _, _ = cont2discrete(
        (A_c, B_c, np.eye(5), np.zeros((5, 1))), dt=dt, method="zoh"
    )
    return A_d, B_d


def solve_dare(A_d: np.ndarray, B_d: np.ndarray,
               Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """
    Solve the Discrete Algebraic Riccati Equation (DARE) and return K.

    DARE:  P = A_d^T P A_d − (A_d^T P B_d)(R + B_d^T P B_d)^−1(B_d^T P A_d) + Q
    Gain:  K = (R + B_d^T P B_d)^−1 (B_d^T P A_d)

    The control law u* = −K x minimises the infinite-horizon quadratic cost:
        J = Σ (x^T Q x  +  u^T R u)

    The DARE is solved once at startup; K does not change during the mission.

    Returns:
        K: (1×5) optimal state-feedback gain matrix
    """
    P = solve_discrete_are(A_d, B_d, Q, R)
    K = np.linalg.solve(R + B_d.T @ P @ B_d, B_d.T @ P @ A_d)
    return K


# ifp2107: ROS 2 NODE — LQR lateral + gain-scheduled PID longitudinal
# =============================================================================
# ROS 2 NODE
# =============================================================================

class LqrPidController(Node):
    """
    ROS 2 node: LQR lateral + gain-scheduled PID longitudinal controller.

    Replaces mpc_python_exec and longitudinal_control in the DUT25 skidpad
    pipeline. skidpad_manager_node requires no modification — this node
    publishes on the same topics with the same message types.
    """

    def __init__(self) -> None:
        super().__init__("lqr_pid_controller")

        # ------------------------------------------------------------------
        # ROS Parameters — all tuning constants are exposed here so that
        # Q/R weights and PID gains can be changed without modifying source.
        # ------------------------------------------------------------------
        self._declare_parameters()
        self._load_parameters()

        # ------------------------------------------------------------------
        # Pre-compute LQR gain table at startup. K is interpolated at runtime
        # from the current measured speed so the controller remains optimal
        # during the acceleration phase, not just at cruise speed.
        # ------------------------------------------------------------------
        self._current_vx: float = self._vx_op  # updated by _on_acc_request
        self._build_lqr_table()

        # ------------------------------------------------------------------
        # Longitudinal PID integrator state
        # ------------------------------------------------------------------
        self._speed_integral: float = 0.0    # accumulated integral term
        self._last_throttle: float  = 0.0    # previous output for jerk limiter
        self._kp: float = LONG_KP_LIST[0]    # active P gain (updated by schedule)
        self._ki: float = LONG_KI_LIST[0]    # active I gain
        self._kd: float = LONG_KD_LIST[0]    # active D gain

        # ------------------------------------------------------------------
        # Publishers and subscribers
        # ------------------------------------------------------------------
        self._opt_result_pub = self.create_publisher(
            OptResult, "/controllers/opt_results", 1
        )
        self._long_pub = self.create_publisher(
            PIDErrors, "/controllers/long", 1
        )

        # Queue depth 1: always process the latest message, discard stale ones.
        self.create_subscription(
            OptRequest, "/controllers/opt_requests", self._on_opt_request, 1
        )
        self.create_subscription(
            AccRequest, "/controllers/accel_request", self._on_acc_request, 1
        )

        self.get_logger().info(
            f"LQR-PID controller initialised.\n"
            f"  vx_schedule = {self._vx_schedule} m/s\n"
            f"  N      = {self._n_horizon} steps\n"
            f"  Tf     = {self._t_horizon:.2f} s\n"
            f"  dt_traj= {self._dt_traj:.4f} s\n"
            f"  K_table entries = {len(self._K_table)}"
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with module-level constants as defaults."""
        self.declare_parameter("vx_operating",      VX_OP)
        self.declare_parameter("vx_schedule",       VX_SCHEDULE)
        self.declare_parameter("n_horizon",          N_HORIZON)
        self.declare_parameter("t_horizon",          T_HORIZON)
        self.declare_parameter("max_steer_angle",    MAX_STEER_ANGLE)
        self.declare_parameter("max_steer_rate",     MAX_STEER_RATE)
        self.declare_parameter("lookahead_steps",    LOOKAHEAD_STEPS)
        # LQR cost entries declared individually for fine-grained tuning
        self.declare_parameter("q_e_y",        Q_MATRIX[0, 0])
        self.declare_parameter("q_e_psi",      Q_MATRIX[1, 1])
        self.declare_parameter("q_vy",         Q_MATRIX[2, 2])
        self.declare_parameter("q_r",          Q_MATRIX[3, 3])
        self.declare_parameter("q_steer",      Q_MATRIX[4, 4])
        self.declare_parameter("r_steer_rate", float(R_MATRIX[0, 0]))
        # Longitudinal parameters
        self.declare_parameter("lower_speed",   LOWER_SPEED)
        self.declare_parameter("upper_speed",   UPPER_SPEED)
        self.declare_parameter("max_throttle",  MAX_THROTTLE)
        self.declare_parameter("max_braking",   MAX_BRAKING)
        self.declare_parameter("max_acc_rate",  MAX_ACC_RATE)
        self.declare_parameter("kp_list",       LONG_KP_LIST)
        self.declare_parameter("ki_list",       LONG_KI_LIST)
        self.declare_parameter("kd_list",       LONG_KD_LIST)

    def _load_parameters(self) -> None:
        """Read declared ROS parameters into instance attributes."""
        p = lambda name: self.get_parameter(name).value

        self._vx_op      = float(p("vx_operating"))
        self._vx_schedule = [float(v) for v in p("vx_schedule")]
        self._n_horizon = int(p("n_horizon"))
        self._t_horizon = float(p("t_horizon"))
        self._dt_traj   = self._t_horizon / self._n_horizon
        self._max_steer = float(p("max_steer_angle"))
        self._max_steer_rate = float(p("max_steer_rate"))
        self._lookahead_steps = int(p("lookahead_steps"))

        # Rebuild Q from individual ROS parameters
        self._Q = np.diag([
            float(p("q_e_y")),
            float(p("q_e_psi")),
            float(p("q_vy")),
            float(p("q_r")),
            float(p("q_steer")),
        ])
        self._R = np.array([[float(p("r_steer_rate"))]])

        self._lower_speed  = float(p("lower_speed"))
        self._upper_speed  = float(p("upper_speed"))
        self._max_throttle = float(p("max_throttle"))
        self._max_braking  = -abs(float(p("max_braking")))  # stored negative
        self._max_acc_rate = float(p("max_acc_rate"))
        self._kp_list      = list(p("kp_list"))
        self._ki_list      = list(p("ki_list"))
        self._kd_list      = list(p("kd_list"))

    def _build_lqr_table(self) -> None:
        """
        Pre-compute a gain lookup table: one K per entry in vx_schedule.

        Each K is the DARE solution for the bicycle model linearized at that
        speed. At runtime _interpolate_K() blends adjacent entries based on
        the current measured speed from AccRequest.
        """
        self._K_table: list = []
        for vx in self._vx_schedule:
            A_c, B_c = build_dynamic_lateral_system(vx)
            A_d, B_d = discretize_system(A_c, B_c, self._dt_traj)
            K = solve_dare(A_d, B_d, self._Q, self._R)
            self._K_table.append(K)
            self.get_logger().info(f"K at vx={vx:.2f} m/s: {K}")

    def _interpolate_K(self, vx: float) -> np.ndarray:
        """
        Linearly interpolate K from the lookup table at the given speed.

        Clamps to the lowest/highest table entry if vx is out of range.
        """
        sched = self._vx_schedule
        table = self._K_table
        if vx <= sched[0]:
            return table[0]
        if vx >= sched[-1]:
            return table[-1]
        for i in range(len(sched) - 1):
            if sched[i] <= vx <= sched[i + 1]:
                alpha = (vx - sched[i]) / (sched[i + 1] - sched[i])
                return (1.0 - alpha) * table[i] + alpha * table[i + 1]
        return table[-1]

    # ------------------------------------------------------------------
    # LATERAL — OptRequest callback
    # ------------------------------------------------------------------

    def _on_opt_request(self, msg: OptRequest) -> None:
        """
        Compute and publish the LQR lateral control output.

        Called on every OptRequest from skidpad_manager (~40 Hz). The LQR
        responds immediately — computation time is a matrix multiply plus
        Euler integration, well within the 25 ms inter-message window.

        Steps:
          1. Extract current vehicle state x0 from the message.
          2. Find the nearest reference waypoint in the path array.
          3. Compute the 5-element LQR error state in the path-tangent frame.
          4. Apply u = −K x to get optimal steering_rate.
          5. Integrate steer forward N+1 steps to build the predicted angle
             sequence that skidpad_manager uses for latency compensation.
          6. Propagate the full nonlinear state N steps for visualization.
          7. Publish OptResult.
        """
        if len(msg.pos_x) == 0:
            self.get_logger().warning(
                "Received OptRequest with empty path — skipping.",
                throttle_duration_sec=2.0,
            )
            return

        x0 = msg.x0  # controller_msgs/State: pos_x, pos_y, heading, vy, r, steer

        # Step 1: nearest reference waypoint
        ref_idx = self._nearest_waypoint(x0.pos_x, x0.pos_y, msg)

        # Step 2: reference pose at that index
        psi_ref = math.atan2(msg.head_sin[ref_idx], msg.head_cos[ref_idx])
        x_ref   = msg.pos_x[ref_idx]
        y_ref   = msg.pos_y[ref_idx]

        # Step 3: error state in path-tangent frame
        dx = x0.pos_x - x_ref
        dy = x0.pos_y - y_ref

        # Lateral error: signed perpendicular distance from the path.
        # Positive when the vehicle is to the left of the reference direction.
        # Derived by projecting (dx, dy) onto the path-normal (−sin ψ, cos ψ).
        e_y = -dx * math.sin(psi_ref) + dy * math.cos(psi_ref)

        # Heading error: vehicle yaw minus path heading, wrapped to [−π, π].
        e_psi = normalize_angle(x0.heading - psi_ref)

        # Path curvature feed-forward — shift vy/r/steer references to the
        # steady-state values needed for the current path curve so the LQR
        # does not fight the non-zero steer and yaw rate required to follow
        # the circle. Without this, K[steer] and K[r] produce a large net
        # command opposing the turn at the moment circular motion begins.
        #
        # Curvature is read from a lookahead point rather than the nearest
        # waypoint so the LQR begins pre-steering before the car reaches the
        # geometric circle entry, reducing the circle-entry transient.
        preview_idx = min(ref_idx + self._lookahead_steps, len(msg.pos_x) - 2)
        kappa     = self._compute_path_curvature(preview_idx, msg)
        vx        = self._current_vx
        r_ref     = vx * kappa                        # expected yaw rate  [rad/s]
        steer_ref = -(LF + LR) * kappa                # kinematic steer    [rad]
        # vy_ref matches skidpad_manager's vy proxy: 0.2429 * vx * r
        vy_ref    = 0.2429 * vx * r_ref

        error_state = np.array([
            e_y,
            e_psi,
            x0.vy    - vy_ref,
            x0.r     - r_ref,
            x0.steer - steer_ref,
        ], dtype=float)

        # Step 4: LQR control law  u = −K x  (K interpolated at current speed)
        u_raw = float(-(self._interpolate_K(vx) @ error_state)[0])
        u = float(np.clip(u_raw, -self._max_steer_rate, self._max_steer_rate))

        # Step 5: predicted steer angle sequence (N+1 elements).
        # skidpad_manager uses interp1d over this array with time axis
        # linspace(0, Tf, N+1) to look up the steer angle at any future time,
        # compensating for computational latency (buffer_delay_advance = 0.15 s).
        # A constant steering_rate u produces a linear ramp in steer angle.
        steer_seq = [
            float(np.clip(
                x0.steer + u * k * self._dt_traj,
                -self._max_steer, self._max_steer,
            ))
            for k in range(self._n_horizon + 1)
        ]

        # Step 6: full nonlinear trajectory for visualization
        traj = self._propagate_trajectory(x0, u, vx)

        # Step 7: publish OptResult
        result = OptResult()
        result.header        = msg.header
        result.pos_x         = [float(s[0]) for s in traj]
        result.pos_y         = [float(s[1]) for s in traj]
        result.heading       = [float(s[2]) for s in traj]
        result.vy            = [float(s[3]) for s in traj]
        result.r             = [float(s[4]) for s in traj]
        result.steer         = steer_seq   # used by manager for actuation
        # steering_rate: constant LQR command repeated for all steps.
        # The LQR does not plan a time-varying rate sequence; the constant
        # value is included for completeness and Foxglove monitoring.
        result.steering_rate = [u] * (self._n_horizon + 1)
        result.solver_status = OptResult.SUCCESS

        self._opt_result_pub.publish(result)

        self.get_logger().info(
            f"LQR | "
            f"e_y={e_y:+.3f}m  e_psi={math.degrees(e_psi):+.1f}deg  "
            f"preview={preview_idx-ref_idx}wp  kappa={kappa:+.4f}/m  "
            f"r_ref={r_ref:+.3f}rad/s  steer_ref={steer_ref:+.3f}rad  "
            f"x0.r={x0.r:+.3f}rad/s  x0.steer={x0.steer:+.3f}rad  "
            f"vx={vx:.2f}m/s  u_raw={u_raw:+.4f}  u={u:+.4f}rad/s",
            throttle_duration_sec=0.25,
        )

    def _nearest_waypoint(self, px: float, py: float,
                          msg: OptRequest) -> int:
        """
        Return the index of the reference waypoint closest to (px, py).

        Using the nearest point ensures the error is always computed against
        the most relevant section of the path, which is important on a circular
        track where the path array wraps around.
        """
        xs = np.array(msg.pos_x)
        ys = np.array(msg.pos_y)
        return int(np.argmin(np.hypot(xs - px, ys - py)))

    def _compute_path_curvature(self, ref_idx: int, msg: OptRequest) -> float:
        """
        Estimate signed path curvature κ = dθ/ds at the reference waypoint.

        Positive κ = left turn (CCW), negative κ = right turn (CW).

        Curvature is the rate of heading change per unit arc length. It is
        computed from the heading angle difference between adjacent waypoints
        divided by their arc-length separation. This gives the feed-forward
        reference for yaw rate and steer angle needed to follow the curve
        without the LQR fighting the required steady-state values.
        """
        n = len(msg.pos_x)
        if n < 2:
            return 0.0
        i = min(ref_idx, n - 2)
        theta0 = math.atan2(msg.head_sin[i],     msg.head_cos[i])
        theta1 = math.atan2(msg.head_sin[i + 1], msg.head_cos[i + 1])
        ds = math.hypot(
            msg.pos_x[i + 1] - msg.pos_x[i],
            msg.pos_y[i + 1] - msg.pos_y[i],
        )
        if ds < 1e-6:
            return 0.0
        return normalize_angle(theta1 - theta0) / ds

    def _propagate_trajectory(self, x0, u: float, vx: float) -> list:
        """
        Propagate the nonlinear global-frame bicycle model forward N steps.

        Uses Euler integration with the constant steering_rate command u.
        The nonlinear (not linearized) model is used here so that the
        predicted trajectory correctly curves, giving an accurate visualization
        in Foxglove and a valid crosstrack error comparison against the MPC.

        Global state: [pos_x, pos_y, heading, vy, r, steer]

        Returns:
            List of N+1 state arrays — initial state at index 0.
        """
        state = np.array(
            [x0.pos_x, x0.pos_y, x0.heading, x0.vy, x0.r, x0.steer],
            dtype=float,
        )
        trajectory = [state.copy()]

        for _ in range(self._n_horizon):
            px, py, psi, vy, r, steer = state

            # Full nonlinear lateral bicycle dynamics (global frame)
            d_px    =  vx * math.cos(psi) - vy * math.sin(psi)
            d_py    =  vx * math.sin(psi) + vy * math.cos(psi)
            d_psi   =  r
            d_vy    = (-(CF + CR) / (M * vx) * vy
                       + (-vx + (CR * LR - CF * LF) / (M * vx)) * r
                       - CF / M * steer)
            d_r     = ((LR * CR - LF * CF) / (IZ * vx) * vy
                       - (LF ** 2 * CF + LR ** 2 * CR) / (IZ * vx) * r
                       - LF * CF / IZ * steer)
            d_steer =  u   # steering angle integrates the control rate

            state = state + self._dt_traj * np.array(
                [d_px, d_py, d_psi, d_vy, d_r, d_steer]
            )
            state[5] = float(np.clip(state[5], -self._max_steer, self._max_steer))
            trajectory.append(state.copy())

        return trajectory

    # ------------------------------------------------------------------
    # LONGITUDINAL — AccRequest callback
    # ------------------------------------------------------------------

    def _on_acc_request(self, msg: AccRequest) -> None:
        """
        Compute and publish the PID longitudinal control output.

        Called at ~250 Hz by skidpad_manager (once per ASControlsEstimations
        message). skidpad_manager reads PIDErrors.input as the throttle value
        and places it in TrajectorySetpoints.acceleration for the simulator.

        Features matching the DUT25 longitudinal_control node:
          - Three-speed gain schedule (low / mid / high)
          - Quadratic feed-forward from measured vehicle data
          - Anti-windup: integrates only when P and I terms are within limits
          - Rate limiter on output to prevent jerk
          - Hard brake mode: outputs −20 when braking flag is set
        """
        current_speed = msg.current_speed
        desired_speed = msg.desired_speed

        # Cache for lateral controller — clamp to minimum schedule speed to
        # avoid extrapolating below the lowest linearization point.
        self._current_vx = float(max(current_speed, self._vx_schedule[0]))

        # Update gain set based on current speed
        self._update_long_gains(current_speed)

        # Speed error — positive means car is slower than desired
        error = desired_speed - current_speed

        # Anti-windup check: only accumulate integral when both the
        # proportional term and the prospective integral term lie within the
        # output saturation limits. Prevents integrator wind-up when the
        # throttle is saturated (e.g., during acceleration from rest).
        p_term        = self._kp * error
        next_integral = self._speed_integral + error * DT_PID
        i_term        = self._ki * next_integral
        if (self._max_braking < p_term < self._max_throttle and
                self._max_braking < i_term < self._max_throttle):
            self._speed_integral = next_integral

        # Derivative term disabled — matches DUT25 baseline where the
        # Savitzky-Golay smoothed derivative is zeroed out in the sim version.
        d_term = 0.0

        if msg.braking:
            throttle = -20.0  # hard brake command, matching existing stack
        else:
            # Feed-forward reduces steady-state error at cruise speed.
            # Only active above 1 m/s to avoid noise influence at rest.
            ff = 0.0
            if current_speed > 1.0:
                ff = FF_A * current_speed ** 2 + FF_B * current_speed + FF_C

            throttle = float(np.clip(
                p_term
                + self._ki * self._speed_integral
                + d_term
                + ff
                + msg.desired_acceleration,  # manager-supplied acceleration request
                self._max_braking,
                self._max_throttle,
            ))

        # Rate limiter: clamp throttle change per step to prevent sudden
        # acceleration demands that the vehicle cannot physically achieve.
        throttle = float(np.clip(
            throttle,
            self._last_throttle - self._max_acc_rate,
            self._last_throttle + self._max_acc_rate,
        ))
        self._last_throttle = throttle

        # Publish PIDErrors — .input is the field skidpad_manager reads
        # as the throttle/acceleration command for TrajectorySetpoints.
        pid_msg = PIDErrors()
        pid_msg.ref        = float(desired_speed)
        pid_msg.current    = float(current_speed)
        pid_msg.error      = float(error)
        pid_msg.integral   = float(self._speed_integral)
        pid_msg.derivative = d_term
        pid_msg.input      = throttle
        self._long_pub.publish(pid_msg)

    def _update_long_gains(self, speed: float) -> None:
        """
        Select the PID gain set for the current speed.

        Three sets cover startup, transition, and cruise speed ranges.
        Matching the gain-scheduling logic of the DUT25 longitudinal_control
        node ensures longitudinal behaviour is comparable to the MPC baseline.
        """
        if speed >= self._upper_speed:
            idx = 2
        elif speed >= self._lower_speed:
            idx = 1
        else:
            idx = 0
        self._kp = self._kp_list[idx]
        self._ki = self._ki_list[idx]
        self._kd = self._kd_list[idx]


# ifp2107: ENTRY POINT
# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LqrPidController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
