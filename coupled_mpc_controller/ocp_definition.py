"""
ocp_definition.py — Coupled MPC Optimal Control Problem Definition
===================================================================
Author:  Ian Pichs (ifp2107) — Columbia University
Course:  ELEN 6760 Discrete Control Systems, Spring 2026
Project: Evolution of Control Architectures for High-Speed Autonomous Racing

PURPOSE
-------
Defines the Nonlinear Optimal Control Problem (OCP) for the Coupled MPC
controller and generates a compiled acados C solver via CasADi symbolic
differentiation. This script is run ONCE offline (inside the Docker container)
to produce the solver shared library used at runtime by controller_node.py.

RUN ONCE inside the DUT25 Docker container:
    cd /dut && python3 src/controllers/coupled_mpc_controller/coupled_mpc_controller/ocp_definition.py

Outputs:
    /dut/coupled_mpc_c_generated_code/   ← compiled C solver (.so + headers)
    /dut/coupled_mpc_ocp.json            ← OCP description for solver reload

WHAT MAKES THIS "COUPLED"
--------------------------
The Decoupled MPC (mpc_python) treats longitudinal velocity vx as a fixed
parameter, solving only for steering. This OCP extends the state to include
vx as a dynamic variable and adds throttle as a second control input. The
two are coupled through the friction circle constraint:

    (ax / ax_max)^2 + (ay / ay_max)^2 <= 1

which forces the solver to trade lateral grip against acceleration whenever
the combined demand approaches the tyre limit. This coupling is what allows
the Coupled MPC to optimally manage the friction budget at circle entries
(C1, C2) where lateral demand spikes simultaneously with speed changes.

STATE VECTOR x (7 states):
    [pos_x, pos_y, psi, vy, r, steer, vx]
     global   global  heading  lat   yaw  front   long
     x [m]    y [m]  [rad]  vel[m/s] rate[r/s] steer[rad] vel[m/s]
                                                              ^--- NEW

CONTROL VECTOR u (2 inputs):
    [steering_rate, throttle]
     d(steer)/dt     net long. accel
     [rad/s]         [m/s^2]
                         ^--- NEW

HORIZON: N=40 steps, dt=25ms, Tf=1.0s
    Matches existing Decoupled MPC so skidpad_manager can be launched
    with Nt=40 in both MPC and Coupled MPC modes without modification.

SOLVER: SQP_RTI (Real-Time Iteration)
    One SQP iteration per 25ms timestep, warm-started from the previous
    solution. This keeps solve time well within the 25ms budget while
    tracking the time-varying optimal solution as the vehicle moves.
"""

# ifp2107 — all code in this file is original work by Ian Pichs (ifp2107)
# Vehicle parameters are sourced from the DUT25 measured dataset and are
# shared with lqr_pid_controller/controller_node.py (same values).

import os
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


# =============================================================================
# ifp2107: VEHICLE PARAMETERS — DUT25 measured values
# Source: parameters_LPV.yaml and tyre stiffness curve (same as LQR node)
# =============================================================================

M   = 180.0     # vehicle mass                          [kg]
IZ  = 294.0     # yaw moment of inertia                 [kg·m²]
LF  = 0.872     # CoG to front axle                     [m]
LR  = 0.658     # CoG to rear axle                      [m]
CF  = 18877.0   # front axle cornering stiffness        [N/rad]
CR  = 24293.0   # rear axle cornering stiffness         [N/rad]

# ifp2107: Drag coefficient — models combined aerodynamic + rolling resistance.
# Formulation: a_drag = DRAG_COEFF * vx^2  (acceleration units, m/s^2)
# Fitted so that at vx=11.25 m/s the drag acceleration matches the LQR
# feed-forward steady-state value (~0.95 m/s^2 from speed_ff_term_calc data).
# DRAG_COEFF = 0.95 / 11.25^2 ≈ 0.0075
DRAG_COEFF = 0.0075    # [1/m] — tune if longitudinal speed drifts at cruise


# =============================================================================
# ifp2107: FRICTION CIRCLE PARAMETERS
# Defines the tyre grip ellipse: (ax/ax_max)^2 + (ay/ay_max)^2 <= 1
# ax_max: maximum longitudinal acceleration (throttle limit ≡ MAX_THROTTLE)
# ay_max: maximum lateral acceleration (~1.2g for DUT25 on skidpad surface)
# SLACK_PENALTY: cost weight on friction circle violation (slack variable);
#   higher values enforce the constraint more strictly at the cost of
#   occasional solver infeasibility near the limit.
# =============================================================================
AX_MAX        = 15.0    # [m/s^2] — matches MAX_THROTTLE controller scale
# ifp2107: AY_MAX = 15.0 m/s² (~1.53g). Track map gives circle radius = 9.125m;
# at v=11.25 m/s cruise: ay = v²/R ≈ 13.87 m/s². AY_MAX must exceed cruise ay so
# the friction circle is not permanently violated (which ill-conditions HPIPM).
# 15.0 gives ~8% margin at cruise; constraint becomes active when throttle is added.
AY_MAX        = 15.0    # [m/s^2] — ~1.53g lateral (DUT25 skidpad, R=9.125m)
SLACK_PENALTY = 200.0   # quadratic penalty per unit of friction circle slack


# =============================================================================
# ifp2107: HORIZON AND TIMING
# N=40, dt=25ms matches the existing Decoupled MPC OCP (acados_ocp.json).
# skidpad_manager is launched with Nt=40 in coupled MPC mode, so it builds
# its interp1d steer buffer over linspace(0, 1.0, 41) — 41 elements.
# =============================================================================
N_HORIZON = 40
DT        = 0.025           # timestep [s] — 25 ms per step (40 Hz)
TF        = N_HORIZON * DT  # total horizon [s] = 1.0s


# =============================================================================
# ifp2107: ACTUATOR LIMITS
# Steering limits match DUT25 spec and LQR controller values.
# Throttle limits match the PID output range in lqr_pid_controller.
# =============================================================================
MAX_STEER_ANGLE = 0.4    # [rad]
MAX_STEER_RATE  = 1.3    # [rad/s]
MAX_THROTTLE    = 15.0   # [m/s^2 equivalent] — max acceleration command
MAX_BRAKING     = 20.0   # [m/s^2 equivalent] — max braking command


# =============================================================================
# ifp2107: PATHS
# Output paths are inside the Docker container (/dut maps to the
# host-mounted DUT25-Autonomous workspace). Keeping generated code at
# /dut/coupled_mpc_c_generated_code/ avoids overwriting the Decoupled MPC
# solver at /dut/c_generated_code/.
# =============================================================================
CODE_EXPORT_DIR = '/dut/coupled_mpc_c_generated_code'
OCP_JSON_PATH   = '/dut/coupled_mpc_ocp.json'
MODEL_NAME      = 'coupled_mpc'


# =============================================================================
# ifp2107: MODEL DEFINITION
# =============================================================================

def build_model() -> AcadosModel:
    """
    ifp2107: Construct the CasADi symbolic model for the Coupled MPC OCP.

    PSEUDO-CODE
    -----------
    Input:  None (uses module-level vehicle constants)
    Output: AcadosModel containing:
              - Symbolic state x (7), control u (2)
              - Explicit continuous-time ODE f_expl
              - Nonlinear h-constraint for the friction circle

    DYNAMICS
    --------
    Global-frame nonlinear bicycle model extended with longitudinal state vx.
    The lateral equations are identical to the LQR bicycle model but vx is
    now symbolic (time-varying), making all vx-dependent terms nonlinear.
    CasADi computes Jacobians automatically — no manual linearization needed.

    d_pos_x = vx*cos(psi) - vy*sin(psi)
    d_pos_y = vx*sin(psi) + vy*cos(psi)
    d_psi   = r
    d_vy    = -(Cf+Cr)/(m*vx)*vy + (-vx + (Cr*lr-Cf*lf)/(m*vx))*r - Cf/m*steer
    d_r     = (lr*Cr-lf*Cf)/(Iz*vx)*vy - (lf^2*Cf+lr^2*Cr)/(Iz*vx)*r - lf*Cf/Iz*steer
    d_steer = u_steer_rate
    d_vx    = u_throttle - DRAG_COEFF * vx^2
    """
    model     = AcadosModel()
    model.name = MODEL_NAME

    # ------------------------------------------------------------------
    # ifp2107: State variables (7)
    # ------------------------------------------------------------------
    pos_x = ca.SX.sym('pos_x')   # global x position          [m]
    pos_y = ca.SX.sym('pos_y')   # global y position          [m]
    psi   = ca.SX.sym('psi')     # vehicle heading            [rad]
    vy    = ca.SX.sym('vy')      # lateral body velocity      [m/s]
    r     = ca.SX.sym('r')       # yaw rate                   [rad/s]
    steer = ca.SX.sym('steer')   # front wheel steer angle    [rad]
    vx    = ca.SX.sym('vx')      # longitudinal velocity      [m/s] — NEW STATE

    x        = ca.vertcat(pos_x, pos_y, psi, vy, r, steer, vx)
    model.x  = x
    model.xdot = ca.SX.sym('xdot', 7)

    # ------------------------------------------------------------------
    # ifp2107: Control inputs (2)
    # ------------------------------------------------------------------
    u_steer_rate = ca.SX.sym('u_steer_rate')  # steering rate [rad/s]
    u_throttle   = ca.SX.sym('u_throttle')    # long. accel   [m/s^2] — NEW INPUT

    u       = ca.vertcat(u_steer_rate, u_throttle)
    model.u = u

    # ------------------------------------------------------------------
    # ifp2107: Numerically safe vx — prevents division by zero at standstill.
    # The bicycle model is invalid at very low speed (tyre slip angles undefined).
    # ca.fmax clamps the Jacobian denominator so CasADi gradients stay finite.
    # Raised from 0.5 to 1.0: at vx=0.5 the gradient of fmax(vx,0.5) w.r.t. vx
    # is zero (the clamp is active and vx=0.5 exactly), giving a degenerate
    # Jacobian that causes permanent QP failures when the car stops. At 1.0 the
    # clamp only becomes active below 1.0 m/s, a speed the car should not sustain.
    # ------------------------------------------------------------------
    vx_safe = ca.fmax(vx, 1.0)

    # ------------------------------------------------------------------
    # ifp2107: Lateral acceleration (used in friction circle constraint).
    # In the body frame: ay = d_vy + vx*r
    #   d_vy  = lateral velocity rate (from tyre forces)
    #   vx*r  = centripetal acceleration component
    # This is the total lateral acceleration experienced by the vehicle.
    # ------------------------------------------------------------------
    d_vy = (
        -(CF + CR) / (M * vx_safe) * vy
        + (-vx_safe + (CR * LR - CF * LF) / (M * vx_safe)) * r
        - CF / M * steer
    )
    ay_body = d_vy + vx_safe * r   # body-frame lateral acceleration [m/s^2]

    # ------------------------------------------------------------------
    # ifp2107: Yaw dynamics
    # ------------------------------------------------------------------
    d_r = (
        (LR * CR - LF * CF) / (IZ * vx_safe) * vy
        - (LF**2 * CF + LR**2 * CR) / (IZ * vx_safe) * r
        - LF * CF / IZ * steer
    )

    # ------------------------------------------------------------------
    # ifp2107: Longitudinal dynamics — NEW for coupled MPC.
    # d_vx = throttle - drag
    # throttle = u_throttle (acceleration command, m/s^2 equivalent)
    # drag     = DRAG_COEFF * vx^2 (quadratic aerodynamic + rolling resistance)
    # At cruise: u_throttle ≈ DRAG_COEFF * vx^2 → d_vx ≈ 0 (speed maintained)
    # ------------------------------------------------------------------
    d_vx = u_throttle - DRAG_COEFF * vx**2

    # ------------------------------------------------------------------
    # ifp2107: Full explicit continuous-time ODE  xdot = f(x, u)
    # ------------------------------------------------------------------
    f_expl = ca.vertcat(
        vx * ca.cos(psi) - vy * ca.sin(psi),   # d_pos_x
        vx * ca.sin(psi) + vy * ca.cos(psi),   # d_pos_y
        r,                                       # d_psi
        d_vy,                                    # d_vy
        d_r,                                     # d_r
        u_steer_rate,                            # d_steer
        d_vx,                                    # d_vx — NEW
    )

    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl

    # ------------------------------------------------------------------
    # ifp2107: Friction circle — nonlinear h-constraint.
    #
    # h(x, u) = (ax / ax_max)^2 + (ay / ay_max)^2
    #
    # acados enforces:  lh <= h(x,u) <= uh
    # We set uh = 1.0 (unit circle) and make it SOFT via a slack variable
    # so the solver always returns a feasible solution even momentarily
    # exceeding the friction limit. The slack is penalised in the cost
    # with weight SLACK_PENALTY.
    #
    # ax = u_throttle (longitudinal acceleration command)
    # ay = ay_body    (lateral acceleration from tyre dynamics)
    # ------------------------------------------------------------------
    ax = u_throttle  # longitudinal acceleration = throttle command [m/s^2]
    h_friction = (ax / AX_MAX)**2 + (ay_body / AY_MAX)**2
    model.con_h_expr = h_friction   # scalar; dimension nh=1

    return model


# =============================================================================
# ifp2107: OCP CONFIGURATION
# =============================================================================

def build_ocp(model: AcadosModel) -> AcadosOcp:
    """
    ifp2107: Configure the acados OCP with cost function, constraints,
    and solver settings.

    PSEUDO-CODE
    -----------
    Input:  AcadosModel (dynamics + friction constraint)
    Output: AcadosOcp ready for code generation

    COST (LINEAR_LS):
        output y_k = Vx * x_k + Vu * u_k   (linear selection of states/controls)
        J = sum_{k=0}^{N-1} (y_k - yref_k)^T W (y_k - yref_k)
              + (y_N - yref_N)^T W_e (y_N - yref_N)
        yref is updated at each solve from the OptRequest waypoint array.
        Weights (W diagonal):
            pos_x=10, pos_y=10, psi=1, vy=0, r=0, steer=1, vx=10,
            steer_rate=1, throttle=0.5

    CONSTRAINTS (summary):
        steer angle : hard ±0.4 rad      (state bound, index 5)
        vx          : hard [0.5, 20] m/s (state bound, index 6)
        steer_rate  : hard ±1.3 rad/s    (control bound, index 0)
        throttle    : hard [-20, 15]     (control bound, index 1)
        friction    : soft ≤ 1.0         (h-constraint, slack penalty)
    """
    ocp       = AcadosOcp()
    ocp.model = model

    nx = 7    # states:   [pos_x, pos_y, psi, vy, r, steer, vx]
    nu = 2    # controls: [steer_rate, throttle]
    ny   = nx + nu   # output dimension for running cost = 9
    ny_e = nx        # output dimension for terminal cost = 7

    ocp.dims.N = N_HORIZON

    # ------------------------------------------------------------------
    # ifp2107: Cost type — LINEAR_LS allows fast gradient computation.
    # y = Vx * x + Vu * u  →  cost = ||y - yref||^2_W
    # Vx (ny x nx): selects all 7 states into first 7 rows of y
    # Vu (ny x nu): selects both controls into last 2 rows of y
    # ------------------------------------------------------------------
    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])    # (9 x 7)
    Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])    # (9 x 2)

    ocp.cost.Vx   = Vx
    ocp.cost.Vu   = Vu
    ocp.cost.Vx_e = np.eye(nx)                          # (7 x 7) terminal

    # ifp2107: Diagonal cost weights.
    # Index order: [pos_x, pos_y, psi, vy, r, steer, vx, steer_rate, throttle]
    # pos_x=10, pos_y=10: equal weight on both global position components.
    #   On the skidpad circles the cross-track direction rotates continuously —
    #   at some points the lateral error is in global-X, at others in global-Y.
    #   Zero weight on pos_x would leave half the circle unpenalised laterally.
    # psi=1:    heading alignment. Increasing this weight causes oscillation on
    #   circular paths because heading changes continuously around the circle —
    #   aggressive heading correction fights the natural heading rotation.
    # steer=1:  penalise unnecessary steer deflection
    # vx=10:    strong speed regulation — equal weight to position.
    #   Previously 2.0. At W_vx=2 the car overshot 11.25 m/s by ~7% during the
    #   first circle and then the optimizer responded to the circle changeover
    #   with throttle=-16 m/s² (hard braking into a speed death-spiral). At 10.0
    #   the vx cost matches the position cost: a 1 m/s speed error costs the same
    #   as a 1 m position error, which creates a strong restoring force that
    #   prevents both overspeed at circle entry and panic braking at changeover.
    # steer_rate=1, throttle=0.5: smooth actuation
    W_diag    = np.array([10.0, 10.0, 1.0, 0.0, 0.0, 1.0, 10.0, 1.0, 0.5])
    W_e_diag  = W_diag[:nx]   # terminal: same weights, no control terms

    ocp.cost.W   = np.diag(W_diag)
    ocp.cost.W_e = np.diag(W_e_diag)

    # Zero reference at construction — updated per-stage at each solve
    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # ------------------------------------------------------------------
    # ifp2107: Initial state constraint — set to actual x0 at each solve.
    # ------------------------------------------------------------------
    ocp.constraints.x0 = np.zeros(nx)

    # ------------------------------------------------------------------
    # ifp2107: State bounds (hard)
    # idxbx indices into x: 5=steer, 6=vx
    # ------------------------------------------------------------------
    ocp.constraints.idxbx = np.array([5, 6])
    ocp.constraints.lbx   = np.array([-MAX_STEER_ANGLE, 0.5])   # steer, vx lower
    ocp.constraints.ubx   = np.array([ MAX_STEER_ANGLE, 20.0])  # steer, vx upper

    # ------------------------------------------------------------------
    # ifp2107: Control bounds (hard)
    # idxbu indices into u: 0=steer_rate, 1=throttle
    # ------------------------------------------------------------------
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu   = np.array([-MAX_STEER_RATE, -MAX_BRAKING])
    ocp.constraints.ubu   = np.array([ MAX_STEER_RATE,  MAX_THROTTLE])

    # ------------------------------------------------------------------
    # ifp2107: Friction circle — soft nonlinear h-constraint.
    # lh=0: h is sum-of-squares so always >= 0, lower bound not needed.
    # uh=1: enforce friction circle (h <= 1).
    # Softened via slack on the upper bound (idxsh=[0]).
    # Slack cost: Zu * slack^2 (quadratic) + zu * slack (linear, zero here).
    # ------------------------------------------------------------------
    ocp.constraints.lh   = np.array([0.0])
    ocp.constraints.uh   = np.array([1.0])
    ocp.constraints.idxsh = np.array([0])          # soften h-constraint index 0

    ocp.cost.Zl = np.array([0.0])                  # no lower-slack penalty
    ocp.cost.Zu = np.array([SLACK_PENALTY])         # upper-slack quadratic penalty
    ocp.cost.zl = np.array([0.0])                  # no lower-slack linear penalty
    ocp.cost.zu = np.array([0.0])                  # no upper-slack linear penalty

    # ------------------------------------------------------------------
    # ifp2107: Solver options.
    # SQP_RTI: real-time iteration — one QP per timestep, warm-started.
    # ERK + 4 stages: 4th-order Runge-Kutta integration of continuous ODE.
    # GAUSS_NEWTON: approximate Hessian (faster than EXACT for RTI).
    # PARTIAL_CONDENSING_HPIPM: efficient QP solver for the inner step.
    # qp_solver_warm_start=2: full warm-start from previous NLP solution.
    # ------------------------------------------------------------------
    ocp.solver_options.tf                      = TF
    ocp.solver_options.integrator_type         = 'ERK'
    ocp.solver_options.sim_method_num_stages   = 4        # RK4
    ocp.solver_options.sim_method_num_steps    = 1
    ocp.solver_options.nlp_solver_type         = 'SQP_RTI'
    ocp.solver_options.qp_solver              = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_cond_N       = N_HORIZON
    ocp.solver_options.hessian_approx         = 'GAUSS_NEWTON'
    ocp.solver_options.print_level            = 0
    ocp.solver_options.qp_solver_warm_start   = 2

    ocp.code_export_directory = CODE_EXPORT_DIR

    return ocp


# =============================================================================
# ifp2107: SOLVER CREATION — used by both this script and controller_node.py
# =============================================================================

def create_solver(build: bool = True) -> AcadosOcpSolver:
    """
    ifp2107: Build or reload the acados OCP solver.

    PSEUDO-CODE
    -----------
    Input:  build (bool) — True: generate C code + compile (run offline)
                           False: load pre-compiled library (runtime use)
    Output: AcadosOcpSolver instance ready for .solve() calls

    When build=True (run from ocp_definition.py __main__):
        - Generates CasADi Jacobian functions for dynamics and constraints
        - Compiles C code into CODE_EXPORT_DIR
        - Writes JSON description to OCP_JSON_PATH

    When build=False (called from controller_node.py at ROS2 startup):
        - Loads the pre-compiled .so from CODE_EXPORT_DIR
        - No code generation; starts in milliseconds
    """
    model  = build_model()
    ocp    = build_ocp(model)
    solver = AcadosOcpSolver(
        ocp,
        json_file=OCP_JSON_PATH,
        build=build,
        generate=build,
        verbose=build,
    )
    return solver


# =============================================================================
# ifp2107: SOLVER VERIFICATION
# =============================================================================

def verify_solver(solver: AcadosOcpSolver) -> None:
    """
    ifp2107: Run a single test solve from a straight-line cruise condition
    to confirm the generated solver initialises and exits without error.

    PSEUDO-CODE
    -----------
    Input:  AcadosOcpSolver (compiled)
    Output: Prints solver status (0 = SUCCESS) and first control output u0

    Test condition: vehicle at origin, heading=0, vx=11.25 m/s (cruise),
    all errors zero. Expected result: near-zero steer_rate and throttle
    sufficient to overcome drag (~0.95 m/s^2).
    """
    nx = 7
    nu = 2
    ny   = nx + nu
    ny_e = nx

    # Straight-line cruise initial state
    x0 = np.zeros(nx)
    x0[6] = 11.25    # vx = skidpad cruise speed

    # ifp2107: Per-stage reference — car moves forward at cruise speed along
    # global-x. With W_pos_x=10, a static pos_x_ref=0 would cause the optimizer
    # to brake (to minimize growing pos_x error), giving a misleading result.
    # Propagating pos_x_ref = k * DT * vx matches the expected forward motion.
    u_th_cruise = DRAG_COEFF * 11.25**2   # drag compensation at cruise [m/s^2]

    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)
    for k in range(N_HORIZON + 1):
        solver.set(k, 'x', x0)
        yref_k = np.zeros(ny if k < N_HORIZON else ny_e)
        yref_k[0] = k * DT * 11.25   # pos_x: forward motion at cruise speed
        yref_k[6] = 11.25             # vx = cruise speed
        if k < N_HORIZON:
            yref_k[8] = u_th_cruise   # throttle ref = drag compensation
            solver.set(k, 'u', np.zeros(nu))
            solver.set(k, 'yref', yref_k)
        else:
            solver.set(k, 'yref', yref_k)

    status = solver.solve()
    u0     = solver.get(0, 'u')
    cost   = solver.get_cost()

    print(f'  status : {status} (0 = OK)')
    print(f'  u0     : steer_rate={u0[0]:.4f} rad/s  throttle={u0[1]:.4f} m/s^2')
    print(f'  cost   : {cost:.6f}')

    if status == 0:
        print('  Solver verified OK — controller_node.py can now load with build=False.')
    else:
        print('  WARNING: solver returned non-zero status. Check OCP formulation.')


# =============================================================================
# ifp2107: ENTRY POINT
# =============================================================================

if __name__ == '__main__':
    print('=' * 60)
    print('Coupled MPC OCP — Code Generation')
    print(f'  Model   : {MODEL_NAME}')
    print(f'  States  : 7  [pos_x, pos_y, psi, vy, r, steer, vx]')
    print(f'  Controls: 2  [steer_rate, throttle]')
    print(f'  Horizon : N={N_HORIZON}, dt={DT}s, Tf={TF}s')
    print(f'  Solver  : SQP_RTI + ERK4 + PARTIAL_CONDENSING_HPIPM')
    print(f'  Output  : {CODE_EXPORT_DIR}')
    print('=' * 60)

    os.makedirs(CODE_EXPORT_DIR, exist_ok=True)

    solver = create_solver(build=True)

    print('\nVerifying solver...')
    verify_solver(solver)
    print('=' * 60)
    print('Done. Next steps:')
    print('  1. colcon build --packages-select coupled_mpc_controller')
    print('  2. Launch with controller_mode:=mpcc')
    print('=' * 60)
