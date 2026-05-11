"""
lqr_pid_controller.launch.py
-----------------------------
Launches the LQR lateral + PID longitudinal controller node for the DUT25
skidpad mission.

This node replaces mpc_python_exec + longitudinal_control. To activate it,
add a mode == "lqr" block in:
    src/mission_control/launch/base_pipeline/controllers.launch.xml
and launch the simulator with:
    ros2 launch simulator simulation.launch.xml \
        mission_name:=skidpad perception:=sim state_estimation:=sim \
        rviz:=false controller_mode:=lqr

All parameters below match DUT25 defaults. Override values here or pass a
YAML config file to tune without modifying source code.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lqr_pid_controller",
            executable="controller_node",
            name="lqr_pid_controller",
            output="screen",
            parameters=[{
                # ----------------------------------------------------------
                # LQR — vehicle model (DUT25 measured values)
                # ----------------------------------------------------------
                # Speed at which the bicycle model is linearized.
                # Must equal the skidpad speed reference in the planner.
                "vx_operating": 11.25,          # [m/s]

                # Prediction horizon — must match skidpad_manager Nt and Tf
                # so that the OptResult.steer array length (Nt+1) aligns with
                # the manager's interpolation time axis.
                "n_horizon": 100,               # steps (Nt)
                "t_horizon": 1.0,               # seconds (Tf)

                # Physical steering limits
                "max_steer_angle": 0.4,         # [rad]
                "max_steer_rate":  0.5,         # [rad/s]

                # ----------------------------------------------------------
                # LQR — cost matrix weights (Q diagonal entries + R)
                # Tune these to adjust lateral tracking behaviour.
                # Starting values mirror MPC weights from parameters_LPV.yaml.
                # ----------------------------------------------------------
                "q_e_y":        8.0,    # lateral error penalty
                "q_e_psi":      1.0,    # heading error penalty
                "q_vy":         5.0,    # lateral velocity damping
                "q_r":          4.0,    # yaw rate penalty — damps oscillation at circle entry
                "q_steer":      1.0,    # steering angle penalty
                "r_steer_rate": 1.0,    # control effort penalty

                # ----------------------------------------------------------
                # PID — longitudinal (matching longitudinal_pid_parameters.yaml)
                # Three-element lists: [low_speed, mid_speed, high_speed]
                # ----------------------------------------------------------
                "lower_speed":  4.0,    # [m/s] below this → use gains[0]
                "upper_speed":  8.0,    # [m/s] above this → use gains[2]
                "kp_list":      [4.0, 4.0, 4.0],
                "ki_list":      [1.3, 1.3, 1.3],
                "kd_list":      [0.0, 0.0, 0.0],
                "max_throttle": 15.0,
                "max_braking":  20.0,
                "max_acc_rate":  3.0,
            }],
        )
    ])
