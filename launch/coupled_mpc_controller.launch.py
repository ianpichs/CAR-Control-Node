"""
coupled_mpc_controller.launch.py — Standalone launch for Coupled MPC node.
===========================================================================
Author:  Ian Pichs (ifp2107) — Columbia University, ELEN 6760 Spring 2026

Launches only the coupled_mpc_controller node (no skidpad_manager).
For full simulation use controllers.launch.xml with mode:=mpcc instead.

All parameters below match ocp_definition.py defaults. Override in
controllers.launch.xml or pass a YAML config to tune without recompiling.
"""
# ifp2107 — original file

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coupled_mpc_controller',
            executable='controller_node',
            name='coupled_mpc_controller',
            output='screen',
            parameters=[{
                # --------------------------------------------------------
                # ifp2107: Speed target and lookahead
                # --------------------------------------------------------
                'vx_target':       11.25,   # [m/s] skidpad cruise speed
                'lookahead_steps': 5,       # waypoints ahead for yref origin

                # --------------------------------------------------------
                # ifp2107: Actuator limits (must match ocp_definition.py)
                # --------------------------------------------------------
                'max_steer_angle': 0.4,     # [rad]
                'max_steer_rate':  1.3,     # [rad/s]
                'max_throttle':    15.0,    # [m/s^2 equivalent]
                'max_braking':     20.0,    # [m/s^2 equivalent]
            }],
        )
    ])
