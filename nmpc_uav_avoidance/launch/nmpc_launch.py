"""
Launch file for C++ NMPC UAV Obstacle Avoidance Node.

Usage:
    ros2 launch nmpc_uav_avoidance nmpc_launch.py
    ros2 launch nmpc_uav_avoidance nmpc_launch.py obs_x:=5.0 r_obs:=0.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("obs_x",       default_value="3.0"),
        DeclareLaunchArgument("obs_y",       default_value="0.0"),
        DeclareLaunchArgument("obs_z",       default_value="1.0"),
        DeclareLaunchArgument("r_obs",       default_value="1.0"),
        DeclareLaunchArgument("yaw_gain",    default_value="1.0"),
        DeclareLaunchArgument("solver_path", default_value=""),

        Node(
            package="nmpc_uav_avoidance",
            executable="nmpc_node",
            name="nmpc_uav_node",
            output="screen",
            parameters=[{
                "obs_x":       LaunchConfiguration("obs_x"),
                "obs_y":       LaunchConfiguration("obs_y"),
                "obs_z":       LaunchConfiguration("obs_z"),
                "r_obs":       LaunchConfiguration("r_obs"),
                "yaw_gain":    LaunchConfiguration("yaw_gain"),
                "solver_path": LaunchConfiguration("solver_path"),
            }],
            remappings=[
                ("/odom", "/odom"),
                ("/planned_path", "/planned_path"),
            ],
        ),
    ])