"""SO-101 on Hakoniwa with ros2_control: robot_state_publisher, the controller
manager with HakoniwaSystem, joint_state_broadcaster and arm_controller
(joint_trajectory_controller). The Hakoniwa ROS 2 Bridge must be running.

ros2 launch hakoniwa_ros2_control so101.launch.py so101_urdf:=<path to so101_new_calib.urdf>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("hakoniwa_ros2_control")
    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([share, "config", "so101", "so101.urdf.xacro"]),
            " so101_urdf:=", LaunchConfiguration("so101_urdf"),
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([share, "config", "so101", "ros2_controllers.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("so101_urdf", description="The SO-101 URDF (so101_new_calib.urdf)"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "arm_controller"]),
    ])
