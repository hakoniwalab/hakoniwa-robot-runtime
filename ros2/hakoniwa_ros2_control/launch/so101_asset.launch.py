"""SO-101 on Hakoniwa with ros2_control as a Hakoniwa asset (HakoniwaAssetSystem):
robot_state_publisher, the controller manager, joint_state_broadcaster,
arm_controller and gripper_controller. ROS 2 and Hakoniwa run on the same Linux;
start this before the Hakoniwa simulation starts (the asset registers then).
The configuration through the Hakoniwa ROS 2 Bridge is so101.launch.py.

ros2 launch hakoniwa_ros2_control so101_asset.launch.py so101_urdf:=/path/to/so101_new_calib.urdf \\
    pdu_definition_path:=/path/to/so101-pdudef.json endpoint_config_path:=/path/to/so101_endpoint.json
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

PACKAGE = "hakoniwa_ros2_control"
ARGUMENTS = ["so101_urdf", "pdu_definition_path", "endpoint_config_path", "robot_name", "asset_name", "delta_usec"]


def launch_setup(context):
    share = get_package_share_directory(PACKAGE)
    mappings = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    mappings["so101_urdf"] = os.path.abspath(mappings["so101_urdf"])
    robot_description = xacro.process_file(
        os.path.join(share, "config", "so101", "so101_asset.urdf.xacro"), mappings=mappings).toxml()
    controllers = os.path.join(share, "config", "so101", "ros2_controllers.yaml")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers, {"robot_description": robot_description}], output="screen"),
        # The controller manager answers once the hardware is active, that is
        # after the simulation has started and the first joint states came.
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                        "--controller-manager-timeout", "120"]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("so101_urdf", description="The SO-101 URDF (so101_new_calib.urdf)"),
        DeclareLaunchArgument("pdu_definition_path", description="The SO-101 PDU definition (so101-pdudef.json)"),
        DeclareLaunchArgument("endpoint_config_path", description="The SO-101 PDU Endpoint (so101_endpoint.json)"),
        DeclareLaunchArgument("robot_name", default_value="SO101"),
        DeclareLaunchArgument("asset_name", default_value="HakoniwaSystem"),
        DeclareLaunchArgument("delta_usec", default_value="2000"),
        OpaqueFunction(function=launch_setup),
    ])
