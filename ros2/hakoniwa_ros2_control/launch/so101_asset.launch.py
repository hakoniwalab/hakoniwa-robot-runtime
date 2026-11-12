"""SO-101 on Hakoniwa with ros2_control as a Hakoniwa asset (HakoniwaAssetSystem):
robot_state_publisher, the controller manager, joint_state_broadcaster,
arm_controller and gripper_controller. ROS 2 and Hakoniwa run on the same Linux;
start this before the Hakoniwa simulation starts (the asset registers then).
With sensors:=true, HakoniwaAssetSystem also publishes the sensors (rates in
config/so101/sensor_publish.json) as /pdu/<PDU name>, and the URDF has their
frames. HakoniwaAssetSystem publishes /clock from the Hakoniwa time; with
sim_time:=true (default) every node runs on it. With moveit:=true also MoveIt's
move_group (as so101.launch.py; with sensors:=true its obstacle map from
/pdu/lidar_scan).
The configuration through the Hakoniwa ROS 2 Bridge is so101.launch.py.

ros2 launch hakoniwa_ros2_control so101_asset.launch.py so101_urdf:=/path/to/so101_new_calib.urdf \\
    pdu_definition_path:=/path/to/so101-pdudef.json endpoint_config_path:=/path/to/so101_endpoint.json
"""
import os
import re
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

PACKAGE = "hakoniwa_ros2_control"
ARGUMENTS = ["so101_urdf", "pdu_definition_path", "endpoint_config_path", "robot_name", "asset_name", "delta_usec"]


def resolved_urdf(so101_urdf):
    """The arm's URDF with its relative mesh paths made absolute (for MoveIt's
    collision checking), in a copy in the temporary directory (as so101.launch.py)."""
    source = os.path.abspath(so101_urdf)
    with open(source, encoding="utf-8") as stream:
        text = stream.read()
    base = os.path.dirname(source)
    text = re.sub(r'filename="(?!/|package://|file://)([^"]+)"',
                  lambda m: f'filename="file://{os.path.join(base, m.group(1))}"', text)
    target = os.path.join(tempfile.gettempdir(), "hakoniwa_so101_asset_resolved.urdf")
    with open(target, "w", encoding="utf-8") as stream:
        stream.write(text)
    return target


def load_yaml(*parts):
    with open(os.path.join(get_package_share_directory(PACKAGE), *parts), encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def move_group(share, robot_description, sim_time, sensors):
    """MoveIt's move_group, configured as in so101.launch.py."""
    with open(os.path.join(share, "config", "so101", "moveit", "so101.srdf"), encoding="utf-8") as stream:
        robot_description_semantic = stream.read()
    ompl = {
        "planning_plugins": ["ompl_interface/OMPLPlanner"],
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
        "start_state_max_bounds_error": 0.1,
    }
    return Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        condition=IfCondition(LaunchConfiguration("moveit")),
        parameters=[{
            "robot_description": robot_description,
            "robot_description_semantic": robot_description_semantic,
            "robot_description_kinematics": load_yaml("config", "so101", "moveit", "kinematics.yaml"),
            "robot_description_planning": load_yaml("config", "so101", "moveit", "joint_limits.yaml"),
            "planning_pipelines": {"pipeline_names": ["ompl"]},
            "default_planning_pipeline": "ompl",
            "ompl": ompl,
            **load_yaml("config", "so101", "moveit", "moveit_controllers.yaml"),
            "trajectory_execution": {
                "allowed_execution_duration_scaling": 1.5,
                "allowed_goal_duration_margin": 1.0,
                "allowed_start_tolerance": 0.05,
            },
            "publish_robot_description_semantic": True,
        }, sim_time, *([load_yaml("config", "so101", "moveit", "sensors_3d.yaml")] if sensors else [])],
    )


def launch_setup(context):
    share = get_package_share_directory(PACKAGE)
    mappings = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    mappings["so101_urdf"] = resolved_urdf(mappings["so101_urdf"])
    sensors = LaunchConfiguration("sensors").perform(context) == "true"
    mappings["sensors"] = "true" if sensors else "false"
    mappings["sensor_publish_config"] = (
        os.path.join(share, "config", "so101", "sensor_publish.json") if sensors else "")
    sim_time = {"use_sim_time": LaunchConfiguration("sim_time").perform(context) == "true"}
    robot_description = xacro.process_file(
        os.path.join(share, "config", "so101", "so101_asset.urdf.xacro"), mappings=mappings).toxml()
    controllers = os.path.join(share, "config", "so101", "ros2_controllers.yaml")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim_time]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers, {"robot_description": robot_description}, sim_time], output="screen"),
        # The controller manager answers once the hardware is active, that is
        # after the simulation has started and the first joint states came.
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                        "--controller-manager-timeout", "120"],
             parameters=[sim_time]),
        move_group(share, robot_description, sim_time, sensors),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("so101_urdf", description="The SO-101 URDF (so101_new_calib.urdf)"),
        DeclareLaunchArgument("pdu_definition_path", description="The SO-101 PDU definition (so101-pdudef.json)"),
        DeclareLaunchArgument("endpoint_config_path", description="The SO-101 PDU Endpoint (so101_endpoint.json)"),
        DeclareLaunchArgument("robot_name", default_value="SO101"),
        DeclareLaunchArgument("asset_name", default_value="HakoniwaSystem"),
        DeclareLaunchArgument("delta_usec", default_value="2000"),
        DeclareLaunchArgument("sensors", default_value="false",
                              description="Publish the sensors (/pdu/...) and add their frames to the URDF"),
        DeclareLaunchArgument("moveit", default_value="false", description="Also start MoveIt's move_group"),
        DeclareLaunchArgument("sim_time", default_value="true",
                              description="Run every node on the Hakoniwa time (/clock from HakoniwaAssetSystem)"),
        OpaqueFunction(function=launch_setup),
    ])
