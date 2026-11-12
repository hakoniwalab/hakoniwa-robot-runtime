"""SO-101 on Hakoniwa with ros2_control: robot_state_publisher, the controller
manager with HakoniwaSystem, joint_state_broadcaster, arm_controller and
gripper_controller (joint_trajectory_controller), and with moveit:=true also
MoveIt's move_group. The Hakoniwa ROS 2 Bridge must be running.

ros2 launch hakoniwa_ros2_control so101.launch.py so101_urdf:=/path/to/so101_new_calib.urdf [moveit:=true]
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


def resolved_urdf(so101_urdf):
    """The arm's URDF with its relative mesh paths (filename="assets/...") made
    absolute, so that MoveIt can load the meshes for collision checking. The
    original file is not changed; the copy goes to the temporary directory."""
    source = os.path.abspath(so101_urdf)
    with open(source, encoding="utf-8") as stream:
        text = stream.read()
    base = os.path.dirname(source)
    text = re.sub(r'filename="(?!/|package://|file://)([^"]+)"',
                  lambda m: f'filename="file://{os.path.join(base, m.group(1))}"', text)
    target = os.path.join(tempfile.gettempdir(), "hakoniwa_so101_resolved.urdf")
    with open(target, "w", encoding="utf-8") as stream:
        stream.write(text)
    return target


def load_yaml(*parts):
    with open(os.path.join(get_package_share_directory(PACKAGE), *parts), encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def launch_setup(context):
    share = get_package_share_directory(PACKAGE)
    urdf = resolved_urdf(LaunchConfiguration("so101_urdf").perform(context))
    robot_description = xacro.process_file(
        os.path.join(share, "config", "so101", "so101.urdf.xacro"),
        mappings={"so101_urdf": urdf, "sensors": LaunchConfiguration("sensors").perform(context)}).toxml()
    controllers = os.path.join(share, "config", "so101", "ros2_controllers.yaml")
    # With sim_time:=true every node runs on the Hakoniwa simulation time, which
    # hakoniwa_clock publishes on /clock from the joint states.
    sim_time = {"use_sim_time": LaunchConfiguration("sim_time").perform(context) == "true"}
    actions = [
        Node(package=PACKAGE, executable="hakoniwa_clock.py",
             condition=IfCondition(LaunchConfiguration("sim_time"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim_time]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers, sim_time], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller"],
             parameters=[sim_time]),
    ]
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
    actions.append(Node(
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
        }, sim_time, *([load_yaml("config", "so101", "moveit", "sensors_3d.yaml")]
              if LaunchConfiguration("sensors").perform(context) == "true" else [])],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("so101_urdf", description="The SO-101 URDF (so101_new_calib.urdf)"),
        DeclareLaunchArgument("moveit", default_value="false", description="Also start MoveIt's move_group"),
        DeclareLaunchArgument("sim_time", default_value="true",
                              description="Run on the Hakoniwa simulation time (/clock from the joint states)"),
        DeclareLaunchArgument("sensors", default_value="false",
                              description="Add the sensor frames (overhead_lidar, wrist_ft) to the robot description"),
        OpaqueFunction(function=launch_setup),
    ])
