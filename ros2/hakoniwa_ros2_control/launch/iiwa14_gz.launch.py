"""Run the iiwa14 contact cell and the in-loop F/T broadcaster in Gazebo Harmonic."""

import importlib.util
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) <= 0:
        raise RuntimeError("Set a nonzero ROS_DOMAIN_ID before launching the Gazebo contact cell")
    config_dir = Path(__file__).resolve().parents[1] / "config/iiwa14/gazebo"
    spec = importlib.util.spec_from_file_location("iiwa14_gz_model", config_dir / "iiwa14_gz_model.py")
    model = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(model)

    noise = LaunchConfiguration("noise").perform(context)
    seed = int(LaunchConfiguration("seed").perform(context))
    rtf = float(LaunchConfiguration("real_time_factor").perform(context))
    if noise not in ("none", "white") or seed < 0 or rtf < 0 or not math.isfinite(rtf):
        raise ValueError("Use noise none or white, a nonnegative seed and a finite nonnegative real-time factor")

    temp = TemporaryDirectory(prefix="iiwa14-gz-")
    scratch = Path(temp.name)
    physical = scratch / "physical.urdf"
    ET.ElementTree(model.physical_urdf(LaunchConfiguration("kinematic_urdf").perform(context),
                                       LaunchConfiguration("mjcf").perform(context))).write(physical, encoding="utf-8")
    robot = ET.fromstring(xacro.process_file(str(config_dir / "iiwa14_gz.urdf.xacro"), mappings={
        "physical_urdf": str(physical), "controllers": str(config_dir / "ros2_controllers.yaml"),
    }).toxml())
    if noise == "white":
        model.apply_noise(robot, LaunchConfiguration("noise_sdf").perform(context))
    description = ET.tostring(robot, encoding="unicode")
    urdf_path = scratch / "iiwa14.urdf"
    urdf_path.write_text(description, encoding="utf-8")
    world_path = scratch / "iiwa14.sdf"
    world_path.write_text((config_dir / "iiwa14_gz_world.sdf.in").read_text(encoding="utf-8")
                          .replace("@REAL_TIME_FACTOR@", str(rtf)), encoding="utf-8")

    def cleanup(_):
        temp.cleanup()

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "-v", "2", "--seed", str(seed), str(world_path)], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", "iiwa14_contact", "-file", str(urdf_path), "-name", "iiwa14"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "arm_controller", "wrist_ft_broadcaster",
                        "--controller-manager-timeout", "60"]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mjcf", description="The original iiwa14.environment.xml"),
        DeclareLaunchArgument("kinematic_urdf", description="Output of mjcf_to_urdf.py for body base"),
        DeclareLaunchArgument("noise_sdf", default_value="", description="Fitted ATI Gamma SDF for white noise"),
        DeclareLaunchArgument("noise", default_value="none", description="none or white"),
        DeclareLaunchArgument("seed", default_value="1", description="Gazebo process RNG seed"),
        DeclareLaunchArgument("real_time_factor", default_value="1"),
        OpaqueFunction(function=launch_setup),
    ])
