"""SO-101 with ros2_control inside the Robot Runtime (the controller plugin,
docs/design-controller-plugin.md): this launch starts only the ROS 2 side that
stays outside, robot_state_publisher and MoveIt's move_group. There is no
ros2_control_node: the controller manager, the controllers, /clock and the
sensors (/pdu/<PDU name>) come from the plugin in the Robot Runtime process.
move_group is configured as in so101_asset.launch.py (with sensors:=true its
obstacle map from /pdu/lidar_scan).

ros2 launch hakoniwa_ros2_control so101_plugin.launch.py so101_urdf:=/path/to/so101_new_calib.urdf \\
    [sensors:=true] [moveit:=true]
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

# This file is used from the source tree and from the install space; in both,
# config/ is next to launch/.
SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_asset_launch():
    """so101_asset.launch.py, for its resolved_urdf() and move_group()."""
    path = os.path.join(SHARE, "launch", "so101_asset.launch.py")
    spec = importlib.util.spec_from_file_location("so101_asset", path)
    asset = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(asset)
    return asset


def launch_setup(context):
    asset = load_asset_launch()
    sensors = LaunchConfiguration("sensors").perform(context) == "true"
    sim_time = {"use_sim_time": LaunchConfiguration("sim_time").perform(context) == "true"}
    so101_urdf = asset.resolved_urdf(LaunchConfiguration("so101_urdf").perform(context))
    # The same xacro as the plugin's robot_description (plugin/plugin.yaml),
    # with absolute mesh paths for MoveIt's collision checking.
    xacro_file = os.path.join(SHARE, "config", "so101", "so101_plugin.urdf.xacro")
    robot_description = xacro.process_file(xacro_file, mappings={
        "so101_urdf": so101_urdf,
        "sensors": "true" if sensors else "false",
    }).toxml()
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim_time]),
        asset.move_group(SHARE, robot_description, sim_time, sensors),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("so101_urdf", description="The SO-101 URDF (so101_new_calib.urdf)"),
        DeclareLaunchArgument("sensors", default_value="false",
                              description="Add the sensor frames to the URDF; MoveIt's map from /pdu/lidar_scan"),
        DeclareLaunchArgument("moveit", default_value="true", description="Start MoveIt's move_group"),
        DeclareLaunchArgument("sim_time", default_value="true",
                              description="Run every node on the Hakoniwa time (/clock from the plugin)"),
        OpaqueFunction(function=launch_setup),
    ])
