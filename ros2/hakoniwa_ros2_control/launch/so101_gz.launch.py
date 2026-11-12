"""SO-101 in Gazebo (Harmonic) with gz_ros2_control, headless, for comparing
with Hakoniwa (so101_asset.launch.py): the same URDF, joints, controllers
(ros2_controllers.yaml) and physics step (2 ms). Gazebo runs as a server only
(gz sim -s); the controller manager runs inside it (gz_ros2_control).

ros2 launch hakoniwa_ros2_control so101_gz.launch.py so101_urdf:=/path/to/so101_new_calib.urdf \\
    [real_time_factor:=1.0]   # 0: as fast as possible
    [collision_hulls:=/path/to/hulls]   # the collision shapes as MuJoCo uses them
"""
import os
import re
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

PACKAGE = "hakoniwa_ros2_control"


def resolved_urdf(so101_urdf, collision_hulls=""):
    """The arm's URDF as Gazebo loads it, in a copy in the temporary directory:
    relative mesh paths made absolute; with collision_hulls (a directory from
    tools/so101_collision_hulls.py), each collision mesh replaced by its convex
    hull, and the collision of base_link (fixed to the world) left out. Both as
    MuJoCo collides: with mesh hulls, and never between bodies fixed to the world."""
    source = os.path.abspath(so101_urdf)
    with open(source, encoding="utf-8") as stream:
        text = stream.read()
    base = os.path.dirname(source)
    text = re.sub(r'filename="(?!/|package://|file://)([^"]+)"',
                  lambda m: f'filename="file://{os.path.join(base, m.group(1))}"', text)
    if collision_hulls:
        hulls = os.path.abspath(collision_hulls)
        text = re.sub(r"<collision>.*?</collision>",
                      lambda block: re.sub(r'filename="file://[^"]*/([^"/]+)"',
                                           lambda m: f'filename="file://{os.path.join(hulls, m.group(1))}"',
                                           block.group(0)),
                      text, flags=re.S)
        text = re.sub(r'(<link name="base_link">.*?</link>)',
                      lambda link: re.sub(r"<collision>.*?</collision>", "", link.group(0), flags=re.S),
                      text, count=1, flags=re.S)
    target = os.path.join(tempfile.gettempdir(), "hakoniwa_so101_gz_resolved.urdf")
    with open(target, "w", encoding="utf-8") as stream:
        stream.write(text)
    return target


def launch_setup(context):
    share = get_package_share_directory(PACKAGE)
    controllers = os.path.join(share, "config", "so101", "ros2_controllers.yaml")
    robot_description = xacro.process_file(
        os.path.join(share, "config", "so101", "gazebo", "so101_gz.urdf.xacro"),
        mappings={"so101_urdf": resolved_urdf(LaunchConfiguration("so101_urdf").perform(context),
                                              LaunchConfiguration("collision_hulls").perform(context)),
                  "controllers": controllers}).toxml()
    with open(os.path.join(share, "config", "so101", "gazebo", "so101_world.sdf.in"), encoding="utf-8") as stream:
        world = stream.read().replace("@REAL_TIME_FACTOR@", LaunchConfiguration("real_time_factor").perform(context))
    world_path = os.path.join(tempfile.gettempdir(), "hakoniwa_so101_gz_world.sdf")
    with open(world_path, "w", encoding="utf-8") as stream:
        stream.write(world)
    sim_time = {"use_sim_time": True}
    return [
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "-v", "1", world_path], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim_time]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "so101"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                        "--controller-manager-timeout", "120"],
             parameters=[sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("so101_urdf", description="The SO-101 URDF (so101_new_calib.urdf)"),
        DeclareLaunchArgument("collision_hulls", default_value="",
                              description="Directory of convex hull collision meshes (tools/so101_collision_hulls.py); "
                                          "also leaves out the collision of base_link, fixed to the world"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0",
                              description="Gazebo's real-time factor (0: as fast as possible)"),
        OpaqueFunction(function=launch_setup),
    ])
