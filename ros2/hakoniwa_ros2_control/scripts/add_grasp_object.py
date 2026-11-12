#!/usr/bin/env python3
"""Register the object to grasp in MoveIt's planning scene as a box.

ros2 run hakoniwa_ros2_control add_grasp_object.py --id grasp_cube \
    --position 0.28 0.0 0.016 --size 0.03 0.03 0.03 [--remove]

The position is the box centre in the frame (default base_link). MoveIt then
plans around the box as a known object, and the obstacle map (octomap) leaves
out the points inside it. The object's pose is given here; finding it from the
point cloud comes later.
"""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--id", default="grasp_cube")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--position", nargs=3, type=float, default=[0.28, 0.0, 0.016], metavar=("X", "Y", "Z"))
    parser.add_argument("--size", nargs=3, type=float, default=[0.03, 0.03, 0.03], metavar=("SX", "SY", "SZ"))
    parser.add_argument("--remove", action="store_true", help="remove the object instead")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("hakoniwa_add_grasp_object")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=10.0):
        print("move_group (/apply_planning_scene) is not running", file=sys.stderr)
        return 1

    obj = CollisionObject()
    obj.header.frame_id = args.frame
    obj.id = args.id
    if args.remove:
        obj.operation = CollisionObject.REMOVE
    else:
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(args.size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = args.position
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(obj)
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    ok = future.result() is not None and future.result().success
    print(f"{'removed' if args.remove else 'added'} {args.id}: {'OK' if ok else 'FAILED'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
