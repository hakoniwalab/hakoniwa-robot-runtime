#!/usr/bin/env python3
"""Register the object to grasp in MoveIt's planning scene as a box.

ros2 run hakoniwa_ros2_control add_grasp_object.py --id grasp_cube \
    --position 0.28 0.0 0.016 --size 0.03 0.03 0.03 [--remove | --attach | --detach]

--attach moves the object from the world to the hand (it then moves with the
hand, the jaws may touch it, and the obstacle map leaves it out); --detach puts
it back in the world at --position, where it was placed.

The position is the box centre in the frame (default base_link). MoveIt then
plans around the box as a known object, and the obstacle map (octomap) leaves
out the points inside it. The object's pose is given here; finding it from the
point cloud comes later.
"""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--id", default="grasp_cube")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--position", nargs=3, type=float, default=[0.28, 0.0, 0.016], metavar=("X", "Y", "Z"))
    parser.add_argument("--size", nargs=3, type=float, default=[0.03, 0.03, 0.03], metavar=("SX", "SY", "SZ"))
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--remove", action="store_true", help="remove the object instead")
    action.add_argument("--attach", action="store_true", help="move the object from the world to the hand")
    action.add_argument("--detach", action="store_true", help="put the object from the hand back in the world at --position")
    parser.add_argument("--hand-link", default="gripper_link")
    parser.add_argument("--touch-links", nargs="+",
                        default=["gripper_link", "gripper_frame_link", "moving_jaw_so101_v1_link"])
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("hakoniwa_add_grasp_object")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=10.0):
        print("move_group (/apply_planning_scene) is not running", file=sys.stderr)
        return 1

    def box_object(operation):
        obj = CollisionObject()
        obj.header.frame_id = args.frame
        obj.id = args.id
        obj.operation = operation
        if operation == CollisionObject.ADD:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(args.size)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = args.position
            pose.orientation.w = 1.0
            obj.primitives.append(box)
            obj.primitive_poses.append(pose)
        return obj

    def attached(operation):
        held = AttachedCollisionObject()
        held.link_name = args.hand_link
        held.object = box_object(operation)
        held.touch_links = list(args.touch_links)
        return held

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    if args.attach:
        # Attaching an object of a world object's id moves it from the world to the
        # hand; a separate remove of the world object would then fail.
        scene.robot_state.attached_collision_objects.append(attached(CollisionObject.ADD))
    elif args.detach:
        scene.robot_state.attached_collision_objects.append(attached(CollisionObject.REMOVE))
        scene.world.collision_objects.append(box_object(CollisionObject.ADD))
    else:
        scene.world.collision_objects.append(
            box_object(CollisionObject.REMOVE if args.remove else CollisionObject.ADD))
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    ok = future.result() is not None and future.result().success
    done = "attached" if args.attach else "detached" if args.detach else "removed" if args.remove else "added"
    print(f"{done} {args.id}: {'OK' if ok else 'FAILED'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
