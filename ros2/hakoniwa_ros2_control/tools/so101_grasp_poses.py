"""SO-101 grasp poses for pick and place, from the MuJoCo model.

Finds the arm joint values that put the grasp centre (the middle of the two
jaw pads) at a point with the tool axis pointing straight down and the jaws
closing along y. The SO-101 arm has five joints, so the pose is searched
numerically (damped least squares, numpy only). Writes the configuration that
hakoniwa_ros2_control's pick_and_place.py reads. Needs only MuJoCo and numpy (no
ROS 2); the model is the SO-101 environment model that hakoniwa-robot-arm forges.

python3 so101_grasp_poses.py --pick 0.28 0.0 --place 0.20 -0.20 \
    --model <work>/model-forge/so101/install/so101.environment.xml --output pick_and_place.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DOWN = np.array([0.0, 0.0, -1.0])


class GraspModel:
    def __init__(self, model_path: Path):
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        name = lambda kind, text: mujoco.mj_name2id(self.m, kind, text)
        self.qadr = [self.m.jnt_qposadr[name(mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
        self.gripper_qadr = self.m.jnt_qposadr[name(mujoco.mjtObj.mjOBJ_JOINT, "gripper")]
        # Keep off the joint limits: a pose exactly at a limit ends up a hair beyond
        # it in the simulation, and MoveIt then refuses to plan from it.
        margin = 0.01
        self.lower = np.array([self.m.jnt_range[name(mujoco.mjtObj.mjOBJ_JOINT, j)][0] for j in ARM]) + margin
        self.upper = np.array([self.m.jnt_range[name(mujoco.mjtObj.mjOBJ_JOINT, j)][1] for j in ARM]) - margin
        self.fixed_pad = name(mujoco.mjtObj.mjOBJ_GEOM, "fixed_jaw_pad")
        self.moving_pad = name(mujoco.mjtObj.mjOBJ_GEOM, "moving_jaw_pad")
        self.hand = name(mujoco.mjtObj.mjOBJ_BODY, "gripper")
        if min(self.fixed_pad, self.moving_pad, self.hand) < 0:
            raise SystemExit("the model has no jaw pads: use the SO-101 environment model")

    def frame(self, q, opening):
        """Grasp centre, tool axis (wrist to centre) and closing direction (pad to pad)."""
        self.d.qpos[self.qadr] = q
        self.d.qpos[self.gripper_qadr] = opening
        mujoco.mj_kinematics(self.m, self.d)
        a = self.d.geom_xpos[self.fixed_pad].copy()
        b = self.d.geom_xpos[self.moving_pad].copy()
        centre = (a + b) / 2.0
        tool = centre - self.d.xpos[self.hand]
        closing = b - a
        return centre, tool / np.linalg.norm(tool), closing / np.linalg.norm(closing)

    def residual(self, q, target, closing_axis, opening):
        # Position at the actual opening; directions with the jaws closed, where
        # the tool axis and the closing direction are (nearly) at right angles.
        centre, _, _ = self.frame(q, opening)
        _, tool, closing = self.frame(q, 0.0)
        return np.concatenate([100.0 * (centre - target), 3.0 * (tool - DOWN), closing - closing_axis])

    def solve(self, target, closing_axis, opening, seeds):
        best_q, best_cost = None, np.inf
        for seed in seeds:
            q = np.clip(np.array(seed, dtype=float), self.lower, self.upper)
            damping = 1e-3
            for _ in range(1000):
                r = self.residual(q, target, closing_axis, opening)
                jac = np.zeros((r.size, q.size))
                for i in range(q.size):
                    step = np.zeros_like(q)
                    step[i] = 1e-6
                    jac[:, i] = (self.residual(q + step, target, closing_axis, opening) - r) / 1e-6
                dq = np.linalg.solve(jac.T @ jac + damping * np.eye(q.size), -jac.T @ r)
                # A joint at a limit that the step pushes beyond stays there; the
                # other joints are solved again without it (instead of the whole
                # step being cut at the limit).
                blocked = ((q <= self.lower + 1e-9) & (dq < 0)) | ((q >= self.upper - 1e-9) & (dq > 0))
                if blocked.any():
                    free = ~blocked
                    jf = jac[:, free]
                    dq = np.zeros_like(q)
                    if free.any():
                        dq[free] = np.linalg.solve(jf.T @ jf + damping * np.eye(int(free.sum())), -jf.T @ r)
                q = np.clip(q + dq, self.lower, self.upper)
                if np.linalg.norm(dq) < 1e-9:
                    break
            cost = float(np.sum(self.residual(q, target, closing_axis, opening) ** 2))
            if cost < best_cost:
                best_q, best_cost = q, cost
        return best_q

    def check(self, q, target, opening, label):
        centre, _, _ = self.frame(q, opening)
        _, tool, _ = self.frame(q, 0.0)
        position_error = float(np.linalg.norm(centre - target))
        tilt_deg = float(np.degrees(np.arccos(np.clip(-tool[2], -1.0, 1.0))))
        print(f"{label:10s} centre {np.round(centre, 4)}  error {position_error * 1000:.1f} mm  tilt {tilt_deg:.1f} deg")
        if position_error > 0.003 or tilt_deg > 5.0:
            raise SystemExit(f"{label}: no pose within 3 mm and 5 deg (out of reach?)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pick", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--place", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--object-size", type=float, default=0.03, help="cube edge [m]")
    parser.add_argument("--grasp-height", type=float, default=0.035,
                        help="grasp centre height [m]; the 4 cm pads then end 0.015 m above the table")
    parser.add_argument("--pick-approach-height", type=float, default=0.090)
    parser.add_argument("--place-approach-height", type=float, default=0.075)
    parser.add_argument("--open", type=float, default=1.0, help="gripper opening for the approach [rad]")
    parser.add_argument("--model", type=Path, required=True,
                        help="the SO-101 environment model (model-forge/so101/install/so101.environment.xml "
                             "in the hakoniwa-robot-arm work directory)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    model = GraspModel(args.model)
    closing_axis = np.array([0.0, -1.0, 0.0])
    def seeds(xy):
        # The shoulder pan turns the arm towards the target (the pan axis points
        # down, so a target at negative y needs a positive pan).
        pan = -float(np.arctan2(xy[1], xy[0]))
        return [[pan, 0.5, -0.2, 1.2, roll] for roll in (-1.6, -0.6, 0.4, 1.5)] + \
               [[pan, 0.6, -0.7, 1.6, roll] for roll in (-1.6, -0.6, 0.4, 1.5)]
    held = 0.279  # the jaws on a 3 cm cube

    def pose(xy, height, opening, seed_list, label):
        target = np.array([xy[0], xy[1], height])
        q = model.solve(target, closing_axis, opening, seed_list)
        model.check(q, target, opening, label)
        return q

    pre_grasp = pose(args.pick, args.pick_approach_height, args.open, seeds(args.pick), "pre_grasp")
    grasp = pose(args.pick, args.grasp_height, args.open, [pre_grasp], "grasp")
    pre_place = pose(args.place, args.place_approach_height, held, seeds(args.place), "pre_place")
    place = pose(args.place, args.grasp_height + 0.002, held, [pre_place], "place")

    half = args.object_size / 2.0
    joints = lambda q: "[" + ", ".join(f"{v:.4f}" for v in q) + "]"
    text = f"""# SO-101 pick and place (written by hakoniwa-robot-runtime ros2/hakoniwa_ros2_control/tools/so101_grasp_poses.py).
# Arm joints in the order {ARM}.
object:
  id: grasp_cube
  size: [{args.object_size}, {args.object_size}, {args.object_size}]
  pick: [{args.pick[0]}, {args.pick[1]}, {half}]
  place: [{args.place[0]}, {args.place[1]}, {half}]
table:
  id: table
  size: [2.4, 2.4, 0.06]
  position: [0.0, 0.0, -0.03]
joints:
  pre_grasp: {joints(pre_grasp)}
  grasp: {joints(grasp)}
  pre_place: {joints(pre_place)}
  place: {joints(place)}
  home: [0.0, 0.0, 0.0, 0.0, 0.0]
gripper:
  open: {args.open}
  close: 0.0
  closed_on_object_min: 0.2  # the jaws stop on the cube above this; below it nothing was grasped
"""
    args.output.write_text(text, encoding="utf-8")
    print(f"written: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
