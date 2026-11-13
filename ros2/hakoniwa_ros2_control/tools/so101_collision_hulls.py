#!/usr/bin/env python3
"""Convex hulls of the SO-101 collision meshes, for Gazebo.

MuJoCo collides with the convex hull of each mesh; Gazebo collides with the
mesh itself. For comparing the two on the same collision shapes, this writes
the hull of every collision mesh of the URDF (same file names) to <output>;
so101_gz.launch.py collision_hulls:=<output> uses them. Needs trimesh.

python3 so101_collision_hulls.py --urdf /path/to/so101_new_calib.urdf --output /path/to/hulls
"""
import argparse
import os
import re

import trimesh


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    base = os.path.dirname(os.path.abspath(args.urdf))
    with open(args.urdf, encoding="utf-8") as stream:
        text = stream.read()
    meshes = sorted(set(re.findall(r'<collision>.*?filename="([^"]+)"', text, flags=re.S)))
    os.makedirs(args.output, exist_ok=True)
    for mesh in meshes:
        path = mesh[len("file://"):] if mesh.startswith("file://") else os.path.join(base, mesh)
        source = trimesh.load(path, force="mesh")
        hull = source.convex_hull
        hull.export(os.path.join(args.output, os.path.basename(path)))
        print(f"{os.path.basename(path)}: {len(source.faces)} -> {len(hull.faces)} faces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
