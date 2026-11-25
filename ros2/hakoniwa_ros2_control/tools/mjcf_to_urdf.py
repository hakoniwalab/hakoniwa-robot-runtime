#!/usr/bin/env python3
"""Writes the kinematic chain of a MuJoCo arm (MJCF) as a URDF, for ros2_control and TF.

    python3 mjcf_to_urdf.py <model.xml> <root body> <out.urdf> [--robot NAME]

Each MJCF body becomes a link; a body with one hinge joint is joined to its parent by a revolute joint with the
MJCF joint's name, axis and range, any other body by a fixed joint. The origin is the body's pos and quat (w x y z).
Defaults (<default class=...>) are followed for the joint axis and range. No geometry, masses or inertias: the
Robot Runtime simulates the arm; this URDF only names the joints and places the frames.
"""
import argparse
import math
import xml.etree.ElementTree as ET


def floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def rpy_from_quat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Defaults:
    """MJCF joint defaults by class, with the class inheritance of nested <default> elements."""

    def __init__(self, root):
        self.joint = {}
        for top in root.findall("default"):
            self._walk(top, {})

    def _walk(self, element, inherited):
        attrs = dict(inherited)
        joint = element.find("joint")
        if joint is not None:
            attrs.update(joint.attrib)
        self.joint[element.get("class", "main")] = attrs
        for child in element.findall("default"):
            self._walk(child, attrs)

    def resolve(self, joint, body_class):
        attrs = dict(self.joint.get("main", {}))
        attrs.update(self.joint.get(joint.get("class") or body_class or "main", {}))
        attrs.update(joint.attrib)
        return attrs


def body_class(body, parents):
    for element in [body, *reversed(parents)]:
        if element.get("childclass"):
            return element.get("childclass")
    return None


def convert(model, root_name, robot):
    tree = ET.parse(model).getroot()
    defaults = Defaults(tree)
    root = next((b for b in tree.iter("body") if b.get("name") == root_name), None)
    if root is None:
        raise SystemExit(f"body {root_name} not found in {model}")
    urdf = ET.Element("robot", name=robot)
    ET.SubElement(urdf, "link", name=root_name)

    def walk(body, parents):
        for child in body.findall("body"):
            name = child.get("name")
            if not name:
                raise SystemExit("unnamed body under " + body.get("name"))
            ET.SubElement(urdf, "link", name=name)
            hinges = [j for j in child.findall("joint") if j.get("type", "hinge") == "hinge"]
            if len(hinges) > 1:
                raise SystemExit(f"body {name}: more than one joint")
            pos = floats(child.get("pos"), (0, 0, 0))
            rpy = rpy_from_quat(*floats(child.get("quat"), (1, 0, 0, 0)))
            if hinges:
                attrs = defaults.resolve(hinges[0], body_class(child, [*parents, body]))
                joint = ET.SubElement(urdf, "joint", name=attrs["name"], type="revolute")
                lower, upper = floats(attrs.get("range"), (-math.pi, math.pi))
                axis = floats(attrs.get("axis"), (0, 0, 1))
            else:
                joint = ET.SubElement(urdf, "joint", name=f"{name}_fixed", type="fixed")
            ET.SubElement(joint, "parent", link=body.get("name"))
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz=" ".join(f"{v:.6g}" for v in pos), rpy=" ".join(f"{v:.6g}" for v in rpy))
            if hinges:
                ET.SubElement(joint, "axis", xyz=" ".join(f"{v:g}" for v in axis))
                ET.SubElement(joint, "limit", lower=f"{lower:g}", upper=f"{upper:g}", effort="1000", velocity="10")
            walk(child, [*parents, body])

    walk(root, [])
    ET.indent(urdf)
    return ET.tostring(urdf, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model")
    parser.add_argument("root")
    parser.add_argument("out")
    parser.add_argument("--robot", default="robot")
    args = parser.parse_args()
    text = convert(args.model, args.root, args.robot)
    with open(args.out, "w") as f:
        f.write('<?xml version="1.0"?>\n' + text + "\n")


if __name__ == "__main__":
    main()
