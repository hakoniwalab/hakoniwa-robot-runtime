#!/usr/bin/env python3
"""Writes the iiwa14 contact cell: the arm environment plus a table and a round tip on the tool.

    python3 make_contact_cell.py <iiwa14.environment.xml> <out.xml>

The output goes beside the input (same directory), so the mesh paths stay valid. The tool box of the model does not
collide; the tip is a sphere of radius 10 mm at the end of the tool (98 mm from the force-torque sensor), the only
part of the arm that touches the table. The table top is at z = 0.20 m, under the tool at x = 0.6 m.
"""
import sys
import xml.etree.ElementTree as ET

TABLE_TOP_Z = 0.20
TIP_RADIUS = 0.010
TIP_OFFSET = 0.098  # tip centre from the tool body origin (the sensor frame), along the tool z axis


def main():
    source, out = sys.argv[1], sys.argv[2]
    tree = ET.parse(source)
    root = tree.getroot()
    tool = next(b for b in root.iter("body") if b.get("name") == "tool")
    ET.SubElement(tool, "geom", name="tool_tip", type="sphere", size=f"{TIP_RADIUS}",
                  pos=f"0 0 {TIP_OFFSET}", rgba="0.9 0.9 0.2 1", contype="1", conaffinity="1", mass="0",
                  friction="0.05 0.005 0.0001", condim="1")  # a frictionless tip; MuJoCo takes the larger condim and friction of the two geoms
    world = root.find("worldbody")
    half = 0.02
    ET.SubElement(world, "geom", name="contact_table", type="box", size=f"0.3 0.3 {half}",
                  pos=f"0.6 0 {TABLE_TOP_Z - half}", rgba="0.75 0.7 0.6 1", contype="1", conaffinity="1")
    tree.write(out, encoding="unicode")


if __name__ == "__main__":
    main()
