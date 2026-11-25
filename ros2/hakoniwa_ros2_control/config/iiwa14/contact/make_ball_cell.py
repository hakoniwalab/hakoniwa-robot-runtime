#!/usr/bin/env python3
"""Writes the iiwa14 ball cell: the contact cell (table and tool tip, make_contact_cell.py) plus a free ball on the table.

    python3 make_ball_cell.py <iiwa14.environment.xml> <out.xml> [--fixed]

With --fixed the ball is welded to the pedestal (no free joint): a fixed dome to trace with the tool.

The ball (radius 0.10 m, 0.5 kg, free joint ball_freejoint) rests on a pedestal (top z = 0.40 m) at x = 0.90 m, its
centre at the height of the sideways approach (z = 0.50 m), where the tool can point along +x. Rolling friction keeps it from drifting on its own;
a push of the tool moves it.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_contact_cell as cell  # noqa: E402

BALL_RADIUS = 0.10
BALL_X = 0.90
PEDESTAL_TOP_Z = 0.40


def main():
    source, out = sys.argv[1], sys.argv[2]
    fixed = "--fixed" in sys.argv[3:]
    cell.main.__globals__["sys"].argv = [sys.argv[0], source, out]
    cell.main()
    tree = ET.parse(out)
    world = tree.getroot().find("worldbody")
    ET.SubElement(world, "geom", name="ball_pedestal", type="box", size=f"0.1 0.1 {PEDESTAL_TOP_Z / 2}",
                  pos=f"{BALL_X} 0 {PEDESTAL_TOP_Z / 2}", rgba="0.5 0.5 0.55 1", contype="1", conaffinity="1")
    ball = ET.SubElement(world, "body", name="ball", pos=f"{BALL_X} 0 {PEDESTAL_TOP_Z + BALL_RADIUS}")
    if not fixed:
        ET.SubElement(ball, "freejoint", name="ball_freejoint")
    # The fixed dome is smooth (tracing, as in polishing); the free ball resists rolling (touch-stop demo).
    ET.SubElement(ball, "geom", name="ball_geom", type="sphere", size=f"{BALL_RADIUS}", mass="0.5",
                  rgba="0.85 0.25 0.2 1", condim=("1" if fixed else "6"), friction=("0.05 0.005 0.0001" if fixed else "0.8 0.005 0.02"), contype="1", conaffinity="1")
    tree.write(out, encoding="unicode")


if __name__ == "__main__":
    main()
