#!/usr/bin/env python3
"""Puts the fitted KUKA sensor set into a hakoniwa-robot-arm checkout, as the iiwa14 recipe's "hakoniwa" sensor set.

    python3 install_sensors.py <hakoniwa-robot-arm>

The iiwa14 recipe reads recipes/iiwa14/config/sensors/iiwa14-sensors-<implementation>.json when it runs with
--sensors --sensor-implementation <implementation>; with "hakoniwa" it then uses Sensor PRO with the ATI Gamma noise
fitted to the real arm and the vibration model T14. The files are copied (not committed there): robot-arm keeps only
its default set (true values). Files that already exist with other contents are not overwritten unless --force.
"""
import argparse
import filecmp
import shutil
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent / "sensors"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot_arm", help="hakoniwa-robot-arm checkout")
    ap.add_argument("--force", action="store_true", help="overwrite files that differ")
    args = ap.parse_args()
    dest = Path(args.robot_arm).resolve() / "recipes" / "iiwa14" / "config" / "sensors"
    if not (dest / "iiwa14-sensors.json").exists():
        sys.exit(f"{dest} has no iiwa14-sensors.json: is {args.robot_arm} a hakoniwa-robot-arm checkout with the iiwa14 recipe?")
    for src in sorted(p for p in SRC.rglob("*") if p.is_file()):
        target = dest / src.relative_to(SRC)
        if target.exists() and not filecmp.cmp(src, target, shallow=False) and not args.force:
            sys.exit(f"{target} exists with other contents (use --force to overwrite)")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)
        print(f"{target.relative_to(dest.parents[3])}")


if __name__ == "__main__":
    main()
