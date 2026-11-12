#!/usr/bin/env python3
"""Sum the pick-and-place runs of run-pick.sh (docs/evaluation-gazebo.md 3.4):
python3 pick_summary.py [/tmp/compare-pick]

Per configuration: the runs that passed every step, the CPU of the container
and of the main processes (cores, over the pick), and the arm's points left in
MoveIt's map per LiDAR scan while the arm moves (a joint faster than 0.05
rad/s since the previous scan) and while it is still."""
import collections
import csv
import glob
import os
import re
import sys

R = sys.argv[1] if len(sys.argv) > 1 else "/tmp/compare-pick"
PROCESSES = ["robot-arm-hakoniwa-asset", "gz sim", "move_group", "ros2_control_node", "parameter_bridge"]


def cpu_times(path):
    times = {}
    for line in open(path):
        parts = line.split(None, 2)
        if len(parts) == 3:
            times[parts[0]] = (int(parts[1]), parts[2])
    return times


def main():
    labels = sorted({re.sub(r"-\d+\.out$", "", os.path.basename(p)) for p in glob.glob(f"{R}/*-[0-9]*.out")
                     if not p.endswith(".ros.out")})
    print("| configuration | passed | CPU (container) | " + " | ".join(PROCESSES)
          + " | left points, moving (mean / max) | left points, still (mean / max) |")
    print("|---" * (5 + len(PROCESSES)) + "|")
    for label in labels:
        runs = sorted(glob.glob(f"{R}/{label}-[0-9]*.out"))
        runs = [r for r in runs if not r.endswith(".ros.out")]
        passed = wall = cpu = 0
        per = collections.Counter()
        moving, still = [], []
        for run in runs:
            text = open(run).read()
            stem = run[:-4]
            passed += "rc=0" in text
            wall += int(re.search(r"wall_ms=(\d+)", text).group(1)) / 1000
            cpu += int(re.search(r"cpu_ms=(\d+)", text).group(1)) / 1000
            before, after = cpu_times(stem + ".ps0"), cpu_times(stem + ".ps1")
            for pid, (t, args) in after.items():
                name = next((p for p in PROCESSES if p in args), None)
                if name:
                    per[name] += t - before.get(pid, (0,))[0]
            for row in csv.DictReader(open(stem + ".leak.csv")):
                (moving if float(row["joint_speed_max"]) > 0.05 else still).append(int(row["leak"]))
        stat = lambda v: f"{sum(v) / len(v):.1f} / {max(v)}" if v else "-"
        print(f"| {label} | {passed} / {len(runs)} | {cpu / wall:.2f} | "
              + " | ".join(f"{per[p] / wall:.2f}" if per[p] else "-" for p in PROCESSES)
              + f" | {stat(moving)} | {stat(still)} |")


if __name__ == "__main__":
    main()
