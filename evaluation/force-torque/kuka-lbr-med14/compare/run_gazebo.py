#!/usr/bin/env python3
"""Runs the Gazebo force-torque sensor (gz-sensors) on the same truth as Hakoniwa, for seeds 1-20.

    python3 compare/run_gazebo.py <ft-series>

<ft-series>: hakoniwa-sensor-adapter's tools/compare ft-series (built with pixi, see its README), which reads a truth
file with --motion file. Under compare/out/:
  gazebo/s<seed>.csv                 truth (MuJoCo, no noise)                   -> Gazebo, fitted noise (seeds 1-20)
  gazebo-vibration/s<seed>.csv       truth + Hakoniwa's vibration model (seed) -> Gazebo, fitted noise (seeds 1-20)
  gazebo-rest/, gazebo-vibration-rest/  the same with the arm held still for 10 s (seeds 1-6)
The noise of compare/config/gz-ati-gamma-fitted.sdf is the same per-axis sigma as sensors/force_torque/
wrist-ft-ati-gamma-fitted.json. The truth runs are runs/truth.npz and runs/truth-vibration-t14.npz.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kukadata as K  # noqa: E402


def write_truth(array, path):
    np.savetxt(path, array[:, :7], delimiter=",", header="t,fx,fy,fz,tx,ty,tz", comments="", fmt="%.10g")


def main():
    tool = sys.argv[1]
    sdf = K.HERE / "config" / "gz-ati-gamma-fitted.sdf"
    sets = (("", "truth.npz", "truth-vibration-t14.npz", 20), ("-rest", "rest-truth.npz", "rest-truth-vibration-t14.npz", 6))
    for suffix, truth_file, vib_file, seeds in sets:
        run_set(tool, sdf, suffix, np.load(K.RUNS / truth_file)["truth"], np.load(K.RUNS / vib_file), seeds)


def run_set(tool, sdf, suffix, truth, vib, seeds):
    with tempfile.TemporaryDirectory() as tmp:
        t_path = Path(tmp) / "truth.csv"
        write_truth(truth, t_path)
        for name, inputs in ((f"gazebo{suffix}", None), (f"gazebo-vibration{suffix}", vib)):
            out = K.OUT / name
            out.mkdir(parents=True, exist_ok=True)
            for seed in range(1, seeds + 1):
                src = t_path
                if inputs is not None:
                    src = Path(tmp) / f"truthvib_{seed}.csv"
                    write_truth(inputs[f"s{seed}"].astype(np.float64), src)
                subprocess.run([tool, "--implementation", "gazebo", "--config", str(sdf), "--motion", "file",
                                "--input", str(src), "--seed", str(seed), "--out", str(out / f"s{seed}.csv")],
                               check=True)
            print(f"{out}: {seeds} runs")


if __name__ == "__main__":
    main()
