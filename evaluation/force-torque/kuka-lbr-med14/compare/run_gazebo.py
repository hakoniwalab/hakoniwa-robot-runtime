#!/usr/bin/env python3
"""Runs the Gazebo force-torque sensor (gz-sensors) on the same truth as Hakoniwa, for seeds 1-20.

    python3 compare/run_gazebo.py <ft-series>

<ft-series>: hakoniwa-sensor-adapter's tools/compare ft-series (built with pixi, see its README), which reads a truth
file with --motion file. Two sets are written under compare/out/:
  gazebo/s<seed>.csv            truth (MuJoCo, no noise)                      -> Gazebo, fitted noise
  gazebo-vibration/s<seed>.csv  truth + Hakoniwa's vibration model (seed)    -> Gazebo, fitted noise
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
    truth = np.load(K.RUNS / "truth.npz")["truth"]
    vib = np.load(K.RUNS / "truth-vibration-t14.npz")
    with tempfile.TemporaryDirectory() as tmp:
        t_path = Path(tmp) / "truth.csv"
        write_truth(truth, t_path)
        for name, inputs in (("gazebo", None), ("gazebo-vibration", vib)):
            out = K.OUT / name
            out.mkdir(parents=True, exist_ok=True)
            for seed in range(1, 21):
                src = t_path
                if inputs is not None:
                    src = Path(tmp) / f"truthvib_{seed}.csv"
                    write_truth(inputs[f"s{seed}"].astype(np.float64), src)
                subprocess.run([tool, "--implementation", "gazebo", "--config", str(sdf), "--motion", "file",
                                "--input", str(src), "--seed", str(seed), "--out", str(out / f"s{seed}.csv")],
                               check=True)
            print(f"{out}: 20 runs")


if __name__ == "__main__":
    main()
